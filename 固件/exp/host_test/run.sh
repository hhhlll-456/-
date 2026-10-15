#!/usr/bin/env bash
# 主机端回归锁：跑 ring_buf_test.cpp（main.cpp 环形缓冲 + TX 路径逻辑的拷贝，见该文件头部说明）。
# 只用 g++，不需要 PlatformIO / 硬件；可重复执行、也适合放进 CI。
#
# 三步：
#   1. 漂移自检：harness 是 main.cpp 的**拷贝**，先证明拷贝与原件没跑偏
#      （否则"改了 main.cpp 忘了同步"会得到一条永远绿的假锁）。
#   2. 构建 + 跑全部断言（全绿才算过）
#   3. 变异自检：对四处拷贝逻辑各做一个人工变异，断言 harness 全部变红 ——
#      M1 resetRunBuf 只清 head（不对称复位；T4/T5 锁）
#      M2 硬错误判成软错误（T7 锁：硬错误必须立刻踢，不能等阈值）
#      M3 软错误判成硬错误（T7 锁：软错误不许立刻踢，要等 5 s 阈值）
#      M4 n==0 报 OK（T7 锁：没发出字节就不算成功，否则 flushRunBuf 空转死循环）
#      逐一证明这条锁真能抓住这四类回归，而不是一条永远绿的假锁。
set -euo pipefail
cd "$(dirname "$0")"

CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -Wall -Wextra -g -fsanitize=address,undefined)

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

MAIN="../src/main.cpp"

echo "[1/3] 漂移自检：harness 拷贝必须与 $MAIN 的 TX 逻辑一致"
drift_fail=0

# (a) 看门狗阈值：TX_STALL_MS 的取值必须一致。
m_val=$(sed -n 's/^static const uint32_t TX_STALL_MS = \([0-9]*\);.*/\1/p' "$MAIN")
t_val=$(sed -n 's/^static const uint32_t TX_STALL_MS = \([0-9]*\);.*/\1/p' ring_buf_test.cpp)
if [ -z "$m_val" ] || [ -z "$t_val" ]; then
  echo "❌ 漂移自检无法进行：TX_STALL_MS 没匹配到（文件被改过？）" >&2; exit 1
fi
if [ "$m_val" != "$t_val" ]; then
  echo "❌ 漂移：main.cpp TX_STALL_MS=$m_val ≠ harness TX_STALL_MS=$t_val" >&2; drift_fail=1
else
  echo "✅ TX_STALL_MS 两边一致（${m_val} ms）"
fi

# (b) txSendNonBlocking 的返回分类序列必须一致（谁返回 OK/SOFT/HARD，一个都不能差）。
m_seq=$(sed -n '/^static TxSend txSendNonBlocking/,/^}/p' "$MAIN" | grep -o 'return TX_SEND_[A-Z]*;')
t_seq=$(sed -n '/^static TxSend txSendNonBlocking/,/^}/p' ring_buf_test.cpp | grep -o 'return TX_SEND_[A-Z]*;')
if [ "$m_seq" != "$t_seq" ]; then
  echo "❌ 漂移：txSendNonBlocking 的硬/软分类两边不一致" >&2
  echo "--- main.cpp ---"; echo "$m_seq"; echo "--- harness ---"; echo "$t_seq" >&2
  drift_fail=1
else
  echo "✅ txSendNonBlocking 硬/软分类序列两边一致（$(echo "$m_seq" | tr '\n' ' '))"
fi

# (c) 归入"软错误"的 errno 集合必须一致（软↔硬边界错一个 errno 就足以误杀健康连接）。
m_cases=$(sed -n '/^static TxSend txSendNonBlocking/,/^}/p' "$MAIN" | grep -o 'case E[A-Z]*:' | tr '\n' ' ')
t_cases=$(sed -n '/^static TxSend txSendNonBlocking/,/^}/p' ring_buf_test.cpp | grep -o 'case E[A-Z]*:' | tr '\n' ' ')
if [ "$m_cases" != "$t_cases" ] || [ -z "$t_cases" ]; then
  echo "❌ 漂移：errno→软错误的 case 集合两边不一致（main: '$m_cases' / harness: '$t_cases'）" >&2
  drift_fail=1
else
  echo "✅ 软错误 errno 集合两边一致（$t_cases）"
fi

[ "$drift_fail" -eq 0 ] || { echo "❌ 漂移自检红：harness 已与固件脱节，请同步 ring_buf_test.cpp" >&2; exit 1; }

echo
echo "[2/3] 构建并运行 ring_buf_test（-fsanitize=address,undefined / -std=c++17）"
"$CXX" "${FLAGS[@]}" ring_buf_test.cpp -o "$TMP/ring_buf_test"
"$TMP/ring_buf_test"

echo
echo "[3/3] 变异自检：四个人工变异都必须被判红"

# 编译一个变异源码并运行：必须失败（断言红）才算自检通过。
expect_red() {
  local name="$1" src="$2"
  "$CXX" "${FLAGS[@]}" "$src" -o "$TMP/ring_buf_test_mutated"
  if "$TMP/ring_buf_test_mutated" > "$TMP/mutated.log" 2>&1; then
    echo "❌ 变异自检失败：$name 没有被判红 —— 回归锁失效" >&2
    exit 1
  fi
  echo "✅ $name 被判红。harness 给出的前几条证据："
  grep -m3 "FAIL" "$TMP/mutated.log" || true
}

# M1：resetRunBuf 改成不对称复位（只清 head）—— 必须被 T4/T5 抓住。
sed 's/run_head = 0; run_tail = 0; run_pending = 0; run_dropped = 0;/run_head = 0; run_pending = 0; run_dropped = 0;/' \
    ring_buf_test.cpp > "$TMP/mut1.cpp"
grep -q "run_head = 0; run_pending = 0; run_dropped = 0;" "$TMP/mut1.cpp" \
  || { echo "❌ M1 变异无法进行：sed 没匹配到 resetRunBuf 原文（harness 被改过？）" >&2; exit 1; }
expect_red "M1 resetRunBuf 只清 head（不对称复位）" "$TMP/mut1.cpp"

# M2：send() 硬错误换成软错误（分类 default 分支返回 SOFT）—— 必须被 T7 抓住。
sed 's/^      return TX_SEND_HARD;$/      return TX_SEND_SOFT;/' \
    ring_buf_test.cpp > "$TMP/mut2.cpp"
if grep -q '^      return TX_SEND_HARD;$' "$TMP/mut2.cpp"; then
  echo "❌ M2 变异无法进行：sed 没把硬错误分类行改掉（harness 被改过？）" >&2
  exit 1
fi
expect_red "M2 硬错误被判成软错误（T7：硬错误必须立刻踢）" "$TMP/mut2.cpp"

# M3：软错误换成硬错误（EAGAIN 等 case 返回 HARD）—— 必须被 T7 抓住。
sed 's/^      return TX_SEND_SOFT;$/      return TX_SEND_HARD;/' \
    ring_buf_test.cpp > "$TMP/mut3.cpp"
if grep -q '^      return TX_SEND_SOFT;$' "$TMP/mut3.cpp"; then
  echo "❌ M3 变异无法进行：sed 没把软错误分类行改掉（harness 被改过？）" >&2
  exit 1
fi
expect_red "M3 软错误被判成硬错误（T7：软错误不许立刻踢）" "$TMP/mut3.cpp"

# M4：n==0 报 OK（*sent=0）—— 必须被 T7 抓住。调用方靠 `r != OK → break` 决定推进游标，
# 报 OK 却一个字节没发会让 flushRunBuf 的 while(off < pend) 空转（看门狗路径上的死循环）。
sed 's/^  if (n == 0) return TX_SEND_SOFT;.*$/  if (n == 0) return TX_SEND_OK;/' \
    ring_buf_test.cpp > "$TMP/mut4.cpp"
if grep -q '^  if (n == 0) return TX_SEND_SOFT;' "$TMP/mut4.cpp"; then
  echo "❌ M4 变异无法进行：sed 没匹配到 n==0 的分类行（harness 被改过？）" >&2
  exit 1
fi
expect_red "M4 n==0 报 OK（调用方空转死循环）" "$TMP/mut4.cpp"
