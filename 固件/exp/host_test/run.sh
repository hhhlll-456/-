#!/usr/bin/env bash
# 主机端回归锁：跑 ring_buf_test.cpp（main.cpp 环形缓冲逻辑的拷贝，见该文件头部说明）。
# 只用 g++，不需要 PlatformIO / 硬件；可重复执行、也适合放进 CI。
#
# 两步：
#   1. 构建 + 跑全部断言（全绿才算过）
#   2. 变异自检：把 resetRunBuf 改成"只清 run_head"的不对称复位，断言 harness 变红 ——
#      证明这条锁真的能抓住"漏清游标"这个雷，而不是一条永远绿的假锁。
set -euo pipefail
cd "$(dirname "$0")"

CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -Wall -Wextra -g -fsanitize=address,undefined)

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "[1/2] 构建并运行 ring_buf_test（-fsanitize=address,undefined / -std=c++17）"
"$CXX" "${FLAGS[@]}" ring_buf_test.cpp -o "$TMP/ring_buf_test"
"$TMP/ring_buf_test"

echo
echo "[2/2] 变异自检：resetRunBuf 改成不对称复位（只清 head），harness 必须变红"
sed 's/run_head = 0; run_tail = 0; run_pending = 0; run_dropped = 0;/run_head = 0; run_pending = 0; run_dropped = 0;/' \
    ring_buf_test.cpp > "$TMP/ring_buf_test_mutated.cpp"
if grep -q "run_head = 0; run_pending = 0; run_dropped = 0;" "$TMP/ring_buf_test_mutated.cpp"; then
  :
else
  echo "❌ 变异自检无法进行：sed 没匹配到 resetRunBuf 的原文（harness 被改过？）" >&2
  exit 1
fi
"$CXX" "${FLAGS[@]}" "$TMP/ring_buf_test_mutated.cpp" -o "$TMP/ring_buf_test_mutated"
if "$TMP/ring_buf_test_mutated" > "$TMP/mutated.log" 2>&1; then
  echo "❌ 变异自检失败：不对称复位没有被判红 —— 回归锁失效" >&2
  exit 1
fi
echo "✅ 变异自检通过：不对称复位被判红。harness 给出的前几条证据："
grep -m3 "FAIL" "$TMP/mutated.log" || true
