// 主机端回归锁：把 固件/exp/src/main.cpp 的环形缓冲逻辑原样拷出来，用小 CAP 跑单元测试。
// 目的：烧录前先证明「回绕不覆盖未发数据」「三个复位点必须两个游标一起清零」，并演示
// 不对称复位（只清一个游标）的真实后果 —— **覆盖未发数据**（不是"判满、静默丢弃"）。
//
// 【这是拷贝，不是共享代码】下面标注"拷贝自 main.cpp"的 4 个函数 + 相关变量必须与
// main.cpp 的逻辑逐字一致（只允许两类替换：CAP 换成小值、client/send/millis 换成假依赖）。
// 拷贝块 1：环形缓冲（resetRunBuf / runPushLine / flushRunBuf）；
// 拷贝块 2：TX 停摆看门狗（checkTxStall，R62 —— "环里有积压且 >2s 没发出一个字节 = 僵尸"）。
// 改了 main.cpp 的这两块，必须同步改这里，否则这条锁会腐烂成"永远绿"的假锁 ——
// 变异自检（run.sh 第 2 步）只兜底不复位一处。
//
// 为什么 CAP 取 40（真实是 32768）：让"回绕"和"缓冲满"在几十行内反复发生，测试才有杠杆。
// 一行 8 B × 一批 5 行 = 40 B 恰好等于 CAP —— 顺便锁住"一整批必须能整批放下"这个边界。
//
// 用法：bash 固件/exp/host_test/run.sh   （纯 g++，不需要 PlatformIO / 硬件）
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#define RUN_BUF_CAP     40      // 拷贝自 main.cpp（真实值 32768；缩小只为测试杠杆）
#define RUN_FLUSH_LINES 5       // 与 main.cpp 相同（一行 = 一个样本，攒 5 行一批）

// ===== 拷贝自 main.cpp 的环形缓冲（变量 + resetRunBuf / runPushLine / flushRunBuf）=====
static char   run_buf[RUN_BUF_CAP];
static size_t run_head = 0;      // 已写入总字节数
static size_t run_tail = 0;      // 已 flush 总字节数
static int    run_pending = 0;   // 已写入、还没 flush 的整行数（flush 的触发条件）
static int    run_dropped = 0;   // 缓冲满被丢弃的行数

void resetRunBuf() { run_head = 0; run_tail = 0; run_pending = 0; run_dropped = 0; }

static bool runPushLine(const char* s, size_t len) {
  size_t need = len + 1;                                   // 含行尾 '\n'
  if (need > RUN_BUF_CAP) return false;
  if (RUN_BUF_CAP - (run_head - run_tail) < need) { run_dropped++; return false; }
  size_t pos   = run_head % RUN_BUF_CAP;
  size_t first = RUN_BUF_CAP - pos;                 // 回绕前还能连着放多少字节
  if (first > len) first = len;
  memcpy(run_buf + pos, s, first);                  // 前半段（不回绕）
  if (len > first) memcpy(run_buf, s + first, len - first);   // 后半段（回绕到 0）
  run_buf[(pos + len) % RUN_BUF_CAP] = '\n';
  run_head += need;
  return true;
}

// —— 假 socket：等价于 main.cpp 的 txSendNonBlocking（send(MSG_DONTWAIT)）——
// 真实现"能写就写一部分、写不动立刻返回 0"；这里用"每次 flushRunBuf 给一份字节额度"复现：
// 额度内尽量写、用完即返回 0（模拟 socket TX 满）。写出去的字节追加到 wire（TCP 是字节流）。
// 与 main.cpp 相同：只有真的写出字节（w>0）才刷新看门狗时间戳（见下一块拷贝）。
static const uint32_t TX_STALL_MS = 2000;            // 拷贝自 main.cpp（看门狗阈值）
static uint32_t tx_last_ok_ms = 0;                   // 拷贝自 main.cpp（最近成功写出字节的时刻）
static uint32_t fake_now_ms = 0;                     // 主机侧假时钟（替代 millis()）
static uint32_t fakeMillis() { return fake_now_ms; }

static bool        fake_connected    = true;         // 等价 client.connected()
static long        fake_budget_cap   = 1L << 30;     // 每次 flush 的可用额度（等价"总能写"）
static long        fake_budget       = 0;
static int         dropped_reported  = 0;            // 等价 main.cpp flush 时对 Serial 的上报
static std::string wire;                             // 收到的全部字节
static const size_t FAKE_WIRE_LIMIT  = 100000;       // 假 socket 的收端总量上限

static size_t fakeSendNonBlocking(const uint8_t* p, size_t n) {
  size_t w = 0;
  if (wire.size() >= FAKE_WIRE_LIMIT) return 0;      // 收端不再吞吐（等价 socket 彻底卡住）
  if (fake_connected && fake_budget > 0) {
    w = ((long)n < fake_budget) ? n : (size_t)fake_budget;
    if (wire.size() + w > FAKE_WIRE_LIMIT) w = FAKE_WIRE_LIMIT - wire.size();
    fake_budget -= (long)w;
    wire.append((const char*)p, w);
  }
  if (w > 0) tx_last_ok_ms = fakeMillis();           // main.cpp: 只有真写出字节才算 TX 活着
  return w;
}

void flushRunBuf() {
  fake_budget = fake_budget_cap;    // 假 socket 专用，非 main.cpp 逻辑
  size_t pend = run_head - run_tail;
  if (fake_connected && pend) {     // main.cpp: if (client && client.connected() && pend)
    size_t off = 0;
    while (off < pend) {
      size_t pos   = (run_tail + off) % RUN_BUF_CAP;
      size_t chunk = RUN_BUF_CAP - pos;
      if (chunk > pend - off) chunk = pend - off;
      size_t w = fakeSendNonBlocking((const uint8_t*)(run_buf + pos), chunk);
      if (w == 0) break;
      off += w;
    }
    run_tail += off;
  }
  if (run_tail == run_head) {               // 全部发出去了才清计数（半发出去时保留，下次接着试）
    run_pending = 0;
    if (run_dropped) { dropped_reported += run_dropped; run_dropped = 0; }   // main.cpp 此处 printf 上报
  }
}
// ===== 拷贝自 main.cpp 的 TX 停摆看门狗（checkTxStall，R62）=====
// 触发条件与 main.cpp 逐字同构；只把依赖换成假对象：
//   millis() → fakeMillis()      Serial.printf → 无
//   client.stop() → fakeClientStop()   stopAll() → fakeStopAll()
//   running / ramp_active → fake_running / fake_ramp_active
static bool fake_client_stopped = false;   // client.stop() 的代理：停掉后 connected() 即 false
static int  fake_watchdog_fires = 0;       // 记录触发次数
static bool fake_running        = false;
static bool fake_ramp_active    = false;

static void fakeClientStop() { fake_client_stopped = true; fake_watchdog_fires++; }
static void fakeStopAll()    {}

void checkTxStall() {
  if (run_head == run_tail) { tx_last_ok_ms = fakeMillis(); return; }   // 环空：不判停摆
  if ((uint32_t)(fakeMillis() - tx_last_ok_ms) <= TX_STALL_MS) return;  // 2s 内还有字节出去
  fakeClientStop();      // main.cpp: Serial.printf(...) + client.stop()
  resetRunBuf();
  fake_running = false;
  fake_ramp_active = false;
  fakeStopAll();
}
// ===== 拷贝结束（以下为测试脚手架）=====

int fails = 0;
static void check(bool ok, const char* what) {
  if (!ok) { printf("  FAIL: %s\n", what); fails++; }
  else      { printf("  ok:   %s\n", what); }
}
static std::string mkline(int i) {
  char b[32]; int n = snprintf(b, sizeof(b), "L%06d", i);
  return std::string(b, n);
}
// 采样循环里的"写一行、攒够一批就发"（对应 streamSample 的 D 分支）：返回本行是否已上线
static bool step(int i) {
  std::string before = wire;
  std::string s = mkline(i);
  if (runPushLine(s.data(), s.size())) {
    run_pending++;
    if (run_pending >= RUN_FLUSH_LINES) flushRunBuf();
  }
  return wire.size() > before.size();
}
static void pushLine(const std::string& s) { if (runPushLine(s.data(), s.size())) run_pending++; }

int main() {
  printf("ring_buf_test: CAP=%u, 每批 %d 行（main.cpp 的环形缓冲拷贝，见文件头）\n\n",
         (unsigned)RUN_BUF_CAP, RUN_FLUSH_LINES);

  // ---- T1 正常流式：每 5 行一批 ----
  printf("T1 正常流式（每 5 行一批）\n");
  resetRunBuf(); wire.clear();
  bool f[12] = {};
  for (int i = 1; i <= 11; i++) f[i] = step(i);
  check(!f[1] && !f[4], "前 4 行未发（攒批中）");
  check(f[5],  "第 5 行触发 flush —— 这是流式的关键证据");
  check(!f[6] && !f[9], "6..9 未发");
  check(f[10], "第 10 行再次 flush");
  check(run_pending == 1, "两次 flush 后 run_pending == 未满批的 1 行");
  check(wire.find("L000001") != std::string::npos, "线上含 L000001");
  check(wire.find("L000010") != std::string::npos, "线上含 L000010");

  // ---- T2 回绕：200 行跨 ~40 次回绕，逐字节完好 ----
  printf("T2 回绕 200 行不损坏、不丢已发数据\n");
  resetRunBuf(); wire.clear();
  std::string want;
  for (int i = 1; i <= 200; i++) { want += mkline(i) + "\n"; step(i); }
  flushRunBuf();
  check(dropped_reported == 0 && run_dropped == 0, "客户端在线时一行都没丢");
  check(run_head - run_tail == 0, "flush 后待发字节为 0");
  check(wire == want, "线上字节流与 200 行原文逐字节相同（跨约 40 次回绕）");

  // ---- T3 缓冲满（判满式真的成立时）：丢新行、绝不覆盖旧行 ----
  printf("T3 缓冲满：丢新不盖旧\n");
  resetRunBuf(); wire.clear(); fake_connected = false;
  for (int i = 0; i < 6; i++) pushLine("ABCDE");
  check(run_head - run_tail == 36, "缓冲里 36 字节（6 行），剩 4 字节放不下第 7 行");
  check(run_pending == 6, "run_pending == 实际入缓冲的 6 行（与写入量一致，未脱节）");
  check(runPushLine("ABCDE", 5) == false, "第 7 行被拒：缓冲满");
  check(run_dropped == 1, "被丢弃的行被计数");
  check(run_head - run_tail == 36, "被拒的行没有推进 run_head（没写进缓冲）");
  check(run_pending == 6, "被拒的行不计入 run_pending（它根本不在缓冲里）");
  fake_connected = true; wire.clear(); flushRunBuf();
  check(wire == "ABCDE\nABCDE\nABCDE\nABCDE\nABCDE\nABCDE\n", "离线期间积压的旧行完好发出");
  check(run_pending == 0, "全部发出后 run_pending 归零");

  // ---- T4 复位：三个复位点（RUN/STOP/M）共用 resetRunBuf，必须两个游标一起清零 ----
  printf("T4 复位：两个游标必须一起清零\n");
  resetRunBuf(); wire.clear(); fake_connected = true;
  pushLine(mkline(1)); flushRunBuf();      // 让 tail 也走到非零（这正是"只清一个"会踩的状态）
  pushLine(mkline(2));
  check(run_head == 16 && run_tail == 8, "复位前 head=16 tail=8（tail 非零）");
  resetRunBuf();
  check(run_head == 0 && run_tail == 0, "resetRunBuf：两个游标一起清零");
  check(run_pending == 0, "resetRunBuf：run_pending 一并清零");

  // ---- T5 不对称复位（只清 head）的真实后果：覆盖未发数据，而不是"判满丢弃" ----
  printf("T5 不对称复位（只清 head）：'还有空间' -> 覆盖未发数据\n");
  resetRunBuf(); wire.clear(); fake_connected = true;
  pushLine(mkline(1)); flushRunBuf();      // 正常发出一行（tail=8）
  fake_connected = false;                  // 从这里开始断流：下面两行发不出去
  pushLine(mkline(2));
  pushLine(mkline(3));                     // 未发数据 = [8..24) = "L000002\nL000003\n"
  std::string unsent_before(run_buf + 8, 8);
  run_head = 0;                            // 错误现场：只清了 head，tail=24 残留上一轮的值
  size_t diff = run_head - run_tail;               // size_t 下溢
  size_t free_looks_like = RUN_BUF_CAP - diff;     // 判满式回绕后的"剩余空间"
  uint32_t diff32  = (uint32_t)0 - (uint32_t)run_tail;          // ESP32（32 位 size_t）上的同名算术
  uint32_t free32  = (uint32_t)RUN_BUF_CAP - diff32;
  printf("  info: head-tail = %zu ; CAP-(head-tail) = %zu（32 位 ESP32 上 = %u / %u）\n",
         diff, free_looks_like, (unsigned)diff32, (unsigned)free32);
  check(diff > RUN_BUF_CAP, "不对称复位 => head-tail 以 size_t 回绕成巨大数");
  check(free_looks_like >= RUN_BUF_CAP, "回绕后的'剩余空间'比整个缓冲还大 => 任何一行都被判成放得下");
  check(diff32 > RUN_BUF_CAP && free32 >= RUN_BUF_CAP, "（32 位 ESP32 算术同样成立 —— 不是只有 64 位主机才有的假象）");
  fake_connected = true;
  check(runPushLine("XXXXXXXX", 8), "新行被【接受】—— 不是被判满丢弃（那是写反的说法）");
  check(runPushLine("YYYYYYYY", 8), "再来一行也被接受，写入指针继续推进");
  std::string unsent_now(run_buf + 8, 8);
  check(unsent_now != unsent_before, "没发出去的旧行已被新数据覆盖（数据静默丢失）");
  check(wire.find("L000002") == std::string::npos, "旧行从未上线 —— 覆盖 = 静默丢数据");

  // ---- T6 部分写（socket TX 满）后接着发，不重不漏 ----
  printf("T6 部分写后接着发\n");
  resetRunBuf(); wire.clear(); fake_connected = true; fake_budget_cap = 4; dropped_reported = 0;
  std::string want6;
  for (int i = 1; i <= 5; i++) { want6 += mkline(i) + "\n"; pushLine(mkline(i)); }
  check(run_pending == 5, "5 行已入缓冲");
  check(run_head - run_tail == 40, "缓冲 40 字节 = 缓冲正好装满一批");
  int calls = 0;
  while (run_head - run_tail > 0 && calls++ < 100) flushRunBuf();   // 每次只放行 4 字节然后卡住
  check(wire == want6, "每次只发 4 字节，最终字节流仍与原文逐字节相同（不重不漏）");
  check(run_pending == 0, "全部发完后 run_pending 归零");
  check(calls == 10, "40 字节 / 每次 4 字节 = 正好 10 次 flushRunBuf 才发干净");
  check(dropped_reported == 0, "部分写不会误判成缓冲满");

  // ---- T7 TX 停摆看门狗（R62）：健康流式不误触；停摆 >2 s 判僵尸 → stop + 清环 + 停车 ----
  printf("T7 TX 停摆看门狗（checkTxStall）：健康不误触 / 停摆 >2s 判僵尸\n");
  resetRunBuf(); wire.clear();
  fake_connected = true; fake_budget_cap = 16; dropped_reported = 0;
  fake_now_ms = 0; tx_last_ok_ms = 0; fake_watchdog_fires = 0;
  fake_client_stopped = false; fake_running = true; fake_ramp_active = true;

  // 健康但"慢"的流式：生产 8 B/50 ms，每 5 ms 一次 flush 发 ≤16 B ——
  // 吞吐不宽裕但**持续有字节出去**，看门狗绝不能触发（跑 10 s）。
  for (int i = 1, t = 0; t <= 10000; t += 5) {
    fake_now_ms = (uint32_t)t;
    if (t % 50 == 0) { pushLine(mkline(i++)); run_pending++; }
    flushRunBuf();
    checkTxStall();
  }
  check(fake_watchdog_fires == 0, "健康慢速流式 10 s：0 次误触发");
  check(run_head == run_tail && run_pending == 0, "健康流式：每次 flush 后环都排空");

  // 空闲：环里没有未发数据，时间过多久都不判停摆（时间戳被刷新）。
  resetRunBuf(); fake_now_ms = 20000; tx_last_ok_ms = 0; fake_watchdog_fires = 0;
  for (int t = 20000; t <= 620000; t += 1000) { fake_now_ms = (uint32_t)t; checkTxStall(); }
  check(fake_watchdog_fires == 0, "空闲 10 分钟：0 次误触发（无积压不判停摆）");

  // 僵尸停摆：连接仍"在线"（fake_connected=true）但一个字节都写不出去 —— 正是 R62。
  fake_budget_cap = 0; fake_watchdog_fires = 0; fake_client_stopped = false;
  int last_ok_ms = (int)fake_now_ms;            // 此刻 TX 还是好的（时间戳新鲜）
  int fired_at = -1;
  for (int k = 0; k <= 5000; k += 50) {
    fake_now_ms = (uint32_t)(last_ok_ms + k);
    pushLine(mkline(900 + k / 50)); run_pending++;
    flushRunBuf();                              // budget 0：一个字节也发不出去
    checkTxStall();
    if (k == 2000) check(fake_watchdog_fires == 0, "停摆满 2.000 s：边界未过，尚未触发");
    if (fake_watchdog_fires) { fired_at = k; break; }
  }
  check(fired_at == 2050, "停摆后第一个检测点（2.05 s）触发 —— 2 s 阈值边界正确");
  check(fake_client_stopped, "触发时调用了 client.stop()（僵尸被踢回可 accept 状态）");
  check(run_head == 0 && run_tail == 0 && run_pending == 0, "触发时清环（resetRunBuf）");
  check(!fake_running && !fake_ramp_active, "触发时停车（running / ramp_active 清零）");

  // 踢掉僵尸后：新会话恢复发送，看门狗不再触发。
  fake_connected = true; fake_budget_cap = 1L << 30;
  fake_watchdog_fires = 0; wire.clear();
  pushLine(mkline(7777)); run_pending++;
  flushRunBuf();
  check(wire.find("L007777") != std::string::npos, "踢掉僵尸后新会话恢复发送");
  check(tx_last_ok_ms == fake_now_ms, "恢复发送后看门狗时间戳被刷新");

  printf("\n%s (%d failure(s))\n", fails ? "FAILED" : "ALL PASS", fails);
  return fails ? 1 : 0;
}
