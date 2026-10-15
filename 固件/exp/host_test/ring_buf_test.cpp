// 主机端回归锁：把 固件/exp/src/main.cpp 的环形缓冲 + TX 路径逻辑原样拷出来，用小 CAP 跑单元测试。
// 目的：烧录前先证明「回绕不覆盖未发数据」「三个复位点必须两个游标一起清零」，并演示
// 不对称复位（只清一个游标）的真实后果 —— **覆盖未发数据**（不是"判满、静默丢弃"）；
// R64 起还锁「send() 硬错误立刻踢 / 软错误等 5 s 阈值」（见拷贝块 2/3）。
//
// 【这是拷贝，不是共享代码】下面标注"拷贝自 main.cpp"的 6 个函数 + 相关变量必须与
// main.cpp 的逻辑逐字一致（只允许两类替换：CAP 换成小值、client/send/millis 换成假依赖）。
// 拷贝块 1：环形缓冲（resetRunBuf / runPushLine）及其变量；
// 拷贝块 2：发送路径（txSendNonBlocking：TX_SEND_OK/SOFT/HARD 三分类）；
// 拷贝块 3：丢弃 + 看门狗（dropFakeTxClient ≈ dropTxClient / flushRunBuf 的环部分 / checkTxStall）——
//   "硬错误立刻踢、软错误 >5s 才踢"。
// 改了 main.cpp 的这几块，必须同步改这里，否则这条锁会腐烂成"永远绿"的假锁 ——
// 变异自检（run.sh 第 2 步）只兜底三类回归，不能代替同步。
//
// 为什么 CAP 取 40（真实是 32768）：让"回绕"和"缓冲满"在几十行内反复发生，测试才有杠杆。
// 一行 8 B × 一批 5 行 = 40 B 恰好等于 CAP —— 顺便锁住"一整批必须能整批放下"这个边界。
//
// 用法：bash 固件/exp/host_test/run.sh   （纯 g++，不需要 PlatformIO / 硬件）
#include <cerrno>
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

// —— 假 socket：等价于 main.cpp 的 send(MSG_DONTWAIT) ——
// 真实现"能写就写一部分、写不动/写失败立刻返回"；这里用"每次 flushRunBuf 给一份字节额度"
// 复现：额度内尽量写、用完即写 0（模拟零窗口）。写出去的字节追加到 wire（TCP 是字节流）。
// R64：fake_send 失败时置**真 errno**，让下面 txSendNonBlocking 拷贝里与 main.cpp 同一套
// errno case 真正被执行 —— 否则"软错误被误判成硬错误"这类回归在测试里根本触发不到。
static const uint32_t TX_STALL_MS = 5000;            // 拷贝自 main.cpp（看门狗阈值）
static uint32_t tx_last_ok_ms = 0;                   // 拷贝自 main.cpp（最近成功写出字节的时刻）
static uint32_t fake_now_ms = 0;                     // 主机侧假时钟（替代 millis()）
static uint32_t fakeMillis() { return fake_now_ms; }

static bool        fake_connected    = true;         // 等价 client.connected()（operator bool 也等于它）
static long        fake_budget_cap   = 1L << 30;     // 每次 flush 的可用额度（等价"总能写"）
static long        fake_budget       = 0;
static int         fake_fd           = 3;            // 等价 client.fd()
static int         fake_send_errno   = 0;            // 非 0：模拟 send() 返回 -1 并置 errno（EPIPE=硬 / EAGAIN=软）
static int         dropped_reported  = 0;            // 等价 main.cpp flush 时对 Serial 的上报
static std::string wire;                             // 收到的全部字节
static const size_t FAKE_WIRE_LIMIT  = 100000;       // 假 socket 的收端总量上限

static int fake_send(int fd, const void* p, size_t n) {   // main.cpp: send(fd, p, n, MSG_DONTWAIT)
  (void)fd;
  if (fake_send_errno) { errno = fake_send_errno; return -1; }   // 等价 send() 失败
  size_t w = 0;
  if (fake_connected && fake_budget > 0) {
    w = ((long)n < fake_budget) ? n : (size_t)fake_budget;
    if (wire.size() + w > FAKE_WIRE_LIMIT) w = FAKE_WIRE_LIMIT - wire.size();
    fake_budget -= (long)w;
    wire.append((const char*)p, w);
  }
  return (int)w;               // 0 = 零窗口（额度 0 / 收端不再吞吐）
}

// ===== 拷贝自 main.cpp 的发送路径（txSendNonBlocking，R64 三分类）=====
// 逐字同构；替换：client → fake_connected、client.fd() → fake_fd、send → fake_send、
// millis() → fakeMillis()。errno 的 switch 保持原样（用真 errno）。
enum TxSend : uint8_t { TX_SEND_OK, TX_SEND_SOFT, TX_SEND_HARD };

static TxSend txSendNonBlocking(const uint8_t* p, size_t n, size_t* sent) {
  *sent = 0;
  if (n == 0) return TX_SEND_SOFT;                          // 见 main.cpp：没发出字节就不算成功（否则调用方空转）
  if (!fake_connected) return TX_SEND_HARD;                 // main.cpp: if (!client || !client.connected())
  int fd = fake_fd;                                         // main.cpp: int fd = client.fd();
  if (fd < 0) return TX_SEND_HARD;                          // 没有有效 fd：没东西可发
  errno = 0;                                                // 防残留：w<0 时立刻读 errno
  int w = fake_send(fd, p, n);                              // main.cpp: int w = send(fd, p, n, MSG_DONTWAIT);
  if (w > 0) { tx_last_ok_ms = fakeMillis(); *sent = (size_t)w; return TX_SEND_OK; }
  if (w == 0) return TX_SEND_SOFT;                          // 0 字节被接受（空间不足）：可自愈，别砍
  switch (errno) {
    case EAGAIN:      // == EWOULDBLOCK（lwip）：对端零窗口 / 本端无缓冲
    case EINTR:       // 被信号打断：重试即可
    case ENOBUFS:     // lwip ERR_BUF（对端停读时常见）：暂时缺缓冲，不是连接死
    case ENOMEM:      // lwip ERR_MEM：同上
      return TX_SEND_SOFT;
    default:          // EPIPE/ECONNRESET/ENOTCONN/ECONNABORTED…：连接确实死了
      return TX_SEND_HARD;
  }
}

// ===== 拷贝自 main.cpp 的丢弃路径（dropTxClient）+ 环 flush（flushRunBuf）=====
static bool fake_client_stopped = false;   // client.stop() 的代理：停掉后 connected() 即 false
static int  fake_watchdog_fires = 0;       // 记录踢连接次数（硬错误与超时都算）
static bool fake_running        = false;
static bool fake_ramp_active    = false;

static void fakeClientStop() { fake_client_stopped = true; fake_watchdog_fires++; fake_connected = false; }
static void fakeStopAll()    {}

static void dropFakeTxClient(const char* why) {          // main.cpp: dropTxClient
  (void)why;                                             // main.cpp 此处 Serial.printf 上报 why 与 pending
  fakeClientStop();                                      // main.cpp: client.stop()
  resetRunBuf();
  fake_running = false;
  fake_ramp_active = false;
  fakeStopAll();                                         // main.cpp: stopAll()
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
      size_t w = 0;
      TxSend r = txSendNonBlocking((const uint8_t*)(run_buf + pos), chunk, &w);
      if (r == TX_SEND_HARD) { dropFakeTxClient("send() hard error"); return; }
      if (r != TX_SEND_OK) break;
      off += w;
    }
    run_tail += off;
  }
  if (run_tail == run_head) {               // 全部发出去了才清计数（半发出去时保留，下次接着试）
    run_pending = 0;
    if (run_dropped) { dropped_reported += run_dropped; run_dropped = 0; }   // main.cpp 此处 printf 上报
  }
}
// ===== 拷贝自 main.cpp 的 TX 停摆看门狗（checkTxStall，R62/R64）=====
// 触发条件与 main.cpp 逐字同构；只把依赖换成假对象：
//   millis() → fakeMillis()      dropTxClient → dropFakeTxClient（后者含 client.stop() 与停车的假代理）
void checkTxStall() {
  if (run_head == run_tail) { tx_last_ok_ms = fakeMillis(); return; }   // 环空：不判停摆
  if ((uint32_t)(fakeMillis() - tx_last_ok_ms) <= TX_STALL_MS) return;  // 5s 内还有字节出去
  dropFakeTxClient("stall >5s");     // main.cpp: dropTxClient("stall >5s")
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

  // ---- T7 TX 路径（R62/R64）：健康不误触；软错误等 5 s 阈值；硬错误立刻踢 ----
  printf("T7 TX 路径（R64）：健康不误触 / 软错误等 5s 阈值 / 硬错误立刻踢\n");
  resetRunBuf(); wire.clear();
  fake_connected = true; fake_budget_cap = 16; dropped_reported = 0;
  fake_now_ms = 0; tx_last_ok_ms = 0; fake_watchdog_fires = 0;
  fake_client_stopped = false; fake_running = true; fake_ramp_active = true;
  fake_send_errno = 0; fake_fd = 3;

  // (a) 健康但"慢"的流式：生产 8 B/50 ms，每 5 ms 一次 flush 发 ≤16 B ——
  // 吞吐不宽裕但**持续有字节出去**，看门狗绝不能触发（跑 10 s）。
  for (int i = 1, t = 0; t <= 10000; t += 5) {
    fake_now_ms = (uint32_t)t;
    if (t % 50 == 0) { pushLine(mkline(i++)); run_pending++; }
    flushRunBuf();
    checkTxStall();
  }
  check(fake_watchdog_fires == 0, "健康慢速流式 10 s：0 次误触发");
  check(run_head == run_tail && run_pending == 0, "健康流式：每次 flush 后环都排空");

  // (b) 空闲：环里没有未发数据，时间过多久都不判停摆（时间戳被刷新）。
  resetRunBuf(); fake_now_ms = 20000; tx_last_ok_ms = 0; fake_watchdog_fires = 0;
  for (int t = 20000; t <= 620000; t += 1000) { fake_now_ms = (uint32_t)t; checkTxStall(); }
  check(fake_watchdog_fires == 0, "空闲 10 分钟：0 次误触发（无积压不判停摆）");

  // (c) 软错误（零窗口 / EAGAIN）：连接仍"在线"（connected()=true）但写不动 ——
  // 绝不能立刻踢；要等满 5 s 阈值（R64：健康链路最长自愈停摆 2.32 s，不能被砍）。
  fake_send_errno = EAGAIN; fake_budget_cap = 0;
  fake_watchdog_fires = 0; fake_client_stopped = false;
  int last_ok_ms = (int)fake_now_ms;            // 此刻 TX 还是好的（时间戳新鲜）
  int fired_at = -1;
  for (int k = 0; k <= 5100; k += 50) {
    fake_now_ms = (uint32_t)(last_ok_ms + k);
    pushLine(mkline(900 + k / 50)); run_pending++;
    flushRunBuf();                              // EAGAIN：一个字节也发不出去（软错误）
    checkTxStall();
    if (k == 1000) check(fake_watchdog_fires == 0, "软错误（EAGAIN）停摆 1.000 s：不立刻踢（正确地等阈值）");
    if (k == 5000) check(fake_watchdog_fires == 0, "软错误停摆满 5.000 s：边界未过，尚未触发");
    if (fake_watchdog_fires) { fired_at = k; break; }
  }
  check(fired_at == 5050, "软错误停摆后第一个检测点（5.05 s）触发 —— 5 s 阈值边界正确");
  check(fake_client_stopped, "触发时调用了 client.stop()（僵尸被踢回可 accept 状态）");
  check(run_head == 0 && run_tail == 0 && run_pending == 0, "触发时清环（resetRunBuf）");
  check(!fake_running && !fake_ramp_active, "触发时停车（running / ramp_active 清零）");

  // (d) 软错误第二种形态：send 返回 0（零窗口，与 errno 无关）—— 同样不许立刻踢。
  resetRunBuf(); fake_connected = true; fake_send_errno = 0; fake_budget_cap = 0;
  fake_watchdog_fires = 0; fake_client_stopped = false;
  fake_running = true; fake_ramp_active = true; tx_last_ok_ms = fake_now_ms;
  pushLine(mkline(8001)); run_pending++;
  flushRunBuf(); checkTxStall();
  fake_now_ms += 100; flushRunBuf(); checkTxStall();
  check(fake_watchdog_fires == 0 && !fake_client_stopped, "软错误（send 写 0 字节）0.1 s：不立刻踢");

  // (d2) n==0：没请求发任何字节 —— 必须报"软"（不能报 OK）。
  // 报 OK 而 *sent=0 会让 flushRunBuf 的 while(off < pend) 原地空转（看门狗路径上的死循环）。
  {
    resetRunBuf(); fake_connected = true; fake_fd = 3;
    fake_send_errno = EPIPE;               // 哪怕 errno 是硬错误：n==0 根本没发，不该据此判死
    fake_budget_cap = 1L << 30;
    fake_watchdog_fires = 0; fake_client_stopped = false;
    fake_running = true; fake_ramp_active = true;
    size_t dummy_sent = 12345;
    check(txSendNonBlocking((const uint8_t*)"X", 0, &dummy_sent) == TX_SEND_SOFT,
          "n==0 报软错误（返回 OK 会让 flushRunBuf 空转）");
    check(dummy_sent == 0, "n==0 时 *sent 被清零");
    // 「没推进游标就 break」的旁证：零窗口时 flush 返回后 run_tail 不动（数据留着下次发），
    // 而不是原地空转到卡死，也没有误踢连接。
    fake_send_errno = 0; fake_budget_cap = 0;   // 纯零窗口软错误
    pushLine(mkline(8002)); run_pending++;
    flushRunBuf();
    check(run_head != run_tail && run_tail == 0, "零窗口：游标未推进（无进展即 break，没有空转）");
    check(!fake_client_stopped && fake_watchdog_fires == 0, "零窗口：不踢连接、也不触发看门狗");
    resetRunBuf();
  }

  // (e) 硬错误：连接"看起来还在"（connected() 仍 true —— 正是僵尸态）但 send 报硬错误
  // —— 必须**立刻**踢，不等 5 s 阈值。EPIPE / ECONNRESET / ENOTCONN / fd<0 四种变体。
  const char* hard_names[] = {"EPIPE", "ECONNRESET", "ENOTCONN", "fd<0"};
  for (int v = 0; v < 4; v++) {
    printf("  硬错误变体：%s（connected() 仍 true，时间戳新鲜）\n", hard_names[v]);
    resetRunBuf(); wire.clear();
    fake_connected = true; fake_budget_cap = 1L << 30; fake_fd = 3; fake_send_errno = 0;
    fake_watchdog_fires = 0; fake_client_stopped = false;
    fake_running = true; fake_ramp_active = true;
    tx_last_ok_ms = fake_now_ms;
    if (v == 0) fake_send_errno = EPIPE;
    if (v == 1) fake_send_errno = ECONNRESET;
    if (v == 2) fake_send_errno = ENOTCONN;
    if (v == 3) fake_fd = -1;
    pushLine(mkline(7000 + v)); run_pending++;
    flushRunBuf();                              // 一次发送就吃到硬错误
    check(fake_watchdog_fires == 1, "硬错误一次 flush 内立刻踢（没踢 = 漏判成软错误）");
    check(fake_client_stopped, "硬错误踢连接：client.stop()");
    check(run_head == 0 && run_tail == 0 && run_pending == 0, "硬错误清环");
    check(!fake_running && !fake_ramp_active, "硬错误停车");
    check((uint32_t)(fake_now_ms - tx_last_ok_ms) < TX_STALL_MS, "踢发生在阈值远未到时（没有走超时路径）");
  }

  // (f) 踢掉后：新会话（connected() 恢复）发送立即恢复，看门狗不再多余触发。
  resetRunBuf(); wire.clear();
  fake_connected = true; fake_fd = 3; fake_send_errno = 0; fake_budget_cap = 1L << 30;
  fake_watchdog_fires = 0; fake_client_stopped = false;
  fake_now_ms += 50;
  pushLine(mkline(7777)); run_pending++;
  flushRunBuf();
  check(wire.find("L007777") != std::string::npos, "踢掉后新会话恢复发送");
  check(tx_last_ok_ms == fake_now_ms, "恢复发送后看门狗时间戳被刷新");
  check(fake_watchdog_fires == 0, "恢复后没有多余触发");

  printf("\n%s (%d failure(s))\n", fails ? "FAILED" : "ALL PASS", fails);
  return fails ? 1 : 0;
}
