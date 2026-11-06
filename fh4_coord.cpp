// ============================================================================
//  Forza Horizon 4  ——  载具自由移动 (两遍值搜索 + DataOut 偏航角)
//  ---------------------------------------------------------------------------
//  静态镜像 (CE 实测):
//      ForzaHorizon4.exe + 0x7FDF980 + 0   -> X (float32)
//      ForzaHorizon4.exe + 0x7FDF980 + 4   -> Y (float32)
//      ForzaHorizon4.exe + 0x7FDF980 + 8   -> Z (float32)
//      三个分量连续 12 字节, 一次读完。
//
//  工作流程 (= 你描述的"2 次搜索", 值搜索, 与指针无关):
//      0) 闸门: 坐标必须有效(非 0)。游戏没完全进去时镜像读出来是 0/0/0,
//         这种时候不搜索, 程序会等你进驾驶状态。
//      1) 第一遍: 以镜像当前 XYZ 为基准, 在整个可写内存里找"连续 3 个 float
//         等于这组 XYZ"的地址 -> 候选列表。
//      2) 你开动车(或等车动起来)。程序检测到位移超过阈值后, 做第二遍:
//         用"增量比对"筛候选 —— 要求 (候选现在的值 - 候选第一遍的值)
//         与 (镜像现在 - 镜像第一遍) 一致。没跟着动的假地址全被干掉,
//         这样比直接比绝对值稳得多(不受 1 帧延迟影响)。
//      3) 剩下的就是所有"活的"坐标副本。默认全部写;
//         也可以用你惯用的启发: --heuristic 丢掉地址序末尾 3 个、取倒数 5 个。
//      4) 方向来自 Data Out 的偏航角 (324 字节包, Yaw 在 offset 56)。
//
//  编译 (VS2026 / MSVC 14.51 x64 已验证):
//      build.bat
//  或手动:
//      cl /utf-8 /EHsc /O2 /std:c++17 /Fe:fh4_coord.exe fh4_coord.cpp
//
//  ? /utf-8 不能省 (源码含中文)
//  ? 需要管理员权限 (程序会自己尝试提权重启)
//  ? 只在单人 / 离线模式使用
// ============================================================================

#define _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX            // 不加这个 windows.h 的 min/max 宏会打断 std::min/std::max

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <tlhelp32.h>
#include <psapi.h>
#include <shellapi.h>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <conio.h>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>

#include <d3d11.h>
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#pragma comment(lib, "psapi.lib")
#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

// ============================== 配置 ========================================

static const wchar_t* PROC_NAME = L"ForzaHorizon4.exe";
static const uint64_t COORD_RVA = 0x7FDF980ULL;
static const int      FH4_PACKET_LEN = 324;

struct Vec3 { float x, y, z; };

// 用户在命令行点名的地址(比如你在 CE 里已经确认能推动车的那一个)。
// 支持 "--addr 0x1C5A4332530" 或 "--addr exe+7ABCDE"
struct AddrSpec { bool exeRelative; uint64_t value; };

struct Options {
    // 搜索
    double tol1        = 0.05;   // 第一遍容差(米); 车停稳时用小的
    double tol1Moving  = 2.0;    // 车在动时自动放宽到这个值
    double tolDelta    = 0.75;   // 第二遍"增量比对"容差(米)
    double moveNeed    = 5.0;    // 需要观察到的位移(米)才做第二遍
    // 等位移最长秒数; 0 = 一直等。
    // ? 车不动时"增量比对"是退化的(位移 0 -> 判据变成"值没变就算通过"),
    //   会把所有静态假地址留下。上次就是因此把垃圾地址写进游戏、把它写崩的。
    //   所以: 超时不再当作有效结果, 而是强制禁用写入。
    double moveTimeout = 0.0;
    bool   allRegions  = false;  // 默认只扫可写私有页
    int    threads     = 0;      // 0 = 自动

    // 选择
    bool   heuristic   = false;  // 可选的"丢末尾/取倒数"启发(默认不用)
    int    dropBottom  = 3;
    int    takeLast    = 5;
    int    maxTargets  = 10;     // 最多写几个地址(你说的"顶死最后15个")
    int    pick        = -1;     // 只写打印列表里的第 N 个(0 基)
    bool   writeAll    = false;  // 强制全部写(默认只写启发式选中的少数)
    bool   writeStatic = false;  // 是否也写静态镜像(默认不写: 它可能被游戏校验)
    bool   armAtStart  = false;  // 启动就武装写入(默认只读, F9 才武装)
    double maxCoord    = 20000.0;// 写之前的值域护栏(米), 超范围拒绝写
    std::vector<AddrSpec> extraAddrs;   // --addr 点名的地址
    // 哪个分量是高度: 0=X 1=Y 2=Z
    // 这一版确认: Y 是高度轴, 地面 = X-Z 平面 (水平移动只动 X 和 Z)
    int    heightAxis  = 1;
    std::string addrPrefix;      // --addr-prefix 0x7 : 只保留地址以该前缀开头的候选

    // 移动
    double speed       = 25.0;
    // 按键 (默认就是你现在用的这套; 想换改 fh4_keys.ini, 或用 --key-* 参数)
    int    keyFwd      = 'I';
    int    keyBack     = 'K';
    int    keyLeft     = 'J';
    int    keyRight    = 'L';
    int    keyUp       = 'U';
    int    keyDown     = 'O';
    double yawOffsetDeg= 0.0;
    bool   invertFwd   = false;
    bool   invertRight = false;
    bool   invertUp    = false;
    int    yawOffBytes = 56;
    int    port        = 5300;

    // 闸门 / 其他
    double zeroEps     = 0.01;
    bool   noWait      = false;
    double waitTimeout = 0.0;
    bool   listOnly    = false;
    bool   pointerMode = false;
    bool   selfTest    = false;
    bool   guiDemo     = false;   // 只看界面, 不连游戏
    int    demoSecs    = 0;       // demo 模式自动关闭秒数(0=不自动关)
    bool   noResidency = false;   // 关掉"只扫常驻页"的优化
    bool   ignoreLastCheck = false; // 忽略"最后一个必须是 exe+偏移"的校验
    bool   showHelp    = false;   // 只是打印帮助: 退出时不要 system("pause") 卡住脚本
};

static Options g_opt;
static unsigned g_keySetMask = 0;   // 命令行显式指定过的按键位(1..6)
uint64_t g_moduleSize = 0;      // 主模块映像大小 (用于打印 exe+偏移)
uint64_t g_moduleBase = 0;      // 主模块基址

// 真身在模块映像里(0x7FF6xxxx, 即 ForzaHorizon4.exe+偏移), 不在堆里。
static bool IsInModule(uint64_t a) {
    return g_moduleBase && a >= g_moduleBase && a < g_moduleBase + g_moduleSize;
}

// ============================== 基础 ========================================

static void InitConsole() {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
}

static bool IsElevated() {
    BOOL elevated = FALSE;
    HANDLE token = nullptr;
    if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token)) {
        TOKEN_ELEVATION te{};
        DWORD sz = sizeof(te);
        if (GetTokenInformation(token, TokenElevation, &te, sz, &sz))
            elevated = te.TokenIsElevated;
        CloseHandle(token);
    }
    return elevated != FALSE;
}

static bool RelaunchAsAdmin() {
    wchar_t path[MAX_PATH]{};
    if (!GetModuleFileNameW(nullptr, path, MAX_PATH)) return false;
    SHELLEXECUTEINFOW sei{};
    sei.cbSize = sizeof(sei);
    sei.fMask  = SEE_MASK_NOCLOSEPROCESS;
    sei.lpVerb = L"runas";
    sei.lpFile = path;
    sei.nShow  = SW_SHOWNORMAL;
    return ShellExecuteExW(&sei) != FALSE;
}

static bool EnableDebugPrivilege() {
    HANDLE token = nullptr;
    if (!OpenProcessToken(GetCurrentProcess(),
                          TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &token))
        return false;
    TOKEN_PRIVILEGES tp{};
    tp.PrivilegeCount = 1;
    tp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;
    if (!LookupPrivilegeValueW(nullptr, L"SeDebugPrivilege", &tp.Privileges[0].Luid)) {
        CloseHandle(token);
        return false;
    }
    SetLastError(ERROR_SUCCESS);
    BOOL ok = AdjustTokenPrivileges(token, FALSE, &tp, sizeof(tp), nullptr, nullptr);
    DWORD err = GetLastError();
    CloseHandle(token);
    return ok && err == ERROR_SUCCESS;
}

static DWORD FindProcessId(const wchar_t* name) {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return 0;
    PROCESSENTRY32W pe{};
    pe.dwSize = sizeof(pe);
    DWORD pid = 0;
    if (Process32FirstW(snap, &pe)) {
        do {
            if (lstrcmpiW(pe.szExeFile, name) == 0) { pid = pe.th32ProcessID; break; }
        } while (Process32NextW(snap, &pe));
    }
    CloseHandle(snap);
    return pid;
}

static uint64_t GetModuleBase(DWORD pid, const wchar_t* module) {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, pid);
    if (snap == INVALID_HANDLE_VALUE) return 0;
    MODULEENTRY32W me{};
    me.dwSize = sizeof(me);
    uint64_t base = 0;
    if (Module32FirstW(snap, &me)) {
        do {
            if (lstrcmpiW(me.szModule, module) == 0) {
                base = reinterpret_cast<uint64_t>(me.modBaseAddr);
                break;
            }
        } while (Module32NextW(snap, &me));
    }
    CloseHandle(snap);
    return base;
}

static bool ReadMem(HANDLE h, uint64_t addr, void* buf, size_t n) {
    SIZE_T rd = 0;
    return ReadProcessMemory(h, reinterpret_cast<LPCVOID>(addr), buf, n, &rd) && rd == n;
}

static bool WriteMem(HANDLE h, uint64_t addr, const void* buf, size_t n) {
    SIZE_T wr = 0;
    return WriteProcessMemory(h, reinterpret_cast<LPVOID>(addr), buf, n, &wr) && wr == n;
}

static bool ReadVec3(HANDLE h, uint64_t addr, Vec3& out) {
    return ReadMem(h, addr, &out, sizeof(out));
}

static bool WriteVec3(HANDLE h, uint64_t addr, const Vec3& in) {
    return WriteMem(h, addr, &in, sizeof(in));
}

// 游戏没进入驾驶状态时, 镜像坐标是 0/0/0 —— 这时不搜、不写。
static bool IsCoordValid(const Vec3& v, double eps) {
    return !(std::fabs((double)v.x) < eps &&
             std::fabs((double)v.y) < eps &&
             std::fabs((double)v.z) < eps);
}

// 关键: 不能用 "fabs(diff) > tol 就跳过" —— 差值是 NaN 时该比较恒为 false,
// 于是被 0xFF 填充的内存(NaN)会全部被当成命中。必须反过来写成 !(<= tol)。
static inline bool NearF(float v, float target, double tol) {
    return std::fabs((double)v - (double)target) <= tol;
}
static inline bool Finite3(float x, float y, float z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

// 地面平面上的位移(不算高度), 用于"车动了多少米"的判断
static double GroundDist(const Vec3& a, const Vec3& b) {
    int hx = g_opt.heightAxis;
    int ax = (hx == 0) ? 1 : 0;
    int ay = (hx == 2) ? 1 : 2;
    const float* pa = reinterpret_cast<const float*>(&a);
    const float* pb = reinterpret_cast<const float*>(&b);
    double d1 = (double)pb[ax] - pa[ax];
    double d2 = (double)pb[ay] - pa[ay];
    return std::sqrt(d1 * d1 + d2 * d2);
}

// ============================== 内存区域 ====================================

struct Region { uint64_t base; uint64_t size; DWORD protect; DWORD type; };

static bool IsWritable(DWORD p) {
    DWORD b = p & 0xFF;
    return b == PAGE_READWRITE || b == PAGE_WRITECOPY ||
           b == PAGE_EXECUTE_READWRITE || b == PAGE_EXECUTE_WRITECOPY;
}

static bool IsReadable(DWORD p) {
    if (p & PAGE_GUARD) return false;
    if (p & PAGE_NOACCESS) return false;
    return true;
}

static std::vector<Region> EnumRegions(HANDLE h) {
    std::vector<Region> out;
    uint64_t addr = 0;
    MEMORY_BASIC_INFORMATION mbi{};
    while (addr < 0x00007FFFFFFF0000ULL) {
        if (!VirtualQueryEx(h, reinterpret_cast<LPCVOID>(addr), &mbi, sizeof(mbi))) break;
        if (mbi.State == MEM_COMMIT && mbi.RegionSize > 0) {
            Region r{};
            r.base    = reinterpret_cast<uint64_t>(mbi.BaseAddress);
            r.size    = mbi.RegionSize;
            r.protect = mbi.Protect;
            r.type    = mbi.Type;
            // 默认扫"所有可写的已提交页", 不区分类型:
            //   MEM_PRIVATE(堆) / MEM_IMAGE(模块映像) / MEM_MAPPED(映射视图) 全都要。
            // 曾经只扫 MEM_PRIVATE, 后来又只优先模块, 都是错的 —— 真身的地址
            // 既不一定在堆也不一定在模块里, 别替游戏假设它把数据放哪。
            bool ok = g_opt.allRegions ? IsReadable(mbi.Protect) : IsWritable(mbi.Protect);
            if (ok && r.size >= 0x1000) out.push_back(r);
        }
        uint64_t next = reinterpret_cast<uint64_t>(mbi.BaseAddress) + mbi.RegionSize;
        if (next <= addr) break;
        addr = next;
    }
    return out;
}

// ============================== 偏航角监听 ==================================

struct Telemetry {
    volatile LONG  hasPacket;
    volatile LONG  seq;
    volatile float yaw;
    volatile float posX, posY, posZ;
};
static Telemetry g_telem{};

static float F32At(const BYTE* p, int off) {
    float v; std::memcpy(&v, p + off, 4); return v;
}

static bool ParsePacket(const BYTE* buf, int len, Telemetry& t, int yawOff) {
    if (len != FH4_PACKET_LEN) return false;
    t.yaw  = F32At(buf, yawOff);
    t.posX = F32At(buf, 232);
    t.posY = F32At(buf, 236);
    t.posZ = F32At(buf, 240);
    InterlockedIncrement(&t.seq);
    InterlockedExchange(&t.hasPacket, 1);
    return true;
}

static DWORD WINAPI UdpThread(LPVOID) {
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 0;
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) { WSACleanup(); return 0; }
    BOOL yes = TRUE;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
    sockaddr_in a{};
    a.sin_family      = AF_INET;
    a.sin_port        = htons((u_short)g_opt.port);
    a.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (bind(s, (sockaddr*)&a, sizeof(a)) == SOCKET_ERROR) {
        printf("[!] 绑定 UDP %d 失败 (错误码 %d)。\n",
               g_opt.port, WSAGetLastError());
        printf("    要么关掉它, 要么让它把包转发到别的端口, 然后用 --port 指定。\n");
        closesocket(s); WSACleanup();
        return 0;
    }
    printf("[+] 监听 UDP %d 中\n", g_opt.port);
    BYTE buf[2048];
    for (;;) {
        int n = recv(s, (char*)buf, sizeof(buf), 0);
        if (n <= 0) continue;
        if (!ParsePacket(buf, n, g_telem, g_opt.yawOffBytes)) {
            static volatile LONG warned = 0;
            if (InterlockedExchange(&warned, 1) == 0)
                printf("[!] 收到 %d 字节包(不是 324 字节的 FH4 格式), 请检查 Data Out 设置\n", n);
        }
    }
}

// 主线程里等包: seq 还在变 = Data Out 正在发数据
static bool WaitForDataOut(DWORD timeoutMs) {
    LONG seq = g_telem.seq;
    DWORD t0 = GetTickCount();
    while (GetTickCount() - t0 < timeoutMs) {
        if (g_telem.seq != seq) return true;
        Sleep(50);
    }
    return false;
}

// ============================== 第一遍: 值搜索 ==============================

// 一个区域里"常驻物理内存"的连续段。
// 为什么要这一步: 游戏进程里有 2-3% 的页已经被换出到页面文件了, 读它们每一页
// 都要触发一次换入(约 1 毫秒)。8 GB 里如果 190 MB 不在内存, 就是约 4.8 万次
// 换入 ≈ 50 秒 —— 实测全量扫描 27-68 秒, 全耗在这上面。
// QueryWorkingSetEx 花 0.3 秒就能问清楚哪些页在物理内存里, 把其余的跳掉。
struct PageRun { uint64_t off; uint64_t len; };

static void QueryResidentRuns(HANDLE h, uint64_t base, uint64_t size,
                              std::vector<PageRun>& runs,
                              std::vector<PSAPI_WORKING_SET_EX_INFORMATION>& scratch) {
    const int BATCH = 4096;
    scratch.resize(BATCH);
    uint64_t pages = size / 4096ULL;
    for (uint64_t p = 0; p < pages; p += BATCH) {
        int n = (int)std::min<uint64_t>(BATCH, pages - p);
        for (int i = 0; i < n; i++) {
            scratch[i].VirtualAddress = (PVOID)(base + (p + i) * 4096ULL);
            scratch[i].VirtualAttributes.Valid = 0;
        }
        if (!QueryWorkingSetEx(h, scratch.data(),
                              n * (int)sizeof(PSAPI_WORKING_SET_EX_INFORMATION)))
            return;   // 查询失败 -> 上层退回"整区域扫"
        for (int i = 0; i < n; i++) {
            if (scratch[i].VirtualAttributes.Valid) {
                uint64_t off = (p + i) * 4096ULL;
                if (!runs.empty() && runs.back().off + runs.back().len == off)
                    runs.back().len += 4096ULL;
                else
                    runs.push_back({ off, 4096ULL });
            }
        }
    }
}

// 在所有目标区域里找"连续 3 个 float 约等于 t"的地址 (4 字节对齐)
static std::vector<uint64_t> ScanTriples(HANDLE h, const Vec3& t, double tol,
                                         uint64_t* scannedMB, bool skipNonResident = true) {
    std::vector<Region> regions = EnumRegions(h);
    const size_t CHUNK = 4u * 1024 * 1024;
    std::atomic<int> nextIdx{0};
    std::atomic<uint64_t> totalRead{0};
    std::atomic<uint64_t> residentBytes{0};
    std::mutex mtx;
    std::vector<uint64_t> hits;

    int hw = (int)std::thread::hardware_concurrency();
    if (hw <= 0) hw = 4;
    int nthreads = g_opt.threads > 0 ? g_opt.threads : std::min(hw, 8);

    auto worker = [&]() {
        std::vector<BYTE> buf(CHUNK + 32);
        std::vector<uint64_t> local;
        std::vector<PageRun> runs;
        std::vector<PSAPI_WORKING_SET_EX_INFORMATION> scratch;

        // 扫描一段连续的常驻内存
        auto scanSpan = [&](uint64_t spanBase, uint64_t spanLen) {
            uint64_t off = 0;
            while (off + 12 <= spanLen) {
                size_t want = (size_t)std::min<uint64_t>(CHUNK, spanLen - off);
                if (want < 12) break;
                if (!ReadMem(h, spanBase + off, buf.data(), want)) break;
                totalRead.fetch_add(want);
                for (size_t i2 = 0; i2 + 12 <= want; i2 += 4) {
                    float x, y, z;
                    std::memcpy(&x, buf.data() + i2,     4);
                    if (!NearF(x, t.x, tol)) continue;
                    std::memcpy(&y, buf.data() + i2 + 4, 4);
                    if (!NearF(y, t.y, tol)) continue;
                    std::memcpy(&z, buf.data() + i2 + 8, 4);
                    if (!NearF(z, t.z, tol)) continue;
                    if (!Finite3(x, y, z)) continue;
                    local.push_back(spanBase + off + i2);
                }
                uint64_t step = (want >= 8) ? (uint64_t)want - 8 : (uint64_t)want;
                if (step == 0) break;
                off += step;
            }
        };

        for (;;) {
            int i = nextIdx.fetch_add(1);
            if (i >= (int)regions.size()) break;
            const Region& r = regions[i];
            if (skipNonResident) {
                runs.clear();
                QueryResidentRuns(h, r.base, r.size, runs, scratch);
                if (runs.empty()) continue;
                uint64_t res = 0;
                for (const auto& run : runs) {
                    res += run.len;
                    scanSpan(r.base + run.off, run.len);
                }
                residentBytes.fetch_add(res);
            } else {
                scanSpan(r.base, r.size);
            }
        }
        std::lock_guard<std::mutex> lk(mtx);
        hits.insert(hits.end(), local.begin(), local.end());
    };

    std::vector<std::thread> pool;
    for (int i = 0; i < nthreads; i++) pool.emplace_back(worker);
    for (auto& th : pool) th.join();

    std::sort(hits.begin(), hits.end());
    hits.erase(std::unique(hits.begin(), hits.end()), hits.end());
    if (scannedMB) *scannedMB = totalRead.load() / (1024 * 1024);
    if (skipNonResident)
        printf("    (常驻内存 %llu MB, 跳过了不在物理内存里的页)\n",
               (unsigned long long)(residentBytes.load() / (1024 * 1024)));
    return hits;
}

// 快速重查: 只读这些地址, 保留当前值仍然约等于 t 的
static std::vector<uint64_t> RecheckTriples(HANDLE h, const std::vector<uint64_t>& addrs,
                                            const Vec3& t, double tol) {
    std::vector<uint64_t> keep;
    keep.reserve(addrs.size());
    for (uint64_t a : addrs) {
        Vec3 v{};
        if (!ReadVec3(h, a, v)) continue;
        if (NearF(v.x, t.x, tol) && NearF(v.y, t.y, tol) && NearF(v.z, t.z, tol))
            keep.push_back(a);
    }
    return keep;
}

// 第二遍: 增量比对 —— 候选的变化量必须和镜像的变化量一致。
// 比直接比绝对值稳得多: 不受刷新延迟影响, 而且"没跟着动"的假地址一律出局。
static std::vector<uint64_t> RecheckByDelta(HANDLE h, const std::vector<uint64_t>& addrs,
                                           const Vec3& v1, const Vec3& p0,
                                           const Vec3& pNow, double tol) {
    double dx = (double)pNow.x - p0.x;
    double dy = (double)pNow.y - p0.y;
    double dz = (double)pNow.z - p0.z;
    std::vector<uint64_t> keep;
    keep.reserve(addrs.size());
    for (uint64_t a : addrs) {
        Vec3 v2{};
        if (!ReadVec3(h, a, v2)) continue;
        double ax = (double)v2.x - v1.x;
        double ay = (double)v2.y - v1.y;
        double az = (double)v2.z - v1.z;
        if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) continue;
        if (std::fabs(ax - dx) <= tol &&
            std::fabs(ay - dy) <= tol &&
            std::fabs(az - dz) <= tol) keep.push_back(a);
    }
    return keep;
}

// ============================== 指针模式 (可选) =============================

struct Candidate { uint64_t slot; uint64_t target; };

static std::vector<Candidate> ScanPointers(HANDLE h, uint64_t lo, uint64_t hi) {
    std::vector<Candidate> found;
    std::vector<Region> regions = EnumRegions(h);
    const size_t CHUNK = 8u * 1024 * 1024;
    std::vector<BYTE> buf(CHUNK);
    for (const Region& r : regions) {
        if (r.type != MEM_PRIVATE && !g_opt.allRegions) continue;
        for (uint64_t off = 0; off < r.size; off += CHUNK) {
            size_t want = (size_t)std::min<uint64_t>(CHUNK, r.size - off);
            want &= ~7ULL;
            if (want == 0) break;
            if (!ReadMem(h, r.base + off, buf.data(), want)) continue;
            for (size_t i = 0; i + 8 <= want; i += 8) {
                uint64_t v; std::memcpy(&v, buf.data() + i, 8);
                if (v >= lo && v <= hi) found.push_back({ r.base + off + i, v });
            }
        }
    }
    return found;
}

// ============================== 工具 ========================================

// 程序所在目录 (用来放按键配置文件)
static std::string ExeDir() {
    char path[MAX_PATH]{};
    GetModuleFileNameA(nullptr, path, MAX_PATH);
    std::string s = path;
    size_t p = s.find_last_of("\\/");
    return (p == std::string::npos) ? std::string(".") : s.substr(0, p);
}

// ------------------------------ 按键配置 ------------------------------------
// 一个动作一个键。值可以是单个字符(I / W / ; ), 也可以是名字:
//   UP DOWN LEFT RIGHT SPACE TAB SHIFT CTRL ALT ENTER BACK
//   NUM0..NUM9 (小键盘) / F1..F12 / 0x5A (十六进制键码)
static int ParseKeyName(const std::string& raw) {
    std::string s;
    for (char c : raw) {
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
        s.push_back((char)std::toupper((unsigned char)c));
    }
    if (s.empty()) return 0;
    if (s == "UP")    return VK_UP;
    if (s == "DOWN")  return VK_DOWN;
    if (s == "LEFT")  return VK_LEFT;
    if (s == "RIGHT") return VK_RIGHT;
    if (s == "SPACE") return VK_SPACE;
    if (s == "TAB")   return VK_TAB;
    if (s == "SHIFT") return VK_SHIFT;
    if (s == "CTRL")  return VK_CONTROL;
    if (s == "ALT")   return VK_MENU;
    if (s == "ENTER") return VK_RETURN;
    if (s == "BACK")  return VK_BACK;
    if (s.rfind("NUM", 0) == 0 && s.size() == 4 && isdigit((unsigned char)s[3]))
        return VK_NUMPAD0 + (s[3] - '0');
    if (s.size() >= 2 && s[0] == 'F' && isdigit((unsigned char)s[1])) {
        int n = atoi(s.c_str() + 1);
        if (n >= 1 && n <= 12) return VK_F1 + (n - 1);
    }
    if (s.size() > 2 && s[0] == '0' && s[1] == 'X')
        return (int)strtoull(s.c_str() + 2, nullptr, 16);
    if (s.size() == 1) return (int)(unsigned char)s[0];   // 单字符(已转大写)
    return 0;                                             // 不认识
}

static std::string KeyName(int vk) {
    if ((vk >= 'A' && vk <= 'Z') || (vk >= '0' && vk <= '9'))
        return std::string(1, (char)vk);
    if (vk >= VK_NUMPAD0 && vk <= VK_NUMPAD9) return "NUM" + std::to_string(vk - VK_NUMPAD0);
    if (vk >= VK_F1 && vk <= VK_F12)          return "F" + std::to_string(vk - VK_F1 + 1);
    switch (vk) {
        case VK_UP: return "UP";       case VK_DOWN: return "DOWN";
        case VK_LEFT: return "LEFT";   case VK_RIGHT: return "RIGHT";
        case VK_SPACE: return "SPACE"; case VK_TAB: return "TAB";
        case VK_SHIFT: return "SHIFT"; case VK_CONTROL: return "CTRL";
        case VK_MENU: return "ALT";    case VK_RETURN: return "ENTER";
        case VK_BACK: return "BACK";
        case VK_OEM_1: return ";";     case VK_OEM_2: return "/";
        case VK_OEM_3: return "`";     case VK_OEM_4: return "[";
        case VK_OEM_5: return "\\";    case VK_OEM_6: return "]";
        case VK_OEM_7: return "'";     case VK_OEM_PLUS: return "=";
        case VK_OEM_MINUS: return "-"; case VK_OEM_COMMA: return ",";
        case VK_OEM_PERIOD: return ".";
    }
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%02X", vk);
    return buf;
}

static std::string KeysPath() { return ExeDir() + "\\fh4_keys.ini"; }

// 没有配置文件就按当前设置写一份出来, 方便你直接改
static void SaveKeys() {
    FILE* f = fopen(KeysPath().c_str(), "w");
    if (!f) return;
    fprintf(f, "# fh4_coord 按键设置 —— 改完保存, 下次启动生效\n");
    fprintf(f, "#\n");
    fprintf(f, "# 值可以写:\n");
    fprintf(f, "#   单个字符      I  W  ;  /  =  ,  .    (字母会自动转大写)\n");
    fprintf(f, "#   名字          UP DOWN LEFT RIGHT SPACE TAB SHIFT CTRL ALT ENTER BACK\n");
    fprintf(f, "#   小键盘        NUM0 NUM1 ... NUM9\n");
    fprintf(f, "#   功能键        F1 ... F12\n");
    fprintf(f, "#   十六进制     0x5A\n");
    fprintf(f, "#\n");
    fprintf(f, "# 6 个动作各一行, 别用重复的键(重复了会一起触发)\n");
    fprintf(f, "# 注意: 别用游戏自己的驾驶键(W/A/S/D), 否则游戏也在动, 分不清是谁推的\n");
    fprintf(f, "\n[keys]\n");
    fprintf(f, "forward = %s\n", KeyName(g_opt.keyFwd).c_str());
    fprintf(f, "back    = %s\n", KeyName(g_opt.keyBack).c_str());
    fprintf(f, "left    = %s\n", KeyName(g_opt.keyLeft).c_str());
    fprintf(f, "right   = %s\n", KeyName(g_opt.keyRight).c_str());
    fprintf(f, "up      = %s\n", KeyName(g_opt.keyUp).c_str());
    fprintf(f, "down    = %s\n", KeyName(g_opt.keyDown).c_str());
    fclose(f);
    printf("[*] 已生成按键配置文件(默认 I/K/J/L/U/O): %s\n", KeysPath().c_str());
}

// 读配置; 命令行显式给过的键优先(用掩码位标记)
static void LoadKeys(unsigned setMask) {
    FILE* f = fopen(KeysPath().c_str(), "r");
    if (!f) { SaveKeys(); return; }
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '[' || line[0] == '\n' || line[0] == '\r') continue;
        char* eq = strchr(line, '=');
        if (!eq) continue;
        *eq = 0;
        std::string k = line, v = eq + 1;
        // 去掉空白
        auto trim = [](std::string& s) {
            while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n')) s.pop_back();
            size_t i = 0; while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) i++;
            s = s.substr(i);
        };
        trim(k); trim(v);
        for (char& c : k) c = (char)std::tolower((unsigned char)c);
        int vk = ParseKeyName(v);
        if (!vk) {
            if (!v.empty()) printf("[!] 按键配置里这行看不懂, 已忽略: %s = %s\n", k.c_str(), v.c_str());
            continue;
        }
        if      (k == "forward") { if (!(setMask & 1)) g_opt.keyFwd   = vk; }
        else if (k == "back")    { if (!(setMask & 2)) g_opt.keyBack  = vk; }
        else if (k == "left")    { if (!(setMask & 4)) g_opt.keyLeft  = vk; }
        else if (k == "right")   { if (!(setMask & 8)) g_opt.keyRight = vk; }
        else if (k == "up")      { if (!(setMask & 16)) g_opt.keyUp    = vk; }
        else if (k == "down")    { if (!(setMask & 32)) g_opt.keyDown  = vk; }
    }
    fclose(f);
    printf("[*] 按键(来自 %s): 前进=%s 后退=%s 左=%s 右=%s 上升=%s 下降=%s\n",
           KeysPath().c_str(),
           KeyName(g_opt.keyFwd).c_str(),  KeyName(g_opt.keyBack).c_str(),
           KeyName(g_opt.keyLeft).c_str(), KeyName(g_opt.keyRight).c_str(),
           KeyName(g_opt.keyUp).c_str(),   KeyName(g_opt.keyDown).c_str());
    // 重复键提醒
    int all[6] = { g_opt.keyFwd, g_opt.keyBack, g_opt.keyLeft,
                   g_opt.keyRight, g_opt.keyUp, g_opt.keyDown };
    for (int i = 0; i < 6; i++)
        for (int j = i + 1; j < 6; j++)
            if (all[i] == all[j])
                printf("[!] 注意: 有两个动作用了同一个键(%s), 会一起触发\n", KeyName(all[i]).c_str());
}

static void FmtAddr(char* out, size_t n, uint64_t v, uint64_t base) {
    extern uint64_t g_moduleSize;
    if (v >= base && v < base + g_moduleSize)
        std::snprintf(out, n, "0x%016llX  (exe+%llX)",
                      (unsigned long long)v, (unsigned long long)(v - base));
    else
        std::snprintf(out, n, "0x%016llX  (堆/私有区)", (unsigned long long)v);
}

// 是否收到过 Data Out 包(启动时 WaitForDataOut 已经保证过一次)
static bool DataOutReady() {
    return InterlockedCompareExchange(&g_telem.hasPacket, 1, 1) == 1;
}

static double YawForMove() {
    double deg = (double)g_telem.yaw * 180.0 / 3.14159265358979323846;
    return deg + g_opt.yawOffsetDeg;
}

static void PrintHelp() {
    printf(
        "用法: fh4_coord.exe [选项]\n"
        "\n搜索相关:\n"
        "  --tol1 F         第一遍容差(米), 默认 0.05 (车停稳时用)\n"
        "  --tol1-moving F  车在动时自动放宽到, 默认 2.0\n"
        "  --tol-delta F    第二遍增量比对容差(米), 默认 0.75\n"
        "  --move-need F    需要看到多少米位移才做第二遍, 默认 5\n"
        "  --move-timeout F 等位移最长秒数, 默认 0 = 一直等(推荐)\n"
        "                   超时不再当作有效结果, 而是禁用写入\n"
        "  --all-regions    连映像/只读页一起搜(更慢)\n"
        "  --threads N      扫描线程数, 默认 min(核数,8)\n"
        "  --pointer        改用指针搜索模式(找指向镜像的槽位)\n"
        "\n选择与写入:\n"
        "  --max-targets N  最多写几个地址 (默认 15)\n"
        "  --pick N         只写打印列表里的第 N 个 (0 基, 用来点名真身)\n"
        "  --addr-prefix X  只保留地址以该前缀开头的候选, 例: --addr-prefix 0x7\n"
        "  --addr HEX       直接点名一个地址 (可重复, 最多 15 个; 也支持 exe+偏移)\n"
        "  --heuristic      改用老启发: 丢地址序末尾 3 个, 取倒数 5 个\n"
        "  --drop-bottom N  自定义丢掉末尾几个\n"
        "  --take-last M    自定义取倒数几个\n"
        "  --write-all      强制写全部候选(候选多于 64 时默认自动改用启发)\n"
        "  --no-static      不写静态镜像, 只写找到的地址\n"
        "\n闸门:\n"
        "  --no-wait        坐标是 0 也照搜(不建议)\n"
        "  --ignore-last-check  忽略\"最后一个地址必须是 exe+偏移\"的校验 (不推荐)\n"
        "  --no-residency   关掉\"只扫常驻页\"的优化\n"
        "  --wait-timeout N 等坐标有效的上限秒数, 默认 0 = 一直等\n"
        "  --zero-eps F     判定坐标是 0 的阈值, 默认 0.01\n"
        "\n移动:\n"
        "  --speed F        基础速度 m/s, 默认 25\n"
        "  --height-axis A  哪个分量是高度: x / y / z (默认 y; 实测 FH4 高度是 Y)\n"
        "                   地面平面 = 另外两个轴; 默认下 yaw=0 车头朝 +Z, yaw=90 朝 +X\n"
        "                   即 X += sin(yaw)*v, Z += cos(yaw)*v\n"
        "                   方向不对先调 --yaw-offset, 再试 --invert-fwd / --invert-right\n"
        "  --yaw-offset DEG 偏航角修正, 默认 0 (方向不对先调这个)\n"
        "  --yaw-bytes N    Data Out 包里 Yaw 偏移, 默认 56\n"
        "  --port N         Data Out 端口, 默认 5300\n"
        "  --invert-fwd / --invert-right / --invert-up  反向\n"
        "\n其他:\n"
        "  --list           只跑搜索并列出结果, 不进交互\n"
        "  --selftest       自检(造包验证解析), 不需要游戏\n"
        "  -h, --help       帮助\n"
        "\n移动按键 (可自定义, 默认 I/K/J/L/U/O):\n"
        "  默认: I 前进   K 后退   J 左移   L 右移   U 上升   O 下降\n"
        "  改法一: 编辑同目录下的 fh4_keys.ini (第一次运行会自动生成, 里面写了格式)\n"
        "  改法二: 命令行 --key-fwd W --key-left A --key-up SPACE ...\n"
        "          值可写单字符(I / W / ;), 或名字 UP DOWN LEFT RIGHT SPACE TAB SHIFT\n"
        "          CTRL ALT ENTER BACK NUM0-NUM9 F1-F12 0x5A(十六进制)\n"
        "  提醒: 别用游戏自己的驾驶键(W/A/S/D), 否则游戏也在动, 分不清是谁推的\n"
        "\n界面内快捷键:\n"
        "  F9 武装/解除写入    F5 暂停写入    小键盘 +/- 调速\n"
        "  1 存点   7 回点    0 切换写静态镜像    F10 退出\n");
}

static bool ParseArgs(int argc, char** argv) {
    g_keySetMask = 0;          // 记录命令行显式设置了哪些键(位 1..6)
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto nd = [&](double d) { return (i + 1 < argc) ? atof(argv[++i]) : d; };
        auto ni = [&](int d)    { return (i + 1 < argc) ? atoi(argv[++i]) : d; };
        if      (a == "--tol1")          g_opt.tol1 = nd(g_opt.tol1);
        else if (a == "--tol1-moving")   g_opt.tol1Moving = nd(g_opt.tol1Moving);
        else if (a == "--tol-delta")     g_opt.tolDelta = nd(g_opt.tolDelta);
        else if (a == "--move-need")     g_opt.moveNeed = nd(g_opt.moveNeed);
        else if (a == "--move-timeout")  g_opt.moveTimeout = nd(g_opt.moveTimeout);
        else if (a == "--all-regions")   g_opt.allRegions = true;
        else if (a == "--threads")       g_opt.threads = ni(0);
        else if (a == "--no-residency")  g_opt.noResidency = true;
        else if (a == "--ignore-last-check") g_opt.ignoreLastCheck = true;
        else if (a == "--pointer")       g_opt.pointerMode = true;
        else if (a == "--heuristic")     g_opt.heuristic = true;
        else if (a == "--drop-bottom")   g_opt.dropBottom = ni(g_opt.dropBottom);
        else if (a == "--take-last")     g_opt.takeLast = ni(g_opt.takeLast);
        else if (a == "--max-targets")   g_opt.maxTargets = ni(g_opt.maxTargets);
        else if (a == "--pick")          g_opt.pick = ni(-1);
        else if (a == "--write-all")     g_opt.writeAll = true;
        else if (a == "--write-static")  g_opt.writeStatic = true;
        else if (a == "--no-static")     g_opt.writeStatic = false;
        else if (a == "--arm")           g_opt.armAtStart = true;
        else if (a == "--max-coord")     g_opt.maxCoord = nd(g_opt.maxCoord);
        else if (a == "--height-axis") {
            if (i + 1 < argc) {
                std::string v = argv[++i];
                if (v == "x" || v == "X" || v == "0") g_opt.heightAxis = 0;
                else if (v == "y" || v == "Y" || v == "1") g_opt.heightAxis = 1;
                else g_opt.heightAxis = 2;
            }
        }
        else if (a == "--addr-prefix") {
            if (i + 1 < argc) g_opt.addrPrefix = argv[++i];
        }
        else if (a == "--addr") {
            if (i + 1 < argc) {
                std::string s = argv[++i];
                AddrSpec sp{};
                if (s.rfind("exe+", 0) == 0 || s.rfind("exe +", 0) == 0) {
                    sp.exeRelative = true;
                sp.value = strtoull(s.c_str() + 4, nullptr, 16);
                } else {
                    sp.exeRelative = false;
                sp.value = strtoull(s.c_str(), nullptr, 0);
                }
                g_opt.extraAddrs.push_back(sp);
            }
        }
        else if (a == "--no-wait")       g_opt.noWait = true;
        else if (a == "--wait-timeout")  g_opt.waitTimeout = nd(0);
        else if (a == "--zero-eps")      g_opt.zeroEps = nd(g_opt.zeroEps);
        else if (a == "--speed")         g_opt.speed = nd(g_opt.speed);
        else if (a == "--key-fwd" || a == "--key-back" || a == "--key-left" ||
                 a == "--key-right" || a == "--key-up" || a == "--key-down") {
            if (i + 1 < argc) {
                int vk = ParseKeyName(argv[++i]);
                if (!vk) printf("[!] %s 的键名看不懂: %s\n", a.c_str(), argv[i]);
                else if (a == "--key-fwd")   { g_opt.keyFwd   = vk; g_keySetMask |= 1;  }
                else if (a == "--key-back")  { g_opt.keyBack  = vk; g_keySetMask |= 2;  }
                else if (a == "--key-left")  { g_opt.keyLeft  = vk; g_keySetMask |= 4;  }
                else if (a == "--key-right") { g_opt.keyRight = vk; g_keySetMask |= 8;  }
                else if (a == "--key-up")    { g_opt.keyUp    = vk; g_keySetMask |= 16; }
                else                         { g_opt.keyDown  = vk; g_keySetMask |= 32; }
            }
        }
        else if (a == "--yaw-offset")    g_opt.yawOffsetDeg = nd(0);
        else if (a == "--yaw-bytes")     g_opt.yawOffBytes = ni(g_opt.yawOffBytes);
        else if (a == "--port")          g_opt.port = ni(g_opt.port);
        else if (a == "--invert-fwd")    g_opt.invertFwd = true;
        else if (a == "--invert-right")  g_opt.invertRight = true;
        else if (a == "--invert-up")     g_opt.invertUp = true;
        else if (a == "--list")          g_opt.listOnly = true;
        else if (a == "--selftest")      g_opt.selfTest = true;
        else if (a == "--gui-demo")      g_opt.guiDemo = true;
        else if (a == "--demo-secs")     g_opt.demoSecs = ni(0);
        else if (a == "-h" || a == "--help") { PrintHelp(); g_opt.showHelp = true; return false; }
        else printf("[!] 未知选项: %s\n", a.c_str());
    }
    return true;
}

static void RunSelfTest() {
    BYTE pkt[FH4_PACKET_LEN]{};
    float yaw = 1.5707f, x = 1234.5678f, y = -987.6543f, z = 56.7891f;
    std::memcpy(pkt + 56,  &yaw, 4);
    std::memcpy(pkt + 232, &x, 4);
    std::memcpy(pkt + 236, &y, 4);
    std::memcpy(pkt + 240, &z, 4);
    Telemetry t{};
    bool ok = ParsePacket(pkt, FH4_PACKET_LEN, t, 56);
    printf("[selftest] parse=%d yaw=%.4f x=%.4f y=%.4f z=%.4f\n",
           (int)ok, t.yaw, t.posX, t.posY, t.posZ);

    // 增量比对逻辑自检
    // v1 = 候选在第一遍时的值 (应当等于当时的镜像 p0), pN = 现在的镜像
    Vec3 p0{100,10,200}, pN{130,10,240};
    Vec3 v1 = p0;
    double dx = (double)pN.x - p0.x, dz = (double)pN.z - p0.z;
    Vec3 live{130, 10, 240}, fake{100, 10, 200};   // 活的跟着动 / 假的没动
    bool liveOk = std::fabs(((double)live.x - v1.x) - dx) <= 0.75 &&
                  std::fabs(((double)live.z - v1.z) - dz) <= 0.75;
    bool fakeOk = std::fabs(((double)fake.x - v1.x) - dx) <= 0.75 &&
                  std::fabs(((double)fake.z - v1.z) - dz) <= 0.75;
    printf("[selftest] 增量比对: 活的副本=%s, 假的=%s (期望 通过 / 拒绝)\n",
           liveOk ? "通过" : "拒绝", fakeOk ? "通过" : "拒绝");

    // 方向公式自检: 前进 = (X: sin yaw, Z: cos yaw)  (地面 = X-Z, 高度 = Y)
    //   —— 2026-10-14 实测确认: yaw=0 车头朝 +Z, 不是 +X
    auto fwdVec = [](double yawDeg, double out[3]) {
        double r = yawDeg * 3.14159265358979323846 / 180.0;
        int hx = g_opt.heightAxis;
        int ax = (hx == 0) ? 1 : 0;
        int ay = (hx == 2) ? 1 : 2;
        out[0] = out[1] = out[2] = 0.0;
        out[ax] = std::sin(r);
        out[ay] = std::cos(r);
    };
    double v0[3], v90[3];
    fwdVec(0.0, v0);
    fwdVec(90.0, v90);
    // 期望: yaw=0 -> Z=+1 ; yaw=90 -> X=+1  (Y 是高度, 水平只在 X-Z 上)
    bool dirOk = std::fabs(v0[0]) < 1e-9 && std::fabs(v0[1]) < 1e-9 && std::fabs(v0[2] - 1.0) < 1e-9 &&
                 std::fabs(v90[0] - 1.0) < 1e-9 && std::fabs(v90[1]) < 1e-9 && std::fabs(v90[2]) < 1e-9;
    printf("[selftest] 方向公式: yaw=0 -> X%+.3f Y%+.3f Z%+.3f ; yaw=90 -> X%+.3f Y%+.3f Z%+.3f  (%s)\n",
           v0[0], v0[1], v0[2], v90[0], v90[1], v90[2], dirOk ? "OK" : "错");
    printf("[selftest] %s\n",
           (ok && fabsf(t.yaw - 1.5707f) < 1e-3f && fabsf(t.posX - 1234.5678f) < 1e-2f &&
            liveOk && !fakeOk && dirOk) ? "PASS" : "FAIL");
}

// ============================== 主流程 ======================================

// ============================== ImGui 窗口 ==================================

static ID3D11Device*           g_pd3dDevice = nullptr;
static ID3D11DeviceContext*    g_pd3dDeviceContext = nullptr;
static IDXGISwapChain*         g_pSwapChain = nullptr;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;
static UINT                    g_ResizeWidth = 0, g_ResizeHeight = 0;

struct GuiState {
    HANDLE   h = nullptr;
    uint64_t A = 0;
    uint64_t base = 0;
    std::vector<uint64_t> targets;
    bool     verified = false;
    bool     lastCheckOk = true;
    bool     canArm = false;
    bool     armed = false;
    bool     paused = false;
    bool     writeStatic = false;
    bool     topmost = true;
    bool     transparentBg = false;  // 默认用 ImGui 默认深色面板; 勾选则只留控件、透出游戏
    bool     hasSaved = false;
    Vec3     saved{};
    Vec3     cur{};
    bool     valid = false;
    bool     hasPkt = false;
    float    speed = 25.0f;
    int      singleIdx = -1;      // -1 = 写全部
    double   dt = 0.005;
    DWORD    holdWriteUntil = 0;
    std::deque<std::string> log;
    bool     wantQuit = false;
    bool     demo = false;
    int      demoSecs = 0;
    bool     collapsed = false;      // 折叠: 只留顶部一条
    bool     forceCollapseSize = false;  // 刚折叠: 只调整一次高度, 之后允许手动缩放
    float    expandW = 540.0f;       // 展开时记住的尺寸(折叠/展开用)
    float    expandH = 751.0f;
    float    winW = 0.0f, winH = 0.0f;   // ImGui 内容尺寸, 用来把无边框窗口贴紧
};

static HWND g_guiHwnd = nullptr;   // 无边框窗口句柄(拖动/贴紧用)

static void GuiLog(GuiState& st, const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    st.log.push_back(buf);
    while (st.log.size() > 300) st.log.pop_front();
    printf("%s\n", buf);
    fflush(stdout);
}

static void CreateRenderTarget() {
    ID3D11Texture2D* pBackBuffer = nullptr;
    if (SUCCEEDED(g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer))) && pBackBuffer) {
        g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
        pBackBuffer->Release();
    }
}

static void CleanupRenderTarget() {
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}

static bool CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd{};
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    const D3D_FEATURE_LEVEL levels[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
    D3D_FEATURE_LEVEL got{};
    HRESULT res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
                                               levels, 2, D3D11_SDK_VERSION, &sd,
                                               &g_pSwapChain, &g_pd3dDevice, &got, &g_pd3dDeviceContext);
    if (res == DXGI_ERROR_UNSUPPORTED)
        res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0,
                                            levels, 2, D3D11_SDK_VERSION, &sd,
                                            &g_pSwapChain, &g_pd3dDevice, &got, &g_pd3dDeviceContext);
    if (res != S_OK) return false;
    CreateRenderTarget();
    return true;
}

static void CleanupDeviceD3D() {
    CleanupRenderTarget();
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

static LRESULT WINAPI GuiWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return true;
    switch (msg) {
        case WM_SIZE:
            if (wParam != SIZE_MINIMIZED) {
                g_ResizeWidth = (UINT)LOWORD(lParam);
                g_ResizeHeight = (UINT)HIWORD(lParam);
            }
            return 0;
        case WM_SYSCOMMAND:
            if ((wParam & 0xFFF0) == SC_KEYMENU) return 0;
            break;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

static void LoadChineseFont() {
    ImGuiIO& io = ImGui::GetIO();
    const char* fonts[] = { "C:\\Windows\\Fonts\\msyh.ttc",
                            "C:\\Windows\\Fonts\\msyh.ttf",
                            "C:\\Windows\\Fonts\\simhei.ttf" };
    for (const char* p : fonts) {
        if (GetFileAttributesA(p) == INVALID_FILE_ATTRIBUTES) continue;
        ImFontConfig cfg{};
        cfg.FontNo = 0;
        if (io.Fonts->AddFontFromFileTTF(p, 18.0f, &cfg, io.Fonts->GetGlyphRangesChineseFull()))
            return;
    }
}

// 按键状态(全局, 窗口失焦时也能用)
static bool KeyDown(int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; }

// 每帧: 读坐标 -> 按方向推进 -> 写目标
static void ApplyMovement(GuiState& st) {
    if (st.demo) {
        // 演示模式: 只动界面上的数字, 不碰任何进程
        st.valid = true;
        st.hasPkt = true;
        if (!st.armed || st.paused) return;
        double yawRad = YawForMove() * 3.14159265358979323846 / 180.0;
        double c = std::cos(yawRad), s = std::sin(yawRad);
        double v = (double)st.speed * st.dt;
        double fwd = 0, right = 0, up = 0;
        if (KeyDown(g_opt.keyFwd))   fwd   += 1;
        if (KeyDown(g_opt.keyBack))  fwd   -= 1;
        if (KeyDown(g_opt.keyRight)) right += 1;
        if (KeyDown(g_opt.keyLeft))  right -= 1;
        if (KeyDown(g_opt.keyUp))    up    += 1;
        if (KeyDown(g_opt.keyDown))  up    -= 1;
        st.cur.x += (float)((s * fwd + c * right) * v);
        st.cur.z += (float)((c * fwd - s * right) * v);
        st.cur.y += (float)(up * v);
        return;
    }
    Vec3 p{};
    if (st.h && ReadVec3(st.h, st.A, p)) {
        st.cur = p;
        st.valid = IsCoordValid(p, g_opt.zeroEps);
    } else {
        st.valid = false;
    }
    st.hasPkt = DataOutReady();

    if (!st.valid || !st.armed || st.paused || !st.h) return;
    if (!st.hasPkt) return;      // 没有 Data Out 就不允许移动

    double yawRad = YawForMove() * 3.14159265358979323846 / 180.0;
    double c = std::cos(yawRad);     // yaw=0 -> +Z
    double s = std::sin(yawRad);     // yaw=90 -> +X
    double v = (double)st.speed * st.dt;
    if (v > 50.0) v = 50.0;

    bool moving = false;
    double fwd  = 0.0, right = 0.0, up = 0.0;
    if (KeyDown(g_opt.keyFwd))   fwd   += 1.0;
    if (KeyDown(g_opt.keyBack))  fwd   -= 1.0;
    if (KeyDown(g_opt.keyRight)) right += 1.0;
    if (KeyDown(g_opt.keyLeft))  right -= 1.0;
    if (KeyDown(g_opt.keyUp))    up    += 1.0;
    if (KeyDown(g_opt.keyDown))  up    -= 1.0;

    if (fwd != 0.0 || right != 0.0) {
        // 前进 = (X: sin, Z: cos) ; 右移 = (X: cos, Z: -sin)  (当前游戏实测的轴)
        p.x += (float)((s * fwd + c * right) * v);
        p.z += (float)((c * fwd - s * right) * v);
        moving = true;
    }
    if (up != 0.0) {
        p.y += (float)(up * v);
        moving = true;
    }
    if (!moving) return;

    if (!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) ||
        std::fabs(p.x) > g_opt.maxCoord || std::fabs(p.y) > g_opt.maxCoord ||
        std::fabs(p.z) > g_opt.maxCoord) return;

    if (st.writeStatic) WriteVec3(st.h, st.A, p);
    if (st.singleIdx >= 0 && st.singleIdx < (int)st.targets.size())
        WriteVec3(st.h, st.targets[st.singleIdx], p);
    else
        for (uint64_t t : st.targets) WriteVec3(st.h, t, p);
}

static void DrawGui(GuiState& st) {
    // 背景: 透明模式 -> 窗口底不画(空白处透出游戏); 不透明模式 -> 近黑面板
    ImGuiStyle& style = ImGui::GetStyle();
    style.Colors[ImGuiCol_WindowBg].w = st.transparentBg ? 0.0f : 0.96f;
    style.Colors[ImGuiCol_ChildBg].w  = st.transparentBg ? 0.55f : 0.35f;
    style.WindowBorderSize = st.transparentBg ? 0.0f : 1.0f;

    // 无标题栏, 但可以自由拖动边缘/右下角缩放
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(st.expandW, st.expandH), ImGuiCond_FirstUseEver);
    // 折叠时允许很小(只留一条), 展开时才限制到 360x160
    ImGui::SetNextWindowSizeConstraints(
        st.collapsed ? ImVec2(200.0f, 40.0f) : ImVec2(360.0f, 160.0f),
        ImVec2(FLT_MAX, FLT_MAX));
    ImGui::Begin("##fh4menu", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);

    // ---- 顶部把手: 折叠 / 拖动移动窗口 / 关闭 ----
    auto toggleCollapse = [&]() {
        ImVec2 sz = ImGui::GetWindowSize();
        if (!st.collapsed) {
            st.expandW = sz.x;
            st.expandH = sz.y;
            st.collapsed = true;
            st.forceCollapseSize = true;
            ImGui::SetWindowSize(ImVec2(sz.x, 68.0f));
        } else {
            st.collapsed = false;
            ImGui::SetWindowSize(ImVec2(st.expandW, st.expandH));
        }
    };
    ImGui::TextUnformatted("FH4 控制台");
    ImGui::SameLine();
    if (ImGui::SmallButton(st.collapsed ? "展开" : "折叠")) toggleCollapse();
    ImGui::SameLine();
    {
        float avail = ImGui::GetContentRegionAvail().x;
        ImGui::InvisibleButton("##drag", ImVec2(avail > 40.0f ? avail - 34.0f : 10.0f, 18.0f));
        if (ImGui::IsItemHovered()) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            toggleCollapse();                  // 双击空白处也能折叠/展开
        if (ImGui::IsItemActive() && g_guiHwnd) {
            ImVec2 d = ImGui::GetIO().MouseDelta;
            if (d.x != 0.0f || d.y != 0.0f) {
                RECT rc{};
                GetWindowRect(g_guiHwnd, &rc);
                SetWindowPos(g_guiHwnd, st.topmost ? HWND_TOPMOST : HWND_NOTOPMOST,
                             rc.left + (int)d.x, rc.top + (int)d.y, 0, 0,
                             SWP_NOSIZE | SWP_NOACTIVATE);
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("X")) st.wantQuit = true;
    }
    ImGui::Separator();

    // ---- 折叠状态: 只留一行状态, 面板缩成一小条 ----
    if (st.collapsed) {
        // 折叠的第一帧还受"展开最小 160"限制, 第二帧补一次; 之后不再强制, 允许你手动拖
        if (st.forceCollapseSize) {
            ImGui::SetWindowSize(ImVec2(ImGui::GetWindowSize().x, 68.0f));
            st.forceCollapseSize = false;
        }
        if (st.valid && st.hasPkt)
            ImGui::Text("X %.1f   Y %.1f   Z %.1f    偏航 %.1f°   %s",
                        st.cur.x, st.cur.y, st.cur.z, YawForMove(), st.armed ? "已武装" : "只读");
        else if (st.valid)
            ImGui::Text("X %.1f   Y %.1f   Z %.1f    无 Data Out   %s",
                        st.cur.x, st.cur.y, st.cur.z, st.armed ? "已武装" : "只读");
        else
            ImGui::Text("坐标无效    %s", st.armed ? "已武装" : "只读");
        st.winW = ImGui::GetWindowSize().x;
        st.winH = ImGui::GetWindowSize().y;
        ImGui::End();
        return;
    }

    // ---- 状态 ----
    if (st.valid)
        ImGui::TextColored(ImVec4(0.5f, 1, 0.5f, 1), "X %.2f    Y %.2f    Z %.2f",
                           st.cur.x, st.cur.y, st.cur.z);
    else
        ImGui::TextColored(ImVec4(1, 0.5f, 0.5f, 1), "坐标无效 (读不到 / 游戏没进驾驶状态)");
    if (st.hasPkt)
        ImGui::Text("偏航 %.1f°    DataOut: 实时", YawForMove());
    else
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1),
                           "DataOut: 无数据 —— 移动已禁用 (设置 -> 抬头显示与游戏玩法 -> 数据输出, 127.0.0.1:%d)",
                           g_opt.port);
    ImGui::Text("写入: %s   目标 %zu 个%s", st.armed ? "已武装" : "只读",
                st.targets.size(), st.paused ? "   [暂停]" : "");
    if (!st.canArm)
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "未通过验证, 不能武装 (看控制台提示)");

    ImGui::Separator();

    // ---- 目标选择 ----
    bool single = (st.singleIdx >= 0);
    if (ImGui::Checkbox("只写单个目标", &single)) st.singleIdx = single ? 0 : -1;
    if (st.singleIdx >= 0) {
        char label[96];
        uint64_t a = (st.singleIdx < (int)st.targets.size()) ? st.targets[st.singleIdx] : 0;
        std::snprintf(label, sizeof(label), "#%d  0x%llX", st.singleIdx, (unsigned long long)a);
        if (ImGui::BeginCombo("目标地址", label)) {
            for (int i = 0; i < (int)st.targets.size(); i++) {
                char item[96];
                std::snprintf(item, sizeof(item), "#%d  0x%016llX", i,
                              (unsigned long long)st.targets[i]);
                if (ImGui::Selectable(item, st.singleIdx == i)) st.singleIdx = i;
            }
            ImGui::EndCombo();
        }
    }
    ImGui::SliderFloat("速度 (m/s)", &st.speed, 20.0f, 500.0f, "%.0f");
    if (st.speed < 20.0f)  st.speed = 20.0f;
    if (st.speed > 500.0f) st.speed = 500.0f;

    // ---- 移动: 只用键盘(点窗口会让游戏暂停, 按钮没意义) ----
    ImGui::SeparatorText("移动 (键盘)");
    ImGui::Text("%s 前进   %s 后退   %s 左移   %s 右移   %s 上升   %s 下降",
                KeyName(g_opt.keyFwd).c_str(),  KeyName(g_opt.keyBack).c_str(),
                KeyName(g_opt.keyLeft).c_str(), KeyName(g_opt.keyRight).c_str(),
                KeyName(g_opt.keyUp).c_str(),   KeyName(g_opt.keyDown).c_str());
    ImGui::TextDisabled("按住按键即可; 键位在 fh4_keys.ini 里改");
    if (!st.hasPkt)
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "无 Data Out: 不开数据输出就不能移动");
    if (!st.armed)
        ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "先勾选\"武装写入\"才能移动");

    // ---- 开关 ----
    ImGui::SeparatorText("开关");
    ImGui::BeginDisabled(!st.canArm);
    ImGui::Checkbox("武装写入 (F9)", &st.armed);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::Checkbox("暂停 (F5)", &st.paused);
    ImGui::Checkbox("同时写静态镜像 (0)", &st.writeStatic);
    ImGui::Checkbox("透明背景 (只留控件)", &st.transparentBg);
    if (ImGui::Checkbox("窗口置顶", &st.topmost)) {
        HWND w = GetActiveWindow();
        SetWindowPos(w, st.topmost ? HWND_TOPMOST : HWND_NOTOPMOST, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE);
    }

    // ---- 存点/回点/退出 ----
    ImGui::Separator();
    if (ImGui::Button("存点 (1)", ImVec2(110, 30))) {
        if (st.valid) { st.saved = st.cur; st.hasSaved = true; GuiLog(st, "存点 %.2f / %.2f / %.2f", st.saved.x, st.saved.y, st.saved.z); }
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!st.hasSaved || !st.h);
    if (ImGui::Button("回点 (7)", ImVec2(110, 30))) {
        if (st.armed) {
            st.holdWriteUntil = GetTickCount() + 800;
            if (st.writeStatic) WriteVec3(st.h, st.A, st.saved);
            if (st.singleIdx >= 0 && st.singleIdx < (int)st.targets.size())
                WriteVec3(st.h, st.targets[st.singleIdx], st.saved);
            else
                for (uint64_t t : st.targets) WriteVec3(st.h, t, st.saved);
            GuiLog(st, "回到存点");
        } else GuiLog(st, "回点需要先武装写入");
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("退出 (F10)", ImVec2(110, 30))) st.wantQuit = true;

    // ---- 日志 ----
    ImGui::SeparatorText("日志");
    ImGui::BeginChild("log", ImVec2(0, 150), ImGuiChildFlags_Borders);
    for (const auto& s : st.log) ImGui::TextUnformatted(s.c_str());
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();

    st.winW = ImGui::GetWindowSize().x;
    st.winH = ImGui::GetWindowSize().y;
    ImGui::End();
}

static int RunImGuiWindow(GuiState& st) {
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.style = CS_CLASSDC;
    wc.lpfnWndProc = GuiWndProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorA(nullptr, IDC_ARROW);   // IDC_ARROW 是 ANSI 资源宏
    wc.lpszClassName = L"FH4CoordGui";
    RegisterClassExW(&wc);

    // WS_POPUP: 没有标题栏和边框; WS_EX_TOOLWINDOW: 不出现在任务栏/Alt-Tab
    // 视口比菜单略大; 多出来的部分是透明+点击穿透的, 看不见也不会挡住游戏
    HWND hwnd = CreateWindowExW(WS_EX_TOOLWINDOW, wc.lpszClassName, L"FH4 菜单", WS_POPUP,
                                80, 80, 540, 780, nullptr, nullptr, wc.hInstance, nullptr);
    g_guiHwnd = hwnd;
    if (!hwnd || !CreateDeviceD3D(hwnd)) {
        printf("[X] 创建窗口/D3D11 失败\n");
        if (g_pd3dDevice) CleanupDeviceD3D();
        if (hwnd) DestroyWindow(hwnd);
        UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    // 色键透明: 清屏色(RGB 20,20,24)会被系统视为透明, 于是"窗口背景"那层不挡游戏,
    // 只留下 ImGui 画的控件。取消"透明背景"勾选时控件面板会盖住它, 看起来就是普通窗口。
    SetWindowLongW(hwnd, GWL_EXSTYLE, GetWindowLongW(hwnd, GWL_EXSTYLE) | WS_EX_LAYERED);
    SetLayeredWindowAttributes(hwnd, RGB(20, 20, 24), 0, LWA_COLORKEY);

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);
    if (st.topmost)
        SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    LoadChineseFont();
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    GuiLog(st, "GUI 就绪. 武装后按住键盘方向键即可移动.");

    bool done = false;
    DWORD demoStart = GetTickCount();
    LARGE_INTEGER freq, last;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&last);

    while (!done) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
            if (msg.message == WM_QUIT) done = true;
        }
        if (done) break;

        if (g_ResizeWidth != 0 && g_ResizeHeight != 0) {
            CleanupRenderTarget();
            g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight,
                                        DXGI_FORMAT_UNKNOWN, 0);
            g_ResizeWidth = g_ResizeHeight = 0;
            CreateRenderTarget();
        }

        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        st.dt = double(now.QuadPart - last.QuadPart) / double(freq.QuadPart);
        last = now;
        if (st.dt > 0.1) st.dt = 0.1;

        if (KeyDown(VK_F10) && !io.WantTextInput) st.wantQuit = true;
        if (st.wantQuit) break;
        if (st.demo && st.demoSecs > 0 && GetTickCount() - demoStart > (DWORD)(st.demoSecs * 1000))
            break;

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        DrawGui(st);
        ApplyMovement(st);
        ImGui::Render();

        // 把无边框 Win32 窗口同步成 ImGui 窗口的大小(用户拖右下角/边缘缩放时跟着变)
        if (g_guiHwnd && st.winW >= 200.0f && st.winH >= 40.0f) {
            RECT rc{};
            GetWindowRect(g_guiHwnd, &rc);
            int w = (int)(st.winW + 0.5f), h = (int)(st.winH + 0.5f);
            if ((rc.right - rc.left) != w || (rc.bottom - rc.top) != h)
                SetWindowPos(g_guiHwnd, nullptr, 0, 0, w, h,
                             SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
        }

        // 清屏色必须和上面的色键完全一致
        const float clear[4] = { 20.0f / 255.0f, 20.0f / 255.0f, 24.0f / 255.0f, 1.0f };
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        g_pSwapChain->Present(1, 0);
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    CleanupDeviceD3D();
    DestroyWindow(hwnd);
    g_guiHwnd = nullptr;
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 0;
}

int main(int argc, char** argv) {
    InitConsole();
    if (!ParseArgs(argc, argv)) {
        fflush(stdout);
        if (!g_opt.showHelp) system("pause");   // --help 不留 pause, 方便管道/脚本
        return 0;
    }
    LoadKeys(g_keySetMask);      // 按键: 配置文件 + 命令行覆盖
    if (g_opt.selfTest) { RunSelfTest(); system("pause"); return 0; }

    // 只看界面(不连游戏, 不写内存) —— 用来预览 UI / 验证 ImGui 能起来
    if (g_opt.guiDemo) {
        GuiState gs;
        gs.demo     = true;
        gs.demoSecs = g_opt.demoSecs;
        gs.canArm   = true;
        gs.armed    = true;
        gs.valid    = true;
        gs.cur      = Vec3{ -250.400f, 230.405f, -5598.420f };
        gs.speed    = (float)std::clamp(g_opt.speed, 20.0, 500.0);
        gs.targets  = { 0x000001C5A4332530ULL, 0x00007FF63C15F980ULL, 0x000001C59C980800ULL };
        gs.base     = 0x00007FF634180000ULL;
        return RunImGuiWindow(gs);
    }

    if (!IsElevated()) {
        printf("[!] 不是管理员, 尝试提权重启...\n");
        if (RelaunchAsAdmin()) { printf("[+] 已弹出 UAC, 本进程退出。\n"); Sleep(1500); return 0; }
        printf("[!] 提权被拒绝, 继续 (可能打不开进程)\n");
    }
    EnableDebugPrivilege();

    DWORD pid = FindProcessId(PROC_NAME);
    if (!pid) { printf("[X] 没找到 %ls, 请先启动游戏\n", PROC_NAME); system("pause"); return 1; }

    HANDLE h = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ |
                           PROCESS_VM_WRITE | PROCESS_VM_OPERATION, FALSE, pid);
    if (!h) { printf("[X] OpenProcess 失败, 错误码 %lu\n", GetLastError()); system("pause"); return 1; }

    uint64_t base = GetModuleBase(pid, PROC_NAME);
    if (!base) { printf("[X] 取模块基址失败\n"); system("pause"); return 1; }
    uint64_t A = base + COORD_RVA;
    {
        HMODULE hmod = reinterpret_cast<HMODULE>(base);
        MODULEINFO mi{};
        if (GetModuleInformation(h, hmod, &mi, sizeof(mi))) g_moduleSize = mi.SizeOfImage;
        g_moduleBase = base;
    }
    printf("[+] PID=%lu  基址=0x%016llX  镜像=0x%016llX (exe+%llX)\n",
           pid, (unsigned long long)base, (unsigned long long)A,
           (unsigned long long)COORD_RVA);

    CreateThread(nullptr, 0, UdpThread, nullptr, 0, nullptr);

    // ---------- 闸门: 坐标必须有效 ----------
    Vec3 cur{};
    if (!ReadVec3(h, A, cur)) {
        printf("[X] 读不到镜像坐标, 错误码 %lu\n", GetLastError());
        system("pause"); return 1;
    }
    printf("[+] 镜像坐标 X=%.3f Y=%.3f Z=%.3f\n", cur.x, cur.y, cur.z);

    if (!IsCoordValid(cur, g_opt.zeroEps)) {
        if (g_opt.noWait) {
            printf("[!] --no-wait: 坐标是 0 也继续 (结果大概率不准)\n");
        } else {
            printf("[*] 镜像坐标是 0 —— 游戏还没进入驾驶状态(载入中/大厅/车库)。\n");
            printf("    按你的要求: 这个阶段不搜索, 先等坐标变成有效值。\n");
            DWORD t0 = GetTickCount();
            for (;;) {
                Sleep(250);
                if (!ReadVec3(h, A, cur)) continue;
                if (IsCoordValid(cur, g_opt.zeroEps)) {
                    printf("\n[+] 坐标有效: X=%.3f Y=%.3f Z=%.3f\n", cur.x, cur.y, cur.z);
                    break;
                }
                DWORD el = GetTickCount() - t0;
                if (g_opt.waitTimeout > 0.0 && el > (DWORD)(g_opt.waitTimeout * 1000.0)) {
                    printf("\n[X] 等超时, 坐标仍是 0, 不搜索。进驾驶状态后再运行。\n");
                    system("pause"); return 1;
                }
                printf("\r    等待中... %lu 秒   X=%.3f Y=%.3f Z=%.3f     ",
                       (unsigned long)(el / 1000), cur.x, cur.y, cur.z);
                fflush(stdout);
            }
        }
    }

    // ---------- 检查 Data Out: 没有数据就不让移动 ----------
    // (你原来那段放在 UDP 线程里, getchar() 只停住那个线程, 主线程照跑;
    //  这里放到主线程, 用"seq 有没有变化"判断是否真的在发数据)
    for (int tries = 0;; ++tries) {
        if (WaitForDataOut(3000)) {
            printf("[+] Data Out 正常: 已收到数据包\n");
            break;
        }
        printf("\n[!] 3 秒内没有收到 Data Out 包 —— 现在不能移动(没有 yaw)。\n");
        printf("    1) 游戏里打开: 设置 -> 抬头显示与游戏玩法 -> 数据输出 = 开\n");
        printf("    2) IP 地址填 127.0.0.1, 端口填 %d\n", g_opt.port);
        printf("    3) 回到驾驶状态(自由漫游里坐在车上)再试\n");
        printf("    按任意键重试, 按 Esc 退出程序\n");
        int c = _getch();
        if (c == 27 || c == -1) {
            printf("\n[X] 已退出: 没有 Data Out 就不能移动。\n");
            CloseHandle(h);
            system("pause");
            return 1;
        }
    }

    // ---------- 搜索前先让车停稳 ----------
    // 车在动的时候坐标每帧都在变, 第一遍值搜索会搜得很不准, 所以先等它停住。
    double tol1 = g_opt.tol1;
    {
        printf("\n[*] 搜索前请先停车: 按 ESC (或直接刹车) 让车完全停住。\n");
        printf("    车动的时候坐标一直在变, 搜出来的东西会不准。\n");
        DWORD t0 = GetTickCount();
        bool stopped = false;
        for (;;) {
            Vec3 a{}, b{};
            if (!ReadVec3(h, A, a)) { Sleep(200); continue; }
            Sleep(250);
            if (!ReadVec3(h, A, b)) continue;
            double d = std::max({ std::fabs((double)b.x - a.x),
                                  std::fabs((double)b.y - a.y),
                                  std::fabs((double)b.z - a.z) });
            if (d <= 0.05) { stopped = true; break; }
            if (GetTickCount() - t0 > 60000) break;      // 最多等 60 秒
            printf("\r    车还在动 (0.25 秒位移 %.2f 米), 请按 ESC / 刹车停下...      ", d);
            fflush(stdout);
        }
        printf("\n");
        if (stopped) {
            printf("[+] 车已停稳, 第一遍用紧容差 %.2f 米 (这次最准)\n", tol1);
        } else {
            printf("[!] 等了 60 秒车还在动, 第一遍容差放宽到 %.2f 米 (结果可能不准)\n",
                   g_opt.tol1Moving);
            tol1 = g_opt.tol1Moving;
        }
    }

    Vec3 p0{};
    ReadVec3(h, A, p0);
    Vec3 S0 = p0;                       // 第一遍的基准值

    std::vector<Region> allRegions = EnumRegions(h);
    std::vector<uint64_t> hits;

    // ---------- 第一遍 (全量搜索) ----------
    // 注: 原来这里有个"学习缓存"(把上次的区域+偏移存起来, 下次直接命中)。
    // 实测命中之后写进去也推不动车 —— 缓存里存的那些地址只是"值相同的副本",
    // 不是真正驱动载具的那份, 所以整个缓存机制已删除, 每次都老老实实全量搜。
    printf("\n[*] 第一遍搜索: 全内存找 XYZ=(%.2f, %.2f, %.2f) ±%.2f 米\n",
           S0.x, S0.y, S0.z, tol1);
    DWORD tick0 = GetTickCount();
    uint64_t scannedMB = 0;
    hits = ScanTriples(h, S0, tol1, &scannedMB, !g_opt.noResidency);
    DWORD tick1 = GetTickCount();
    printf("[+] 扫了 %llu MB, 用时 %lu ms, 命中 %zu 个地址\n",
           (unsigned long long)scannedMB, (unsigned long)(tick1 - tick0), hits.size());

    // 护身符: 命中数量级不对时(容差过大 / 大量 NaN 垃圾), 别拿几千万个地址去逐个复核
    if (hits.size() > 500000) {
        printf("[!] 命中异常多 (%zu), 只取前 500000 个继续 (建议 --tol1 收紧)\n", hits.size());
        hits.resize(500000);
    }

    {
        Vec3 pA{};
        ReadVec3(h, A, pA);
        hits = RecheckTriples(h, hits, pA, tol1 + 0.5);    // 立刻收紧一次
        printf("[+] 立刻复查后剩 %zu 个 (这步把绝大部分假地址干掉了)\n", hits.size());
    }

    if (hits.empty()) {
        printf("[!] 一个都没命中。可以试: --tol1 1.0 放宽, 或 --all-regions\n");
    }

    // ---------- 等车真的动起来, 再做第二遍 ----------
    // 车没动 = 增量比对无法区分真假 (判据退化成"值没变就算通过"), 这时候的结果
    // 绝对不能拿去写内存 —— 上次把游戏写崩就是这么来的。
    bool verifiedByMovement = false;         // 只有"车真的开动过"才算验证
    double movedDist = 0.0;
    if (!hits.empty()) {
        printf("\n[*] 第二遍准备: 请开动车, 移动超过 %.1f 米我才会筛。\n", g_opt.moveNeed);
        printf("    为什么必须动: 车静止时\"增量比对\"退化成\"值没变就算通过\",\n");
        printf("    静态假地址会全部留下 —— 拿去写就是上次把游戏写崩的原因。\n");
        Vec3 start{};
        ReadVec3(h, A, start);
        DWORD t0 = GetTickCount();
        Vec3 now = start;
        bool timedOut = false;
        for (;;) {
            Sleep(100);
            if (!ReadVec3(h, A, now)) continue;
            if (!IsCoordValid(now, g_opt.zeroEps)) {
                printf("\r    坐标变成 0 了(可能进了菜单), 继续等...            ");
                fflush(stdout);
                continue;
            }
            double d = GroundDist(now, start);
            movedDist = d;
            if (d >= g_opt.moveNeed) break;
            if (g_opt.moveTimeout > 0.0 &&
                (GetTickCount() - t0) > (DWORD)(g_opt.moveTimeout * 1000.0)) {
                timedOut = true;
                break;
            }
            printf("\r    已移动 %.1f / %.1f 米 ...            ", d, g_opt.moveNeed);
            fflush(stdout);
        }
        printf("\n");

        if (timedOut || movedDist < g_opt.moveNeed) {
            printf("\n[!!] 只观察到 %.1f 米位移 —— 第二遍没能真正筛。\n", movedDist);
            printf("[!!] 这批候选里混着静态假地址, 拿它们写内存会把游戏写崩。\n");
            printf("[!!] 所以: 写入被强制禁用, 这次只当只读结果看。\n");
            printf("     正确做法: 进自由驾驶, 开动十几米(或跑一小段), 再重新运行本程序。\n");
            verifiedByMovement = false;
        } else {
            size_t before = hits.size();
            hits = RecheckByDelta(h, hits, S0, p0, now, g_opt.tolDelta);
            printf("[+] 第二遍(增量比对, 位移 %.1f 米)结束: %zu -> %zu 个存活\n",
                   movedDist, before, hits.size());
            verifiedByMovement = true;
            if (hits.empty()) {
                printf("[!] 全部被筛掉 = 一个都不跟着动, 说明第一遍的命中都是假的。\n");
            }

            // ---------- 第三遍: 再动一段距离, 用同一基准再筛一次 ----------
            // 目的: 干掉"偶然同值"的地址(它们只会对上一次, 多动一段就露馅)。
            if (!hits.empty()) {
                printf("[*] 第三遍复筛: 请再开动大约 1 米 (最多等 6 秒)...\n");
                Vec3 s2 = now;
                DWORD t2 = GetTickCount();
                for (;;) {
                    Sleep(100);
                    if (!ReadVec3(h, A, now)) continue;
                    if (!IsCoordValid(now, g_opt.zeroEps)) continue;
                    double d2 = GroundDist(now, s2);
                    if (d2 >= 1.0) break;
                    if (GetTickCount() - t2 > 6000) break;
                }
                size_t b2 = hits.size();
                hits = RecheckByDelta(h, hits, S0, p0, now, g_opt.tolDelta);
                printf("[+] 第三遍结束: %zu -> %zu 个存活\n", b2, hits.size());
            }
        }
    }

    // ---------- 可选: 指针模式 ----------
    if (g_opt.pointerMode) {
        printf("\n[*] 指针模式: 找存放\"镜像地址\"的 8 字节槽位...\n");
        uint64_t lo = (A > 0x400) ? A - 0x400 : 0;
        auto ptrs = ScanPointers(h, lo, A + 0x40);
        printf("[+] 找到 %zu 个槽位\n", ptrs.size());
        char b1[80];
        for (size_t i = 0; i < std::min<size_t>(ptrs.size(), 20); i++) {
            FmtAddr(b1, sizeof(b1), ptrs[i].slot, base);
            printf("    [%2zu] %s -> 0x%llX\n", i, b1, (unsigned long long)ptrs[i].target);
        }
    }

    // ---------- 列出结果 ----------
    std::sort(hits.begin(), hits.end());

    // 按地址前缀过滤 (你知道真身是以 0x7 开头的, 用 --addr-prefix 0x7 就能只留那一批)
    if (!g_opt.addrPrefix.empty()) {
        std::string pfx = g_opt.addrPrefix;
        if (pfx.rfind("0x", 0) == 0 || pfx.rfind("0X", 0) == 0) pfx = pfx.substr(2);
        for (auto& c : pfx) c = (char)std::toupper((unsigned char)c);
        size_t before = hits.size();
        std::vector<uint64_t> keep;
        for (uint64_t a : hits) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%llX", (unsigned long long)a);
            if (std::string(buf).rfind(pfx, 0) == 0) keep.push_back(a);
        }
        hits.swap(keep);
        printf("\n[*] --addr-prefix %s: %zu -> %zu 个\n", g_opt.addrPrefix.c_str(),
               before, hits.size());
    }

    // 按地址高位分组, 让人一眼看出候分布在哪些段
    if (hits.size() > 1) {
        printf("\n[*] 候选中地址分布 (前 4 位十六进制 : 个数):\n    ");
        std::string cur;
        int n = 0;
        for (uint64_t a : hits) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%llX", (unsigned long long)a);
            std::string band = std::string(buf).substr(0, std::min<size_t>(4, std::strlen(buf)));
            if (band != cur) {
                if (!cur.empty()) printf("%s=%d  ", cur.c_str(), n);
                cur = band; n = 0;
            }
            n++;
        }
        if (!cur.empty()) printf("%s=%d", cur.c_str(), n);
        printf("\n");
    }

    printf("\n===== 候选地址 (按地址升序, 共 %zu 个) =====\n", hits.size());
    char b1[80];
    for (size_t i = 0; i < hits.size(); i++) {
        FmtAddr(b1, sizeof(b1), hits[i], base);
        printf("  [%2zu] %s\n", i, b1);
    }
    if (g_opt.heuristic && !hits.empty()) {
        printf("\n[*] 你的启发: 从末尾丢 %d 个, 再取倒数 %d 个\n",
               g_opt.dropBottom, g_opt.takeLast);
    }

    // ---------- 选择要写的地址 ----------
    // 默认: 存活的全写, 但最多 maxTargets 个(你说的"顶死最后15个")。
    // 存活名单已经过了位移验证, 所以不需要再靠位置猜。要精确点名用 --pick N。
    std::vector<uint64_t> targets;
    if (verifiedByMovement && !hits.empty()) {
        if (g_opt.pick >= 0) {
            if (g_opt.pick < (int)hits.size()) {
                targets.push_back(hits[g_opt.pick]);
                printf("\n[*] --pick %d: 只写列表里第 %d 个\n", g_opt.pick, g_opt.pick);
            } else {
                printf("\n[!] --pick %d 超范围 (列表只有 %zu 个)\n",
                       g_opt.pick, hits.size());
            }
        } else if (g_opt.heuristic) {
            std::vector<uint64_t> tmp = hits;
            int drop = g_opt.dropBottom > 0 ? g_opt.dropBottom : 0;
            if (drop > 0 && (int)tmp.size() > drop) tmp.resize(tmp.size() - drop);
            int take = g_opt.takeLast > 0 ? g_opt.takeLast : (int)tmp.size();
            if (take > (int)tmp.size()) take = (int)tmp.size();
            for (int i = (int)tmp.size() - take; i < (int)tmp.size(); i++)
                if (i >= 0) targets.push_back(tmp[i]);
            printf("\n[*] --heuristic: 丢末尾 %d 个 + 取倒数 %d 个 = %zu 个\n",
                   drop, take, targets.size());
        } else {
            // 不再假设真身在模块内 —— 地址在堆、在映射视图、在模块里都可能。
            // 存活名单已经过位移验证, 直接全写, 只按上限截断。
            targets = hits;
            if ((int)targets.size() > g_opt.maxTargets) {
                printf("\n[*] 存活 %zu 个 > 上限 %d, 只取地址序最后 %d 个\n"
                       "    (要指定某一个用 --pick N; 要按前缀筛用 --addr-prefix 0x7)\n",
                       targets.size(), g_opt.maxTargets, g_opt.maxTargets);
                targets.erase(targets.begin(), targets.end() - g_opt.maxTargets);
            }
        }
    } else if (!hits.empty()) {
        printf("\n[!!] 没有通过位移验证 -> 一个都不写 (只读看结果)。\n");
    }

    // 命令行点名的地址 (比如你在 CE 里已经验证过能推动车的那一个)
    for (const AddrSpec& sp : g_opt.extraAddrs) {
        uint64_t a = sp.exeRelative ? (base + sp.value) : sp.value;
        targets.push_back(a);
    }
    if (g_opt.extraAddrs.size() > 0) {
        printf("[*] 你点名了 %zu 个地址:\n", g_opt.extraAddrs.size());
        for (const AddrSpec& sp : g_opt.extraAddrs) {
            uint64_t a = sp.exeRelative ? (base + sp.value) : sp.value;
            FmtAddr(b1, sizeof(b1), a, base);
            printf("      %s\n", b1);
        }
    }
    if ((int)targets.size() > g_opt.maxTargets) {
        printf("[*] 写入目标 %zu 个超过上限 %d, 只保留最后 %d 个\n",
               targets.size(), g_opt.maxTargets, g_opt.maxTargets);
        targets.erase(targets.begin(), targets.end() - g_opt.maxTargets);
    }
    printf("\n[*] 实际写入 %zu 个地址%s\n", targets.size(),
           g_opt.writeStatic ? " + 静态镜像" : "");
    if (targets.size() <= 16) {
        for (size_t i = 0; i < targets.size(); i++) {
            FmtAddr(b1, sizeof(b1), targets[i], base);
            printf("      写: %s\n", b1);
        }
    }
    if (targets.empty()) {
        printf("[!] 没有可写地址。加 --tol1 1.0 或 --all-regions 重试。\n");
    }

    // ---------- 末尾校验: 最后一个必须是模块内的静态地址 ----------
    // 经验: 真身那种"静态的"地址在 ForzaHorizon4.exe 映像里, 打印出来是 exe+偏移。
    // 排完序后它在名单最末尾。如果最后一条不是这种地址, 说明这次搜索是错的
    // (最常见原因: 搜的时候车没停住, 值一直在变)。
    bool lastCheckOk = true;
    if (!targets.empty()) {
        uint64_t lastA = targets.back();
        lastCheckOk = IsInModule(lastA);
        char bb[80];
        FmtAddr(bb, sizeof(bb), lastA, base);
        printf("\n[*] 末尾校验: 最后一个地址 %s\n", bb);
        if (lastCheckOk) {
            printf("    -> 在 ForzaHorizon4.exe 映像内 (exe+偏移), 符合经验: 通过\n");
        } else {
            printf("    -> 不在模块映像里! 按你的经验这说明这次搜索是错的。\n"
                   "       多半是搜的时候车在动: 请按 ESC / 刹车把车彻底停住, 再跑一遍。\n"
                   "       (确实要用这批地址: 加 --ignore-last-check)\n");
        }
    }

    if (g_opt.listOnly) { CloseHandle(h); system("pause"); return 0; }

    printf("\n移动按键: %s 前进  %s 后退  %s 左移  %s 右移  %s 上升  %s 下降   (改 fh4_keys.ini)\n",
           KeyName(g_opt.keyFwd).c_str(),  KeyName(g_opt.keyBack).c_str(),
           KeyName(g_opt.keyLeft).c_str(), KeyName(g_opt.keyRight).c_str(),
           KeyName(g_opt.keyUp).c_str(),   KeyName(g_opt.keyDown).c_str());
    printf("搜索/验证完成后会自动打开图形界面, 所有操作都在窗口里进行\n");
    printf("(窗口里同样支持键盘: 方向键位见 fh4_keys.ini, F9 武装, F10 退出)\n");
    if (targets.empty())
        printf("[!] 当前没有可写目标 —— 程序处于纯只读监视状态。\n");

    // ---------- 进入 ImGui 图形界面 ----------
    // 搜索/验证都在上面用控制台完成了(需要停车、开车配合), 这里只做操作界面。
    GuiState gs;
    gs.h      = h;
    gs.A      = A;
    gs.base   = base;
    gs.targets = targets;
    gs.verified = verifiedByMovement;
    gs.lastCheckOk = lastCheckOk;
    gs.canArm = !targets.empty()
             && (verifiedByMovement || !g_opt.extraAddrs.empty())
             && (lastCheckOk || g_opt.ignoreLastCheck);
    gs.armed      = g_opt.armAtStart && gs.canArm;
    gs.paused     = false;
    gs.writeStatic = g_opt.writeStatic;
    gs.speed      = (float)std::clamp(g_opt.speed, 20.0, 500.0);
    gs.singleIdx  = -1;
    gs.hasSaved   = false;

    int rc = RunImGuiWindow(gs);

    CloseHandle(h);
    return rc;
}
