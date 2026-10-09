//=============================================================================
// sas_raid_tlm.h — SAS RAID 层事务级模型（卷表/条带/校验/重建/日志）  [2026-10-09 建]
//-----------------------------------------------------------------------------
// 规格出处：`架构文档` §4.13（RAID 引擎）+ `规格文档`（地址翻译五式/EI/TB ✓）
//   与 `内部内部开发计划` §0（**驱动兼容定案：hisi_sas 零改动 ✓**）
//
// ★ 分工（内部开发计划 §0 ✓）：
//   · 元命令（INQUIRY/RCAP/MODE SENSE/TUR/REQ SENSE/REPORT LUNS）⇒ **本层本地应答**
//     （内容与 `sas_hdd/tlm` 的字节级已验证数据一致 ✓ —— 由上层调用方取用 ✓）
//   · 数据命令（READ/WRITE 6/10/16）⇒ 本层**条带翻译 → 成员盘 I/O** ✓
//   · 成员盘对 guest **不可见** ✓（由本层经成员后端驱动 ✓ —— 同真实 RAID card firmware ✓）
//
// ★ 成员后端（**可绑接口** ✓）：单元台绑内存数组 ✓；联合台绑 `SasHddTlm`（经 link ✓）
//   ⇒ 本模型**不直接依赖** HDD 模型 ✓（松耦合 ✓，便于两类台各自例化 ✓）
//
// ★ 观测/注入（判据与负控 ✓）：卷状态/降级/重建进度/日志计数 + `member_fail(vol,drv)`（拔盘 ✓）
//
// ⛔ 简化（内部开发计划 §4 ✓）：不做 DDR4 缓存（成员后端直通 ✓）、不做 SMP/STP、不做 GTH/PHY；
//   RAID 元数据应答复用 HD D 侧数据 ⇒ 见 §"本地应答" ✓
//=============================================================================
#ifndef SAS_RAID_TLM_H
#define SAS_RAID_TLM_H

#include <systemc.h>
#include <cstring>
#include <cstdint>
#include <vector>
#include "sas_raid_regs.h"        // -I../sw ✓（几何/公式/卷表/日志 ✓）

#define SAS_RAID_TLM_VERSION_STR "1.0"

struct SasRaidTlm : sc_core::sc_module {
    // ── 时钟/复位（与全工程同纪律：SC_METHOD 每拍一步 ✓）──
    sc_core::sc_in<bool> clk, rst_n;

    // ── 白盒观测（⛔ 只 TB 用 ✓）──
    sc_core::sc_out<sc_uint<32>> o_reads;        // 虚拟读命令数
    sc_core::sc_out<sc_uint<32>> o_writes;       // 虚拟写命令数
    sc_core::sc_out<sc_uint<32>> o_rmw;          // RMW 次数（R5 部分条带写 ✓）
    sc_core::sc_out<sc_uint<32>> o_full_stripe;  // 全条带写次数 ✓
    sc_core::sc_out<sc_uint<32>> o_degraded;     // 降级读次数 ✓
    sc_core::sc_out<sc_uint<32>> o_rebuild_blocks; // 重建块数 ✓
    sc_core::sc_out<sc_uint<32>> o_journal_ents; // 当前日志占用 ✓
    sc_core::sc_out<sc_uint<32>> o_member_ios;   // 成员盘 I/O 数 ✓

    // ── 卷表（4 卷 ✓；配置经 vol_config ✓）──
    raid_volume_t vol[RAID_MAX_VOLUMES];

    // ── 成员后端（TB/顶层绑定 ✓；返回 false = 该盘故障 ✗）──
    //    单元台：内存数组 ✓；联合台：HDD TLM 经 link ✓（同步调用 ✓ 本模型逐拍推进 ✓）
    bool (*backend_read )(void *ctx, unsigned vol, unsigned drv, uint64_t lba, uint8_t *buf, unsigned nblk);
    bool (*backend_write)(void *ctx, unsigned vol, unsigned drv, uint64_t lba, const uint8_t *buf, unsigned nblk);
    void *backend_ctx;

    SC_HAS_PROCESS(SasRaidTlm);
    SasRaidTlm(sc_core::sc_module_name nm);
    static const char *version() { return SAS_RAID_TLM_VERSION_STR; }

    // ── 白盒计数读数（⛔ 只 TB 用 ✓；与 o_* 端口同源 ✓ 免时钟驱动即可读 ✓）──
    uint32_t cnt_rd()   const { return c_rd; }
    uint32_t cnt_wr()   const { return c_wr; }
    uint32_t cnt_rmw()  const { return c_rmw; }    // RMW 单元写次数 ✓
    uint32_t cnt_full() const { return c_full; }   // 全条带写次数 ✓
    uint32_t cnt_deg()  const { return c_deg; }    // 降级重建读次数 ✓
    uint32_t cnt_rb()   const { return c_rb; }     // 重建已写块数 ✓

    // ── 卷配置（卷行 = (N-1)×stripe（R5）/ (N-2)×stripe（R6）/ 1×stripe（R1）✓）──
    void vol_config(unsigned v, int level, unsigned n_drives, uint32_t stripe_size,
                    uint64_t cap_blocks);           // cap_blocks = 虚拟盘块数（512B ✓）

    // ── 故障注入（负控 ✓）：拔/插盘（= DRV_FAILED / DRV_ONLINE ✓，联动卷状态 ✓）──
    void member_fail(unsigned v, unsigned drv);
    void member_insert(unsigned v, unsigned drv);   // 插回 ⇒ 若卷降级则转 REBUILDING ✓

    // ── 虚拟盘面（= guest/HBA 侧调用 ✓；返回 false ⇒ 该卷不可服务 ✗）──
    bool vdisk_read (unsigned v, uint64_t lba, uint8_t *buf, unsigned nblk);
    bool vdisk_write(unsigned v, uint64_t lba, const uint8_t *buf, unsigned nblk);

    // ── 重建（M5 ✓）：按条带扫描、逐条带重建到 SPARE/回插盘 ✓ ──
    bool rebuild_step(unsigned v);                  // 推进一步（一条带行 ✓）
    uint64_t rebuild_lba(unsigned v) const { return rb_scan[v]; }
    unsigned rebuild_done(unsigned v) const { return vol[v].state == VOL_ONLINE && rb_scan[v] >= vol_cap[v]; }

    // ── 写洞日志（M5 ✓；简化：记录 + 幂等重放 ✓）──
    unsigned journal_count() const;
    void     journal_replay(unsigned v);            // 崩后重放（幂等 ✓）

    // ── 本地应答数据（元命令；与 sas_hdd/tlm 字节级一致 ✓ —— 由调用方取用 ✓）──
    static const int INQ_LEN = 36, MODE_SENSE_LEN = 64, REQ_SENSE_LEN = 64, REP_LUNS_LEN = 64, RCAP10_LEN = 8;
    void meta_inquiry(uint8_t out[INQ_LEN]) const;              // 复用 HDD 侧逐字数据 ✓
    void meta_rcap10(uint8_t out[RCAP10_LEN]) const;            // **容量 = 卷容量** ✓（非成员盘 ✓）
    void meta_mode_sense(uint8_t out[MODE_SENSE_LEN]) const;
    void meta_req_sense(uint8_t out[REQ_SENSE_LEN]) const;

private:
    void step();
    // ── 写洞日志小件（写校验前记 ✓ 完成后清 ✓ 环满丢弃 ✓ 简化实现）──
    void journal_add(unsigned v, uint64_t stripe) {
        unsigned nx = (jr_head + 1) & (RAID_JOURNAL_ENTS - 1);
        if (nx == jr_tail) return;                    // 环满：丢（简化 ✓）
        jrnl[jr_head].vol = (uint8_t)v; jrnl[jr_head].stripe = stripe; jrnl[jr_head].in_use = 1;
        jr_head = nx;
    }
    void journal_clear() { if (jr_tail != jr_head) jr_tail = (jr_tail + 1) & (RAID_JOURNAL_ENTS - 1); }
    // ── 内部：成员访问（含故障/状态检查 ✓）──
    bool m_read (unsigned v, unsigned drv, uint64_t lba, uint8_t *buf, unsigned nblk);
    bool m_write(unsigned v, unsigned drv, uint64_t lba, const uint8_t *buf, unsigned nblk);
    // ★ 读/写口径分离（2026-10-09 定型 ✓ 台逼出来的）：
    //   `drv_ok`   = 仅 ONLINE ⇒ **读**可信（REBUILDING 盘内容未建全 ✗ 读必走重建 ✓）
    //   `drv_wr_ok` = ONLINE|REBUILDING ⇒ **写**允许（重建写回目标盘 ✓ 主机写不落空 ✓）
    bool drv_ok(unsigned v, unsigned drv) const
        { return vol[v].m[drv].state == DRV_ONLINE; }
    bool drv_wr_ok(unsigned v, unsigned drv) const
        { return vol[v].m[drv].state == DRV_ONLINE || vol[v].m[drv].state == DRV_REBUILDING; }
    // ── 内部：条带级操作（步进缓冲 = 一条带行 ✓ 64KB×(N-1) 上限 = 15×64K = 960KB ✓）──
    bool read_row (unsigned v, uint64_t stripe, uint8_t *row);      // 整行读（降级时重建 ✓）
    bool write_row(unsigned v, uint64_t stripe, const uint8_t *row); // 整行写（重算校验 ✓）
    bool read_stripe_data(unsigned v, uint64_t stripe, unsigned data_idx, uint64_t off, uint8_t *buf, unsigned nblk);
    // ── 观测计数 ──
    uint32_t c_rd, c_wr, c_rmw, c_full, c_deg, c_rb, c_mio;
    uint32_t rr[RAID_MAX_VOLUMES];   // R1/R10 读均衡轮转（逐次交替起读副本 ✓ M2-B5 ✓）
    // ── 重建扫描位点（每卷 ✓）与缓冲 ──
    uint64_t rb_scan[RAID_MAX_VOLUMES];
    uint64_t vol_cap[RAID_MAX_VOLUMES];
    std::vector<uint8_t> rowbuf;    // 一条带行缓冲（惰性分配 ✓）
    // ── 写洞日志（环 256 ✓ 简化实现：只记最近 N 条 + 重放幂等 ✓）──
    raid_journal_ent_t jrnl[RAID_JOURNAL_ENTS];
    unsigned jr_head, jr_tail;
    // ── 元数据应答（构造期填好，字节级照 sas_hdd/tlm ✓）──
    uint8_t meta_inq[INQ_LEN], meta_ms[MODE_SENSE_LEN], meta_rs[REQ_SENSE_LEN];
};

#endif // SAS_RAID_TLM_H
