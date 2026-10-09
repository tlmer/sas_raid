//=============================================================================
// sas_raid_array_top.h — RAID ⇄ N×(HBA TLM ⇄ link ⇄ HDD TLM) 联合顶层
//   [2026-10-09 建；M6 交互性台用 ✓ 用户令"交互性测试" ✓]
//-----------------------------------------------------------------------------
// 结构（内部开发计划 §2/§3-M6 ✓）：
//   · **每成员盘一路独立点对点 SAS 域** —— 直接复用 `SasPairTop`（自带
//     hba+link+hdd+mem ✓ 已 23 判据全绿 ✓ `tb_pair_bringup`）
//   · RAID 层 `SasRaidTlm` 的成员后端 = 本顶层的 `member_io` ⇒ 成员 I/O 走
//     **真帧级** SSP 栈：填 CT(READ10/WRITE10)+SGE+DQ → 门铃 → HBA 取指 →
//     SSP COMMAND → link → HDD → DATA 帧/RESPONSE → CQE → 读回 ✓
//
// ⚠ 接线纪律（照 `tb_pair_bringup` 实踩 ✓）：
//   · **每通道自有 mem** —— VDK 从口绑策略 = ONE ✗（多 HBA 共绑一个 mem 会
//     E109「2 binds exceeds maximum」✗）⇒ 各通道地址区间相同（空间独立 ✓）
//   · 各通道 `hba.reg_s` 须绑**占位主端**（`DummyAmbaMaster` ¥ 1/通道 ✓）
//   · `member_io` 内含 `wait(clk.posedge)` ⇒ **只能在 SC_THREAD 上下文调用** ✓
//
// ⚠ 简化（内部开发计划 §4 ✓）：单 SGE（HBA 侧既有简化 ✓）、点对点无扩展器
//   （dev_id 恒 1/通道 ✓）、HDD 侧错误注入未在本台单列（能力见 tb_pair ✓）。
//=============================================================================
#ifndef SAS_RAID_ARRAY_TOP_H
#define SAS_RAID_ARRAY_TOP_H

#include <systemc.h>
#include <cstring>
#include <cstdint>
#include "sas_pair_top.h"          // ← 现 SAS HBA/HDD TLM（-I 路径由 Makefile 给 ✓）
#include "sas_hba_hw.h"            // DQ/CT/SGE 打包件 + CQE 访问子 ✓（-I../sw ✓）
#include "sas_raid_tlm.h"

struct SasRaidArrayTop : sc_core::sc_module {
    static const unsigned NCH = 4;                 // R5 4 成员 = 4 路 ✓（R6=6 需扩 ✓ 扩展点）

    sc_core::sc_in<bool> clk, rst_n;

    // ── 每通道：独立 HBA ⇄ link ⇄ HDD ⇄ mem（地址区间相同、空间独立 ✓）──
    SasPairTop ch0, ch1, ch2, ch3;

    SasRaidTlm raid;
    sc_core::sc_signal<sc_uint<32>> r_rd, r_wr, r_rmw, r_full, r_deg, r_rb, r_jr, r_mio;

    // ── 成员 I/O 账目（TB 对账用 ✓）──
    uint32_t cqes_done[NCH];
    uint32_t io_cnt[NCH];
    uint64_t blk_rd[NCH], blk_wr[NCH];
    uint32_t tag[NCH];

    // ── 每通道地址规划（**通道自有 mem** 的同一组偏移 ✓）──
    static const uint64_t MEM_BASE = 0xfe400000ull;
    static const uint64_t MEM_SIZE = 0x01000000ull;        // 16MB ✓
    static const uint64_t DQ_BASE  = MEM_BASE + 0x00000;   // 64 条 ×64B = 4KB ✓
    static const uint64_t CQ_BASE  = MEM_BASE + 0x01000;   // 64 条 ×16B = 1KB ✓
    static const uint64_t CT_BASE  = MEM_BASE + 0x02000;
    static const uint64_t SGE_BASE = MEM_BASE + 0x03000;
    static const uint64_t STS_BASE = MEM_BASE + 0x04000;
    static const uint64_t DAT_BASE = MEM_BASE + 0x10000;   // 4 页 ×256KB 轮转 ✓
    static const uint32_t DQ_DEPTH = 64, CQ_DEPTH = 64;

    SC_HAS_PROCESS(SasRaidArrayTop);
    SasRaidArrayTop(sc_core::sc_module_name nm)
      : sc_core::sc_module(nm), clk("clk"), rst_n("rst_n"),
        ch0("ch0", MEM_BASE, MEM_SIZE, 4096), ch1("ch1", MEM_BASE, MEM_SIZE, 4096),
        ch2("ch2", MEM_BASE, MEM_SIZE, 4096), ch3("ch3", MEM_BASE, MEM_SIZE, 4096),
        raid("raid"),
        r_rd("r_rd"), r_wr("r_wr"), r_rmw("r_rmw"), r_full("r_full"), r_deg("r_deg"),
        r_rb("r_rb"), r_jr("r_jr"), r_mio("r_mio")
    {
        ch0.clk(clk); ch0.rst_n(rst_n);
        ch1.clk(clk); ch1.rst_n(rst_n);
        ch2.clk(clk); ch2.rst_n(rst_n);
        ch3.clk(clk); ch3.rst_n(rst_n);
        raid.clk(clk); raid.rst_n(rst_n);
        raid.o_reads(r_rd); raid.o_writes(r_wr); raid.o_rmw(r_rmw); raid.o_full_stripe(r_full);
        raid.o_degraded(r_deg); raid.o_rebuild_blocks(r_rb); raid.o_journal_ents(r_jr);
        raid.o_member_ios(r_mio);
        for (unsigned i = 0; i < NCH; i++) {
            cqes_done[i] = 0; io_cnt[i] = 0; blk_rd[i] = 0; blk_wr[i] = 0; tag[i] = 0;
        }
        raid.backend_ctx   = this;
        raid.backend_read  = &SasRaidArrayTop::be_read;
        raid.backend_write = &SasRaidArrayTop::be_write;
    }

    SasPairTop &chan(unsigned d) {
        return d == 0 ? ch0 : d == 1 ? ch1 : d == 2 ? ch2 : ch3;
    }

    static bool be_read(void *ctx, unsigned v, unsigned d, uint64_t lba, uint8_t *buf, unsigned nblk)
    { (void)v; return ((SasRaidArrayTop *)ctx)->member_io(d, false, lba, buf, nblk); }
    static bool be_write(void *ctx, unsigned v, unsigned d, uint64_t lba, const uint8_t *buf, unsigned nblk)
    { (void)v; return ((SasRaidArrayTop *)ctx)->member_io(d, true, lba, (uint8_t *)buf, nblk); }

    // ── 每通道 HBA 编程（复位释放后调用一次 ✓）──
    void hba_setup(unsigned d) {
        SasPairTop &p = chan(d);
        // ⚠ 关模型内响应超时（默认 4096 拍 ✗ 实测：64KB = ~67 帧 > 4096 ⇒ 半途报错 ✗）
        //   —— 本台 per-I/O 自带上限（500k 拍 ✓）兜底 ✓
        p.hba.cfg_resp_timeout = 0;
        p.hba.reg_write(SAS_HBA_CFG_DLVRY_QUEUE_ENABLE, sc_uint<32>(1));
        p.hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(DQ_BASE & 0xFFFFFFFFu));
        p.hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_HI), sc_uint<32>(DQ_BASE >> 32));
        p.hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_DEPTH), sc_uint<32>(DQ_DEPTH));
        p.hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(CQ_BASE & 0xFFFFFFFFu));
        p.hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_HI), sc_uint<32>(CQ_BASE >> 32));
        p.hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_DEPTH), sc_uint<32>(CQ_DEPTH));
        p.hba.reg_write(SAS_HBA_INT_OQ_SRC_MSK, sc_uint<32>(0));
    }

    // ── 成员 I/O（**帧级全栈** ✓）：读/写 nblk 块 @ lba —— ⚠ 仅 SC_THREAD 内可调 ✓ ──
    bool member_io(unsigned d, bool write, uint64_t lba, uint8_t *buf, unsigned nblk) {
        SasPairTop &p = chan(d);
        uint32_t t = ++tag[d];
        unsigned bytes = nblk * 512u;
        uint64_t dat = DAT_BASE + (uint64_t)((t - 1) & 3u) * 0x40000;      // 4 页轮转 ✓
        if (write) p.mem.poke(dat, buf, bytes);

        // CT：SSP 命令表（CDB = READ10 0x28 / WRITE10 0x2A ✓ LBA 大端 ✓）
        uint8_t cdb[16];
        std::memset(cdb, 0, sizeof(cdb));
        cdb[0] = write ? 0x2A : 0x28;
        cdb[2] = (uint8_t)(lba >> 24); cdb[3] = (uint8_t)(lba >> 16);
        cdb[4] = (uint8_t)(lba >> 8);  cdb[5] = (uint8_t)lba;
        cdb[7] = (uint8_t)(nblk >> 8); cdb[8] = (uint8_t)nblk;
        uint8_t tbl[SAS_HBA_CTBL_SSP_BYTES];
        uint8_t lun[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        sas_ctbl_pack_ssp(tbl, cdb, lun, 0, 0, 0);
        p.mem.poke(CT_BASE, tbl, sizeof(tbl));

        // SGE（内核 24B 布局 ✓ 红线⑤）
        uint8_t sge[SAS_HBA_SGE_BYTES];
        sas_sge_pack(sge, dat, bytes, 0);
        p.mem.poke(SGE_BASE, sge, sizeof(sge));

        // DQ 条目（环形：条目 idx=(t-1)%深度 ✓ 门铃 wp=t%深度 ✓ 与模型 rd_ptr 取模口径一致 ✓）
        uint32_t dqw[SAS_HBA_DQ_ENTRY_DW];
        sas_dq_pack_ssp(dqw, (uint16_t)t, 0x0001,
                        write ? SAS_HBA_DIR_TO_DEVICE : SAS_HBA_DIR_TO_INI,
                        bytes, CT_BASE, SGE_BASE);
        sas_dw_set_u64(dqw, SAS_HBA_DQ_STS_BUF_LO_DW, STS_BASE);
        p.mem.poke(DQ_BASE + (uint64_t)((t - 1) % DQ_DEPTH) * 64u, dqw, sizeof(dqw));
        p.hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), sc_uint<32>(t % DQ_DEPTH));

        // 等 CQE（累计 +1 ✓ 上限防挂 ✓）
        uint32_t want = cqes_done[d] + 1u;
        long cap = 500000;
        while (p.hba.o_cqes.read().to_uint() < want && cap-- > 0)
            wait(clk.posedge_event());
        if (cap <= 0) return false;                                        // 超时 ✗（上层按失败处理 ✓）
        cqes_done[d] = want;
        io_cnt[d]++;
        // 读回 CQE（环形：第 want 条 idx=(want-1)%深度 ✓）
        uint32_t cqe[SAS_HBA_CQE_BYTES / 4];
        p.mem.peek(CQ_BASE + (uint64_t)((want - 1u) % CQ_DEPTH) * SAS_HBA_CQE_BYTES,
                   cqe, sizeof(cqe));
        bool good = (sas_cqe_cmplt(cqe) == 0u);
        if (!good) return false;
        if (write) blk_wr[d] += nblk; else { blk_rd[d] += nblk; p.mem.peek(dat, buf, bytes); }
        return true;
    }
};

#endif // SAS_RAID_ARRAY_TOP_H
