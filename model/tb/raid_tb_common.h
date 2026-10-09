//=============================================================================
// raid_tb_common.h — RAID 台公共件：内存成员后端 + 小宿主  [2026-10-09 建]
//-----------------------------------------------------------------------------
// 成员后端（单元台 ✓）：每卷每盘一块内存阵列（`backend_read/write` ✓）——
//   联合台（M6 ✓）改用 `SasHddTlm` 经 link 实现同一对函数指针 ✓（松耦合 ✓）
//=============================================================================
#ifndef RAID_TB_COMMON_H
#define RAID_TB_COMMON_H

#include <systemc.h>
#include <vector>
#include <cstring>
#include <cstdio>
#include "sas_raid_tlm.h"

// ── 内存成员后端（16 盘 × 4 卷 × 8MB ✓ 够单元台用 ✓）──
struct RaidMemBackend {
    static const int MAXV = RAID_MAX_VOLUMES, MAXD = RAID_MAX_DRIVES;
    std::vector<std::vector<std::vector<uint8_t>>> mem;   // [v][d][bytes]
    size_t cap_bytes;
    uint64_t rd_cnt[MAXV][MAXD], wr_cnt[MAXV][MAXD];      // 逐盘计数（观测用 ✓：读均衡/写惩罚 ✓）
    RaidMemBackend(size_t bytes = 8u * 1024u * 1024u) : cap_bytes(bytes) {
        mem.assign(MAXV, std::vector<std::vector<uint8_t>>(MAXD));
        for (int v = 0; v < MAXV; v++)
            for (int d = 0; d < MAXD; d++) mem[v][d].assign(cap_bytes, 0);
        reset_cnt();
    }
    void reset_cnt() { memset(rd_cnt, 0, sizeof(rd_cnt)); memset(wr_cnt, 0, sizeof(wr_cnt)); }
    static bool rd(void *ctx, unsigned v, unsigned d, uint64_t lba, uint8_t *buf, unsigned nblk) {
        RaidMemBackend *b = (RaidMemBackend *)ctx;
        size_t off = (size_t)lba * RAID_BLK_BYTES, len = (size_t)nblk * RAID_BLK_BYTES;
        if (v >= MAXV || d >= MAXD || off + len > b->cap_bytes) return false;
        b->rd_cnt[v][d]++;
        memcpy(buf, &b->mem[v][d][off], len);
        return true;
    }
    static bool wr(void *ctx, unsigned v, unsigned d, uint64_t lba, const uint8_t *buf, unsigned nblk) {
        RaidMemBackend *b = (RaidMemBackend *)ctx;
        size_t off = (size_t)lba * RAID_BLK_BYTES, len = (size_t)nblk * RAID_BLK_BYTES;
        if (v >= MAXV || d >= MAXD || off + len > b->cap_bytes) return false;
        b->wr_cnt[v][d]++;
        memcpy(&b->mem[v][d][off], buf, len);
        return true;
    }
    void bind(SasRaidTlm &m) { m.backend_ctx = this; m.backend_read = rd; m.backend_write = wr; }
    uint8_t *ptr(unsigned v, unsigned d) { return mem[v][d].data(); }
};

// ── 模式填充/校验小工具（判据用 ✓）──
static inline void pat_fill(uint8_t *p, size_t n, uint32_t seed) {
    for (size_t i = 0; i < n; i++) p[i] = (uint8_t)((seed * 131u + i * 7u + (i >> 8)) & 0xFF);
}
static inline bool pat_check(const uint8_t *p, size_t n, uint32_t seed) {
    for (size_t i = 0; i < n; i++)
        if (p[i] != (uint8_t)((seed * 131u + i * 7u + (i >> 8)) & 0xFF)) return false;
    return true;
}

// ── TB 记分小工具（口径同全工程 ✓：预登记 + chk + 终判行 + 退出码 ✓）──
struct RaidTbScore {
    int pass = 0, fail = 0;
    void chk(bool ok, const char *name) {
        printf("   %s %s\n", ok ? "✓" : "✗", name);
        if (ok) pass++; else fail++;
    }
    int verdict(const char *tb) {
        printf("[合计] PASS=%d FAIL=%d\n", pass, fail);
        printf("%s %s\n", tb, fail == 0 ? "PASS" : "FAIL");
        return fail == 0 ? 0 : 1;
    }
};

#endif // RAID_TB_COMMON_H
