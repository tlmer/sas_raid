/*============================================================================
 * sas_raid_regs.h — SAS RAID HBA 共同基准：几何/翻译公式/卷表/日志  [2026-10-09 建]
 *
 * ★★ 本文件 = **RAID 层（TLM / RTL / TB / 序列层）的唯一基准** —— 每条带出处：
 *     · 几何/地址翻译 = `规格文档` §"RAID Volume Address Translation" ✓
 *     · RAID 等级/卷状态机/日志/条带映射 = `架构文档` §4.13 ✓
 *     · 帧/命令口径**沿用现 SAS HBA/HDD**（CDB@36 / tag@16-17 / LUN@24-31 ✓）
 *       ⇒ 见 `sas_hba/tlm/sw/sas_hba_regs.h` ✓（两侧对拍红线 ✓）
 *
 * ⚠ 改本文件 ⇒ 必须同步 `内部内部开发计划` §6 与 RAID《设计方案》✓
 *============================================================================*/
#ifndef SAS_RAID_REGS_H
#define SAS_RAID_REGS_H

#include <stdint.h>
#include <stddef.h>

/*==========================================================================*/
/* §1 几何常量（arch §4.13 / spec ✓）                                        */
/*==========================================================================*/
#define RAID_MAX_VOLUMES      4        /* 卷表 4 卷 ✓ */
#define RAID_MAX_DRIVES       16       /* 每卷 ≤16 盘 ✓ */
#define RAID_BLK_BYTES        512      /* 逻辑块 512B ✓（与 HDD 侧一致 ✓）*/
#define RAID_STRIPE_SIZE      (64u*1024u)  /* 默认条带 64KB ✓（可配 64K..1M ✓）*/
#define RAID_STRIPE_MAP_ENTS  256      /* Stripe Map Cache：256 项 4 路组相联 ✓ */
#define RAID_STRIPE_MAP_WAYS  4
#define RAID_JOURNAL_ENTS     256      /* 写洞日志：环 256 项 ✓ */
#define RAID_MAX_MISSING_R5   1        /* R5 容忍 1 盘缺失 ✓ */
#define RAID_MAX_MISSING_R6   2        /* R6 容忍 2 盘缺失 ✓ */

/*==========================================================================*/
/* §2 RAID 等级与卷状态（arch §4.13 ✓）                                      */
/*==========================================================================*/
enum {
    RAID_LVL_R0  = 0,     /* 条带化，无冗余 ✓（2..16 盘 ✓）*/
    RAID_LVL_R1  = 1,     /* 镜像 ✓（恰 2 盘 ✓）*/
    RAID_LVL_R5  = 5,     /* 条带 + 分布式 XOR 校验 ✓（3..16 盘 ✓）*/
    RAID_LVL_R6  = 6,     /* 条带 + 双校验 P/Q ✓（4..16 盘 ✓）*/
    RAID_LVL_R10 = 10,    /* 镜像 of 条带 ✓（4..16 偶数 ✓）*/
};
enum {
    VOL_OFFLINE = 0, VOL_INITIALIZING, VOL_ONLINE, VOL_DEGRADED, VOL_REBUILDING,
};
enum {                    /* 成员盘状态 ✓ */
    DRV_ONLINE = 0, DRV_FAILED, DRV_SPARE, DRV_REBUILDING,
};

/*==========================================================================*/
/* §3 地址翻译（★ spec §"RAID Volume Address Translation" 五式，逐位固化 ✓）  */
/*==========================================================================*/
typedef struct {
    uint64_t stripe;         /* 条带号 ✓ */
    uint64_t offset;         /* 条带内字节偏移（0..STRIPE_SIZE*(N-1)-1 ✓）*/
    unsigned drive;          /* **条带内数据盘序**（0..N-2 ✓）*/
    unsigned p_drive;        /* **物理盘号**上的奇偶盘（left-asymmetric ✓）*/
    uint64_t drive_lba;      /* 成员盘上的 LBA（512B 块 ✓）*/
} raid_xlate_t;

/* 说明（防歧义 ✓）：`drive` 是"条带内第几个数据块"，**物理盘号** = 依奇偶盘位置跳过的映射 ✓
 *   —— 对 R5：物理盘 d 若 == p_drive ⇒ 该行放 P；否则按数据序跳过 p_drive 落位 ✓
 *   （left-asymmetric：p_drive = (N-1) - (stripe % N) ✓ spec 例：N=4,stripe=0 ⇒ p=3 ✓）*/
static inline unsigned raid_phys_drive_r5(const raid_xlate_t *x, unsigned n, unsigned data_idx) {
    unsigned d = 0, k = 0;
    for (d = 0; d < n; d++) {
        if (d == x->p_drive) continue;          /* 跳过奇偶盘 ✓ */
        if (k == data_idx) return d;
        k++;
    }
    return 0;
}

/* 五式（spec ✓）：LBA 为 512B 块号 ✓；n = 卷成员盘数；nd = **数据盘数** ✓
 *   nd：R0/R10 ⇒ n（R10 按 n/2 对 ✓，见调用方）、R5 ⇒ n-1 ✓、R6 ⇒ n-2 ✓、R1 ⇒ 1 ✓ */
static inline void raid_xlate_nd(uint64_t lba, unsigned n, unsigned nd,
                                 unsigned stripe_size, raid_xlate_t *x) {
    uint64_t bytes = lba * (uint64_t)RAID_BLK_BYTES;
    uint64_t row   = (uint64_t)stripe_size * (uint64_t)nd;            /* 一条带行的字节数 ✓ */
    x->stripe      = bytes / row;                                     /* stripe ✓ */
    x->offset      = bytes % row;                                     /* offset ✓ */
    x->drive       = (unsigned)(x->offset / stripe_size);             /* drive（数据序 ✓）*/
    x->p_drive     = (unsigned)((n - 1) - (x->stripe % n));           /* parity drive ✓ */
    x->drive_lba   = (x->stripe * stripe_size + (x->offset % stripe_size)) / RAID_BLK_BYTES;
}
/* R5 特例（spec 原式 ✓ N-1）：保留给逐字对拍用 ✓ */
static inline void raid_xlate(uint64_t lba, unsigned n, unsigned stripe_size, raid_xlate_t *x) {
    raid_xlate_nd(lba, n, n - 1, stripe_size, x);
}

/* 条带内数据盘/校验盘容量口径：一条带行 = (N-1)×stripe_size（R5 ✓）/(N-2)（R6 ✓）*/
static inline uint64_t raid_row_bytes(unsigned n, unsigned stripe_size, int lvl) {
    unsigned nd = (lvl == RAID_LVL_R6) ? (n - 2) : (lvl == RAID_LVL_R1 ? 1 : (n - 1));
    return (uint64_t)stripe_size * nd;
}

/*==========================================================================*/
/* §4 卷表 / 成员盘（arch §4.13 ✓）                                          */
/*==========================================================================*/
typedef struct {
    uint64_t sas_addr;       /* 成员 SAS 地址（低位 24b 进帧目的哈希 ✓）*/
    uint8_t  index;          /* 卷内盘序 ✓ */
    uint8_t  state;          /* DRV_* ✓ */
} raid_member_t;

typedef struct {
    uint8_t  in_use;
    uint8_t  level;          /* RAID_LVL_* ✓ */
    uint8_t  n_drives;       /* 成员盘数 ✓ */
    uint8_t  state;          /* VOL_* ✓ */
    uint32_t stripe_size;    /* 64K..1M ✓ */
    raid_member_t m[RAID_MAX_DRIVES];
} raid_volume_t;

/*==========================================================================*/
/* §5 写洞日志条目（arch §4.13 ✓：写前记 {VolID,stripe,old_P,new_P} ✓）      */
/*==========================================================================*/
typedef struct {
    uint8_t  vol;
    uint8_t  in_use;
    uint64_t stripe;
    uint32_t old_p_crc;      /* 旧校验指纹（模型：直接存 old_P 内容哈希 ✓）*/
    uint32_t new_p_crc;
} raid_journal_ent_t;

/*==========================================================================*/
/* §6 寄存器面（RAID 段；spec §"Error Injection & Debug" 0x800.. 段口径 ✓）   */
/*==========================================================================*/
#define RAID_REG_VOL_BASE(v)     (0x400u + ((v) * 0x40u))   /* 卷表窗口：每卷 0x40 ✓ */
#define RAID_VOL_CTRL            0x00   /* [0]=enable [3:1]=level [11:8]=ndrv [15:12]=state ✓ */
#define RAID_VOL_STRIPE_SZ       0x04   /* 条带字节数（64K..1M ✓）*/
#define RAID_VOL_CAP_LO          0x08   /* 虚拟容量低 32（块 ✓）*/
#define RAID_VOL_CAP_HI          0x0C
#define RAID_VOL_MBR(i)          (0x10u + ((i) * 2u))      /* 成员盘状态字节×16 ✓ */
#define RAID_REG_JOURNAL_BASE    0x600  /* 日志环窗口（256×16B ✓）*/
#define RAID_REG_JOURNAL_PTR     0x7F0  /* 头/尾 ✓ */
#define RAID_REG_REBUILD_CTRL    0x800  /* [7:0]=rate% [8]=start [9]=pause ✓ */
#define RAID_REG_REBUILD_SCAN    0x804  /* 当前扫描 LBA ✓ */
#define RAID_REG_REBUILD_STS     0x808  /* [3:0]=状态（idle/scan/checkpoint ✓）*/
#define RAID_REG_ERR_INJ         0x820  /* 错误注入（spec 0x800–0x83F 段 ✓）*/
#define RAID_REG_DBG_SEL         0x840  /* 调试监视选择 ✓ */

/*==========================================================================*/
/* §7 与现 SAS HBA 的交界（复用边界 ✓ 内部开发计划 §1）                          */
/*==========================================================================*/
/* 命令流：host CDB（虚拟卷）⇒ RAID 层翻译 ⇒ 每成员盘一条**标准 SSP CDB** ✓
 *   · 帧/命令口径 = 现 SAS HBA 既有口径，**不改帧** ✓
 *   · 成员盘命令的 `dev_id`（DQ dw1[31:16]）= 成员盘 index ✓
 *   · RAID 元数据（journal/重建扫描）走 AXI-ID 组 `11` ✓（spec AXI-ID 表 ✓）*/

#endif /* SAS_RAID_REGS_H */
