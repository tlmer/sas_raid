#!/bin/sh
#=============================================================================
# regress.sh — RAID TLM 回归  [2026-10-09 建]
#   口径照全工程 ✓：**只认终判行**（`TB_* PASS/FAIL` ✓）；台清单由 Makefile
#   `TBS =` 自动同步 ✓（防"加了台忘了跑" ✗）；失败汇总退出码非零 ✓
#=============================================================================
cd "$(dirname "$0")" || exit 1

TBS=$(sed -n 's/^TBS *= *//p' Makefile)
[ -z "$TBS" ] && { echo "[regress] 台清单空 ✗"; exit 1; }

make -s || { echo "[regress] 构建失败 ✗"; exit 1; }

fail=0
for tb in $TBS; do
    line=$(./$tb 2>&1 | grep -E '^TB_[A-Z0-9_]+ (PASS|FAIL)$' | tail -1)
    printf '%-16s %s\n' "$tb" "${line:-<无终判行 ✗>}"
    case "$line" in
        *PASS) ;;
        *) fail=1 ;;
    esac
done

[ "$fail" -eq 0 ] && echo "[regress] ALL PASS ✓" || echo "[regress] FAIL ✗"
exit $fail
