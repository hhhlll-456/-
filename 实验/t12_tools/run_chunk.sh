#!/usr/bin/env bash
# Task 12 Step 4：逐 chunk 跑一个实验（3 次），chunk 之间强制归位 + 巡检。
#
#   ./t12_tools/run_chunk.sh <exp_id>
#
# 每 chunk 的固定动作（顺序即安全顺序）：
#   ① 归位到场地中心 (0,0) 并把车头摆正（只用带时长的 RUN，绝不发裸 M）。
#      归位不收敛就**中止本 chunk**、不开电机跑实验。
#   ② 跑 `--only <id>` 的 3 条实验（写表）。
#   ③ 收尾巡检：量车体原点（harness 口径）+ 包络离场地余量 + 各码检出率。
#
# 批量参数沿用 Task 12 Step 4 全量实测那一组。
set -u
cd "$(dirname "$0")/.." || exit 2          # → 实验/
ID="$1"
TAG=$(printf 'c%02d' "$ID")
D=t12_tools

echo "=============== chunk $TAG (#$ID) 归位 ==============="
timeout 900 python3 "$D/recenter.py" "pre_${TAG}" 0 0 2>&1 | tail -30
RC=${PIPESTATUS[0]}
if [ "$RC" -ne 0 ]; then
    echo "⚠ 归位未收敛（rc=$RC）—— 中止本 chunk，不开电机"
    exit 1
fi

echo "=============== chunk $TAG (#$ID) 开跑 ==============="
timeout 1800 python3 run_all_experiments.py --only "$ID" \
    --pulse-ms 120 --coarse-ms 320 --yaw-tol-deg 6 --tol-mm 4 --max-iter 24 \
    2>&1 | tee "$D/chunk_${TAG}.log" | grep -vE "RuntimeWarning|^  ff = " | tail -12

echo "=============== chunk $TAG 收尾巡检 ==============="
timeout 300 python3 "$D/carpos.py" "post_${TAG}" 9 2>&1 | tail -30