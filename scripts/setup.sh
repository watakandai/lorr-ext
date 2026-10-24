#!/usr/bin/env bash
# Fetch the LoRR sources, build patched copies of every solver kit, and stage
# the benchmark instance. Safe to re-run; kits are rebuilt from clean copies.
#
#   bash scripts/setup.sh            # all kits
#   KITS="lorr2026 nms2024" bash scripts/setup.sh
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
TP=$ROOT/third_party
mkdir -p "$TP" kits instances/base/{maps,agents,tasks}

START_KIT_TAG=v3.1.0
NMS=44950d1b61b7482b4529a9dc127fa9d220962bdc      # No Man's Sky, 2024 1st (Combined)
KITTY=da7d428b40cc1425b381adefe9709d4accbfdfe1    # Kitty_Knight, 2024 2nd
DEF24=5ea4fa167908e5bfe53a7f2a768f5d12dccf3c5a    # 2024 default entry
A24="2024 Competition"

[ -d "$TP/Start-Kit" ] || git clone -q https://github.com/MAPF-Competition/Start-Kit.git "$TP/Start-Kit"
git -C "$TP/Start-Kit" checkout -q "$START_KIT_TAG"
if [ ! -d "$TP/Code-Archive" ]; then
  git clone -q --depth 1 --filter=blob:none --sparse https://github.com/MAPF-Competition/Code-Archive.git "$TP/Code-Archive"
  git -C "$TP/Code-Archive" sparse-checkout set "$A24/Team_No_Man's_Sky/$NMS" "$A24/Team_Kitty_Knight/$KITTY" "$A24/Team_Default_Entry/$DEF24"
fi
if [ ! -d "$TP/Benchmark-Archive" ]; then
  git clone -q --depth 1 --filter=blob:none --sparse https://github.com/MAPF-Competition/Benchmark-Archive.git "$TP/Benchmark-Archive"
  git -C "$TP/Benchmark-Archive" sparse-checkout set "$A24/Main Round Evaluation Instances/random.domain" \
    "$A24/Problem Generator/script"
fi
# research layouts (lorrext/instances.py) read warehouse_small.map / sortation_small.map from here
git -C "$TP/Benchmark-Archive" sparse-checkout add "$A24/Problem Generator/script" 2>/dev/null || true

# benchmark instance: 2024 main-round RANDOM-01 map/agents/tasks (100 robots)
R="$TP/Benchmark-Archive/$A24/Main Round Evaluation Instances/random.domain"
cp "$R/maps/random-32-32-20.map" instances/base/maps/
cp "$R/agents/random_32_32_20_100.agents" "$R/agents/random_32_32_20_200.agents" "$R/agents/random_32_32_20_400.agents" instances/base/agents/
cp "$R/tasks/random_32_32_20.tasks" instances/base/tasks/random_32_32_20_official.tasks

declare -A SRC=(
  [lorr2026]="$TP/Start-Kit"
  [nms2024]="$TP/Code-Archive/$A24/Team_No_Man's_Sky/$NMS"
  [kitty2024]="$TP/Code-Archive/$A24/Team_Kitty_Knight/$KITTY"
  [default2024]="$TP/Code-Archive/$A24/Team_Default_Entry/$DEF24"
  [nms2024fix]="$TP/Code-Archive/$A24/Team_No_Man's_Sky/$NMS"   # research variants (docs/research)
  [lorr2026nx]="$TP/Start-Kit"
  [lorr2026g]="$TP/Start-Kit"
  [kitty2024rs]="$TP/Code-Archive/$A24/Team_Kitty_Knight/$KITTY"
)
declare -A FLAGS=([kitty2024]="--kitty-guard" [nms2024fix]="--nms-arrival-fix" [lorr2026nx]="--plan-ahead" [lorr2026g]="--plan-ahead --pibt-reserve 20" [kitty2024rs]="--kitty-guard --kitty-resync")

build() {
  local k=$1
  rm -rf "kits/$k"
  rsync -a --exclude build --exclude .git "${SRC[$k]}/" "kits/$k/"
  python3 tools/patch_kit.py "kits/$k" ${FLAGS[$k]:-}
  (cd "kits/$k" && cmake -B build . -DPYTHON=false -DCMAKE_BUILD_TYPE=Release > build.log 2>&1 \
     && make -C build -j"$(( $(nproc) / 4 > 1 ? $(nproc) / 4 : 1 ))" lifelong >> build.log 2>&1) \
     && echo "built $k" || { echo "FAILED $k (see kits/$k/build.log)"; return 1; }
}

for k in ${KITS:-lorr2026 nms2024 kitty2024 default2024 nms2024fix lorr2026nx lorr2026g kitty2024rs}; do build "$k" & done
wait
