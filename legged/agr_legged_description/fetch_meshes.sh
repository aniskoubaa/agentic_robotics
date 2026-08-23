#!/usr/bin/env bash
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
# Fetch the official Unitree Go2 visual meshes.
#
# Same pattern as px4_msgs: upstream assets are CLONED AT SETUP TIME, pinned to
# a commit, and gitignored — never vendored into this repo. 25 MB of .dae has no
# business in git history, and copying it would also fork it: a copy silently
# stops tracking upstream fixes.
#
# The URDF works WITHOUT these — it falls back to primitive shapes with the same
# kinematics — so a machine with no network still gets a functioning robot. The
# meshes only change how it looks.
#
#   ./fetch_meshes.sh          fetch (skips files already present)
#   ./fetch_meshes.sh --force  re-fetch
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="${HERE}/meshes"
# Pinned: upstream master moves, and a mesh that silently changes shape under a
# published experiment is exactly the kind of drift this project exists to catch.
COMMIT="daadf41ee9afce8f90fdc09a98506012691fa122"
BASE="https://raw.githubusercontent.com/unitreerobotics/unitree_ros/${COMMIT}/robots/go2_description/meshes"
FILES=(base.dae hip.dae thigh.dae thigh_mirror.dae calf.dae calf_mirror.dae foot.dae)

FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

mkdir -p "$DEST"
missing=0
for f in "${FILES[@]}"; do
  if [[ -s "${DEST}/${f}" && $FORCE -eq 0 ]]; then
    printf '  %-20s present\n' "$f"; continue
  fi
  printf '  %-20s fetching…' "$f"
  if curl -fsSL --retry 3 --max-time 120 -o "${DEST}/${f}.part" "${BASE}/${f}"; then
    mv "${DEST}/${f}.part" "${DEST}/${f}"
    printf ' %s\n' "$(du -h "${DEST}/${f}" | cut -f1)"
  else
    rm -f "${DEST}/${f}.part"
    printf ' FAILED\n'; missing=$((missing+1))
  fi
done

if (( missing > 0 )); then
  cat >&2 <<EOF

⚠ ${missing} mesh(es) could not be fetched. The Go2 will still load and behave
  identically — the URDF falls back to primitives when a mesh is absent. Only
  the visuals differ. Re-run this script when you have network.
EOF
  exit 0     # not fatal: a mesh-less robot is still a working robot
fi

echo
echo "✓ Go2 meshes ready in ${DEST}"
echo "  Licence: BSD-3-Clause, unitreerobotics/unitree_ros @ ${COMMIT:0:10}"
