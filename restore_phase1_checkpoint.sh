#!/usr/bin/env bash
# ==============================================================================
# One-Click Restore Script for Swarm Drones FYP (Phase 1 Checkpoint)
# Restores the codebase to the verified Phase 1 working state (Tag: v1.0-phase1-complete)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}/swarm_drones_fyp"
BACKUP_ARCHIVE="${SCRIPT_DIR}/swarm_drones_backup_20260930_phase1.tar.gz"
CHECKPOINT_DIR="${SCRIPT_DIR}/swarm_drones_fyp_phase1_checkpoint"

echo "================================================================="
echo "  Restoring Swarm Drones FYP to Phase 1 Checkpoint"
echo "  Tag: v1.0-phase1-complete (Sep 30, 2026)"
echo "================================================================="

# Method 1: Git-based clean restore if git repository is intact
if [ -d "${PROJECT_DIR}/.git" ]; then
    echo "-> Step 1: Performing clean Git restore in ${PROJECT_DIR}..."
    cd "${PROJECT_DIR}"
    git reset --hard v1.0-phase1-complete
    git clean -fd
    git checkout milestone/phase1-stable
    echo "   [Git checkout successful: pointing to milestone/phase1-stable]"
else
    # Method 2: Filesystem-based restore from standalone backup
    echo "-> Step 1: Git repository missing or corrupted. Restoring from archive..."
    if [ -f "${BACKUP_ARCHIVE}" ]; then
        rm -rf "${PROJECT_DIR}"
        tar -xzf "${BACKUP_ARCHIVE}" -C "${SCRIPT_DIR}"
        echo "   [Extracted cleanly from ${BACKUP_ARCHIVE}]"
    elif [ -d "${CHECKPOINT_DIR}" ]; then
        rm -rf "${PROJECT_DIR}"
        cp -r "${CHECKPOINT_DIR}" "${PROJECT_DIR}"
        echo "   [Copied cleanly from ${CHECKPOINT_DIR}]"
    else
        echo "ERROR: Could not find backup archive or checkpoint directory!"
        exit 1
    fi
fi

echo "-> Step 2: Running verification test suite..."
cd "${PROJECT_DIR}"
PYTHONPATH=. python3 tests/test_graph.py
PYTHONPATH=. python3 tests/test_formations.py
PYTHONPATH=. python3 tests/test_simulation.py
PYTHONPATH=. python3 tests/test_hybrid_features.py

echo ""
echo "================================================================="
echo "  RESTORE COMPLETE! All tests verified and passing 100%."
echo "  Project restored to Phase 1 verified checkpoint."
echo "================================================================="
