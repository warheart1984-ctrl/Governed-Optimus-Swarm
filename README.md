# Governed Optimus Swarm

A governed multi-robot swarm simulation framework featuring strict law enforcement, role-based specialization, and deterministic behavior. Includes a lightweight mining-swarm reference implementation for comparison.

## Features
- **GovernedSwarm**: Every robot action is validated against `SwarmLaw` (fail-closed). Violations result in immediate locking.
- **Role Specialization**: `SpecialistRegistry` with locked roles (`assembler`, `carrier`, `inspector`, `charger`).
- **Deterministic Task Assignment**: Manhattan-distance nearest viable task with per-tick claim management.
- **Identity Anchoring & Auditing**: Cryptographic snapshot hashing and immutable identity anchors.
- **Mining Swarm Baseline**: Simple, ungoverned drone mining system for contrast.

## Project Structure
- `spatial_model.py` – Core data models (Robot, TaskNode, FloorModel, etc.)
- `specialist_registry.py` – Role registration and permission system
- `swarm_law.py` – ARIS-style authority gate (Rules R1–R7)
- `governed_swarm.py` – Governed swarm orchestrator
- `swarm_core.py` – Lightweight mining swarm reference
- `mining_scenario.py` – Example instantiation of the mining swarm

## Memoryboard + RAG adapter

The swarm can persist and recall governed memory through the Jarvis Memoryboard
(EMR continuity ledger) and its AMUL RAG knowledge base via
`memoryboard_adapter.py`. The adapter is a thin, fail-closed HTTP client:

- `remember(...)` — persist a swarm event as a governed **draft** memory (EMR
  write gateway; autonomous `user_requested=False` writes are refused by design).
- `recall(...)` / `retrieve(...)` — EMR governed retrieval into swarm context.
- `rag_query(...)` — query the AMUL RAG knowledge base for governed facts.
- Offline by default (`offline_ok=True`): degrades to offline markers instead of
  crashing the swarm. Set `JARVIS_MEMORYBOARD_URL` when a memoryboard is running.
- Hot-swappable: `attach(url)` / `detach()` dock or undock the board at runtime.
  The swarm keeps ticking. A new URL mints a new `session_id` so two ledgers
  are never merged. Undocked recall returns `_undocked` (not an empty memory
  list). Dock/undock events are stamped on `adapter.dock_log`.

A successful draft write proves persistence, provenance, evidence, and its
content hash. It does **not** by itself prove STM-to-LTM promotion or continuity
consolidation. When the promotion decision abstains, report the result as
"draft persisted, promotion correctly abstained."

```bash
# Run the governed swarm + memoryboard adapter demo
python governed_swarm_memory_demo.py

# Run adapter tests (no live memoryboard required — uses a local stub server)
python -m pytest test_memoryboard_adapter.py -q
```

Project structure additions:
- `memoryboard_adapter.py` – Jarvis Memoryboard (EMR) + RAG integration adapter
- `governed_swarm_memory_demo.py` – Governed swarm wired to the memoryboard
- `test_memoryboard_adapter.py` – Adapter tests (stub HTTP server)

## OmniSim seam

`omnisim_seam` is a bounded mobile-robot adapter, not a full navigation-stack
integration. Its shipped example defaults target the OmniSim Husky world:
`husky_ne:8865`, `husky_nw:8866`, `husky_se:8867`, and `husky_sw:8868`.
Endpoints are configurable, for example:

```bash
python3 omnisim_seam/__init__.py --host 127.0.0.1 \
  --robot-endpoint robot_a:8765 --robot-endpoint robot_b:8766
```

Completion is recorded only when OmniSim reports `arrived=true`,
`settled=true`, and `timed_out=false`. The HTTP budget is derived from
the **observed starting pose** (configured spawn is fallback only); use
`--timeout-s`, `--cruise-speed-mps`, and `--settle-timeout-s` for a
particular bridge. Adapter-side notes, defensive checks, and the
completion-gate contract: [`omnisim_seam/ADAPTER.md`](omnisim_seam/ADAPTER.md).

Offline rerun (no OmniSim required):

```bash
./omnisim_seam/rerun.sh
```

## Quick Start
```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/governed-optimus-swarm.git
cd governed-optimus-swarm

# Run a simple mining swarm demo
python -c "
from mining_scenario import create_mining_swarm
swarm = create_mining_swarm()
for _ in range(20):
    swarm.step()
print('Mining simulation complete.')
print(f'Resources remaining: {[r.remaining for r in swarm.model.resources]}')
"
