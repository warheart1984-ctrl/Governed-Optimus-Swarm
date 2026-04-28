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
