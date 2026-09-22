# NEXFLEET EdgeFleet-AI

Federated Graph-Risk Coordination with Multi-Agent Safety for Decentralized Autonomous Mobile Robot Fleets.

## Project Structure
- `backend/` - FastAPI server, Discrete-Tick Simulation, AI (GNN, MARL, FL), Coordination
- `frontend/` - React + Vite Real-time Dashboard
- `algorithms/` - A*, LaCAM+PIBT adapter
- `ros2_ws/` - Custom ROS 2 Packages (nexfleet_msgs, amr, coordination)
- `scenarios/` - Benchmark scenario JSON files
- `scripts/` - Helper scripts for training and building
- `docs/` - Architecture documentation

## Getting Started
1. **Start Backend**: `python -m backend`
2. **Start Frontend**: `cd frontend && npm run dev`
3. **Train AI (Background)**: Trigger from Dashboard -> AI / FL tabs.
4. **Build ROS 2**: `./scripts/build_ros2.sh` (requires ROS 2 Humble/Iron setup)
