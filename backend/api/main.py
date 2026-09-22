"""NEXFLEET FastAPI Backend — main application."""
from __future__ import annotations
import asyncio
import json
import os
import random
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware

from backend.api.schemas import (
    StartSimRequest, CreateTaskRequest, AddObstacleRequest,
    RemoveObstacleRequest, NetworkConfigRequest, ComparisonRequest,
    TrainGNNRequest, TrainMARLRequest, FederatedTrainRequest,
)
from backend.api.simulation_runner import runner

app = FastAPI(title="NEXFLEET EdgeFleet-AI")

app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_credentials=True,
    allow_methods=["*"], allow_headers=["*"],
)

class ConnectionManager:
    def __init__(self):
        self.active = []
    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)
    def disconnect(self, ws: WebSocket):
        if ws in self.active: self.active.remove(ws)
    async def broadcast(self, data: dict):
        for ws in list(self.active):
            try: await ws.send_json(data)
            except: self.disconnect(ws)

manager = ConnectionManager()

@app.on_event("startup")
async def startup():
    runner.add_ws_callback(manager.broadcast)
    for d in ["experiments/results", "models", "data/gnn", "scenarios"]:
        Path(d).mkdir(parents=True, exist_ok=True)

@app.get("/api/health")
async def health():
    return {"status": "ok", "service": "NEXFLEET", "transport": runner.transport.get_transport_name()}

@app.get("/api/state")
async def get_state():
    return runner.get_dual_state()

@app.get("/api/ai/status")
async def get_ai_status():
    return runner._ai_status

@app.post("/api/simulation/start")
async def start_simulation():
    await runner.start()
    return {"status": "started"}

@app.post("/api/simulation/pause")
async def pause_simulation():
    await runner.stop()
    return {"status": "paused"}

@app.post("/api/simulation/reset")
async def reset_simulation():
    await runner.stop()
    runner.reset()
    return {"status": "reset"}

@app.post("/api/tasks")
async def create_task(req: CreateTaskRequest):
    runner.queue_task(req.pickup_station, req.delivery_station, req.priority)
    return {"status": "queued"}

@app.post("/api/obstacles")
async def add_obstacle(req: AddObstacleRequest):
    runner.add_obstacle(req.row, req.col)
    return {"status": "added"}

@app.post("/api/robots/{robot_id}/fail")
async def fail_robot(robot_id: str):
    if runner.trad_engine: runner.trad_engine.fail_robot(robot_id)
    if runner.nex_engine: runner.nex_engine.fail_robot(robot_id)
    return {"status": "failed"}

@app.post("/api/robots/{robot_id}/recover")
async def recover_robot(robot_id: str):
    if runner.trad_engine: runner.trad_engine.recover_robot(robot_id)
    if runner.nex_engine: runner.nex_engine.recover_robot(robot_id)
    return {"status": "recovered"}

@app.websocket("/ws/fleet")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        await websocket.send_json({"type": "STATE_UPDATE", "data": runner.get_dual_state()})
        while True:
            await asyncio.wait_for(websocket.receive_json(), timeout=30.0)
    except Exception:
        pass
    finally:
        manager.disconnect(websocket)
