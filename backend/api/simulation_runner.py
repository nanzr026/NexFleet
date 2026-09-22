"""Dual Simulation Runner — manages side-by-side comparison."""
from __future__ import annotations
import asyncio
import time
from typing import Any, Callable, Dict, List, Optional

from backend.simulation.engine import SimulationEngine, SimulationMode, SimulationState
from backend.communication.inprocess import InProcessTransport


class ComparisonRunner:
    """Runs Traditional and NexFleet engines side-by-side."""

    def __init__(self):
        self.trad_engine: Optional[SimulationEngine] = None
        self.nex_engine: Optional[SimulationEngine] = None
        self.transport = InProcessTransport()
        self._tick_task: Optional[asyncio.Task] = None
        self._ws_callbacks: List[Callable] = []
        self._running = False
        self._tick_interval = 0.15
        
        self.task_queue: List[dict] = []
        self.obstacle_queue: List[list] = []
        
        self._ai_status = {
            "risk_engine": "HEURISTIC_FALLBACK",
            "decision_engine": "RULE_BASED_FALLBACK",
        }
        self._gnn_engine = None
        self._marl_engine = None
        self._init_ai()
        self.reset()

    def _init_ai(self):
        try:
            from backend.ai.gnn.inference import GNNInferenceEngine
            self._gnn_engine = GNNInferenceEngine()
            self._ai_status["risk_engine"] = self._gnn_engine.risk_engine
        except Exception: pass

        try:
            from backend.ai.marl.inference import MARLInferenceEngine
            self._marl_engine = MARLInferenceEngine()
            self._ai_status["decision_engine"] = self._marl_engine.decision_engine
        except Exception: pass

    def reset(self, seed: int = 42):
        self.trad_engine = SimulationEngine(mode=SimulationMode.TRADITIONAL, seed=seed)
        self.nex_engine = SimulationEngine(mode=SimulationMode.NEXFLEET, seed=seed)
        self.task_queue = []
        self.obstacle_queue = []
        
        # Wire AI to NexFleet engine
        if hasattr(self.nex_engine.controller, "set_risk_fn") and self._gnn_engine:
            self.nex_engine.controller.set_risk_fn(self._gnn_engine.compute_risk, self._gnn_engine.risk_engine)
        if hasattr(self.nex_engine.controller, "set_action_fn") and self._marl_engine:
            self.nex_engine.controller.set_action_fn(self._marl_engine.get_action, self._marl_engine.decision_engine)

    def queue_task(self, pickup: str, delivery: str, priority: int = 1):
        self.task_queue.append({"pickup": pickup, "delivery": delivery, "priority": priority})
        if self._running:
            self.trad_engine.add_task_from_stations(pickup, delivery, priority)
            self.nex_engine.add_task_from_stations(pickup, delivery, priority)

    def add_obstacle(self, row: int, col: int):
        self.obstacle_queue.append([row, col])
        if self.trad_engine:
            self.trad_engine.add_obstacle(row, col)
        if self.nex_engine:
            self.nex_engine.add_obstacle(row, col)

    def add_ws_callback(self, cb: Callable):
        self._ws_callbacks.append(cb)

    async def start(self):
        if self._tick_task and not self._tick_task.done():
            self._tick_task.cancel()
        
        # Re-initialize engines with exact queues
        self.trad_engine = SimulationEngine(mode=SimulationMode.TRADITIONAL, seed=42)
        self.nex_engine = SimulationEngine(mode=SimulationMode.NEXFLEET, seed=42)
        
        if hasattr(self.nex_engine.controller, "set_risk_fn") and self._gnn_engine:
            self.nex_engine.controller.set_risk_fn(self._gnn_engine.compute_risk, self._gnn_engine.risk_engine)
        if hasattr(self.nex_engine.controller, "set_action_fn") and self._marl_engine:
            self.nex_engine.controller.set_action_fn(self._marl_engine.get_action, self._marl_engine.decision_engine)

        for obs in self.obstacle_queue:
            self.trad_engine.add_obstacle(obs[0], obs[1])
            self.nex_engine.add_obstacle(obs[0], obs[1])
            
        for t in self.task_queue:
            self.trad_engine.add_task_from_stations(t['pickup'], t['delivery'], t['priority'])
            self.nex_engine.add_task_from_stations(t['pickup'], t['delivery'], t['priority'])

        self.trad_engine.start()
        self.nex_engine.start()
        
        self._running = True
        self._tick_task = asyncio.create_task(self._tick_loop())

    async def stop(self):
        self._running = False
        if self.trad_engine: self.trad_engine.pause()
        if self.nex_engine: self.nex_engine.pause()
        if self._tick_task: self._tick_task.cancel()

    async def _tick_loop(self):
        while self._running:
            active = False
            if self.trad_engine and self.trad_engine.state == SimulationState.RUNNING:
                self.trad_engine.step()
                active = True
            if self.nex_engine and self.nex_engine.state == SimulationState.RUNNING:
                self.nex_engine.step()
                active = True

            state = self.get_dual_state()
            for cb in list(self._ws_callbacks):
                try: await cb({"type": "STATE_UPDATE", "data": state})
                except Exception: pass

            if not active:
                break
            await asyncio.sleep(self._tick_interval)

    def get_dual_state(self) -> dict:
        trad_state = self.trad_engine.get_full_state() if self.trad_engine else {}
        nex_state = self.nex_engine.get_full_state() if self.nex_engine else {}
        
        trad_ms = trad_state.get('metrics', {}).get('makespan_ticks', 0) or 0
        nex_ms = nex_state.get('metrics', {}).get('makespan_ticks', 0) or 0
        
        reduction = 0.0
        if trad_ms > 0 and nex_ms > 0 and nex_ms < trad_ms:
            reduction = ((trad_ms - nex_ms) / trad_ms) * 100.0
        elif trad_ms > 0 and nex_ms > trad_ms:
            reduction = ((trad_ms - nex_ms) / trad_ms) * 100.0 # Negative
            
        target_achieved = (reduction >= 20.0 and nex_state.get('metrics', {}).get('collisions', 0) == 0)

        return {
            "traditional": trad_state,
            "nexfleet": nex_state,
            "comparison": {
                "completion_time_reduction_pct": round(reduction, 1),
                "target_achieved": target_achieved,
                "queued_tasks": len(self.task_queue),
                "is_running": self._running
            },
            "ai_status": self._ai_status
        }

runner = ComparisonRunner()
