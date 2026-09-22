"""Pydantic schemas for NEXFLEET API."""
from __future__ import annotations
from enum import Enum
from typing import Any, Dict, List, Optional
from pydantic import BaseModel


class SimModeEnum(str, Enum):
    traditional = "traditional"
    nexfleet = "nexfleet"


class NetworkProfileEnum(str, Enum):
    NORMAL = "NORMAL"
    HIGH_LATENCY = "HIGH_LATENCY"
    PACKET_LOSS = "PACKET_LOSS"
    DEAD_ZONE = "DEAD_ZONE"


class StartSimRequest(BaseModel):
    mode: SimModeEnum = SimModeEnum.nexfleet
    seed: int = 42
    tick_interval: float = 0.2
    max_ticks: int = 1000


class CreateTaskRequest(BaseModel):
    pickup_station: str
    delivery_station: str
    priority: int = 1


class AddObstacleRequest(BaseModel):
    row: int
    col: int


class RemoveObstacleRequest(BaseModel):
    row: int
    col: int


class NetworkConfigRequest(BaseModel):
    profile: NetworkProfileEnum = NetworkProfileEnum.NORMAL
    latency_ms: float = 0.0
    packet_loss_rate: float = 0.0


class ComparisonRequest(BaseModel):
    scenario: str = "baseline"
    seed: int = 42
    max_ticks: int = 800
    n_tasks: int = 10


class TrainGNNRequest(BaseModel):
    episodes: int = 20
    epochs: int = 20
    seed: int = 42


class TrainMARLRequest(BaseModel):
    episodes: int = 30
    seed: int = 42


class FederatedTrainRequest(BaseModel):
    rounds: int = 5
    episodes: int = 15
    seed: int = 99
