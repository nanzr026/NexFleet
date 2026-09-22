"""Transport abstraction layer for NEXFLEET."""
from __future__ import annotations
import asyncio
import json
import random
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional


class NetworkProfile(str, Enum):
    NORMAL = "NORMAL"
    HIGH_LATENCY = "HIGH_LATENCY"
    PACKET_LOSS = "PACKET_LOSS"
    DEAD_ZONE = "DEAD_ZONE"


@dataclass
class NetworkConfig:
    profile: NetworkProfile = NetworkProfile.NORMAL
    latency_ms: float = 0.0       # added per-message latency
    packet_loss_rate: float = 0.0  # 0.0 to 1.0
    peer_disconnected: bool = False

    @classmethod
    def from_profile(cls, profile: NetworkProfile) -> "NetworkConfig":
        presets = {
            NetworkProfile.NORMAL: cls(profile, 0.0, 0.0, False),
            NetworkProfile.HIGH_LATENCY: cls(profile, 150.0, 0.02, False),
            NetworkProfile.PACKET_LOSS: cls(profile, 10.0, 0.15, False),
            NetworkProfile.DEAD_ZONE: cls(profile, 500.0, 0.8, False),
        }
        return presets[profile]


@dataclass
class Message:
    topic: str
    sender: str
    payload: Any
    timestamp: float = field(default_factory=time.time)
    message_id: str = ""

    def __post_init__(self):
        if not self.message_id:
            import uuid
            self.message_id = str(uuid.uuid4())[:8]


class Transport(ABC):
    """Abstract transport — same interface for InProcess and ROS2/DDS."""

    @abstractmethod
    async def publish(self, topic: str, sender: str, payload: Any) -> bool:
        """Publish a message to a topic. Returns True if delivered."""
        ...

    @abstractmethod
    def subscribe(self, topic: str, callback: Callable[[Message], None]):
        """Subscribe to a topic with a callback."""
        ...

    @abstractmethod
    async def broadcast(self, sender: str, payload: Any):
        """Broadcast to all robots on fleet-wide topic."""
        ...

    @abstractmethod
    def get_transport_name(self) -> str:
        ...

    @abstractmethod
    def get_message_count(self) -> int:
        ...
