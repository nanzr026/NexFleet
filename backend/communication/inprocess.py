"""In-process transport — no ROS2 required."""
from __future__ import annotations
import asyncio
import random
import time
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

from backend.communication.transport import (
    Message, NetworkConfig, NetworkProfile, Transport
)


class InProcessTransport(Transport):
    """
    In-memory pub/sub transport for single-process simulation.
    Supports network impairment simulation.
    """

    BROADCAST_TOPIC = "nexfleet/fleet/broadcast"

    def __init__(self, network_config: Optional[NetworkConfig] = None):
        self._subscribers: Dict[str, List[Callable[[Message], None]]] = defaultdict(list)
        self._message_count: int = 0
        self._dropped_count: int = 0
        self._latency_total_ms: float = 0.0
        self.network_config: NetworkConfig = network_config or NetworkConfig()
        self._peer_stats: Dict[str, Dict] = {}  # sender -> stats

    def subscribe(self, topic: str, callback: Callable[[Message], None]):
        self._subscribers[topic].append(callback)

    async def publish(self, topic: str, sender: str, payload: Any) -> bool:
        cfg = self.network_config

        # Simulate disconnection
        if cfg.peer_disconnected:
            self._dropped_count += 1
            return False

        # Simulate packet loss
        if cfg.packet_loss_rate > 0 and random.random() < cfg.packet_loss_rate:
            self._dropped_count += 1
            return False

        msg = Message(topic=topic, sender=sender, payload=payload)
        self._message_count += 1

        # Track per-sender stats
        if sender not in self._peer_stats:
            self._peer_stats[sender] = {"sent": 0, "dropped": 0, "last_seen": 0.0}
        self._peer_stats[sender]["sent"] += 1
        self._peer_stats[sender]["last_seen"] = time.time()

        # Simulate latency
        if cfg.latency_ms > 0:
            await asyncio.sleep(cfg.latency_ms / 1000.0)
            self._latency_total_ms += cfg.latency_ms

        # Deliver to subscribers
        for cb in self._subscribers.get(topic, []):
            try:
                cb(msg)
            except Exception as e:
                print(f"[Transport] Callback error on {topic}: {e}")

        return True

    async def broadcast(self, sender: str, payload: Any):
        await self.publish(self.BROADCAST_TOPIC, sender, payload)

    def get_transport_name(self) -> str:
        return "IN-PROCESS"

    def get_message_count(self) -> int:
        return self._message_count

    def get_dropped_count(self) -> int:
        return self._dropped_count

    def set_network_config(self, config: NetworkConfig):
        self.network_config = config

    def get_peer_stats(self) -> Dict[str, Dict]:
        return dict(self._peer_stats)

    def get_stats(self) -> dict:
        return {
            "transport": "IN-PROCESS",
            "messages_sent": self._message_count,
            "messages_dropped": self._dropped_count,
            "network_profile": self.network_config.profile.value,
            "latency_ms": self.network_config.latency_ms,
            "packet_loss_rate": self.network_config.packet_loss_rate,
        }
