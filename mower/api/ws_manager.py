"""WebSocket connection manager — broadcast messages to all connected clients."""
import json
import logging
from typing import Any

from fastapi import WebSocket

logger = logging.getLogger(__name__)


class ConnectionManager:
    def __init__(self) -> None:
        self._connections: list[WebSocket] = []

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._connections.append(ws)
        logger.info("WebSocket connected (%d total)", len(self._connections))

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self._connections:
            self._connections.remove(ws)
            logger.info("WebSocket disconnected (%d remaining)", len(self._connections))

    async def broadcast(self, payload: dict[str, Any]) -> None:
        """Send JSON payload to all connected clients; remove broken connections."""
        message = json.dumps(payload)
        dead: list[WebSocket] = []
        for ws in list(self._connections):
            try:
                await ws.send_text(message)
            except Exception as exc:
                logger.warning("WebSocket send failed: %s", exc)
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)
