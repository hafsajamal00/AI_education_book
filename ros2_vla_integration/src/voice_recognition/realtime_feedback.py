"""
Real-time Feedback System for Voice Command Status
Implements WebSocket communication for real-time status updates
"""
import asyncio
import json
from typing import Dict, Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

class StatusUpdate(BaseModel):
    command_id: str
    status: str  # e.g., "processing", "executing", "completed", "failed"
    message: str
    timestamp: str

class ConnectionManager:
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.add(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.discard(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                # Remove connection if sending fails
                self.disconnect(connection)

manager = ConnectionManager()

# Dictionary to track command statuses
command_statuses: Dict[str, StatusUpdate] = {}

async def update_command_status(command_id: str, status: str, message: str):
    """
    Update the status of a command and broadcast to all connected clients
    """
    from datetime import datetime
    
    status_update = StatusUpdate(
        command_id=command_id,
        status=status,
        message=message,
        timestamp=datetime.utcnow().isoformat()
    )
    
    command_statuses[command_id] = status_update
    
    # Broadcast the update to all connected clients
    await manager.broadcast(json.dumps(status_update.dict()))

async def get_command_status(command_id: str) -> StatusUpdate:
    """
    Get the current status of a command
    """
    return command_statuses.get(command_id)