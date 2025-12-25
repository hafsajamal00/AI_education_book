"""
Voice Command Processing API
Implements the POST /voice-commands/process endpoint
"""
import asyncio
import base64
import json
from typing import Dict, Any
from fastapi import FastAPI, HTTPException, BackgroundTasks, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
import openai
import os
import uuid
from datetime import datetime

# Initialize FastAPI app
app = FastAPI(title="VLA Voice Command API", version="1.0.0")

# Set OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

# Pydantic models for request/response
class VoiceCommandRequest(BaseModel):
    audio_data: str  # base64 encoded audio
    student_id: str
    simulation_environment_id: str

class VoiceCommandResponse(BaseModel):
    command_id: str
    recognized_text: str
    confidence_score: float
    action_sequence: Dict[str, Any]
    safety_constraints_applied: list

# In-memory storage for demonstration purposes
# In production, use a proper database
command_history = {}

# WebSocket connection manager for real-time feedback
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                self.disconnect(connection)

manager = ConnectionManager()

@app.post("/voice-commands/process", response_model=VoiceCommandResponse)
async def process_voice_command(request: VoiceCommandRequest, background_tasks: BackgroundTasks):
    """
    Submit a voice command for processing and robot execution
    """
    # Validate request
    validation_errors = validate_voice_command_request(request)
    if validation_errors:
        raise HTTPException(status_code=400, detail=f"Invalid request: {', '.join(validation_errors)}")

    try:
        # Decode base64 audio data
        try:
            audio_bytes = base64.b64decode(request.audio_data)
        except Exception:
            raise HTTPException(status_code=400, detail="Invalid audio data: unable to decode base64")

        # Validate audio data length (prevent extremely large files)
        if len(audio_bytes) > 10 * 1024 * 1024:  # 10MB limit
            raise HTTPException(status_code=400, detail="Audio data too large (max 10MB)")

        # Save to temporary file for Whisper processing
        temp_filename = f"temp_audio_{uuid.uuid4().hex}.wav"
        with open(temp_filename, "wb") as audio_file:
            audio_file.write(audio_bytes)

        try:
            # Transcribe audio using OpenAI Whisper
            with open(temp_filename, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)

            recognized_text = transcript.text.strip()

            # Validate recognized text
            if not recognized_text:
                raise HTTPException(status_code=422, detail="No speech detected in audio")

            # Basic confidence estimation (in a real system, this would come from Whisper)
            confidence_score = calculate_confidence_score(recognized_text, request.audio_data)

            # Generate a unique command ID
            command_id = str(uuid.uuid4())

            # Map recognized text to robot actions
            action_sequence = map_voice_to_actions(recognized_text)

            # Apply safety constraints
            safety_constraints = apply_safety_constraints(action_sequence)

            # Validate action sequence
            if not action_sequence.get("sequence"):
                raise HTTPException(status_code=422, detail="No valid robot actions could be derived from the command")

            # Create response
            response = VoiceCommandResponse(
                command_id=command_id,
                recognized_text=recognized_text,
                confidence_score=confidence_score,
                action_sequence=action_sequence,
                safety_constraints_applied=safety_constraints
            )

            # Store command in history
            command_history[command_id] = {
                "request": request.dict(),
                "response": response.dict(),
                "timestamp": datetime.utcnow().isoformat()
            }

            # Schedule background task to execute the action sequence
            background_tasks.add_task(execute_action_sequence, action_sequence, request.simulation_environment_id)

            return response

        finally:
            # Clean up temporary file
            import os
            if os.path.exists(temp_filename):
                os.remove(temp_filename)

    except openai.error.AuthenticationError:
        raise HTTPException(status_code=401, detail="Invalid OpenAI API key")
    except openai.error.RateLimitError:
        raise HTTPException(status_code=429, detail="OpenAI API rate limit exceeded")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing voice command: {str(e)}")

def validate_voice_command_request(request: VoiceCommandRequest) -> list:
    """
    Validate the voice command request
    Returns a list of validation errors, or empty list if valid
    """
    errors = []

    if not request.student_id or not request.student_id.strip():
        errors.append("student_id is required")

    if not request.simulation_environment_id or not request.simulation_environment_id.strip():
        errors.append("simulation_environment_id is required")

    if not request.audio_data:
        errors.append("audio_data is required")

    # Check if audio data is valid base64 (basic check)
    if request.audio_data:
        try:
            base64.b64decode(request.audio_data)
        except Exception:
            errors.append("audio_data is not valid base64")

    return errors

def calculate_confidence_score(text: str, audio_data: str) -> float:
    """
    Calculate a basic confidence score based on text quality
    In a real system, this would use Whisper's confidence scores
    """
    # Simple heuristic for confidence calculation
    score = 0.8  # Base score

    # Increase confidence for longer, more specific commands
    if len(text.split()) >= 2:
        score += 0.1

    # Decrease confidence for very short or ambiguous commands
    if len(text.strip()) < 3:
        score -= 0.2

    # Ensure score is within [0, 1] range
    return max(0.0, min(1.0, score))

def map_voice_to_actions(text: str) -> Dict[str, Any]:
    """
    Map recognized voice commands to robot action sequences
    """
    # Define command patterns
    command_patterns = {
        'move_forward': ['move forward', 'go forward', 'move ahead', 'go ahead'],
        'move_backward': ['move backward', 'go backward', 'move back', 'go back'],
        'turn_left': ['turn left', 'rotate left', 'pivot left'],
        'turn_right': ['turn right', 'rotate right', 'pivot right'],
        'pick_up': ['pick up', 'grab', 'take'],
        'place': ['place', 'put down', 'set down']
    }
    
    text_lower = text.lower().strip()
    action_sequence = {
        "id": str(uuid.uuid4()),
        "sequence": [],
        "status": "pending",
        "created_at": datetime.utcnow().isoformat()
    }
    
    # Find matching commands
    for action, patterns in command_patterns.items():
        for pattern in patterns:
            if pattern in text_lower:
                # Extract parameters if needed
                remaining_text = text_lower.replace(pattern, '').strip()
                params = {}
                if action in ['pick_up']:
                    params['object'] = remaining_text
                
                action_item = {
                    "id": str(uuid.uuid4()),
                    "type": action,
                    "parameters": params,
                    "order": len(action_sequence["sequence"])
                }
                
                action_sequence["sequence"].append(action_item)
                
                # Only add the first matching command for this example
                break
        if action_sequence["sequence"]:  # If we found a match, break
            break
    
    # If no commands were recognized, add an unknown command
    if not action_sequence["sequence"]:
        action_sequence["sequence"].append({
            "id": str(uuid.uuid4()),
            "type": "unknown",
            "parameters": {"text": text},
            "order": 0
        })
    
    return action_sequence

def apply_safety_constraints(action_sequence: Dict[str, Any]) -> list:
    """
    Apply safety constraints to the action sequence
    """
    constraints = []

    for action in action_sequence.get("sequence", []):
        # Example safety checks
        if action["type"] == "move_forward":
            # Check if moving forward is safe in the current environment
            constraints.extend([
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "collision_avoidance",
                    "condition": "check_for_obstacles_ahead",
                    "severity": "warning"
                },
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "speed_limit",
                    "condition": "limit_forward_speed_to_safe_value",
                    "severity": "warning"
                }
            ])
        elif action["type"] == "move_backward":
            constraints.extend([
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "collision_avoidance",
                    "condition": "check_for_obstacles_behind",
                    "severity": "warning"
                },
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "speed_limit",
                    "condition": "limit_backward_speed_to_safe_value",
                    "severity": "warning"
                }
            ])
        elif action["type"] in ["turn_left", "turn_right"]:
            constraints.append({
                "id": str(uuid.uuid4()),
                "constraint_type": "environment_boundary",
                "condition": "ensure_turn_keeps_robot_in_safe_area",
                "severity": "warning"
            })
        elif action["type"] == "pick_up":
            constraints.extend([
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "object_validation",
                    "condition": f"verify_object_exists_and_is_grabbable: {action['parameters'].get('object', 'unknown')}",
                    "severity": "error"
                },
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "robot_state_check",
                    "condition": "ensure_robot_is_stable_before_manipulation",
                    "severity": "error"
                }
            ])
        elif action["type"] == "place":
            constraints.extend([
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "workspace_check",
                    "condition": "verify_placement_location_is_safe_and_valid",
                    "severity": "error"
                },
                {
                    "id": str(uuid.uuid4()),
                    "constraint_type": "robot_state_check",
                    "condition": "ensure_robot_is_holding_an_object_before_placement",
                    "severity": "error"
                }
            ])
        elif action["type"] == "unknown":
            constraints.append({
                "id": str(uuid.uuid4()),
                "constraint_type": "command_validation",
                "condition": "unknown_command",
                "severity": "error"
            })

    return constraints


def validate_action_sequence_safety(action_sequence: Dict[str, Any], simulation_env_id: str) -> tuple[bool, list]:
    """
    Validate if an action sequence is safe to execute in the given simulation environment
    Returns (is_safe, list_of_violations)
    """
    safety_constraints = apply_safety_constraints(action_sequence)
    violations = []

    for constraint in safety_constraints:
        # In a real system, these checks would query the simulation environment
        # For this example, we'll just flag high-severity constraints as potential violations
        if constraint["severity"] == "error":
            violations.append({
                "constraint_id": constraint["id"],
                "message": f"Potential safety violation for constraint: {constraint['condition']}",
                "action_required": "review_and_approve_manually" if constraint["severity"] == "error" else "log_for_review"
            })

    is_safe = len(violations) == 0
    return is_safe, violations

def execute_action_sequence(action_sequence: Dict[str, Any], simulation_env_id: str):
    """
    Execute the action sequence in the simulation environment
    This would typically communicate with ROS 2 via rosbridge
    """
    # Import here to avoid circular dependencies
    from action_executor import execute_action_sequence_from_api

    # In a real implementation, this would send commands to the simulation
    # via ROS 2 or a similar mechanism
    print(f"Executing action sequence {action_sequence['id']} in simulation {simulation_env_id}")

    # Execute the action sequence using the ROS 2 action executor
    execute_action_sequence_from_api(action_sequence, simulation_env_id)

@app.websocket("/ws/voice-status")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time voice command status updates
    """
    await manager.connect(websocket)
    try:
        while True:
            # Keep the connection alive
            data = await websocket.receive_text()
            # Optionally handle messages from client
            await manager.send_personal_message(f"Received: {data}", websocket)
    except WebSocketDisconnect:
        manager.disconnect(websocket)

async def broadcast_status_update(command_id: str, status: str, message: str):
    """
    Broadcast status updates to all connected WebSocket clients
    """
    from datetime import datetime

    status_update = {
        "command_id": command_id,
        "status": status,
        "message": message,
        "timestamp": datetime.utcnow().isoformat()
    }

    await manager.broadcast(json.dumps(status_update))

def execute_action_sequence(action_sequence: Dict[str, Any], simulation_env_id: str):
    """
    Execute the action sequence in the simulation environment
    This would typically communicate with ROS 2 via rosbridge
    """
    # Import here to avoid circular dependencies
    from action_executor import execute_action_sequence_from_api

    # First, validate safety constraints
    is_safe, violations = validate_action_sequence_safety(action_sequence, simulation_env_id)

    if not is_safe:
        # If there are safety violations, broadcast a warning but still proceed
        # (In a real system, you might want to stop execution based on violation severity)
        violation_msg = f"Safety violations detected: {len(violations)} issues found. Proceeding with caution."
        asyncio.create_task(
            broadcast_status_update(
                action_sequence['id'],
                "warning",
                violation_msg
            )
        )
        print(f"Safety warning for sequence {action_sequence['id']}: {violation_msg}")

    # Broadcast initial execution status
    asyncio.create_task(
        broadcast_status_update(
            action_sequence['id'],
            "executing",
            f"Starting execution of sequence in simulation {simulation_env_id}"
        )
    )

    try:
        # Execute the action sequence using the ROS 2 action executor
        execute_action_sequence_from_api(action_sequence, simulation_env_id)

        # Broadcast completion status
        asyncio.create_task(
            broadcast_status_update(
                action_sequence['id'],
                "completed",
                f"Successfully completed execution of sequence in simulation {simulation_env_id}"
            )
        )
    except Exception as e:
        # Broadcast error status
        asyncio.create_task(
            broadcast_status_update(
                action_sequence['id'],
                "failed",
                f"Failed to execute sequence: {str(e)}"
            )
        )

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "timestamp": datetime.utcnow().isoformat()}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)