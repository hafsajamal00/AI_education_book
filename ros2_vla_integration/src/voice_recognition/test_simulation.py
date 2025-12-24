"""
Test Simulation for Voice Command Validation
This script provides a simple simulation environment to test voice commands
"""
import asyncio
import base64
import json
from datetime import datetime
from pydantic import BaseModel
from typing import Dict, Any
import os
import sys

# Add the voice recognition module to the path
sys.path.append(os.path.join(os.path.dirname(__file__)))

from voice_command_api import process_voice_command, VoiceCommandRequest
from realtime_feedback import update_command_status


class MockSimulationEnvironment:
    """
    A mock simulation environment for testing voice commands
    """
    def __init__(self):
        self.robot_position = {"x": 0, "y": 0, "theta": 0}  # x, y, orientation
        self.robot_state = "idle"
        self.objects = [
            {"name": "red_cube", "position": {"x": 1, "y": 1}, "picked_up": False},
            {"name": "blue_sphere", "position": {"x": -1, "y": 2}, "picked_up": False}
        ]
        self.command_history = []
    
    def execute_command(self, action_sequence: Dict[str, Any]):
        """
        Execute a command sequence in the mock simulation
        """
        print(f"Executing action sequence: {action_sequence['id']}")
        
        for action in sorted(action_sequence.get("sequence", []), key=lambda x: x["order"]):
            action_type = action["type"]
            params = action["parameters"]
            
            print(f"  Executing: {action_type} with params: {params}")
            
            if action_type == "move_forward":
                self.move_forward()
            elif action_type == "move_backward":
                self.move_backward()
            elif action_type == "turn_left":
                self.turn_left()
            elif action_type == "turn_right":
                self.turn_right()
            elif action_type == "pick_up":
                obj_name = params.get("object", "")
                self.pick_up_object(obj_name)
            elif action_type == "place":
                self.place_object()
            elif action_type == "unknown":
                print(f"    Unknown command: {params.get('text', 'no text')}")
            else:
                print(f"    Unsupported action: {action_type}")
    
    def move_forward(self):
        """Move the robot forward"""
        print("    Moving forward...")
        self.robot_position["x"] += 0.5  # Move 0.5 units forward
        self.robot_state = "moving"
        print(f"    New position: {self.robot_position}")
        self.robot_state = "idle"
    
    def move_backward(self):
        """Move the robot backward"""
        print("    Moving backward...")
        self.robot_position["x"] -= 0.5  # Move 0.5 units backward
        self.robot_state = "moving"
        print(f"    New position: {self.robot_position}")
        self.robot_state = "idle"
    
    def turn_left(self):
        """Turn the robot left"""
        print("    Turning left...")
        self.robot_position["theta"] += 1.57  # Turn 90 degrees (π/2 radians)
        self.robot_state = "turning"
        print(f"    New orientation: {self.robot_position['theta']}")
        self.robot_state = "idle"
    
    def turn_right(self):
        """Turn the robot right"""
        print("    Turning right...")
        self.robot_position["theta"] -= 1.57  # Turn 90 degrees (π/2 radians)
        self.robot_state = "turning"
        print(f"    New orientation: {self.robot_position['theta']}")
        self.robot_state = "idle"
    
    def pick_up_object(self, object_name: str):
        """Pick up an object if it's nearby"""
        print(f"    Attempting to pick up: {object_name}")
        
        for obj in self.objects:
            if obj["name"] == object_name and not obj["picked_up"]:
                # Check if robot is close enough (simplified check)
                if abs(self.robot_position["x"] - obj["position"]["x"]) < 1.0 and \
                   abs(self.robot_position["y"] - obj["position"]["y"]) < 1.0:
                    obj["picked_up"] = True
                    print(f"    Picked up {object_name}")
                    return
                else:
                    print(f"    {object_name} is not nearby")
                    return
        
        print(f"    Object {object_name} not found or already picked up")
    
    def place_object(self):
        """Place the currently held object"""
        print("    Placing object...")
        # In a real implementation, we would place the object at the current position
        print("    Object placed at current position")


async def simulate_voice_command(text_command: str, student_id: str = "test_student"):
    """
    Simulate processing a voice command through the API
    """
    print(f"\n--- Simulating voice command: '{text_command}' ---")
    
    # Create a mock audio data (in a real scenario, this would be actual audio)
    # For testing, we'll use a placeholder base64 string
    mock_audio_data = base64.b64encode(b"mock audio data").decode('utf-8')
    
    # Create the request object
    request = VoiceCommandRequest(
        audio_data=mock_audio_data,
        student_id=student_id,
        simulation_environment_id="test_sim_env"
    )
    
    # Mock the transcription process by directly using the text command
    # In a real scenario, this would happen inside the API
    from voice_command_api import map_voice_to_actions, apply_safety_constraints
    import uuid
    
    # Map the text command to actions
    action_sequence = map_voice_to_actions(text_command)
    safety_constraints = apply_safety_constraints(action_sequence)
    
    print(f"Mapped to action sequence: {action_sequence}")
    print(f"Applied safety constraints: {safety_constraints}")
    
    # Execute in mock simulation
    sim_env = MockSimulationEnvironment()
    sim_env.execute_command(action_sequence)
    
    # Return mock response
    response = {
        "command_id": str(uuid.uuid4()),
        "recognized_text": text_command,
        "confidence_score": 0.9,
        "action_sequence": action_sequence,
        "safety_constraints_applied": safety_constraints
    }
    
    return response


async def run_test_suite():
    """
    Run a series of tests with different voice commands
    """
    print("=== Voice Command Test Suite ===\n")
    
    test_commands = [
        "move forward",
        "turn left",
        "pick up red cube",
        "move forward and turn right",
        "invalid command"
    ]
    
    for i, command in enumerate(test_commands):
        try:
            response = await simulate_voice_command(command, f"test_student_{i}")
            print(f"✓ Command '{command}' processed successfully")
            print(f"  Response: {response}\n")
        except Exception as e:
            print(f"✗ Command '{command}' failed: {str(e)}\n")
    
    print("=== Test Suite Complete ===")


def create_test_audio_file():
    """
    Create a simple test audio file for actual API testing
    This is just a placeholder - in reality, you'd record actual audio
    """
    import wave
    import struct
    
    # Create a simple test WAV file
    sample_rate = 44100  # CD quality
    duration = 1  # 1 second
    frequency = 440  # A4 note
    
    # Generate simple sine wave
    samples = []
    for i in range(int(sample_rate * duration)):
        value = int(32767.0 * 0.5 * 440.0 * 3.14159 * 2.0 * i / sample_rate)
        samples.append(struct.pack('<h', value))
    
    # Write to WAV file
    with wave.open('test_audio.wav', 'w') as wav_file:
        wav_file.setnchannels(1)  # Mono
        wav_file.setsampwidth(2)  # 16-bit
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(b''.join(samples))
    
    print("Test audio file 'test_audio.wav' created")
    
    # Read and encode to base64
    with open('test_audio.wav', 'rb') as audio_file:
        audio_bytes = audio_file.read()
        base64_audio = base64.b64encode(audio_bytes).decode('utf-8')
    
    return base64_audio


if __name__ == "__main__":
    # Run the test suite
    asyncio.run(run_test_suite())
    
    # Create a test audio file
    test_audio = create_test_audio_file()
    print(f"Test audio length: {len(test_audio)} characters")