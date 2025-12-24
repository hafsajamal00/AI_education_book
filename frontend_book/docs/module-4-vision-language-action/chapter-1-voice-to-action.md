---
sidebar_label: "Chapter 1: Voice-to-Action with OpenAI Whisper"
---

# Chapter 1: Voice-to-Action with OpenAI Whisper

## Overview

In this chapter, we'll explore how to integrate OpenAI Whisper for real-time voice command recognition in humanoid robots. You'll learn to map voice commands to robot actions and integrate them with ROS 2 for execution.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the basics of speech recognition using OpenAI Whisper
- Map voice commands to specific robot actions
- Integrate voice recognition with ROS 2 action execution
- Test voice commands in a simulated environment

## Prerequisites

Before starting this chapter, you should have:
- Basic understanding of ROS 2 concepts
- Experience with Python programming
- Familiarity with simulation environments (Gazebo, Unity, or NVIDIA Isaac)

## Introduction to Voice Command Recognition

Voice command recognition is a fundamental component of human-robot interaction. It enables users to control robots using natural language, making the interaction more intuitive and accessible.

### OpenAI Whisper

OpenAI Whisper is a state-of-the-art speech recognition model that provides high accuracy in various languages and audio conditions. In this module, we'll use Whisper to convert spoken commands into text that can be processed by our robot control system.

## Setting Up Voice Recognition

### 1. Audio Input Configuration

First, we need to configure audio input for the robot system. This can be done through the robot's microphone or via a connected device.

```python
import pyaudio
import wave
import threading
import queue

class AudioInput:
    def __init__(self):
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.record_seconds = 3
        self.audio_queue = queue.Queue()

    def record_audio(self):
        """Record audio from microphone"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        print("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Finished recording")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save the recorded data as WAV file
        wf = wave.open("temp_audio.wav", 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return "temp_audio.wav"
```

### 2. Whisper Integration

Next, we'll integrate Whisper to convert the recorded audio to text:

```python
import openai
import os

class WhisperRecognizer:
    def __init__(self):
        # Set your OpenAI API key
        openai.api_key = os.getenv("OPENAI_API_KEY")

    def transcribe_audio(self, audio_file_path):
        """Transcribe audio file using OpenAI Whisper"""
        with open(audio_file_path, "rb") as audio_file:
            transcript = openai.Audio.transcribe("whisper-1", audio_file)

        return transcript.text
```

## Mapping Voice Commands to Robot Actions

### 1. Command Parsing

Once we have the transcribed text, we need to parse it and map it to specific robot actions:

```python
class CommandMapper:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'move_forward': ['move forward', 'go forward', 'move ahead', 'go ahead'],
            'move_backward': ['move backward', 'go backward', 'move back', 'go back'],
            'turn_left': ['turn left', 'rotate left', 'pivot left'],
            'turn_right': ['turn right', 'rotate right', 'pivot right'],
            'pick_up': ['pick up', 'grab', 'take'],
            'place': ['place', 'put down', 'set down']
        }

    def parse_command(self, text):
        """Parse the transcribed text and return the corresponding action"""
        text_lower = text.lower().strip()

        for action, patterns in self.command_patterns.items():
            for pattern in patterns:
                if pattern in text_lower:
                    return action, self.extract_parameters(text_lower, pattern)

        return 'unknown', {}

    def extract_parameters(self, text, command_pattern):
        """Extract parameters from the command"""
        # Simple parameter extraction based on command pattern
        remaining_text = text.replace(command_pattern, '').strip()

        # Extract object names for pick_up commands
        if 'pick up' in command_pattern or 'grab' in command_pattern or 'take' in command_pattern:
            return {'object': remaining_text}

        return {}
```

### 2. ROS 2 Action Integration

Now, let's connect our voice command system to ROS 2 actions:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Publisher for robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for voice commands
        self.voice_command_subscriber = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        self.get_logger().info('Voice Command Node initialized')

    def voice_command_callback(self, msg):
        """Process voice command and execute corresponding action"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Parse the command
        mapper = CommandMapper()
        action, params = mapper.parse_command(command)

        if action == 'move_forward':
            self.move_forward()
        elif action == 'move_backward':
            self.move_backward()
        elif action == 'turn_left':
            self.turn_left()
        elif action == 'turn_right':
            self.turn_right()
        elif action == 'pick_up':
            self.pick_up_object(params.get('object', ''))
        elif action == 'place':
            self.place_object()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def move_forward(self):
        """Move the robot forward"""
        msg = Twist()
        msg.linear.x = 0.5  # Forward speed
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Moving forward')

    def move_backward(self):
        """Move the robot backward"""
        msg = Twist()
        msg.linear.x = -0.5  # Backward speed
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Moving backward')

    def turn_left(self):
        """Turn the robot left"""
        msg = Twist()
        msg.angular.z = 0.5  # Left turn speed
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Turning left')

    def turn_right(self):
        """Turn the robot right"""
        msg = Twist()
        msg.angular.z = -0.5  # Right turn speed
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Turning right')

    def pick_up_object(self, object_name):
        """Simulate picking up an object"""
        self.get_logger().info(f'Attempting to pick up: {object_name}')
        # In a real implementation, this would call a manipulation action server
        # For simulation, we'll just log the action

    def place_object(self):
        """Simulate placing an object"""
        self.get_logger().info('Placing object')
        # In a real implementation, this would call a manipulation action server
        # For simulation, we'll just log the action
```

## Testing in Simulation

### 1. Setting Up the Simulation Environment

To test our voice-to-action system, we'll use a simulation environment. Here's how to set up a basic test:

```python
def main():
    # Initialize ROS 2
    rclpy.init()

    # Create the voice command node
    voice_node = VoiceCommandNode()

    # Create audio input and recognizer
    audio_input = AudioInput()
    recognizer = WhisperRecognizer()
    mapper = CommandMapper()

    # Main loop - continuously listen for voice commands
    try:
        while rclpy.ok():
            # Record audio
            audio_file = audio_input.record_audio()

            # Transcribe audio to text
            text = recognizer.transcribe_audio(audio_file)
            print(f"Transcribed: {text}")

            # Parse the command
            action, params = mapper.parse_command(text)
            print(f"Action: {action}, Parameters: {params}")

            # Publish the command to the ROS 2 system
            cmd_msg = String()
            cmd_msg.data = text
            voice_node.voice_command_publisher.publish(cmd_msg)

            # Sleep briefly
            voice_node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        # Clean up
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Running the Test

1. Make sure your simulation environment is running with a robot that accepts `/cmd_vel` commands.
2. Set your OpenAI API key in the environment variables:
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```
3. Run the voice command system:
   ```bash
   python voice_command_system.py
   ```
4. Speak commands like "move forward" or "turn left" to control the robot.

## Safety Considerations

When implementing voice command systems, always consider safety:

1. **Validation**: Always validate commands before executing them
2. **Speed Limiting**: Limit robot speeds to prevent accidents
3. **Emergency Stop**: Implement an emergency stop command
4. **Environment Awareness**: Ensure the robot is aware of its surroundings

## Summary

In this chapter, we've learned how to:
- Integrate OpenAI Whisper for voice command recognition
- Map voice commands to robot actions
- Connect the system with ROS 2 for execution
- Test the system in a simulated environment

## Exercises

1. Add more command patterns to the CommandMapper class
2. Implement an emergency stop command
3. Add voice feedback to confirm command recognition
4. Create a more sophisticated parameter extraction system

## Next Steps

In the next chapter, we'll explore cognitive planning with LLMs, where you'll learn to translate natural language instructions into complex ROS 2 action sequences.