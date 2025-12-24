---
sidebar_label: "Chapter 2: Cognitive Planning with LLMs"
---

# Chapter 2: Cognitive Planning with LLMs

## Overview

In this chapter, we'll explore how to use Large Language Models (LLMs) for cognitive planning in robotics. You'll learn to translate natural language instructions into complex ROS 2 action sequences, implement task decomposition, and ensure safe and feasible robot actions.

## Learning Objectives

By the end of this chapter, you will be able to:
- Translate natural language instructions into ROS 2 action sequences
- Implement task decomposition algorithms
- Plan complex multi-step tasks
- Validate action feasibility and safety
- Test cognitive planning in simulation

## Introduction to Cognitive Planning

Cognitive planning in robotics involves translating high-level human instructions into detailed action sequences that a robot can execute. With the power of LLMs, we can now process natural language instructions and generate sophisticated robotic behaviors.

### LLMs in Robotics

Large Language Models like GPT-4, Claude, or open-source alternatives can understand complex instructions and reason about the steps required to accomplish tasks. In robotics, this capability enables more natural human-robot interaction and more flexible robot behavior.

## Natural Language Instruction Processing

### 1. Instruction Parsing

The first step in cognitive planning is to parse and understand the natural language instruction:

```python
class InstructionParser:
    def __init__(self):
        # Define common instruction patterns
        self.patterns = {
            'fetch': ['get', 'bring', 'fetch', 'pick up and bring'],
            'navigate': ['go to', 'move to', 'navigate to', 'travel to'],
            'manipulate': ['pick up', 'grasp', 'take', 'place', 'put', 'set'],
            'inspect': ['look at', 'examine', 'check', 'verify']
        }

    def parse_instruction(self, instruction):
        """Parse a natural language instruction into actionable components"""
        instruction_lower = instruction.lower().strip()

        # Identify the main action type
        action_type = None
        for action, keywords in self.patterns.items():
            if any(keyword in instruction_lower for keyword in keywords):
                action_type = action
                break

        if not action_type:
            return {
                'action_type': 'unknown',
                'instruction': instruction,
                'parameters': {}
            }

        # Extract parameters based on the instruction
        parameters = self.extract_parameters(instruction_lower, action_type)

        return {
            'action_type': action_type,
            'instruction': instruction,
            'parameters': parameters
        }

    def extract_parameters(self, instruction, action_type):
        """Extract specific parameters from the instruction"""
        params = {}

        if action_type == 'fetch':
            # Extract object and destination
            parts = instruction.split(' and bring')
            if len(parts) > 1:
                obj_part = parts[0]
                dest_part = parts[1]

                # Extract object name
                obj_keywords = ['get', 'bring', 'fetch', 'pick up']
                for keyword in obj_keywords:
                    if keyword in obj_part:
                        obj_part = obj_part.replace(keyword, '').strip()
                        break
                params['object'] = obj_part

                # Extract destination
                dest_keywords = ['to', 'at']
                for keyword in dest_keywords:
                    if keyword in dest_part:
                        dest_part = dest_part.split(keyword)[-1].strip()
                        break
                params['destination'] = dest_part
            else:
                # Simple fetch instruction
                for keyword in ['get', 'bring', 'fetch']:
                    if keyword in instruction:
                        obj = instruction.replace(keyword, '').strip()
                        params['object'] = obj
                        break

        elif action_type == 'navigate':
            # Extract destination
            for keyword in ['go to', 'move to', 'navigate to', 'travel to']:
                if keyword in instruction:
                    dest = instruction.replace(keyword, '').strip()
                    params['destination'] = dest
                    break

        elif action_type == 'manipulate':
            # Extract object and action details
            if 'pick up' in instruction or 'grasp' in instruction or 'take' in instruction:
                obj = instruction.replace('pick up', '').replace('grasp', '').replace('take', '').strip()
                params['object'] = obj
                params['action'] = 'pick_up'
            elif 'place' in instruction or 'put' in instruction or 'set' in instruction:
                obj = instruction.replace('place', '').replace('put', '').replace('set', '').strip()
                params['object'] = obj
                params['action'] = 'place'

        return params
```

### 2. LLM Integration for Task Decomposition

Now let's implement the LLM integration for cognitive planning:

```python
import openai
import os
import json
from typing import List, Dict, Any

class CognitivePlanner:
    def __init__(self):
        # Set your OpenAI API key
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.instruction_parser = InstructionParser()

    def decompose_task(self, instruction: str) -> List[Dict[str, Any]]:
        """
        Use an LLM to decompose a complex instruction into a sequence of actions
        """
        # First, parse the instruction to understand the high-level intent
        parsed = self.instruction_parser.parse_instruction(instruction)

        # Create a prompt for the LLM to decompose the task
        prompt = f"""
        You are a robotic task decomposition expert.
        Please decompose the following natural language instruction into a sequence of specific robotic actions.

        Instruction: "{instruction}"

        The output should be a JSON array of actions with the following structure:
        [
            {{
                "id": "unique_action_id",
                "type": "action_type" (e.g., "navigate", "manipulate", "inspect", "wait"),
                "description": "Brief description of the action",
                "parameters": {{"param_name": "param_value", ...}},
                "order": 0  // Execution order
            }}
        ]

        Action types should be limited to: navigate, manipulate, inspect, wait, communicate.
        The parameters should include all necessary information for the robot to execute the action.
        Be specific and ensure each action is feasible for a humanoid robot in a simulated environment.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3
            )

            # Extract the JSON response
            content = response.choices[0].message['content'].strip()

            # Clean up the response to extract JSON
            if content.startswith("```json"):
                content = content[7:]  # Remove ```json
            if content.endswith("```"):
                content = content[:-3]  # Remove ```

            action_sequence = json.loads(content)

            # Add order to each action if not present
            for i, action in enumerate(action_sequence):
                if 'order' not in action:
                    action['order'] = i
                if 'id' not in action:
                    import uuid
                    action['id'] = str(uuid.uuid4())

            return action_sequence

        except Exception as e:
            print(f"Error decomposing task with LLM: {e}")
            # Fallback to a simple decomposition
            return self.fallback_decomposition(instruction)

    def fallback_decomposition(self, instruction: str) -> List[Dict[str, Any]]:
        """
        Fallback task decomposition if LLM fails
        """
        # Parse the instruction
        parsed = self.instruction_parser.parse_instruction(instruction)

        actions = []

        if parsed['action_type'] == 'fetch':
            # Fetch task: navigate to object, pick it up, navigate to destination, place it
            actions = [
                {
                    "id": "nav-to-object",
                    "type": "navigate",
                    "description": f"Navigate to the {parsed['parameters'].get('object', 'object')}",
                    "parameters": {"target_location": f"location_of_{parsed['parameters'].get('object', 'object')}"},
                    "order": 0
                },
                {
                    "id": "pick-up-object",
                    "type": "manipulate",
                    "description": f"Pick up the {parsed['parameters'].get('object', 'object')}",
                    "parameters": {"object": parsed['parameters'].get('object', 'object')},
                    "order": 1
                },
                {
                    "id": "nav-to-dest",
                    "type": "navigate",
                    "description": f"Navigate to {parsed['parameters'].get('destination', 'destination')}",
                    "parameters": {"target_location": parsed['parameters'].get('destination', 'destination')},
                    "order": 2
                },
                {
                    "id": "place-object",
                    "type": "manipulate",
                    "description": f"Place the {parsed['parameters'].get('object', 'object')}",
                    "parameters": {"object": parsed['parameters'].get('object', 'object')},
                    "order": 3
                }
            ]

        elif parsed['action_type'] == 'navigate':
            actions = [
                {
                    "id": "navigate",
                    "type": "navigate",
                    "description": f"Navigate to {parsed['parameters'].get('destination', 'destination')}",
                    "parameters": {"target_location": parsed['parameters'].get('destination', 'destination')},
                    "order": 0
                }
            ]

        elif parsed['action_type'] == 'manipulate':
            actions = [
                {
                    "id": "manipulate",
                    "type": "manipulate",
                    "description": f"{parsed['parameters'].get('action', 'manipulate')} {parsed['parameters'].get('object', 'object')}",
                    "parameters": {
                        "action": parsed['parameters'].get('action', 'manipulate'),
                        "object": parsed['parameters'].get('object', 'object')
                    },
                    "order": 0
                }
            ]

        else:
            actions = [
                {
                    "id": "unknown",
                    "type": "unknown",
                    "description": f"Unknown instruction: {instruction}",
                    "parameters": {"instruction": instruction},
                    "order": 0
                }
            ]

        return actions
```

## Action Sequence Planning

### 1. Creating Action Sequences

Once we have decomposed the task, we need to create a proper action sequence:

```python
import uuid
from datetime import datetime

class ActionSequencePlanner:
    def __init__(self):
        self.cognitive_planner = CognitivePlanner()

    def create_action_sequence(self, instruction: str, student_id: str) -> Dict[str, Any]:
        """
        Create an action sequence from a natural language instruction
        """
        # Decompose the task using cognitive planning
        actions = self.cognitive_planner.decompose_task(instruction)

        # Create the action sequence
        sequence = {
            "id": str(uuid.uuid4()),
            "instruction": instruction,
            "student_id": student_id,
            "actions": actions,
            "status": "pending",
            "created_at": datetime.utcnow().isoformat(),
            "estimated_duration": self.estimate_duration(actions)
        }

        return sequence

    def estimate_duration(self, actions: List[Dict[str, Any]]) -> float:
        """
        Estimate the duration of the action sequence
        """
        # Rough estimation based on action types
        duration = 0.0

        for action in actions:
            action_type = action.get('type', 'unknown')

            if action_type == 'navigate':
                duration += 5.0  # 5 seconds per navigation
            elif action_type == 'manipulate':
                duration += 3.0  # 3 seconds per manipulation
            elif action_type == 'inspect':
                duration += 2.0  # 2 seconds per inspection
            elif action_type == 'wait':
                # Use the wait duration if specified
                wait_time = action.get('parameters', {}).get('duration', 1.0)
                duration += wait_time
            else:
                duration += 1.0  # 1 second for other actions

        return duration
```

### 2. Safety Validation for Planned Actions

We need to validate the planned actions for safety before execution:

```python
class SafetyValidator:
    def __init__(self):
        pass

    def validate_action_sequence(self, sequence: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate an action sequence for safety
        """
        violations = []

        for action in sequence.get('actions', []):
            action_violations = self.validate_single_action(action, sequence)
            violations.extend(action_violations)

        return {
            "sequence_id": sequence['id'],
            "is_safe": len(violations) == 0,
            "violations": violations,
            "action_sequence": sequence
        }

    def validate_single_action(self, action: Dict[str, Any], sequence: Dict[str, Any]) -> List[Dict[str, str]]:
        """
        Validate a single action for safety
        """
        violations = []

        action_type = action.get('type', 'unknown')

        if action_type == 'navigate':
            # Check if navigation target is valid
            target = action.get('parameters', {}).get('target_location', '')
            if not target:
                violations.append({
                    "id": str(uuid.uuid4()),
                    "type": "missing_parameter",
                    "message": "Navigation action missing target location",
                    "severity": "error",
                    "action_id": action['id']
                })

        elif action_type == 'manipulate':
            # Check if manipulation parameters are valid
            obj = action.get('parameters', {}).get('object', '')
            action_param = action.get('parameters', {}).get('action', '')

            if not obj:
                violations.append({
                    "id": str(uuid.uuid4()),
                    "type": "missing_parameter",
                    "message": "Manipulation action missing object parameter",
                    "severity": "error",
                    "action_id": action['id']
                })

            if not action_param:
                violations.append({
                    "id": str(uuid.uuid4()),
                    "type": "missing_parameter",
                    "message": "Manipulation action missing action parameter",
                    "severity": "error",
                    "action_id": action['id']
                })

        elif action_type == 'unknown':
            violations.append({
                "id": str(uuid.uuid4()),
                "type": "invalid_action",
                "message": f"Unknown action type in sequence: {action}",
                "severity": "error",
                "action_id": action['id']
            })

        # Add more validation rules as needed

        return violations
```

## Implementation in the API

### 1. Processing Natural Language Instructions

Let's implement the API endpoint for processing natural language instructions:

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Any

app = FastAPI()

class InstructionRequest(BaseModel):
    instruction: str
    student_id: str
    simulation_environment_id: str

class InstructionResponse(BaseModel):
    instruction_id: str
    decomposed_actions: List[Dict[str, Any]]
    action_sequence: Dict[str, Any]
    safety_constraints_applied: List[Dict[str, str]]

@app.post("/instructions/process", response_model=InstructionResponse)
async def process_instruction(request: InstructionRequest):
    """
    Submit a complex natural language instruction for cognitive planning and execution
    """
    try:
        # Create the action sequence planner
        planner = ActionSequencePlanner()

        # Create the action sequence
        sequence = planner.create_action_sequence(request.instruction, request.student_id)

        # Validate the sequence for safety
        validator = SafetyValidator()
        validation_result = validator.validate_action_sequence(sequence)

        # Prepare the response
        response = InstructionResponse(
            instruction_id=sequence['id'],
            decomposed_actions=sequence['actions'],
            action_sequence={
                "id": sequence['id'],
                "status": "pending" if validation_result['is_safe'] else "validation_failed"
            },
            safety_constraints_applied=validation_result['violations']
        )

        # If there are safety violations, we might want to prevent execution
        if not validation_result['is_safe']:
            raise HTTPException(
                status_code=422,
                detail=f"Unprocessable instruction: {len(validation_result['violations'])} safety violations detected"
            )

        # Store the sequence for execution
        store_action_sequence(sequence, request.simulation_environment_id)

        return response

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing instruction: {str(e)}")

def store_action_sequence(sequence: Dict[str, Any], simulation_env_id: str):
    """
    Store the action sequence for execution in the simulation
    """
    # In a real implementation, this would store the sequence in a database
    # and trigger the execution process
    print(f"Storing action sequence {sequence['id']} for execution in {simulation_env_id}")
    # Add to execution queue, etc.
```

## Testing Cognitive Planning

### 1. Testing Complex Instructions

Let's create a test for complex instruction processing:

```python
def test_cognitive_planning():
    """
    Test cognitive planning with various complex instructions
    """
    planner = ActionSequencePlanner()

    test_instructions = [
        "Go to the kitchen and bring me the red cup",
        "Navigate to the table, pick up the book, and place it on the shelf",
        "Check if the door is open, and if it is, go through it"
    ]

    for instruction in test_instructions:
        print(f"\nTesting instruction: '{instruction}'")

        try:
            sequence = planner.create_action_sequence(instruction, "test_student")
            print(f"Generated action sequence: {sequence['id']}")
            print(f"Number of actions: {len(sequence['actions'])}")
            print(f"Estimated duration: {sequence['estimated_duration']:.2f} seconds")

            for action in sequence['actions']:
                print(f"  - {action['order']}: {action['type']} - {action['description']}")

        except Exception as e:
            print(f"Error processing instruction '{instruction}': {e}")

if __name__ == "__main__":
    test_cognitive_planning()
```

## Summary

In this chapter, we've learned how to:
- Parse natural language instructions for robotic tasks
- Use LLMs for cognitive planning and task decomposition
- Create action sequences from high-level instructions
- Validate planned actions for safety
- Implement API endpoints for instruction processing

## Exercises

1. Implement additional action types in the cognitive planner
2. Add more sophisticated safety validation rules
3. Implement a feedback mechanism for users to correct plan mistakes
4. Add support for conditional actions in the planning process

## Next Steps

In the next chapter, we'll integrate all components into a complete VLA pipeline for the capstone project.