# Data Model: Vision-Language-Action (VLA) Robotics Education Module

## Overview
This document defines the key data entities for the VLA educational module, including their properties and relationships.

## Entity: Voice Command
**Description**: Natural language input from students that triggers robot actions

**Properties**:
- `id` (string): Unique identifier for the command
- `content` (string): The actual text content of the voice command
- `confidence_score` (number): Confidence level of speech recognition (0-1)
- `timestamp` (datetime): When the command was issued
- `student_id` (string): Identifier of the student who issued the command
- `raw_audio` (string): Path/reference to the original audio file

**Validation Rules**:
- Content must not be empty
- Confidence score must be between 0 and 1
- Timestamp must be in ISO 8601 format

## Entity: Robot Action Sequence
**Description**: A series of executable steps derived from natural language, including navigation, manipulation, and perception tasks

**Properties**:
- `id` (string): Unique identifier for the action sequence
- `sequence` (array): Ordered list of individual robot actions
- `status` (string): Current status (pending, executing, completed, failed)
- `created_at` (datetime): When the sequence was created
- `executed_at` (datetime): When the sequence execution started
- `completed_at` (datetime): When the sequence execution completed
- `robot_id` (string): Identifier of the robot executing the sequence

**Validation Rules**:
- Sequence must contain at least one action
- Status must be one of the allowed values
- Timestamps must be in ISO 8601 format

## Entity: Individual Robot Action
**Description**: A single action within a sequence (navigation, manipulation, perception, etc.)

**Properties**:
- `id` (string): Unique identifier for the action
- `type` (string): Type of action (navigation, manipulation, perception, custom)
- `parameters` (object): Action-specific parameters
- `sequence_id` (string): Reference to the parent action sequence
- `order` (number): Execution order within the sequence

**Validation Rules**:
- Type must be one of the allowed values
- Parameters must match the expected structure for the action type
- Order must be a non-negative integer

## Entity: Safety Constraint
**Description**: Conditions that must be validated before executing robot actions to ensure safe operation

**Properties**:
- `id` (string): Unique identifier for the constraint
- `constraint_type` (string): Type of constraint (environmental, physical, operational)
- `condition` (string): The specific condition that must be validated
- `severity` (string): Severity level (warning, error)
- `action_sequence_id` (string): Reference to the action sequence this constraint applies to

**Validation Rules**:
- Constraint type must be one of the allowed values
- Severity must be one of the allowed values

## Entity: Simulation Environment
**Description**: Virtual space where students can test VLA systems with humanoid robots and objects

**Properties**:
- `id` (string): Unique identifier for the environment
- `name` (string): Name of the simulation environment
- `type` (string): Type of environment (Gazebo, Unity, NVIDIA Isaac, Custom)
- `description` (string): Description of the environment
- `objects` (array): List of objects available in the environment
- `robot_configurations` (array): List of robot configurations available

**Validation Rules**:
- Type must be one of the allowed values
- Name must not be empty

## Entity: Student Assessment
**Description**: Data related to measuring student understanding of VLA concepts

**Properties**:
- `id` (string): Unique identifier for the assessment
- `student_id` (string): Identifier of the student
- `module_id` (string): Identifier of the VLA module
- `task_completed` (boolean): Whether the student completed the task
- `execution_time` (number): Time taken to complete the task (in seconds)
- `accuracy_rate` (number): Accuracy of command execution (0-1)
- `feedback` (string): Qualitative feedback on performance
- `timestamp` (datetime): When the assessment was recorded

**Validation Rules**:
- Accuracy rate must be between 0 and 1
- Timestamp must be in ISO 8601 format
- Execution time must be a positive number

## Relationships
- A Voice Command is issued by one Student and may result in one Robot Action Sequence
- A Robot Action Sequence contains multiple Individual Robot Actions
- A Robot Action Sequence may have multiple Safety Constraints applied
- A Simulation Environment hosts multiple Robot Action Sequences
- Multiple Student Assessments can be associated with one Student and one VLA module