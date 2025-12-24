# Data Model: NVIDIA Isaac™ Robot Training Module

## Entities

### Training Module
- **Description**: Represents a complete section of the curriculum (e.g., Isaac Sim Fundamentals, Isaac ROS for Perception)
- **Fields**:
  - id: string (unique identifier)
  - title: string (e.g., "NVIDIA Isaac Sim Fundamentals")
  - description: string (brief overview of the module content)
  - chapters: array of Chapter entities
  - prerequisites: array of strings (required knowledge areas)
  - estimatedDuration: number (in hours)
  - learningObjectives: array of strings (what students will learn)

### Chapter
- **Description**: Individual chapter within a training module
- **Fields**:
  - id: string (unique identifier)
  - title: string (e.g., "Photorealistic simulation concepts")
  - content: string (markdown content of the chapter)
  - module: reference to TrainingModule
  - order: number (sequence within the module)
  - duration: number (estimated time in minutes)
  - exercises: array of Exercise entities

### Student
- **Description**: Represents a learner engaging with the training content, with progress tracking
- **Fields**:
  - id: string (unique identifier)
  - name: string
  - email: string
  - enrollmentDate: date
  - progress: array of Progress entities
  - prerequisiteAssessment: Assessment entity
  - completedModules: array of TrainingModule references

### Simulation Environment
- **Description**: Represents a virtual space where students can practice Isaac tools
- **Fields**:
  - id: string (unique identifier)
  - name: string (e.g., "Isaac Sim Basic Environment")
  - description: string
  - tools: array of string (e.g., "Isaac Sim", "ROS", "Nav2")
  - configuration: object (simulation settings)
  - humanoidModel: string (type of humanoid robot model)
  - tutorials: array of string (associated tutorial IDs)

### Assessment
- **Description**: Represents evaluation tools to measure student understanding of concepts
- **Fields**:
  - id: string (unique identifier)
  - title: string (e.g., "Prerequisite Assessment")
  - type: string (e.g., "prerequisite", "module", "final")
  - questions: array of Question entities
  - passingScore: number (percentage required to pass)
  - timeLimit: number (in minutes, optional)
  - student: reference to Student

### Question
- **Description**: Individual question within an assessment
- **Fields**:
  - id: string (unique identifier)
  - text: string (the question text)
  - type: string (e.g., "multiple-choice", "short-answer", "practical")
  - options: array of string (for multiple-choice questions)
  - correctAnswer: string (the correct answer)
  - explanation: string (explanation of the correct answer)
  - assessment: reference to Assessment

### Exercise
- **Description**: Hands-on activity for students to practice concepts
- **Fields**:
  - id: string (unique identifier)
  - title: string
  - description: string
  - instructions: string (step-by-step guide)
  - requiredTools: array of string (e.g., "Isaac Sim", "ROS")
  - expectedOutcome: string (what the student should achieve)
  - chapter: reference to Chapter
  - difficulty: string (e.g., "beginner", "intermediate", "advanced")

### Progress
- **Description**: Tracks student progress through modules and chapters
- **Fields**:
  - id: string (unique identifier)
  - student: reference to Student
  - module: reference to TrainingModule
  - chapter: reference to Chapter (optional, if tracking at chapter level)
  - status: string (e.g., "not-started", "in-progress", "completed")
  - completionDate: date (when the item was completed)
  - score: number (if applicable, from associated assessments)