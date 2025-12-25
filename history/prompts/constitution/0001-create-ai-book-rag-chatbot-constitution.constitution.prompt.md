---
id: 0001
title: Create AI Book RAG Chatbot Constitution
stage: constitution
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: ai-book-rag-chatbot
branch: 001-ros2-robotics
user: laiba
command: /sp.constitution
labels: ["constitution", "spec-first", "rag", "docusaurus", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# PHR-0001: Create AI Book RAG Chatbot Constitution

## Prompt Text

```
Project: AI-Authored Book with Embedded RAG Chatbot

Purpose:
Create a spec-driven technical book using Docusaurus and deploy it to GitHub Pages, with an embedded RAG chatbot that answers questions strictly from the book's content.

Core Principles:
- Spec-first development (Spec-Kit Plus)
- Accuracy and zero hallucination
- Clear explanations for technical readers
- Fully reproducible builds and deployments
- Modular and maintainable architecture

Book Standards:
- Framework: Docusaurus
- Format: Markdown (MD/MDX)
- Deployment: GitHub Pages
- Readability: Flesch-Kincaid grade 9–12
- No placeholder or unverifiable content

RAG Chatbot Standards:
- Backend: FastAPI
- AI: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- Metadata DB: Neon Serverless Postgres
- Features:
  - Book-wide Q&A
  - Selected-text-only Q&A
  - Source citations for all answers
  - "Not found in book" for missing context

Data & Indexing:
- Deterministic chunking and embeddings
- Auto re-indexing on content changes
- Metadata: chapter, section, URL

Constraints:
- No external knowledge in responses
- No hard-coded secrets
- Free-tier compatible services only

Success Criteria:
- Book live on GitHub Pages
- RAG chatbot embedded and functional
- Answers grounded only in book content
- Specs traceable to implementation
```

## Response Text

Created a comprehensive project constitution file that defines the core principles, technology stack standards, quality standards, and governance procedures for the AI-authored book with embedded RAG chatbot project.

## Outcome & Impact

Successfully created the project constitution at `.specify/memory/constitution.md` with all specified principles and standards. The constitution establishes clear guidelines for spec-first development, zero hallucination requirements for the RAG system, technical clarity standards, reproducible builds, modular architecture, free-tier compatibility, deterministic data processing, and source attribution requirements.

## Files Summary

- `.specify/memory/constitution.md` - New constitution file created with all project principles and standards

## Tests Summary

No tests required for constitution creation.

## Next Prompts

- `/sp.specify` to create the feature specification
- `/sp.plan` to create the architectural plan
- `/sp.tasks` to generate implementation tasks

## Reflection Note

Establishing clear constitutional principles upfront will guide all future development decisions and ensure the project maintains its core values of technical excellence and accuracy.

## Failure Modes Observed

None observed. All required information was provided in the user input and properly incorporated into the constitution.

## Next Experiment to Improve Prompt Quality

N/A - The prompt was comprehensive and clearly specified all required elements for the constitution.

---
Acceptance Criteria:
- [x] Full prompt preserved verbatim (no truncation)
- [x] Stage and routing determined correctly
- [x] Metadata fields populated; missing values noted explicitly