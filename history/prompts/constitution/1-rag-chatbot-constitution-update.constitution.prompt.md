---
id: "1"
title: "rag-chatbot-constitution-update"
stage: "constitution"
date_iso: "2025-12-18"
surface: "agent"
model: "Qwen"
feature: "none"
branch: "main"
user: "M.R Computers"
command: "/sp.constitution"
labels: ["constitution", "rag", "chatbot", "textbook"]
spec: null
ticket: null
adr: null
pr: null
outcome: "Successfully updated constitution from Physical AI textbook to RAG chatbot textbook"
evaluation: "Constitution updated with new principles, version bumped from 1.0.0 to 2.0.0"
---

# Prompt History Record: RAG Chatbot Constitution Update

## Original Prompt

```
$ARGUMENTS
```

## User Input

Project: Integrated RAG Chatbot for AI-Powered Textbook

Purpose:
Design and operate a Retrieval-Augmented Generation (RAG) chatbot embedded inside a published textbook website. The chatbot must answer user questions strictly based on the textbook content or explicitly selected text provided by the user.

Core Principles:
- Groundedness: All answers must be derived only from retrieved book content.
- Zero Hallucination: No assumptions, external knowledge, or fabricated information.
- Context Fidelity: When user selects text, answers must rely exclusively on that selection.
- Explainability: Every answer should be traceable to retrieved chunks.
- Educational Clarity: Responses must be clear, concise, and student-friendly.

Knowledge Constraints:
- Allowed sources: 
  - Indexed textbook content only
  - User-selected text (if provided)
- Disallowed sources:
  - General internet knowledge
  - Model prior knowledge
  - Training data assumptions

RAG Standards:
- Retrieval must precede generation in all cases.
- Minimum similarity threshold must be enforced.
- If no relevant context is retrieved, respond with:
  "This question cannot be answered from the provided book content."

Selected Text Rule:
- If user provides selected text:
  - Ignore entire vector database
  - Use ONLY the selected text as context
  - Do not supplement with any additional information

Answer Style Guidelines:
- Tone: Professional, educational, neutral
- Length: Short to medium (unless explanation is explicitly requested)
- Formatting:
  - Bullet points for lists
  - Code blocks for technical examples
- Language: Simple English (CS-aware audience)

Safety & Accuracy Rules:
- Never guess or extrapolate beyond context
- Never merge multiple unrelated chunks
- Never answer partially if context is insufficient
- Prefer saying "Not found in the book" over guessing

Backend & Tooling Assumptions:
- LLM: Gemini (via API key)
- Backend: FastAPI
- Vector Store: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless Postgres
- Embeddings must be consistent across ingestion and querying

Evaluation Criteria:
- 100% answers grounded in retrieved content
- 0 hallucinated responses
- Correct handling of selected-text-only queries
- Clear refusal when content is missing

Success Criteria:
- Users can accurately query textbook concepts
- Chatbot behaves as a strict textbook assistant
- No answer violates content-only constraints
- Passes manual hallucination testing

## Response Summary

Updated the project constitution to reflect the new Integrated RAG Chatbot for AI-Powered Textbook project. The previous constitution was for a Physical AI & Humanoid Robotics Textbook, which has been completely replaced with the new RAG chatbot principles while maintaining the same document structure. Key changes include:

- Updated all core principles to match RAG chatbot requirements (Groundedness, Zero Hallucination, Context Fidelity, Explainability, Educational Clarity)
- Changed development standards to reflect the new tech stack (FastAPI, Qdrant Cloud, Neon Serverless Postgres, Gemini LLM API)
- Updated review and quality processes to focus on content accuracy and hallucination testing
- Updated governance to reflect content-only constraints compliance

Version was incremented from 1.0.0 to 2.0.0 due to major change in project focus and principles.

## Files Modified

- `.specify/memory/constitution.md`

## Tests Performed

None required as this was a documentation update.