<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0 (major change: project focus changed from Physical AI textbook to RAG chatbot textbook)
- Modified principles: All principles completely revised (old principles removed and new ones added)
- Added sections: Groundedness, Zero Hallucination, Context Fidelity, Explainability, Educational Clarity principles
- Removed sections: All previous Physical AI textbook principles
- Templates requiring updates: ✅ plan-template.md, ✅ spec-template.md, ✅ tasks-template.md (need to reflect new RAG chatbot project focus)
- Follow-up TODOs: None
-->
# Integrated RAG Chatbot for AI-Powered Textbook Constitution

## Core Principles

### Groundedness
All answers must be derived only from retrieved book content; No external knowledge or assumptions allowed during response generation

### Zero Hallucination
Never fabricate information, make assumptions, or use model prior knowledge; Strictly adhere to provided textbook content only

### Context Fidelity
When user selects text, answers must rely exclusively on that selection; Ignore entire vector database when user provides selected text as context

### Explainability
Every answer should be traceable to retrieved chunks; Provide clear references to source material when possible

### Educational Clarity
Responses must be clear, concise, and student-friendly; Maintain professional, educational tone appropriate for learning context

## Development Standards
Technology stack: FastAPI backend, Qdrant Cloud vector store, Neon Serverless Postgres, Gemini LLM API; All content must be grounded in textbook material with verifiable citations

## Review and Quality Process
All responses validated for content accuracy against textbook material; Hallucination testing performed regularly; Documentation updates synchronized with model improvements

## Governance
Constitution governs all chatbot behavior and response practices; Amendments require explicit team approval and documentation; Regular reviews scheduled to ensure compliance with content-only constraints

**Version**: 2.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-18
