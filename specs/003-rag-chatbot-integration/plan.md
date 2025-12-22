# Implementation Plan: Integrated RAG Chatbot for AI-Powered Textbook

**Branch**: `003-rag-chatbot-integration` | **Date**: 2025-12-18 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-rag-chatbot-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation of a Retrieval-Augmented Generation (RAG) system that integrates with the existing AI-powered textbook frontend. The system will enable students to ask questions about textbook content and receive hallucination-free, grounded responses. The RAG pipeline will extract textbook content, generate embeddings, store them in Qdrant Cloud, and use Gemini LLM for response generation. The system will also support "selected text only" mode where user-highlighted text is used as the exclusive context, bypassing the vector database entirely. The backend will be implemented using FastAPI with Neon Serverless Postgres for metadata storage.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Qdrant client, Pydantic, Google AI SDK (for Gemini), AsyncIO, Requests
**Storage**: Qdrant Cloud (vector store), Neon Serverless Postgres (metadata), with fallback to local SQLite for development
**Testing**: pytest with integration and unit test frameworks
**Target Platform**: Linux server (cloud deployment)
**Project Type**: Web application with backend API service
**Performance Goals**: <10 second response time for queries, support for 100 concurrent users
**Constraints**: <200ms p95 retrieval time, <100MB memory usage for normal operation, zero hallucination tolerance, no external knowledge usage
**Scale/Scope**: Single textbook with up to 1 million content chunks, 10,000 concurrent users, 100 queries per second

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Groundedness compliance**: Backend will retrieve content only from indexed book content, with no external knowledge
2. **Zero Hallucination compliance**: System will refuse to answer when no relevant context is found, with strict adherence to provided content
3. **Context Fidelity compliance**: Selected text mode will completely bypass vector database, using only user-provided text as context
4. **Explainability compliance**: Each response will include traceability to source chunks with metadata
5. **Educational Clarity compliance**: Responses will maintain professional, educational tone appropriate for students

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-chatbot-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── checklists/          # Validation checklists
    └── requirements.md
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py              # FastAPI application entry point
│   ├── config/              # Configuration and environment variables
│   │   ├── settings.py
│   │   └── constants.py
│   ├── models/              # Pydantic models for API requests/responses
│   │   ├── chat.py
│   │   ├── embeddings.py
│   │   └── common.py
│   ├── services/            # Business logic services
│   │   ├── retrieval_service.py
│   │   ├── generation_service.py
│   │   ├── embedding_service.py
│   │   └── validation_service.py
│   ├── api/                 # API route handlers
│   │   ├── chat_routes.py
│   │   ├── health_routes.py
│   │   └── __init__.py
│   ├── database/            # Database operations (metadata)
│   │   ├── postgres_client.py
│   │   └── models.py
│   └── vector_store/        # Vector store operations
│       ├── qdrant_client.py
│       └── schemas.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── requirements.txt
├── Dockerfile
└── docker-compose.yml
```

**Structure Decision**: Web application structure with separate backend service for the RAG implementation, as frontend already exists and is deployed in the physical-ai-book directory.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
