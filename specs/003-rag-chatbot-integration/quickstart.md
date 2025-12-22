# Quickstart Guide: Integrated RAG Chatbot for AI-Powered Textbook

**Feature**: 003-rag-chatbot-integration  
**Date**: 2025-12-18

## Overview

This guide provides the essential information to get started with developing the RAG chatbot backend for the AI-powered textbook. The system implements a hallucination-free architecture that strictly answers from textbook content or user-selected text.

## Prerequisites

- Python 3.11 or higher
- Access to Qdrant Cloud (vector database)
- Access to Neon Serverless Postgres
- Google AI API key for Gemini
- Git for version control

## Environment Setup

### 1. Clone the repository

```bash
git clone <repository-url>
cd hackathon2
git checkout 003-rag-chatbot-integration
```

### 2. Set up the backend environment

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Configure environment variables

Create a `.env` file in the backend directory with the following variables:

```env
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
NEON_DATABASE_URL=your_neon_database_connection_string_here
QDRANT_CLUSTER_ID=your_cluster_id_here
ENVIRONMENT=development  # or 'production'
```

## Development Workflow

### 1. Running the development server

```bash
cd backend
python -m src.main
```

The server will start on `http://localhost:8000`

### 2. Running tests

```bash
cd backend
pytest tests/unit/
pytest tests/integration/
```

### 3. Applying database migrations

```bash
cd backend
python -m src.database.migrations
```

## Key Architecture Components

### 1. Content Chunking Pipeline
- Located in `src/services/embedding_service.py`
- Extracts textbook content and creates semantic chunks
- Generates embeddings for each chunk
- Stores metadata in Postgres and embeddings in Qdrant

### 2. Retrieval Service
- Located in `src/services/retrieval_service.py`
- Handles semantic search in Qdrant
- Applies minimum similarity threshold
- Returns relevant chunks for generation

### 3. Generation Service
- Located in `src/services/generation_service.py`
- Uses Gemini LLM to generate responses
- Follows strict context-only answering rules
- Refuses to answer when no relevant context is found

### 4. API Endpoints
- `/chat/query` - For general textbook questions
- `/chat/selected-text` - For questions about user-selected text
- `/health` - Service health check

## Key Development Patterns

### 1. Hallucination Prevention
```python
# Always validate that context exists before generation
if not retrieved_chunks and not selected_text:
    return refusal_response("No relevant context found for your question.")
```

### 2. Selected Text Only Mode
```python
# When selected text is provided, bypass vector database completely
if query.selected_text:
    return self._generate_from_selected_text(query.question, query.selected_text)
else:
    return self._generate_from_retrieved_context(query.question)
```

### 3. Context Traceability
```python
# Always track which chunks were used to generate a response
response.metadata = {
    "retrieved_chunks": [chunk.id for chunk in relevant_chunks],
    "confidence_score": confidence_score
}
```

## Testing Strategy

### 1. Unit Testing
- Test each service independently
- Mock external dependencies (Qdrant, Gemini API)
- Verify business logic follows constitutional principles

### 2. Integration Testing
- Test end-to-end workflows
- Verify data flows between components
- Validate hallucination prevention mechanisms

### 3. Contract Testing
- Use contracts in `specs/003-rag-chatbot-integration/contracts/`
- Ensure API responses match defined schemas
- Verify proper error handling

## Configuration Parameters

### Performance Settings
- `RETRIEVAL_TIMEOUT`: Maximum time to wait for vector search (default: 5 seconds)
- `MAX_CONTEXT_LENGTH`: Maximum characters allowed in context (default: 3000)
- `MIN_CONFIDENCE_THRESHOLD`: Minimum similarity score to return results (default: 0.5)

### Security Settings
- `MAX_QUESTION_LENGTH`: Maximum length of user questions (default: 1000)
- `RATE_LIMIT_REQUESTS`: Max requests per minute per IP (default: 60)

## Troubleshooting

### Common Issues
1. **"No relevant context found" responses**: Check that textbook content has been properly indexed in Qdrant
2. **Slow response times**: Verify Qdrant and Postgres connections are optimized
3. **Hallucination detection**: Check that all responses are properly validated against context before generation

### Useful Commands
```bash
# Verify service health
curl http://localhost:8000/health

# Check Qdrant connection
python -c "from src.vector_store.qdrant_client import check_connection; check_connection()"

# Verify database schema
python -c "from src.database.postgres_client import check_schema; check_schema()"
```