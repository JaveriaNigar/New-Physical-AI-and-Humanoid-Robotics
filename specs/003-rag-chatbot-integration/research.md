# Research: Integrated RAG Chatbot for AI-Powered Textbook

**Feature**: 003-rag-chatbot-integration  
**Date**: 2025-12-18  
**Status**: Complete

## Overview

This research document addresses the technical decisions and best practices for implementing a Retrieval-Augmented Generation (RAG) system integrated with an AI-powered textbook. The focus is on creating a hallucination-free system that strictly answers from book content or user-selected text.

## Research Areas

### 1. RAG Architecture Patterns

**Decision**: Multi-stage RAG pipeline with clear separation of ingestion, retrieval, and generation

**Rationale**: This architecture provides clear interfaces between components, making the system maintainable and ensuring compliance with the constitution's requirement for clear separation between ingestion, retrieval, and generation components. It also allows for independent testing and validation of each component.

**Alternatives considered**: 
- End-to-end RAG without clear separation: Rejected because it makes it difficult to validate that each component is functioning correctly and makes compliance with hallucination-free requirements harder to verify.

### 2. Vector Database Selection

**Decision**: Qdrant Cloud for vector storage

**Rationale**: Qdrant was explicitly specified in the feature requirements and offers good support for similarity search, which is essential for RAG systems. It's also suitable for the free-tier compatibility requirement.

**Alternatives considered**:
- Pinecone: A strong alternative but not specified in requirements
- Weaviate: Good functionality but Qdrant was specified
- FAISS: Requires more infrastructure management

### 3. Embedding Models

**Decision**: Google's embedding model compatible with Qdrant for consistency with Gemini LLM

**Rationale**: Using Google's embedding model ensures compatibility with the Gemini LLM as specified in requirements. This creates consistency in the Google AI ecosystem.

**Alternatives considered**:
- OpenAI embeddings: Rejected due to constraints against OpenAI APIs
- Sentence Transformers: Would require additional infrastructure but could be an option if needed

### 4. Chunking Strategy

**Decision**: Semantic chunking based on textbook section boundaries with overlap for context preservation

**Rationale**: Since this is for a textbook, content is already organized in semantically coherent sections. Breaking on natural boundaries will preserve meaning while allowing for effective retrieval. Overlap ensures no context is lost at boundaries.

**Alternatives considered**:
- Fixed-size token chunking: May break up related content
- Sentence-based chunking: Could create too many small chunks

### 5. Retrieval Threshold Configuration

**Decision**: Configurable minimum similarity threshold with fallback to refusal response

**Rationale**: The system must maintain zero hallucination tolerance, so having a configurable threshold that defaults to a conservative setting ensures responses are only generated from sufficiently relevant context.

**Alternatives considered**:
- No threshold (always return results): Rejected as it would violate hallucination-free requirement
- Fixed threshold: Less flexible for different content types

### 6. Metadata Storage Strategy

**Decision**: Store chunk metadata in Neon Postgres for traceability and source attribution

**Rationale**: This supports explainability requirements by enabling the system to reference which textbook section a chunk came from, making responses traceable to source material.

**Alternatives considered**:
- Store only in vector store: Less query flexibility
- File-based storage: Less scalable for large textbooks

### 7. Selected-Text-Only Mode Implementation

**Decision**: Separate endpoint that bypasses vector database completely

**Rationale**: This ensures complete isolation between the general textbook mode and selected-text mode, guaranteeing that selected text is the ONLY context used as required by the constitution.

**Alternatives considered**:
- Shared endpoint with mode flag: Higher risk of accidentally allowing other context to influence responses

## Implementation Considerations

### Security and Credential Handling

Based on the constitution and feature requirements, all credentials must be loaded from environment variables, with no hardcoded secrets in code, prompts, or configuration files. The implementation will use Python's os.environ for secure credential loading.

### Performance Requirements

The system must respond within 10 seconds for user queries. This will be achieved through:
- Optimized vector database queries
- Connection pooling for database operations
- Efficient embedding model usage
- Caching where appropriate

### Testing Strategy

The system requires comprehensive testing to ensure:
- Zero hallucination tolerance: Tests that verify responses are only from provided content
- Correct refusal behavior: Tests for when no relevant context is found
- Selected-text mode isolation: Tests that verify vector database is bypassed
- Performance compliance: Load testing to ensure response time requirements are met