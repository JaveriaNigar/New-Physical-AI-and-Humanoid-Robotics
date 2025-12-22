# Data Model: Integrated RAG Chatbot for AI-Powered Textbook

**Feature**: 003-rag-chatbot-integration  
**Date**: 2025-12-18  
**Status**: Draft

## Overview

This document defines the data model for the Retrieval-Augmented Generation (RAG) system that integrates with an AI-powered textbook. The system stores textbook content as embeddings in a vector database, with metadata in a relational database for traceability.

## Core Entities

### ContentChunk

Represents a semantically coherent piece of textbook content that has been vectorized for retrieval.

**Fields**:
- `chunk_id`: String (Primary Key) - Unique identifier for the content chunk
- `text_content`: String - The actual text content of the chunk
- `embedding`: Float Array - Vector representation of the text content (for Qdrant)
- `metadata`: JSON Object - Additional information about the chunk
  - `chapter`: String - Chapter name or identifier
  - `section`: String - Section name or identifier
  - `page_number`: Integer - Page number in the textbook
  - `source_file`: String - Original source file name
  - `created_at`: DateTime - Timestamp when the chunk was created
- `created_at`: DateTime - Timestamp when the chunk was created

**Relationships**:
- None (self-contained for vector storage in Qdrant)

### ChunkReference

Metadata for content chunks stored in the relational database for traceability and querying.

**Fields**:
- `id`: UUID (Primary Key) - Unique identifier for the reference
- `chunk_id`: String - Reference to the content chunk in vector store
- `chapter`: String - Chapter name or identifier
- `section`: String - Section name or identifier
- `page_number`: Integer - Page number in the textbook
- `source_file`: String - Original source file name
- `text_preview`: String - Short preview of the text content (first 200 chars)
- `vector_id`: String - ID in the vector database (Qdrant)
- `created_at`: DateTime - Timestamp when the reference was created
- `updated_at`: DateTime - Timestamp when the reference was last updated

**Relationships**:
- None (standalone metadata table)

### ChatSession

Represents a user session for tracking conversations with the RAG chatbot.

**Fields**:
- `session_id`: UUID (Primary Key) - Unique identifier for the session
- `user_id`: String (Optional) - Identifies the user (for analytics)
- `created_at`: DateTime - Timestamp when the session started
- `updated_at`: DateTime - Timestamp when the session was last updated
- `active`: Boolean - Whether the session is currently active

**Relationships**:
- One-to-many with ChatMessage

### ChatMessage

Represents a message within a chat session, including the retrieved context and response.

**Fields**:
- `message_id`: UUID (Primary Key) - Unique identifier for the message
- `session_id`: UUID - Reference to the parent session
- `role`: String (Values: "user", "assistant") - Who sent the message
- `content`: String - The actual message content
- `retrieved_chunks`: Array of ChunkReference IDs - IDs of chunks used in response
- `selected_text_used`: Boolean - Whether user-selected text was used as context
- `confidence_score`: Float (0.0-1.0) - Confidence level of the response
- `created_at`: DateTime - Timestamp when the message was created

**Relationships**:
- Many-to-one with ChatSession

### QueryResponseLog

Logs for tracking queries and responses for analysis and debugging.

**Fields**:
- `log_id`: UUID (Primary Key) - Unique identifier for the log entry
- `query`: String - The original query from the user
- `response`: String - The system's response
- `retrieved_chunks`: Array of ChunkReference IDs - IDs of chunks used in response
- `query_type`: String (Values: "general", "selected_text") - Type of query
- `response_time`: Float - Time in seconds to generate response
- `confidence_level`: Float (0.0-1.0) - Confidence level of the response
- `refused_answer`: Boolean - Whether the system refused to answer
- `created_at`: DateTime - Timestamp when the log entry was created

**Relationships**:
- None (standalone log table)

## Validation Rules

### ContentChunk Validation
- `text_content` must be between 10 and 4000 characters
- `chunk_id` must be unique
- `embedding` must have a consistent dimension (1536 for Google embedding model)
- `metadata` must include chapter and section information

### ChunkReference Validation
- `chunk_id` must be unique
- `chapter` and `section` must not be empty
- `vector_id` must be present

### ChatMessage Validation
- `role` must be either "user" or "assistant"
- `session_id` must reference an existing session
- `retrieved_chunks` may be empty only if `selected_text_used` is true

## State Transitions

### ChatSession
- `active: false` → `active: true` when user starts a new session
- `active: true` → `active: false` when session times out or user ends session

## Relationships

```
ChatSession (1) → (0..n) ChatMessage
ChatMessage may reference (0..n) ChunkReference
QueryResponseLog may reference (0..n) ChunkReference
```

## Indexes

### ChunkReference
- Index on (chapter, section) for efficient content lookup
- Index on source_file for file-based queries

### ChatMessage
- Index on session_id for session-based queries
- Index on created_at for chronological queries

### QueryResponseLog
- Index on created_at for chronological queries
- Index on query_type for filtering by query type
- Index on refused_answer for performance analysis