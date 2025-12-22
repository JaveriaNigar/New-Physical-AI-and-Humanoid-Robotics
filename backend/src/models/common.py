"""
Pydantic models for common data structures in the RAG Chatbot backend.
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import UUID
from datetime import datetime


class CommonRequest(BaseModel):
    """Base class for common request models."""
    pass


class CommonResponse(BaseModel):
    """Base class for common response models."""
    pass


class ChunkMetadata(BaseModel):
    """Metadata for content chunks used in responses."""
    chunk_id: str
    chapter: str
    section: str
    page_number: Optional[int] = None
    text_preview: Optional[str] = None


class ChatQueryRequest(BaseModel):
    """Request model for asking questions about textbook content."""
    question: str = Field(..., min_length=5, max_length=1000)
    session_id: str  # This could be a UUID string
    context: Optional[Dict[str, Any]] = None  # Additional context for the query


class SelectedTextQueryRequest(BaseModel):
    """Request model for asking questions with selected text context."""
    question: str = Field(..., min_length=5, max_length=1000)
    selected_text: str = Field(..., min_length=1, max_length=5000)
    session_id: str  # This could be a UUID string


class ChatQueryResponse(BaseModel):
    """Response model for chat queries."""
    response: str
    confidence: float = Field(ge=0.0, le=1.0)
    retrieved_chunks: List[ChunkMetadata]
    sources: List[str]
    session_id: str
    query_type: str = Field(default="general", pattern="^(general|selected_text)$")


class RefusalResponse(BaseModel):
    """Response model when the system refuses to answer."""
    message: str
    reason: str = Field(pattern="^(no_context_found|insufficient_context|context_not_applicable|selected_text_insufficient)$")
    session_id: str


class ErrorResponse(BaseModel):
    """Response model for API errors."""
    error: str
    message: str


class HealthResponse(BaseModel):
    """Response model for health checks."""
    status: str = Field(pattern="^(healthy|degraded|unhealthy)$")
    timestamp: datetime
    services: Optional[Dict[str, str]] = None  # Status of dependent services (qdrant, postgres, gemini)