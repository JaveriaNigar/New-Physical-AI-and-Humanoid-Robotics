"""
Chat-specific Pydantic models for the RAG Chatbot backend.
"""
from pydantic import BaseModel, Field, field_validator
from typing import List, Optional, Dict, Any
import re


class ChatQueryRequest(BaseModel):
    """Request model for asking questions about textbook content."""
    question: str = Field(..., min_length=5, max_length=1000)
    session_id: str  # This could be a UUID string
    context: Optional[Dict[str, Any]] = None  # Additional context for the query

    @field_validator('question')
    def validate_question(cls, v):
        """Validate question content."""
        if not v or len(v.strip()) == 0:
            raise ValueError('Question cannot be empty or just whitespace')
        if len(v) < 5:
            raise ValueError('Question must be at least 5 characters long')
        if len(v) > 1000:
            raise ValueError('Question must not exceed 1000 characters')
        # Check if question contains actual content (not just punctuation)
        if not re.search(r'[a-zA-Z0-9]', v):
            raise ValueError('Question must contain meaningful content')
        return v.strip()

    @field_validator('session_id')
    def validate_session_id(cls, v):
        """Validate session ID format."""
        if not v:
            raise ValueError('Session ID is required')
        return v


class SelectedTextQueryRequest(BaseModel):
    """Request model for asking questions with selected text context."""
    question: str = Field(..., min_length=5, max_length=1000)
    selected_text: str = Field(..., min_length=1, max_length=5000)
    session_id: str  # This could be a UUID string

    @field_validator('question')
    def validate_question(cls, v):
        """Validate question content."""
        if not v or len(v.strip()) == 0:
            raise ValueError('Question cannot be empty or just whitespace')
        if len(v) < 5:
            raise ValueError('Question must be at least 5 characters long')
        if len(v) > 1000:
            raise ValueError('Question must not exceed 1000 characters')
        # Check if question contains actual content (not just punctuation)
        if not re.search(r'[a-zA-Z0-9]', v):
            raise ValueError('Question must contain meaningful content')
        return v.strip()

    @field_validator('selected_text')
    def validate_selected_text(cls, v):
        """Validate selected text content."""
        if not v or len(v.strip()) == 0:
            raise ValueError('Selected text cannot be empty or just whitespace')
        if len(v) < 1:
            raise ValueError('Selected text must be at least 1 character long')
        if len(v) > 5000:
            raise ValueError('Selected text must not exceed 5000 characters')
        # Check if selected text contains actual content (not just punctuation)
        if not re.search(r'[a-zA-Z0-9]', v):
            raise ValueError('Selected text must contain meaningful content')

        # Check for potential security issues
        suspicious_patterns = [
            r'<script',  # Potential XSS
            r'eval\s*\(',  # Potential JavaScript eval
            r'exec\s*\(',  # Potential code execution
            r'<iframe',  # Potential iframe injection
        ]

        for pattern in suspicious_patterns:
            if re.search(pattern, v, re.IGNORECASE):
                raise ValueError(f'Selected text contains suspicious pattern: {pattern}')
        return v

    @field_validator('session_id')
    def validate_session_id(cls, v):
        """Validate session ID format."""
        if not v:
            raise ValueError('Session ID is required')
        return v


class ChunkMetadata(BaseModel):
    """Metadata for content chunks used in responses."""
    chunk_id: str
    chapter: str
    section: str
    page_number: Optional[int] = None
    text_preview: Optional[str] = None


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