"""
Database models for the RAG Chatbot backend.
Defines the structure of data stored in the Postgres database.
"""
# This file contains the database models that were already defined in postgres_client.py
# For organization purposes, we'll keep the model definitions there and this file can be used
# for any additional database-related models or helper functions if needed

# Import models from postgres_client to make them easily accessible
from .postgres_client import ChunkReference, ChatSession, ChatMessage, QueryResponseLog, Base
from sqlalchemy import Column, String, Integer, DateTime, Boolean, Text, ARRAY
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func


class ContentChunk:
    """Represents a semantically coherent piece of textbook content that has been vectorized for retrieval.

    This is a conceptual model - actual content chunks are stored in Qdrant,
    but metadata about them is stored in Postgres using ChunkReference.
    """
    # This is a conceptual class that represents the structure for content chunks
    # The actual implementation is in Qdrant, with metadata in ChunkReference

    chunk_id: str  # Primary Key - Unique identifier for the content chunk
    text_content: str  # The actual text content of the chunk
    # embedding: List[float]  # Vector representation stored in Qdrant
    metadata: dict  # Additional information about the chunk
    created_at: str  # Timestamp when the chunk was created


# Additional model-related utility functions can go here if needed

__all__ = ["ChunkReference", "ChatSession", "ChatMessage", "QueryResponseLog", "Base", "ContentChunk"]