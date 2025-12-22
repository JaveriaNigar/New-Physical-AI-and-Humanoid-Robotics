"""
Qdrant schemas for the RAG Chatbot backend.
Defines the structure of data stored in the Qdrant vector database.
"""
from typing import Dict, Any, Optional
from pydantic import BaseModel
from qdrant_client.http.models import PointStruct, VectorParams
from src.config.constants import DEFAULT_EMBEDDING_DIMENSION


class ContentChunkPayload(BaseModel):
    """Payload structure for content chunks stored in Qdrant."""
    text_content: str
    metadata: Dict[str, Any]  # Additional metadata about the chunk
    chunk_id: str  # Unique identifier for the content chunk
    
    class Config:
        """Pydantic configuration."""
        # Allow extra fields in case of additional metadata
        extra = "allow"


class QdrantSchema:
    """Schema definitions for Qdrant collections."""
    
    @staticmethod
    def get_textbook_collection_config():
        """Get configuration for the textbook content collection."""
        return {
            "vector_size": DEFAULT_EMBEDDING_DIMENSION,
            "distance": "Cosine"  # Using cosine distance for similarity search
        }
    
    @staticmethod
    def create_content_chunk_point(
        chunk_id: str,
        vector: list,
        text_content: str,
        metadata: Dict[str, Any]
    ) -> PointStruct:
        """Create a PointStruct for storing a content chunk in Qdrant."""
        payload = {
            "text_content": text_content,
            "metadata": metadata,
            "chunk_id": chunk_id
        }
        
        return PointStruct(
            id=chunk_id,
            vector=vector,
            payload=payload
        )


# Create schema instance
qdrant_schema = QdrantSchema()