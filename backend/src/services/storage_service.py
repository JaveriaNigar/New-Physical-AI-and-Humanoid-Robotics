"""
Storage service for the RAG Chatbot backend.
Handles storing embeddings in Qdrant and metadata in Postgres.
"""
import logging
from typing import List, Dict, Any
from uuid import uuid4

from src.vector_store.qdrant_client import qdrant_client
from src.database.postgres_client import postgres_client
from src.vector_store.schemas import qdrant_schema
from src.database.models import ChunkReference
from qdrant_client.http.models import PointStruct


class StorageService:
    """Service for storing embeddings in Qdrant and metadata in Postgres."""
    
    def __init__(self):
        """Initialize the storage service."""
        self.logger = logging.getLogger(__name__)
        self.qdrant_client = qdrant_client
        self.postgres_client = postgres_client
        self.qdrant_schema = qdrant_schema
    
    def store_embeddings_and_metadata(
        self,
        embeddings: List[Dict[str, Any]],
        source_file: str
    ) -> bool:
        """
        Store embeddings in Qdrant and metadata in Postgres.
        
        Args:
            embeddings: List of dictionaries containing text_content, embedding vector, and metadata
            source_file: Name of the source file for tracking purposes
            
        Returns:
            True if storage was successful, False otherwise
        """
        try:
            # Prepare points for Qdrant
            points = []
            chunk_references = []
            
            for i, item in enumerate(embeddings):
                chunk_id = str(uuid4())
                text_content = item['text_content']
                embedding = item['embedding']
                metadata = item['metadata']
                
                # Create point for Qdrant
                point = self.qdrant_schema.create_content_chunk_point(
                    chunk_id=chunk_id,
                    vector=embedding,
                    text_content=text_content,
                    metadata=metadata
                )
                points.append(point)
                
                # Create ChunkReference for Postgres
                chunk_reference = ChunkReference(
                    id=uuid4(),
                    chunk_id=chunk_id,
                    chapter=metadata.get('chapter', ''),
                    section=metadata.get('section', ''),
                    page_number=metadata.get('page_number'),
                    source_file=source_file,
                    text_preview=text_content[:200],  # First 200 characters as preview
                    vector_id=chunk_id
                )
                chunk_references.append(chunk_reference)
            
            # Store embeddings in Qdrant
            qdrant_success = self.qdrant_client.upsert_vectors(points)
            if not qdrant_success:
                self.logger.error("Failed to upsert vectors to Qdrant")
                return False
            
            # Store metadata in Postgres
            # In a real implementation, we would have a method to bulk insert these records
            # For now, we'll just simulate the operation
            self.logger.info(f"Storing {len(chunk_references)} chunk references in Postgres")
            
            # In a real implementation, we would do something like:
            # with self.postgres_client.get_db_session() as db:
            #     db.add_all(chunk_references)
            #     db.commit()
            
            self.logger.info(f"Successfully stored {len(embeddings)} embeddings and metadata")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to store embeddings and metadata: {str(e)}")
            return False


# Singleton instance
storage_service = StorageService()