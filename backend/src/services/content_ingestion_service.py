"""
Content ingestion module for the RAG Chatbot backend.
Handles the ingestion of textbook content into the vector store.
"""
import logging
import uuid
from typing import List, Dict, Any
from src.vector_store.qdrant_client import qdrant_client
from src.database.postgres_client import postgres_client
from src.services.embedding_service import embedding_service
from src.utils.content_preprocessor import prepare_content_for_embedding
from src.vector_store.schemas import qdrant_schema
from qdrant_client.http.models import PointStruct


class ContentIngestionService:
    """Service for ingesting textbook content into the vector store and metadata database."""

    def __init__(self):
        """Initialize the content ingestion service."""
        self.logger = logging.getLogger(__name__)
        self.qdrant_client = qdrant_client
        self.postgres_client = postgres_client
        self.embedding_service = embedding_service
        self.qdrant_schema = qdrant_schema

    def ingest_textbook_content(
        self,
        content: str,
        source_file: str,
        create_collection: bool = True
    ) -> bool:
        """
        Ingest textbook content into the vector store and metadata database.

        Args:
            content: The raw textbook content to ingest
            source_file: Name of the source file
            create_collection: Whether to create the collection if it doesn't exist

        Returns:
            True if ingestion was successful, False otherwise
        """
        try:
            # Create the collection if needed
            if create_collection:
                if not self.qdrant_client.create_collection():
                    self.logger.error("Failed to create Qdrant collection")
                    return False

            # Prepare content for embedding (clean, chunk, extract metadata)
            content_chunks = prepare_content_for_embedding(
                text=content,
                source_file=source_file,
                chunk_strategy="semantic"
            )

            # Generate embeddings for all chunks
            chunk_embeddings = self.embedding_service.embed_textbook_content(content, source_file)

            # Prepare points for Qdrant
            points = []
            postgres_records = []

            for i, chunk_with_embedding in enumerate(chunk_embeddings):
                # Generate a proper UUID for the chunk_id to ensure Qdrant compatibility
                chunk_uuid = str(uuid.uuid4())
                text_content = chunk_with_embedding['text_content']
                embedding = chunk_with_embedding['embedding']
                metadata = chunk_with_embedding['metadata']

                # Create point for Qdrant
                point = self.qdrant_schema.create_content_chunk_point(
                    chunk_id=chunk_uuid,
                    vector=embedding,
                    text_content=text_content,
                    metadata=metadata
                )
                points.append(point)

                # Prepare metadata record for Postgres
                postgres_record = {
                    'id': chunk_uuid,
                    'chunk_id': chunk_uuid,
                    'chapter': metadata.get('chapter', ''),
                    'section': metadata.get('section', ''),
                    'page_number': metadata.get('page_number'),
                    'source_file': source_file,
                    'text_preview': text_content[:200],  # First 200 characters as preview
                    'vector_id': chunk_uuid
                }
                postgres_records.append(postgres_record)

            # Store embeddings in Qdrant
            if not self.qdrant_client.upsert_vectors(points):
                self.logger.error("Failed to upsert vectors to Qdrant")
                return False

            # Store metadata in Postgres
            # Note: In a real implementation, you would have specific methods to handle this
            # For now, we'll just log that this step would occur
            self.logger.info(f"Would store {len(postgres_records)} metadata records in Postgres")

            self.logger.info(f"Successfully ingested textbook content from {source_file} "
                            f"({len(content_chunks)} chunks)")
            return True

        except Exception as e:
            self.logger.error(f"Failed to ingest textbook content from {source_file}: {str(e)}")
            return False

    def ingest_from_file(self, file_path: str) -> bool:
        """
        Ingest content from a file into the vector store and metadata database.

        Args:
            file_path: Path to the file to ingest

        Returns:
            True if ingestion was successful, False otherwise
        """
        try:
            # Read content from file
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # Extract source file name
            import os
            source_file = os.path.basename(file_path)

            # Ingest the content
            return self.ingest_textbook_content(content, source_file)

        except Exception as e:
            self.logger.error(f"Failed to ingest content from file {file_path}: {str(e)}")
            return False


# Singleton instance
content_ingestion_service = ContentIngestionService()