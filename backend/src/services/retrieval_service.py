"""
Retrieval service for the RAG Chatbot backend.
Handles retrieving relevant content chunks from the vector database based on user queries.
"""
import logging
from typing import List, Optional
from src.vector_store.qdrant_client import qdrant_client
from src.services.embedding_service import embedding_service
from src.config.settings import settings
from src.config.constants import MAX_CONTENT_CHUNKS_PER_RETRIEVAL


class RetrievalService:
    """Service for retrieving relevant content chunks based on user queries."""

    def __init__(self):
        """Initialize the retrieval service."""
        self.logger = logging.getLogger(__name__)
        self.qdrant_client = qdrant_client
        self.embedding_service = embedding_service

    def retrieve_relevant_chunks(
        self,
        query: str,
        limit: int = MAX_CONTENT_CHUNKS_PER_RETRIEVAL,
        min_similarity_score: float = 0.35  # Lowered from 0.5 to improve recall
    ) -> List[dict]:
        """
        Retrieve relevant content chunks for a given query.

        Args:
            query: The user's query
            limit: Maximum number of chunks to retrieve
            min_similarity_score: Minimum similarity score for retrieval

        Returns:
            List of relevant content chunks with metadata
        """
        try:
            # Generate embedding for the query
            query_embedding = self.embedding_service.embed_single_text(query)

            # Search for similar vectors in Qdrant
            results = self.qdrant_client.search_vectors(
                query_vector=query_embedding,
                limit=limit,
                min_similarity_score=min_similarity_score
            )

            self.logger.info(f"Retrieved {len(results)} relevant chunks for query: {query[:50]}...")
            return results

        except Exception as e:
            self.logger.error(f"Failed to retrieve relevant chunks for query '{query[:50]}...': {str(e)}")
            return []

    def retrieve_with_selected_text(
        self,
        selected_text: str
    ) -> List[dict]:
        """
        Return chunks based on selected text context only (not from vector database).

        Args:
            selected_text: The text that the user has selected/highlighted

        Returns:
            List with a single chunk representing the selected text
        """
        # For selected text, we create a virtual chunk that represents the selected content
        selected_chunk = {
            'chunk_id': 'selected_text_chunk',
            'text_content': selected_text,
            'metadata': {
                'source': 'user_selected_text',
                'type': 'selected_text'
            },
            'similarity_score': 1.0  # Perfect match since it's the selected text
        }

        self.logger.info(f"Created virtual chunk for selected text of length {len(selected_text)}")
        return [selected_chunk]

    def retrieve_chunks_by_strategy(
        self,
        query: str,
        selected_text: str = None,
        limit: int = MAX_CONTENT_CHUNKS_PER_RETRIEVAL,
        min_similarity_score: float = 0.5
    ) -> List[dict]:
        """
        Retrieve chunks using appropriate strategy. If selected_text is provided,
        bypass the vector database and return virtual chunks for the selected text.
        Otherwise, perform semantic search in the vector database.

        Args:
            query: The user's query
            selected_text: The text that the user has selected/highlighted (optional)
            limit: Maximum number of chunks to retrieve
            min_similarity_score: Minimum similarity score for retrieval

        Returns:
            List of relevant content chunks with metadata
        """
        if selected_text:
            # Bypass vector database when selected text is provided
            self.logger.info("Using selected text retrieval strategy (bypassing vector DB)")
            return self.retrieve_with_selected_text(selected_text)
        else:
            # Use vector database for general queries
            self.logger.info("Using semantic search retrieval strategy")
            return self.retrieve_relevant_chunks(query, limit, min_similarity_score)

    def validate_retrieval_config(self) -> bool:
        """
        Validate that the retrieval service is properly configured.

        Returns:
            True if properly configured, False otherwise
        """
        try:
            # Check if Qdrant client is available
            if not self.qdrant_client.health_check():
                self.logger.error("Qdrant client not available")
                return False

            # Check if embedding service is available
            test_embedding = self.embedding_service.embed_single_text("test")
            if not test_embedding or len(test_embedding) == 0:
                self.logger.error("Embedding service not available")
                return False

            self.logger.info("Retrieval service configuration is valid")
            return True

        except Exception as e:
            self.logger.error(f"Failed to validate retrieval service configuration: {str(e)}")
            return False


# Singleton instance
retrieval_service = RetrievalService()