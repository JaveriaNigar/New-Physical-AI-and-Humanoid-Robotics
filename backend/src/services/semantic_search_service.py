"""
Semantic search implementation for the RAG Chatbot backend.
Handles searching for relevant content chunks with configurable similarity threshold.
"""
import logging
from typing import List, Dict, Any, Optional

from src.vector_store.qdrant_client import qdrant_client
from src.services.embedding_service import embedding_service
from src.config.settings import settings


class SemanticSearchService:
    """Service for performing semantic search with configurable similarity threshold."""
    
    def __init__(self):
        """Initialize the semantic search service."""
        self.logger = logging.getLogger(__name__)
        self.qdrant_client = qdrant_client
        self.embedding_service = embedding_service
    
    def search_similar(
        self,
        query: str,
        top_k: int = 10,
        min_similarity_score: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform semantic search for similar content chunks.
        
        Args:
            query: The query text to search for
            top_k: Number of top results to return
            min_similarity_score: Minimum similarity score threshold (uses default if None)
            
        Returns:
            List of similar content chunks with metadata and similarity scores
        """
        try:
            # Use default threshold from settings if not provided
            if min_similarity_score is None:
                min_similarity_score = settings.MIN_SIMILARITY_THRESHOLD
            
            # Generate embedding for the query
            query_embedding = self.embedding_service.embed_single_text(query)
            
            # Search for similar vectors in Qdrant
            results = self.qdrant_client.search_vectors(
                query_vector=query_embedding,
                limit=top_k,
                min_similarity_score=min_similarity_score
            )
            
            self.logger.info(f"Found {len(results)} similar chunks for query: '{query[:50]}...'")
            return results
            
        except Exception as e:
            self.logger.error(f"Error performing semantic search: {str(e)}")
            return []
    
    def search_with_context(
        self,
        query: str,
        context: Optional[Dict[str, Any]] = None,
        top_k: int = 10,
        min_similarity_score: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform semantic search with additional context filtering.
        
        Args:
            query: The query text to search for
            context: Additional context for filtering (e.g., chapter, section)
            top_k: Number of top results to return
            min_similarity_score: Minimum similarity score threshold (uses default if None)
            
        Returns:
            List of similar content chunks with metadata and similarity scores
        """
        try:
            # For now, implement basic search - in a full implementation we would 
            # add filtering based on the context parameters
            return self.search_similar(query, top_k, min_similarity_score)
            
        except Exception as e:
            self.logger.error(f"Error performing contextual semantic search: {str(e)}")
            return []


# Singleton instance
semantic_search_service = SemanticSearchService()