"""
Qdrant vector database client implementation for the RAG Chatbot backend.
Handles all interactions with the Qdrant vector store for textbook content.
"""
from typing import List, Optional, Dict, Any
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from src.config.settings import settings
from src.config.constants import DEFAULT_EMBEDDING_DIMENSION, QDRANT_TIMEOUT_SECONDS


class QdrantManager:
    """Manages all interactions with the Qdrant vector database."""

    def __init__(self):
        """Initialize the Qdrant client with configuration from settings."""
        # Initialize Qdrant client
        if settings.QDRANT_URL.startswith('https://') or settings.QDRANT_URL.startswith('http://'):
            # Using Qdrant Cloud with API key
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=QDRANT_TIMEOUT_SECONDS,
                check_compatibility=False
            )
        else:
            # Using local Qdrant instance
            self.client = QdrantClient(host=settings.QDRANT_URL, port=6333, check_compatibility=False)

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.embedding_dimension = DEFAULT_EMBEDDING_DIMENSION
        self.logger = logging.getLogger(__name__)

    def create_collection(self) -> bool:
        """Create the collection for textbook content if it doesn't exist."""
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            existing_collection = next((c for c in collections.collections if c.name == self.collection_name), None)

            if existing_collection:
                self.logger.info(f"Collection '{self.collection_name}' already exists")
                return True

            # Create new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.embedding_dimension,
                    distance=Distance.COSINE
                )
            )

            self.logger.info(f"Created collection '{self.collection_name}' successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to create collection '{self.collection_name}': {str(e)}")
            return False

    def upsert_vectors(self, points: List[PointStruct]) -> bool:
        """Upsert vectors into the collection."""
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            self.logger.info(f"Upserted {len(points)} vectors successfully")
            return True
        except Exception as e:
            self.logger.error(f"Failed to upsert vectors: {str(e)}")
            return False

    def search_vectors(
        self,
        query_vector: List[float],
        limit: int = 10,
        min_similarity_score: float = 0.5
    ) -> List[Dict[str, Any]]:
        """Search for similar vectors in the collection."""
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit,
                score_threshold=min_similarity_score  # Minimum similarity score
            )

            # Format results to return
            formatted_results = []
            for result in results.points:
                formatted_results.append({
                    'chunk_id': result.id,
                    'text_content': result.payload.get('text_content', ''),
                    'metadata': result.payload.get('metadata', {}),
                    'similarity_score': result.score
                })

            self.logger.info(f"Found {len(formatted_results)} similar vectors")
            return formatted_results

        except Exception as e:
            self.logger.error(f"Failed to search vectors: {str(e)}")
            return []

    def delete_by_ids(self, ids: List[str]) -> bool:
        """Delete points by their IDs."""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=ids
                )
            )
            self.logger.info(f"Deleted {len(ids)} vectors by IDs")
            return True
        except Exception as e:
            self.logger.error(f"Failed to delete vectors by IDs: {str(e)}")
            return False

    def count_vectors(self) -> int:
        """Get the total number of vectors in the collection."""
        try:
            count = self.client.count(
                collection_name=self.collection_name
            )
            return count.count
        except Exception as e:
            self.logger.error(f"Failed to count vectors: {str(e)}")
            return 0

    def health_check(self) -> bool:
        """Perform a health check on the Qdrant connection."""
        try:
            # Attempt to get collections to verify connection
            self.client.get_collections()
            return True
        except Exception:
            return False


# Singleton instance
qdrant_client = QdrantManager()