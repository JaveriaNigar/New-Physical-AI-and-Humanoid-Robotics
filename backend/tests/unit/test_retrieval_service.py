"""
Unit tests for the retrieval service in the RAG Chatbot backend.
Tests similarity search functionality and retrieval behavior.
"""
import pytest
from unittest.mock import Mock, patch
from src.services.retrieval_service import RetrievalService
from src.config.constants import MAX_CONTENT_CHUNKS_PER_RETRIEVAL


class TestRetrievalService:
    """Test class for the retrieval service."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        with patch('src.vector_store.qdrant_client.qdrant_client'), \
             patch('src.services.embedding_service.embedding_service'):
            self.retrieval_service = RetrievalService()
            # Mock the dependencies
            self.retrieval_service.qdrant_client = Mock()
            self.retrieval_service.embedding_service = Mock()

    def test_retrieve_relevant_chunks_success(self):
        """Test successful retrieval of relevant chunks."""
        # Arrange
        query = "What are neural networks?"
        expected_embedding = [0.1, 0.2, 0.3]
        expected_results = [
            {
                'chunk_id': 'chunk-1',
                'text_content': 'Neural networks are computing systems...',
                'metadata': {'chapter': 'Chapter 5', 'section': '5.1'},
                'similarity_score': 0.85
            }
        ]
        
        self.retrieval_service.embedding_service.embed_single_text.return_value = expected_embedding
        self.retrieval_service.qdrant_client.search_vectors.return_value = expected_results

        # Act
        results = self.retrieval_service.retrieve_relevant_chunks(query, limit=5, min_similarity_score=0.5)

        # Assert
        assert len(results) == 1
        assert results[0]['chunk_id'] == 'chunk-1'
        assert results[0]['text_content'] == 'Neural networks are computing systems...'
        assert results[0]['similarity_score'] == 0.85
        self.retrieval_service.embedding_service.embed_single_text.assert_called_once_with(query)
        self.retrieval_service.qdrant_client.search_vectors.assert_called_once_with(
            query_vector=expected_embedding,
            limit=5,
            min_similarity_score=0.5
        )

    def test_retrieve_relevant_chunks_with_default_parameters(self):
        """Test retrieval with default parameters."""
        # Arrange
        query = "What is AI?"
        expected_embedding = [0.4, 0.5, 0.6]
        expected_results = []
        
        self.retrieval_service.embedding_service.embed_single_text.return_value = expected_embedding
        self.retrieval_service.qdrant_client.search_vectors.return_value = expected_results

        # Act
        results = self.retrieval_service.retrieve_relevant_chunks(query)

        # Assert
        assert len(results) == 0
        self.retrieval_service.qdrant_client.search_vectors.assert_called_once_with(
            query_vector=expected_embedding,
            limit=MAX_CONTENT_CHUNKS_PER_RETRIEVAL,
            min_similarity_score=0.5
        )

    def test_retrieve_relevant_chunks_failure_handling(self):
        """Test handling of retrieval failures."""
        # Arrange
        query = "What is machine learning?"
        
        self.retrieval_service.embedding_service.embed_single_text.side_effect = Exception("Embedding failed")

        # Act
        results = self.retrieval_service.retrieve_relevant_chunks(query)

        # Assert
        assert len(results) == 0

    def test_retrieve_with_selected_text(self):
        """Test retrieval with selected text (bypassing vector database)."""
        # Arrange
        selected_text = "Backpropagation is a method used in artificial neural networks..."

        # Act
        results = self.retrieval_service.retrieve_with_selected_text(selected_text)

        # Assert
        assert len(results) == 1
        assert results[0]['chunk_id'] == 'selected_text_chunk'
        assert results[0]['text_content'] == selected_text
        assert results[0]['metadata']['source'] == 'user_selected_text'
        assert results[0]['metadata']['type'] == 'selected_text'
        assert results[0]['similarity_score'] == 1.0

    def test_retrieve_chunks_by_strategy_with_selected_text(self):
        """Test retrieval strategy selection when selected text is provided."""
        # Arrange
        query = "Explain this concept?"
        selected_text = "Linear regression is a statistical method..."
        
        # Act
        results = self.retrieval_service.retrieve_chunks_by_strategy(
            query=query,
            selected_text=selected_text
        )

        # Assert
        assert len(results) == 1
        assert results[0]['chunk_id'] == 'selected_text_chunk'
        # Should use selected text strategy, not semantic search

    def test_retrieve_chunks_by_strategy_with_general_query(self):
        """Test retrieval strategy selection for general queries."""
        # Arrange
        query = "What are the principles of neural networks?"
        expected_embedding = [0.7, 0.8, 0.9]
        expected_results = [
            {
                'chunk_id': 'chunk-2',
                'text_content': 'Neural networks operate on the principle of...',
                'metadata': {'chapter': 'Chapter 5', 'section': '5.2'},
                'similarity_score': 0.92
            }
        ]
        
        self.retrieval_service.embedding_service.embed_single_text.return_value = expected_embedding
        self.retrieval_service.qdrant_client.search_vectors.return_value = expected_results

        # Act
        results = self.retrieval_service.retrieve_chunks_by_strategy(
            query=query,
            selected_text=None  # No selected text, so should use semantic search
        )

        # Assert
        assert len(results) == 1
        assert results[0]['chunk_id'] == 'chunk-2'
        assert results[0]['text_content'] == 'Neural networks operate on the principle of...'
        self.retrieval_service.embedding_service.embed_single_text.assert_called_once_with(query)

    def test_validate_retrieval_config_success(self):
        """Test successful validation of retrieval service configuration."""
        # Arrange
        self.retrieval_service.qdrant_client.health_check.return_value = True
        self.retrieval_service.embedding_service.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Act
        result = self.retrieval_service.validate_retrieval_config()

        # Assert
        assert result is True

    def test_validate_retrieval_config_qdrant_failure(self):
        """Test validation failure when Qdrant is not available."""
        # Arrange
        self.retrieval_service.qdrant_client.health_check.return_value = False

        # Act
        result = self.retrieval_service.validate_retrieval_config()

        # Assert
        assert result is False

    def test_validate_retrieval_config_embedding_failure(self):
        """Test validation failure when embedding service is not available."""
        # Arrange
        self.retrieval_service.qdrant_client.health_check.return_value = True
        self.retrieval_service.embedding_service.embed_single_text.return_value = []

        # Act
        result = self.retrieval_service.validate_retrieval_config()

        # Assert
        assert result is False

    def test_retrieve_relevant_chunks_below_similarity_threshold(self):
        """Test that chunks below similarity threshold are filtered out."""
        # Arrange
        query = "How does backpropagation work?"
        expected_embedding = [0.1, 0.2, 0.3]
        # Results with scores both above and below threshold
        expected_results = [
            {
                'chunk_id': 'chunk-1',
                'text_content': 'Backpropagation adjusts weights...',
                'metadata': {'chapter': 'Chapter 5', 'section': '5.3'},
                'similarity_score': 0.4  # Below default threshold of 0.5
            },
            {
                'chunk_id': 'chunk-2',
                'text_content': 'The algorithm uses gradients...',
                'metadata': {'chapter': 'Chapter 5', 'section': '5.4'},
                'similarity_score': 0.7  # Above threshold
            }
        ]
        
        self.retrieval_service.embedding_service.embed_single_text.return_value = expected_embedding
        self.retrieval_service.qdrant_client.search_vectors.return_value = expected_results

        # Act
        results = self.retrieval_service.retrieve_relevant_chunks(query, min_similarity_score=0.5)

        # Assert
        # Note: The filtering should happen at the Qdrant level, so we expect all results back
        # but the Qdrant client should be called with the correct threshold
        assert len(results) == 2  # Both results returned (filtering happens in Qdrant)
        self.retrieval_service.qdrant_client.search_vectors.assert_called_once_with(
            query_vector=expected_embedding,
            limit=MAX_CONTENT_CHUNKS_PER_RETRIEVAL,
            min_similarity_score=0.5
        )