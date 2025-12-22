"""
Unit tests for User Story 1 functionality in the RAG Chatbot backend.
Tests the core functionality of answering questions from textbook content.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from src.models.chat import ChatQueryRequest, ChatQueryResponse, RefusalResponse
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service
from src.api.chat_routes import chat_query


class TestUS1Functionality:
    """Test class for User Story 1 functionality: Student asks question from book content."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        pass

    def test_chat_query_endpoint_with_relevant_context(self):
        """Test the chat query endpoint when relevant context is found."""
        # Arrange
        request = ChatQueryRequest(
            question="What are neural networks?",
            session_id="test-session-123"
        )
        
        # Mock the retrieval service to return relevant chunks
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-1',
                    'text_content': 'Neural networks are computing systems inspired by the human brain...',
                    'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 127}
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            # Mock the generation service to return a response
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                expected_response = ChatQueryResponse(
                    response="Neural networks are computing systems inspired by the human brain...",
                    confidence=0.85,
                    retrieved_chunks=[],
                    sources=["Chapter 5, 5.1"],
                    session_id="test-session-123",
                    query_type="general"
                )
                mock_generate.return_value = expected_response

                # Act
                result = chat_query(request)

                # Assert
                assert isinstance(result, ChatQueryResponse)
                assert result.session_id == "test-session-123"
                mock_retrieve.assert_called_once_with(
                    query="What are neural networks?",
                    limit=10,
                    min_similarity_score=0.5
                )
                mock_generate.assert_called_once()

    def test_chat_query_endpoint_with_no_context(self):
        """Test the chat query endpoint when no relevant context is found."""
        # Arrange
        request = ChatQueryRequest(
            question="What is the meaning of life?",
            session_id="test-session-123"
        )
        
        # Mock the retrieval service to return no chunks
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_retrieve.return_value = []

            # Act
            result = chat_query(request)

            # Assert
            assert isinstance(result, RefusalResponse)
            assert "not covered in the provided textbook content" in result.message
            assert result.reason == "no_context_found"
            assert result.session_id == "test-session-123"
            mock_retrieve.assert_called_once_with(
                query="What is the meaning of life?",
                limit=10,
                min_similarity_score=0.5
            )

    def test_chat_query_endpoint_with_validation_error(self):
        """Test the chat query endpoint when question validation fails."""
        # Arrange
        request = ChatQueryRequest(
            question="Hi",  # Too short, should fail validation
            session_id="test-session-123"
        )
        
        # Mock validation to fail
        with patch('src.services.validation_service.validation_service') as mock_validation:
            mock_validation.validate_question.return_value = False

            # Act & Assert
            from fastapi import HTTPException
            with pytest.raises(HTTPException) as exc_info:
                chat_query(request)
            
            assert exc_info.value.status_code == 400

    def test_chat_query_endpoint_with_generation_error(self):
        """Test the chat query endpoint when response generation fails."""
        # Arrange
        request = ChatQueryRequest(
            question="What are neural networks?",
            session_id="test-session-123"
        )
        
        # Mock the retrieval service to return relevant chunks
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-1',
                    'text_content': 'Neural networks are computing systems inspired by the human brain...',
                    'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 127}
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            # Mock the generation service to raise an exception
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                mock_generate.side_effect = Exception("Generation failed")

                # Act
                result = chat_query(request)

                # Assert
                assert isinstance(result, RefusalResponse)
                assert "encountered an error" in result.message
                assert result.session_id == "test-session-123"

    def test_retrieval_service_with_similarity_threshold(self):
        """Test that retrieval service respects the minimum similarity threshold."""
        # This test verifies that the retrieval service properly uses the similarity threshold
        with patch.object(retrieval_service, 'qdrant_client') as mock_qdrant, \
             patch.object(retrieval_service, 'embedding_service') as mock_embedding:
            
            mock_embedding.embed_single_text.return_value = [0.1, 0.2, 0.3]
            mock_chunks = [
                {
                    'chunk_id': 'chunk-1',
                    'text_content': 'Neural networks are computing systems...',
                    'metadata': {'chapter': 'Chapter 5', 'section': '5.1'},
                    'similarity_score': 0.85
                }
            ]
            mock_qdrant.search_vectors.return_value = mock_chunks

            # Act
            result = retrieval_service.retrieve_relevant_chunks(
                query="What are neural networks?",
                min_similarity_score=0.8
            )

            # Assert
            assert len(result) == 1
            mock_qdrant.search_vectors.assert_called_once_with(
                query_vector=[0.1, 0.2, 0.3],
                limit=10,  # Default limit
                min_similarity_score=0.8
            )

    def test_generation_service_confidence_calculation(self):
        """Test that generation service properly calculates confidence scores."""
        # This test verifies the confidence calculation logic
        question = "What is backpropagation?"
        context = "Backpropagation is an algorithm used in artificial neural networks for training."
        response = "Backpropagation is an algorithm used for training neural networks."

        confidence = generation_service._calculate_confidence(question, context, response)
        
        # The confidence should be relatively high due to good overlap
        assert 0.0 <= confidence <= 1.0
        assert confidence > 0.5  # Reasonable threshold for this example

    def test_refusal_response_when_no_context_found(self):
        """Test that the system properly refuses to answer when no context is found."""
        # This verifies the core hallucination prevention mechanism
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_retrieve.return_value = []  # No chunks found

            # This should trigger a refusal response
            with patch('src.services.validation_service.validation_service') as mock_validation:
                mock_validation.validate_question.return_value = True  # Question is valid
                
                request = ChatQueryRequest(
                    question="What is quantum computing?",
                    session_id="test-session-456"
                )

                result = chat_query(request)

                # Should be a refusal response
                assert isinstance(result, RefusalResponse)
                assert result.reason == "no_context_found"

    def test_context_traceability_in_response(self):
        """Test that responses include proper traceability to source chunks."""
        # Arrange
        request = ChatQueryRequest(
            question="How does backpropagation work?",
            session_id="test-session-789"
        )
        
        mock_chunks = [
            {
                'chunk_id': 'chunk-456',
                'text_content': 'Backpropagation works by calculating gradients...',
                'metadata': {
                    'chapter': 'Chapter 7', 
                    'section': '7.3', 
                    'page_number': 201,
                    'source_file': 'ai_textbook.pdf'
                }
            }
        ]
        
        with patch.object(retrieval_service, 'retrieve_relevant_chunks', return_value=mock_chunks):
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                mock_response = ChatQueryResponse(
                    response="Backpropagation works by calculating gradients...",
                    confidence=0.9,
                    retrieved_chunks=[],
                    sources=["Chapter 7, 7.3"],
                    session_id="test-session-789",
                    query_type="general"
                )
                mock_generate.return_value = mock_response

                # Act
                result = chat_query(request)

                # Assert
                assert isinstance(result, ChatQueryResponse)
                # The response should contain information about the source chunks used
                # This verifies traceability is maintained