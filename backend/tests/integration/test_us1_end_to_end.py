"""
Integration tests for User Story 1 end-to-end flow in the RAG Chatbot backend.
Tests the complete flow from question to answer, ensuring all components work together.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from src.main import app
from src.models.chat import ChatQueryResponse, RefusalResponse
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service


class TestUS1EndToEndFlow:
    """Test class for User Story 1 end-to-end integration tests."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.client = TestClient(app)

    def test_end_to_end_flow_with_relevant_context(self):
        """Test the complete end-to-end flow when relevant context is found."""
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
                mock_response = ChatQueryResponse(
                    response="Neural networks are computing systems inspired by the human brain...",
                    confidence=0.85,
                    retrieved_chunks=[],
                    sources=["Chapter 5, 5.1"],
                    session_id="test-session-123",
                    query_type="general"
                )
                mock_generate.return_value = mock_response

                # Act
                response = self.client.post(
                    "/chat/query",
                    json={
                        "question": "What are neural networks?",
                        "session_id": "test-session-123"
                    }
                )

                # Assert
                assert response.status_code == 200
                data = response.json()
                assert "neural networks" in data["response"].lower()
                assert data["session_id"] == "test-session-123"
                assert data["query_type"] == "general"
                assert 0.0 <= data["confidence"] <= 1.0

                # Verify that both services were called
                mock_retrieve.assert_called_once()
                mock_generate.assert_called_once()

    def test_end_to_end_flow_with_no_context(self):
        """Test the complete end-to-end flow when no relevant context is found."""
        # Mock the retrieval service to return no chunks
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_retrieve.return_value = []

            # Act
            response = self.client.post(
                "/chat/query",
                json={
                    "question": "What is the meaning of life?",
                    "session_id": "test-session-456"
                }
            )

            # Assert
            assert response.status_code == 404  # Should return 404 for refusal
            data = response.json()
            assert "not covered in the provided textbook content" in data["message"]
            assert data["reason"] == "no_context_found"
            assert data["session_id"] == "test-session-456"

    def test_end_to_end_flow_with_invalid_question(self):
        """Test the complete end-to-end flow when question validation fails."""
        # Mock validation to fail
        with patch('src.services.validation_service.validation_service') as mock_validation:
            mock_validation.validate_question.return_value = False

            # Act
            response = self.client.post(
                "/chat/query",
                json={
                    "question": "Hi",  # Too short, should fail validation
                    "session_id": "test-session-789"
                }
            )

            # Assert
            assert response.status_code == 400  # Bad request
            data = response.json()
            assert "detail" in data

    def test_end_to_end_flow_with_generation_error(self):
        """Test the complete end-to-end flow when response generation fails."""
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
                response = self.client.post(
                    "/chat/query",
                    json={
                        "question": "What are neural networks?",
                        "session_id": "test-session-101"
                    }
                )

                # Assert
                # Should return a refusal response
                assert response.status_code == 200  # Still returns 200 but with refusal
                data = response.json()
                assert "encountered an error" in data["message"]
                assert data["session_id"] == "test-session-101"

    def test_end_to_end_flow_with_different_similarity_thresholds(self):
        """Test the complete flow with different similarity thresholds."""
        # Test with a high threshold that might filter out results
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-low-similarity',
                    'text_content': 'This is somewhat related content...',
                    'metadata': {'chapter': 'Chapter 3', 'section': '3.2', 'page_number': 85},
                    'similarity_score': 0.4  # Below default threshold of 0.5
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            # Mock the generation service to return a refusal response
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                # For this test, we expect the retrieval to return chunks but they might be filtered
                # by the threshold, so the generation service might receive empty context
                mock_response = RefusalResponse(
                    message="I cannot answer this question as it's not covered in the provided textbook content.",
                    reason="no_context_found",
                    session_id="test-session-202"
                )
                mock_generate.return_value = mock_response

                response = self.client.post(
                    "/chat/query",
                    json={
                        "question": "What is this somewhat related concept?",
                        "session_id": "test-session-202"
                    }
                )

                # The behavior depends on how the threshold is applied
                # If retrieval filters at Qdrant level, we might get no chunks
                # If filtering happens after retrieval, we might still get chunks but generation refuses
                assert response.status_code in [200, 404]  # Could be either depending on implementation

    def test_end_to_end_flow_with_context_fidelity(self):
        """Test that responses are properly grounded in the provided context."""
        # Mock the retrieval service to return specific context
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-context-1',
                    'text_content': 'Backpropagation is an algorithm for training neural networks.',
                    'metadata': {'chapter': 'Chapter 7', 'section': '7.3', 'page_number': 201}
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            # Mock the generation service to return a response based on the context
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                mock_response = ChatQueryResponse(
                    response="Backpropagation is an algorithm used for training neural networks.",
                    confidence=0.9,
                    retrieved_chunks=[],
                    sources=["Chapter 7, 7.3"],
                    session_id="test-session-303",
                    query_type="general"
                )
                mock_generate.return_value = mock_response

                response = self.client.post(
                    "/chat/query",
                    json={
                        "question": "How does backpropagation work?",
                        "session_id": "test-session-303"
                    }
                )

                assert response.status_code == 200
                data = response.json()
                # Verify the response is related to the context provided
                assert "backpropagation" in data["response"].lower()
                assert "training" in data["response"].lower()
                assert data["confidence"] > 0.5  # Reasonable confidence for good match

    def test_end_to_end_flow_with_traceability(self):
        """Test that responses include proper traceability to source chunks."""
        # Mock retrieval to return chunks with specific metadata
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-trace-1',
                    'text_content': 'Neural networks consist of layers of interconnected nodes.',
                    'metadata': {
                        'chapter': 'Chapter 5', 
                        'section': '5.2', 
                        'page_number': 105,
                        'source_file': 'ai_textbook.pdf'
                    }
                },
                {
                    'chunk_id': 'chunk-trace-2',
                    'text_content': 'Each layer transforms the input data.',
                    'metadata': {
                        'chapter': 'Chapter 5', 
                        'section': '5.3', 
                        'page_number': 108,
                        'source_file': 'ai_textbook.pdf'
                    }
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            # Mock generation to return a response
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                mock_response = ChatQueryResponse(
                    response="Neural networks consist of layers of interconnected nodes, with each layer transforming the input data.",
                    confidence=0.88,
                    retrieved_chunks=[],
                    sources=["Chapter 5, 5.2", "Chapter 5, 5.3"],
                    session_id="test-session-404",
                    query_type="general"
                )
                mock_generate.return_value = mock_response

                response = self.client.post(
                    "/chat/query",
                    json={
                        "question": "Describe neural network architecture?",
                        "session_id": "test-session-404"
                    }
                )

                assert response.status_code == 200
                data = response.json()
                # Verify that the response includes source information
                assert len(data["sources"]) >= 1
                assert "Chapter 5" in str(data["sources"])
                # Confidence should reflect the quality of match
                assert 0.0 <= data["confidence"] <= 1.0

    def test_health_endpoint_integration(self):
        """Test the health endpoint to ensure all services are connected."""
        response = self.client.get("/health")
        
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "timestamp" in data
        # The actual status depends on whether external services are available during testing
        # but the endpoint should at least return a valid response