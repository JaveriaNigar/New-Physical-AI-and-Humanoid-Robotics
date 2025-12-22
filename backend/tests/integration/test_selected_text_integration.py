"""
Integration tests for the selected-text functionality in the RAG Chatbot backend.
Tests the complete end-to-end flow for selected-text queries.
"""
import pytest
import asyncio
from unittest.mock import AsyncMock, patch
from fastapi.testclient import TestClient

from src.main import app
from src.models.chat import SelectedTextQueryRequest
from src.services.retrieval_service import retrieval_service
from src.services.generation_service import generation_service


@pytest.fixture
def client():
    """Create a test client for the FastAPI application."""
    return TestClient(app)


@pytest.mark.asyncio
class TestSelectedTextIntegration:
    """Integration tests for the selected-text end-to-end flow."""
    
    async def test_selected_text_end_to_end_flow(self, client):
        """Test the complete end-to-end flow for selected-text queries."""
        # Define test data
        test_request = SelectedTextQueryRequest(
            question="What is ROS?",
            selected_text="The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.",
            session_id="test-session-123"
        )
        
        # Mock the generation service to return a predefined response
        with patch.object(generation_service, 'generate_response_from_selected_text') as mock_gen_method:
            mock_gen_method.return_value = ChatQueryResponse(
                response="ROS is a flexible framework for writing robot software.",
                confidence=0.9,
                retrieved_chunks=[],
                sources=["Selected Text"],
                session_id=test_request.session_id,
                query_type="selected_text"
            )
            
            # Make the request to the API
            response = client.post(
                "/chat/selected-text",
                json={
                    "question": test_request.question,
                    "selected_text": test_request.selected_text,
                    "session_id": test_request.session_id
                }
            )
            
            # Verify the response
            assert response.status_code == 200
            data = response.json()
            assert "ROS is a flexible framework" in data["response"]
            assert data["session_id"] == test_request.session_id
            assert data["query_type"] == "selected_text"
            assert 0.0 <= data["confidence"] <= 1.0
            
            # Verify that the generation service was called with correct parameters
            mock_gen_method.assert_called_once_with(
                question=test_request.question,
                selected_text=test_request.selected_text,
                session_id=test_request.session_id
            )
    
    async def test_selected_text_with_insufficient_text(self, client):
        """Test the flow when selected text is insufficient to answer the question."""
        # Create a request with insufficient selected text
        test_request = SelectedTextQueryRequest(
            question="What is the detailed architecture of ROS 2?",
            selected_text="ROS is a framework.",
            session_id="test-session-456"
        )
        
        # Mock the generation service to return a refusal response
        with patch.object(generation_service, 'generate_response_from_selected_text') as mock_gen_method:
            mock_gen_method.return_value = RefusalResponse(
                message="Cannot answer this question as the selected text does not contain sufficient information.",
                reason="selected_text_insufficient",
                session_id=test_request.session_id
            )
            
            # Make the request to the API
            response = client.post(
                "/chat/selected-text",
                json={
                    "question": test_request.question,
                    "selected_text": test_request.selected_text,
                    "session_id": test_request.session_id
                }
            )
            
            # Verify the response is a refusal response
            assert response.status_code == 200  # 200 because refusal is a valid response
            data = response.json()
            assert "Cannot answer" in data["message"]
            assert data["reason"] == "selected_text_insufficient"
            assert data["session_id"] == test_request.session_id
    
    async def test_selected_text_validation_error(self, client):
        """Test the flow when validation fails."""
        # Send a request with empty question and selected text
        response = client.post(
            "/chat/selected-text",
            json={
                "question": "",  # Invalid input
                "selected_text": "",  # Invalid input
                "session_id": "test-session-789"
            }
        )
        
        # Should return a validation error (422 for Pydantic validation error)
        # or 400 for custom validation
        assert response.status_code in [400, 422]
    
    async def test_selected_text_generation_failure_handling(self, client):
        """Test that the system handles generation failures gracefully."""
        test_request = SelectedTextQueryRequest(
            question="What is ROS?",
            selected_text="The Robot Operating System (ROS) is a flexible framework.",
            session_id="test-session-000"
        )
        
        # Mock the generation service to raise an exception
        with patch.object(generation_service, 'generate_response_from_selected_text') as mock_gen_method:
            mock_gen_method.side_effect = Exception("API error")
            
            # Make the request to the API
            response = client.post(
                "/chat/selected-text",
                json={
                    "question": test_request.question,
                    "selected_text": test_request.selected_text,
                    "session_id": test_request.session_id
                }
            )
            
            # Should return an internal server error (500)
            assert response.status_code == 500
    
    async def test_selected_text_flow_with_real_services_stubbed(self, client):
        """Test the flow with retrieval and generation services properly coordinated."""
        test_request = SelectedTextQueryRequest(
            question="How does ROS handle messaging?",
            selected_text="In ROS, messaging is handled through topics, services, and actions. Topics use a publish-subscribe pattern.",
            session_id="test-session-999"
        )
        
        # Mock the services to simulate the flow
        with patch.object(retrieval_service, 'retrieve_chunks_by_strategy') as mock_retrieve:
            with patch.object(generation_service, 'generate_response_from_selected_text') as mock_generate:
                # For selected text, retrieval bypasses vector DB, so we expect specific call
                mock_generate.return_value = ChatQueryResponse(
                    response="ROS handles messaging through topics using a publish-subscribe pattern.",
                    confidence=0.85,
                    retrieved_chunks=[],
                    sources=["Selected Text"],
                    session_id=test_request.session_id,
                    query_type="selected_text"
                )
                
                response = client.post(
                    "/chat/selected-text",
                    json={
                        "question": test_request.question,
                        "selected_text": test_request.selected_text,
                        "session_id": test_request.session_id
                    }
                )
                
                # Verify response
                assert response.status_code == 200
                data = response.json()
                
                assert "messaging" in data["response"].lower()
                assert data["session_id"] == test_request.session_id
                assert abs(data["confidence"] - 0.85) < 0.01  # Account for potential rounding
                
                # Verify the generation service was called with the right parameters
                mock_generate.assert_called_once_with(
                    question=test_request.question,
                    selected_text=test_request.selected_text,
                    session_id=test_request.session_id
                )


# Additional tests for edge cases

class TestSelectedTextEdgeCases:
    """Tests for edge cases in the selected-text functionality."""
    
    def test_selected_text_with_special_characters(self, client):
        """Test handling of selected text with special characters."""
        test_request = SelectedTextQueryRequest(
            question="What about special chars?",
            selected_text="Text with special chars: àáâãäåæçèéêë, также 中文 and эмодзи: 😀🚀.",
            session_id="test-session-special"
        )
        
        with patch.object(generation_service, 'generate_response_from_selected_text') as mock_gen_method:
            mock_gen_method.return_value = ChatQueryResponse(
                response="The text contains special characters as requested.",
                confidence=0.7,
                retrieved_chunks=[],
                sources=["Selected Text"],
                session_id=test_request.session_id,
                query_type="selected_text"
            )
            
            response = client.post(
                "/chat/selected-text",
                json={
                    "question": test_request.question,
                    "selected_text": test_request.selected_text,
                    "session_id": test_request.session_id
                }
            )
            
            assert response.status_code == 200
            data = response.json()
            assert "special characters" in data["response"].lower()
    
    def test_very_long_selected_text(self, client):
        """Test handling of very long selected text."""
        long_text = "This is a very long text. " + " ".join([f"word{i}" for i in range(1000)]) + " End of long text."
        test_request = SelectedTextQueryRequest(
            question="What is this long text about?",
            selected_text=long_text,
            session_id="test-session-long"
        )
        
        with patch.object(generation_service, 'generate_response_from_selected_text') as mock_gen_method:
            mock_gen_method.return_value = ChatQueryResponse(
                response="This is a very long text with many words.",
                confidence=0.6,
                retrieved_chunks=[],
                sources=["Selected Text"],
                session_id=test_request.session_id,
                query_type="selected_text"
            )
            
            response = client.post(
                "/chat/selected-text",
                json={
                    "question": test_request.question,
                    "selected_text": test_request.selected_text,
                    "session_id": test_request.session_id
                }
            )
            
            assert response.status_code == 200
            data = response.json()
            assert "long text" in data["response"].lower()