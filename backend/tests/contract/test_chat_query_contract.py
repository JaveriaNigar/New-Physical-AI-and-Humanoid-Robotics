"""
Contract tests for the /chat/query endpoint in the RAG Chatbot backend.
Validates that the API conforms to the defined contract in contracts/chat-api.yaml.
"""
import pytest
from fastapi.testclient import TestClient
from src.main import app
import json


class TestChatQueryContract:
    """Test class for /chat/query endpoint contract validation."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.client = TestClient(app)

    def test_chat_query_endpoint_contract_success_response(self):
        """Test that successful response conforms to contract schema."""
        # Mock the services to return known data that matches the contract
        from unittest.mock import patch
        from src.services.retrieval_service import retrieval_service
        from src.services.generation_service import generation_service
        from src.models.chat import ChatQueryResponse, ChunkMetadata
        
        # Mock retrieval to return known chunks
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-1',
                    'text_content': 'Neural networks are computing systems inspired by the human brain...',
                    'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 127}
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            # Mock generation to return a response that matches the contract
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                mock_response = ChatQueryResponse(
                    response="Neural networks are computing systems inspired by the human brain...",
                    confidence=0.85,
                    retrieved_chunks=[
                        ChunkMetadata(
                            chunk_id='chunk-1',
                            chapter='Chapter 5',
                            section='5.1',
                            page_number=127,
                            text_preview='Neural networks are computing systems...'
                        )
                    ],
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
                
                # Validate response structure according to contract
                assert "response" in data
                assert "confidence" in data
                assert "retrieved_chunks" in data
                assert "sources" in data
                assert "session_id" in data
                assert "query_type" in data
                
                # Validate data types according to contract
                assert isinstance(data["response"], str)
                assert isinstance(data["confidence"], float)
                assert 0.0 <= data["confidence"] <= 1.0
                assert isinstance(data["retrieved_chunks"], list)
                assert isinstance(data["sources"], list)
                assert isinstance(data["session_id"], str)
                assert data["query_type"] in ["general", "selected_text"]

    def test_chat_query_endpoint_contract_refusal_response(self):
        """Test that refusal response conforms to contract schema."""
        # Mock retrieval to return no chunks, triggering refusal
        from unittest.mock import patch
        from src.services.retrieval_service import retrieval_service
        
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_retrieve.return_value = []  # No chunks found

            response = self.client.post(
                "/chat/query",
                json={
                    "question": "What is the meaning of life?",
                    "session_id": "test-session-456"
                }
            )

            # Should return 404 with refusal response
            assert response.status_code == 404
            data = response.json()
            
            # Validate refusal response structure according to contract
            assert "message" in data
            assert "reason" in data
            assert "session_id" in data
            
            # Validate data types according to contract
            assert isinstance(data["message"], str)
            assert isinstance(data["reason"], str)
            assert data["reason"] in ["no_context_found", "insufficient_context", "context_not_applicable"]
            assert isinstance(data["session_id"], str)

    def test_chat_query_endpoint_contract_bad_request(self):
        """Test that bad request response conforms to contract schema."""
        # Send a request that should fail validation
        response = self.client.post(
            "/chat/query",
            json={
                "question": "Hi",  # Too short, should fail validation
                "session_id": "test-session-789"
            }
        )

        # Should return 400 for bad request
        assert response.status_code == 400
        data = response.json()
        
        # Validate error response structure according to contract
        assert "detail" in data  # FastAPI standard error format

    def test_chat_query_endpoint_request_validation(self):
        """Test that request validation conforms to contract."""
        # Test missing required fields
        response = self.client.post(
            "/chat/query",
            json={
                # Missing required 'question' and 'session_id'
            }
        )

        # Should return 422 for validation error (missing required fields)
        assert response.status_code == 422
        data = response.json()
        
        # FastAPI standard validation error format
        assert "detail" in data

    def test_chat_query_endpoint_request_body_schema(self):
        """Test that request body conforms to contract schema."""
        # Test with valid request structure but mocked response
        from unittest.mock import patch
        from src.services.retrieval_service import retrieval_service
        from src.services.generation_service import generation_service
        from src.models.chat import ChatQueryResponse, ChunkMetadata
        
        with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
            mock_chunks = [
                {
                    'chunk_id': 'chunk-1',
                    'text_content': 'Test content',
                    'metadata': {'chapter': 'Chapter 1', 'section': '1.1', 'page_number': 1}
                }
            ]
            mock_retrieve.return_value = mock_chunks
            
            with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                mock_response = ChatQueryResponse(
                    response="Test response",
                    confidence=0.75,
                    retrieved_chunks=[
                        ChunkMetadata(
                            chunk_id='chunk-1',
                            chapter='Chapter 1',
                            section='1.1',
                            page_number=1,
                            text_preview='Test content'
                        )
                    ],
                    sources=["Chapter 1, 1.1"],
                    session_id="test-session-999",
                    query_type="general"
                )
                mock_generate.return_value = mock_response

                # Valid request according to contract
                response = self.client.post(
                    "/chat/query",
                    json={
                        "question": "What is artificial intelligence?",
                        "session_id": "test-session-999",
                        "context": {
                            "chapter": "Chapter 1",
                            "section": "1.1"
                        }
                    }
                )

                assert response.status_code == 200
                data = response.json()
                
                # Validate the response structure
                assert "response" in data
                assert len(data["response"]) > 0
                assert 0.0 <= data["confidence"] <= 1.0

    def test_chat_query_endpoint_response_schema_consistency(self):
        """Test that responses are consistent with the contract schema."""
        from unittest.mock import patch
        from src.services.retrieval_service import retrieval_service
        from src.services.generation_service import generation_service
        from src.models.chat import ChatQueryResponse, ChunkMetadata
        
        # Test with various types of responses to ensure schema consistency
        test_cases = [
            {
                "question": "What are neural networks?",
                "expected_contains": ["neural", "network"]
            },
            {
                "question": "Explain backpropagation",
                "expected_contains": ["backpropagation"]
            }
        ]
        
        for case in test_cases:
            with patch.object(retrieval_service, 'retrieve_relevant_chunks') as mock_retrieve:
                mock_chunks = [
                    {
                        'chunk_id': 'chunk-test',
                        'text_content': f'{case["question"]} is an important concept in AI.',
                        'metadata': {'chapter': 'Chapter 5', 'section': '5.1', 'page_number': 100}
                    }
                ]
                mock_retrieve.return_value = mock_chunks
                
                with patch.object(generation_service, 'generate_response_from_context') as mock_generate:
                    mock_response = ChatQueryResponse(
                        response=f'{case["question"]} is an important concept in AI.',
                        confidence=0.8,
                        retrieved_chunks=[
                            ChunkMetadata(
                                chunk_id='chunk-test',
                                chapter='Chapter 5',
                                section='5.1',
                                page_number=100,
                                text_preview=f'{case["question"]} is an important concept in AI.'
                            )
                        ],
                        sources=["Chapter 5, 5.1"],
                        session_id="test-session-consistent",
                        query_type="general"
                    )
                    mock_generate.return_value = mock_response

                    response = self.client.post(
                        "/chat/query",
                        json={
                            "question": case["question"],
                            "session_id": "test-session-consistent"
                        }
                    )

                    assert response.status_code == 200
                    data = response.json()
                    
                    # Validate all required fields are present according to contract
                    required_fields = ["response", "confidence", "retrieved_chunks", "sources", "session_id", "query_type"]
                    for field in required_fields:
                        assert field in data, f"Required field '{field}' missing from response"
                    
                    # Validate field types
                    assert isinstance(data["response"], str)
                    assert isinstance(data["confidence"], float)
                    assert 0.0 <= data["confidence"] <= 1.0
                    assert isinstance(data["retrieved_chunks"], list)
                    assert isinstance(data["sources"], list)
                    assert isinstance(data["session_id"], str)
                    assert isinstance(data["query_type"], str)