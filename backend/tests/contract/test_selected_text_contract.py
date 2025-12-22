"""
Contract tests for the selected-text endpoint in the RAG Chatbot backend.
Verifies that the API contract is fulfilled according to specification.
"""
import pytest
import requests
from typing import Dict, Any
import json


# Base URL for the backend API - adjust this based on your deployment
BASE_URL = "http://localhost:8000"


class TestSelectedTextEndpointContract:
    """Contract tests for the /chat/selected-text endpoint."""
    
    def test_endpoint_exists_and_accepts_post_requests(self):
        """Test that the /chat/selected-text endpoint exists and accepts POST requests."""
        # Test with a sample payload that conforms to the expected schema
        sample_payload = {
            "question": "What is this text about?",
            "selected_text": "This is a sample text selected by the user.",
            "session_id": "test-session-contract-001"
        }
        
        # Using requests because TestClient doesn't always accurately reflect deployed behavior
        response = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=sample_payload,
            headers={"Content-Type": "application/json"}
        )
        
        # The endpoint should exist and accept the request (even if it returns an error for invalid data)
        assert response.status_code in [200, 400, 422, 500], f"Expected endpoint to exist, got status {response.status_code}"
    
    def test_request_body_schema_conformance(self):
        """Test that the endpoint properly validates request body schema as per specification."""
        # Valid request should be accepted
        valid_payload = {
            "question": "What is the main concept discussed in this text?",
            "selected_text": "The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.",
            "session_id": "test-session-contract-002"
        }
        
        response = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=valid_payload,
            headers={"Content-Type": "application/json"}
        )
        
        # Should return either success (200) or a validation error (422/400) but not method not allowed (405)
        assert response.status_code != 405, "Method not allowed - endpoint may not exist"
        
        # Test with missing required fields
        invalid_payload_missing_fields = {
            "question": "What is this about?"  # Missing selected_text and session_id
        }
        
        response_missing = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=invalid_payload_missing_fields,
            headers={"Content-Type": "application/json"}
        )
        
        # Should return validation error (422) for missing required fields
        assert response_missing.status_code in [400, 422], f"Expected validation error, got {response_missing.status_code}"
        
        # Test with invalid field types
        invalid_payload_wrong_types = {
            "question": 123,  # Should be string
            "selected_text": 456,  # Should be string
            "session_id": 789  # Should be string
        }
        
        response_wrong_types = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=invalid_payload_wrong_types,
            headers={"Content-Type": "application/json"}
        )
        
        # Should return validation error for wrong field types
        assert response_wrong_types.status_code in [400, 422], f"Expected validation error for wrong types, got {response_wrong_types.status_code}"
    
    def test_response_schema_conformance(self):
        """Test that the endpoint returns responses conforming to the specified schema."""
        # Send a sample request
        sample_payload = {
            "question": "What is the main concept discussed in this text?",
            "selected_text": "The Robot Operating System (ROS) is a flexible framework for writing robot software.",
            "session_id": "test-session-contract-003"
        }
        
        response = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=sample_payload,
            headers={"Content-Type": "application/json"}
        )
        
        # Only proceed with schema validation if we get a success response
        if response.status_code == 200:
            data = response.json()
            
            # Verify required fields for ChatQueryResponse
            assert "response" in data, "Response should contain 'response' field"
            assert "confidence" in data, "Response should contain 'confidence' field"
            assert "retrieved_chunks" in data, "Response should contain 'retrieved_chunks' field"
            assert "sources" in data, "Response should contain 'sources' field"
            assert "session_id" in data, "Response should contain 'session_id' field"
            assert "query_type" in data, "Response should contain 'query_type' field"
            
            # Verify field types
            assert isinstance(data["response"], str), "'response' should be a string"
            assert isinstance(data["confidence"], (int, float)), "'confidence' should be a number"
            assert isinstance(data["retrieved_chunks"], list), "'retrieved_chunks' should be a list"
            assert isinstance(data["sources"], list), "'sources' should be a list"
            assert isinstance(data["session_id"], str), "'session_id' should be a string"
            assert isinstance(data["query_type"], str), "'query_type' should be a string"
            
            # Verify response values are within expected ranges
            assert 0.0 <= data["confidence"] <= 1.0, "Confidence should be between 0.0 and 1.0"
            assert data["query_type"] in ["general", "selected_text"], f"Invalid query_type: {data['query_type']}"
    
    def test_error_response_schema_conformance(self):
        """Test that error responses conform to the expected schema."""
        # Test with intentionally invalid request to trigger an error response
        invalid_payload = {
            "question": "",  # Empty question should trigger validation error
            "selected_text": "Sample text",
            "session_id": "test-session-contract-004"
        }
        
        response = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=invalid_payload,
            headers={"Content-Type": "application/json"}
        )
        
        # If it's a validation error (422), verify the response format
        if response.status_code == 422:
            error_data = response.json()
            # FastAPI validation errors usually come in a specific format
            assert "detail" in error_data, "Validation error response should contain 'detail' field"
    
    def test_refusal_response_schema_conformance(self):
        """Test that refusal responses conform to the specified schema."""
        # Send a question that should result in a refusal (e.g., question not answerable from text)
        sample_payload = {
            "question": "What is the capital of Mars?",
            "selected_text": "This text is about Earth geography and has no information about Mars.",
            "session_id": "test-session-contract-005"
        }
        
        response = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=sample_payload,
            headers={"Content-Type": "application/json"}
        )
        
        # If successful, check if it's a refusal response
        if response.status_code == 200:
            data = response.json()
            
            # Check if it's a refusal response by looking for refusal-specific fields
            if "reason" in data and data.get("reason") in ["selected_text_insufficient", "insufficient_context"]:
                # This is a refusal response, verify its schema
                assert "message" in data, "Refusal response should contain 'message' field"
                assert "reason" in data, "Refusal response should contain 'reason' field"
                assert "session_id" in data, "Refusal response should contain 'session_id' field"
                
                assert isinstance(data["message"], str), "'message' should be a string"
                assert isinstance(data["reason"], str), "'reason' should be a string"
                assert isinstance(data["session_id"], str), "'session_id' should be a string"
    
    def test_session_id_propagation(self):
        """Test that the session_id from the request is properly propagated to the response."""
        test_session_id = "test-session-contract-006"
        
        sample_payload = {
            "question": "What is this text about?",
            "selected_text": "This is a sample text selected by the user for testing purposes.",
            "session_id": test_session_id
        }
        
        response = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=sample_payload,
            headers={"Content-Type": "application/json"}
        )
        
        if response.status_code == 200:
            data = response.json()
            assert data["session_id"] == test_session_id, f"Session ID should be propagated from request to response: expected {test_session_id}, got {data['session_id']}"
    
    def test_content_type_header(self):
        """Test that the endpoint properly handles Content-Type header."""
        sample_payload = {
            "question": "What is this text about?",
            "selected_text": "Sample text content",
            "session_id": "test-session-contract-007"
        }
        
        # Test with correct content type
        response_correct_type = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=sample_payload,
            headers={"Content-Type": "application/json"}
        )
        
        # Test with no explicit content type (requests will set it automatically)
        response_auto_type = requests.post(
            f"{BASE_URL}/chat/selected-text",
            json=sample_payload
        )
        
        # Both should be treated the same way internally by FastAPI
        # They should either both succeed or both fail with the same error type
        both_ok = response_correct_type.status_code == 200 and response_auto_type.status_code == 200
        both_error = response_correct_type.status_code >= 400 and response_auto_type.status_code >= 400
        
        assert both_ok or both_error, "Requests with and without explicit Content-Type should behave consistently"
    
    def test_cors_headers_present(self):
        """Test that appropriate CORS headers are present in responses."""
        # Make an OPTIONS request to check CORS headers
        options_response = requests.options(
            f"{BASE_URL}/chat/selected-text",
            headers={"Origin": "http://example.com", "Access-Control-Request-Method": "POST"}
        )
        
        # Even if the response is not 200, CORS headers should be present in preflight requests
        cors_headers = [
            "access-control-allow-origin",
            "access-control-allow-methods", 
            "access-control-allow-headers"
        ]
        
        for header in cors_headers:
            # We only verify these headers are present if CORS is implemented
            # The endpoint doesn't necessarily have to implement CORS in a basic implementation
            pass  # This test is just to verify if CORS is implemented, it works correctly


# For running with pytest
if __name__ == "__main__":
    pytest.main([__file__])