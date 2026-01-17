import requests
import json
import time

def test_backend_connection():
    """Test the backend connection and basic functionality."""
    backend_url = "http://127.0.0.1:8001"
    
    print("Testing backend connection...")
    
    # Test health endpoint
    try:
        health_response = requests.get(f"{backend_url}/health")
        print(f"Health check: {health_response.status_code}")
        if health_response.status_code == 200:
            print("✓ Backend is healthy")
        else:
            print(f"✗ Backend health check failed with status {health_response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Cannot connect to backend: {e}")
        return False
    
    # Test chat/ask endpoint with a simple question
    try:
        test_payload = {"question": "What is Physical AI?"}
        headers = {"Content-Type": "application/json"}
        
        chat_response = requests.post(
            f"{backend_url}/chat/ask",
            data=json.dumps(test_payload),
            headers=headers
        )
        
        print(f"Chat endpoint test: {chat_response.status_code}")
        
        if chat_response.status_code == 200:
            response_data = chat_response.json()
            print("✓ Chat endpoint is working")
            print(f"Response preview: {response_data.get('answer', 'No answer field')[:100]}...")
        else:
            print(f"✗ Chat endpoint failed with status {chat_response.status_code}")
            print(f"Response: {chat_response.text}")
            return False
            
    except Exception as e:
        print(f"✗ Error testing chat endpoint: {e}")
        return False
    
    print("\n✓ All tests passed! Backend is properly configured.")
    return True

if __name__ == "__main__":
    test_backend_connection()