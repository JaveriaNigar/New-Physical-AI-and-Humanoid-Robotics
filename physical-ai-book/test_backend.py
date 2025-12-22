import requests
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def test_backend_endpoints():
    """Test the backend endpoints to ensure they work properly with selected text"""
    try:
        # Test the health endpoint first
        print("Testing health endpoint...")
        health_response = requests.get("http://localhost:8001/health")
        print(f"Health endpoint status: {health_response.status_code}")
        
        if health_response.status_code == 200:
            print("✓ Health endpoint is working")
        else:
            print(f"✗ Health endpoint failed with status {health_response.status_code}")
        
        # Test the chat ask endpoint with selected text
        print("\nTesting chat/ask endpoint with selected text...")
        test_data = {
            "question": "What is this text about?",
            "selectedText": "This is a sample text about artificial intelligence and machine learning."
        }
        
        chat_response = requests.post(
            "http://localhost:8001/chat/ask",
            json=test_data,
            headers={"Content-Type": "application/json"}
        )
        
        print(f"Chat ask endpoint status: {chat_response.status_code}")
        
        if chat_response.status_code == 200:
            response_json = chat_response.json()
            print(f"✓ Chat endpoint responded with: {response_json.get('answer', 'No answer field')[:100]}...")
        else:
            print(f"✗ Chat endpoint failed with status {chat_response.status_code}")
            print(f"Response: {chat_response.text}")
            
    except Exception as e:
        print(f"Error during testing: {str(e)}")
        print("This is expected if the backend server is not running.")

if __name__ == "__main__":
    test_backend_endpoints()