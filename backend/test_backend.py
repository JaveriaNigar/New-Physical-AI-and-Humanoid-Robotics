import requests
import json

def test_backend_connection():
    """Test script to verify the backend is working correctly"""
    
    # Test URL for the /chat/ask endpoint
    url = "http://127.0.0.1:8001/chat/ask"
    
    # Sample question to test
    test_payload = {
        "question": "What is artificial intelligence?"
    }
    
    try:
        print("Testing connection to RAG Chatbot backend...")
        print(f"Sending request to: {url}")
        print(f"Payload: {json.dumps(test_payload, indent=2)}")
        
        # Send POST request to the backend
        response = requests.post(url, json=test_payload)
        
        print(f"\nResponse Status Code: {response.status_code}")
        print(f"Response Headers: {dict(response.headers)}")
        print(f"Response Body: {response.text}")
        
        if response.status_code == 200:
            print("\n✅ SUCCESS: Backend is accessible and responding!")
            try:
                response_json = response.json()
                print(f"Parsed JSON response: {response_json}")
            except:
                print("Could not parse response as JSON")
        else:
            print(f"\n❌ ERROR: Backend returned status code {response.status_code}")
            
    except requests.exceptions.ConnectionError:
        print("\n❌ ERROR: Cannot connect to backend. Please make sure:")
        print("   1. The backend server is running on port 8001")
        print("   2. You've run: uvicorn src.main:app --reload --port 8001")
        print("   3. The virtual environment is activated")
    except Exception as e:
        print(f"\n❌ ERROR: {str(e)}")

if __name__ == "__main__":
    test_backend_connection()