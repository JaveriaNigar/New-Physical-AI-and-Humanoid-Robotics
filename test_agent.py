import requests
import json

def test_agent_functionality():
    """Test the agent functionality by querying the backend."""
    backend_url = "http://127.0.0.1:8001"
    
    print("Testing agent functionality...")
    
    # Test the /chat/ask endpoint with a question about the textbook
    try:
        test_payload = {"question": "What is Physical AI?"}
        headers = {"Content-Type": "application/json"}
        
        response = requests.post(
            f"{backend_url}/chat/ask",
            data=json.dumps(test_payload),
            headers=headers
        )
        
        print(f"Status Code: {response.status_code}")
        
        if response.status_code == 200:
            response_data = response.json()
            print("✓ Agent responded successfully!")
            print(f"Answer: {response_data.get('answer', 'No answer field')}")
        elif response.status_code == 422:
            print("✗ Request validation error - incorrect payload format")
            print(f"Response: {response.text}")
        else:
            print(f"✗ Agent failed with status {response.status_code}")
            print(f"Response: {response.text}")
            
    except Exception as e:
        print(f"✗ Error testing agent functionality: {e}")
        import traceback
        traceback.print_exc()

    # Test health endpoint as well
    try:
        health_response = requests.get(f"{backend_url}/health")
        print(f"Health check: {health_response.status_code}")
    except Exception as e:
        print(f"✗ Health check failed: {e}")

if __name__ == "__main__":
    test_agent_functionality()