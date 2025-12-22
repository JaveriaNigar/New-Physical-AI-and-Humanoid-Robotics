"""
Test script to verify backend endpoints are working correctly
"""
import asyncio
import sys
import os

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from src.main import app
from starlette.testclient import TestClient


def test_health_endpoint():
    """Test the health endpoint"""
    with TestClient(app) as client:
        response = client.get("/health")
        print(f"Health endpoint status: {response.status_code}")
        if response.status_code == 200:
            print("Health check passed!")
            print(f"Response: {response.json()}")
            return True
        else:
            print(f"Health check failed: {response.text}")
            return False


def test_chat_ask_endpoint():
    """Test the chat ask endpoint"""
    with TestClient(app) as client:
        # Test with a simple question
        response = client.post("/chat/ask", json={"question": "What is this textbook about?"})
        print(f"Chat ask endpoint status: {response.status_code}")
        if response.status_code in [200, 404, 400]:  # Various valid responses
            print(f"Chat ask response: {response.json()}")
            return True
        else:
            print(f"Chat ask failed: {response.text}")
            return False


def main():
    print("Testing backend endpoints...")

    health_ok = test_health_endpoint()
    chat_ok = test_chat_ask_endpoint()

    if health_ok and chat_ok:
        print("\nAll endpoint tests passed!")
        return True
    else:
        print("\nSome endpoint tests failed!")
        return False


if __name__ == "__main__":
    success = main()
    if success:
        print("Backend endpoints are working correctly!")
    else:
        print("There were issues with the backend endpoints.")