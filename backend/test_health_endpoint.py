"""
Test script to test the health endpoint using FastAPI test client
"""
import sys
from pathlib import Path

# Add the src directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.main import app
from fastapi.testclient import TestClient
import json

# Create a test client
client = TestClient(app)

# Test the health endpoint
response = client.get("/health")

print(f"Health endpoint status code: {response.status_code}")
if response.status_code == 200:
    print(f"Health endpoint response: {json.dumps(response.json(), indent=2, default=str)}")
else:
    print(f"Health endpoint error: {response.text}")