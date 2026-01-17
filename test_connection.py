import time
import requests

def test_connection():
    url = "http://127.0.0.1:8001/health"
    max_attempts = 30  # Wait up to 30 seconds
    for i in range(max_attempts):
        try:
            response = requests.get(url, timeout=1)
            print(f"✓ Connected! Status code: {response.status_code}")
            return True
        except requests.exceptions.RequestException:
            print(f"Attempt {i+1}: Still waiting for server to start...")
            time.sleep(1)
    
    print("✗ Failed to connect to the server after 30 seconds")
    return False

if __name__ == "__main__":
    test_connection()