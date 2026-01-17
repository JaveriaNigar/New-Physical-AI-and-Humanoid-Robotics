import time
import requests

def test_connection():
    url = 'http://127.0.0.1:8001/health'
    max_attempts = 30  # Wait up to 30 seconds
    for i in range(max_attempts):
        try:
            response = requests.get(url, timeout=5)
            print(f'Connected! Status code: {response.status_code}')
            return True
        except requests.exceptions.RequestException as e:
            if i < max_attempts - 1:
                print(f'Attempt {i+1}: Still waiting for server to start...')
                time.sleep(1)
            else:
                print(f'Failed to connect to the server: {e}')
                return False

if __name__ == "__main__":
    result = test_connection()
    if result:
        print('Backend server is running successfully!')
    else:
        print('Backend server failed to start.')