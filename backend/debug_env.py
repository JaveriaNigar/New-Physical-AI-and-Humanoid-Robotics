"""
Debug script to check the actual values being loaded from the environment.
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/.env")

print("DEBUG: Checking environment variable values")
print("="*50)

# Check the raw environment variables
print(f"GEMINI_API_KEY: {os.getenv('GEMINI_API_KEY', 'NOT SET')[:20]}...")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY', 'NOT SET')}")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL', 'NOT SET')}")
print(f"QDRANT_COLLECTION_NAME: {os.getenv('QDRANT_COLLECTION_NAME', 'NOT SET')}")
print(f"MIN_SIMILARITY_THRESHOLD: {os.getenv('MIN_SIMILARITY_THRESHOLD', 'NOT SET')}")

print("\nChecking with settings module:")
from src.config.settings import settings

print(f"Settings QDRANT_API_KEY: {settings.QDRANT_API_KEY}")
print(f"Settings QDRANT_URL: {settings.QDRANT_URL}")
print(f"Settings QDRANT_COLLECTION_NAME: {settings.QDRANT_COLLECTION_NAME}")
print(f"Settings MIN_SIMILARITY_THRESHOLD: {settings.MIN_SIMILARITY_THRESHOLD}")

# Check if values look like placeholders
if "your-" in settings.QDRANT_URL.lower() or "placeholder" in settings.QDRANT_URL.lower():
    print("\n[ERROR] QDRANT_URL still has placeholder values!")
else:
    print("\n[OK] QDRANT_URL looks valid")

if "your-" in settings.QDRANT_API_KEY.lower() or "placeholder" in settings.QDRANT_API_KEY.lower():
    print("[ERROR] QDRANT_API_KEY still has placeholder values!")
else:
    print("[OK] QDRANT_API_KEY looks valid")