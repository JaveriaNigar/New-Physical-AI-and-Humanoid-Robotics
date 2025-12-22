"""
Test script to verify Qdrant connectivity.
"""
import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

def test_qdrant_connectivity():
    print("Testing Qdrant connectivity...")
    
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/.env")
    
    # Import required modules
    from qdrant_client import QdrantClient
    from src.config.settings import settings
    
    print(f"QDRANT_URL from settings: {settings.QDRANT_URL}")
    print(f"QDRANT_API_KEY available: {'Yes' if settings.QDRANT_API_KEY else 'No'}")
    print(f"QDRANT_COLLECTION_NAME: {settings.QDRANT_COLLECTION_NAME}")
    
    if settings.QDRANT_URL == "your-qdrant-cluster-url-here" or settings.QDRANT_API_KEY == "your-qdrant-api-key-here":
        print("❌ Qdrant configuration contains placeholder values!")
        print("Please update your .env file with actual Qdrant credentials.")
        return False
    
    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=10  # 10 seconds timeout
        )
        
        # Test connection by getting collections
        collections = client.get_collections()
        print(f"✅ Connected to Qdrant successfully!")
        print(f"Available collections: {[col.name for col in collections.collections]}")
        
        # Check if our collection exists
        collection_names = [col.name for col in collections.collections]
        if settings.QDRANT_COLLECTION_NAME in collection_names:
            print(f"✅ Collection '{settings.QDRANT_COLLECTION_NAME}' exists!")
            
            # Get collection info
            collection_info = client.get_collection(settings.QDRANT_COLLECTION_NAME)
            print(f"Collection points count: {collection_info.point_count}")
            print(f"Collection vectors count: {collection_info.vectors_count}")
            
            if collection_info.point_count > 0:
                print("✅ Collection contains vectors!")
            else:
                print("⚠️  Collection exists but is empty - needs to be populated with textbook content")
        else:
            print(f"❌ Collection '{settings.QDRANT_COLLECTION_NAME}' not found!")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to connect to Qdrant: {str(e)}")
        return False

if __name__ == "__main__":
    print("Running Qdrant Connectivity Test...\n")
    success = test_qdrant_connectivity()
    
    if success:
        print("\n🎉 Qdrant connectivity test PASSED!")
        print("The Qwen agent should be able to retrieve textbook content.")
    else:
        print("\n❌ Qdrant connectivity test FAILED!")
        print("Please update your .env file with correct Qdrant credentials.")