"""
Test script to verify the Qwen agent functionality after setup.
"""
import requests
import json
import sys
import os

# Add backend to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def test_agent_setup():
    """Test the agent setup and functionality."""
    print("Testing Qwen Agent Setup...")
    print("="*50)
    
    # Test if environment is configured
    from src.config.settings import settings
    
    print("Checking environment configuration...")
    if settings.QDRANT_URL and "your-actual" not in settings.QDRANT_URL:
        print("✅ Qdrant URL is configured")
    else:
        print("❌ Qdrant URL needs to be configured")
        
    if settings.QDRANT_API_KEY and "your-actual" not in settings.QDRANT_API_KEY:
        print("✅ Qdrant API Key is configured")
    else:
        print("❌ Qdrant API Key needs to be configured")
        
    if settings.GEMINI_API_KEY and "your-" not in settings.GEMINI_API_KEY:
        print("✅ Gemini API Key is configured")
    else:
        print("❌ Gemini API Key needs to be configured")
    
    print()
    
    # Test Qdrant connection
    print("Testing Qdrant connection...")
    try:
        from src.vector_store.qdrant_client import qdrant_client
        
        if qdrant_client.health_check():
            print("✅ Qdrant connection successful")
            
            # Check collection
            count = qdrant_client.count_vectors()
            print(f"✅ Collection contains {count} vectors")
            
            if count > 0:
                print("✅ Textbook content has been populated")
            else:
                print("⚠️  Textbook content has not been populated yet")
                print("   Run 'python populate_textbook_qdrant.py' in the backend directory")
        else:
            print("❌ Qdrant connection failed")
            print("   Check your QDRANT_URL and QDRANT_API_KEY in the .env file")
    except Exception as e:
        print(f"❌ Error testing Qdrant connection: {e}")
    
    print()
    
    # Test embedding service
    print("Testing embedding service...")
    try:
        from src.services.embedding_service import embedding_service
        test_embedding = embedding_service.embed_single_text("test")
        if test_embedding and len(test_embedding) > 0:
            print("✅ Embedding service is working")
        else:
            print("❌ Embedding service is not working properly")
    except Exception as e:
        print(f"❌ Error testing embedding service: {e}")
    
    print()
    
    print("Setup Status Summary:")
    print("- Update .env file with your Qdrant and Gemini credentials")
    print("- Run populate_textbook_qdrant.py to populate Qdrant with textbook content")
    print("- Start the backend server with: python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload")
    print("- Test the agent functionality")
    print()
    print("For detailed instructions, see SETUP_INSTRUCTIONS.md")

if __name__ == "__main__":
    test_agent_setup()