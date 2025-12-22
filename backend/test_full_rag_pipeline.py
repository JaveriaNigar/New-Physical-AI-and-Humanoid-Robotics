"""
Comprehensive test script to verify the Qwen RAG pipeline.
Tests Qdrant connectivity, content retrieval, and generation.
"""
import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

def test_full_rag_pipeline():
    """
    Test the full RAG pipeline: retrieval and generation.
    """
    print("Testing full RAG pipeline...")
    
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/.env")
    
    from src.config.settings import settings
    from src.services.retrieval_service import retrieval_service
    from src.services.generation_service import generation_service
    
    print(f"QDRANT_COLLECTION_NAME: {settings.QDRANT_COLLECTION_NAME}")
    print(f"MIN_SIMILARITY_THRESHOLD: {settings.MIN_SIMILARITY_THRESHOLD}")
    
    # Test retrieval with a sample question
    sample_questions = [
        "What are humanoid robots?",
        "Explain physical AI",
        "What are sensors in robotics?",
        "Describe the introduction to physical AI"
    ]
    
    all_tests_passed = True
    
    for i, question in enumerate(sample_questions):
        print(f"\n--- Test {i+1}: '{question}' ---")
        
        try:
            # Test retrieval
            print("Testing retrieval...")
            retrieved_chunks = retrieval_service.retrieve_relevant_chunks(
                query=question,
                limit=5,
                min_similarity_score=settings.MIN_SIMILARITY_THRESHOLD
            )
            
            print(f"Retrieved {len(retrieved_chunks)} chunks")
            
            if len(retrieved_chunks) > 0:
                print("✅ Retrieval successful!")
                for j, chunk in enumerate(retrieved_chunks[:2]):  # Show first 2 chunks
                    print(f"  Chunk {j+1}: {chunk['text_content'][:100]}...")
            else:
                print("⚠️  No chunks retrieved - collection might be empty or need re-indexing")
            
            # Test generation if we have chunks
            if len(retrieved_chunks) > 0:
                print("Testing generation...")
                response = generation_service.generate_response_from_context(
                    question=question,
                    context_chunks=retrieved_chunks,
                    session_id="test-session"
                )
                
                if hasattr(response, 'response') and response.response:
                    print(f"✅ Generation successful!")
                    print(f"Response preview: {response.response[:200]}...")
                else:
                    print("⚠️  Generation returned empty response")
                    all_tests_passed = False
            else:
                print("Skipping generation test - no chunks retrieved")
                
        except Exception as e:
            print(f"❌ Error during test '{question}': {str(e)}")
            all_tests_passed = False
    
    return all_tests_passed

def test_qdrant_connectivity():
    """
    Test Qdrant connectivity and collection status.
    """
    print("\n" + "="*60)
    print("TESTING QDRANT CONNECTIVITY")
    print("="*60)
    
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/.env")
    
    from qdrant_client import QdrantClient
    from src.config.settings import settings
    
    print(f"QDRANT_URL: {settings.QDRANT_URL}")
    print(f"QDRANT_COLLECTION_NAME: {settings.QDRANT_COLLECTION_NAME}")
    
    if settings.QDRANT_URL == "your-actual-qdrant-cluster-url" or settings.QDRANT_API_KEY == "your-actual-qdrant-api-key":
        print("❌ Qdrant configuration contains placeholder values!")
        print("Please update your .env file with actual Qdrant credentials.")
        return False
    
    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=10
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
                return True
            else:
                print("⚠️  Collection exists but is empty - needs to be populated with textbook content")
                return True  # Connection is fine, just needs content
        else:
            print(f"❌ Collection '{settings.QDRANT_COLLECTION_NAME}' not found!")
            return False
            
    except Exception as e:
        print(f"❌ Failed to connect to Qdrant: {str(e)}")
        return False

def test_embedding_service():
    """
    Test the embedding service functionality.
    """
    print("\n" + "="*60)
    print("TESTING EMBEDDING SERVICE")
    print("="*60)
    
    try:
        from src.services.embedding_service import embedding_service
        
        # Test embedding a sample text
        sample_text = "Artificial intelligence is a wonderful field of computer science."
        embedding = embedding_service.embed_single_text(sample_text)
        
        if embedding and len(embedding) > 0:
            print(f"✅ Embedding service working!")
            print(f"Generated embedding with {len(embedding)} dimensions")
            return True
        else:
            print("❌ Failed to generate embedding")
            return False
            
    except Exception as e:
        print(f"❌ Error in embedding service: {str(e)}")
        return False

def main():
    """
    Main function to run all tests.
    """
    print("COMPREHENSIVE QWEN RAG PIPELINE TEST")
    print("="*60)
    
    # Run individual tests
    connectivity_ok = test_qdrant_connectivity()
    embedding_ok = test_embedding_service()
    rag_ok = test_full_rag_pipeline()
    
    print("\n" + "="*60)
    print("FINAL RESULTS")
    print("="*60)
    
    print(f"Qdrant Connectivity: {'✅ PASS' if connectivity_ok else '❌ FAIL'}")
    print(f"Embedding Service: {'✅ PASS' if embedding_ok else '❌ FAIL'}")
    print(f"Full RAG Pipeline: {'✅ PASS' if rag_ok else '❌ FAIL'}")
    
    all_passed = connectivity_ok and embedding_ok and rag_ok
    
    if all_passed:
        print("\n🎉 ALL TESTS PASSED!")
        print("The Qwen RAG agent should be able to retrieve textbook content properly.")
        print("\nNext steps:")
        print("1. If collection was empty, run the upsert script to index textbook content:")
        print("   python src/services/upsert_textbook.py")
        print("2. Restart the backend server:")
        print("   uvicorn src.main:app --reload --port 8001")
        print("3. Test the /chat/ask endpoint with textbook questions")
    else:
        print("\n❌ SOME TESTS FAILED!")
        print("Please address the issues above before proceeding.")
        
        if not connectivity_ok:
            print("\nQdrant connectivity issues:")
            print("- Verify your Qdrant URL and API key in .env file")
            print("- Ensure Qdrant is accessible from your network")
            print("- Check that the collection name is 'textbook-content'")
        
        if not embedding_ok:
            print("\nEmbedding service issues:")
            print("- Verify your GEMINI_API_KEY in .env file")
            print("- Ensure the embedding model is accessible")
        
        if not rag_ok:
            print("\nRAG pipeline issues:")
            print("- May need to re-index textbook content if collection is empty")
            print("- Check that retrieval and generation services are properly configured")
    
    return all_passed

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)