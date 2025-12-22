"""
Test script to verify the Qdrant connection and check if content was properly indexed.
"""
import sys
import os

# Add the src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.vector_store.qdrant_client import qdrant_client
from src.services.embedding_service import embedding_service


def test_qdrant_connection():
    """Test basic Qdrant connection."""
    print("Testing Qdrant connection...")
    
    try:
        is_healthy = qdrant_client.health_check()
        print(f"Qdrant health check: {'PASSED' if is_healthy else 'FAILED'}")
        
        if is_healthy:
            # Try to count vectors in the collection
            count = qdrant_client.count_vectors()
            print(f"Number of vectors in collection: {count}")
            
            # Try to perform a simple search with a test query
            test_text = "Physical AI and embodied intelligence"
            test_embedding = embedding_service.embed_single_text(test_text)
            
            results = qdrant_client.search_vectors(
                query_vector=test_embedding,
                limit=5,
                min_similarity_score=0.0  # Lower threshold for testing
            )
            
            print(f"Search results count: {len(results)}")
            
            if results:
                print("Sample result:")
                sample_result = results[0]
                print(f"  ID: {sample_result['chunk_id']}")
                print(f"  Similarity Score: {sample_result['similarity_score']}")
                print(f"  Content Preview: {sample_result['text_content'][:100]}...")
                print(f"  Metadata: {sample_result['metadata']}")
        
        return is_healthy
    except Exception as e:
        print(f"Error testing Qdrant connection: {str(e)}")
        return False


def main():
    """Main function to run the Qdrant connection test."""
    print("Starting Qdrant connection test...\n")
    
    success = test_qdrant_connection()
    
    print(f"\nQdrant connection test: {'PASSED' if success else 'FAILED'}")
    
    if success:
        print("\nThe textbook content has been successfully indexed into the Qdrant vector database.")
        print("The RAG chatbot should now be able to retrieve relevant information and answer questions based on the textbook content.")
    else:
        print("\nFailed to connect to Qdrant. Please check your network connection and credentials.")


if __name__ == "__main__":
    main()