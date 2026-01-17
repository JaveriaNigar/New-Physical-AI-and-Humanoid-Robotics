import sys
import os

# Add the backend/src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.vector_store.qdrant_client import qdrant_client

def test_qdrant_population():
    """Test that the Qdrant collection has been populated with textbook content."""
    print("Testing Qdrant population...")

    try:
        # Check if Qdrant is healthy
        is_healthy = qdrant_client.health_check()
        print(f"Qdrant health check: {'PASSED' if is_healthy else 'FAILED'}")

        if is_healthy:
            # Count vectors in the collection
            count = qdrant_client.count_vectors()
            print(f"Number of vectors in collection: {count}")

            if count > 0:
                print("✓ Qdrant has been successfully populated with textbook content.")
                print("The agent should now be able to retrieve content and answer questions.")
            else:
                print("⚠️  Qdrant collection is empty. Content may not have been populated yet.")
        else:
            print("✗ Failed to connect to Qdrant.")

    except Exception as e:
        print(f"Error testing Qdrant population: {str(e)}")

if __name__ == "__main__":
    test_qdrant_population()