import os
import sys
import threading
import time

# Add backend to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def run_populate():
    """Function to run the populate script"""
    try:
        from populate_textbook_qdrant import populate_qdrant
        print("Starting Qdrant population process...")
        success = populate_qdrant()
        
        if success:
            print("\nSuccess! Qdrant has been populated with textbook content.")
            print("The Qwen agent should now be able to retrieve textbook content properly.")
        else:
            print("\nFailed to populate Qdrant. Check the error messages above.")
    except Exception as e:
        print(f"Error running populate: {e}")

def check_collection_exists():
    """Function to periodically check if the collection exists"""
    time.sleep(10)  # Wait a bit before starting to check
    
    while True:
        try:
            from src.vector_store.qdrant_client import qdrant_client
            count = qdrant_client.count_vectors()
            print(f"Number of vectors in collection: {count}")
            
            if count > 0:
                print("✓ SUCCESS: Embeddings have been inserted into Qdrant!")
                break
        except Exception as e:
            if "doesn't exist" in str(e):
                print("Collection doesn't exist yet, waiting...")
            else:
                print(f"Still waiting... (Error: {e})")
        
        time.sleep(30)  # Check every 30 seconds

# Run both functions concurrently
populate_thread = threading.Thread(target=run_populate)
checker_thread = threading.Thread(target=check_collection_exists)

populate_thread.start()
checker_thread.start()

# Wait for the populate thread to finish
populate_thread.join()

# Give a bit more time for the checker to confirm
time.sleep(60)