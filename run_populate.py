import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

# Import and run the populate function directly
from populate_textbook_qdrant import populate_qdrant

print("Starting Qdrant population process...")
success = populate_qdrant()

if success:
    print("\nSuccess! Qdrant has been populated with textbook content.")
    print("The Qwen agent should now be able to retrieve textbook content properly.")
else:
    print("\nFailed to populate Qdrant. Check the error messages above.")