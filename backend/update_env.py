"""
Utility script to update the .env file with proper Qdrant configuration.
"""
import os
from pathlib import Path

def update_env_file():
    """
    Update the .env file with proper Qdrant configuration values.
    """
    env_file_path = Path("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/.env")
    
    # Read the current .env file
    with open(env_file_path, 'r') as file:
        lines = file.readlines()
    
    # Update the Qdrant configuration values
    updated_lines = []
    for line in lines:
        if line.startswith("QDRANT_URL="):
            updated_lines.append("QDRANT_URL=your-actual-qdrant-cluster-url\n")
        elif line.startswith("QDRANT_API_KEY="):
            updated_lines.append("QDRANT_API_KEY=your-actual-qdrant-api-key\n")
        elif line.startswith("QDRANT_COLLECTION_NAME="):
            # Ensure the collection name is correct
            updated_lines.append("QDRANT_COLLECTION_NAME=textbook-content\n")
        else:
            # Keep other lines as they are
            updated_lines.append(line)
    
    # Write the updated content back to the file
    with open(env_file_path, 'w') as file:
        file.writelines(updated_lines)
    
    print(f"Updated {env_file_path} with proper Qdrant configuration")
    print("Please replace 'your-actual-qdrant-cluster-url' and 'your-actual-qdrant-api-key' with real values")

if __name__ == "__main__":
    update_env_file()