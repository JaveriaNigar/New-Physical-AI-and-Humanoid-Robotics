"""
Script to help set up proper credentials for the Qwen RAG agent.
This script will guide the user to input their actual credentials.
"""
import os
import sys
from pathlib import Path

def setup_credentials():
    """
    Guide the user to set up proper credentials in the .env file.
    """
    print("Setting up credentials for Qwen RAG Agent")
    print("="*50)
    
    print("\nYou need to provide the following credentials:")
    print("1. Google Gemini API Key")
    print("2. Qdrant API Key") 
    print("3. Qdrant URL")
    print("4. Neon Database URL (optional)")
    
    print("\nIf you don't have these, you'll need to:")
    print("- Get a Google Gemini API key from: https://aistudio.google.com/")
    print("- Get Qdrant Cloud account from: https://qdrant.tech/")
    print("- Get Neon Postgres database from: https://neon.tech/")
    
    # Get user input for credentials
    print("\n" + "-"*50)
    gemini_key = input("Enter your Google Gemini API key (or press Enter to skip): ").strip()
    qdrant_key = input("Enter your Qdrant API key (or press Enter to skip): ").strip()
    qdrant_url = input("Enter your Qdrant URL (or press Enter to skip): ").strip()
    neon_url = input("Enter your Neon Database URL (or press Enter to skip): ").strip()
    
    # Read the current .env file
    env_file_path = Path(".env")
    if not env_file_path.exists():
        print(f"Error: .env file not found at {env_file_path}")
        return False
    
    with open(env_file_path, 'r') as file:
        lines = file.readlines()
    
    # Update the values in the file
    updated_lines = []
    for line in lines:
        if line.startswith("GEMINI_API_KEY="):
            if gemini_key:
                updated_lines.append(f"GEMINI_API_KEY={gemini_key}\n")
            else:
                updated_lines.append(line)
        elif line.startswith("QDRANT_API_KEY="):
            if qdrant_key:
                updated_lines.append(f"QDRANT_API_KEY={qdrant_key}\n")
            else:
                updated_lines.append(line)
        elif line.startswith("QDRANT_URL="):
            if qdrant_url:
                updated_lines.append(f"QDRANT_URL={qdrant_url}\n")
            else:
                updated_lines.append(line)
        elif line.startswith("NEON_DATABASE_URL="):
            if neon_url:
                updated_lines.append(f"NEON_DATABASE_URL={neon_url}\n")
            else:
                updated_lines.append(line)
        else:
            updated_lines.append(line)
    
    # Write the updated content back to the file
    with open(env_file_path, 'w') as file:
        file.writelines(updated_lines)
    
    print(f"\nUpdated .env file with new credentials at: {env_file_path}")
    
    # Verify that the credentials have been updated
    print("\nVerifying credentials in .env file...")
    with open(env_file_path, 'r') as file:
        content = file.read()
        
    has_gemini = "your_actual_gemini_api_key_here" not in content
    has_qdrant_key = "your_actual_qdrant_api_key_here" not in content
    has_qdrant_url = "your_actual_qdrant_cluster_url_here" not in content
    
    if has_gemini and has_qdrant_key and has_qdrant_url:
        print("✅ Credentials have been updated successfully!")
        return True
    else:
        print("⚠️  Some credentials may still be placeholders. Please check your .env file.")
        return False

def main():
    """
    Main function to run the credential setup.
    """
    print("Qwen RAG Agent - Credential Setup")
    print("="*50)
    
    success = setup_credentials()
    
    if success:
        print("\n" + "="*50)
        print("CREDENTIAL SETUP COMPLETE!")
        print("="*50)
        print("Next steps:")
        print("1. Run the backend server: uvicorn src.main:app --reload --port 8001")
        print("2. Make sure to populate Qdrant with textbook content")
        print("3. Test the /chat/ask endpoint")
    else:
        print("\n" + "="*50)
        print("CREDENTIAL SETUP INCOMPLETE!")
        print("="*50)
        print("Please manually update your .env file with proper credentials.")

if __name__ == "__main__":
    main()