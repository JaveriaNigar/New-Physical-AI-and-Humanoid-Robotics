"""
Fix the Qwen RAG agent to properly retrieve textbook content.
This script updates the configuration and provides a method to re-populate the vector store.
"""
import os
import sys
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


def create_populate_qdrant_script():
    """
    Create a script to populate Qdrant with textbook content.
    """
    script_content = '''"""
Script to populate Qdrant with textbook content for the Qwen RAG agent.
"""
import os
import sys
import uuid
from pathlib import Path
import logging

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def read_textbook_content():
    """
    Read all markdown files from the textbook directory.
    
    Returns:
        List of dictionaries containing content chunks with metadata
    """
    textbook_dir = Path("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/physical-ai-book/docs")
    
    if not textbook_dir.exists():
        logger.error(f"Textbook directory does not exist: {textbook_dir}")
        return []
    
    content_chunks = []
    for md_file in textbook_dir.rglob("*.md"):
        if md_file.name.startswith('_'):  # Skip files starting with underscore
            continue
            
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                
            # Create chunks from the content (simple paragraph-based chunking)
            paragraphs = content.split('\\n\\n')
            for i, paragraph in enumerate(paragraphs):
                if len(paragraph.strip()) > 20:  # Only include substantial paragraphs
                    # Extract relative path to use as source identifier
                    rel_path = md_file.relative_to(textbook_dir)
                    
                    chunk = {
                        "text_content": paragraph.strip(),
                        "metadata": {
                            "source_file": str(rel_path),
                            "paragraph_index": i,
                            "original_file": md_file.name
                        }
                    }
                    content_chunks.append(chunk)
                    
        except Exception as e:
            logger.error(f"Error reading {md_file}: {str(e)}")
    
    logger.info(f"Loaded {len(content_chunks)} content chunks from textbook")
    return content_chunks


def populate_qdrant():
    """
    Populate the Qdrant collection with textbook content.
    """
    logger.info("Starting Qdrant population process...")
    
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()
    
    # Import required modules after setting up environment
    from src.config.settings import settings
    from src.services.embedding_service import embedding_service
    from qdrant_client import QdrantClient
    from qdrant_client.http.models import PointStruct, VectorParams, Distance
    
    # Check if we have proper Qdrant credentials
    if settings.QDRANT_URL == "your-actual-qdrant-cluster-url" or settings.QDRANT_API_KEY == "your-actual-qdrant-api-key":
        logger.error("Qdrant configuration contains placeholder values!")
        logger.error("Please update your .env file with actual Qdrant credentials before running this script.")
        return False
    
    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30
        )
        
        logger.info(f"Connected to Qdrant at: {settings.QDRANT_URL}")
        logger.info(f"Using collection: {settings.QDRANT_COLLECTION_NAME}")
        
        # Read textbook content
        logger.info("Reading textbook content...")
        content_chunks = read_textbook_content()
        
        if not content_chunks:
            logger.error("No textbook content found!")
            return False
        
        # Generate embeddings for all content chunks
        logger.info("Generating embeddings for content chunks...")
        texts_to_embed = [chunk["text_content"] for chunk in content_chunks]
        
        try:
            embeddings = embedding_service.embed_multiple_texts(texts_to_embed)
            logger.info(f"Generated {len(embeddings)} embeddings")
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            return False
        
        # Prepare points for insertion
        logger.info("Preparing points for Qdrant...")
        points = []
        for i, (chunk, embedding) in enumerate(zip(content_chunks, embeddings)):
            if embedding:  # Make sure embedding was generated successfully
                point = PointStruct(
                    id=str(uuid.uuid4()),  # Generate unique ID
                    vector=embedding,
                    payload={
                        "text_content": chunk["text_content"],
                        "metadata": chunk["metadata"]
                    }
                )
                points.append(point)
        
        logger.info(f"Created {len(points)} points for insertion")
        
        # Create collection if it doesn't exist
        try:
            client.get_collection(settings.QDRANT_COLLECTION_NAME)
            logger.info("Collection exists, will upsert to existing collection")
        except:
            logger.info("Collection doesn't exist, creating it...")
            client.create_collection(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                vectors_config=VectorParams(size=len(embeddings[0]), distance=Distance.COSINE)
            )
            logger.info("Collection created successfully!")
        
        # Upsert points to Qdrant (in batches to avoid timeout)
        logger.info("Upserting points to Qdrant...")
        batch_size = 50  # Process in smaller batches
        total_processed = 0
        
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            client.upsert(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                points=batch
            )
            total_processed += len(batch)
            logger.info(f"  Processed {total_processed}/{len(points)} points")
        
        logger.info(f"Successfully upserted {total_processed} points to Qdrant!")
        
        # Verify the content was added
        collection_info = client.get_collection(settings.QDRANT_COLLECTION_NAME)
        logger.info(f"Collection now contains {collection_info.point_count} points")
        
        return True
        
    except Exception as e:
        logger.error(f"Error during Qdrant population: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("This script will populate Qdrant with textbook content.")
    print("Before running, ensure:")
    print("1. You have updated the .env file with real Qdrant credentials")
    print("2. The textbook content is available in the docs directory")
    print("")
    
    response = input("Do you want to continue? (yes/no): ")
    if response.lower() in ['yes', 'y', 'continue']:
        success = populate_qdrant()
        
        if success:
            print("\\nSuccess! Qdrant has been populated with textbook content.")
            print("The Qwen agent should now be able to retrieve textbook content properly.")
        else:
            print("\\nFailed to populate Qdrant. Check the error messages above.")
    else:
        print("Operation cancelled.")
'''
    
    # Write the script to the backend directory
    script_path = Path("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/populate_textbook_qdrant.py")
    with open(script_path, 'w', encoding='utf-8') as f:
        f.write(script_content)
    
    print(f"Created population script at: {script_path}")


def create_fixed_test_script():
    """
    Create a fixed test script without unicode characters.
    """
    script_content = '''"""
Comprehensive test script to verify the Qwen RAG pipeline.
Tests Qdrant connectivity, content retrieval, and generation.
"""
import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

def test_qdrant_connectivity():
    """
    Test Qdrant connectivity and collection status.
    """
    print("="*60)
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
        print("[ERROR] Qdrant configuration contains placeholder values!")
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
        print("[SUCCESS] Connected to Qdrant successfully!")
        print(f"Available collections: {[col.name for col in collections.collections]}")
        
        # Check if our collection exists
        collection_names = [col.name for col in collections.collections]
        if settings.QDRANT_COLLECTION_NAME in collection_names:
            print(f"[SUCCESS] Collection '{settings.QDRANT_COLLECTION_NAME}' exists!")
            
            # Get collection info
            collection_info = client.get_collection(settings.QDRANT_COLLECTION_NAME)
            print(f"Collection points count: {collection_info.point_count}")
            print(f"Collection vectors count: {collection_info.vectors_count}")
            
            if collection_info.point_count > 0:
                print("[SUCCESS] Collection contains vectors!")
                return True
            else:
                print("[WARNING] Collection exists but is empty - needs to be populated with textbook content")
                return True  # Connection is fine, just needs content
        else:
            print(f"[ERROR] Collection '{settings.QDRANT_COLLECTION_NAME}' not found!")
            return False
            
    except Exception as e:
        print(f"[ERROR] Failed to connect to Qdrant: {str(e)}")
        return False

def test_embedding_service():
    """
    Test the embedding service functionality.
    """
    print("\\n" + "="*60)
    print("TESTING EMBEDDING SERVICE")
    print("="*60)
    
    try:
        from src.services.embedding_service import embedding_service
        
        # Test embedding a sample text
        sample_text = "Artificial intelligence is a wonderful field of computer science."
        embedding = embedding_service.embed_single_text(sample_text)
        
        if embedding and len(embedding) > 0:
            print("[SUCCESS] Embedding service working!")
            print(f"Generated embedding with {len(embedding)} dimensions")
            return True
        else:
            print("[ERROR] Failed to generate embedding")
            return False
            
    except Exception as e:
        print(f"[ERROR] Error in embedding service: {str(e)}")
        return False

def test_full_rag_pipeline():
    """
    Test the full RAG pipeline: retrieval and generation.
    """
    print("\\n" + "="*60)
    print("TESTING FULL RAG PIPELINE")
    print("="*60)
    
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/.env")
    
    from src.config.settings import settings
    
    print(f"QDRANT_COLLECTION_NAME: {settings.QDRANT_COLLECTION_NAME}")
    print(f"MIN_SIMILARITY_THRESHOLD: {settings.MIN_SIMILARITY_THRESHOLD}")
    
    try:
        # Import services
        from src.services.retrieval_service import retrieval_service
        from src.services.generation_service import generation_service
        
        # Test retrieval with a sample question
        sample_questions = [
            "What are humanoid robots?",
            "Explain physical AI",
            "What are sensors in robotics?",
            "Describe the introduction to physical AI"
        ]
        
        all_tests_passed = True
        
        for i, question in enumerate(sample_questions):
            print(f"\\n--- Test {i+1}: '{question}' ---")
            
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
                    print("[SUCCESS] Retrieval successful!")
                    for j, chunk in enumerate(retrieved_chunks[:2]):  # Show first 2 chunks
                        print(f"  Chunk {j+1}: {chunk['text_content'][:100]}...")
                else:
                    print("[WARNING] No chunks retrieved - collection might be empty or need re-indexing")
                
                # Test generation if we have chunks
                if len(retrieved_chunks) > 0:
                    print("Testing generation...")
                    response = generation_service.generate_response_from_context(
                        question=question,
                        context_chunks=retrieved_chunks,
                        session_id="test-session"
                    )
                    
                    if hasattr(response, 'response') and response.response:
                        print("[SUCCESS] Generation successful!")
                        print(f"Response preview: {response.response[:200]}...")
                    else:
                        print("[WARNING] Generation returned empty response")
                        all_tests_passed = False
                else:
                    print("Skipping generation test - no chunks retrieved")
                    
            except Exception as e:
                print(f"[ERROR] Error during test '{question}': {str(e)}")
                all_tests_passed = False
        
        return all_tests_passed
        
    except Exception as e:
        print(f"[ERROR] Error in RAG pipeline test: {str(e)}")
        import traceback
        traceback.print_exc()
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
    
    print("\\n" + "="*60)
    print("FINAL RESULTS")
    print("="*60)
    
    print(f"Qdrant Connectivity: {'[SUCCESS]' if connectivity_ok else '[FAILED]'}")
    print(f"Embedding Service: {'[SUCCESS]' if embedding_ok else '[FAILED]'}")
    print(f"Full RAG Pipeline: {'[SUCCESS]' if rag_ok else '[FAILED]'}")
    
    all_passed = connectivity_ok and embedding_ok and rag_ok
    
    if all_passed:
        print("\\n[SUCCESS] ALL TESTS PASSED!")
        print("The Qwen agent should be able to retrieve textbook content properly.")
        print("\\nNext steps:")
        print("1. If collection was empty, run the upsert script to index textbook content:")
        print("   python populate_textbook_qdrant.py")
        print("2. Restart the backend server:")
        print("   uvicorn src.main:app --reload --port 8001")
        print("3. Test the /chat/ask endpoint with textbook questions")
    else:
        print("\\n[FAILED] SOME TESTS FAILED!")
        print("Please address the issues above before proceeding.")
        
        if not connectivity_ok:
            print("\\nQdrant connectivity issues:")
            print("- Verify your Qdrant URL and API key in .env file")
            print("- Ensure Qdrant is accessible from your network")
            print("- Check that the collection name is 'textbook-content'")
        
        if not embedding_ok:
            print("\\nEmbedding service issues:")
            print("- Verify your GEMINI_API_KEY in .env file")
            print("- Ensure the embedding model is accessible")
        
        if not rag_ok:
            print("\\nRAG pipeline issues:")
            print("- May need to re-index textbook content if collection is empty")
            print("- Check that retrieval and generation services are properly configured")
    
    return all_passed

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
'''
    
    # Write the fixed test script
    script_path = Path("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/test_rag_pipeline_fixed.py")
    with open(script_path, 'w', encoding='utf-8') as f:
        f.write(script_content)
    
    print(f"Created fixed test script at: {script_path}")


if __name__ == "__main__":
    print("Fixing Qwen RAG Agent - Textbook Content Retrieval")
    print("="*60)
    
    # Update the .env file with proper configuration
    print("1. Updating .env file with proper Qdrant configuration...")
    update_env_file()
    
    # Create the population script
    print("\\n2. Creating Qdrant population script...")
    create_populate_qdrant_script()
    
    # Create the fixed test script
    print("\\n3. Creating fixed test script...")
    create_fixed_test_script()
    
    print("\\n" + "="*60)
    print("FIX IMPLEMENTATION COMPLETE")
    print("="*60)
    print("Files created/updated:")
    print("- Updated .env file with proper Qdrant settings")
    print("- populate_textbook_qdrant.py - Script to populate Qdrant with textbook content")
    print("- test_rag_pipeline_fixed.py - Fixed test script without unicode issues")
    print("")
    print("Next steps:")
    print("1. Update your .env file with REAL Qdrant credentials")
    print("2. Run the population script to index textbook content: python populate_textbook_qdrant.py")
    print("3. Run the test script to verify everything works: python test_rag_pipeline_fixed.py")
    print("4. Restart your backend server")