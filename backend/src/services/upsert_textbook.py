
import os
import sys
from pathlib import Path
import uuid

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

def read_markdown_files(docs_dir: str):
    """
    Read all markdown files from the textbook directory.
    
    Args:
        docs_dir: Path to the textbook documentation directory
        
    Returns:
        List of dictionaries containing file content and metadata
    """
    markdown_files = []
    docs_path = Path(docs_dir)
    
    for md_file in docs_path.rglob("*.md"):
        if md_file.name.startswith('_'):  # Skip files starting with underscore
            continue
            
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                
            # Extract relative path to use as source identifier
            rel_path = md_file.relative_to(docs_path)
            
            file_info = {
                'filename': str(rel_path),
                'content': content,
                'source_file': str(rel_path)
            }
            markdown_files.append(file_info)
            
            print(f"Loaded: {rel_path}")
            
        except Exception as e:
            print(f"Error reading {md_file}: {str(e)}")
    
    return markdown_files

def upsert_textbook_content():
    """
    Upsert textbook content to Qdrant vector store.
    """
    print("Loading environment variables...")
    from dotenv import load_dotenv
    load_dotenv("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/backend/.env")
    
    print("Importing modules...")
    from qdrant_client import QdrantClient
    from qdrant_client.http.models import PointStruct
    from src.config.settings import settings
    from src.services.embedding_service import embedding_service
    from src.utils.content_preprocessor import prepare_content_for_embedding
    
    # Check if we have proper Qdrant credentials
    if settings.QDRANT_URL == "your-qdrant-cluster-url-here" or settings.QDRANT_API_KEY == "your-qdrant-api-key-here":
        print("❌ Qdrant configuration contains placeholder values!")
        print("Please update your .env file with actual Qdrant credentials before running this script.")
        return False
    
    print(f"Connecting to Qdrant at: {settings.QDRANT_URL}")
    print(f"Using collection: {settings.QDRANT_COLLECTION_NAME}")
    
    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30
        )
        
        print("Connected to Qdrant successfully!")
        
        # Read all textbook content
        docs_dir = "C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/physical-ai-book/docs"
        print(f"Reading textbook content from: {docs_dir}")
        
        textbook_files = read_markdown_files(docs_dir)
        print(f"Found {len(textbook_files)} markdown files")
        
        if not textbook_files:
            print("No textbook content found!")
            return False
        
        # Prepare content for embedding (chunking)
        all_chunks = []
        for file_info in textbook_files:
            print(f"Processing {file_info['source_file']}...")
            
            # Prepare content for embedding (this handles chunking)
            content_chunks = prepare_content_for_embedding(
                text=file_info['content'],
                source_file=file_info['source_file'],
                chunk_strategy="semantic"
            )
            
            # Add source file info to each chunk's metadata
            for chunk in content_chunks:
                chunk['source_file'] = file_info['source_file']
            
            all_chunks.extend(content_chunks)
            print(f"  -> Created {len(content_chunks)} chunks")
        
        print(f"Total chunks to index: {len(all_chunks)}")
        
        if not all_chunks:
            print("No content chunks created!")
            return False
        
        # Generate embeddings for all chunks
        print("Generating embeddings...")
        texts_to_embed = [chunk['text_content'] for chunk in all_chunks]
        
        try:
            embeddings = embedding_service.generate_embeddings(texts_to_embed)
            print(f"Generated {len(embeddings)} embeddings")
        except Exception as e:
            print(f"Error generating embeddings: {str(e)}")
            return False
        
        # Create points for Qdrant
        print("Creating Qdrant points...")
        points = []
        for i, (chunk, embedding) in enumerate(zip(all_chunks, embeddings)):
            # Generate a unique ID for each chunk
            chunk_id = str(uuid.uuid4())
            
            # Create payload with content and metadata
            payload = {
                "text_content": chunk['text_content'],
                "metadata": {
                    "source_file": chunk.get('source_file', 'unknown'),
                    "chapter": chunk.get('metadata', {}).get('chapter', ''),
                    "section": chunk.get('metadata', {}).get('section', ''),
                    "page_number": chunk.get('metadata', {}).get('page_number', 0),
                    "chunk_index": i
                },
                "chunk_id": chunk_id
            }
            
            # Create point structure
            point = PointStruct(
                id=chunk_id,
                vector=embedding,
                payload=payload
            )
            
            points.append(point)
        
        print(f"Created {len(points)} points for insertion")
        
        # Create collection if it doesn't exist
        print("Checking if collection exists...")
        try:
            client.get_collection(settings.QDRANT_COLLECTION_NAME)
            print("Collection exists, will upsert to existing collection")
        except:
            print("Collection doesn't exist, creating it...")
            from qdrant_client.http.models import Distance, VectorParams
            from src.config.constants import DEFAULT_EMBEDDING_DIMENSION
            
            client.create_collection(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=DEFAULT_EMBEDDING_DIMENSION,
                    distance=Distance.COSINE
                )
            )
            print("Collection created successfully!")
        
        # Upsert points to Qdrant
        print("Upserting points to Qdrant...")
        batch_size = 100  # Process in batches to avoid timeouts
        total_processed = 0
        
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            client.upsert(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                points=batch
            )
            total_processed += len(batch)
            print(f"  Processed {total_processed}/{len(points)} points")
        
        print(f"Successfully upserted {total_processed} points to Qdrant!")
        
        # Verify the content was added
        collection_info = client.get_collection(settings.QDRANT_COLLECTION_NAME)
        print(f"Collection now contains {collection_info.point_count} points")
        
        return True
        
    except Exception as e:
        print(f"❌ Error during upsert process: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Starting textbook content upsert to Qdrant...")
    success = upsert_textbook_content()
    
    if success:
        print("\n🎉 Textbook content successfully upserted to Qdrant!")
        print("The Qwen agent should now be able to retrieve textbook content properly.")
    else:
        print("\n❌ Failed to upsert textbook content to Qdrant!")
        print("Please check the error messages above and ensure your Qdrant credentials are correct.")
