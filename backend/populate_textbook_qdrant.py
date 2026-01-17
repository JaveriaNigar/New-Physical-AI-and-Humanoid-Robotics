"""
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
            paragraphs = content.split('\n\n')
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
            embeddings = embedding_service.generate_embeddings(texts_to_embed)
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

    # Automatically continue without user input for automation
    success = populate_qdrant()

    if success:
        print("\nSuccess! Qdrant has been populated with textbook content.")
        print("The Qwen agent should now be able to retrieve textbook content properly.")
    else:
        print("\nFailed to populate Qdrant. Check the error messages above.")
