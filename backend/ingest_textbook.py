"""
Script to ingest the textbook content into the vector database (Qdrant).
This script reads all markdown files from the docs directory and indexes them
into the Qdrant vector store for RAG functionality.
"""
import os
import glob
from pathlib import Path
from src.services.content_ingestion_service import content_ingestion_service


def extract_chapter_info_from_path(file_path: str) -> tuple:
    """
    Extract chapter/week information from the file path.

    Args:
        file_path: Path to the markdown file

    Returns:
        Tuple of (chapter_name, section_name)
    """
    path_parts = Path(file_path).parts

    # Look for week information in the path
    week_info = ""
    for part in path_parts:
        if "week" in part.lower():
            week_info = part.replace("-", " ").title()
            break

    # Get the file name without extension as section
    section_name = Path(file_path).stem.replace("-", " ").title()

    return week_info, section_name


def read_markdown_file(file_path: str) -> str:
    """
    Read content from a markdown file, extracting only the main content
    (excluding frontmatter if present).

    Args:
        file_path: Path to the markdown file

    Returns:
        Content of the file as a string
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Remove frontmatter if present (content between --- delimiters at the start)
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            content = parts[2]  # Get content after the second ---

    return content


def ingest_textbook_content():
    """
    Main function to ingest all textbook content from the docs directory.
    """
    print("Starting textbook content ingestion...")

    # Get all markdown files from the docs directory
    docs_path = Path("C:/Users/M.R Computers/OneDrive/Desktop/hackathon2/physical-ai-book/docs")
    markdown_files = list(docs_path.rglob("*.md"))  # Recursively find all .md files

    print(f"Found {len(markdown_files)} markdown files to process")

    for i, file_path in enumerate(markdown_files, 1):
        print(f"Processing ({i}/{len(markdown_files)}): {file_path}")

        try:
            # Read the content from the markdown file
            content = read_markdown_file(str(file_path))

            # Extract chapter/section info from the path
            chapter_info, section_info = extract_chapter_info_from_path(str(file_path))

            print(f"  - Chapter: {chapter_info}")
            print(f"  - Section: {section_info}")

            # Create source file identifier
            relative_path = file_path.relative_to(docs_path.parent)  # Get relative path from project root
            source_file = str(relative_path)

            # Ingest the content into the vector store
            success = content_ingestion_service.ingest_textbook_content(
                content=content,
                source_file=source_file,
                create_collection=(i == 1)  # Only create collection on first file
            )

            if success:
                print(f"  [SUCCESS] Successfully ingested: {file_path.name}")
            else:
                print(f"  [FAILED] Failed to ingest: {file_path.name}")

        except Exception as e:
            print(f"  [ERROR] Error processing {file_path}: {str(e)}")

    print("\nTextbook content ingestion completed!")


if __name__ == "__main__":
    ingest_textbook_content()