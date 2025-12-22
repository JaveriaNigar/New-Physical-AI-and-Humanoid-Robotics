"""
Content preprocessing utilities for the RAG Chatbot backend.
Provides functions for preparing textbook content for embedding and storage.
"""
import re
from typing import List, Tuple, Dict, Any
from src.config.constants import CHUNK_SIZE_TOKENS, CHUNK_OVERLAP_TOKENS


def clean_text(text: str) -> str:
    """
    Clean and normalize text content.
    
    Args:
        text: Raw text content to clean
        
    Returns:
        Cleaned and normalized text
    """
    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)
    
    # Remove special characters that might interfere with processing
    text = re.sub(r'[^\w\s\-\.\,\!\?\;\:]', ' ', text)
    
    # Strip leading/trailing whitespace
    text = text.strip()
    
    return text


def chunk_text_by_tokens(text: str, max_tokens: int = CHUNK_SIZE_TOKENS, overlap: int = CHUNK_OVERLAP_TOKENS) -> List[str]:
    """
    Split text into chunks based on token count with overlap.
    
    Args:
        text: Text to be chunked
        max_tokens: Maximum number of tokens per chunk
        overlap: Number of overlapping tokens between chunks
        
    Returns:
        List of text chunks
    """
    # Simple tokenization by words (in practice, you might use a more sophisticated tokenizer)
    words = text.split()
    
    if len(words) <= max_tokens:
        return [text]
    
    chunks = []
    start_idx = 0
    
    while start_idx < len(words):
        end_idx = start_idx + max_tokens
        chunk_words = words[start_idx:end_idx]
        chunk = ' '.join(chunk_words)
        chunks.append(chunk)
        
        # Move start index by max_tokens minus overlap
        start_idx = end_idx - overlap if end_idx < len(words) else len(words)
    
    return chunks


def chunk_text_by_semantic_boundaries(text: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """
    Split text into semantically coherent chunks based on paragraph or sentence boundaries.
    
    Args:
        text: Text to be chunked
        max_chunk_size: Maximum character size per chunk
        overlap: Number of overlapping characters between chunks
        
    Returns:
        List of semantically coherent text chunks
    """
    # Split text into paragraphs first
    paragraphs = text.split('\n\n')
    
    # If no double newlines, split by single newlines
    if len(paragraphs) <= 1:
        paragraphs = text.split('\n')
    
    chunks = []
    current_chunk = ""
    
    for paragraph in paragraphs:
        # If adding this paragraph would exceed max size, start a new chunk
        if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
            chunks.append(current_chunk.strip())
            
            # Add overlap by including part of the previous chunk
            if overlap > 0:
                # Get the last 'overlap' characters from the previous chunk to start the new one
                overlap_text = current_chunk[-overlap:] if len(current_chunk) > overlap else current_chunk
                current_chunk = overlap_text + " " + paragraph
            else:
                current_chunk = paragraph
        else:
            # Add paragraph to current chunk
            if current_chunk:
                current_chunk += "\n\n" + paragraph
            else:
                current_chunk = paragraph
    
    # Add the last chunk if it's not empty
    if current_chunk.strip():
        chunks.append(current_chunk.strip())
    
    return chunks


def extract_document_structure(text: str) -> Dict[str, Any]:
    """
    Extract structural information from the document like chapters and sections.
    
    Args:
        text: Document text to analyze
        
    Returns:
        Dictionary with structural information
    """
    structure = {
        'chapters': [],
        'sections': [],
        'metadata': {}
    }
    
    # Look for chapter patterns (e.g., "Chapter 1", "CHAPTER 1", etc.)
    chapter_pattern = r'(?:^|\n)(?:Chapter|CHAPTER|chapter)\s+(\d+)[\s\.]([^\n]+)'
    chapters = re.findall(chapter_pattern, text, re.IGNORECASE)
    structure['chapters'] = [(num, title.strip()) for num, title in chapters]
    
    # Look for section patterns (e.g., "Section 1.2", "1.2 Section Title", etc.)
    section_pattern = r'(?:^|\n)(?:Section|SECTION|section)?\s*([0-9]+(?:\.[0-9]+)*)[\s\.]([^\n]+)'
    sections = re.findall(section_pattern, text, re.IGNORECASE)
    structure['sections'] = [(num, title.strip()) for num, title in sections]
    
    return structure


def prepare_content_for_embedding(
    text: str, 
    source_file: str, 
    chunk_strategy: str = "semantic"
) -> List[Dict[str, Any]]:
    """
    Prepare content for embedding by cleaning and chunking.
    
    Args:
        text: Raw text content
        source_file: Name of source file
        chunk_strategy: Strategy for chunking ("semantic" or "token")
        
    Returns:
        List of dictionaries with chunk information ready for embedding
    """
    # Clean the text
    cleaned_text = clean_text(text)
    
    # Extract document structure
    structure = extract_document_structure(cleaned_text)
    
    # Choose chunking strategy
    if chunk_strategy == "token":
        chunks = chunk_text_by_tokens(cleaned_text)
    else:  # semantic
        chunks = chunk_text_by_semantic_boundaries(cleaned_text)
    
    # Prepare the result
    result = []
    for i, chunk in enumerate(chunks):
        chunk_info = {
            'text_content': chunk,
            'metadata': {
                'source_file': source_file,
                'chunk_index': i,
                'total_chunks': len(chunks)
            }
        }
        
        # Add chapter/section info if available
        if structure['chapters']:
            # For simplicity, just add the first chapter found
            chapter_num, chapter_title = structure['chapters'][0]
            chunk_info['metadata']['chapter'] = f"Chapter {chapter_num}: {chapter_title}"
        
        if structure['sections']:
            # For simplicity, just add the first section found
            section_num, section_title = structure['sections'][0]
            chunk_info['metadata']['section'] = f"{section_num} {section_title}"
        
        result.append(chunk_info)
    
    return result