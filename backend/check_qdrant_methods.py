"""
Script to check the actual methods available in the Qdrant client.
"""
from qdrant_client import QdrantClient
from src.config.settings import settings

def check_qdrant_methods():
    """Check the actual methods available in the Qdrant client."""
    # Initialize Qdrant client similar to how it's done in our code
    if settings.QDRANT_URL.startswith('https://') or settings.QDRANT_URL.startswith('http://'):
        # Using Qdrant Cloud with API key
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30
        )
    else:
        # Using local Qdrant instance
        client = QdrantClient(host=settings.QDRANT_URL, port=6333)
    
    print("Qdrant client initialized successfully")
    
    # Get all public methods and attributes
    all_attrs = [attr for attr in dir(client) if not attr.startswith('_')]
    
    # Filter for search/query related methods
    search_related = [attr for attr in all_attrs if any(keyword in attr.lower() for keyword in ['search', 'query', 'find'])]
    
    print(f"All attributes: {all_attrs}")
    print(f"Search/query related attributes: {search_related}")
    
    # Try different possible search methods
    possible_search_methods = [
        'search',
        'search_points',
        'query',
        'query_points',
        'find',
        'find_points'
    ]
    
    print("\nTrying different search methods:")
    for method_name in possible_search_methods:
        if hasattr(client, method_name):
            print(f"  ✓ {method_name}: EXISTS")
        else:
            print(f"  ✗ {method_name}: NOT FOUND")

if __name__ == "__main__":
    check_qdrant_methods()