"""
Test script to verify the health status of all backend services
"""
import asyncio
import os
import sys
from pathlib import Path

# Add the src directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.vector_store.qdrant_client import qdrant_client
from src.database.postgres_client import postgres_client
import google.generativeai as genai
from src.config.settings import settings


async def test_qdrant_connection():
    """Test Qdrant connection"""
    print("Testing Qdrant connection...")
    try:
        result = qdrant_client.health_check()
        print(f"Qdrant health check: {'OK' if result else 'FAILED'}")
        return result
    except Exception as e:
        print(f"Qdrant health check failed: {str(e)}")
        return False


async def test_postgres_connection():
    """Test Postgres connection"""
    print("\nTesting Postgres connection...")
    try:
        result = postgres_client.health_check()
        print(f"Postgres health check: {'OK' if result else 'FAILED'}")
        return result
    except Exception as e:
        print(f"Postgres health check failed: {str(e)}")
        return False


async def test_gemini_connection():
    """Test Gemini connection"""
    print("\nTesting Gemini connection...")
    try:
        genai.configure(api_key=settings.GEMINI_API_KEY)
        # Check if we can list models as a basic connectivity test
        # This is a lightweight operation that doesn't consume quota
        models = genai.list_models()
        # Verify that we have at least one model available
        if models:
            for model in models:
                if model and model.name:
                    print(f"Gemini health check: OK")
                    return True
        print("Gemini health check: FAILED - No models available")
        return False
    except Exception as e:
        print(f"Gemini health check failed: {str(e)}")
        return False


async def main():
    """Main function to run all health checks"""
    print("Running backend health checks...\n")

    # Run all health checks
    qdrant_ok = await test_qdrant_connection()
    postgres_ok = await test_postgres_connection()
    gemini_ok = await test_gemini_connection()

    print(f"\nSummary:")
    print(f"Qdrant: {'OK' if qdrant_ok else 'FAILED'}")
    print(f"Postgres: {'OK' if postgres_ok else 'FAILED'}")
    print(f"Gemini: {'OK' if gemini_ok else 'FAILED'}")

    all_healthy = qdrant_ok and postgres_ok and gemini_ok
    print(f"\nOverall Status: {'All services healthy' if all_healthy else 'Some services unhealthy'}")

    return all_healthy


if __name__ == "__main__":
    # Set environment variables if they're not already set
    if not os.getenv("GEMINI_API_KEY"):
        print("Warning: GEMINI_API_KEY environment variable not set")
    if not os.getenv("QDRANT_API_KEY"):
        print("Warning: QDRANT_API_KEY environment variable not set")
    if not os.getenv("NEON_DATABASE_URL"):
        print("Warning: NEON_DATABASE_URL environment variable not set")

    # Run the async main function
    asyncio.run(main())