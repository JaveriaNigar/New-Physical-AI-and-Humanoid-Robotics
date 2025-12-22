"""
Health check endpoint for the RAG Chatbot backend.
Provides health status of the service and its dependencies.
"""
from fastapi import APIRouter
from datetime import datetime
from src.models.common import HealthResponse
from src.vector_store.qdrant_client import qdrant_client
from src.database.postgres_client import postgres_client
import google.generativeai as genai
from src.config.settings import settings
from src.utils.monitoring import monitoring_service
import asyncio
from typing import Dict, Literal


router = APIRouter()


async def check_qdrant_health() -> Literal["connected", "disconnected"]:
    """Asynchronously check Qdrant health with retries."""
    for attempt in range(3):  # Try up to 3 times
        try:
            if qdrant_client.health_check():
                return "connected"
        except Exception as e:
            if attempt == 2:  # Last attempt
                print(f"Qdrant health check failed after 3 attempts: {str(e)}")
        await asyncio.sleep(0.5)  # Wait 0.5 seconds between attempts
    return "disconnected"


async def check_postgres_health() -> Literal["connected", "disconnected"]:
    """Asynchronously check Postgres health with retries."""
    for attempt in range(3):  # Try up to 3 times
        try:
            if postgres_client.health_check():
                return "connected"
        except Exception as e:
            if attempt == 2:  # Last attempt
                print(f"Postgres health check failed after 3 attempts: {str(e)}")
        await asyncio.sleep(0.5)  # Wait 0.5 seconds between attempts
    return "disconnected"


async def check_gemini_health() -> Literal["connected", "disconnected"]:
    """Asynchronously check Gemini health with retries."""
    for attempt in range(3):  # Try up to 3 times
        try:
            genai.configure(api_key=settings.GEMINI_API_KEY)
            # Check if we can list models as a basic connectivity test
            # This is a lightweight operation that doesn't consume quota
            models = genai.list_models()
            # Verify that we have at least one model available
            if models:
                for model in models:
                    if model and model.name:
                        return "connected"
        except Exception as e:
            error_msg = str(e).lower()
            if "quota" in error_msg or "rate limit" in error_msg or "429" in error_msg:
                # If it's a quota issue, the API is technically connected but limited
                return "connected"
            if attempt == 2:  # Last attempt
                print(f"Gemini health check failed after 3 attempts: {str(e)}")
        await asyncio.sleep(1)  # Wait 1 second between attempts
    return "disconnected"


@router.get("", response_model=HealthResponse)
async def health_check():
    """Health check endpoint that verifies the service and its dependencies are operational."""

    # Initialize service status dictionary
    services_status: Dict[str, str] = {
        "qdrant": "disconnected",
        "postgres": "disconnected",
        "gemini": "disconnected"
    }

    # Run health checks concurrently for better performance
    qdrant_task = check_qdrant_health()
    postgres_task = check_postgres_health()
    gemini_task = check_gemini_health()

    # Execute all health checks concurrently
    qdrant_result, postgres_result, gemini_result = await asyncio.gather(
        qdrant_task,
        postgres_task,
        gemini_task
    )

    # Update service status
    services_status["qdrant"] = qdrant_result
    services_status["postgres"] = postgres_result
    services_status["gemini"] = gemini_result

    # Get system health from monitoring service
    try:
        system_health = monitoring_service.get_system_health()
        system_status = system_health["status"]
        if system_status == "critical":
            system_status = "unhealthy"
    except Exception as e:
        print(f"Monitoring service health check failed: {str(e)}")
        system_status = "unhealthy"

    # Determine overall health status
    # All three services must be connected for the system to be healthy
    all_services_connected = all(status == "connected" for status in services_status.values())

    # Prioritize service connectivity over system metrics for the overall health status
    # If all services are connected, the system is healthy regardless of monitoring status
    # Only return unhealthy if services are disconnected
    overall_status = "healthy" if all_services_connected else "unhealthy"

    return HealthResponse(
        status=overall_status,
        timestamp=datetime.utcnow(),
        services=services_status
    )