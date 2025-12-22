"""
Metrics endpoint for the RAG Chatbot backend.
Provides detailed performance metrics and monitoring data.
"""
from fastapi import APIRouter
from typing import Dict, Any
from src.utils.monitoring import monitoring_service


router = APIRouter()


@router.get("/metrics")
async def get_metrics() -> Dict[str, Any]:
    """Get detailed performance metrics and system health."""
    return monitoring_service.get_detailed_metrics()


@router.get("/health/extended")
async def extended_health_check() -> Dict[str, Any]:
    """Get extended health information including performance metrics."""
    system_health = monitoring_service.get_system_health()
    performance_metrics = monitoring_service.get_performance_metrics()
    
    return {
        "health": system_health,
        "performance": performance_metrics,
        "timestamp": system_health.get("timestamp")
    }