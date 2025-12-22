"""
Comprehensive monitoring and metrics for the RAG Chatbot backend.
Provides performance metrics, health checks, and observability.
"""
import time
import psutil
import GPUtil
from typing import Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime
import logging

from src.config.settings import settings


@dataclass
class PerformanceMetrics:
    """Data class for storing performance metrics."""
    timestamp: datetime
    response_time: float
    memory_usage: float
    cpu_usage: float
    gpu_usage: Optional[float] = None
    active_requests: int = 0
    error_rate: float = 0.0
    retrieval_time: Optional[float] = None
    generation_time: Optional[float] = None


class MetricsCollector:
    """Collects and manages performance metrics for the RAG system."""
    
    def __init__(self):
        """Initialize the metrics collector."""
        self.logger = logging.getLogger(__name__)
        self.metrics_history = []
        self.active_requests = 0
        self.request_count = 0
        self.error_count = 0
        self.start_time = time.time()
    
    def start_request(self):
        """Track a new active request."""
        self.active_requests += 1
        self.request_count += 1
    
    def end_request(self, has_error: bool = False):
        """Track the end of a request."""
        self.active_requests -= 1
        if has_error:
            self.error_count += 1
    
    def get_system_metrics(self) -> Dict[str, Any]:
        """Get current system resource usage."""
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_info = psutil.virtual_memory()
        memory_percent = memory_info.percent
        
        # Try to get GPU metrics if available
        gpu_percent = None
        gpus = GPUtil.getGPUs()
        if gpus:
            gpu_percent = gpus[0].load * 100  # Use first GPU
        
        return {
            "cpu_percent": cpu_percent,
            "memory_percent": memory_percent,
            "memory_available_gb": memory_info.available / (1024**3),
            "memory_total_gb": memory_info.total / (1024**3),
            "gpu_percent": gpu_percent,
            "active_requests": self.active_requests,
            "uptime_seconds": time.time() - self.start_time,
            "total_requests": self.request_count,
            "error_count": self.error_count,
            "error_rate": self.error_count / max(1, self.request_count)
        }
    
    def collect_metrics(
        self,
        response_time: Optional[float] = None,
        retrieval_time: Optional[float] = None,
        generation_time: Optional[float] = None
    ) -> PerformanceMetrics:
        """Collect performance metrics for a request."""
        system_metrics = self.get_system_metrics()
        
        metrics = PerformanceMetrics(
            timestamp=datetime.utcnow(),
            response_time=response_time or 0,
            memory_usage=system_metrics["memory_percent"],
            cpu_usage=system_metrics["cpu_percent"],
            gpu_usage=system_metrics["gpu_percent"],
            active_requests=system_metrics["active_requests"],
            error_rate=system_metrics["error_rate"],
            retrieval_time=retrieval_time,
            generation_time=generation_time
        )
        
        # Store in history (keep last 1000 metrics)
        self.metrics_history.append(metrics)
        if len(self.metrics_history) > 1000:
            self.metrics_history.pop(0)
        
        return metrics
    
    def get_average_metrics(self, minutes: int = 5) -> Dict[str, float]:
        """Get average metrics for the last N minutes."""
        cutoff_time = time.time() - (minutes * 60)
        recent_metrics = [
            m for m in self.metrics_history
            if (datetime.utcnow() - m.timestamp).total_seconds() < minutes * 60
        ]
        
        if not recent_metrics:
            return {}
        
        avg_response_time = sum(m.response_time for m in recent_metrics) / len(recent_metrics)
        avg_memory = sum(m.memory_usage for m in recent_metrics) / len(recent_metrics)
        avg_cpu = sum(m.cpu_usage for m in recent_metrics) / len(recent_metrics)
        
        result = {
            "avg_response_time": avg_response_time,
            "avg_memory_usage": avg_memory,
            "avg_cpu_usage": avg_cpu,
            "sample_size": len(recent_metrics)
        }
        
        # Add optional metrics if available
        valid_retrieval_times = [m.retrieval_time for m in recent_metrics if m.retrieval_time is not None]
        if valid_retrieval_times:
            result["avg_retrieval_time"] = sum(valid_retrieval_times) / len(valid_retrieval_times)
        
        valid_generation_times = [m.generation_time for m in recent_metrics if m.generation_time is not None]
        if valid_generation_times:
            result["avg_generation_time"] = sum(valid_generation_times) / len(valid_generation_times)
        
        return result
    
    def get_health_status(self) -> Dict[str, Any]:
        """Get overall health status of the system."""
        system_metrics = self.get_system_metrics()
        
        # Determine health based on thresholds
        memory_critical = system_metrics["memory_percent"] > 90
        cpu_critical = system_metrics["cpu_percent"] > 95
        error_rate_critical = system_metrics["error_rate"] > 0.05  # 5% error rate
        active_requests_high = system_metrics["active_requests"] > 50  # Adjust as needed
        
        status = "healthy"
        if memory_critical or cpu_critical or error_rate_critical:
            status = "critical"
        elif active_requests_high or system_metrics["error_rate"] > 0.02:
            status = "degraded"
        
        return {
            "status": status,
            "timestamp": datetime.utcnow().isoformat(),
            "system_metrics": system_metrics,
            "issues": {
                "high_memory": memory_critical,
                "high_cpu": cpu_critical,
                "high_error_rate": error_rate_critical,
                "high_active_requests": active_requests_high
            }
        }


class MonitoringService:
    """Main monitoring service that provides metrics and health checks."""
    
    def __init__(self):
        """Initialize the monitoring service."""
        self.metrics_collector = MetricsCollector()
        self.logger = logging.getLogger(__name__)
    
    def record_request(
        self,
        response_time: float,
        retrieval_time: Optional[float] = None,
        generation_time: Optional[float] = None,
        has_error: bool = False
    ):
        """Record metrics for a completed request."""
        self.metrics_collector.end_request(has_error)
        return self.metrics_collector.collect_metrics(
            response_time,
            retrieval_time,
            generation_time
        )
    
    def get_system_health(self) -> Dict[str, Any]:
        """Get the current health status of the system."""
        return self.metrics_collector.get_health_status()
    
    def get_performance_metrics(self, minutes: int = 5) -> Dict[str, Any]:
        """Get performance metrics for the specified time window."""
        return self.metrics_collector.get_average_metrics(minutes)
    
    def get_detailed_metrics(self) -> Dict[str, Any]:
        """Get detailed metrics including system resources."""
        system_metrics = self.metrics_collector.get_system_metrics()
        performance_metrics = self.metrics_collector.get_average_metrics(5)
        
        return {
            "system": system_metrics,
            "performance": performance_metrics,
            "health_status": self.get_system_health()
        }


# Global monitoring service instance
monitoring_service = MonitoringService()