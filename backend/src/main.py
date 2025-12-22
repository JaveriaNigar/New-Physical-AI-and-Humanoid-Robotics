"""
Main FastAPI application entry point for the RAG Chatbot backend.
"""
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
import time

from src.api.health_routes import router as health_router
from src.api.chat_routes import router as chat_router
from src.api.metrics_routes import router as metrics_router
from src.utils.rate_limiter import rate_limiter
from src.config.settings import settings
from src.utils.logging_config import log_api_request

# Create FastAPI application instance
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Retrieval-Augmented Generation chatbot integrated with AI-powered textbook",
    version="1.0.0"
)

# Security headers middleware
class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request, call_next):
        response = await call_next(request)
        # Add security headers to all responses
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['X-Frame-Options'] = 'DENY'
        response.headers['X-XSS-Protection'] = '1; mode=block'
        response.headers['Strict-Transport-Security'] = 'max-age=31536000; includeSubDomains'
        response.headers['Referrer-Policy'] = 'no-referrer-when-downgrade'
        response.headers['Content-Security-Policy'] = "default-src 'self'; script-src 'self'"
        return response

# Add security headers middleware
app.add_middleware(SecurityHeadersMiddleware)

# Add trusted host middleware
if settings.ENVIRONMENT != "development":
    app.add_middleware(
        TrustedHostMiddleware,
        allowed_hosts=["yourdomain.com", "www.yourdomain.com"]  # Replace with actual domains
    )

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS if settings.ALLOWED_ORIGINS != [""] else ["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Additional option to allow credentials from same origin
    allow_origin_regex=None,
    # Expose headers to frontend
    expose_headers=["Access-Control-Allow-Origin"]
)

# Rate limiting middleware
@app.middleware("http")
async def add_rate_limiting(request, call_next):
    # Skip rate limiting for health checks and other internal endpoints
    if request.url.path in ["/health", "/health/", "/metrics", "/metrics/"]:
        response = await call_next(request)
        return response

    # Get client IP
    client_ip = request.client.host

    # Check if request is allowed
    if not rate_limiter.is_allowed(client_ip):
        return Response(
            status_code=429,
            content="Rate limit exceeded. Please try again later."
        )

    # Log API request
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time

    # Log the request with timing information
    log_api_request(
        endpoint=request.url.path,
        method=request.method,
        user_id=None,  # Will be filled in when authentication is implemented
        session_id=request.headers.get("session-id", "unknown"),
        query=request.url.query,
        response_status=response.status_code,
        response_time=process_time
    )

    return response

# Register API routes
app.include_router(health_router, prefix="/health", tags=["health"])
app.include_router(metrics_router, prefix="/metrics", tags=["metrics"])
app.include_router(chat_router, prefix="/chat", tags=["chat"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=os.getenv("HOST", "0.0.0.0"),
        port=int(os.getenv("PORT", 8001)),
        reload=os.getenv("RELOAD", "false").lower() == "true"
    )