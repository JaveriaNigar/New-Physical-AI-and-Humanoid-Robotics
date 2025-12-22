"""
Rate limiting implementation for the RAG Chatbot backend.
Prevents abuse by limiting the number of requests per IP address.
"""
import time
from collections import defaultdict, deque
from typing import Dict
from src.config.settings import settings


class RateLimiter:
    """Manages rate limiting for API endpoints to prevent abuse."""
    
    def __init__(self):
        """Initialize the rate limiter with configuration from settings."""
        # Dictionary to store request timestamps for each IP
        self.requests: Dict[str, deque] = defaultdict(deque)
        self.rate_limit = settings.RATE_LIMIT_REQUESTS  # requests per minute
        self.time_window = 60  # seconds
    
    def is_allowed(self, ip_address: str) -> bool:
        """
        Check if a request from the given IP is allowed based on rate limits.
        
        Args:
            ip_address: The IP address of the client making the request
            
        Returns:
            True if the request is allowed, False otherwise
        """
        current_time = time.time()
        
        # Clean up old requests that are outside the time window
        while (self.requests[ip_address] and 
               current_time - self.requests[ip_address][0] > self.time_window):
            self.requests[ip_address].popleft()
        
        # Check if the number of requests is within the limit
        if len(self.requests[ip_address]) < self.rate_limit:
            # Add current request timestamp
            self.requests[ip_address].append(current_time)
            return True
        
        # Rate limit exceeded
        return False
    
    def get_remaining_requests(self, ip_address: str) -> int:
        """
        Get the number of remaining requests for the given IP within the time window.
        
        Args:
            ip_address: The IP address to check
            
        Returns:
            Number of remaining requests allowed
        """
        current_time = time.time()
        
        # Clean up old requests
        while (self.requests[ip_address] and 
               current_time - self.requests[ip_address][0] > self.time_window):
            self.requests[ip_address].popleft()
        
        return max(0, self.rate_limit - len(self.requests[ip_address]))
    
    def reset_for_ip(self, ip_address: str):
        """
        Reset the rate limit counter for a specific IP address.
        
        Args:
            ip_address: The IP address to reset
        """
        if ip_address in self.requests:
            del self.requests[ip_address]


# Global rate limiter instance
rate_limiter = RateLimiter()