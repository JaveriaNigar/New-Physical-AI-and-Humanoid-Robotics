"""
Script to list available Google Gemini models.
"""
import google.generativeai as genai
from src.config.settings import settings

def list_models():
    """List available models using the Google Generative AI API."""
    # Configure the Google Generative AI client
    genai.configure(api_key=settings.GEMINI_API_KEY)
    
    try:
        # List all available models
        models = genai.list_models()
        
        print("Available models:")
        for model in models:
            print(f"  - {model.name}")
            print(f"    Supported operations: {[m for m in model.supported_generation_methods]}")
            
        # Filter for text generation models
        text_models = [model for model in models if 'generateContent' in model.supported_generation_methods]
        print(f"\nText generation models: {len(text_models)}")
        for model in text_models:
            print(f"  - {model.name}")
    
    except Exception as e:
        print(f"Error listing models: {str(e)}")
        print("\nThis error might occur if the API key is invalid or the service is not available.")

if __name__ == "__main__":
    list_models()