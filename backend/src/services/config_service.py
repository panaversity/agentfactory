import os
from typing import Dict, Optional
from openai import OpenAI


class ConfigService:
    """Service to handle configuration settings, particularly API keys."""
    
    @staticmethod
    def validate_gemini_api_key(api_key: str) -> bool:
        """
        Validate a Gemini API key by making a test request through OpenAI-compatible interface.
        
        Args:
            api_key: The API key to validate
            
        Returns:
            True if the API key is valid, False otherwise
        """
        try:
            # Create a client instance with the provided key using Google's endpoint
            client = OpenAI(
                api_key=api_key,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
            )
            
            # Try to make a simple request to validate the key
            response = client.chat.completions.create(
                model="gemini-2.5-flash",
                messages=[{"role": "user", "content": "test"}],
                max_tokens=5
            )
            
            # If we get here, the key is valid
            content = response.choices[0].message.content
            return content is not None and len(content) > 0
        except Exception as e:
            # Log the error for debugging
            print(f"API key validation error: {str(e)}")
            return False
    
    @staticmethod
    def get_current_config_status() -> Dict[str, Optional[str]]:
        """
        Get the current configuration status.
        
        Returns:
            Dictionary with configuration status information
        """
        api_key = os.getenv("GEMINI_API_KEY")
        
        status = {
            "api_key_configured": bool(api_key),
            "api_key_valid": None,
            "model": "gemini-2.5-flash"  # Using Google's model
        }
        
        if api_key:
            status["api_key_valid"] = ConfigService.validate_gemini_api_key(api_key)
        
        return status