import os
from typing import Optional
from openai import OpenAI


class GeminiService:
    """Service class to handle interactions with the Google Gemini API through OpenAI-compatible interface."""
    
    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize the Gemini service using OpenAI client with Google endpoint.
        
        Args:
            api_key: Optional API key. If not provided, will use GEMINI_API_KEY from environment.
        """
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("Gemini API key is required")
        
        # Use OpenAI client but with Google's endpoint to access Gemini models
        self.client = OpenAI(
            api_key=self.api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )
    
    def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """
        Generate a response from the Gemini model through OpenAI-compatible interface.
        
        Args:
            prompt: The user's query or instruction
            context: Optional context that was highlighted by the user
            
        Returns:
            Generated response from the model
        """
        try:
            # Combine context and prompt
            full_prompt = prompt
            if context:
                full_prompt = f"Context: {context}\n\nUser Query: {prompt}"
            
            print(f"Sending request to Gemini with prompt: {full_prompt[:100]}...")  # Log first 100 chars
            
            # Use the OpenAI-compatible chat completions API to access Google models
            response = self.client.chat.completions.create(
                model="gemini-2.5-flash",  # Google's model accessed through OpenAI interface
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that explains technical concepts clearly."},
                    {"role": "user", "content": full_prompt}
                ],
                # No max_tokens specified to allow unlimited response length
                temperature=0.5
            )
            
            print(f"Response received: {response}")  # Debug log the full response
            print(f"Response choices: {response.choices}")  # Debug the choices
            if response.choices and len(response.choices) > 0:
                choice = response.choices[0]
                print(f"Choice: {choice}")
                print(f"Choice message: {choice.message}")
                print(f"Choice message content: {choice.message.content}")
                
                # Check if the response has finish_reason
                print(f"Finish reason: {choice.finish_reason}")
            
                content = choice.message.content
                if content is not None:
                    return content
                else:
                    print(f"Content is None, finish reason: {choice.finish_reason}")
                    if choice.finish_reason == "content_filter":
                        return "Response was blocked due to content filtering."
                    elif choice.finish_reason == "length":
                        # Even if it was cut off due to length, there might be partial content
                        # But since content is None, we return a specific message
                        return "Response was generated but was cut off due to length limitations."
                    else:
                        return f"No content returned. Finish reason: {choice.finish_reason or 'unknown'}"
            else:
                return "No choices returned in the response."
            
        except Exception as e:
            # Log the error with more detail
            print(f"Error generating response: {str(e)}")
            print(f"Error type: {type(e)}")
            import traceback
            traceback.print_exc()  # Print full stack trace
            raise
    
    def validate_api_key(self) -> bool:
        """
        Validate the API key by making a simple request.
        
        Returns:
            True if the API key is valid, False otherwise
        """
        try:
            # Make a simple test request to the Google endpoint through OpenAI interface
            response = self.client.chat.completions.create(
                model="gemini-2.5-flash",
                messages=[
                    {"role": "user", "content": "test"}
                ],
                max_tokens=5
            )
            
            # If we successfully get a response object (no exception thrown), 
            # the API key is likely valid - Google's OpenAI-compatible endpoint
            # may return responses in various formats
            print(f"API validation successful, response type: {type(response)}")
            return True
        except Exception as e:
            print(f"API key validation error: {str(e)}")
            return False