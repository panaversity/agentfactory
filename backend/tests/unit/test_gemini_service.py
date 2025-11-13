import pytest
from unittest.mock import patch, MagicMock
from backend.src.services.gemini_service import GeminiService


class TestGeminiService:
    """Unit tests for the GeminiService class."""

    def test_initialization_with_api_key(self):
        """Test initializing GeminiService with an API key."""
        api_key = "test_api_key"
        service = GeminiService(api_key=api_key)
        
        assert service.api_key == api_key

    def test_initialization_without_api_key_uses_env(self):
        """Test initializing GeminiService without API key uses environment variable."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'env_api_key'}):
            service = GeminiService()
            
            assert service.api_key == 'env_api_key'

    def test_initialization_without_api_key_raises_error(self):
        """Test initializing GeminiService without API key raises ValueError."""
        with patch.dict('os.environ', {}, clear=True):
            with pytest.raises(ValueError, match="Gemini API key is required"):
                GeminiService()

    @patch('backend.src.services.gemini_service.genai')
    @patch('backend.src.services.gemini_service.os.getenv')
    async def test_generate_response_success(self, mock_getenv, mock_genai):
        """Test successful response generation."""
        # Mock environment
        mock_getenv.return_value = 'test_api_key'
        
        # Mock Gemini API
        mock_model_instance = MagicMock()
        mock_response = MagicMock()
        mock_response.text = 'Test response'
        mock_model_instance.generate_content_async = MagicMock(return_value=mock_response)
        mock_genai.GenerativeModel.return_value = mock_model_instance
        mock_genai.configure = MagicMock()
        
        service = GeminiService(api_key='test_api_key')
        
        result = await service.generate_response(prompt='Test prompt')
        
        assert result == 'Test response'
        mock_genai.GenerativeModel.assert_called_once_with('gemini-2.5-flash')
        mock_model_instance.generate_content_async.assert_called_once()

    @patch('backend.src.services.gemini_service.genai')
    @patch('backend.src.services.gemini_service.os.getenv')
    async def test_generate_response_with_context_success(self, mock_getenv, mock_genai):
        """Test successful response generation with context."""
        # Mock environment
        mock_getenv.return_value = 'test_api_key'
        
        # Mock Gemini API
        mock_model_instance = MagicMock()
        mock_response = MagicMock()
        mock_response.text = 'Test response with context'
        mock_model_instance.generate_content_async = MagicMock(return_value=mock_response)
        mock_genai.GenerativeModel.return_value = mock_model_instance
        mock_genai.configure = MagicMock()
        
        service = GeminiService(api_key='test_api_key')
        
        result = await service.generate_response(prompt='Test prompt', context='Test context')
        
        assert result == 'Test response with context'
        # Verify that the prompt was combined with context
        mock_model_instance.generate_content_async.assert_called_once()

    @patch('backend.src.services.gemini_service.genai')
    @patch('backend.src.services.gemini_service.os.getenv')
    def test_validate_api_key_success(self, mock_getenv, mock_genai):
        """Test successful API key validation."""
        # Mock environment
        mock_getenv.return_value = 'test_api_key'
        
        # Mock Gemini API
        mock_model_instance = MagicMock()
        mock_response = MagicMock()
        mock_model_instance.generate_content = MagicMock(return_value=mock_response)
        mock_genai.GenerativeModel.return_value = mock_model_instance
        mock_genai.configure = MagicMock()
        
        service = GeminiService(api_key='test_api_key')
        
        result = service.validate_api_key()
        
        assert result is True

    @patch('backend.src.services.gemini_service.genai')
    @patch('backend.src.services.gemini_service.os.getenv')
    def test_validate_api_key_failure(self, mock_getenv, mock_genai):
        """Test API key validation failure."""
        # Mock environment
        mock_getenv.return_value = 'test_api_key'
        
        # Mock Gemini API to raise an exception
        mock_genai.GenerativeModel.side_effect = Exception("Invalid API key")
        mock_genai.configure = MagicMock()
        
        service = GeminiService(api_key='test_api_key')
        
        result = service.validate_api_key()
        
        assert result is False