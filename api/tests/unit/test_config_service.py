import pytest
from unittest.mock import patch, MagicMock
from backend.src.services.config_service import ConfigService


class TestConfigService:
    """Unit tests for the ConfigService class."""

    @patch('backend.src.services.config_service.OpenAI')
    def test_validate_gemini_api_key_valid(self, mock_openai_class):
        """Test validating a valid Gemini API key."""
        # Mock a successful response from the OpenAI-compatible Gemini API
        mock_client_instance = MagicMock()
        mock_response = MagicMock()
        mock_response.choices[0].message.content = "test response"
        mock_client_instance.chat.completions.create.return_value = mock_response
        mock_openai_class.return_value = mock_client_instance
        
        result = ConfigService.validate_gemini_api_key("valid_api_key")
        
        assert result is True
        mock_openai_class.assert_called_once_with(
            api_key="valid_api_key",
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        mock_client_instance.chat.completions.create.assert_called_once_with(
            model="gemini-2.5-flash",
            messages=[{"role": "user", "content": "test"}],
            max_tokens=5
        )

    @patch('backend.src.services.config_service.OpenAI')
    def test_validate_gemini_api_key_invalid(self, mock_openai_class):
        """Test validating an invalid Gemini API key."""
        # Mock an exception when calling the OpenAI-compatible Gemini API
        mock_client_instance = MagicMock()
        mock_client_instance.chat.completions.create.side_effect = Exception("Invalid API key")
        mock_openai_class.return_value = mock_client_instance
        
        result = ConfigService.validate_gemini_api_key("invalid_api_key")
        
        assert result is False
        mock_openai_class.assert_called_once_with(
            api_key="invalid_api_key",
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        mock_client_instance.chat.completions.create.assert_called_once_with(
            model="gemini-2.5-flash",
            messages=[{"role": "user", "content": "test"}],
            max_tokens=5
        )

    @patch('backend.src.services.config_service.OpenAI')
    def test_validate_gemini_api_key_exception_during_request(self, mock_openai_class):
        """Test validating an API key that causes an exception during the request."""
        # Mock a successful client creation but an exception during the chat.completions.create call
        mock_client_instance = MagicMock()
        mock_client_instance.chat.completions.create.side_effect = Exception("Request failed")
        mock_openai_class.return_value = mock_client_instance
        
        result = ConfigService.validate_gemini_api_key("failing_api_key")
        
        assert result is False
        mock_openai_class.assert_called_once_with(
            api_key="failing_api_key",
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        mock_client_instance.chat.completions.create.assert_called_once_with(
            model="gemini-2.5-flash",
            messages=[{"role": "user", "content": "test"}],
            max_tokens=5
        )

    def test_get_current_config_status_not_configured(self):
        """Test getting config status when API key is not configured."""
        with patch.dict('os.environ', {}, clear=True):
            result = ConfigService.get_current_config_status()
        
        assert result == {
            "api_key_configured": False,
            "api_key_valid": None,
            "model": "gemini-2.5-flash"
        }

    @patch('backend.src.services.config_service.ConfigService.validate_gemini_api_key')
    def test_get_current_config_status_configured_valid(self, mock_validate):
        """Test getting config status when API key is configured and valid."""
        mock_validate.return_value = True
        
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'valid_api_key'}):
            result = ConfigService.get_current_config_status()
        
        assert result == {
            "api_key_configured": True,
            "api_key_valid": True,
            "model": "gemini-2.5-flash"
        }
        mock_validate.assert_called_once_with('valid_api_key')

    @patch('backend.src.services.config_service.ConfigService.validate_gemini_api_key')
    def test_get_current_config_status_configured_invalid(self, mock_validate):
        """Test getting config status when API key is configured but invalid."""
        mock_validate.return_value = False
        
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'invalid_api_key'}):
            result = ConfigService.get_current_config_status()
        
        assert result == {
            "api_key_configured": True,
            "api_key_valid": False,
            "model": "gemini-2.5-flash"
        }
        mock_validate.assert_called_once_with('invalid_api_key')