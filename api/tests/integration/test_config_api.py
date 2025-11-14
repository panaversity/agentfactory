import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import os

# Import the FastAPI app
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from backend.main import app


client = TestClient(app)


class TestConfigAPIEndpoints:
    """Integration tests for the configuration API endpoints."""

    @patch('backend.src.api.routes.GeminiService')
    def test_set_gemini_key_success(self, mock_gemini_service_class):
        """Test successful API key configuration."""
        # Mock the Gemini service validation
        mock_service_instance = MagicMock()
        mock_service_instance.validate_api_key = MagicMock(return_value=True)
        mock_gemini_service_class.return_value = mock_service_instance
        
        response = client.post(
            "/api/config/gemini-key",
            json={
                "api_key": "valid_api_key"
            }
        )
        
        assert response.status_code == 200
        assert response.json() == {"status": "success"}
        mock_gemini_service_class.assert_called_once()

    def test_set_gemini_key_without_key_fails(self):
        """Test setting API key without providing the key fails."""
        response = client.post(
            "/api/config/gemini-key",
            json={}
        )
        
        assert response.status_code == 400
        data = response.json()
        assert "API key is required" in data["detail"]

    @patch('backend.src.api.routes.GeminiService')
    def test_set_gemini_key_invalid_key_fails(self, mock_gemini_service_class):
        """Test setting API key with an invalid key fails."""
        # Mock the Gemini service to return False for validation
        mock_service_instance = MagicMock()
        mock_service_instance.validate_api_key = MagicMock(return_value=False)
        mock_gemini_service_class.return_value = mock_service_instance
        
        response = client.post(
            "/api/config/gemini-key",
            json={
                "api_key": "invalid_api_key"
            }
        )
        
        assert response.status_code == 400
        data = response.json()
        assert "Invalid API key" in data["detail"]

    @patch('backend.src.api.routes.GeminiService')
    def test_set_gemini_key_with_exception_fails(self, mock_gemini_service_class):
        """Test setting API key with an exception during validation fails."""
        # Mock the Gemini service to raise an exception during validation
        mock_service_instance = MagicMock()
        mock_service_instance.validate_api_key = MagicMock(side_effect=Exception("Validation error"))
        mock_gemini_service_class.return_value = mock_service_instance
        
        response = client.post(
            "/api/config/gemini-key",
            json={
                "api_key": "exception_key"
            }
        )
        
        assert response.status_code == 400
        data = response.json()
        assert "Invalid API key" in data["detail"]

    def test_get_config_status_not_configured(self):
        """Test getting config status when API key is not configured."""
        # Clear the environment API key
        with patch.dict(os.environ, {}, clear=True):
            response = client.get("/api/config/status")
        
        assert response.status_code == 200
        data = response.json()
        assert data["is_configured"] is False
        assert data["is_valid"] is None

    @patch('backend.src.api.routes.GeminiService')
    def test_get_config_status_configured_valid(self, mock_gemini_service_class):
        """Test getting config status when API key is configured and valid."""
        # Mock the Gemini service validation
        mock_service_instance = MagicMock()
        mock_service_instance.validate_api_key = MagicMock(return_value=True)
        mock_gemini_service_class.return_value = mock_service_instance
        
        with patch.dict(os.environ, {'GEMINI_API_KEY': 'valid_api_key'}):
            response = client.get("/api/config/status")
        
        assert response.status_code == 200
        data = response.json()
        assert data["is_configured"] is True
        assert data["is_valid"] is True

    @patch('backend.src.api.routes.GeminiService')
    def test_get_config_status_configured_invalid(self, mock_gemini_service_class):
        """Test getting config status when API key is configured but invalid."""
        # Mock the Gemini service to return False for validation
        mock_service_instance = MagicMock()
        mock_service_instance.validate_api_key = MagicMock(return_value=False)
        mock_gemini_service_class.return_value = mock_service_instance
        
        with patch.dict(os.environ, {'GEMINI_API_KEY': 'invalid_api_key'}):
            response = client.get("/api/config/status")
        
        assert response.status_code == 200
        data = response.json()
        assert data["is_configured"] is True
        assert data["is_valid"] is False

    @patch('backend.src.api.routes.GeminiService')
    def test_get_config_status_configured_exception(self, mock_gemini_service_class):
        """Test getting config status when API key is configured but validation raises exception."""
        # Mock the Gemini service to raise an exception during validation
        mock_service_instance = MagicMock()
        mock_service_instance.validate_api_key = MagicMock(side_effect=Exception("Validation error"))
        mock_gemini_service_class.return_value = mock_service_instance
        
        with patch.dict(os.environ, {'GEMINI_API_KEY': 'exception_api_key'}):
            response = client.get("/api/config/status")
        
        assert response.status_code == 200
        data = response.json()
        assert data["is_configured"] is True
        assert data["is_valid"] is False
        assert "Validation error" in data.get("message", "")

    @patch('backend.src.api.routes.GeminiService')
    def test_get_config_status_error_handling(self, mock_gemini_service_class):
        """Test error handling in get config status endpoint."""
        # Mock the Gemini service to raise an exception during setup
        mock_gemini_service_class.side_effect = Exception("Service setup error")
        
        with patch.dict(os.environ, {'GEMINI_API_KEY': 'problematic_api_key'}):
            response = client.get("/api/config/status")
        
        # The endpoint should still return a 200 but with error information
        # in the message field, since the service validates on each call
        assert response.status_code == 200


class TestHealthEndpointIntegration:
    """Integration tests for the health endpoint."""

    def test_health_endpoint(self):
        """Test the health check endpoint works."""
        response = client.get("/health")
        
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert data["service"] == "Highlight Selection AI Dialog Backend"