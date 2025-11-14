import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import os

# Import the FastAPI app
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from backend.main import app


client = TestClient(app)


class TestAIQueryEndpoint:
    """Integration tests for the AI query endpoint."""

    @patch('backend.src.api.routes.GeminiService')
    def test_ai_query_success(self, mock_gemini_service_class):
        """Test successful AI query request."""
        # Mock the Gemini service
        mock_service_instance = MagicMock()
        mock_service_instance.generate_response = MagicMock(return_value='Test response')
        mock_gemini_service_class.return_value = mock_service_instance
        
        # Set up environment for API key
        with patch.dict(os.environ, {'GEMINI_API_KEY': 'test_api_key'}):
            response = client.post(
                "/api/ai/query",
                json={
                    "highlighted_text": "Test highlighted text",
                    "query": "Explain this"
                }
            )
        
        assert response.status_code == 200
        assert response.json() == {"response": "Test response"}
        mock_gemini_service_class.assert_called_once_with(api_key='test_api_key')

    @patch('backend.src.api.routes.GeminiService')
    def test_ai_query_without_highlighted_text(self, mock_gemini_service_class):
        """Test AI query without highlighted text."""
        # Mock the Gemini service
        mock_service_instance = MagicMock()
        mock_service_instance.generate_response = MagicMock(return_value='Test response')
        mock_gemini_service_class.return_value = mock_service_instance
        
        # Set up environment for API key
        with patch.dict(os.environ, {'GEMINI_API_KEY': 'test_api_key'}):
            response = client.post(
                "/api/ai/query",
                json={
                    "query": "Explain this"
                }
            )
        
        assert response.status_code == 200
        assert response.json() == {"response": "Test response"}

    def test_ai_query_without_query_fails(self):
        """Test AI query without query parameter fails."""
        response = client.post(
            "/api/ai/query",
            json={
                "highlighted_text": "Test highlighted text"
            }
        )
        
        assert response.status_code == 400
        assert "Query is required" in response.json()["detail"]

    def test_ai_query_without_api_key_fails(self):
        """Test AI query without API key fails."""
        # Clear the environment API key
        with patch.dict(os.environ, {}, clear=True):
            response = client.post(
                "/api/ai/query",
                json={
                    "query": "Explain this"
                }
            )
        
        assert response.status_code == 500
        assert "API key not configured" in response.json()["detail"]


class TestConfigEndpoints:
    """Integration tests for the config endpoints."""

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

    def test_set_gemini_key_without_key_fails(self):
        """Test setting API key without key fails."""
        response = client.post(
            "/api/config/gemini-key",
            json={}
        )
        
        assert response.status_code == 400
        assert "API key is required" in response.json()["detail"]

    @patch('backend.src.api.routes.GeminiService')
    def test_set_gemini_key_invalid_key_fails(self, mock_gemini_service_class):
        """Test setting invalid API key fails."""
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
        assert "Invalid API key" in response.json()["detail"]

    def test_get_config_status_not_configured(self):
        """Test getting config status when not configured."""
        # Clear the environment API key
        with patch.dict(os.environ, {}, clear=True):
            response = client.get("/api/config/status")
        
        assert response.status_code == 200
        data = response.json()
        assert data["is_configured"] is False
        assert data["is_valid"] is None

    @patch('backend.src.api.routes.GeminiService')
    def test_get_config_status_configured_valid(self, mock_gemini_service_class):
        """Test getting config status when configured and valid."""
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
        """Test getting config status when configured but invalid."""
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
    def test_get_config_status_configured_error(self, mock_gemini_service_class):
        """Test getting config status when configured but validation fails."""
        # Mock the Gemini service to raise an exception
        mock_service_instance = MagicMock()
        mock_service_instance.validate_api_key = MagicMock(side_effect=Exception("Validation error"))
        mock_gemini_service_class.return_value = mock_service_instance
        
        with patch.dict(os.environ, {'GEMINI_API_KEY': 'error_api_key'}):
            response = client.get("/api/config/status")
        
        assert response.status_code == 200
        data = response.json()
        assert data["is_configured"] is True
        assert data["is_valid"] is False
        assert "Validation error" in data.get("message", "")


class TestHealthEndpoint:
    """Integration tests for the health endpoint."""

    def test_health_check(self):
        """Test health check endpoint."""
        response = client.get("/health")
        
        assert response.status_code == 200
        assert response.json() == {
            "status": "healthy", 
            "service": "Highlight Selection AI Dialog Backend"
        }