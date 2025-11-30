"""Tests for API endpoints."""

import pytest
from fastapi.testclient import TestClient
from app import app


@pytest.fixture
def client():
    """Test client fixture."""
    return TestClient(app)


def test_root_endpoint(client):
    """Test root endpoint returns service info."""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["service"] == "RoboLearn Backend"
    assert "modules" in data
    assert "endpoints" in data
    assert "tool" in data


def test_health_endpoint(client):
    """Test health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert "qdrant" in data
    assert "collection" in data
    # Status can be healthy, degraded, or unhealthy - all are valid responses
    assert data["status"] in ["healthy", "degraded", "unhealthy"]


def test_search_endpoint_missing_query(client):
    """Test search endpoint requires query."""
    response = client.post("/search", json={})
    assert response.status_code == 422  # Validation error


def test_search_endpoint_short_query(client):
    """Test search endpoint requires query of at least 3 characters."""
    response = client.post("/search", json={"query": "ab"})
    assert response.status_code == 422  # Validation error


def test_search_endpoint_valid_request(client):
    """Test search endpoint with valid request."""
    response = client.post(
        "/search",
        json={
            "query": "How do I create a ROS 2 node?",
            "limit": 5
        }
    )
    # May return 200 (if Qdrant available) or 503 (if unavailable)
    assert response.status_code in [200, 503]
    
    if response.status_code == 200:
        data = response.json()
        assert "query" in data
        assert "results" in data
        assert "total_found" in data
        assert isinstance(data["results"], list)
    else:
        # Service unavailable - check error message
        assert "unavailable" in response.json()["detail"].lower()


def test_search_endpoint_with_filters(client):
    """Test search endpoint with filters."""
    response = client.post(
        "/search",
        json={
            "query": "ROS 2",
            "hardware_tier": 2,
            "module": "ros2",
            "chapter_min": 1,
            "chapter_max": 5,
            "lesson": 1,
            "limit": 3
        }
    )
    # May return 200 (if Qdrant available) or 503 (if unavailable)
    assert response.status_code in [200, 503]


def test_search_endpoint_invalid_filters(client):
    """Test search endpoint rejects invalid filter values."""
    # Invalid hardware_tier
    response = client.post(
        "/search",
        json={
            "query": "test query",
            "hardware_tier": 5  # Invalid (max is 4)
        }
    )
    assert response.status_code == 422
    
    # Invalid limit
    response = client.post(
        "/search",
        json={
            "query": "test query",
            "limit": 25  # Invalid (max is 20)
        }
    )
    assert response.status_code == 422

