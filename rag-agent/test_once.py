#!/usr/bin/env python3
"""
One-time test script to verify the backend is working.

Run this before deployment to ensure everything is set up correctly.
"""

import sys
from pathlib import Path

# Add current directory to path
sys.path.insert(0, str(Path(__file__).parent))

def test_imports():
    """Test all critical imports."""
    print("🔍 Testing imports...")
    try:
        from app import app
        from rag.tools import search_tool, SEARCH_TOOL_SCHEMA, get_search_client
        from core.config import get_settings
        from api import search, health
        print("✅ All imports successful")
        return True
    except Exception as e:
        print(f"❌ Import failed: {e}")
        return False


def test_config():
    """Test configuration loading."""
    print("\n🔍 Testing configuration...")
    try:
        from core.config import get_settings
        settings = get_settings()
        print(f"✅ Config loaded: book_id={settings.book_id}")
        print(f"   Collection: {settings.collection_name}")
        print(f"   Frontend URL: {settings.frontend_base_url}")
        return True
    except Exception as e:
        print(f"❌ Config failed: {e}")
        return False


def test_search_tool():
    """Test search tool function."""
    print("\n🔍 Testing search tool...")
    try:
        from rag.tools import search_tool, SEARCH_TOOL_SCHEMA
        
        # Test schema
        assert SEARCH_TOOL_SCHEMA["type"] == "function"
        assert SEARCH_TOOL_SCHEMA["function"]["name"] == "search_robolearn_content"
        print("✅ Tool schema valid")
        
        # Test tool call (may fail if Qdrant unavailable, that's OK)
        result = search_tool("test query", limit=1)
        assert isinstance(result, dict)
        assert "query" in result
        assert "results" in result
        
        if "error" in result:
            print(f"⚠️  Search tool returned error (Qdrant may be unavailable): {result['error']}")
        else:
            print(f"✅ Search tool working: found {result['total_found']} results")
        
        return True
    except Exception as e:
        print(f"❌ Search tool failed: {e}")
        return False


def test_app_routes():
    """Test FastAPI app routes."""
    print("\n🔍 Testing FastAPI app...")
    try:
        from app import app
        from fastapi.testclient import TestClient
        
        client = TestClient(app)
        
        # Test root
        response = client.get("/")
        assert response.status_code == 200
        data = response.json()
        assert data["service"] == "RoboLearn Backend"
        print("✅ Root endpoint working")
        
        # Test health
        response = client.get("/health")
        assert response.status_code == 200
        health_data = response.json()
        print(f"✅ Health endpoint: status={health_data['status']}, qdrant={health_data['qdrant']}")
        
        # Test search validation
        response = client.post("/search", json={"query": "ab"})  # Too short
        assert response.status_code == 422
        print("✅ Search validation working")
        
        return True
    except Exception as e:
        print(f"❌ App routes failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("RoboLearn Backend - One-Time Test")
    print("=" * 60)
    
    results = []
    results.append(("Imports", test_imports()))
    results.append(("Configuration", test_config()))
    results.append(("Search Tool", test_search_tool()))
    results.append(("App Routes", test_app_routes()))
    
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    all_passed = True
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {name}")
        if not passed:
            all_passed = False
    
    print("=" * 60)
    if all_passed:
        print("✅ All tests passed! Ready for deployment.")
        return 0
    else:
        print("❌ Some tests failed. Please fix issues before deployment.")
        return 1


if __name__ == "__main__":
    sys.exit(main())

