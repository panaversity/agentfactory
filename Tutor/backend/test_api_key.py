#!/usr/bin/env python3
"""
API Key Diagnostic Tool

Tests which Gemini API endpoints your key has access to.
"""

import os
import httpx
from dotenv import load_dotenv

load_dotenv()

print("\n" + "=" * 80)
print("🔑 Gemini API Key Diagnostic")
print("=" * 80)
print()

api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    print("❌ No GEMINI_API_KEY found in .env")
    exit(1)

print(f"API Key: {api_key[:20]}...")
print()

# Test different endpoints
tests = [
    {
        "name": "Standard Gemini API (generateContent)",
        "url": f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent?key={api_key}",
        "method": "POST",
        "json": {
            "contents": [{
                "parts": [{"text": "Say 'Hello'"}]
            }]
        }
    },
    {
        "name": "OpenAI-Compatible API (chat/completions)",
        "url": "https://generativelanguage.googleapis.com/v1beta/openai/chat/completions",
        "method": "POST",
        "headers": {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        },
        "json": {
            "model": "gemini-2.0-flash-exp",
            "messages": [{"role": "user", "content": "Say 'Hello'"}]
        }
    },
]

print("Testing API endpoints...")
print()

for i, test in enumerate(tests, 1):
    print(f"{i}. {test['name']}")
    print(f"   URL: {test['url'][:80]}...")

    try:
        if test['method'] == 'POST':
            headers = test.get('headers', {})
            response = httpx.post(
                test['url'],
                json=test['json'],
                headers=headers,
                timeout=10.0
            )

            if response.status_code == 200:
                print(f"   ✅ SUCCESS! Status: {response.status_code}")
                # Try to parse response
                try:
                    data = response.json()
                    if 'candidates' in data:
                        print(f"   📝 Response type: Standard Gemini API")
                    elif 'choices' in data:
                        print(f"   📝 Response type: OpenAI-compatible API")
                except:
                    pass
            else:
                print(f"   ❌ FAILED! Status: {response.status_code}")
                print(f"   Error: {response.text[:200]}")

    except Exception as e:
        print(f"   ❌ ERROR: {str(e)[:100]}")

    print()

print("=" * 80)
print("💡 Recommendation")
print("=" * 80)
print()
print("If Standard Gemini API ✅ but OpenAI-Compatible API ❌:")
print("  → Your key works, but not with OpenAI-compatible endpoint")
print("  → Solution: We need to switch agent to use standard Gemini API")
print()
print("If both ❌:")
print("  → API key might be restricted or suspended")
print("  → Solution: Create new key at https://aistudio.google.com/app/apikey")
print()
