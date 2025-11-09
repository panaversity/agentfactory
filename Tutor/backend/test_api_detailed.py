#!/usr/bin/env python3
"""
Detailed API Test - Check what's wrong
"""

import httpx
import os
from dotenv import load_dotenv

load_dotenv()

api_key = os.getenv("GEMINI_API_KEY")

print("\n" + "=" * 80)
print("🔬 Detailed API Key Test")
print("=" * 80)
print()
print(f"API Key: {api_key[:20]}...")
print()

# Test 1: List available models
print("Test 1: Can we list available models?")
print("-" * 80)

url = f"https://generativelanguage.googleapis.com/v1beta/models?key={api_key}"

try:
    response = httpx.get(url, timeout=10.0)

    if response.status_code == 200:
        print("✅ SUCCESS! API key is valid and working!")
        print()
        data = response.json()
        if 'models' in data:
            print(f"Found {len(data['models'])} available models:")
            for model in data['models'][:5]:
                print(f"  - {model.get('name', 'Unknown')}")
        print()
        print("✅ Your API key works! The issue is with the specific endpoint.")
        print("💡 Solution: We need to switch agent to use standard Gemini API")
    else:
        print(f"❌ FAILED! Status: {response.status_code}")
        print(f"Response: {response.text[:500]}")
        print()

        if response.status_code == 403:
            print("💡 403 Forbidden means:")
            print("   1. API is not enabled in Google Cloud Console")
            print("   2. Billing is not set up (Gemini requires billing)")
            print("   3. API key is restricted")
            print()
            print("🔧 Solutions:")
            print("   1. Go to https://console.cloud.google.com/")
            print("   2. Enable 'Generative Language API'")
            print("   3. Set up billing (has free tier)")
            print("   4. Or try AI Studio API key: https://aistudio.google.com/app/apikey")
        elif response.status_code == 429:
            print("💡 429 Rate Limit means:")
            print("   - Too many requests")
            print("   - Wait 1 hour and try again")

except Exception as e:
    print(f"❌ Error: {e}")

print()
print("=" * 80)
print("📋 Summary")
print("=" * 80)
print()
print("Your API key test result: See above")
print()
print("If you got 403 Forbidden:")
print("  → The API key exists but API is not enabled")
print("  → Need to enable Generative Language API in Google Cloud")
print("  → Or create key from AI Studio instead of Cloud Console")
print()
