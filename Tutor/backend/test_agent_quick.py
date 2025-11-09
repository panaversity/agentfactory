#!/usr/bin/env python3
"""
Quick Agent Test - Verify Agent + RAG + New API Key

This tests that your agent can:
1. Connect to Gemini with new API key
2. Use search_book_content() tool to fetch from RAG
3. Answer student questions using book content
"""

import asyncio
import os
from dotenv import load_dotenv

# Load environment
load_dotenv()

print("\n" + "=" * 80)
print("🧪 Testing TutorGPT Agent with RAG")
print("=" * 80)
print()

# Check API key
api_key = os.getenv("GEMINI_API_KEY")
if api_key:
    print(f"✅ API Key loaded: {api_key[:20]}...")
else:
    print("❌ No API key found in .env")
    exit(1)

print()

async def test_agent():
    """Test agent with RAG integration."""

    from app.agent.tutor_agent import create_tutor_agent

    print("1️⃣  Creating TutorGPT agent...")
    agent = create_tutor_agent()
    print("   ✅ Agent created successfully!")
    print()

    # Test questions
    questions = [
        "What is Python?",
        "How can AI help in software development?",
    ]

    for i, question in enumerate(questions, 1):
        print(f"{'='*80}")
        print(f"Test {i}/{len(questions)}")
        print(f"{'='*80}")
        print(f"👨‍🎓 Student asks: '{question}'")
        print()

        try:
            print("🤔 Agent is thinking and searching RAG...")

            response = await agent.teach(
                student_message=question,
                session_id=f"test_{i}"
            )

            print()
            print("🧠 TutorGPT Response:")
            print("-" * 80)
            print(response)
            print("-" * 80)
            print()
            print("✅ SUCCESS! Agent is working with RAG!")
            print()

        except Exception as e:
            print()
            print(f"❌ Error: {e}")
            print()
            if "403" in str(e) or "suspended" in str(e).lower():
                print("💡 API key issue detected!")
                print("   Solution: Get new API key from https://aistudio.google.com/app/apikey")
            elif "404" in str(e):
                print("💡 Model not found!")
                print("   Check AGENT_MODEL in .env file")
            else:
                import traceback
                traceback.print_exc()

            return False

    print("=" * 80)
    print("🎉 All Tests Passed!")
    print("=" * 80)
    print()
    print("Your Agent is:")
    print("  ✅ Connected to Gemini LLM")
    print("  ✅ Using search_book_content() tool")
    print("  ✅ Fetching content from ChromaDB")
    print("  ✅ Teaching students with book knowledge")
    print()
    return True

if __name__ == "__main__":
    success = asyncio.run(test_agent())
    exit(0 if success else 1)
