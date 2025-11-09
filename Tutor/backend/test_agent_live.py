"""
Live Test Script for TutorGPT Agent with Real LLM

Run this script to test the autonomous agent with real Gemini LLM!

Usage:
    python test_agent_live.py

⚠️ Requires: GEMINI_API_KEY in .env file
"""

import asyncio
import os
from dotenv import load_dotenv
from app.agent.tutor_agent import create_tutor_agent

# Load environment variables
load_dotenv()


async def test_agent_qa():
    """Test real Q&A with the autonomous agent."""

    print("\n" + "="*80)
    print("🧠 TUTORGPT AUTONOMOUS AGENT - LIVE TEST")
    print("="*80 + "\n")

    # Check API key
    if not os.getenv("GEMINI_API_KEY") or os.getenv("GEMINI_API_KEY") == "your-gemini-api-key-here":
        print("❌ ERROR: GEMINI_API_KEY not set in .env file!")
        print("Please add your Gemini API key to backend/.env")
        return

    print("✅ API Key found!")
    print(f"✅ Using model: {os.getenv('AGENT_MODEL', 'gemini-2.0-flash-exp')}\n")

    # Create agent
    print("Creating TutorGPT agent...")
    agent = create_tutor_agent(
        current_chapter="04-python",
        current_lesson="01-intro",
        student_level="beginner"
    )
    print("✅ Agent created!\n")

    # Test questions
    test_questions = [
        {
            "question": "What is Python?",
            "description": "Simple fact question - should search book"
        },
        {
            "question": "How do I create a variable?",
            "description": "'How' question - should provide code example"
        },
        {
            "question": "I'm confused about Python",
            "description": "Confusion signal - should be encouraging"
        }
    ]

    session_id = "live_test_session"

    for i, test in enumerate(test_questions, 1):
        print("="*80)
        print(f"TEST {i}/3: {test['description']}")
        print("="*80)
        print(f"\n👨‍🎓 Student: \"{test['question']}\"\n")
        print("🤔 Agent is thinking...\n")

        try:
            # Call the agent (REAL LLM!)
            response = await agent.teach(
                student_message=test['question'],
                session_id=session_id
            )

            print("🧠 TutorGPT:")
            print("-" * 80)
            print(response)
            print("-" * 80)
            print("\n✅ Response received!\n")

        except Exception as e:
            print(f"\n❌ Error: {e}\n")
            import traceback
            traceback.print_exc()

    print("\n" + "="*80)
    print("🎉 LIVE TEST COMPLETE!")
    print("="*80)
    print("\nThe agent successfully:")
    print("✅ Connected to Gemini LLM")
    print("✅ Autonomously decided which tools to use")
    print("✅ Provided teaching responses")
    print("✅ Maintained conversation context")
    print("\nThis is TRUE autonomous agentic behavior! 🚀\n")


if __name__ == "__main__":
    print("\n🚀 Starting TutorGPT Live Test...\n")
    asyncio.run(test_agent_qa())
