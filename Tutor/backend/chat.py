#!/usr/bin/env python3
"""
Interactive TutorGPT Chat - Real-time Agent Interaction

Chat with your TutorGPT agent in real-time!
Type your questions and get instant responses.
"""

import asyncio
import os
from dotenv import load_dotenv

# Load environment
load_dotenv()

print("\n" + "=" * 80)
print("🎓 TutorGPT - Interactive Chat")
print("=" * 80)
print()

# Check API key
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    print("❌ No GEMINI_API_KEY found in .env")
    print("Please add your API key to .env file")
    exit(1)

print("✅ API Key loaded")
print()

# Import agent
from app.agent.tutor_agent import create_tutor_agent

print("🤖 Initializing TutorGPT agent...")
agent = create_tutor_agent()
print("✅ Agent ready!")
print()

print("=" * 80)
print("💬 Chat Interface")
print("=" * 80)
print()
print("Commands:")
print("  - Type your question and press Enter")
print("  - Type 'quit' or 'exit' to end chat")
print("  - Type 'clear' to clear screen")
print("  - Type 'help' for tips")
print()
print("=" * 80)
print()

# Session ID for conversation memory
session_id = "interactive_session"
message_count = 0


async def chat_loop():
    """Main chat loop."""
    global message_count

    while True:
        # Get user input
        try:
            user_input = input("You: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n\n👋 Goodbye!")
            break

        # Check for commands
        if not user_input:
            continue

        if user_input.lower() in ['quit', 'exit', 'bye']:
            print("\n👋 Goodbye! Keep learning!")
            break

        if user_input.lower() == 'clear':
            os.system('cls' if os.name == 'nt' else 'clear')
            print("🎓 TutorGPT - Interactive Chat")
            print("=" * 80)
            continue

        if user_input.lower() == 'help':
            print("\n💡 Tips:")
            print("  - Ask about Python concepts: 'What is a variable?'")
            print("  - Ask for code examples: 'Show me how to use async/await'")
            print("  - Ask about the book: 'What are the nine pillars of AIDD?'")
            print("  - Ask for clarification: 'I'm confused about...'")
            print()
            continue

        # Send to agent
        message_count += 1
        print()
        print("🤔 TutorGPT is thinking...")
        print()

        try:
            response = await agent.teach(
                student_message=user_input,
                session_id=session_id
            )

            print("🧠 TutorGPT:")
            print("-" * 80)
            print(response)
            print("-" * 80)
            print()

        except Exception as e:
            print(f"❌ Error: {e}")
            print()
            if "403" in str(e):
                print("💡 API key issue. Check your .env file.")
                break
            elif "429" in str(e):
                print("💡 Rate limit hit. Wait a moment and try again.")
            else:
                import traceback
                traceback.print_exc()


def main():
    """Run the interactive chat."""
    try:
        asyncio.run(chat_loop())
    except KeyboardInterrupt:
        print("\n\n👋 Chat ended. Goodbye!")


if __name__ == "__main__":
    main()
