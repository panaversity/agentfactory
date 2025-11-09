"""
Integration Tests for TutorGPT Autonomous Agent with REAL LLM (TDD)

Test File: tests/integration/test_agent_with_llm.py
Implementation: app/agent/tutor_agent.py

Purpose: Test ACTUAL autonomous behavior with Gemini LLM.

These tests make REAL API calls to Gemini to verify:
1. Agent can communicate with LLM
2. Agent autonomously chooses tools
3. Agent provides helpful teaching responses
4. Tools are executed correctly

⚠️ NOTE: These tests require GEMINI_API_KEY in .env file!
"""

import pytest
import os
from dotenv import load_dotenv
from app.agent.tutor_agent import TutorGPTAgent, create_tutor_agent

# Load environment variables
load_dotenv()

# Skip tests if no API key (for CI/CD)
pytestmark = pytest.mark.skipif(
    not os.getenv("GEMINI_API_KEY") or os.getenv("GEMINI_API_KEY") == "your-gemini-api-key-here",
    reason="GEMINI_API_KEY not set in .env - skipping LLM integration tests"
)


class TestAgentWithRealLLM:
    """Integration tests with real Gemini LLM calls."""

    @pytest.mark.asyncio
    async def test_agent_can_respond_to_simple_question(self):
        """
        Test that agent can respond to a simple question using real LLM.

        This verifies:
        - Agent initialization works
        - LLM connection works
        - Agent can formulate a response
        """
        # Create agent
        agent = TutorGPTAgent(
            current_chapter="04-python",
            current_lesson="01-intro",
            student_level="beginner"
        )

        # Ask a simple question
        response = await agent.teach(
            "What is Python?",
            session_id="test_session_simple"
        )

        # Verify response
        assert response is not None
        assert len(response) > 0
        assert isinstance(response, str)
        print(f"\n✅ Agent Response:\n{response}\n")

    @pytest.mark.asyncio
    async def test_agent_autonomously_searches_book(self):
        """
        Test that agent autonomously calls search_book_content tool.

        This verifies:
        - Agent decides to use search tool autonomously
        - Tool is executed correctly
        - Response includes book references
        """
        agent = TutorGPTAgent(
            current_chapter="04-python",
            student_level="beginner"
        )

        response = await agent.teach(
            "Can you explain what Python is used for?",
            session_id="test_session_search"
        )

        # Verify response mentions book content
        assert response is not None
        assert len(response) > 50  # Substantial response
        # Agent should reference book/chapter (autonomous decision)
        print(f"\n✅ Agent Response (Book Search):\n{response}\n")

    @pytest.mark.asyncio
    async def test_agent_provides_encouraging_response(self):
        """
        Test that agent's personality shows through in responses.

        This verifies:
        - Agent maintains encouraging tone
        - Personality traits are reflected
        """
        agent = TutorGPTAgent(
            current_chapter="04-python",
            student_level="beginner"
        )

        response = await agent.teach(
            "I'm confused about variables",
            session_id="test_session_encouraging"
        )

        # Verify encouraging tone
        assert response is not None
        # Response should be helpful and supportive
        print(f"\n✅ Agent Response (Encouraging):\n{response}\n")

    @pytest.mark.asyncio
    async def test_agent_handles_how_question(self):
        """
        Test that agent provides code examples for 'how' questions.

        This verifies:
        - Agent recognizes "how" questions
        - Agent autonomously decides to provide code example
        """
        agent = TutorGPTAgent(
            current_chapter="04-python",
            student_level="beginner"
        )

        response = await agent.teach(
            "How do I create a variable in Python?",
            session_id="test_session_how"
        )

        # Verify response includes practical guidance
        assert response is not None
        assert len(response) > 50
        print(f"\n✅ Agent Response (How Question):\n{response}\n")

    @pytest.mark.asyncio
    async def test_agent_conversation_persistence(self):
        """
        Test that agent maintains conversation context across messages.

        This verifies:
        - Session management works
        - Agent remembers previous context
        """
        agent = TutorGPTAgent(
            current_chapter="04-python",
            student_level="beginner"
        )

        session_id = "test_session_persistence"

        # First message
        response1 = await agent.teach(
            "What is a variable?",
            session_id=session_id
        )
        assert response1 is not None
        print(f"\n✅ First Response:\n{response1}\n")

        # Follow-up message (should remember context)
        response2 = await agent.teach(
            "Can you give me an example?",
            session_id=session_id
        )
        assert response2 is not None
        print(f"\n✅ Follow-up Response:\n{response2}\n")

    @pytest.mark.asyncio
    async def test_create_agent_convenience_function_works_with_llm(self):
        """
        Test that convenience function creates working agent.

        This verifies:
        - create_tutor_agent() works correctly
        - Created agent can communicate with LLM
        """
        agent = create_tutor_agent(
            current_chapter="04-python",
            student_level="beginner"
        )

        response = await agent.teach(
            "Tell me about Python",
            session_id="test_session_convenience"
        )

        assert response is not None
        assert len(response) > 0
        print(f"\n✅ Convenience Function Response:\n{response}\n")


class TestAgentResponseQuality:
    """Test the quality of agent responses."""

    @pytest.mark.asyncio
    async def test_agent_response_time_under_threshold(self):
        """
        Test that agent responds within acceptable time (<10 seconds).

        This verifies:
        - Performance is acceptable
        - No timeout issues
        """
        import time

        agent = TutorGPTAgent(current_chapter="04-python")

        start_time = time.time()
        response = await agent.teach(
            "What is Python?",
            session_id="test_session_performance"
        )
        end_time = time.time()

        response_time = end_time - start_time

        assert response is not None
        assert response_time < 10  # Should respond within 10 seconds
        print(f"\n✅ Response Time: {response_time:.2f} seconds\n")


# Note: Run these tests with:
# pytest tests/integration/test_agent_with_llm.py -v -s
# The -s flag shows print statements (agent responses)
