"""
Unit tests for TutorGPT Autonomous Agent (TDD)

Test File: tests/unit/agent/test_tutor_agent.py
Implementation: app/agent/tutor_agent.py

Purpose: Verify that the agent is TRULY AUTONOMOUS using OpenAI Agents SDK.

KEY TESTING PRINCIPLE:
We test that the agent CAN use tools autonomously, NOT that it follows
hardcoded logic. The LLM decides which tools to use!
"""

import pytest
from app.agent.tutor_agent import TutorGPTAgent, create_tutor_agent


class TestTutorGPTAgent:
    """Test suite for TutorGPT autonomous agent."""

    def test_agent_exists(self):
        """Test that TutorGPTAgent can be instantiated."""
        agent = TutorGPTAgent(
            current_chapter="04-python",
            current_lesson="01-intro",
            student_level="beginner"
        )
        assert agent is not None

    def test_agent_has_personality(self):
        """Test that agent has personality traits."""
        agent = TutorGPTAgent()
        assert agent.personality is not None
        assert "patient" in agent.personality.traits

    def test_agent_has_tools(self):
        """Test that agent is equipped with teaching tools."""
        agent = TutorGPTAgent()
        # Agent should have tools available
        assert agent.agent.tools is not None
        assert len(agent.agent.tools) > 0

    def test_agent_has_instructions(self):
        """Test that agent has core teaching instructions."""
        agent = TutorGPTAgent(
            current_chapter="04-python",
            student_level="beginner"
        )
        # Agent should have instructions
        assert agent.agent.instructions is not None
        assert len(agent.agent.instructions) > 50

    def test_agent_context_update(self):
        """Test that agent context can be updated."""
        agent = TutorGPTAgent(current_chapter="04-python")

        # Update context
        agent.update_context(
            current_chapter="05-ai",
            current_lesson="02-ml-basics",
            student_level="intermediate"
        )

        assert agent.current_chapter == "05-ai"
        assert agent.current_lesson == "02-ml-basics"
        assert agent.student_level == "intermediate"

    def test_create_tutor_agent_convenience_function(self):
        """Test the convenience function creates agent correctly."""
        agent = create_tutor_agent(
            current_chapter="04-python",
            student_level="beginner"
        )

        assert isinstance(agent, TutorGPTAgent)
        assert agent.current_chapter == "04-python"
        assert agent.student_level == "beginner"


class TestAgentTools:
    """Test that agent has access to all required tools."""

    def test_agent_has_search_tool(self):
        """Test that agent has search_book_content tool."""
        agent = TutorGPTAgent()
        tool_names = [tool.name for tool in agent.agent.tools]
        assert "search_book_content" in tool_names

    def test_agent_has_explain_tool(self):
        """Test that agent has explain_concept tool."""
        agent = TutorGPTAgent()
        tool_names = [tool.name for tool in agent.agent.tools]
        assert "explain_concept" in tool_names

    def test_agent_has_code_example_tool(self):
        """Test that agent has provide_code_example tool."""
        agent = TutorGPTAgent()
        tool_names = [tool.name for tool in agent.agent.tools]
        assert "provide_code_example" in tool_names

    def test_agent_has_confusion_detection_tool(self):
        """Test that agent has detect_student_confusion tool."""
        agent = TutorGPTAgent()
        tool_names = [tool.name for tool in agent.agent.tools]
        assert "detect_student_confusion" in tool_names

    def test_agent_has_celebration_tool(self):
        """Test that agent has celebrate_milestone tool."""
        agent = TutorGPTAgent()
        tool_names = [tool.name for tool in agent.agent.tools]
        assert "celebrate_milestone" in tool_names


# NOTE: Integration tests for actual autonomous behavior
# will be in tests/integration/test_agent_autonomous_behavior.py
# Those tests will verify the agent autonomously chooses tools
# when given student questions (requires actual LLM calls)
