"""
Unit tests for TutorGPT Agent Personality (TDD)

Test File: tests/unit/agent/test_personality.py
Implementation: app/agent/personality.py

Purpose: Verify TutorGPT has the correct teaching personality and traits.
"""

import pytest
from app.agent.personality import AgentPersonality, TeachingStyle


class TestAgentPersonality:
    """Test suite for AgentPersonality class."""

    def test_personality_exists(self):
        """Test that AgentPersonality class can be instantiated."""
        personality = AgentPersonality()
        assert personality is not None

    def test_personality_has_encouraging_coach_style(self):
        """Test that agent has 'Encouraging Coach' teaching style."""
        personality = AgentPersonality()
        assert personality.style == TeachingStyle.ENCOURAGING_COACH

    def test_personality_has_core_traits(self):
        """Test that agent has essential teaching traits."""
        personality = AgentPersonality()

        # Must have these traits
        required_traits = {"patient", "encouraging", "adaptive", "supportive"}
        assert required_traits.issubset(set(personality.traits))

    def test_personality_is_book_focused(self):
        """Test that agent prioritizes book content over general knowledge."""
        personality = AgentPersonality()
        assert personality.book_focused is True

    def test_personality_uses_socratic_method(self):
        """Test that agent can use Socratic questioning."""
        personality = AgentPersonality()
        assert "socratic" in personality.teaching_methods

    def test_personality_celebrates_progress(self):
        """Test that agent celebrates student milestones."""
        personality = AgentPersonality()
        assert personality.celebrates_progress is True

    def test_personality_detects_confusion(self):
        """Test that agent monitors student confusion."""
        personality = AgentPersonality()
        assert personality.monitors_confusion is True

    def test_personality_description_is_meaningful(self):
        """Test that personality has a clear description."""
        personality = AgentPersonality()
        description = personality.get_description()

        assert len(description) > 50  # Substantial description
        assert "teach" in description.lower()
        assert "student" in description.lower()

    def test_personality_tone_guidelines(self):
        """Test that agent has tone guidelines for responses."""
        personality = AgentPersonality()
        tone = personality.tone_guidelines

        assert "friendly" in tone.lower() or "encouraging" in tone.lower()
        assert "professional" in tone.lower() or "respectful" in tone.lower()


class TestTeachingStyle:
    """Test suite for TeachingStyle enum."""

    def test_encouraging_coach_exists(self):
        """Test that ENCOURAGING_COACH style is defined."""
        assert TeachingStyle.ENCOURAGING_COACH is not None

    def test_teaching_style_has_value(self):
        """Test that teaching style has meaningful string value."""
        style = TeachingStyle.ENCOURAGING_COACH
        assert isinstance(style.value, str)
        assert len(style.value) > 0
