"""
TutorGPT Autonomous Agent - TRUE AGENTIC SYSTEM

This module creates the autonomous TutorGPT agent using OpenAI Agents SDK.

KEY PRINCIPLE: The LLM (agent) autonomously decides which tools to use based
on the student's question and the instructions. NO HARDCODED LOGIC!

The agent is the BRAIN - it thinks, decides, and acts autonomously.
"""

import os
from typing import Optional
from agents import Agent, Runner, SQLiteSession, AsyncOpenAI, OpenAIChatCompletionsModel, set_tracing_disabled
from dotenv import load_dotenv

from app.agent.personality import AgentPersonality
from app.agent.prompts.core_instructions import get_core_instructions
from app.tools.teaching_tools import TUTORGPT_TOOLS

# Load environment variables
load_dotenv()

# Disable tracing for faster responses (optional)
set_tracing_disabled(True)

# Set up Gemini API provider via OpenAI-compatible endpoint
Provider = AsyncOpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Set up the chat completion model with Gemini
model = OpenAIChatCompletionsModel(
    model=os.getenv("AGENT_MODEL", "gemini-2.0-flash-exp"),
    openai_client=Provider,
)


class TutorGPTAgent:
    """
    TutorGPT - Autonomous AI Teaching Agent

    This class wraps the OpenAI Agents SDK Agent to provide an autonomous
    teaching system that decides its own actions based on student needs.

    The agent:
    - Reads student questions
    - Decides which tools to use (autonomously!)
    - Searches the book when needed
    - Explains concepts adaptively
    - Celebrates milestones
    - Detects and responds to confusion

    All decisions are made by the LLM, NOT hardcoded logic!
    """

    def __init__(
        self,
        current_chapter: Optional[str] = None,
        current_lesson: Optional[str] = None,
        student_level: str = "beginner"
    ):
        """
        Initialize TutorGPT autonomous agent.

        Args:
            current_chapter: Current chapter student is reading (e.g., "04-python")
            current_lesson: Current lesson student is on (e.g., "01-intro")
            student_level: Student's proficiency level (beginner/intermediate/advanced)
        """
        self.personality = AgentPersonality()
        self.current_chapter = current_chapter
        self.current_lesson = current_lesson
        self.student_level = student_level

        # Get core instructions (the agent's "teaching philosophy")
        instructions = get_core_instructions(
            current_chapter=current_chapter,
            current_lesson=current_lesson,
            student_level=student_level
        )

        # Create the AUTONOMOUS AGENT using OpenAI Agents SDK with Gemini LLM
        # The agent gets instructions and tools, then DECIDES autonomously!
        self.agent = Agent(
            name="TutorGPT",
            instructions=instructions,
            tools=TUTORGPT_TOOLS,  # Agent can use these tools - IT DECIDES WHEN!
            model=model,  # Gemini 2.0 Flash via OpenAI-compatible API
        )

    async def teach(
        self,
        student_message: str,
        session_id: str = "default"
    ) -> str:
        """
        Teach a student by responding to their question autonomously.

        The agent (LLM) will:
        1. Read the student's question
        2. Decide which tools to use (search book? explain? provide example?)
        3. Execute the tools autonomously
        4. Formulate an encouraging, helpful response

        NO HARDCODED LOGIC! The LLM decides everything based on instructions.

        Args:
            student_message: The student's question or message
            session_id: Session ID for conversation persistence

        Returns:
            Agent's teaching response

        Examples:
            >>> agent = TutorGPTAgent(current_chapter="04-python")
            >>> response = await agent.teach("What is Python?")
            >>> print(response)
            # Agent autonomously:
            # 1. Searches the book for "Python"
            # 2. Explains the concept
            # 3. Returns encouraging response
        """
        # Create session for conversation memory
        session = SQLiteSession(session_id)

        # RUN THE AUTONOMOUS AGENT!
        # The LLM will decide which tools to use based on the student's question
        result = await Runner.run(
            self.agent,
            input=student_message,
            session=session
        )

        return result.final_output

    def teach_sync(
        self,
        student_message: str,
        session_id: str = "default"
    ) -> str:
        """
        Synchronous version of teach() for non-async contexts.

        Args:
            student_message: The student's question
            session_id: Session ID for conversation persistence

        Returns:
            Agent's teaching response
        """
        session = SQLiteSession(session_id)

        # Synchronous execution
        result = Runner.run_sync(
            self.agent,
            input=student_message,
            session=session
        )

        return result.final_output

    def update_context(
        self,
        current_chapter: Optional[str] = None,
        current_lesson: Optional[str] = None,
        student_level: Optional[str] = None
    ):
        """
        Update the agent's context (chapter, lesson, student level).

        This recreates the agent with updated instructions.

        Args:
            current_chapter: New current chapter
            current_lesson: New current lesson
            student_level: New student level
        """
        if current_chapter:
            self.current_chapter = current_chapter
        if current_lesson:
            self.current_lesson = current_lesson
        if student_level:
            self.student_level = student_level

        # Recreate agent with updated context
        instructions = get_core_instructions(
            current_chapter=self.current_chapter,
            current_lesson=self.current_lesson,
            student_level=self.student_level
        )

        self.agent = Agent(
            name="TutorGPT",
            instructions=instructions,
            tools=TUTORGPT_TOOLS,
            model=model,  # Gemini 2.0 Flash
        )


# Convenience function to create agent quickly
def create_tutor_agent(
    current_chapter: Optional[str] = None,
    current_lesson: Optional[str] = None,
    student_level: str = "beginner"
) -> TutorGPTAgent:
    """
    Create a TutorGPT autonomous agent.

    Args:
        current_chapter: Current chapter student is reading
        current_lesson: Current lesson student is on
        student_level: Student's proficiency level

    Returns:
        TutorGPTAgent instance ready to teach

    Example:
        >>> agent = create_tutor_agent(current_chapter="04-python", student_level="beginner")
        >>> response = await agent.teach("What is a variable?")
    """
    return TutorGPTAgent(
        current_chapter=current_chapter,
        current_lesson=current_lesson,
        student_level=student_level
    )
