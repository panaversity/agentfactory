"""
Co-Learning Agent - Autonomous Step-by-Step Teaching System

This module implements an advanced agentic teaching system that autonomously
guides students through the entire book using the 17-step teaching flow.

Based on:
- OpenAI Agents SDK patterns
- Anthropic's Effective Context Engineering
- Advanced prompt engineering techniques
- Autonomous decision-making loops
"""

import os
import json
from typing import Optional, Dict, List, Any
from enum import Enum
from agents import Agent, Runner, SQLiteSession, AsyncOpenAI, OpenAIChatCompletionsModel
from dotenv import load_dotenv

from app.agent.personality import AgentPersonality
from app.tools.teaching_tools import TUTORGPT_TOOLS

load_dotenv()

# Gemini API configuration
Provider = AsyncOpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

model = OpenAIChatCompletionsModel(
    model=os.getenv("AGENT_MODEL", "gemini-2.0-flash"),
    openai_client=Provider,
)


class TeachingPhase(Enum):
    """Current phase in the teaching flow"""
    GREETING = "greeting"
    LANGUAGE_SELECTION = "language_selection"
    CHAPTER_SELECTION = "chapter_selection"
    LESSON_PLAN = "lesson_plan"
    SECTION_TEACHING = "section_teaching"
    REFLECTION_QUESTION = "reflection_question"
    STUDENT_ANSWER_EVAL = "student_answer_evaluation"
    PRACTICE_TASK = "practice_task"
    KEY_POINT_REPEAT = "key_point_repeat"
    CHAPTER_SUMMARY = "chapter_summary"
    QUIZ_PREPARATION = "quiz_preparation"
    QUIZ_TAKING = "quiz_taking"
    QUIZ_GRADING = "quiz_grading"
    REMEDIAL_PATH = "remedial_path"
    PROGRESS_UPDATE = "progress_update"
    NEXT_STEP_CHOICE = "next_step_choice"
    BOOK_COMPLETION = "book_completion"


class CoLearningAgent:
    """
    Advanced Co-Learning Agent with autonomous decision-making.

    This agent implements the 17-step teaching flow with full autonomy:
    - Dynamically manages teaching context
    - Makes pedagogical decisions autonomously
    - Adapts to student responses in real-time
    - Maintains conversation state and learning progress
    """

    def __init__(
        self,
        session_id: str,
        student_profile: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize Co-Learning Agent.

        Args:
            session_id: Unique session identifier for conversation persistence
            student_profile: Student's learning profile (name, level, language, etc.)
        """
        self.session_id = session_id
        self.student_profile = student_profile or {}
        self.personality = AgentPersonality()

        # Teaching state
        self.current_phase = TeachingPhase.GREETING
        self.current_chapter = self.student_profile.get('current_chapter', 1)
        self.current_section = self.student_profile.get('current_section', 0)
        self.language = self.student_profile.get('language', 'en')
        self.student_level = self.student_profile.get('level', 'beginner')
        self.completed_chapters = self.student_profile.get('completed_chapters', [])
        self.wrong_answer_streak = 0
        self.confusion_indicators = []

        # Create the autonomous agent with enhanced instructions
        self.agent = self._create_agent()

    def _create_agent(self) -> Agent:
        """
        Create the autonomous teaching agent with context-aware instructions.

        Returns:
            Configured Agent instance
        """
        instructions = self._generate_dynamic_instructions()

        return Agent(
            name="CoLearningTutor",
            instructions=instructions,
            tools=TUTORGPT_TOOLS,
            model=model,
        )

    def _generate_dynamic_instructions(self) -> str:
        """
        Generate dynamic, context-aware system instructions.

        Based on STUDY_MODE_V2 principles adapted for:
        - Book-based learning (not course-based)
        - Real teacher persona (active teaching, not co-teaching)
        - Performance-based adaptive greetings
        - Text-based conversational flow
        - Friendly/casual tone with minimal emojis

        Returns:
            Complete system instructions with current context
        """
        # Get student info
        student_name = self.student_profile.get('name', 'friend')
        completed_count = len(self.completed_chapters)
        total_chapters = 46
        performance_level = self._get_performance_level()

        instructions = f"""You are an AI teacher for "AI-Native Software Development" - a real teacher who TEACHES actively, not just assists.

<context>
Student: {student_name}
Progress: {completed_count}/{total_chapters} chapters completed
Performance: {performance_level}
Current Chapter: {self.current_chapter}
Language: English (keep it casual and friendly)
</context>

<CRITICAL_CONVERSATION_MEMORY>
YOU HAVE FULL ACCESS TO CONVERSATION HISTORY - All previous messages in this session are visible to you!

MANDATORY RULES:
1. **ALWAYS read the full conversation history** before responding
2. **Continue conversations naturally** from where they left off
3. **Remember what the student said** in previous messages
4. **NEVER repeat the same response** twice in a row
5. **If student says "yes", "ya", "okay"** - refer to what YOU just asked them and continue that topic

Example Correct Behavior:
- You: "Which chapter - Chapter 1 or jump ahead?"
- Student: "chapter 1"
- You: "Great! Chapter 1 is about... [start teaching]"  ← CORRECT

- You: "...what exactly makes software 'AI-native'? Any thoughts?"
- Student: "ya"
- You: "[Continue explaining AI-native, don't repeat question]"  ← CORRECT

Example WRONG Behavior (DO NOT DO THIS):
- You: "Which chapter?"
- Student: "chapter 1"
- You: "Which chapter?" ← WRONG! You already asked this!

- You: "What makes software AI-native?"
- Student: "ya"
- You: "Hey! Ready to dive in?" ← WRONG! Don't restart, continue conversation!

If there are NO previous messages (truly first interaction):
- Give one brief greeting
- Ask which chapter
- Keep it 2-3 sentences

If there ARE previous messages (conversation in progress):
- Build on what was already discussed
- Respond to their actual message
- Reference earlier context
- NEVER restart with a greeting
</CRITICAL_CONVERSATION_MEMORY>

<hello_trigger>
ONLY use this if conversation history is EMPTY (first ever message).

For this trigger, respond with exactly one action: a warm, performance-aware greeting that sets the tone.

Greeting Guidelines:
- Keep it friendly and casual (like talking to a friend, not a formal student)
- Adapt to their performance:
  * New students: "Hey! Ready to dive into AI-native development?"
  * Progressing well: "Welcome back! You're doing great - let's keep the momentum going"
  * Struggling: "Hey there! Don't worry, we'll work through this together"
  * High achievers: "Nice! You're crushing it. Ready for the next challenge?"
- After greeting, ask which chapter they want to learn (in text, not buttons)
- Keep it short - 2-3 sentences max
- Use emojis sparingly (one max, if at all)

Example good greeting:
"Hey! Ready to learn some AI-native development? Which chapter are you interested in - want to start from Chapter 1 or jump to a specific topic?"

Do NOT:
- Give long explanations in greeting
- List all chapters
- Share progress details yet
- Start teaching immediately
</hello_trigger>

<teaching_flow>
After greeting, follow this natural conversation flow:

1. Student tells you which chapter → Confirm and start teaching
2. Teach in small chunks → One concept at a time
3. Ask questions to check understanding → Make them think
4. Give practice tasks → Hands-on learning
5. Move to next section when ready → Keep momentum

IMPORTANT:
- Advance one step at a time (no rushing through multiple topics)
- If student asks about a different chapter, switch to it immediately
- Always plan the next step but let the conversation flow naturally
- End each response with a clear prompt for what comes next
</teaching_flow>

<teaching_identity>
You are a TEACHER who TEACHES, not an assistant who helps with homework.

Think like a real classroom teacher:
- You LEAD the lesson (don't wait for them to ask everything)
- You EXPLAIN concepts (don't just answer questions)
- You ASK questions to make them think (Socratic method)
- You GIVE examples and analogies (make it click)
- You PRACTICE with them (learn by doing)
- You CHECK understanding (can they explain it back?)

Active teaching example:
"Let's talk about agents. An agent is basically a program that can make decisions and take actions on its own. Think of it like a smart assistant that doesn't just follow orders - it figures out what to do.

Here's a simple way to see it: if I tell you 'book me a flight', you'd need to check dates, compare prices, pick the best option, and complete the booking. That's what an agent does - takes a goal and figures out the steps.

Quick check: can you think of another example where an agent would be useful?"

NOT like this:
"Agents are autonomous systems that can reason and act. Let me know if you have questions!"
</teaching_identity>

<study_mode_rules>
You are a teacher who guides learning through collaboration, NOT someone who does homework for students.

Core Teaching Rules:

1. Get to know the student
   - If you don't know their goals or experience level, ask once (keep it lightweight)
   - Default to explaining things that a beginner programmer would understand

2. Build on existing knowledge
   - Connect new ideas to what they already know
   - "Remember when we talked about X? This is similar..."

3. Guide, don't give answers
   - Use questions, hints, and small steps
   - Let them discover the answer themselves
   - NEVER solve their homework problems directly
   - If they ask a coding problem, guide them step by step (one question at a time)

4. Check and reinforce
   - After tough parts, confirm they can restate the idea
   - "Can you explain that back to me in your own words?"
   - Use analogies and examples to make ideas stick

5. Vary the rhythm
   - Mix explanations, questions, and activities
   - Sometimes ask them to teach you
   - Keep it conversational, not lecture-style

Above all: DO NOT DO THE STUDENT'S WORK FOR THEM.

### Things You Can Do:

**Teach new concepts:**
- Explain at their level
- Ask guiding questions
- Use analogies and examples
- Review with practice questions

Example:
"Let me break down how async works. Imagine you're at a restaurant ordering food. You don't stand at the counter waiting for your burger to cook - you sit down, and they bring it when it's ready. That's async - your code keeps running while waiting for tasks.

Try this: can you think of another real-life example of something asynchronous?"

**Help with problems (NOT homework):**
- Start from what they know
- Fill in gaps with questions
- Give them a chance to respond
- Never ask more than one question at a time

**Practice together:**
- Ask them to summarize
- Have them explain concepts back
- Quick practice rounds
- Role-play scenarios

**Quizzes & prep:**
- One question at a time
- Let them try twice before revealing answers
- Review mistakes in depth

### Tone & Approach:

- Warm, patient, plain-spoken
- Don't use too many exclamation marks or emojis (use them sparingly)
- Keep moving - always know the next step
- Be brief - aim for 3-5 sentences, not essays
- Good back-and-forth conversation

Good example:
"Agents are programs that can make decisions on their own. Think of it like a smart assistant - instead of just following orders, it figures out what steps to take.

If I say 'book me a flight', a regular program would need exact instructions. An agent would check dates, compare prices, and complete the booking by itself.

Can you think of a task where an agent would be useful?"

Bad example:
"Not quite! Let me clarify... Great question! Let me break this down in simpler terms... Does this make sense? Feel free to ask if you need more clarification!"
(Too formulaic, too many phrases, no substance)

</study_mode_rules>

<tool_usage>
Use available tools smartly:
- `search_book_content` - Use this for finding information in the book
- `explain_concept` - Use when explaining complex topics
- `provide_code_example` - Use when showing code
- Plan tool calls, reflect on results, integrate seamlessly
- Don't expose tool internals to the student
</tool_usage>

<persistence>
Continue until the lesson goal is met or student signals to stop.
Never terminate early due to uncertainty - research via tools or deduce reasonably.
</persistence>

IMPORTANT:
DO NOT GIVE ANSWERS OR DO HOMEWORK FOR THE USER.
If they ask a math or logic problem, or upload an image of one, DO NOT SOLVE IT in your first response.
Instead: talk through the problem step by step, one question at a time, giving space to answer.
"""

        return instructions.strip()

    def _get_tone_guidance(self) -> str:
        """Get tone guidance based on student level"""
        tone_map = {
            'beginner': 'Warm, encouraging, simple language. Lots of analogies.',
            'intermediate': 'Friendly but technical. Balance explanation and challenge.',
            'advanced': 'Professional, deep technical detail. Socratic questions.'
        }
        return tone_map.get(self.student_level, tone_map['beginner'])

    def _get_performance_level(self) -> str:
        """
        Determine student's performance level for adaptive greeting.

        Returns:
            Performance descriptor: 'new', 'progressing', 'struggling', or 'excelling'
        """
        completed_count = len(self.completed_chapters)
        total_chapters = 46

        # New student
        if completed_count == 0:
            return 'new'

        # Check if struggling (high wrong answer streak)
        if self.wrong_answer_streak >= 3:
            return 'struggling'

        # Calculate progress percentage
        progress_pct = (completed_count / total_chapters) * 100

        # Excelling (>70% completion OR rapid progress)
        if progress_pct > 70 or (completed_count > 5 and self.wrong_answer_streak == 0):
            return 'excelling'

        # Progressing normally
        return 'progressing'

    async def teach(self, student_message: str) -> Dict[str, Any]:
        """
        Main teaching method - autonomous response to student.

        The agent autonomously:
        1. Analyzes student message and context
        2. Determines current teaching phase
        3. Selects appropriate tools to use
        4. Generates pedagogically sound response
        5. Updates teaching state

        Args:
            student_message: Student's message or question

        Returns:
            Dict with response, phase, metadata
        """
        # Ensure data directory exists for SQLite session
        import os
        os.makedirs("data", exist_ok=True)

        # Create session for conversation memory
        session = SQLiteSession(self.session_id)

        # Run the autonomous agent
        result = await Runner.run(
            self.agent,
            input=student_message,
            session=session
        )

        # Update teaching state based on response
        await self._update_teaching_state(student_message, result.final_output)

        return {
            'response': result.final_output,
            'phase': self.current_phase.value,
            'chapter': self.current_chapter,
            'section': self.current_section,
            'metadata': {
                'language': self.language,
                'level': self.student_level,
                'wrong_streak': self.wrong_answer_streak,
                'completed_chapters': self.completed_chapters
            }
        }

    async def _update_teaching_state(self, student_input: str, agent_response: str):
        """
        Update internal teaching state based on conversation.

        This implements state transitions in the 17-step flow.
        """
        # Detect phase transitions autonomously
        lower_input = student_input.lower()
        lower_response = agent_response.lower()

        # Language selection detection
        if any(lang in lower_input for lang in ['english', 'urdu', 'spanish', 'roman']):
            if 'english' in lower_input:
                self.language = 'en'
            elif 'urdu' in lower_input or 'roman' in lower_input:
                self.language = 'roman_ur'
            elif 'spanish' in lower_input:
                self.language = 'es'
            self.current_phase = TeachingPhase.CHAPTER_SELECTION

        # Chapter selection detection
        if 'chapter' in lower_input and any(str(i) in lower_input for i in range(1, 11)):
            for i in range(1, 11):
                if str(i) in lower_input:
                    self.current_chapter = i
                    self.current_phase = TeachingPhase.LESSON_PLAN
                    break

        # Quiz transition detection
        if 'quiz' in lower_response or 'test your understanding' in lower_response:
            self.current_phase = TeachingPhase.QUIZ_PREPARATION

        # Confusion detection (wrong answer streak)
        if 'not quite' in lower_response or 'let me explain differently' in lower_response:
            self.wrong_answer_streak += 1
        else:
            self.wrong_answer_streak = 0

        # Chapter completion detection
        if 'completed chapter' in lower_response or 'chapter done' in lower_response:
            if self.current_chapter not in self.completed_chapters:
                self.completed_chapters.append(self.current_chapter)
            self.current_phase = TeachingPhase.PROGRESS_UPDATE

    def update_profile(self, updates: Dict[str, Any]):
        """
        Update student profile and regenerate agent with new context.

        Args:
            updates: Dictionary of profile updates
        """
        self.student_profile.update(updates)

        # Update internal state
        if 'current_chapter' in updates:
            self.current_chapter = updates['current_chapter']
        if 'language' in updates:
            self.language = updates['language']
        if 'level' in updates:
            self.student_level = updates['level']
        if 'completed_chapters' in updates:
            self.completed_chapters = updates['completed_chapters']

        # Recreate agent with updated context
        self.agent = self._create_agent()

    async def get_lesson_plan(self, chapter: int) -> str:
        """
        Generate lesson plan for a chapter (Step 3 of teaching flow).

        Args:
            chapter: Chapter number

        Returns:
            Lesson plan summary
        """
        self.current_chapter = chapter
        self.current_phase = TeachingPhase.LESSON_PLAN

        # Use agent to generate personalized lesson plan
        prompt = f"Generate a brief lesson plan for Chapter {chapter}. What will we cover?"
        result = await self.teach(prompt)
        return result['response']

    async def generate_quiz(self, chapter: int, num_questions: int = 10) -> List[Dict]:
        """
        Generate quiz questions for a chapter (Step 11-12).

        Args:
            chapter: Chapter number
            num_questions: Number of questions

        Returns:
            List of quiz questions
        """
        self.current_phase = TeachingPhase.QUIZ_PREPARATION

        # Use agent's quiz generation tool
        prompt = f"Generate a {num_questions}-question quiz for Chapter {chapter}. Mix multiple choice, true/false, and short answer."

        session = SQLiteSession(self.session_id)
        result = await Runner.run(self.agent, input=prompt, session=session)

        # Parse quiz from response (agent should format it)
        return self._parse_quiz_questions(result.final_output)

    def _parse_quiz_questions(self, quiz_text: str) -> List[Dict]:
        """Parse quiz questions from agent's formatted response"""
        # This would parse the structured quiz output from the agent
        # For now, return a placeholder structure
        return []


def create_colearning_agent(
    session_id: str,
    student_profile: Optional[Dict[str, Any]] = None
) -> CoLearningAgent:
    """
    Factory function to create a Co-Learning Agent.

    Args:
        session_id: Unique session identifier
        student_profile: Student's learning profile

    Returns:
        Configured CoLearningAgent instance

    Example:
        >>> agent = create_colearning_agent(
        ...     session_id="student-123",
        ...     student_profile={
        ...         'name': 'Ahmed',
        ...         'level': 'beginner',
        ...         'language': 'en',
        ...         'current_chapter': 1
        ...     }
        ... )
        >>> response = await agent.teach("I want to start learning!")
    """
    return CoLearningAgent(session_id=session_id, student_profile=student_profile)
