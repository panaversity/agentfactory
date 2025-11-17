"""
OpenAI Agent Service - integrates with OpenAI Agents SDK for summary generation
"""

import os
import logging
from typing import AsyncGenerator
from agents import (
    Runner,
    Agent,
    OpenAIChatCompletionsModel,
    SQLiteSession,
    set_tracing_disabled,
)
from openai import AsyncOpenAI
from dotenv import load_dotenv, find_dotenv

load_dotenv(find_dotenv())

logger = logging.getLogger(__name__)

GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

if not GOOGLE_API_KEY:
    logger.error(
        "GOOGLE_API_KEY not found in environment variables. Please set it in .env file."
    )
    raise ValueError(
        "GOOGLE_API_KEY is required. Please:\n"
        "1. Copy api/.env.example to api/.env\n"
        "2. Add your Google API key to GOOGLE_API_KEY in .env file"
    )

set_tracing_disabled(False)

external_client = AsyncOpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=GOOGLE_API_KEY,
)

model = OpenAIChatCompletionsModel(
    openai_client=external_client,
    model="gemini-2.0-flash",
)


async def generate_summary(content: str, page_id: str) -> AsyncGenerator[str, None]:
    """
    Generate AI-powered summary of content using OpenAI Agents SDK.

    Uses Runner.run_streamed() with Agent for streaming responses with
    proportional summary length calculation (30-35% of original, 150-500 word bounds).

    Args:
        content: Full page content text to summarize
        page_id: Unique identifier for the content page (for logging)

    Yields:
        str: Chunks of summary text as they are generated

    Raises:
        Exception: If OpenAI Agents SDK call fails
    """
    logger.info(f"Starting summary generation for page_id: {page_id}")

    # Calculate word count for proportional summary
    word_count = len(content.split())
    target_word_count = max(150, min(500, int(word_count * 0.225)))  # 20-25%, bounded

    logger.info(
        f"Content word count: {word_count}, Target summary: {target_word_count} words"
    )

    instructions = f"""You are an expert content summarizer. Your task is to create a clear, concise summary of the provided text in natural paragraph form.

Requirements:
- Target length: {target_word_count} words (±10%)
- Write in flowing paragraphs, not bullet points or lists
- Use single line breaks between paragraphs
- Maintain key concepts and insights
- Use clear, professional language
- Preserve important technical details
- Do not add information not present in the original text
- No markdown formatting (no ###, **, or -)

Structure your response as natural paragraphs covering: brief overview, main points, and key takeaways.
"""

    try:
        # Create Agent instance
        agent = Agent(
            name="Content Summarizer",
            instructions=instructions,
            model=model,
        )

        # Create in-memory session (no file needed)
        session = SQLiteSession(session_id=page_id)  # Defaults to in-memory

        # Run agent with streaming
        result = Runner.run_streamed(
            starting_agent=agent,
            input=f"Summarize this content:\n\n{content}",
            session=session,
        )

        # Stream response chunks - iterate over events
        async for event in result.stream_events():
            # Log event type for debugging
            logger.debug(f"Event type: {event.type}")

            # Handle different event types for text streaming
            if event.type == "raw_response_event":
                # Check various possible delta locations in the event
                delta_text = None

                if hasattr(event, "data"):
                    # Try event.data.delta
                    if hasattr(event.data, "delta") and event.data.delta:
                        delta_text = event.data.delta
                    # Try event.data.content if delta not available
                    elif hasattr(event.data, "content") and event.data.content:
                        delta_text = event.data.content

                # Direct event.delta check
                if not delta_text and hasattr(event, "delta") and event.delta:
                    delta_text = event.delta

                # Direct event.content check
                if not delta_text and hasattr(event, "content") and event.content:
                    delta_text = event.content

                if delta_text:
                    logger.debug(f"Yielding delta: {delta_text[:50]}...")
                    yield delta_text

        logger.info(f"Summary generation completed for page_id: {page_id}")

    except Exception as e:
        logger.error(f"Error generating summary for page_id {page_id}: {str(e)}")
        raise


# T037-T042: Separate personalization agent
async def generate_personalized_content(
    content: str, page_id: str, programming_level: str, ai_proficiency: str
) -> AsyncGenerator[str, None]:
    """
    Generate AI-personalized content based on user proficiency levels.

    Uses separate Agent instance (Content Personalizer) with proficiency-specific
    instructions to tailor content complexity to user's programming experience
    and AI knowledge.

    Args:
        content: Full page content text to personalize
        page_id: Unique identifier for the content page
        programming_level: User's programming proficiency (Novice/Beginner/Intermediate/Expert)
        ai_proficiency: User's AI knowledge proficiency (Novice/Beginner/Intermediate/Expert)

    Yields:
        str: Chunks of personalized content as they are generated

    Raises:
        Exception: If OpenAI Agents SDK call fails
    """
    logger.info(
        f"Starting personalization for page_id: {page_id}, Programming: {programming_level}, AI: {ai_proficiency}"
    )

    # T039: Build proficiency-specific instructions
    instructions = build_personalization_instructions(programming_level, ai_proficiency)

    try:
        # T038: Create separate Agent instance (Content Personalizer)
        agent = Agent(
            name="Content Personalizer",
            instructions=instructions,
            model=model,
        )

        # T040: Construct session_id with proficiency levels for context isolation
        session_id = f"{page_id}_{programming_level}_{ai_proficiency}"
        session = SQLiteSession(session_id=session_id)

        # T041: Implement streaming with Runner.run_streamed()
        result = Runner.run_streamed(
            starting_agent=agent,
            input=f"Personalize this content for the user:\n\n{content}",
            session=session,
        )

        # Stream response chunks
        async for event in result.stream_events():
            logger.debug(f"Event type: {event.type}")

            if event.type == "raw_response_event":
                delta_text = None

                if hasattr(event, "data"):
                    if hasattr(event.data, "delta") and event.data.delta:
                        delta_text = event.data.delta
                    elif hasattr(event.data, "content") and event.data.content:
                        delta_text = event.data.content

                if not delta_text and hasattr(event, "delta") and event.delta:
                    delta_text = event.delta

                if not delta_text and hasattr(event, "content") and event.content:
                    delta_text = event.content

                if delta_text:
                    logger.debug(f"Yielding personalized delta: {delta_text[:50]}...")
                    yield delta_text

        logger.info(f"Personalization completed for page_id: {page_id}")

    except Exception as e:
        logger.error(
            f"Error generating personalized content for page_id {page_id}: {str(e)}"
        )
        raise


def build_personalization_instructions(
    programming_level: str, ai_proficiency: str
) -> str:
    """
    Build proficiency-specific instructions for the personalization agent.

    Decision 5 from research.md: Level-specific instruction templates

    Args:
        programming_level: Novice, Beginner, Intermediate, or Expert
        ai_proficiency: Novice, Beginner, Intermediate, or Expert

    Returns:
        str: Tailored instructions for the agent
    """

    # Define unified reader personas
    personas = {
        (
            "Novice",
            "Novice",
        ): "a complete beginner to both programming and AI. Use very simple language and everyday analogies throughout. Explain all technical terms. Focus on 'what' and 'why' before 'how'.",
        (
            "Novice",
            "Beginner",
        ): "new to programming but has heard about AI tools. Keep programming explanations very simple with analogies, but you can mention AI tools and concepts directly without over-explaining them.",
        (
            "Novice",
            "Intermediate",
        ): "new to programming but comfortable with AI concepts. Simplify programming explanations heavily with analogies, while using proper AI terminology naturally (agents, prompts, models).",
        (
            "Novice",
            "Expert",
        ): "new to programming but an AI expert. Use beginner-friendly programming analogies while freely discussing advanced AI patterns, agent architectures, and prompt engineering.",
        (
            "Beginner",
            "Novice",
        ): "has basic coding knowledge but new to AI. Use simple programming examples and syntax, but explain AI concepts from first principles (what agents are, how they help).",
        (
            "Beginner",
            "Beginner",
        ): "a beginner in both programming and AI. Use clear code examples and straightforward AI tool explanations. Balance simplicity with building understanding.",
        (
            "Beginner",
            "Intermediate",
        ): "a beginner programmer comfortable with AI tools. Keep programming examples simple, but reference AI frameworks and concepts naturally.",
        (
            "Beginner",
            "Expert",
        ): "a beginner programmer but an AI expert. Use basic programming explanations while discussing advanced AI agent patterns and architectures.",
        (
            "Intermediate",
            "Novice",
        ): "an experienced programmer new to AI. Use standard programming terminology and patterns freely, but explain AI concepts from scratch.",
        (
            "Intermediate",
            "Beginner",
        ): "an experienced programmer learning AI. Reference programming best practices normally while explaining AI tools and concepts clearly.",
        (
            "Intermediate",
            "Intermediate",
        ): "experienced in both programming and AI. Use technical terminology naturally for both domains. Focus on practical integration.",
        (
            "Intermediate",
            "Expert",
        ): "an experienced programmer and AI expert. Use standard programming terminology while diving deep into AI agent orchestration and advanced patterns.",
        (
            "Expert",
            "Novice",
        ): "a senior developer new to AI. Discuss architectural patterns and advanced programming freely, but introduce AI concepts from first principles.",
        (
            "Expert",
            "Beginner",
        ): "a senior developer learning AI. Use advanced programming terminology while explaining AI tools and workflows clearly.",
        (
            "Expert",
            "Intermediate",
        ): "a senior developer comfortable with AI. Discuss architecture, performance, and trade-offs naturally while referencing AI frameworks and patterns.",
        (
            "Expert",
            "Expert",
        ): "a senior developer and AI expert. Focus on advanced techniques, architectural decisions, production considerations, and cutting-edge AI patterns.",
    }

    persona = personas.get(
        (programming_level, ai_proficiency), personas[("Beginner", "Beginner")]
    )

    instructions = f"""You are rewriting educational technical content for **{persona}** using the original document as the authoritative source. Preserve the original’s structure, depth, examples, and code while adapting wording to the reader profile.

CRITICAL RULES
1. NEVER use meta-commentary such as "Here's a personalized version", "Okay, here's", "tailored for", "adapted for", or similar phrases.
2. NEVER add section headers that label skill levels (e.g., "Programming Experience:", "AI Proficiency:").
3. START DIRECTLY with the content — begin with the title or the first paragraph (no preamble, no framing lines).
4. Produce ONE unified explanation that blends proficiency dimensions naturally; do NOT split the document into separate sections by level.
5. PRESERVE THE FULL DEPTH and the exact section organization of the original content (headings, subheadings, order).

FORMATTING REQUIREMENTS — Match the original style exactly
- Headers & structure
  - Use `##` for main sections.
  - Use `###` for subsections.
  - Use `####` for minor headings and special callouts.
  - Preserve the same headings and the same ordering as the original.
- Text formatting
  - Use `**bold**` for key terms and emphasis.
  - Use `` `inline code` `` for commands, variables, filenames, and technical tokens.
  - Use normal paragraphs for explanations.
- Code blocks (MUST include)
  - Use fenced code blocks with language tags (```bash```, ```python```, ```json```, etc.).
  - Include **all** code examples, commands, and terminal output from the original.
  - Preserve exact formatting inside code blocks; do not reformat or shorten them.
- Lists
  - Use `1. 2. 3.` for ordered steps; `-` for bullets.
  - Preserve multi-level nesting exactly as in the original.
- Special callouts
  - Preserve blockquotes (`>`), `:::tip`, `:::note`, `:::warning`, and any special sections such as `#### 💬 AI Colearning Prompt`, `#### 🎓 Expert Insight`, `#### 🤝 Practice Exercise`.
  - Keep emojis exactly as in the original.
- Tables
  - Use Markdown table syntax (`|` and `-`) and preserve all rows/columns.
- Diagrams
  - Place ASCII diagrams inside code blocks and keep them verbatim.
- Expected Outcomes / Common Mistakes / Practice Exercises
  - Preserve these sections and their level of detail when present.

CONTENT DEPTH & LENGTH
- Match or slightly exceed the original length:
  - If original = 500 words → produce 450–550 words.
  - If original = 1000 words → produce 900–1100 words.
- NEVER reduce the original’s substantive content; you may add clarifications but must not remove original details.

ADAPTATION STRATEGY (how to change wording without removing content)
1. Terminology
   - Novice programming: add short analogies and inline clarifications for technical terms.
   - Expert programming: use precise technical vocabulary and concise phrasing.
   - Novice AI: add brief, accurate explanations of AI concepts when referenced.
   - Expert AI: reference advanced patterns naturally without over-explaining.
2. Examples
   - Keep ALL examples from the original unchanged.
   - Add brief clarifying sentences for lower-proficiency readers (outside code blocks).
   - For experts, omit only redundant explanatory sentences (do NOT remove examples).
3. Depth
   - Maintain full technical accuracy — do not drop steps or reasoning.
   - Clarify; do not simplify by deletion.
4. Tone
   - Adjust tone to persona (encouraging for novices; direct for experts) but do not label or separate content by tone.

CLARIFICATIONS & ADDITIONS
- If a short clarification is required, place it as a brief paragraph outside code blocks.
- Mark clarifications with `**Note:**` only if the original uses that style; otherwise use plain text.
- Never modify original code, tables, diagrams, special callouts, or required examples.

DELIVERY RULES
- Begin immediately with the content (title or first paragraph).
- Provide a single, unified narrative voice that adapts complexity naturally.
- Preserve every structural element from the original (headings, code, tables, diagrams, callouts).
- If the original includes "What Not To Do" or bad examples, keep them intact.

EXAMPLE OF CORRECT OUTPUT STYLE (follow exactly)
Begin directly with content, e.g.:

## Part A: What Is Spec-Kit Plus?

Before installing anything, let's understand what Spec-Kit Plus actually is.

### The Architecture: Three Independent Layers

Spec-Kit Plus is an opinionated toolkit for Specification-Driven Development (SDD). It has three independent but integrated components:

**1. The Framework** (The actual Spec-Kit Plus toolkit)
- File templates for specifications, plans, and tasks
...

(Include all original code blocks, callouts, tables, diagrams, and sections exactly.)

END.

"""

    return instructions
