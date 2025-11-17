"""
OpenAI Agent Service - integrates with OpenAI Agents SDK for summary generation
"""
import os
import logging
from typing import AsyncGenerator
from agents import Runner, Agent, OpenAIChatCompletionsModel, SQLiteSession,set_tracing_disabled
from openai import AsyncOpenAI
from dotenv import load_dotenv, find_dotenv

load_dotenv(find_dotenv())

logger = logging.getLogger(__name__)

GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

if not GOOGLE_API_KEY:
    logger.error("GOOGLE_API_KEY not found in environment variables. Please set it in .env file.")
    raise ValueError(
        "GOOGLE_API_KEY is required. Please:\n"
        "1. Copy api/.env.example to api/.env\n"
        "2. Add your Google API key to GOOGLE_API_KEY in .env file"
    )

set_tracing_disabled(False)

external_client = AsyncOpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=GOOGLE_API_KEY
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
    
    logger.info(f"Content word count: {word_count}, Target summary: {target_word_count} words")
    
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
                
                if hasattr(event, 'data'):
                    # Try event.data.delta
                    if hasattr(event.data, 'delta') and event.data.delta:
                        delta_text = event.data.delta
                    # Try event.data.content if delta not available
                    elif hasattr(event.data, 'content') and event.data.content:
                        delta_text = event.data.content
                
                # Direct event.delta check
                if not delta_text and hasattr(event, 'delta') and event.delta:
                    delta_text = event.delta
                
                # Direct event.content check
                if not delta_text and hasattr(event, 'content') and event.content:
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
    content: str,
    page_id: str,
    programming_level: str,
    ai_proficiency: str
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
    logger.info(f"Starting personalization for page_id: {page_id}, Programming: {programming_level}, AI: {ai_proficiency}")
    
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
                
                if hasattr(event, 'data'):
                    if hasattr(event.data, 'delta') and event.data.delta:
                        delta_text = event.data.delta
                    elif hasattr(event.data, 'content') and event.data.content:
                        delta_text = event.data.content
                
                if not delta_text and hasattr(event, 'delta') and event.delta:
                    delta_text = event.delta
                
                if not delta_text and hasattr(event, 'content') and event.content:
                    delta_text = event.content
                
                if delta_text:
                    logger.debug(f"Yielding personalized delta: {delta_text[:50]}...")
                    yield delta_text
        
        logger.info(f"Personalization completed for page_id: {page_id}")
        
    except Exception as e:
        logger.error(f"Error generating personalized content for page_id {page_id}: {str(e)}")
        raise


def build_personalization_instructions(programming_level: str, ai_proficiency: str) -> str:
    """
    Build proficiency-specific instructions for the personalization agent.
    
    Decision 5 from research.md: Level-specific instruction templates
    
    Args:
        programming_level: Novice, Beginner, Intermediate, or Expert
        ai_proficiency: Novice, Beginner, Intermediate, or Expert
    
    Returns:
        str: Tailored instructions for the agent
    """
    # Base instruction
    base = "You are an expert educational content personalizer. Your task is to adapt the provided content to match the user's experience level.\n\n"
    
    # Programming experience adaptations
    prog_instructions = {
        "Novice": "Programming Experience (Novice): Explain fundamental programming concepts in simple terms. Avoid jargon. Use everyday analogies. Break down complex ideas into small steps. Assume no prior coding knowledge.",
        "Beginner": "Programming Experience (Beginner): Explain programming concepts clearly with basic examples. Use simple code snippets when helpful. Assume familiarity with basic syntax but not advanced patterns.",
        "Intermediate": "Programming Experience (Intermediate): Reference standard programming patterns and best practices. Use technical terminology appropriately. Assume comfort with common algorithms and data structures.",
        "Expert": "Programming Experience (Expert): Focus on advanced techniques, performance optimizations, and architectural patterns. Skip basic explanations. Discuss trade-offs and edge cases."
    }
    
    # AI proficiency adaptations
    ai_instructions = {
        "Novice": "AI Proficiency (Novice): Introduce AI concepts from first principles. Explain what AI agents are, how they work, and why they matter. Avoid assuming any AI/ML background.",
        "Beginner": "AI Proficiency (Beginner): Explain AI concepts with practical examples. Clarify common AI terminology. Show how AI tools are used in development without deep theory.",
        "Intermediate": "AI Proficiency (Intermediate): Reference AI frameworks, model types, and prompt engineering techniques. Assume understanding of basic ML concepts and agent architectures.",
        "Expert": "AI Proficiency (Expert): Discuss advanced AI agent patterns, fine-tuning strategies, and production deployment considerations. Focus on cutting-edge techniques and research insights."
    }
    
    # Combine instructions
    prog_inst = prog_instructions.get(programming_level, prog_instructions["Beginner"])
    ai_inst = ai_instructions.get(ai_proficiency, ai_instructions["Beginner"])
    
    instructions = f"""{base}
{prog_inst}

{ai_inst}

Guidelines:
- Adapt the tone and depth to match both proficiency levels
- Maintain the original content's structure and key points
- Add clarifying examples for lower proficiency levels
- Skip redundant explanations for higher proficiency levels
- Use clear paragraphs, not bullet points
- Keep response focused and concise (aim for 300-500 words)
- Do not use markdown formatting (no ###, **, or -)
"""
    
    return instructions
