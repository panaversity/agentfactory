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

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

set_tracing_disabled(False)

external_client = AsyncOpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=os.getenv("GOOGLE_API_KEY")
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
