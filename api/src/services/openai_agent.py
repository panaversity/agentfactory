"""
OpenAI Agent Service - integrates with OpenAI Agents SDK for summary generation
"""
import os
import logging
from typing import AsyncGenerator
from openai import AsyncOpenAI

logger = logging.getLogger(__name__)

# Initialize OpenAI client
client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))


async def generate_summary(content: str, page_id: str) -> AsyncGenerator[str, None]:
    """
    Generate AI-powered summary of content using OpenAI streaming completion.
    
    Args:
        content: Full page content text to summarize
        page_id: Unique identifier for the content page (for logging)
    
    Yields:
        str: Chunks of summary text as they are generated
    
    Raises:
        Exception: If OpenAI API call fails
    """
    logger.info(f"Starting summary generation for page_id: {page_id}")
    
    # Calculate word count for proportional summary
    word_count = len(content.split())
    target_word_count = max(150, min(500, int(word_count * 0.225)))  # 20-25%, bounded
    
    system_prompt = f"""You are an expert content summarizer. Your task is to create a clear, 
    concise summary of the provided text.
    
    Requirements:
    - Target length: {target_word_count} words (±10%)
    - Maintain key concepts and insights
    - Use clear, professional language
    - Preserve important technical details
    - Do not add information not present in the original text
    """
    
    try:
        # Create streaming completion
        stream = await client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Summarize this content:\n\n{content}"}
            ],
            stream=True,
            temperature=0.7,
        )
        
        # Stream response chunks
        async for chunk in stream:
            if chunk.choices[0].delta.content:
                yield chunk.choices[0].delta.content
        
        logger.info(f"Summary generation completed for page_id: {page_id}")
        
    except Exception as e:
        logger.error(f"Error generating summary for page_id {page_id}: {str(e)}")
        raise
