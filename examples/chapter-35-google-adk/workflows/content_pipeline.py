"""
SequentialAgent Example: Content Creation Pipeline

Demonstrates a three-stage content pipeline where output flows sequentially:
1. Researcher: Gathers information on a topic
2. Writer: Creates a draft based on research
3. Editor: Refines and polishes the content

Key Pattern:
    Each agent's output becomes part of the conversation context for the next.
    The pipeline enforces a deterministic order: research -> write -> edit.

Usage:
    python workflows/content_pipeline.py
"""

import asyncio
from google.adk.agents import LlmAgent, SequentialAgent
from google.adk.runners import InMemoryRunner
from google.adk.tools import google_search


# =============================================================================
# Stage 1: Researcher Agent
# =============================================================================

def create_research_notes(topic: str, key_points: list[str]) -> dict:
    """
    Store structured research notes for the writer.

    Args:
        topic: The main topic being researched
        key_points: List of important findings

    Returns:
        Confirmation of stored research
    """
    return {
        "status": "research_complete",
        "topic": topic,
        "key_points": key_points,
        "note": "Research ready for writing phase"
    }


researcher = LlmAgent(
    name="researcher",
    model="gemini-2.5-flash",
    instruction="""You are a research specialist. Your job is to:
1. Analyze the given topic thoroughly
2. Identify 3-5 key points that should be covered
3. Use google_search if you need current information
4. Call create_research_notes with your findings

Be concise but comprehensive. Focus on facts, not opinions.
End your response with a summary of key findings for the writer.""",
    description="Researches topics and gathers key information",
    tools=[google_search, create_research_notes]
)


# =============================================================================
# Stage 2: Writer Agent
# =============================================================================

def create_draft(title: str, sections: list[str], word_count: int) -> dict:
    """
    Create a structured draft document.

    Args:
        title: Article title
        sections: List of section texts
        word_count: Approximate word count

    Returns:
        Draft document structure
    """
    return {
        "status": "draft_complete",
        "title": title,
        "sections": sections,
        "word_count": word_count,
        "note": "Draft ready for editing phase"
    }


writer = LlmAgent(
    name="writer",
    model="gemini-2.5-flash",
    instruction="""You are a content writer. Based on the research provided:
1. Create a compelling title
2. Write 2-3 clear sections covering the key points
3. Use simple, engaging language
4. Call create_draft with your structured content

Write in a professional but accessible tone.
Focus on clarity over complexity.""",
    description="Creates draft content from research",
    tools=[create_draft]
)


# =============================================================================
# Stage 3: Editor Agent
# =============================================================================

def publish_content(final_title: str, final_content: str, improvements: list[str]) -> dict:
    """
    Finalize and publish the edited content.

    Args:
        final_title: Polished title
        final_content: Final edited content
        improvements: List of improvements made

    Returns:
        Publication confirmation
    """
    return {
        "status": "published",
        "title": final_title,
        "content": final_content,
        "improvements": improvements,
        "note": "Content pipeline complete"
    }


editor = LlmAgent(
    name="editor",
    model="gemini-2.5-flash",
    instruction="""You are an editor. Review the draft and:
1. Improve clarity and flow
2. Fix any grammatical issues
3. Ensure the title is compelling
4. Make the content more engaging
5. Call publish_content with the final version

List the specific improvements you made.
Be thorough but preserve the writer's voice.""",
    description="Refines and polishes content",
    tools=[publish_content]
)


# =============================================================================
# Sequential Pipeline: Research -> Write -> Edit
# =============================================================================

content_pipeline = SequentialAgent(
    name="content_pipeline",
    description="Three-stage content creation: research, write, edit",
    sub_agents=[researcher, writer, editor]
)

# Output:
# The pipeline executes in strict order:
# 1. Researcher outputs: research notes and key findings
# 2. Writer sees researcher's output, creates draft
# 3. Editor sees full conversation, produces final content


# =============================================================================
# Evaluation Test Cases
# =============================================================================

EVAL_CASES = {
    "eval_set_id": "content_pipeline_tests",
    "eval_cases": [
        {
            "eval_id": "basic_content_creation",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "Create content about the benefits of remote work"}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Content about remote work benefits..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "create_research_notes", "args": {"topic": "remote work benefits"}},
                            {"name": "create_draft", "args": {"title": "Benefits of Remote Work"}},
                            {"name": "publish_content", "args": {"final_title": "Benefits of Remote Work"}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "content_pipeline",
                "user_id": "test_user"
            }
        },
        {
            "eval_id": "technical_topic",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "Write about Kubernetes container orchestration"}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Article about Kubernetes..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "create_research_notes", "args": {"topic": "Kubernetes"}},
                            {"name": "create_draft", "args": {}},
                            {"name": "publish_content", "args": {}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "content_pipeline",
                "user_id": "test_user"
            }
        }
    ]
}


# =============================================================================
# Main Execution
# =============================================================================

async def main():
    """Run the content pipeline with a sample topic."""
    print("=" * 60)
    print("Content Pipeline: SequentialAgent Example")
    print("=" * 60)
    print("\nPipeline stages:")
    print("  1. Researcher -> 2. Writer -> 3. Editor")
    print("\n" + "-" * 60)

    runner = InMemoryRunner(agent=content_pipeline)

    topic = "the rise of AI agents in software development"
    print(f"\nInput Topic: {topic}")
    print("-" * 60)

    response = await runner.run_debug(
        f"Create an article about: {topic}"
    )

    print("\nPipeline Output:")
    print("-" * 60)
    print(response)


if __name__ == "__main__":
    asyncio.run(main())
