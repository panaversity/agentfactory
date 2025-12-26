"""
ParallelAgent Example: Multi-Dimensional Text Analysis

Demonstrates parallel execution where multiple agents analyze the same input
simultaneously, producing independent results that are then merged.

Agents:
- Sentiment Analyzer: Determines emotional tone
- Entity Extractor: Identifies people, places, organizations
- Topic Classifier: Categorizes content themes

Key Pattern:
    All sub-agents receive the same input and run concurrently.
    Results are collected and available in the conversation context.
    Use this when analyses are independent and can benefit from parallelism.

Usage:
    python workflows/parallel_analysis.py
"""

import asyncio
from google.adk.agents import LlmAgent, ParallelAgent
from google.adk.runners import InMemoryRunner


# =============================================================================
# Analyzer 1: Sentiment Analysis
# =============================================================================

def report_sentiment(
    overall: str,
    confidence: float,
    positive_signals: list[str],
    negative_signals: list[str]
) -> dict:
    """
    Report sentiment analysis results.

    Args:
        overall: "positive", "negative", or "neutral"
        confidence: Confidence score 0.0 to 1.0
        positive_signals: Phrases indicating positive sentiment
        negative_signals: Phrases indicating negative sentiment

    Returns:
        Structured sentiment report
    """
    return {
        "analyzer": "sentiment",
        "overall_sentiment": overall,
        "confidence": confidence,
        "positive_signals": positive_signals,
        "negative_signals": negative_signals
    }


sentiment_analyzer = LlmAgent(
    name="sentiment_analyzer",
    model="gemini-2.5-flash",
    instruction="""You are a sentiment analysis specialist. For any text:
1. Determine if the overall sentiment is positive, negative, or neutral
2. Rate your confidence from 0.0 to 1.0
3. List specific phrases that indicate positive sentiment
4. List specific phrases that indicate negative sentiment
5. Call report_sentiment with your analysis

Be precise and objective. Base analysis only on the text provided.""",
    description="Analyzes emotional tone and sentiment of text",
    tools=[report_sentiment]
)


# =============================================================================
# Analyzer 2: Entity Extraction
# =============================================================================

def report_entities(
    people: list[str],
    organizations: list[str],
    locations: list[str],
    products: list[str],
    dates: list[str]
) -> dict:
    """
    Report extracted named entities.

    Args:
        people: Names of people mentioned
        organizations: Company/organization names
        locations: Geographic locations
        products: Product or service names
        dates: Date references

    Returns:
        Structured entity report
    """
    return {
        "analyzer": "entities",
        "people": people,
        "organizations": organizations,
        "locations": locations,
        "products": products,
        "dates": dates,
        "total_entities": len(people) + len(organizations) + len(locations) + len(products) + len(dates)
    }


entity_extractor = LlmAgent(
    name="entity_extractor",
    model="gemini-2.5-flash",
    instruction="""You are a named entity recognition specialist. For any text:
1. Identify all people mentioned by name
2. Identify all organizations (companies, institutions)
3. Identify all locations (cities, countries, regions)
4. Identify all products or services mentioned
5. Identify any date references
6. Call report_entities with your findings

Only extract entities explicitly mentioned. Don't infer or assume.""",
    description="Extracts named entities from text",
    tools=[report_entities]
)


# =============================================================================
# Analyzer 3: Topic Classification
# =============================================================================

def report_topics(
    primary_topic: str,
    secondary_topics: list[str],
    keywords: list[str],
    domain: str
) -> dict:
    """
    Report topic classification results.

    Args:
        primary_topic: Main topic/theme
        secondary_topics: Additional themes
        keywords: Key terms representing content
        domain: General domain (tech, business, health, etc.)

    Returns:
        Structured topic report
    """
    return {
        "analyzer": "topics",
        "primary_topic": primary_topic,
        "secondary_topics": secondary_topics,
        "keywords": keywords,
        "domain": domain
    }


topic_classifier = LlmAgent(
    name="topic_classifier",
    model="gemini-2.5-flash",
    instruction="""You are a topic classification specialist. For any text:
1. Identify the primary topic or theme
2. List 2-3 secondary topics if present
3. Extract 5-10 keywords that capture the essence
4. Classify the general domain (technology, business, health, politics, etc.)
5. Call report_topics with your classification

Focus on the main themes, not minor details.""",
    description="Classifies content by topic and domain",
    tools=[report_topics]
)


# =============================================================================
# Parallel Analysis: All Three Run Concurrently
# =============================================================================

parallel_analyzer = ParallelAgent(
    name="parallel_analyzer",
    description="Runs sentiment, entity, and topic analysis in parallel",
    sub_agents=[sentiment_analyzer, entity_extractor, topic_classifier]
)

# Output:
# All three analyzers run simultaneously on the same input.
# Each produces its own structured output via its tool.
# Results are independent and can be merged by a coordinator.


# =============================================================================
# Result Merger (Aggregator Pattern)
# =============================================================================

def merge_analysis_results(
    sentiment_result: dict,
    entity_result: dict,
    topic_result: dict
) -> dict:
    """
    Merge results from parallel analyzers into a unified report.

    Args:
        sentiment_result: Output from sentiment analyzer
        entity_result: Output from entity extractor
        topic_result: Output from topic classifier

    Returns:
        Unified analysis report
    """
    return {
        "analysis_complete": True,
        "sentiment": sentiment_result,
        "entities": entity_result,
        "topics": topic_result,
        "summary": {
            "overall_sentiment": sentiment_result.get("overall_sentiment"),
            "entity_count": entity_result.get("total_entities", 0),
            "primary_topic": topic_result.get("primary_topic"),
            "domain": topic_result.get("domain")
        }
    }


# =============================================================================
# Evaluation Test Cases
# =============================================================================

EVAL_CASES = {
    "eval_set_id": "parallel_analysis_tests",
    "eval_cases": [
        {
            "eval_id": "tech_news_analysis",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "Apple announced the new iPhone 16 in Cupertino today. Tim Cook said it's their best phone ever with revolutionary AI features."}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Analysis complete..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "report_sentiment", "args": {"overall": "positive"}},
                            {"name": "report_entities", "args": {"people": ["Tim Cook"], "organizations": ["Apple"], "products": ["iPhone 16"]}},
                            {"name": "report_topics", "args": {"primary_topic": "product launch", "domain": "technology"}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "parallel_analyzer",
                "user_id": "test_user"
            }
        },
        {
            "eval_id": "negative_sentiment_detection",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "The company's stock crashed after the disappointing earnings report. Investors are frustrated with management's poor decisions."}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Analysis complete..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "report_sentiment", "args": {"overall": "negative"}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "parallel_analyzer",
                "user_id": "test_user"
            }
        }
    ]
}


# =============================================================================
# Main Execution
# =============================================================================

async def main():
    """Run parallel analysis on sample text."""
    print("=" * 60)
    print("Parallel Analysis: ParallelAgent Example")
    print("=" * 60)
    print("\nAnalyzers running in parallel:")
    print("  - Sentiment Analyzer")
    print("  - Entity Extractor")
    print("  - Topic Classifier")
    print("\n" + "-" * 60)

    runner = InMemoryRunner(agent=parallel_analyzer)

    sample_text = """
    Google announced today that their new Gemini 2.5 model has achieved
    remarkable performance on coding benchmarks. Sundar Pichai, speaking
    from Mountain View, called it "a major leap forward for AI." The model
    will be available through Google Cloud starting next month. Developers
    are excited about the improved reasoning capabilities, though some
    researchers have raised concerns about the environmental impact of
    training such large models.
    """

    print(f"\nInput Text:\n{sample_text.strip()}")
    print("\n" + "-" * 60)

    response = await runner.run_debug(
        f"Analyze this text:\n\n{sample_text}"
    )

    print("\nParallel Analysis Results:")
    print("-" * 60)
    print(response)


if __name__ == "__main__":
    asyncio.run(main())
