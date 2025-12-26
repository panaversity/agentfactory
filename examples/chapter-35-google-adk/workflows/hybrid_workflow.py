"""
Hybrid Workflow Example: Combining Sequential, Parallel, and Loop Agents

Demonstrates a real-world pattern combining all three workflow types:
1. ParallelAgent: Gather data from multiple sources simultaneously
2. SequentialAgent: Process gathered data through a pipeline
3. LoopAgent: Iteratively refine until quality threshold met

Pattern:
    SequentialAgent(
        stage1: ParallelAgent([gatherers...]),    # Parallel gather
        stage2: SequentialAgent([processors...]), # Sequential process
        stage3: LoopAgent([refiner])              # Loop refine
    )

Real-World Use Case:
    Research Report Generation:
    - Parallel: Gather from news, academic, industry sources
    - Sequential: Synthesize, structure, format
    - Loop: Refine until publication-ready

Usage:
    python workflows/hybrid_workflow.py
"""

import asyncio
from google.adk.agents import LlmAgent, SequentialAgent, ParallelAgent, LoopAgent
from google.adk.runners import InMemoryRunner
from google.adk.tools import exit_loop, google_search


# =============================================================================
# STAGE 1: Parallel Data Gathering
# =============================================================================

def store_news_data(headlines: list[str], key_facts: list[str]) -> dict:
    """Store news research findings."""
    return {
        "source": "news",
        "headlines": headlines,
        "key_facts": key_facts
    }


def store_academic_data(papers: list[str], findings: list[str]) -> dict:
    """Store academic research findings."""
    return {
        "source": "academic",
        "papers": papers,
        "findings": findings
    }


def store_industry_data(companies: list[str], trends: list[str]) -> dict:
    """Store industry research findings."""
    return {
        "source": "industry",
        "companies": companies,
        "trends": trends
    }


news_gatherer = LlmAgent(
    name="news_gatherer",
    model="gemini-2.5-flash",
    instruction="""You are a news researcher. For the given topic:
1. Use google_search to find recent news
2. Identify 3-5 key headlines
3. Extract important facts
4. Call store_news_data with your findings

Focus on recent, reliable news sources.""",
    description="Gathers news and current events",
    tools=[google_search, store_news_data]
)

academic_gatherer = LlmAgent(
    name="academic_gatherer",
    model="gemini-2.5-flash",
    instruction="""You are an academic researcher. For the given topic:
1. Use google_search to find research papers and studies
2. Identify 2-3 relevant papers or studies
3. Extract key findings and conclusions
4. Call store_academic_data with your findings

Focus on peer-reviewed and credible academic sources.""",
    description="Gathers academic research and papers",
    tools=[google_search, store_academic_data]
)

industry_gatherer = LlmAgent(
    name="industry_gatherer",
    model="gemini-2.5-flash",
    instruction="""You are an industry analyst. For the given topic:
1. Use google_search to find industry reports and company news
2. Identify key companies and players
3. Extract market trends and predictions
4. Call store_industry_data with your findings

Focus on industry publications and company announcements.""",
    description="Gathers industry trends and company data",
    tools=[google_search, store_industry_data]
)

# All gatherers run in parallel
parallel_gatherers = ParallelAgent(
    name="data_gatherers",
    description="Gathers news, academic, and industry data in parallel",
    sub_agents=[news_gatherer, academic_gatherer, industry_gatherer]
)


# =============================================================================
# STAGE 2: Sequential Processing Pipeline
# =============================================================================

def create_synthesis(
    combined_insights: list[str],
    contradictions: list[str],
    confidence_areas: list[str]
) -> dict:
    """Create synthesized research summary."""
    return {
        "stage": "synthesis",
        "insights": combined_insights,
        "contradictions": contradictions,
        "confidence_areas": confidence_areas
    }


def create_structure(
    title: str,
    executive_summary: str,
    sections: list[dict]
) -> dict:
    """Create structured report outline."""
    return {
        "stage": "structure",
        "title": title,
        "executive_summary": executive_summary,
        "sections": sections
    }


def create_draft(
    full_report: str,
    word_count: int,
    citations: list[str]
) -> dict:
    """Create formatted report draft."""
    return {
        "stage": "draft",
        "report": full_report,
        "word_count": word_count,
        "citations": citations
    }


synthesizer = LlmAgent(
    name="synthesizer",
    model="gemini-2.5-flash",
    instruction="""You are a research synthesizer. Based on the gathered data:
1. Combine insights from all sources (news, academic, industry)
2. Identify any contradictions between sources
3. Note areas of high confidence (multiple sources agree)
4. Call create_synthesis with your analysis

Be objective and note source disagreements.""",
    description="Synthesizes data from multiple sources",
    tools=[create_synthesis]
)

structurer = LlmAgent(
    name="structurer",
    model="gemini-2.5-flash",
    instruction="""You are a report structurer. Based on the synthesis:
1. Create a compelling title
2. Write a brief executive summary (2-3 sentences)
3. Outline 3-4 main sections with key points
4. Call create_structure with your outline

Create a logical flow from overview to details.""",
    description="Creates report structure and outline",
    tools=[create_structure]
)

formatter = LlmAgent(
    name="formatter",
    model="gemini-2.5-flash",
    instruction="""You are a report formatter. Based on the structure:
1. Write the full report following the outline
2. Include proper citations for claims
3. Aim for 300-500 words
4. Call create_draft with the formatted report

Use clear, professional language.""",
    description="Formats the final report draft",
    tools=[create_draft]
)

# Processing happens in sequence
sequential_processor = SequentialAgent(
    name="report_processor",
    description="Synthesize, structure, then format the report",
    sub_agents=[synthesizer, structurer, formatter]
)


# =============================================================================
# STAGE 3: Quality Refinement Loop
# =============================================================================

def assess_report_quality(
    clarity: int,
    evidence_quality: int,
    structure: int,
    issues: list[str]
) -> dict:
    """Assess report quality on multiple dimensions."""
    avg = (clarity + evidence_quality + structure) / 3
    return {
        "scores": {
            "clarity": clarity,
            "evidence_quality": evidence_quality,
            "structure": structure,
            "average": round(avg, 1)
        },
        "issues": issues,
        "publication_ready": avg >= 7.5 and len(issues) == 0
    }


def refine_report(
    refinements: list[str],
    refined_report: str
) -> dict:
    """Apply refinements to the report."""
    return {
        "refinements_applied": refinements,
        "refined_report": refined_report
    }


quality_refiner = LlmAgent(
    name="quality_refiner",
    model="gemini-2.5-flash",
    instruction="""You are a report quality specialist. For the report:

1. ASSESS using assess_report_quality:
   - clarity: How clear and readable (1-10)
   - evidence_quality: How well-supported claims are (1-10)
   - structure: How well-organized (1-10)
   - issues: List specific problems

2. DECIDE:
   - If average >= 7.5 AND no issues: Call exit_loop with final report
   - If score < 7.5 OR issues exist: Call refine_report with improvements

3. REFINE if needed:
   - Address each issue specifically
   - Strengthen weak evidence
   - Improve clarity and flow

IMPORTANT: Call exit_loop when quality is sufficient.
Max 3 iterations to prevent over-editing.""",
    description="Iteratively refines report quality",
    tools=[assess_report_quality, refine_report, exit_loop]
)

# Refinement loops until quality threshold
refinement_loop = LoopAgent(
    name="quality_refinement",
    description="Iterate on report quality until publication-ready",
    sub_agents=[quality_refiner],
    max_iterations=3
)


# =============================================================================
# COMPLETE HYBRID WORKFLOW
# =============================================================================

hybrid_workflow = SequentialAgent(
    name="research_report_workflow",
    description="Complete research report: parallel gather, sequential process, loop refine",
    sub_agents=[
        parallel_gatherers,     # Stage 1: Parallel data gathering
        sequential_processor,   # Stage 2: Sequential processing
        refinement_loop         # Stage 3: Iterative refinement
    ]
)

# Output:
# ┌─────────────────────────────────────────────────────────────────┐
# │                    Hybrid Workflow Execution                     │
# ├─────────────────────────────────────────────────────────────────┤
# │                                                                  │
# │  Stage 1: PARALLEL (all run simultaneously)                     │
# │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐             │
# │  │ News         │ │ Academic     │ │ Industry     │             │
# │  │ Gatherer     │ │ Gatherer     │ │ Gatherer     │             │
# │  └──────┬───────┘ └──────┬───────┘ └──────┬───────┘             │
# │         │                │                │                      │
# │         └────────────────┼────────────────┘                      │
# │                          ▼                                       │
# │  Stage 2: SEQUENTIAL (one after another)                        │
# │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐             │
# │  │ Synthesizer  │→│ Structurer   │→│ Formatter    │             │
# │  └──────────────┘ └──────────────┘ └──────────────┘             │
# │                          │                                       │
# │                          ▼                                       │
# │  Stage 3: LOOP (iterate until quality met)                      │
# │  ┌──────────────────────────────────────────┐                   │
# │  │    Quality Refiner                        │                   │
# │  │    ┌────────┐   ┌────────┐   ┌────────┐  │                   │
# │  │    │Assess  │ → │Refine  │ → │Assess  │  │                   │
# │  │    └────────┘   └────────┘   └────────┘  │                   │
# │  │                      ↓                    │                   │
# │  │               exit_loop()                 │                   │
# │  └──────────────────────────────────────────┘                   │
# │                          │                                       │
# │                          ▼                                       │
# │              Publication-Ready Report                            │
# │                                                                  │
# └─────────────────────────────────────────────────────────────────┘


# =============================================================================
# Evaluation Test Cases
# =============================================================================

EVAL_CASES = {
    "eval_set_id": "hybrid_workflow_tests",
    "eval_cases": [
        {
            "eval_id": "complete_research_workflow",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "Create a research report on the current state of AI agents in enterprise software"}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Research report on AI agents..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            # Parallel stage
                            {"name": "store_news_data", "args": {}},
                            {"name": "store_academic_data", "args": {}},
                            {"name": "store_industry_data", "args": {}},
                            # Sequential stage
                            {"name": "create_synthesis", "args": {}},
                            {"name": "create_structure", "args": {}},
                            {"name": "create_draft", "args": {}},
                            # Loop stage
                            {"name": "assess_report_quality", "args": {}},
                            {"name": "exit_loop", "args": {}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "hybrid_workflow",
                "user_id": "test_user"
            }
        },
        {
            "eval_id": "workflow_with_refinement_iterations",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "Research and report on quantum computing breakthroughs in 2024"}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Quantum computing report..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "store_news_data", "args": {}},
                            {"name": "store_academic_data", "args": {}},
                            {"name": "store_industry_data", "args": {}},
                            {"name": "create_synthesis", "args": {}},
                            {"name": "create_structure", "args": {}},
                            {"name": "create_draft", "args": {}},
                            # Multiple refinement iterations
                            {"name": "assess_report_quality", "args": {"clarity": 6}},
                            {"name": "refine_report", "args": {}},
                            {"name": "assess_report_quality", "args": {"clarity": 8}},
                            {"name": "exit_loop", "args": {}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "hybrid_workflow",
                "user_id": "test_user"
            }
        }
    ]
}


# =============================================================================
# Main Execution
# =============================================================================

async def main():
    """Run the complete hybrid workflow."""
    print("=" * 70)
    print("Hybrid Workflow: Combined Sequential + Parallel + Loop Example")
    print("=" * 70)
    print("""
Workflow Architecture:

    ┌─────────────────────────────────────────────────────────────────┐
    │  Stage 1: PARALLEL GATHERING                                    │
    │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐               │
    │  │   News      │ │  Academic   │ │  Industry   │               │
    │  │  Gatherer   │ │  Gatherer   │ │  Gatherer   │               │
    │  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘               │
    │         └───────────────┼───────────────┘                       │
    │                         ▼                                       │
    │  Stage 2: SEQUENTIAL PROCESSING                                 │
    │  ┌─────────────┐→┌─────────────┐→┌─────────────┐               │
    │  │ Synthesizer │ │ Structurer  │ │  Formatter  │               │
    │  └─────────────┘ └─────────────┘ └──────┬──────┘               │
    │                                         ▼                       │
    │  Stage 3: REFINEMENT LOOP                                       │
    │  ┌───────────────────────────────────────────────┐             │
    │  │  Assess → Refine → Assess → ... → exit_loop  │             │
    │  └───────────────────────────────────────────────┘             │
    │                         ▼                                       │
    │              Publication-Ready Report                           │
    └─────────────────────────────────────────────────────────────────┘
    """)
    print("-" * 70)

    runner = InMemoryRunner(agent=hybrid_workflow)

    topic = "the impact of AI agents on software development productivity"
    print(f"\nResearch Topic: {topic}")
    print("-" * 70)

    response = await runner.run_debug(
        f"Create a comprehensive research report on: {topic}"
    )

    print("\nFinal Report:")
    print("-" * 70)
    print(response)


if __name__ == "__main__":
    asyncio.run(main())
