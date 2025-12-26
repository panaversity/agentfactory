"""
LoopAgent Example: Iterative Quality Improvement

Demonstrates iterative refinement where an agent loops until quality
criteria are met or max iterations reached.

Pattern:
- Agent evaluates content quality
- If quality threshold not met, agent improves content
- Calls exit_loop when quality criteria satisfied
- max_iterations provides safety limit

Key Insight:
    The exit_loop tool is critical - without it, the loop never terminates.
    The agent must be instructed clearly when to call exit_loop.

Usage:
    python workflows/quality_loop.py
"""

import asyncio
from google.adk.agents import LlmAgent, LoopAgent
from google.adk.runners import InMemoryRunner
from google.adk.tools import exit_loop


# =============================================================================
# Quality Metrics Tool
# =============================================================================

def evaluate_quality(
    clarity_score: int,
    completeness_score: int,
    accuracy_score: int,
    issues_found: list[str]
) -> dict:
    """
    Evaluate content quality on multiple dimensions.

    Args:
        clarity_score: 1-10 rating for clarity
        completeness_score: 1-10 rating for completeness
        accuracy_score: 1-10 rating for accuracy
        issues_found: List of specific issues to address

    Returns:
        Quality assessment with pass/fail determination
    """
    avg_score = (clarity_score + completeness_score + accuracy_score) / 3
    passed = avg_score >= 7.0 and len(issues_found) == 0

    return {
        "scores": {
            "clarity": clarity_score,
            "completeness": completeness_score,
            "accuracy": accuracy_score,
            "average": round(avg_score, 1)
        },
        "issues": issues_found,
        "quality_passed": passed,
        "recommendation": "Ready to publish" if passed else "Needs improvement"
    }


def improve_content(
    original_text: str,
    improvements: list[str],
    improved_text: str
) -> dict:
    """
    Apply improvements to content.

    Args:
        original_text: The text being improved
        improvements: List of improvements made
        improved_text: The improved version

    Returns:
        Improvement record
    """
    return {
        "status": "improved",
        "changes_made": improvements,
        "improved_text": improved_text,
        "iteration_complete": True
    }


# =============================================================================
# Quality Checker Agent
# =============================================================================

quality_checker = LlmAgent(
    name="quality_checker",
    model="gemini-2.5-flash",
    instruction="""You are a content quality specialist. Your job is to:

1. EVALUATE the content using evaluate_quality:
   - clarity_score: How clear and readable (1-10)
   - completeness_score: How complete and thorough (1-10)
   - accuracy_score: How accurate and correct (1-10)
   - issues_found: List specific problems

2. DECIDE based on quality:
   - If average score >= 7.0 AND no issues: Call exit_loop with the final content
   - If score < 7.0 OR issues exist: Call improve_content with fixes

3. IMPROVE if needed:
   - Address each issue specifically
   - Make the content clearer and more complete
   - After improving, evaluate again in the next iteration

IMPORTANT: You MUST call exit_loop when quality is sufficient.
Never get stuck in an infinite improvement loop.
If content is already good, call exit_loop immediately.

Quality thresholds:
- Clarity: Clear sentences, good structure, easy to understand
- Completeness: All key points covered, examples included
- Accuracy: Factually correct, no contradictions""",
    description="Evaluates and improves content quality iteratively",
    tools=[evaluate_quality, improve_content, exit_loop]
)


# =============================================================================
# Quality Loop: Iterate Until Good Enough
# =============================================================================

quality_loop = LoopAgent(
    name="quality_loop",
    description="Iteratively improves content until quality threshold met",
    sub_agents=[quality_checker],
    max_iterations=5  # Safety limit prevents infinite loops
)

# Output:
# Iteration 1: Evaluate -> Score 5.5 -> Improve
# Iteration 2: Evaluate -> Score 6.2 -> Improve
# Iteration 3: Evaluate -> Score 7.8 -> exit_loop (quality achieved)
#
# If max_iterations reached before quality threshold, loop exits with
# the best version achieved.


# =============================================================================
# Convergence Tracking
# =============================================================================

def track_convergence(iteration: int, score: float, threshold: float = 7.0) -> dict:
    """
    Track quality score convergence across iterations.

    Args:
        iteration: Current iteration number
        score: Current quality score
        threshold: Target quality threshold

    Returns:
        Convergence metrics
    """
    gap = threshold - score
    return {
        "iteration": iteration,
        "current_score": score,
        "threshold": threshold,
        "gap": max(0, gap),
        "converged": score >= threshold,
        "progress_percentage": min(100, (score / threshold) * 100)
    }


# =============================================================================
# Evaluation Test Cases
# =============================================================================

EVAL_CASES = {
    "eval_set_id": "quality_loop_tests",
    "eval_cases": [
        {
            "eval_id": "poor_quality_needs_improvement",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "Improve this: AI is good. It does things."}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Improved content..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "evaluate_quality", "args": {"clarity_score": 3, "completeness_score": 2}},
                            {"name": "improve_content", "args": {}},
                            {"name": "evaluate_quality", "args": {"clarity_score": 7, "completeness_score": 7}},
                            {"name": "exit_loop", "args": {}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "quality_loop",
                "user_id": "test_user"
            }
        },
        {
            "eval_id": "already_high_quality",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "Check quality: AI agents represent a paradigm shift in software development, enabling autonomous task completion through tool use, planning, and iterative refinement."}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Content passes quality check..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "evaluate_quality", "args": {"clarity_score": 9}},
                            {"name": "exit_loop", "args": {}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "quality_loop",
                "user_id": "test_user"
            }
        },
        {
            "eval_id": "max_iterations_reached",
            "conversation": [
                {
                    "user_content": {
                        "parts": [{"text": "This content has fundamental issues that require major rewriting."}],
                        "role": "user"
                    },
                    "final_response": {
                        "parts": [{"text": "Best effort after max iterations..."}],
                        "role": "model"
                    },
                    "intermediate_data": {
                        "tool_uses": [
                            {"name": "evaluate_quality", "args": {}},
                            {"name": "improve_content", "args": {}},
                            {"name": "evaluate_quality", "args": {}},
                            {"name": "improve_content", "args": {}},
                            {"name": "evaluate_quality", "args": {}},
                            {"name": "improve_content", "args": {}},
                            {"name": "evaluate_quality", "args": {}},
                            {"name": "improve_content", "args": {}},
                            {"name": "evaluate_quality", "args": {}}
                        ]
                    }
                }
            ],
            "session_input": {
                "app_name": "quality_loop",
                "user_id": "test_user"
            }
        }
    ]
}


# =============================================================================
# Main Execution
# =============================================================================

async def main():
    """Run the quality loop on sample content."""
    print("=" * 60)
    print("Quality Loop: LoopAgent Example")
    print("=" * 60)
    print("\nLoop behavior:")
    print("  - Evaluate quality (clarity, completeness, accuracy)")
    print("  - If score < 7.0: Improve and re-evaluate")
    print("  - If score >= 7.0: Call exit_loop")
    print("  - Max iterations: 5 (safety limit)")
    print("\n" + "-" * 60)

    runner = InMemoryRunner(agent=quality_loop)

    # Intentionally poor content to trigger improvement loop
    poor_content = """
    AI is cool. It can do stuff. Agents are like AI but they do more things.
    You can use them for work. They're pretty good.
    """

    print(f"\nInput (poor quality):\n{poor_content.strip()}")
    print("\n" + "-" * 60)

    print("\nExpected behavior:")
    print("  Iteration 1: Low score -> Improve")
    print("  Iteration 2: Better score -> Maybe improve more")
    print("  Iteration N: Score >= 7.0 -> exit_loop")
    print("\n" + "-" * 60)

    response = await runner.run_debug(
        f"Evaluate and improve this content until it meets quality standards:\n\n{poor_content}"
    )

    print("\nFinal Output:")
    print("-" * 60)
    print(response)


if __name__ == "__main__":
    asyncio.run(main())
