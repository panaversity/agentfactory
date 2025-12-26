"""
Google ADK Workflow Examples

This package demonstrates the three primary workflow agents in Google ADK:
- SequentialAgent: Pipeline execution (A -> B -> C)
- ParallelAgent: Concurrent execution (A | B | C)
- LoopAgent: Iterative refinement until exit condition

Examples:
    >>> from workflows.content_pipeline import content_pipeline
    >>> from workflows.parallel_analysis import parallel_analyzer
    >>> from workflows.quality_loop import quality_loop
    >>> from workflows.hybrid_workflow import hybrid_workflow
"""

from .content_pipeline import content_pipeline
from .parallel_analysis import parallel_analyzer
from .quality_loop import quality_loop
from .hybrid_workflow import hybrid_workflow

__all__ = [
    "content_pipeline",
    "parallel_analyzer",
    "quality_loop",
    "hybrid_workflow",
]
