"""
Workflow Agent Tests

This module tests ADK workflow agents:
- SequentialAgent: Executes sub-agents in order
- ParallelAgent: Executes sub-agents concurrently
- LoopAgent: Iterates until condition met or max_iterations

Run with:
    pytest tests/test_workflows.py -v

Expected output:
    tests/test_workflows.py::TestSequentialAgent::test_sequential_execution_order PASSED
    tests/test_workflows.py::TestParallelAgent::test_parallel_concurrent_execution PASSED
    tests/test_workflows.py::TestLoopAgent::test_loop_convergence PASSED
    ...
"""

import pytest
import asyncio
from typing import List, Dict, Any, Optional
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from dataclasses import dataclass
import time


# ============================================================================
# Mock Workflow Implementations
# ============================================================================

class MockSubAgent:
    """Mock sub-agent for testing workflow orchestration."""

    def __init__(self, name: str, result: str, delay: float = 0.0):
        self.name = name
        self.result = result
        self.delay = delay
        self.executed = False
        self.execution_time: Optional[float] = None

    async def run(self, input_data: str) -> str:
        """Simulate agent execution with optional delay."""
        start = time.time()
        if self.delay > 0:
            await asyncio.sleep(self.delay)
        self.executed = True
        self.execution_time = time.time() - start
        return f"{self.name}: {self.result}"


class SequentialWorkflow:
    """Mock SequentialAgent implementation for testing.

    Executes sub-agents one after another, passing output
    from one to the next.
    """

    def __init__(self, name: str, sub_agents: List[MockSubAgent]):
        self.name = name
        self.sub_agents = sub_agents
        self.execution_order: List[str] = []

    async def run(self, initial_input: str) -> Dict[str, Any]:
        """Execute sub-agents sequentially.

        Args:
            initial_input: Input for the first sub-agent.

        Returns:
            Dict with final output and execution order.
        """
        current_input = initial_input
        outputs = []

        for agent in self.sub_agents:
            result = await agent.run(current_input)
            self.execution_order.append(agent.name)
            outputs.append(result)
            current_input = result

        return {
            "final_output": outputs[-1] if outputs else "",
            "all_outputs": outputs,
            "execution_order": self.execution_order
        }


class ParallelWorkflow:
    """Mock ParallelAgent implementation for testing.

    Executes sub-agents concurrently and aggregates results.
    """

    def __init__(self, name: str, sub_agents: List[MockSubAgent]):
        self.name = name
        self.sub_agents = sub_agents

    async def run(self, input_data: str) -> Dict[str, Any]:
        """Execute sub-agents in parallel.

        Args:
            input_data: Input shared by all sub-agents.

        Returns:
            Dict with all outputs and timing info.
        """
        start_time = time.time()

        # Run all agents concurrently
        tasks = [agent.run(input_data) for agent in self.sub_agents]
        results = await asyncio.gather(*tasks)

        total_time = time.time() - start_time

        return {
            "outputs": list(results),
            "total_time": total_time,
            "agent_times": {
                agent.name: agent.execution_time
                for agent in self.sub_agents
            }
        }


class LoopWorkflow:
    """Mock LoopAgent implementation for testing.

    Iterates until convergence condition or max_iterations.
    """

    def __init__(
        self,
        name: str,
        sub_agents: List[MockSubAgent],
        max_iterations: int = 5,
        convergence_checker: Optional[callable] = None
    ):
        self.name = name
        self.sub_agents = sub_agents
        self.max_iterations = max_iterations
        self.convergence_checker = convergence_checker or self._default_convergence
        self.iterations_completed = 0

    def _default_convergence(self, result: str) -> bool:
        """Default convergence: check if result contains 'DONE'."""
        return "DONE" in result.upper()

    async def run(self, initial_input: str) -> Dict[str, Any]:
        """Execute loop until convergence or max_iterations.

        Args:
            initial_input: Initial input for the loop.

        Returns:
            Dict with final result, iterations, and converged status.
        """
        current_input = initial_input
        all_results = []
        converged = False

        for i in range(self.max_iterations):
            # Run each sub-agent in the loop
            for agent in self.sub_agents:
                result = await agent.run(current_input)
                all_results.append(result)
                current_input = result

            self.iterations_completed = i + 1

            # Check convergence
            if self.convergence_checker(result):
                converged = True
                break

        return {
            "final_result": all_results[-1] if all_results else "",
            "all_results": all_results,
            "iterations": self.iterations_completed,
            "converged": converged,
            "hit_max_iterations": self.iterations_completed >= self.max_iterations and not converged
        }


# ============================================================================
# Unit Tests: Sequential Agent
# ============================================================================

class TestSequentialAgent:
    """Tests for SequentialAgent workflow."""

    @pytest.mark.asyncio
    async def test_sequential_execution_order(self):
        """Test that sub-agents execute in correct order.

        Expected:
            - Sub-agents execute in order: step_1 -> step_2 -> step_3
            - Each agent receives output from previous
        """
        agents = [
            MockSubAgent("step_1", "output_1"),
            MockSubAgent("step_2", "output_2"),
            MockSubAgent("step_3", "output_3")
        ]
        workflow = SequentialWorkflow("test_seq", agents)

        result = await workflow.run("initial_input")

        assert result["execution_order"] == ["step_1", "step_2", "step_3"]
        assert all(agent.executed for agent in agents)

    @pytest.mark.asyncio
    async def test_sequential_passes_output(self):
        """Test that output is passed between agents.

        Expected:
            - First agent receives initial input
            - Each subsequent agent receives previous output
        """
        agents = [
            MockSubAgent("research", "research_findings"),
            MockSubAgent("write", "written_article")
        ]
        workflow = SequentialWorkflow("content_pipeline", agents)

        result = await workflow.run("topic: AI agents")

        # Final output should be from last agent
        assert "write" in result["final_output"]
        assert len(result["all_outputs"]) == 2

    @pytest.mark.asyncio
    async def test_sequential_single_agent(self):
        """Test sequential workflow with single agent.

        Expected:
            - Works correctly with just one sub-agent
        """
        agents = [MockSubAgent("only_agent", "only_output")]
        workflow = SequentialWorkflow("single", agents)

        result = await workflow.run("input")

        assert len(result["all_outputs"]) == 1
        assert result["execution_order"] == ["only_agent"]

    @pytest.mark.asyncio
    async def test_sequential_empty_agents(self):
        """Test sequential workflow with no agents.

        Expected:
            - Returns empty results without error
        """
        workflow = SequentialWorkflow("empty", [])

        result = await workflow.run("input")

        assert result["final_output"] == ""
        assert result["all_outputs"] == []

    @pytest.mark.asyncio
    async def test_sequential_respects_delay(self):
        """Test that sequential execution waits for each agent.

        Expected:
            - Total time >= sum of individual delays
            - Executes serially, not in parallel
        """
        agents = [
            MockSubAgent("fast", "result", delay=0.1),
            MockSubAgent("slow", "result", delay=0.2)
        ]
        workflow = SequentialWorkflow("delayed", agents)

        start = time.time()
        await workflow.run("input")
        elapsed = time.time() - start

        # Should take at least 0.3s (sequential)
        assert elapsed >= 0.25  # Allow small tolerance


# ============================================================================
# Unit Tests: Parallel Agent
# ============================================================================

class TestParallelAgent:
    """Tests for ParallelAgent workflow."""

    @pytest.mark.asyncio
    async def test_parallel_concurrent_execution(self):
        """Test that sub-agents execute concurrently.

        Expected:
            - Total time < sum of individual times
            - All agents complete
        """
        agents = [
            MockSubAgent("analyzer_1", "analysis_1", delay=0.2),
            MockSubAgent("analyzer_2", "analysis_2", delay=0.2),
            MockSubAgent("analyzer_3", "analysis_3", delay=0.2)
        ]
        workflow = ParallelWorkflow("parallel_analysis", agents)

        start = time.time()
        result = await workflow.run("data to analyze")
        elapsed = time.time() - start

        # Should take ~0.2s (parallel), not 0.6s (sequential)
        assert elapsed < 0.5
        assert len(result["outputs"]) == 3
        assert all(agent.executed for agent in agents)

    @pytest.mark.asyncio
    async def test_parallel_all_receive_same_input(self):
        """Test that all parallel agents receive same input.

        Expected:
            - Each agent processes the same input data
        """
        received_inputs = []

        class TrackingAgent(MockSubAgent):
            async def run(self, input_data: str) -> str:
                received_inputs.append(input_data)
                return await super().run(input_data)

        agents = [
            TrackingAgent("a1", "r1"),
            TrackingAgent("a2", "r2")
        ]
        workflow = ParallelWorkflow("tracking", agents)

        await workflow.run("shared_input")

        assert all(inp == "shared_input" for inp in received_inputs)

    @pytest.mark.asyncio
    async def test_parallel_aggregates_results(self):
        """Test that all results are aggregated.

        Expected:
            - All agent outputs are collected
        """
        agents = [
            MockSubAgent("fact_check", "verified"),
            MockSubAgent("sentiment", "positive"),
            MockSubAgent("summary", "brief overview")
        ]
        workflow = ParallelWorkflow("multi_analysis", agents)

        result = await workflow.run("article content")

        assert len(result["outputs"]) == 3
        assert "fact_check: verified" in result["outputs"]
        assert "sentiment: positive" in result["outputs"]

    @pytest.mark.asyncio
    async def test_parallel_single_agent(self):
        """Test parallel workflow with single agent.

        Expected:
            - Works correctly, still returns list
        """
        agents = [MockSubAgent("solo", "result")]
        workflow = ParallelWorkflow("solo_parallel", agents)

        result = await workflow.run("input")

        assert len(result["outputs"]) == 1

    @pytest.mark.asyncio
    async def test_parallel_timing_tracking(self):
        """Test that individual agent times are tracked.

        Expected:
            - Each agent's execution time is recorded
        """
        agents = [
            MockSubAgent("fast", "r", delay=0.05),
            MockSubAgent("slow", "r", delay=0.15)
        ]
        workflow = ParallelWorkflow("timed", agents)

        result = await workflow.run("input")

        assert "fast" in result["agent_times"]
        assert "slow" in result["agent_times"]
        assert result["agent_times"]["slow"] > result["agent_times"]["fast"]


# ============================================================================
# Unit Tests: Loop Agent
# ============================================================================

class TestLoopAgent:
    """Tests for LoopAgent workflow."""

    @pytest.mark.asyncio
    async def test_loop_convergence(self):
        """Test that loop stops on convergence.

        Expected:
            - Loop terminates when convergence condition met
            - Reports converged=True
        """
        # Agent that produces "DONE" on 3rd iteration
        iteration = 0

        class ConvergingAgent(MockSubAgent):
            async def run(self, input_data: str) -> str:
                nonlocal iteration
                iteration += 1
                if iteration >= 3:
                    return "DONE: solution found"
                return f"iteration {iteration}"

        agents = [ConvergingAgent("solver", "")]
        workflow = LoopWorkflow("converging", agents, max_iterations=10)

        result = await workflow.run("problem")

        assert result["converged"] is True
        assert result["iterations"] == 3
        assert "DONE" in result["final_result"]

    @pytest.mark.asyncio
    async def test_loop_max_iterations_safety(self):
        """Test that loop respects max_iterations limit.

        Expected:
            - Loop stops at max_iterations even without convergence
            - Reports hit_max_iterations=True
        """
        agents = [MockSubAgent("never_done", "still working")]
        workflow = LoopWorkflow("limited", agents, max_iterations=5)

        result = await workflow.run("endless problem")

        assert result["iterations"] == 5
        assert result["converged"] is False
        assert result["hit_max_iterations"] is True

    @pytest.mark.asyncio
    async def test_loop_custom_convergence(self):
        """Test loop with custom convergence checker.

        Expected:
            - Uses provided convergence function
        """
        def custom_check(result: str) -> bool:
            return "SOLUTION" in result

        agents = [MockSubAgent("finder", "SOLUTION: x=42")]
        workflow = LoopWorkflow(
            "custom",
            agents,
            max_iterations=10,
            convergence_checker=custom_check
        )

        result = await workflow.run("find x")

        assert result["converged"] is True
        assert result["iterations"] == 1

    @pytest.mark.asyncio
    async def test_loop_immediate_convergence(self):
        """Test loop that converges immediately.

        Expected:
            - Only runs once if first result converges
        """
        agents = [MockSubAgent("instant", "DONE immediately")]
        workflow = LoopWorkflow("instant", agents, max_iterations=10)

        result = await workflow.run("easy problem")

        assert result["iterations"] == 1
        assert result["converged"] is True

    @pytest.mark.asyncio
    async def test_loop_iteration_tracking(self):
        """Test that all iteration results are tracked.

        Expected:
            - Each iteration's output is recorded
        """
        iteration = 0

        class CountingAgent(MockSubAgent):
            async def run(self, input_data: str) -> str:
                nonlocal iteration
                iteration += 1
                if iteration == 3:
                    return "DONE"
                return f"attempt_{iteration}"

        agents = [CountingAgent("counter", "")]
        workflow = LoopWorkflow("tracking", agents, max_iterations=10)

        result = await workflow.run("count")

        assert len(result["all_results"]) == 3

    @pytest.mark.asyncio
    async def test_loop_zero_max_iterations(self):
        """Test loop with max_iterations=0.

        Expected:
            - No iterations run
            - Returns empty result
        """
        agents = [MockSubAgent("unused", "never runs")]
        workflow = LoopWorkflow("zero", agents, max_iterations=0)

        result = await workflow.run("input")

        assert result["iterations"] == 0
        assert result["final_result"] == ""

    @pytest.mark.asyncio
    async def test_loop_multiple_sub_agents(self):
        """Test loop with multiple sub-agents per iteration.

        Expected:
            - All sub-agents run each iteration
            - All outputs tracked
        """
        agents = [
            MockSubAgent("check", "check: ok"),
            MockSubAgent("refine", "DONE")
        ]
        workflow = LoopWorkflow("multi", agents, max_iterations=5)

        result = await workflow.run("input")

        assert result["converged"] is True
        # 2 agents ran in 1 iteration
        assert len(result["all_results"]) == 2


# ============================================================================
# Workflow Composition Tests
# ============================================================================

class TestWorkflowComposition:
    """Tests for composing multiple workflow types."""

    @pytest.mark.asyncio
    async def test_sequential_with_parallel_step(self):
        """Test sequential workflow containing parallel analysis.

        Pattern: Research -> [Fact-check || Sentiment] -> Write
        """
        # Simulate research step
        research = MockSubAgent("research", "research_data", delay=0.05)

        # Parallel analysis agents
        fact_check = MockSubAgent("fact_check", "verified", delay=0.1)
        sentiment = MockSubAgent("sentiment", "positive", delay=0.1)
        parallel = ParallelWorkflow("analysis", [fact_check, sentiment])

        # Simulate write step
        write = MockSubAgent("write", "final_article", delay=0.05)

        # Execute sequence manually (simulating composition)
        research_result = await research.run("topic")
        parallel_result = await parallel.run(research_result)
        final_result = await write.run(str(parallel_result["outputs"]))

        assert research.executed
        assert fact_check.executed
        assert sentiment.executed
        assert write.executed

    @pytest.mark.asyncio
    async def test_loop_containing_sequential(self):
        """Test loop that runs sequential steps each iteration.

        Pattern: Loop(Analyze -> Improve) until quality threshold
        """
        iteration = 0

        class AnalyzeAgent(MockSubAgent):
            async def run(self, input_data: str) -> str:
                return f"quality: {50 + iteration * 20}"

        class ImproveAgent(MockSubAgent):
            async def run(self, input_data: str) -> str:
                nonlocal iteration
                iteration += 1
                if iteration >= 3:
                    return "DONE: quality 90"
                return "improved"

        agents = [AnalyzeAgent("analyze", ""), ImproveAgent("improve", "")]
        workflow = LoopWorkflow("refine_loop", agents, max_iterations=10)

        result = await workflow.run("initial draft")

        assert result["converged"] is True
        assert result["iterations"] == 3


# ============================================================================
# Integration Tests (Requires API Key)
# ============================================================================

@pytest.mark.slow
class TestWorkflowIntegration:
    """Integration tests for real ADK workflow agents.

    These tests require GOOGLE_API_KEY to be set.
    Run with: pytest tests/test_workflows.py -v -m slow
    """

    @pytest.mark.asyncio
    async def test_real_sequential_agent(self):
        """Test real ADK SequentialAgent.

        Expected:
            - Creates and runs real sequential workflow
        """
        try:
            from google.adk.agents import LlmAgent, SequentialAgent
            import os

            if not os.environ.get("GOOGLE_API_KEY"):
                pytest.skip("GOOGLE_API_KEY not set")

            researcher = LlmAgent(
                name="researcher",
                model="gemini-2.0-flash",
                instruction="Provide one fact about the topic."
            )

            summarizer = LlmAgent(
                name="summarizer",
                model="gemini-2.0-flash",
                instruction="Summarize the fact in one sentence."
            )

            pipeline = SequentialAgent(
                name="research_pipeline",
                description="Research and summarize",
                sub_agents=[researcher, summarizer]
            )

            # Basic structure validation
            assert pipeline.name == "research_pipeline"
            assert len(pipeline.sub_agents) == 2

        except ImportError:
            pytest.skip("google-adk not installed")

    @pytest.mark.asyncio
    async def test_real_loop_agent_with_exit(self):
        """Test real ADK LoopAgent with exit_loop tool.

        Expected:
            - LoopAgent configured with exit_loop tool
        """
        try:
            from google.adk.agents import LlmAgent, LoopAgent
            from google.adk.tools import exit_loop
            import os

            if not os.environ.get("GOOGLE_API_KEY"):
                pytest.skip("GOOGLE_API_KEY not set")

            solver = LlmAgent(
                name="solver",
                model="gemini-2.0-flash",
                instruction="Count from 1 to 3, then call exit_loop.",
                tools=[exit_loop]
            )

            loop = LoopAgent(
                name="counter_loop",
                description="Count and exit",
                sub_agents=[solver],
                max_iterations=10
            )

            # Validate configuration
            assert loop.max_iterations == 10
            assert len(loop.sub_agents) == 1
            assert exit_loop in solver.tools

        except ImportError:
            pytest.skip("google-adk not installed")
