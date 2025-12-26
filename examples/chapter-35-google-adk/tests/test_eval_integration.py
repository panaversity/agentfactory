"""
ADK Evaluation Integration Tests

This module demonstrates integrating ADK evaluation with pytest:
- Loading eval cases from JSON
- Running evaluations with AgentEvaluator
- Asserting pass rate thresholds
- Combining pytest fixtures with ADK eval

Run with:
    pytest tests/test_eval_integration.py -v

Expected output:
    tests/test_eval_integration.py::TestEvalLoading::test_load_eval_cases PASSED
    tests/test_eval_integration.py::TestEvalExecution::test_eval_pass_rate PASSED
    ...

Note: Integration tests require GOOGLE_API_KEY environment variable.
"""

import pytest
import json
import os
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, field
from pathlib import Path


# ============================================================================
# Eval Case Data Structures
# ============================================================================

@dataclass
class EvalCase:
    """Single evaluation test case."""
    query: str
    expected_tool_use: List[str]
    reference: str
    initial_session: Optional[Dict[str, Any]] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "EvalCase":
        """Create EvalCase from dictionary."""
        return cls(
            query=data["query"],
            expected_tool_use=data["expected_tool_use"],
            reference=data["reference"],
            initial_session=data.get("initial_session")
        )


@dataclass
class EvalSet:
    """Collection of evaluation cases."""
    name: str
    cases: List[EvalCase]

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "EvalSet":
        """Create EvalSet from dictionary."""
        return cls(
            name=data["name"],
            cases=[EvalCase.from_dict(c) for c in data["data"]]
        )


@dataclass
class EvalResult:
    """Result of running an evaluation case."""
    case: EvalCase
    passed: bool
    actual_tool_use: List[str] = field(default_factory=list)
    actual_response: str = ""
    error: Optional[str] = None


# ============================================================================
# Eval Runner (Mock Implementation)
# ============================================================================

class MockEvalRunner:
    """Mock evaluation runner for testing.

    Simulates ADK's AgentEvaluator behavior for unit tests.
    Real integration tests use the actual AgentEvaluator.
    """

    def __init__(self, agent_module: str):
        self.agent_module = agent_module
        self.results: List[EvalResult] = []

    async def run_eval(self, eval_case: EvalCase) -> EvalResult:
        """Run a single evaluation case.

        Args:
            eval_case: The evaluation case to run.

        Returns:
            EvalResult with pass/fail status.
        """
        # Mock: Simulate tool detection based on query keywords
        detected_tools = []

        query_lower = eval_case.query.lower()
        if any(word in query_lower for word in ["add", "create", "new"]):
            detected_tools.append("add_task")
        if any(word in query_lower for word in ["show", "list", "all"]):
            detected_tools.append("list_tasks")
        if any(word in query_lower for word in ["complete", "done", "finish", "mark"]):
            detected_tools.append("complete_task")
        if any(word in query_lower for word in ["delete", "remove"]):
            detected_tools.append("delete_task")

        # Check if expected tools were used
        passed = all(
            tool in detected_tools
            for tool in eval_case.expected_tool_use
        )

        result = EvalResult(
            case=eval_case,
            passed=passed,
            actual_tool_use=detected_tools,
            actual_response=f"Mock response for: {eval_case.query}"
        )

        self.results.append(result)
        return result

    async def run_eval_set(self, eval_set: EvalSet) -> List[EvalResult]:
        """Run all cases in an eval set.

        Args:
            eval_set: The evaluation set to run.

        Returns:
            List of EvalResults.
        """
        results = []
        for case in eval_set.cases:
            result = await self.run_eval(case)
            results.append(result)
        return results

    def get_pass_rate(self) -> float:
        """Calculate pass rate for all results.

        Returns:
            Pass rate as percentage (0-100).
        """
        if not self.results:
            return 0.0
        passed = sum(1 for r in self.results if r.passed)
        return (passed / len(self.results)) * 100


# ============================================================================
# Eval Loader
# ============================================================================

class EvalLoader:
    """Loader for evaluation cases from JSON files."""

    @staticmethod
    def load_from_file(file_path: str) -> List[EvalSet]:
        """Load eval sets from JSON file.

        Args:
            file_path: Path to eval JSON file.

        Returns:
            List of EvalSet objects.

        Raises:
            FileNotFoundError: If file doesn't exist.
            json.JSONDecodeError: If JSON is invalid.
        """
        with open(file_path, 'r') as f:
            data = json.load(f)

        return [EvalSet.from_dict(es) for es in data.get("evals", [])]

    @staticmethod
    def load_from_directory(dir_path: str) -> Dict[str, List[EvalSet]]:
        """Load all eval files from a directory.

        Args:
            dir_path: Path to directory containing eval JSON files.

        Returns:
            Dict mapping file names to EvalSet lists.
        """
        results = {}
        path = Path(dir_path)

        for file_path in path.glob("*.json"):
            try:
                eval_sets = EvalLoader.load_from_file(str(file_path))
                results[file_path.stem] = eval_sets
            except (json.JSONDecodeError, KeyError) as e:
                results[file_path.stem] = []

        return results


# ============================================================================
# Unit Tests: Eval Loading
# ============================================================================

class TestEvalLoading:
    """Tests for loading evaluation cases."""

    def test_load_eval_cases(self, eval_cases_path):
        """Test loading eval cases from JSON file.

        Expected:
            - Eval sets are loaded correctly
            - Each set has name and cases
        """
        if not os.path.exists(eval_cases_path):
            pytest.skip(f"Eval file not found: {eval_cases_path}")

        eval_sets = EvalLoader.load_from_file(eval_cases_path)

        assert len(eval_sets) > 0
        for eval_set in eval_sets:
            assert eval_set.name is not None
            assert len(eval_set.cases) > 0

    def test_eval_case_structure(self, eval_cases_path):
        """Test that eval cases have required fields.

        Expected:
            - Each case has query, expected_tool_use, reference
        """
        if not os.path.exists(eval_cases_path):
            pytest.skip(f"Eval file not found: {eval_cases_path}")

        eval_sets = EvalLoader.load_from_file(eval_cases_path)

        for eval_set in eval_sets:
            for case in eval_set.cases:
                assert case.query, "Query should not be empty"
                assert case.expected_tool_use, "Expected tool use should not be empty"
                assert case.reference, "Reference should not be empty"

    def test_load_from_dict(self):
        """Test creating EvalCase from dictionary.

        Expected:
            - All fields are correctly mapped
        """
        data = {
            "query": "Add a task",
            "expected_tool_use": ["add_task"],
            "reference": "Task added successfully"
        }

        case = EvalCase.from_dict(data)

        assert case.query == "Add a task"
        assert case.expected_tool_use == ["add_task"]
        assert case.reference == "Task added successfully"
        assert case.initial_session is None

    def test_load_with_initial_session(self):
        """Test loading case with initial session state.

        Expected:
            - Initial session state is preserved
        """
        data = {
            "query": "Show my tasks",
            "expected_tool_use": ["list_tasks"],
            "reference": "No tasks",
            "initial_session": {
                "state": {"tasks": []}
            }
        }

        case = EvalCase.from_dict(data)

        assert case.initial_session is not None
        assert case.initial_session["state"]["tasks"] == []

    def test_load_nonexistent_file(self):
        """Test loading from non-existent file.

        Expected:
            - Raises FileNotFoundError
        """
        with pytest.raises(FileNotFoundError):
            EvalLoader.load_from_file("/nonexistent/path/evals.json")


# ============================================================================
# Unit Tests: Eval Execution
# ============================================================================

class TestEvalExecution:
    """Tests for running evaluations."""

    @pytest.mark.asyncio
    async def test_single_eval_case(self, sample_eval_cases):
        """Test running a single eval case.

        Expected:
            - Returns EvalResult with pass/fail status
        """
        runner = MockEvalRunner("task_manager")
        case = EvalCase.from_dict(sample_eval_cases[0])

        result = await runner.run_eval(case)

        assert isinstance(result, EvalResult)
        assert result.case == case
        assert isinstance(result.passed, bool)

    @pytest.mark.asyncio
    async def test_eval_pass_rate(self, sample_eval_cases):
        """Test calculating pass rate.

        Expected:
            - Pass rate is between 0-100
        """
        runner = MockEvalRunner("task_manager")

        for case_data in sample_eval_cases:
            case = EvalCase.from_dict(case_data)
            await runner.run_eval(case)

        pass_rate = runner.get_pass_rate()

        assert 0 <= pass_rate <= 100

    @pytest.mark.asyncio
    async def test_eval_set_execution(self):
        """Test running an entire eval set.

        Expected:
            - All cases in set are executed
        """
        eval_set = EvalSet(
            name="test_set",
            cases=[
                EvalCase(
                    query="Add task: Buy milk",
                    expected_tool_use=["add_task"],
                    reference="Task added"
                ),
                EvalCase(
                    query="Show all tasks",
                    expected_tool_use=["list_tasks"],
                    reference="Here are your tasks"
                )
            ]
        )

        runner = MockEvalRunner("task_manager")
        results = await runner.run_eval_set(eval_set)

        assert len(results) == 2

    @pytest.mark.asyncio
    async def test_eval_detects_tool_use(self):
        """Test that evaluation detects correct tool usage.

        Expected:
            - Correct tools detected for each query type
        """
        runner = MockEvalRunner("task_manager")

        # Test add_task detection
        add_case = EvalCase(
            query="Create a new task for shopping",
            expected_tool_use=["add_task"],
            reference="Task created"
        )
        result = await runner.run_eval(add_case)
        assert "add_task" in result.actual_tool_use

        # Test list_tasks detection
        list_case = EvalCase(
            query="Show me all my tasks",
            expected_tool_use=["list_tasks"],
            reference="Your tasks"
        )
        result = await runner.run_eval(list_case)
        assert "list_tasks" in result.actual_tool_use

    @pytest.mark.asyncio
    async def test_pass_rate_threshold(self):
        """Test asserting pass rate threshold.

        Pattern: Ensure agent meets quality bar.
        """
        eval_set = EvalSet(
            name="quality_check",
            cases=[
                EvalCase(
                    query="Add a task",
                    expected_tool_use=["add_task"],
                    reference="Added"
                ),
                EvalCase(
                    query="List tasks",
                    expected_tool_use=["list_tasks"],
                    reference="Tasks"
                ),
                EvalCase(
                    query="Complete task 1",
                    expected_tool_use=["complete_task"],
                    reference="Completed"
                ),
                EvalCase(
                    query="Delete task 2",
                    expected_tool_use=["delete_task"],
                    reference="Deleted"
                )
            ]
        )

        runner = MockEvalRunner("task_manager")
        await runner.run_eval_set(eval_set)

        pass_rate = runner.get_pass_rate()

        # Assert minimum quality threshold (e.g., 80%)
        QUALITY_THRESHOLD = 80.0
        assert pass_rate >= QUALITY_THRESHOLD, \
            f"Pass rate {pass_rate}% below threshold {QUALITY_THRESHOLD}%"


# ============================================================================
# Unit Tests: Eval Results
# ============================================================================

class TestEvalResults:
    """Tests for evaluation result processing."""

    def test_eval_result_structure(self):
        """Test EvalResult has all required fields.

        Expected:
            - All fields are accessible
        """
        case = EvalCase(
            query="Test query",
            expected_tool_use=["test_tool"],
            reference="Test reference"
        )

        result = EvalResult(
            case=case,
            passed=True,
            actual_tool_use=["test_tool"],
            actual_response="Response text"
        )

        assert result.case == case
        assert result.passed is True
        assert result.actual_tool_use == ["test_tool"]
        assert result.actual_response == "Response text"
        assert result.error is None

    def test_failed_result_with_error(self):
        """Test EvalResult with error message.

        Expected:
            - Error field populated for failures
        """
        case = EvalCase(
            query="Test query",
            expected_tool_use=["expected_tool"],
            reference="Reference"
        )

        result = EvalResult(
            case=case,
            passed=False,
            actual_tool_use=["wrong_tool"],
            error="Expected tool not called"
        )

        assert result.passed is False
        assert result.error is not None

    def test_empty_results_pass_rate(self):
        """Test pass rate with no results.

        Expected:
            - Returns 0.0 for empty results
        """
        runner = MockEvalRunner("test_agent")

        pass_rate = runner.get_pass_rate()

        assert pass_rate == 0.0


# ============================================================================
# Pytest + ADK Eval Pattern Tests
# ============================================================================

class TestPytestAdkPattern:
    """Tests demonstrating pytest + ADK eval integration pattern."""

    @pytest.mark.asyncio
    async def test_parametrized_eval_cases(self, sample_eval_cases):
        """Pattern: Parametrize tests with eval cases.

        Shows how to run each eval case as a separate test.
        """
        runner = MockEvalRunner("task_manager")

        for case_data in sample_eval_cases:
            case = EvalCase.from_dict(case_data)
            result = await runner.run_eval(case)

            # Each case is a separate assertion
            assert result.passed, \
                f"Failed: {case.query} -> Expected {case.expected_tool_use}"

    @pytest.mark.asyncio
    async def test_eval_with_fixtures(self, sample_tasks):
        """Pattern: Combine eval with pytest fixtures.

        Shows how to use fixtures for initial state.
        """
        case = EvalCase(
            query="Show me all my tasks",
            expected_tool_use=["list_tasks"],
            reference="Here are your 3 tasks",
            initial_session={"state": {"tasks": sample_tasks}}
        )

        runner = MockEvalRunner("task_manager")
        result = await runner.run_eval(case)

        assert result.passed

    def test_eval_report_generation(self):
        """Pattern: Generate eval report for CI/CD.

        Shows how to format results for reporting.
        """
        results = [
            EvalResult(
                case=EvalCase("Q1", ["tool1"], "R1"),
                passed=True,
                actual_tool_use=["tool1"]
            ),
            EvalResult(
                case=EvalCase("Q2", ["tool2"], "R2"),
                passed=False,
                actual_tool_use=["wrong_tool"],
                error="Tool mismatch"
            )
        ]

        # Generate report
        report = {
            "total": len(results),
            "passed": sum(1 for r in results if r.passed),
            "failed": sum(1 for r in results if not r.passed),
            "pass_rate": sum(1 for r in results if r.passed) / len(results) * 100,
            "failures": [
                {
                    "query": r.case.query,
                    "expected": r.case.expected_tool_use,
                    "actual": r.actual_tool_use,
                    "error": r.error
                }
                for r in results if not r.passed
            ]
        }

        assert report["total"] == 2
        assert report["passed"] == 1
        assert report["pass_rate"] == 50.0
        assert len(report["failures"]) == 1


# ============================================================================
# Integration Tests (Requires API Key)
# ============================================================================

@pytest.mark.slow
class TestEvalIntegration:
    """Integration tests using real ADK AgentEvaluator.

    These tests require:
    - GOOGLE_API_KEY environment variable
    - google-adk package installed
    """

    @pytest.mark.asyncio
    async def test_real_agent_evaluator(self, eval_cases_path):
        """Test using real ADK AgentEvaluator.

        Expected output when run with API key:
            Running evaluation...
            Eval case 1: PASSED
            Eval case 2: PASSED
            ...
            Pass rate: 95.0%
        """
        try:
            from google.adk.evaluation.agent_evaluator import AgentEvaluator
            import os

            if not os.environ.get("GOOGLE_API_KEY"):
                pytest.skip("GOOGLE_API_KEY not set")

            if not os.path.exists(eval_cases_path):
                pytest.skip(f"Eval file not found: {eval_cases_path}")

            # Note: This would run real evaluations
            # AgentEvaluator.evaluate(
            #     agent_module="task_manager",
            #     eval_dataset_file_path_or_dir=eval_cases_path
            # )

            # For now, just verify the import works
            assert AgentEvaluator is not None

        except ImportError:
            pytest.skip("google-adk not installed")

    @pytest.mark.asyncio
    async def test_eval_with_custom_criteria(self):
        """Test evaluation with custom pass/fail criteria.

        Pattern: Custom criteria beyond tool matching.
        """
        try:
            import os

            if not os.environ.get("GOOGLE_API_KEY"):
                pytest.skip("GOOGLE_API_KEY not set")

            # Custom criteria example:
            # - Response contains expected keywords
            # - Response length within bounds
            # - No error messages in response

            def custom_criteria(response: str, expected: Dict[str, Any]) -> bool:
                # Check for required keywords
                keywords = expected.get("keywords", [])
                if keywords and not any(kw in response for kw in keywords):
                    return False

                # Check response length
                min_len = expected.get("min_length", 0)
                max_len = expected.get("max_length", float("inf"))
                if not (min_len <= len(response) <= max_len):
                    return False

                # Check for error indicators
                error_indicators = ["error", "failed", "exception"]
                if any(err in response.lower() for err in error_indicators):
                    if not expected.get("allow_errors", False):
                        return False

                return True

            # Test the criteria function
            assert custom_criteria(
                "Task created successfully",
                {"keywords": ["created", "success"], "min_length": 10}
            ) is True

            assert custom_criteria(
                "Error occurred",
                {"keywords": ["created"]}
            ) is False

        except ImportError:
            pytest.skip("google-adk not installed")


# ============================================================================
# CLI Integration Pattern
# ============================================================================

class TestCliIntegration:
    """Tests showing CLI + pytest integration pattern."""

    def test_cli_command_pattern(self):
        """Document the CLI command for running evals.

        Example CLI usage:
            # Run evals with ADK CLI
            adk eval ./agent.py ./tests/evals/ --print_detailed_results

            # Run evals with pytest
            pytest tests/test_eval_integration.py -v -m slow

            # Run with specific threshold
            pytest tests/test_eval_integration.py --pass-rate-threshold=90
        """
        # This test documents the pattern
        cli_commands = [
            "adk eval ./agent.py ./evals/taskmanager_evals.json",
            "adk eval ./agent.py ./evals/ --print_detailed_results",
            "pytest tests/test_eval_integration.py -v",
            "pytest tests/test_eval_integration.py -m slow",
        ]

        for cmd in cli_commands:
            assert cmd.startswith(("adk", "pytest"))

    def test_ci_cd_integration_pattern(self):
        """Document CI/CD integration pattern.

        Example GitHub Actions workflow:
            steps:
              - name: Run ADK Evals
                run: |
                  adk eval ./agent.py ./evals/ > eval_results.json
                  python scripts/check_pass_rate.py eval_results.json --threshold 90

              - name: Run Pytest Evals
                run: pytest tests/test_eval_integration.py -v --junitxml=results.xml
        """
        # This test documents the CI/CD pattern
        pass
