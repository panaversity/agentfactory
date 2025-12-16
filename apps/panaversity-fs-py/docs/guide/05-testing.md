# Testing Guide

> Run, understand, and extend the test suite

**Spec Reference**: [Feature 039: PanaversityFS Production Hardening](../../../../specs/039-panaversity-fs-hardening/spec.md)

The test suite verifies all functional requirements (FR-001 to FR-031), success criteria (SC-001 to SC-010), and Alloy invariants (R1-R7) defined in the authoritative specification.

## Test Overview

PanaversityFS has **301 tests** organized into 6 categories:

| Category | Count | Purpose | Location |
|----------|-------|---------|----------|
| Unit | ~170 | Component isolation | `tests/unit/` |
| Integration | 24 | Component interactions | `tests/integration/` |
| Property | 33 | Invariant verification | `tests/property/` |
| Performance | 9 | Latency/throughput | `tests/performance/` |
| E2E | 3 | Complete workflows | `tests/e2e/` |
| Edge Cases | 13 | Production scenarios | `tests/edge_cases/` |

## Running Tests

### All Tests
```bash
# Quick run (minimal output)
uv run pytest tests/ -q

# Verbose (see each test)
uv run pytest tests/ -v

# With coverage
uv run pytest tests/ --cov=panaversity_fs --cov-report=html
```

### By Category
```bash
# Unit tests only
uv run pytest tests/unit/ -v

# Integration tests
uv run pytest tests/integration/ -v

# Property-based tests (slower)
uv run pytest tests/property/ -v

# Performance benchmarks
uv run pytest tests/performance/ -v

# End-to-end workflows
uv run pytest tests/e2e/ -v

# Production edge cases
uv run pytest tests/edge_cases/ -v
```

### Single Test File
```bash
uv run pytest tests/unit/test_content_tools.py -v
```

### Single Test
```bash
uv run pytest tests/unit/test_content_tools.py::test_write_content_creates_file -v
```

### By Marker
```bash
# Async tests only
uv run pytest tests/ -m asyncio -v
```

## Test Categories Explained

### Unit Tests (`tests/unit/`)

Test individual components in isolation:

| File | Tests | Focus |
|------|-------|-------|
| `test_content_tools.py` | Content CRUD operations |
| `test_path_utils.py` | Path validation, traversal attacks |
| `test_journal.py` | FileJournal database operations |
| `test_audit_chain.py` | Audit log hash chain integrity |
| `test_metrics.py` | Prometheus instrumentation |
| `test_auth.py` | JWT token verification |
| `test_overlay_content.py` | User overlay personalization |
| `test_validate_book.py` | Book structure validation |
| `test_delta_build.py` | Incremental build detection |

### Integration Tests (`tests/integration/`)

Test component interactions:

| File | Tests | Focus |
|------|-------|-------|
| `test_conflict_detection.py` | Concurrent write scenarios |
| `test_journal_storage_atomic.py` | Transaction atomicity |
| `test_streaming_archive.py` | ZIP generation with real data |

### Property Tests (`tests/property/`) - Alloy Invariants

Use Hypothesis to verify Alloy-style invariants from the spec hold for all inputs. Each property test implements small-scope verification (3-5 instances) as specified in the formal verification section.

| File | Invariant | Alloy Assertion | Small-Scope Test |
|------|-----------|-----------------|------------------|
| `test_invariant_r1_schema.py` | R1 | `all p: ContentPath \| validContentPath[p]` | 3 paths: valid, invalid prefix, traversal |
| `test_invariant_r2_journal.py` | R2 | `journalStorageConsistent[j, s]` | 5 files: write, read, write, delete, write |
| `test_invariant_r5_overlay.py` | R5 | `readContent[l, u] = overlay or base` | 2 users × 2 lessons |
| `test_invariant_r6_audit.py` | R6 | `entry[n].new_hash == entry[n+1].prev_hash` | 4 consecutive operations |
| `test_invariant_r7_agent.py` | R7 | `agent_id != "system" and some agent_id` | 5 operations from different agents |

### Performance Tests (`tests/performance/`) - Success Criteria

Verify success criteria from the spec section "Measurable Outcomes":

| File | Success Criteria | Requirement |
|------|------------------|-------------|
| `test_archive_throughput.py` | SC-001, R4 | 500 files/200MB in <60s, <64MB memory |
| `test_overlay_latency.py` | SC-006 | Overlay reads add <10ms vs base |

### E2E Tests (`tests/e2e/`)

Complete user workflows:

- `test_complete_book_workflow.py`: Create book → add content → search → archive

### Edge Cases (`tests/edge_cases/`)

Production-like scenarios:

- `test_production_structure.py`: Real book directory structures

## Test Fixtures

Defined in `tests/conftest.py`:

```python
@pytest.fixture
async def setup_fs_backend():
    """Configure filesystem backend with temp directory."""
    # Sets up PANAVERSITY_STORAGE_BACKEND=fs
    # Creates temp directory
    # Yields for test
    # Cleans up after

@pytest.fixture
async def sample_book(setup_fs_backend):
    """Create a sample book with content."""
    # Creates books/test-book/content/...
    # Returns book_id, paths
```

## Writing New Tests

### Unit Test Example

```python
import pytest
from panaversity_fs.tools.content import read_content
from panaversity_fs.models import ReadContentInput

class TestMyFeature:
    @pytest.mark.asyncio
    async def test_feature_basic(self, setup_fs_backend):
        """Basic functionality test."""
        # Arrange
        params = ReadContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-lesson.md"
        )

        # Act
        result = await read_content(params)

        # Assert
        assert "content" in result

    @pytest.mark.asyncio
    async def test_feature_error_case(self, setup_fs_backend):
        """Test error handling."""
        params = ReadContentInput(
            book_id="nonexistent",
            path="missing.md"
        )

        with pytest.raises(ContentNotFoundError):
            await read_content(params)
```

### Property Test Example

```python
from hypothesis import given, strategies as st, settings

class TestMyInvariant:
    @pytest.mark.asyncio
    @given(
        user_id=st.text(min_size=5, max_size=10),
        content=st.text(min_size=10, max_size=100)
    )
    @settings(max_examples=20, deadline=None)
    async def test_invariant_holds(self, setup_fs_backend, user_id, content):
        """Verify invariant holds for all generated inputs."""
        # Invariant: After write, read returns same content
        await write_content(WriteContentInput(
            book_id="test",
            path="content/01-Part/01-Chapter/01-test.md",
            content=content,
            user_id=user_id
        ))

        result = await read_content(ReadContentInput(
            book_id="test",
            path="content/01-Part/01-Chapter/01-test.md",
            user_id=user_id
        ))

        assert json.loads(result)["content"] == content
```

### Integration Test Example

```python
class TestConflictScenario:
    @pytest.mark.asyncio
    async def test_concurrent_writes_detected(self, setup_fs_backend):
        """Two agents writing simultaneously should detect conflict."""
        # Setup: Create initial content
        path = "content/01-Part/01-Chapter/01-lesson.md"
        await create_base_content(path)

        # Agent 1 reads
        result1 = await read_content(...)
        hash1 = json.loads(result1)["file_hash_sha256"]

        # Agent 2 reads (same content)
        result2 = await read_content(...)
        hash2 = json.loads(result2)["file_hash_sha256"]

        # Agent 1 writes (succeeds)
        await write_content(..., expected_hash=hash1)

        # Agent 2 tries to write with stale hash (fails)
        with pytest.raises(ConflictError):
            await write_content(..., expected_hash=hash2)
```

## Debugging Tests

### Verbose Output
```bash
uv run pytest tests/unit/test_content_tools.py -v -s
# -s shows print statements
```

### Stop on First Failure
```bash
uv run pytest tests/ -x
```

### Show Local Variables
```bash
uv run pytest tests/ --tb=long
```

### Run Only Failed Tests
```bash
uv run pytest tests/ --lf
```

### PDB on Failure
```bash
uv run pytest tests/ --pdb
```

## Continuous Integration

GitHub Actions workflow (`.github/workflows/test.yml`):

```yaml
name: Test
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: astral-sh/setup-uv@v4
      - run: uv python install 3.13
      - run: uv sync --all-extras
      - run: uv run pytest tests/ -v --cov=panaversity_fs
        env:
          PANAVERSITY_STORAGE_BACKEND: fs
          PANAVERSITY_STORAGE_ROOT: /tmp/panaversity-test
```

## Test Maintenance

### Check Coverage
```bash
uv run pytest tests/ --cov=panaversity_fs --cov-report=term-missing
```

### Find Slow Tests
```bash
uv run pytest tests/ --durations=10
```

### Verify Property Test Examples
```bash
# Run with more examples
uv run pytest tests/property/ --hypothesis-seed=12345
```
