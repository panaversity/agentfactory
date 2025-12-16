# Extending Guide

> Add new tools, features, and capabilities

**Spec Reference**: [Feature 039: PanaversityFS Production Hardening](../../../../specs/039-panaversity-fs-hardening/spec.md)

When extending PanaversityFS, follow the spec-driven development pattern:
1. Define requirements as FR-* (functional requirements)
2. Define invariants as R* (Alloy-style formal properties)
3. Define measurable outcomes as SC-* (success criteria)
4. Write property tests BEFORE implementation (verify invariants)
5. Implement with tests

## Adding a New MCP Tool

### Step 1: Define the Input Model

In `src/panaversity_fs/models.py`:

```python
class MyNewToolInput(BaseModel):
    """Input for my_new_tool."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(
        ...,
        pattern=r'^[a-z0-9-]+$',
        min_length=3,
        max_length=50,
        description="Book identifier"
    )
    some_param: str = Field(
        ...,
        min_length=1,
        description="Description of parameter"
    )
    optional_param: bool = Field(
        default=False,
        description="Optional flag"
    )
```

### Step 2: Implement the Tool

Create `src/panaversity_fs/tools/my_tool.py`:

```python
"""My new tool implementation."""
import json
from panaversity_fs.models import MyNewToolInput
from panaversity_fs.storage import get_operator
from panaversity_fs.audit import log_operation, OperationType
from panaversity_fs.path_utils import validate_content_path
from panaversity_fs.errors import SchemaViolationError


async def my_new_tool(params: MyNewToolInput) -> str:
    """
    Do something useful.

    Args:
        params: Validated input parameters

    Returns:
        JSON string with result
    """
    op = get_operator()

    # Validate path if applicable
    validation = validate_content_path(params.some_param)
    if not validation.is_valid:
        raise SchemaViolationError(validation.errors)

    # Do the work
    result = await op.read(f"books/{params.book_id}/{params.some_param}")

    # Log the operation
    await log_operation(
        operation=OperationType.READ,
        book_id=params.book_id,
        path=params.some_param,
        agent_id="system"  # Extract from context in real impl
    )

    return json.dumps({
        "status": "success",
        "data": result.decode()
    })
```

### Step 3: Export from tools/__init__.py

In `src/panaversity_fs/tools/__init__.py`:

```python
from .my_tool import my_new_tool

__all__ = [
    # ... existing tools
    "my_new_tool",
]
```

### Step 4: Register in the Server

In `src/panaversity_fs/app.py`:

```python
from panaversity_fs.models import MyNewToolInput
from panaversity_fs import tools

@mcp.tool(
    annotations={
        "readOnlyHint": True,      # or False if modifies data
        "idempotentHint": True,    # or False if not safe to retry
        "destructiveHint": False,  # True if deletes/overwrites
    }
)
async def my_new_tool(params: MyNewToolInput) -> str:
    """
    Brief description for MCP clients.

    Longer description if needed.
    """
    return await tools.my_new_tool(params)
```

### Step 5: Write Tests

Create `tests/unit/test_my_tool.py`:

```python
import pytest
import json
from panaversity_fs.tools.my_tool import my_new_tool
from panaversity_fs.models import MyNewToolInput


class TestMyNewTool:
    @pytest.mark.asyncio
    async def test_basic_functionality(self, setup_fs_backend):
        """Test the happy path."""
        # Setup
        # ... create necessary data

        # Execute
        result = await my_new_tool(MyNewToolInput(
            book_id="test-book",
            some_param="content/01-Part/01-Chapter/01-lesson.md"
        ))

        # Verify
        data = json.loads(result)
        assert data["status"] == "success"

    @pytest.mark.asyncio
    async def test_error_handling(self, setup_fs_backend):
        """Test error cases."""
        with pytest.raises(SchemaViolationError):
            await my_new_tool(MyNewToolInput(
                book_id="test-book",
                some_param="../../../etc/passwd"  # Invalid path
            ))
```

### Step 6: Update Documentation

Add to `docs/MCP-TOOLS.md`:

```markdown
### `my_new_tool`

Brief description.

**Annotations**: `readOnlyHint=true`, `idempotentHint=true`

**Input**:
```json
{
  "book_id": "ai-native-python",
  "some_param": "content/..."
}
```

**Output**:
```json
{
  "status": "success",
  "data": "..."
}
```
```

## Adding a New Error Type

In `src/panaversity_fs/errors.py`:

```python
class MyNewError(PanaversityFSError):
    """Raised when something specific happens."""

    def __init__(self, message: str, context: dict | None = None):
        super().__init__(message)
        self.context = context or {}
```

Use in tools:

```python
from panaversity_fs.errors import MyNewError

if some_condition:
    raise MyNewError(
        "Descriptive message",
        context={"key": "value"}
    )
```

## Adding Database Models

### Step 1: Define the Model

In `src/panaversity_fs/database/models.py`:

```python
class MyNewModel(Base):
    """My new database table."""
    __tablename__ = "my_new_table"

    id: Mapped[int] = mapped_column(primary_key=True)
    book_id: Mapped[str] = mapped_column(String(50), nullable=False, index=True)
    some_field: Mapped[str] = mapped_column(Text, nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        default=lambda: datetime.now(timezone.utc)
    )

    __table_args__ = (
        Index("idx_my_table_book", "book_id"),
    )
```

### Step 2: Generate Migration

```bash
uv run alembic revision --autogenerate -m "add my_new_table"
```

### Step 3: Review and Apply Migration

Check `src/panaversity_fs/database/migrations/versions/xxx_add_my_new_table.py`:

```python
def upgrade() -> None:
    op.create_table(
        'my_new_table',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('book_id', sa.String(50), nullable=False),
        sa.Column('some_field', sa.Text(), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index('idx_my_table_book', 'my_new_table', ['book_id'])


def downgrade() -> None:
    op.drop_index('idx_my_table_book', 'my_new_table')
    op.drop_table('my_new_table')
```

Apply:

```bash
uv run alembic upgrade head
```

## Adding Metrics

In `src/panaversity_fs/metrics.py`:

```python
# Counter
my_operation_total = Counter(
    "panaversityfs_my_operation_total",
    "Total my_operation calls",
    ["book_id", "status"]
)

# Histogram
my_operation_duration = Histogram(
    "panaversityfs_my_operation_duration_seconds",
    "My operation duration",
    ["book_id"]
)

# Gauge
my_resource_count = Gauge(
    "panaversityfs_my_resource_count",
    "Current resource count"
)

# Decorator for timing
def instrument_my_operation(func):
    @functools.wraps(func)
    async def wrapper(*args, **kwargs):
        book_id = kwargs.get("book_id", "unknown")
        with my_operation_duration.labels(book_id=book_id).time():
            try:
                result = await func(*args, **kwargs)
                my_operation_total.labels(book_id=book_id, status="success").inc()
                return result
            except Exception:
                my_operation_total.labels(book_id=book_id, status="error").inc()
                raise
    return wrapper
```

Use in tool:

```python
from panaversity_fs.metrics import instrument_my_operation

@instrument_my_operation
async def my_tool(params):
    ...
```

## Adding Property Tests

In `tests/property/test_invariant_rx_description.py`:

```python
"""Property-based tests for [invariant description]."""
import pytest
from hypothesis import given, strategies as st, settings, HealthCheck

# Hypothesis settings
HYPOTHESIS_SETTINGS = {
    "max_examples": 20,
    "deadline": None,
    "suppress_health_check": [HealthCheck.function_scoped_fixture]
}

# Custom strategies
valid_book_id = st.text(
    alphabet="abcdefghijklmnopqrstuvwxyz0123456789-",
    min_size=3,
    max_size=20
).filter(lambda s: s[0].isalnum() and not s.endswith('-'))


class TestMyInvariant:
    """Property tests for invariant RX."""

    @pytest.mark.asyncio
    @given(book_id=valid_book_id)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_invariant_holds(self, setup_fs_backend, book_id):
        """Invariant RX: [description] always holds."""
        # Arrange
        ...

        # Act
        ...

        # Assert invariant
        assert condition, f"Invariant violated: {details}"
```

## Adding Configuration Options

In `src/panaversity_fs/config.py`:

```python
class Config(BaseSettings):
    # Existing fields...

    # New configuration
    my_new_option: str = Field(
        default="default_value",
        description="Description of the option"
    )
    my_new_flag: bool = Field(
        default=False,
        description="Enable/disable something"
    )

    model_config = SettingsConfigDict(
        env_prefix="PANAVERSITY_",
        env_file=".env"
    )
```

Use:

```python
from panaversity_fs.config import get_config

config = get_config()
if config.my_new_flag:
    do_something(config.my_new_option)
```

## Adding Path Validation Rules

In `src/panaversity_fs/path_utils.py`:

```python
# Add new pattern
MY_NEW_PATTERN = re.compile(r'^my-prefix/[a-z0-9-]+\.ext$')


def validate_my_path(path: str) -> ValidationResult:
    """Validate my new path type."""
    errors = []

    # Security checks (always include these)
    if ".." in path:
        errors.append("Path traversal detected")
    if "\x00" in path:
        errors.append("Null byte injection detected")
    if path.startswith("/"):
        errors.append("Absolute path not allowed")

    # Format validation
    if not MY_NEW_PATTERN.match(path):
        errors.append(f"Path must match pattern: my-prefix/{{name}}.ext")

    return ValidationResult(
        is_valid=len(errors) == 0,
        errors=errors,
        normalized_path=path if not errors else None
    )
```

## Checklist for New Features (Spec-Driven)

### Before Implementation
- [ ] Define FR-* requirements in spec or feature doc
- [ ] Identify R* invariants that must hold
- [ ] Define SC-* success criteria with measurable thresholds
- [ ] Design instrumentation hooks per spec "Instrumentation Requirements"

### Implementation
- [ ] Input model in `models.py` with Pydantic validation
- [ ] Path validation in `path_utils.py` if new paths (FR-007 pattern)
- [ ] Implementation in `tools/` with error handling
- [ ] Export in `tools/__init__.py`
- [ ] Registration in `app.py` with MCP annotations
- [ ] Audit logging for operations (FR-020 pattern)

### Testing (Verify Invariants)
- [ ] Unit tests in `tests/unit/` for FR-* requirements
- [ ] Integration tests if multi-component (SC-002 pattern)
- [ ] Property tests in `tests/property/` for R* invariants (Hypothesis)
- [ ] Performance tests in `tests/performance/` for SC-* thresholds

### Documentation
- [ ] Update `docs/guide/03-tools-reference.md` with FR-* references
- [ ] Update `docs/guide/` if architectural changes
- [ ] Metrics for observability (SC-* instrumentation)
- [ ] Database migration if persistent state (FR-030, FR-031 pattern)
