---
title: "Capstone Project: Type-Safe Configuration Manager"
chapter: 27
lesson: 6
duration_minutes: 75
sidebar_position: 6
description: "Build a production-quality configuration management system using Pydantic, Generics, and multi-source loading. Synthesize the entire chapter's concepts into a portfolio-worthy project demonstrating professional design patterns."

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "System Architecture Design"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design multi-component system with clear responsibilities, documenting architectural decisions with reference to production requirements"

  - name: "Pydantic + Generics Integration"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can combine Pydantic models with Generic containers, implementing type-safe configuration access that enables IDE autocomplete"

  - name: "Configuration Precedence Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement layered config with documented precedence (defaults < file < env < CLI) and handle conflicts gracefully"

  - name: "Production Code Quality"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can write production-grade code with comprehensive error handling, security practices, logging, and unit tests"

  - name: "Architectural Justification"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain WHY each design decision was made, with reference to non-functional requirements (type safety, testability, maintainability)"

learning_objectives:
  - objective: "Synthesize Pydantic validation and Generic containers into a cohesive configuration system"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Capstone project demonstrates integration of both concepts"

  - objective: "Design multi-source configuration loading with documented precedence rules"
    proficiency_level: "B2"
    bloom_level: "Design"
    assessment_method: "Project supports YAML, environment variables, and CLI with clear priority handling"

  - objective: "Evaluate architectural tradeoffs between Pydantic, TypedDict, and dataclasses"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Student articulates reasoning for design choices in project documentation"

  - objective: "Justify design decisions with reference to production requirements"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Project README documents why each component was designed that way"

cognitive_load:
  new_concepts: 10
  assessment: "10 new concepts at B2 limit: Config Manager Pattern, Multi-Source Loading, Precedence Rules, Nested Config Models, Type-Safe Access, Graceful Defaults, Validation at Boundaries, Environment-Specific Overrides, Testing Config Systems, Production Patterns ✓"

differentiation:
  extension_for_advanced: "Implement secrets management (AWS Secrets Manager integration); add hot-reload capabilities with watchdog; design remote config sources (S3, etcd); create metrics and observability"
  remedial_for_struggling: "Start with Section 1-2 only (models and YAML loading); use AI to scaffold the architecture before diving into multi-source implementation; focus on getting basic version working first"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-part-4-chapter-27/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 6: Capstone Project - Type-Safe Configuration Manager

## The Configuration Problem in Production

Every real application needs configuration. When you deploy to production, you need different database credentials than your local development environment. Your API timeout settings change. Your logging level shifts from DEBUG to INFO. These values **cannot** be hardcoded in your source code—they belong in configuration files, environment variables, or command-line arguments.

But here's where it gets tricky: **configuration is fragile**. A typo in an environment variable name silently uses a default value instead of failing. Missing required settings crash the app hours into production rather than at startup. Different environments have different precedence rules, confusing developers about where values come from. Without type safety, you don't discover missing fields until runtime.

This is the capstone project: you'll build a **production-quality ConfigManager** that:

- Loads configuration from multiple sources (YAML files, environment variables, CLI arguments)
- Enforces type safety with Pydantic and Generics
- Implements clear precedence rules (CLI overrides environment, environment overrides files)
- Validates configuration on startup, failing fast if anything is wrong
- Provides helpful error messages so developers know exactly what's misconfigured
- Includes comprehensive tests proving it works in all scenarios

By the end, you'll have a portfolio-worthy project demonstrating mastery of Pydantic, Generics, and production engineering practices—something you can show in technical interviews or include on GitHub.

---

## Section 1: Requirements and Architecture

Before writing a single line of code, let's clarify what a production config system needs.

### Functional Requirements (What it does)

Your ConfigManager must:

1. **Load from YAML files** — Read `config.yaml`, `dev.yaml`, or `prod.yaml` and parse structured data
2. **Load from environment variables** — Allow overrides via `APP_DATABASE_HOST`, `APP_LOG_LEVEL`, etc.
3. **Load from CLI arguments** — Accept `--debug` or `--log-level=DEBUG` to override everything else
4. **Merge with precedence** — CLI args win over env vars, which win over file values, which win over defaults
5. **Validate everything** — Ensure types, required fields, and constraints are satisfied

### Non-Functional Requirements (How it must work)

1. **Type-safe access** — Use Generics so `config.get[DatabaseConfig]("database")` returns a typed object with full IDE autocomplete
2. **Fail fast** — If config is invalid, crash at startup with a clear error, not 3 hours into production
3. **Testable** — Unit tests can verify each loading strategy independently
4. **Documented** — A user reading the code understands why each piece exists
5. **Secure** — Never log passwords; handle secrets safely

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    ConfigManager                             │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  1. ConfigLoader                                             │
│     ├─ load_yaml() → dict                                    │
│     ├─ load_env() → dict                                     │
│     └─ load_cli() → dict                                     │
│                                                               │
│  2. Merge with Precedence                                   │
│     ├─ defaults (BaseModel field defaults)                  │
│     ├─ + YAML file values                                   │
│     ├─ + Environment variable values                        │
│     └─ + CLI argument values (highest priority)             │
│                                                               │
│  3. Pydantic Validation                                      │
│     └─ Validate merged dict against AppConfig model         │
│                                                               │
│  4. Generic[T] Wrapper                                       │
│     ├─ Type-safe access: config.get[DatabaseConfig]("db")   │
│     └─ IDE autocomplete on DatabaseConfig fields            │
│                                                               │
│  5. Return Validated AppConfig                               │
│     └─ App uses with confidence: no more type errors        │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

#### 💬 AI Colearning Prompt

> Ask your AI: "Compare Pydantic BaseSettings vs manually reading environment variables. What are the tradeoffs of each approach?"

**Expected Outcome**: Explanation showing that manual reading is repetitive and error-prone, while BaseSettings automates the process but requires understanding configuration structure.

---

### Design Decisions: Why This Architecture?

**Why Pydantic?** Type hints alone don't enforce constraints. Pydantic validates at runtime, ensuring your configuration is actually correct before your app tries to use it.

**Why BaseSettings?** It automates the common pattern of "load from env vars with a prefix." Without it, you'd manually check `os.environ.get("APP_DATABASE_HOST")` for every single field.

**Why Generic[T] wrapper?** When you write `config.get("database")`, Python doesn't know what type you're getting back—is it a dict? A DatabaseConfig object? The Generic wrapper lets you specify the return type: `config.get[DatabaseConfig]("database")`, and your IDE gives you perfect autocomplete on all DatabaseConfig fields.

#### 🎓 Instructor Commentary

> In production, configuration errors are some of the most frustrating bugs to debug. A misconfigured database connection string silently causes connection timeouts 3 hours into production. You don't want to learn this lesson the hard way. **Config management is unglamorous but essential—this pattern will save you hours on every project.**

---

## Section 2: Defining Config Models

Let's build the nested Pydantic models that describe your application's configuration.

### Creating the DatabaseConfig Model

```python
from pydantic import BaseModel, Field

class DatabaseConfig(BaseModel):
    """Database connection configuration."""

    host: str = "localhost"
    port: int = 5432
    name: str  # Required field—no default
    user: str  # Required field
    password: str = Field(
        default="",
        repr=False  # Security: don't show password in repr()
    )

    class Config:
        """Tell Pydantic to validate environment variables."""
        env_prefix = "APP_DATABASE_"
```

The `env_prefix` means environment variables like `APP_DATABASE_HOST` automatically map to the `host` field. This eliminates manual string matching and reduces typos.

### Creating the APIConfig Model

```python
class APIConfig(BaseModel):
    """External API configuration."""

    base_url: str
    timeout: int = 30  # Seconds, with sensible default
    retry_count: int = 3

    class Config:
        env_prefix = "APP_API_"
```

### Creating the Top-Level AppConfig Model

```python
from pydantic_settings import BaseSettings

class AppConfig(BaseSettings):
    """Complete application configuration."""

    debug: bool = False
    log_level: str = "INFO"
    database: DatabaseConfig
    api: APIConfig

    class Config:
        # Load from .env file if it exists
        env_file = ".env"
        env_prefix = "APP_"
        env_nested_delimiter = "__"  # APP_DATABASE__HOST maps to database.host
```

The `env_nested_delimiter` is key: it lets you set nested values from environment variables. `APP_DATABASE__HOST=prod-db.example.com` sets the database's host field without repeating the full path.

#### 🚀 CoLearning Challenge

Tell your AI: "Scaffold the three config models (DatabaseConfig, APIConfig, AppConfig) with realistic defaults and validation constraints. Add validation to ensure port is 1-65535, timeout is > 0, and log_level is one of 'DEBUG', 'INFO', 'WARNING', 'ERROR'."

**Expected Outcome**: Three complete models with Field() constraints and field validators, ready for the next section.

---

## Section 3: Multi-Source Loading

Now implement the ConfigLoader that reads from all three sources and merges them with proper precedence.

### Loading from YAML Files

```python
import yaml
from pathlib import Path
from typing import Any

def load_yaml_config(filepath: str) -> dict[str, Any]:
    """Load configuration from a YAML file."""
    config_path = Path(filepath)

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {filepath}")

    with open(config_path) as f:
        return yaml.safe_load(f) or {}
```

### Loading from Environment Variables

```python
import os
from typing import Any

def load_env_config() -> dict[str, Any]:
    """Load configuration from environment variables with APP_ prefix."""
    result: dict[str, Any] = {}

    for key, value in os.environ.items():
        if not key.startswith("APP_"):
            continue

        # APP_DEBUG=true → debug: true
        # APP_DATABASE__HOST=localhost → database.host: localhost
        config_key = key[4:].lower()  # Remove "APP_" prefix

        if "__" in config_key:
            # Handle nested keys: DATABASE__HOST → database.host
            parts = config_key.split("__")
            current = result
            for part in parts[:-1]:
                if part not in current:
                    current[part] = {}
                current = current[part]
            current[parts[-1]] = _parse_value(value)
        else:
            result[config_key] = _parse_value(value)

    return result

def _parse_value(value: str) -> Any:
    """Parse environment variable strings to Python types."""
    if value.lower() in ("true", "false"):
        return value.lower() == "true"

    if value.isdigit():
        return int(value)

    return value
```

#### ✨ Teaching Tip

> **Use BaseSettings' env_prefix to namespace environment variables:** `APP_DATABASE_HOST=localhost` is clearer than `DATABASE_HOST`. The `APP_` prefix prevents collisions with unrelated environment variables on your system.

### Loading from CLI Arguments

```python
import argparse
from typing import Any

def load_cli_config() -> dict[str, Any]:
    """Load configuration from command-line arguments."""
    parser = argparse.ArgumentParser()

    # Top-level arguments
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument("--log-level", default=None, help="Logging level: DEBUG, INFO, WARNING, ERROR")

    # Database arguments
    parser.add_argument("--database-host", help="Database host")
    parser.add_argument("--database-port", type=int, help="Database port")
    parser.add_argument("--database-name", help="Database name")
    parser.add_argument("--database-user", help="Database user")
    parser.add_argument("--database-password", help="Database password")

    # API arguments
    parser.add_argument("--api-base-url", help="API base URL")
    parser.add_argument("--api-timeout", type=int, help="API timeout (seconds)")
    parser.add_argument("--api-retry-count", type=int, help="Number of retries")

    args = parser.parse_args()

    # Convert flat CLI args to nested dict matching AppConfig structure
    result: dict[str, Any] = {}

    if args.debug:
        result["debug"] = True
    if args.log_level:
        result["log_level"] = args.log_level

    # Build nested database section
    if any([args.database_host, args.database_port, args.database_name,
            args.database_user, args.database_password]):
        result["database"] = {}
        if args.database_host:
            result["database"]["host"] = args.database_host
        if args.database_port:
            result["database"]["port"] = args.database_port
        if args.database_name:
            result["database"]["name"] = args.database_name
        if args.database_user:
            result["database"]["user"] = args.database_user
        if args.database_password:
            result["database"]["password"] = args.database_password

    # Similar for API section...

    return result
```

### Merging with Precedence

```python
from functools import reduce
from typing import Any

def merge_configs(*configs: dict[str, Any]) -> dict[str, Any]:
    """
    Merge configuration dictionaries with precedence.
    Later arguments override earlier ones.
    """
    def merge_dict(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
        result = base.copy()
        for key, value in override.items():
            if isinstance(value, dict) and key in result and isinstance(result[key], dict):
                result[key] = merge_dict(result[key], value)
            else:
                result[key] = value
        return result

    return reduce(merge_dict, configs, {})
```

### The Complete load_config() Function

```python
def load_config(yaml_file: str = "config.yaml") -> AppConfig:
    """
    Load configuration from all sources with precedence:
    1. Defaults (from AppConfig field defaults)
    2. YAML file (config.yaml or prod.yaml)
    3. Environment variables (APP_*)
    4. CLI arguments (--flag)

    Returns validated AppConfig instance.
    """
    # Load from each source
    try:
        yaml_config = load_yaml_config(yaml_file)
    except FileNotFoundError:
        yaml_config = {}

    env_config = load_env_config()
    cli_config = load_cli_config()

    # Merge with precedence: later overrides earlier
    merged = merge_configs(yaml_config, env_config, cli_config)

    # Validate with Pydantic
    try:
        return AppConfig(**merged)
    except ValidationError as e:
        # Provide helpful error message
        print("Configuration validation failed:")
        for error in e.errors():
            print(f"  - {error['loc']}: {error['msg']}")
        raise
```

---

## Section 4: Generic Type-Safe Access

Now we add the `ConfigValue[T]` wrapper that provides type-safe configuration access with IDE autocomplete.

### Why Type-Safe Access Matters

Without Generics, when you retrieve a config subsection, Python doesn't know its type:

```python
# ❌ Without Generics: type is lost
config = load_config()
db = config.database  # IDE: what type is this? Dict? DatabaseConfig?
```

With Generics, you make the type explicit:

```python
# ✅ With Generics: type is crystal clear
config = load_config()
db: DatabaseConfig = config.get[DatabaseConfig]("database")  # IDE knows exactly what this is
print(db.host)  # IDE autocomplete works perfectly
```

### Implementing ConfigValue[T]

```python
from typing import Generic, TypeVar

T = TypeVar('T')  # Generic type parameter

class ConfigValue(Generic[T]):
    """Type-safe wrapper for configuration values."""

    def __init__(self, value: T) -> None:
        """Initialize with a typed value."""
        self._value = value

    def get(self) -> T:
        """Retrieve the typed value."""
        return self._value

    def __repr__(self) -> str:
        """String representation (useful for debugging)."""
        return f"ConfigValue({self._value!r})"
```

### Adding get() Method to AppConfig

```python
class AppConfig(BaseSettings):
    """Complete application configuration."""

    debug: bool = False
    log_level: str = "INFO"
    database: DatabaseConfig
    api: APIConfig

    def get(self, key: str) -> Any:
        """Retrieve a configuration value by key."""
        if not hasattr(self, key):
            raise KeyError(f"Configuration has no key: {key}")
        return getattr(self, key)
```

### Using Type-Safe Access

```python
# Load configuration
config = load_config()

# Type-safe access with full IDE autocomplete
db_config: DatabaseConfig = config.database
print(f"Connecting to {db_config.host}:{db_config.port}")

api_config: APIConfig = config.api
print(f"API timeout: {api_config.timeout} seconds")

# Using ConfigValue wrapper (if you prefer explicit typing)
db = ConfigValue[DatabaseConfig](config.database)
actual_db: DatabaseConfig = db.get()
```

#### 💬 AI Colearning Prompt

> Ask your AI: "Why is `config.get[DatabaseConfig]('database')` better than `config['database']`? Show the type safety difference and explain how IDE autocomplete works with Generics."

**Expected Outcome**: Clear explanation that Generics provide compile-time type checking and IDE assistance, preventing AttributeError bugs that would otherwise surface at runtime.

---

## Section 5: Error Handling and Validation

Production systems must fail gracefully. Configuration errors should crash at startup with clear messages, not 3 hours into production.

### Validating Required Fields

```python
from pydantic import ValidationError

def load_config_safe(yaml_file: str = "config.yaml") -> AppConfig:
    """Load configuration with detailed error reporting."""

    # Load from all sources
    yaml_config = load_yaml_config(yaml_file) if Path(yaml_file).exists() else {}
    env_config = load_env_config()
    cli_config = load_cli_config()

    merged = merge_configs(yaml_config, env_config, cli_config)

    # Validate and provide helpful errors
    try:
        return AppConfig(**merged)

    except ValidationError as e:
        print("\n" + "="*60)
        print("CONFIGURATION ERROR - Cannot start application")
        print("="*60 + "\n")

        for error in e.errors():
            field_path = ".".join(str(x) for x in error["loc"])
            error_type = error["type"]
            message = error["msg"]

            print(f"Field: {field_path}")
            print(f"  Error: {message}")
            print(f"  Type: {error_type}\n")

        print("Configuration sources (in order of precedence):")
        print(f"  1. Defaults (from config.py)")
        print(f"  2. YAML file: {yaml_file}")
        print(f"  3. Environment variables (APP_*)")
        print(f"  4. CLI arguments (--flag)")

        raise
```

### Logging Configuration Sources

```python
import logging

logger = logging.getLogger(__name__)

def load_config_with_logging(yaml_file: str = "config.yaml") -> AppConfig:
    """Load configuration and log what sources were used."""

    yaml_config = load_yaml_config(yaml_file) if Path(yaml_file).exists() else {}
    env_config = load_env_config()
    cli_config = load_cli_config()

    if yaml_config:
        logger.info(f"Loaded YAML config from {yaml_file}")
    if env_config:
        logger.debug(f"Loaded environment variables: {list(env_config.keys())}")
    if cli_config:
        logger.debug(f"Loaded CLI arguments: {list(cli_config.keys())}")

    merged = merge_configs(yaml_config, env_config, cli_config)
    config = AppConfig(**merged)

    # Log final configuration (without secrets)
    logger.info(f"Debug mode: {config.debug}")
    logger.info(f"Log level: {config.log_level}")
    logger.info(f"Database: {config.database.host}:{config.database.port}/{config.database.name}")
    logger.info(f"API timeout: {config.api.timeout}s")

    return config
```

#### 🎓 Instructor Commentary

> **Config errors should crash the app at startup, not 3 hours into production.** This is why we validate immediately and provide helpful error messages. In production, your app should refuse to start if the database password is missing or the API timeout is invalid. This "fail fast" philosophy is essential to professional engineering.

---

## Section 6: Testing

A production system needs comprehensive tests. You can't deploy configuration code to production without proving it handles all scenarios.

### Test Setup with Temporary Files

```python
import pytest
import tempfile
import os
from pathlib import Path

@pytest.fixture
def temp_yaml_config():
    """Create a temporary YAML config file for testing."""
    yaml_content = """
debug: false
log_level: INFO
database:
  host: localhost
  port: 5432
  name: testdb
  user: testuser
  password: testpass
api:
  base_url: https://api.example.com
  timeout: 30
  retry_count: 3
"""

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(yaml_content)
        f.flush()
        yield f.name

    # Cleanup
    os.unlink(f.name)
```

### Testing YAML Loading

```python
def test_load_yaml_config(temp_yaml_config):
    """Test that YAML files are loaded correctly."""
    config = load_config(yaml_file=temp_yaml_config)

    assert config.debug is False
    assert config.log_level == "INFO"
    assert config.database.host == "localhost"
    assert config.database.port == 5432
    assert config.database.name == "testdb"
    assert config.api.timeout == 30
```

### Testing Environment Variable Overrides

```python
def test_env_override(temp_yaml_config, monkeypatch):
    """Test that environment variables override YAML values."""
    # Set environment variable
    monkeypatch.setenv("APP_DEBUG", "true")
    monkeypatch.setenv("APP_DATABASE__HOST", "prod-db.example.com")

    config = load_config(yaml_file=temp_yaml_config)

    # Environment variables override YAML
    assert config.debug is True
    assert config.database.host == "prod-db.example.com"

    # Other values come from YAML
    assert config.database.port == 5432
```

### Testing Precedence Rules

```python
def test_cli_overrides_all(temp_yaml_config, monkeypatch):
    """Test that CLI arguments have highest precedence."""
    # Set environment variable
    monkeypatch.setenv("APP_LOG_LEVEL", "WARNING")

    # Set CLI argument (simulated)
    monkeypatch.setattr("sys.argv", [
        "app.py",
        "--log-level=ERROR",
        "--api-timeout=60"
    ])

    config = load_config(yaml_file=temp_yaml_config)

    # CLI wins over environment
    assert config.log_level == "ERROR"

    # CLI override of API timeout
    assert config.api.timeout == 60
```

#### ✨ Teaching Tip

> **Use pytest fixtures to create temporary config files for tests.** The `@pytest.fixture` decorator lets you create test resources (files, data, connections) that are automatically cleaned up afterward. This keeps your tests isolated and prevents leftover files from polluting your system.

### Testing Validation Errors

```python
def test_missing_required_field():
    """Test that missing required fields produce validation errors."""
    invalid_config = {
        "debug": False,
        "log_level": "INFO",
        "database": {
            "host": "localhost",
            "port": 5432,
            # Missing "name" and "user" fields!
        },
        "api": {
            "base_url": "https://api.example.com"
        }
    }

    with pytest.raises(ValidationError) as exc_info:
        AppConfig(**invalid_config)

    # Verify error messages are helpful
    errors = exc_info.value.errors()
    assert any("database" in str(e["loc"]) for e in errors)
```

---

## Section 7: Project Deliverables

Your capstone project should include all of these components:

### Project Structure

```
config-manager/
├── config_manager/
│   ├── __init__.py
│   ├── models.py           # DatabaseConfig, APIConfig, AppConfig
│   ├── loader.py           # load_yaml, load_env, load_cli, merge_configs
│   ├── manager.py          # ConfigManager class with get[T]() method
│   └── exceptions.py       # Custom exceptions
├── configs/
│   ├── dev.yaml           # Development configuration
│   ├── prod.yaml          # Production configuration
│   └── .env.example       # Example environment variables
├── tests/
│   ├── conftest.py        # Pytest fixtures
│   ├── test_yaml_loading.py
│   ├── test_env_loading.py
│   ├── test_precedence.py
│   ├── test_validation.py
│   └── test_integration.py
├── example_app.py         # Demo application using ConfigManager
├── README.md              # Project documentation
├── requirements.txt       # Dependencies (pydantic, pyyaml)
└── pytest.ini            # Pytest configuration
```

### Example Configuration Files

**configs/dev.yaml**:

```yaml
debug: true
log_level: DEBUG
database:
  host: localhost
  port: 5432
  name: myapp_dev
  user: dev_user
  password: dev_password
api:
  base_url: http://localhost:8000
  timeout: 5
  retry_count: 1
```

**configs/prod.yaml**:

```yaml
debug: false
log_level: INFO
database:
  host: prod-db.example.com
  port: 5432
  name: myapp_prod
  user: prod_user
  password: ${DB_PASSWORD}  # Load from env
api:
  base_url: https://api.example.com
  timeout: 30
  retry_count: 3
```

### Example Application

```python
# example_app.py
import logging
from config_manager.manager import ConfigManager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    """Example application using ConfigManager."""

    # Load configuration
    config = ConfigManager.load(yaml_file="configs/dev.yaml")

    # Set up logging based on config
    logging.getLogger().setLevel(config.log_level)

    logger.info("Starting application")
    logger.info(f"Debug mode: {config.debug}")

    # Access database configuration with type safety
    db_config = config.database
    logger.info(f"Connecting to {db_config.host}:{db_config.port}/{db_config.name}")

    # Access API configuration with type safety
    api_config = config.api
    logger.info(f"API base URL: {api_config.base_url} (timeout: {api_config.timeout}s)")

    logger.info("Application running successfully")

if __name__ == "__main__":
    main()
```

### Test Coverage

Run tests with:

```bash
pytest tests/ -v --cov=config_manager
```

Aim for 90%+ test coverage of your ConfigManager code.

#### 🚀 CoLearning Challenge

Tell your AI: "Build a complete, production-ready ConfigManager project. Include: nested config models (DatabaseConfig, APIConfig, AppConfig), YAML + environment + CLI loading with precedence, validation with helpful errors, comprehensive tests, and a demo application. Make it portfolio-worthy—something you'd submit with a GitHub portfolio."

**Expected Outcome**: Complete project with all components working together, tests passing, and a README explaining the architecture.

---

## Common Mistakes to Avoid

### Mistake 1: Not Validating at Startup

```python
# ❌ WRONG: Missing required field isn't caught until runtime
def get_database_password():
    return os.environ.get("DATABASE_PASSWORD")  # Returns None if missing!

password = get_database_password()
# Crashes hours later when you try to use the connection
```

```python
# ✅ RIGHT: Validate at startup, fail immediately
class DatabaseConfig(BaseModel):
    password: str  # Required field—no default

config = load_config()  # Raises ValidationError if password missing
```

### Mistake 2: Hardcoding Defaults in Code

```python
# ❌ WRONG: Change requires redeployment
def connect_to_api(timeout=30):
    requests.get(..., timeout=timeout)
```

```python
# ✅ RIGHT: Defaults in config files, easily overridable
class APIConfig(BaseModel):
    timeout: int = 30  # Default value, but overridable via env/CLI
```

### Mistake 3: Not Documenting Precedence

```python
# ❌ WRONG: Developer doesn't know why their value isn't being used
config = load_from_env()  # Oops, ignoring YAML file!
```

```python
# ✅ RIGHT: Clear precedence documented in code and README
"""
Load configuration with precedence (later wins):
1. Defaults (AppConfig field defaults)
2. YAML file (config.yaml)
3. Environment variables (APP_*)
4. CLI arguments (--flag)
"""
```

### Mistake 4: Overcomplicating the System

```python
# ❌ WRONG: Too many config sources creates confusion
configs = [
    load_from_yaml(),
    load_from_env(),
    load_from_cli(),
    load_from_consul(),  # Remote configuration!
    load_from_vault(),   # Secrets!
    load_from_redis(),   # Cache!
]
```

**Lesson**: Start simple. Add remote configs and secrets management only when you actually need them (that's your extension activity for B2+ students).

---

## Try With AI

This capstone synthesizes everything from Chapter 27. Use your AI companion to explore each prompt deeply.

### Prompt 1: Understand the Architecture

**Tell your AI**:

> "Explain the architecture of a production configuration system. Why do we need multiple sources (file, env, CLI) and precedence rules? What real-world scenarios would require each?"

**Expected Outcome**:

A clear explanation covering:
- Why YAML files are good for development (easy to edit, version control)
- Why environment variables are good for production (easy to deploy, secure)
- Why CLI arguments are good for debugging (quick overrides without redeploying)
- Concrete use cases (local dev uses `config.yaml`, production uses env vars, debugging uses `--database-host=localhost`)

### Prompt 2: Build and Integrate

**Tell your AI**:

> "Implement a complete ConfigManager with DatabaseConfig, APIConfig, and AppConfig models. Show loading from YAML file with environment variable and CLI argument overrides. Include error handling for missing required fields and logging of configuration sources."

**Expected Outcome**:

Working code with:
- All three models with realistic fields and defaults
- `load_config()` function that merges sources with precedence
- Pydantic validation catching missing fields
- Helpful error messages
- Logging showing which config sources were used

### Prompt 3: Compare Approaches

**Ask your AI**:

> "Compare three approaches to configuration management: 1) Pydantic BaseSettings, 2) TypedDict, 3) dataclasses. What are the tradeoffs? When would you use each? Which is best for production systems?"

**Expected Outcome**:

A comparison table showing:
- Pydantic BaseSettings: Validation ✓, Environment integration ✓, Production-ready ✓ (best choice)
- TypedDict: Simpler syntax, no validation ✗, good for type hints only
- Dataclasses: Clean syntax, optional validation, no env integration ✗

With recommendations for different use cases.

### Prompt 4: Build Production-Ready Project

**Tell your AI**:

> "Build a complete, production-ready ConfigManager project. Include: nested config models with validation constraints, YAML + environment + CLI loading with clear precedence handling, comprehensive error messages, unit and integration tests (pytest), example configurations for dev and prod, a working demo app, and a README explaining the architecture and how to use it. Make this portfolio-worthy—something you'd show in technical interviews."

**Expected Outcome**:

A complete, deployable project with:
- All source code organized in proper modules
- Dev config (debug=true) and prod config (debug=false)
- Unit tests proving each loading strategy works
- Integration tests proving precedence works correctly
- Demo app showing real-world usage
- README with architecture diagram and usage examples
- All code following Python best practices (type hints, docstrings, error handling)