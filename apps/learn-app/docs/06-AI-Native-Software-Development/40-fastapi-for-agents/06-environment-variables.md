---
sidebar_position: 6
title: "Environment Variables"
description: "Configure your API securely using environment variables and pydantic-settings"
keywords: [environment-variables, pydantic-settings, dotenv, configuration, secrets]
chapter: 40
lesson: 6
duration_minutes: 40

skills:
  - name: "Environment Variable Configuration"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student creates Settings class with pydantic-settings"

  - name: "Secret Management"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "4.2 Protecting Personal Data"
    measurable_at_this_level: "Student explains why secrets don't belong in code"

  - name: "dotenv File Usage"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student creates and uses .env files with .gitignore"

learning_objectives:
  - objective: "Create a Settings class using pydantic-settings"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Settings class loads from environment variables"

  - objective: "Use .env files for local development"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: ".env file exists and is gitignored"

  - objective: "Inject settings as a FastAPI dependency"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Endpoints receive settings via Depends()"

cognitive_load:
  new_concepts: 5
  assessment: "BaseSettings, .env files, env_file config, .gitignore, lru_cache"

differentiation:
  extension_for_advanced: "Add environment-specific settings (dev/prod) and validation"
  remedial_for_struggling: "Start with single hardcoded value before adding settings"
---

# Environment Variables

Your API needs configuration: database URLs, API keys, secret tokens. Hardcoding these values is a security risk and makes deployment painful. Environment variables solve this—they let you configure your app differently in development, staging, and production without changing code.

This lesson teaches the pattern you'll use throughout the chapter. Every lesson from here forward uses settings from environment variables.

## Why Environment Variables Matter

**The problem with hardcoding:**

```python
# NEVER DO THIS
database_url = "postgresql://admin:password123@db.example.com/prod"
api_key = "sk-secret-key-that-leaks-to-github"
```

If this code gets committed:
- Secrets leak to version control
- You can't use different databases in dev vs prod
- Rotating secrets requires code changes and redeployment

**The environment variable solution:**

```python
import os
database_url = os.getenv("DATABASE_URL")
api_key = os.getenv("API_KEY")
```

Now configuration lives outside your code. Different environments set different values.

## pydantic-settings: Type-Safe Configuration

Raw `os.getenv()` works but has problems:
- Returns strings only (need to convert `"30"` to `30`)
- No validation (missing variables fail silently)
- No documentation of required variables

pydantic-settings fixes all of this:

```bash
uv add pydantic-settings
```

```python
# config.py
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application configuration from environment variables."""

    database_url: str
    api_key: str
    debug: bool = False
    max_connections: int = 10
```

**What this gives you:**

- **Type conversion**: `"10"` becomes `10`, `"true"` becomes `True`
- **Validation**: Missing required variables raise clear errors
- **Defaults**: Optional variables have default values
- **Documentation**: The class itself documents what's configurable

## Using Settings in Your App

```python
# main.py
from fastapi import FastAPI, Depends
from config import Settings

app = FastAPI()


def get_settings() -> Settings:
    return Settings()


@app.get("/config")
def show_config(settings: Settings = Depends(get_settings)):
    return {
        "debug": settings.debug,
        "max_connections": settings.max_connections
    }
```

**Output (with DEBUG=true set):**
```json
{
  "debug": true,
  "max_connections": 10
}
```

## The .env File

Typing `export DATABASE_URL=...` every time is tedious. Use a `.env` file:

```bash
# .env
DATABASE_URL=postgresql://localhost/devdb
API_KEY=dev-key-not-for-production
DEBUG=true
MAX_CONNECTIONS=5
```

Tell pydantic-settings to load it:

```python
class Settings(BaseSettings):
    database_url: str
    api_key: str
    debug: bool = False
    max_connections: int = 10

    class Config:
        env_file = ".env"
```

Now `Settings()` reads from `.env` automatically.

## Critical: Gitignore Your Secrets

**Never commit .env files with real secrets.**

Create `.gitignore`:

```
# .gitignore
.env
*.env
.env.*
!.env.example
```

Create `.env.example` (this one IS committed):

```bash
# .env.example - Copy to .env and fill in values
DATABASE_URL=postgresql://user:pass@host/database
API_KEY=your-api-key-here
DEBUG=false
MAX_CONNECTIONS=10
```

This pattern:
1. Documents what variables are needed
2. Keeps actual secrets out of version control
3. Makes onboarding new developers easy

## Caching Settings

Creating `Settings()` reads from disk each time. Cache it:

```python
from functools import lru_cache


@lru_cache
def get_settings() -> Settings:
    return Settings()
```

Now settings load once and reuse the same instance. This is more efficient and ensures consistency.

## Complete Settings Example

Here's the pattern you'll use throughout this chapter:

```python
# config.py
from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment."""

    # Database
    database_url: str

    # Authentication
    secret_key: str
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # API Keys
    anthropic_api_key: str

    # Development
    debug: bool = False

    class Config:
        env_file = ".env"


@lru_cache
def get_settings() -> Settings:
    """Cached settings instance."""
    return Settings()
```

```python
# main.py
from fastapi import FastAPI, Depends
from config import Settings, get_settings

app = FastAPI()


@app.get("/health")
def health_check(settings: Settings = Depends(get_settings)):
    return {
        "status": "healthy",
        "debug_mode": settings.debug
    }
```

**Output:**
```json
{
  "status": "healthy",
  "debug_mode": false
}
```

## Validation Errors

What happens with missing or invalid values?

```python
# .env is empty or missing DATABASE_URL
settings = Settings()
```

**Output:**
```
pydantic_settings.sources.SettingsError: error loading settings
  database_url
    Field required [type=missing]
```

Clear error message telling you exactly what's missing.

```python
# .env has MAX_CONNECTIONS=not_a_number
settings = Settings()
```

**Output:**
```
pydantic_settings.sources.SettingsError: error loading settings
  max_connections
    Input should be a valid integer [type=int_parsing]
```

These errors happen at startup, not during a request. Fail fast.

## Hands-On Exercise

Set up configuration for your Task API:

**Step 1:** Install pydantic-settings:

```bash
uv add pydantic-settings
```

**Step 2:** Create `config.py`:

```python
from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    app_name: str = "Task API"
    debug: bool = False
    max_tasks_per_user: int = 100

    class Config:
        env_file = ".env"


@lru_cache
def get_settings() -> Settings:
    return Settings()
```

**Step 3:** Create `.env`:

```bash
APP_NAME=My Task API
DEBUG=true
MAX_TASKS_PER_USER=50
```

**Step 4:** Create `.gitignore`:

```
.env
```

**Step 5:** Create `.env.example`:

```bash
APP_NAME=Task API
DEBUG=false
MAX_TASKS_PER_USER=100
```

**Step 6:** Use settings in your app:

```python
from fastapi import FastAPI, Depends
from config import Settings, get_settings

app = FastAPI()


@app.get("/info")
def app_info(settings: Settings = Depends(get_settings)):
    return {
        "app_name": settings.app_name,
        "debug": settings.debug,
        "max_tasks": settings.max_tasks_per_user
    }
```

**Step 7:** Test it:

```bash
curl http://localhost:8000/info
```

**Output:**
```json
{
  "app_name": "My Task API",
  "debug": true,
  "max_tasks": 50
}
```

## Common Mistakes

**Mistake 1:** Committing .env with secrets

```bash
# Check if .env is tracked
git status

# If it shows .env, remove it from tracking
git rm --cached .env
echo ".env" >> .gitignore
git commit -m "Remove .env from tracking"
```

**Mistake 2:** Wrong variable names

```python
class Settings(BaseSettings):
    database_url: str  # Expects DATABASE_URL in environment
```

Environment variables are case-insensitive but conventionally UPPER_CASE. pydantic-settings converts `database_url` to look for `DATABASE_URL`.

**Mistake 3:** Forgetting the Config class

```python
# Wrong - won't read from .env
class Settings(BaseSettings):
    database_url: str

# Correct
class Settings(BaseSettings):
    database_url: str

    class Config:
        env_file = ".env"
```

**Mistake 4:** Not caching settings

```python
# Wrong - reads file on every call
def get_settings():
    return Settings()

# Correct - reads once
@lru_cache
def get_settings():
    return Settings()
```

## Security Checklist

Before deploying any API:

- [ ] `.env` is in `.gitignore`
- [ ] `.env.example` documents required variables
- [ ] No secrets appear in code or comments
- [ ] Production uses different secrets than development
- [ ] Secret rotation doesn't require code changes

## Try With AI

After completing the exercise, explore these scenarios.

**Prompt 1: Environment-Specific Settings**

```text
I have Settings that work for development. How do I handle different
configurations for production? For example:
- Development: DEBUG=true, local database
- Production: DEBUG=false, remote database

Should I use multiple .env files or conditional logic in Settings?
```

**What you're learning:** Real deployments need environment-specific configuration. There are several patterns—AI can explain tradeoffs.

**Prompt 2: Validating Settings**

```text
I want to validate that my DATABASE_URL is a valid PostgreSQL
connection string, not just any string. Can pydantic-settings
validate the format of environment variables?
```

**What you're learning:** pydantic validators work with settings too. You can enforce URL formats, string patterns, and more.

**Prompt 3: Secrets in Production**

```text
In development, I use .env files. In production on Railway or
Fly.io, how do I set environment variables? What's the best
practice for managing production secrets?
```

**What you're learning:** .env is for local development. Production platforms have their own secrets management—understanding this completes the picture.
