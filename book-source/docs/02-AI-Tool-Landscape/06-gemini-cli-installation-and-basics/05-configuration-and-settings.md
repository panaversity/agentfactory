---
sidebar_position: 5
title: Configuration & Settings
cefr_level: A2
proficiency: Beginner
teaching_stage: 2
stage_name: "AI Collaboration"
cognitive_load:
  concepts_count: 5
  a2_compliant: true
  scaffolding_level: "Heavy"
learning_objectives:
  - id: LO1
    description: "Explain the 7-level configuration hierarchy and precedence order"
    bloom_level: "Understand"
  - id: LO2
    description: "Create project-level .env file with environment variables"
    bloom_level: "Create"
  - id: LO3
    description: "Apply security best practices using .env, .gitignore, and .env.example pattern"
    bloom_level: "Apply"
  - id: LO4
    description: "Configure project-specific settings that override user-level defaults"
    bloom_level: "Apply"
  - id: LO5
    description: "Analyze configuration errors and determine which level is causing conflicts"
    bloom_level: "Analyze"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Evaluating data, information and digital content"
    proficiency_level: "Basic"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
    proficiency_level: "Basic"
  - objective_id: LO3
    competency_area: "4. Safety"
    competency: "4.2 Protecting personal data and privacy"
    proficiency_level: "Basic"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
    proficiency_level: "Basic"
  - objective_id: LO5
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
    proficiency_level: "Basic"
---

# Configuration & Settings

Imagine you're working on two projects with Gemini CLI:

**Your Personal Projects**: You prefer `gemini-2.5-flash` (fast responses) and dark theme.

**Your Team's Enterprise Project**: Team requires `gemini-2.5-pro` (high accuracy) and light theme (accessibility compliance).

You open Gemini CLI in the team project. What happens?

### WITHOUT Understanding Configuration

You create `~/.gemini/settings.json` with your preferences:
```json
{
  "theme": "dark",
  "model": "gemini-2.5-flash"
}
```

**Problem**: Now the team project uses dark theme + flash model. Your teammates complain: "Why is the output different?" "The theme doesn't match our standards!"

You manually change settings every time you switch projects. **Frustrating.**

---

### WITH Configuration Hierarchy

You keep your personal preferences in `~/.gemini/settings.json`:
```json
{
  "theme": "dark",
  "model": "gemini-2.5-flash"
}
```

Your team adds `team-project/.gemini/settings.json`:
```json
{
  "theme": "light",
  "model": "gemini-2.5-pro"
}
```

**What happens now:**

When you run `gemini` in **your personal projects**:
- Theme: `dark` ✅ (from user-level config)
- Model: `gemini-2.5-flash` ✅ (from user-level config)

When you run `gemini` in **team project**:
- Theme: `light` ✅ (project-level overrides user-level)
- Model: `gemini-2.5-pro` ✅ (project-level overrides user-level)

**Zero manual switching. Automatic per-project configuration.**

---

## The Security Problem

Now your team adds an MCP server to connect to the company database. Someone creates this config:

```json
{
  "mcpServers": {
    "database": {
      "command": "python",
      "args": ["-m", "db_mcp"],
      "env": {
        "DATABASE_URL": "postgresql://admin:secret123@prod-db:5432/company"
      }
    }
  }
}
```

They commit this to the repository.

**❌ DISASTER**: Database password is now in version control. Anyone with repo access has production database credentials.

### The Solution: Environment Variables

Update config to reference environment variable:
```json
{
  "mcpServers": {
    "database": {
      "command": "python",
      "args": ["-m", "db_mcp"],
      "env": {
        "DATABASE_URL": "${DATABASE_URL}"
      }
    }
  }
}
```

Each developer creates their own `.env` file (gitignored):
```bash
# Alice's .env (local dev database)
DATABASE_URL=postgresql://localhost:5432/dev_db

# Bob's .env (local dev database)
DATABASE_URL=postgresql://localhost:5433/dev_db

# Production .env (never committed)
DATABASE_URL=postgresql://admin:prod_pass@prod-db:5432/company
```

**✅ Security**: No secrets in version control
**✅ Flexibility**: Each environment has its own values
**✅ Safety**: Production secrets never leak

---

## What You'll Master

By the end of this lesson, you'll understand:

- **Configuration hierarchy**: How 7 levels merge (CLI flags → env vars → `.env` → project → workspace → user → system)
- **Project vs. personal settings**: When to use each level and how they override
- **Environment variables**: How to use `${VAR}` syntax to keep secrets out of version control
- **Security patterns**: `.env` + `.gitignore` + `.env.example` workflow
- **Real-world scenarios**: Team collaboration, multi-environment deployments, secret management

One of the biggest advantages of Gemini CLI being open source and command-line based is that you can configure it exactly how you need. Unlike web interfaces where settings are fixed, Gemini CLI offers a powerful configuration system that lets you customize behavior globally, by project, or even with environment-specific secrets.

---

## Part 1: The Configuration Hierarchy

Gemini CLI doesn't just read one configuration file. Instead, it checks **7 different levels** and merges them in order, with higher levels overriding lower ones.

### Visual Hierarchy: Precedence Flow

```
┌─────────────────────────────────────────────────────────────┐
│  1. CLI Flags (--model, --theme)          [HIGHEST PRIORITY]│
│     ↓ (overrides everything below)                          │
├─────────────────────────────────────────────────────────────┤
│  2. Environment Variables (GEMINI_MODEL=...)                 │
│     ↓ (session-specific temporary overrides)                │
├─────────────────────────────────────────────────────────────┤
│  3. .env File (project secrets)                              │
│     ↓ (project-level secrets, gitignored)                   │
├─────────────────────────────────────────────────────────────┤
│  4. Project Settings (.gemini/settings.json in project root) │
│     ↓ (team-shared configuration)                           │
├─────────────────────────────────────────────────────────────┤
│  5. Workspace Settings (.gemini/settings.json in workspace)  │
│     ↓ (multi-project shared settings)                       │
├─────────────────────────────────────────────────────────────┤
│  6. User Settings (~/.gemini/settings.json)                  │
│     ↓ (your personal defaults)                              │
├─────────────────────────────────────────────────────────────┤
│  7. System Defaults (built-in fallbacks)    [LOWEST PRIORITY]│
└─────────────────────────────────────────────────────────────┘
```

**Key Rule**: If a setting exists at a higher level, it completely overrides the same setting at a lower level.

### Detailed Level Reference

Here's the complete precedence order (highest to lowest priority):

| Level | Location | Scope | Use Case |
|-------|----------|-------|----------|
| 1 (Highest) | CLI flags | Command-specific | Override everything for one command |
| 2 | Environment variables | Session-specific | Temporary overrides (e.g., `export GEMINI_MODEL=gemini-2.5-pro`) |
| 3 | `.env` file | Project-specific | Project secrets and variables |
| 4 | Project settings | Project-specific | Project-level configuration |
| 5 | Workspace settings | Workspace-specific | Shared across multiple projects |
| 6 | User settings | User-specific | Your personal defaults |
| 7 (Lowest) | System settings | Machine-wide | System administrator settings |

**Key Rule**: If a setting exists at a higher level, it completely overrides the same setting at a lower level.

### Real-World Example

Imagine you have:
- **System level**: `model: gemini-2.5-pro`
- **User level**: `model: gemini-2.5-flash` (you prefer the faster model)
- **Project level**: `model: gemini-2.5-pro` (your team needs the more powerful model for this project)

When you run `gemini` in that project directory, it uses **`gemini-2.5-pro`** because the project level overrides the user level, which overrides the system level.

If you add a CLI flag: `gemini --model gemini-2.5-pro-128k`, that flag wins—it's the highest priority.

#### 💬 AI Colearning Prompt

> "Why does Gemini CLI use 7 configuration levels instead of just one global config file? What problems does this hierarchy solve for teams?"

---

## Part 2: File Locations and Structure

**Configuration files are stored in JSON format**:

```
~/.gemini/settings.json          (User level - your personal settings)
<workspace-root>/.gemini/settings.json    (Workspace level)
<project-root>/.gemini/settings.json      (Project level)
```

On Windows:
```
C:\Users\<username>\.gemini\settings.json
```

### Basic Configuration File Structure

```json
{
  "theme": "dark",
  "model": "gemini-2.5-pro",
  "auth": {
    "method": "google",
    "persistTokens": true
  },
  "checkpointing": {
    "enabled": true,
    "interval": 300000
  }
}
```

:::tip Verified Configuration
This JSON structure has been validated in Gemini CLI 0.15.0. All keys (`theme`, `model`, `auth`, `checkpointing`) are confirmed working configuration options.
:::

:::note MCP Server Configuration
You can also configure **MCP (Model Context Protocol) servers** in `settings.json` to connect Gemini CLI to external tools, databases, and APIs. We'll cover this in detail in the next lesson ([MCP Servers & Integration](./06-mcp-servers-and-integration.md)). For now, just know that `mcpServers` is a valid configuration section.
:::

The most important part for beginners: **You usually don't need to edit this file directly**. Configuration is often managed through environment variables and the `.env` file, which we'll cover next.

---

## Part 3: Environment Variables and .env Files

Environment variables are powerful because they're:
1. **Easy to share**: Commit `.env` to `.gitignore` to protect secrets, but keep the `.env` pattern in version control
2. **Language-agnostic**: Work with Python, Node.js, or any tool
3. **CI/CD friendly**: Easy to set in deployment pipelines

### Syntax in Configuration Files

Inside `settings.json`, you can reference environment variables using two syntaxes:

```json
{
  "mcpServers": {
    "myDatabase": {
      "command": "python",
      "args": ["-m", "database_mcp"],
      "env": {
        "DATABASE_URL": "${DB_CONNECTION_STRING}",
        "API_KEY": "$EXTERNAL_API_KEY"
      }
    }
  }
}
```

Both `${VAR_NAME}` (preferred, explicit) and `$VAR_NAME` (shorthand) work.

:::note Environment Variable Syntax
Both syntaxes are verified working: `${DB_URL}` and `$DB_URL`. The `${VAR}` syntax is recommended for clarity.
:::

### The .env File

Create a `.env` file in your project root:

```bash
# Gemini CLI Configuration
# These settings override defaults when you start Gemini CLI

# Model Selection
# Options: gemini-2.5-flash (faster), gemini-2.5-pro (more accurate)
GEMINI_MODEL=gemini-2.5-flash

# Theme Preference
# Options: dark, light, auto (matches system)
GEMINI_THEME=dark

# Auto-save your conversations every 5 minutes (in milliseconds)
GEMINI_CHECKPOINT_INTERVAL=300000
```

**Loading Order**:
1. Gemini CLI reads your `.env` file first
2. It merges with environment variables already set
3. Those override settings in `settings.json`
4. CLI flags override everything

### The CRITICAL Security Rule

**NEVER commit secrets to version control.**

Always:
1. Add `.env` to `.gitignore`:
```
# .gitignore
.env
.env.local
*.key
```

2. Create a `.env.example` file showing the structure:
```
# .env.example (safe to commit)
DB_CONNECTION_STRING=postgresql://localhost:5432/mydb
EXTERNAL_API_KEY=your_api_key_here
```

This way, team members know what variables are needed without exposing real secrets.

---

## Part 4: Common Configuration Parameters

Here are the settings you'll use most often:

:::tip All Parameters Verified
All configuration parameters shown below have been tested and verified in Gemini CLI 0.15.0.
:::

### Theme Selection
```json
{
  "theme": "dark"    // Options: "dark", "light", "auto"
}
```

### Model Selection
```json
{
  "model": "gemini-2.5-pro"    // Default model for sessions
}
```

### Auto-Save Sessions (Checkpointing)
```json
{
  "checkpointing": {
    "enabled": true,
    "interval": 300000    // Auto-save every 5 minutes (milliseconds)
  }
}
```

### Authentication Method
```json
{
  "auth": {
    "method": "google",    // Options: "google", "apiKey", "vertexAI"
    "persistTokens": true  // Keep you logged in across sessions
  }
}
```

### Custom API Endpoint (Advanced)
```json
{
  "apiEndpoint": "https://custom-gemini-api.example.com/v1"
}
```

---

## Part 5: Security Best Practices (Enhanced)

### The Three-File Security Pattern

Here's the industry-standard workflow for keeping secrets safe:

#### 1. `.env` File (NEVER COMMIT)
Contains your personal settings and any sensitive values. Add to `.gitignore` immediately.

```bash
# .env (GITIGNORED - your personal settings)
GEMINI_MODEL=gemini-2.5-pro
GEMINI_THEME=dark
GEMINI_API_KEY=your-actual-api-key-here
```

#### 2. `.env.example` File (SAFE TO COMMIT)
Template showing structure without real values. Commit this to version control.

```bash
# .env.example (SAFE TO COMMIT - no real values)
# Copy this file to .env and fill in your actual preferences

# Gemini CLI Configuration
GEMINI_MODEL=gemini-2.5-flash
GEMINI_THEME=dark
GEMINI_API_KEY=your_api_key_here
```

#### 3. `.gitignore` File (COMMIT THIS)
Prevents accidental commits of secrets.

```bash
# .gitignore

# Environment files with secrets
.env
.env.local
.env.production
.env.*.local

# API keys and credentials
*.key
*.pem
secrets/
credentials.json

# OS-specific files
.DS_Store
Thumbs.db

# IDE files
.vscode/
.idea/
```

### Complete Security Workflow

**Step 1**: Create `.gitignore` FIRST (before creating .env)
```bash
echo ".env" >> .gitignore
echo ".env.local" >> .gitignore
echo "*.key" >> .gitignore
```

**Step 2**: Create `.env.example` with structure only
```bash
# .env.example
GEMINI_MODEL=gemini-2.5-flash
GEMINI_THEME=dark
GEMINI_API_KEY=your_api_key_here
```

**Step 3**: Copy to `.env` and add your actual values
```bash
cp .env.example .env
# Edit .env with your preferences and API key (never commit this!)
```

### ✅ DO:
- Use environment variables for API keys, database URLs, and secrets
- Store sensitive values in `.env` files
- Add `.env` to `.gitignore` **before** creating the file
- Create `.env.example` with structure only (safe to commit)
- Rotate API keys periodically (every 90 days recommended)
- Use different API keys for development, staging, and production
- Review `.gitignore` before every commit

### ⚠️ DON'T:
- Hardcode secrets in `settings.json`
- Commit `.env` files to version control
- Use the same API key across multiple projects
- Leave API keys in shell history
- Share `.env` files via email or chat (use secure secrets managers)
- Commit database passwords or JWT secrets
- Use production credentials in development

### Red Flag: Secrets in Git History

If you accidentally committed secrets:

```bash
# DON'T just delete the file - it's still in git history!
# DO this instead:

# 1. Rotate the compromised credential immediately
# 2. Remove from git history using git-filter-repo or BFG Repo-Cleaner
# 3. Force push (only if safe to do so)
# 4. Notify your team
```

### Example: Safe Configuration Setup

**Your `.gemini/settings.json`** (project level, safe to commit):
```json
{
  "model": "gemini-2.5-pro",
  "mcpServers": {
    "myDatabase": {
      "command": "python",
      "args": ["-m", "database_mcp"],
      "env": {
        "DATABASE_URL": "${DATABASE_URL}",
        "API_KEY": "${DATABASE_API_KEY}"
      }
    }
  }
}
```

**Your `.env`** (project root, in .gitignore):
```
DATABASE_URL=postgresql://prod-db.example.com:5432/myapp
DATABASE_API_KEY=sk-secret-key-xyz789
```

**Result**: When you run `gemini`, it:
1. Loads variables from `.env`
2. Replaces `${DATABASE_URL}` with the actual connection string
3. Connects to your database safely
4. No secrets in version control ✅

#### 🎓 Expert Insight

> The `.env` + `.gitignore` + `.env.example` pattern is universal across all modern development (Python, Node.js, Go, Rust). Learn it once with Gemini CLI, apply it everywhere. This is why AI-driven learning emphasizes transferable patterns over tool-specific tricks.

#### 🤝 Practice Exercise

> **Ask your AI**: "I have a Gemini CLI project with a PostgreSQL database connection. Create a complete secure configuration setup including: 1) settings.json with environment variable references, 2) .env file with example secrets, 3) .env.example template, and 4) .gitignore entries. Make sure no secrets end up in version control."
>
> **Expected Outcome**: Complete, copy-paste-ready configuration that follows security best practices.

---

## Red Flags to Watch

### "Configuration not found"
- Check file path: `~/.gemini/settings.json` (macOS/Linux) or `C:\Users\<username>\.gemini\settings.json` (Windows)
- Create the file if it doesn't exist
- Format must be valid JSON (no trailing commas, quotes around keys)

### "Variable not found: DATABASE_URL"
- Check your `.env` file exists in project root
- Verify variable name matches exactly (case-sensitive)
- Ensure Gemini CLI can read the file (not hidden or permission-restricted)

### "Connection refused" (MCP server error)
- MCP server environment variables may be missing
- Check all required variables are in `.env`
- Run MCP server manually to see specific error

---

## Try With AI

:::tip Instruction for AI Tools
When practicing these prompts, use **Gemini CLI** as your preferred tool for this lesson. Gemini's configuration system with `.env` support is specifically designed for multi-level settings management.
:::

### Prompt 1: Configuration Hierarchy Explained
```
I'm confused about Gemini CLI's configuration hierarchy.
Explain the 7 levels and show an example where project-level settings override user-level settings.
Also explain why this matters for teams.
```

**Expected outcome**: Clear explanation of precedence with a practical team scenario.

### Prompt 2: Setting Up .env for Your Project
```
I'm setting up a Gemini CLI project that needs to connect to a PostgreSQL database
and use an external API. Show me:
1. What to put in settings.json
2. What to put in .env
3. What to add to .gitignore
4. How to access these variables in a custom MCP server
```

**Expected outcome**: Complete secure setup example you can copy for your project.

### Prompt 3: Troubleshooting Configuration Issues
```
I'm getting "Variable not found: DATABASE_URL" when trying to use an MCP server.
My .env file has DATABASE_URL=postgresql://localhost/mydb.
What could be wrong? How do I debug this?
```

**Expected outcome**: Troubleshooting checklist (file path, formatting, permissions).

### Prompt 4: Team Configuration Strategy
```
My team of 5 developers is starting a Gemini CLI project. We need to:
- Share common settings (model, theme, MCP servers)
- Keep secrets (API keys) private per developer
- Allow each developer to customize their personal preferences

Design a configuration strategy using hierarchy, .env, and version control.
Which settings go where?
```

**Expected outcome**: Team-friendly configuration architecture.

