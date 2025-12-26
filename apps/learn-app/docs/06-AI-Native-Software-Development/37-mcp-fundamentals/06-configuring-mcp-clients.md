---
sidebar_position: 6
title: "Configuring MCP Clients"
chapter: 37
lesson: 6
duration_minutes: 14
description: "Master MCP client configuration across Claude Code, Claude Desktop, VS Code, and programmatic clients. Learn environment variable handling, multi-server setup, and security best practices for production deployment."
keywords: [MCP, configuration, client, JSON, environment variables, secrets, Claude Code, VS Code, programmatic]

# HIDDEN SKILLS METADATA
skills:
  - name: "Configuring MCP Clients Across Platforms"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can configure MCP servers in Claude Code, Claude Desktop, and VS Code using JSON configuration files, understanding file locations and server structure"

  - name: "Managing Secrets and Environment Variables in MCP"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Security"
    measurable_at_this_level: "Student can correctly use environment variable references (${VAR_NAME}) in configuration, understand .env file handling, and identify API key exposure risks"

  - name: "Understanding MCP Server Connection Lifecycle"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can explain how MCP clients discover server capabilities, initialize connections, and route tool requests to appropriate servers"

  - name: "Debugging MCP Configuration Issues"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can diagnose configuration problems through understanding of path resolution, subprocess execution, and error messages"

  - name: "Comparing Configuration Approaches Across Interfaces"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can analyze configuration requirements (local vs global, desktop vs IDE vs programmatic) and select appropriate approach"

learning_objectives:
  - objective: "Understand where MCP configuration files live across different platforms and when to use project-level vs global configuration"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Identification of correct configuration file paths for Claude Code, Claude Desktop, and VS Code based on scenario"

  - objective: "Apply configuration patterns for common MCP servers (GitHub, filesystem, databases) using JSON syntax"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Hands-on configuration of multiple MCP servers in JSON format with proper command and environment setup"

  - objective: "Manage API keys and sensitive credentials using environment variables without hardcoding secrets"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Configuration exercise requiring environment variable substitution and .env file management"

  - objective: "Diagnose MCP connection problems through understanding of server discovery, capability negotiation, and error messages"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Troubleshooting dialogue with AI given configuration issues and error output"

  - objective: "Compose multiple MCP servers in single configuration and understand independent client connection model"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Analysis of multi-server configuration; identification of correct vs incorrect connection models"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (configuration files, JSON structure, environment variables, server discovery, authentication, stdio/HTTP config, path resolution, multi-server orchestration) fits B1 tier with comparison frameworks ✓"

differentiation:
  extension_for_advanced: "Research MCP configuration validation tools; analyze configuration schema from MCP specification. Compare declarative (JSON) vs programmatic (Python/Node) configuration approaches."
  remedial_for_struggling: "Start with single Claude Code configuration using filesystem server. Practice environment variable substitution with simple example before multi-server setup."
---

# Configuring MCP Clients

You've understood MCP architecture: the Host-Client-Server model, transport layers, and the three primitives (tools, resources, prompts). Now comes the practical question: **How do you actually connect MCP servers to your tools so they can use them?**

Configuration is where theory meets reality. And it's deceptively straightforward—JSON files that define which servers to launch, how to authenticate, what parameters to pass. But getting it wrong breaks everything silently. A misconfigured environment variable means your agent can't access GitHub. A wrong path means the server won't start. A typo means no error message—the tool simply fails to load.

This lesson covers configuration across every major platform: Claude Code (where you probably started), Claude Desktop (the standalone application), VS Code (for developers integrating MCP), and programmatic clients (when you build agents in Python or Node.js). By the end, you'll understand how to set up production-ready MCP configurations that work reliably.

## Configuration Locations: Where MCP Looks for Servers

MCP clients search for configuration in predictable locations:

### Claude Code (Project-Level Configuration)

**Path**: `.claude/settings.json` (in your project root)

This is **project-specific**. Each project can have different MCP servers configured.

```json
{
  "mcpServers": {
    "filesystem": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-filesystem", "/path/to/project"]
    },
    "github": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-github"],
      "env": {
        "GITHUB_TOKEN": "${GITHUB_TOKEN}"
      }
    }
  }
}
```

**Advantages**:
- Project isolation (different projects, different servers)
- Committed to git (team visibility)
- Workspace-specific setup

**Use when**:
- Building agents for specific projects
- Team collaboration (configuration in version control)
- Different projects need different server sets

### Claude Desktop (Global Configuration)

**Path**:
- **macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Windows**: `%APPDATA%\Claude\claude_desktop_config.json`
- **Linux**: `~/.config/Claude/claude_desktop_config.json`

This is **global**—applies to all Claude Desktop sessions.

```json
{
  "mcpServers": {
    "github": {
      "command": "uvx",
      "args": ["--python", "3.12", "mcp-github"],
      "env": {
        "GITHUB_TOKEN": "${GITHUB_TOKEN}"
      }
    },
    "filesystem": {
      "command": "uvx",
      "args": ["mcp-filesystem", "/Users/yourname/Documents"]
    }
  }
}
```

**Advantages**:
- Always available across sessions
- Centralized configuration
- Perfect for frequently-used servers

**Use when**:
- Tools you use in every conversation
- Personal assistant-style MCP servers
- Development tools needed across projects

### VS Code / Cursor (User Settings)

**Path**:
- **User settings** (global): Via VS Code settings UI or `settings.json`
- **Workspace settings** (project): `.vscode/settings.json`

VS Code integrates MCP through extensions. Configuration looks slightly different:

```json
{
  "mcpServers": {
    "github": {
      "command": "uvx",
      "args": ["mcp-github"],
      "env": {
        "GITHUB_TOKEN": "${GITHUB_TOKEN}"
      }
    }
  }
}
```

Settings are managed through VS Code's configuration system, not directly as files (though the underlying format is JSON).

**Use when**:
- IDE-integrated MCP usage
- VS Code extensions that speak MCP
- Cursor IDE integration

## Configuration Structure: The mcpServers Object

Every MCP client configuration follows the same structure:

```json
{
  "mcpServers": {
    "server-name": {
      "command": "executable or npx",
      "args": ["arg1", "arg2", "arg3"],
      "env": {
        "VAR_NAME": "${VAR_NAME}"
      }
    }
  }
}
```

Let's break down each field:

### `command`: How to Start the Server

**For local executables**:
```json
"command": "/usr/local/bin/mcp-server"
```

**For npm packages** (via npx):
```json
"command": "npx",
"args": ["-y", "@modelcontextprotocol/server-github"]
```

**For Python packages** (via uvx):
```json
"command": "uvx",
"args": ["--python", "3.12", "mcp-github"]
```

The client executes this command as a subprocess and communicates via stdio.

### `args`: Command Arguments

Arguments passed to the command. For npm packages:
```json
"args": ["-y", "@modelcontextprotocol/server-filesystem", "/path/to/serve"]
```

Breaking this down:
- `-y`: Auto-answer yes to npm prompts
- `@modelcontextprotocol/server-filesystem`: Package name
- `/path/to/serve`: Path the server will expose

**For HTTP servers** (if you're pointing to remote MCP):
```json
"args": ["--host", "localhost", "--port", "8000"]
```

### `env`: Environment Variables

Passes environment variables to the server process. **This is critical for secrets**.

```json
"env": {
  "GITHUB_TOKEN": "${GITHUB_TOKEN}",
  "DATABASE_URL": "${DATABASE_URL}",
  "API_KEY": "${API_KEY}"
}
```

The `${VAR_NAME}` syntax tells the client: "Look up VAR_NAME from the environment where you're running and substitute it here."

This protects you from accidentally committing secrets to git:
- Configuration file contains `${GITHUB_TOKEN}` (safe)
- Your shell environment has `GITHUB_TOKEN=ghp_xxxx` (secret)
- Client substitutes at runtime

## Environment Variables: Where Secrets Come From

Configuration references environment variables. The client retrieves their actual values from:

1. **Shell environment** (what you have in your terminal)
2. **.env files** (if the client supports them)
3. **System variables** (machine-wide settings)

### .env Files (For Development)

Create a `.env` file in your project root:

```
GITHUB_TOKEN=ghp_xxxxxxxxxxxxxxxxxxxx
DATABASE_URL=postgresql://user:pass@localhost/db
API_KEY=sk-1234567890abcdef
```

**Don't commit this to git.** Add to `.gitignore`:
```
.env
.env.local
```

Some MCP clients automatically load `.env` files. Others require you to `source` the file:

```bash
export $(cat .env | xargs)
```

Then start Claude Code or your application.

### System Environment Variables

For production, set variables in your deployment environment:

```bash
# In your shell
export GITHUB_TOKEN=ghp_xxxx

# Or in deployment platform (Docker, Kubernetes, etc.)
ENV GITHUB_TOKEN=ghp_xxxx
```

The client reads these at runtime.

## Common Configuration Patterns

### Pattern 1: Filesystem Server (Local Files)

```json
{
  "mcpServers": {
    "filesystem": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-filesystem", "/Users/you/project"]
    }
  }
}
```

**What it does**: Exposes files in `/Users/you/project` as MCP resources. The agent can read files, search directories.

**No environment variables needed**: Filesystem doesn't require secrets.

### Pattern 2: GitHub Server (Requires Authentication)

```json
{
  "mcpServers": {
    "github": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-github"],
      "env": {
        "GITHUB_TOKEN": "${GITHUB_TOKEN}"
      }
    }
  }
}
```

**What it does**: Enables MCP tools for creating issues, fetching PRs, managing workflows.

**Requires**: GitHub Personal Access Token with appropriate permissions.

**Get token**:
1. Go to GitHub Settings → Developer Settings → Personal Access Tokens
2. Create token with `repo`, `read:org` scopes
3. Store in `.env`: `GITHUB_TOKEN=ghp_xxxx`

### Pattern 3: Database Server (PostgreSQL, MySQL, etc.)

```json
{
  "mcpServers": {
    "database": {
      "command": "uvx",
      "args": ["mcp-database"],
      "env": {
        "DATABASE_URL": "${DATABASE_URL}"
      }
    }
  }
}
```

**What it does**: Exposes database tables as MCP resources; provides tools for querying and updates.

**Requires**: Database connection string.

**Format**: `postgresql://user:password@host:port/database`

**Get connection string**:
```bash
# For local development
DATABASE_URL=postgresql://user:pass@localhost:5432/mydb

# For managed services (Supabase, Railway, etc.)
DATABASE_URL=postgresql://user:xxx@server.replication.supabase.co:5432/postgres?sslmode=require
```

### Pattern 4: Multiple Servers in One Configuration

```json
{
  "mcpServers": {
    "filesystem": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-filesystem", "/project"]
    },
    "github": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-github"],
      "env": {
        "GITHUB_TOKEN": "${GITHUB_TOKEN}"
      }
    },
    "database": {
      "command": "uvx",
      "args": ["mcp-database"],
      "env": {
        "DATABASE_URL": "${DATABASE_URL}"
      }
    }
  }
}
```

**Important**: Each server is independent. The client launches one subprocess per server. When you ask the agent to execute a tool:

1. Agent requests tool `create_issue` (GitHub tool)
2. Client routes to GitHub server
3. GitHub server executes; returns result

The agent doesn't know (or care) about database server—it's not involved unless you explicitly request a database tool.

## Configuration for Programmatic Clients (Python)

When building agents programmatically, you configure MCP differently—through code instead of JSON files.

### Using Claude SDK with Stdio Transport

```python
from mcp import ClientSession, StdioClientTransport

# Configure server
transport = StdioClientTransport(
    command="npx",
    args=[
        "-y",
        "@modelcontextprotocol/server-github"
    ],
    env={
        "GITHUB_TOKEN": os.getenv("GITHUB_TOKEN")
    }
)

# Create session
async with ClientSession(transport) as session:
    await session.initialize()
    tools = await session.list_tools()
    print(tools)
```

**Key differences from JSON**:
- Direct Python code instead of configuration files
- Environment variables via `os.getenv()` instead of `${VAR_NAME}`
- Explicit session initialization

### Using Anthropic SDK with MCP Integration

```python
from anthropic import Anthropic

client = Anthropic()

# Configure MCP servers
mcp_config = {
    "mcpServers": {
        "github": {
            "command": "npx",
            "args": ["-y", "@modelcontextprotocol/server-github"],
            "env": {
                "GITHUB_TOKEN": os.getenv("GITHUB_TOKEN")
            }
        }
    }
}

# Pass to client
response = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    tools=[...],  # MCP tools discovered via mcp_config
    messages=[...]
)
```

## Configuration Comparison: Which to Use When

| Scenario | Location | Format | Best For |
|----------|----------|--------|----------|
| **Claude Code** | `.claude/settings.json` | JSON | Project-specific agents, team workflows |
| **Claude Desktop** | `~/.config/Claude/claude_desktop_config.json` (Linux) | JSON | Personal use, always-available tools |
| **VS Code** | `.vscode/settings.json` or user settings | JSON | IDE integration, extension support |
| **Python Agent** | Programmatic (code) | Python objects | Custom agent applications, deployment |
| **Node.js Agent** | Programmatic (code) | JavaScript objects | Backend services, serverless functions |

**Decision framework**:
- **Team collaboration**: Use Claude Code with `.claude/settings.json` (in git)
- **Personal tools**: Use Claude Desktop global config
- **IDE workflows**: Use VS Code settings integration
- **Production agents**: Programmatic configuration in Python/Node

## Security Best Practices for Production

### Never Hardcode Secrets

**WRONG**:
```json
{
  "env": {
    "GITHUB_TOKEN": "ghp_xxxxxxxxxxxxxxxxxxxx"
  }
}
```

Commit this to git, and it's compromised forever. GitHub revokes it, but the token is in your git history permanently.

**RIGHT**:
```json
{
  "env": {
    "GITHUB_TOKEN": "${GITHUB_TOKEN}"
  }
}
```

Let the deployment platform provide the secret.

### Use Secret Management for Production

**For Kubernetes**:
```yaml
apiVersion: v1
kind: Secret
metadata:
  name: mcp-secrets
data:
  GITHUB_TOKEN: base64encodedtoken
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: mcp-config
data:
  settings.json: |
    {
      "mcpServers": {
        "github": {
          "command": "npx",
          "args": ["-y", "@modelcontextprotocol/server-github"],
          "env": {
            "GITHUB_TOKEN": "${GITHUB_TOKEN}"
          }
        }
      }
    }
```

**For Docker**:
```dockerfile
FROM node:20

COPY . /app
WORKDIR /app

# Read secret from build arg (passed at build time, not stored in image)
ARG GITHUB_TOKEN
ENV GITHUB_TOKEN=${GITHUB_TOKEN}

CMD ["node", "agent.js"]
```

Build with: `docker build --build-arg GITHUB_TOKEN=ghp_xxx .`

### Rotate Credentials Regularly

MCP agents running for weeks/months should use credentials that can be rotated without restarting:
- GitHub: Regularly create new Personal Access Tokens
- Database: Use database users with limited permissions
- API keys: Implement key rotation policies

### Principle of Least Privilege

Only grant credentials the minimum permissions needed:
- **GitHub**: If MCP server only reads PRs, create token with read-only scopes
- **Database**: Create database user with SELECT on specific tables (not admin)
- **API Keys**: Use API keys with rate limits, specific resource access

This limits damage if credentials leak.

## Troubleshooting Configuration Issues

### Server Won't Start

**Error**: `Command not found: mcp-server`

**Causes**:
- Executable doesn't exist at path
- npx/uvx not installed
- Path is wrong

**Fix**:
```bash
# Verify command works outside config
npx -y @modelcontextprotocol/server-github

# If it works manually, config path might be relative
# Use absolute paths instead
"command": "/usr/local/bin/mcp-server"
# Or use npx/uvx (which finds packages)
"command": "npx"
```

### Server Starts But No Tools Available

**Cause**: Environment variable not found

**Error**: `GITHUB_TOKEN not provided`

**Fix**:
```bash
# Set variable before starting
export GITHUB_TOKEN=ghp_xxxx
# Then start Claude Code

# Or add to .env and source it
source .env
```

### Configuration Not Being Loaded

**Cause**: Wrong file path or location

**Check**:
- Claude Code: `./.claude/settings.json` in project root
- Claude Desktop (Linux): `~/.config/Claude/claude_desktop_config.json`
- VS Code: Settings UI or `.vscode/settings.json`

**Verify**:
```bash
# List Claude Code config
cat .claude/settings.json | jq .mcpServers

# Check Claude Desktop location (macOS)
cat ~/Library/Application\ Support/Claude/claude_desktop_config.json | jq .
```

### JSON Syntax Error

**Error**: `Invalid JSON in configuration`

**Check**:
- Missing commas between properties
- Trailing commas
- Unquoted keys

**Validate**:
```bash
# Use jq to validate
cat .claude/settings.json | jq .

# Or use online JSON validator
```

## Try With AI

Use your AI companion to explore MCP configuration scenarios and troubleshoot setup issues.

### Safety Note on Configuration Security

When configuring MCP servers, remember:
- **Never commit `.env` files** or configuration with hardcoded secrets to git
- **Use environment variable references** (`${VAR_NAME}`) in configuration files
- **Rotate credentials** regularly—if a secret leaks, create new credentials immediately
- **Principle of least privilege**—only grant each MCP server the permissions it needs
- **Verify SSL/TLS** when connecting to remote MCP servers—don't accept unencrypted connections in production

Configuration security determines whether your agents can be compromised. Get this right.

### Prompt 1: Analyze Your Current Configuration

**Setup**: Understanding what's already configured in your environment

```
I have MCP servers configured in Claude Code. Help me understand what I have set up.

Here's my .claude/settings.json:
[paste your configuration]

For each server in my configuration:
1. Identify what tool this server provides
2. What permissions or secrets does it need?
3. How would I know if this server is working correctly?
4. What would break if I misconfigured the environment variables?
```

**What you're learning**: Analyzing existing configuration to understand server requirements and potential failure modes.

### Prompt 2: Configure a New Server

**Setup**: Adding GitHub server to an existing setup

```
I want to add a GitHub MCP server to my configuration. I already have:
[paste your current mcpServers object]

I want to be able to create issues and fetch pull requests. Help me:
1. Write the JSON configuration for a GitHub MCP server
2. Tell me what permissions I need on my GitHub token
3. Explain how to securely provide the token (I should never commit it to git)
4. Walk me through testing to verify it's working

Here's what my environment looks like:
- I use Claude Code
- My project is at /Users/me/project
- I have a GitHub Personal Access Token already created
```

**What you're learning**: Configuring new servers, managing environment variables securely, and validating setup through testing.

### Prompt 3: Troubleshoot Configuration Problems

**Setup**: Debugging a configuration that isn't working

```
I configured a database MCP server but when I try to query the database,
I get this error: "[ENOENT] DATABASE_URL environment variable not set"

Here's my configuration:
[paste your configuration]

Here's what I did to set up the environment:
1. Created a .env file with DATABASE_URL=postgresql://...
2. Didn't source the .env file
3. Started Claude Code directly

What's wrong? Walk me through:
1. Why this error is happening
2. How environment variable substitution works
3. The correct way to provide DATABASE_URL
4. How to debug this in the future
```

**What you're learning**: Understanding why configuration fails, how to diagnose environment variable issues, and implementing proper secret management.

### Prompt 4: Design Configuration for Team Collaboration

**Setup**: Setting up MCP for a team project

```
I'm setting up MCP configuration for my team's project. Here's what we need:

- 5 developers, each with their own GitHub token
- Shared project codebase (all on same git repo)
- Database server accessible from cloud (not local)
- Filesystem access to /project directory

For this scenario:
1. Should we use .claude/settings.json or global Claude Desktop config?
2. How do we handle different GITHUB_TOKEN values per developer?
3. Should DATABASE_URL go in .env, environment variables, or somewhere else?
4. What do we commit to git? What do we keep local?

Help me design a configuration strategy that works for a team.
```

**What you're learning**: Scaling configuration from personal use to team collaboration; understanding security implications of shared configuration.

