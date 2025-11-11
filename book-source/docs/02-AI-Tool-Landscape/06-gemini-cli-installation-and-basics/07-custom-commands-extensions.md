---
sidebar_position: 7
title: "Custom Commands & Extensions"
duration: "40 min"
---

# Custom Commands & Extensions: Extending Gemini CLI

## Introduction: Building Your Own Toolset

By now, you've learned:
- **Slash commands** (`/memory`, `/stats`, `/mcp`) for orchestration
- **GEMINI.md** for project context
- **Memory checkpoints** for conversation persistence
- **MCP servers** for external integrations

Now it's time to create your own commands and install capability bundles that package everything together.

---

## Custom Slash Commands: Creating Your Own

### Why Custom Commands?

Imagine repeating the same multi-step workflow daily:

```
Without custom command (manual):
1. /mcp list
2. Ask Gemini to research competitor prices
3. Ask Gemini to summarize
4. /memory save results
(5 minutes, multiple steps)

With custom command:
/research competitors
(30 seconds, one command)
```

Custom commands automate repeated patterns.

---

## TOML Command Syntax

Gemini CLI uses **TOML** (simple configuration format) for custom commands.

### Basic Structure

```toml
[command]
name = "research"
description = "Research a topic using web browsing"
prompt = "Search the web for {{query}} and summarize findings in 5 bullet points"
```

### With Variables

```toml
[command]
name = "research"
description = "Research a topic"
prompt = """
Using the Playwright MCP server, research {{topic}}.
Look for:
- Current trends
- Key players
- Market size
- Challenges
Summarize in a structured report.
"""
```

### With Shell Integration

```toml
[command]
name = "audit-code"
description = "Security audit of your code"
shell = "!find src -name '*.ts' -type f"
prompt = "These are my TypeScript files. {{shell_result}} Perform a security audit and list vulnerabilities."
```

---

## Two Levels: User vs. Project Commands

### User Commands (~/.gemini/commands/)

**For:** Personal workflow patterns you use across projects

**Examples:**
- `/research` — Search and summarize
- `/brainstorm` — Creative ideation
- `/code-review` — Review code patterns
- `/architecture` — Architecture discussion

**Setup:**
```bash
mkdir -p ~/.gemini/commands/
# Create command files here
```

### Project Commands (./gemini/commands/)

**For:** Project-specific workflows unique to this codebase

**Examples:**
- `/spec` — Generate API specification
- `/test-strategy` — Suggest test approach
- `/refactor-proposal` — Suggest refactorings
- `/migration-plan` — Plan database migration

**Setup:**
```bash
mkdir -p ./gemini/commands/
# Create command files here (committed to Git)
```

---

## Real Examples

### Example 1: Research Command

```toml
# ~/.gemini/commands/research.toml
[command]
name = "research"
description = "Research a topic using web browsing"
prompt = """
Using the Context7 MCP server, research the following topic: {{topic}}

Please provide:
1. Summary of current state
2. Key trends
3. Recent developments (last 6 months)
4. Key players/companies
5. Recommended resources

Format as structured markdown.
"""
```

**Usage:**
```
/research AI safety regulations in Europe
```

### Example 2: Code Review Command

```toml
# ./gemini/commands/code-review.toml
[command]
name = "code-review"
description = "Review code in project for patterns"
prompt = """
Review @src/{{path}}.

Check for:
- Security issues
- Performance problems
- Maintainability
- Testing coverage
- Best practices

Format as:
## Security
## Performance
## Maintainability
## Testing
## Best Practices
"""
```

**Usage:**
```
/code-review routes/auth.ts
```

---

## Extensions: Bundled Capabilities

### What Is an Extension?

An **extension** packages:
- Custom commands (TOML files)
- MCP server configurations
- GEMINI.md context snippets
- Settings overrides

Into a shareable, installable bundle.

### How Extensions Work

**Install:**
```
/extension install my-org/productivity-suite
```

Gemini CLI downloads and installs:
- 5+ custom commands (bundled)
- 2 MCP servers (preconfigured)
- Best practices (GEMINI.md)
- Settings (model, token limits, etc.)

**Result:** You get a complete workflow setup in one command.

---

## Security Evaluation Framework

Before installing an extension, evaluate:

### 1. Code Review
- ✅ Is source available on GitHub?
- ✅ Can you review the commands?
- ✅ Do they do what they claim?

### 2. Permission Review
- ✅ What files can it access?
- ✅ What external services does it use?
- ✅ Does it need your API keys?

### 3. Network Activity
- ✅ Does it send data externally?
- ✅ Is it encrypted?
- ✅ Can you see the requests?

### 4. Data Handling
- ✅ Does it log conversations?
- ✅ Does it cache your code?
- ✅ How long is data retained?

### 5. Author Reputation
- ✅ Who maintains it?
- ✅ How active is development?
- ✅ Community feedback?

### 6. Community Trust
- ✅ GitHub stars/downloads
- ✅ Reviews and ratings
- ✅ Issue resolution rate

---

## Exercises

### Exercise 1: Create a User Command

Create your first custom command:

**Step 1:** Create command file
```bash
mkdir -p ~/.gemini/commands/
touch ~/.gemini/commands/hello.toml
```

**Step 2:** Add content
```toml
[command]
name = "hello"
description = "Greet me with context about my work"
prompt = "Greet me warmly and remind me that I'm building {{project}}, focusing on {{goal}}."
```

**Step 3:** Use it
```
/hello project=my-api goal=security
```

### Exercise 2: Create a Project Command

**Step 1:** Create in project
```bash
mkdir -p ./gemini/commands/
touch ./gemini/commands/spec.toml
```

**Step 2:** Add content
```toml
[command]
name = "spec"
description = "Generate API spec from code"
prompt = "Looking at my code, generate an OpenAPI specification for my {{endpoint}} endpoint."
```

**Step 3:** Use it
```
/spec endpoint=users
```

---

## Best Practices

### ✅ DO

- ✅ Name commands after their function (`/research`, `/review`, `/spec`)
- ✅ Use descriptive descriptions
- ✅ Include variables for flexibility (`{{topic}}`, `{{path}}`)
- ✅ Test locally before sharing
- ✅ Document what the command does
- ✅ Review extensions before installing

### ❌ DON'T

- ❌ Don't create commands that duplicate slash commands
- ❌ Don't hardcode project-specific paths in user commands
- ❌ Don't trust unreviewed extensions
- ❌ Don't install extensions with unclear permissions
- ❌ Don't share commands with hardcoded secrets

---

## Key Takeaways

- **Custom commands** automate repeated workflows
- **TOML syntax** is simple, approachable, powerful
- **User commands** (~/.gemini/) for personal patterns
- **Project commands** (./gemini/) for team workflows
- **Extensions** bundle commands + configs for distribution
- **Security evaluation** is your responsibility

This completes Chapter 6! You now understand:
1. ✅ Slash commands for orchestration
2. ✅ GEMINI.md for project context
3. ✅ Memory checkpoints for persistence
4. ✅ MCP servers for external integrations
5. ✅ Custom commands for automation
6. ✅ Extensions for bundled capabilities

You're ready to use Gemini CLI as a true development partner.


<Quiz title="Chapter 7 Quiz" questions={[{"question":"The chapter describes an evolution of AI coding tools through four generations. What is the correct order of these generations?","options":{"a":"Function Generation → Autocomplete → Feature Implementation → Autonomous Agents","b":"Autocomplete → Function Generation → Feature Implementation → Autonomous Agents","c":"Autonomous Agents → Feature Implementation → Function Generation → Autocomplete","d":"Feature Implementation → Autonomous Agents → Autocomplete → Function Generation"},"correct_answer":"b","explanation":"The chapter explicitly lists the progression: \u0027code completion → function generation → feature implementation → autonomous agents.\u0027"},{"question":"What distinguishes a \u0027Generation 4: Autonomous Agent\u0027 from previous generations of AI tools?","options":{"a":"It can only suggest the next line of code.","b":"It can take high-level goals, break them into tasks, and execute them with minimal human intervention.","c":"It can only generate a single function or class in isolation.","d":"It requires the human to manually run tests and fix all bugs."},"correct_answer":"b","explanation":"The text defines a Generation 4 agent\u0027s capability as being able to \u0027Take high-level goals, break them into tasks, implement solutions, test, debug, and iterate—all with minimal human intervention.\u0027"},{"question":"Which of the following is a task the chapter states an autonomous agent CANNOT do yet?","options":{"a":"Write code and run tests.","b":"Break a high-level goal into subtasks.","c":"Make strategic trade-off decisions, like performance vs. simplicity.","d":"Fix failing tests based on error messages."},"correct_answer":"c","explanation":"The chapter lists things an agent cannot do yet, including: \u0027Make strategic trade-off decisions (performance vs. simplicity, cost vs. scalability).\u0027"},{"question":"What is the primary benefit of using a multi-agent system (a team of specialized agents)?","options":{"a":"It is cheaper than using a single agent.","b":"It allows for specialization, where a testing agent can be more thorough, and a review agent can focus on security.","c":"It is simpler to manage than a single agent.","d":"It eliminates the need for a human orchestrator."},"correct_answer":"b","explanation":"The text explains the benefit of specialization: \u0027Just like human teams benefit from specialization, agent teams can be more effective... Testing Agent can be more thorough... Review Agent can check security...\u0027"},{"question":"What is the chapter\u0027s main point about the role of human judgment in the autonomous agent era?","options":{"a":"Human judgment will no longer be necessary.","b":"Human judgment remains essential for tasks like understanding user needs, making trade-offs, and navigating ambiguity.","c":"Only junior developers will need to use their judgment.","d":"Human judgment is only needed for the initial prompt."},"correct_answer":"b","explanation":"The chapter emphasizes, \u0027Even in the full autonomous agent era, human developers remain essential,\u0027 because development requires \u0027human judgment, domain expertise, and strategic thinking.\u0027"}]} />

