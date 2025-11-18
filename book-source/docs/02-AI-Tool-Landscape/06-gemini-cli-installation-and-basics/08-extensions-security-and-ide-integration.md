---
sidebar_position: 8
title: Extensions, Security & Tool Safety
cefr_level: A2
proficiency: Beginner
teaching_stage: 2
stage_name: "AI Collaboration"
cognitive_load:
  concepts_count: 7
  a2_compliant: true
learning_objectives:
  - id: LO1
    description: "Understand what Gemini CLI extensions are and why they simplify setup sharing"
    bloom_level: "Understand"
    digcomp: "1.3 Managing data, information and digital content"
  - id: LO2
    description: "Install and manage extensions using basic commands"
    bloom_level: "Apply"
    digcomp: "3.1 Developing digital content"
  - id: LO3
    description: "Recognize why tool filtering protects your computer from unwanted changes"
    bloom_level: "Understand"
    digcomp: "4.2 Protecting personal data and privacy"
  - id: LO4
    description: "Apply allowlist approach to control which tools AI can use"
    bloom_level: "Apply"
    digcomp: "4.2 Protecting personal data and privacy"
  - id: LO5
    description: "Explain when extensions are useful for learning setups"
    bloom_level: "Understand"
    digcomp: "1.3 Managing data, information and digital content"
  - id: LO6
    description: "Evaluate which MCP capabilities are safe for beginner learning"
    bloom_level: "Evaluate"
    digcomp: "4.2 Protecting personal data and privacy"
---

# Extensions, Security & Tool Safety

You've spent hours perfecting your Gemini CLI setup: 2 MCP servers, 3 custom commands, a detailed GEMINI.md file. Now your study partner asks: "How do I get the same setup for our learning group?" You realize you're about to write them a long step-by-step guide. There has to be a better way. In this lesson, you'll learn how extensions turn "follow 15 instructions" into one command: `gemini extensions install`

---

## The Problem: Sharing Your Setup is Painful

You've built a useful Gemini CLI setup: MCP servers, custom commands, learning context. Now your study partner asks: "How do I get the same setup for our study group?"

**The manual way**:
```
STEP 1: Install MCP servers (2 commands)
STEP 2: Configure tool safety (edit settings.json)
STEP 3: Download 3 custom command files
STEP 4: Create GEMINI.md with learning standards
STEP 5: Set environment settings
```

**The problems**:
- ❌ 15+ manual steps per person
- ❌ Easy to miss steps (wrong folder, missed safety config)
- ❌ Different versions (everyone has slightly different setups)
- ❌ Update difficulty (re-send everything when you improve something)

---

## The Solution: Extensions Bundle Everything

What if your study partners could get your **entire setup** with **one command**?

```bash
gemini extensions install https://github.com/learning-group/ai-study-tools
```

**That's what extensions do.**

### What is an Extension?

An **extension** is a **pre-packaged bundle** containing everything:

**Included automatically**:
- ✅ MCP servers (pre-configured with safety settings)
- ✅ Custom slash commands (all `.toml` files)
- ✅ Persistent context (GEMINI.md with learning standards)
- ✅ Configuration templates
- ✅ Tool filtering rules (safety controls)
- ✅ Environment settings
- ✅ Documentation

**MCP server alone**: Single capability (e.g., Playwright for web research)
**Extension (complete package)**: MCP servers + commands + context + safety + docs

#### 💬 AI Colearning Prompt
> "If extensions just bundle MCP servers and commands, couldn't we just share a folder with setup files? What's the real advantage of the extension format?"
>
> **Hint**: Think about what happens when you update the extension vs. when you update a document with setup instructions.

---

## Seeing the Difference: Before and After

**Without Extension**:
- Follow 15-step document (30 minutes)
- Miss a step (wrong folder, no tool filtering)
- Fix issues (another 30 minutes)
- Update available next week → Re-download and replace files

**With Extension**:
```bash
gemini extensions install https://github.com/learning-group/ai-study-tools
```

**What happens**: Everything installs automatically—MCP servers, commands, context, safety settings.

**Update**: `gemini extensions update ai-study-tools`

**The transformation**: 60 minutes → 2 minutes (30x faster), zero errors, always in sync.

---

## Part 1: Working with Extensions

### Installing Extensions

You now understand WHY extensions matter. Let's see HOW to use them.

#### Install from URL

```bash
gemini extensions install https://github.com/learning-group/ai-study-tools
```

**What this does**:
- Downloads all extension files
- Installs MCP servers automatically
- Adds custom commands
- Applies safety settings
- Creates GEMINI.md context file

#### What's Inside an Extension?

When you install an extension, it creates this structure:

```
ai-study-tools/
├── gemini-extension.json   (configuration file - lists what's included)
├── GEMINI.md                (learning context for AI)
├── commands/                (custom slash commands)
│   ├── study-plan.toml     (command to create study plans)
│   └── quiz-me.toml        (command to generate quizzes)
└── docs/
    └── how-to-use.md       (instructions)
```

#### Understanding the Configuration

The `gemini-extension.json` file tells Gemini what to install:

```json
{
  "name": "ai-study-tools",
  "version": "1.0.0",
  "description": "Tools for learning with AI",
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["@playwright/mcp@latest"]
    }
  },
  "contextFileName": "GEMINI.md",
  "excludeTools": ["delete_file", "run_shell_command"]
}
```

**Key parts**:
- **name**: What the extension is called
- **mcpServers**: Which MCP servers to install (Playwright for web research)
- **contextFileName**: The GEMINI.md file with learning context
- **excludeTools**: Dangerous tools AI cannot use (for safety)

### Managing Extensions

**List installed extensions**:
```bash
gemini extensions list
```

This shows all extensions you've installed.

**Update extensions**:
```bash
gemini extensions update ai-study-tools
```

When the extension creator improves it, you get the updates automatically.

**Update all extensions at once**:
```bash
gemini extensions update --all
```

**Remove an extension**:
```bash
gemini extensions uninstall ai-study-tools
```

This removes the extension but keeps any work you created with it.

---

## Part 2: Tool Filtering for Safety

### Why Tool Filtering Matters

MCP servers give AI capabilities. Some are safe; some could change your files. A single MCP server might offer:
- `read_file` (safe - just reads, doesn't change anything)
- `write_file` (risky - could change your files)
- `delete_file` (very risky - could remove your files)
- `run_shell_command` (extremely risky - could run any command)

**Without filtering**: AI has access to all tools, including risky ones.
**With filtering**: You control exactly which tools AI can use.

### Two Filtering Approaches

#### Allowlist: `includeTools` (Safer)

Only allow specific safe tools:

```json
{
  "mcpServers": {
    "file-helper": {
      "command": "npx",
      "args": ["@modelcontextprotocol/server-filesystem"],
      "includeTools": ["read_file", "list_directory"]
    }
  }
}
```

**Result**: AI can ONLY use `read_file` and `list_directory`. All other tools are blocked.

**Think of it like**: "AI can ONLY do these specific things I list."

#### Blocklist: `excludeTools` (Less Safe)

Block specific dangerous tools:

```json
{
  "mcpServers": {
    "file-helper": {
      "command": "npx",
      "args": ["@modelcontextprotocol/server-filesystem"],
      "excludeTools": ["delete_file", "run_shell_command"]
    }
  }
}
```

**Result**: All tools available EXCEPT `delete_file` and `run_shell_command`.

**Risk**: If the MCP server adds a new dangerous tool later, you might not know to block it.

**Think of it like**: "AI can do anything EXCEPT these specific things."

### Learning Scenario: Choosing Safe Tools

**Situation**: You want to use an MCP server for organizing study notes. The server offers these tools:
- `read_file` - Read your notes ✅ Safe (just reading)
- `search_content` - Find topics in notes ✅ Safe (just searching)
- `write_file` - Edit notes ⚠️ Risky (could change your notes)
- `delete_file` - Remove notes ❌ Very Risky (could delete your work)
- `run_command` - Run terminal commands ❌ Extremely Risky

**Safe Configuration**:
```json
{
  "mcpServers": {
    "noteOrganizer": {
      "command": "npx",
      "args": ["note-organizer"],
      "includeTools": ["read_file", "search_content"]
    }
  }
}
```

**Result**: AI can read and search your notes but **cannot change or delete them**.

### Safety Best Practices

1. **Ask before enabling**: "What tools does this MCP server have?"
2. **Use allowlist**: Only include tools you need and trust
3. **Start small**: Begin with reading/searching tools only
4. **Test safely**: Try on a test folder first, not your real work
5. **Check regularly**: Review which tools you've allowed

---

## Part 3: When to Use Extensions

### Decision Framework

Extensions are most useful when you need to:

**Share a complete setup**:
- Study group wants same MCP servers and commands
- Learning partner needs your GEMINI.md context
- Online course provides standard AI setup for all students
- Want everyone to have identical, tested configuration

**Example**: "Our Python learning group uses the same research tools and study commands. Installing the extension gives everyone the setup instantly."

**Keep setups in sync**:
- Creator improves the extension → Everyone updates easily
- No "version drift" (everyone different)
- Updates happen with one command

**Example**: "When the course instructor improves the study tools, we all run `gemini extensions update course-tools` instead of re-downloading files."

**Bundle safety settings**:
- Extension includes pre-configured tool filtering
- Beginners don't need to figure out which tools are safe
- Safety rules update when extension updates

**Example**: "The extension blocks risky file operations automatically. I don't have to remember which tools to exclude."

### When Extensions Are NOT Needed

**Simple, personal setup**:
- You're just trying different MCP servers
- Experimenting with commands
- Not sharing with anyone
- Setup changes frequently

**Example**: "I'm still figuring out which MCP servers I like. I'll just install them one at a time for now."

**One-time use**:
- Testing a single MCP server
- Temporary project
- Don't need updates

**Example**: "I just want to try Playwright MCP once. I don't need a full extension."

---

## Common Questions

### "Can I use multiple extensions at once?"

Yes! Install as many as you need:
```bash
gemini extensions install https://github.com/group-a/study-tools
gemini extensions install https://github.com/course-b/research-tools
```

Both extensions work together. Each adds its own MCP servers and commands.

### "What if two extensions conflict?"

If two extensions try to add the same command name, the second one wins. You can disable one:
```bash
gemini extensions list
gemini extensions disable study-tools
```

Now only `research-tools` is active.

### "How do I know which tools an extension allows?"

Look inside the `gemini-extension.json` file:
- Check `includeTools` (what's allowed)
- Check `excludeTools` (what's blocked)

Or ask AI: "What tools does this extension give you access to?"

### "Can I modify an extension after installing?"

Extensions install to a special folder. Your changes get overwritten on update.

**Better approach**: Create your own GEMINI.md in your project folder. It adds to (doesn't replace) the extension's context.

---

## Troubleshooting

### "Extension installation failed"

**Check these**:
1. Is the GitHub URL correct? `github.com/user/repo`
2. Is your internet connection working?
3. Try installing again

**Fix**: Copy the URL exactly as shown, including `https://`

### "Tool not available"

**Error message**: `Tool "search_web" not available`

**This means**: The extension's tool filtering blocked this tool, OR the MCP server doesn't have it.

**Check**:
1. Look at extension's `gemini-extension.json`
2. Check if tool is in `excludeTools` (blocked)
3. Check if tool is missing from `includeTools` (not allowed)

**Fix**: If you need the tool and it's safe, you can install the MCP server separately without the extension's restrictions.

### "Safety concern: Extension wants too much access"

**Before installing**, check what tools it enables:
1. Look at the extension's GitHub page
2. Read `gemini-extension.json`
3. Check if `includeTools` limits access (safer)
4. Check if `excludeTools` only blocks a few things (less safe)

**Ask AI**: "This extension includes [MCP server name]. What tools does it give you? Are any risky?"

**Rule**: If unsure, don't install. Try the MCP server individually first with strict `includeTools`.

---

## Try With AI

**Use Gemini CLI** for these prompts to practice what you learned.

### Prompt 1: Understanding Tool Safety
```
I'm looking at an extension that includes the filesystem MCP server.
It has these tools available:
- read_file
- write_file
- delete_file
- list_directory
- create_directory

Which tools are safe for a beginner learning AI tools?
Which should I block with excludeTools?
Explain why each dangerous tool is risky.
```

**Expected outcome**: AI explains safety levels of each tool and recommends allowlist configuration.

### Prompt 2: Planning a Learning Group Extension
```
I'm in a Python learning group (5 beginners).
We want to create an extension for our study setup.

We need:
- Playwright MCP for researching documentation
- A custom /study-plan command
- A GEMINI.md with our learning goals
- Safe tool filtering (read-only access)

Show me:
1. What the gemini-extension.json should include
2. Which tools to allow (and which to block)
3. How to share it with the group
```

**Expected outcome**: Basic extension structure designed for learning context, with safe tool filtering.

### Prompt 3: Evaluating Extension Safety
```
I found a Gemini CLI extension for "AI Research Tools" on GitHub.
Before installing, what questions should I ask to evaluate if it's safe?

Specifically about:
- Which MCP servers it includes
- Tool filtering approach (includeTools vs excludeTools)
- What capabilities AI will have
- Whether it's appropriate for beginners
```

**Expected outcome**: Checklist of safety questions to ask before installing any extension.

### Prompt 4: Choosing Between MCP and Extension
```
My study partner shared:
Option A: Install Playwright MCP directly
Option B: Install their "research-bundle" extension (includes Playwright + commands + context)

I'm a beginner just learning AI tools.
Which should I choose and why?

Consider: ease of use, safety, flexibility, learning value.
```

**Expected outcome**: Decision framework comparing direct MCP installation vs extension, tailored to beginner needs.



