---
sidebar_position: 8
chapter: 6
lesson: 8
duration_minutes: 35
title: "Gemini CLI Extensions"
cefr_level: A2
proficiency: Beginner
teaching_stage: 2
stage_name: "AI Collaboration"
cognitive_load:
  concepts_count: 6
  a2_compliant: true
learning_objectives:
  - id: LO1
    description: "Explain what extensions are and why they simplify sharing complete setups"
    bloom_level: "Understand"
    digcomp: "1.3 Managing data, information and digital content"
  - id: LO2
    description: "Install extensions from GitHub repositories using gemini extensions install"
    bloom_level: "Apply"
    digcomp: "3.1 Developing digital content"
  - id: LO3
    description: "Manage installed extensions using list, update, disable, and uninstall commands"
    bloom_level: "Apply"
    digcomp: "1.2 Evaluating data, information and digital content"
  - id: LO4
    description: "Understand extension structure (gemini-extension.json, commands/, GEMINI.md)"
    bloom_level: "Understand"
    digcomp: "1.3 Managing data, information and digital content"
  - id: LO5
    description: "Create a basic extension using gemini extensions new command"
    bloom_level: "Create"
    digcomp: "3.2 Integrating and re-elaborating digital content"
  - id: LO6
    description: "Evaluate when extensions are useful versus individual component setup"
    bloom_level: "Evaluate"
    digcomp: "5.2 Identifying needs and technological responses"
---

# Gemini CLI Extensions

You've built the perfect Gemini CLI setup over the past few weeks: a security MCP server for vulnerability analysis, custom security analysis commands, and a GEMINI.md file with your security review procedures. Your security team member asks: "Can I get your setup?" You realize you'd need to send them a 15-step installation guide. In this lesson, you'll learn how extensions package everything into a single installable bundle—turning complex setup sharing into one command.

---

## The Problem: Setup Sharing is Complex

Imagine you've crafted an effective Gemini CLI security analysis environment. You want to share it with your security team (5 people).

### What Your Setup Contains

```
Your Gemini CLI Setup:
├── MCP Servers
│   └── securityServer (for vulnerability analysis)
├── Custom Commands
│   ├── /security:analyze (analyze code for vulnerabilities)
│   └── /security:analyze-github-pr (analyze GitHub PRs)
├── GEMINI.md (security analysis guidelines and procedures)
└── Settings (configuration preferences)
```

### The Manual Sharing Process

**What you'd need to send your security team member**:

```markdown
SETUP GUIDE - FOLLOW CAREFULLY

Step 1: Install security MCP server
Run: gemini mcp add securityServer
[configure MCP server path]

Step 2: Create commands directory
Create folder: .gemini/commands/security/

Step 3: Download analyze.toml
Copy this code → save as .gemini/commands/security/analyze.toml
[paste 50 lines of TOML]

Step 4: Download analyze-github-pr.toml
Copy this code → save as .gemini/commands/security/analyze-github-pr.toml
[paste 60 lines of TOML]

Step 5: Create GEMINI.md
Copy this → save as .gemini/GEMINI.md
[paste 200 lines of security analysis context]

Step 6: Restart Gemini CLI
Run: gemini
```

### What Goes Wrong

Reality check for your 5 security team members:

**Person 1**: Misses Step 2 (creates commands in wrong folder) → commands don't work
**Person 2**: Copies TOML with formatting errors → syntax errors
**Person 3**: Forgets to configure the MCP server → security analysis fails
**Person 4**: Completes all steps but has older MCP versions → inconsistent behavior
**Person 5**: Succeeds but it takes 45 minutes → frustrated

**Two weeks later**: You improve the `/security:analyze` command. Now you need to send an update guide to all 5 people.

**The problems**:
- ❌ **Time-consuming**: 15+ manual steps per person
- ❌ **Error-prone**: Easy to miss steps or make typos
- ❌ **Version drift**: Everyone ends up with slightly different setups
- ❌ **Update nightmare**: Sharing improvements requires resending everything
- ❌ **Lost productivity**: Your team spends time on setup instead of security analysis

---

## The Solution: Extensions Package Everything

What if your security team members could get your entire setup with one command?

```bash
gemini extensions install https://github.com/gemini-cli-extensions/security
```

**That's exactly what extensions do.**

### What is a Gemini CLI Extension?

An **extension** is a **complete package** containing:

- ✅ **MCP servers** (with configurations)
- ✅ **Custom slash commands** (all `.toml` files)
- ✅ **Persistent context** (GEMINI.md files)
- ✅ **Settings** (environment variables, preferences)
- ✅ **Documentation** (how to use everything)

**Think of it like this**:
- **MCP server alone**: Single capability (e.g., web browsing)
- **Custom command alone**: One reusable prompt
- **Extension**: Complete, shareable environment (MCP servers + commands + context + settings)

### The Transformation

**Before extensions** (manual setup):
```
Time: 45 minutes per person
Errors: 60% of people hit issues
Updates: Resend entire guide
Result: Frustration, inconsistency
```

**With extensions**:
```bash
gemini extensions install https://github.com/gemini-cli-extensions/security
```

```
Time: 2 minutes per person
Errors: Near zero (automated installation)
Updates: gemini extensions update gemini-cli-security
Result: Everyone has identical, working setup
```

**The transformation**: 45 minutes → 2 minutes (22x faster), near-zero errors, instant updates.

---

## Part 1: Working with Extensions

**Important**: All extension management commands (install, uninstall, update, enable, disable) will only be reflected in active CLI sessions after you restart Gemini CLI.

### Installing Extensions

Extensions can be installed from GitHub repositories or local directories.

#### Install from GitHub

The most common way to install extensions:

```bash
gemini extensions install https://github.com/username/extension-name
```

**What happens**:
1. Downloads the extension from GitHub
2. Installs all MCP servers listed in the extension
3. Adds custom commands to your Gemini CLI
4. Loads GEMINI.md context files
5. Applies settings and configurations

**Example - Installing the Security extension to enhance your repository's security posture**:
```bash
gemini extensions install https://github.com/gemini-cli-extensions/security
```

**Success message**:
```
Installing extension "gemini-cli-security".
**The extension you are about to install may have been created by a third-party developer and sourced from a public repository. Google does not vet, endorse, or guarantee the functionality or security of extensions. Please carefully inspect any extension and its source code before installing to understand the permissions it requires and the actions it may perform.**
This extension will run the following MCP servers:
  * securityServer (local): node C:\Users\A\AppData\Local\Temp\gemini-extensionTJGjh7/mcp-server/dist/security.js
This extension will append info to your gemini.md context using GEMINI.md
Do you want to continue? [Y/n]: Y
Extension "gemini-cli-security" installed successfully and enabled.
```

#### Install from Local Directory

If you're developing an extension or have it downloaded locally:

```bash
gemini extensions install /path/to/extension-directory
```

**Note**: This creates a copy in your extensions directory. If you make changes to the original directory, you'll need to run `gemini extensions update <name>` to pull in those changes.

#### Advanced Installation Options

**Install specific version**:
```bash
gemini extensions install https://github.com/gemini-cli-extensions/security --ref v0.3.0
```

**Enable auto-updates**:
```bash
gemini extensions install https://github.com/gemini-cli-extensions/security --auto-update
```

**Include pre-releases**:
```bash
gemini extensions install https://github.com/gemini-cli-extensions/security --pre-release
```

**Skip confirmation prompt**:
```bash
gemini extensions install https://github.com/gemini-cli-extensions/security --consent
```

### Listing Installed Extensions

See all extensions you currently have:

```bash
gemini extensions list
```

**Example output**:
```
✓ context7 (1.0.0)
 ID: 16a7277d62eea8e1bcc1a9a47e65b76f3d3aae8bae5f630b8bc028116b09ea25
 name: 0472116c93c00c7ca34dc790f805a3fd637a5a02f690d03f3d427bf273eb6492
 Path: C:\Users\A\.gemini\extensions\context7
 Source: https://github.com/upstash/context7 (Type: git)
 Enabled (User): true
 Enabled (Workspace): true
 MCP servers:
  context7

✓ gemini-cli-security (0.3.0)
 ID: 3cd3f2199bc2138259a06d9fcff3e3c9e5bd0150ddf64c051097080eebe88f29
 name: b8fcb80afdae08dc5e5c0d5b2ae222172e85337818d98e22d04c59328fdc8a58
 Path: C:\Users\A\.gemini\extensions\gemini-cli-security
 Source: https://github.com/gemini-cli-extensions/security (Type: github-release)
 Release tag: v0.3.0
 Enabled (User): true
 Enabled (Workspace): true
 Context files:
  C:\Users\A\.gemini\extensions\gemini-cli-security\GEMINI.md
 MCP servers:
  securityServer
```

**You can also list extensions from within Gemini CLI**:
```
/extensions list
```

This shows the same information in your active session.

**Note**: Extension management commands (install, uninstall, update, enable, disable) are not supported from within the CLI. You must use them from your terminal. Only the `/extensions list` command is available inside the CLI.

### Updating Extensions

#### Update Single Extension

```bash
gemini extensions update gemini-cli-security
```

**What this does**:
- Checks for new version in the original source (GitHub/local)
- Downloads the latest version
- Updates MCP servers, commands, and context
- Shows what changed

**Example output**:
```
Extension "gemini-cli-security" is already up to date.
```

**When to update**:
- Extension creator announces new features
- Bug fixes are released
- MCP servers need updating
- You see "Update available" message

#### Update All Extensions

```bash
gemini extensions update --all
```

Updates every installed extension to the latest version.

### Disabling Extensions

Sometimes you want to temporarily disable an extension without uninstalling it.

#### Disable Everywhere

```bash
gemini extensions disable gemini-cli-security
```

The extension stays installed but won't load in any session.

#### Disable in Current Workspace Only

From inside your project directory:

```bash
gemini extensions disable gemini-cli-security --scope workspace
```

**Scope options**: `user` (default, disables everywhere) or `workspace` (disables only in current workspace)

**Use case**: "I want the security extension for my personal projects, but not when working on client projects that have their own security review process."

### Enabling Extensions

Re-enable a disabled extension:

```bash
gemini extensions enable gemini-cli-security
```

Or enable just for current workspace:

```bash
gemini extensions enable gemini-cli-security --scope workspace
```

**Scope options**: `user` (default, enables everywhere) or `workspace` (enables only in current workspace)

### Uninstalling Extensions

Remove an extension completely:

```bash
gemini extensions uninstall gemini-cli-security
```

**What gets removed**:
- ✓ The extension directory
- ✓ MCP servers installed by the extension (if not used elsewhere)
- ✓ Custom commands from the extension

**What stays**:
- ✓ Any work you created using the extension
- ✓ Your personal GEMINI.md files (not part of the extension)
- ✓ Your settings.json configurations

---

## Part 2: Understanding Extension Structure

When you install an extension, it creates a directory with a specific structure. Let's understand what's inside.

### Extension Directory Layout

```
gemini-cli-security/
├── gemini-extension.json    ← Configuration (what's included)
├── GEMINI.md                 ← Context for AI (optional)
├── commands/                 ← Custom slash commands
│   ├── analyze-github-pr.toml
│   ├── analyze.toml
├── docs/                     ← Documentation (optional)
│   └── README.md

```

### The Configuration File: `gemini-extension.json`

This is the heart of every extension. It tells Gemini CLI what to install and how.

**Basic example**:
```json
{
  "name": "gemini-cli-security",
  "version": "0.3.0",
  "contextFileName": "GEMINI.md",
  "mcpServers": {
    "securityServer": {
      "command": "node",
      "args": [
        "${extensionPath}/mcp-server/dist/security.js"
      ]
    }
  }
}
```

**What each part means**:

**`name`** (required): The extension's unique identifier
- Must be lowercase with dashes (not spaces or underscores)
- How users refer to the extension
- **Important**: This name should match the extension directory name
- Example: `"gemini-cli-security"` not `"Gemini Cli Security"`

**`version`** (required): Current version number
- Format: `"1.0.0"` (major.minor.patch)
- Used for update checking
- Increment when you make changes

**`description`** (optional): What the extension does
- Brief explanation for users
- Shows in extension lists

**`mcpServers`** (optional): MCP servers to install
- Each server has a name (key) and configuration (value)
- Server configuration includes:
  - `command`: How to run the server
  - `args`: Arguments to pass
  - `cwd`: Working directory (optional)

**`contextFileName`** (optional): Context file to load
- Defaults to `GEMINI.md` if present
- If this property is not used but a `GEMINI.md` file is present in your extension directory, that file will be loaded automatically

**`excludeTools`** (optional): Array of tool names to exclude from the model
- You can specify command-specific restrictions for tools that support it
- Example: `"excludeTools": ["run_shell_command(rm -rf)"]` will block the `rm -rf` command
- Note: This differs from MCP server `excludeTools` functionality, which can be listed in the MCP server config

### MCP Server Configuration

Extensions can include multiple MCP servers:

```json
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["@playwright/mcp@latest"]
    },
    "context7": {
      "command": "npx",
      "args": ["context7"]
    },
    "filesystem": {
      "command": "node",
      "args": ["${extensionPath}${/}server${/}filesystem.js"],
      "cwd": "${extensionPath}"
    }
  }
}
```

**Using variables**:
- `${extensionPath}`: The fully-qualified path of the extension in the user's filesystem (e.g., `/Users/username/.gemini/extensions/example-extension`). This will not unwrap symlinks.
- `${workspacePath}`: The fully-qualified path of the current workspace
- `${/}` or `${pathSeparator}`: The path separator (differs per OS: `/` on Mac/Linux, `\` on Windows)

**Example variable usage**:
```json
"args": ["${extensionPath}${/}dist${/}server.js"]
```

On Mac/Linux becomes: `/Users/you/.gemini/extensions/ai-kit/dist/server.js`
On Windows becomes: `C:\Users\you\.gemini\extensions\ai-kit\dist\server.js`

### Custom Commands

Extensions automatically include any `.toml` files in the `commands/` directory.

**Flat structure**:
```
commands/
├── analyze.toml           → /analyze
└── scan.toml              → /scan
```

**Nested structure** (as used in gemini-cli-security):
```
commands/
└── security/
    ├── analyze.toml              → /security:analyze
    └── analyze-github-pr.toml    → /security:analyze-github-pr
```

**Command naming**:
- Top-level files: `/commandname`
- Nested in folders: `/folder:commandname`

### The GEMINI.md Context File

Extensions can include a `GEMINI.md` file that provides persistent context to the AI.

**Example - Security extension context (from gemini-cli-security)**:
```markdown
# Standard Operating Procedures: Security Analysis Guidelines

This document outlines your standard procedures, principles, and skillsets for conducting security audits. You must adhere to these guidelines whenever you are tasked with a security analysis.

## Persona and Guiding Principles

You are a highly skilled senior security engineer. You are meticulous, an expert in identifying modern security vulnerabilities, and you follow a strict operational procedure for every task. You MUST adhere to these core principles:

*   **Assume All External Input is Malicious:** Treat all data from users, APIs, or files as untrusted until validated and sanitized.
*   **Principle of Least Privilege:** Code should only have the permissions necessary to perform its function.
*   **Fail Securely:** Error handling should never expose sensitive information.

## Skillset: SAST Vulnerability Analysis

When you need to do a security audit, you will methodically check for:
- Hardcoded secrets and credentials
- Broken access control (IDOR, privilege escalation)
- Insecure data handling (weak crypto, PII violations)
- Injection vulnerabilities (SQLi, XSS, command injection)
- Authentication weaknesses
- LLM safety issues (prompt injection, unsafe output handling)
```

**When this loads**: The AI will follow these guidelines automatically when you use extension commands like `/security:analyze`.

### Extension Settings

**Note**: This is an experimental feature. We do not yet recommend extension authors introduce settings as part of their core flows.

Extensions can request settings from users during installation.

**In `gemini-extension.json`**:
```json
{
  "settings": [
    {
      "name": "API Key",
      "description": "Your API key for the documentation service",
      "envVar": "DOCS_API_KEY",
      "sensitive": false
    }
  ]
}
```

**On installation**, Gemini CLI prompts:
```
Extension 'doc-tools' requires settings:

API Key: Your API key for the documentation service
Enter value: [user types key]

✓ Settings saved to ~/.gemini/extensions/doc-tools/.env
```

**The `.env` file**:
```bash
DOCS_API_KEY=your-key-here
```

---

## Part 3: Creating Your First Extension

Now that you understand extensions, let's create one. Gemini CLI includes templates to get started quickly.

### Using the Template Generator

Gemini CLI provides example templates to start from:

```bash
gemini extensions new <extension-name> [template]
```

**Available templates**:
- `context`: Extension with just a GEMINI.md file
- `custom-commands`: Extension with example commands
- `mcp-server`: Extension with an MCP server
- `exclude-tools`: Extension that restricts tool access

### Creating a Simple Security Extension

Let's create an extension for security analysis workflow.

**Step 1: Generate the template**

```bash
gemini extensions new my-security-extension custom-commands
```

**What this creates**:
```
my-security-extension/
├── gemini-extension.json
├── commands/
│   └── example.toml
└── package.json
```

**Navigate to the directory**:
```bash
cd my-security-extension
```

**Step 2: Edit the configuration**

Open `gemini-extension.json`:

```json
{
  "name": "my-security-tools",
  "version": "1.0.0",
  "description": "Personal security analysis commands"
}
```

**Step 3: Create your first command**

Create the commands directory structure:

```bash
mkdir -p commands/security
```

Create `commands/security/check-secrets.toml`:

```toml
description = "Check for hardcoded secrets in code"

prompt = """
Analyze the codebase for hardcoded secrets and credentials.

Search Results:
!{grep -r "API_KEY\|_SECRET\|password\|PRIVATE_KEY" . --include="*.js" --include="*.ts" --include="*.py" || echo "No matches found"}

Please summarize the findings for the pattern above.
Look for:
1. API keys (patterns like API_KEY, _SECRET)
2. Passwords and private keys
3. Database connection strings
4. Base64-encoded credentials

For each finding, provide:
- File path and line number
- Type of secret detected
- Severity (Critical/High/Medium/Low)
- Recommendation for remediation
"""
```

**Note**: The `!{command}` syntax in the prompt executes shell commands and pipes the results into the prompt. This allows you to dynamically gather information before the model processes it.

**Step 4: Test locally**

Link the extension for testing:

```bash
gemini extensions link .
```

Restart Gemini CLI:
```bash
gemini
```

Try your new command:
```
/security:check-secrets
```

**Step 5: Add more commands**

Create `commands/security/scan-injection.toml`:

```toml
description = "Scan for injection vulnerabilities"

prompt = """
Scan the codebase for injection vulnerabilities including:
1. SQL injection (unparameterized queries)
2. XSS (unsanitized user input in HTML)
3. Command injection (user input in shell commands)
4. Path traversal (unsanitized file paths)

For each vulnerability:
- Identify the source (where user input enters)
- Trace to the sink (where it's executed/rendered)
- Check for sanitization/validation
- Provide remediation guidance
"""
```

**Step 6: Add a GEMINI.md context file**

You can provide persistent context to the model by adding a `GEMINI.md` file to your extension. This is useful for giving the model instructions on how to behave or information about your extension's tools.

Create a file named `GEMINI.md` in the root of your extension directory:

```markdown
# Security Extension Instructions

You are a security analysis assistant. When the user asks you to check for secrets
or scan for vulnerabilities, use the appropriate security commands. Be thorough
and provide actionable recommendations.
```

Update your `gemini-extension.json` to tell the CLI to load this file:

```json
{
  "name": "my-security-tools",
  "version": "1.0.0",
  "description": "Personal security analysis commands",
  "contextFileName": "GEMINI.md"
}
```

Restart the CLI again. The model will now have the context from your `GEMINI.md` file in every session where the extension is active.

**Step 7: Share with others**

Once you're happy with your extension:

1. Create a GitHub repository
2. Push your extension files
3. Share the install command:
   ```bash
   gemini extensions install https://github.com/yourname/my-security-tools
   ```

### Creating an Extension with MCP Server

Let's create a more advanced extension that includes an MCP server.

**Step 1: Generate MCP template**

```bash
gemini extensions new my-security-extension mcp-server
```

**What you get**:
```
my-security-extension/
├── gemini-extension.json
├── example.ts              ← TypeScript MCP server code
├── package.json
└── tsconfig.json
```

**Navigate to the directory**:
```bash
cd my-security-extension
```

**Step 2: Review the configuration**

`gemini-extension.json`:
```json
{
  "name": "my-security-extension",
  "version": "1.0.0",
  "mcpServers": {
    "nodeServer": {
      "command": "node",
      "args": ["${extensionPath}${/}dist${/}example.js"],
      "cwd": "${extensionPath}"
    }
  }
}
```

**Step 3: Build the server**

Install dependencies:
```bash
npm install
```

Build the server:
```bash
npm run build
```

This compiles `example.ts` → `dist/example.js`

**Step 4: Link and test**

Link the extension:
```bash
gemini extensions link .
```

Restart Gemini CLI and the MCP server will be available. You can test it by asking the model to use the tools provided by your MCP server.

**Step 5: Add a GEMINI.md context file (optional)**

You can provide persistent context to the model by adding a `GEMINI.md` file to your extension.

Create a file named `GEMINI.md` in the root of your extension directory:

```markdown
# Security Extension Instructions

You are a security analysis assistant. When using the security tools from this
extension, follow these guidelines:

- Be thorough in your analysis
- Provide actionable recommendations
- Classify vulnerabilities by severity (Critical/High/Medium/Low)
```

Update your `gemini-extension.json` to include the context file:

```json
{
  "name": "my-security-extension",
  "version": "1.0.0",
  "contextFileName": "GEMINI.md",
  "mcpServers": {
    "nodeServer": {
      "command": "node",
      "args": ["${extensionPath}${/}dist${/}example.js"],
      "cwd": "${extensionPath}"
    }
  }
}
```

Restart the CLI again. The model will now have the context from your `GEMINI.md` file in every session where the extension is active.

### Updating Your Extension

When you make changes to a linked extension:

**Option 1: Already linked** (changes apply immediately after restart)
```bash
# Edit your files
# Restart Gemini CLI - changes are live
gemini
```

**Option 2: Installed (not linked)**
```bash
gemini extensions update my-security-tools
```

---

## Part 4: When to Use Extensions

Extensions are powerful, but not always necessary. Here's when they make sense.

### ✅ Use Extensions When:

**Sharing with multiple people**:
- Security team needs same analysis tools (5+ people)
- Organization providing standard security environment for developers
- Team wants consistent security tooling
- Open source project recommending security configurations

**Example**: "Our security team uses the gemini-cli-security extension with custom security commands. Instead of a 20-step setup guide, team members run one install command."

**Complex setup with many components**:
- 3+ MCP servers
- 5+ custom commands
- Specific context requirements
- Environment settings needed

**Example**: "My security analysis setup has the securityServer MCP, multiple security scanning commands, and security analysis guidelines in GEMINI.md. Extension packages it all."

**Need version control and updates**:
- Setup will improve over time
- Users need updates easily
- Want to track changes
- Need rollback capability

**Example**: "I improve my security analysis commands every week. Extension users get updates with one command instead of re-downloading files."

**Collaboration with specific context**:
- AI needs domain-specific instructions
- Special prompting guidelines
- Consistent behavior across team
- Shared knowledge base

**Example**: "Our security team needs AI to follow specific vulnerability assessment procedures and severity classification. The extension's GEMINI.md ensures consistency across all security reviews."

### ❌ Don't Use Extensions When:

**Simple, exploratory setup**:
- Trying one MCP server
- Testing a single command
- Experimenting with configurations
- Personal, changing workflow

**Example**: "I'm trying different security MCP servers to see which I like. I'll install them individually for now."

**Solo user, no sharing**:
- Only you will use it
- No collaboration planned
- Setup rarely changes
- Don't need version control

**Example**: "These security commands are just for me and I change them frequently based on my current security analysis needs."

**Temporary/one-time use**:
- One-off project
- Short-term experiment
- Testing before committing
- No maintenance needed

**Example**: "I need the security extension for this one security audit project. I'll install it directly."

---

## Common Questions

### Can I use multiple extensions together?

Yes! Extensions compose nicely:

```bash
gemini extensions install https://github.com/gemini-cli-extensions/security
gemini extensions install https://github.com/user/code-helpers
gemini extensions install https://github.com/org/research-kit
```

All three work simultaneously, combining:
- All MCP servers from each
- All commands from each
- All context files (merged)

### What if extensions conflict?

**Command name conflicts**:
Extension commands have the lowest precedence. When a conflict occurs with user or project commands:

- **No conflict**: Extension command uses its natural name (e.g., `/security:analyze`)
- **With conflict**: Extension command is renamed with the extension prefix (e.g., `/gemini-cli-security.analyze`)

For example, if both a user and the `gemini-cli-security` extension define an `analyze` command:
- `/analyze` - Executes the user's analyze command
- `/gemini-cli-security.analyze` - Executes the extension's analyze command (marked with `[gemini-cli-security]` tag in help)

You can also:
- Disable one extension
- Ask extension creator to rename command

**MCP server conflicts**:
If two extensions configure the same MCP server name, your `settings.json` wins (takes precedence over extensions).

### How do updates work?

**For GitHub extensions**:
```bash
gemini extensions update extension-name
```
- Checks GitHub for new version
- Downloads if version number increased
- Applies changes automatically

**For local extensions**:
```bash
gemini extensions update extension-name
```
- Copies latest files from original location
- Updates if `version` field increased

**Auto-update** (if enabled during install):
- Checks for updates on Gemini CLI startup
- Prompts to update if new version available

### How do I know what an extension does?

Before installing:

1. **Visit the GitHub repository**
2. **Read the README.md** (extension documentation)
3. **Check `gemini-extension.json`**:
   - What MCP servers it installs
   - What commands it provides
   - What context it loads

**Ask AI** (if you trust the source):
```
I'm considering installing this extension: [URL]
What capabilities would it add to you? What MCP servers and commands are included?
```

### Can I modify an extension after installing?

Extensions install to `~/.gemini/extensions/extension-name/`

**But**: Changes get overwritten on update.

**Better approach**:
1. **Fork the extension** on GitHub
2. **Make your changes** in your fork
3. **Install from your fork**:
   ```bash
   gemini extensions install https://github.com/yourname/forked-extension
   ```

Or:

1. **Keep extension as-is**
2. **Add personal overrides** in workspace:
   - Project-specific GEMINI.md (adds to extension context)
   - Additional commands in `.gemini/commands/`
   - Settings in workspace `settings.json`

---

## Troubleshooting

### "Extension installation failed"

**Check**:
1. Is the GitHub URL correct?
   - Format: `https://github.com/username/repo`
   - Not: `github.com/username/repo` (missing https://)
2. Is the repository public or do you have access?
3. Is your internet connection working?
4. Do you have `git` installed?

**Install git if needed**:
- Mac: `brew install git`
- Ubuntu: `sudo apt install git`
- Windows: Download from git-scm.com

**Try again**:
```bash
gemini extensions install <url> --consent
```

### "Extension installed but commands not working"

**Check**:
1. Did you restart Gemini CLI after installing?
   ```bash
   # Exit current session, start new one
   gemini
   ```

2. Is the extension enabled?
   ```bash
   gemini extensions list
   # Look for "Status: Enabled"
   ```

3. Are you typing the command correctly?
   ```bash
   /security:analyze    ← Correct
   /security:analyze Analyze all files in src/    ← With arguments
   security:analyze     ← Won't work (missing /)
   ```

4. Check available commands:
   ```
   /help
   # Scroll to bottom - extension commands listed
   ```

### "MCP server from extension not available"

**Restart required**: MCP servers load on startup, not mid-session.

```bash
# Exit and restart
gemini
```

**Check installation**:
```
/extensions list
```

Look for "MCP Servers: [server-name]"

**Check MCP server itself**:
```bash
gemini mcp list
```

Should show the MCP server from the extension.

### "Extension update does nothing"

**Version must increase**: Update only works if the extension's `version` field increased.

**Check current version**:
```bash
gemini extensions list
```

**Check if source has newer version**: Look at the extension's `gemini-extension.json` on GitHub.

**Force reinstall**:
```bash
gemini extensions uninstall extension-name
gemini extensions install <url>
```

---

## Try With AI

Now let's practice working with extensions collaboratively. These prompts guide you through understanding, evaluating, and creating extensions with AI as your co-learner.

### 🔍 Understanding Extension Structure

Use this to explore what's inside an extension and how it works:

> "I'm looking at a Gemini CLI extension repository. Here's its `gemini-extension.json`:
>
> ```json
> {
>   "name": "research-tools",
>   "version": "2.0.0",
>   "mcpServers": {
>     "playwright": {
>       "command": "npx",
>       "args": ["@playwright/mcp@latest"]
>     },
>     "context7": {
>       "command": "npx",
>       "args": ["context7"]
>     }
>   },
>   "contextFileName": "RESEARCH_GUIDE.md"
> }
> ```
>
> Walk me through what this extension provides:
> 1. What MCP servers will be installed?
> 2. What capabilities will you gain?
> 3. What happens with RESEARCH_GUIDE.md?
> 4. Is there anything I should verify before installing?
>
> Then, help me understand: if I install this and later find an extension with better research tools, can I uninstall this one safely?"

**Expected outcome**: Clear explanation of extension components, installation effects, and management implications.

**Three Roles in action**:
- 🎓 **AI teaches**: Explains each part of the configuration
- 📚 **You teach AI**: Provide your specific use case (research needs)
- 🤝 **Co-work**: Together evaluate if this extension suits your needs

---

### 🛠️ Creating Your Personal Extension

Use this to design and build a custom extension for your workflow:

> "I want to create a personal learning extension for studying Python. My workflow includes:
> - Researching topics on official Python docs
> - Creating study plans for new concepts
> - Generating practice exercises
> - Reviewing code examples
>
> Help me design an extension step-by-step:
>
> **Step 1**: What should I name it (remember naming rules)?
>
> **Step 2**: Which MCP servers would be useful? (I'm thinking Playwright for browsing docs and Context7 for getting latest documentation)
>
> **Step 3**: What custom commands should I include? Help me design 3 commands:
> - One for research (with web browsing)
> - One for study planning
> - One for practice exercises
>
> **Step 4**: What should go in my GEMINI.md context file to make you better at helping me learn Python?
>
> **Step 5**: Write the complete `gemini-extension.json` file for me.
>
> **Step 6**: Write one of the command `.toml` files (your choice of which would be most useful).
>
> As we go, explain your choices so I understand the design decisions."

**Expected outcome**: Complete, functional extension design tailored to your learning needs with explanations for each component.

**Three Roles in action**:
- 🎓 **AI teaches**: Suggests best practices for extension design
- 📚 **You teach AI**: Describe your specific learning workflow and preferences
- 🤝 **Co-work**: Iteratively refine the extension design together

---

### 📊 Evaluating Extension vs. Manual Setup

Use this to decide whether creating an extension makes sense for your situation:

> "I have a decision to make. Here's my situation:
>
> **Current setup**:
> - 2 MCP servers (Playwright, filesystem)
> - 4 custom commands I created
> - A GEMINI.md file with my learning preferences
>
> **Context**:
> - I'm in a study group with 3 other people
> - We meet weekly and work on similar projects
> - We often ask each other "how did you configure that?"
> - We're all learning together (beginners)
>
> **Question**: Should I create an extension for my study group, or is it overkill?
>
> Help me think through:
> 1. What are the benefits of creating an extension for us?
> 2. What are the downsides or complexity costs?
> 3. How much time would it save the group?
> 4. What's the alternative approach?
> 5. Given we're beginners, what would you recommend and why?
>
> Give me a clear recommendation with reasoning."

**Expected outcome**: Thoughtful analysis of trade-offs with a clear recommendation based on your specific context.

**Three Roles in action**:
- 🎓 **AI teaches**: Explains decision framework for extensions
- 📚 **You teach AI**: Provide context about your group and skill levels
- 🤝 **Co-work**: Together evaluate the best path forward

---

### 🚀 Publishing and Sharing Your Extension

Use this when you're ready to share your extension with others:

> "I've created a Gemini CLI extension for Python learning called 'python-study-tools'. It's working great on my machine and I want to share it with my study group (5 people).
>
> Walk me through the publishing process:
>
> **Part 1**: How do I prepare it for sharing?
> - What files must be included?
> - What should I document?
> - How do I test that it will work for others?
>
> **Part 2**: How do I publish it?
> - Do I need to create a GitHub repository?
> - What should the README include?
> - Are there any specific repository settings needed?
>
> **Part 3**: How do my study partners install it?
> - What exact command should I give them?
> - What should they expect to see?
> - How do they verify it worked?
>
> **Part 4**: How do updates work?
> - When I improve the extension, how do I release updates?
> - How do they get the updates?
> - What's the version numbering convention?
>
> Give me a step-by-step checklist I can follow."

**Expected outcome**: Complete publishing workflow from preparation through ongoing maintenance, with specific commands and best practices.

**Three Roles in action**:
- 🎓 **AI teaches**: Explains GitHub workflow and extension distribution
- 📚 **You teach AI**: Describe what you want users to experience
- 🤝 **Co-work**: Create documentation and installation instructions together

---

### 🔧 Debugging Extension Installation

Use this when you or others encounter installation problems:

> "My study partner is trying to install my extension but getting errors. Here's what happened:
>
> **What they ran**:
> ```bash
> gemini extensions install https://github.com/myname/python-study-tools
> ```
>
> **Error message**:
> ```
> Error: Failed to install extension 'python-study-tools'
> Could not find gemini-extension.json in repository
> ```
>
> **What I need**:
> 1. What does this error mean?
> 2. What probably went wrong?
> 3. How do I check my repository to verify the structure?
> 4. What should my study partner do now?
> 5. How can I prevent this for other people?
>
> Also, create a troubleshooting checklist I can add to my README so others can self-diagnose common issues."

**Expected outcome**: Root cause analysis, immediate fix, and preventive measures documentation.

**Three Roles in action**:
- 🎓 **AI teaches**: Explains common extension installation errors
- 📚 **You teach AI**: Provide specific error details and repository structure
- 🤝 **Co-work**: Debug together and create user-facing documentation

---

## What You've Learned

In this lesson, you've discovered how Gemini CLI extensions transform setup sharing:

**Core Concepts**:
- ✅ **Extensions package complete environments**: MCP servers + commands + context + settings in one bundle
- ✅ **Installation is simple**: One command (`gemini extensions install`) vs. 15-step manual setup
- ✅ **Management is straightforward**: List, update, disable, enable, and uninstall with clear commands
- ✅ **Structure is understandable**: `gemini-extension.json` + `commands/` + `GEMINI.md` + optional settings
- ✅ **Creation is accessible**: Templates (`gemini extensions new`) help you start quickly
- ✅ **Sharing is powerful**: GitHub distribution enables instant setup for teams and study groups

**Practical Skills**:
- Installing extensions from GitHub repositories
- Managing multiple extensions (enabling/disabling per workspace)
- Understanding extension configurations and structure
- Creating basic extensions using templates
- Deciding when extensions are valuable vs. overkill

**Key Insight**:
Extensions aren't just about bundling—they're about **maintaining consistency** across collaborators and **evolving setups** without resending files. The transformation from 45-minute manual setup to 2-minute installation isn't just about speed; it's about removing friction from learning and collaboration.
