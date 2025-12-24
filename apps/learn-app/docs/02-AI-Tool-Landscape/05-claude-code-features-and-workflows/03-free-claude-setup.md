---
title: "Free Claude Code Setup with Google Gemini"
sidebar_position: 3
chapter: 5
lesson: 3
duration_minutes: 15

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 1"
layer_progression: "L1 (Manual Foundation - Alternative Path)"
layer_1_foundation: "API-based architecture setup, environment configuration, backend routing"
layer_2_collaboration: "N/A"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA
skills:
  - name: "Alternative Claude Code Backend Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can configure Claude Code to use alternative AI backends via API routing, understand the architecture of backend abstraction, and evaluate trade-offs between official and alternative setups"

learning_objectives:
  - objective: "Understand API-based architecture where frontend (Claude Code CLI) separates from backend (AI model)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of three-layer architecture (CLI → Router → API)"
  - objective: "Configure Claude Code Router to translate API formats between Anthropic and OpenAI standards"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful router configuration with Google Gemini backend"
  - objective: "Set up environment variables for secure API key management"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "API key stored as environment variable, not hardcoded"
  - objective: "Verify alternative setup produces identical Claude Code functionality"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Completion of same verification tasks as Lesson 2 (official setup)"
  - objective: "Evaluate trade-offs between official subscription and free API backend"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Articulation of when each approach is appropriate"

# Cognitive load tracking
cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (API routing, backend abstraction, format translation, environment variables, free tier limits, router configuration, daily workflow, architecture layers, trade-off evaluation) - within B1 limit of 10 ✓"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Configure multiple backends (Gemini + local Ollama), implement custom routing logic, monitor API usage patterns"
  remedial_for_struggling: "Focus on copy-paste setup first, understand architecture later; verify it works before understanding why"

# Generation metadata
generated_by: "AI-Native Software Development Curriculum Team"
source_spec: "Educational accessibility initiative"
created: "2025-11-20"
last_modified: "2025-12-24"
version: "2.0.0"

# Legacy compatibility
prerequisites:
  - "Lesson 1: Understanding Claude Code paradigm"
  - "Node.js 18+ installed"
  - "Free Google Account"
  - "Terminal access"
---

# Free Claude Code Setup with Google Gemini

**This lesson provides a free alternative to use Claude Code** using Google's free Gemini API as the backend. You'll learn the same Claude Code CLI interface and features covered in Lesson 2.

**All features work identically**: Subagents, skills, MCP servers, hooks, and all other capabilities covered in Lessons 04-12 function the same way with this free setup. The only difference is the backend AI model (Gemini instead of Claude) and the setup process (router configuration instead of direct authentication).

:::tip Free Ongoing Usage
By using **Gemini's free tier** or **OpenRouter's free models**, you get ongoing free consumption—no subscription required. This setup isn't just for learning; many developers use it as their daily driver. The free tiers are generous enough for real development work.
:::

---

## Step 1: Get Your Free Google API Key

1. Go to: [Google AI Studio](https://aistudio.google.com/api-keys)
2. Click **"Get API Key"**
3. Sign in with Google
4. Click **"Create API Key"**
5. **Copy the key** (looks like: `AIzaSyAaBbCcDd...`)

---

## Step 2: Install and Configure

**Select your operating system:**

::::os-tabs

::windows

### Step 0: Install Node.js (Skip if Already Installed)

**Check if you have Node.js:**

Open **PowerShell** (search "PowerShell" in Windows Start menu) and type:

```powershell
node --version
```

- **If you see `v18.x.x` or higher** → Skip to Step 1 ✅
- **If you see an error or version lower than v18** → Follow these steps:

1. Go to: [nodejs.org](https://nodejs.org/)
2. Click the big green button that says **"Download Node.js (LTS)"**
3. Run the downloaded file (it's called something like `node-v20.x.x-x64.msi`)
4. Click **Next** → **Next** → **Next** → **Install**
5. Wait for it to finish
6. **Close ALL PowerShell windows completely**
7. Open a **new PowerShell** window
8. Type `node --version` again to confirm it works

You should now see a version number like `v20.11.0` ✅

---

### Step 1: Install Tools

Open PowerShell and run:

```powershell
npm install -g @anthropic-ai/claude-code @musistudio/claude-code-router
```

---

### Step 2: Create Config Directories

```powershell
New-Item -ItemType Directory -Force -Path "$env:USERPROFILE\.claude-code-router"
New-Item -ItemType Directory -Force -Path "$env:USERPROFILE\.claude"
```

---

### Step 3: Create the Config File

1. Open **Notepad** (search "Notepad" in Windows Start menu)
2. Copy and paste this text exactly as-is:

```json
{
  "LOG": true,
  "LOG_LEVEL": "info",
  "HOST": "127.0.0.1",
  "PORT": 3456,
  "API_TIMEOUT_MS": 600000,
  "Providers": [
    {
      "name": "gemini",
      "api_base_url": "https://generativelanguage.googleapis.com/v1beta/models/",
      "api_key": "$GOOGLE_API_KEY",
      "models": [
        "gemini-2.5-flash-lite",
        "gemini-2.0-flash"
      ],
      "transformer": {
        "use": ["gemini"]
      }
    }
  ],
  "Router": {
    "default": "gemini,gemini-2.5-flash-lite",
    "background": "gemini,gemini-2.5-flash-lite",
    "think": "gemini,gemini-2.5-flash-lite",
    "longContext": "gemini,gemini-2.5-flash-lite",
    "longContextThreshold": 60000
  }
}
```

:::warning Do NOT Change $GOOGLE_API_KEY
Leave `"api_key": "$GOOGLE_API_KEY"` exactly as written. Do NOT replace it with your actual key here—the router will automatically read your key from the environment variable you set in Step 4.
:::

3. Click **File → Save As**
4. In the "File name" field, type exactly: `%USERPROFILE%\.claude-code-router\config.json`
5. Click **Save**

---

### Step 4: Set Your API Key

**Run PowerShell as Administrator:**
1. Search "PowerShell" in Windows Start menu
2. **Right-click** on "Windows PowerShell"
3. Click **"Run as administrator"**
4. Click "Yes" if prompted

Run this command (replace `YOUR_KEY_HERE` with your actual API key from Step 1):

```powershell
[System.Environment]::SetEnvironmentVariable('GOOGLE_API_KEY', 'YOUR_KEY_HERE', 'User')
```

5. **Close PowerShell completely** (not just the tab—close the whole window)
6. Open a **new regular PowerShell** (not as admin this time)
7. Verify it worked:

```powershell
echo $env:GOOGLE_API_KEY
```

You should see your API key displayed ✅

---

### ✅ Verify Setup

```powershell
claude --version     # Should show: Claude Code v2.x.x
ccr version          # Should show version number
echo $env:GOOGLE_API_KEY  # Should show your key
```

✅ **Done!** Proceed to Step 3: Daily Workflow below.

::macos

### Verify Node.js

```bash
node --version  # Should show v18.x.x or higher
```

If missing, install from [nodejs.org](https://nodejs.org/)

---

### Copy-Paste Setup

Copy and paste this entire block into Terminal:

```bash
# Install tools
npm install -g @anthropic-ai/claude-code @musistudio/claude-code-router

# Create config directories
mkdir -p ~/.claude-code-router ~/.claude

# Create router config
cat > ~/.claude-code-router/config.json << 'EOF'
{
  "LOG": true,
  "LOG_LEVEL": "info",
  "HOST": "127.0.0.1",
  "PORT": 3456,
  "API_TIMEOUT_MS": 600000,
  "Providers": [
    {
      "name": "gemini",
      "api_base_url": "https://generativelanguage.googleapis.com/v1beta/models/",
      "api_key": "$GOOGLE_API_KEY",
      "models": [
        "gemini-2.5-flash-lite",
        "gemini-2.0-flash"
      ],
      "transformer": {
        "use": ["gemini"]
      }
    }
  ],
  "Router": {
    "default": "gemini,gemini-2.5-flash-lite",
    "background": "gemini,gemini-2.5-flash-lite",
    "think": "gemini,gemini-2.5-flash-lite",
    "longContext": "gemini,gemini-2.5-flash-lite",
    "longContextThreshold": 60000
  }
}
EOF

# Verify file was created
cat ~/.claude-code-router/config.json
```

---

### Set Your API Key

Replace `YOUR_KEY_HERE` with your actual API key:

```bash
# For zsh (default on macOS):
echo 'export GOOGLE_API_KEY="YOUR_KEY_HERE"' >> ~/.zshrc
source ~/.zshrc
```

---

### ✅ Verify Setup

```bash
claude --version     # Should show: Claude Code v2.x.x
ccr version          # Should show version number
echo $GOOGLE_API_KEY # Should show your key
```

✅ **Done!** Proceed to Step 3: Daily Workflow below.

::linux

### Verify Node.js

```bash
node --version  # Should show v18.x.x or higher
```

If missing, install from [nodejs.org](https://nodejs.org/) or use your package manager.

---

### Copy-Paste Setup

Copy and paste this entire block into your terminal:

```bash
# Install tools
npm install -g @anthropic-ai/claude-code @musistudio/claude-code-router

# Create config directories
mkdir -p ~/.claude-code-router ~/.claude

# Create router config
cat > ~/.claude-code-router/config.json << 'EOF'
{
  "LOG": true,
  "LOG_LEVEL": "info",
  "HOST": "127.0.0.1",
  "PORT": 3456,
  "API_TIMEOUT_MS": 600000,
  "Providers": [
    {
      "name": "gemini",
      "api_base_url": "https://generativelanguage.googleapis.com/v1beta/models/",
      "api_key": "$GOOGLE_API_KEY",
      "models": [
        "gemini-2.5-flash-lite",
        "gemini-2.0-flash"
      ],
      "transformer": {
        "use": ["gemini"]
      }
    }
  ],
  "Router": {
    "default": "gemini,gemini-2.5-flash-lite",
    "background": "gemini,gemini-2.5-flash-lite",
    "think": "gemini,gemini-2.5-flash-lite",
    "longContext": "gemini,gemini-2.5-flash-lite",
    "longContextThreshold": 60000
  }
}
EOF

# Verify file was created
cat ~/.claude-code-router/config.json
```

---

### Set Your API Key

Replace `YOUR_KEY_HERE` with your actual API key:

```bash
# For bash:
echo 'export GOOGLE_API_KEY="YOUR_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
```

:::tip Check Your Shell
Run `echo $SHELL` to see your shell. If it shows `/bin/zsh`, use `~/.zshrc` instead of `~/.bashrc`.
:::

---

### ✅ Verify Setup

```bash
claude --version     # Should show: Claude Code v2.x.x
ccr version          # Should show version number
echo $GOOGLE_API_KEY # Should show your key
```

✅ **Done!** Proceed to Step 3: Daily Workflow below.

::::

---

## Step 3: Daily Workflow

**Every time you want to code:**

::::os-tabs

::windows

**PowerShell 1** - Start router FIRST:
```powershell
ccr start
```

Leave this window running. You'll see a warning message—that's normal!

**PowerShell 2** - Open a NEW PowerShell window and run:
```powershell
cd C:\your\project\folder
ccr code
```

:::tip First Startup Takes Time
**Wait 10-20 seconds** after running `ccr code` on first startup. The router needs time to initialize. If it seems stuck, just wait—it's working!
:::

**When done:** Press `Ctrl+C` in both windows.

::macos

**Terminal 1** - Start router FIRST:
```bash
ccr start
# Wait for: ✅ Service started successfully
```

**Terminal 2** - THEN use Claude:
```bash
cd ~/your-project
ccr code
```

**When done:** Press `Ctrl+C` in both terminals.

::linux

**Terminal 1** - Start router FIRST:
```bash
ccr start
# Wait for: ✅ Service started successfully
```

**Terminal 2** - THEN use Claude:
```bash
cd ~/your-project
ccr code
```

**When done:** Press `Ctrl+C` in both terminals.

::::

---

## Verification

**Start a Claude session and say hi:**

```
hi
```

**Expected**: Claude responds with a greeting confirming it's working! ✅ Success!

---

## Alternative: DeepSeek Setup

**DeepSeek offers another free alternative** with competitive pricing and strong coding capabilities.

### Get Your DeepSeek API Key

1. Go to: [DeepSeek API Platform](https://platform.deepseek.com/)
2. Sign up or log in with your account
3. Navigate to **API Keys** section
4. Click **"Create API Key"**
5. **Copy the key** (looks like: `sk-...`)

### DeepSeek Configuration

::::os-tabs

::windows

:::info Already Completed Gemini Setup?
If you completed the Windows Gemini setup above, you already have Node.js and the tools installed. Just create the new config and set the API key.
:::

**Step 1**: Create the config file. Open **Notepad** and paste:

```json
{
  "LOG": true,
  "LOG_LEVEL": "info",
  "HOST": "127.0.0.1",
  "PORT": 3456,
  "API_TIMEOUT_MS": 600000,
  "Providers": [
    {
      "name": "deepseek",
      "api_base_url": "https://api.deepseek.com/v1",
      "api_key": "$DEEPSEEK_API_KEY",
      "models": [
        "deepseek-chat",
        "deepseek-reasoner"
      ],
      "transformer": {
        "use": ["openai"]
      }
    }
  ],
  "Router": {
    "default": "deepseek,deepseek-chat",
    "background": "deepseek,deepseek-chat",
    "think": "deepseek,deepseek-reasoner",
    "longContext": "deepseek,deepseek-chat",
    "longContextThreshold": 60000
  }
}
```

:::warning Do NOT Change $DEEPSEEK_API_KEY
Leave `"api_key": "$DEEPSEEK_API_KEY"` exactly as written.
:::

Save as: `%USERPROFILE%\.claude-code-router\config.json`

**Step 2**: Set your API key (Run PowerShell as Administrator):

```powershell
[System.Environment]::SetEnvironmentVariable('DEEPSEEK_API_KEY', 'YOUR_KEY_HERE', 'User')
```

Close and reopen PowerShell, then verify:

```powershell
echo $env:DEEPSEEK_API_KEY
```

::macos

**Create config and set API key:**

```bash
cat > ~/.claude-code-router/config.json << 'EOF'
{
  "LOG": true,
  "LOG_LEVEL": "info",
  "HOST": "127.0.0.1",
  "PORT": 3456,
  "API_TIMEOUT_MS": 600000,
  "Providers": [
    {
      "name": "deepseek",
      "api_base_url": "https://api.deepseek.com/v1",
      "api_key": "$DEEPSEEK_API_KEY",
      "models": [
        "deepseek-chat",
        "deepseek-reasoner"
      ],
      "transformer": {
        "use": ["openai"]
      }
    }
  ],
  "Router": {
    "default": "deepseek,deepseek-chat",
    "background": "deepseek,deepseek-chat",
    "think": "deepseek,deepseek-reasoner",
    "longContext": "deepseek,deepseek-chat",
    "longContextThreshold": 60000
  }
}
EOF

# Set your API key
echo 'export DEEPSEEK_API_KEY="YOUR_KEY_HERE"' >> ~/.zshrc
source ~/.zshrc
```

::linux

**Create config and set API key:**

```bash
cat > ~/.claude-code-router/config.json << 'EOF'
{
  "LOG": true,
  "LOG_LEVEL": "info",
  "HOST": "127.0.0.1",
  "PORT": 3456,
  "API_TIMEOUT_MS": 600000,
  "Providers": [
    {
      "name": "deepseek",
      "api_base_url": "https://api.deepseek.com/v1",
      "api_key": "$DEEPSEEK_API_KEY",
      "models": [
        "deepseek-chat",
        "deepseek-reasoner"
      ],
      "transformer": {
        "use": ["openai"]
      }
    }
  ],
  "Router": {
    "default": "deepseek,deepseek-chat",
    "background": "deepseek,deepseek-chat",
    "think": "deepseek,deepseek-reasoner",
    "longContext": "deepseek,deepseek-chat",
    "longContextThreshold": 60000
  }
}
EOF

# Set your API key
echo 'export DEEPSEEK_API_KEY="YOUR_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
```

::::

The daily workflow is identical—use `ccr start` and `ccr code` as shown above.

---

## Troubleshooting

::::os-tabs

::windows

**"command not found" or "not recognized"**

Close and reopen PowerShell completely. If still failing, the npm global bin directory isn't in your PATH.

**"API key not found" or empty variable**

1. Make sure you ran the `SetEnvironmentVariable` command as Administrator
2. Close ALL PowerShell windows and open a fresh one
3. Check with `echo $env:GOOGLE_API_KEY`

**Stuck at "starting service"**

Wait 20-30 seconds on first run. This is normal.

**Router starts but Claude hangs**

Make sure `ccr start` is running in PowerShell 1 before running `ccr code` in PowerShell 2.

::macos

**"command not found: claude" or "command not found: ccr"**

The npm global bin directory isn't in your PATH:

```bash
npm config get prefix
echo 'export PATH="$PATH:/usr/local/bin"' >> ~/.zshrc
source ~/.zshrc
```

**"API key not found" or empty GOOGLE_API_KEY**

```bash
echo $GOOGLE_API_KEY
# If empty, re-add:
echo 'export GOOGLE_API_KEY="YOUR_KEY_HERE"' >> ~/.zshrc
source ~/.zshrc
```

**Router starts but Claude hangs**

Wait 2-3 seconds after `ccr start` shows "Service started" before running `ccr code`.

::linux

**"command not found: claude" or "command not found: ccr"**

The npm global bin directory isn't in your PATH:

```bash
npm config get prefix
echo 'export PATH="$PATH:/usr/local/bin"' >> ~/.bashrc
source ~/.bashrc
```

**"API key not found" or empty GOOGLE_API_KEY**

```bash
echo $GOOGLE_API_KEY
# If empty, re-add:
echo 'export GOOGLE_API_KEY="YOUR_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
```

**Router starts but Claude hangs**

Wait 2-3 seconds after `ccr start` shows "Service started" before running `ccr code`.

::::

---

## Try With AI

Once your free setup is working, try these prompts to verify everything works:

**Verify Basic Functionality:**

> "Hello! Confirm you're working by telling me: (1) what model you're using, (2) can you see files in this directory? List them if so."

**Test File Operations:**

> "Create a simple test file called `hello.txt` with the text 'Free Claude Code setup works!' Then read it back to confirm."

**Understand the Architecture:**

> "Explain the architecture of my current setup: I'm using Claude Code CLI with a router pointing to Gemini. What's happening when I send you a message? Walk me through the request flow."

---

That's it. Proceed to **Lesson 04** to learn about teaching Claude your way of working.
