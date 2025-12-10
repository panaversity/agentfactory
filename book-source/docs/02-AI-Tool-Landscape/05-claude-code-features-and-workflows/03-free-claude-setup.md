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
last_modified: "2025-11-20"
version: "1.0.0"

# Legacy compatibility
prerequisites:
  - "Lesson 1: Understanding Claude Code paradigm"
  - "Node.js 18+ installed"
  - "Free Google Account"
  - "Terminal access"
---

# Extension: Free Claude Code Setup with Google Gemini

**This lesson provides a free alternative to use Claude Code** using Google's free Gemini API as the backend. You'll learn the same Claude Code CLI interface and features covered in Lesson 2.

**All features work identically**: Subagents, skills, MCP servers, hooks, and all other capabilities covered in Lessons 3-9 function the same way with this free setup. The only difference is the backend AI model (Gemini instead of Claude) and the setup process (router configuration instead of direct authentication).

---

🎥 Video TutorialFull step-by-step video (with voice) : 👉 https://www.youtube.com/watch?v=HQ6dqd7QY38

---

## Reality Check: It's Just Copy-Paste

**Setup Complexity**: Copy 3 text blocks, type 3 commands. That's it.

**What you need**:
- Node.js 18+ ([nodejs.org](https://nodejs.org/))
- Free Google Account
- 5 minutes

**Verify Node.js** (if unsure):
```bash
node --version  # Should show v18.x.x or higher
```

If missing, install from [nodejs.org](https://nodejs.org/)

---

## 🔥 STEP 1 — Get Your Free Google API Key

1. Go to: [Google AI Studio](https://aistudio.google.com/api-keys)
2. Click **"Get API Key"**
3. Sign in with Google
4. Click **"Create API Key"**
5. **Copy the key** (looks like: `AIzaSyAaBbCcDd...`)

---

## 🔥 STEP 2 — INSTALL CLAUDE TOOLS
### Windows (PowerShell or CMD):
```powershell
npm install -g @anthropic-ai/claude-code @musistudio/claude-code-router
```
### macOS / Linux:
```bash
sudo npm install -g @anthropic-ai/claude-code @musistudio/claude-code-router
```

## 🔥 STEP 3 — CREATE REQUIRED FOLDERS
### Windows PowerShell:
```powershell
New-Item -ItemType Directory -Force -Path $HOME\.claude-code-router
New-Item -ItemType Directory -Force -Path $HOME\.claude
```
### Windows CMD:
```cmd
mkdir %USERPROFILE%\.claude-code-router
mkdir %USERPROFILE%\.claude
```
### macOS / Linux:
```bash
mkdir -p ~/.claude-code-router ~/.claude
```
*`-Force` / `-p` = no error if folder already exists*

## 🔥 STEP 4 — CREATE `config.json` (Correct For Each OS)
### 🟦 Windows (PowerShell + CMD) — Use Notepad
```powershell
notepad $HOME\.claude-code-router\config.json
```
Or in CMD:
```cmd
notepad %USERPROFILE%\.claude-code-router\config.json
```
Paste this exact JSON:
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
        "gemini-1.5-flash",
        "gemini-1.5-flash-exp-0827"
      ],
      "transformer": {
        "use": ["gemini"]
      }
    }
  ],
  "Router": {
    "default": "gemini,gemini-1.5-flash",
    "background": "gemini,gemini-1.5-flash",
    "think": "gemini,gemini-1.5-flash",
    "longContext": "gemini,gemini-1.5-flash",
    "longContextThreshold": 60000
  }
}
```
Save & close.

### 🟩 macOS / Linux
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
      "name": "gemini",
      "api_base_url": "https://generativelanguage.googleapis.com/v1beta/models/",
      "api_key": "$GOOGLE_API_KEY",
      "models": [
        "gemini-1.5-flash",
        "gemini-1.5-flash-exp-0827"
      ],
      "transformer": {
        "use": ["gemini"]
      }
    }
  ],
  "Router": {
    "default": "gemini,gemini-1.5-flash",
    "background": "gemini,gemini-1.5-flash",
    "think": "gemini,gemini-1.5-flash",
    "longContext": "gemini,gemini-1.5-flash",
    "longContextThreshold": 60000
  }
}
EOF
```

## 🔥 STEP 5 — SET GOOGLE API KEY
### Windows PowerShell (Permanent):
```powershell
[System.Environment]::SetEnvironmentVariable('GOOGLE_API_KEY', 'YOUR_KEY_HERE', 'User')
```
→ Restart PowerShell, then check:
```powershell
echo $env:GOOGLE_API_KEY
```

### macOS / Linux
#### Bash:
```bash
echo 'export GOOGLE_API_KEY="YOUR_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
```
#### Zsh (default on macOS):
```bash
echo 'export GOOGLE_API_KEY="YOUR_KEY_HERE"' >> ~/.zshrc
source ~/.zshrc
```

## 🔥 STEP 6 — VERIFY INSTALLATION
```bash
claude --version
ccr version
```
For macOS/Linux:
```bash
echo $GOOGLE_API_KEY
```
For Windows:
```powershell
echo $env:GOOGLE_API_KEY
```
All should show output → Ready!

## 🔥 STEP 7 — DAILY USAGE
### Terminal 1 — Start the router:
```bash
ccr start
```
Wait for:
```
✔ Service started successfully
or
⚠️ API key is not set. HOST is forced to 127.0.0.1.
Loaded JSON config from: C:\Users\User\.claude-code-router\config.json
```

### Terminal 2 — Start Coding (Cross-Platform)
#### macOS / Linux Option 1:
```bash
ccr code
```
#### macOS / Linux Option 2 (Recommended):
```bash
# Activate Claude environment
eval "$(ccr activate)"

# Start Claude CLI
claude
```

#### Windows CMD:
```cmd
set ANTHROPIC_AUTH_TOKEN=test
set ANTHROPIC_API_KEY=
set ANTHROPIC_BASE_URL=http://127.0.0.1:3456
set NO_PROXY=127.0.0.1
set DISABLE_TELEMETRY=true
set DISABLE_COST_WARNINGS=true
set API_TIMEOUT_MS=600000
```
```cmd
claude
```

#### Windows PowerShell:
```powershell
$env:ANTHROPIC_AUTH_TOKEN = "test"
$env:ANTHROPIC_API_KEY = ""
$env:ANTHROPIC_BASE_URL = "http://127.0.0.1:3456"
$env:NO_PROXY = "127.0.0.1"
$env:DISABLE_TELEMETRY = "true"
$env:DISABLE_COST_WARNINGS = "true"
$env:API_TIMEOUT_MS = "600000"
```
```powershell
claude
```
**Test it:**  
Type `hi` → Claude should reply → 🎉 Success!

---

## ⚠️ Troubleshooting
| Problem | Solution |
| :-------------------- | :---------------------------------------------------------------------- |
| `mkdir` error | Use `-Force` (Windows) or `-p` (macOS/Linux) with `New-Item` / `mkdir` |
| "API key not found" | Restart terminal after setting environment variable |
| `ccr: command not found` | Close & reopen terminal, or restart computer |
| Port 3456 already in use | Kill process: `taskkill //F //PID <pid>` (Win) or `kill -9 <pid>` (Unix) |


That's it. If Claude responds, your free setup is working perfectly.

---

## Alternative: DeepSeek Setup

**DeepSeek offers another free alternative** with competitive pricing and strong coding capabilities. Follow these steps to configure DeepSeek as your backend.

### Step 1: Get Your DeepSeek API Key

1. Go to: [DeepSeek API Platform](https://platform.deepseek.com/)
2. Sign up or log in with your account
3. Navigate to **API Keys** section
4. Click **"Create API Key"**
5. **Copy the key** (looks like: `sk-...`)

---

### Step 2: DeepSeek Copy-Paste Setup

**Just copy commands from this block and paste into terminal:**

```bash
# Install tools (if not already installed)
npm install -g @anthropic-ai/claude-code @musistudio/claude-code-router

# Create config directories (if not already created)
mkdir -p ~/.claude-code-router ~/.claude

# Create router config with DeepSeek endpoint
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

# Verify file was created
cat ~/.claude-code-router/config.json

# Set your API key (REPLACE "YOUR_KEY_HERE" with actual key!)
echo 'export DEEPSEEK_API_KEY="YOUR_KEY_HERE"' >> ~/.zshrc
source ~/.zshrc
```

**Windows users**: Replace last 2 lines with:
```powershell
# Windows PowerShell (Run as Administrator)
[System.Environment]::SetEnvironmentVariable('DEEPSEEK_API_KEY', 'YOUR_KEY_HERE', 'User')

# Then CLOSE and REOPEN PowerShell completely
# Verify it worked:
echo $env:DEEPSEEK_API_KEY
```

**Bash users** (older macOS/Linux):
```bash
# Check your shell first:
echo $SHELL

# If shows /bin/zsh → use ~/.zshrc (already done above)
# If shows /bin/bash → Change last 2 lines to:
echo 'export DEEPSEEK_API_KEY="YOUR_KEY_HERE"' >> ~/.bashrc
source ~/.bashrc
```

---

### ✅ Verify DeepSeek Setup Worked

**After pasting setup commands, verify immediately:**

```bash
claude --version        # Should show: Claude Code v2.x.x
ccr version             # Should show version number
echo $DEEPSEEK_API_KEY  # Should show your key (not empty!)

# If any fail, see Troubleshooting section
```

✅ **Done!** DeepSeek setup is complete.

**Note**: The daily workflow (Step 3) and verification steps are identical - just use `ccr start` and `ccr code` as shown above. DeepSeek will work with all the same Claude Code features.

---

That's it. Proceed to **Lesson 3** to learn persistent project context.

