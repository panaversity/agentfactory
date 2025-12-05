---
sidebar_position: 4
title: "Lesson 1.3: Environment Setup"
description: "Set up your complete development environment with UV, Python, and OpenAI SDK"
keywords: [UV, Python, virtual environment, .env, package manager, development setup]
---

# Lesson 1.3: Environment Setup

**Duration**: 40-50 minutes
**Difficulty**: Beginner (A1-A2)

## Learning Goal

By the end of this lesson, you will:
- Have UV package manager installed on your computer
- Create a properly structured DocuBot project folder
- Configure environment variables securely
- Verify everything works with a simple test

---

## What You'll Learn

Before you can build an AI agent, you need a proper workspace. Think of this like a chef preparing their station before cooking — everything organized, tools ready, ingredients accessible.

You'll learn about:

1. **UV Package Manager**: A modern, fast tool for managing Python projects
2. **Virtual Environments**: Isolated spaces that keep your project's dependencies separate
3. **Environment Variables**: Secure way to store secrets like API keys
4. **Folder Structure**: How to organize your code professionally

:::info Why This Matters
A well-configured environment prevents countless headaches later. Taking 45 minutes now saves hours of debugging mysterious errors.
:::

---

## Key Points

Four things you need to understand:

- **UV vs pip**: UV is like pip (Python's traditional package manager) but 10-100x faster. It also handles Python versions and virtual environments automatically
- **Virtual environments isolate your work**: Each project has its own dependencies, so they don't conflict with other projects
- **Never commit secrets**: API keys go in `.env` files that are NOT uploaded to Git
- **Consistent structure helps everyone**: Following standard folder layouts makes collaboration easier

---

## Simple Analogy

:::tip The Chef's Station Analogy
Imagine a professional kitchen:

**Before service (what we're doing now):**
- Organize ingredients in their proper places (install dependencies)
- Prepare tools and equipment (set up UV and virtual environment)
- Keep recipes accessible but private (store API keys in .env)
- Set up the workstation cleanly (create folder structure)

**Why chefs do this:**
- No hunting for ingredients during rush (no dependency errors during development)
- Clean station = clear mind (organized code = fewer bugs)
- Recipes are valuable secrets (API keys must be protected)

You're setting up your coding kitchen. Once it's ready, you can focus on cooking (building DocuBot).
:::

---

## 🤖 Apply to DocuBot Project

Time to set up your DocuBot workspace! Follow these steps carefully.

### Task

Set up your complete DocuBot development environment:

1. **Install UV** — the modern Python package manager
2. **Create the docubot folder** — your project home
3. **Initialize a UV project** — creates pyproject.toml and virtual environment
4. **Create .env file** — store your OpenAI API key securely
5. **Add the OpenAI package** — install the SDK
6. **Verify everything works** — run the test from Lesson 1.1

### Outcome

*A fully configured project folder with UV, virtual environment, and .env file ready. Running `test_api.py` successfully proves your environment is set up correctly.*

---

## 💡 Hints

Work through these hints step by step. Each builds on the previous one.

<details>
<summary><strong>Hint 1</strong> (Install UV)</summary>

**macOS/Linux:**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**Windows (PowerShell):**
```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

After installation, restart your terminal and verify:
```bash
uv --version
```

You should see something like `uv 0.4.x`.

</details>

<details>
<summary><strong>Hint 2</strong> (Create Project Folder)</summary>

Choose a location for your projects (like Documents or a coding folder):

```bash
# Navigate to where you want the project
cd ~/Documents  # or wherever you prefer

# Create the docubot folder
mkdir docubot

# Go into it
cd docubot
```

</details>

<details>
<summary><strong>Hint 3</strong> (Initialize UV Project)</summary>

Inside the docubot folder:

```bash
uv init
```

This creates:
- `pyproject.toml` — project configuration file
- `.python-version` — specifies Python version
- `hello.py` — sample file (you can delete this)

</details>

<details>
<summary><strong>Hint 4</strong> (Create .env File)</summary>

Create a file called `.env` (note the dot at the start):

```bash
# Create the file
touch .env  # Mac/Linux
# Or on Windows: New-Item .env -ItemType File
```

Open it in a text editor and add:
```
OPENAI_API_KEY=sk-your-actual-key-here
```

Replace `sk-your-actual-key-here` with your real API key from OpenAI.

</details>

<details>
<summary><strong>Hint 5</strong> (Install OpenAI Package)</summary>

Add the OpenAI SDK to your project:

```bash
uv add openai
```

UV will:
- Create a virtual environment automatically
- Install the openai package
- Update pyproject.toml with the dependency
- Create uv.lock for reproducibility

</details>

<details>
<summary><strong>Hint 6</strong> (Test Everything)</summary>

Create your test file and run it:

```bash
# Create test_api.py with the code from Lesson 1.1
# Then run it with UV:
uv run python test_api.py
```

The `uv run` prefix ensures the command runs in your project's virtual environment.

If you see an AI response, congratulations — your environment is ready!

</details>

---

## OS-Specific Instructions

### macOS / Linux

**Installing UV:**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**After installation:**
- Restart your terminal
- Or run: `source ~/.bashrc` (or `~/.zshrc` for Zsh)

**Verifying:**
```bash
uv --version
which uv
```

**Common path issues:**
If `uv` command not found, the install script will tell you what to add to your shell config file.

### Windows

**Installing UV (PowerShell):**
```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**After installation:**
- Close and reopen PowerShell
- Or start a new PowerShell window

**Verifying:**
```powershell
uv --version
```

**Important Windows notes:**
- Always use PowerShell, not Command Prompt
- If you get "execution policy" errors, run PowerShell as Administrator first
- Path separators are `\` not `/`

### Alternative: pip Fallback

If UV installation fails (corporate firewall, etc.), you can use pip:

```bash
# Create virtual environment manually
python -m venv .venv

# Activate it
# Mac/Linux:
source .venv/bin/activate
# Windows:
.venv\Scripts\activate

# Install OpenAI
pip install openai
```

UV is preferred, but pip works fine for learning.

---

## Troubleshooting

### UV Installation Failed

**Corporate firewall blocking the download:**
- Ask IT if they can whitelist `astral.sh`
- Or use the pip fallback method above

**Permission denied:**
- Don't use `sudo` with UV installation
- The installer puts UV in your home directory

### Python Version Issues

**"Python 3.11+ required" error:**
```bash
# Let UV install Python for you:
uv python install 3.11

# Or check what you have:
python --version
```

### .env Not Loading

**Symptoms:** API key errors even though .env looks correct.

**Checklist:**
1. File is named `.env` (not `env` or `.env.txt`)
2. File is in the project root (same folder as pyproject.toml)
3. No quotes around the value: `OPENAI_API_KEY=sk-xxx` (not `"sk-xxx"`)
4. No spaces around the `=`

**Test if it's loading:**
```python
import os
print(os.getenv("OPENAI_API_KEY"))  # Should print your key
```

### Virtual Environment Not Activating

UV handles this automatically with `uv run`. If you need manual activation:

```bash
# Mac/Linux:
source .venv/bin/activate

# Windows:
.venv\Scripts\activate
```

Your prompt should change to show `(.venv)` at the start.

---

## Verification Checklist

Before moving on, confirm each of these:

- [ ] **UV is installed**: `uv --version` shows a version number
- [ ] **Project folder exists**: You have a `docubot` folder
- [ ] **UV project initialized**: `pyproject.toml` file exists in the folder
- [ ] **.env file created**: Contains your `OPENAI_API_KEY=sk-...`
- [ ] **OpenAI package installed**: `uv add openai` completed successfully
- [ ] **Test works**: `uv run python test_api.py` shows an AI response

If all boxes are checked, your environment is ready for agent development!

---

## Project Structure

After completing this lesson, your docubot folder should look like this:

```
docubot/
├── .env                 # Your API key (NEVER commit this!)
├── .python-version      # Python version for this project
├── pyproject.toml       # Project configuration and dependencies
├── uv.lock             # Locked dependency versions
└── test_api.py         # Your API verification script
```

### What Each File Does

| File | Purpose |
|------|---------|
| `.env` | Stores secrets (API keys). Listed in .gitignore |
| `.python-version` | Tells UV which Python version to use |
| `pyproject.toml` | Lists dependencies and project metadata |
| `uv.lock` | Ensures everyone gets exact same package versions |
| `test_api.py` | Your first working code (from Lesson 1.1) |

---

## Try With AI

:::tip Practice with ChatGPT
Use [ChatGPT](https://chat.openai.com) to troubleshoot any issues:

1. Copy the exact error message you're seeing
2. Mention your operating system (Windows/Mac/Linux)
3. Ask for step-by-step help

Example: "I'm on Windows 11 and getting this error when running `uv add openai`: [paste error]. How do I fix this?"

ChatGPT is great at debugging environment issues!
:::

---

## What's Next

Congratulations! You now have:
- ✅ UV package manager installed
- ✅ A properly structured docubot project
- ✅ Secure API key configuration
- ✅ All dependencies installed
- ✅ Verified working environment

In the final lesson of this chapter, you'll see the big picture — the complete architecture of what you'll build across all 16 chapters.

---

**Next**: [Lesson 1.4 — Architecture Overview](./04-architecture-overview.md)
