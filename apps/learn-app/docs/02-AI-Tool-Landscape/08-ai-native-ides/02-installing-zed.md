---
title: "Installing Zed IDE"
lesson_number: 2
proficiency_level: "A2-B1"
estimated_time: "60-90 minutes"
learning_objectives:
  - "Install Zed IDE on your platform (macOS, Linux, or Windows)"
  - "Authenticate with an AI provider (Anthropic, OpenAI, Google, or Ollama)"
  - "Configure basic settings"
  - "Verify installation by generating code via inline assistant"
---

# Lesson 2: Installing Zed IDE

## Why Zed First?

You will start your AI-native IDE journey with Zed for three main reasons: speed, simplicity, and Anthropic backing.

### 1. Speed

Zed is written in Rust and designed for high performance. You will notice the difference immediately with no lag opening large files, instant AI responses, smooth scrolling with background AI, and startup under 200ms versus VS Code's 1-3 seconds. Speed removes friction from AI-native workflows.

### 2. Simplicity  

Zed has a cleaner interface than VS Code with fewer options, making it easier to learn, with integrated AI features, and the perfect foundation before customization.

### 3. Anthropic Backing

Zed was acquired by Anthropic in 2024, providing tight Claude integration, AI-first philosophy, native inline generation, and alignment with your course.

## What You Will Do in This Lesson

Install Zed on your platform, connect to an AI provider, verify AI assistance, and configure preferences. This uses observational learning - you install and verify but do not code yet.

---

## Part 1: Platform-Specific Installation

::::os-tabs

::macos
Using Homebrew:
```bash
brew install zed
zed
```

Or download from zed.dev/download and drag to Applications.

::linux
Run the install script:
```bash
curl https://zed.dev/install.sh | sh
zed
```

Or use your package manager (apt, dnf, pacman) or AppImage.

::windows
Download .msi from zed.dev/download and run, or:
```powershell
winget install zed
```

::::

---

## Part 2: Initial Setup Wizard

Choose Dark theme, your keymap, font size (14 or 16), and enable AI assistance.

---

## Part 3: AI Provider Authentication

Choose ONE provider:

**Anthropic (Claude)**: Visit console.anthropic.com, create API key, paste in Zed. Cost: $0.03 per 1M input tokens.

**OpenAI**: Visit platform.openai.com, create key, paste in Zed. Choose gpt-4o, gpt-4-turbo, or gpt-3.5-turbo.

**Google (Gemini)**: Visit aistudio.google.com, get API key, paste in Zed. First 15k requests daily free.

**Ollama (Local)**: Install ollama, run ollama pull mistral, start with ollama serve, select Local in Zed.

---

## Part 4: Verification Test

Create test.py file. Type comment about prime checker. Trigger assistant with Cmd+K / Ctrl+K then Cmd+/ / Ctrl+/. Ask for prime function. Accept generated code.

---

## Part 5: Configuration Deep Dive

Settings accessed via Zed menu opens settings.json file. Configure theme, buffer_font_size, tab_size, formatter, and assistant default_model. Recommended settings use One Dark theme, font 14, tab 4, black formatter, Claude model.

---

## Part 6: Troubleshooting Guide

API Key Invalid: Regenerate at provider. Connection Timeout: Check internet, restart Zed, check provider status. Panel Missing: Verify keyboard shortcut or use command palette. Slow: Normal for first request, check model selection.

---

## Part 7: Practice Checklist

Launch Zed, create Python file, open settings.json, trigger assistant, ask question, accept code, change model, find API key. Eight items - all required before Lesson 3.

---

## What You Learned

Installed Zed on your platform, chose and authenticated with AI provider, verified installation, configured settings, and learned troubleshooting basics. You now have working AI-native IDE.

---

## Next Lesson Preview

Lesson 3 covers building real projects, using inline assistant incrementally, experiencing Three Roles collaboration, iterating based on feedback, and specification-driven prompting.

---

## Additional Resources

Zed: zed.dev/docs
Anthropic: docs.anthropic.com
OpenAI: platform.openai.com/docs
Google: ai.google.dev
Ollama: ollama.ai
Community: zed.dev/community
