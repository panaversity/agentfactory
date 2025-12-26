---
title: "Installing Cursor IDE"
lesson_number: 4
proficiency_level: "A2-B1"
estimated_time: "60-90 minutes"
learning_objectives:
  - "Install Cursor IDE on your platform (macOS, Linux, or Windows)"
  - "Migrate settings and extensions from VS Code (optional)"
  - "Authenticate with an AI provider (Cursor Pro or bring-your-own-key)"
  - "Configure .cursorrules for project context"
  - "Verify installation with inline code generation"
---

# Lesson 4: Installing Cursor IDE

## Why Cursor After Zed?

You've now experienced Zed, a Rust-based IDE built from scratch for AI. Cursor represents a different approach: taking VS Code's proven architecture and rebuilding it with AI-first design.

### Three Reasons to Learn Cursor Now

**1. Familiarity If You Know VS Code**

If you've used VS Code before, Cursor's interface will feel immediately familiar. This means you can focus on learning AI-native features instead of learning a new editor layout. If you haven't used VS Code, don't worry—the interface is intuitive.

**2. VS Code Extensions Support**

Cursor can use extensions from the VS Code marketplace. If you've built a workflow in VS Code, you can migrate it to Cursor without starting from scratch. This bridges the gap between traditional and AI-native development.

**3. Different AI Feature Set**

While Zed excels at speed and simplicity, Cursor's strength is **Agent mode**—AI that can autonomously write code across multiple files, showing you a diff-based review before applying changes. This is fundamentally different from Zed's inline assistant approach.

## What You Will Do in This Lesson

Install Cursor on your platform, optionally migrate VS Code settings, authenticate with an AI provider, and verify that inline code generation works. Like Lesson 2, this is observational learning—you install, configure, and verify.

---

## Part 1: Platform-Specific Installation

::::os-tabs

::macos
**Option 1: Download from Website**

1. Visit [cursor.com/download](https://cursor.com/download)
2. Download the macOS version (choose Apple Silicon if you have M1/M2/M3, Intel if you have older Mac)
3. Open the .dmg file
4. Drag Cursor.app to the Applications folder
5. Launch Cursor from Applications

**Option 2: Homebrew (if installed)**

```bash
brew install --cask cursor
cursor
```

Whichever you choose, Cursor will launch and show a welcome screen.

::linux
**Option 1: Using AppImage (works on most Linux distributions)**

```bash
# Download AppImage
curl -L https://cursor.sh/linux/appimage -o cursor.AppImage
chmod +x cursor.AppImage

# Run it
./cursor.AppImage

# Optional: Move to /usr/local/bin for system-wide access
sudo mv cursor.AppImage /usr/local/bin/cursor
```

**Option 2: .deb Package (Debian/Ubuntu)**

```bash
# Download and install
curl -L https://cursor.sh/linux/deb -o cursor.deb
sudo apt install ./cursor.deb

# Launch
cursor
```

**Option 3: Package Manager (if available)**

Some Linux distributions may have Cursor in their package managers. Check your distribution's repository, but downloading directly from cursor.com is most reliable.

::windows
**Option 1: Download Installer**

1. Visit [cursor.com/download](https://cursor.com/download)
2. Download the Windows .exe installer
3. Run the installer and follow the setup wizard
4. Cursor will launch automatically after installation

**Option 2: Using Winget (Windows Package Manager)**

```powershell
winget install cursor
cursor
```

::::

**Verification**: On all platforms, Cursor should launch, showing a welcome screen with options for theme selection and keyboard shortcuts.

---

## Part 2: Initial Setup Wizard

Cursor's setup wizard walks you through basic configuration:

### Step 1: Welcome Screen

Choose your experience level: "Beginner," "Intermediate," or "Advanced." This affects UI complexity and default settings. Choose what matches your experience.

### Step 2: Theme Selection

Choose between:
- **Light**: VS Code Light+ (default, good for many)
- **Dark**: VS Code Dark+ (reduces eye strain in low light)
- **High Contrast**: For accessibility

Pick whichever you prefer. You can change themes anytime in Settings.

### Step 3: Keyboard Shortcuts

Cursor offers several keymap profiles:

- **Cursor Default**: Cursor's own shortcuts (optimized for AI features)
- **VS Code**: Familiar if you've used VS Code before
- **Vim**: If you use Vim keybindings
- **Emacs**: If you prefer Emacs-style shortcuts

**Recommendation**: Choose "VS Code" if you know VS Code, otherwise "Cursor Default" works fine.

### Step 4: Font and Appearance

Set your preferred font size (14 is common, 16 for larger screens, 12 for smaller). You can adjust this later in Settings.

---

## Part 3: Optional—Migrating from VS Code

If you've used VS Code and want to bring your settings with you:

### What Gets Migrated

**Automatically migrated**:
- User settings (font size, theme, tab size, etc.)
- Keybindings customizations
- Installed extensions (VS Code extensions work in Cursor)

**NOT migrated**:
- Workspace-specific settings (.vscode/settings.json in projects)
- Some extension configurations (depends on the extension)
- Snippets (may need manual copy)

### How to Migrate

On Cursor's welcome screen, look for "Import from VS Code" or similar option. Click it, and Cursor will:
1. Find your VS Code settings folder
2. Import themes, keybindings, and extensions
3. Show you what was imported

**Manual migration** (if automatic import doesn't work):

VS Code stores settings in:
- **macOS**: `~/Library/Application\ Support/Code/User/settings.json`
- **Linux**: `~/.config/Code/User/settings.json`
- **Windows**: `%APPDATA%\Code\User\settings.json`

Copy this file to Cursor's settings location:
- **macOS**: `~/Library/Application\ Support/Cursor/User/settings.json`
- **Linux**: `~/.config/Cursor/User/settings.json`
- **Windows**: `%APPDATA%\Cursor\User\settings.json`

### Verifying Migration

After import:
1. Open Cursor's Settings (Cmd+, or Ctrl+,)
2. Check that your preferred theme is active
3. Open a project and verify extensions are installed (Extensions sidebar, Cmd+Shift+X or Ctrl+Shift+X)
4. Test a keyboard shortcut you use frequently

---

## Part 4: AI Provider Authentication

Cursor offers two authentication paths:

### Option 1: Cursor Pro Subscription

**What it includes**:
- Built-in Claude (Anthropic)
- GPT-4 Turbo and GPT-4o (OpenAI)
- 50 daily premium requests (unlimited basic requests)
- Agent mode access

**Cost**: $20/month

**Setup**:
1. Click the Cursor icon in the top-right corner of the editor
2. Select "Subscribe to Cursor Pro"
3. Follow the web flow to create an account and payment method
4. Return to Cursor—you're authenticated

**After subscribing**, Cursor automatically selects Claude as your default model (configurable in settings).

### Option 2: Bring Your Own Key (BYOK)

Use API keys from providers you already subscribe to:

**Anthropic (Claude)**:
1. Visit [console.anthropic.com](https://console.anthropic.com)
2. Create or copy an existing API key
3. In Cursor, go to Settings (Cmd+, / Ctrl+,)
4. Search for "Anthropic API key"
5. Paste your key
6. Cost: ~$0.03 per 1M input tokens (much cheaper than Pro subscription)

**OpenAI (GPT-4)**:
1. Visit [platform.openai.com/api/keys](https://platform.openai.com/api/keys)
2. Create or copy an API key
3. In Cursor Settings, search for "OpenAI API key"
4. Paste your key
5. Choose your model: gpt-4o, gpt-4-turbo, or gpt-3.5-turbo

**Google (Gemini)**:
1. Visit [aistudio.google.com](https://aistudio.google.com)
2. Get an API key (first 15,000 requests daily are free)
3. In Cursor Settings, search for "Google API key"
4. Paste your key

**Local LLM (Ollama)**:
1. Install [ollama.ai](https://ollama.ai)
2. Run: `ollama pull mistral` (or another model)
3. Start Ollama: `ollama serve`
4. In Cursor, select "Local" as your provider
5. Cursor will connect to localhost:11434

### Which Option?

- **Use Cursor Pro** if you want convenience and don't mind the monthly fee
- **Use BYOK** if you already have API credits with Anthropic or OpenAI
- **Use Ollama** if you want everything running locally (good for privacy, but slower)

---

## Part 5: .cursorrules—Project-Level Context

Unlike Zed, Cursor supports `.cursorrules`, a special file that tells Cursor about your project's coding standards and preferences.

### What It Is

`.cursorrules` is a plain text file placed in your project root that provides context to Cursor's AI. Think of it as "coding guidelines for the AI to follow."

### Where to Create It

In the root directory of any project, create a file named `.cursorrules` (no extension, just like .gitignore).

```bash
# Navigate to project root
cd ~/projects/my-project

# Create .cursorrules
touch .cursorrules
```

### Simple Example

Open `.cursorrules` in Cursor and add:

```
You are helping with a Python data analysis project.

Technical stack:
- Python 3.11+
- pandas for data manipulation
- matplotlib for visualization
- Jupyter notebooks for exploratory analysis

Coding standards:
- Follow PEP 8 style guide
- Add docstrings to all functions using Google style
- Use type hints for function parameters and returns
- Write unit tests for data processing functions
- Use meaningful variable names (never single letters except in loops)

Project context:
- Data files are in ./data/ directory
- Results go in ./output/ directory
- Notebooks are in ./notebooks/ directory
```

### How Cursor Uses It

When you ask Cursor to generate code (Cmd+K or Ctrl+K), it reads `.cursorrules` and uses that context. So your prompt "Create a function to load CSV data" becomes internally "Create a Python 3.11+ function with docstrings and type hints that loads CSV data using pandas."

### Advanced Example

For a JavaScript web project:

```
You are helping with a React/Next.js web application.

Technology stack:
- React 18+
- Next.js 14+ (App Router)
- TypeScript (strict mode)
- Tailwind CSS for styling
- Vitest for unit tests

Code standards:
- Use functional components with hooks
- Export components as named exports
- Add JSDoc comments for components
- Write tests for complex logic
- Avoid prop drilling (use Context API for global state)
- Keep components under 200 lines
- Use semantic HTML

File structure:
- Components: /app/components/
- Utilities: /lib/utils.ts
- Tests: Colocated with components as .test.ts

Accessibility:
- WCAG 2.1 AA compliance required
- Use semantic HTML elements
- Ensure keyboard navigation
```

### When to Use .cursorrules

- **Do create**: When working on projects with consistent standards (existing codebases, team projects)
- **Do update**: When adding new technologies or changing standards mid-project
- **Don't create**: For very small one-off scripts or learning exercises

---

## Part 6: Verification Test

Now verify that Cursor's AI integration works:

### Step 1: Create a Test File

```bash
# In your home directory or a test folder
touch test_prime.py
```

Open test_prime.py in Cursor.

### Step 2: Trigger Inline Code Generation

Type a comment:
```python
# Function to check if a number is prime
```

Now trigger Cursor's inline assistant:
- **macOS**: Press Cmd+K
- **Windows/Linux**: Press Ctrl+K

A text input field will appear at the bottom of the editor saying "Ask Cursor anything..."

### Step 3: Request Code Generation

In the input field, type:
```
Create a function called is_prime that takes an integer and returns True if prime, False otherwise. Include docstring and type hints.
```

Press Enter.

### Step 4: Observe the Output

Cursor generates code and shows it inline:

```python
# Function to check if a number is prime
def is_prime(n: int) -> bool:
    """
    Check if a number is prime.

    Args:
        n: Integer to check

    Returns:
        True if n is prime, False otherwise
    """
    if n < 2:
        return False
    for i in range(2, int(n**0.5) + 1):
        if n % i == 0:
            return False
    return True
```

### Step 5: Accept or Reject

- **Accept**: Press Tab or click the checkmark
- **Reject**: Press Escape or click the X

Try accepting the generated code, then test it:

```python
# Function to check if a number is prime
def is_prime(n: int) -> bool:
    """Check if a number is prime."""
    if n < 2:
        return False
    for i in range(2, int(n**0.5) + 1):
        if n % i == 0:
            return False
    return True

# Test it
print(is_prime(17))  # Should print True
print(is_prime(10))  # Should print False
```

Run this in Cursor's terminal (Ctrl+` or Cmd+`) or in a Python REPL:

```bash
python test_prime.py
```

Expected output:
```
True
False
```

If you see this output, **Cursor is working correctly**.

---

## Part 7: Configuration Deep Dive

Access Cursor's settings to customize behavior:

### Opening Settings

- **macOS**: Cmd+,
- **Windows/Linux**: Ctrl+,

This opens a Settings UI (or settings.json for advanced users).

### Key Settings for AI

**Default AI Model**:
- Search "Default Model" in settings
- Choose your preferred model (claude-3-5-sonnet, gpt-4o, etc.)
- This is the model used when you press Cmd+K

**Auto-Indexing**:
- Search "Index Project" in settings
- Enable/disable automatic project indexing for better AI context
- Recommended: ON (helps AI understand your code)

**Agent Auto-Mode**:
- Search "Agent Auto-Mode" in settings
- If enabled, Agent mode runs without asking confirmation (dangerous for learning, safer OFF initially)
- Recommended: OFF while learning

**Temperature**:
- Controls randomness in AI output (0.0 = deterministic, 1.0 = creative)
- Default 0.5 is fine; change only if you want very different results each time

### Recommended Starting Settings

```json
{
  "editor.defaultModel": "claude-3-5-sonnet",
  "cursor.indexProject": true,
  "cursor.agent.autoMode": false,
  "editor.fontSize": 14,
  "editor.tabSize": 4,
  "editor.formatOnSave": true
}
```

You can edit these by opening Settings as JSON (click the {} icon in Settings UI) or by searching for each setting.

---

## Part 8: Troubleshooting Common Issues

### Issue 1: "No API Key Configured" Error

**Symptom**: You see an error like "No API keys found. Please configure an API key."

**Solution**:
1. Check if you subscribed to Cursor Pro. If not, subscribe or add an API key (see Part 4).
2. If you added an API key, verify it's correct: Open Settings, search for your provider name, check the key is fully pasted (no missing characters).
3. Try regenerating the API key from your provider (Anthropic/OpenAI/Google) and re-pasting.

### Issue 2: "Model Not Available" Error

**Symptom**: Error like "gpt-4o is not available for your account."

**Solution**:
1. Verify your OpenAI account has GPT-4 access. New accounts sometimes start with GPT-3.5-turbo only.
2. Try a different model: Switch to "gpt-4-turbo" or "gpt-3.5-turbo" in Settings.
3. If using Cursor Pro, ensure your subscription is active (check cursor.com account).

### Issue 3: Cursor Feels Slow on Large Projects

**Symptom**: Inline generation (Cmd+K) takes 10+ seconds.

**Solution**:
1. Check your internet connection (AI requests need bandwidth).
2. In Settings, search "Exclude" and configure "Files Exclude" to skip node_modules, .venv, __pycache__:
   ```json
   {
     "files.exclude": {
       "**/*.venv": true,
       "**/node_modules": true,
       "**/__pycache__": true
     }
   }
   ```
3. Reduce indexing scope: Settings → search "Index Ignore" → add directories to skip.
4. Switch to a faster model temporarily: gpt-3.5-turbo or mistral (if using BYOK).

### Issue 4: Extensions Not Working

**Symptom**: An extension you migrated from VS Code doesn't work in Cursor.

**Solution**:
1. Some VS Code extensions aren't fully compatible with Cursor. Check Cursor's extension marketplace (Cmd+Shift+X / Ctrl+Shift+X).
2. Try reinstalling the extension: Uninstall and install again from the marketplace.
3. Check the extension's documentation for Cursor-specific setup.
4. If the extension is essential, open an issue on [cursor.sh/feedback](https://cursor.sh/feedback).

### Issue 5: Can't Find .cursorrules Effects

**Symptom**: You created .cursorrules but AI prompts ignore it.

**Solution**:
1. Verify the file is named exactly `.cursorrules` (no extension, starts with dot).
2. Ensure it's in the project root (same directory as .gitignore or package.json).
3. Restart Cursor (quit completely and reopen).
4. The .cursorrules is only used for Cursor's AI features; if using a non-Cursor tool, it won't apply.

---

## Part 9: First Comparison with Zed

Now that you have Cursor installed, let's compare your two IDEs:

### Generation Speed

Both IDEs can generate code, but the experience differs:

**Zed**:
- Inline assistant with Cmd+/ then Cmd+K
- Slightly faster generation (tight Rust integration)
- Simpler interface, fewer options

**Cursor**:
- Inline generation with Cmd+K directly
- Slightly more control options (model selection, temperature)
- More features (Agent mode, Chat mode)

### Code Quality

Both use the same AI models (Claude, GPT-4, etc.) when configured with the same provider, so generated code quality should be comparable. Differences come from how each IDE sends context to the AI.

### Workflow Differences

**Zed workflow**:
1. Type comment or code fragment
2. Trigger assistant (Cmd+/)
3. Choose action (generate, explain, etc.)
4. Accept or reject

**Cursor workflow**:
1. Type comment or code fragment
2. Trigger inline (Cmd+K)
3. Type your request
4. Accept or reject with Tab/Escape

Cursor feels slightly more direct (fewer menu clicks), while Zed's structured approach may feel safer.

### When to Use Each

- **Zed**: When you want speed and simplicity, or on lower-performance machines (Rust efficiency)
- **Cursor**: When you're familiar with VS Code, need Agent mode, or want more configuration options

You'll dive deeper into each IDE's advanced features in Lessons 5 and 7.

---

## Practice Checklist

Before moving to Lesson 5, verify you can do all of these:

- [ ] Cursor launches without errors
- [ ] You can create and save files
- [ ] You can open Settings (Cmd+, / Ctrl+,)
- [ ] You know your configured AI model (check Settings → Default Model)
- [ ] Cmd+K (Ctrl+K) triggers inline code generation
- [ ] You can accept generated code with Tab
- [ ] You can open .cursorrules in a project and edit it
- [ ] You know your keyboard shortcut for opening Extensions (Cmd+Shift+X / Ctrl+Shift+X)

Tick all eight boxes before Lesson 5.

---

## What You Learned

You installed Cursor on your platform, authenticated with an AI provider (either Cursor Pro or bring-your-own-key), created a .cursorrules file to provide project context, and verified that inline code generation works. You now have a second AI-native IDE installed alongside Zed, and you understand key differences between them.

---

## Next Lesson Preview

Lesson 5 explores Cursor's unique AI features: **Chat mode** (long-form conversation), **Agent mode** (autonomous multi-file coding), and **diff-based change review**. You'll see how these differ from Zed's approach and practice with a more complex project than a simple prime checker.

---

## Additional Resources

**Official Documentation**:
- Cursor: [cursor.com/docs](https://cursor.com/docs)
- .cursorrules specification: [cursor.com/docs/context/rules](https://cursor.com/docs/context/rules)

**Guides**:
- Cursor vs Code: [cursor.com/compare](https://cursor.com/compare)
- API Key Setup: [platform.openai.com/docs](https://platform.openai.com/docs), [console.anthropic.com/docs](https://console.anthropic.com/docs)

**Community**:
- Cursor Discord: [discord.gg/cursor](https://discord.gg/cursor)
- Subreddit: r/cursor_ai (for tips and troubleshooting)

**Practice Projects**:
- Try creating .cursorrules for a personal project
- Migrate a VS Code workspace to Cursor and compare workflows
- Use Cursor and Zed side-by-side for the same task, noting differences
