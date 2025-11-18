---
sidebar_position: 8
title: "IDE Setup and Integration"
description: "Install and configure your IDE with Git integration and AI coding extensions for professional development"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Choose and Install IDE"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Information & Digital Literacy"
    measurable_at_this_level: "Student can select appropriate IDE, download installer, and launch successfully on their OS"

  - name: "Configure Git in IDE"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy & Problem-Solving"
    measurable_at_this_level: "Student can enable Git integration in IDE and navigate Source Control panel showing repository status"

  - name: "Install AI Coding Extensions"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy & Tool Usage"
    measurable_at_this_level: "Student can search, install, and activate GitHub Copilot, Cursor AI, or Continue extension and test basic functionality"

  - name: "Use IDE Git Tools"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy & Problem-Solving"
    measurable_at_this_level: "Student can stage files, write commit messages, and push changes using IDE GUI without command line"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (IDE, Source Control panel, Diff viewer, Git extensions, AI coding assistant) within A1 limit of 5 ✓"

differentiation:
  extension_for_advanced: "Research advanced IDE features: keyboard shortcuts, custom themes, workspace configuration, extension marketplace ecosystem"
  remedial_for_struggling: "Start with VS Code only (simplest); skip optional Git Graph/GitLens extensions; focus on Source Control panel basics before AI extensions"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# IDE Setup and Integration

Typing Git commands every time is slow. You can't see your changes visually.

**Solution**: An IDE - a text editor with Git built in, plus AI help.

Even if you've never written code, an IDE helps you see what's changing in your learning exercises.

**Time**: 20 minutes

---

## What Is an IDE?

**IDE** = Integrated Development Environment (fancy text editor for working with files)

**Simple explanation**:
- Regular text editor = just typing
- IDE = typing + Git + file explorer + AI + everything in one place

**Why this matters**:
- See Git changes with colors (green = added, red = removed)
- Click buttons instead of typing commands
- AI can help as you work
- Everything integrated

---

## Your Three Choices

Pick **ONE**. Don't install all three.

| IDE | Why Choose It | Download |
|-----|---------------|----------|
| **VS Code** | Most popular, easiest to learn | code.visualstudio.com |
| **Cursor** | Built-in AI (Claude, GPT-4) | cursor.sh |
| **Zed** | Super fast, minimal | zed.dev |

**Recommendation**: **VS Code** (most tutorials/help available) or **Cursor** (AI built-in).

---

## Step 1: Install Your IDE

### VS Code Installation

**Windows**:
1. Go to code.visualstudio.com
2. Click "Download for Windows"
3. Run the installer
4. Check "Add to PATH" option
5. Click Finish

**Mac**:
1. Go to code.visualstudio.com
2. Download for macOS
3. Drag to Applications folder
4. Open from Applications

**Linux**:

**You ask Gemini CLI**: "How do I install VS Code on [your distribution]?"

Gemini provides the right package manager command.

---

**Verify installation**:

Open VS Code. You should see:
- Welcome screen
- Blue sidebar on left
- Menu bar at top

✓ If you see this, installation succeeded.

**If VS Code won't open**:

**Ask Gemini CLI**: "VS Code won't launch on [your OS]. Here's the error: [paste error]"

---

## Step 2: Open Your Git Project

**In VS Code**:

1. Click **File → Open Folder** (or Ctrl+K Ctrl+O)
2. Navigate to your Git project folder
3. Click "Select Folder"

Your project opens with files listed in the left sidebar.

**Verify it's a Git project**:

Look at bottom-left corner - you should see your branch name (e.g., "main").

**If no branch name appears**:

The folder might not have Git initialized.

**Ask Gemini CLI**: "Initialize Git in my current folder"

Gemini runs: `git init`

Reload VS Code - branch name should appear.

---

## Step 3: Find the Source Control Panel

**Where**: Click the branch icon (3rd icon) in left sidebar

**OR**: Press `Ctrl+Shift+G` (Windows/Linux) or `Cmd+Shift+G` (Mac)

**What you see**:

```
┌─────────────────────────────┐
│ SOURCE CONTROL              │
│ ┌─────────────────────────┐ │
│ │ main ▼                  │ │ ← Current branch
│ └─────────────────────────┘ │
│                             │
│ Changes (2)                 │ ← Modified files
│ □ calculator.py         M   │
│ □ README.md            M    │
│                             │
│ ┌─────────────────────────┐ │
│ │ Message (Ctrl+Enter)    │ │ ← Commit message
│ └─────────────────────────┘ │
│ ✓ Commit                    │ ← Commit button
└─────────────────────────────┘
```

**What the symbols mean**:
- **M** = Modified
- **A** = Added (new file)
- **D** = Deleted
- **U** = Untracked (Git doesn't know about it yet)

---

## Step 4: Stage and Commit Using GUI

**Traditional way**: `git add file` then `git commit -m "message"`

**IDE way** (visual):

1. **See what changed**:
   - Look at Source Control panel
   - Files with changes are listed

2. **Stage files** (prepare to commit):
   - Hover over filename
   - Click the **+** icon

   *(Equivalent to: `git add filename`)*

3. **Write commit message**:
   - Type in the message box
   - Be clear and specific

4. **Commit**:
   - Click the **✓ checkmark** above message box
   - Or press `Ctrl+Enter`

   *(Equivalent to: `git commit -m "your message"`)*

---

**View what changed before staging**:

Click the filename in Source Control panel.

A diff view opens:
- **Red**: Lines removed
- **Green**: Lines added
- **White**: Context (unchanged)

Always review before committing.

---

## Step 5: Push to GitHub from IDE

**After committing**:

1. Click the **...** menu (three dots in Source Control panel)
2. Select **Push**

*(Equivalent to: `git push`)*

**First time?**

If asked to authenticate, enter your GitHub username and Personal Access Token.

**Verify it worked**:

Visit your GitHub repository - your commit should appear.

---

## IDE Git Operations Reference

Replace command-line with GUI clicks:

| Task | Command Line | IDE Action |
|------|-------------|-----------|
| See what changed | `git status` | Open Source Control panel |
| View file diff | `git diff filename` | Click filename in panel |
| Stage file | `git add filename` | Click **+** next to file |
| Stage all files | `git add .` | Click **+** next to "Changes" |
| Commit | `git commit -m "msg"` | Type message, click ✓ |
| Push to GitHub | `git push` | Click **...** → Push |
| Pull from GitHub | `git pull` | Click **...** → Pull |
| Create branch | `git checkout -b name` | Click branch name (bottom-left) → New Branch |
| Switch branch | `git checkout name` | Click branch name → Select branch |
| View commit history | `git log` | Install GitLens extension (optional) |

---

## Step 6: Install AI Coding Assistant (Optional)

Get AI suggestions as you work.

### Option 1: GitHub Copilot (Recommended)

**Cost**: $10/month (free for students with GitHub Student Pack)

**Install**:

1. Click **Extensions** icon (4 squares in left sidebar)
2. Search "GitHub Copilot"
3. Click "Install"
4. Click "Sign in to GitHub"
5. Authorize in browser
6. Reload VS Code

**Test it works**:

Type a comment in any file: `# function to add two numbers`

Copilot suggests text in gray. Press `Tab` to accept. You can learn by seeing what it suggests.

---

### Option 2: Cursor (Built-in AI)

If you installed Cursor IDE instead of VS Code:

AI is already built-in. Press `Ctrl+K` to open AI chat.

---

### Option 3: Continue (Free)

**Install**:

1. Extensions → Search "Continue"
2. Install
3. Configure with your AI API key (Gemini, Claude, GPT)

---

## Visual Workflow Example

**Traditional workflow** (command line):
```bash
$ git status
$ git diff calculator.py
$ git add calculator.py
$ git commit -m "Add calculator functions"
$ git push
```

**IDE workflow** (visual):

1. Open Source Control (1 click)
2. Click `calculator.py` to see diff
3. Click **+** to stage
4. Type message "Add calculator functions"
5. Click ✓ to commit
6. Click **...** → Push

Same result, more visual, easier to understand.

---

#### 💬 AI Colearning Prompt
> "Why use an IDE with visual Git tools instead of just typing Git commands in the terminal? What problems does visual Git solve?"

---

## Safety Tips

**Always**:
- Review the diff before staging (see what changed)
- Read what you're committing
- Write clear commit messages
- Check which branch you're on (bottom-left)

**Never**:
- Commit without reviewing changes
- Stage files you don't understand
- Ignore warnings in your IDE
- Rush through changes without understanding

---

#### 🎓 Expert Insight
> In AI-native development, you don't memorize which buttons to click—you understand visual Git intent. When reviewing changes, you want to SEE diffs (green/red), not READ text output. IDE tools translate Git commands into visual feedback: colors show changes, panels show status, diffs show line-by-line edits. Your job: understand what you're looking at, not memorize button locations.

---

#### 🤝 Practice Exercise

> **Ask your AI**: "Guide me through making a change in VS Code: edit a file, view the diff in the IDE, stage it visually, and commit using the GUI. Then explain what each visual element represents."

**Expected Outcome**: You'll understand IDE Git tools aren't about learning a new interface—they're about making Git visible. Diff colors = what changed. Source Control panel = status. Commit button = save point. Visual Git removes command-line mystery.

---

## Troubleshooting Common Issues

**"Git not found"**:
- Install Git first (see Lesson 2)
- Restart VS Code after installing

**"No repository found"**:
- Make sure folder has `.git` directory
- Ask Gemini: "Initialize Git here"

**"Authentication failed" when pushing**:
- Use Personal Access Token, not password
- Create new token on GitHub if needed

**Copilot not suggesting**:
- Check you're signed in (bottom-right status bar)
- Make sure subscription is active
- Reload VS Code

---

## Try With AI

Practice using your IDE.

**Tool**: Gemini CLI (or Claude Code, ChatGPT)

### Exercise 1: Install and Setup

```
Guide me through:
1. Installing VS Code
2. Opening my project folder
3. Finding the Source Control panel
4. Explaining what I'm looking at
```

### Exercise 2: First GUI Commit

```
I edited README.md in my IDE.
Walk me through committing it using only the GUI:
- Where do I click?
- What do I type?
- How do I know it worked?
```

### Exercise 3: View Diff

```
How do I see exactly what changed in a file
before I commit it? Show me the diff view.
```

### Exercise 4: Install AI Extension

```
I want GitHub Copilot.
Walk me through:
1. Installing the extension
2. Signing in
3. Testing it works
```

### Exercise 5: IDE vs Command Line

```
Explain the difference between:
1. Typing "git add ." in terminal
2. Clicking the + button in VS Code

Are they doing the same thing?
```
