# The Three Surfaces: Architecture Overview

## What Makes Antigravity Different?

Zed and Cursor are **editor-first**—they're code editors with AI assistants built in. Antigravity is **agent-first**—it's an orchestration system where AI agents coordinate your work, and the editor is one tool agents use.

This architectural difference manifests as **three distinct surfaces** working together:

1. **Agent Manager**: Where you orchestrate agents
2. **Editor**: Where you write/review code
3. **Integrated Browser**: Where agents research and test

Understanding these three surfaces is essential to using Antigravity effectively. This section explains each surface, how they interact, and which surface to use for different tasks.

**Time estimate**: 15-20 minutes reading + hands-on exploration

---

## Surface 1: Agent Manager (The Command Center)

### What It Does

The Agent Manager is your **control plane**—where you:
- Create and manage agents
- Monitor agent progress
- Review artifacts (task lists, implementation plans, walkthroughs)
- Configure workspace settings
- Manage parallel task execution

### Key UI Elements

```
┌─────────────────────────────────────────────────────────┐
│ Agent Manager                               [⚙️] [⚡]   │
├─────────────────┬─────────────────────────────────────┤
│  Workspace:     │  Agent Details                      │
│  My First Proj  │  ┌─────────────────────────────────┐│
│                 │  │ Agent: code-generator           ││
│  Agents:        │  │ Status: Running                 ││
│  ├─ Agent-1 📊  │  │ Progress: ████░░░░ 60%          ││
│  │  Status: ✅  │  │                                 ││
│  ├─ Agent-2 🤖  │  │ Task List Artifact:             ││
│  │  Status: ⏳  │  │ ├─ Task 1: Plan structure   ✅  ││
│  └─ New Agent   │  │ ├─ Task 2: Write code      ⏳  ││
│                 │  │ └─ Task 3: Write tests     ❌  ││
│ Settings ⚙️     │  │                                 ││
│ Artifacts 📄    │  │ [Review Plan] [Approve] [Stop]  ││
│                 │  └─────────────────────────────────┘│
└─────────────────┴─────────────────────────────────────┘
```

### Common Tasks in Agent Manager

| Task | How to Do It |
|------|-------------|
| **Create new agent** | Click "+ New Agent" button |
| **Monitor progress** | Watch progress bar and task list |
| **Review plan** | Click "Review Plan" to see implementation plan artifact |
| **Approve changes** | Click "Approve" when you agree with agent's plan |
| **Stop agent** | Click "Stop" to interrupt running agent |
| **View artifacts** | Click agent to expand and see task lists, plans, walkthroughs |

### When to Use Agent Manager

- Orchestrating multi-step tasks
- Running agents in parallel
- Reviewing agent work before it's committed
- Managing multiple agents on same project
- Monitoring long-running tasks

### Example Interaction

```
You: "Write a weather app with error handling"
Agent Manager displays:
├─ Task List (agent's breakdown of work)
│  ├─ Task 1: Design API structure
│  ├─ Task 2: Write request handler
│  └─ Task 3: Add error handling
├─ Implementation Plan (agent's proposed approach)
├─ [You review both]
├─ [You click "Approve"]
└─ Agent executes, generating code files

Agent Manager shows progress as each task completes.
When complete, you see Walkthrough artifact summarizing work.
```

---

## Surface 2: Editor (The Coding Environment)

### What It Does

The Editor is where **you write or review code**. But unlike traditional editors, it's:
- **Agent-aware**: You can see what agents have done
- **Context-rich**: Tab autocomplete based on agent work and project files
- **Integrated**: Press a button to hand off to agent or take over from agent

### Key UI Elements

```
┌───────────────────────────────────────────────────────┐
│ Antigravity Editor: my-project                 [🔄]   │
├───────────────────────────────────────────────────────┤
│ main.py  [+] [x]                                      │
├───────────────────────────────────────────────────────┤
│ 1  def process_data(input_file):                      │
│ 2      """Load and process CSV file."""               │
│ 3      try:                                           │
│ 4          data = pd.read_csv(input_file)│ ◄── Agent │
│ 5          return data.dropna()                       │  autocomplete
│ 6      except FileNotFoundError:                      │  (ghost text)
│ 7          raise ValueError("File not found")        │
│ 8                                                     │
│ [Side panel: Agent Sidebar]                          │
│ ├─ Running Agent: code-review                        │
│ ├─ Current Task: Analyze loops                       │
│ └─ [Open in Manager] [Take over] [Hand off]          │
└───────────────────────────────────────────────────────┘
```

### Editor Features

| Feature | What It Does | When to Use |
|---------|-------------|-----------|
| **Tab Autocomplete** | AI suggests next line of code | Writing code, reviewing agent work |
| **Agent Sidebar** | Shows running agent progress | Monitoring parallel work |
| **Open in Manager** | Jump to Agent Manager | Want full task visibility |
| **Take Over** | Pause agent, edit code manually | Agent stuck or needs refinement |
| **Hand Off** | Give code to agent for continuation | You're done, let agent finish |

### When to Use Editor

- Reviewing code that agents generated
- Making small manual edits to agent code
- Writing code alongside agents (you write some, agent writes some)
- Using tab autocomplete for faster coding
- Comparing different versions of code

### Example Interaction

```
Agent Manager shows: Agent generated three versions of a function.

You open one version in Editor.
Tab autocomplete suggests improvements as you read code.

You hand off to agent: "Optimize this function for memory usage"

Editor stays open, side panel shows agent working.
When done, agent places updated code in same file.
You review in editor, make small tweaks, confirm.
```

---

## Surface 3: Integrated Browser (The Research & Testing Tool)

### What It Does

The Integrated Browser is a **Chrome-based browser inside Antigravity** that agents use to:
- Research APIs and documentation
- Test web applications
- Take screenshots of results
- Interact with web interfaces

### Key Capabilities

| Capability | Use Case |
|-----------|----------|
| **API documentation lookup** | Agent reads API docs while implementing (no context switching) |
| **Live testing** | Agent clicks buttons in your app, verifies functionality |
| **Screenshot capture** | Agent takes screenshots for documentation/walkthrough |
| **Automation** | Agent can automate form filling, testing workflows |
| **Cookie/session management** | Agents can test authenticated features |

### When Agents Use Integrated Browser

Agents automatically open the Integrated Browser when they need to:
1. **Research**: "Look up how to use the Firebase API"
2. **Test**: "Launch the app and verify the login form works"
3. **Validate**: "Visit the deployed app and test all features"
4. **Document**: "Take a screenshot of the dashboard for the walkthrough"

### Example: Agent Using Browser

```
You: "Create a weather app that fetches data from OpenWeatherMap API"

Agent workflow:
1. Recognizes need for API documentation
2. Opens Integrated Browser
3. Visits https://openweathermap.org/api
4. Reads API structure and endpoints
5. Closes browser, returns to Editor
6. Writes code using learned API structure
7. Re-opens browser to test the app
8. Clicks around app to verify it works
9. Takes screenshot for walkthrough artifact
```

### You Don't Usually Interact With Integrated Browser Directly

In most cases, agents manage the Integrated Browser automatically. You'd only manually use it if:
- Agent asks you a research question ("Can you verify this API endpoint works?")
- You want to manually test your app between agent tasks
- You want to explore the agent's research process

---

## How the Three Surfaces Work Together

### Workflow: Temperature Converter Project

**Scenario**: You want agents to create a temperature converter CLI.

```
Step 1: You (Agent Manager)
  └─> Create new agent with task: "Create a temperature converter (C↔F) with input validation"

Step 2: Agent Manager
  ├─> Agent breaks down into tasks (Task List artifact)
  ├─> Agent proposes implementation (Implementation Plan artifact)
  └─> Displays for you to review

Step 3: You (Agent Manager)
  └─> Review artifacts, click "Approve"

Step 4: Agent (Editor)
  ├─> Generates code files (main.py, test.py)
  └─> Each file appears in Editor tabs

Step 5: You (Editor)
  ├─> Review generated code
  ├─> Make small edits if needed
  └─> Optionally hand off to agent for refinement

Step 6: Agent (Integrated Browser)
  ├─> Wants to test the app
  ├─> Opens browser inside Antigravity
  ├─> Runs: python main.py
  ├─> Types test inputs: "32" (32°F = 0°C)
  └─> Takes screenshot: Verifies output correct

Step 7: Agent (Agent Manager)
  └─> Generates Walkthrough artifact (final summary with screenshots)

Step 8: You (Agent Manager)
  └─> Review final walkthrough, confirm project complete
```

### Key Insight: Surfaces Are Interconnected

- **Agent Manager** → Agent creates breakdown
- **Agent Manager** → You approve plan
- **Editor** → Agent generates code
- **You** (Editor) → Review/edit code
- **Integrated Browser** → Agent tests code
- **Agent Manager** → You see final results in walkthrough

All three surfaces feed into each other seamlessly.

---

## Surface Comparison: Antigravity vs Zed vs Cursor

| Dimension | Zed | Cursor | Antigravity |
|-----------|-----|--------|-------------|
| **Primary interaction** | Write code | Write code | Orchestrate agents |
| **Where agents work** | In editor sidebar | In chat/editor | Across all three surfaces |
| **Code review** | In editor | In diff panel | Agent Manager + Editor |
| **Research** | You use external browser | You use chat | Agent uses integrated browser |
| **Testing** | You run manually | You run manually | Agent runs + logs results |
| **Best for** | Solo coding + AI help | Team coding + AI help | Complex multi-step tasks |

---

## Practical Example: Where to Start

### First-time Antigravity User Journey

1. **Open Agent Manager** (right after installation)
   - This is where you always start
   - No agents yet, so you see empty list

2. **Click "Create Agent"** (next section)
   - Define the task
   - Choose AI provider
   - Agent Manager shows progress

3. **Review Task List Artifact** (still Agent Manager)
   - See agent's proposed breakdown
   - Approve or ask for changes

4. **Agent generates code** (Editor opens automatically)
   - Code appears in tabs
   - You review without leaving workspace

5. **Agent tests code** (Integrated Browser opens automatically)
   - Agent shows results
   - You see screenshots in walkthrough

6. **Final Walkthrough** (Agent Manager)
   - Review complete project summary
   - See all artifacts and results in one place

---

## Verification: Understanding the Surfaces

Before moving to agent creation, verify you understand:

- [ ] **Agent Manager** is where you control agents and review artifacts
- [ ] **Editor** is where code lives and you can make edits
- [ ] **Integrated Browser** is where agents research and test
- [ ] All three work together in a coordinated workflow
- [ ] You don't need to use all three simultaneously (agents manage most of it)

**Ready?** Proceed to [05-verification.md](./05-verification.md) to create your first agent and see these surfaces in action.

---

## Quick Reference: Which Surface When?

**Use Agent Manager when you want to**:
- Create a new agent
- Monitor agent progress
- Review task lists and implementation plans
- Approve agent work
- See final walkthroughs

**Use Editor when you want to**:
- Review code an agent generated
- Make small code edits
- See tab autocomplete suggestions
- Use "take over" / "hand off" controls
- Compare multiple code files

**Agents use Integrated Browser when they need to**:
- Look up API documentation
- Test running apps
- Verify web functionality
- Capture screenshots for walkthroughs

---

## Advanced: Customizing Surface Layout

In settings, you can customize how surfaces appear:
- **Split view**: Agent Manager and Editor side-by-side
- **Full screen**: One surface at a time (maximize focus)
- **Tab view**: Switch between surfaces with tabs

This is optional—defaults work fine for learning.

---

## Why This Architecture?

You might wonder: "Why not just have an editor like Zed or Cursor?"

**Reason 1: Parallel work**
Agent Manager lets multiple agents work on different parts simultaneously. Cursor has one chat. Antigravity coordinates many agents.

**Reason 2: Artifact visibility**
Agents produce task lists, plans, walkthroughs that you can review together. This provides visibility into agent reasoning.

**Reason 3: Agent autonomy**
Agents can research, test, and validate independently using the Integrated Browser. You don't lose context switching.

This three-surface design enables truly collaborative agent-assisted development, not just an editor with a chatbot.

---

## Next Step

You understand the architecture. Time to create your first agent and see it in action.

**Next**: [05-verification.md](./05-verification.md) - Create your first agent

**Time check**: You have 10-15 minutes remaining for this lesson. Next section will take ~10-15 minutes.

---

**Checkpoint**: Can you explain to someone else what the three surfaces do? If yes, you're ready for the next section!
