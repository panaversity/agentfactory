# Verification: Creating Your First Agent

## Overview

You've installed Antigravity, created a workspace, configured AI providers, and learned about the three surfaces. Now it's time to **create your first agent** and verify everything works.

This section walks through creating a simple agent, observing how it works, and understanding the artifacts it generates (Task List, Implementation Plan, Walkthrough).

**Time estimate**: 10-15 minutes

---

## What You're About to Do

You'll create an agent with this task:
```
"Create a simple Python function that converts temperatures from Celsius to Fahrenheit.
Include input validation to ensure the input is a number."
```

This is:
- **Simple enough** to complete in seconds
- **Real enough** to see all three artifact types (Task List, Implementation Plan, Walkthrough)
- **Informative enough** to understand agent workflow

---

## Step 1: Open Agent Manager

Launch Antigravity and navigate to your workspace's Agent Manager. You should see:
- Workspace name (top left)
- Empty agents list (left sidebar)
- **"Create Agent"** button (center)

---

## Step 2: Create Agent

Click **"Create Agent"** button. A dialog appears with:

```
┌──────────────────────────────────────────────┐
│ Create New Agent                         [X] │
├──────────────────────────────────────────────┤
│ Agent Name: [________________]                │
│ Task Description: [______________________]   │
│ AI Provider: [Dropdown menu]                │
│ Model: [Dropdown menu]                      │
│ [Cancel] [Create Agent]                      │
└──────────────────────────────────────────────┘
```

### Fill in the Fields

**Agent Name**: Give your agent a meaningful name
- Example: "temp-converter" or "first-agent"
- Naming tip: Use lowercase with hyphens (helps when agents reference each other later)

**Task Description**: Paste the task below
```
Create a simple Python function that converts temperatures from Celsius to Fahrenheit.
Include input validation to ensure the input is a number.
```

**AI Provider**: Select from your configured providers
- If you configured multiple, pick **Anthropic (Claude)** (most reliable for coding tasks)
- If only Google AI, pick Google AI
- If only OpenAI, pick OpenAI

**Model**: Select model version
- **Claude Sonnet** (if Anthropic): Recommended
- **GPT-4-mini** (if OpenAI): Good alternative
- **Gemini Pro** (if Google AI): Good option

---

## Step 3: Create Agent (Click Button)

Click **"Create Agent"** button. Antigravity initializes the agent:
- Status changes from "Initializing" to "Running"
- Task List artifact starts appearing
- Progress bar shows agent working

---

## Step 4: Observe Task List Artifact

As the agent runs, you'll see a **Task List** artifact building in real-time:

```
Task List: Celsius to Fahrenheit Converter
├─ Task 1: Design function signature and docstring           [✓] 15 seconds
├─ Task 2: Implement temperature conversion formula          [✓] 30 seconds
├─ Task 3: Add input validation for numeric inputs           [✓] 25 seconds
├─ Task 4: Create test function to verify functionality      [✓] 20 seconds
└─ Task 5: Generate project summary and implementation notes [⏳] Running...
```

### What This Shows

- **Agent's breakdown**: How the agent decomposed your task
- **Progress tracking**: Each task and its completion time
- **Transparency**: You can see what agent is doing at each step

---

## Step 5: Observe Implementation Plan Artifact

After Task List completes, the agent generates an **Implementation Plan**:

```
Implementation Plan: Celsius to Fahrenheit Converter

Overview:
The agent will create a Python module with a temperature conversion function.

Proposed Structure:
- main.py: Contains celsius_to_fahrenheit() function with validation
- test.py: Contains unit tests for the function
- README.md: Documents usage and examples

Key Implementation Details:
1. Function signature: celsius_to_fahrenheit(celsius: float) -> float
2. Formula: F = (C × 9/5) + 32
3. Input validation: Check if input is numeric, raise TypeError if not
4. Edge cases: Handle None, empty string, non-numeric inputs

Expected output structure:
```python
def celsius_to_fahrenheit(celsius):
    """Convert Celsius to Fahrenheit."""
    if not isinstance(celsius, (int, float)):
        raise TypeError("Input must be a number")
    return (celsius * 9/5) + 32
```

### What This Shows

- **Agent's reasoning**: How it plans to solve the problem
- **File structure**: What files agent will create
- **Key decisions**: Implementation details and edge cases
- **You review before implementation**: Chance to say "hold on, change X" if needed

---

## Step 6: Agent Generates Code

After you review (or agent auto-proceeds), the agent generates actual code. You'll see:

1. **Editor opens automatically** with files the agent created:
   - `main.py` with temperature conversion function
   - `test.py` with unit tests
   - `README.md` with usage documentation

2. **Integrated Browser opens** (sometimes) if agent needs to test the code

3. **Files appear in Editor tabs** for your review

---

## Step 7: Review Code in Editor

Switch to the Editor surface (or it opens automatically). You see:

**main.py**:
```python
def celsius_to_fahrenheit(celsius):
    """Convert Celsius to Fahrenheit with input validation.

    Args:
        celsius: Temperature in Celsius (int or float)

    Returns:
        Temperature in Fahrenheit (float)

    Raises:
        TypeError: If input is not numeric
    """
    if not isinstance(celsius, (int, float)):
        raise TypeError("Input must be a number")
    return (celsius * 9/5) + 32


if __name__ == "__main__":
    # Test cases
    print(f"0°C = {celsius_to_fahrenheit(0)}°F")      # 32°F
    print(f"100°C = {celsius_to_fahrenheit(100)}°F")  # 212°F
    print(f"-40°C = {celsius_to_fahrenheit(-40)}°F")  # -40°F
```

**test.py**:
```python
import pytest
from main import celsius_to_fahrenheit

def test_freezing_point():
    assert celsius_to_fahrenheit(0) == 32

def test_boiling_point():
    assert celsius_to_fahrenheit(100) == 212

def test_absolute_zero_equivalent():
    assert celsius_to_fahrenheit(-40) == -40

def test_invalid_input():
    with pytest.raises(TypeError):
        celsius_to_fahrenheit("not a number")
```

### What to Observe

- ✅ Function matches implementation plan
- ✅ Docstring explains what function does
- ✅ Input validation present (checks if numeric)
- ✅ Tests verify function works correctly
- ✅ Code is readable and well-structured

---

## Step 8: Agent Tests Code

Agent automatically tests the code:
1. Runs `python main.py` to verify basic execution
2. Runs `pytest test.py` to run all unit tests
3. Captures results and screenshots

You might see:
- Integrated Browser tab showing terminal output
- Test results: ✅ All 4 tests passed
- Agent taking screenshots for documentation

---

## Step 9: Observe Walkthrough Artifact

After completion, agent generates a **Walkthrough** artifact (final deliverable):

```
WALKTHROUGH: Celsius to Fahrenheit Converter

Project Summary:
Created a Python module with temperature conversion functionality,
complete with input validation and comprehensive tests.

Files Generated:
├─ main.py (45 lines)
│  └─ celsius_to_fahrenheit() function with validation
├─ test.py (20 lines)
│  └─ 4 unit tests covering normal cases and error handling
└─ README.md (15 lines)
   └─ Usage documentation with examples

Implementation Highlights:
1. Input validation: Rejects non-numeric inputs with TypeError
2. Formula correctness: Uses standard C = (F × 9/5) + 32 conversion
3. Edge case handling: Tested at 0°C (freezing), 100°C (boiling), -40°C
4. Test coverage: 4 tests covering happy path and error cases

Verification Results:
├─ ✓ main.py runs without errors
├─ ✓ All 4 unit tests pass
├─ ✓ Function correctly converts 0°C → 32°F
├─ ✓ Function correctly converts 100°C → 212°F
├─ ✓ Error handling works (rejects string input)

Test Output:
```
test_freezing_point PASSED
test_boiling_point PASSED
test_absolute_zero_equivalent PASSED
test_invalid_input PASSED
====== 4 passed in 0.23s ======
```

Generated Code:
[Actual code displayed]

Screenshots:
[Screenshots of test execution and output]
```

### What This Shows

- **Project completion summary**: What was accomplished
- **Files created**: List with line counts
- **Implementation rationale**: Why choices were made
- **Test results**: Proof that code works
- **Screenshots**: Evidence of execution
- **Ready for use**: You can copy these files to your project

---

## Step 10: Completion Status

Back in **Agent Manager**, you see:

```
Agent: temp-converter
Status: ✅ Complete
Duration: 2 minutes 45 seconds

Artifacts Generated:
├─ Task List (5 tasks completed)
├─ Implementation Plan (reviewed and approved)
└─ Walkthrough (with test results and code)

Actions:
[View Details] [Download Files] [Archive Agent] [Create New Agent]
```

---

## Verification Checklist

Your first agent successfully completed if you observe:

- [ ] Agent Manager shows "Status: Complete"
- [ ] Task List artifact generated with multiple tasks
- [ ] Implementation Plan artifact shows proposed solution
- [ ] Editor shows generated code files (main.py, test.py, README.md)
- [ ] Agent tests code and shows passing results
- [ ] Walkthrough artifact summarizes complete project
- [ ] No error messages in any surface

**All checked?** Congratulations! Antigravity is working correctly.

---

## What You've Learned

By completing this verification, you've experienced:

1. **Agent orchestration** (Agent Manager)
   - How to create an agent
   - How agents break down tasks
   - Artifact visibility into agent reasoning

2. **Artifact system** (Agent Manager)
   - Task Lists show progress
   - Implementation Plans show design decisions
   - Walkthroughs summarize results

3. **Multi-surface workflow**
   - Agent Manager for orchestration
   - Editor for code review
   - Integrated Browser for testing (handled automatically)

4. **Agent autonomy**
   - Agents research (read implementation plan before coding)
   - Agents implement (write code files)
   - Agents test (run tests and verify)
   - Agents document (generate walkthroughs)

---

## Common Observations

### "The agent completed the task perfectly—is there anything left to do?"

Yes! Lesson 7 focuses on:
- Creating more complex agents (multi-file projects)
- Iterating on agent work (asking agent to refactor, improve, etc.)
- Using agent features like parallel task execution
- Advanced artifact system usage

For now, you've verified installation works.

### "What if the agent generated code that wasn't quite right?"

In real workflow, you'd:
1. Point out the issue in Agent Manager
2. Agent provides a revised implementation plan
3. You approve changes
4. Agent re-implements

For this lesson, assuming the provided task generates quality code.

### "Can I save the generated code?"

Yes! In Editor:
1. Right-click on file tab
2. Select "Save to project" or "Export"
3. Or copy-paste code to your own project

Workspace files persist in `~/.antigravity/workspaces/[name]/projects/`

---

## Troubleshooting First Agent Creation

### Problem: "Agent creation dialog won't open"
**Solution**:
1. Click outside any open dialogs to close them
2. Try again: Agent Manager → Create Agent

### Problem: "Agent appears stuck (progress bar frozen)"
**Solution**:
1. Wait 30 seconds (sometimes takes time to process)
2. If still frozen: Click "Stop" button
3. Create new agent with simpler task
4. Check logs: `~/.antigravity/logs/agent.log`

### Problem: "Code generated but won't run (errors in test output)"
**Solution**:
1. This is OK for learning! Agent tried its best.
2. You can fix in Editor and try again
3. Or create new agent with clearer task description
4. For Lesson 7, you'll learn how to iterate: ask agent to "fix the error"

### Problem: "I don't see the Walkthrough artifact"
**Solution**:
1. Wait a bit longer (agent might still be writing it)
2. Click "Refresh" if available
3. Close Agent Manager and reopen workspace
4. Check agent status—might still be "Running", not yet "Complete"

---

## Success Metrics

Your first agent is successful if:

1. ✅ Agent completes without errors
2. ✅ All three artifact types generated (Task List, Plan, Walkthrough)
3. ✅ Code generated and tested
4. ✅ Test results show passing tests
5. ✅ You can explain what the agent did at each stage

---

## What's Next?

You've successfully installed, configured, and verified Antigravity! You've seen:
- Agent Manager orchestration
- Task List and Implementation Plan artifacts
- Editor code review
- Integrated Browser testing (automatic)
- Walkthrough documentation

**Next lesson**: Lesson 7 - Antigravity Agent Architecture & Features (deeper exploration of workflows, advanced features, multi-step tasks)

---

## Take Your Time

Don't rush to Lesson 7 yet! Spend a few minutes:
1. **Review the code** the agent generated—does it look right to you?
2. **Play with Editor**—make a small change to the code, observe tab autocomplete
3. **Check artifacts** in different orders (Task List → Plan → Walkthrough)
4. **Explore Agent Manager**—click around, see what controls are available

Familiarity builds confidence. Lesson 6 is complete when you feel comfortable with Antigravity's interface.

---

## Summary

| Stage | Surface | What Happens |
|-------|---------|--------------|
| **1. Create** | Agent Manager | You create agent with task |
| **2. Plan** | Agent Manager | Agent generates Task List, Implementation Plan |
| **3. Code** | Editor | Agent generates code files in tabs |
| **4. Test** | Integrated Browser | Agent tests code automatically |
| **5. Document** | Agent Manager | Agent generates Walkthrough artifact |
| **6. Review** | Agent Manager | You review final walkthrough |

**Congratulations!** You're ready for Lesson 7 when you feel comfortable with this workflow.
