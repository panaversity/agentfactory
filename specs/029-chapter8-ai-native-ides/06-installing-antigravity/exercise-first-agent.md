# Exercise: Create Your First Agent with a Simple Task

## Objective

Reinforce your understanding of agent creation, task decomposition, and the artifact system by creating a second agent with a slightly more complex task than the verification exercise.

**Time estimate**: 15-20 minutes

**Success criteria**: Create agent, observe all three artifact types (Task List, Implementation Plan, Walkthrough), understand agent decision-making

---

## Exercise Task

Create an agent with this task description:

```
Create a Python function named "calculate_bmi" that calculates Body Mass Index
given weight (in kilograms) and height (in meters).

The function should:
1. Take weight and height as float parameters
2. Return BMI as a float
3. Include input validation for negative or zero values
4. Include a docstring explaining usage
5. Provide at least 3 test cases covering normal BMI values

Include a main module that demonstrates the function with sample inputs.
```

---

## Step-by-Step Instructions

### Step 1: Prepare Task Description

Copy the task description above. You'll paste it into Agent Manager in next step.

**Tip**: The more specific your task description, the better agents perform. Notice this task:
- Specifies exact function name
- Lists numbered requirements
- Mentions parameter types and units
- Asks for validation
- Requests examples and tests

Compare to vague task: "Create a BMI calculator" (agent might guess wrong on function signature)

### Step 2: Open Agent Manager

1. Launch Antigravity
2. Navigate to your workspace from Lesson 6
3. You should see Agent Manager with "temp-converter" agent from verification exercise

### Step 3: Create New Agent

Click **"Create Agent"** button (or **"New Agent"** in left sidebar)

**Agent Name**: Give it a meaningful name
- Example: "bmi-calculator" or "calculate-bmi"
- Tip: Use naming consistent with function you're creating

**Task Description**: Paste the complete task from Step 1

**AI Provider**: Choose same provider as before (Anthropic/Claude recommended)

**Model**: Select same model as before (Claude Sonnet recommended)

### Step 4: Click Create

Agent initializes. Watch as it works:
- Progress bar appears
- Task List starts generating
- Agent works through tasks

### Step 5: Observe Task List (First Artifact)

As agent runs, watch Task List build in real-time:

Expected tasks might include:
- Task 1: Design function signature and docstring
- Task 2: Implement BMI calculation formula (BMI = weight / (height²))
- Task 3: Add input validation for edge cases
- Task 4: Create comprehensive test cases
- Task 5: Create main module demonstrating function
- Task 6: Verify all tests pass

**What to notice**:
- Agent breaks down your requirements into smaller tasks
- Each task appears with estimated time
- You see the decomposition happen in real-time

**Reflection question**: Does agent's decomposition match how YOU would break down the task? Why or why not?

### Step 6: Observe Implementation Plan (Second Artifact)

After Task List completes, Implementation Plan generates:

**Expected content**:
- File structure (main.py, test.py, maybe README.md)
- Function signature: `def calculate_bmi(weight: float, height: float) -> float`
- Formula: BMI = weight / (height * height)
- Validation rules (reject weight ≤ 0, reject height ≤ 0)
- Test cases (normal BMI, edge cases, error handling)

**What to notice**:
- Agent explicitly states file structure BEFORE writing code
- Agent shows proposed function signature
- Agent explains validation approach
- Agent lists test cases

**Reflection question**: Before reading this plan, what file structure would you have chosen? Same or different from agent's proposal?

### Step 7: Agent Generates Code

Agent implements code and files appear in Editor automatically.

**Expected files**:
- **main.py**: Function definition with validation and docstring
- **test.py**: Unit tests covering normal cases and edge cases
- **README.md**: Usage documentation

### Step 8: Review Code in Editor

Switch to Editor (or it opens automatically):

**main.py expected content**:
```python
def calculate_bmi(weight: float, height: float) -> float:
    """Calculate Body Mass Index (BMI).

    Args:
        weight: Weight in kilograms (must be positive)
        height: Height in meters (must be positive)

    Returns:
        BMI as float (weight / height²)

    Raises:
        ValueError: If weight or height ≤ 0
    """
    if weight <= 0:
        raise ValueError("Weight must be positive")
    if height <= 0:
        raise ValueError("Height must be positive")

    return weight / (height * height)
```

**test.py expected content**:
```python
import pytest
from main import calculate_bmi

def test_normal_bmi():
    # 70kg, 1.75m → BMI ≈ 22.9
    assert abs(calculate_bmi(70, 1.75) - 22.86) < 0.01

def test_underweight():
    # 50kg, 1.8m → BMI ≈ 15.4
    assert calculate_bmi(50, 1.8) < 18.5

def test_overweight():
    # 100kg, 1.75m → BMI ≈ 32.7
    assert calculate_bmi(100, 1.75) > 25

def test_invalid_weight():
    with pytest.raises(ValueError):
        calculate_bmi(-10, 1.75)

def test_invalid_height():
    with pytest.raises(ValueError):
        calculate_bmi(70, 0)
```

**What to observe**:
- ✅ Function signature matches Implementation Plan
- ✅ Validation logic matches proposed approach
- ✅ Docstring clear and helpful
- ✅ Tests cover normal cases AND edge cases (negative values)

**Reflection question**: Which part of the generated code is most impressive? Which part could be improved?

### Step 9: Observe Test Results

Agent automatically runs tests. You see:
- Terminal output showing test execution
- All tests passing (or errors if validation wasn't right)
- Duration (usually <1 second)

**Expected output**:
```
test_normal_bmi PASSED
test_underweight PASSED
test_overweight PASSED
test_invalid_weight PASSED
test_invalid_height PASSED
====== 5 passed in 0.18s ======
```

### Step 10: Observe Walkthrough (Third Artifact)

After tests pass, agent generates Walkthrough:

**Expected content**:
- Project summary
- Files created with line counts
- Implementation highlights (formula, validation logic)
- Test results with proof of passing
- Usage examples
- Screenshots of execution

**What to observe**:
- Walkthrough ties everything together
- You see proof that code works (test output)
- You see how to use the function (examples)
- You understand why agent made certain choices

---

## Comparative Analysis: Exercise vs Verification

Compare this exercise (BMI calculator) to Lesson 6's verification exercise (temperature converter):

| Aspect | Temperature Converter (Verification) | BMI Calculator (Exercise) |
|--------|-------------------------------------|-------------------------|
| **Task complexity** | Simple (one calculation) | Moderate (validation + tests) |
| **Validation required** | Basic (numeric check) | Advanced (range checking) |
| **Test cases** | Simple | More comprehensive |
| **Files generated** | Similar (main.py, test.py) | Likely similar structure |
| **Time to complete** | 2-3 minutes | 3-4 minutes |

**Reflection question**: Did the agent handle the more complex task just as well as the simpler one? Why?

---

## Observation Checklist

As your agent runs, observe and check these:

- [ ] Task List artifact generated with 5+ tasks
- [ ] Implementation Plan shows clear file structure
- [ ] Function signature in plan matches requirements (exact parameter names)
- [ ] Validation logic proposed before code written
- [ ] Code appears in Editor matching plan
- [ ] All tests pass (green check marks)
- [ ] Walkthrough artifact comprehensive and clear
- [ ] Agent completed in <5 minutes

**All checked?** Exercise successfully completed!

---

## Key Learnings from This Exercise

### 1. Artifact-Driven Development

You saw:
- **Task List** → Agent breaks down work
- **Implementation Plan** → Agent proposes approach
- **Code** → Agent implements
- **Tests** → Agent validates
- **Walkthrough** → Agent documents

This is artifact-driven development—every stage produces reviewable artifacts.

### 2. Specificity Matters

Your task description was specific:
- Named the function
- Listed requirements
- Specified types (float)
- Mentioned edge cases

Agent used this specificity to generate better code. Vague task = vague results.

### 3. Validation as First-Class Feature

Agent understood validation wasn't optional—it was a requirement. Agent:
- Checked for negative/zero values
- Raised appropriate exceptions
- Tested validation thoroughly

In Zed/Cursor, you'd need to remember to add validation. Antigravity agents make it explicit in planning stage.

### 4. Complete Lifecycle

You witnessed the complete agent lifecycle:
1. Planning (Task List, Implementation Plan)
2. Implementation (Code generation)
3. Validation (Tests)
4. Documentation (Walkthrough)

This is how Antigravity agents approach problems—systematically with artifacts at each stage.

---

## Challenge Extension (Optional)

If you want to push further, try:

**Create a third agent** with this more complex task:

```
Create a Python function named "convert_temperature" that converts between
Celsius, Fahrenheit, and Kelvin scales.

Requirements:
1. Function signature: convert_temperature(value: float, from_scale: str, to_scale: str) -> float
2. Support scales: "C" (Celsius), "F" (Fahrenheit), "K" (Kelvin)
3. Validate that from_scale and to_scale are valid (raise ValueError if not)
4. Validate that value is reasonable (reject <-273.15K / absolute zero)
5. Handle bidirectional conversion (C→F, F→C, C→K, K→C, F→K, K→F)
6. Return temperature as float
7. Include comprehensive docstring
8. Provide test cases for all 6 conversion pairs
9. Include usage examples in main module
```

This is significantly more complex:
- Multiple parameters
- More validation rules
- More test cases
- Richer logic

**Observe**: Does agent handle complexity as smoothly as simple tasks? Where does it excel? Where does it struggle?

---

## Troubleshooting This Exercise

### Problem: Agent generates code but tests fail

**Solution**:
1. Review the error message in test output
2. This is OK! Shows agent tried to solve the problem
3. You could:
   - Edit code manually in Editor to fix
   - Or create new agent: "Fix the calculate_bmi function. Error: [describe error]"

### Problem: Task List or Plan doesn't generate

**Solution**:
1. Wait 30 seconds (sometimes slow on first run)
2. Refresh Agent Manager
3. Check logs: `tail ~/.antigravity/logs/agent.log`
4. If stuck, click "Stop" and try again

### Problem: Can't compare code to Implementation Plan

**Solution**:
1. Both Agent Manager (plan) and Editor (code) should be visible
2. If not, resize windows: drag divider between surfaces
3. Or use split-view mode (Settings → Layout)

---

## Reflection Questions

Answer these to solidify your learning:

1. **Task Decomposition**: Look at the Task List. Does agent's breakdown match how you would break down the problem?

2. **Implementation Decisions**: In Implementation Plan, agent proposed a specific approach to validation. Did you agree with it? Why/why not?

3. **Test Coverage**: Agent generated 5 test cases. Are they comprehensive? What additional tests would you add?

4. **Edge Cases**: Agent handled negative values. What other edge cases might cause issues?

5. **Artifact Transparency**: How does the artifact system (Task List → Plan → Code → Tests → Walkthrough) compare to just seeing final code without intermediate steps?

---

## Next: Iteration Skill

In Lesson 7, you'll learn to **iterate** on agent work:
- "That function doesn't handle infinity properly. Fix it."
- "Refactor this for better performance."
- "Add more comprehensive error messages."

This exercise is your foundation—you understand how agents approach problems. Next, you'll guide them through refinement cycles.

---

## Time Check

This exercise took 15-20 minutes. You now understand:
- Agent creation workflow
- All three artifact types
- How agents decompose tasks
- Testing and validation
- Multi-file projects

You're ready for Lesson 7's more advanced topics.

---

## Summary

| Stage | What Happened | What You Learned |
|-------|--------------|-----------------|
| **Agent Creation** | Entered task, chose provider | Tasks drive agent behavior |
| **Planning** | Task List + Implementation Plan generated | Agents are systematic, transparent |
| **Implementation** | Code files appeared in Editor | Agents follow their own plans |
| **Validation** | Tests ran automatically | Agents verify their own work |
| **Documentation** | Walkthrough artifact created | Artifacts are first-class output |

**Great work!** You're building real skills with agent-assisted development.
