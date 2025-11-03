---
title: "SMART Acceptance Criteria: Writing Clear Requirements"
chapter: 31
lesson: 1
duration_minutes: 120

skills:
  - name: "Understanding SMART Criteria"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Communication & Problem-Solving"
    measurable_at_this_level: "Student identifies which criteria are SMART and which are vague; explains each component"

  - name: "Identifying Vague Requirements"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Understand, Analyze"
    digcomp_area: "Communication & Problem-Solving"
    measurable_at_this_level: "Student spots ambiguous language ('helpful', 'fast', 'easy') and proposes specific alternatives"

  - name: "Writing Testable Criteria"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student writes criteria with clear pass/fail conditions; no interpretation needed"

learning_objectives:
  - objective: "Convert vague requirements into SMART acceptance criteria that are specific, measurable, achievable, relevant, and time-bound"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Given a vague feature requirement, student writes 3+ SMART criteria with testable pass/fail conditions"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (SMART Framework, Vagueness Problem, Clarity Solution, Testability, Cascade Starting Point) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Convert vague enterprise requirements (production performance targets, multi-tenant scenarios) to SMART criteria; evaluate criteria completeness at B1 level"
  remedial_for_struggling: "Guided template with fill-in-the-blank SMART criteria; simplified examples focusing on S and M only first"
---

# Lesson 1: SMART Acceptance Criteria

## Opening Hook

Imagine you ask an AI assistant: "Build a calculator that gives helpful feedback and is super fast." You run the code it generates and immediately spot the problem. The feedback is generic, the speed is untested, and the features don't match what you imagined.

This is the problem that haunts every software project: **vague requirements produce useless code**. In AI-native development, this problem gets worse because AI can't read your mind. It takes your words literally. If you say "helpful," it might generate a generic message. If you say "fast," it might optimize one part while ignoring the bottleneck.

The solution is simple but powerful: **write acceptance criteria so clear that AI (and humans) can't misinterpret them**. This lesson teaches you how.

---

## Part A: The Problem — Experience Vagueness (30 min)

### Vague Requirements Fail

Start with a vague feature request. Here's an example:

> "The calculator should give helpful feedback and provide fast error handling."

This sounds reasonable in conversation, but watch what happens when you ask AI to build from it.

**Vague Requirement Demo:**

When you ask an AI assistant to build a calculator with this requirement, you get:

```python
def add(a, b):
    """Adds two numbers with feedback."""
    result = a + b
    print(f"Calculation complete: {result}")
    return result
```

Is this "helpful"? Is it "fast"? According to a literal reading of the requirement, yes! But it misses your actual intent. You probably wanted:

- Error messages to identify which operation failed
- Response time under 100ms
- User-friendly explanations, not just generic acknowledgments

The problem cascades: bad specification → bad code → bad user experience → wasted time fixing it.

### Why Vagueness Breaks the Cascade

This is the fundamental insight of specification-driven development: **Vagueness starts at the specification level and cascades through the entire workflow.**

- **Spec is vague** → Plan is unclear → Tasks are confusing → Code misses the mark
- **Spec is clear** → Plan is logical → Tasks are atomic → Code works perfectly

Your job in this lesson: become an expert at clarity. Write requirements so specific that AI can execute them without guessing.

---

## Part B: The Framework — SMART Criteria (40 min)

### What Makes Criteria "SMART"?

SMART is an acronym that stands for **Specific, Measurable, Achievable, Relevant, Time-bound**. Let's explore each:

#### **S — Specific**

A specific criterion tells exactly what should happen, not vaguely what's desired.

| Vague | Specific |
|-------|----------|
| "Display helpful error messages" | "Display error messages in format: ERROR: [operation name] failed because [reason]" |
| "The system should be user-friendly" | "Users can complete a calculation and view history in under 3 clicks" |
| "Provide good feedback" | "Show feedback within 1 second of user action; include operation name and result" |

**How to write specific criteria**:
- Name the exact action ("Display error messages")
- Describe the exact format or output ("In format: ERROR: [operation] failed")
- Avoid subjective words ("helpful," "user-friendly," "good")
- Use concrete examples

#### **M — Measurable**

A measurable criterion includes numbers, metrics, or clear pass/fail conditions.

| Not Measurable | Measurable |
|---|---|
| "The calculator should be fast" | "Response time ≤ 100ms for all operations" |
| "Handle large numbers" | "Support numbers up to 1 billion (10^9)" |
| "Show useful calculation history" | "Display last 10 calculations; allow user to clear history with one command" |

**How to write measurable criteria**:
- Include specific numbers ("100ms," "10 digits," "50 items")
- Use comparison operators ("≤," "≥," "exactly")
- Define the unit ("milliseconds," "characters," "entries")
- Make it testable: "I can verify this by [running test / checking output / timing execution]"

#### **A — Achievable**

An achievable criterion is realistic given project constraints. It doesn't require magic or unrealistic scope.

| Not Achievable | Achievable |
|---|---|
| "AI predicts user intent perfectly" | "When user enters ambiguous input, show 3 interpretation options" |
| "Works offline and online simultaneously" | "Works offline using local storage; syncs when connection returns" |
| "Supports unlimited users with no lag" | "Supports 100 concurrent users with ≤ 1 second response time" |

**How to write achievable criteria**:
- Understand the project constraints (time, resources, technology)
- Ask: "Is this humanly possible within our timeline?"
- Break impossible requirements into achievable parts
- Clarify assumptions ("Works for standard Python environments on Windows, Mac, Linux")

#### **R — Relevant**

A relevant criterion matters to the project's goals and users.

| Not Relevant | Relevant |
|---|---|
| "Display calculation history in 47 different languages" | "Display calculation history in English; support future language expansion" |
| "Optimize for quantum computers" | "Code runs efficiently on current laptops and servers" |
| "Include blockchain verification" | "Ensure calculation accuracy by including automated tests for every operation" |

**How to write relevant criteria**:
- Connect to project goals ("Why does this matter?")
- Consider actual user needs ("Who uses this? What do they actually need?")
- Skip gold-plating (features no one asked for)
- Ask: "Will this criterion directly serve a real user or business goal?"

#### **T — Time-bound**

A time-bound criterion specifies when something should happen or be completed.

| Not Time-bound | Time-bound |
|---|---|
| "Eventually add more features" | "MVP includes add, subtract, multiply, divide operations (complete by sprint 1)" |
| "Improve error handling someday" | "All error cases return meaningful messages by end of Week 1" |
| "Make it faster later" | "Performance baseline established by code review; optimization by Week 2" |

**How to write time-bound criteria**:
- Use milestones ("By Week 2," "Sprint 1," "Before code review")
- Define scope boundaries ("MVP includes X, Y, Z; advanced features in Phase 2")
- Be realistic ("We'll handle this edge case in the next iteration")

### Worked Example: Converting Vague to SMART

**Vague Feature Request:**
> "The grading system should be fair and give students useful feedback on their submissions."

**Converted to SMART Acceptance Criteria:**

1. **Specific + Measurable + Achievable + Relevant + Time-bound:**
   - Criterion 1: "The system calculates weighted grades (homework 30%, quizzes 20%, exams 50%) and displays the final grade to ±0.1% accuracy"
   - Criterion 2: "For each submission, generate feedback that includes: (1) score, (2) what was correct, (3) one area for improvement; feedback appears within 5 seconds of submission"
   - Criterion 3: "Support up to 100 students per teacher and 10 assignments per course without performance degradation"
   - Criterion 4: "By Week 1, teachers can view all student grades; by Week 2, students can view their own grades and feedback"

Each criterion is:
- **Specific**: You know exactly what should happen
- **Measurable**: You can test whether it works (weighted percentages, response time, user count, timeline)
- **Achievable**: Realistic for a Python implementation
- **Relevant**: Directly serves students and teachers
- **Time-bound**: Clear phases (Week 1, Week 2)

---

## Part C: Practice — Refine Real Requirements (40 min)

### Exercise 1: Identify Vague Terms

**Instructions**: Read the following feature request. Identify vague terms (words like "helpful," "fast," "good," "user-friendly"). Rewrite each vague term as a specific, measurable criterion.

**Feature Request:**
> "The calculator should have a helpful user interface and quick response times. It should handle errors gracefully and provide good feedback."

**Vague Terms to Identify**:
- "helpful" →
- "quick" →
- "gracefully" →
- "good" →

**Solution** (for instructor reference):
- "helpful" → "Easy to navigate; all buttons labeled clearly; operations visible on screen at once"
- "quick" → "Response time ≤ 100ms for all operations"
- "gracefully" → "Display error message for division by zero: 'ERROR: Cannot divide by zero. Enter a new expression.'"
- "good" → "Feedback includes operation name, result, and next available actions"

### Exercise 2: Write 5 SMART Acceptance Criteria

**Instructions**: Choose one feature from below. Write 5 acceptance criteria using the SMART framework.

**Feature Options**:
1. A to-do list app that lets users organize tasks
2. A quiz system that tests student knowledge
3. A password manager that stores credentials securely
4. A weather app that shows current conditions

**Template**:
```
Criterion 1 (Specific): [What exactly happens?]
Criterion 2 (Measurable): [How do we measure success?]
Criterion 3 (Achievable): [What's realistic?]
Criterion 4 (Relevant): [Why does this matter?]
Criterion 5 (Time-bound): [When should this be done?]
```

**Example Solution** (for a to-do list):
1. Users can add a task by typing text and pressing Enter
2. Each task displays with checkbox, text (max 100 characters), and delete button
3. Users can mark tasks complete (checkbox checked = 1-second animation, strikethrough text)
4. Support up to 100 tasks per user without lag
5. All features complete by end of Week 1

### Exercise 3: Trade and Critique

**Instructions**: Trade your 5 criteria with a partner (or imagine teaching a peer).

**Partner's Task**:
- Read the criteria aloud to the author
- After each criterion, ask: "Is this SMART?"
- If not, ask: "Which part is vague? (S/M/A/R/T?)"
- Suggest one improvement

**Reflection Question**:
- What made the most confusing criteria improve? (More specifics? More numbers? Clearer examples?)

---

## Part D: Key Insight — The Cascade Effect

Write this down: **SMART acceptance criteria are the foundation of the entire SpecifyPlus workflow.**

Here's why:

1. **Vague criteria** → AI builds generic code → validation fails → iteration hell
2. **SMART criteria** → AI builds specific code → validation passes → deployment ready

As you move through the remaining lessons, you'll see this pattern repeat:
- Lesson 2: Project structure enforces specification thinking
- Lesson 3: Specification templates embed SMART criteria thinking
- Lesson 4: `/sp.specify` tools identify gaps in criteria
- Lesson 5-7: Plans, tasks, and code all flow from criteria clarity

---

## Try With AI

**Tool**: ChatGPT web (or your AI companion if already set up)
**Duration**: 10-15 minutes

### Prompts

1. **Refine a Vague Requirement**:
   ```
   I have a vague feature request: "Build a calculator that gives helpful feedback and is super fast."

   Help me convert this to 5 SMART acceptance criteria. For each, tell me which part of SMART
   (Specific, Measurable, Achievable, Relevant, Time-bound) makes it clear.
   ```

2. **Review Your Criteria**:
   ```
   Here are my SMART acceptance criteria for [your feature]:
   [Paste your 5 criteria from Exercise 2]

   Are these truly SMART? Point out any that are still vague.
   For each, tell me which component (S/M/A/R/T) needs improvement.
   ```

3. **Compare Vague vs. SMART**:
   ```
   Compare these two versions of the same criterion:

   Vague: "The system should provide good error messages."
   SMART: "When user enters invalid input, display error message within 500ms that includes: (1) the operation name, (2) what was wrong, (3) an example of valid input."

   Why is the SMART version better for an AI building this feature?
   ```

### Expected Outcomes

- ChatGPT helps you identify vague terms in your own language
- You see concrete examples of SMART criteria for your chosen feature
- You understand why clarity matters for AI collaboration
- You're ready for Lesson 2 with stronger criteria-writing skills

### Safety & Ethics Note

**Remember**: AI tools won't magically understand vague requests. Your job is clarity. Write acceptance criteria so clear that an AI (or any developer) can't misinterpret your intent.

When you give clear criteria, AI becomes a powerful collaborator. When you're vague, you waste both your time and the AI's time on back-and-forth clarification.

---

## Checkpoint: Are You Ready for Lesson 2?

Before moving forward, verify you can do these:

- [ ] Identify vague terms in written requirements
- [ ] Explain what each letter of SMART stands for
- [ ] Write at least 3 acceptance criteria for a real feature
- [ ] Explain why "helpful," "fast," and "good" don't work without SMART conversion
- [ ] Understand that vague specifications cascade into vague code
