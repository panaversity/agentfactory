---
title: "Watch Your Companions Coordinate (Without Talking)"
chapter: 32
lesson: 1
duration: "2 hours"
skills:
  - name: "Team Coordination"
    proficiency: "B1"
    category: "Conceptual"
  - name: "AI Agent Collaboration"
    proficiency: "A2"
    category: "Conceptual"
  - name: "Specification as Communication"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Understand how specifications enable coordination without conversation (B1)"
  - "Observe how multiple AI agents execute the same specification consistently (A2)"
  - "Recognize specifications as the lingua franca for distributed team execution (B1)"
---

# Watch Your Companions Coordinate (Without Talking)

## What You're About to Witness

Imagine two engineers work on the same project. They never talk. No meetings, no emails, no Slack messages. Yet their code fits together perfectly, as if they planned it together. How is this possible?

The answer: clear specifications.

In this lesson, you'll run a real experiment. You'll build a grading system specification, decompose it into two independent features, give each feature to a different AI companion, and watch them implement in parallel without any direct communication. Then you'll integrate their outputs and discover something profound: **clear specs eliminate the need for synchronous communication**.

This is how professional teams scale. This is why specification-driven development works at 5 people, 50 people, or 500 people.

---

## Part 1: The Grading System Specification (Your Starting Point)

Before we begin, you need to build a grading system specification. If you haven't completed it, do that first. It should include:

- **What the system does**: Accept student submissions and rubrics, produce grades and feedback
- **Key features**: Grade calculation, feedback generation, error handling
- **Acceptance criteria**: How you know it works correctly
- **Constraints**: Response time, reliability, scalability

If you have this spec, great. We'll use it as the foundation for this experiment.

---

## Part 2: How Specs Enable Parallelization

Before we run the experiment, let's understand the theory.

When teams work in parallel without specifications, they need constant communication:
- "What data format are you using for submissions?"
- "How do you represent rubrics?"
- "Should feedback be a string or structured data?"
- "What happens if grading fails?"

This creates bottlenecks. Progress stalls while people communicate.

With specifications, the spec answers all these questions *before* implementation starts. Two engineers can work independently because they share a common source of truth: the specification.

**The Pattern**:
1. Write a clear specification (answers all questions)
2. Decompose into independent features
3. Define integration contracts (how features connect)
4. Assign features to team members
5. Team members implement from their spec alone
6. Integrate the outputs
7. Watch it work because specs were clear

This lesson walks you through this pattern using AI companions.

---

## Part 3: Decompose Your Specification

Your grading system likely has at least two natural features. Let's identify them.

### Step 1: Build Specification

Get the specification you wrote for the grading system. Review it carefully. What are the major components?

### Step 2: Identify Feature Boundaries (With Your Companion's Help)

Ask your AI companion:

> **Prompt**: "I have a grading system specification. Here it is: [paste your spec]. Looking at this specification, what are the 2-3 most independent features I could decompose this into? For each feature, tell me: (1) What it does, (2) Its inputs, (3) Its outputs, (4) How other features depend on it."

Your companion will likely suggest something like:
- **Feature 1: Grading Engine** — Takes a submission and rubric, calculates a grade (0-100) and explanation
- **Feature 2: Feedback Generator** — Takes a submission, grade, and rubric, generates personalized feedback (200-500 words)

These are naturally independent. Feature 2 depends on Feature 1's output, but they don't need to communicate during development.

### Step 3: Document the Integration Contract

The key to parallel development is defining exactly how features connect. Ask your companion:

> **Prompt**: "Here are my two features: [Feature 1 and Feature 2 descriptions]. How should Feature 1's output connect to Feature 2's input? Please define this as a JSON schema showing the exact data structure that Feature 1 produces and Feature 2 expects."

Your companion might produce something like:

```json
{
  "grading_output": {
    "submission_id": "string (UUID)",
    "score": "number (0-100)",
    "explanation": "string (max 500 chars)",
    "rubric_used": "object (the original rubric)",
    "timestamp": "ISO 8601 datetime"
  }
}
```

Save this schema. It's the contract. Feature 2 will expect exactly this format.

### Step 4: Write Feature Specifications (Standalone and Complete)

Now create two separate specifications, each one complete and standalone:

**Feature 1 Specification** should include:
- Purpose: What this feature does
- Inputs: What data it receives (with type specifications)
- Outputs: What data it produces (with the schema from Step 3)
- Acceptance criteria: How you know it works
- Error cases: What happens if inputs are invalid
- Constraints: Response time, reliability, etc.

**Feature 2 Specification** should include:
- Purpose: What this feature does
- Inputs: What data it receives (the output from Feature 1)
- Outputs: What data it produces
- Acceptance criteria: How you know it works
- Error cases: What happens if inputs are invalid
- Constraints: Response time, reliability, etc.

Each spec should be complete enough that someone could implement it without seeing the other spec.

---

## Part 4: Set Up Your Two Companions

This is the critical part. You'll set up two separate conversations, one for each feature. Each companion gets only their feature spec.

### Step 1: Create Conversation with Companion A

In your AI tool (Claude Code, ChatGPT, Gemini, etc.), start a new conversation. Name it: **"Feature 1: Grading Engine Implementation"**

Give Companion A this information:

> **Initial Prompt**: "You are implementing Feature 1 of a grading system. Here is your specification: [paste Feature 1 spec]. Your job is to build a Python implementation of this feature that: (1) Matches this spec exactly, (2) Includes type hints on all functions, (3) Includes docstrings in PEP 257 format, (4) Passes all acceptance criteria, (5) Handles all error cases listed in the spec. Do not look at other features or implementation details outside this spec. Your only goal is to implement Feature 1 correctly."

**IMPORTANT**: Do NOT give Companion A any information about Feature 2. Companion A should not know that Feature 2 exists.

### Step 2: Create Conversation with Companion B

Start a second separate conversation. Name it: **"Feature 2: Feedback Generator Implementation"**

Give Companion B this information:

> **Initial Prompt**: "You are implementing Feature 2 of a grading system. Here is your specification: [paste Feature 2 spec]. Your job is to build a Python implementation of this feature that: (1) Matches this spec exactly, (2) Includes type hints on all functions, (3) Includes docstrings in PEP 257 format, (4) Passes all acceptance criteria, (5) Handles all error cases listed in the spec. Do not look at other features or implementation details outside this spec. Your only goal is to implement Feature 2 correctly."

**IMPORTANT**: Do NOT give Companion B the Feature 1 spec. Companion B should only know that their feature receives input in a specific format (the integration contract from Step 3).

---

## Part 5: Watch Them Implement in Parallel

Now the experiment happens. In each conversation, ask your companion to build their feature:

**To Companion A:**
> "Build the complete implementation of Feature 1 according to the specification. Include all functions, type hints, docstrings, error handling, and test cases. Show me the complete code when done."

**To Companion B:**
> "Build the complete implementation of Feature 2 according to the specification. Remember that your input will come from Feature 1 in this JSON format: [paste the integration contract]. Include all functions, type hints, docstrings, error handling, and test cases. Show me the complete code when done."

Let both companions work. Watch what they produce. Notice:
- Do they ask clarifying questions about the other feature? (They shouldn't; the spec should answer everything)
- Do they make assumptions about the integration format? (They shouldn't; the integration contract is clear)
- How much code does each one produce?
- Do they both follow the spec exactly?

---

## Part 6: Integrate the Results

Now comes the magic moment. Take Companion A's output (Feature 1 implementation) and Companion B's output (Feature 2 implementation) and put them together.

### Step 1: Create an Integration Test File

Create a simple Python file that demonstrates integration:

```python
"""
Integration test: Feature 1 output → Feature 2 input
"""
from feature_1_implementation import grade_submission
from feature_2_implementation import generate_feedback

# Example data
submission = {
    "student_id": "s123",
    "text": "The quick brown fox jumps over the lazy dog."
}

rubric = {
    "criteria": [
        {"name": "Grammar", "weight": 0.3},
        {"name": "Clarity", "weight": 0.4},
        {"name": "Organization", "weight": 0.3}
    ]
}

# Run Feature 1: Grading
grade_result = grade_submission(submission, rubric)
print("Grading Result:")
print(grade_result)

# Run Feature 2: Feedback (using Feature 1's output)
feedback = generate_feedback(
    submission=submission,
    grade_result=grade_result,
    rubric=rubric
)
print("\nFeedback:")
print(feedback)

# Verify integration
assert isinstance(grade_result["score"], (int, float))
assert 0 <= grade_result["score"] <= 100
assert isinstance(feedback, str)
assert len(feedback) > 0
print("\nIntegration test PASSED!")
```

### Step 2: Run the Integration Test

Run this file. Does it work? If yes, then:
- Feature 1 produced the exact format Feature 2 expected
- Feature 2 accepted Feature 1's output without modification
- They integrated seamlessly

**This is the proof that specs enable parallelization.**

If the integration test fails, debug it:
- What format did Feature 1 actually produce?
- What format does Feature 2 expect?
- Where's the mismatch?

Document the mismatch. It reveals where the integration contract was unclear.

---

## Part 7: Analyze What Happened

Now reflect on the experiment. Ask your companion:

> **Prompt**: "I just ran an experiment where two AI systems implemented different features of a grading system in parallel. Feature 1 produced a grade. Feature 2 used that grade to generate feedback. They coordinated perfectly without talking. Here's what I'm curious about: (1) How did they coordinate without direct communication? (2) What made this possible? (3) What would have happened if the specifications were vague? (4) How does this scale to 5 features? 10 features? 100 features?"

Your companion will explain something like:

> "Your companions coordinated through the specifications. Feature 1 didn't need to ask Feature 2 what format to use—the integration contract specified it. Feature 2 didn't need to negotiate with Feature 1 about inputs—the spec defined them. This works because specifications are a contract. Everyone commits to meeting the contract. When contracts are clear, parallelization is automatic.
>
> If specifications were vague—if Feature 1 didn't know what format to produce, or Feature 2 didn't know what to expect—they would have needed direct communication. They would have asked 'What format?' 'What should I expect?' 'Can you change it?' This creates bottlenecks.
>
> At scale, this pattern is essential. 100 engineers can't all talk to each other. Instead, they each receive clear specifications for their feature, implement to the spec, and integration happens because specs define the contracts."

---

## Part 8: Create Your Decomposition Diagram

Document your learning by creating a visual representation of what you discovered:

**Your diagram should show**:
1. Original specification (at the top)
2. Decomposition into Feature 1 and Feature 2 (in the middle)
3. Integration contract (how they connect)
4. Parallel implementation (Feature 1 and Feature 2 being built independently)
5. Integration result (working together seamlessly)

You can create this as:
- An ASCII diagram in a markdown file
- A drawn diagram and photograph
- A Mermaid diagram (if your documentation tool supports it)
- A simple text description with numbered steps

Save this somewhere you'll reference later. It's your visual proof that specs enable parallelization.

---

## Part 9: Reflection Questions

Before moving to the next lesson, answer these questions for yourself:

1. **What surprised you most** about watching two companions coordinate without talking?
2. **Where would this break down** if the specifications were vague? Give a concrete example.
3. **How is this different** from traditional team coordination (meetings, emails, Slack)?
4. **What happens at scale?** If you had 10 features instead of 2, how would this pattern change?
5. **What's the cost of clarity?** How much time did you spend writing specifications? How much time did it save during implementation?

Write brief answers to these questions. You'll reference them in the capstone.

---

## Try With AI

Now it's your turn to solidify this learning by exploring a real-world example.

**Setup**: Use your AI companion tool (Claude Code, ChatGPT, Gemini CLI, or your preferred assistant). This lesson teaches concepts that benefit from conversational exploration.

**Prompt Set**:

1. **Understand the principle:**
   > "Explain specification-driven parallelization. Use an analogy from a real-world team or project (not software). Show me how specifications eliminate the need for synchronous communication."

2. **Explore the constraints:**
   > "What happens to parallelization when specifications are 50% complete? 75% complete? Show me realistic failure modes with examples."

3. **Scale your thinking:**
   > "I just built Feature 1 and Feature 2 in parallel using clear specs. How would I extend this to 5 independent features? 10 features? What breaks at each scale?"

4. **Challenge yourself (optional):**
   > "Design a system where this approach fails. What kind of features can't be parallelized through specs alone? When does direct communication still matter?"

**Expected Outcomes**:
- You understand why specs enable parallelization (not magic; clear contracts)
- You recognize where this approach works (well-defined interfaces)
- You know where it breaks (tight coupling, ambiguous requirements)
- You can articulate the difference between specification-driven and synchronous-communication-driven teams

**Safety Note**: As you explore this pattern, remember that real teams need both specifications AND communication. Specs aren't about eliminating people talking—they're about making communication more effective. The best teams write clear specs AND talk frequently about what those specs mean. Specifications are a tool for clarity, not isolation.
