---
title: "Capstone Part 2: Implement in Parallel With Two Companions"
chapter: 32
lesson: 9
lesson_type: "capstone_part_2"
duration: "3 hours"
skills:
  - name: "Parallel Implementation"
    proficiency: "B1"
    category: "Technical"
  - name: "Multi-Agent Coordination"
    proficiency: "B1"
    category: "Conceptual"
  - name: "Integration Planning"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Execute specification-driven implementation in parallel with multiple AI agents (B1)"
  - "Coordinate independent work without synchronous communication (B1)"
  - "Integrate parallel work back into cohesive system (B1)"
---

# Capstone Part 2: Implement in Parallel With Two Companions

## What You're About to Do

In Part 1, you prepared specifications for a team. Now you'll actually see team-scale development in action.

You'll run an experiment:
1. Create two separate AI conversations (two "companions")
2. Give Companion A the Feature 1 specification (no information about Feature 2)
3. Give Companion B the Feature 2 specification (plus the integration contract)
4. Watch them implement in parallel without any direct communication
5. Integrate their outputs
6. Evaluate whether your specs were clear enough

This simulates how real teams work—developers working independently, coordinating through specs.

---

## Part 1: Set Up Your Two AI Companions

### Step 1: Open Conversation for Companion A

Create a new conversation with your AI companion. Name it clearly:
**"Feature 1 Implementation: [Feature Name]"**

In that conversation, write this initial message:

> **Prompt**: "You are implementing Feature 1 of a grading system for a team of developers. Your team is coordinating through specifications, not meetings. Here is your specification: [paste your Feature 1 specification from Part 1]. Your job is to build a complete Python implementation that: (1) Matches this specification exactly, (2) Includes type hints on every function, (3) Includes PEP 257 docstrings, (4) Validates all inputs and handles all error cases, (5) Includes comprehensive test cases, (6) Is production-ready. Do not guess about other features or how this integrates with other code. Build only what the spec requires. When complete, provide the full implementation with tests."

**Critical**: Do NOT tell Companion A about Feature 2. Do NOT share Feature 2's specification. Companion A should only know their own feature.

### Step 2: Open Conversation for Companion B

Create a second separate conversation. Name it:
**"Feature 2 Implementation: [Feature Name]"**

In that conversation, write this initial message:

> **Prompt**: "You are implementing Feature 2 of a grading system for a team of developers. Your team is coordinating through specifications, not meetings. Here is your specification: [paste your Feature 2 specification from Part 1]. Additionally, here is the integration contract showing what Feature 1 will provide: [paste the integration contract]. Your input will come from Feature 1 in this exact format. Your job is to build a complete Python implementation that: (1) Matches this specification exactly, (2) Accepts input in the format from Feature 1, (3) Includes type hints on every function, (4) Includes PEP 257 docstrings, (5) Validates all inputs and handles all error cases, (6) Includes comprehensive test cases, (7) Is production-ready. Do not look at Feature 1's implementation. Trust that it will produce the output format specified in the integration contract. When complete, provide the full implementation with tests."

**Critical**: Companion B knows about Feature 1's output format (the integration contract) but NOT about Feature 1's implementation details.

---

## Part 2: Companion A Implements Feature 1

Now you'll guide Companion A through implementation.

### Step 1: Request Implementation

In Companion A's conversation, write:

> "Now, implement Feature 1 completely. Show me the full code with: all functions, type hints, docstrings, error handling, and test cases. Make sure the output matches the integration contract exactly."

### Step 2: Review Companion A's Output

When Companion A provides code:
- Save it to a file: `feature_1_implementation.py`
- Review the code for:
  - Does it include all required functions?
  - Are type hints present?
  - Are docstrings present?
  - Does error handling match the spec?
  - Do tests cover acceptance criteria?
  - Does output format match the integration contract?

### Step 3: Ask Clarifying Questions (If Needed)

If something isn't clear:

> "In your implementation of [function], I notice [observation]. The spec says [spec requirement]. Does your implementation match the spec? If not, please adjust."

Let Companion A refine until the implementation matches the spec exactly.

### Step 4: Test Companion A's Code (Optional)

If you want to be thorough:
1. Copy Companion A's implementation to a Python file
2. Run the test cases they provided
3. Verify tests pass
4. Try one manual test case

This validates that the implementation works.

---

## Part 3: Companion B Implements Feature 2

Now Companion B implements Feature 2, using Feature 1's output format.

### Step 1: Request Implementation

In Companion B's conversation, write:

> "Now, implement Feature 2 completely. Remember that your input comes from Feature 1 in the JSON format shown in the integration contract. Show me the full code with: all functions, type hints, docstrings, error handling, and test cases. Make sure your tests use the expected input format from Feature 1."

### Step 2: Review Companion B's Output

When Companion B provides code:
- Save it to a file: `feature_2_implementation.py`
- Review the code for:
  - Does it include all required functions?
  - Are type hints present?
  - Are docstrings present?
  - Does it accept the integration contract format?
  - Does error handling match the spec?
  - Do tests use the correct input format from Feature 1?

### Step 3: Ask Clarifying Questions (If Needed)

If something doesn't align with the integration contract:

> "In your implementation, you accept input in [format]. The integration contract specifies [contract format]. Do these match? If not, please adjust."

Let Companion B refine until the implementation matches both the spec AND the integration contract.

### Step 4: Note the Integration Points

As you review Companion B's code:
- Identify exactly how it consumes Feature 1's output
- Document any assumptions Companion B made
- These assumptions will either validate or reveal issues in your specs

---

## Part 4: The Critical Moment - Integration

Now comes the real test. Do Feature 1 and Feature 2 work together?

### Step 1: Create Integration Script

Create a new Python file called `integration_test.py`:

```python
"""
Integration Test: Feature 1 → Feature 2

This test validates that Feature 1's output feeds correctly into Feature 2's input.
"""

from feature_1_implementation import [main_function_from_feature_1]
from feature_2_implementation import [main_function_from_feature_2]

# Test Data
submission_text = "The quick brown fox jumps over the lazy dog."
rubric_criteria = [
    {"name": "Grammar", "weight": 0.3},
    {"name": "Clarity", "weight": 0.4},
    {"name": "Organization", "weight": 0.3}
]

print("=" * 60)
print("INTEGRATION TEST: Grading System")
print("=" * 60)

# Step 1: Run Feature 1 (Grading Engine)
print("\n[1] Running Feature 1: Grading Engine")
print("-" * 60)
try:
    grade_result = [main_function_from_feature_1](submission_text, rubric_criteria)
    print(f"✓ Feature 1 completed successfully")
    print(f"  Grade: {grade_result['grade']}")
    print(f"  Explanation: {grade_result['explanation'][:100]}...")
except Exception as e:
    print(f"✗ Feature 1 failed: {e}")
    exit(1)

# Step 2: Validate Feature 1 Output Format
print("\n[2] Validating Feature 1 Output Format")
print("-" * 60)
required_fields = ['grade', 'explanation', 'submission_id', 'timestamp']
missing_fields = [f for f in required_fields if f not in grade_result]
if missing_fields:
    print(f"✗ Feature 1 output is missing fields: {missing_fields}")
    exit(1)
else:
    print(f"✓ Feature 1 output has all required fields")

# Step 3: Run Feature 2 (Feedback Generator)
print("\n[3] Running Feature 2: Feedback Generator")
print("-" * 60)
try:
    feedback_input = {
        "submission_text": submission_text,
        "grade_result": grade_result,
        "rubric_criteria": rubric_criteria
    }
    feedback = [main_function_from_feature_2](**feedback_input)
    print(f"✓ Feature 2 completed successfully")
    print(f"  Feedback: {feedback[:100]}...")
except Exception as e:
    print(f"✗ Feature 2 failed: {e}")
    print(f"  This might indicate a mismatch in the integration contract")
    exit(1)

# Step 4: Validate Final Output
print("\n[4] Validating Final Output")
print("-" * 60)
print(f"✓ Grade: {grade_result['grade']}")
print(f"✓ Feedback length: {len(feedback)} characters")
print(f"✓ Feedback preview: {feedback[:100]}...")

print("\n" + "=" * 60)
print("INTEGRATION TEST PASSED!")
print("=" * 60)
print("\nConclusion:")
print("Your specifications were clear enough that two developers")
print("could implement in parallel and integrate seamlessly.")
```

### Step 2: Run the Integration Test

Run the integration test:
```bash
python integration_test.py
```

**Three possible outcomes**:

**Outcome A: Success (Integration Test Passes)**
```
INTEGRATION TEST PASSED!

Your specifications were clear enough that two developers
could implement in parallel and integrate seamlessly.
```

This means:
- Feature 1 output format matched expectations
- Feature 2 accepted Feature 1 output without modification
- Specs were clear enough to guide independent development

**Outcome B: Type/Format Mismatch (Moderate Issue)**
```
✗ Feature 2 failed: expected 'grade' but got 'grade_score'

This might indicate a mismatch in the integration contract
```

This means:
- Companion A and B interpreted the integration contract differently
- One of them deviated from the spec
- Minor fixes will resolve this

**Outcome C: Logic Mismatch (Major Issue)**
```
✗ Feature 2 failed: Feature 1 output doesn't include grades

This might indicate a mismatch in the integration contract
```

This means:
- The specs were genuinely ambiguous
- Team members made different assumptions
- Specs need significant clarification

### Step 3: Debug Any Failures

If the integration test fails:

1. **Identify the exact mismatch**:
   - What did Feature 1 produce?
   - What did Feature 2 expect?
   - Where do they differ?

2. **Determine root cause**:
   - Was the integration contract unclear?
   - Did one companion misinterpret the spec?
   - Was there a typo or naming mismatch?

3. **Document the issue**:
   - Write down what went wrong
   - This reveals where your specs need clarity

4. **Fix and retry**:
   - Either refine the spec and re-implement
   - Or make a minor adjustment to align them
   - Re-run the integration test

---

## Part 5: Analyze What Happened

Now reflect on the experiment.

### Step 1: Self-Reflection

Answer these questions:

1. **Did the integration test pass on the first try?**
   - If yes: Congratulations, your specs were clear
   - If no: Where was the mismatch?

2. **What surprised you** about the implementations?
   - Did Companion A interpret the spec differently than you expected?
   - Did Companion B make assumptions you didn't anticipate?

3. **How much time did it take?**
   - Feature 1 implementation + testing: [X hours]
   - Feature 2 implementation + testing: [X hours]
   - Integration debugging: [X hours]
   - Total: [X hours]

4. **What would have made specs clearer?**
   - More examples?
   - More detail on error handling?
   - Diagrams showing data flow?
   - More precise naming?

### Step 2: Ask Your Companion for Analysis

Ask one of your companions (use Companion A for this):

> **Prompt**: "I just ran an experiment where two AI systems implemented two features in parallel using specifications. Feature 1 and Feature 2 were implemented completely independently. [If integration passed: They integrated perfectly on the first try!] [If integration failed: There was a mismatch in [describe mismatch]]. What does this tell us about specification quality? What went well? What could be improved?"

Your companion will provide insights like:

- If specs were clear: "Your specs were precise and unambiguous. Both implementations aligned immediately because specifications defined clear contracts."
- If specs were unclear: "The mismatch in [area] reveals that your spec was ambiguous here. Here's how to clarify it..."
- What worked well: "Your acceptance criteria were specific, which meant both implementations could validate they were correct."
- What to improve: "More detailed examples in the integration contract would have prevented this mismatch."

### Step 3: Document Lessons Learned

Create a document called **"Specification Clarity Report"** that includes:

```markdown
# Specification Clarity Report

## The Experiment

Two AI systems implemented two grading system features independently:
- Feature 1: [Feature name]
- Feature 2: [Feature name]

They had no direct communication. They only had:
- Their individual feature specifications
- A shared integration contract

## Results

Integration test result: [PASSED / FAILED]

If passed:
- Time to integration: [X hours]
- Issues encountered: [0 / X]
- Rework needed: [0% / X%]

If failed:
- Mismatch: [Description]
- Root cause: [Ambiguous spec / Typo / Misinterpretation]
- Time to fix: [X hours]

## What This Reveals

Specifications that can guide independent parallel development must:
1. [Something you learned from this experiment]
2. [Something else you learned]
3. [Something else you learned]

## Specification Quality Score

Rate your specification on:
- Clarity: [1-10] (How unambiguous is it?)
- Completeness: [1-10] (Does it answer all questions?)
- Accuracy: [1-10] (Is everything correct?)
- Examples: [1-10] (Are examples helpful and correct?)
- Overall: [1-10] (Would you use this spec for a real team?)

Areas for improvement:
- [Area 1]: Improve by [how]
- [Area 2]: Improve by [how]

## Professional Insight

This experiment simulates how real teams work at scale:
- Stripe: 100,000+ companies implement features in parallel using Stripe's API spec
- Amazon: 5000+ engineers build independently using service specs
- Google Cloud: Hundreds of services coordinate through service contracts

Your specifications either enable or prevent this kind of parallelization.
```

---

## Part 6: Reflection Questions

Before moving to Part 3, answer:

1. **How confident would you be** handing these specs to a real team?

2. **What would you change** in your specs before handing them to real developers?

3. **How did AI handle parallelization?** Could real humans do what the AI companions did (implement independently without talking)?

4. **What's the cost-benefit** of specification quality?
   - Cost: Time spent on specs upfront
   - Benefit: Smooth parallel development with no miscommunication

5. **How does this apply** to your next real project?

---

## What's Next

You've completed Part 2 of the capstone. You've experienced:
- Writing specifications that guide independent development
- Testing whether specs are clear enough to prevent miscommunication
- Debugging integration issues that reveal spec ambiguities
- Understanding the direct relationship between spec clarity and development speed

**In Capstone Part 3**, you'll reflect on this experience and extract lessons that apply to Parts 6-13 (where you'll learn about agent behavior specs, API specs, deployment specs, and more).

---

## Quick Reference: Specification Quality Checklist

Before you move forward, validate your specifications:

- [ ] Each feature spec is standalone (can be understood without other specs)
- [ ] Integration contract specifies exact data format (JSON schema)
- [ ] Input requirements specify format, constraints, and examples
- [ ] Output requirements specify format, constraints, and examples
- [ ] Acceptance criteria are measurable and testable
- [ ] Error cases specify what happens when things go wrong
- [ ] Constraints specify response time, availability, etc.
- [ ] Example inputs and outputs are provided
- [ ] No ambiguous language ("should be", "maybe", "approximately")
- [ ] Integration test validates that specs were clear

If you can check all these boxes, your specs are ready for a real team.
