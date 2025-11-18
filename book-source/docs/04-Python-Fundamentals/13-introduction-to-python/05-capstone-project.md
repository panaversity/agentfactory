---
title: "Capstone Project – Personal Information Collector"
chapter: 13
lesson: 5
duration_minutes: 90

skills:
  - name: "Integration of All Python Fundamentals"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student builds working program applying variables, type hints, input, output, and formatted display"

  - name: "Specification-First Program Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student designs program in plain English before coding; matches code to specification"

  - name: "Program Testing and Verification"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student runs program end-to-end; tests with different inputs; verifies output matches specification"

learning_objectives:
  - objective: "Design and build an interactive program that demonstrates all Chapter 13 concepts"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Complete capstone program runs without errors and meets all requirements"

cognitive_load:
  new_concepts: 0
  assessment: "0 new concepts—pure integration of Lessons 1-4 ✓"

differentiation:
  extension_for_advanced: "Add database storage; implement multiple user profiles; create interactive menu system"
  remedial_for_struggling: "Provide detailed code template; break into micro-steps; validate each section separately"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/016-part-4-chapter-13/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 5: Capstone Project – Personal Information Collector

You've learned all the pieces. Now you'll assemble them into one complete, working program.

This capstone isn't just a code exercise. It's your first experience with **specification-driven development in miniature**. You'll:

1. **Understand WHAT** the program does (specification)
2. **Design HOW** you'll build it (planning)
3. **Code and test** (implementation)
4. **Verify it matches your design** (validation)

This cycle—spec, plan, implement, validate—is exactly how professional AI-Driven Development works.

## Understanding the Capstone

Throughout Chapter 13, you've learned individual pieces:
- Variables (Lesson 3)
- Type hints (Lesson 3)
- Syntax, `print()`, f-strings (Lesson 4)
- Now you'll use `input()` to ask users questions

The capstone program brings all these pieces together in a real program that does something useful: **collects user information and displays a formatted summary.**

This is specification-driven development in practice. You're not typing random code. You're building something with a clear purpose.

## Capstone Specification

**Program Purpose**: An interactive program that asks for user information (name, age, favorite color, hobby, city) and displays a personalized summary.

**Requirements**:

1. **Collect User Information** (5 pieces):
   - Name (string)
   - Age (string)
   - Favorite color (string)
   - Favorite hobby (string)
   - City (string)

2. **Use Type Hints for All Variables**
   - Every variable declares its type: `name: str = input(...)`

3. **Display Formatted Summary**
   - Use f-strings to show collected information
   - Make output conversational and clear

4. **Include Comments**
   - Section headers: `# Collect Information`, `# Display Summary`
   - Explain what each section does

**Success Criteria**:
- [ ] Program runs without errors
- [ ] Collects 5 pieces of information
- [ ] Has type hints on all variables (all `str`)
- [ ] Uses `input()` to collect data
- [ ] Uses f-strings for formatted output
- [ ] Includes comments explaining sections
- [ ] Displays formatted summary with user's information

## Phase 1: Design First (Plain English)

**Before you write any Python code, write your design in plain English.**

This is the specification phase. Answer these questions:

1. **What will the program do?**
   - Ask the user for their name, age, favorite color, hobby, and city
   - Store these answers in typed variables (all strings)
   - Display a nice summary

2. **What input do we need?**
   - Name (text)
   - Age (text)
   - Favorite color (text)
   - Favorite hobby (text)
   - City (text)

3. **What output should we show?**
   - A greeting
   - A formatted summary of their information
   - A thank you message

Write this design in a text document or comment in your code. Share it with your AI companion:

#### 💬 AI Colearning Prompt
> "Here's my program design: (1) Ask user for name, age, favorite color, hobby, city, (2) Store all as strings (no type conversion yet), (3) Display formatted summary with f-strings. Before I code this, does this design make sense? Should I use str for all variables or would int be better for age?"

#### 🎓 Expert Insight
> In AI-native development, capstone projects demonstrate specification-first thinking. You describe intent (collect information, format output) before writing code. This design → validate → implement pattern is how professional teams build software. Type hints in your spec tell AI exactly what data structures you need, preventing bugs before code is written.

## Phase 2: Step-by-Step Build

Now build your program step by step.

### Step 1: Collect Information

**The `input()` function** — The `input()` function asks the user to type something. It shows a prompt, waits for them to type and press Enter, then gives you what they typed as a string.

```python
# Collect Information
print("=== Personal Information Collector ===\n")

name: str = input("What is your name? ")
age: str = input("How old are you? ")
favorite_color: str = input("What is your favorite color? ")
hobby: str = input("What is your favorite hobby? ")
city: str = input("What city do you live in? ")
```

Run this so far. It asks five questions and stores all the answers as strings.

### Step 2: Display Summary

Use f-strings to display formatted output:

```python
# Display Summary
print("\n=== Your Profile ===")
print(f"Name: {name}")
print(f"Age: {age}")
print(f"Favorite Color: {favorite_color}")
print(f"Favorite Hobby: {hobby}")
print(f"City: {city}")

print(f"\nThank you, {name}! Your information has been recorded.")
print("This profile demonstrates:")
print("- Variables with type hints (all str)")
print("- input() function to collect information")
print("- F-strings to format output")
print("- Comments to explain sections")
```

## Complete Capstone Code

Here's the full program together:

```python
# Personal Information Collector
# This program collects user information and displays a formatted summary

# ===== COLLECT INFORMATION =====
print("=== Personal Information Collector ===\n")

name: str = input("What is your name? ")
age: str = input("How old are you? ")
favorite_color: str = input("What is your favorite color? ")
hobby: str = input("What is your favorite hobby? ")
city: str = input("What city do you live in? ")

# ===== DISPLAY SUMMARY =====
print("\n=== Your Profile ===")
print(f"Name: {name}")
print(f"Age: {age}")
print(f"Favorite Color: {favorite_color}")
print(f"Favorite Hobby: {hobby}")
print(f"City: {city}")

print(f"\nThank you, {name}! Your information has been recorded.")
print("This profile demonstrates:")
print("- Variables with type hints (all str)")
print("- input() function to collect information")
print("- F-strings to format output")
print("- Comments to explain sections")
```

## Phase 3: Test and Validate

### Run the Program

Save this code as `capstone.py`. Run it:

```
python capstone.py
```

You should see:
```
=== Personal Information Collector ===

What is your name? Alice
How old are you? 25
What is your favorite color? Blue
What is your favorite hobby? Reading
What city do you live in? Portland

=== Your Profile ===
Name: Alice
Age: 25
Favorite Color: Blue
Favorite Hobby: Reading
City: Portland

Thank you, Alice! Your information has been recorded.
This profile demonstrates:
- Variables with type hints (all str)
- input() function to collect information
- F-strings to format output
- Comments to explain sections
```

### Test Your Program

Run through with different inputs. Try:
- Your real information
- Made-up information
- Very short answers
- Very long answers

### Verify Against Specification

Does your program:
- [ ] Collect 5 pieces of information? ✓
- [ ] Have type hints on all variables? ✓
- [ ] Use `input()` to collect data? ✓
- [ ] Display formatted output with f-strings? ✓
- [ ] Include section comments? ✓
- [ ] Include comments explaining sections? ✓

If you checked everything, your capstone meets the specification.

#### 🤝 Practice Exercise

> **Ask your AI**: "Review my Personal Information Collector program: [paste your complete code]. Check: (1) Are all type hints present and correct? (2) Is variable naming descriptive? (3) Are f-strings used appropriately? (4) Do comments explain intent? (5) Suggest one improvement for code quality. Then explain why professional developers always validate code (whether human or AI-generated) before deployment."

**Expected Outcome**: You'll practice code review with AI partnership, understand that validation is non-negotiable in professional development, learn to accept constructive feedback gracefully, and see how iteration improves code quality systematically.

## Extending Your Program

Once your capstone works, try extensions:

**Extension 1**: Add one more question
- Ask for favorite book or movie
- Add type hint (`str`)
- Include in output summary

**Extension 2**: Add formatting
- Use more section dividers
- Add emoji to make output fun (optional)
- Create a more detailed profile display

**Extension 3**: Ask your AI for suggestions
- "How can I make my capstone program more interesting?"
- "What other information could I collect?"
- "How can I improve the output formatting?"

Share your extension with AI:

> "I extended my capstone with [description]. Does it look good? Any improvements?"

## Common Mistakes

**Mistake 1**: Forgetting type hints

```python
name = input("What is your name? ")        # No type hint
name: str = input("What is your name? ")   # Type hint—describes intent
```

Type hints are core. Every variable gets one.

**Mistake 2**: Not using f-strings for formatted output

```python
print("Name: " + name)           # String concatenation (harder to read)
print(f"Name: {name}")           # F-string (clear and readable)
```

F-strings are the modern way to format output.

**Mistake 3**: Forgetting comments

```python
# No comments - harder to understand
name: str = input("What is your name? ")
print(f"Name: {name}")

# With comments - shows intent
# Collect user information
name: str = input("What is your name? ")
# Display the information
print(f"Name: {name}")
```

Comments explain WHAT and WHY, not just HOW.

---

## Try With AI

Build a complete information collector integrating all Chapter 13 concepts.

**🔍 Explore Domain Design:**
> "I'm building a [Book Tracker/Recipe Collector/Travel Journal - choose one]. Help me design it to collect 5 fields with appropriate type hints. For a Book Tracker: title, author, genre, pages, rating. Should I use `str` for everything, or would `int` be better for some fields like rating or pages? Show me the complete Python code with `input()` for all fields, formatted output with f-strings, and section comments."

**🧪 Test Edge Cases:**
> "My information collector has three problems: users can enter empty strings for required fields, numeric values are stored as strings (can't do math), and output looks messy with very long field values. For each problem, show me example input that breaks the current code, explain why it fails, and propose a fix with validation. Then challenge: what if a user enters '999999' for pages—should we reject it?"

**🎯 Practice Feature Extension:**
> "Enhance the collector with: asking 'Add another entry? (yes/no)' after each entry, storing multiple entries in a list, displaying all entries in a formatted table with aligned columns, and calculating statistics (total pages read, average rating, etc.). Show me the updated code with type hints for the list structure."

**🚀 Reflect on Spec-Driven Process:**
> "Explain how this project demonstrates specification-first thinking. How did designing before coding help? What was the division of labor between me (design intent) and you (implementation details)? If building a REAL app with database and web interface, which parts of this process would stay the same? Give me a Development Readiness Checklist: what Chapter 13 skills am I confident with vs what needs review?"

---

## Capstone Checklist

Before you submit your capstone, verify:

- [ ] Program runs without errors
- [ ] Collects 5 pieces of information (name, age, favorite color, hobby, city)
- [ ] Has type hints on all variables (`name: str`, `age: str`, etc.)
- [ ] Uses `input()` to collect data
- [ ] Uses f-strings for formatted output
- [ ] Includes comments explaining each section
- [ ] Displays formatted summary with user's information
- [ ] You can explain how it works to someone else

If you checked everything, congratulations! You've completed the Chapter 13 capstone.
