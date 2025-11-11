---
title: "Capstone: Time Zone Converter Project"
chapter: 23
lesson: 6
sidebar_position: 6
duration_minutes: 180
estimated_completion_time: "3-4 hours with breaks"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "AI-Assisted Project Planning"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can break down application requirements with AI guidance, decompose into functions, and plan implementation"

  - name: "Integrative Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student synthesizes math validation, datetime parsing, formatting, and timezone conversion into working application"

  - name: "AI-Assisted Debugging and Validation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student tests application with multiple scenarios, identifies issues with AI help, validates results against requirements"

  - name: "Python 3.14 Datetime Patterns"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student applies date.strptime(), datetime.now(timezone.utc), strftime(), and timezone conversions in integrated application"

  - name: "Error Handling and User Experience"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student handles invalid input gracefully, provides helpful error messages, enables user recovery"

learning_objectives:
  - objective: "Build a complete Time Zone Converter application that integrates all chapter concepts"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Working application with all 8 capstone requirements"

  - objective: "Decompose complex project into modular, testable functions"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Function design with clear separation of concerns"

  - objective: "Demonstrate AI partnership throughout development workflow"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Reflection on how AI helped with planning, debugging, and validation"

  - objective: "Validate application behavior across multiple timezone scenarios"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Test cases covering common timezones, edge cases, error conditions"

cognitive_load:
  new_concepts: 0
  assessment: "Zero new concepts (integration only). Synthesizing 5 lessons of prior knowledge. Appropriate for B1 capstone."

differentiation:
  extension_for_advanced: "Add batch timezone conversion, DST transition handling, calendar integration, persistent timezone configuration"
  remedial_for_struggling: "Work through core application step-by-step with AI, focusing on parse_datetime() first, then other functions progressively"

# Generation metadata
generated_by: "lesson-writer v3.0.2"
source_spec: "specs/001-part-4-chapter-23/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Capstone: Time Zone Converter Project

You've spent this entire chapter learning mathematical operations, time concepts, datetime parsing with Python 3.14's new methods, formatting, and timezone conversions. Now it's time to bring everything together into a real, working application.

This capstone project is intentionally comprehensive. You're not building a toy example anymore—you're building something practical that demonstrates your mastery of all chapter concepts. And you're not building it alone. Your AI companion will be with you throughout: helping you plan, suggesting implementations, debugging errors, and validating your work.

Welcome to AI-native development in practice.

## What You're Building

Your **Time Zone Converter** is a Python application that takes user input (a date, time, and source timezone) and converts it to a target timezone, displaying the result in multiple formats.

**Why this matters**: Every global application needs timezone handling. Companies like Stripe, Slack, and Notion all solved this problem. You're building the same skill they rely on, and you're doing it the AI-native way: thinking clearly about requirements, partnering with AI on implementation, and validating thoroughly.

### The 8 Capstone Requirements

Your application must:

1. **Accept user input**: Date/time string, source timezone, target timezone
2. **Parse input correctly**: Using Python 3.14's `date.strptime()` or `datetime.strptime()` with validation
3. **Convert to target timezone**: Handle UTC, EST, PST, GMT, and at least 3 other timezones
4. **Display results in multiple formats**: ISO 8601, friendly format, timestamp (demonstrating Lessons 1-4)
5. **Handle errors gracefully**: Invalid dates, unknown timezones, parsing failures (demonstrating Lesson 1-2)
6. **Provide clear feedback**: User-friendly error messages and recovery options
7. **Use type hints throughout**: All functions have parameter and return type annotations
8. **Demonstrate AI partnership**: Show (in comments or output) how AI helped at each stage

Let's walk through the planning, implementation, and testing phases together.

---

## Phase 1: Planning with AI

Before you write a single line of code, you're going to architect your application with your AI companion. This is professional practice: **specification before implementation**.

#### 💬 AI Colearning Prompt

> "I'm building a Time Zone Converter application. I need to parse user input (date, time, timezone), convert it to another timezone, and display results in multiple formats. What functions should I create to keep this modular and testable? What should each function do?"

**Why you're asking this**: You're getting AI to help you think through architecture *before* coding. This saves hours of refactoring later.

**What to expect from AI**: A breakdown something like:
- `parse_input()` or `parse_datetime()`: Extract and validate user input
- `convert_timezone()`: Handle the actual conversion
- `format_output()`: Display results in different formats
- `validate_timezone()`: Check if timezone is valid
- `main()`: User interface and error handling

Take that suggestion and adapt it. Maybe you want to add more functions. Maybe you want to combine some. The point is: **you decide the design with AI's help, not AI deciding for you**.

#### 🎓 Instructor Commentary

> In AI-native development, architecture comes from thinking clearly about requirements, not from memorizing design patterns. You described what you need, AI suggested a structure, and now you're taking ownership of that design. This is what professionals do every day.

---

## Phase 2: Core Implementation

Now you're going to build each function. We'll show you the complete application, then walk through how to build it with AI.

### The Complete Application

Here's what a working Time Zone Converter looks like:

```python
from datetime import datetime, date, time, timezone, timedelta
import sys

def parse_datetime(date_str: str, time_str: str) -> datetime:
    """Parse date and time strings into a timezone-naive datetime object.

    Args:
        date_str: Date string in format "YYYY-MM-DD"
        time_str: Time string in format "HH:MM:SS" (optional seconds)

    Returns:
        A timezone-naive datetime object

    Raises:
        ValueError: If date or time strings are invalid
    """
    try:
        # Python 3.14: Use date.strptime() for parsing date
        parsed_date: date = date.strptime(date_str, "%Y-%m-%d")

        # Python 3.14: Use time.strptime() for parsing time
        parsed_time: time = time.strptime(time_str, "%H:%M:%S")

        # Combine into datetime
        result: datetime = datetime.combine(parsed_date, parsed_time)
        return result
    except ValueError as e:
        raise ValueError(f"Invalid date or time format: {e}") from e


def get_timezone_offset(tz_name: str) -> timezone:
    """Get timezone object from timezone name.

    Args:
        tz_name: Timezone name (e.g., 'UTC', 'US/Eastern', 'Europe/London', 'Asia/Tokyo')

    Returns:
        A timezone object

    Raises:
        ValueError: If timezone name is not recognized
    """
    # Common timezone offsets (UTC hour offset)
    timezone_map: dict[str, int] = {
        "UTC": 0,
        "GMT": 0,
        "US/Eastern": -5,
        "US/Central": -6,
        "US/Mountain": -7,
        "US/Pacific": -8,
        "Europe/London": 0,
        "Europe/Paris": 1,
        "Asia/Tokyo": 9,
        "Asia/Shanghai": 8,
        "Australia/Sydney": 10,
        "India/Kolkata": 5.5,
    }

    if tz_name not in timezone_map:
        available: str = ", ".join(sorted(timezone_map.keys()))
        raise ValueError(
            f"Timezone '{tz_name}' not recognized. Available: {available}"
        )

    offset_hours: float = timezone_map[tz_name]
    # Handle half-hour offsets like India (UTC+5:30)
    hours: int = int(offset_hours)
    minutes: int = int((offset_hours % 1) * 60)
    tz_offset: timezone = timezone(timedelta(hours=hours, minutes=minutes))
    return tz_offset


def convert_timezone(
    dt: datetime,
    source_tz: timezone,
    target_tz: timezone
) -> datetime:
    """Convert datetime from source timezone to target timezone.

    Args:
        dt: Timezone-naive datetime object
        source_tz: Source timezone
        target_tz: Target timezone

    Returns:
        Converted datetime in target timezone
    """
    # Make the datetime timezone-aware with source timezone
    aware_dt: datetime = dt.replace(tzinfo=source_tz)

    # Convert to target timezone
    converted_dt: datetime = aware_dt.astimezone(target_tz)
    return converted_dt


def format_output(dt: datetime, tz_name: str) -> str:
    """Format datetime in multiple formats for display.

    Args:
        dt: Datetime object to format
        tz_name: Timezone name for context

    Returns:
        Formatted string with multiple representations
    """
    # ISO 8601 format
    iso_format: str = dt.strftime("%Y-%m-%dT%H:%M:%S")

    # Friendly format
    friendly_format: str = dt.strftime("%A, %B %d, %Y at %I:%M %p")

    # Timestamp (seconds since epoch)
    timestamp: float = dt.timestamp()

    # Format UTC offset
    offset: timedelta = dt.utcoffset() or timedelta(0)
    offset_hours: float = offset.total_seconds() / 3600
    offset_str: str = f"UTC{offset_hours:+.1f}"

    return f"""
Converted Time in {tz_name}:
  Friendly:    {friendly_format}
  ISO 8601:    {iso_format}
  Timestamp:   {timestamp:.0f} seconds since epoch
  Timezone:    {offset_str}
"""


def main() -> None:
    """Main application loop with user interface and error handling."""
    print("Welcome to the Time Zone Converter!")
    print("-" * 50)

    try:
        # Get user input
        date_input: str = input("Enter date (YYYY-MM-DD): ").strip()
        time_input: str = input("Enter time (HH:MM:SS): ").strip()
        source_tz_name: str = input("Enter source timezone (e.g., UTC, US/Eastern): ").strip()
        target_tz_name: str = input("Enter target timezone (e.g., US/Pacific, Asia/Tokyo): ").strip()

        # Parse input
        print("\nParsing input...")
        dt: datetime = parse_datetime(date_input, time_input)

        # Get timezone objects
        print("Validating timezones...")
        source_tz: timezone = get_timezone_offset(source_tz_name)
        target_tz: timezone = get_timezone_offset(target_tz_name)

        # Convert
        print("Converting...")
        converted: datetime = convert_timezone(dt, source_tz, target_tz)

        # Display result
        output: str = format_output(converted, target_tz_name)
        print(output)

        # Debug info (timestamp from Lesson 2)
        print("\nDebug Info (from Lesson 2):")
        print(f"  Source datetime: {dt} in {source_tz_name}")
        print(f"  Target datetime: {converted} in {target_tz_name}")

    except ValueError as e:
        print(f"\nError: {e}")
        print("Please check your input and try again.")
        sys.exit(1)


if __name__ == "__main__":
    main()
```

**What just happened**: You saw a complete, working Time Zone Converter. Every function has:
- Type hints (from Lesson 1: validation)
- Docstrings explaining purpose
- Error handling (try/except from Chapter 21)
- Clear responsibility (one thing per function)

Now let's understand how to build this with your AI companion.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Help me build the `parse_datetime()` function. I need to use Python 3.14's `date.strptime()` and `time.strptime()` methods to parse user input, combine them into a datetime, and handle ValueError if parsing fails. Show me the implementation with type hints and docstring, then explain why we separate date and time parsing."

**Expected Outcome**: You'll understand how Python 3.14's new methods work together, why we split date/time parsing, and how to structure functions for clarity.

---

## Phase 3: Building Each Component

Rather than repeat the entire code again, here's how to build it with AI assistance. We'll walk through each function type.

### Function 1: Input Parsing (from Lessons 3-4)

**Your job**: Describe what you need.
**AI's job**: Suggest implementation.
**Your job**: Validate and understand.

#### 💬 AI Colearning Prompt

> "In Python 3.14, I can use `date.strptime()` to parse a date string and `time.strptime()` to parse a time string. How do I parse '2025-11-09' and '14:30:45', combine them into a datetime, and handle errors if the user enters bad data?"

**What AI will explain**:
- `date.strptime()` returns a `date` object
- `time.strptime()` returns a `time` object
- `datetime.combine()` merges them
- ValueError is raised for invalid formats

**What you learn**: The mechanism, not the syntax (syntax is cheap).

### Function 2: Timezone Handling (from Lesson 4)

**Challenge**: Timezones are complex. DST transitions, half-hour offsets (India), different naming conventions.

**Solution**: Let your AI handle the reference, you handle the design.

#### ✨ Teaching Tip

> Use Claude Code to explore timezone edge cases: "Show me how to handle timezone offsets with timedelta. What happens with India (UTC+5:30)? How do I check if a timezone name is valid?"

**Why this matters**: Timezone handling has real-world complexity. Professionals don't memorize all timezone rules—they partner with tools to handle them correctly. That's what you're learning here.

### Function 3: Format Output (from Lesson 4)

You already know `strftime()` from Lesson 4. This function just applies it:

```python
def format_output(dt: datetime, tz_name: str) -> str:
    # Use strftime() formats you learned in Lesson 4
    iso_format: str = dt.strftime("%Y-%m-%dT%H:%M:%S")
    friendly_format: str = dt.strftime("%A, %B %d, %Y at %I:%M %p")

    # Timestamp for debugging (Lesson 2)
    timestamp: float = dt.timestamp()

    # Return formatted string
    return f"... {friendly_format} ... {timestamp} ..."
```

**What you're doing**: Combining learned concepts into one function.

#### 🎓 Instructor Commentary

> This is synthesis—you're not learning new datetime methods here. You're taking `strftime()` from Lesson 4, combining it with timestamp from Lesson 2, and building one function that displays multiple formats. This is how professionals build real applications: by composing smaller pieces you already understand into larger wholes.

---

## Phase 4: Testing and Validation

Once you've built your application, you need to test it thoroughly. This is where you think like a quality engineer.

### Test Scenarios

**Basic Conversion**:
- Input: 2025-11-09, 14:30:00 in UTC
- Output: 09:30:00 in US/Eastern (5 hours behind UTC)
- Verify: Timestamp is correct, all three output formats work

**Edge Case: Half-Hour Timezone**:
- Input: 2025-11-09, 12:00:00 in UTC
- Output: 17:30:00 in India/Kolkata (UTC+5:30)
- Verify: Handles non-hour offsets correctly

**Error Handling**:
- Invalid date: "2025-13-45"
- Invalid timezone: "Planet/Mars"
- Invalid time: "25:99:99"
- Verify: Application displays helpful error, doesn't crash

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Help me test my Time Zone Converter. What test cases should I run? Show me how to test that my application handles invalid dates, unknown timezones, and displays correct results. Then I'll run these tests and show you the output."

**Expected Outcome**:
- You'll run your application with multiple inputs
- You'll verify outputs are correct
- You'll debug any failures with AI's help
- You'll have confidence it works

---

## Phase 5: Understanding Your Code

Your capstone isn't complete until you can explain it. This is the most important phase.

### Integrated Concepts Checklist

Verify your application demonstrates all chapter concepts:

- **From Lesson 1 (Math)**: Input validation, error handling patterns, type hints ✓
- **From Lesson 2 (Time/Epoch)**: Timestamp display for debugging, understanding time measurement ✓
- **From Lesson 3 (Parsing)**: Using `date.strptime()` and `time.strptime()` (Python 3.14 new methods) ✓
- **From Lesson 4 (Formatting/Conversion)**: `strftime()` for multiple output formats, timezone conversion ✓
- **From Lesson 5 (Advanced)**: Optional—could add calendar display of converted date, or math validation ✓

#### 💬 AI Colearning Prompt

> "I've built my Time Zone Converter. Here's how it works [paste your code]. Explain how this application demonstrates concepts from all 5 lessons we learned. Which functions use which concepts?"

**What this does**: Solidifies your understanding through explanation. Talking through your code with AI forces you to articulate your thinking.

---

## Phase 6: Extensions (Optional, for Advanced Learners)

Once your core application works, consider these extensions:

### Extension 1: Batch Conversion

Convert multiple timezones in one run:

```
Enter times to convert (comma-separated):
  2025-11-09 14:30 in UTC → US/Eastern, Europe/London, Asia/Tokyo
  2025-12-25 18:00 in US/Pacific → UTC, India/Kolkata, Australia/Sydney
```

#### ✨ Teaching Tip

> Ask your AI: "Help me refactor my main() function to support batch conversions. How would I structure this? Should I create a new function or modify existing ones?"

### Extension 2: DST Awareness

Add warnings for Daylight Saving Time transitions:

```python
# Before running conversion, check if it's a DST boundary
# If converting to US/Eastern on specific dates, warn user
```

#### 🚀 CoLearning Challenge

Ask your AI:
> "During DST transitions, some times don't exist (2:30 AM during 'spring forward') and some exist twice (during 'fall back'). How do I detect and warn about these edge cases in my timezone converter?"

### Extension 3: Calendar Integration (from Lesson 5)

Display the calendar of the target month with the converted date highlighted:

```python
import calendar

# Get month/year of converted datetime
converted_month = converted.month
converted_year = converted.year

# Display calendar (Lesson 5)
print(calendar.month(converted_year, converted_month))
```

The converted date will appear highlighted in your terminal (Python 3.14 feature).

---

## AI Partnership Throughout

Let's be explicit about what just happened:

### What You Did (Specification, Design, Validation)
1. Described requirements ("Parse date, timezone, convert")
2. Decided architecture ("Create these functions...")
3. Tested thoroughly ("Does it handle India/Kolkata correctly?")
4. Debugged failures ("Why is this conversion off by an hour?")

### What AI Did (Implementation, Explanation, Debugging Partner)
1. Suggested function structure
2. Explained Python 3.14's new methods
3. Generated implementations you reviewed
4. Explained error messages when things failed
5. Suggested improvements (half-hour timezones, edge cases)

### What This Teaches

This is **professional AI-native development**:
- You think strategically (requirements, design, testing)
- AI executes tactically (syntax, implementation details)
- You own the quality (validation, decisions, understanding)

This is how teams at Google, Anthropic, and Stripe actually work. You just practiced it.

---

## Try With AI

You're almost done. But the capstone isn't complete until you reflect on what you built and what you learned. Your AI can help with this.

### Setup

Open **Claude Code**, **Gemini CLI**, or **ChatGPT web**. Have your completed Time Zone Converter code ready.

### Prompt 1: Recall

> "I just built a Time Zone Converter capstone for a Python chapter on math, datetime, and calendars. List all the concepts from the chapter that my application uses. Be specific about which functions use which concepts."

**Expected outcome**: Complete inventory of integrated concepts (math validation, time module for timestamps, datetime module parsing and conversion, optional calendar display). You'll see how comprehensively your application synthesized the chapter.

### Prompt 2: Understand

> "Look at my application architecture. Why did I structure it with separate functions for parsing, timezone conversion, and formatting? What would break if I combined them all into one big main() function?"

**Expected outcome**: Explanation of separation of concerns, modularity, testability. You'll deepen your understanding of why professionals structure code this way.

### Prompt 3: Apply

> "Add a feature that converts multiple timezones at once. For example, given a datetime in UTC, show the time in NYC, London, and Tokyo simultaneously. How would I refactor my code to support this without duplicating logic?"

**Expected outcome**: Working extension demonstrating your ability to evolve the application. You'll experience how good design makes extensions easy.

### Prompt 4: Synthesize

> "Reflect on building this capstone with AI help. How did AI change the way you approached this project compared to coding alone? What did AI do well? What did you do better than AI? How is this different from traditional 'copy-paste code from Stack Overflow' learning?"

**Expected outcome**: Metacognitive reflection on AI partnership. This is the most important outcome. You're not just learning Python—you're learning a fundamentally new way of working with intelligent tools.

**What you're validating**: Understanding of AI-native development, not just technical syntax.

---

**Your capstone is complete.** You've built a practical application that integrates all chapter concepts, partnered with AI throughout the process, and reflected on how AI-native development actually works.

You're ready for the next chapter.
