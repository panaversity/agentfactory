# Chapter 23: Math, Date Time Calendar — Lesson Plan

**Chapter Type**: Technical
**Chapter Objective**: Students will perform mathematical operations with validation, work with dates and times using Python 3.14's new parsing methods, format and manipulate datetime objects, handle timezones correctly, and build a complete Time Zone Converter application demonstrating AI-native development.
**Estimated Total Time**: 8-10 hours (5 foundational lessons + 1 capstone)
**Part**: 4 (Python Fundamentals)
**Complexity Tier**: Intermediate (A2-B1)
**CEFR Progression**: A2 → A2-B1 → B1

---

## Chapter Overview

This chapter teaches students to work with mathematical operations and date/time handling using Python 3.14's modern approaches. Unlike traditional Python courses that teach deprecated methods, this chapter presents Python 3.14's new `date.strptime()` and `time.strptime()` methods as the PRIMARY parsing approach and emphasizes `datetime.now(timezone.utc)` over deprecated alternatives.

**Python 3.14 Native Approach**: Students learn the modern standard from day one—no backward compatibility content, no mention of deprecated methods like `utcnow()`. This prepares them for current professional development.

**AI-Native Learning Integration**: Throughout all lessons, students use AI as co-reasoning partner to explore mathematical concepts, understand datetime complexities, debug timezone issues, and validate implementations. The capstone demonstrates full AI partnership from planning through deployment.

**Prerequisites**:
- Chapter 14 (Data Types): Type hints, int, float, str
- Chapter 16 (Strings): String formatting, f-strings
- Chapter 21 (Exception Handling): try/except for error handling

**Success Metrics** (from specification):
- 85%+ capstone completion rate
- 80%+ ask AI for concepts (not just code)
- 90%+ explain WHEN to use methods (not just HOW)
- 100% CoLearning compliance (💬🎓🚀✨ throughout)

---

## Lesson Architecture

### Lesson 1: Math Module Fundamentals (CEFR: A2, Duration: 90-120 min)

**Learning Objective**: Students will perform basic mathematical operations using Python's math module, distinguish between built-in functions and math module functions, understand when to use different rounding approaches, and validate mathematical inputs with Python 3.14's enhanced error messages.

**CEFR Proficiency Level**: A2 (Basic application in familiar contexts)

**Skills Taught** (Skills Proficiency Mapping):

1. **Mathematical Operations with Type Hints** — A2 — Technical
   - **Measurable at A2**: Student can write functions that perform math operations (sqrt, round, ceil, floor) with correct type hints (float → float, int → int) and validate inputs
   - **Bloom's Level**: Apply (using math functions in code)
   - **DigComp Area**: Content Creation (writing mathematical functions)
   - **Assessment**: Function creation with type-annotated parameters and returns

2. **Domain Error Understanding** — A2 — Conceptual
   - **Measurable at A2**: Student can interpret Python 3.14's enhanced "math domain error" messages and explain why operations fail (negative sqrt, invalid log input)
   - **Bloom's Level**: Understand (interpreting error messages)
   - **DigComp Area**: Problem-Solving (debugging mathematical constraints)
   - **Assessment**: Error interpretation and explanation

3. **Mathematical Constant Application** — A2 — Technical
   - **Measurable at A2**: Student can use math.pi, math.e, math.tau in calculations and explain when to use each (circles use pi or tau, exponential growth uses e)
   - **Bloom's Level**: Apply (using constants appropriately)
   - **DigComp Area**: Information Literacy (selecting appropriate constants)
   - **Assessment**: Practical calculations with correct constant selection

**Concepts** (6 total, within A2 limit of 7):
1. Built-in math functions vs math module (abs, pow, round, max, min are built-ins; sqrt requires import)
2. `math.sqrt()` for square roots with validation (positive numbers only)
3. Rounding functions: `round()` vs `math.ceil()` vs `math.floor()` (different behaviors)
4. Mathematical constants: `math.pi`, `math.e`, `math.tau` (what they represent, when to use)
5. Type hints for math operations (`float`, `int` parameters and returns)
6. Enhanced domain error messages in Python 3.14 (understanding "math domain error")

**Cognitive Load Validation**:
- New concepts: 6 (within A2 limit of 7) ✓
- Assessment: Appropriate for intermediate learners with Chapter 14 type hints foundation

**Content Outline**:
1. Introduction: Why math module matters (scientific computing, precision calculations)
2. Built-in vs Module Functions (when to import, when to use built-ins)
3. Square Roots and Validation (positive numbers, error handling with try/except)
4. Rounding Strategies (round=nearest, ceil=up, floor=down with examples showing differences)
5. Mathematical Constants (pi for circles, e for exponential, tau for full rotations)
6. Type Hints for Math Functions (annotating parameters and returns)
7. Python 3.14 Enhanced Errors (interpreting domain error messages with AI)

**Code Examples** (4 examples):
1. **Arithmetic with Type Hints**: Basic operations with proper annotations
   - Purpose: Establish type hint pattern for math operations
   - Complexity: A2 (simple function with type hints)
   - Demonstrates: Function signatures, type annotations, return types

2. **Square Root with Error Handling**: Validate positive numbers, catch ValueError
   - Purpose: Teach input validation and Python 3.14's enhanced error messages
   - Complexity: A2 (validation + error handling)
   - Demonstrates: try/except, domain errors, user feedback

3. **Rounding Comparisons**: Show round(), ceil(), floor() differences
   - Purpose: Clarify when to use each rounding approach
   - Complexity: A2 (comparison demonstration)
   - Demonstrates: Edge cases (2.5, -2.5), behavioral differences

4. **Using Pi for Calculations**: Circle area and circumference
   - Purpose: Practical constant application
   - Complexity: A2 (formula implementation)
   - Demonstrates: math.pi vs hardcoded 3.14, precision matters

**AI CoLearning Elements** (throughout lesson, NOT just at end):

- 💬 **AI Colearning Prompt** (after introducing sqrt):
  - "Explain why `math.sqrt(-1)` raises an error and what Python 3.14's enhanced message tells us"
  - Placement: After showing sqrt example

- 🎓 **Instructor Commentary** (after rounding examples):
  - "You don't memorize which rounding function to use—you understand the difference (round=nearest, ceil=up, floor=down) and ask AI when unsure. Syntax is cheap, knowing WHEN to round up vs down in business logic is gold."
  - Placement: After code examples section

- 🚀 **CoLearning Challenge** (mid-lesson):
  - "Ask AI to generate a function that calculates circle area with type hints, then explain why pi is more accurate than 3.14"
  - Expected Outcome: Understanding precision and type annotations
  - Placement: After constants introduction

- ✨ **Teaching Tip** (near end):
  - "Use Claude Code to explore edge cases: 'What happens if I round 2.5? What about -2.5? Why are the results different?'"
  - Placement: Before exercises

**Try With AI** (4 prompts, progressive Bloom's levels - ONLY closure section):

1. **Recall** (Remember): "Explain the difference between round(), ceil(), and floor() with examples"
   - Expected: Definition of each function and behavioral differences
   - Validates: Basic comprehension

2. **Understand**: "Why does Python separate built-in functions from the math module?"
   - Expected: Explanation of namespace management and import philosophy
   - Validates: Conceptual understanding

3. **Apply**: "Generate a function that validates input before calculating square root"
   - Expected: Function with try/except, type hints, user-friendly error messages
   - Validates: Application of validation pattern

4. **Analyze**: "When would you use math.tau instead of math.pi in real applications?"
   - Expected: Understanding of full rotation (2π) vs half rotation (π) contexts
   - Validates: Higher-order thinking about when to choose approaches

**Tool Selection**: ChatGPT web OR learner's AI companion (Gemini CLI, Claude CLI) if already configured

**No sections after "Try With AI"** — lesson ends here (no "Key Takeaways", no "What's Next")

---

### Lesson 2: Time and Epoch Concepts (CEFR: A2, Duration: 90-120 min)

**Learning Objective**: Students will understand how computers measure time using epoch, work with Unix timestamps, convert between timestamp and human-readable formats, and use time tuples to access individual time components.

**CEFR Proficiency Level**: A2 (Basic application in familiar contexts)

**Skills Taught** (Skills Proficiency Mapping):

1. **Time Concept Understanding** — A2 — Conceptual
   - **Measurable at A2**: Student can explain epoch (January 1, 1970 UTC) as reference point and why computers use timestamps for time measurement
   - **Bloom's Level**: Understand (explaining epoch concept)
   - **DigComp Area**: Information Literacy (understanding how systems represent time)
   - **Assessment**: Explanation of epoch and timestamp purpose

2. **Timestamp Operations** — A2 — Technical
   - **Measurable at A2**: Student can get current timestamp with time.time(), convert to time tuple with time.localtime(), and format with time.asctime()
   - **Bloom's Level**: Apply (using time module functions)
   - **DigComp Area**: Content Creation (working with timestamps)
   - **Assessment**: Code that captures, converts, and displays timestamps

3. **Time Tuple Navigation** — A2 — Technical
   - **Measurable at A2**: Student can access time tuple fields (year, month, day, hour, minute, second) and extract specific components
   - **Bloom's Level**: Apply (accessing tuple fields)
   - **DigComp Area**: Information Literacy (understanding time representation)
   - **Assessment**: Extracting specific time components from tuple

**Concepts** (5 total, within A2 limit of 7):
1. Epoch concept (January 1, 1970 UTC as reference point)
2. `time.time()` for current timestamp (floating-point seconds since epoch)
3. `time.localtime()` for converting timestamp to time tuple
4. `time.asctime()` for formatting time tuple to human-readable string
5. Time tuples (year, month, day, hour, minute, second, weekday, yearday, isdst)

**Cognitive Load Validation**:
- New concepts: 5 (within A2 limit of 7) ✓
- Assessment: Builds on Lesson 1 validation patterns, introduces time concepts

**Content Outline**:
1. Introduction: How computers measure time (timestamps vs human dates)
2. Epoch Explained (why January 1, 1970 UTC? Historical context)
3. Getting Current Timestamp (time.time() returns float)
4. Converting Timestamps to Time Tuples (time.localtime() structure)
5. Formatting with asctime() (human-readable output)
6. Time Tuple Components (accessing year, month, weekday, etc.)
7. Calculating Elapsed Time (subtracting timestamps)

**Code Examples** (4 examples):
1. **Getting and Displaying Timestamp**: time.time() with type hints
   - Purpose: Introduce timestamp concept
   - Complexity: A2 (simple function call)
   - Demonstrates: Floating-point seconds since epoch

2. **Converting Timestamp to Time Tuple**: time.localtime() usage
   - Purpose: Show conversion from timestamp to structured representation
   - Complexity: A2 (conversion function)
   - Demonstrates: Time tuple structure and fields

3. **Formatting Time with asctime()**: Human-readable output
   - Purpose: Display time in familiar format
   - Complexity: A2 (formatting function)
   - Demonstrates: Conversion to readable string

4. **Calculating Elapsed Time**: Two timestamps, difference in seconds/minutes/hours
   - Purpose: Practical time calculation
   - Complexity: A2 (arithmetic with timestamps)
   - Demonstrates: Duration calculation and unit conversion

**AI CoLearning Elements** (throughout lesson):

- 💬 **AI Colearning Prompt** (after epoch introduction):
  - "Why is January 1, 1970 chosen as epoch? What happens with dates before 1970?"
  - Placement: After epoch explanation

- 🎓 **Instructor Commentary** (after timestamp examples):
  - "Timestamps might look confusing (1699564800.123), but they're just seconds since a fixed point—perfect for calculations. Let AI handle the conversions; you focus on understanding WHY timestamps matter for storage and comparison."
  - Placement: After code examples

- 🚀 **CoLearning Challenge** (mid-lesson):
  - "Ask AI to explain time tuple fields and generate code that extracts just the weekday"
  - Expected Outcome: Understanding time tuple structure and field access
  - Placement: After time tuple introduction

- ✨ **Teaching Tip** (near end):
  - "Explore with your AI: 'Show me the timestamp for my birthday and explain how Python calculates it'"
  - Placement: Before exercises

**Try With AI** (4 prompts, progressive):

1. **Recall**: "What is the Unix epoch and why do computers use it?"
2. **Understand**: "Explain how time.time() relates to time.localtime() and time.asctime()"
3. **Apply**: "Create a function that calculates how many days have passed since a given timestamp"
4. **Analyze**: "Why are timestamps better than formatted date strings for storage and comparison?"

**Tool Selection**: ChatGPT web OR learner's AI companion

**Lesson ends with "Try With AI"** — no additional sections

---

### Lesson 3: Date and Time Objects (Python 3.14) (CEFR: A2-B1, Duration: 120-150 min)

**Learning Objective**: Students will create and parse date/time objects using Python 3.14's new `date.strptime()` and `time.strptime()` methods, understand timezone awareness basics, and work with `datetime.now()` for current timestamps.

**CEFR Proficiency Level**: A2-B1 (Bridging to intermediate proficiency)

**Skills Taught** (Skills Proficiency Mapping):

1. **Date/Time Object Creation** — A2 — Technical
   - **Measurable at A2**: Student can create date, time, and datetime objects with appropriate parameters and type hints
   - **Bloom's Level**: Apply (creating objects programmatically)
   - **DigComp Area**: Content Creation (working with datetime objects)
   - **Assessment**: Code creating various datetime objects

2. **Python 3.14 String Parsing** — B1 — Technical
   - **Measurable at B1**: Student can parse user-provided date/time strings using Python 3.14's new `date.strptime()` and `time.strptime()` methods and handle parsing errors
   - **Bloom's Level**: Apply (using new parsing methods independently)
   - **DigComp Area**: Problem-Solving (handling user input)
   - **Assessment**: Real-world parsing with error handling

3. **Timezone Awareness Basics** — A2 — Conceptual
   - **Measurable at A2**: Student can explain difference between naive and timezone-aware datetime objects and create timezone-aware objects with `datetime.now(timezone.utc)`
   - **Bloom's Level**: Understand (explaining timezone concepts)
   - **DigComp Area**: Information Literacy (understanding timezone implications)
   - **Assessment**: Explanation and basic timezone-aware object creation

**Concepts** (7 total, max for A2-B1):
1. `datetime.date` objects (year, month, day)
2. `datetime.time` objects (hour, minute, second, microsecond)
3. `datetime.datetime` objects (combines date and time)
4. **Python 3.14 NEW**: `date.strptime()` for parsing date strings directly
5. **Python 3.14 NEW**: `time.strptime()` for parsing time strings directly
6. `datetime.now()` with optional timezone parameter (modern approach)
7. Timezone awareness basics (naive vs aware datetime objects)

**Cognitive Load Validation**:
- New concepts: 7 (at A2-B1 max limit of 7) ✓
- Assessment: This is the most concept-dense lesson; manageable for intermediate learners with prior foundation

**Content Outline**:
1. Introduction: Why datetime module over time module? (Higher-level abstraction)
2. Date Objects (year, month, day representation)
3. Time Objects (hour, minute, second, microsecond)
4. Datetime Objects (combining date and time)
5. **Python 3.14 NEW**: date.strptime() (PRIMARY method for parsing date strings)
6. **Python 3.14 NEW**: time.strptime() (PRIMARY method for parsing time strings)
7. Getting Current Datetime (datetime.now() with timezone parameter)
8. Timezone Awareness (naive objects vs aware objects, why awareness matters)

**Code Examples** (5 examples):
1. **Creating Date/Time/Datetime Objects**: Basic object creation with type hints
   - Purpose: Introduce object creation patterns
   - Complexity: A2 (object instantiation)
   - Demonstrates: Constructor parameters, type annotations

2. **Parsing Dates with date.strptime()** (NEW in Python 3.14):
   - Purpose: PRIMARY parsing method for date strings
   - Complexity: B1 (parsing with format codes)
   - Demonstrates: Format string patterns, validation

3. **Parsing Times with time.strptime()** (NEW in Python 3.14):
   - Purpose: PRIMARY parsing method for time strings
   - Complexity: B1 (parsing with format codes)
   - Demonstrates: Time format patterns, validation

4. **Getting Current Datetime**: datetime.now() with and without timezone
   - Purpose: Capture current timestamp in different ways
   - Complexity: A2 (function call with optional parameter)
   - Demonstrates: Default (naive) vs timezone-aware objects

5. **Creating Timezone-Aware Datetime**: datetime.now(timezone.utc)
   - Purpose: Modern approach for timezone-aware objects
   - Complexity: B1 (using timezone objects)
   - Demonstrates: UTC timezone, awareness importance

**AI CoLearning Elements** (throughout lesson):

- 💬 **AI Colearning Prompt** (after introducing Python 3.14 methods):
  - "Python 3.14 added date.strptime() and time.strptime(). How do these improve on previous approaches?"
  - Placement: After new methods introduction

- 🎓 **Instructor Commentary** (after parsing examples):
  - "Before Python 3.14, parsing dates required workarounds. Now you parse directly—syntax is cheap, knowing WHEN to parse (user input) vs construct (programmatic dates) is gold."
  - Placement: After code examples

- 🚀 **CoLearning Challenge** (mid-lesson):
  - "Ask AI to generate a function that parses user birthday input and calculates age"
  - Expected Outcome: Understanding parsing, date arithmetic, and current date
  - Placement: After parsing methods

- ✨ **Teaching Tip** (near end):
  - "Always use datetime.now(timezone.utc) instead of deprecated utcnow()—ask AI why this matters for production code"
  - Placement: After timezone awareness section

**Try With AI** (4 prompts, progressive):

1. **Recall**: "What's new in Python 3.14 for date/time parsing?"
2. **Understand**: "Explain the difference between naive and timezone-aware datetime objects"
3. **Apply**: "Create a function that parses '2025-11-09' and calculates days until that date"
4. **Analyze**: "Why is timezone awareness important for global applications? When can you skip it?"

**Tool Selection**: ChatGPT web OR learner's AI companion

**Lesson ends with "Try With AI"** — no additional sections

---

### Lesson 4: Date/Time Formatting and Manipulation (CEFR: B1, Duration: 120-150 min)

**Learning Objective**: Students will format datetime objects for display using `strftime()`, perform date arithmetic with timedelta, convert between timezones using Python 3.14's recommended approaches, and handle timezone-related edge cases with AI assistance.

**CEFR Proficiency Level**: B1 (Intermediate - applying to real problems)

**Skills Taught** (Skills Proficiency Mapping):

1. **DateTime Formatting** — B1 — Technical
   - **Measurable at B1**: Student can format datetime objects for different contexts (ISO 8601, localized, custom) using strftime() without memorizing all format codes
   - **Bloom's Level**: Apply (formatting for real use cases)
   - **DigComp Area**: Content Creation (producing user-facing datetime displays)
   - **Assessment**: Multiple output formats for same datetime

2. **Date Arithmetic** — B1 — Technical
   - **Measurable at B1**: Student can add/subtract timedelta to/from datetime, calculate durations between two dates, and convert durations to different units (days, hours, minutes)
   - **Bloom's Level**: Apply (performing calculations independently)
   - **DigComp Area**: Problem-Solving (solving temporal calculations)
   - **Assessment**: Real-world date calculation problems

3. **Timezone Conversion** — B1 — Technical
   - **Measurable at B1**: Student can convert datetime objects between timezones (UTC to local, local to local) using timezone and timedelta objects
   - **Bloom's Level**: Apply (performing conversions in real contexts)
   - **DigComp Area**: Problem-Solving (handling global time requirements)
   - **Assessment**: Multi-timezone conversion scenarios

**Concepts** (6 total, within B1 limit of 7):
1. `strftime()` format codes (%Y, %m, %d, %H, %M, %S, %A, %B, etc.)
2. Formatting for different contexts (ISO 8601, localized, custom displays)
3. `timedelta` for representing durations (days, hours, minutes, seconds)
4. Date arithmetic (adding/subtracting timedelta to/from datetime)
5. Timezone conversion basics (UTC to local, local to local)
6. Handling timezone conversions with `timezone` and `timedelta` objects

**Cognitive Load Validation**:
- New concepts: 6 (within B1 limit of 7) ✓
- Assessment: Appropriate for intermediate learners; builds on Lesson 3 foundations

**Content Outline**:
1. Introduction: Why formatting matters (user interfaces, logs, APIs)
2. strftime() Format Codes (common patterns: %Y-%m-%d, %H:%M:%S, %A, %B)
3. Formatting for Different Contexts (ISO 8601, US format, EU format, custom)
4. Timedelta Objects (representing durations, not specific times)
5. Date Arithmetic (adding days, subtracting timedelta, calculating differences)
6. Timezone Conversion (UTC as reference, converting to local timezones)
7. Handling Timezone Complexity with AI (DST transitions, ambiguous times)

**Code Examples** (5 examples):
1. **Formatting in Multiple Styles**: ISO, US, EU, and custom formats
   - Purpose: Show format code flexibility
   - Complexity: B1 (multiple format patterns)
   - Demonstrates: strftime() versatility, locale considerations

2. **Adding/Subtracting Days with Timedelta**: Future and past dates
   - Purpose: Practical date arithmetic
   - Complexity: B1 (arithmetic operations)
   - Demonstrates: timedelta creation and application

3. **Calculating Duration Between Dates**: Days, hours, minutes difference
   - Purpose: Duration calculations for real scenarios
   - Complexity: B1 (subtraction and unit conversion)
   - Demonstrates: Date difference, timedelta components

4. **Converting UTC to Another Timezone**: UTC to US/Eastern
   - Purpose: Practical timezone conversion
   - Complexity: B1 (timezone objects and conversion)
   - Demonstrates: timezone parameter, offset application

5. **Formatting with Localized Month/Weekday Names**: Human-friendly output
   - Purpose: User-facing datetime displays
   - Complexity: B1 (advanced formatting)
   - Demonstrates: %A (weekday), %B (month) format codes

**AI CoLearning Elements** (throughout lesson):

- 💬 **AI Colearning Prompt** (after format codes):
  - "Explain strftime format codes—why %Y vs %y, %H vs %I?"
  - Placement: After introducing format codes

- 🎓 **Instructor Commentary** (after examples):
  - "You don't memorize all format codes (there are 30+). You understand the pattern (%Y=year, %m=month) and ask AI when you need a specific format. Syntax is cheap, knowing WHEN to use ISO 8601 vs localized format is gold."
  - Placement: After code examples

- 🚀 **CoLearning Challenge** (mid-lesson):
  - "Ask AI to generate code that displays 'meeting in 3 days, 4 hours' using timedelta"
  - Expected Outcome: Understanding timedelta components and natural language formatting
  - Placement: After timedelta section

- ✨ **Teaching Tip** (near end):
  - "Explore timezone complexity with AI: 'What happens during DST transitions? How do I handle ambiguous times?'"
  - Placement: After timezone conversion section

**Try With AI** (4 prompts, progressive):

1. **Recall**: "List 5 common strftime format codes and their meanings"
2. **Understand**: "How does timedelta differ from datetime? When do you use each?"
3. **Apply**: "Create a function that formats a datetime in both ISO 8601 and 'friendly' format (e.g., 'November 9, 2025 at 2:30 PM')"
4. **Analyze**: "What are the challenges of timezone conversion? How can AI help you handle edge cases?"

**Tool Selection**: ChatGPT web OR learner's AI companion

**Lesson ends with "Try With AI"** — no additional sections

---

### Lesson 5: Calendar and Advanced Math (CEFR: B1, Duration: 120-150 min)

**Learning Objective**: Students will generate calendar displays using Python 3.14's enhanced calendar module, perform trigonometric calculations for angles, work with logarithms for scientific computing, understand special numeric values (infinity, NaN) as validation concepts, and apply advanced math functions to real-world scenarios.

**CEFR Proficiency Level**: B1 (Intermediate - applying to real problems)

**Skills Taught** (Skills Proficiency Mapping):

1. **Calendar Display and Scheduling** — B1 — Technical
   - **Measurable at B1**: Student can generate calendar grids with calendar.month(), understand week calculations, and observe Python 3.14's color highlighting feature
   - **Bloom's Level**: Apply (creating calendar interfaces)
   - **DigComp Area**: Content Creation (generating calendar displays)
   - **Assessment**: Calendar display with scheduling context

2. **Trigonometric Calculations** — B1 — Technical
   - **Measurable at B1**: Student can perform sin, cos, tan calculations for angles in radians and explain when trigonometry is needed (rotations, waves, geometry)
   - **Bloom's Level**: Apply (using trigonometric functions)
   - **DigComp Area**: Problem-Solving (applying math to real scenarios)
   - **Assessment**: Trigonometric calculations with real-world context

3. **Advanced Math Functions** — B1 — Technical
   - **Measurable at B1**: Student can use logarithms (log, log10) for appropriate contexts, calculate factorials, and handle special values (inf, nan) as validation concepts
   - **Bloom's Level**: Apply (using advanced functions appropriately)
   - **DigComp Area**: Problem-Solving (selecting appropriate mathematical tools)
   - **Assessment**: Real-world problems requiring logarithms, factorials, or special value handling

**Concepts** (7 total, max for B1):
1. `calendar.month()` for generating calendar grids with Python 3.14's color highlighting
2. Using calendar for scheduling interfaces and week calculations
3. Trigonometric functions (`math.sin`, `math.cos`, `math.tan`) for angles in radians
4. Logarithms (`math.log`, `math.log10`) for scientific computing
5. `math.factorial()` for combinatorial calculations
6. Special values `math.inf` (infinity) and `math.nan` (not a number) as validation concepts
7. Testing for special values with `math.isinf()` and `math.isnan()`

**Cognitive Load Validation**:
- New concepts: 7 (at B1 max limit of 7) ✓
- Assessment: Appropriate for intermediate learners; specialized topics with clear applications

**Content Outline**:
1. Introduction: Advanced math and calendar for specialized applications
2. Calendar Module Basics (month(), Python 3.14's color highlighting)
3. Calendar for Scheduling (week numbers, date grids)
4. Trigonometric Functions (sin, cos, tan with radians)
5. Practical Trigonometry (right triangles, rotations, waves)
6. Logarithms (natural log, base-10 log, applications)
7. Factorial for Combinatorics (permutations, combinations)
8. Special Numeric Values (inf, nan as validation concepts)
9. Testing for Special Values (isinf(), isnan() for edge case handling)

**Code Examples** (5 examples):
1. **Displaying Month Calendar**: calendar.month() with Python 3.14 color feature
   - Purpose: Show calendar generation
   - Complexity: B1 (module usage, terminal observation)
   - Demonstrates: Calendar display, color highlighting

2. **Trigonometric Calculations for Right Triangles**: Angle calculations
   - Purpose: Practical trigonometry application
   - Complexity: B1 (trig functions with real context)
   - Demonstrates: sin, cos, tan usage with radians

3. **Logarithm Applications**: Exponential growth, decibels, pH scale
   - Purpose: Show when to use logarithms
   - Complexity: B1 (logarithmic functions)
   - Demonstrates: log() for natural, log10() for base-10

4. **Factorial for Permutations/Combinations**: Combinatorial calculations
   - Purpose: Practical factorial usage
   - Complexity: B1 (factorial function)
   - Demonstrates: Combinatorics applications

5. **Handling Infinity and NaN in Validation**: Edge case handling
   - Purpose: Validation logic with special values
   - Complexity: B1 (testing for special values)
   - Demonstrates: isinf(), isnan() for robust validation

**AI CoLearning Elements** (throughout lesson):

- 💬 **AI Colearning Prompt** (after calendar):
  - "When running calendar.month() in your terminal, you'll see today's date in color (Python 3.14 feature). How does Python know 'today'?"
  - Placement: After calendar introduction

- 🎓 **Instructor Commentary** (after trigonometry):
  - "Trigonometry might seem abstract, but you don't memorize formulas—you understand WHEN to use sin/cos/tan (angles, waves, rotations) and let AI help with the math. Syntax is cheap, recognizing when a problem needs trigonometry is gold."
  - Placement: After trig examples

- 🚀 **CoLearning Challenge** (mid-lesson):
  - "Ask AI to generate a calculator that converts degrees to radians and computes all three trig functions"
  - Expected Outcome: Understanding degree-to-radian conversion and trig function usage
  - Placement: After trigonometry section

- ✨ **Teaching Tip** (near end):
  - "Explore edge cases: 'What is log(0)? What is factorial(-5)? How do I test if a result is infinite?'"
  - Placement: After special values section

**Try With AI** (4 prompts, progressive):

1. **Recall**: "What are three practical uses for logarithms in programming?"
2. **Understand**: "Explain what math.inf and math.nan represent. Why are they useful for validation?"
3. **Apply**: "Generate code that displays next month's calendar and highlights specific dates"
4. **Analyze**: "When would you use trigonometric functions in a real application? (Think beyond math class)"

**Tool Selection**: ChatGPT web OR learner's AI companion

**Lesson ends with "Try With AI"** — no additional sections

---

### Lesson 6 (Capstone): Time Zone Converter Project (CEFR: B1, Duration: 180-240 min)

**Learning Objective**: Students will build a complete Time Zone Converter application that integrates all chapter concepts (parsing dates/times with Python 3.14 methods, timezone conversion, formatting, validation, error handling) with AI assistance throughout development (planning, implementation, debugging, validation).

**CEFR Proficiency Level**: B1 (Intermediate - synthesizing knowledge in real project)

**Skills Taught** (Skills Proficiency Mapping):

1. **AI-Assisted Project Planning** — B1 — Soft
   - **Measurable at B1**: Student can describe project requirements to AI, decompose into functions, and plan implementation with AI guidance
   - **Bloom's Level**: Analyze (breaking down project into components)
   - **DigComp Area**: Problem-Solving (planning with AI collaboration)
   - **Assessment**: Project plan with clear function breakdown

2. **Integrative Implementation** — B1 — Technical
   - **Measurable at B1**: Student can combine math validation, datetime parsing, formatting, timezone conversion into working application with AI implementation support
   - **Bloom's Level**: Create (synthesizing concepts into new application)
   - **DigComp Area**: Content Creation (building complete application)
   - **Assessment**: Working Time Zone Converter application

3. **AI-Assisted Debugging and Validation** — B1 — Technical
   - **Measurable at B1**: Student can test application with multiple scenarios, identify issues with AI help, and validate results against requirements
   - **Bloom's Level**: Evaluate (assessing quality and correctness)
   - **DigComp Area**: Safety (ensuring application reliability)
   - **Assessment**: Test cases, error handling, validation checks

**Integrated Concepts** (from all 5 lessons):
- Math validation (ensure valid dates, handle errors)
- Time concepts (display timestamps for debugging)
- Date/time parsing (Python 3.14's `strptime()` methods)
- Formatting (display results in multiple formats)
- Timezone conversion (core functionality)
- Calendar display (optional: show current month with highlighted date)
- AI partnership (planning, implementation, debugging, validation)

**Cognitive Load Validation**:
- New concepts: 0 (integration and synthesis, not new concepts)
- Assessment: Appropriate for capstone; students apply existing knowledge

**Capstone Requirements** (from specification):
1. Accept user input: date/time string, source timezone, target timezone
2. Parse input using Python 3.14's new methods with validation
3. Convert datetime to target timezone (handle UTC, EST, PST, GMT, etc.)
4. Display results in multiple formats (ISO, friendly, timestamp)
5. Handle errors gracefully (invalid dates, unknown timezones, parsing failures)
6. Provide clear user feedback and recovery options
7. Use type hints throughout
8. Demonstrate AI-assisted development (show how AI helped at each stage)

**Content Outline**:
1. Introduction: Capstone Overview (integrating all chapter concepts)
2. Project Planning with AI (architecture, function breakdown)
3. Core Functionality:
   - User input collection and validation
   - Date/time parsing with Python 3.14 methods
   - Timezone conversion logic
   - Multi-format output
4. Error Handling (invalid dates, unknown timezones, parsing failures)
5. AI-Assisted Implementation (working with AI to build each component)
6. Testing and Validation (multiple timezone scenarios, edge cases)
7. Extensions (batch conversion, calendar view, DST warnings)
8. Reflection on AI Partnership (how AI helped throughout)

**Code Examples** (1 complete application):
1. **Full Time Zone Converter with Modular Functions**:
   - Purpose: Complete working application demonstrating all concepts
   - Complexity: B1 (integration of multiple concepts)
   - Components:
     - `parse_datetime()`: Parse user input with Python 3.14 methods
     - `convert_timezone()`: Convert between timezones
     - `format_output()`: Display in multiple formats
     - `main()`: User interface and error handling
   - Demonstrates: Full AI-native development workflow

**AI CoLearning Elements** (throughout capstone):

- 💬 **AI Colearning Prompt** (at start):
  - "Help me plan the Time Zone Converter architecture—what functions do I need?"
  - Placement: Before implementation

- 🎓 **Instructor Commentary** (during implementation):
  - "You're not writing this alone. Your AI is your pair programmer: you describe intent, AI suggests implementation, you validate and refine. This is AI-native development in practice."
  - Placement: Mid-implementation

- 🚀 **CoLearning Challenge** (during testing):
  - "Ask AI to help you handle edge cases: 'What if user enters invalid timezone? How do I test DST transitions?'"
  - Expected Outcome: Robust error handling and edge case coverage
  - Placement: After core implementation

- ✨ **Teaching Tip** (near end):
  - "Use AI for the whole workflow: planning → implementation → debugging → validation → documentation. This is how professionals work with AI."
  - Placement: During reflection phase

**Try With AI** (4 prompts, progressive):

1. **Recall**: "List all chapter concepts used in this capstone (math, time, datetime, calendar)"
   - Expected: Complete inventory of integrated concepts
   - Validates: Comprehensive understanding

2. **Understand**: "Explain your design choices—why did you structure functions this way?"
   - Expected: Architectural justification
   - Validates: Design reasoning

3. **Apply**: "Add a feature that converts multiple timezones at once (e.g., show current time in NYC, London, Tokyo)"
   - Expected: Extended functionality with batch conversion
   - Validates: Extension capability

4. **Synthesize**: "Reflect on your AI partnership: How did AI help you learn vs just complete the project?"
   - Expected: Metacognitive reflection on AI-native learning
   - Validates: Understanding of AI collaboration patterns

**Tool Selection**: ChatGPT web OR learner's AI companion

**Lesson ends with "Try With AI"** — no additional sections

---

## Content Flow & Dependencies

**Lesson Progression**:
1. **Lesson 1** establishes mathematical foundations (operations, validation, constants)
2. **Lesson 2** introduces time concepts (epoch, timestamps, low-level operations)
3. **Lesson 3** builds datetime objects and parsing (Python 3.14 native methods)
4. **Lesson 4** adds formatting and manipulation (strftime, timedelta, timezone conversion)
5. **Lesson 5** covers advanced topics (calendar, trigonometry, logarithms, special values)
6. **Lesson 6** integrates everything into real application (capstone project)

**Dependency Chain**:
- Lesson 2 depends on Lesson 1 (validation patterns)
- Lesson 3 depends on Lesson 2 (time concepts foundation)
- Lesson 4 depends on Lesson 3 (datetime objects)
- Lesson 5 is parallel to Lessons 3-4 (independent advanced topics)
- Lesson 6 depends on Lessons 1-5 (integration of all concepts)

**Prerequisites from Prior Chapters**:
- Chapter 14 (Data Types): Type hints, int, float, str
- Chapter 16 (Strings): f-strings, string formatting
- Chapter 21 (Exception Handling): try/except for validation

---

## Scaffolding Strategy

**A2 Lessons (1-2)**: Foundation building
- Introduce concepts with clear examples
- Provide worked examples before independent practice
- Validate with simple error handling
- Use AI for concept exploration (not just code generation)
- Max 5-7 new concepts per lesson

**A2-B1 Transition (Lesson 3)**: Bridging to intermediate
- Introduce Python 3.14's new methods as PRIMARY approach
- Balance worked examples with guided practice
- Emphasize timezone awareness without overwhelming
- Use AI to explore complexity (DST, edge cases)
- Max 7 new concepts (at limit)

**B1 Lessons (4-5)**: Independent application
- Students apply concepts to real scenarios
- Less scaffolding, more problem-solving
- AI assists with complex topics (timezone conversions, advanced math)
- Students explain WHEN to use methods (not just HOW)
- Max 7 new concepts (appropriate for B1)

**B1 Integration (Lesson 6)**: Synthesis and creation
- No new concepts; integration only
- AI partnership throughout development workflow
- Students demonstrate comprehensive understanding
- Focus on design decisions and validation

**Graduated Teaching Pattern Application** (Constitution Principle 13):

**Tier 1 - Foundational** (Book teaches directly):
- Epoch concept (what it is, why it matters)
- Date/time object structure (what fields exist)
- Format code patterns (%Y, %m, %d basics)
- Mathematical constants (pi, e, tau)

**Tier 2 - Complex Syntax** (AI companion handles):
- Complete strftime format code reference (30+ codes)
- DST edge cases and ambiguous times
- Advanced timezone conversion scenarios
- Trigonometric formula derivations

**Tier 3 - Scaling** (AI orchestration):
- Batch timezone conversions (convert 10 dates at once)
- Calendar generation for multiple months
- Complex date calculations across timezones
- Multi-format output generation

---

## Integration Points

**Connection to Previous Chapters**:
- Chapter 14: Type hints for math/datetime functions
- Chapter 16: String formatting for datetime output
- Chapter 21: Exception handling for validation

**Connection to Future Chapters** (no forward references in content):
- Chapter 24+ (OOP): datetime classes as examples of object design
- Part 5 (SDD): Specifications for datetime applications
- Part 12 (Databases): Storing datetime in databases

**Cross-Cutting Themes**:
- Validation-first approach (all lessons)
- AI as co-reasoning partner (all lessons)
- Type hints for clarity (all lessons)
- Understanding WHEN to use methods, not just HOW (all lessons)

---

## Validation Strategy

**Technical Validation**:
- All code examples run on Python 3.14+ without errors
- No deprecated methods (`utcnow()`, `utcfromtimestamp()`)
- Type hints present in all examples
- Error handling demonstrates best practices

**Pedagogical Validation**:
- Cognitive load limits respected (max 7 concepts per lesson)
- CEFR progression is smooth (A2 → A2-B1 → B1, no zigzag)
- Learning objectives are measurable and testable
- AI CoLearning elements throughout (not just "Try With AI" at end)

**AI-Native Learning Validation**:
- 💬🎓🚀✨ elements present in ALL lessons
- "Try With AI" is ONLY closure section (no summaries after)
- Conversational tone throughout (you, your, we)
- Exploration-focused language (discover, explore, try)
- 4 progressive prompts in "Try With AI" (Recall → Understand → Apply → Analyze/Synthesize)

**Content Quality Validation**:
- No terminology from future chapters (OOP, SDD, deployment)
- Examples are practical and relevant
- Explanations are clear at grade 7 baseline
- Code standards: f-strings only, `X | None`, modern type hints

**Assessment Methods by Proficiency Level**:

**A2-Level Assessment**:
- Multiple choice: Identify correct format code for year
- Short answer: Explain what epoch means
- Code completion: Fill in missing type hints
- Simple application: Parse given date string

**B1-Level Assessment**:
- Real problem: Parse user input and convert timezone
- Analysis: Compare two formatting approaches; explain when to use each
- Design: Sketch error handling strategy for timezone converter
- Application: Calculate duration between two dates with timezone conversion

**Capstone Assessment** (B1 synthesis):
- Complete working application (all requirements met)
- Multiple timezone scenarios tested (NYC, London, Tokyo, etc.)
- Error handling for invalid inputs (dates, timezones, formats)
- Reflection on AI partnership (metacognitive assessment)

---

## Skills Proficiency Summary (All Lessons)

**Chapter-Wide CEFR Progression**:
- Lesson 1: A2 (basic mathematical operations)
- Lesson 2: A2 (basic time concepts)
- Lesson 3: A2-B1 (bridging with Python 3.14 parsing)
- Lesson 4: B1 (intermediate formatting and conversion)
- Lesson 5: B1 (intermediate advanced topics)
- Lesson 6: B1 (synthesis and integration)

**Total Skills Taught**: 12 skills across 6 lessons

**By Category**:
- Technical: 8 skills (mathematical operations, parsing, formatting, conversion, calendar, trigonometry)
- Conceptual: 3 skills (domain errors, time concepts, timezone awareness)
- Soft: 1 skill (AI-assisted project planning)

**By Proficiency Level**:
- A2: 6 skills (foundation and basic application)
- B1: 6 skills (intermediate application and synthesis)

**Bloom's Taxonomy Distribution**:
- Remember: 0 (no pure memorization)
- Understand: 3 (conceptual understanding)
- Apply: 7 (practical application)
- Analyze: 1 (decomposition in capstone)
- Evaluate: 1 (validation in capstone)
- Create: 1 (synthesis in capstone)

**DigComp Areas Covered**:
- Information Literacy: 3 skills
- Content Creation: 5 skills
- Problem-Solving: 4 skills
- Safety: 1 skill (validation in capstone)

**Cognitive Load Compliance**:
- All lessons ≤ 7 concepts (validated)
- Progressive increase appropriate for CEFR levels
- Integration capstone introduces 0 new concepts

---

## Code Example Inventory

**Total Examples**: 18-20 across all lessons

**Lesson 1** (4 examples):
- Arithmetic with type hints
- Square root with validation
- Rounding comparisons
- Pi for calculations

**Lesson 2** (4 examples):
- Current timestamp
- Timestamp to time tuple
- Formatting with asctime()
- Elapsed time calculation

**Lesson 3** (5 examples):
- Creating date/time/datetime objects
- date.strptime() (Python 3.14 NEW)
- time.strptime() (Python 3.14 NEW)
- datetime.now()
- Timezone-aware datetime

**Lesson 4** (5 examples):
- Multiple formatting styles
- Timedelta arithmetic
- Duration calculations
- UTC to timezone conversion
- Localized formatting

**Lesson 5** (5 examples):
- Calendar display
- Trigonometric calculations
- Logarithm applications
- Factorial for combinatorics
- Handling inf/nan

**Lesson 6** (1 complete application):
- Time Zone Converter (modular functions, full workflow)

**Example Complexity Progression**:
- A2 examples: Simple, single-concept demonstrations
- A2-B1 examples: Multi-step with some complexity
- B1 examples: Real-world scenarios, multiple concepts
- B1 capstone: Full application, all concepts integrated

---

## Python 3.14 Feature Integration

**Primary Features** (taught as standard, not alternatives):
1. `date.strptime()` — PRIMARY method for parsing date strings (Lesson 3)
2. `time.strptime()` — PRIMARY method for parsing time strings (Lesson 3)
3. `datetime.now(timezone.utc)` — Modern timezone-aware approach (Lesson 3)
4. Enhanced math error messages — Better domain error feedback (Lesson 1)
5. Calendar color highlighting — Today's date in terminal (Lesson 5)

**NOT Taught** (deprecated or backward compatibility):
- ❌ `utcnow()` — deprecated in Python 3.12+
- ❌ `utcfromtimestamp()` — deprecated in Python 3.12+
- ❌ Old parsing workarounds — Python 3.14 has direct methods

**Teaching Approach**:
- Present Python 3.14 methods as "how you do it"
- No mention of "new in 3.14" in student-facing content (just use it)
- Plan documents this for instructor context
- Students learn modern approach from day one

---

## Quality Assurance Checklist

**Constitution Alignment**:
- [x] Evals-first approach (success criteria defined in spec before planning)
- [x] Specification-first development (spec approved before plan)
- [x] AI as co-reasoning partner (throughout all lessons)
- [x] Validation-first safety (error handling in all examples)
- [x] Graduated Teaching Pattern applied (Tier 1/2/3 defined)
- [x] Progressive complexity (A2 → A2-B1 → B1)
- [x] Learning by building (capstone integrates concepts)

**AI-Native Learning Compliance**:
- [x] CoLearning elements (💬🎓🚀✨) in ALL lessons throughout
- [x] "Try With AI" ONLY closure section (no summaries after)
- [x] 4 progressive prompts (Recall → Understand → Apply → Analyze/Synthesize)
- [x] Conversational tone (you, your, we)
- [x] Exploration-focused language (discover, explore, try)
- [x] Tool selection policy (ChatGPT web OR learner's AI companion)

**Python 3.14 Native Approach**:
- [x] `date.strptime()` as PRIMARY parsing method
- [x] `time.strptime()` as PRIMARY parsing method
- [x] `datetime.now(timezone.utc)` for timezone-aware objects
- [x] Enhanced error messages mentioned
- [x] Calendar color highlighting mentioned
- [x] NO deprecated methods (`utcnow()`, `utcfromtimestamp()`)

**Skills Proficiency Mapping**:
- [x] All skills mapped to CEFR levels (A1/A2/B1/B2/C1)
- [x] Proficiency progression validated (A2 → A2-B1 → B1, no zigzag)
- [x] Bloom's taxonomy aligned with CEFR levels
- [x] DigComp areas assigned
- [x] Measurable indicators defined for each skill
- [x] Cognitive load tracked (max 7 concepts per lesson)

**Cognitive Load Validation**:
- [x] Lesson 1: 6 concepts (within A2 limit of 7) ✓
- [x] Lesson 2: 5 concepts (within A2 limit of 7) ✓
- [x] Lesson 3: 7 concepts (at A2-B1 max limit of 7) ✓
- [x] Lesson 4: 6 concepts (within B1 limit of 7) ✓
- [x] Lesson 5: 7 concepts (at B1 max limit of 7) ✓
- [x] Lesson 6: 0 new concepts (integration only) ✓

**Code Quality Standards**:
- [x] Python 3.14+ syntax specified
- [x] Modern type hints (`date`, `datetime`, `float`, `int`, `str`, `X | None`)
- [x] f-strings only (no `.format()` or `%`)
- [x] Input validation before operations specified
- [x] All datetime code uses modern approaches

**Pedagogical Quality**:
- [x] Learning objectives are measurable
- [x] CEFR progression is smooth (no regression)
- [x] Prerequisites explicitly listed (Ch 14, 16, 21)
- [x] No forward references to future chapters
- [x] Capstone integrates all chapter concepts
- [x] Success metrics defined (from spec)

**Content Completeness**:
- [x] 5 foundational lessons + 1 capstone defined
- [x] 18-20 code examples specified
- [x] Time Zone Converter capstone requirements detailed
- [x] All lessons have 4-prompt "Try With AI" sections
- [x] CoLearning elements specified for all lessons

---

## Implementation Notes for lesson-writer Subagent

**Priorities**:
1. **Python 3.14 Native**: Teach `date.strptime()` and `time.strptime()` as PRIMARY methods (never mention as "new" or "alternative")
2. **AI-Native Learning Throughout**: Apply 💬🎓🚀✨ elements THROUGHOUT lessons (not just "Try With AI" at end)
3. **Graduated Teaching Pattern**: Book teaches foundational → AI companion handles complex → AI orchestration at scale
4. **Cognitive Load Respect**: Max 7 concepts per lesson enforced strictly
5. **Lesson Closure**: "Try With AI" ONLY (4 prompts, progressive Bloom's levels, NO summaries/checklists/what's next after)

**Tone Requirements**:
- ✅ Conversational (you, your, we)
- ✅ Exploration-focused (discover, explore, try)
- ✅ AI partnership (co-teacher, pair-teacher)
- ❌ NOT documentation style
- ❌ NOT reference manual

**Code Standards**:
- Python 3.14+ syntax
- f-strings only (no `.format()` or `%`)
- `X | None` instead of `Optional[X]`
- Type hints: `date`, `datetime`, `float`, `int`, `str`
- No deprecated methods
- Validate before operating

**Context Materials** (Ruthlessly Filtered):
- ✅ USE: Math functions, datetime objects, epoch, calendar, Python 3.14 features
- ❌ SKIP: File I/O (Ch 22), OOP (Ch 24+), Advanced error handling (Ch 21 basics only), Future chapters

**Validation Checkpoints**:
- technical-reviewer MUST verify: AI CoLearning elements throughout, lesson closure pattern, Python 3.14 native approach, no SDD terminology
- Automated testing MUST verify: All code runs on Python 3.14+, no deprecated methods, type hints present
- Skills-proficiency-mapper MUST validate: A2-B1 progression smooth, cognitive load within limits, CEFR levels appropriate

**Success Metrics** (from specification SC section):
- 85%+ capstone completion rate
- 80%+ demonstrate asking AI for concepts (not just code)
- 90%+ can explain WHEN to use methods (not just HOW)
- 100% compliance with AI CoLearning elements and lesson closure pattern

---

## Capstone Integration Details

**How Capstone Pulls Together All 5 Foundational Lessons**:

**From Lesson 1 (Math Module)**:
- Input validation (ensure dates are valid before parsing)
- Error handling patterns (try/except for robust application)
- Type hints throughout (all functions properly annotated)

**From Lesson 2 (Time and Epoch)**:
- Timestamp display for debugging (show epoch seconds)
- Understanding time measurement (background knowledge)
- Time tuple structure (if displaying detailed time info)

**From Lesson 3 (Date/Time Objects)**:
- Python 3.14 parsing methods (`date.strptime()`, `time.strptime()`)
- Creating timezone-aware datetime objects
- Understanding naive vs aware objects

**From Lesson 4 (Formatting and Manipulation)**:
- Multi-format output (ISO 8601, friendly, timestamp)
- Timezone conversion (core functionality)
- Timedelta for duration display (optional: "in 3 days, 4 hours")

**From Lesson 5 (Calendar and Advanced Math)**:
- Optional calendar display (show current month with highlighted date)
- Math validation (edge cases, special values)
- Advanced error handling (inf/nan if relevant)

**AI Partnership Workflow** (Planning → Implementation → Debugging → Validation):

**Phase 1: Planning with AI**:
- Student: "I need to build a timezone converter. What functions should I create?"
- AI: Suggests architecture (parse_input, convert_timezone, format_output, main)
- Student: Reviews, refines, approves plan

**Phase 2: Implementation with AI**:
- Student: "Generate parse_input() that uses Python 3.14's date.strptime()"
- AI: Provides code with type hints and error handling
- Student: Reviews, tests, validates against requirements

**Phase 3: Debugging with AI**:
- Student: Encounters parsing error, shares with AI
- AI: Explains error, suggests fix
- Student: Applies fix, tests, confirms resolution

**Phase 4: Validation with AI**:
- Student: "Help me test edge cases: invalid dates, unknown timezones, DST transitions"
- AI: Suggests test scenarios and validation strategies
- Student: Implements tests, validates application works

**Reflection Questions** (Lesson 6 "Try With AI" Prompt 4):
- How did AI help you plan vs implementing this project?
- What did you learn from AI explanations vs just getting code?
- When did you validate AI suggestions vs accepting them directly?
- How will you apply AI partnership in future projects?

---

## Status & Next Steps

**Plan Status**: ✅ Complete and ready for `/sp.tasks` generation

**Next Phase**: Generate detailed task checklist (`tasks.md`) breaking down each lesson into specific implementation tasks

**Estimated Implementation Timeline**:
- Lesson 1: 8-10 hours (content writing, 4 code examples, exercises, review)
- Lesson 2: 8-10 hours (content writing, 4 code examples, exercises, review)
- Lesson 3: 10-12 hours (content writing, 5 code examples, Python 3.14 focus, exercises, review)
- Lesson 4: 10-12 hours (content writing, 5 code examples, timezone complexity, exercises, review)
- Lesson 5: 10-12 hours (content writing, 5 code examples, advanced topics, exercises, review)
- Lesson 6: 12-15 hours (capstone application, testing, validation, reflection)
- **Total**: 58-71 hours for complete chapter

**Validation Requirements**:
- Technical review by technical-reviewer subagent
- Skills proficiency validation by skills-proficiency-mapper
- Constitution compliance check (AI-Native Learning, Python 3.14 native, no deprecated content)
- Code testing (all examples run on Python 3.14+)

**Human Approval Required**:
- Plan approval before `/sp.tasks` generation
- Spec alignment confirmation
- Complexity tier appropriateness verification

---

**Plan Version**: 1.0.0
**Created**: 2025-11-09
**Source Spec**: `specs/001-part-4-chapter-23/spec.md`
**Generated By**: chapter-planner v3.0.2
**Constitutional Alignment**: v3.0.2 (AI-Native Learning, Python 3.14 native, Skills Proficiency Mapping)
