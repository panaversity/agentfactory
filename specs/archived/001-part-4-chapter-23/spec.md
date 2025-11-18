# Feature Specification: Chapter 23 - Math, Date Time Calendar

**Feature Branch**: `001-part-4-chapter-23`
**Created**: 2025-11-09
**Status**: Draft
**Input**: User description: "Chapter 23: Math, Date Time Calendar - Python 3.14 native datetime parsing, math operations, and calendar functionality with AI-Native Learning pedagogy"

**Chapter Metadata**:
- **Number**: 23
- **Title**: "Math, Date Time Calendar" (from chapter-index.md)
- **Part**: 4 (Python Fundamentals)
- **Complexity Tier**: Intermediate (Chapters 17-23)
- **CEFR Range**: A2-B1 (max 7 concepts per lesson)
- **Prerequisites**: Chapters 1-22 (especially Ch 14 Data Types, Ch 16 Strings, Ch 21 Exception Handling)
- **Python Version**: 3.14+ (uses NEW strptime() methods, enhanced error messages, calendar color highlighting)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Perform Mathematical Calculations with Validation (Priority: P1)

As a Python learner, I want to perform mathematical calculations (square roots, logarithms, trigonometry, rounding) with proper error handling so I can build applications that validate numeric inputs and prevent domain errors.

**Why this priority**: Mathematical operations are foundational for scientific computing, data analysis, and everyday programming tasks. Students must understand WHEN to use math functions (not memorize all 40+) and how to validate results.

**Independent Test**: Can be fully tested by having students write functions that perform calculations with type hints, validate inputs (negative numbers for sqrt, zero for division), and handle domain errors using Python 3.14's enhanced error messages. Delivers immediate value for any numeric computation task.

**Acceptance Scenarios**:

1. **Given** a need to calculate square root, **When** student uses `math.sqrt()` with positive number, **Then** result is calculated correctly with proper type hints
2. **Given** an invalid input (negative number for sqrt), **When** student attempts calculation, **Then** Python 3.14 provides detailed domain error message that student can interpret with AI assistance
3. **Given** need for mathematical constants, **When** student uses `math.pi`, `math.e`, `math.tau`, **Then** precise values are available for calculations
4. **Given** floating-point results, **When** student uses `round()`, `math.ceil()`, `math.floor()`, **Then** values are rounded appropriately with understanding of differences
5. **Given** special numeric values, **When** student encounters `math.inf` or `math.nan`, **Then** they understand these as validation concepts for edge cases

---

### User Story 2 - Work with Dates and Times Programmatically (Priority: P1)

As a Python learner, I want to create, parse, format, and manipulate dates and times using Python 3.14's datetime module so I can build applications that track events, calculate durations, and display time information correctly.

**Why this priority**: Datetime handling is critical for any real-world application (scheduling, logging, analytics, user interfaces). Python 3.14 introduces NEW `date.strptime()` and `time.strptime()` methods that simplify parsing - students should learn these as PRIMARY approaches.

**Independent Test**: Can be fully tested by having students create date/time objects, parse user-provided date strings using Python 3.14's new methods, format output with `strftime()`, and calculate durations with timedelta. Delivers immediate value for scheduling and time-tracking applications.

**Acceptance Scenarios**:

1. **Given** a date string like "2025-11-09", **When** student uses `date.strptime()` (NEW in 3.14), **Then** date object is created correctly with type hints
2. **Given** a time string like "14:30:45", **When** student uses `time.strptime()` (NEW in 3.14), **Then** time object is created correctly
3. **Given** need for current date/time, **When** student uses `datetime.now()`, **Then** current timestamp is captured with optional timezone awareness
4. **Given** two dates, **When** student calculates difference using timedelta, **Then** duration (days, hours, minutes) is computed accurately
5. **Given** a datetime object, **When** student formats it using `strftime()` with format codes, **Then** output matches specified format (e.g., "November 09, 2025" or "2025-11-09 14:30")

---

### User Story 3 - Understand Time Concepts and Epoch (Priority: P2)

As a Python learner, I want to understand how computers measure time (epoch, timestamps, ticks) and use the `time` module so I can work with Unix timestamps and convert between time representations.

**Why this priority**: Understanding epoch (January 1, 1970 UTC) is essential for working with timestamps in APIs, databases, and logs. This is foundational knowledge for professional development but builds on datetime concepts (hence P2, not P1).

**Independent Test**: Can be fully tested by having students explain what epoch means, use `time.time()` to get current timestamp, convert timestamps to human-readable format with `time.localtime()` and `time.asctime()`. Delivers value for understanding system logs and API timestamps.

**Acceptance Scenarios**:

1. **Given** need to understand time measurement, **When** student explains epoch concept, **Then** they can articulate "seconds since January 1, 1970 UTC" as the reference point
2. **Given** need for current timestamp, **When** student uses `time.time()`, **Then** floating-point seconds since epoch is returned
3. **Given** a timestamp, **When** student converts using `time.localtime()`, **Then** time tuple with year, month, day, hour, minute, second is produced
4. **Given** a time tuple, **When** student formats using `time.asctime()`, **Then** human-readable string like "Sat Nov 09 14:30:00 2025" is generated

---

### User Story 4 - Handle Timezones in Applications (Priority: P2)

As a Python learner, I want to work with timezone-aware datetime objects and convert times between timezones so I can build applications that serve global users correctly.

**Why this priority**: Timezone handling is crucial for global applications but introduces complexity (DST, UTC offsets). Taught after basic datetime concepts (hence P2). Uses Python 3.14's recommended `datetime.now(timezone.utc)` instead of deprecated `utcnow()`.

**Independent Test**: Can be fully tested by having students create timezone-aware datetime objects, convert between timezones (e.g., UTC to EST), and handle edge cases (DST transitions) with AI assistance. Delivers value for building international applications.

**Acceptance Scenarios**:

1. **Given** need for UTC time, **When** student uses `datetime.now(timezone.utc)` (not deprecated `utcnow()`), **Then** timezone-aware datetime object is created
2. **Given** a UTC datetime, **When** student converts to another timezone (e.g., US/Eastern), **Then** offset is applied correctly and local time is displayed
3. **Given** two times in different timezones, **When** student calculates difference, **Then** actual duration is computed accounting for timezone offsets
4. **Given** ambiguous times (DST transitions), **When** student encounters errors, **Then** they can use AI to understand and handle edge cases

---

### User Story 5 - Display Calendars and Advanced Math (Priority: P3)

As a Python learner, I want to generate calendar displays and perform advanced mathematical operations (trigonometry, logarithms) so I can build diverse applications from scheduling interfaces to scientific calculators.

**Why this priority**: Calendar and advanced math are useful but less frequently needed than core datetime/math operations (hence P3). Includes Python 3.14's calendar color highlighting feature and trigonometric functions.

**Independent Test**: Can be fully tested by having students generate calendar displays with `calendar.month()` (observing color highlighting in terminal), perform trigonometric calculations for angles, and work with logarithms. Delivers value for specialized applications (scheduling UIs, scientific computing).

**Acceptance Scenarios**:

1. **Given** need to display a month calendar, **When** student uses `calendar.month(2025, 11)`, **Then** calendar grid is generated with today's date highlighted in color (Python 3.14 terminal feature)
2. **Given** an angle in radians, **When** student calculates `math.sin()`, `math.cos()`, `math.tan()`, **Then** trigonometric values are computed accurately
3. **Given** a number, **When** student calculates `math.log()`, `math.log10()`, **Then** natural and base-10 logarithms are computed with understanding of when to use each
4. **Given** need for factorial, **When** student uses `math.factorial(5)`, **Then** result (120) is computed and student understands combinatorial applications

---

### User Story 6 - Build Practical Time Zone Converter (Capstone) (Priority: P1)

As a Python learner, I want to build a complete Time Zone Converter application that demonstrates all chapter concepts (date parsing, time conversion, formatting, validation) so I can apply AI-Native Learning to a real-world project.

**Why this priority**: Capstone projects are critical for integrating all chapter knowledge into a working application. Time Zone Converter is practical, uses all learned concepts, and demonstrates AI partnership throughout development (hence P1 for pedagogical completeness).

**Independent Test**: Can be fully tested by students building the application with AI assistance, testing with multiple timezone scenarios, validating user input, handling errors gracefully, and demonstrating the working converter. Delivers value as a portfolio piece and practical tool.

**Acceptance Scenarios**:

1. **Given** user input (date/time string and source timezone), **When** student's app parses using `date.strptime()` or `datetime.strptime()`, **Then** datetime object is created with validation
2. **Given** a datetime in one timezone, **When** student's app converts to target timezone, **Then** correct local time is displayed with offset information
3. **Given** invalid input (malformed date, invalid timezone), **When** user submits, **Then** app provides helpful error messages and recovery options
4. **Given** multiple timezone conversions, **When** student's app displays results, **Then** output is formatted clearly using `strftime()` with user-friendly labels
5. **Given** completed capstone, **When** student explains design choices, **Then** they articulate how AI assisted with planning, implementation, debugging, and validation

---

### Edge Cases

- **What happens when parsing invalid date strings?** Python 3.14 `date.strptime()` raises `ValueError` with detailed message - students learn to handle with try/except and ask AI to interpret errors
- **How does system handle negative numbers in math.sqrt()?** Raises `ValueError` with Python 3.14's enhanced domain error message - students use AI to understand mathematical constraints
- **What happens during DST transitions?** Ambiguous or non-existent times can occur - students explore with AI to understand fold parameter and timezone complexity
- **How are leap years handled in date arithmetic?** Python's datetime correctly accounts for leap years - students validate with test cases (e.g., February 29, 2024)
- **What happens with very large or very small numbers?** Math operations may produce `math.inf` (infinity) or underflow to zero - students learn these as validation concepts
- **How does calendar.month() detect "today" for color highlighting?** Uses system date - students experiment in terminal to see Python 3.14's color feature

## Requirements *(mandatory)*

### Functional Requirements

**Math Module Requirements**:

- **FR-001**: Chapter MUST teach core math functions (abs, pow, round, max, min, sqrt) with clear explanations of WHEN to use each (not just HOW)
- **FR-002**: Chapter MUST introduce mathematical constants (pi, e, tau) with practical examples showing their use in calculations
- **FR-003**: Chapter MUST demonstrate error handling for domain errors (e.g., negative sqrt) using Python 3.14's enhanced error messages
- **FR-004**: Chapter MUST teach special numeric values (inf, nan) as validation concepts for edge cases, not just trivia
- **FR-005**: Chapter MUST include trigonometric functions (sin, cos, tan) and logarithms (log, log10) for scientific computing applications
- **FR-006**: Chapter MUST show rounding differences (round vs ceil vs floor) with examples students can test and validate with AI

**Time Module Requirements**:

- **FR-007**: Chapter MUST explain epoch concept (January 1, 1970 UTC) as the foundation of time measurement in computing
- **FR-008**: Chapter MUST teach `time.time()` for current timestamp with type hints (`float`)
- **FR-009**: Chapter MUST demonstrate converting timestamps to human-readable format using `time.localtime()` and `time.asctime()`
- **FR-010**: Chapter MUST clarify that time module provides low-level utilities while datetime module provides higher-level objects

**Datetime Module Requirements (Python 3.14 Native)**:

- **FR-011**: Chapter MUST teach Python 3.14's NEW `date.strptime()` method as PRIMARY approach for parsing date strings (not as alternative)
- **FR-012**: Chapter MUST teach Python 3.14's NEW `time.strptime()` method as PRIMARY approach for parsing time strings (not as alternative)
- **FR-013**: Chapter MUST use `datetime.now(timezone.utc)` and NEVER teach deprecated `utcnow()` or `utcfromtimestamp()`
- **FR-014**: Chapter MUST demonstrate creating date, time, and datetime objects with clear type hints
- **FR-015**: Chapter MUST teach `strftime()` formatting with practical format codes and reference to complete documentation
- **FR-016**: Chapter MUST demonstrate timedelta for date arithmetic (adding/subtracting days, hours, calculating durations)
- **FR-017**: Chapter MUST introduce timezone awareness gradually (Lesson 3: basics, Lesson 4: conversions) without overwhelming students

**Calendar Module Requirements**:

- **FR-018**: Chapter MUST demonstrate `calendar.month()` with mention of Python 3.14's color highlighting feature (visible in terminal)
- **FR-019**: Chapter MUST show practical use cases for calendar (displaying month grids, calculating week numbers)

**AI-Native Learning Requirements**:

- **FR-020**: ALL lessons MUST apply Graduated Teaching Pattern (Book teaches foundational → AI companion handles complex → AI orchestration at scale)
- **FR-021**: ALL lessons MUST include AI CoLearning elements THROUGHOUT (💬 prompts, 🎓 commentaries, 🚀 challenges, ✨ tips) - not just at end
- **FR-022**: ALL lessons MUST end with "Try With AI" section ONLY (4 progressive prompts, NO summaries/checklists after)
- **FR-023**: ALL lessons MUST use conversational tone (you, your, we) and exploration-focused language, NOT documentation style
- **FR-024**: Chapter MUST teach students to ask AI for explanations (not memorize formulas/format codes)
- **FR-025**: Chapter MUST emphasize validation-first practice: test code, interpret errors with AI, iterate understanding

**Code Quality Requirements**:

- **FR-026**: ALL code examples MUST use Python 3.14+ syntax with modern type hints (`list[int]`, `X | None`, `date`, `datetime`, `float`)
- **FR-027**: ALL code examples MUST use f-strings only (no `.format()` or `%`)
- **FR-028**: ALL datetime code MUST use timezone-aware approaches where appropriate (no deprecated methods)
- **FR-029**: ALL code examples MUST validate user input before parsing dates or performing math operations

**Pedagogical Requirements**:

- **FR-030**: Chapter MUST respect cognitive load limits (max 7 concepts per lesson for A2-B1 intermediate tier)
- **FR-031**: Chapter MUST progress from A2 → A2-B1 → B1 complexity across 5-6 lessons (not counting capstone)
- **FR-032**: Chapter MUST include 15-20 code examples total with clear pedagogical purpose for each
- **FR-033**: Capstone (Time Zone Converter) MUST integrate all chapter concepts (parsing, conversion, formatting, validation, error handling)
- **FR-034**: Chapter MUST reference Chapter 14 (Data Types), Chapter 16 (Strings), Chapter 21 (Exception Handling) as prerequisites
- **FR-035**: Chapter MUST NOT reference future chapters (24+ OOP, 30+ SDD) or use terminology inappropriate for Part 4

### Key Entities *(include if feature involves data)*

**Core Python Modules** (what students learn):
- **math module**: Built-in module providing mathematical functions (sqrt, sin, cos, log, etc.) and constants (pi, e, tau, inf, nan) - students learn WHEN to use each function
- **time module**: Low-level module for time-related functions (epoch timestamps, time tuples, formatting) - foundation for understanding time measurement
- **datetime module**: High-level module providing date, time, and datetime classes with Python 3.14's new strptime() methods for parsing
- **calendar module**: Module for calendar-related functions (month grids, week calculations) with Python 3.14's color highlighting

**Python 3.14 New Features** (students learn these as standard):
- **date.strptime()**: NEW class method in Python 3.14 for parsing date strings directly into date objects
- **time.strptime()**: NEW class method in Python 3.14 for parsing time strings directly into time objects
- **Enhanced math errors**: More detailed error messages for domain errors (helps students debug with AI)
- **Calendar color highlighting**: Automatic color highlighting of today's date in terminal output

**Time Concepts** (foundational knowledge):
- **Epoch**: Reference point (January 1, 1970 UTC) from which time is measured in seconds
- **Timestamp**: Floating-point number representing seconds since epoch
- **Timezone**: Geographic region's time offset from UTC (students learn basics, explore edge cases with AI)
- **Timedelta**: Object representing duration between two dates/times

**Pedagogical Entities** (how students learn):
- **AI Co-Teacher**: Claude Code or Gemini CLI as pair-teacher for exploration, debugging, validation
- **Graduated Complexity**: A2 → A2-B1 → B1 progression matching intermediate learners' growth
- **CoLearning Elements**: 💬🎓🚀✨ positioned throughout lessons (not just "Try With AI" at end)
- **Capstone Project**: Time Zone Converter application integrating all chapter concepts

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Technical Proficiency** (Technology-Agnostic):

- **SC-001**: Students can perform mathematical calculations with validation (test inputs, handle domain errors) in under 5 minutes for common operations (sqrt, round, trigonometry)
- **SC-002**: Students can parse date/time strings from user input and convert to internal objects with 95%+ success rate on first attempt (using Python 3.14's new methods)
- **SC-003**: Students can format datetime objects for display in multiple formats (ISO 8601, localized, custom) with 100% accuracy
- **SC-004**: Students can calculate time differences and convert between timezones with correct results for 90%+ of test cases
- **SC-005**: 85%+ of students successfully build working Time Zone Converter capstone project that handles multiple timezones and validates input

**AI Partnership Skills** (Behavioral):

- **SC-006**: Students demonstrate asking AI for concept explanations (not just code) in 80%+ of "Try With AI" exercises
- **SC-007**: Students use AI to debug domain errors and timezone issues, showing iteration (ask → learn → refine) in 75%+ of debugging sessions
- **SC-008**: 90%+ of students can explain WHEN to use specific math functions or datetime methods (not just HOW) after AI-assisted exploration

**Learning Effectiveness** (Cognitive):

- **SC-009**: Students complete all 5-6 lessons without cognitive overload (max 7 concepts/lesson respected, feedback indicates manageable pace)
- **SC-010**: 80%+ of students progress from A2 (basic math/datetime operations) to B1 (timezone conversions, complex validation) by chapter end
- **SC-011**: Students retain knowledge 2 weeks post-chapter, demonstrating 70%+ accuracy on datetime parsing and math validation tasks

**Content Quality** (Pedagogical):

- **SC-012**: All lessons include AI CoLearning elements throughout (💬🎓🚀✨), verified by validation-auditor with 100% compliance
- **SC-013**: All lessons end with "Try With AI" ONLY (no summaries/checklists after), verified by validation-auditor with 100% compliance
- **SC-014**: All code examples run on Python 3.14+ without errors, verified by automated testing with 100% pass rate
- **SC-015**: All datetime code uses modern approaches (no deprecated `utcnow()`), verified by static analysis with zero violations

**Engagement and Application** (User-Facing):

- **SC-016**: Students complete Time Zone Converter capstone in 60-90 minutes with AI assistance, demonstrating practical application of all chapter concepts
- **SC-017**: 75%+ of students report increased confidence working with dates/times/math after chapter completion (post-chapter survey)
- **SC-018**: Students use Time Zone Converter project in portfolios or expand it for personal use, demonstrating real-world value

## Lesson Structure *(pedagogical specification)*

### Lesson 1: Math Module Fundamentals (CEFR: A2, Concepts: 6)

**Learning Objective**: Students will perform basic mathematical operations using Python's math module, understand when to use different rounding functions, and work with mathematical constants.

**Concepts** (6 total, within A2 limit of 7):
1. Built-in math functions vs math module (abs, pow, round, max, min are built-ins; sqrt requires import)
2. `math.sqrt()` for square roots with validation (positive numbers only)
3. Rounding functions: `round()` vs `math.ceil()` vs `math.floor()` (different behaviors)
4. Mathematical constants: `math.pi`, `math.e`, `math.tau` (what they represent, when to use)
5. Type hints for math operations (`float`, `int` parameters and returns)
6. Enhanced domain error messages in Python 3.14 (understanding "math domain error")

**Code Examples** (3-4 examples):
- Basic arithmetic with type hints and validation
- Square root with error handling for negative inputs
- Rounding comparisons (show differences between round/ceil/floor)
- Using pi for circle calculations

**AI CoLearning Elements**:
- 💬 Prompt: "Explain why `math.sqrt(-1)` raises an error and what Python 3.14's enhanced message tells us"
- 🎓 Commentary: "You don't memorize which rounding function to use—you understand the difference (round=nearest, ceil=up, floor=down) and ask AI when unsure"
- 🚀 Challenge: "Ask AI to generate a function that calculates circle area with type hints, then explain why pi is more accurate than 3.14"
- ✨ Tip: "Use Claude Code to explore edge cases: 'What happens if I round 2.5? What about -2.5?'"

**Try With AI** (4 prompts, progressive):
1. Recall: "Explain the difference between round(), ceil(), and floor() with examples"
2. Understand: "Why does Python separate built-in functions from the math module?"
3. Apply: "Generate a function that validates input before calculating square root"
4. Analyze: "When would you use math.tau instead of math.pi in real applications?"

---

### Lesson 2: Time and Epoch Concepts (CEFR: A2, Concepts: 5)

**Learning Objective**: Students will understand how computers measure time using epoch, work with timestamps, and convert between timestamp and human-readable formats.

**Concepts** (5 total, within A2 limit of 7):
1. Epoch concept (January 1, 1970 UTC as reference point)
2. `time.time()` for current timestamp (floating-point seconds since epoch)
3. `time.localtime()` for converting timestamp to time tuple
4. `time.asctime()` for formatting time tuple to human-readable string
5. Time tuples (year, month, day, hour, minute, second, weekday, yearday, isdst)

**Code Examples** (3-4 examples):
- Getting current timestamp and understanding epoch
- Converting timestamp to local time tuple
- Formatting time with asctime()
- Calculating elapsed time between two timestamps

**AI CoLearning Elements**:
- 💬 Prompt: "Why is January 1, 1970 chosen as epoch? What happens with dates before 1970?"
- 🎓 Commentary: "Timestamps might look confusing (1699564800.123), but they're just seconds since a fixed point—perfect for calculations. Let AI handle the conversions."
- 🚀 Challenge: "Ask AI to explain time tuple fields and generate code that extracts just the weekday"
- ✨ Tip: "Explore with your AI: 'Show me the timestamp for my birthday and explain how Python calculates it'"

**Try With AI** (4 prompts, progressive):
1. Recall: "What is the Unix epoch and why do computers use it?"
2. Understand: "Explain how time.time() relates to time.localtime() and time.asctime()"
3. Apply: "Create a function that calculates how many days have passed since a given timestamp"
4. Analyze: "Why are timestamps better than formatted date strings for storage and comparison?"

---

### Lesson 3: Date and Time Objects (Python 3.14) (CEFR: A2-B1, Concepts: 7)

**Learning Objective**: Students will create and parse date/time objects using Python 3.14's new `date.strptime()` and `time.strptime()` methods, work with `datetime.now()`, and understand timezone awareness basics.

**Concepts** (7 total, max for A2-B1):
1. `datetime.date` objects (year, month, day)
2. `datetime.time` objects (hour, minute, second, microsecond)
3. `datetime.datetime` objects (combines date and time)
4. **Python 3.14 NEW**: `date.strptime()` for parsing date strings directly
5. **Python 3.14 NEW**: `time.strptime()` for parsing time strings directly
6. `datetime.now()` with optional timezone parameter (modern approach)
7. Timezone awareness basics (naive vs aware datetime objects)

**Code Examples** (4-5 examples):
- Creating date/time/datetime objects with type hints
- Parsing dates using Python 3.14's `date.strptime()` (NEW)
- Parsing times using Python 3.14's `time.strptime()` (NEW)
- Getting current datetime with `datetime.now()`
- Creating timezone-aware datetime with `datetime.now(timezone.utc)`

**AI CoLearning Elements**:
- 💬 Prompt: "Python 3.14 added date.strptime() and time.strptime(). How do these improve on previous approaches?"
- 🎓 Commentary: "Before Python 3.14, parsing dates required workarounds. Now you parse directly—syntax is cheap, knowing WHEN to parse (user input) vs construct (programmatic dates) is gold"
- 🚀 Challenge: "Ask AI to generate a function that parses user birthday input and calculates age"
- ✨ Tip: "Always use datetime.now(timezone.utc) instead of deprecated utcnow()—ask AI why this matters"

**Try With AI** (4 prompts, progressive):
1. Recall: "What's new in Python 3.14 for date/time parsing?"
2. Understand: "Explain the difference between naive and timezone-aware datetime objects"
3. Apply: "Create a function that parses '2025-11-09' and calculates days until that date"
4. Analyze: "Why is timezone awareness important for global applications? When can you skip it?"

---

### Lesson 4: Date/Time Formatting and Manipulation (CEFR: B1, Concepts: 6)

**Learning Objective**: Students will format datetime objects for display using `strftime()`, perform date arithmetic with timedelta, and convert between timezones using Python 3.14's recommended approaches.

**Concepts** (6 total, within B1 limit of 7):
1. `strftime()` format codes (%Y, %m, %d, %H, %M, %S, %A, %B, etc.)
2. Formatting for different contexts (ISO 8601, localized, custom displays)
3. `timedelta` for representing durations (days, hours, minutes, seconds)
4. Date arithmetic (adding/subtracting timedelta to/from datetime)
5. Timezone conversion basics (UTC to local, local to local)
6. Handling timezone conversions with `timezone` and `timedelta` objects

**Code Examples** (4-5 examples):
- Formatting datetime in multiple styles (ISO, US, EU formats)
- Adding/subtracting days with timedelta
- Calculating duration between two datetimes
- Converting UTC to another timezone
- Formatting with localized month/weekday names

**AI CoLearning Elements**:
- 💬 Prompt: "Explain strftime format codes—why %Y vs %y, %H vs %I?"
- 🎓 Commentary: "You don't memorize all format codes (there are 30+). You understand the pattern (%Y=year, %m=month) and ask AI when you need a specific format"
- 🚀 Challenge: "Ask AI to generate code that displays 'meeting in 3 days, 4 hours' using timedelta"
- ✨ Tip: "Explore timezone complexity with AI: 'What happens during DST transitions? How do I handle ambiguous times?'"

**Try With AI** (4 prompts, progressive):
1. Recall: "List 5 common strftime format codes and their meanings"
2. Understand: "How does timedelta differ from datetime? When do you use each?"
3. Apply: "Create a function that formats a datetime in both ISO 8601 and 'friendly' format (e.g., 'November 9, 2025 at 2:30 PM')"
4. Analyze: "What are the challenges of timezone conversion? How can AI help you handle edge cases?"

---

### Lesson 5: Calendar and Advanced Math (CEFR: B1, Concepts: 7)

**Learning Objective**: Students will generate calendar displays using Python 3.14's enhanced calendar module, perform trigonometric calculations, work with logarithms, and understand special numeric values (infinity, NaN) as validation concepts.

**Concepts** (7 total, max for B1):
1. `calendar.month()` for generating calendar grids with Python 3.14's color highlighting
2. Using calendar for scheduling interfaces and week calculations
3. Trigonometric functions (`math.sin`, `math.cos`, `math.tan`) for angles in radians
4. Logarithms (`math.log`, `math.log10`) for scientific computing
5. `math.factorial()` for combinatorial calculations
6. Special values `math.inf` (infinity) and `math.nan` (not a number) as validation concepts
7. Testing for special values with `math.isinf()` and `math.isnan()`

**Code Examples** (4-5 examples):
- Displaying month calendar (observe color in terminal)
- Trigonometric calculations for right triangles
- Logarithm applications (exponential growth, decibels)
- Factorial for permutations/combinations
- Handling infinity and NaN in validation logic

**AI CoLearning Elements**:
- 💬 Prompt: "When running calendar.month() in your terminal, you'll see today's date in color (Python 3.14 feature). How does Python know 'today'?"
- 🎓 Commentary: "Trigonometry might seem abstract, but you don't memorize formulas—you understand WHEN to use sin/cos/tan (angles, waves, rotations) and let AI help with the math"
- 🚀 Challenge: "Ask AI to generate a calculator that converts degrees to radians and computes all three trig functions"
- ✨ Tip: "Explore edge cases: 'What is log(0)? What is factorial(-5)? How do I test if a result is infinite?'"

**Try With AI** (4 prompts, progressive):
1. Recall: "What are three practical uses for logarithms in programming?"
2. Understand: "Explain what math.inf and math.nan represent. Why are they useful for validation?"
3. Apply: "Generate code that displays next month's calendar and highlights specific dates"
4. Analyze: "When would you use trigonometric functions in a real application? (Think beyond math class)"

---

### Lesson 6 (Capstone): Time Zone Converter Project (CEFR: B1, Integration)

**Learning Objective**: Students will build a complete Time Zone Converter application that integrates all chapter concepts (parsing dates/times, timezone conversion, formatting, validation, error handling) with AI assistance throughout development.

**Integrated Concepts** (from all 5 lessons):
- Math validation (ensure valid dates, handle errors)
- Time concepts (display timestamps for debugging)
- Date/time parsing (Python 3.14's `strptime()` methods)
- Formatting (display results in multiple formats)
- Timezone conversion (core functionality)
- Calendar display (optional: show current month with highlighted date)
- AI partnership (planning, implementation, debugging, validation)

**Capstone Requirements**:
1. Accept user input: date/time string, source timezone, target timezone
2. Parse input using Python 3.14's new methods with validation
3. Convert datetime to target timezone (handle UTC, EST, PST, GMT, etc.)
4. Display results in multiple formats (ISO, friendly, timestamp)
5. Handle errors gracefully (invalid dates, unknown timezones, parsing failures)
6. Provide clear user feedback and recovery options
7. Use type hints throughout
8. Demonstrate AI-assisted development (show how AI helped at each stage)

**Code Examples** (1 complete application):
- Full Time Zone Converter with modular functions
- Input validation and error handling
- Multiple output formats
- Optional extensions (batch conversion, calendar view, DST warnings)

**AI CoLearning Elements**:
- 💬 Prompt: "Help me plan the Time Zone Converter architecture—what functions do I need?"
- 🎓 Commentary: "You're not writing this alone. Your AI is your pair programmer: you describe intent, AI suggests implementation, you validate and refine"
- 🚀 Challenge: "Ask AI to help you handle edge cases: 'What if user enters invalid timezone? How do I test DST transitions?'"
- ✨ Tip: "Use AI for the whole workflow: planning → implementation → debugging → validation → documentation"

**Try With AI** (4 prompts, progressive):
1. Recall: "List all chapter concepts used in this capstone (math, time, datetime, calendar)"
2. Understand: "Explain your design choices—why did you structure functions this way?"
3. Apply: "Add a feature that converts multiple timezones at once (e.g., show current time in NYC, London, Tokyo)"
4. Synthesize: "Reflect on your AI partnership: How did AI help you learn vs just complete the project?"

---

## Implementation Notes *(for content-implementer subagent)*

**Pedagogical Priorities**:
1. **Python 3.14 Native**: Teach `date.strptime()` and `time.strptime()` as PRIMARY methods (not alternatives). Never mention deprecated `utcnow()`.
2. **AI-Native Learning Throughout**: Apply 💬🎓🚀✨ elements THROUGHOUT lessons (not just "Try With AI" at end). Conversational tone, exploration-focused.
3. **Graduated Teaching Pattern**: Book teaches foundational (epoch, what is strptime) → AI companion handles complex (timezone DST edge cases) → AI orchestration at scale (batch timezone conversions).
4. **Cognitive Load Respect**: Max 7 concepts per lesson (A2-B1 intermediate tier). Progressive complexity: A2 → A2-B1 → B1.
5. **Lesson Closure**: ALL lessons end with "Try With AI" ONLY (4 prompts, progressive Bloom's). NO summaries, checklists, or "what's next" after.

**Code Quality Standards**:
- Python 3.14+ syntax with modern type hints (`date`, `datetime`, `float`, `int`, `str`)
- f-strings only (no `.format()` or `%`)
- `X | None` instead of `Optional[X]`
- All datetime code uses modern approaches (no deprecated methods)
- Validate user input before parsing dates or performing math operations
- No hardcoded secrets or credentials

**Context Materials (Ruthlessly Filtered)**:
- ✅ USE: Math functions, datetime objects, epoch, calendar, Python 3.14 new features
- ❌ SKIP: File I/O (Ch 22), OOP (Ch 24+), Advanced error handling (Ch 21 basics only), Future chapters

**Validation Checkpoints**:
- validation-auditor MUST verify: AI CoLearning elements throughout, lesson closure pattern, Python 3.14 native approach, no SDD terminology
- Automated testing MUST verify: All code runs on Python 3.14+, no deprecated methods, type hints present
- Skills-proficiency-mapper MUST validate: A2-B1 progression smooth, cognitive load within limits, CEFR levels appropriate

**Success Metrics (from SC section)**:
- 85%+ capstone completion rate
- 80%+ demonstrate asking AI for concepts (not just code)
- 90%+ can explain WHEN to use methods (not just HOW)
- 100% compliance with AI CoLearning elements and lesson closure pattern

---

## Dependencies and Assumptions

**Dependencies**:
- **Chapter 14 (Data Types)**: Students know `int`, `float`, `str`, type hints basics
- **Chapter 16 (Strings)**: Students can work with string formatting, understand f-strings
- **Chapter 21 (Exception Handling)**: Students can use try/except for error handling
- **Python 3.14+**: All students must have Python 3.14 or later installed (new `strptime()` methods required)
- **Terminal with color support**: For observing calendar color highlighting feature (works in most modern terminals)

**Assumptions**:
- Students have Claude Code or Gemini CLI configured for AI-Native Learning exercises
- Students are comfortable asking AI questions and iterating based on responses
- Students can run Python code in terminal/IDE and observe output
- Students are intermediate learners (A2-B1 CEFR) who have completed Chapters 1-22
- No prior knowledge of advanced math concepts (calculus, complex numbers) required
- Basic understanding of timezones from daily life (EST, PST, UTC) but not technical details

**Non-Goals** (out of scope):
- Teaching advanced timezone libraries (pytz, dateutil) - use built-in `timezone` class only
- Comprehensive math theory (calculus, linear algebra) - focus on practical functions
- Database datetime storage - that's covered in later chapters (Part 12)
- Async datetime operations - that's Chapter 28 (Asyncio)
- Full-featured calendar applications - capstone is educational, not production-ready

---

## Acceptance Criteria Checklist

**Content Completeness**:
- [ ] 5 foundational lessons + 1 capstone lesson structure defined
- [ ] All lessons respect A2-B1 cognitive load limits (max 7 concepts)
- [ ] Python 3.14 features integrated as PRIMARY teaching (not alternatives)
- [ ] 15-20 code examples total with clear pedagogical purpose
- [ ] Time Zone Converter capstone requirements fully specified
- [ ] All prerequisites explicitly listed (Ch 14, 16, 21)
- [ ] No forward references to future chapters (24+, 30+)

**AI-Native Learning Compliance**:
- [ ] Graduated Teaching Pattern applied (Book → AI Companion → AI Orchestration)
- [ ] AI CoLearning elements (💬🎓🚀✨) specified for ALL lessons throughout
- [ ] "Try With AI" format with 4 progressive prompts for ALL lessons
- [ ] Lesson closure pattern: "Try With AI" ONLY (no summaries after)
- [ ] Conversational tone specified (you, your, we - not documentation style)
- [ ] Exploration-focused language (discover, explore, try - not directive)

**Python 3.14 Native Approach**:
- [ ] `date.strptime()` taught as PRIMARY parsing method (NEW in 3.14)
- [ ] `time.strptime()` taught as PRIMARY parsing method (NEW in 3.14)
- [ ] `datetime.now(timezone.utc)` used (no deprecated `utcnow()`)
- [ ] Enhanced math error messages mentioned (Python 3.14 feature)
- [ ] Calendar color highlighting mentioned (Python 3.14 feature)
- [ ] NO backward compatibility content (students use 3.14+)

**Code Quality Standards**:
- [ ] Modern type hints specified (`date`, `datetime`, `float`, `int`, `str`, `X | None`)
- [ ] f-strings only (no `.format()` or `%`)
- [ ] Input validation before parsing/calculations specified
- [ ] No security issues (hardcoded secrets, unsafe practices)
- [ ] All code runnable on Python 3.14+ specified

**Pedagogical Quality**:
- [ ] User stories prioritized (P1, P2, P3) with independent testability
- [ ] Acceptance scenarios defined for all 6 user stories
- [ ] Edge cases identified (DST, leap years, domain errors, invalid input)
- [ ] Success criteria are measurable and technology-agnostic
- [ ] Lesson progression A2 → A2-B1 → B1 defined
- [ ] Capstone integrates all chapter concepts

**Validation Readiness**:
- [ ] Specification ready for `/sp.clarify` (no [NEEDS CLARIFICATION] markers remaining)
- [ ] Specification ready for `/sp.plan` (clear lesson structure for chapter-planner)
- [ ] Specification ready for validation-auditor (acceptance criteria measurable)

---

**Status**: ✅ Specification complete and ready for clarification phase (`/sp.clarify`)
