---
title: "Date and Time Objects (Python 3.14)"
chapter: 23
lesson: 3
sidebar_position: 3
duration_minutes: 120

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Date/Time Object Creation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create date, time, and datetime objects with appropriate parameters and type hints"

  - name: "Python 3.14 String Parsing (NEW)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can parse user-provided date/time strings using Python 3.14's new date.strptime() and time.strptime() methods and handle parsing errors"

  - name: "Timezone Awareness Basics"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain difference between naive and timezone-aware datetime objects and create timezone-aware objects with datetime.now(timezone.utc)"

learning_objectives:
  - objective: "Create date, time, and datetime objects using constructor syntax with type hints"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Code examples creating various datetime objects"

  - objective: "Parse user-provided date and time strings using Python 3.14's new strptime() methods with validation"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Real-world parsing with error handling"

  - objective: "Explain the difference between naive and timezone-aware datetime objects and when each is appropriate"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation and practical object creation"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (date, time, datetime, date.strptime, time.strptime, datetime.now, timezone-aware) at A2-B1 max limit ✓"

differentiation:
  extension_for_advanced: "Explore DST edge cases with AI; research historical date changes (Julian vs Gregorian calendar); investigate leap second handling"
  remedial_for_struggling: "Focus on object creation first; use simple date strings (YYYY-MM-DD format); ask AI for format code explanations before complex parsing"

# Generation metadata
generated_by: "lesson-writer v3.0.2"
source_spec: "specs/001-part-4-chapter-23/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Date and Time Objects (Python 3.14)

When you build real applications—event scheduling, logging, analytics, user interfaces—you need to work with dates and times. Python's `datetime` module provides high-level objects that make this manageable. And Python 3.14 introduces NEW parsing methods that simplify a common task: converting user-provided date strings into objects your code can work with.

In this lesson, you'll learn to create date, time, and datetime objects, parse user input using Python 3.14's new methods, and understand the basics of timezone-aware objects. You'll discover that working with dates is less about memorizing format codes and more about understanding WHEN to parse (from user input) versus construct (programmatic dates).

## Understanding datetime Objects: Beyond Timestamps

In the previous lesson, you learned about timestamps (seconds since epoch) and the `time` module. The `datetime` module takes a different approach—instead of storing everything as a number, it provides **objects** that represent dates and times as structured data.

Think of it like this: A timestamp (`1699564800.12345`) is efficient for comparison and calculation, but hard for humans to read. A `datetime` object (`2025-11-09 14:30:00`) is structured—it knows about years, months, days, hours separately.

#### 💬 AI Colearning Prompt

> "Why does Python have both the `time` module (timestamps) and the `datetime` module (objects)? What's the use case for each?"

Python gives you both because they solve different problems. Use `datetime` for working with human-readable dates and times. Use `time` module when you need low-level timing or epoch calculations. This lesson focuses on `datetime`—the higher-level abstraction you'll use most often.

## Creating Date Objects

A **date** object represents a specific calendar date: year, month, and day.

```python
from datetime import date

# Create a date object
my_birthday: date = date(1999, 3, 15)
print(my_birthday)  # Output: 1999-03-15
print(f"Day: {my_birthday.day}, Month: {my_birthday.month}, Year: {my_birthday.year}")
```

**What's happening**: The `date` constructor takes three parameters: `year`, `month` (1-12), and `day` (1-31, depending on the month). The type hint `date` tells Python (and anyone reading your code) that the variable holds a date object.

#### 🎓 Instructor Commentary

> In AI-native development, you don't memorize date constructor parameters—you type `date(` in your IDE and let AI autocomplete show you the parameters. Your job: understand that a date object separates year/month/day into distinct attributes.

Let's verify this works with different dates:

```python
from datetime import date

# Different dates with type hints
new_years: date = date(2025, 1, 1)
today_example: date = date(2025, 11, 9)

print(f"New Year 2025: {new_years}")
print(f"Today's example: {today_example}")
print(f"Today is in month {today_example.month}")  # Extract just the month
```

**Output**:
```
New Year 2025: 2025-01-01
Today's example: 2025-11-09
Today is in month 11
```

Notice: Python stores dates in ISO 8601 format (`YYYY-MM-DD`), which is the international standard for date representation—perfect for sorting, comparison, and database storage.

## Creating Time Objects

A **time** object represents a time of day: hour, minute, second, and microsecond (millionths of a second).

```python
from datetime import time

# Create a time object
meeting_time: time = time(14, 30, 0)  # 2:30 PM
print(meeting_time)  # Output: 14:30:00

# With microseconds (precision)
precise_time: time = time(14, 30, 45, 123456)  # 14:30:45.123456
print(precise_time)
```

**What's happening**: The `time` constructor takes: `hour` (0-23, military time), `minute` (0-59), `second` (0-59), and optional `microsecond` (0-999999).

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Generate a function that creates a time object from a string like '2:30 PM' and explain why microsecond precision matters in scientific computing."

**Expected Outcome**: You'll understand why Python uses 24-hour format and why microseconds exist for high-precision timing.

Let's practice:

```python
from datetime import time

# Common times
morning: time = time(9, 0, 0)      # 9:00 AM
noon: time = time(12, 0, 0)        # Noon
afternoon: time = time(14, 30, 0)  # 2:30 PM
night: time = time(23, 59, 59)     # 11:59:59 PM

print(f"Morning: {morning}")
print(f"Afternoon: {afternoon}")
print(f"Night: {night}")
```

## Creating Datetime Objects

A **datetime** object combines both—it represents a complete timestamp with both date and time.

```python
from datetime import datetime

# Create a datetime object
event: datetime = datetime(2025, 11, 9, 14, 30, 0)
print(event)  # Output: 2025-11-09 14:30:00

# Access date and time components
print(f"Date: {event.date()}")  # 2025-11-09
print(f"Time: {event.time()}")  # 14:30:00
print(f"Year: {event.year}")    # 2025
print(f"Hour: {event.hour}")    # 14
```

**What's happening**: `datetime` takes all the parameters of `date` (year, month, day) plus all the parameters of `time` (hour, minute, second, microsecond).

#### ✨ Teaching Tip

> Use Claude Code to explore what happens when you pass invalid values: "What error do I get if I create date(2025, 2, 30)? Why does February 30th not exist?"

Let's see `datetime` in context:

```python
from datetime import datetime

# Schedule a conference call
conference_call: datetime = datetime(2025, 12, 15, 10, 0, 0)
print(f"Conference call: {conference_call}")  # 2025-12-15 10:00:00

# A meeting with seconds precision
standup: datetime = datetime(2025, 11, 9, 9, 30, 0)
print(f"Daily standup: {standup}")
```

Now you can represent a complete moment in time—not just a date or a time, but both together in a single structured object.

## Python 3.14 NEW: Parsing Dates with `date.strptime()`

Here's where Python 3.14 introduces something powerful: Before this version, parsing a date string (like "2025-11-09" from a web form) required workarounds. Now you have a direct method: **`date.strptime()`**.

The name `strptime` means "string parse time"—it converts a string INTO a date object.

```python
from datetime import date

# User enters: "2025-11-09"
date_string: str = "2025-11-09"
parsed_date: date = date.strptime(date_string, "%Y-%m-%d")
print(parsed_date)  # Output: 2025-11-09
print(type(parsed_date))  # <class 'datetime.date'>
```

**What's happening**:
- `date.strptime(string, format)` takes two arguments
- `string`: The user-provided text ("2025-11-09")
- `format`: A pattern describing the string's structure ("%Y-%m-%d" means year-month-day with hyphens)

The format codes are standard:
- `%Y`: 4-digit year (2025)
- `%m`: 2-digit month (01-12)
- `%d`: 2-digit day (01-31)

#### 💬 AI Colearning Prompt

> "Python 3.14 added `date.strptime()` and `time.strptime()` as class methods. How do these improve on previous approaches? What was the old way of parsing dates?"

Before Python 3.14, you had to create a `datetime` object and extract the date—now you parse directly into a `date` object. This is simpler and faster.

Let's practice with different date formats:

```python
from datetime import date

# ISO 8601 format (international standard)
iso_date: date = date.strptime("2025-11-09", "%Y-%m-%d")

# US format (November 9, 2025)
us_date: date = date.strptime("11/09/2025", "%m/%d/%Y")

# European format (9 November 2025)
eu_date: date = date.strptime("09-11-2025", "%d-%m-%Y")

# Day month year
dmy_date: date = date.strptime("09/11/2025", "%d/%m/%Y")

print(f"ISO: {iso_date}")
print(f"US: {us_date}")
print(f"EU: {eu_date}")
print(f"DMY: {dmy_date}")
```

**Output**:
```
ISO: 2025-11-09
US: 2025-11-09
EU: 2025-11-09
DMY: 2025-11-09
```

All parse to the same date object because Python internally stores dates the same way—the format string just tells the parser how to interpret the input.

**CRITICAL FOR USERS**: When you receive a date string, you need to know its format. If the user enters "11/09/2025", is that November 9 (US) or September 11 (EU)? Always clarify the expected format in your application!

#### 🎓 Instructor Commentary

> Before Python 3.14, parsing dates required workarounds. Now you parse directly—syntax is cheap, knowing WHEN to parse (user input) vs construct (programmatic dates) is gold. Ask AI when you're unsure about format codes; you don't memorize all 30+.

## Python 3.14 NEW: Parsing Times with `time.strptime()`

Similarly, Python 3.14 adds **`time.strptime()`** for parsing time strings directly into time objects.

```python
from datetime import time

# User enters: "14:30:45"
time_string: str = "14:30:45"
parsed_time: time = time.strptime(time_string, "%H:%M:%S")
print(parsed_time)  # Output: 14:30:45
print(type(parsed_time))  # <class 'datetime.time'>
```

**Format codes for time**:
- `%H`: Hour (00-23, 24-hour format)
- `%M`: Minute (00-59)
- `%S`: Second (00-59)

Let's parse different time formats:

```python
from datetime import time

# Standard 24-hour format
military_time: time = time.strptime("14:30:00", "%H:%M:%S")

# 12-hour format with AM/PM
twelve_hour: time = time.strptime("02:30:45 PM", "%I:%M:%S %p")

# Without seconds
short_time: time = time.strptime("14:30", "%H:%M")

print(f"Military: {military_time}")
print(f"12-hour: {twelve_hour}")
print(f"Short: {short_time}")
```

**Output**:
```
Military: 14:30:00
12-hour: 14:30:45
Short: 14:30:00
```

Notice that `time` objects always display in 24-hour format internally, even if the user entered "PM".

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Generate a function that validates a user's time input by trying to parse it with `time.strptime()` and returning a helpful error message if the format is wrong. What format should you expect from users?"

**Expected Outcome**: Understanding parsing errors and error handling for user input.

## Getting the Current Datetime

You often need to capture "right now"—the current date and time. The `datetime.now()` method does this.

```python
from datetime import datetime

# Get current date and time
current: datetime = datetime.now()
print(current)  # Output: 2025-11-09 14:35:22.123456
```

**What's happening**: `datetime.now()` returns a `datetime` object representing this exact moment (down to microseconds).

This is crucial for timestamps in logging, user actions, and event tracking:

```python
from datetime import datetime

# Log when a user logs in
login_time: datetime = datetime.now()
print(f"User logged in at {login_time}")  # 2025-11-09 14:35:22.123456

# Calculate how long an operation took
start: datetime = datetime.now()
# ... do some work ...
end: datetime = datetime.now()
duration = end - start  # timedelta object (we'll cover this in the next lesson)
print(f"Operation took {duration}")
```

#### ✨ Teaching Tip

> Always use `datetime.now()` in your code. NEVER use the deprecated `datetime.utcnow()` method—ask your AI why: "Why is utcnow() deprecated? What's the modern approach?"

## Understanding Timezone Awareness

Here's a critical concept that trips up many developers: **naive vs timezone-aware datetime objects**.

A **naive** datetime doesn't know what timezone it's in:

```python
from datetime import datetime

naive: datetime = datetime.now()
print(naive)  # 2025-11-09 14:30:00
print(naive.tzinfo)  # None - it doesn't know the timezone
```

An **aware** datetime knows its timezone:

```python
from datetime import datetime, timezone

# Create timezone-aware datetime (UTC)
aware: datetime = datetime.now(timezone.utc)
print(aware)  # 2025-11-09 19:30:00+00:00
print(aware.tzinfo)  # UTC (or +00:00)
```

**Why does this matter?**

Imagine your application runs in three offices: New York, London, and Tokyo. If you store a naive datetime "14:30:00", which office is that? 2:30 PM where?

When building applications that span timezones (or might in the future), **always use timezone-aware datetimes**.

```python
from datetime import datetime, timezone

# Store the moment in UTC (the international standard)
moment_utc: datetime = datetime.now(timezone.utc)
print(moment_utc)  # 2025-11-09 19:30:00+00:00

# The +00:00 tells you this is UTC (zero offset from UTC)
# Later, we'll convert this to other timezones as needed
```

#### 💬 AI Colearning Prompt

> "What's the difference between naive and timezone-aware datetime objects? Why do you hear 'always use UTC for storage' as a best practice?"

Here's the principle: Store all times in UTC (a universal reference), then convert to local timezones only when displaying to users.

```python
from datetime import datetime, timezone

# Imagine an international conference call
# It happens at UTC 19:30 on Nov 9, 2025
conference: datetime = datetime(2025, 11, 9, 19, 30, 0, tzinfo=timezone.utc)
print(f"Conference UTC: {conference}")  # 2025-11-09 19:30:00+00:00

# In New York (EST, UTC-5), it's 2:30 PM
# In Tokyo (JST, UTC+9), it's 4:30 AM the next day
# We'll show how to convert in the next lesson
```

Notice the `tzinfo=timezone.utc` parameter—that's how you mark a datetime as timezone-aware.

## Example: Complete Date and Time Parsing Workflow

Let's bring everything together with a realistic example:

**Specification Reference**: User enters birthday as "1999-03-15", and we need to store it as a date object.

**AI Prompt Used**: "Generate a function that parses a date string in YYYY-MM-DD format and returns a date object with error handling."

```python
from datetime import date

def parse_birthday(birthday_string: str) -> date | None:
    """
    Parse a birthday string in YYYY-MM-DD format.

    Args:
        birthday_string: Date string like "1999-03-15"

    Returns:
        A date object if valid, None if invalid
    """
    try:
        return date.strptime(birthday_string, "%Y-%m-%d")
    except ValueError as e:
        print(f"Invalid date format: {e}")
        return None

# Test the function
result: date | None = parse_birthday("1999-03-15")
if result:
    print(f"Birthday parsed: {result}")
    print(f"Birth year: {result.year}")
else:
    print("Failed to parse birthday")

# Test with invalid input
invalid: date | None = parse_birthday("not-a-date")
# Output: Invalid date format: time data 'not-a-date' does not match format '%Y-%m-%d'
```

**Validation steps**:
1. ✅ Function accepts string input
2. ✅ Uses `date.strptime()` (Python 3.14 new method)
3. ✅ Returns `date | None` (handles both success and failure)
4. ✅ Catches `ValueError` when parsing fails
5. ✅ Includes type hints throughout
6. ✅ Tested with valid and invalid input

#### 🎓 Instructor Commentary

> Notice the error handling: When the user enters "not-a-date", Python raises `ValueError` with a descriptive message. Your job isn't to memorize error messages—it's to catch them and provide helpful feedback to the user. AI helps you understand what went wrong.

## Try With AI

Use **Claude Code** (or your preferred AI companion) to explore datetime parsing and object creation with these progressive prompts.

**Prompt 1 (Recall)**: "What's new in Python 3.14 for date and time parsing? Explain the difference between date.strptime() and the old approach."

**Expected outcome**: You understand Python 3.14's new methods and why they're improvements.

**Prompt 2 (Understand)**: "Explain the difference between naive and timezone-aware datetime objects. When would you use each one?"

**Expected outcome**: You can articulate when timezone awareness matters and when you can skip it.

**Prompt 3 (Apply)**: "Generate a Python function that takes a user's birth date string (YYYY-MM-DD format) and calculates how many days old they are today. Include type hints and error handling."

**Expected outcome**: You understand parsing user input, date arithmetic (coming in the next lesson), and validation.

**Prompt 4 (Analyze)**: "Why is timezone awareness important for global applications? When building a scheduling app used in NYC, London, and Tokyo, what could go wrong if you use naive datetimes?"

**Expected outcome**: You recognize that timezone handling is a real-world concern and understand the principle of storing UTC + converting on display.

**Safety Note**: When parsing user input, always validate the format. Never assume users will enter dates correctly—use try/except to catch and report errors gracefully. Ask your AI: "How do I provide helpful error messages when date parsing fails?"
