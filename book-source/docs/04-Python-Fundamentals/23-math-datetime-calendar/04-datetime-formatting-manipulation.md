---
title: "Date/Time Formatting and Manipulation"
chapter: 23
lesson: 4
sidebar_position: 4
duration_minutes: 120

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "DateTime Formatting with strftime()"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can format datetime objects for different contexts (ISO 8601, localized, custom) using strftime() without memorizing all 30+ format codes"

  - name: "Date Arithmetic with Timedelta"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can add/subtract timedelta to/from datetime, calculate durations between two dates, and convert durations to different units (days, hours, minutes)"

  - name: "Timezone Conversion"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can convert datetime objects between timezones (UTC to local, local to local) using timezone and timedelta objects"

learning_objectives:
  - objective: "Format datetime objects for display using strftime() in multiple contexts (ISO 8601, localized, custom)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Create formatted output matching specified formats"

  - objective: "Perform date arithmetic using timedelta objects to add/subtract time intervals and calculate durations"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Solve real-world date calculation problems"

  - objective: "Convert between timezones using Python's timezone objects and understand timezone edge cases"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Convert times between multiple timezones correctly"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (strftime codes, format contexts, timedelta, arithmetic, timezone conversion, handling complexity) within B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore DST transitions programmatically; build timezone-aware scheduling systems; handle ambiguous times during DST changes"
  remedial_for_struggling: "Focus on ISO 8601 format first; use timedelta for simple day arithmetic; default to UTC timezone before exploring local conversions"

# Generation metadata
generated_by: "content-implementer v3.0.2"
source_spec: "specs/001-part-4-chapter-23/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Date/Time Formatting and Manipulation

You've built datetime objects from user input and understand the basics of timezone awareness. Now you'll learn how to **format** those objects for display and **manipulate** them to solve real-world scheduling problems. By the end of this lesson, you'll format dates for different audiences, calculate durations, and convert times across timezones—the exact skills you need for building global applications.

## Why Formatting Matters

Think about your favorite app. When it shows you an event time, it doesn't say "2025-11-09T14:30:00+00:00" (that's for computers). It says something like "Sunday, November 9 at 2:30 PM" or "2025-11-09" depending on context. The raw datetime object is useless to humans without **formatting**.

Similarly, when you're building a scheduling feature or calculating project deadlines, you need to manipulate dates—add days, subtract hours, find the difference between two moments. That's where **timedelta** comes in.

And when your app serves users around the world, you need to **convert** between timezones correctly. A meeting at 9 AM UTC is 4 AM EST—not handling that properly creates chaos.

This lesson teaches you all three skills, emphasizing that you don't memorize all 30+ format codes. You understand the pattern and ask AI when needed.

## Formatting Dates and Times with strftime()

Strings. That's how humans read dates. The `strftime()` method converts datetime objects into human-readable text using format codes.

### Understanding Format Codes

Each format code starts with `%` and represents a specific part of the date:

```python
from datetime import datetime, timezone

# Create a specific datetime (November 9, 2025 at 2:30:45 PM UTC)
moment: datetime = datetime(2025, 11, 9, 14, 30, 45, tzinfo=timezone.utc)

# Common format codes
print(moment.strftime("%Y-%m-%d"))  # 2025-11-09 (year-month-day)
print(moment.strftime("%H:%M:%S"))  # 14:30:45 (hours:minutes:seconds)
print(moment.strftime("%A, %B %d")) # Sunday, November 09 (weekday, full month, day)
```

The key insight: **You don't memorize all 30+ codes.** You learn the common ones (`%Y`, `%m`, `%d`, `%H`, `%M`, `%S`, `%A`, `%B`) and ask AI when you need something specific.

#### 💬 AI Colearning Prompt

> "Explain the difference between `%Y` and `%y`, and between `%H` and `%I`. When would you use each?"

### Common Format Patterns

Here are the patterns you'll use most often:

```python
from datetime import datetime, timezone

moment: datetime = datetime(2025, 11, 9, 14, 30, 45, tzinfo=timezone.utc)

# ISO 8601 (standard for data storage and APIs)
iso_format: str = moment.strftime("%Y-%m-%dT%H:%M:%S%z")
print(iso_format)  # 2025-11-09T14:30:45+0000

# US Format (casual, for display)
us_format: str = moment.strftime("%m/%d/%Y %I:%M %p")
print(us_format)  # 11/09/2025 02:30 PM

# European Format
eu_format: str = moment.strftime("%d/%m/%Y %H:%M")
print(eu_format)  # 09/11/2025 14:30

# Friendly Format (for humans)
friendly: str = moment.strftime("%A, %B %d, %Y at %I:%M %p")
print(friendly)  # Sunday, November 09, 2025 at 02:30 PM
```

Notice how the same `moment` looks different depending on the format. Your job isn't to remember these strings—it's to choose the right format for your audience.

#### 🎓 Expert Insight

> In AI-native development, you don't memorize all 30+ format codes. You understand the pattern (`%Y` = year, `%m` = month, `%d` = day) and ask AI when you need a specific format. **Syntax is cheap; knowing WHEN to use ISO 8601 vs localized format is gold.** ISO for data storage and APIs. Friendly format for user interfaces.

### Practical Formatting Exercise

Let's build a function that formats a datetime in multiple ways:

```python
from datetime import datetime, timezone

def format_datetime_multiple_ways(dt: datetime) -> dict[str, str]:
    """
    Format a datetime object in multiple contexts.

    Args:
        dt: A datetime object (timezone-aware preferred)

    Returns:
        Dictionary with formatted strings for different contexts
    """
    return {
        "iso": dt.strftime("%Y-%m-%dT%H:%M:%S"),
        "us": dt.strftime("%m/%d/%Y"),
        "friendly": dt.strftime("%A, %B %d, %Y"),
        "time_only": dt.strftime("%I:%M %p"),
        "timestamp": dt.strftime("%s"),  # Unix timestamp as string
    }

# Test it
now: datetime = datetime.now(timezone.utc)
formats = format_datetime_multiple_ways(now)

for context, formatted in formats.items():
    print(f"{context}: {formatted}")
```

This function shows the real-world pattern: capture different formats once, use them throughout your application.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Generate code that formats a datetime in both ISO 8601 and 'Meeting on Saturday, 2:30 PM' format. Then explain what each format code does."

**Expected Outcome**: You'll understand how to combine format codes to create human-friendly output.

## Working with Time Differences: Timedelta

A **timedelta** represents a duration—the difference between two points in time. Unlike datetime (which represents a specific moment), timedelta represents "how much time."

### Creating Timedelta Objects

```python
from datetime import timedelta

# Create durations
one_day: timedelta = timedelta(days=1)
three_hours: timedelta = timedelta(hours=3)
two_weeks: timedelta = timedelta(weeks=2)

# Combine multiple units
complex_duration: timedelta = timedelta(
    days=5,
    hours=3,
    minutes=30,
    seconds=45
)

print(complex_duration)  # 5 days, 3:30:45
print(complex_duration.total_seconds())  # 460245.0
```

Timedelta objects know how to convert between units automatically.

### Date Arithmetic: Adding and Subtracting

Now here's where timedelta shines. Add it to a datetime to get a future date:

```python
from datetime import datetime, timedelta, timezone

# Start with today
today: datetime = datetime.now(timezone.utc)

# Calculate important dates
tomorrow: datetime = today + timedelta(days=1)
next_week: datetime = today + timedelta(weeks=1)
next_year: datetime = today + timedelta(days=365)

# Go backwards
yesterday: datetime = today - timedelta(days=1)
last_month: datetime = today - timedelta(days=30)

print(f"Today: {today.strftime('%Y-%m-%d')}")
print(f"Tomorrow: {tomorrow.strftime('%Y-%m-%d')}")
print(f"In 30 days: {next_month.strftime('%Y-%m-%d')}")
```

Timedelta handles all the complexity for you—leap years, different month lengths, everything.

#### ✨ Teaching Tip

> Use Claude Code to explore edge cases: "Add 30 days to January 15. Then check: does it land on Feb 14? Why not 45 days?" This teaches you timedelta respects calendar reality.

### Calculating Duration Between Two Moments

One of the most practical uses: find how much time has passed or remains:

```python
from datetime import datetime, timezone

# Two specific moments
launch_time: datetime = datetime(2025, 11, 9, 14, 30, tzinfo=timezone.utc)
current_time: datetime = datetime.now(timezone.utc)

# Calculate the difference
elapsed: timedelta = current_time - launch_time

print(f"Days elapsed: {elapsed.days}")
print(f"Total seconds: {elapsed.total_seconds()}")
print(f"Hours: {elapsed.total_seconds() / 3600:.1f}")

# Practical use: countdown
deadline: datetime = datetime(2025, 12, 31, 23, 59, tzinfo=timezone.utc)
remaining: timedelta = deadline - current_time

if remaining.total_seconds() > 0:
    print(f"Time until deadline: {remaining.days} days, {remaining.seconds // 3600} hours")
else:
    print("Deadline passed!")
```

Notice we calculate remaining and check if it's positive. This is the pattern for deadlines, event scheduling, and countdowns.

#### 🎓 Expert Insight

> You're not calculating duration by hand (that's error-prone and wasteful). You subtract two datetime objects and timedelta does the work. **Syntax is cheap; knowing to subtract datetime objects and handle the result is gold.** This is why we use timedelta.

### Code Example: Duration Calculation with Validation

```python
from datetime import datetime, timedelta, timezone

def time_until_event(event_time: datetime) -> str:
    """
    Calculate human-readable time remaining until an event.

    Args:
        event_time: A future datetime object

    Returns:
        Formatted string like "2 days, 3 hours"
    """
    now: datetime = datetime.now(timezone.utc)
    remaining: timedelta = event_time - now

    if remaining.total_seconds() <= 0:
        return "Event already happened!"

    days = remaining.days
    hours = remaining.seconds // 3600
    minutes = (remaining.seconds % 3600) // 60

    # Build friendly string
    parts: list[str] = []
    if days > 0:
        parts.append(f"{days} day{'s' if days != 1 else ''}")
    if hours > 0:
        parts.append(f"{hours} hour{'s' if hours != 1 else ''}")
    if minutes > 0:
        parts.append(f"{minutes} minute{'s' if minutes != 1 else ''}")

    return ", ".join(parts) if parts else "Less than a minute"

# Test it
conference: datetime = datetime(2025, 12, 15, 9, 0, tzinfo=timezone.utc)
print(time_until_event(conference))  # Might print "36 days, 5 hours"
```

This function shows a real pattern: calculate timedelta, extract components, format for humans.

## Converting Between Timezones

Timezones are your biggest challenge in datetime work. A time can't be "correct" or "incorrect" without a timezone—you need to know: "4 PM what? UTC? EST? JST?"

### The Timezone Object

Python represents timezone offsets with the `timezone` class:

```python
from datetime import datetime, timezone, timedelta

# UTC (the reference point)
utc_now: datetime = datetime.now(timezone.utc)
print(utc_now)  # Shows timezone info: ...+00:00

# Other timezones are offsets from UTC
eastern: timezone = timezone(timedelta(hours=-5))  # EST (UTC-5)
pacific: timezone = timezone(timedelta(hours=-8))  # PST (UTC-8)

# Create a specific time in a timezone
meeting_time: datetime = datetime(2025, 11, 9, 14, 30, tzinfo=eastern)
print(meeting_time)  # Shows: 2025-11-09 14:30:00-05:00
```

The `:−5` (negative 5) means "5 hours behind UTC." When it's noon UTC, it's 7 AM Eastern.

#### 💬 AI Colearning Prompt

> "Explain why timezone offsets are negative for Western Hemisphere and positive for Eastern Hemisphere."

### Converting UTC to Local Time

Here's the practical scenario: you have a UTC timestamp (from a database), and you need to show the user their local time.

```python
from datetime import datetime, timezone, timedelta

# A meeting time in UTC
meeting_utc: datetime = datetime(2025, 11, 9, 14, 30, tzinfo=timezone.utc)

# Convert to different timezones
eastern: timezone = timezone(timedelta(hours=-5))
pacific: timezone = timezone(timedelta(hours=-8))
london: timezone = timezone(timedelta(hours=0))  # UTC
tokyo: timezone = timezone(timedelta(hours=9))

# Use astimezone() to convert
meeting_eastern = meeting_utc.astimezone(eastern)
meeting_pacific = meeting_utc.astimezone(pacific)
meeting_tokyo = meeting_utc.astimezone(tokyo)

print(f"Meeting UTC: {meeting_utc.strftime('%H:%M')}")
print(f"Meeting Eastern: {meeting_eastern.strftime('%H:%M')}")  # 09:30
print(f"Meeting Pacific: {meeting_pacific.strftime('%H:%M')}")  # 06:30
print(f"Meeting Tokyo: {meeting_tokyo.strftime('%H:%M')}")     # 23:30
```

The magic: `astimezone()` adjusts both the time AND the offset automatically.

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Generate a function that takes a UTC timestamp and returns a dictionary with the time in NYC, London, and Tokyo timezones. Then explain what astimezone() does under the hood."

**Expected Outcome**: You'll understand how to display times for a global audience.

### Real-World Timezone Conversion: Meeting Scheduler

```python
from datetime import datetime, timezone, timedelta

class MeetingScheduler:
    """Helper for scheduling meetings across timezones."""

    def __init__(self, meeting_utc: datetime):
        """
        Args:
            meeting_utc: Meeting time in UTC (timezone-aware)
        """
        if meeting_utc.tzinfo is None:
            raise ValueError("Must provide timezone-aware datetime")
        self.meeting_utc = meeting_utc.astimezone(timezone.utc)

    def time_in(self, timezone_obj: timezone) -> str:
        """Show meeting time in a specific timezone."""
        local = self.meeting_utc.astimezone(timezone_obj)
        return local.strftime("%H:%M (%Z)")

    def create_agenda(self) -> str:
        """Generate meeting agenda with multiple timezones."""
        timezones = {
            "NYC": timezone(timedelta(hours=-5)),
            "London": timezone(timedelta(hours=0)),
            "Tokyo": timezone(timedelta(hours=9)),
        }

        lines = ["MEETING AGENDA\n"]
        for city, tz in timezones.items():
            local = self.meeting_utc.astimezone(tz)
            lines.append(f"{city}: {local.strftime('%A %I:%M %p')}")

        return "\n".join(lines)

# Use it
meeting = MeetingScheduler(datetime(2025, 11, 9, 14, 30, tzinfo=timezone.utc))
print(meeting.create_agenda())
```

Output:
```
MEETING AGENDA

NYC: Sunday 09:30 AM
London: Sunday 02:30 PM
Tokyo: Monday 11:30 PM
```

This is real production code. You're solving an actual problem: helping people understand when a meeting happens in their timezone.

#### 🎓 Expert Insight

> You don't calculate timezone offsets in your head (that's why timezones exist as objects). You understand: "UTC is the reference, offsets are +/- hours, astimezone() does the conversion." **Syntax is cheap; knowing to keep everything in UTC internally and convert on display is gold.** This pattern prevents bugs.

## Handling Timezone Edge Cases

Timezones aren't simple. During daylight saving time transitions, an hour doesn't exist (spring forward) or exists twice (fall back). For now, know this happens and use AI when you encounter it.

#### ✨ Teaching Tip

> When building a production scheduling system, ask your AI: "How do I handle times during DST transitions? When a time is ambiguous, which version should I pick?" This is where you lean on AI expertise—it's complex enough that asking beats guessing.

## Summary of Formatting and Manipulation

**Formatting** with `strftime()`: Convert datetime objects to human-readable strings using format codes. You know the common ones (`%Y`, `%m`, `%d`, `%H`, `%M`, `%S`, `%A`, `%B`) and ask AI for others.

**Manipulation** with `timedelta`: Represent durations, add/subtract from datetimes, calculate differences. Always subtract datetime objects rather than calculating durations manually.

**Conversion** with `timezone` and `astimezone()`: Keep your system in UTC internally, convert to local timezones for display. Never hardcode timezone offsets—use timezone objects.

---

## Try With AI: The Date Format Migration Challenge

You've learned `strftime()` for formatting and `timedelta` for manipulation. Now you'll migrate between different date formats, understand formatting edge cases, and build a multi-format date parser/generator.

### Challenge Overview

**The Date Format Migration**: You're building a system that converts dates between multiple formats—ISO 8601 for storage, US format for display, friendly format for user interfaces. This demonstrates why understanding format codes and timezone conversion is critical in real applications.

---

## Part 1: Format Ambiguity Analysis (Independent Exploration)

**Your Role**: Investigator discovering format ambiguity issues

### Part 1 Task: Create `format_ambiguity_analysis.md`

**Scenario 1: Date Format Confusion**
- You receive: "11/09/2025"
- Is this November 9 (US) or September 11 (EU)?
- What does ISO 8601 say? (YYYY-MM-DD)
- Try parsing with different format codes
- Question: How would you build a system that accepts multiple formats safely?

**Scenario 2: Timezone Offset in Output**
- Format same datetime three ways:
  - `strftime("%Y-%m-%d %H:%M:%S")` (no timezone)
  - `strftime("%Y-%m-%dT%H:%M:%S%z")` (with offset)
  - `strftime("%Y-%m-%dT%H:%M:%S%Z")` (with zone name)
- Compare outputs
- Question: Which format is best for APIs? Logs? User display?

**Scenario 3: Localization Challenges**
- Month name: `strftime("%B")` produces English
- What if users are in different locales?
- Question: How do you handle translating month names?

### Part 1 Deliverable

File: `format_ambiguity_analysis.md`

**Time**: 15-20 minutes

---

## Part 2: AI as Teacher

**Your Role**: Student learning format patterns

### Part 2 Prompts

**Prompt 1**: "I'm confused about date formats. When should I use ISO 8601 vs US format vs friendly format? Where does each format belong in an application?"

**Prompt 2**: "Show me how to convert between formats. If I have a datetime in US format (MM/DD/YYYY), how do I convert it to ISO 8601 and friendly format?"

**Prompt 3**: "Explain timedelta. I calculated end - start and got a timedelta. How do I extract days, hours, and minutes separately?"

### Part 2 Outcome

You understand:
- Format codes and their purposes
- Where each format is appropriate
- How to convert between formats

---

## Part 3: Student as Teacher (Edge Cases)

**Your Role**: Tester challenging format logic

### Part 3 Challenges

**Challenge 1: Ambiguous Format**
> "If I parse '01-02-2025', three formats all work: MM-DD-YYYY (Jan 2), DD-MM-YYYY (Feb 1), and YYYY-MM-DD fails. How do I prevent bugs where the wrong format is assumed?"

**Challenge 2: Timezone Offset Ambiguity**
> "The format code %z produces '+0500' but I want '+05:00'. Is there a code for that? How do I format timezone offsets correctly?"

**Challenge 3: Leap Second Handling**
> "If a timestamp represents a leap second (23:59:60), how does strftime handle it? What seconds value would be displayed?"

### Part 3 Deliverable

File: `format_challenges.md`

**Time**: 15-20 minutes

---

## Part 4: Build Multi-Format Date Tool (Convergence)

**Your Role**: Developer building format migration utilities

### Part 4 Deliverable: `date_formatter.py`

```python
from datetime import datetime, timezone, timedelta

class DateFormatter:
    """Format and parse dates across multiple contexts."""

    FORMATS = {
        "iso": "%Y-%m-%dT%H:%M:%S",
        "us": "%m/%d/%Y",
        "eu": "%d/%m/%Y",
        "friendly": "%A, %B %d, %Y",
        "iso_time": "%H:%M:%S",
        "us_time": "%I:%M %p",
        "log": "%Y-%m-%d %H:%M:%S",
    }

    @staticmethod
    def format_datetime(dt: datetime, format_name: str = "iso") -> str:
        """Format datetime using predefined format names.

        Args:
            dt: datetime to format
            format_name: Name of format ('iso', 'us', 'friendly', etc.)

        Returns:
            Formatted datetime string
        """
        if format_name not in DateFormatter.FORMATS:
            raise ValueError(f"Unknown format: {format_name}")
        return dt.strftime(DateFormatter.FORMATS[format_name])

    @staticmethod
    def format_multiple(dt: datetime) -> dict[str, str]:
        """Format datetime in all available formats.

        Args:
            dt: datetime to format

        Returns:
            Dictionary with all formats
        """
        return {
            name: dt.strftime(code)
            for name, code in DateFormatter.FORMATS.items()
        }

    @staticmethod
    def format_with_timezone(dt: datetime) -> str:
        """Format datetime with timezone information.

        Args:
            dt: timezone-aware datetime

        Returns:
            Formatted string with offset
        """
        base = dt.strftime("%Y-%m-%dT%H:%M:%S")
        if dt.tzinfo:
            offset = dt.strftime("%z")
            return f"{base}{offset[:3]}:{offset[3:]}"
        return base

    @staticmethod
    def format_duration(td: timedelta) -> str:
        """Format timedelta as human-readable.

        Args:
            td: timedelta object

        Returns:
            Formatted string
        """
        total_seconds = int(td.total_seconds())
        hours = total_seconds // 3600
        minutes = (total_seconds % 3600) // 60
        return f"{hours}h {minutes}m"
```

### Part 4 Requirements

Your tool must:
1. Define common format patterns
2. Format datetime in multiple contexts
3. Handle timezone display
4. Format durations
5. Include type hints and docstrings

### Part 4 Deliverable

File: `date_formatter.py`

**Time**: 25-35 minutes

---

## Integrated Learning Outcomes

You've practiced all Three Roles and synthesized format knowledge into reusable tools.

### What You've Created

1. `format_ambiguity_analysis.md` — Your investigation of format issues
2. `format_challenges.md` — Edge cases you challenged AI with
3. `date_formatter.py` — Working format utilities

---

**Next**: Lesson 5 covers calendar operations and advanced math, bringing together everything you've learned about time, precision, and mathematical operations.
