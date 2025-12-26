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
source_spec: "specs/001-part-4-chapter-25/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Date/Time Formatting and Manipulation

You've built datetime objects from user input and understand the basics of timezone awareness. Now you'll learn how to **format** those objects for display and **manipulate** them to solve real-world scheduling problems. A core use case we'll focus on: **task and deadline management**. By the end of this lesson, you'll format dates for different audiences, calculate durations between task deadlines, check if tasks are overdue, and convert times across timezones—the exact skills you need for building task management and scheduling applications.

## Why Formatting Matters: Task Scheduling Example

Imagine you're building a task management app. Behind the scenes, you store task deadlines as UTC datetimes: `datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc)`. But when you display that task to a user in Tokyo, you can't just show raw UTC—you need to show "January 16 at 2:00 AM JST" (what it is in their timezone) and format it beautifully for display.

Here's the challenge:

1. **Formatting**: Task stored as raw datetime → Human-readable "Wednesday, January 15 at 5:00 PM UTC" or adjusted to local time
2. **Manipulation**: Check if task is overdue, calculate days until due date, handle recurring tasks at regular intervals
3. **Timezone conversion**: Show deadline in user's local timezone (UTC to EST, UTC to JST, etc.)

Without these three skills, your task app breaks:
- You show tasks in the wrong timezone
- Users can't tell if a task is overdue
- Recurring task scheduling becomes error-prone
- Global teams see conflicting deadline information

This lesson teaches you all three skills using a task scheduling system as the primary example, emphasizing that you don't memorize all 30+ format codes—you understand the pattern and ask AI when needed.

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

### Common Format Patterns for Task Display

Here are the patterns you'll use most often in a task management system:

```python
from datetime import datetime, timezone

# Task deadline in UTC
task_due: datetime = datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc)

# ISO 8601 (standard for data storage and APIs)
# Store tasks in databases as ISO format
iso_format: str = task_due.strftime("%Y-%m-%dT%H:%M:%S%z")
print(iso_format)  # 2025-01-15T17:00:00+0000

# Dashboard Format (compact, for task lists)
# Show on the app's task list
dashboard: str = task_due.strftime("%m/%d %I:%M %p")
print(dashboard)  # 01/15 05:00 PM

# Email/Notification Format (user-friendly)
# Send in notifications to users
user_friendly: str = task_due.strftime("%A, %B %d at %I:%M %p")
print(user_friendly)  # Wednesday, January 15 at 05:00 PM

# Log Format (for debugging)
# Store in system logs
log_format: str = task_due.strftime("%Y-%m-%d %H:%M:%S UTC")
print(log_format)  # 2025-01-15 17:00:00 UTC
```

Notice how the same task deadline looks different depending on the audience. Your job isn't to remember these strings—it's to choose the right format for your purpose (storage, display, notification, logging).

#### 🎓 Expert Insight

> In AI-native development, you don't memorize all 30+ format codes. You understand the pattern (`%Y` = year, `%m` = month, `%d` = day) and ask AI when you need a specific format. **Syntax is cheap; knowing WHEN to use ISO 8601 vs localized format is gold.** ISO for data storage and APIs. Friendly format for user interfaces.

### Practical Formatting Exercise: Task Display Function

Let's build a function that formats a task deadline in multiple ways for different parts of your application:

```python
from datetime import datetime, timezone

def format_task_deadline(due_date: datetime) -> dict[str, str]:
    """
    Format a task deadline for different contexts (storage, display, notification).

    Args:
        due_date: Task deadline as datetime object (timezone-aware preferred)

    Returns:
        Dictionary with formatted strings for different contexts
    """
    return {
        "database": due_date.strftime("%Y-%m-%dT%H:%M:%S%z"),  # Store in DB
        "dashboard": due_date.strftime("%m/%d %I:%M %p"),       # Show in app
        "email": due_date.strftime("%A, %B %d at %I:%M %p"),    # Send in emails
        "log": due_date.strftime("%Y-%m-%d %H:%M:%S UTC"),      # Debug logs
    }

# Test with a task due date
task_deadline: datetime = datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc)
formats = format_task_deadline(task_deadline)

for context, formatted in formats.items():
    print(f"{context:12} : {formatted}")
```

**Output:**
```
database     : 2025-01-15T17:00:00+0000
dashboard    : 01/15 05:00 PM
email        : Wednesday, January 15 at 05:00 PM
log          : 2025-01-15 17:00:00 UTC
```

This function shows the real-world pattern: capture different formats once, reuse throughout your task management system. Each context gets the format it needs—the database gets ISO 8601 for consistency, the dashboard gets compact format for space, emails get friendly format for users.

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

### Date Arithmetic for Task Scheduling: Adding and Subtracting

Now here's where timedelta shines. Use it to create task deadlines and calculate important dates:

```python
from datetime import datetime, timedelta, timezone

# Task created today
created_date: datetime = datetime.now(timezone.utc)

# Create different deadline options for the task
due_tomorrow: datetime = created_date + timedelta(days=1)
due_next_week: datetime = created_date + timedelta(weeks=1)
due_end_of_month: datetime = created_date + timedelta(days=30)
due_next_quarter: datetime = created_date + timedelta(days=90)

# For recurring tasks, calculate previous deadline
previous_weekly: datetime = created_date - timedelta(weeks=1)

# Build a task with deadline
task = {
    "title": "Complete project proposal",
    "created": created_date,
    "due": due_end_of_month,
    "reminder": due_end_of_month - timedelta(days=7)  # One week before
}

print(f"Task created: {task['created'].strftime('%Y-%m-%d')}")
print(f"Due date: {task['due'].strftime('%Y-%m-%d')}")
print(f"Reminder: {task['reminder'].strftime('%Y-%m-%d')}")
```

Timedelta handles all the complexity for you—leap years, different month lengths, everything. You just say "add 30 days" and it handles February having fewer days than March.

#### ✨ Teaching Tip

> Use Claude Code to explore edge cases: "Add 30 days to January 15. Then check: does it land on Feb 14? Why not 45 days?" This teaches you timedelta respects calendar reality.

### Calculating Duration for Task Management

One of the most practical uses: find how much time has passed or remains until a task deadline:

```python
from datetime import datetime, timezone, timedelta

# Task with deadline
task = {
    "title": "Complete project proposal",
    "due_date": datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc),
    "created_at": datetime(2024, 12, 15, 9, 0, tzinfo=timezone.utc)
}

# How much time from task creation to deadline?
created = task["created_at"]
due = task["due_date"]
time_to_complete: timedelta = due - created

print(f"Days to complete: {time_to_complete.days}")
print(f"Total hours available: {time_to_complete.total_seconds() / 3600:.1f}")

# Check if task is overdue NOW
now: datetime = datetime.now(timezone.utc)
remaining: timedelta = due - now

if remaining.total_seconds() > 0:
    print(f"Time until deadline: {remaining.days} days, {remaining.seconds // 3600} hours")
else:
    overdue_by: timedelta = now - due
    print(f"Task is overdue by: {overdue_by.days} days")
```

Notice we calculate `remaining` and check if it's positive. This is the pattern for task deadlines: if remaining is positive, the task is on time; if negative, it's overdue.

#### 🎓 Expert Insight

> You're not calculating duration by hand (that's error-prone and wasteful). You subtract two datetime objects and timedelta does the work. **Syntax is cheap; knowing to subtract datetime objects and handle the result is gold.** This is why we use timedelta.

### Code Example: Task Status Checker with Overdue Detection

Here's the key task scheduling pattern: check if a task is overdue and show how much time remains:

```python
from datetime import datetime, timedelta, timezone

def check_task_status(task: dict) -> str:
    """
    Calculate task status and time remaining/overdue.

    Args:
        task: Dict with keys 'title' and 'due_date' (timezone-aware datetime)

    Returns:
        Status string like "2 days remaining" or "3 hours overdue"
    """
    due_date: datetime = task["due_date"]
    now: datetime = datetime.now(timezone.utc)
    remaining: timedelta = due_date - now

    # Calculate days, hours, minutes
    total_seconds = remaining.total_seconds()
    days = remaining.days
    hours = remaining.seconds // 3600
    minutes = (remaining.seconds % 3600) // 60

    if total_seconds > 0:
        # Task is not yet due
        parts: list[str] = []
        if days > 0:
            parts.append(f"{days} day{'s' if days != 1 else ''}")
        if hours > 0:
            parts.append(f"{hours} hour{'s' if hours != 1 else ''}")
        if minutes > 0:
            parts.append(f"{minutes} minute{'s' if minutes != 1 else ''}")

        status = ", ".join(parts) if parts else "Less than a minute"
        return f"DUE IN: {status}"
    else:
        # Task is overdue
        overdue = abs(remaining)
        overdue_days = overdue.days
        overdue_hours = overdue.seconds // 3600

        return f"OVERDUE: {overdue_days} days, {overdue_hours} hours"

# Test with multiple tasks
tasks = [
    {"title": "Project proposal", "due_date": datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc)},
    {"title": "Code review", "due_date": datetime(2024, 12, 20, 12, 0, tzinfo=timezone.utc)},
]

for task in tasks:
    status = check_task_status(task)
    print(f"{task['title']:20} : {status}")
```

**Output** (assuming today is 2024-12-26):
```
Project proposal     : DUE IN: 20 days
Code review          : OVERDUE: 6 days, 12 hours
```

This function shows the real task scheduling pattern: calculate timedelta, check if positive (on time) or negative (overdue), extract components, format for display in a task management dashboard.

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

### Real-World Task Scheduling: Global Task Manager with Timezones

Here's a practical example: your task management system needs to show deadlines in the user's local timezone:

```python
from datetime import datetime, timezone, timedelta

class TaskManager:
    """Handle tasks across global timezones."""

    def __init__(self, task: dict):
        """
        Args:
            task: Dict with 'title' and 'due_date' (UTC datetime)
        """
        if task["due_date"].tzinfo is None:
            raise ValueError("due_date must be timezone-aware")
        self.task = task

    def due_in_timezone(self, tz_name: str, offset_hours: int) -> str:
        """Show task deadline in a specific timezone."""
        tz_obj = timezone(timedelta(hours=offset_hours))
        local_time = self.task["due_date"].astimezone(tz_obj)
        return local_time.strftime("%A, %I:%M %p")

    def global_deadline_display(self) -> str:
        """Generate deadline display for global users."""
        timezones = {
            "New York": -5,
            "London": 0,
            "Tokyo": 9,
        }

        lines = [f"Task: {self.task['title']}", "Deadline in your timezone:"]
        for city, offset in timezones.items():
            deadline_local = self.due_in_timezone(city, offset)
            lines.append(f"  {city:15} : {deadline_local}")

        return "\n".join(lines)

# Use it
task = {
    "title": "Complete project proposal",
    "due_date": datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc)
}

manager = TaskManager(task)
print(manager.global_deadline_display())
```

**Output:**
```
Task: Complete project proposal
Deadline in your timezone:
  New York        : Wednesday, 12:00 PM
  London          : Wednesday, 05:00 PM
  Tokyo           : Thursday, 02:00 AM
```

This is real production code. You're solving an actual problem: helping users in different timezones understand when a task is due in THEIR local time (not UTC). Store everything in UTC in your database, convert to local time for display.

#### 🎓 Expert Insight

> You don't calculate timezone offsets in your head (that's why timezones exist as objects). You understand: "UTC is the reference, offsets are +/- hours, astimezone() does the conversion." **Syntax is cheap; knowing to keep everything in UTC internally and convert on display is gold.** This pattern prevents bugs.

## Handling Timezone Edge Cases

Timezones aren't simple. During daylight saving time transitions, an hour doesn't exist (spring forward) or exists twice (fall back). For now, know this happens and use AI when you encounter it.

#### ✨ Teaching Tip

> When building a production scheduling system, ask your AI: "How do I handle times during DST transitions? When a time is ambiguous, which version should I pick?" This is where you lean on AI expertise—it's complex enough that asking beats guessing.

## Complete Task Scheduling Example

Let's bring all three concepts together in a complete task scheduling system:

```python
from datetime import datetime, timedelta, timezone
import calendar

# Task with due date
task = {
    "title": "Complete project proposal",
    "due_date": datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc),  # Jan 15, 5 PM UTC
    "created_at": datetime.now(timezone.utc)
}

# Check if task is overdue
def is_overdue(task: dict) -> bool:
    """Check if task is past its due date."""
    return datetime.now(timezone.utc) > task["due_date"]

# Calculate days until due
def days_until_due(task: dict) -> int:
    """Return days until task is due (negative if overdue)."""
    delta = task["due_date"] - datetime.now(timezone.utc)
    return delta.days

# Schedule recurring task
def create_recurring_task(title: str, start_date: datetime, interval_days: int, count: int) -> list:
    """Create multiple tasks at regular intervals."""
    tasks = []
    for i in range(count):
        due_date = start_date + timedelta(days=interval_days * i)
        tasks.append({
            "title": f"{title} (#{i+1})",
            "due_date": due_date,
            "done": False
        })
    return tasks

# Format task for display
def format_task_for_display(task: dict, user_timezone_offset: int = 0) -> str:
    """Format task deadline for user display in their timezone."""
    tz = timezone(timedelta(hours=user_timezone_offset))
    local_due = task["due_date"].astimezone(tz)

    days_left = days_until_due(task)
    if days_left < 0:
        return f"{task['title']}: OVERDUE {abs(days_left)} days - {local_due.strftime('%m/%d %I:%M %p')}"
    else:
        return f"{task['title']}: Due in {days_left} days - {local_due.strftime('%m/%d %I:%M %p')}"

# Example: Weekly review tasks for a month
weekly_reviews = create_recurring_task("Weekly review", datetime(2025, 1, 6, 9, 0, tzinfo=timezone.utc), 7, 4)

print("=== Recurring Weekly Tasks ===")
for task in weekly_reviews:
    print(f"{task['title']}: due {task['due_date'].strftime('%Y-%m-%d %H:%M UTC')}")

print("\n=== Task Status Check ===")
print(f"Main task overdue? {is_overdue(task)}")
print(f"Days until main task due: {days_until_due(task)}")

print("\n=== Display for User (EST: UTC-5) ===")
print(format_task_for_display(task, user_timezone_offset=-5))
```

**Output** (assuming today is 2024-12-26):
```
=== Recurring Weekly Tasks ===
Weekly review (#1): due 2025-01-06 09:00 UTC
Weekly review (#2): due 2025-01-13 09:00 UTC
Weekly review (#3): due 2025-01-20 09:00 UTC
Weekly review (#4): due 2025-01-27 09:00 UTC

=== Task Status Check ===
Main task overdue? False
Days until main task due: 20

=== Display for User (EST: UTC-5) ===
Complete project proposal: Due in 20 days - 01/15 12:00 PM
```

This example combines:
1. **Formatting**: `strftime()` to display deadlines in user-friendly format
2. **Manipulation**: `timedelta` to check overdue status and create recurring tasks
3. **Timezone conversion**: `astimezone()` to show deadlines in user's local timezone

This is the complete pattern you'll use in real task management applications.

---

## Try With AI

Apply task scheduling through AI collaboration. Build the exact patterns you'll use in real task management systems.

**🔍 Task Deadline Formatting:**
> "Format a task deadline datetime(2025, 1, 15, 17, 0, tzinfo=timezone.utc) for three contexts: database (ISO 8601), user notification (friendly format like 'Wednesday, January 15 at 5:00 PM'), and dashboard display (compact like '01/15 05:00 PM'). Explain when to use each context in a task app."

---

**🎯 Task Overdue Detection:**
> "Create a task dict with 'title' and 'due_date'. Write code that calculates if it's overdue NOW by subtracting datetimes. If overdue, show how many days/hours past deadline. If not overdue, show how many days/hours remain. Extract and display as 'DUE IN: 5 days, 3 hours' or 'OVERDUE: 2 days'."

---

**🧪 Recurring Task Scheduling:**
> "Build recurring weekly tasks for 'Team standup' starting January 6, 2025 at 9 AM UTC. Create 4 weekly instances using timedelta(weeks=1). For each, calculate days until that instance is due (as of today). Show which ones are past due and which are coming up."

---

**🚀 Global Task Management:**
> "Build a task scheduler that takes a UTC deadline and converts it to display in 3 timezones (EST, GMT, JST). For each timezone, show the deadline as 'City: Day HH:MM AM/PM'. Explain why storing all task deadlines in UTC matters when your users are globally distributed."

---
