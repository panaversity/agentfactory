---
title: "Contact Card Validator: Real-World Integration Project"
chapter: 16
lesson: 5
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Program Design with AI Partnership"
    proficiency_level: "B1"
    category: "Soft/Metacognitive"
    bloom_level: "Apply + Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can describe program intent before implementation; work with AI to design user interactions; validate final product against intent"

  - name: "Integration of Concepts (Lessons 1-4)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can apply string methods, formatting, and type conversions in a coherent program; make decisions about which string operation to use when"

  - name: "Error Handling and Robustness"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply + Analyze"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student can handle invalid conversions gracefully; validate input; explain why robustness matters"

  - name: "Testing and Validation"
    proficiency_level: "B1"
    category: "Soft/Metacognitive"
    bloom_level: "Apply + Analyze"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can test program with multiple inputs; predict edge cases; validate output correctness"

  - name: "AI-Native Development Pattern (Describe → Explore → Validate → Learn)"
    proficiency_level: "B1"
    category: "Soft/Professional"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can describe program intent to AI; use feedback to improve; validate results; reflect on learning"

learning_objectives:
  - objective: "Design a real-world contact validation program that integrates string operations and type casting before implementation"
    proficiency_level: "B1"
    bloom_level: "Apply + Create"
    assessment_method: "Design phase produces clear intent description and validation requirements"

  - objective: "Implement a complete working program that integrates all Chapter 16 concepts (Lessons 1-4) to solve a practical problem"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Running program validates contact data using string methods, formatting, type conversions, and validation"

  - objective: "Validate program functionality with realistic contact data, including edge cases"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Testing summary shows program handles valid and invalid contact information gracefully"

  - objective: "Reflect on how contact validator demonstrates Chapter 16 concepts in real-world context"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Reflection connects validator features to specific Lesson 1-4 skills and real applications"

cognitive_load:
  new_concepts: 0
  assessment: "Project integrates Lessons 1-4 concepts only (0 new concepts). Real-world context adds motivation without increasing cognitive load. Appropriate for B1 synthesis level. ✓"

differentiation:
  extension_for_advanced: "Add email format validation (checking for @ and domain), phone number formatting (different international formats), postal code validation, multiple contact cards with data storage."
  remedial_for_struggling: "Start with Code Example 5.1 (basic validator) and modify incrementally. Add one validation rule at a time. Use AI to help debug each addition. Focus on getting name cleaning working before moving to email/phone validation."

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/part-4-chapter-16/spec.md"
created: "2025-11-08"
last_modified: "2025-11-08"
git_author: "Claude Code"
workflow: "lesson-implementation"
version: "1.0.0"
---

# Lesson 5: Contact Card Validator — Real-World Integration Project

You've learned four essential skills in this chapter: creating and manipulating strings, formatting output with f-strings, and safely converting between types. Now it's time to apply everything to solve a real problem—a **Contact Card Validator** that cleans, validates, and formats contact information.

Every application needs to collect user data: registration forms, profile pages, contact forms, CRM systems. Messy input like `"  JOHN DOE  "`, invalid emails, or non-numeric ages cause problems. Your job as a developer is to clean and validate this data before storing it.

This project is different from exercises. You'll start by describing what the program should do, then build it with AI as your partner, test it with realistic data, and validate that it works correctly. This is how real programming works: you solve a problem someone has, not just demonstrate concepts.

By the end of this lesson, you'll have built a practical tool that validates contact information using all Chapter 16 concepts—string methods, f-strings, type casting, and validation—skills you'll use in every application you build.

---

## Phase 1: Program Design — Describing Intent First

Before writing code, the most important step is clarifying what problem you're solving. This is the **intent-first** approach that separates professional developers from code copiers.

#### What is the Contact Card Validator?

The Contact Card Validator is a practical tool that solves a real problem: **cleaning and validating messy contact information**.

**The Problem**: Users enter contact data inconsistently:
- Names: `"  john doe  "`, `"JANE SMITH"`, `"bob_jones"`
- Emails: `"user@example"`, `"invalid.com"`, `"@nouser"`
- Phone: `"(555) 123-4567"`, `"555.123.4567"`, `"5551234567"`
- Age: `"25"`, `"twenty-five"`, `"99999"`

**The Solution**: A validator that:

1. **Cleans names** (Lesson 2: String Methods):
   - Removes extra whitespace: `strip()`
   - Fixes capitalization: `title()` for proper names
   - Replaces underscores with spaces: `replace("_", " ")`

2. **Validates emails** (Lesson 2: String Methods):
   - Checks for `@` symbol: `find("@")`
   - Ensures not empty after cleaning: `strip()`
   - Basic format validation

3. **Formats phone numbers** (Lesson 2: String Methods):
   - Removes formatting characters: `replace("(", "").replace(")", "").replace("-", "").replace(".", "").replace(" ", "")`
   - Validates length (10 digits)
   - Displays in standard format

4. **Validates age** (Lesson 4: Type Casting):
   - Converts string to integer: `int(age_str)`
   - Validates reasonable range (0-120)
   - Handles invalid input gracefully

5. **Displays formatted contact card** (Lesson 3: F-Strings):
   - Shows cleaned data in professional format
   - Indicates validation status for each field

#### 💬 AI Colearning Prompt

> "I'm building a contact validator. What fields should I validate? What are common data quality issues with names, emails, and phone numbers? How should I handle invalid data?"

This conversation helps you understand the problem domain before coding.

#### Design Intent — User Story

**User Story**: As a developer building a registration form, I need to clean and validate contact information so that my database contains high-quality data.

**User Interaction Flow**:

```
1. Program starts → Explains what it does
2. Asks for name → User enters messy name
3. Cleans name → Shows original vs. cleaned
4. Asks for email → User enters email
5. Validates email → Shows valid/invalid with reason
6. Asks for phone → User enters phone
7. Formats phone → Shows cleaned number
8. Asks for age → User enters age
9. Validates age → Converts and validates range
10. Displays contact card → Shows all cleaned data formatted nicely
```

#### 🎓 Expert Insight

> In real programming, you're solving problems for users. The Contact Card Validator solves data quality problems every application faces. You're not just learning string methods—you're learning when to use `strip()` (always for user input), `title()` (for names), and `find()` (for validation). This is professional development: understanding the problem before writing a single line of code.

---

## Phase 2: Building Your Validator — AI-Guided Implementation

Now let's build the Contact Card Validator. This code demonstrates all Chapter 16 concepts solving a real problem.

### Code Example 5.1: Basic Contact Card Validator

This is a working version that validates and formats contact information using all Lessons 1-4 concepts:

```python
# Contact Card Validator — Real-World Application
# Demonstrates: string cleaning, validation, f-strings, type casting

print("=== Contact Card Validator ===")
print("Clean and validate contact information\n")

# Step 1: Name Validation (Lessons 1-2: Strings and Methods)
print("STEP 1: NAME VALIDATION")
print("-" * 50)

name_input: str = input("Enter full name: ")

# Clean the name (Lesson 2: String Methods)
cleaned_name: str = name_input.strip()  # Remove leading/trailing whitespace
cleaned_name = cleaned_name.replace("_", " ")  # Replace underscores with spaces
cleaned_name = cleaned_name.title()  # Capitalize properly (Title Case)

# Display results (Lesson 3: F-Strings)
print(f"\nOriginal name: '{name_input}'")
print(f"Cleaned name: '{cleaned_name}'")
print(f"Length: {len(cleaned_name)} characters")

# Validation status
if len(cleaned_name) > 0:
    print("✓ Valid name")
else:
    print("✗ Invalid: Name cannot be empty")

# Step 2: Email Validation (Lesson 2: String Methods)
print(f"\nSTEP 2: EMAIL VALIDATION")
print("-" * 50)

email_input: str = input("Enter email address: ")

# Clean email
cleaned_email: str = email_input.strip().lower()  # Remove whitespace, lowercase

# Basic validation: must contain @ and not be empty
at_position: int = cleaned_email.find("@")

print(f"\nOriginal email: '{email_input}'")
print(f"Cleaned email: '{cleaned_email}'")

if at_position > 0 and at_position < len(cleaned_email) - 1:
    # @ exists and is not first or last character
    print(f"✓ Valid email (@ found at position {at_position})")
else:
    print("✗ Invalid: Email must contain @ symbol (not at start or end)")

# Step 3: Phone Number Formatting (Lesson 2: String Methods)
print(f"\nSTEP 3: PHONE NUMBER FORMATTING")
print("-" * 50)

phone_input: str = input("Enter phone number: ")

# Remove all formatting characters (Lesson 2: replace)
cleaned_phone: str = phone_input.strip()
cleaned_phone = cleaned_phone.replace("(", "")
cleaned_phone = cleaned_phone.replace(")", "")
cleaned_phone = cleaned_phone.replace("-", "")
cleaned_phone = cleaned_phone.replace(".", "")
cleaned_phone = cleaned_phone.replace(" ", "")

print(f"\nOriginal phone: '{phone_input}'")
print(f"Digits only: '{cleaned_phone}'")

# Validate length and format nicely
if cleaned_phone.isdigit() and len(cleaned_phone) == 10:
    # Format as (XXX) XXX-XXXX
    formatted_phone: str = f"({cleaned_phone[0:3]}) {cleaned_phone[3:6]}-{cleaned_phone[6:10]}"
    print(f"Formatted phone: {formatted_phone}")
    print("✓ Valid phone number")
else:
    print("✗ Invalid: Phone must be exactly 10 digits")

# Step 4: Age Validation (Lesson 4: Type Casting)
print(f"\nSTEP 4: AGE VALIDATION")
print("-" * 50)

age_input: str = input("Enter age: ")

# Clean and validate (Lesson 4: Type Casting with validation)
cleaned_age: str = age_input.strip()

if cleaned_age.isdigit():
    age: int = int(cleaned_age)

    print(f"\nString: '{cleaned_age}'")
    print(f"Integer: {age} (type: {type(age).__name__})")

    # Validate reasonable age range
    if 0 <= age <= 120:
        print(f"✓ Valid age")

        # Boolean truthiness demonstration
        is_adult: bool = age >= 18
        print(f"Is adult (18+): {is_adult}")
    else:
        print("✗ Invalid: Age must be between 0 and 120")
else:
    print(f"✗ Invalid: '{age_input}' is not a valid number")
    print("Reason: Age must contain only digits")

# Step 5: Display Contact Card (Lesson 3: F-Strings)
print(f"\n{'=' * 50}")
print("CONTACT CARD")
print("=" * 50)

# Only show card if we have valid data
if len(cleaned_name) > 0:
    print(f"Name:  {cleaned_name}")
    print(f"Email: {cleaned_email}")

    if cleaned_phone.isdigit() and len(cleaned_phone) == 10:
        print(f"Phone: {formatted_phone}")
    else:
        print(f"Phone: (invalid)")

    if cleaned_age.isdigit() and 0 <= int(cleaned_age) <= 120:
        print(f"Age:   {age} years old")
    else:
        print(f"Age:   (invalid)")

    print("=" * 50)
    print("Contact validated successfully!")
else:
    print("Error: Cannot create contact card with invalid name")

print("\n=== Validation Complete ===")
```

**What This Code Demonstrates**:

- **Lesson 1 (String Fundamentals)**: Using `len()`, string indexing for phone formatting (`cleaned_phone[0:3]`), type validation with `type()`
- **Lesson 2 (String Methods)**: `strip()` for all input, `title()` for names, `lower()` for emails, multiple `replace()` calls for phone cleaning, `find()` for email validation, `isdigit()` for number validation
- **Lesson 3 (F-Strings)**: Formatting output throughout (`f"Name: {cleaned_name}"`), formatting phone number with slicing in f-string
- **Lesson 4 (Type Casting)**: Converting age string to integer with validation, checking valid ranges, boolean logic for adult check

**Real-World Application**:

This code solves actual problems:
- **Names**: Handles `"  john doe  "` → `"John Doe"`, `"bob_smith"` → `"Bob Smith"`
- **Emails**: Validates basic format, cleans whitespace, normalizes to lowercase
- **Phone**: Accepts any format → converts to standard `(555) 123-4567`
- **Age**: Validates numeric input and reasonable ranges

**Specification Reference & Validation**:

- **Spec**: Contact validator must clean and validate contact data using Lessons 1-4 concepts
- **AI Prompt Used**: (The above code implements the design intent from Phase 1)
- **Validation Steps**:
  1. Run with messy name (`"  john DOE  "`) → Cleans to `"John Doe"`
  2. Run with valid email (`"user@example.com"`) → Validates successfully
  3. Run with formatted phone (`"(555) 123-4567"`) → Extracts digits and reformats
  4. Run with valid age (`"25"`) → Converts and validates range
  5. Run with invalid data → Shows helpful error messages

---

### Code Example 5.2: Enhanced Validator with Functions and Error Tracking

This version adds organization, better error handling, and validation summary:

```python
# Contact Card Validator — Enhanced Version
# Demonstrates improved code organization and comprehensive validation tracking

def clean_name(raw_name: str) -> tuple[str, bool]:
    """Clean and validate name. Returns (cleaned_name, is_valid)."""
    cleaned: str = raw_name.strip().replace("_", " ").title()
    is_valid: bool = len(cleaned) > 0
    return cleaned, is_valid

def validate_email(raw_email: str) -> tuple[str, bool, str]:
    """Validate email. Returns (cleaned_email, is_valid, message)."""
    cleaned: str = raw_email.strip().lower()
    at_pos: int = cleaned.find("@")

    if at_pos > 0 and at_pos < len(cleaned) - 1:
        return cleaned, True, "Valid email format"
    else:
        return cleaned, False, "Email must contain @ (not at start/end)"

def format_phone(raw_phone: str) -> tuple[str, bool, str]:
    """Format and validate phone. Returns (formatted_phone, is_valid, message)."""
    # Remove all formatting
    digits: str = raw_phone.strip()
    digits = digits.replace("(", "").replace(")", "")
    digits = digits.replace("-", "").replace(".", "").replace(" ", "")

    if digits.isdigit() and len(digits) == 10:
        formatted: str = f"({digits[0:3]}) {digits[3:6]}-{digits[6:10]}"
        return formatted, True, "Valid phone number"
    else:
        return digits, False, f"Phone must be 10 digits (got {len(digits)})"

def validate_age(raw_age: str) -> tuple[int | None, bool, str]:
    """Validate age. Returns (age_int, is_valid, message)."""
    cleaned: str = raw_age.strip()

    if not cleaned.isdigit():
        return None, False, "Age must be a number"

    age: int = int(cleaned)

    if 0 <= age <= 120:
        return age, True, "Valid age"
    else:
        return age, False, "Age must be between 0 and 120"

# Main program
print("=== Contact Card Validator (Enhanced) ===\n")

# Collect all input
name_raw: str = input("Enter full name: ")
email_raw: str = input("Enter email: ")
phone_raw: str = input("Enter phone: ")
age_raw: str = input("Enter age: ")

# Validate all fields
name_clean, name_valid = clean_name(name_raw)
email_clean, email_valid, email_msg = validate_email(email_raw)
phone_formatted, phone_valid, phone_msg = format_phone(phone_raw)
age_value, age_valid, age_msg = validate_age(age_raw)

# Display validation results
print(f"\n{'=' * 50}")
print("VALIDATION RESULTS")
print("=" * 50)

print(f"\nName:  '{name_raw}' → '{name_clean}'")
print(f"       {'✓ Valid' if name_valid else '✗ Invalid'}")

print(f"\nEmail: '{email_raw}' → '{email_clean}'")
print(f"       {'✓' if email_valid else '✗'} {email_msg}")

print(f"\nPhone: '{phone_raw}' → '{phone_formatted}'")
print(f"       {'✓' if phone_valid else '✗'} {phone_msg}")

print(f"\nAge:   '{age_raw}' → {age_value if age_value else '(invalid)'}")
print(f"       {'✓' if age_valid else '✗'} {age_msg}")

# Display contact card only if all valid
all_valid: bool = name_valid and email_valid and phone_valid and age_valid

print(f"\n{'=' * 50}")
if all_valid:
    print("CONTACT CARD — ALL FIELDS VALID")
    print("=" * 50)
    print(f"Name:  {name_clean}")
    print(f"Email: {email_clean}")
    print(f"Phone: {phone_formatted}")
    print(f"Age:   {age_value} years old")
    if age_value and age_value >= 18:
        print("       (Adult)")
    print("=" * 50)
    print("✓ Contact validated successfully!")
else:
    print("VALIDATION FAILED")
    print("=" * 50)
    print("Fix the errors above and try again.")

print("\n=== Validation Complete ===")
```

**Why This Version Is Better**:

1. **Organization**: Functions handle one validation task each (single responsibility)
2. **Reusability**: Each validator can be called independently or reused in other programs
3. **Comprehensive Feedback**: Returns both cleaned data AND validation status with messages
4. **Type Hints**: Uses advanced type hints (`tuple[str, bool]`, `int | None`) showing professional patterns
5. **Validation Summary**: Shows all results together, not step-by-step (better UX)
6. **All-or-Nothing**: Contact card only displays if ALL fields valid (real-world requirement)

**Real-World Pattern**:

This is how professional applications validate forms:
1. Collect all input first
2. Validate each field independently
3. Show all errors at once (not one-by-one)
4. Only proceed if everything is valid

**Note**: Functions and tuples are from Chapter 20. This example shows how your validator could evolve; focus on understanding the validation patterns, not memorizing function syntax.

---

## Phase 3: Testing Your Validator — Real-World Scenarios

Now you validate that your contact validator works correctly with realistic data. Testing means running your code with messy, invalid, and edge-case inputs—just like real users provide.

#### 🤝 Practice Exercise

> **Ask your AI**: "I've built a Contact Card Validator. Help me test it with these realistic scenarios:
> 1. Name with extra whitespace and wrong capitalization
> 2. Email missing @ symbol or domain
> 3. Phone number in different formats (dashes, parentheses, dots)
> 4. Age as text instead of number, or unrealistic ages (like 999)
> 5. All fields empty
>
> For each, what should my program output? What makes good validation? Then explain the clean → validate → convert → use pattern."

**Expected Outcome**: You understand how to validate real user input and predict edge cases your program should handle gracefully.

### Test Cases You Should Try

**Test 1: Messy Name**
```
Enter full name:   john_doe
Expected: Cleans to "John Doe" (strips whitespace, replaces underscore, title case)
Validation: ✓ Valid name
```

**Test 2: Valid Contact**
```
Name: Sarah Wilson
Email: sarah.wilson@example.com
Phone: (555) 123-4567
Age: 28
Expected: All fields validate, contact card displays, adult status shown
```

**Test 3: Invalid Email**
```
Email: userexample.com (missing @)
Expected: Shows "✗ Email must contain @ (not at start/end)"
Email: @example.com (@ at start)
Expected: Shows same error (invalid format)
```

**Test 4: Phone Number Variations**
```
Phone: 555-123-4567     → Formatted as (555) 123-4567 ✓
Phone: (555) 123-4567   → Formatted as (555) 123-4567 ✓
Phone: 555.123.4567     → Formatted as (555) 123-4567 ✓
Phone: 5551234567       → Formatted as (555) 123-4567 ✓
Phone: 123-456          → ✗ Invalid (only 6 digits)
Phone: 555-123-456a     → ✗ Invalid (contains letter)
```

**Test 5: Age Validation**
```
Age: 25        → ✓ Valid age, Adult
Age: 17        → ✓ Valid age, (not adult)
Age: 0         → ✓ Valid age (newborn)
Age: 150       → ✗ Invalid: Age must be between 0 and 120
Age: twenty    → ✗ Invalid: Age must be a number
Age: -5        → ✗ Invalid: Age must be a number (negative fails isdigit())
```

**Test 6: Edge Case - All Empty**
```
Name: (press Enter)
Email: (press Enter)
Phone: (press Enter)
Age: (press Enter)
Expected: All fields fail validation, contact card not displayed, clear error messages
```

**Test 7: Realistic Messy Data**
```
Name:   BOB_SMITH   (extra spaces, underscores, all caps)
Email: BOB.SMITH@EXAMPLE.COM (uppercase)
Phone: 1-555-123-4567 (includes country code digit)
Age:  30  (whitespace)

Expected behavior:
- Name: Cleans to "Bob Smith" ✓
- Email: Cleans to "bob.smith@example.com" ✓
- Phone: ✗ Invalid (11 digits instead of 10)
- Age: Cleans to "30" ✓
```


---

## Phase 4: Reflection — Connecting Back to Chapter 16

The most important part of this project is understanding how all Chapter 16 concepts solve a real problem together.

### What Lessons 1-4 Did Your Validator Use?

#### String Fundamentals (Lesson 1)
Your validator uses:
- **String creation**: All contact fields start as strings from user input
- **String length**: `len(cleaned_name)` validates non-empty names, `len(cleaned_phone)` checks 10 digits
- **String indexing**: `cleaned_phone[0:3]` extracts area code for formatting
- **Immutability understanding**: Each cleaning operation returns a new string

#### String Methods (Lesson 2)
Your validator applies (most critical lesson!):
- **`strip()`**: Applied to ALL user input first (professional best practice)
- **`title()`**: Fixes name capitalization (`"john doe"` → `"John Doe"`)
- **`lower()`**: Normalizes emails (`"USER@EXAMPLE.COM"` → `"user@example.com"`)
- **`replace()`**: Cleans phone numbers (multiple calls to remove `()-.` characters)
- **`find()`**: Validates email format (checks for `@` symbol)
- **`isdigit()`**: Validates phone digits and age numbers

**Key Insight**: Every method solves a specific real-world problem. You now know WHEN to use each method, not just HOW.

#### F-String Formatting (Lesson 3)
Your validator uses:
- **Variable embedding**: `f"Name: {cleaned_name}"` displays contact fields
- **Expressions**: `f"Phone must be 10 digits (got {len(digits)})"` shows why validation failed
- **Conditional formatting**: `f"{'✓ Valid' if name_valid else '✗ Invalid'}"` shows status
- **String slicing in f-strings**: `f"({cleaned_phone[0:3]}) {cleaned_phone[3:6]}-{cleaned_phone[6:10]}"` formats phone

#### Type Casting (Lesson 4)
Your validator demonstrates:
- **String to integer**: `int(cleaned_age)` converts age for range validation
- **Validation before conversion**: `cleaned_age.isdigit()` prevents crashes
- **Boolean logic**: `age >= 18` determines adult status, `all_valid = name_valid and email_valid and phone_valid and age_valid` checks all fields
- **Error handling**: When conversion fails, returns `None` and shows message (no crashes)

### How These Skills Connect in Real Applications

Think of it like a professional workflow:

1. **Input arrives messy** → User types `"  JOHN DOE  "`, `"555-123-4567"`, `"  25  "`
2. **Clean immediately** → `strip()` removes whitespace (ALWAYS do this first)
3. **Transform format** → `title()` for names, `lower()` for emails, digit extraction for phones
4. **Validate structure** → Check length, check for required characters (`@`), check numeric
5. **Convert types** → String `"25"` becomes integer `25` for math/logic
6. **Format output** → F-strings present clean, professional results
7. **Handle errors gracefully** → No crashes; helpful messages explain what's wrong

**Critical Pattern**: Clean → Validate → Convert → Use

This exact pattern appears in:
- Registration forms (exactly what you built!)
- User profile updates
- Payment processing (credit card numbers)
- Address validation
- Search queries
- API request validation
- Database input sanitization

### Why This Matters

You didn't just learn string methods—you learned **data quality engineering**. Every application has the same problem:

> Users provide messy, inconsistent, invalid data. Your job is to clean, validate, and standardize it before using it.

The Contact Card Validator demonstrates this principle. The skills you practiced here apply to every form, every database input, every API endpoint you'll ever build.

---

## Next Steps: Extending Your Validator

Once your basic validator works, you can add real-world features:

**Enhanced Validations**:
- **Email domain validation**: Check if domain has `.` (e.g., `user@example.com` has `.com`)
- **Phone area code validation**: Validate specific area codes (e.g., `(555)` is not a real US area code)
- **Name length limits**: Enforce minimum/maximum name lengths
- **International phone formats**: Support formats from different countries

**Additional Fields**:
- **Address validation**: City, state, ZIP code
- **Date of birth**: Validate date format and calculate age automatically
- **Username validation**: Check length, allowed characters, no spaces
- **Password strength**: Check length, special characters, numbers

**Data Storage** (Chapter 22: File Handling):
- **Save contacts to file**: Store validated contacts in CSV or JSON format
- **Load and edit**: Read existing contacts and update them
- **Search contacts**: Find contacts by name or email

**User Experience** (Chapter 17: Control Flow):
- **Retry on error**: Let users re-enter invalid data instead of restarting
- **Menu system**: Choose to add, view, edit, or delete contacts
- **Batch import**: Validate multiple contacts at once from a file

Ask your AI companion how to add any of these features. Each one uses Chapter 16 concepts plus skills from upcoming chapters.

---

## Try With AI: Contact Validator Capstone Challenge

You've completed all 4 lessons of Chapter 16. Now integrate everything—strings, methods, f-strings, and type casting—into a production-quality contact validator. This is YOUR capstone project with AI as your code review partner.

### Part 1: Design Your Validator (Your Turn First)

**Before asking AI to write code**, design the validation logic yourself:

**Data Quality Problems**:
```python
# Users will enter messy data like:
name_input = "  bob_SMITH  "          # Extra spaces, underscores, wrong case
email_input = "BOB@EXAMPLE.COM"       # Uppercase
phone_input = "555-123-4567"          # Formatted with dashes
age_input = "  30  "                  # Extra whitespace
```

**Your Design Task**:
1. List the string methods you'll use for EACH field (name, email, phone, age)
2. Define validation rules:
   - Name: Non-empty after stripping, title case
   - Email: Contains `@` (not at start/end), lowercase
   - Phone: Exactly 10 digits, format as (XXX) XXX-XXXX
   - Age: Must be number 0-120, convert to int
3. Decide: Show errors one-at-a-time or all-at-once? (Which is better UX?)

Write your design (pseudocode or bullet points) BEFORE coding.

---

### Part 2: Build Validator with AI (Collaborative Construction)

Share your design with AI:

> "Here's my contact validator design: [paste your design]. Build the code with:
> 1. Type hints for all variables
> 2. Clean → Validate → Convert pattern for each field
> 3. F-string formatted output showing validation results
> 4. Contact card displays ONLY if all fields valid
>
> Show complete working code I can test."

**Your task**: Review AI's implementation critically.
- Does it use the methods you identified?
- Does it validate BEFORE converting (age to int)?
- Does it match your UX decision (one-at-a-time vs. all-at-once)?
- Can you explain EVERY line?

If AI's code differs from your design, ask:
> "Why did you use `.title()` instead of `.capitalize()` for names? What's the difference?"

---

### Part 3: Student Teaches AI (Edge Case Stress Test)

AI built a basic validator. But does it handle production-level edge cases?

Challenge AI with impossible scenarios:

> "My validator passes normal tests. But what happens with:
> 1. Name: '123' (all numbers—is that valid?)
> 2. Email: 'user@@example.com' (double @)
> 3. Phone: '(555) 123-4567 ext 89' (contains extension text)
> 4. Age: '30.5' (decimal—should we round or reject?)
> 5. All fields empty (user just hits Enter 4 times)
>
> For EACH, tell me: Should validation pass or fail? If fail, what error message? If pass, what's the cleaned result? Then show code that handles ALL these cases."

**Your task**: Evaluate AI's hardened validator.
- Does it reject numeric-only names?
- Does it handle multiple `@` symbols gracefully?
- Does it strip ALL non-digits from phone (including text)?
- Does it have clear error messages for each failure?

If AI's solution is too complex, iterate:
> "The `.replace()` chain for phone cleaning is hard to read. Is there a cleaner pattern using a loop or list comprehension?"

---

### Part 4: Production Review (Convergence to Best Practices)

Now get AI to review YOUR implementation:

> "Here's my complete contact validator: [paste YOUR code after incorporating AI feedback]. Review it like a senior developer:
> 1. Code quality: Are variable names clear? Is logic readable?
> 2. Validation completeness: Did I miss any edge cases?
> 3. User experience: Are error messages helpful?
> 4. Type safety: Are type hints accurate?
> 5. Efficiency: Any redundant operations?
>
> Give me specific improvements with before/after code examples."

**Your task**: Apply AI's review feedback selectively.
- Which suggestions improve readability?
- Which catch bugs you missed?
- Which are over-engineering for this skill level?

---

### Part 5: Extend Beyond Chapter 16 (Preview Next Skills)

Final challenge—preview what's possible with future chapters:

> "My validator works perfectly for one contact. How would I extend it to:
> 1. Let users re-enter invalid fields (instead of restarting)?
> 2. Validate 10 contacts in a batch?
> 3. Save validated contacts to a CSV file?
>
> For EACH, tell me: What concept from future chapters do I need? (Control flow? Lists? File I/O?) Show a preview of the pattern WITHOUT full implementation."

**Outcome**: You see how Chapter 16 skills integrate with upcoming chapters—understanding the learning path ahead.

---

**Time**: 45-60 minutes total (this is a capstone!)
**Outcome**: You've designed, built, hardened, and reviewed a production-quality contact validator—experiencing the full development lifecycle with AI as a collaborative partner, not just a code generator.

---

**Safety & Ethics Note**: When building validators:
- **You decide validation rules** (AI implements them)
- **Test with realistic messy data** before deploying
- **Balance strictness vs. user frustration** (too strict → users give up)
- **Your judgment matters**—AI can't know your users' needs
