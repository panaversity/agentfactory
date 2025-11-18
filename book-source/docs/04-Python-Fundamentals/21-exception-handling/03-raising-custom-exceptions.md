---
title: "Raising and Custom Exceptions"
chapter: 21
lesson: 3
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Raising Exceptions Strategically"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write functions that check preconditions and raise exceptions when violated, with appropriate error messages"

  - name: "Custom Exception Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create custom exception classes inheriting from Exception that represent domain-specific errors"

  - name: "Error Message Clarity"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can write error messages that explain what went wrong, why it matters, and what the user should do"

learning_objectives:
  - objective: "Write functions that check preconditions and raise exceptions when violated"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Function with validation logic that raises appropriate exception"

  - objective: "Create custom exception classes that inherit from Exception"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Custom exception class used to represent domain-specific error"

  - objective: "Write error messages that explain what went wrong and why"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Error messages in custom exceptions demonstrate understanding of context and user needs"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (raise statement, custom exception classes, meaningful error messages) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design exception hierarchies for complex domain (e.g., banking, e-commerce); consider exception inheritance patterns"
  remedial_for_struggling: "Start with simple validation (single check, simple message); build to multiple validations"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/015-part-4-chapter-21/spec.md"
created: "2025-11-09"
last_modified: "2025-01-18"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Raising and Custom Exceptions

So far, you've written code that catches exceptions—code that anticipates errors and handles them gracefully. But what about the functions you write? How do they signal errors to their callers?

This lesson teaches you to think defensively from the function designer's perspective. You'll start by designing domain exceptions for real-world systems, learn from AI about when to create custom vs use built-in exceptions, challenge AI with exception inheritance and `__init__` customization, and finally build a custom exception library for domain modeling.

---

## Part 1: Student Designs Domain Exceptions

**Your Role**: Domain modeler designing exception types for business logic

Before learning syntax, think about error categories in real systems. Professional developers design exception hierarchies that communicate business domain knowledge.

### Domain Design Exercise: User Authentication System

You're building a user authentication system. Identify what can go wrong and design appropriate exception types.

**System Requirements**:
- Users must provide username and password
- Passwords must meet complexity requirements (8+ characters, mix of types)
- User accounts can be locked after failed attempts
- User accounts can expire
- Sessions can time out

### Your Design Task

**Before writing code**, design exception types for each error scenario:

| Error Scenario | Exception Name | Parent Class | Error Message Pattern |
|---------------|---------------|--------------|----------------------|
| Username not found | UserNotFoundError | Exception | `"User '{username}' does not exist"` |
| Password incorrect | InvalidCredentialsError | Exception | `"Invalid password for user '{username}'"` |
| Password too weak | WeakPasswordError | ValueError | "Password must be 8+ chars with mixed types" |
| Account locked | AccountLockedError | Exception | `"Account '{username}' is locked (too many failed attempts)"` |
| Account expired | AccountExpiredError | Exception | `"Account '{username}' expired on {date}"` |
| Session timeout | SessionExpiredError | Exception | `"Session expired after {minutes} minutes of inactivity"` |

### Design Questions

For each exception, answer:

1. **Should this be a custom exception or use a built-in?**
   - Custom if it represents domain-specific business logic
   - Built-in if it represents a generic validation error

2. **What parent class should it inherit from?**
   - `Exception` for most custom exceptions
   - `ValueError` if it's specifically about invalid input values
   - `RuntimeError` if it's about state violations

3. **What information should the error message include?**
   - What went wrong (the error)
   - What was expected (the rule)
   - What value caused the error (the context)

### Your Design Deliverable

Create a file called `auth_exceptions_design.md`:

```markdown
# Authentication System Exception Design

## Exception Hierarchy

```
Exception
├── AuthenticationError (base for all auth errors)
│   ├── UserNotFoundError
│   ├── InvalidCredentialsError
│   ├── AccountLockedError
│   └── AccountExpiredError
├── ValueError
│   └── WeakPasswordError
└── SessionExpiredError
```

## Exception Specifications

### UserNotFoundError
**When raised**: User lookup fails in database
**Parent**: AuthenticationError
**Required info**: username attempted
**Error message**: `"User '{username}' does not exist in the system"`
**Recovery strategy**: Prompt user to check spelling or register

### InvalidCredentialsError
**When raised**: Password doesn't match stored hash
**Parent**: AuthenticationError
**Required info**: username (NOT password for security)
**Error message**: `"Invalid credentials for user '{username}'"`
**Recovery strategy**: Prompt user to retry or reset password

[Continue for all exception types...]

## Design Rationale

### Why AuthenticationError base class?
Allows callers to catch all auth-related errors with single except block:
```python
try:
    authenticate(username, password)
except AuthenticationError:
    # Handle any authentication failure
except ValueError:
    # Handle validation errors (weak password)
```

### Why WeakPasswordError inherits from ValueError?
Password validation is input validation—ValueError is semantically correct.
Users familiar with ValueError will understand this pattern.
```

**Deliverable**: Complete `auth_exceptions_design.md` with exception hierarchy, specifications for all 6 exception types, and design rationale explaining your inheritance choices.

---

## Part 2: AI Explains When to Create Custom vs Use Built-In

**Your Role**: Student learning exception design principles from AI Teacher

Now that you've designed exceptions, understand when custom exceptions add value vs when built-ins suffice.

### AI Teaching Prompt

Ask your AI companion:

> "I've designed a custom exception hierarchy for an authentication system:
> - AuthenticationError (base)
> - UserNotFoundError
> - InvalidCredentialsError
> - AccountLockedError
> - WeakPasswordError (inherits from ValueError)
>
> Explain:
> 1. When should I create custom exceptions vs using built-in ValueError, TypeError, or RuntimeError?
> 2. What are the benefits of a base exception class like AuthenticationError?
> 3. Show me how callers can catch exceptions at different levels of granularity (specific error vs category of errors).
> 4. Is WeakPasswordError better as a custom exception or inheriting from ValueError? What are the tradeoffs?"

### What You'll Learn from AI

**Expected AI Response** (summary):

**When to create custom exceptions**:
- Domain-specific business logic errors (not generic validation)
- When you want callers to handle different error types differently
- When error messages need domain-specific context
- When you want to add custom attributes (e.g., `username`, `lock_reason`)

**Benefits of base exception classes**:
- Callers can catch all related errors with one except block
- Organizes exception hierarchy by domain
- Makes error handling intent clearer
- Allows for shared behavior (custom `__init__`, `__str__`)

**Granularity example**:
```python
# Catch specific error
try:
    authenticate(user, pwd)
except UserNotFoundError:
    print("Create account?")
except InvalidCredentialsError:
    print("Forgot password?")

# Catch category of errors
try:
    authenticate(user, pwd)
except AuthenticationError:
    print("Authentication failed")

# Catch validation errors separately
try:
    set_password(new_pwd)
except WeakPasswordError:
    print("Password too weak")
except ValueError:
    print("Invalid input")
```

**WeakPasswordError tradeoffs**:
- **Inherit from ValueError**: Semantically correct (input validation), familiar to Python developers
- **Custom exception**: More explicit about domain, easier to catch specifically
- **Recommendation**: Inherit from ValueError for input validation, custom for business logic

### Convergence Activity

After AI explains, test your understanding:

Ask AI: "Show me an authentication function that raises all these exception types. For each exception, include context (username, attempt count, etc.) in the error message."

**Example AI might show**:
```python
class AuthenticationError(Exception):
    """Base class for authentication errors."""
    pass

class UserNotFoundError(AuthenticationError):
    """Raised when username doesn't exist."""
    def __init__(self, username: str):
        self.username = username
        super().__init__(f"User '{username}' does not exist")

class InvalidCredentialsError(AuthenticationError):
    """Raised when password is incorrect."""
    def __init__(self, username: str, attempt_count: int):
        self.username = username
        self.attempt_count = attempt_count
        super().__init__(
            f"Invalid credentials for '{username}' (attempt {attempt_count})"
        )

class WeakPasswordError(ValueError):
    """Raised when password doesn't meet requirements."""
    def __init__(self, password_length: int, min_length: int = 8):
        self.password_length = password_length
        self.min_length = min_length
        super().__init__(
            f"Password too short ({password_length} chars, need {min_length}+)"
        )

def authenticate(username: str, password: str, user_db: dict) -> bool:
    """Authenticate user, raising specific exceptions for different failures."""
    if username not in user_db:
        raise UserNotFoundError(username)

    if user_db[username]['password'] != password:
        attempt_count = user_db[username].get('failed_attempts', 0) + 1
        raise InvalidCredentialsError(username, attempt_count)

    return True

def set_password(password: str) -> None:
    """Set password, validating complexity."""
    if len(password) < 8:
        raise WeakPasswordError(len(password), min_length=8)
    print(f"Password set successfully")
```

**Your turn**: Explain back to AI:
- Why does `UserNotFoundError.__init__` call `super().__init__(message)`?
- What's the benefit of storing `username` and `attempt_count` as instance attributes?
- How would a caller differentiate between `UserNotFoundError` and `InvalidCredentialsError`?

**Deliverable**: Write a 1-paragraph summary explaining when to create custom exceptions (domain logic) vs when to use built-in exceptions (generic validation), with examples from the authentication system.

---

## Part 3: Student Challenges AI with Exception Inheritance and `__init__` Customization

**Your Role**: Student teaching AI by exploring advanced exception patterns

Now reverse the roles. You'll design challenges that test AI's understanding of exception customization, particularly `__init__` methods and instance attributes.

### Challenge Design Pattern

Create scenarios where:
1. Custom exceptions need additional context beyond a message
2. `__init__` methods accept domain-specific parameters
3. Exception attributes enable programmatic error handling

### Challenge 1: Exception with Validation Context

**Your prompt to AI**:
> "I want an exception that captures multiple validation errors. Design a `ValidationError` exception that:
> - Accepts a list of field names and error messages
> - Stores them as instance attributes
> - Formats them nicely in the exception message
> - Allows callers to iterate through individual field errors
>
> Show me the exception class and an example of how to use it."

**Expected AI Response**:
```python
class ValidationError(Exception):
    """Raised when multiple validation rules fail."""
    def __init__(self, errors: dict[str, str]):
        self.errors = errors
        error_summary = "; ".join(f"{field}: {msg}" for field, msg in errors.items())
        super().__init__(f"Validation failed: {error_summary}")

# Usage
errors = {
    "username": "Must be 3-20 characters",
    "email": "Must contain @",
    "age": "Must be positive integer"
}
raise ValidationError(errors)

# Caller can access individual errors
try:
    validate_form(data)
except ValidationError as e:
    for field, message in e.errors.items():
        print(f"Field '{field}': {message}")
```

**Your follow-up**: "Now show me how to extend this to support error codes for internationalization."

### Challenge 2: Exception Chaining with Context

**Your prompt to AI**:
> "Explain exception chaining with `raise ... from e`. Show me an example where:
> - An inner function raises a `json.JSONDecodeError`
> - An outer function catches it and raises a custom `ConfigurationError`
> - The original exception is preserved for debugging
> - The new exception adds business context
>
> Why is this better than just catching and re-raising the original exception?"

**Expected AI Response**:
```python
class ConfigurationError(Exception):
    """Raised when configuration file is invalid."""
    pass

def load_config(filename: str) -> dict:
    """Load configuration from JSON file."""
    try:
        with open(filename) as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        # Chain exception: preserve original, add business context
        raise ConfigurationError(
            f"Invalid configuration file '{filename}'"
        ) from e

# When this fails, traceback shows both exceptions
try:
    config = load_config("settings.json")
except ConfigurationError as e:
    print(f"Config error: {e}")
    print(f"Caused by: {e.__cause__}")  # Original JSONDecodeError
```

**Key insight**: Exception chaining preserves debugging information while adding business context.

### Challenge 3: Exception Hierarchy for E-Commerce

**Your prompt to AI**:
> "Design an exception hierarchy for an e-commerce order system. The system has:
> - Product not found
> - Insufficient inventory
> - Invalid payment method
> - Payment declined
> - Shipping address invalid
>
> Create:
> 1. A base `OrderError` exception
> 2. Subclasses for each error type
> 3. Custom `__init__` methods with relevant attributes
> 4. Example usage showing how callers can catch at different levels
>
> Explain: which exceptions should be recoverable (user can retry) vs fatal (cancel order)?"

**Expected AI Response** (structure):
```python
class OrderError(Exception):
    """Base exception for order processing errors."""
    pass

class ProductNotFoundError(OrderError):
    """Product ID doesn't exist."""
    def __init__(self, product_id: str):
        self.product_id = product_id
        super().__init__(f"Product '{product_id}' not found")

class InsufficientInventoryError(OrderError):
    """Not enough stock available."""
    def __init__(self, product_id: str, requested: int, available: int):
        self.product_id = product_id
        self.requested = requested
        self.available = available
        super().__init__(
            f"Product '{product_id}': requested {requested}, only {available} available"
        )

class PaymentError(OrderError):
    """Base for payment-related errors."""
    pass

class InvalidPaymentMethodError(PaymentError):
    """Payment method not supported."""
    pass

class PaymentDeclinedError(PaymentError):
    """Payment processor declined transaction."""
    def __init__(self, reason: str, transaction_id: str):
        self.reason = reason
        self.transaction_id = transaction_id
        super().__init__(f"Payment declined: {reason} (txn: {transaction_id})")

# Granular error handling
try:
    process_order(order)
except InsufficientInventoryError as e:
    # Recoverable: reduce quantity or wait
    print(f"Only {e.available} available, retry with less?")
except PaymentDeclinedError as e:
    # Recoverable: try different payment method
    print(f"Payment failed: {e.reason}, try another card?")
except ProductNotFoundError:
    # Fatal: remove item from cart
    print("Product no longer exists")
except OrderError:
    # Catch-all for unexpected order errors
    print("Order processing failed")
```

**Deliverable**: Document three advanced exception challenges you posed to AI, AI's solutions, and your analysis of:
- When to add custom `__init__` methods
- When to store exception context as instance attributes
- When to use exception chaining vs simple re-raising

---

## Part 4: Build Custom Exception Library for Domain Modeling

**Your Role**: Library designer creating reusable exception types

Now integrate everything into a production-ready exception library that models domain concepts clearly.

### Your Custom Exception Library

Create a Python file called `domain_exceptions.py` with these patterns:

```python
"""
Custom Exception Library for Domain Modeling
Chapter 21, Lesson 3

This library demonstrates professional exception design patterns:
- Base exception classes for domains
- Custom __init__ with context attributes
- Exception chaining
- Clear error messages
"""

from datetime import datetime
from typing import Optional


# =============================================================================
# Authentication Domain
# =============================================================================

class AuthenticationError(Exception):
    """Base exception for authentication failures."""
    pass


class UserNotFoundError(AuthenticationError):
    """User doesn't exist in system."""

    def __init__(self, username: str):
        self.username = username
        self.timestamp = datetime.now()
        super().__init__(f"User '{username}' does not exist")


class InvalidCredentialsError(AuthenticationError):
    """Invalid username/password combination."""

    def __init__(self, username: str, attempt_number: int):
        self.username = username
        self.attempt_number = attempt_number
        self.timestamp = datetime.now()
        super().__init__(
            f"Invalid credentials for '{username}' (attempt {attempt_number})"
        )


class AccountLockedError(AuthenticationError):
    """Account locked due to too many failed attempts."""

    def __init__(self, username: str, locked_until: datetime):
        self.username = username
        self.locked_until = locked_until
        super().__init__(
            f"Account '{username}' locked until {locked_until.isoformat()}"
        )


class WeakPasswordError(ValueError):
    """Password doesn't meet complexity requirements."""

    def __init__(
        self,
        password_length: int,
        min_length: int = 8,
        missing_requirements: Optional[list[str]] = None
    ):
        self.password_length = password_length
        self.min_length = min_length
        self.missing_requirements = missing_requirements or []

        msg = f"Password too weak ({password_length} chars, need {min_length}+)"
        if self.missing_requirements:
            msg += f". Missing: {', '.join(self.missing_requirements)}"

        super().__init__(msg)


# =============================================================================
# Data Validation Domain
# =============================================================================

class ValidationError(Exception):
    """Base exception for validation failures."""

    def __init__(self, errors: dict[str, str]):
        """
        Args:
            errors: Dict mapping field names to error messages
        """
        self.errors = errors
        self.field_count = len(errors)

        error_summary = "; ".join(
            f"{field}: {msg}" for field, msg in errors.items()
        )
        super().__init__(f"Validation failed ({self.field_count} errors): {error_summary}")

    def has_error(self, field: str) -> bool:
        """Check if specific field has error."""
        return field in self.errors

    def get_error(self, field: str) -> Optional[str]:
        """Get error message for specific field."""
        return self.errors.get(field)


# =============================================================================
# File Processing Domain
# =============================================================================

class FileProcessingError(Exception):
    """Base exception for file processing failures."""
    pass


class InvalidFileFormatError(FileProcessingError):
    """File format doesn't match expected structure."""

    def __init__(self, filename: str, expected_format: str, detected_format: str):
        self.filename = filename
        self.expected_format = expected_format
        self.detected_format = detected_format
        super().__init__(
            f"File '{filename}': expected {expected_format}, got {detected_format}"
        )


class CorruptedDataError(FileProcessingError):
    """Data in file is corrupted or invalid."""

    def __init__(self, filename: str, line_number: int, reason: str):
        self.filename = filename
        self.line_number = line_number
        self.reason = reason
        super().__init__(
            f"File '{filename}' line {line_number}: {reason}"
        )


# =============================================================================
# Business Logic Domain (E-Commerce)
# =============================================================================

class OrderError(Exception):
    """Base exception for order processing errors."""
    pass


class ProductNotFoundError(OrderError):
    """Product ID doesn't exist in catalog."""

    def __init__(self, product_id: str, catalog_version: str = "latest"):
        self.product_id = product_id
        self.catalog_version = catalog_version
        super().__init__(
            f"Product '{product_id}' not found in catalog ({catalog_version})"
        )


class InsufficientInventoryError(OrderError):
    """Not enough inventory to fulfill order."""

    def __init__(self, product_id: str, requested: int, available: int):
        self.product_id = product_id
        self.requested = requested
        self.available = available
        self.shortfall = requested - available
        super().__init__(
            f"Product '{product_id}': requested {requested}, "
            f"only {available} available (short by {self.shortfall})"
        )


class PaymentError(OrderError):
    """Base exception for payment failures."""
    pass


class PaymentDeclinedError(PaymentError):
    """Payment processor declined transaction."""

    def __init__(self, reason: str, transaction_id: str, retry_allowed: bool = True):
        self.reason = reason
        self.transaction_id = transaction_id
        self.retry_allowed = retry_allowed
        super().__init__(
            f"Payment declined: {reason} (txn: {transaction_id}, "
            f"retry: {'yes' if retry_allowed else 'no'})"
        )


# =============================================================================
# Example Usage and Testing
# =============================================================================

def example_authentication():
    """Example: Authentication error handling."""
    print("=== Authentication Example ===\n")

    try:
        # Simulate failed login
        raise InvalidCredentialsError("alice", attempt_number=3)
    except InvalidCredentialsError as e:
        print(f"Login failed: {e}")
        print(f"  Username: {e.username}")
        print(f"  Attempt: {e.attempt_number}")
        print(f"  Time: {e.timestamp.isoformat()}\n")


def example_validation():
    """Example: Multi-field validation."""
    print("=== Validation Example ===\n")

    errors = {
        "username": "Must be 3-20 characters",
        "email": "Must contain @",
        "age": "Must be positive integer",
    }

    try:
        raise ValidationError(errors)
    except ValidationError as e:
        print(f"Form validation failed: {e}\n")
        print("Individual field errors:")
        for field, message in e.errors.items():
            print(f"  - {field}: {message}")
        print()


def example_inventory():
    """Example: Inventory error with context."""
    print("=== Inventory Example ===\n")

    try:
        raise InsufficientInventoryError("WIDGET-123", requested=50, available=30)
    except InsufficientInventoryError as e:
        print(f"Order failed: {e}")
        print(f"  Product: {e.product_id}")
        print(f"  Requested: {e.requested}")
        print(f"  Available: {e.available}")
        print(f"  Shortfall: {e.shortfall}")
        print(f"  Suggestion: Reduce quantity by {e.shortfall} and retry\n")


def example_payment():
    """Example: Payment error with retry logic."""
    print("=== Payment Example ===\n")

    try:
        raise PaymentDeclinedError(
            reason="Insufficient funds",
            transaction_id="TXN-2025-001",
            retry_allowed=True
        )
    except PaymentDeclinedError as e:
        print(f"Payment failed: {e}")
        print(f"  Reason: {e.reason}")
        print(f"  Transaction ID: {e.transaction_id}")
        print(f"  Retry allowed: {e.retry_allowed}")
        if e.retry_allowed:
            print("  Action: Prompt user for different payment method\n")


if __name__ == "__main__":
    print("=== Custom Exception Library Examples ===\n")

    example_authentication()
    example_validation()
    example_inventory()
    example_payment()

    print("=== Exception Hierarchy Example ===\n")

    # Demonstrate catching at different levels
    try:
        raise PaymentDeclinedError("Card expired", "TXN-999", retry_allowed=False)
    except PaymentError:
        print("Caught as PaymentError (base class)")
    except OrderError:
        print("Caught as OrderError (would not reach here)")

    try:
        raise InsufficientInventoryError("PROD-001", 10, 5)
    except OrderError:
        print("Caught as OrderError (base class)\n")
```

### Library Requirements

Your exception library must include:

1. **Authentication domain** (4 exception types)
   - Base `AuthenticationError`
   - User-specific errors with context
   - Custom `__init__` methods storing username, timestamps, attempt counts

2. **Validation domain** (1 exception type)
   - `ValidationError` accepting dict of field errors
   - Helper methods (`has_error`, `get_error`)
   - Useful for form validation

3. **File processing domain** (3 exception types)
   - Base `FileProcessingError`
   - Format and corruption errors
   - Line numbers and file context

4. **Business logic domain** (5 exception types)
   - E-commerce order errors
   - Inventory and payment errors
   - Retry logic and context for recovery

5. **Example usage** for each domain
   - Demonstrates exception raising
   - Shows attribute access
   - Illustrates recovery strategies

### Validation with AI

Once your library is complete, validate it by asking AI:

> "Review my custom exception library. For each exception:
> 1. Is the inheritance hierarchy appropriate?
> 2. Are the custom `__init__` parameters useful for callers?
> 3. Do the error messages provide enough context?
> 4. What exception types am I missing for these domains?
> 5. Are there any patterns I should add (e.g., exception chaining, error codes)?"

**Deliverable**: Complete `domain_exceptions.py` with all 4 domains, comprehensive `__init__` methods, clear error messages, and example usage for each exception type.

---

## Summary: Bidirectional Learning Pattern

In this lesson, you experienced all three roles:

**Part 1 (Student designs)**: You designed domain exceptions for authentication system
**Part 2 (AI teaches)**: AI explained when to create custom vs use built-in exceptions
**Part 3 (Student teaches)**: You challenged AI with exception inheritance and `__init__` customization
**Part 4 (Knowledge synthesis)**: You built a reusable exception library for domain modeling

This pattern ensures you understand exceptions not just as error handling, but as a domain modeling tool that communicates business logic through type systems.

### What You've Built

1. `auth_exceptions_design.md` — Your exception hierarchy design with rationale
2. Custom vs built-in summary — Understanding of when each approach is appropriate
3. Advanced exception challenges — Three patterns exploring `__init__`, inheritance, and chaining
4. `domain_exceptions.py` — Production-ready exception library for 4 domains

### Key Insights

**Exception design principles**:
- **Custom exceptions for domain logic** (business rules, not generic validation)
- **Built-in exceptions for generic errors** (ValueError for input validation)
- **Base exception classes** enable catching categories of errors
- **Custom `__init__` methods** store context for programmatic error handling
- **Exception chaining** preserves debugging information while adding business context

**Error message best practices**:
- Explain what went wrong (the error)
- Show what was expected (the rule)
- Include relevant context (values, timestamps, IDs)
- Suggest recovery actions when possible

### Next Steps

Lesson 4 builds on this foundation, teaching error handling strategies: when to retry, when to fall back to defaults, when to degrade gracefully, and how to log errors for debugging. Your custom exception library will serve as the foundation for strategic error recovery.
