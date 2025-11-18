---
title: "Advanced Pydantic Patterns"
chapter: 27
lesson: 2
sidebar_position: 2
duration_minutes: 40-45
description: "Master custom field validators, Field() constraints, cross-field validation, and BaseSettings for production configuration management. Learn when to use each validation approach and apply Pydantic to real-world config systems."

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Custom Validation Logic with @field_validator"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write @field_validator for email format validation without template"

  - name: "Field Constraints and Business Rules"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can apply Field() with min/max/regex to enforce business rules independently"

  - name: "Environment Configuration Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student can create BaseSettings model that reads from .env and validates config with type checking"

  - name: "Validation Strategy Selection"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can justify choosing custom validator vs Field() constraint for 3+ scenarios"

learning_objectives:
  - objective: "CREATE custom field validators using @field_validator decorator for complex validation logic"
    proficiency_level: "B1-B2"
    bloom_level: "Apply"
    assessment_method: "Email validator implementation and field-specific error handling"

  - objective: "DESIGN Pydantic models with Field() constraints (min/max, regex, examples) for business rules"
    proficiency_level: "B1-B2"
    bloom_level: "Apply"
    assessment_method: "User model with mixed Field() constraints and custom validators"

  - objective: "IMPLEMENT settings management using BaseSettings for environment variables and .env file loading"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "AppSettings model with required/optional fields, secrets, and environment integration"

  - objective: "ANALYZE when to use custom validators vs built-in Field() constraints based on requirements"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Comparison table and architectural justification for validation approach selection"

cognitive_load:
  new_concepts: 10
  assessment: "10 new concepts at B1-B2 limit (@field_validator, Field(), validator mode, model validators, BaseSettings, .env files, secret fields, field examples, validation context, precedence rules) ✓"

differentiation:
  extension_for_advanced: "B2 students: Explore multi-field validators and conditional validation patterns; design custom validator classes; implement database uniqueness checks"
  remedial_for_struggling: "B1 students: Start with simple Field() constraints, then add one custom @field_validator; use templates for @field_validator structure; pair with AI exploration"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-27/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Advanced Pydantic Patterns

When Lesson 1 showed you basic Pydantic models, you learned that type hints document what your data should be. But what if your data has *business rules* that go beyond types? What if an email must have specific format? What if an age must fall between 13 and 120? What if two passwords must match?

This is where advanced Pydantic patterns come in. You'll move beyond simple validation to enforce your specific rules—the rules YOUR application needs, not just the rules Python's type system provides. You'll also learn the production pattern every modern Python project uses: pulling configuration from environment variables without hardcoding secrets or defaults into your code.

By the end of this lesson, you'll know exactly when to use custom validators versus simple Field constraints, and you'll be able to build production-quality configuration systems that validate data before your application even starts.

---

## Section 1: Custom Field Validators

Beyond basic type checking, Pydantic lets you write custom validation functions. A **field validator** is a function decorated with `@field_validator` that runs when data enters your model.

Let's say you need to validate an email address. Python's type system knows an email is a string—but it doesn't know that a string should contain `@`, have a domain with a dot, and contain no spaces. You need to enforce YOUR specific email rules.

### The @field_validator Decorator

Here's a User model with email validation:

```python
from pydantic import BaseModel, field_validator, ValidationError

class User(BaseModel):
    """User model with custom email validation."""
    email: str
    username: str

    @field_validator('email')
    @classmethod
    def validate_email(cls, v: str) -> str:
        """Validate email format: must contain @, domain must have dot, no spaces."""
        # Check for @ symbol
        if '@' not in v:
            raise ValueError('Email must contain @ symbol')

        # Check for dot in domain
        domain: str = v.split('@')[1]
        if '.' not in domain:
            raise ValueError('Email domain must have at least one dot (e.g., example.com)')

        # Check for spaces
        if ' ' in v:
            raise ValueError('Email cannot contain spaces')

        # Normalize to lowercase
        return v.lower()

# Valid email: passes all checks
user1: User = User(email="alice@example.com", username="alice")
print(user1)
# Output: email='alice@example.com' username='alice'

# Invalid email: fails @ check
try:
    bad_user1: User = User(email="alice.example.com", username="alice")
except ValidationError as e:
    print(e)
    # Output: ValidationError with "Email must contain @ symbol"

# Invalid email: fails domain check
try:
    bad_user2: User = User(email="alice@example", username="alice")
except ValidationError as e:
    print(e)
    # Output: ValidationError with "Email domain must have at least one dot"
```

**How it works:**
1. The `@field_validator('email')` decorator tells Pydantic: "When someone sets the email field, run this function first"
2. The function receives the value being set (the email string) in parameter `v`
3. Your function checks the rules. If something is wrong, raise `ValueError` with a clear message
4. If validation passes, return the (possibly modified) value
5. On invalid data, Pydantic raises `ValidationError` with your custom message

#### 💬 AI Colearning Prompt
> "Show me 3 different ways to validate an email in Pydantic: basic @ symbol check, regex pattern matching, and checking against allowed domains. For each, explain when to use that approach and the tradeoffs."

### Multiple Validators on One Field

You can attach multiple validators to the same field. They run in the order you define them:

```python
from pydantic import field_validator

class StrictUser(BaseModel):
    """User with progressive email validation."""
    email: str

    @field_validator('email')
    @classmethod
    def email_not_empty(cls, v: str) -> str:
        """First check: email must not be empty."""
        if not v or not v.strip():
            raise ValueError('Email cannot be empty')
        return v.strip()

    @field_validator('email')
    @classmethod
    def email_format_valid(cls, v: str) -> str:
        """Second check: email must have @ and domain with dot."""
        if '@' not in v:
            raise ValueError('Email must contain @')
        local, domain = v.split('@', 1)
        if not local or not domain:
            raise ValueError('Invalid email format')
        if '.' not in domain:
            raise ValueError('Domain must have dot')
        return v.lower()

# Validators run in definition order
user: StrictUser = StrictUser(email="  bob@company.com  ")
print(user.email)
# Output: bob@company.com (whitespace stripped by first validator, lowercased by second)
```

Each validator sees the output of the previous one. This pattern lets you build validation progressively, with each step handling one concern.

#### 🎓 Expert Insight
> In AI-native development, syntax is cheap—semantics is gold. You don't memorize regex patterns; you understand YOUR business rules and specify them clearly. AI generates the validator code; you validate the logic matches your requirements.

---

## Section 2: Field Constraints

For simpler validation, don't write a whole `@field_validator`. Use the `Field()` function with constraints. Field constraints are built-in, optimized, and clearer for simple cases.

### When to Use Field() Instead of @field_validator

**Use Field()** when you need:
- Minimum or maximum values (`ge=0` for non-negative)
- String length constraints (`min_length`, `max_length`)
- Pattern matching (`pattern=r"..."` for regex)
- Examples for documentation

**Use @field_validator** when you need:
- Complex conditional logic
- Checking against external data
- Cross-field relationships
- Custom error messages based on input

### Field Constraints in Action

```python
from pydantic import BaseModel, Field

class Product(BaseModel):
    """Product with Field() constraints."""
    name: str = Field(min_length=1, max_length=100, description="Product name")
    price: float = Field(ge=0, le=1_000_000, description="Price in dollars")
    quantity: int = Field(ge=0, description="Stock quantity (non-negative)")
    sku: str = Field(pattern=r"^[A-Z]{3}-\d{4}$", description="Format: ABC-1234")
    discount_percent: float = Field(ge=0, le=100, description="Discount 0-100%")

# Valid product: all constraints satisfied
product1: Product = Product(
    name="Laptop",
    price=999.99,
    quantity=5,
    sku="LAP-0001",
    discount_percent=10.0
)
print(product1)

# Invalid price: negative (fails ge=0)
try:
    bad: Product = Product(
        name="Widget",
        price=-5.00,  # Violates ge=0
        quantity=10,
        sku="WID-0001",
        discount_percent=0
    )
except ValidationError as e:
    print(f"Error: {e}")
    # Shows: "ensure this value is greater than or equal to 0"

# Invalid SKU: wrong format (fails pattern)
try:
    bad: Product = Product(
        name="Widget",
        price=5.00,
        quantity=10,
        sku="WIDGET-001",  # Doesn't match ABC-1234 pattern
        discount_percent=0
    )
except ValidationError as e:
    print(f"Error: {e}")
    # Shows: "string should match regex"
```


### Combining Field Constraints with Custom Validators

You can use both together. Field() handles simple structural validation; @field_validator handles complex logic:

```python
class UserAccount(BaseModel):
    """User with both Field constraints and custom validation."""
    username: str = Field(min_length=3, max_length=20, pattern=r"^[a-z0-9_]+$")
    email: str
    age: int = Field(ge=13, le=120)  # Field constraint: age must be 13-120

    @field_validator('email')
    @classmethod
    def validate_email_format(cls, v: str) -> str:
        """Custom: email must be valid format (beyond Field constraints)."""
        if '@' not in v or '.' not in v.split('@')[1]:
            raise ValueError('Invalid email format')
        return v.lower()

# Field constraints validated automatically
# Custom validator runs after Field constraints pass
user: UserAccount = UserAccount(
    username="alice_123",
    email="alice@example.com",
    age=25
)
print(user)
```

Field constraints (handled by Pydantic automatically) run first. Custom validators run after. This separation makes your code clear: simple rules → Field(), complex rules → validator.

---

## Section 3: Cross-Field Validation with @model_validator

Sometimes a field's validity depends on OTHER fields. This is where `@model_validator` comes in. While `@field_validator` checks one field, `@model_validator` can see the entire model after all field validators pass.

### Example: Password Confirmation

A common scenario: user sets new_password and must confirm it with confirm_password. These fields must match:

```python
from pydantic import BaseModel, field_validator, model_validator

class PasswordChange(BaseModel):
    """Change password with confirmation matching."""
    old_password: str = Field(min_length=8, description="Current password")
    new_password: str = Field(min_length=8, description="New password (min 8 chars)")
    confirm_password: str = Field(description="Confirm new password")

    @model_validator(mode='after')
    def passwords_match(self) -> 'PasswordChange':
        """After all field validators, check that new passwords match."""
        if self.new_password != self.confirm_password:
            raise ValueError('new_password and confirm_password must match')
        return self

    @model_validator(mode='after')
    def new_differs_from_old(self) -> 'PasswordChange':
        """New password must be different from old password."""
        if self.new_password == self.old_password:
            raise ValueError('new_password must be different from old_password')
        return self

# Valid: passwords match and differ from old
change1: PasswordChange = PasswordChange(
    old_password="oldpass123",
    new_password="newpass456",
    confirm_password="newpass456"
)
print("Password change valid:", change1)

# Invalid: passwords don't match
try:
    change2: PasswordChange = PasswordChange(
        old_password="oldpass123",
        new_password="newpass456",
        confirm_password="different789"  # Doesn't match new_password
    )
except ValidationError as e:
    print(f"Mismatch error: {e}")

# Invalid: new password same as old
try:
    change3: PasswordChange = PasswordChange(
        old_password="samepass123",
        new_password="samepass123",  # Same as old—not allowed
        confirm_password="samepass123"
    )
except ValidationError as e:
    print(f"Reuse error: {e}")
```

**Key difference**: `@model_validator(mode='after')` runs AFTER all field validators. By the time your model validator runs, you know all fields have passed their individual validations.

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a PasswordChange model where: new_password != old_password, new_password == confirm_password, and new_password must be at least 8 characters AND contain at least one number. Include helpful error messages for each rule."

**Expected Outcome**: You'll understand how to combine Field() constraints with @model_validator for comprehensive validation, seeing how different validation layers work together to enforce complex business rules.

---

## Section 4: Settings Management with BaseSettings

Now you know how to validate data. The next level: validating CONFIGURATION. Every application needs settings (database host, API keys, debug mode).

The problem: where do you put these settings? Hardcoding is dangerous. Environment variables are common but error-prone. BaseSettings solves this: it reads configuration from environment variables or .env files, validates it with Pydantic, and makes it available to your application.

### Why BaseSettings Instead of os.getenv()

Without Pydantic:
```python
import os

# Manual .env reading (error-prone)
database_url: str = os.getenv('DATABASE_URL', 'sqlite:memory')  # What if it's missing?
api_key: str = os.getenv('API_KEY')  # What if missing or invalid?
debug_mode: str = os.getenv('DEBUG', 'false')  # Is this string 'false' or bool False?
timeout: str = os.getenv('TIMEOUT', '30')  # Is this a string or int? Can it be negative?

# No validation—bad data gets into your app undetected
```

With BaseSettings:
```python
from pydantic_settings import BaseSettings
from pydantic import Field

class AppSettings(BaseSettings):
    """Application settings from environment."""
    database_url: str  # Required—fails if missing
    api_key: str = Field(repr=False)  # Required, never logged
    debug_mode: bool = False  # Type checked and converted
    timeout: int = Field(default=30, ge=1, le=300)  # Validated: 1-300 seconds

    class Config:
        env_file = '.env'
        env_file_encoding = 'utf-8'

# Reads from .env file or environment
# Validates types and constraints
# Fails fast with clear error if required fields missing
settings = AppSettings()

# Type-safe access
print(f"Debug: {settings.debug_mode}")  # Actually a bool, not string
print(f"Timeout: {settings.timeout}s")  # Actually an int, with validated range
# api_key never printed (repr=False prevents logging secrets)
```

### Complete BaseSettings Example

Here's a realistic application settings model:

```python
from pydantic_settings import BaseSettings
from pydantic import Field, field_validator

class DatabaseConfig(BaseSettings):
    """Database connection settings."""
    host: str = Field(default="localhost", description="Database hostname")
    port: int = Field(default=5432, ge=1, le=65535, description="Database port")
    username: str = Field(description="Database username")
    password: str = Field(repr=False, description="Database password (secret)")
    database: str = Field(description="Database name")
    ssl_enabled: bool = Field(default=False, description="Use SSL for connection")

    class Config:
        env_prefix = "DB_"  # Read DB_HOST, DB_PORT, etc.
        env_file = '.env'

class ApiConfig(BaseSettings):
    """External API settings."""
    base_url: str = Field(description="API base URL")
    api_key: str = Field(repr=False, description="API key (secret)")
    timeout_seconds: int = Field(default=30, ge=1, le=300, description="Request timeout")
    max_retries: int = Field(default=3, ge=0, le=10, description="Retry attempts")

    class Config:
        env_prefix = "API_"
        env_file = '.env'

class AppSettings(BaseSettings):
    """Complete application settings."""
    app_name: str = "MyApp"
    log_level: str = Field(default="INFO", description="Logging level")
    debug: bool = Field(default=False, description="Debug mode")
    database: DatabaseConfig
    api: ApiConfig

    class Config:
        env_file = '.env'
        env_file_encoding = 'utf-8'

# Example .env file content:
# APP_NAME=ProductionApp
# LOG_LEVEL=INFO
# DEBUG=false
# DB_HOST=db.example.com
# DB_PORT=5432
# DB_USERNAME=appuser
# DB_PASSWORD=secret123
# DB_DATABASE=appdb
# API_BASE_URL=https://api.example.com
# API_API_KEY=key-123456
# API_TIMEOUT_SECONDS=60

# Load and validate
settings = AppSettings()

# Type-safe access to nested settings
print(f"Connecting to {settings.database.host}:{settings.database.port}")
print(f"API timeout: {settings.api.timeout_seconds}s")
# Output:
# Connecting to db.example.com:5432
# API timeout: 60s
```


### Secret Fields: Hiding Sensitive Data

Notice `Field(repr=False)`. This tells Pydantic: "Don't display this field when someone prints the settings object." This prevents accidentally logging API keys or passwords:

```python
class AppSettings(BaseSettings):
    api_key: str = Field(repr=False)  # Won't appear in str(settings)
    debug: bool = True

settings = AppSettings(api_key="secret-key-123", debug=True)
print(settings)
# Output: api_key=PydanticUndefinedType debug=True
# Notice: api_key is hidden even though it's set

# You can still access it in code:
api_key = settings.api_key  # Works—you have access
print(api_key)  # This is YOUR responsibility—you're explicitly accessing it
```


---

## Common Mistakes

**Mistake 1: Overusing custom validators when Field() suffices**

```python
# ❌ Over-engineered: Field() would be clearer
class User(BaseModel):
    age: int

    @field_validator('age')
    @classmethod
    def validate_age(cls, v: int) -> int:
        if v < 0 or v > 150:
            raise ValueError('Age must be 0-150')
        return v

# ✅ Clearer: just use Field constraints
class User(BaseModel):
    age: int = Field(ge=0, le=150)
```

**Mistake 2: Not validating early (validate at boundaries!)**

```python
# ❌ Bad: config loaded but never validated
config_dict = {"api_key": None, "timeout": -1}
app = App(config=config_dict)  # Invalid config inside app, discovered hours later

# ✅ Good: validate immediately when config enters system
settings = AppSettings()  # Fails immediately if invalid
app = App(config=settings)  # All config guaranteed valid
```

**Mistake 3: Hardcoding secrets instead of using BaseSettings**

```python
# ❌ Never commit this!
API_KEY = "secret-key-12345"  # Git history keeps this forever

# ✅ Use environment variables
# .env (in .gitignore):
# API_KEY=secret-key-12345
# Then in code:
class AppSettings(BaseSettings):
    api_key: str = Field(repr=False)
```

**Mistake 4: Forgetting the mode parameter in @field_validator**

```python
# 🤔 What mode? 'before' or 'after'?
# 'before': validator receives raw input (useful for transformation)
# 'after': validator receives Python object after type conversion (default, usually what you want)

class Model(BaseModel):
    value: int

    # Default 'after' mode: receives int (type already converted)
    @field_validator('value')
    @classmethod
    def validate_positive(cls, v: int) -> int:
        if v < 0:
            raise ValueError('Must be positive')
        return v

    # 'before' mode: receives raw input before type conversion
    # Useful for transforming strings before parsing
    @field_validator('value', mode='before')
    @classmethod
    def ensure_string(cls, v: any) -> any:
        if isinstance(v, str):
            return v.upper()
        return v
```

---

## Try With AI: The Custom Validator Builder (4-Part Learning Challenge)

This challenge teaches you to design validation strategies: understanding when tools apply, building mixed approaches, evaluating tradeoffs, and creating production systems.

---

### Part 1: You Discover (Student Discovers Problems)

**Your Turn** — Implement validation WITHOUT proper tools to see limitations:

```python
# Part 1: Ad-hoc validation (the fragile way)
# You're building an e-commerce system with products and users.
# Implement validation the wrong way first:

def validate_email_manually(email: str) -> bool:
    # Your attempt at email validation (incomplete)
    if '@' not in email:
        return False
    if email.count('@') > 1:
        return False
    # What about domain format? Special characters? Encoding?
    return True

def validate_password_manually(pwd: str) -> bool:
    # Your attempt at password validation
    if len(pwd) < 8:
        return False
    # Does it need uppercase? Numbers? What about weak patterns?
    # How do you validate password != username?
    return True

# Problem to discover:
# - Email validation is incomplete (doesn't validate domain format)
# - Password validation doesn't check complexity
# - Can't validate password != username (cross-field rule)
# - Can't add regex patterns, examples, or constraints
# - No way to collect ALL validation errors at once
# - No way to share validation logic across projects
```

**What You'll Realize**: Ad-hoc validation is fragile, incomplete, and impossible to maintain. You need a declarative way to specify rules.

---

### Part 2: AI as Teacher (AI Explains Concepts)

**Ask your AI:**

> "I'm struggling with validation design. I need to validate:
> 1. Email: Must contain @, domain must have a dot, no spaces
> 2. Password: Min 8 chars, must differ from email, should have number and uppercase
> 3. Username: 3-20 chars, alphanumeric + underscore only
>
> Show me the difference between Field() constraints and @field_validator. When would you use each? Build a User model that uses BOTH correctly—which fields use Field(), which use @field_validator, and why?"

**What AI Should Show You**:
- Field() for structural constraints (length, regex, range)
- @field_validator for logic-based validation (format checking, cross-field rules)
- How to combine them: Field() runs first (fast), then @field_validator (complex logic)
- Side-by-side examples showing when to use each

**Your Role**: Push for clarity. "Why does email need @field_validator when Field(pattern=...) could work?" This forces AI to explain tradeoffs.

---

### Part 3: You as Teacher (You Challenge AI's Approach)

**Challenge AI** — Ask it to handle production requirements:

> "Your User model looks good, but I have production requirements:
> 1. Passwords must contain: at least 1 uppercase, 1 lowercase, 1 number, 1 special character
> 2. Email addresses from 'test.com' domain are not allowed (testing only)
> 3. Password must be different from username AND email
> 4. When validation fails, show all errors at once
>
> For each requirement, show me: a) How to implement it, b) Whether to use Field() or @field_validator, c) How to test that the validation actually works.
>
> Also, can you create a BaseSettings model that loads these from environment variables with APP_USER_ prefix and validates them?"

**Your Role**: Push back on incomplete solutions. "Your regex doesn't check for special characters." or "How do I test that password != email actually works?" This forces AI to refine the design.

**AI's Response Should Show**:
- Multi-line @field_validator for complex password rules
- @model_validator for cross-field relationships
- How BaseSettings automates environment variable loading
- Testing patterns for validation

---

### Part 4: You Build (Production Artifact)

**Build a Complete User Settings System** — Synthesize into a deployable module:

```python
# deliverable: user_settings.py

from pydantic import BaseModel, Field, field_validator, model_validator, ValidationError
from pydantic_settings import BaseSettings
import re

class User(BaseModel):
    """User model with comprehensive validation strategy."""

    username: str = Field(
        min_length=3,
        max_length=20,
        pattern=r"^[a-z0-9_]+$",
        description="Username (3-20 chars, alphanumeric + underscore)"
    )
    email: str = Field(
        description="Valid email address"
    )
    password: str = Field(
        min_length=8,
        description="Password (min 8 chars, complex)"
    )

    # Custom validators for complex rules
    @field_validator('email')
    @classmethod
    def validate_email_format(cls, v: str) -> str:
        """Validate email format and reject test domains."""
        if '@' not in v or '.' not in v.split('@')[1]:
            raise ValueError('Invalid email format')

        domain = v.split('@')[1]
        if domain == 'test.com':
            raise ValueError('Test domain not allowed in production')

        return v.lower()

    @field_validator('password')
    @classmethod
    def validate_password_strength(cls, v: str) -> str:
        """Validate password complexity requirements."""
        if not re.search(r'[A-Z]', v):
            raise ValueError('Password must contain uppercase letter')
        if not re.search(r'[a-z]', v):
            raise ValueError('Password must contain lowercase letter')
        if not re.search(r'\d', v):
            raise ValueError('Password must contain number')
        if not re.search(r'[!@#$%^&*]', v):
            raise ValueError('Password must contain special character (!@#$%^&*)')
        return v

    # Cross-field validation
    @model_validator(mode='after')
    def password_not_username_or_email(self) -> 'User':
        """Ensure password differs from username and email."""
        if self.password.lower() == self.username.lower():
            raise ValueError('Password cannot be same as username')
        if self.password.lower() == self.email.split('@')[0]:
            raise ValueError('Password cannot be same as email username')
        return self


class AppSettings(BaseSettings):
    """Application settings with environment variable loading."""

    app_name: str = "MyApp"
    debug: bool = False
    user: User  # Nested validation

    class Config:
        env_prefix = "APP_"
        env_nested_delimiter = "__"
        env_file = ".env"

# Test implementation
if __name__ == "__main__":
    # Valid user
    user = User(
        username="alice_123",
        email="alice@example.com",
        password="SecurePass123!"
    )
    print(f"✓ User created: {user.username}")

    # Invalid users—shows validation strategy
    test_cases = [
        {
            "name": "Weak password",
            "data": {"username": "bob", "email": "bob@example.com", "password": "weak"},
            "expected_error": "Password must contain uppercase letter"
        },
        {
            "name": "Password = username",
            "data": {"username": "charlie", "email": "charlie@example.com", "password": "Charlie123!"},
            "expected_error": "Password cannot be same as username"
        },
        {
            "name": "Test domain",
            "data": {"username": "dave", "email": "dave@test.com", "password": "TestPass123!"},
            "expected_error": "Test domain not allowed"
        }
    ]

    for test in test_cases:
        try:
            User(**test["data"])
            print(f"✗ {test['name']}: Should have failed")
        except ValidationError as e:
            print(f"✓ {test['name']}: Caught correctly")
```

**Success Criteria**:
- Field() handles structural constraints (length, pattern, range)
- @field_validator handles complex format checking
- @model_validator handles cross-field relationships
- All validation errors reported together
- Validation strategy is clear and maintainable

---

## Time Estimate
**35-40 minutes** (7 min discover, 10 min AI teaches, 10 min you challenge, 8 min build)

