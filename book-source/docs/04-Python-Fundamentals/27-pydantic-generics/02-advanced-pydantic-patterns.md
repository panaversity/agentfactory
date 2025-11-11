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
generated_by: "lesson-writer v3.0.0"
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

> "Show me 3 different ways to validate an email in Pydantic, from simple to complex. Include: 1) basic check for @ symbol, 2) regex pattern matching, and 3) checking against a list of allowed domains."

This prompt helps you see different complexity levels and when each approach makes sense. Simple checks run faster; regex is more comprehensive; domain lists add security.

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

#### 🎓 Instructor Commentary

> In AI-native development, you don't memorize email regex patterns. Your job is understanding YOUR business rules: What makes an email valid for YOUR system? Then you describe that intent to Pydantic (or ask AI to generate the validator). Syntax is cheap; understanding your validation rules is gold.

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

#### ✨ Teaching Tip

> Start with Field() constraints. Add custom @field_validator ONLY when you need logic. This keeps your code readable and lets Pydantic optimize validation. Common progression: 1) Try Field(), 2) Test with invalid data, 3) If constraints aren't enough, add custom validator.

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

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Create a PasswordChange model where: new_password != old_password, new_password == confirm_password, and new_password must be at least 8 characters AND contain at least one number. Include helpful error messages for each rule."

**Expected Outcome**: A complete PasswordChange model that validates password security AND matching AND difference from current password. You'll see how to combine Field constraints with model validators for comprehensive validation.

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

#### 💬 AI Colearning Prompt

> "Generate an AppSettings model using BaseSettings that reads from environment. Include DatabaseConfig (host, port, username, password), API_KEY (secret field), and DEBUG (bool). Show a complete .env example and how to load it in your application."

This helps you see the complete pattern: model definition → .env file → loading → usage. You'll understand how environment variables flow into typed, validated settings.

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

#### 🎓 Instructor Commentary

> In production systems, config is the gateway between your code and the real world. Validate it early (BaseSettings does this), use type hints to catch errors, and protect secrets (repr=False). A config error caught at startup beats a crash 3 hours into production.

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

## Try With AI

Now let's apply these patterns with AI as your collaborative partner. Each prompt below moves from understanding to synthesis, with expected outcomes that show what correct application looks like.

### Prompt 1: Understanding (Explain the Decision)

**Your turn**: Ask your AI to explain when you'd use Field() constraints vs @field_validator. Request a decision tree or comparison table.

```
Ask your AI:
"When should I use Field() constraints versus @field_validator in Pydantic?
Create a comparison table with these columns:
- Validation Type
- Use Field() When...
- Use @field_validator When...
- Example for each

Include examples like: email validation, age range, password confirmation,
product SKU."
```

**Expected Outcome**: A clear table showing Field() for simple structural constraints (min/max, length, pattern) and @field_validator for logic-based validation (format checking, cross-field relationships, external validation). The table should justify each choice with reasoning.

---

### Prompt 2: Application (Design with Mixed Approaches)

**Your turn**: Tell your AI to create a User model combining both approaches correctly.

```
Tell your AI:
"Create a Pydantic User model with these requirements:
- email field: use @field_validator to check format (must contain @,
  domain must have dot, no spaces)
- age field: use Field() constraint (must be 13-120)
- bio field: use Field() constraint (max 500 characters)
- username field: use Field() constraint (3-20 chars, alphanumeric + underscore)

Show the model code and 3 test cases:
1. Valid user
2. Invalid email
3. Invalid age

Explain which approach you used for each field and why."
```

**Expected Outcome**: A working User model demonstrating correct tool selection. Email uses @field_validator for format logic; age, bio, and username use Field() constraints. Test cases show validation working correctly.

---

### Prompt 3: Analysis (Evaluate Architectural Choices)

**Your turn**: Ask your AI to explain why BaseSettings is better than manual environment variable reading.

```
Ask your AI:
"Why is Pydantic BaseSettings better than manually reading environment
variables with os.getenv()? What problems does it solve?

Create two code examples:
1. Reading config with os.getenv() (manual way)
2. Reading config with BaseSettings (Pydantic way)

Then explain the problems with approach 1 and how BaseSettings solves them.
Include: type validation, required fields, .env file support, testing
benefits, and secret field handling."
```

**Expected Outcome**: Explanation covering: type safety (os.getenv returns strings; BaseSettings validates types), required field enforcement (os.getenv with defaults hides errors; BaseSettings fails if missing), .env file support (BaseSettings loads .env automatically), testing benefits (can override via code in tests), and secret handling (Field(repr=False)). Both code examples should be runnable.

---

### Prompt 4: Creation (Build Production Settings System)

**Your turn**: Tell your AI to build a complete AppSettings model for a real system.

```
Tell your AI:
"Build a production-quality AppSettings model with:
1. DatabaseConfig (nested model):
   - host (required, default localhost)
   - port (required, valid range 1-65535)
   - username (required, secret field)
   - password (required, secret field)
   - database (required)

2. Main AppSettings:
   - api_key (required, secret field)
   - debug (bool, default False)
   - log_level (enum: DEBUG, INFO, WARNING, ERROR)
   - database (nested DatabaseConfig)

Include:
- Type hints on all fields
- Field constraints (min/max for port, valid values for log_level)
- Environment variable mapping (use env_prefix)
- A complete .env file example
- Code to load and use the settings
- Example of accessing nested config: settings.database.host

Make it production-quality and comment it."
```

**Expected Outcome**: Complete AppSettings implementation with nested models, environment prefixes, validation, secrets management, and .env example. Should demonstrate: 1) Nested model composition, 2) Environment variable reading with prefixes, 3) Type validation and constraints, 4) Secret field handling, 5) Production-ready error messages, 6) Real usage example. This becomes a template for every project configuration you build.

