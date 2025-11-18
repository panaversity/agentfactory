---
title: "Advanced Dataclass Features – Fields, Metadata, Post-Init, and Validation"
chapter: 26
lesson: 4
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Field Customization with field()"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Can use field() with default_factory, metadata, init, and repr parameters; explain when each is needed"

  - name: "Post-Init Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Can implement __post_init__() for validation and computed fields; handle validation errors gracefully"

  - name: "InitVar for Temporary Data"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Can use InitVar to pass data to __post_init__ without storing as instance attribute"

  - name: "Dataclass Serialization"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Can serialize/deserialize dataclasses to/from JSON and dict; handle nested objects"

  - name: "Production Data Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Can design validation strategy for dataclass; validate inputs safely; prevent invalid states"

  - name: "Dataclass Field Metadata"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Can attach arbitrary metadata to dataclass fields; access metadata at runtime for validation/serialization"

learning_objectives:
  - objective: "Use field(), default_factory, and metadata to customize dataclass behavior"
    proficiency_level: "B1-B2"
    bloom_level: "Apply"
    assessment_method: "Create dataclass with custom fields; explain purpose of each parameter"

  - objective: "Implement __post_init__() for validation and computed fields"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Write dataclass with validation logic; handle invalid inputs correctly"

  - objective: "Use InitVar to process temporary data during initialization"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create dataclass with InitVar field; explain why it's not stored"

  - objective: "Serialize and deserialize dataclasses to/from JSON"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Write functions converting dataclass to dict and JSON; handle nested objects"

  - objective: "Design validation strategies preventing invalid dataclass states"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Analyze dataclass design; identify potential invalid states; propose validation approach"

cognitive_load:
  new_concepts: 10
  assessment: "10 concepts (at B1-B2 limit): field(), default_factory, metadata, init/repr/compare, __post_init__, InitVar, serialization, validation; each concept scaffolded from prior lesson"

differentiation:
  extension_for_advanced: "Build custom validator library for dataclasses; explore Pydantic as alternative; design complex nested dataclass hierarchies"
  remedial_for_struggling: "Focus on default_factory pattern first (most common mistake); start with simple __post_init__; progress to metadata/InitVar"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-part-4-chapter-26/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Advanced Dataclass Features – Fields, Metadata, Post-Init, and Validation

In Lesson 3, you learned how `@dataclass` eliminates boilerplate by auto-generating `__init__()`, `__repr__()`, and `__eq__()`. But real-world data models need more control: mutable defaults without gotchas, validation on creation, computed fields, metadata for serialization. That's where advanced dataclass features come in.

In this lesson, you'll master the tools that let dataclasses handle production complexity while staying clean and readable. We'll explore `field()` for customization, `__post_init__()` for validation, InitVar for temporary data, and practical JSON serialization. By the end, you'll build dataclasses that enforce their own correctness and integrate seamlessly with APIs.

## The Challenge: Default Values and Mutable Objects

Before diving into solutions, let's see why basic dataclass defaults can be dangerous. In Python, if you write this:

```python
from dataclasses import dataclass

# This demonstrates the error - mutable defaults are not allowed
try:
    @dataclass
    class TodoList:
        items: list[str] = []  # ❌ DANGER!
except ValueError as e:
    print(f"Error: {e}")
    # Output: ValueError: mutable default <class 'list'> for field items is not allowed
```

This mutable default attempt fails immediately in Python 3.10+. If it were allowed, all instances would share the same list object:

```python
# This is what would happen (conceptual - shown for understanding):
# list1 = TodoList()
# list2 = TodoList()
# list1.items.append("Task 1")
# print(list2.items)  # ['Task 1'] — shared state problem!
```

This mutable default gotcha is the most common dataclass mistake. The solution: `default_factory`.

#### 💬 AI Colearning Prompt

> "Explain why mutable default arguments in Python are dangerous. Why does `default=[]` cause shared state between instances?"

## The Solution: field() and default_factory

The `field()` function gives you fine control over each dataclass field. Here's the production-ready pattern:

```python
from dataclasses import dataclass, field

@dataclass
class TodoList:
    name: str
    items: list[str] = field(default_factory=list)  # ✅ Each instance gets its own list
    tags: dict[str, str] = field(default_factory=dict)  # ✅ Each instance gets its own dict
    priority: int = 5  # Immutable defaults work fine
```

Now each instance gets its own mutable containers:

```python
from dataclasses import dataclass, field

@dataclass
class TodoList:
    name: str
    items: list[str] = field(default_factory=list)
    tags: dict[str, str] = field(default_factory=dict)
    priority: int = 5

list1 = TodoList(name="Work")
list2 = TodoList(name="Personal")

list1.items.append("Review PR")
print(list2.items)  # [] — correct, no shared state
```

**Why this matters**: In production APIs and databases, shared mutable state causes subtle bugs that only appear when you create multiple instances. Using `default_factory` is non-negotiable for production dataclasses.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize Python's mutable default gotcha—you recognize "mutable type as default?" and immediately reach for `default_factory`. The pattern becomes automatic.

### Code Example 1: Using default_factory for Mutable Defaults

Let's see `field()` in action. This is the foundation for all advanced dataclass features.

```python
from dataclasses import dataclass, field
from typing import Any

@dataclass
class Product:
    """Production-ready product data model."""
    name: str
    price: float
    tags: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)
    quantity: int = 0

    def __post_init__(self) -> None:
        """Validate on creation (explained next)."""
        if self.price < 0:
            raise ValueError(f"Price cannot be negative: {self.price}")
        if self.quantity < 0:
            raise ValueError(f"Quantity cannot be negative: {self.quantity}")

# Usage
product1 = Product(name="Laptop", price=999.99)
product2 = Product(name="Mouse", price=29.99)

# Each gets its own lists/dicts (no sharing)
product1.tags.append("electronics")
product2.tags.append("hardware")

print(product1.tags)  # ['electronics']
print(product2.tags)  # ['hardware']
```

**Validation Step**: Run this with type checking:

```bash
python script.py
mypy --strict script.py  # Should pass type check
```

**Specification Reference**: This example demonstrates Spec Example 1 from the plan: "Using `default_factory` for mutable defaults (list, dict)"

**AI Prompts Used**:
- "Create a dataclass for a Product with name, price, and mutable fields (tags, metadata). Use field() with default_factory for mutable types."
- Validate: Run the code, confirm each instance has its own list/dict

## Field Customization: More Control Over Individual Fields

Beyond `default_factory`, `field()` offers other parameters for controlling how fields behave:

```python
from dataclasses import dataclass, field, fields

@dataclass
class APIResponse:
    """API response with customized fields."""
    user_id: int
    username: str

    # Don't include in __init__ (computed later)
    display_name: str = field(init=False, default="")

    # Don't include in __repr__ (sensitive data)
    api_key: str = field(repr=False, default="", doc="Secret API key (hidden from repr)")

    # Don't compare (for equality checks)
    request_id: str = field(compare=False, default="", doc="Request ID for tracing (not part of equality)")

    # Metadata for validation/serialization (Python 3.14+ supports doc parameter)
    email: str = field(
        metadata={"validation": "email", "required": True},
        default="",
        doc="User email address with validation metadata"
    )
```

**Why each parameter matters**:
- `init=False`: Field won't appear in `__init__()` signature (good for computed fields)
- `repr=False`: Field excluded from string representation (good for secrets)
- `compare=False`: Field excluded from equality comparisons
- `metadata`: Arbitrary data attached to field (for validators, serialization hints, documentation)
- `doc`: **NEW in Python 3.14** – Field documentation string (accessible via introspection)

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a User dataclass with name, email, created_at. Add metadata to the email field for validation. Then explain what metadata is and how you'd use it for validation."

**Expected Outcome**: You'll understand that metadata is arbitrary data you attach to fields for use in custom validation functions.

### Code Example 2: Field with Metadata and init/repr Control

Here's a realistic example showing how these parameters work together:

```python
from dataclasses import dataclass, field, fields
import re
from datetime import datetime
from typing import Any

@dataclass
class User:
    """User with custom field behavior."""
    user_id: int
    name: str
    email: str = field(metadata={"pattern": r"^[^@]+@[^@]+\.[^@]+$", "required": True})

    # Computed field (not in __init__)
    email_verified: bool = field(init=False, default=False)

    # Sensitive field (not in __repr__)
    password_hash: str = field(repr=False, default="")

    # Internal field (not compared in __eq__)
    created_at: datetime = field(compare=False, default_factory=datetime.now)

    # Custom metadata for validation
    age: int = field(
        metadata={"min": 0, "max": 150, "type": "age"},
        default=0
    )

# Access metadata at runtime
def validate_field(dataclass_instance: Any, field_name: str, value: Any) -> bool:
    """Validate a field using its metadata."""
    for f in fields(dataclass_instance):
        if f.name == field_name:
            meta = f.metadata
            if "pattern" in meta and isinstance(value, str):
                pattern = meta["pattern"]
                return bool(re.match(pattern, value))
            if "min" in meta and isinstance(value, (int, float)):
                return value >= meta["min"]
            if "max" in meta and isinstance(value, (int, float)):
                return value <= meta["max"]
    return True

# Usage
user = User(
    user_id=1,
    name="Alice",
    email="alice@example.com",
    password_hash="hashed_password_here",
    age=30
)

print(user)  # password_hash not shown ✓
print(user == User(user_id=1, name="Alice", email="alice@example.com", created_at=datetime(2020, 1, 1)))
# True (created_at not compared because compare=False)

# Validate using metadata
print(validate_field(user, "email", "bob@example.com"))  # True
print(validate_field(user, "email", "not_an_email"))      # False
print(validate_field(user, "age", 25))   # True (within range)
print(validate_field(user, "age", 200))  # False (exceeds max)
```

**Specification Reference**: Spec Example 2: "Field with metadata (for serialization, validation)"

**Validation**: Run code, verify field behavior (email not in repr, created_at not compared, etc.)

## Validation After Creation: __post_init__()

The `__post_init__()` method runs immediately after `__init__()` completes. It's perfect for validation and computed fields that depend on other fields.

```python
from dataclasses import dataclass, field
from datetime import datetime, timedelta

@dataclass
class Order:
    """Order with validation in __post_init__()."""
    order_id: int
    customer_name: str
    amount: float

    def __post_init__(self) -> None:
        """Validate order on creation."""
        if self.amount <= 0:
            raise ValueError(f"Amount must be positive, got {self.amount}")

        if not self.customer_name or not self.customer_name.strip():
            raise ValueError("Customer name cannot be empty")

# Valid order
order = Order(order_id=1, customer_name="Alice", amount=99.99)

# Invalid order — raises immediately
try:
    bad_order = Order(order_id=2, customer_name="", amount=-50)
except ValueError as e:
    print(f"Order creation failed: {e}")
```

**Why __post_init__() is essential**:
1. Validation happens at creation time (fail fast)
2. Invalid states are impossible to create
3. Cleaner than manual validation after instantiation
4. Computed fields can depend on constructor parameters

#### 💬 AI Colearning Prompt
> "What happens if I try to create an Order with amount=0? How would I handle that differently than amount=-50?"

### Code Example 3: __post_init__() for Validation and Computed Fields

Here's a practical example combining validation with computed attributes:

```python
from dataclasses import dataclass, field
from datetime import datetime, timedelta

@dataclass
class Subscription:
    """Subscription with validation and computed expiry date."""
    user_id: int
    plan_name: str
    billing_cycle_days: int

    # Computed fields (calculated in __post_init__)
    created_at: datetime = field(init=False)
    expires_at: datetime = field(init=False)
    is_active: bool = field(init=False)

    def __post_init__(self) -> None:
        """Validate and compute fields."""
        # Validation: plan_name must be one of valid plans
        valid_plans = {"starter", "pro", "enterprise"}
        if self.plan_name not in valid_plans:
            raise ValueError(f"Invalid plan: {self.plan_name}. Must be one of {valid_plans}")

        # Validation: billing_cycle_days must be positive
        if self.billing_cycle_days <= 0:
            raise ValueError(f"Billing cycle must be positive, got {self.billing_cycle_days}")

        # Compute fields
        self.created_at = datetime.now()
        self.expires_at = self.created_at + timedelta(days=self.billing_cycle_days)
        self.is_active = self.expires_at > datetime.now()

# Usage
sub = Subscription(user_id=1, plan_name="pro", billing_cycle_days=30)
print(f"Subscription active: {sub.is_active}")
print(f"Expires: {sub.expires_at}")

# Invalid plan
try:
    bad_sub = Subscription(user_id=2, plan_name="premium", billing_cycle_days=30)
except ValueError as e:
    print(f"Subscription error: {e}")
```

**Specification Reference**: Spec Example 3: "__post_init__() for validation and computed fields"

**Validation**: Run code, verify validation works, check computed fields are set correctly

## InitVar: Temporary Data for Initialization

Sometimes you need to pass data to `__post_init__()` for processing, but don't want to store it as an instance field. That's where `InitVar` comes in:

```python
from dataclasses import dataclass, field, InitVar

@dataclass
class Account:
    """Account with password hashing (password not stored, hash is)."""
    username: str
    password_hash: str = field(init=False, repr=False, default="")

    # InitVar: passed to __post_init__ but not stored
    password: InitVar[str] = ""

    def __post_init__(self, password: str) -> None:
        """Hash password on creation."""
        if not password:
            raise ValueError("Password required")

        # Simple hash (use bcrypt in real code!)
        self.password_hash = f"hashed_{password}"

# Usage: pass password, but it's not stored
account = Account(username="alice", password="secret123")
print(account)  # password_hash shown, password not shown
# Account(username='alice', password_hash='hashed_secret123')

# The password parameter was used in __post_init__ but is not an instance field
print(hasattr(account, 'password'))  # False
```

**Key insight**: `InitVar` fields appear in `__init__()` signature but NOT as instance fields. They're for data needed during initialization but not afterwards.

### Code Example 4: InitVar for Post-Init Processing Without Storage

Here's a more complex example showing InitVar's power:

```python
from dataclasses import dataclass, field, InitVar
import json

@dataclass
class Product:
    """Product with price validation and optional discount processing."""
    sku: str
    base_price: float
    name: str

    # Computed field
    final_price: float = field(init=False, default=0.0)

    # InitVar: discount percentage, used in __post_init__ but not stored
    discount_percent: InitVar[int] = 0

    def __post_init__(self, discount_percent: int) -> None:
        """Calculate final price after discount."""
        if self.base_price <= 0:
            raise ValueError(f"Base price must be positive, got {self.base_price}")

        if not 0 <= discount_percent <= 100:
            raise ValueError(f"Discount must be 0-100%, got {discount_percent}")

        discount_amount = self.base_price * (discount_percent / 100.0)
        self.final_price = self.base_price - discount_amount

# Usage
product = Product(sku="SKU-001", base_price=100.0, name="Laptop", discount_percent=10)
print(f"Base: ${product.base_price}, Discount: 10%, Final: ${product.final_price}")

# discount_percent was used in __post_init__ but isn't stored
print(hasattr(product, 'discount_percent'))  # False ✓
```

**Specification Reference**: Spec Example 4: "InitVar for post-init processing without storage"

**Validation**: Run code, verify discount_percent is not a stored field, verify final_price is computed correctly

## Serialization: Converting Dataclasses to JSON and Dicts

Real-world applications need to convert dataclasses to JSON (for APIs) and back. Python 3.10+ has `asdict()` and `astuple()` built in:

```python
from dataclasses import dataclass, field, asdict, astuple
import json

@dataclass
class Address:
    """Simple address dataclass."""
    street: str
    city: str
    zip_code: str

@dataclass
class Person:
    """Person with nested address."""
    name: str
    age: int
    address: Address | None = None

# Create instance
person = Person(
    name="Alice",
    age=30,
    address=Address(street="123 Main St", city="San Francisco", zip_code="94105")
)

# Convert to dict (handles nested objects!)
person_dict = asdict(person)
print(person_dict)
# {
#   'name': 'Alice',
#   'age': 30,
#   'address': {'street': '123 Main St', 'city': 'San Francisco', 'zip_code': '94105'}
# }

# Convert to JSON string
person_json = json.dumps(person_dict)
print(person_json)

# Convert back from dict
restored_person = Person(
    name=person_dict['name'],
    age=person_dict['age'],
    address=Address(**person_dict['address']) if person_dict['address'] else None
)
print(restored_person)
```

**Specification Reference**: Spec Example 5: "Dataclass with JSON serialization (to_dict/from_dict)"


### Code Example 6: Real-World API Model with All Advanced Features

Here's a production-ready example combining everything: validation, computed fields, field customization, and serialization:

```python
from dataclasses import dataclass, field, asdict, InitVar, fields
from datetime import datetime, timedelta
from typing import Any
import re
import json

@dataclass
class APIUser:
    """Real-world user model for API responses."""

    # Required fields
    user_id: int
    email: str = field(
        metadata={"pattern": r"^[^@]+@[^@]+\.[^@]+$"},
        doc="User's email address (validated against email regex pattern)"
    )

    # Optional fields with defaults
    username: str = field(default="", doc="Display username (defaults to empty string if not provided)")

    # Mutable default (MUST use default_factory)
    roles: list[str] = field(default_factory=lambda: ["user"])

    # Field metadata for validation (Python 3.14+ supports doc parameter)
    age: int = field(
        default=0,
        metadata={"min": 0, "max": 150},
        doc="User age in years (must be 0-150)"
    )

    # Computed/internal fields (not in __init__)
    created_at: datetime = field(init=False, repr=False)
    is_verified: bool = field(init=False, default=False)

    # Sensitive field (not in __repr__)
    password_hash: str = field(repr=False, default="")

    # InitVar for validation data
    password: InitVar[str] = ""

    def __post_init__(self, password: str) -> None:
        """Validate and compute fields."""
        # Validate email format
        if not re.match(r"^[^@]+@[^@]+\.[^@]+$", self.email):
            raise ValueError(f"Invalid email format: {self.email}")

        # Validate age range
        if not (0 <= self.age <= 150):
            raise ValueError(f"Age out of range: {self.age}")

        # Hash password
        if password:
            if len(password) < 8:
                raise ValueError("Password must be at least 8 characters")
            self.password_hash = f"bcrypt_hash({password})"

        # Set computed fields
        self.created_at = datetime.now()
        self.is_verified = False

def to_dict(user: APIUser) -> dict[str, Any]:
    """Convert user to dict for JSON serialization."""
    data = asdict(user)
    # Convert datetime to ISO string
    data['created_at'] = user.created_at.isoformat()
    return data

def from_dict(data: dict[str, Any]) -> APIUser:
    """Create user from dict (e.g., from API request)."""
    # Remove fields that are init=False (computed fields)
    data.pop('created_at', None)
    data.pop('is_verified', None)

    # Extract password for InitVar
    password = data.pop('password', "")

    # Create user (password goes to __post_init__)
    return APIUser(**data, password=password)

# Usage
user = APIUser(
    user_id=1,
    email="alice@example.com",
    username="alice_wonderland",
    age=28,
    password="securepassword123"
)

print(f"User created: {user}")  # password_hash not shown due to repr=False

# Convert to JSON
user_dict = to_dict(user)
user_json = json.dumps(user_dict, indent=2)
print(user_json)

# Convert back from JSON
restored_user = from_dict(user_dict)
print(f"Restored: {restored_user}")

# Validation catches errors
try:
    bad_user = APIUser(
        user_id=2,
        email="not_an_email",
        password="short"
    )
except ValueError as e:
    print(f"Validation error: {e}")
```

**Specification Reference**: Spec Example 6: "Real-world API model (combining all features)"

**Validation Steps**:
1. Run the code successfully
2. Check JSON serialization handles nested datetime
3. Verify password_hash is not shown in repr
4. Confirm validation catches invalid email and short password
5. Verify asdict() includes all fields except InitVar

## Common Mistakes to Avoid

You now understand the tools. Here are the pitfalls to watch for:

### Mistake 1: Forgetting default_factory for Mutable Defaults

```python
from dataclasses import dataclass, field

# ❌ WRONG - Python 3.10+ catches this error
# @dataclass
# class Config:
#     items: list[str] = []  # ValueError: mutable default not allowed

# ✅ RIGHT - each instance gets its own list
@dataclass
class Config:
    items: list[str] = field(default_factory=list)
```

### Mistake 2: Complex Logic in __post_init__()

`__post_init__()` should validate and compute simple fields. Complex logic belongs in methods:

```python
# ❌ Too much in __post_init__()
def __post_init__(self):
    # Calculate complex metrics
    self.roi = self.revenue - self.costs / self.initial_investment
    self.percentile = self.calculate_percentile()

# ✅ Keep __post_init__() simple
def __post_init__(self):
    if self.revenue < 0:
        raise ValueError("Revenue must be positive")

def calculate_roi(self) -> float:
    """ROI calculation as separate method."""
    return (self.revenue - self.costs) / self.initial_investment
```

### Mistake 3: Not Validating Field Metadata

Metadata is inert—it doesn't auto-validate. You must write validation logic:

```python
from dataclasses import dataclass, field

# ❌ Metadata alone doesn't validate
@dataclass
class User:
    age: int = field(metadata={"min": 0, "max": 150})  # Just metadata, no validation!

# ✅ Write validation in __post_init__()
@dataclass
class User:
    age: int = field(metadata={"min": 0, "max": 150})

    def __post_init__(self) -> None:
        meta = field(age).__metadata__  # Access metadata
        if not (meta["min"] <= self.age <= meta["max"]):
            raise ValueError(f"Age out of range")
```

### Mistake 4: Comparing Instances When You Shouldn't

By default, `__eq__()` compares all fields. Use `compare=False` for fields that shouldn't affect equality:

```python
from dataclasses import dataclass, field
from datetime import datetime

# ❌ created_at affects equality (usually not desired)
@dataclass
class User:
    user_id: int
    name: str
    created_at: datetime

# ✅ created_at doesn't affect equality
@dataclass
class User:
    user_id: int
    name: str
    created_at: datetime = field(compare=False)
```

---

## Part 1: Discover Validation by Building Broken Code First

**Your Role**: Active experimenter discovering why validation matters

Before learning `__post_init__()`, experience what happens without validation.

### Discovery Exercise: Invalid States Without Validation

**Step 1: Create invalid instances easily**

```python
from dataclasses import dataclass

@dataclass
class Product:
    name: str
    price: float

# Easy to create invalid states:
p1 = Product("Widget", -50)      # Negative price!
p2 = Product("", 100)             # Empty name!
p3 = Product("Invalid", "lots")   # Wrong type!

print(p1)  # Product(name='Widget', price=-50)
# No error—bad data is silently accepted

# Hours later, bugs appear:
print(p1.price * 100)  # Calculation with negative price
```

**Problem you'll notice**: Dataclasses accept any data without validation. Invalid states silently propagate through your code.

**Step 2: What we want instead**

```python
# Desired behavior:
# Try to create Product("Widget", -50) → Immediate error: "Price must be positive"
# Try to create Product("", 100) → Immediate error: "Name cannot be empty"
```

**Deliverable**: Document problems with unvalidated data:
- Invalid states accepted silently
- Bugs appear far from the source
- Hard to debug

---

## Part 2: AI Teaches `__post_init__()` Validation

**Your Role**: Student learning from AI Teacher

Now ask AI to teach you how `__post_init__()` enables validation.

### AI Teaching Prompt

Ask your AI companion:

> "I want to add validation to a dataclass so invalid instances can't be created. Explain:
>
> 1. What is `__post_init__()` and when does it run?
> 2. Show me how to validate fields in `__post_init__()` (raise ValueError if invalid)
> 3. What's the difference between `default` and `default_factory`?
> 4. What is `InitVar` and when would you use it?
> 5. Show me a complete Product dataclass with validation, defaults, and InitVar"

### What You'll Learn from AI

**Expected AI Response** (summary):
- **`__post_init__()`**: Runs after `__init__()`, perfect for validation
- **Validation pattern**: Raise ValueError/TypeError with clear messages
- **`default`**: For immutable types (int, str, tuple)
- **`default_factory`**: For mutable types (list, dict)
- **`InitVar`**: Temporary fields passed to `__post_init__()` but not stored

### Convergence Activity

After AI explains, **test your understanding**:

Ask AI: "Create a Product dataclass with:
1. name (required, non-empty string)
2. price (required, positive float)
3. discount_percent (InitVar, optional, 0-100)
4. final_price (computed field, set in __post_init__)
5. tags (optional, default empty list)

Show __post_init__() that validates all inputs and computes final_price."

**Deliverable**: Write a 3-paragraph explanation:
1. How `__post_init__()` enables fail-fast validation
2. The difference between `default` and `default_factory`
3. When and why you'd use `InitVar`

---

## Part 3: Student Challenges AI with Edge Cases

**Your Role**: Student teaching AI about validation subtleties

Test AI's understanding of dataclass validation patterns.

### Challenge 1: Mutable Defaults in `__post_init__()`

**Your prompt to AI**:
> "Here's code with a bug:
>
> ```python
> @dataclass
> class Container:
>     items: list = field(default_factory=list)
>
>     def __post_init__(self):
>         if len(self.items) > 10:
>             raise ValueError('Too many items')
> ```
>
> If I do this:
> ```python
> c1 = Container([1, 2, 3, 4, 5])  # Valid
> c2 = Container()  # Creates empty list
> ```
>
> Predict: Will c2.items be shared across instances? Why or why not?"

**Expected learning**: `default_factory=list` creates a NEW list each time, so no sharing. AI should explain why this is essential.

### Challenge 2: Validation After Nested Object Creation

**Your prompt to AI**:
> "I have nested dataclasses:
>
> ```python
> @dataclass
> class Address:
>     zip_code: str
>
>     def __post_init__(self):
>         if not self.zip_code.isdigit() or len(self.zip_code) != 5:
>             raise ValueError('Invalid zip code')
>
> @dataclass
> class Person:
>     name: str
>     address: Address
>
>     def __post_init__(self):
>         if len(self.name) < 2:
>             raise ValueError('Name too short')
> ```
>
> If I create `Person('Alice', Address('INVALID'))`, which error appears first and why?"

**Expected learning**: Address validation runs first (in Address's `__post_init__()`), so that error appears before Person's validation.

### Challenge 3: Computing Derived Fields

**Your prompt to AI**:
> "Show me how to use InitVar to pass a discount_percent, then compute final_price in __post_init__():
>
> ```python
> @dataclass
> class Product:
>     name: str
>     price: float
>     discount_percent: InitVar[float] = 0  # ???
>     final_price: float = field(init=False)  # ???
> ```
>
> Explain: What happens to discount_percent? Why is final_price set to field(init=False)?"

**Deliverable**: Document three edge cases and verify AI's predictions through testing.

---

## Part 4: Build Advanced Dataclass Patterns Reference

**Your Role**: Knowledge synthesizer creating production patterns

### Your Advanced Dataclass Patterns Reference

Create a file called `advanced_dataclass_patterns.md`:

```markdown
# Advanced Dataclass Patterns and Validation
*Chapter 26, Lesson 4*

## Pattern 1: Basic Validation in `__post_init__()`

```python
from dataclasses import dataclass

@dataclass
class User:
    name: str
    age: int

    def __post_init__(self):
        if len(self.name) < 2:
            raise ValueError("Name must be at least 2 characters")
        if self.age < 0 or self.age > 150:
            raise ValueError("Age must be between 0 and 150")

# Valid
u = User("Alice", 30)

# Invalid - raises ValueError immediately
try:
    u = User("A", 25)  # Error: Name must be at least 2 characters
except ValueError as e:
    print(f"Validation failed: {e}")
```

## Pattern 2: Using `field()` with Defaults

```python
from dataclasses import dataclass, field

@dataclass
class TaskList:
    name: str
    tasks: list = field(default_factory=list)
    metadata: dict = field(default_factory=dict)
    priority: int = 0

# All these work
tl1 = TaskList("Work")
tl2 = TaskList("Personal", [1, 2], {"owner": "me"})

# Each instance gets its own list/dict
tl1.tasks.append("Task A")
tl2.tasks  # Still empty
```

## Pattern 3: Using `InitVar` for Temporary Initialization Data

```python
from dataclasses import dataclass, InitVar, field

@dataclass
class Product:
    name: str
    price: float
    discount_percent: InitVar[float] = 0
    final_price: float = field(init=False)

    def __post_init__(self, discount_percent):
        # discount_percent is available here but NOT stored as attribute
        self.final_price = self.price * (1 - discount_percent / 100)

p = Product("Widget", 100.0, discount_percent=10)
print(p.final_price)  # 90.0
print(hasattr(p, 'discount_percent'))  # False - not stored
```

## Pattern 4: Field Metadata for Documentation

```python
from dataclasses import dataclass, field

@dataclass
class APIResponse:
    user_id: int = field(metadata={"description": "Unique user ID"})
    email: str = field(metadata={"description": "User email", "pattern": ".*@.*"})
    created_at: str = field(metadata={"format": "ISO8601"})

# Access metadata
for f in APIResponse.__dataclass_fields__.values():
    print(f.name, f.metadata)
```

## Pattern 5: Serialization Methods

```python
from dataclasses import dataclass, asdict, astuple
import json

@dataclass
class Person:
    name: str
    age: int

p = Person("Alice", 30)

# Convert to dict
d = asdict(p)  # {'name': 'Alice', 'age': 30}

# Convert to JSON string
json_str = json.dumps(d)

# Reconstruct from dict
p2 = Person(**json.loads(json_str))
```

## Pattern 6: Nested Dataclasses

```python
from dataclasses import dataclass

@dataclass
class Address:
    street: str
    city: str

@dataclass
class Person:
    name: str
    address: Address  # Nested dataclass

p = Person("Alice", Address("Main St", "NYC"))
print(p.address.city)  # NYC

# Equality works recursively
p2 = Person("Alice", Address("Main St", "NYC"))
print(p == p2)  # True
```

---

## Validation Best Practices

1. **Fail fast**: Validate in `__post_init__()`, not later
2. **Clear messages**: Always include what was wrong in ValueError
3. **Type hints first**: Use @dataclass with full type hints
4. **Immutable when possible**: Use frozen=True for config objects
5. **Test invalid creation**: Always test that invalid inputs raise errors

## Common Gotchas

### Gotcha 1: InitVar not accessible outside `__post_init__()`
```python
@dataclass
class Bad:
    password: InitVar[str]

    def some_method(self):
        print(self.password)  # Error! Not stored as attribute
```

### Gotcha 2: Validation doesn't prevent mutation
```python
@dataclass(frozen=False)
class Config:
    value: int

    def __post_init__(self):
        if self.value < 0:
            raise ValueError("Must be positive")

c = Config(10)
c.value = -5  # Works! Validation only runs at creation
```

### Gotcha 3: Field metadata is not enforced
```python
@dataclass
class User:
    age: int = field(metadata={"min": 0})

    def __post_init__(self):
        # Metadata doesn't auto-validate!
        # You must manually check:
        if self.age < 0:
            raise ValueError("Age must be positive")
```
```

**Guide Requirements**:
1. **Six practical patterns** — Basic validation through nested dataclasses
2. **Validation best practices** — 5+ guidelines
3. **Common gotchas** — 3-4 with fixes

**Deliverable**: Complete `advanced_dataclass_patterns.md` as your production reference.

---

## Summary: Bidirectional Learning Pattern

**Part 1 (Student explores)**: You experienced problems with unvalidated dataclasses
**Part 2 (AI teaches)**: AI explained `__post_init__()`, `InitVar`, and field()
**Part 3 (Student teaches)**: You challenged AI with mutable defaults, nested validation, and InitVar semantics
**Part 4 (Knowledge synthesis)**: You built production-ready validation patterns

### What You've Built

1. Documentation of validation problems
2. Understanding of `__post_init__()`, `InitVar`, and `field()` (in your own words)
3. Edge case testing with AI
4. `advanced_dataclass_patterns.md` — Production patterns

### Next Steps

You've now mastered dataclasses. Future chapters will show you Pydantic (which automates even more validation), and you'll understand why Pydantic is sometimes worth adding as a dependency.
