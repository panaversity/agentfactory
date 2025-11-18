---
title: "Pydantic for AI-Native Development"
chapter: 27
lesson: 5
duration_minutes: 45
sidebar_position: 5

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "LLM Output Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can validate structured AI output, detect ValidationError failures, and request regeneration with improved prompts"

  - name: "Prompt Engineering for Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can improve prompts based on validation errors to guide AI toward correct output format"

  - name: "Error-Driven Iteration"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can analyze ValidationError messages, modify approach, and retry the AI-native loop"

  - name: "AI Pipeline Design"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain where validation fits: prompt → generate → validate → use or retry"

  - name: "FastAPI Integration (Intro)"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can recognize how Pydantic enables automatic API validation without deep implementation"

learning_objectives:
  - objective: "Apply Pydantic to validate LLM-generated JSON outputs from Claude Code"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates validation code for real AI-generated data"

  - objective: "Implement iterative refinement: validation fails → improve prompt → regenerate"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student demonstrates the full loop in hands-on exercise"

  - objective: "Analyze validation error patterns to improve AI prompts"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student identifies root causes and explains how to fix prompts"

  - objective: "Evaluate when Pydantic validation adds value in AI pipelines"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Student discusses tradeoffs and production considerations"

  - objective: "Integrate Pydantic with FastAPI for automatic API validation (overview only)"
    proficiency_level: "B1-B2"
    bloom_level: "Understand"
    assessment_method: "Student recognizes FastAPI validation patterns without implementing from scratch"

cognitive_load:
  new_concepts: 10
  assessment: "10 concepts at B1-B2 limit (structured output, validation-first, iterative refinement, error patterns, graceful degradation, FastAPI integration, Pydantic as specification, AI-native workflow, production safety, retry patterns) ✓"

differentiation:
  extension_for_advanced: "Design a production RetryValidator class with exponential backoff and metrics tracking; integrate with logging and monitoring systems"
  remedial_for_struggling: "Focus on Section 1 (validating basic models) and Section 2 (iterative refinement). Practice Try With AI Prompt 1-2 before attempting 3-4"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-part-4-chapter-27/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 5: Pydantic for AI-Native Development

## Introduction: The AI Trust Problem

AI is powerful, but it's **probabilistic, not deterministic**. When you ask Claude Code or another LLM to generate JSON, you get a response that *looks* right but might have subtle issues: a string where you expected an integer, a missing field, an unexpected extra field. These errors don't fail silently—they crash your production system or corrupt your data.

Here's the harsh reality: **Never trust AI output without validation.**

This is where Pydantic becomes your safety net. While Chapters 1-4 showed you how to *define* data structures with Pydantic, this lesson shows you why validation is **critical** in AI systems and how to build the iterative loop that makes AI-native development reliable: **describe your intent → generate output → validate it → if it fails, improve your prompt and try again.**

This lesson teaches you to think like an AI-native engineer: validation isn't optional error handling; it's the core of how you work with unpredictable AI systems.

---

## Section 1: Validating LLM Outputs

When you ask Claude Code to generate structured data (a recipe, a user profile, configuration), it returns JSON as text. Your job is to parse that text and validate it against your Pydantic model.

### The Validation Workflow

Let's say you want Claude Code to generate a recipe:

```python
from pydantic import BaseModel, ValidationError
from typing import Annotated

class Recipe(BaseModel):
    name: str
    ingredients: list[str]
    steps: list[str]
    prep_time_minutes: int  # Must be an integer (minutes)
```

You ask Claude Code: "Generate a recipe for chocolate chip cookies as JSON." Claude responds with something like:

```json
{
    "name": "Chocolate Chip Cookies",
    "ingredients": ["2 cups flour", "1 cup sugar", "2 eggs", "chocolate chips"],
    "steps": ["Mix ingredients", "Bake at 350F for 12 minutes"],
    "prep_time_minutes": 30
}
```

Now comes the validation:

```python
llm_response = '''
{
    "name": "Chocolate Chip Cookies",
    "ingredients": ["2 cups flour", "1 cup sugar", "2 eggs", "chocolate chips"],
    "steps": ["Mix ingredients", "Bake at 350F for 12 minutes"],
    "prep_time_minutes": 30
}
'''

try:
    recipe: Recipe = Recipe.model_validate_json(llm_response)
    print(f"✓ Success! Recipe validated: {recipe.name}")
    print(f"  Prep time: {recipe.prep_time_minutes} minutes")
except ValidationError as e:
    print("✗ Validation failed:")
    for error in e.errors():
        print(f"  Field: {error['loc'][0]}")
        print(f"  Error: {error['msg']}")
```

**Key method**: `model_validate_json()` parses JSON directly from a string and validates it in one step. This is faster and cleaner than parsing with `json.loads()` then calling `Recipe(**data)`.

#### 🎓 Expert Insight
> In AI-native development, validation is your contract with uncertainty. AI probabilistically generates output; validation deterministically checks it. This duality—probabilistic generation, deterministic validation—is the foundation of reliable AI systems.

---

### Handling Validation Errors

Let's see what happens when Claude generates something invalid:

```python
# LLM sometimes generates this (mixing string and int for time)
bad_response = '''
{
    "name": "Cookies",
    "ingredients": ["flour", "sugar"],
    "steps": ["Mix", "Bake"],
    "prep_time_minutes": "30 minutes"  # Wrong! String instead of int
}
'''

try:
    recipe = Recipe.model_validate_json(bad_response)
except ValidationError as e:
    print("Validation Error Details:")
    for error in e.errors():
        location: str = str(error['loc'][0])
        message: str = error['msg']
        print(f"  {location}: {message}")
```

Output:
```
Validation Error Details:
  prep_time_minutes: Input should be a valid integer [type=int_parsing, input_value='30 minutes', input_type=str]
```

**The error tells you exactly what's wrong**: Pydantic expected an integer but got a string. This is *actionable feedback*—you can now improve your prompt to guide the LLM.

#### 💬 AI Colearning Prompt
> "When Pydantic validation fails with 'Input should be a valid integer', what does that tell you about the AI's output? Show examples of prompt improvements that would fix this error."

---

## Section 2: Iterative Refinement Pattern

Here's where AI-native development gets powerful: when validation fails, you don't give up—you iterate.

### First Attempt: Vague Prompt

```python
def generate_recipe_attempt_1() -> Recipe | None:
    """First try: vague prompt"""
    prompt: str = "Generate a recipe for chocolate cookies as JSON."

    # In practice, you'd call Claude Code here
    # llm_response = claude_code.generate(prompt)

    # For demo, simulating what a vague prompt might produce:
    llm_response = '''
    {
        "name": "Chocolate Cookies",
        "ingredients": ["flour", "sugar", "chocolate"],
        "steps": ["Mix", "Bake"],
        "prep_time_minutes": "25 minutes"
    }
    '''

    try:
        return Recipe.model_validate_json(llm_response)
    except ValidationError as e:
        print("❌ First attempt failed:")
        for error in e.errors():
            print(f"   {error['loc'][0]}: {error['msg']}")
        return None

# Result: prep_time_minutes validation fails
generate_recipe_attempt_1()
```

**Why it failed**: The prompt didn't specify the format for `prep_time_minutes`. Claude generated a human-readable string instead of a number.

### Second Attempt: Improved Prompt

```python
def generate_recipe_attempt_2() -> Recipe | None:
    """Second try: explicit format requirements"""
    prompt: str = """
    Generate a recipe for chocolate cookies as JSON.

    CRITICAL: prep_time_minutes MUST be an integer (whole number of minutes),
    NOT a string. Example: 30 (not "30 minutes").

    JSON format:
    {
        "name": "Recipe Name",
        "ingredients": ["ingredient1", "ingredient2"],
        "steps": ["step1", "step2"],
        "prep_time_minutes": <integer>
    }
    """

    # Simulating improved response
    llm_response = '''
    {
        "name": "Chocolate Cookies",
        "ingredients": ["2 cups flour", "1 cup sugar", "1 cup butter", "chocolate chips"],
        "steps": ["Cream butter and sugar", "Add eggs", "Mix in flour", "Add chocolate chips", "Bake at 350F"],
        "prep_time_minutes": 25
    }
    '''

    try:
        recipe: Recipe = Recipe.model_validate_json(llm_response)
        print(f"✅ Success! {recipe.name}")
        print(f"   Prep time: {recipe.prep_time_minutes} minutes")
        return recipe
    except ValidationError as e:
        print("❌ Still failing:")
        for error in e.errors():
            print(f"   {error['loc'][0]}: {error['msg']}")
        return None

# Result: ✓ Validation passes!
generate_recipe_attempt_2()
```

**Why this works**: By explicitly stating "MUST be an integer" and showing an example (30 not "30 minutes"), you guide the LLM to format the data correctly.


#### 🤝 Practice Exercise

> **Ask your AI**: "I need to generate a User profile with fields: username (str), email (str), age (int), is_premium (bool). Generate a sample profile as JSON, then validate it with Pydantic. If validation fails, show me the error and how you'd improve the prompt to fix it."

**Expected Outcome**: You'll experience the complete AI-native validation loop: generate → validate → analyze errors → improve prompt → retry. This iterative refinement is how professional AI-native development works.

---

## Section 3: Error Pattern Analysis

After validating AI outputs for a while, you notice patterns. The same types of errors keep appearing. Understanding these patterns helps you write prompts that prevent failures.

### Common LLM Mistakes

**Pattern 1: Wrong Data Types**
```
LLM generates: "prep_time_minutes": "30"  (string)
You expect: "prep_time_minutes": 30  (integer)

Prevention: Explicit examples in your prompt
"prep_time_minutes must be an integer. Example: 30 (not '30' or '30 minutes')"
```

**Pattern 2: Missing Fields**
```
LLM generates: {"name": "Cookies", "ingredients": [...]}  (missing "steps")
You expect: All fields required

Prevention: List required fields and show complete example
"All fields required: name, ingredients, steps, prep_time_minutes"
```

**Pattern 3: Unexpected Extra Fields**
```
LLM generates: {"name": "...", "ingredients": [...], "difficulty": "easy", ...}
You expect: Only the fields in your model

Prevention: Use Pydantic's ConfigDict to reject extra fields
```

### Using Field Examples to Guide LLMs

Pydantic's `Field()` with `examples` parameter is a powerful hint system:

```python
from pydantic import BaseModel, Field

class Recipe(BaseModel):
    name: str = Field(..., description="Recipe name")
    ingredients: list[str] = Field(
        ...,
        description="List of ingredients"
    )
    steps: list[str] = Field(
        ...,
        description="Cooking steps"
    )
    prep_time_minutes: int = Field(
        ...,
        description="Preparation time in minutes (integer only)",
        examples=[15, 30, 45, 60]  # Show examples!
    )

    model_config = {
        "json_schema_extra": {
            "example": {
                "name": "Chocolate Chip Cookies",
                "ingredients": ["2 cups flour", "1 cup sugar"],
                "steps": ["Mix", "Bake"],
                "prep_time_minutes": 30
            }
        }
    }
```

When you show this model to an LLM, it sees the examples and is more likely to generate correct data.


---

## Section 4: FastAPI Integration (Overview)

While this chapter doesn't teach FastAPI deeply (that's for agent framework chapters), you should understand how Pydantic validation is *automatic* in FastAPI.

### The Pattern

When you build a web API with FastAPI, you define request models as Pydantic classes:

```python
from fastapi import FastAPI

app = FastAPI()

class RecipeInput(BaseModel):
    name: str
    ingredients: list[str]
    prep_time_minutes: int

@app.post("/recipes/")
async def create_recipe(recipe: RecipeInput) -> dict[str, str]:
    """Create a recipe. FastAPI automatically validates the input."""
    # If validation fails, FastAPI returns a 422 error before your code runs
    # If validation passes, recipe is a valid RecipeInput instance
    return {"message": f"Recipe '{recipe.name}' created!"}
```

**Magic**: FastAPI validates the request body against `RecipeInput` *automatically*. If someone sends invalid JSON, FastAPI rejects it with a clear error message before your code ever runs.

You don't write validation code—Pydantic does it for you.

### Request Validation

When a user sends a POST request to `/recipes/`:

```json
{
    "name": "Cookies",
    "ingredients": ["flour", "sugar"],
    "prep_time_minutes": "30 minutes"
}
```

FastAPI:
1. Receives the JSON
2. Validates it against `RecipeInput` model
3. If invalid → returns 422 error with helpful message
4. If valid → deserializes to Python object, calls your function

**Response Validation** works the same way for outputs. You define a response model:

```python
class RecipeOutput(BaseModel):
    id: int
    name: str
    prep_time_minutes: int

@app.get("/recipes/{id}")
async def get_recipe(id: int) -> RecipeOutput:
    """FastAPI validates that your response matches RecipeOutput"""
    # If you return invalid data, FastAPI catches it
    return RecipeOutput(id=1, name="Cookies", prep_time_minutes=30)
```


---

## Section 5: Production Patterns

In production, validation failures are *expected*. LLMs make mistakes. Networks fail. Users send bad data. Your job is to design systems that handle these failures gracefully.

### Pattern 1: Try-Except with Logging

```python
import logging
from typing import TypeVar

logger = logging.getLogger(__name__)
T = TypeVar("T", bound=BaseModel)

def validate_llm_output(json_string: str, model: type[T]) -> T | None:
    """Validate LLM output with logging"""
    try:
        return model.model_validate_json(json_string)
    except ValidationError as e:
        logger.error(f"Validation failed for {model.__name__}")
        for error in e.errors():
            logger.error(f"  Field '{error['loc'][0]}': {error['msg']}")
        return None
```

Always log validation failures. These logs are gold for understanding what's going wrong with your prompts.

### Pattern 2: Retry with Prompt Improvement

```python
def generate_and_validate_with_retry(
    prompt: str,
    model: type[T],
    max_attempts: int = 3
) -> T | None:
    """Generate AI output with automatic retry and prompt improvement"""

    for attempt in range(max_attempts):
        print(f"Attempt {attempt + 1}/{max_attempts}")

        # In practice, call your AI here
        # llm_response = call_claude_code(prompt)

        try:
            result: T = model.model_validate_json(llm_response)
            print(f"✓ Success on attempt {attempt + 1}")
            return result
        except ValidationError as e:
            print(f"✗ Failed: {e.error_count()} errors")

            if attempt < max_attempts - 1:
                # Improve prompt based on errors
                improved_prompt: str = improve_prompt_from_errors(prompt, e)
                prompt = improved_prompt
            else:
                print("✗ Max attempts reached")
                return None

    return None

def improve_prompt_from_errors(original: str, error: ValidationError) -> str:
    """Generate improved prompt based on validation errors"""
    error_details: str = "\n".join([
        f"- {e['loc'][0]}: {e['msg']}"
        for e in error.errors()
    ])

    improved: str = f"""
    {original}

    IMPORTANT: Fix these validation errors from the previous attempt:
    {error_details}

    Make sure to return ONLY valid JSON matching the schema exactly.
    """

    return improved
```

This pattern automatically iterates on your prompt until validation succeeds or you hit the retry limit.

### Pattern 3: Fallback to Human Intervention

When AI can't generate valid data after N retries, escalate:

```python
def generate_with_fallback(
    prompt: str,
    model: type[T],
    max_attempts: int = 3
) -> T | None:
    """Try AI generation, fallback to human if all attempts fail"""

    result: T | None = generate_and_validate_with_retry(
        prompt,
        model,
        max_attempts
    )

    if result is None:
        logger.warning(f"AI generation failed for {model.__name__}. Escalating to human.")
        # In production: send alert, queue for manual review, etc.
        return None

    return result
```


---

## Common Mistakes

**Mistake 1: Using AI output without validation**

```python
# DON'T DO THIS
recipe_json = call_claude_code("Generate a recipe")
recipe = Recipe(**json.loads(recipe_json))  # Crashes if invalid!
```

**Fix**: Always use `model_validate_json()` with try-except.

**Mistake 2: Not giving LLM format examples**

```python
# WEAK PROMPT
"Generate a recipe."

# STRONG PROMPT
"Generate a recipe as JSON with exact format:
{
    'name': 'string',
    'prep_time_minutes': integer (e.g., 30, not '30 minutes')
}"
```

**Mistake 3: Giving up after first failure**

AI often succeeds on second or third try with improved prompts. Don't assume failure is permanent.

**Mistake 4: Overcomplicating prompts**

Start simple. Add detail only when validation fails:

```python
# ITERATION 1 (simple)
"Generate a recipe as JSON."

# ITERATION 2 (add format spec if needed)
"Generate a recipe. prep_time_minutes must be an integer."

# ITERATION 3 (add examples if still failing)
"Generate a recipe. Examples: prep_time_minutes: 30 (not '30 minutes')"
```

---

## Try With AI: The LLM Validation Loop (4-Part Learning Challenge)

This challenge teaches you the core AI-native pattern: generate structured data, validate it, learn from failures, and iterate until successful.

---

### Part 1: You Discover (Student Experiences AI Failures)

**Your Turn** — Ask AI to generate something and experience validation failures:

```python
# Part 1: Generate from AI, discover failures (the learning way)

from pydantic import BaseModel, ValidationError
import json

class BlogPost(BaseModel):
    title: str
    author: str
    content: str  # Hint: should have min length
    tags: list[str]  # Hint: 1-5 tags
    published_date: str  # Hint: YYYY-MM-DD format

# Write a WEAK prompt (no format guidance):
weak_prompt = "Generate a blog post as JSON about Python programming."

# Simulate AI response (often looks reasonable but has errors):
ai_response_1 = '''
{
    "title": "Learning Python",
    "author": "Jane Doe",
    "content": "Python is great.",
    "tags": ["python", "coding", "beginners", "web", "tutorial", "advanced"],
    "published_date": "November 18, 2025"
}
'''

# Try to validate:
try:
    post = BlogPost.model_validate_json(ai_response_1)
except ValidationError as e:
    print("Validation errors:")
    for error in e.errors():
        print(f"  - {error['loc'][0]}: {error['msg']}")

# Problem to discover:
# - AI generated too many tags (6 instead of 1-5)
# - Date format is wrong (needs YYYY-MM-DD, got human-readable date)
# - Content is too short (minimal effort)
# - AI doesn't know your requirements without explicit guidance!
```

**What You'll Realize**: AI is probabilistic. It makes educated guesses. Without explicit requirements, it often gets details wrong. Your job is to guide it with better prompts and validate the results.

---

### Part 2: AI as Teacher (AI Explains the Loop)

**Ask your AI:**

> "I'm building a system where Claude generates structured data (blog posts, recipes, API specs as JSON). The generated data often fails validation. I don't want to hardcode everything, but I need structured data I can trust.
>
> Explain:
> 1. Why validation is essential when using AI output (not optional)
> 2. The 'Generate → Validate → Improve → Retry' loop
> 3. How to analyze validation errors to improve your prompt
> 4. How to design prompts that make AI output more likely to pass validation
>
> Show me a concrete example: BlogPost model with a weak prompt, validation failure, improved prompt, and success."

**What AI Should Show You**:
- Why AI output is probabilistic (even good models make mistakes)
- The iterative refinement loop: prompt → generate → validate → analyze → improve → retry
- How to read ValidationError messages to understand what went wrong
- Strategies to prevent common errors (examples, explicit format specs, constraints)

**Your Role**: Ask clarifying questions. "How many retries should I allow?" "Should I use a different prompt for retry 2 vs retry 3?" Push for understanding the pattern, not just the code.

---

### Part 3: You as Teacher (You Challenge AI's Approach)

**Challenge AI** — Ask it to handle production constraints:

> "Your validation example works, but production needs more:
>
> 1. **Multiple Validation Attempts**: Show a function that retries up to 3 times with progressively better prompts. How do you track which attempt we're on?
>
> 2. **Learning from Errors**: When validation fails, extract the specific errors and build them into the next prompt. Show code that analyzes ValidationError and creates an improved prompt.
>
> 3. **Logging and Metrics**: How would you log each attempt, track success/failure rates, and know which prompts work best?
>
> 4. **Fallback Strategy**: What if validation fails after 3 retries? Should we use default values, escalate to humans, or reject the request?
>
> Show working code for each, including: a) How retries work, b) How you track attempt number, c) How you improve the prompt, d) What metrics you collect"

**Your Role**: Push back on incomplete solutions. "What if the error message is unclear?" or "Should different errors get different retry prompts?" This forces AI to think about robustness.

**AI's Response Should Show**:
- RetryValidator class that attempts generation multiple times
- Prompt engineering that improves after each failure
- Structured logging and metrics collection
- Clear handling of exhausted retries

---

### Part 4: You Build (Production Artifact)

**Build a Complete AI Validation System** — Production-ready retry and validation:

```python
# deliverable: ai_validation.py

from pydantic import BaseModel, ValidationError, Field
import logging
from typing import TypeVar, Generic
import json

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

T = TypeVar("T", bound=BaseModel)

class BlogPost(BaseModel):
    """Blog post model with validation constraints."""
    title: str = Field(min_length=5, max_length=200)
    author: str
    content: str = Field(min_length=100)
    tags: list[str] = Field(min_items=1, max_items=5)
    published_date: str = Field(pattern=r'^\d{4}-\d{2}-\d{2}$')

class ValidationAttempt:
    """Track a single validation attempt."""
    def __init__(self, attempt_num: int):
        self.attempt_num = attempt_num
        self.success = False
        self.errors: list[dict] = []
        self.result = None

class AIValidator[T: BaseModel]:
    """Validates AI-generated structured data with retry logic."""

    def __init__(self, model: type[T], max_attempts: int = 3):
        self.model = model
        self.max_attempts = max_attempts
        self.attempts: list[ValidationAttempt] = []

    def validate(self, json_string: str) -> T | None:
        """Attempt validation with retries and prompt improvement."""

        for attempt_num in range(1, self.max_attempts + 1):
            attempt = ValidationAttempt(attempt_num)
            self.attempts.append(attempt)

            logger.info(f"Validation attempt {attempt_num}/{self.max_attempts}")

            try:
                # Parse and validate
                result = self.model.model_validate_json(json_string)
                attempt.success = True
                attempt.result = result
                logger.info(f"✓ Validation succeeded on attempt {attempt_num}")
                return result

            except ValidationError as e:
                # Extract errors for logging and retry
                attempt.errors = [
                    {
                        "field": str(error["loc"][0]) if error["loc"] else "unknown",
                        "message": error["msg"],
                        "type": error["type"]
                    }
                    for error in e.errors()
                ]

                logger.warning(f"✗ Attempt {attempt_num} failed with {len(e.errors())} errors:")
                for error_info in attempt.errors:
                    logger.warning(f"  {error_info['field']}: {error_info['message']}")

                # If this is the last attempt, don't retry
                if attempt_num >= self.max_attempts:
                    logger.error(f"✗ All {self.max_attempts} attempts failed")
                    return None

                # Otherwise, prepare for retry (in production, you'd call AI again)
                logger.info(f"Preparing retry {attempt_num + 1}...")
                improved_prompt = self._improve_prompt(
                    json_string, attempt.errors
                )
                # In production: json_string = call_ai_with_improved_prompt(improved_prompt)

        return None

    def _improve_prompt(self, previous_json: str, errors: list[dict]) -> str:
        """Generate improved prompt based on validation errors."""
        error_summary = "\n".join([
            f"- {e['field']}: {e['message']}"
            for e in errors
        ])

        return f"""
The previous JSON had these validation errors:
{error_summary}

Please regenerate the data, fixing ALL errors. Return ONLY valid JSON.
Ensure:
- published_date is in YYYY-MM-DD format (e.g., 2025-11-18)
- content is at least 100 characters
- tags is a list with 1-5 items
- title is 5-200 characters
"""

    def get_metrics(self) -> dict:
        """Return validation metrics."""
        successful = sum(1 for a in self.attempts if a.success)
        return {
            "total_attempts": len(self.attempts),
            "successful": successful,
            "success_rate": successful / len(self.attempts) if self.attempts else 0,
            "final_result": self.attempts[-1].result if self.attempts else None
        }

# Test implementation
if __name__ == "__main__":
    # Test Case 1: Validation fails due to date format
    bad_json = '''
    {
        "title": "Python Programming Guide",
        "author": "John Doe",
        "content": "Python is a versatile programming language that is widely used in web development, data science, machine learning, and automation. It is known for its simple and readable syntax, which makes it easy to learn and understand.",
        "tags": ["python", "tutorial"],
        "published_date": "November 18, 2025"
    }
    '''

    validator = AIValidator(BlogPost, max_attempts=3)
    result = validator.validate(bad_json)

    print("\nValidation Metrics:")
    metrics = validator.get_metrics()
    for key, value in metrics.items():
        if key != "final_result":
            print(f"  {key}: {value}")

    # Test Case 2: Valid JSON
    good_json = '''
    {
        "title": "Python Programming Guide",
        "author": "John Doe",
        "content": "Python is a versatile programming language that is widely used in web development, data science, machine learning, and automation. It is known for its simple and readable syntax, which makes it easy to learn and understand.",
        "tags": ["python", "tutorial"],
        "published_date": "2025-11-18"
    }
    '''

    validator2 = AIValidator(BlogPost)
    result2 = validator2.validate(good_json)
    if result2:
        print(f"\n✓ Valid blog post: {result2.title}")
```

**Success Criteria**:
- Validation catches all errors in AI-generated data
- Retry logic improves prompts based on validation failures
- Metrics track success/failure rates
- Logging provides clear visibility into what's happening
- Production-ready error handling (graceful fallback after max retries)
- You understand the iterative refinement loop: generate → validate → analyze → improve → retry

---

## Time Estimate
**40-45 minutes** (8 min discover, 10 min AI teaches, 10 min you challenge, 12 min build)

