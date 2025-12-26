---
sidebar_position: 5
title: "Agent Skills in Code"
description: "Load organizational expertise into agents through settingSources and SKILL.md files. Understand how agent skills encode domain intelligence and make agents genuinely valuable."
keywords: [agent skills, settingSources, SKILL.md, organizational intelligence, domain expertise, reusable patterns]
chapter: 36
lesson: 5
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Configuring settingSources to Load Agent Skills"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can configure ClaudeAgentOptions with setting_sources and allowed_tools=['Skill'] to load skills from .claude/skills/ directory"

  - name: "Understanding SKILL.md Structure and Frontmatter"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can read and interpret SKILL.md files with YAML frontmatter (name, description, proficiency_level, category) and understand when skills activate"

  - name: "Creating Domain-Specific Agent Skills"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design and write a SKILL.md file that encodes organizational patterns for code review, API design, or testing workflows"

learning_objectives:
  - objective: "Configure agent to load skills from filesystem using setting_sources parameter"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code configuration task: Student writes ClaudeAgentOptions with correct setting_sources and tool settings"

  - objective: "Identify when SKILL.md files should be created and what content they should contain"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Reading task: Student reviews SKILL.md examples and explains when each would activate"

  - objective: "Design and implement a skill for a recurring workflow pattern"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Creation task: Student writes SKILL.md for domain-specific pattern with complete structure"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (settingSources, SKILL.md structure, YAML frontmatter, Skill tool, skill activation, organizational intelligence, reusability) within B1-B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design multi-layer skill system where skills reference other skills; implement skill versioning and namespace management; analyze how skills reduce hallucination in specialized domains"
  remedial_for_struggling: "Focus on SKILL.md basics first (name, description, one section of content); practice with provided skill examples before writing original skills; use decision framework: 'Would I use this pattern 3+ times?' → create skill"
---

# Agent Skills in Code

No other SDK has this: your agent inherits organizational expertise directly from your filesystem.

When you build agents with most SDKs, all domain knowledge lives in system prompts. Your prompt grows to thousands of tokens, becomes brittle to small changes, and can't adapt to different user contexts.

With Claude Agent SDK, something profound happens. Your agent can **load and apply specialized skills**—reusable intelligence encoded as SKILL.md files in your .claude/skills/ directory.

A skill isn't just documentation. It's an active intelligence that your agent can discover, evaluate, and apply autonomously. When your agent encounters a task, it recognizes applicable skills and activates them.

This transforms your agent from a general-purpose tool into a **specialized expert** that embodies your organization's knowledge, standards, and best practices.

## The Problem Skills Solve

Imagine you're building an agent for code review. Without skills:

```python
options = ClaudeAgentOptions(
    allowed_tools=["Read", "Bash", "Grep"],
    system_prompt="""You are a code review expert. When reviewing code:

    Security patterns to check:
    - SQL injection: All database queries must use parameterized statements
    - XSS: All user input must be sanitized before rendering
    - CSRF: All state-changing operations require CSRF tokens
    - Auth: All endpoints except login/register must verify JWT tokens
    - Rate limiting: All APIs must throttle requests (100/min default)
    [50 KB more security patterns...]

    Performance patterns to check:
    - N+1 queries: Avoid loops with database calls inside
    - Caching: Use Redis for frequently accessed data
    - Connection pooling: Reuse database connections
    [30 KB more performance patterns...]

    Testing patterns to check:
    - Unit test coverage: Aim for 80%+ coverage
    - Integration tests: Every API endpoint needs 2+ integration tests
    - Mocking: Mock external services, not internal functions
    [20 KB more testing patterns...]"""
)
```

The system prompt becomes a brittle knowledge dump. Changes require careful editing. Different customers have different standards—you'd need multiple prompts.

**With skills**, it becomes elegant:

```python
options = ClaudeAgentOptions(
    setting_sources=["project"],     # Load from .claude/skills/
    allowed_tools=["Skill", "Read", "Bash", "Grep"]
)
```

Your agent discovers three skills:
- `.claude/skills/security-review/SKILL.md` → Activates when reviewing authentication code
- `.claude/skills/performance-review/SKILL.md` → Activates when analyzing query patterns
- `.claude/skills/testing-standards/SKILL.md` → Activates when evaluating test coverage

Each skill can be updated independently. Different customers load different skill sets. Your agent becomes **domain-aware** rather than domain-confused.

## Setting Up settingSources

The `setting_sources` parameter tells your agent where to find organizational intelligence.

### Basic Configuration

```python
from claude_agent_sdk import query, ClaudeAgentOptions

options = ClaudeAgentOptions(
    setting_sources=["project"],      # Load from .claude/skills/ in current directory
    allowed_tools=["Skill", "Read", "Edit", "Bash"]
)

async for message in query(
    prompt="Review this code for security issues",
    options=options
):
    print(message)
```

**Output:**
```
Assistant: I'm analyzing this code against my security review skill...
[Agent activates .claude/skills/security-review/SKILL.md and applies patterns]
Review complete: Found 3 security issues in authentication handling
```

### How settingSources Works

The `setting_sources` parameter accepts three values:

| Option | Where It Looks | Use Case |
|--------|---|----------|
| `"user"` | `~/.claude/skills/` | Global skills available across all projects |
| `"project"` | `./.claude/skills/` in current working directory | Project-specific organizational standards |
| `["user", "project"]` | Both locations (project takes precedence) | Combined knowledge—global patterns + project customization |

When you specify `setting_sources=["project"]`, your agent loads all SKILL.md files from `.claude/skills/` subdirectories.

**Directory structure your agent understands:**

```
project-root/
├── .claude/
│   └── skills/
│       ├── code-review/
│       │   └── SKILL.md
│       ├── api-design/
│       │   └── SKILL.md
│       └── deployment-checklist/
│           └── SKILL.md
├── src/
└── tests/
```

When you enable `allowed_tools=["Skill"]`, your agent can **invoke and apply these skills** during task execution.

### Multi-Source Precedence

When loading from both "user" and "project":

```python
options = ClaudeAgentOptions(
    setting_sources=["user", "project"],  # Global skills + project skills
    allowed_tools=["Skill"]
)
```

**Precedence rule**: Project skills override user skills with the same name.

**Example**:
- `~/.claude/skills/code-review/SKILL.md` → Global standard (Django/REST patterns)
- `./.claude/skills/code-review/SKILL.md` → Your project overrides with FastAPI patterns
- Agent uses project version when both exist

This enables: Global best practices + project-specific customization.

## Understanding SKILL.md Structure

A skill is a markdown file with YAML frontmatter that teaches your agent specialized knowledge.

### Complete SKILL.md Example

```markdown
---
name: security-code-review
description: |
  Guidelines for security-focused code review. Activates when reviewing authentication,
  encryption, credential handling, or access control code.

proficiency_level: "B2"
category: "Security"
use_when: |
  - Reviewing authentication logic
  - Analyzing credential storage
  - Evaluating encryption implementations
  - Checking authorization patterns
---

# Security Code Review Skill

## What You're Protecting Against

Modern applications face 5 primary attack surfaces:

1. **Authentication**: Weak credential verification or session management
2. **Authorization**: Insufficient access controls allowing privilege escalation
3. **Data Protection**: Sensitive data exposed in transit or at rest
4. **Injection**: Malicious input executed as commands or queries
5. **Cryptography**: Weak algorithms or improper implementation

## Security Patterns for Review

### Pattern 1: Authentication Flow

When reviewing authentication code, verify:

**Questions to ask:**
- Are passwords hashed using bcrypt (12+ rounds) or Argon2?
- Are login endpoints rate-limited (max 5 attempts per hour)?
- Are sessions created with secure cookies (HttpOnly, Secure, SameSite)?
- Is there protection against CSRF (token validation on state changes)?

**What good looks like:**
```python
# Secure: Bcrypt with 12 rounds, enforced rate limiting
from flask_limiter import Limiter

@app.route('/login', methods=['POST'])
@limiter.limit("5 per hour")
def login():
    user = User.query.filter_by(email=request.json['email']).first()
    if user and bcrypt.check_password_hash(user.password_hash,
                                           request.json['password']):
        # Create secure session
        session.cookie_secure = True
        session.cookie_httponly = True
        session.cookie_samesite = 'Lax'
```

**Red flags:**
- Passwords stored plaintext or with MD5
- No rate limiting on login endpoints
- Sessions created with default/weak cookie settings
- Credentials passed in URL parameters

### Pattern 2: Authorization Checks

Verify access control is enforced consistently:

**Questions to ask:**
- Does every protected endpoint verify user permissions?
- Are permissions checked at function entry, not in business logic?
- Is there a consistent permission model (RBAC, ABAC)?
- Can users directly bypass checks via URL manipulation?

**What good looks like:**
```python
# Secure: Permission checked before operation
@app.route('/users/<user_id>/data', methods=['GET'])
def get_user_data(user_id):
    # Check permission FIRST
    if not current_user.has_permission('view_user_data', user_id):
        abort(403)
    # Then retrieve data
    return User.query.get(user_id).sensitive_data
```

**Red flags:**
- Permission checks inside conditional logic (can be bypassed)
- Mixed business logic with authorization decisions
- Different permission models across endpoints
- User IDs passed in URLs without validation

### Pattern 3: Data Protection

Sensitive data must be protected in transit and at rest.

**Questions to ask:**
- Are all API calls using HTTPS/TLS 1.2+?
- Are sensitive fields encrypted at rest (PII, financial data, secrets)?
- Are encryption keys managed separately from code?
- Are secrets (API keys, database passwords) excluded from version control?

**What good looks like:**
```python
# Secure: Encryption at rest + separate key management
from cryptography.fernet import Fernet
import os

encryption_key = os.environ['ENCRYPTION_KEY']  # From secure store
cipher = Fernet(encryption_key)

# Store encrypted
user.ssn_encrypted = cipher.encrypt(user.ssn.encode())
db.session.commit()

# Retrieve and decrypt
decrypted_ssn = cipher.decrypt(user.ssn_encrypted)
```

**Red flags:**
- Sensitive data stored plaintext in database
- Encryption keys embedded in code
- Secrets in version control history
- API calls over HTTP (not HTTPS)

## Activation Examples

### Example 1: Agent Reviews Authentication Module

**Agent prompt:**
"Review src/auth.py for security issues"

**Agent reasoning:**
1. Agent reads src/auth.py
2. Recognizes password hashing, session creation → Authentication context
3. Loads security-code-review skill
4. Applies authentication patterns (bcrypt, rate limiting, secure cookies)
5. Reports: "Authentication looks secure. Sessions use HttpOnly cookies. But login endpoint missing rate limiting."

### Example 2: Agent Reviews API Authorization

**Agent prompt:**
"Analyze src/api/endpoints.py for authorization gaps"

**Agent reasoning:**
1. Agent reads src/api/endpoints.py
2. Recognizes permission checks, role-based access → Authorization context
3. Loads security-code-review skill
4. Applies authorization patterns (permission checks, consistent model)
5. Reports: "3 endpoints missing permission verification before data access"

## When to Create This Skill

Create a security-code-review skill when your organization:

- Has repeated security issues in code reviews
- Needs consistent security standards across projects
- Wants specialized security review agents
- Trains new developers on security patterns
- Must comply with security frameworks (PCI DSS, HIPAA, SOC 2)

---

# Skills vs. System Prompts: The Key Insight

When you add knowledge to your agent, you have two options:

### Option A: System Prompt (Old Way)

```python
options = ClaudeAgentOptions(
    system_prompt="""You are a code review expert...
    [All knowledge embedded in prompt]"""
)
```

**Problems:**
- Prompt becomes thousands of tokens (expensive, slow)
- Can't selectively enable/disable patterns
- Different customers need different expertise
- Updating patterns requires editing system prompt carefully
- No clear organization of domain knowledge

### Option B: Skills Ecosystem (New Way)

```python
options = ClaudeAgentOptions(
    setting_sources=["project"],
    allowed_tools=["Skill"]
)
# Agent discovers and applies skills autonomously
```

**Advantages:**
- Each skill is independent file (easy to update)
- Agent decides which skills apply (contextual activation)
- Different customers load different skill sets
- Organize expertise by domain (security, performance, testing)
- Skills compose—one skill can reference another

**The insight**: As your agents become more specialized, **skills become your competitive advantage**. They encode your organizational knowledge in a form that AI agents can discover and apply.

This is what makes agents built with Claude SDK genuinely valuable as Digital FTEs—they inherit years of your organization's accumulated expertise.

## Creating Your First Skill

Let's create a simple skill together: testing-standards.

### Step 1: Create the Directory

```bash
mkdir -p .claude/skills/testing-standards
```

### Step 2: Write SKILL.md

```markdown
---
name: testing-standards
description: |
  Guidelines for evaluating test quality and coverage. Activates when analyzing
  test suites, validating test design, or assessing code coverage.
proficiency_level: "B1"
category: "Testing"
---

# Testing Standards Skill

## Code Coverage Targets

Aim for these coverage minimums:

- **Libraries**: 85%+ line coverage
- **Business logic**: 80%+ coverage
- **Infrastructure code**: 50%+ coverage (integration tests sufficient)

Don't optimize for coverage percentage. Optimize for:
- Critical paths fully tested
- Error cases validated
- Integration points verified

## Test Organization

Structure tests by type:

```
tests/
├── unit/          # Test single functions in isolation
├── integration/   # Test components working together
└── e2e/          # Test full workflows end-to-end
```

## What Good Tests Look Like

Each test should:

1. **Have a clear name** describing what it tests
   ```python
   # Good
   def test_login_with_invalid_password_returns_401():

   # Bad
   def test_login():
   ```

2. **Test one behavior**
   ```python
   # Good: Tests password validation only
   def test_login_rejects_incorrect_password():
       user = create_user(password="correct_password")
       result = login("user@example.com", "wrong_password")
       assert result.status_code == 401

   # Bad: Tests multiple behaviors in one test
   def test_login_flow():
       user = create_user()
       result = login(...)
       assert result.status_code == 200
       assert 'token' in result.json
       assert cache.get(...) == result.json['token']
   ```

3. **Be deterministic** (same input = same output always)
   ```python
   # Good: Uses fixed test data
   def test_user_created_at_timestamp():
       with freeze_time("2025-01-01 12:00:00"):
           user = create_user(name="Alice")
           assert user.created_at == datetime(2025, 1, 1, 12, 0, 0)

   # Bad: Non-deterministic (might fail randomly)
   def test_user_timestamp():
       user = create_user()  # Uses current time, unpredictable
       assert user.created_at is not None
   ```

## When to Create Tests

- **Before implementation**: Test-driven development (recommended)
- **After critical functions**: Always test auth, payments, data validation
- **Before refactoring**: Ensure current behavior is tested
- **For bugs**: Write test that reproduces bug, then fix

---
```

### Step 3: Enable the Skill

```python
options = ClaudeAgentOptions(
    setting_sources=["project"],
    allowed_tools=["Skill", "Read", "Grep", "Bash"]
)

async for message in query(
    prompt="Evaluate the test suite in tests/ for coverage and quality",
    options=options
):
    print(message)
```

**Output:**
```
Agent: Analyzing test suite against testing-standards skill...

[Agent activates .claude/skills/testing-standards/SKILL.md]

Coverage analysis:
- Unit tests: 82% (good)
- Integration tests: 45% (could be better)

Issues found:
1. Login tests don't validate rate limiting
2. Payment API tests lack error case coverage
3. Database transaction tests missing rollback verification

Recommendations:
- Add rate limiting tests
- Expand payment API error cases
- Verify transaction isolation
```

## Try With AI

**Setup:** You'll create a skill for your domain, then test it with an agent.

### Prompt 1: Design a Skill for Your Domain

Ask Claude to help you outline a skill:

```
I want to create an agent skill for [your domain].
My team repeatedly faces these challenges:
[list 3-5 challenges]

Help me design the SKILL.md file structure including:
- Clear name and description
- Key patterns and questions
- Real-world examples
- Activation triggers

What should this skill teach the agent?
```

**What you're learning**: How to identify knowledge worth encoding as a skill. Not every pattern deserves a skill—only those that recur and compound over time.

### Prompt 2: Write Your First Skill

Based on Claude's outline, ask:

```
Write the complete SKILL.md file for [skill-name] skill.

Include:
- YAML frontmatter with name, description, proficiency_level
- 2-3 major patterns with real code examples
- Questions to ask when evaluating this pattern
- Red flags and anti-patterns
- When to activate this skill

Make it specific to [your domain], not generic.
```

**What you're learning**: How to structure knowledge so agents can apply it contextually. Good skills are specific (not "write good code") but generalizable (work across multiple projects).

### Prompt 3: Test Skill Activation

After creating your skill, ask:

```
Create a Python script that:

1. Configures ClaudeAgentOptions with setting_sources=["project"]
2. Loads my new [skill-name] skill
3. Runs a query() call that would trigger the skill
4. Prints the agent's response

Show me how the agent would use this skill in practice.

Also: What edge cases might cause the skill to NOT activate?
```

**What you're learning**: How skills activate in real agents. Understanding when your skill applies (and doesn't) determines whether agents use it effectively.

