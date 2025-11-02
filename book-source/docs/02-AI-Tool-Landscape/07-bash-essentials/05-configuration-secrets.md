---
sidebar_position: 5
title: "Understanding Configuration and Secrets Safely"
chapter: 7
lesson: 5
duration_minutes: 40

skills:
  - name: "Environment Variable Configuration"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner understands temporary (export) vs. persistent (~/.bashrc, .env) configuration through dialogue"

  - name: "Secrets Management Security"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner never hardcodes secrets; understands passwords/keys belong in environment, not code"

  - name: "Configuration Verification"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner can verify configuration is set correctly and accessible to the right places"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (environment variables, export, .bashrc, .env, secrets) at A2 max limit ✓"
---

# Understanding Configuration and Secrets Safely

## Passwords Should Never Go In Your Code

Imagine giving your front door key to a locksmith who puts it in a labeled envelope on your doorstep: "Smith Family House Key—Keep This Safe!" That's what happens when you hardcode passwords or API keys in your code. If someone gets your code, they get your secrets.

This lesson teaches you how to keep secrets safe. Configuration (like API keys, database passwords, and tokens) should live in environment variables or configuration files, never in code.

By the end of this lesson, you'll:
1. **Understand temporary configuration** (export commands that last while terminal is open)
2. **Understand persistent configuration** (settings that survive closing the terminal)
3. **Never hardcode secrets** into your code (security mindset)

---

## Example 1: Temporary Configuration (Sessions)

When you want to set a value temporarily (just for your current terminal session), you use `export`.

**You**: "I need to set an API key so my Python script can access the API. But I don't want to hardcode it in the code."

**Agent**: "Smart. I'll set the API key as an environment variable using `export`. This keeps it in your terminal session, not in code."

```
$ export API_KEY="sk-abc123def456ghi789"
$ echo $API_KEY
sk-abc123def456ghi789
```

**Agent**: "I've set the API_KEY environment variable. Your script can now read it using code like `os.getenv('API_KEY')`. Notice: the key is NOT in any file—it's only in this terminal session. When you close the terminal, it's gone."

**You**: "So this is temporary. If I close the terminal and reopen it, the key won't be set anymore?"

**Agent**: "Exactly. That's the limitation of `export`. It's useful for testing, but for persistent configuration, we need a different approach."

---

## Understanding Temporary Configuration

That dialogue shows something important:

**Temporary configuration with `export`**:
- Exists only in the current terminal session
- Perfect for testing or temporary values
- Disappears when you close the terminal
- Useful for: API keys during development, test credentials, temporary paths

**When to use temporary configuration**:
- You're testing an API key and will change it soon
- You're working on a single machine temporarily
- You want to avoid persisting sensitive values

---

## Example 2: Persistent Configuration (.bashrc and .env)

For values that need to survive terminal restarts, you store them in configuration files.

**You**: "I need my API key to be available every time I open a terminal, but I still don't want it in my code."

**Agent**: "I'll show you two approaches:

1. **Store in ~/.bashrc** (persistent across terminal sessions):
   This is a configuration file that runs every time you open a new terminal. We can put `export API_KEY="..."` there.

2. **Store in a .env file** (common in Python and Node.js projects):
   This is a file in your project folder that your code loads. It keeps secrets out of code but separate from system configuration."

**You**: "What's the difference? Which should I use?"

**Agent**: "Good question. Let me show you both:

**Option 1: ~/.bashrc (system-wide)**

```
$ nano ~/.bashrc
# Add this line to the end:
export API_KEY="sk-abc123def456ghi789"

$ source ~/.bashrc
$ echo $API_KEY
sk-abc123def456ghi789
```

This sets the API_KEY for every terminal session on your system.

**Option 2: .env file in your project (project-specific)**

```
$ cat > .env << 'EOF'
API_KEY=sk-abc123def456ghi789
DATABASE_URL=postgres://user:pass@localhost/db
EOF

$ cat .env
API_KEY=sk-abc123def456ghi789
DATABASE_URL=postgres://user:pass@localhost/db
```

Then in your Python code:
```python
from dotenv import load_dotenv
import os

load_dotenv()  # Load variables from .env
api_key = os.getenv('API_KEY')
```

**Use ~/.bashrc** when: You want system-wide settings (PATH, common tools)
**Use .env** when: You want project-specific secrets (API keys, database credentials)"

**You**: "Which is safer?"

**Agent**: "Both are safer than hardcoding. But here's the critical safety rule: **NEVER commit .env files to GitHub**. Add `.env` to your `.gitignore` so it never gets shared. ~/.bashrc is on your local machine only, so it's naturally safer from accidental sharing."

---

## The Critical Security Principle

This dialogue reveals the most important lesson:

**NEVER hardcode secrets.**

Instead:
- Read from environment variables: `os.getenv('API_KEY')`
- Read from .env files: Use libraries like `python-dotenv`
- Store in system configuration: ~/.bashrc
- Never put passwords, tokens, or keys in your code

**Why this matters**:
- If you share code (GitHub, email, etc.), your secrets are exposed
- If someone gets your repository, they get all your keys
- If you hardcode credentials, attackers can use them to access your systems

---

## Example 3: Why Hardcoding Is Dangerous

Let's see what NOT to do:

**Bad code** (DO NOT DO THIS):
```python
# DANGEROUS! Never hardcode secrets!
API_KEY = "sk-abc123def456ghi789"
DATABASE_PASSWORD = "MySecretPassword123"
```

**What goes wrong**:
1. You commit this to GitHub
2. Someone finds your repository
3. They use your API key and database password to access your systems
4. Now your account is compromised

**Good code** (DO THIS):
```python
import os
from dotenv import load_dotenv

load_dotenv()  # Load from .env file
api_key = os.getenv('API_KEY')
db_password = os.getenv('DATABASE_PASSWORD')

# Never print or log these values!
```

Your .env file (never committed):
```
API_KEY=sk-abc123def456ghi789
DATABASE_PASSWORD=MySecretPassword123
```

Now your secrets are external. If someone gets your code, they only get the template, not the actual values.

---

## Example 4: Verification—Making Sure Configuration Works

After setting configuration, you MUST verify it's accessible.

**You**: "I've set up environment variables. How do I verify they're working?"

**Agent**: "Let me show you verification commands:

For environment variables:
```
$ echo $API_KEY
sk-abc123def456ghi789
```

For .env files, I'll create a test script:
```python
import os
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv('API_KEY')
if api_key:
    print(f'✓ API_KEY is set (first 10 chars: {api_key[:10]}...')')
else:
    print('✗ API_KEY is NOT set. Check your .env file.')
```

Running the test:
```
$ python test_config.py
✓ API_KEY is set (first 10 chars: sk-abc123d...)
```

This verifies your configuration works without printing the full secret."

**You**: "Why not print the full key?"

**Agent**: "Security. Never log or print secrets. You only verify they exist, not their full value. This prevents accidental exposure in log files."

---

## Configuration: Temporary vs. Persistent

Here's the comparison:

| Approach | Persistence | Scope | Use Case |
|----------|------------|-------|----------|
| `export` | Session only | Current terminal | Testing, temporary values |
| ~/.bashrc | Permanent | All your terminals | System-wide settings (PATH) |
| .env file | Permanent | Single project | Project-specific secrets |
| Code (NEVER!) | Permanent | Everywhere | DANGEROUS—Never do this |

---

## Exercise 1: Identify Safe vs. Unsafe Configuration

Read these code examples and mark them SAFE or UNSAFE:

**Example 1**:
```python
import os
api_key = os.getenv('API_KEY')
```
- SAFE ✓ (reads from environment, not hardcoded)

**Example 2**:
```python
API_KEY = "sk-abc123def456ghi789"
response = requests.get(url, headers={'Authorization': API_KEY})
```
- UNSAFE ✗ (hardcoded in code)

**Example 3**:
```python
from dotenv import load_dotenv
import os
load_dotenv()
db_password = os.getenv('DATABASE_PASSWORD')
```
- SAFE ✓ (loads from .env file)

**Example 4**:
```python
DATABASE_PASSWORD = "MySecretPassword123"
connection = database.connect(password=DATABASE_PASSWORD)
```
- UNSAFE ✗ (hardcoded in code)

**Why Examples 1 and 3 are safe**:
- Secrets come from environment, not code
- Even if code is shared, secrets aren't exposed
- Easy to change secrets without changing code

---

## Exercise 2: Configuration Verification Plan

Your AI sets up an API key. Write what commands/checks should verify it works:

**Your configuration setup dialogue**:

```
You: "Set up my Stripe API key safely. I'll need it for payment processing."

Agent: "I'll add it to your .env file and verify it works."
```

**Write 3 verification steps you should request**:

1. "Show me the .env file exists and has the key" (read the file)

2. Your verification:

3. Your verification:

---

**Model verification steps**:
1. Verify the .env file exists: `ls -la .env`
2. Verify the key is in the file: `grep STRIPE_API_KEY .env`
3. Verify your code can read it: Run a test script that loads and confirms the variable exists
4. Verify .env is in .gitignore: `cat .gitignore | grep .env`

---

## Exercise 3: Configuration Planning

You have three values to configure:

1. **API key for a payment service** (sensitive, changes rarely)
2. **Database password** (sensitive, different for dev/production)
3. **Project path** (not sensitive, system-specific)

For each, decide: Should this go in `~/.bashrc`, `.env file`, or neither?

**Your answers**:

1. API key: `.env file` (project-specific, sensitive, shouldn't commit)
2. Database password:
3. Project path:

---

**Model answers**:
1. API key → `.env file` (project-specific, never commit)
2. Database password → `.env file` (project-specific, must not hardcode)
3. Project path → `~/.bashrc` (system-specific, not sensitive, can be public)

---

## Formative Assessment: Configuration Concepts

**Question 1**: What's the main difference between `export API_KEY="..."` and storing it in .env?
- A) `export` is temporary (session only); .env is persistent (survives terminal restart)
- B) `export` is for passwords; .env is for everything
- C) They're the same

*Correct: A. `export` is temporary; .env is permanent.*

**Question 2**: What should you NEVER do with API keys?
- A) Use them in your code (hardcode them)
- B) Store them in environment variables
- C) Load them from .env files

*Correct: A. NEVER hardcode secrets in code.*

**Question 3**: Why should .env files never be committed to GitHub?
- A) They make your repository bigger
- B) They contain secrets that would be exposed to anyone with repository access
- C) GitHub doesn't allow .env files

*Correct: B. .env files contain secrets; committing them exposes them.*

---

## Summative Assessment: Configuration with Verification

Have a real conversation with your AI where you:

1. **Ask your AI to set up an API key or sensitive value** (use a test key, not real credentials)
2. **Request it be stored safely** (in .env, not hardcoded)
3. **Have your AI show the setup commands**
4. **Ask verification questions**: "How do I verify this is set? How will my code access it?"
5. **Verify it works** by running a test
6. **Confirm .env is in .gitignore** so it won't be accidentally committed

**Success criteria**:
- Configuration is stored externally (not in code)
- You can verify the configuration is set
- You understand how your code will access the value
- You know how to prevent accidental exposure (via .gitignore)

---

## Try With AI

**Tool**: Claude Code, ChatGPT Code Interpreter, Gemini CLI, or your preferred AI companion

**Setup**: You're going to set up configuration safely and verify it works.

### Prompt 1: Set Up Configuration Safely

Copy and paste this prompt:

```
I need to set up configuration for my project.
I have an API key (use a test value like "test-key-12345") that my code needs.
Show me:
1. How to store it in a .env file safely
2. How to load it in Python code without hardcoding
3. How to verify it's accessible
4. How to keep it out of version control (.gitignore)

Don't hardcode the key anywhere.
```

**Expected Outcome**:
- Your AI shows you how to create a .env file
- Shows you Python code that loads from environment (not hardcoded)
- Provides verification commands
- Explains .gitignore configuration

### Prompt 2: Security Check

```
I'm worried I might accidentally commit my .env file to GitHub.
Show me:
1. How to verify .env is in .gitignore
2. What happens if someone finds my .env file
3. How to recover if secrets are exposed

Make it practical, not theoretical.
```

**Expected Outcome**: Your AI explains real security consequences and practical recovery steps

### Prompt 3: Temporary vs. Persistent

```
When should I use 'export' command vs. .env file?
Give me a real scenario for each:
- When I'd use 'export' (temporary)
- When I'd use .env (persistent)

Show the commands for both approaches.
```

**Expected Outcome**: You understand the tradeoffs and when to use each approach
