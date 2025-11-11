---
sidebar_position: 8
title: "Real Project Setup and Troubleshooting"
chapter: 7
lesson: 8
duration_minutes: 50

skills:
  - name: "Project Setup and Scaffolding"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information"
    measurable_at_this_level: "Learner can set up project structure with AI guidance; create folders and files; install dependencies"

  - name: "Error Interpretation and Troubleshooting"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Learner encounters real errors; asks AI for clarification; understands error messages; follows debugging steps"

  - name: "Integrated Bash Workflow"
    proficiency_level: "A2→B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Learner integrates navigation, safety pattern, file ops, configuration, packages, pipelines in realistic workflow"

  - name: "Resilience and Problem-Solving Mindset"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Learner recognizes errors as learning opportunities; develops confidence debugging with AI"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (project structure, error messages, troubleshooting, debugging, logs, testing, deployment) within B1 transition limit ✓"
---

# Real Project Setup and Troubleshooting—The Capstone

## This Lesson Brings Everything Together

You've learned individual concepts:
- How to navigate (paths, directories)
- How to operate on files safely (5-step pattern)
- How to configure projects (environment variables)
- How to install packages (dependencies)
- How to build complex operations (pipes)

This capstone lesson integrates everything into a realistic workflow. You'll set up a real project with your AI, encounter real errors, and troubleshoot them together.

This is where learning transforms into **capability**. You're not learning bash—you're learning to supervise real project work.

---

## The Real Workflow: Setup → Error → Debug → Succeed

In professional work, this is the pattern:

**Step 1: Set up project** (create structure, install dependencies)
**Step 2: Encounter an error** (something doesn't work)
**Step 3: Interpret the error** (understand what went wrong)
**Step 4: Troubleshoot with AI** (ask questions, follow steps)
**Step 5: Verify success** (confirm the fix worked)
**Step 6: Document the solution** (remember what you learned)

This lesson guides you through all 6 steps.

---

## Set Up a Real Project Structure

Let's watch a realistic conversation where you and your AI set up a Python project from scratch.

**You**: "I want to create a new Python project to fetch weather data and save it to a file. Can you help me set it up?"

**Agent**: "Absolutely. I'll create the project structure step by step. Let me start with the directories and files we'll need."

**Step 1: Create project structure**

```
$ mkdir weather-project
$ cd weather-project
$ mkdir src tests data
$ ls -la
drwxr-xr-x   4 user  staff    128 Nov  2 10:00 src
drwxr-xr-x   3 user  staff     96 Nov  2 10:00 tests
drwxr-xr-x   3 user  staff     96 Nov  2 10:00 data
```

**Agent**: "I've created the project folder with subdirectories:
- `src/` for source code
- `tests/` for test files
- `data/` for output data files"

**Step 2: Create initial files**

```
$ cat > src/main.py << 'EOF'
import requests
import json
import os

api_key = os.getenv('WEATHER_API_KEY')
if not api_key:
    raise ValueError("WEATHER_API_KEY environment variable not set")

response = requests.get(
    f'https://api.openweathermap.org/data/2.5/weather?q=London&appid={api_key}'
)
data = response.json()
print(json.dumps(data, indent=2))
EOF

$ cat > README.md << 'EOF'
# Weather Project

Fetches current weather data and saves to file.

## Setup

1. Install dependencies: `pip install -r requirements.txt`
2. Set API key: `export WEATHER_API_KEY="your-key-here"`
3. Run: `python src/main.py`
EOF

$ cat > .env << 'EOF'
WEATHER_API_KEY=test-key-12345
EOF

$ cat > .gitignore << 'EOF'
.env
__pycache__/
*.pyc
EOF
```

**Agent**: "I've created the main project files. Now let's install dependencies."

**Step 3: Install dependencies**

```
$ cat > requirements.txt << 'EOF'
requests==2.31.0
python-dotenv==1.0.0
EOF

$ pip install -r requirements.txt
Collecting requests==2.31.0
...
Successfully installed requests-2.31.0 python-dotenv-1.0.0
```

**Agent**: "Dependencies installed. The project is ready to test. But first, let me check we have everything..."

**Step 4: Verify setup**

```
$ ls -la
total 48
-rw-r--r--   1 user  staff   1500 Nov  2 10:00 README.md
-rw-r--r--   1 user  staff     150 Nov  2 10:00 .env
-rw-r--r--   1 user  staff      60 Nov  2 10:00 .gitignore
-rw-r--r--   1 user  staff     300 Nov  2 10:00 requirements.txt
drwxr-xr-x   3 user  staff     96 Nov  2 10:00 src
drwxr-xr-x   3 user  staff     96 Nov  2 10:00 tests
drwxr-xr-x   3 user  staff     96 Nov  2 10:00 data

$ pip list | grep -E "requests|dotenv"
python-dotenv   1.0.0
requests        2.31.0
```

**Agent**: "Setup complete. The project structure is ready, dependencies are installed, and configuration is set."

---

## Encounter and Interpret Real Errors

Now let's test the project and encounter a realistic error.

**You**: "Let's run the script and see if it works."

**Agent**: "Running the main script..."

```
$ python src/main.py
Traceback (most recent call last):
  File "/Users/mjs/weather-project/src/main.py", line 8, in <module>
    response = requests.get(
  File "/Users/mjs/weather-project/src/main.py", line 9, in <module>
    f'https://api.openweathermap.org/data/2.5/weather?q=London&appid={api_key}'
requests.exceptions.ConnectionError: HTTPSConnectionPool(host='api.openweathermap.org', port=443):
Max retries exceeded with url: /api.openweathermap.org/data/2.5/weather?q=London&appid=test-key-12345
(Caused by NewConnectionError('<urllib3.connection.HTTPSConnection object at 0x...>'))
```

**You**: "Oh no! There's an error. What does this mean?"

**Agent**: "Let me break down the error:

1. **Error type**: `requests.exceptions.ConnectionError`
2. **What it means**: The code tried to connect to the weather API but failed
3. **Why it failed**: Either no internet connection, or the URL is wrong, or the API server is down

Looking at the error, the API key in the URL shows `test-key-12345`, which is our test value. The real issue is: we're using a test API key that the service doesn't recognize, so the request fails. This is expected because we haven't set up a real API key yet."

**You**: "So the error is expected? The code is working?"

**Agent**: "The error is expected because we used a test key. The code structure is correct. To make it work with a real API, you'd:
1. Get a real API key from openweathermap.org
2. Set it: `export WEATHER_API_KEY='your-real-key'`
3. Run the script again

Let me modify the code to handle this more gracefully."

---

## Debug and Fix Errors with AI Guidance

Let's fix the code to handle errors better.

**You**: "How would you make the error message clearer?"

**Agent**: "I'll modify the code to catch the connection error and provide helpful feedback:"

```
$ cat > src/main.py << 'EOF'
import requests
import json
import os
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv('WEATHER_API_KEY')

if not api_key:
    print("ERROR: WEATHER_API_KEY environment variable not set")
    print("Set it with: export WEATHER_API_KEY='your-key-here'")
    exit(1)

try:
    response = requests.get(
        f'https://api.openweathermap.org/data/2.5/weather?q=London&appid={api_key}',
        timeout=5
    )
    if response.status_code == 401:
        print("ERROR: Invalid API key. Check your WEATHER_API_KEY.")
        print(f"Response: {response.json()}")
    elif response.status_code == 200:
        data = response.json()
        print(json.dumps(data, indent=2))
    else:
        print(f"ERROR: API returned status {response.status_code}")

except requests.exceptions.Timeout:
    print("ERROR: Request timed out. Check your internet connection.")
except requests.exceptions.ConnectionError:
    print("ERROR: Cannot connect to API server. Check internet connection or API status.")
except Exception as e:
    print(f"ERROR: Unexpected error: {e}")
EOF

$ python src/main.py
ERROR: Invalid API key. Check your WEATHER_API_KEY.
Response: {'cod': '401', 'message': 'Invalid API key. Please see http://openweathermap.org/faq#error401 for more info.'}
```

**Agent**: "Much better! Now the error message is clear: the API key is invalid. When you get a real key and set it, the script will work."

**You**: "So you added error handling so the user knows what went wrong instead of seeing a cryptic traceback?"

**Agent**: "Exactly. Error handling makes debugging easier. Users (and you, when troubleshooting) get clear messages about what's wrong and how to fix it."

---

## Integrate All Concepts in a Real Workflow

Now let's add complexity that integrates everything you've learned.

**You**: "I want the script to save weather data to a file, with different output locations for development vs. production. How do I set that up?"

**Agent**: "I'll modify .env and the code to support both environments. This integrates configuration, file operations, and error handling."

**First, update .env**:

```
$ cat > .env << 'EOF'
WEATHER_API_KEY=test-key-12345
ENVIRONMENT=development
OUTPUT_FILE=data/weather.json
EOF
```

**Then update the code**:

```
$ cat > src/main.py << 'EOF'
import requests
import json
import os
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv('WEATHER_API_KEY')
environment = os.getenv('ENVIRONMENT', 'development')
output_file = os.getenv('OUTPUT_FILE', 'data/weather.json')

if not api_key:
    print("ERROR: WEATHER_API_KEY not set")
    exit(1)

try:
    response = requests.get(
        f'https://api.openweathermap.org/data/2.5/weather?q=London&appid={api_key}',
        timeout=5
    )

    if response.status_code == 200:
        data = response.json()

        # Save to file
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"✓ Weather data saved to {output_file}")
        print(f"✓ Environment: {environment}")
    else:
        print(f"ERROR: API returned {response.status_code}")

except requests.exceptions.ConnectionError:
    print("ERROR: Cannot connect to API")
except IOError as e:
    print(f"ERROR: Cannot write to {output_file}: {e}")
EOF

$ python src/main.py
✓ Weather data saved to data/weather.json
✓ Environment: development
```

**Agent**: "The script now:
1. Reads configuration from .env (configuration lesson)
2. Validates input and handles errors (troubleshooting)
3. Saves output safely to the correct file (file operations with error handling)
4. Works differently based on environment settings (applied configuration)"

---

## Understanding Error Messages

Real errors fall into categories. Here's how to interpret them:

| Error Type | Example | What It Means | First Step |
|------------|---------|---------------|-----------|
| **Import Error** | `ModuleNotFoundError: No module named 'requests'` | Package not installed | `pip install requests` |
| **Syntax Error** | `SyntaxError: invalid syntax` | Typo in code | Check the code on that line |
| **File Error** | `FileNotFoundError: [Errno 2] No such file` | File/folder doesn't exist | Check the path |
| **Connection Error** | `ConnectionError: Max retries exceeded` | Can't reach external service | Check internet, URL, credentials |
| **Permission Error** | `PermissionError: [Errno 13] Permission denied` | No permission to read/write | Check file permissions, sudo |

**Your job**: Read the error type and message, then ask your AI "What does this mean?" before trying random fixes.

---

## Lesson in Practice: A Complete Session

Let's watch a realistic troubleshooting conversation:

**You**: "I'm trying to run my Python script but it's not working. The error says something about a module."

**Agent**: "Show me the error message."

**You**: (Copies error)
```
ModuleNotFoundError: No module named 'requests'
```

**Agent**: "The module `requests` isn't installed. Let me check what's in requirements.txt and install it."

```
$ cat requirements.txt
requests==2.31.0
python-dotenv==1.0.0

$ pip install -r requirements.txt
```

**Agent**: "Installed. Now let's test the script again."

**You**: (Runs script, gets different error)
```
FileNotFoundError: [Errno 2] No such file or directory: 'data/weather.json'
```

**Agent**: "The script is trying to write to `data/weather.json` but the `data` folder doesn't exist. Let me create it:"

```
$ mkdir -p data
$ python src/main.py
ERROR: Invalid API key...
```

**Agent**: "Good! Now we're getting the expected API key error (because we used a test key). The infrastructure is working. You'd set a real API key to complete setup."

**You**: "So we went from 'module missing' → 'folder missing' → 'API key missing'. Each error was real?"

**Agent**: "Yes. Each revealed a real setup issue. The workflow is:
1. Hit error
2. Read error message
3. Ask 'What does this mean?'
4. Fix one issue
5. Run again, hit next issue
6. Repeat until success

This is completely normal. Troubleshooting is an essential skill."

---

## The Mindset: Errors Are Learning

**Important**: In professional development, encountering errors is not failure. It's progress.

- **No error** → Your code never ran (missing setup)
- **Error** → Your code ran, hit a problem, produced a useful message
- **Debugging error** → You understand what's wrong and how to fix it

Each error message is your AI telling you what to fix next.

---

## Try With AI: Real Project Setup and Troubleshooting

Now that you understand the complete project setup workflow, let's apply it with your AI.

### Full Project Setup Prompt

Open your AI tool and ask:

**Prompt:**
```
Help me set up a complete Python project.
Project: A simple weather data fetcher.

Walk me through:
1. Project structure (folders and files)
2. Code to implement the functionality
3. Dependencies to install
4. How to run it

After setup, I'll run it and see what errors appear.
I want to understand the full workflow: setup → run → error → debug → success.
```

**What to Expect**:
- Your AI creates the project structure
- Shows dependencies and installs them
- You run the project and encounter real errors
- Your AI explains each error and guides you through fixing it
- By the end, you have a working project

**Key Insight**: This is the real developer workflow. Errors are normal. The skill is understanding and fixing them with AI guidance.

---

## Try With AI: Troubleshooting Real Errors

When your project encounters errors, ask your AI:

**Prompt:**
```
I got this error: [paste the error message]

Walk me through:
1. What does this error mean in plain language?
2. What caused it?
3. How do I fix it?
4. After I fix it, what should I try next?
```

**Expected Response**:
Your AI will explain the error, show you the fix, and help you understand why the error occurred. This builds your debugging confidence.

**Key Principle**: Every error is a learning opportunity. Your AI is your guide through the troubleshooting process.

---

You can now supervise sophisticated bash workflows. Your AI is the expert in commands and syntax. **You're the expert in understanding, verifying, and catching mistakes.**

This is the foundation of productive collaboration with AI tools.

---



<Quiz title="Chapter 8 Quiz" questions={[{"question":"Which of the following is NOT listed as a traditional \u0027gatekeeper\u0027 to learning software development that AI is now helping to remove?","options":{"a":"Syntax Memorization","b":"The need for creative problem-solving","c":"Debugging Cryptic Errors","d":"Configuration Hell"},"correct_answer":"b","explanation":"The chapter lists Syntax Memorization, Debugging Cryptic Errors, and Configuration Hell as barriers being removed. Creative problem-solving, however, is a skill that becomes more important."},{"question":"Why do experienced developers often see greater productivity gains with AI tools compared to beginners?","options":{"a":"Because they can type faster than the AI.","b":"Because their experience allows them to provide more precise specifications and better evaluate the AI\u0027s output.","c":"Because the AI tools are exclusively designed for them.","d":"Because they refuse to use the AI tools and work manually."},"correct_answer":"b","explanation":"The text states that experienced developers know \u0027What to ask for,\u0027 \u0027What to look for,\u0027 and \u0027What matters,\u0027 which allows them to leverage AI more effectively."},{"question":"According to the technology adoption curve shown in the chapter, where are AI coding tools currently positioned?","options":{"a":"Innovators","b":"Early Adopters","c":"Transitioning from Early Adopters to Early Majority","d":"Laggards"},"correct_answer":"c","explanation":"The chapter explicitly states: \u0027AI coding tools are currently transitioning from Early Adopters to Early Majority. → WE ARE HERE ←\u0027"},{"question":"For people with deep expertise in non-technical domains (like healthcare or finance), what is the key opportunity presented by AI coding tools?","options":{"a":"They can now easily switch to a career in AI research.","b":"Their domain expertise is no longer valuable.","c":"They can now build software solutions for their field without needing to become full-time programmers, as AI handles the coding.","d":"They must now get a traditional computer science degree."},"correct_answer":"c","explanation":"The chapter highlights this opportunity: \u0027AI handles coding; you provide domain expertise. The combination is incredibly valuable... AI doesn\u0027t replace domain expertise. It enables domain experts to build solutions...\u0027"},{"question":"What is the chapter\u0027s main argument for why \u0027this is the best time in three to four decades to be learning software development\u0027?","options":{"a":"Because developer salaries are the highest they have ever been.","b":"Because the barriers to entry have been dramatically lowered by AI, while the opportunity for growth is massive.","c":"Because there are fewer developers now, meaning less competition.","d":"Because universities have finally caught up and are teaching all the necessary skills."},"correct_answer":"b","explanation":"The central theme of the chapter is that the combination of falling barriers (syntax, debugging, etc.) and being in the \u0027Early Majority\u0027 phase of a massive technological shift creates a unique and powerful opportunity window."}]} />

