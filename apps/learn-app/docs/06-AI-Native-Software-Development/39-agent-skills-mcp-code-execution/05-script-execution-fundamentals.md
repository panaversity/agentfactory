---
sidebar_position: 5
title: "Script Execution Fundamentals"
description: "Master the write-execute-analyze loop that powers autonomous code execution. Learn to design skills that generate code, run it, detect and fix errors, and iterate toward complete solutions."
keywords: ["script execution", "write-execute-analyze loop", "error recovery", "syntax errors", "runtime errors", "convergence criteria", "script safety", "iterative refinement"]
chapter: 39
lesson: 5
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Write-Execute-Analyze Loop Pattern"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can articulate the three phases of iterative script execution (write code from spec, execute and capture output/errors, analyze results and fix issues) and explain why iteration is necessary for robust code generation"

  - name: "Error Detection and Recovery in Script Execution"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can distinguish syntax errors (code won't parse) from runtime errors (code parses but fails during execution), identify patterns in error messages, and explain recovery strategies for each type"

  - name: "Convergence and Iteration Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can define success criteria for script execution (spec complete, edge cases handled), set iteration limits to prevent infinite loops, and design convergence checks that stop iteration when criteria are met"

learning_objectives:
  - objective: "Understand write-execute-analyze loop as iterative pattern for autonomous code generation"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Manual walkthrough of loop phases; articulation of why iteration is necessary"

  - objective: "Recognize when script execution is appropriate versus MCP (standalone computation vs delegating to tools)"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Comparison of use cases; decision-making about which approach fits problem"

  - objective: "Identify error types (syntax vs runtime) and explain recovery strategies for each"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Error classification exercise; strategy articulation for each error type"

  - objective: "Design skill specification for script-execution task including success criteria, edge cases, and convergence conditions"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Specification design exercise for data processing task"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (write-execute-analyze loop, syntax error detection, runtime error handling, timeout/resource constraints, partial execution state, safety constraints, output parsing, convergence criteria, MCP comparison) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design a skill that handles timeout scenarios—code starts executing but takes too long. How would you pause, analyze partial progress, and resume? How would that change the iteration loop? Explore edge cases: what if the same error occurs twice in a row?"
  remedial_for_struggling: "Focus on one concrete example: CSV data processing. Walk through the write-execute-analyze loop step by step, showing exactly what each phase produces. Practice identifying one syntax error in provided code and explaining the fix."
---

# Script Execution Fundamentals

You've learned to wrap MCP servers with intelligence (Lessons 3-4). But MCP has a limitation: it gives you access only to what the MCP server exposes as tools. What if you need to solve a problem that requires arbitrary computation—data transformation, complex analysis, custom algorithms—that no existing MCP server provides?

That's where script execution comes in. Instead of invoking a tool via MCP, your skill writes code, executes it, analyzes the results, fixes problems, and iterates until the problem is solved. This generalization beyond MCP unlocks a different class of agent capabilities.

Consider this scenario: You have a CSV file with customer data. Your task is to identify patterns—which customer segments have high churn risk? Which products correlate with customer satisfaction? Which time periods show seasonal trends? No existing API or MCP server solves this problem the way your specific business needs it solved. You need your skill to write analysis code, execute it against your data, and refine the analysis based on what it learns.

This is autonomous script execution. And it follows a pattern you'll use repeatedly: write code from specification → execute it → analyze output and errors → iterate until complete.

## The Write-Execute-Analyze Loop: The Core Pattern

This loop is the foundation of autonomous code execution. Let's break it down with a concrete example.

### The Specification (Starting Point)

Before your skill generates any code, it needs a specification. What problem are we solving?

```yaml
Intent: "Analyze customer CSV data"

Input:
  data_file: "customers.csv"
  analysis_type: "churn_risk_analysis"

Output:
  format: "JSON report"
  required_fields:
    - segment: "Customer segment (high, medium, low risk)"
    - percentage: "% of customers in this segment"
    - primary_factors: "Top 3 factors driving churn for this segment"
    - recommendations: "Business actions for this segment"

Success Criteria:
  - Output must categorize ALL customers into risk segments
  - Each segment must have primary factors identified (not generic)
  - Edge cases handled: missing data, outliers, incomplete records
```

This specification tells the skill what problem to solve before it writes any code.

### Phase 1: Write (Generate Code from Spec)

The skill generates Python code that implements the specification:

```python
import pandas as pd
import json
from typing import List, Dict

def analyze_churn_risk(file_path: str) -> Dict:
    """
    Analyze customer data and categorize by churn risk.
    Output: JSON report with segments and factors.
    """
    # Load data
    df = pd.read_csv(file_path)

    # Calculate risk metrics
    df['recency_score'] = calculate_recency(df['last_purchase_date'])
    df['purchase_frequency_score'] = df['purchase_count'] / df['account_age_months']
    df['spend_score'] = df['annual_spend'] / df['account_age_months']

    # Combine into risk score
    df['churn_risk_score'] = (
        0.4 * df['recency_score'] +
        0.35 * df['purchase_frequency_score'] +
        0.25 * df['spend_score']
    )

    # Segment by risk
    segments = {
        'high_risk': df[df['churn_risk_score'] < 0.3],
        'medium_risk': df[(df['churn_risk_score'] >= 0.3) & (df['churn_risk_score'] < 0.7)],
        'low_risk': df[df['churn_risk_score'] >= 0.7]
    }

    # Generate report
    report = build_segment_report(segments)
    return report

def calculate_recency(dates):
    """Calculate recency score: recent = high score"""
    # ... implementation
    pass

def build_segment_report(segments):
    """Build JSON report with segment analysis"""
    # ... implementation
    pass

if __name__ == "__main__":
    result = analyze_churn_risk("customers.csv")
    print(json.dumps(result, indent=2))
```

**What happened**: The skill read the specification and generated Python code that should implement it.

### Phase 2: Execute (Run the Code)

The skill executes the generated code. Let's say it produces this output:

```
Traceback (most recent call last):
  File "analysis.py", line 24, in <module>
    df['recency_score'] = calculate_recency(df['last_purchase_date'])
NameError: name 'last_purchase_date' is not defined
```

**Error encountered**: The code references a column that doesn't exist in the CSV. Synthesis error? No—syntax is fine. It's a **runtime error**: the code runs, Python parses it, but it fails when it tries to access a non-existent column.

### Phase 3: Analyze (Read the Error and Understand the Problem)

The skill reads the error message:
```
NameError: name 'last_purchase_date' is not defined
```

What does this tell us?
- The code tried to access something called `last_purchase_date` that doesn't exist
- This could mean: (1) Column name in CSV is different, (2) We need to check what columns exist first, (3) The column might be missing entirely

### Phase 4: Fix and Retry

The skill generates revised code:

```python
import pandas as pd

# First, let's inspect the data structure
def analyze_churn_risk(file_path: str) -> Dict:
    """
    Analyze customer data and categorize by churn risk.
    """
    # Load data
    df = pd.read_csv(file_path)

    # Inspect available columns
    print(f"Available columns: {list(df.columns)}")

    # Check if the column exists; if not, use similar column
    if 'last_purchase_date' in df.columns:
        date_col = 'last_purchase_date'
    elif 'last_purchase' in df.columns:
        date_col = 'last_purchase'
    elif 'purchase_date' in df.columns:
        date_col = 'purchase_date'
    else:
        raise ValueError("No date column found")

    # Now use the correct column
    df['recency_score'] = calculate_recency(df[date_col])
    # ... rest of code
```

**Key improvement**: The code now inspects what columns actually exist before trying to use them.

Execute this revised code → It prints:
```
Available columns: ['customer_id', 'signup_date', 'last_purchase', 'purchase_count', 'annual_spend', 'account_age_months']
```

Excellent! The column is `last_purchase`, not `last_purchase_date`. The code adapted and continues. Now execution produces:

```
{
  "high_risk": {
    "count": 245,
    "percentage": 18.5,
    "primary_factors": ["No purchase in 6+ months", "Low annual spend", "New account"]
  },
  "medium_risk": {
    "count": 623,
    "percentage": 47.0,
    "primary_factors": ["Infrequent purchases", "Moderate spend", "Variable engagement"]
  },
  "low_risk": {
    "count": 544,
    "percentage": 34.5,
    "primary_factors": ["Recent purchases", "High annual spend", "Long account tenure"]
  }
}
```

Great! But wait—let's check against the specification. Does this output match what was required?

**Analyze against spec:**
- ✓ Output is JSON
- ✓ All customers categorized into segments
- ✓ Percentages included
- ✓ Primary factors identified
- ✗ Missing: "Recommendations" field (what should the business DO for each segment?)

The output is incomplete. One more iteration needed.

### Phase 5: Refine (Iterate Again)

Generate code that adds the missing recommendations:

```python
def build_segment_report(segments):
    """Build JSON report with segment analysis and recommendations"""
    report = {}

    for segment_name, segment_df in segments.items():
        segment_size = len(segment_df)
        total = sum(len(s) for s in segments.values())

        # Extract primary factors
        factors = identify_risk_factors(segment_df, segment_name)

        # Generate recommendations based on segment
        recommendations = generate_recommendations(segment_name, factors)

        report[segment_name] = {
            'count': segment_size,
            'percentage': round(100 * segment_size / total, 1),
            'primary_factors': factors,
            'recommendations': recommendations  # NEW
        }

    return report

def generate_recommendations(segment: str, factors: List[str]) -> List[str]:
    """Generate business recommendations for each segment"""
    if segment == 'high_risk':
        return [
            "Launch re-engagement campaign (email, special offers)",
            "Schedule personalized outreach calls",
            "Offer loyalty rewards to encourage repeat purchases"
        ]
    elif segment == 'medium_risk':
        return [
            "Increase purchase frequency incentives",
            "Send targeted product recommendations",
            "Create exclusive early-access programs"
        ]
    else:  # low_risk
        return [
            "Maintain quality service (primary retention factor)",
            "Offer premium/VIP benefits to increase lifetime value",
            "Use as model for successful customer patterns"
        ]
```

Execute this revised code → Output now includes recommendations:

```json
{
  "high_risk": {
    "count": 245,
    "percentage": 18.5,
    "primary_factors": ["No purchase in 6+ months", "Low annual spend", "New account"],
    "recommendations": ["Launch re-engagement campaign...", "Schedule personalized outreach calls...", "..."]
  },
  ...
}
```

**Check against spec again:**
- ✓ All required fields present
- ✓ All customers categorized
- ✓ Edge cases handled (inspected columns first)
- ✓ Output format matches specification

**Convergence achieved**: The specification is satisfied. Stop iterating.

---

## Three Types of Errors: Recognizing and Recovering

As the skill iterates, it will encounter different error types. Recognizing which type helps design the right fix.

### Error Type 1: Syntax Errors (Code Won't Parse)

**What happens**: The Python parser reads your code and says "This doesn't look like valid Python syntax."

**Example**:
```python
def analyze_churn_risk(file_path: str) -> Dict:
    df = pd.read_csv(file_path)
    df['risk_score'] = calculate_score(df)  # Missing closing parenthesis
    print(df  # ← Syntax error here!
```

**Error message**:
```
SyntaxError: invalid syntax (line 34)
```

**Recovery**: The code didn't even run. The parser caught it. Fix: Add the missing parenthesis.

```python
print(df)  # Fixed
```

**Skill's logic**: "The error happened before execution. This is a typo or malformed code. Show the error line and regenerate that section with correct syntax."

### Error Type 2: Runtime Errors (Code Parses But Fails During Execution)

**What happens**: The Python parser accepts the code, but when the code runs, it encounters a problem.

**Example**:
```python
def analyze_churn_risk(file_path: str) -> Dict:
    df = pd.read_csv(file_path)

    # This line runs, but 'last_purchase_date' column doesn't exist
    df['recency_score'] = calculate_recency(df['last_purchase_date'])
    # Runtime error ↑
```

**Error message**:
```
KeyError: 'last_purchase_date' is not in index
```

**Recovery**: The code ran partially. We know exactly where it failed. Fix: Check what columns exist and use the correct name.

```python
if 'last_purchase_date' in df.columns:
    date_col = 'last_purchase_date'
else:
    date_col = 'last_purchase'  # Use actual column name

df['recency_score'] = calculate_recency(df[date_col])
```

**Skill's logic**: "The error happened during execution. Understand what the code was trying to do when it failed. Fix: adjust data access, add defensive checks, or change algorithm."

### Error Type 3: Logic Errors (Code Runs But Produces Wrong Results)

**What happens**: Code executes without errors, but the output is incorrect—doesn't match specification or shows nonsensical results.

**Example**:
```python
# Risk score calculation (WRONG)
df['churn_risk_score'] = (
    0.4 * df['recency_score'] +  # Weights old customers high (backwards!)
    0.35 * df['purchase_frequency_score'] +
    0.25 * df['spend_score']
)

# Result: Customers who HAVEN'T purchased recently get HIGH scores
# But spec says high score = LOW risk. This is backwards.
```

**Detection**: Code runs, but when you compare output to specification:
- Expected: High-risk segment = customers not purchasing recently
- Actual: High-risk segment = customers purchasing frequently
- Something is inverted.

**Recovery**: Fix the algorithm logic.

```python
# Corrected: Recent purchase = LOW risk
# Infrequent purchase = HIGH risk
recency_weight = 0.4 * (1.0 - df['recency_score'])  # Invert the score
# Now recent purchases reduce risk correctly
```

**Skill's logic**: "No error message, but output is wrong. Analyze what the code produces vs. specification. Identify which calculation is backwards. Regenerate that part."

---

## Safety Constraints: What Scripts Should and Shouldn't Do

When your skill generates code that executes autonomously, safety matters. Not all operations are safe in all contexts.

### Filesystem Access: When Safe, When Risky

**SAFE**:
- Reading data files (CSV, JSON) you've specified
- Writing analysis results to a specific output directory
- Temporary files for intermediate processing (within execution environment)

**RISKY**:
- Deleting files (what if the code targets the wrong file?)
- Writing to arbitrary filesystem locations (could overwrite system files)
- Accessing files outside your designated data directory

**Skill constraint**: "Before any file operation, verify the exact path. Use whitelisted directories only. Never construct paths from user input without validation."

```python
ALLOWED_DIRECTORIES = ["/data/", "/results/", "/temp/"]

def safe_write_file(filepath: str, content: str):
    """Only write to allowed directories"""
    filepath = os.path.abspath(filepath)

    # Verify the path is within allowed directories
    if not any(filepath.startswith(allowed) for allowed in ALLOWED_DIRECTORIES):
        raise PermissionError(f"Write to {filepath} not allowed")

    # Now safe to write
    with open(filepath, 'w') as f:
        f.write(content)
```

### API and Database Access: Rate Limits and Quotas

**SAFE**:
- Queries within rate limits (e.g., max 100 API calls per minute)
- Read-only access to databases you control
- Transactions with explicit commit/rollback

**RISKY**:
- Unlimited API calls (you'll get blocked or charged heavily)
- Writing to databases without transaction control
- Assuming network reliability (APIs can be unavailable)

**Skill constraint**: "Any network operation includes retry logic, rate limiting, and explicit error handling."

```python
from time import sleep
from functools import wraps

def rate_limited(max_calls: int, time_window: int):
    """Decorator to rate-limit API calls"""
    def decorator(func):
        call_count = [0]
        last_reset = [time.time()]

        def wrapper(*args, **kwargs):
            now = time.time()
            if now - last_reset[0] > time_window:
                call_count[0] = 0
                last_reset[0] = now

            if call_count[0] >= max_calls:
                sleep_time = time_window - (now - last_reset[0])
                sleep(sleep_time)
                call_count[0] = 0
                last_reset[0] = time.time()

            call_count[0] += 1
            return func(*args, **kwargs)

        return wrapper
    return decorator

@rate_limited(max_calls=100, time_window=60)  # 100 calls per minute
def fetch_data_from_api(endpoint: str):
    response = requests.get(endpoint, timeout=5)
    return response.json()
```

### Resource Constraints: Preventing Runaway Execution

**SAFE**:
- Setting maximum execution time (30 seconds per script)
- Limiting memory usage (no loading entire datasets into RAM)
- Iteration limits (max 5 retries, then give up)

**RISKY**:
- Infinite loops (forgot to increment counter)
- Loading gigabytes of data into memory
- No timeout (script runs forever, consuming resources)

**Skill constraint**: "Every script has explicit resource guards."

```python
import signal

def timeout_handler(signum, frame):
    raise TimeoutError("Script execution exceeded time limit")

# Set 30-second timeout
signal.signal(signal.SIGALRM, timeout_handler)
signal.alarm(30)

try:
    # Your script runs here
    result = analyze_churn_risk("customers.csv")
finally:
    signal.alarm(0)  # Cancel timeout
```

---

## Convergence Criteria: When to Stop Iterating

The loop must eventually stop. Otherwise, your skill iterates forever. How do you know when iteration has accomplished the goal?

### Success Criteria (We're Done—Stop Iterating)

The skill stops when:

1. **Specification is fully satisfied**
   - All output fields present ✓
   - All required analysis complete ✓
   - Edge cases handled ✓

2. **No new errors emerge**
   - Previous iteration's fix resolved the problem
   - No new runtime errors appear
   - Output validates against specification

3. **Quality gates pass**
   - Output format correct (JSON if specified, not malformed)
   - Data values reasonable (not NaN, not infinite)
   - All assertions pass

**Example convergence check:**

```python
def check_convergence(output: Dict, spec: Dict) -> bool:
    """Has the output satisfied the specification?"""

    # Check all required fields present
    for required_field in spec['output']['required_fields']:
        if required_field not in output:
            return False  # Missing field, not converged

    # Check segment categories
    expected_segments = ['high_risk', 'medium_risk', 'low_risk']
    for segment in expected_segments:
        if segment not in output:
            return False

        # Each segment must have required fields
        segment_data = output[segment]
        if 'percentage' not in segment_data:
            return False
        if segment_data['percentage'] < 0 or segment_data['percentage'] > 100:
            return False  # Invalid percentage

    # All checks passed
    return True
```

### Failure Criteria (We're Stuck—Stop Iterating)

The skill stops and reports failure when:

1. **Iteration limit reached**
   - Attempted 5 fixes, same error persists
   - Spec is fundamentally unclear or unachievable
   - Action: Report error, ask for human clarification

2. **Resource limit exceeded**
   - Execution took longer than 30 seconds
   - Memory usage exceeded limit
   - Action: Fail gracefully, report which resource was exceeded

3. **Non-recoverable error**
   - Missing required data file (can't generate without it)
   - Permission denied accessing data
   - Action: Fail immediately, no retry

**Example failure check:**

```python
def should_stop_iterating(iteration: int, error_type: str, last_error_message: str) -> bool:
    """Should we give up on this script?"""

    # Iteration limit: stop after 5 attempts
    if iteration >= 5:
        return True

    # Non-recoverable errors: stop immediately
    if error_type in ['FileNotFoundError', 'PermissionError', 'TimeoutError']:
        return True

    # Repeated same error: after 2 identical errors, probably unfixable
    if error_type == 'SameAsLastIteration':
        return True

    # Converged successfully
    if error_type == 'ConvergenceSuccess':
        return True

    # Otherwise, keep iterating
    return False
```

---

## Script Execution vs. MCP: When to Use Each

You've learned both MCP (Lesson 3-4) and script execution. When do you use each?

| Scenario | Use MCP | Use Script |
|----------|---------|-----------|
| **Accessing external service** (fetch documentation, browse web) | ✓ MCP is designed for this | ✗ Can't make web requests from script |
| **Custom data transformation** (aggregate CSV, calculate metrics) | ✗ MCP doesn't have your specific logic | ✓ Write code to transform data |
| **Combining multiple MCP calls** (fetch docs, then analyze them) | ✓ MCP + orchestration skill | ~ Script could do it, but MCP is cleaner |
| **Complex business logic** (customer segmentation with domain rules) | ✗ Not in any MCP server | ✓ Write the logic |
| **Real-time integration** (call API, process, store result) | ✓ MCP tools are reliable | ~ Script works, but MCP is more robust |
| **One-time data analysis** (analyze this specific CSV once) | ✗ Overkill | ✓ Perfect use case |

**Decision rule**: If a tool exists via MCP, use MCP. If you need custom computation, write a script. If you need both (fetch via MCP, then process via script), orchestrate them together.

---

## Manual Exercise: Walk Through the Loop Yourself

You won't code yet—this is design and analysis practice.

### Scenario: Log File Analysis

**Specification:**
```
Intent: "Analyze application logs for error patterns"

Input:
  log_file: "application.log"
  date_range: "last 7 days"

Output:
  - error_categories: List of error types found
  - frequency: How many times each error occurred
  - timeline: When errors occurred (by hour)
  - top_affected_users: Users who experienced most errors

Success Criteria:
  - All error types categorized correctly
  - No duplicate error categories
  - Timeline shows granular hourly breakdown
  - Edge cases: Missing timestamps handled gracefully
```

### Your Task: Walk Through the Loop (Manually)

Imagine a skill is trying to solve this. The skill will:

1. **Write Phase**: Generate Python code to parse the log file
2. **Execute Phase**: Run the code (let's say it fails)
3. **Analyze Phase**: Read the error message and understand the problem
4. **Fix Phase**: Generate corrected code
5. **Repeat**: Execute again

**Provide responses** (no coding, just analysis):

**Step 1**: What would the skill's first attempt at code look like? (Pseudocode—what functions would it call?)

```
[Your answer: Describe the initial approach]
```

**Step 2**: What error might occur? (What could go wrong?)

```
[Your answer: Describe a likely failure]
```

**Step 3**: How would you read that error message and understand what went wrong?

```
[Your answer: Interpret the error]
```

**Step 4**: How would you fix it?

```
[Your answer: Describe the fix]
```

**Step 5**: When would you stop iterating? What indicates "complete"?

```
[Your answer: Define convergence for this task]
```

---

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini, or similar) to explore the write-execute-analyze pattern.

### Prompt 1: Design Your Spec

```
I need to analyze a dataset (you pick what kind: customer data, sales,
logs, etc.). Write a brief specification for what code should do:
- What's the input?
- What output should look like?
- What are 2-3 edge cases the code must handle?

Then, ask AI: "Does my specification have all the information a code-writing
AI would need to generate correct code?"
```

**What you're learning**: How specification quality affects whether generated code will be correct on the first try.

### Prompt 2: Simulate an Error

```
Here's a Python script that implements my spec:
[paste a simple script with an intentional bug—maybe missing column name,
or math operation on wrong data type]

When this runs, what error will occur?
What does the error message tell us about what went wrong?
How would you fix it?
```

**What you're learning**: Error messages tell you exactly what went wrong. Reading them carefully is half the battle.

### Prompt 3: Design Iteration Criteria

```
I'm building a skill that writes code, executes it, and iterates until
the spec is satisfied. Here are my success criteria:
[list 3-4 criteria: all output fields present, no errors, values are reasonable]

What iteration limits should I set?
- How many times should the skill retry before giving up?
- What resource limits should I enforce (time, memory)?
- How would I detect if the skill is stuck in an infinite loop?
```

**What you're learning**: Convergence criteria and iteration limits prevent runaway execution while allowing enough iterations to reach the solution.

### Safety Note

As you design autonomous code execution, remember: the code your skill generates will run against real data. Design your convergence criteria conservatively—it's better to stop early and ask for human review than to iterate endlessly. Set explicit time limits, memory limits, and iteration counts. These constraints are safety features, not limitations. They're what make skills reliable instead of unpredictable.

---

**Takeaway**: The write-execute-analyze loop is how autonomous skills solve problems through code generation. You understand the phases, error types, convergence criteria, and safety constraints. In Lesson 6, you'll implement this loop, building a skill that orchestrates all three phases into a reliable execution system.
