---
sidebar_position: 7
title: "Deployment & Verification"
description: "Deploy agents to Vertex AI and verify behavior matches local tests"
keywords: [google adk, vertex ai, agent engine, deployment, adk deploy, agent verification]
chapter: 35
lesson: 7
duration_minutes: 55

# HIDDEN SKILLS METADATA
skills:
  - name: "Vertex AI Deployment"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can deploy agent to Vertex AI Agent Engine using adk deploy and receive production endpoint"

  - name: "Deployment Verification"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run the same eval cases against remote endpoint and verify behavior matches local tests"

  - name: "Production Safety Patterns"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Digital Safety"
    measurable_at_this_level: "Student can identify pre-deployment checks (rate limits, guardrails, monitoring) required before production traffic"

learning_objectives:
  - objective: "Deploy agent to Vertex AI Agent Engine using adk deploy command"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student successfully deploys agent and receives resource endpoint URL"

  - objective: "Verify deployed agent matches local test behavior"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student runs eval cases against remote endpoint and confirms matching pass rates"

  - objective: "Establish pre-deployment safety checklist"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student verifies guardrails, rate limits, and monitoring before production rollout"

cognitive_load:
  new_concepts: 3
  assessment: "3 concepts (deployment, verification, safety) within B1 limit (7-10 concepts) - Appropriate for capstone-adjacent lesson"

differentiation:
  extension_for_advanced: "Implement blue-green deployment strategy with traffic gradual rollout and monitoring dashboards"
  remedial_for_struggling: "Focus on basic deployment steps first; verification as separate exercise after endpoint confirmed working"
---

# Deployment & Verification

Your agent passes all local tests. You've built it with evaluation-first discipline, refined it through iterations, architected it for reliability. Now comes the critical question: **Does it work the same way in production?**

The gap between "works on my machine" and "works in production" is where real reliability is tested. This lesson teaches you to bridge that gap—deploying to Vertex AI while verifying that remote behavior matches local expectations.

## The Deployment Problem: Beyond the Local Environment

In development, your agent runs against:
- Your local file system with test data
- Your API keys with full permissions
- Predictable network conditions
- No concurrent users

In production (Vertex AI), your agent runs against:
- Real databases with variable data
- Rate-limited API calls with quota constraints
- Unpredictable network latency
- Multiple concurrent users
- Real error conditions your tests didn't anticipate

**The question isn't "Can I deploy?"** Most frameworks make deployment trivial—a single command. **The real question is "Can I deploy safely and verify it still works?"**

This is where evaluation-first development pays off. You have executable tests. Deploy the agent, run the same tests against the production endpoint, and confirm the behavior matches. No guessing. No "seems to be working."

## Prerequisites: Preparation Before Deployment

Before deploying, ensure you have:

```bash
# Google Cloud project with Vertex AI enabled
gcloud auth application-default login
gcloud services enable aiplatform.googleapis.com

# Set your project
gcloud config set project YOUR_PROJECT_ID

# Create a staging bucket for deployment artifacts
gsutil mb gs://your-bucket-adk-staging

# Install latest ADK and deployment tools
uv add google-cloud-aiplatform
uv add "google-adk>=0.3.0"
```

**Output:**

```
Created Google Cloud project configuration.
Staging bucket: gs://your-bucket-adk-staging ready.
Vertex AI APIs enabled.
ADK version checked and installed.
```

### Pre-Deployment Checklist

Before running `adk deploy`, verify:

| Check | Verification | Issue |
|-------|----------------|-------|
| **All eval cases pass locally** | `adk eval ./my_agent ./tests/evals.json` shows 100% pass | Deploy failures are harder to debug—ensure local tests are solid first |
| **Agent runs without errors** | `adk run my_agent` completes at least 5 test interactions | Syntax errors or import failures surface in dev, not production |
| **Guardrails configured** | `before_model_callback` and `before_tool_callback` present | Unguarded agents can misbehave on untrusted input |
| **Rate limits set** | Tool execution limits defined | Runaway loops consume quotas silently |
| **Logging enabled** | Standard Python logging configured | Production debugging requires logs |

If any check fails → Fix locally before deploying.

## Step 1: Deploy to Vertex AI Agent Engine

Vertex AI Agent Engine is Google's managed infrastructure for production agents. It handles scaling, monitoring, and resource management automatically.

### CLI Deployment (Recommended for First Deploy)

```bash
# Deploy your agent to Vertex AI Agent Engine
adk deploy agent_engine \
  --project=YOUR_PROJECT_ID \
  --region=us-central1 \
  --staging_bucket="gs://your-bucket-adk-staging" \
  --display_name="TaskManager Agent" \
  ./task_manager
```

**Output:**

```
Deploying agent to Vertex AI Agent Engine...
✓ Validating agent structure
✓ Building container
✓ Uploading to staging bucket
✓ Creating reasoning engine

Deployment successful!
Resource name: projects/123456/locations/us-central1/reasoningEngines/engine-abc123
Endpoint: https://us-central1-aiplatform.googleapis.com/v1beta1/projects/123456/locations/us-central1/reasoningEngines/engine-abc123
```

**What happened:**
1. ADK validated your agent structure
2. Built a container image with your agent code
3. Uploaded to staging bucket
4. Created a managed Vertex AI reasoning engine
5. Assigned you a production endpoint URL

Store the resource name—you'll use it to query the deployed agent and run verification tests.

### Python SDK Deployment (Programmatic)

If you need finer control over deployment:

```python
from vertexai.preview import reasoning_engines
from vertexai import agent_engines
import vertexai

# Initialize Vertex AI
vertexai.init(
    project="YOUR_PROJECT_ID",
    location="us-central1",
    staging_bucket="gs://your-bucket-adk-staging"
)

# Wrap your agent for deployment
adk_app = reasoning_engines.AdkApp(
    agent=root_agent,
    enable_tracing=True  # Cloud Trace integration for debugging
)

# Deploy to Vertex AI
remote_app = agent_engines.create(
    agent_engine=adk_app,
    extra_packages=["./task_manager"],  # Include local modules
    requirements=["google-cloud-aiplatform[adk,agent_engines]"]
)

print(f"Deployed: {remote_app.resource_name}")
```

**Output:**

```
Deployed: projects/123456/locations/us-central1/reasoningEngines/engine-abc123
Endpoint ready for queries.
```

**Key differences from CLI:**
- Programmatic control over deployment parameters
- Can integrate into CI/CD pipelines
- Enables blue-green deployments (deploy new version, verify, then switch traffic)

## Step 2: Verify Deployment Is Live

Before running tests, confirm the endpoint is responding:

```python
from vertexai import agent_engines

# Get the deployed agent
remote_app = agent_engines.get(
    resource_name="projects/123456/locations/us-central1/reasoningEngines/engine-abc123"
)

# Send a simple query
response = remote_app.query(
    user_id="test-user",
    message="Hello, are you there?"
)
print(response)
```

**Output:**

```
Hello! I'm the TaskManager agent. I can help you:
- Add new tasks
- List your tasks
- Mark tasks as complete
- Delete tasks

What would you like to do?
```

If you see a response → Deployment is live. Proceed to verification.

If you get an error → Check:
- Resource name is correct
- Project ID is configured
- You have `aiplatform.reasoningEngines.query` IAM permission

## Step 3: Run Eval Cases Against Remote Endpoint

Here's where evaluation-first development validates deployment. You'll run the exact same eval cases that passed locally, but against the production endpoint.

### Create Verification Test Script

```python
import asyncio
from google.adk.evaluation.agent_evaluator import AgentEvaluator
from vertexai import agent_engines
import vertexai

vertexai.init(
    project="YOUR_PROJECT_ID",
    location="us-central1"
)

async def verify_remote_agent():
    """Run eval cases against deployed agent endpoint."""

    # Get the deployed agent
    remote_app = agent_engines.get(
        resource_name="projects/123456/locations/us-central1/reasoningEngines/engine-abc123"
    )

    # Run evaluation against remote endpoint
    results = await AgentEvaluator.evaluate(
        agent_engine=remote_app,
        eval_dataset_file_path="tests/taskmanager_evals.json"
    )

    # Print summary
    print(f"Eval Results (Remote):")
    print(f"  Passed: {results.num_eval_cases_passed}")
    print(f"  Failed: {results.num_eval_cases_failed}")
    print(f"  Pass Rate: {results.pass_rate:.1%}")

    # Compare with local baseline
    if results.pass_rate >= 0.95:
        print("✓ Remote behavior matches local tests")
        return True
    else:
        print("✗ Remote behavior differs from local")
        print("Failed cases:")
        for case in results.failed_cases:
            print(f"  - {case.eval_id}: {case.failure_reason}")
        return False

# Run verification
if __name__ == "__main__":
    success = asyncio.run(verify_remote_agent())
    exit(0 if success else 1)
```

**Output (Success):**

```
Eval Results (Remote):
  Passed: 12
  Failed: 0
  Pass Rate: 100.0%
✓ Remote behavior matches local tests
```

**Output (Failure):**

```
Eval Results (Remote):
  Passed: 10
  Failed: 2
  Pass Rate: 83.3%
✗ Remote behavior differs from local

Failed cases:
  - complete_task_by_id: Expected tool call to complete_task, got unknown_tool
  - delete_task_missing: Expected error message, got generic response
```

If remote tests fail → Do NOT proceed to production traffic. Instead:

1. **Diagnose the failure**: Review Cloud Logs for the failed requests
2. **Identify the cause**: Is it a code issue, environment issue, or permission issue?
3. **Fix and redeploy**: Iterate until remote tests pass

### Interpreting Verification Results

| Scenario | Pass Rate | Action |
|----------|-----------|--------|
| **All local tests pass, all remote tests pass** | 100% | Safe to route traffic |
| **All local tests pass, some remote tests fail** | <100% | Diagnose: Environment issue, guardrail blocking, or latency timeout |
| **Local tests already failing** | N/A | Fix locally first before deploying |

## Step 4: Pre-Production Safety Checks

After verification passes, before routing real traffic:

### Enable Monitoring and Logging

```python
# Enable Cloud Trace for debugging
from google.cloud import trace_v2

trace_client = trace_client.TraceServiceClient()

# Enable Cloud Logging
import logging
from google.cloud import logging as cloud_logging

logging_client = cloud_logging.Client()
logging_client.setup_logging()

# Now all Python logs route to Cloud Logging
logging.info("Agent deployed and monitoring enabled")
```

**Output:**

```
Agent logs available at:
https://console.cloud.google.com/logs/query?project=YOUR_PROJECT_ID
```

### Configure Rate Limiting

```python
# Set per-user rate limit: 100 queries per minute
adk_app = reasoning_engines.AdkApp(
    agent=root_agent,
    enable_tracing=True,
    config=reasoning_engines.AdkAppConfig(
        rate_limit=100,  # requests per minute
        rate_limit_unit="per_minute"
    )
)
```

### Test Error Handling

Send invalid inputs to verify graceful failure:

```python
# Test 1: Empty query
response = remote_app.query(user_id="test", message="")
assert "error" in response.lower() or "specify" in response.lower()

# Test 2: Malformed input
response = remote_app.query(user_id="test", message="!@#$%^&*()")
assert response is not None  # Should not crash

# Test 3: Very long input
long_message = "a" * 10000
response = remote_app.query(user_id="test", message=long_message)
assert response is not None  # Should handle gracefully
```

**Output:**

```
✓ Empty query handled
✓ Malformed input handled
✓ Long input handled
All error conditions pass
```

## Step 5: Production Rollout

Once verification passes and safety checks complete:

### Option A: Immediate Rollout

```bash
# Route all traffic to deployed agent
gcloud run services update task-manager-api \
  --update-env-vars=AGENT_ENDPOINT=projects/123456/locations/us-central1/reasoningEngines/engine-abc123
```

**When to use**: Low-risk agents (internal tools, non-critical services)

### Option B: Gradual Rollout (Recommended)

```python
# Start with 10% traffic, monitor for errors
gcloud run services update task-manager-api \
  --traffic=current=90,new-revision=10

# Monitor for 1 hour
# If no errors, increase to 50%
gcloud run services update task-manager-api \
  --traffic=current=50,new-revision=50

# Monitor for 1 hour
# If still healthy, route 100%
gcloud run services update task-manager-api \
  --traffic=new-revision=100
```

**When to use**: Production systems with real users, customer-facing features

## Deployment Verification Checklist

| Phase | Check | Status | Notes |
|-------|-------|--------|-------|
| **Pre-Deploy** | All local eval cases pass | ☐ | 100% pass rate required |
| **Pre-Deploy** | Guardrails configured | ☐ | input/tool callbacks present |
| **Deploy** | `adk deploy` succeeds | ☐ | Resource endpoint received |
| **Post-Deploy** | Remote endpoint responds | ☐ | Basic query returns response |
| **Verify** | Remote eval cases pass | ☐ | Same pass rate as local |
| **Safety** | Error handling tested | ☐ | Graceful failures confirmed |
| **Safety** | Logging enabled | ☐ | Cloud Logging configured |
| **Safety** | Rate limits set | ☐ | Per-user limits configured |
| **Rollout** | Gradual traffic increase | ☐ | 10% → 50% → 100% strategy |
| **Monitor** | No error spikes | ☐ | Dashboard shows healthy metrics |

## Common Deployment Issues

| Issue | Symptom | Fix |
|-------|---------|-----|
| **Permissions denied** | 403 error on deploy | Check IAM role: `roles/aiplatform.admin` on service account |
| **Container build fails** | Upload fails during deploy | Verify all imports work locally (`adk run` works) |
| **Remote tests fail, local pass** | Environment discrepancy | Check: Different API keys? Missing environment variables? Latency timeouts? |
| **Endpoint timeout** | Queries take >30 seconds | Agent is too slow for production; optimize tool calls or split into smaller agents |
| **Rate limit hit instantly** | 429 errors from first query | Quota exceeded; deploy with lower iteration limits or increase Vertex AI quota |

## Exercise 1: Pre-Deployment Verification Checklist

You're about to deploy your TaskManager agent (6 tools, 2 callbacks, Firestore backend, 10 eval cases) to production. Before running `adk deploy`, create a comprehensive verification checklist specific to your agent.

**Your Agent Has:**
- **Tools**: add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks
- **Callbacks**: before_model_callback (input validation), before_tool_callback (tool protection)
- **State Backend**: Firestore SessionService with rate limiting
- **Eval Dataset**: 10 test cases covering normal flow, error conditions, and edge cases

**Your Task:**

Create a verification checklist with at least 10 items. For each item, specify:
1. What to check
2. The command or code to verify it
3. The expected result for "ready to deploy"

Here's the structure to follow:

| # | Check | Command/Method | Expected Result | Status |
|---|-------|-----------------|------------------|--------|
| 1 | All eval cases pass locally | `adk eval ./task_manager ./tests/taskmanager_evals.json` | 100% pass rate (10/10) | ☐ |
| 2 | Agent runs without errors (5+ interactions) | `adk run task_manager` then send 5 test queries | All queries complete without timeout or exception | ☐ |
| 3 | ? | ? | ? | ☐ |

**Fill in at least 8 more checks covering:**
- Agent structure validation (syntax, imports, tool registration)
- Callbacks present and functional (input validation, tool protection)
- Environment variables set (Firebase credentials, API keys)
- State management working (session persistence, user isolation)
- Error handling graceful (test with empty input, malformed data)
- Performance acceptable (response time <5s for normal queries)
- Guardrails active (rate limiting configured, max iterations set)
- Documentation complete (agent purpose, tool descriptions, error messages)

**Solution:**

<details>
<summary>Click to reveal complete checklist</summary>

| # | Check | Command/Method | Expected Result | Status |
|---|-------|-----------------|------------------|--------|
| 1 | All eval cases pass locally | `adk eval ./task_manager ./tests/taskmanager_evals.json` | 100% pass rate (10/10) | ✓ |
| 2 | Agent runs without errors (5+ interactions) | `adk run task_manager` then send 5 test queries | All queries complete without timeout or exception | ✓ |
| 3 | Agent structure is valid | `adk validate ./task_manager` | Validation passes, no syntax errors | ✓ |
| 4 | All tool functions are registered | `adk describe ./task_manager \| grep -c "tool"` | Output shows exactly 6 tools (add_task, list_tasks, complete_task, delete_task, edit_task, search_tasks) | ✓ |
| 5 | Input callback is present | Check `agent.py` for `before_model_callback` definition | Function exists and validates user_id, message length | ✓ |
| 6 | Tool callback is present | Check `agent.py` for `before_tool_callback` definition | Function exists and checks tool arguments against guardrails | ✓ |
| 7 | Firestore credentials configured | Test: `python -c "from google.cloud import firestore; db = firestore.Client(); print('OK')"` | Connection succeeds, no auth errors | ✓ |
| 8 | SessionService persists state | Run agent twice with same user_id and verify state carries over | Second session retrieves previous tasks without re-adding | ✓ |
| 9 | Agent handles empty input gracefully | Send `message=""` to running agent | Response includes helpful error message, no crash | ✓ |
| 10 | Agent handles malformed input gracefully | Send `message="!@#$%^&*()"` | Response is sensible (error or clarification), not hallucinated | ✓ |
| 11 | Response time acceptable | Time 10 normal queries with `time adk run task_manager` | Average response time <5 seconds per query | ✓ |
| 12 | Rate limiting configured | Check `agent.py` or deployment config for rate limits | Max iterations set (e.g., 10 per query), per-user limits configured | ✓ |

**Why each check matters:**

- **Checks 1-2**: If local tests fail, deployment will fail. Production is not the place to debug.
- **Checks 3-6**: Agent structure and callbacks are validation gates. Missing them causes cryptic cloud deployment errors.
- **Checks 7-8**: State management is critical. If Firestore connection fails here, it will fail in production.
- **Checks 9-10**: Error handling must be robust. Production users will send unexpected input.
- **Checks 11-12**: Performance and safety. Slow agents or unchecked loops consume quota and money.

**When to deploy**: All checks are green (✓). If any check fails, fix locally first and re-run before deploying.

</details>

---

## Exercise 2: Diagnose a Deployment Failure

You deployed your TaskManager agent and ran remote verification tests. The deployment succeeded and received an endpoint URL. But when you ran `verify_remote_agent()` against the endpoint, you got this error:

```
Eval Results (Remote):
  Passed: 8
  Failed: 2
  Pass Rate: 80.0%
✗ Remote behavior differs from local

Failed cases:
  - list_tasks_empty_user: Expected "You have no tasks", got "Error: User not authenticated"
  - search_task_by_id: Expected tool call to search_tasks, got unknown_tool_error
```

**Your Task:**

For each failed case, answer:
1. **What likely caused this specific failure?** (Pick the most probable cause from: code issue, environment issue, permission issue, configuration issue)
2. **How would you diagnose it?** (What logs or tests would you run?)
3. **How would you fix it?**
4. **How would you prevent it in CI/CD?** (What automated check catches this before deployment?)

**Example for Case 1:**

**Failed Case**: `list_tasks_empty_user: Expected "You have no tasks", got "Error: User not authenticated"`

**Likely Cause**: *Environment issue* — The remote deployment is not inheriting the Firestore credentials that work locally. SessionService is failing to authenticate, so user_id validation is rejecting the request.

**Diagnosis**:
```python
# Check logs for authentication error
gcloud logging read "resource.type=cloud_run_revision AND jsonPayload.message:Authentication" \
  --project=YOUR_PROJECT_ID --limit=10

# Or query the remote agent with debug flag
response = remote_app.query(
    user_id="test-user",
    message="list tasks",
    debug=True  # Enables verbose logging
)
```

**Fix**:
- Ensure Firestore credentials are set in the deployment environment: `gcloud run deploy task-manager-api --set-env-vars GOOGLE_APPLICATION_CREDENTIALS=/path/to/creds.json`
- Or use Workload Identity to bind the Cloud Run service account to Firestore permissions

**Prevention in CI/CD**:
```yaml
# In your CI/CD pipeline, add a "smoke test" step
- name: Verify Remote Auth
  run: |
    # Deploy to staging first
    adk deploy agent_engine --staging ./task_manager

    # Test authentication immediately
    python -c "
    from vertexai import agent_engines
    remote = agent_engines.get(resource_name=...)
    response = remote.query(user_id='ci-test', message='list tasks')
    assert 'not authenticated' not in response.lower()
    "
```

**Now it's your turn.** Answer the same 4 questions for Case 2:

**Failed Case**: `search_task_by_id: Expected tool call to search_tasks, got unknown_tool_error`

**Likely Cause**: [Your answer]

**Diagnosis**: [Your answer]

**Fix**: [Your answer]

**Prevention in CI/CD**: [Your answer]

<details>
<summary>Click to reveal solution for Case 2</summary>

**Failed Case**: `search_task_by_id: Expected tool call to search_tasks, got unknown_tool_error`

**Likely Cause**: *Code issue* — The `search_tasks` tool was not included in the deployed agent. Either:
- Tool was renamed locally but not in the deployment, or
- Tool registration is conditional (e.g., `if debug_mode: register_search()`) and debug_mode is False in production

**Diagnosis**:
```python
# Get the remote agent and list its available tools
remote_app = agent_engines.get(resource_name=...)

# Call adk describe on the deployed agent
adk describe <resource_name> | grep search_tasks

# Should show search_tasks in the tool list. If not, it wasn't deployed.

# Check Cloud Logs for tool registration errors
gcloud logging read "resource.type=cloud_run_revision AND jsonPayload.message:search_tasks" \
  --project=YOUR_PROJECT_ID
```

**Fix**:
1. Check your `agent.py` — ensure search_tasks is registered unconditionally (not behind a feature flag)
2. Verify tool is exported in `__init__.py` if using modules
3. Redeploy: `adk deploy agent_engine --project=YOUR_PROJECT_ID ./task_manager`
4. Re-run verification: `adk eval --endpoint=<resource_name> ./tests/taskmanager_evals.json`

**Prevention in CI/CD**:
```yaml
# Add a "tool inventory" check before deployment
- name: Verify All Tools Present
  run: |
    # Extract expected tools from evals
    EXPECTED_TOOLS=$(grep -o '"tool": "[^"]*"' tests/taskmanager_evals.json | cut -d'"' -f4 | sort -u)

    # Get tools from agent
    DEPLOYED_TOOLS=$(adk describe ./task_manager | grep "tool:" | cut -d' ' -f2 | sort -u)

    # Compare
    comm -23 <(echo "$EXPECTED_TOOLS") <(echo "$DEPLOYED_TOOLS") && echo "ERROR: Missing tools" && exit 1
    echo "All expected tools are registered"
```

**Key insight**: Remote failures often trace back to environment or configuration differences, not code. Always verify:
1. Local tests pass first
2. Environment variables/credentials are set in deployment
3. Tool registration is not conditional
4. Cloud Logs show what the agent actually did (vs. what you expected)

</details>

---

## Exercise 3: Build a Pre-Deployment Validation Script

You want to automate the checks from Exercise 1 so they run before every deployment. Write a Python script that validates your TaskManager agent is ready for production.

**Your Script Should:**

1. Run local eval cases and report pass rate
2. Validate agent structure (all 6 tools registered)
3. Test Firestore connectivity
4. Verify error handling (empty input, malformed input)
5. Check response times (all <5 seconds)
6. Confirm callbacks are registered
7. Exit with status 0 (success) or 1 (failure) for CI/CD integration

**Start with this template:**

```python
#!/usr/bin/env python3
"""Pre-deployment validation for TaskManager agent."""

import sys
import asyncio
import time
from typing import List, Dict
from google.adk.evaluation import AgentEvaluator
from task_manager import root_agent
from task_manager.services import SessionService

class PreDeploymentValidator:
    def __init__(self, agent, eval_dataset_path: str):
        self.agent = agent
        self.eval_dataset_path = eval_dataset_path
        self.checks_passed = 0
        self.checks_failed = 0
        self.results = []

    async def run_all_checks(self) -> bool:
        """Run all validation checks and return True if all pass."""
        # Check 1: Local eval cases
        # Check 2: Agent structure
        # Check 3: Firestore connectivity
        # Check 4: Error handling
        # Check 5: Response times
        # ... etc

        return self.checks_failed == 0

    def print_summary(self):
        """Print validation report."""
        print(f"\n{'='*60}")
        print(f"Pre-Deployment Validation Report")
        print(f"{'='*60}")
        print(f"Passed: {self.checks_passed}")
        print(f"Failed: {self.checks_failed}")
        print(f"{'='*60}\n")

if __name__ == "__main__":
    validator = PreDeploymentValidator(
        agent=root_agent,
        eval_dataset_path="tests/taskmanager_evals.json"
    )

    # Run validation
    success = asyncio.run(validator.run_all_checks())
    validator.print_summary()

    # Exit with appropriate code for CI/CD
    sys.exit(0 if success else 1)
```

**Fill in the missing checks. For each check:**
- Use descriptive test names
- Provide clear pass/fail output
- Log the result to `self.results`
- Update `self.checks_passed` or `self.checks_failed`

<details>
<summary>Click to reveal solution</summary>

```python
#!/usr/bin/env python3
"""Pre-deployment validation for TaskManager agent."""

import sys
import asyncio
import time
import json
from typing import List, Dict
from google.adk.evaluation import AgentEvaluator
from task_manager import root_agent
from task_manager.services import SessionService

class PreDeploymentValidator:
    def __init__(self, agent, eval_dataset_path: str):
        self.agent = agent
        self.eval_dataset_path = eval_dataset_path
        self.checks_passed = 0
        self.checks_failed = 0
        self.results = []

    async def check_local_evals(self) -> bool:
        """Check 1: All local eval cases pass."""
        print("Check 1: Running local eval cases...", end=" ")
        try:
            results = await AgentEvaluator.evaluate(
                agent=self.agent,
                eval_dataset_file_path=self.eval_dataset_path
            )

            passed = results.pass_rate >= 0.95  # 95% = 9.5/10, so at least 9/10
            if passed:
                print(f"✓ PASS ({results.num_eval_cases_passed}/{results.num_eval_cases_passed + results.num_eval_cases_failed})")
                self.checks_passed += 1
            else:
                print(f"✗ FAIL ({results.pass_rate:.0%})")
                self.checks_failed += 1
                for case in results.failed_cases:
                    print(f"    Failed: {case.eval_id}")

            self.results.append({
                "check": "Local Eval Cases",
                "status": "PASS" if passed else "FAIL",
                "details": f"{results.num_eval_cases_passed} passed, {results.num_eval_cases_failed} failed"
            })
            return passed
        except Exception as e:
            print(f"✗ ERROR: {str(e)}")
            self.checks_failed += 1
            self.results.append({"check": "Local Eval Cases", "status": "ERROR", "details": str(e)})
            return False

    def check_agent_structure(self) -> bool:
        """Check 2: All 6 tools are registered."""
        print("Check 2: Validating agent structure...", end=" ")
        try:
            expected_tools = {
                "add_task", "list_tasks", "complete_task",
                "delete_task", "edit_task", "search_tasks"
            }

            # Get registered tools from agent
            registered_tools = set(self.agent.tools.keys()) if hasattr(self.agent, 'tools') else set()

            missing_tools = expected_tools - registered_tools
            if not missing_tools:
                print(f"✓ PASS (all 6 tools registered)")
                self.checks_passed += 1
                self.results.append({"check": "Agent Structure", "status": "PASS", "details": "All tools registered"})
                return True
            else:
                print(f"✗ FAIL (missing: {', '.join(missing_tools)})")
                self.checks_failed += 1
                self.results.append({"check": "Agent Structure", "status": "FAIL", "details": f"Missing tools: {missing_tools}"})
                return False
        except Exception as e:
            print(f"✗ ERROR: {str(e)}")
            self.checks_failed += 1
            self.results.append({"check": "Agent Structure", "status": "ERROR", "details": str(e)})
            return False

    async def check_firestore_connectivity(self) -> bool:
        """Check 3: Firestore backend is accessible."""
        print("Check 3: Testing Firestore connectivity...", end=" ")
        try:
            session_service = SessionService()

            # Try to create and retrieve a test session
            test_user_id = "validation-test-user"
            test_data = {"test": "connectivity", "timestamp": time.time()}

            await session_service.save_session(test_user_id, test_data)
            retrieved = await session_service.get_session(test_user_id)

            if retrieved and retrieved.get("test") == "connectivity":
                print("✓ PASS (Firestore read/write working)")
                self.checks_passed += 1
                self.results.append({"check": "Firestore Connectivity", "status": "PASS"})

                # Cleanup
                await session_service.delete_session(test_user_id)
                return True
            else:
                print("✗ FAIL (Could not verify data)")
                self.checks_failed += 1
                self.results.append({"check": "Firestore Connectivity", "status": "FAIL", "details": "Data not retrieved"})
                return False
        except Exception as e:
            print(f"✗ ERROR: {str(e)}")
            self.checks_failed += 1
            self.results.append({"check": "Firestore Connectivity", "status": "ERROR", "details": str(e)})
            return False

    async def check_error_handling(self) -> bool:
        """Check 4: Agent handles error conditions gracefully."""
        print("Check 4: Testing error handling...", end=" ")

        error_tests = [
            ("empty_message", {"user_id": "test", "message": ""}),
            ("malformed_input", {"user_id": "test", "message": "!@#$%^&*()"}),
            ("very_long_input", {"user_id": "test", "message": "a" * 5000}),
        ]

        try:
            all_pass = True
            for test_name, query in error_tests:
                try:
                    # Simulate agent query with timeout
                    response = await asyncio.wait_for(
                        self.agent.process(query),
                        timeout=5.0
                    )

                    if response is None:
                        print(f"\n    {test_name}: ✗ No response")
                        all_pass = False
                    # else: response is valid, test passes
                except asyncio.TimeoutError:
                    print(f"\n    {test_name}: ✗ Timeout")
                    all_pass = False
                except Exception as e:
                    print(f"\n    {test_name}: ✗ Exception: {str(e)}")
                    all_pass = False

            if all_pass:
                print("✓ PASS (all error cases handled)")
                self.checks_passed += 1
            else:
                print("✗ FAIL (some error cases failed)")
                self.checks_failed += 1

            self.results.append({
                "check": "Error Handling",
                "status": "PASS" if all_pass else "FAIL"
            })
            return all_pass
        except Exception as e:
            print(f"✗ ERROR: {str(e)}")
            self.checks_failed += 1
            self.results.append({"check": "Error Handling", "status": "ERROR", "details": str(e)})
            return False

    async def check_response_times(self) -> bool:
        """Check 5: Response times are acceptable (<5 seconds)."""
        print("Check 5: Testing response times...", end=" ")
        try:
            test_queries = [
                {"user_id": "perf-test", "message": "list tasks"},
                {"user_id": "perf-test", "message": "add task buy milk"},
                {"user_id": "perf-test", "message": "list tasks"},
            ]

            times = []
            for query in test_queries:
                start = time.time()
                response = await asyncio.wait_for(
                    self.agent.process(query),
                    timeout=10.0
                )
                elapsed = time.time() - start
                times.append(elapsed)

                if elapsed > 5.0:
                    print(f"\n    Query slow: {elapsed:.1f}s (>5s threshold)")

            avg_time = sum(times) / len(times)
            max_time = max(times)

            if max_time <= 5.0:
                print(f"✓ PASS (avg: {avg_time:.2f}s, max: {max_time:.2f}s)")
                self.checks_passed += 1
                self.results.append({
                    "check": "Response Times",
                    "status": "PASS",
                    "details": f"avg: {avg_time:.2f}s, max: {max_time:.2f}s"
                })
                return True
            else:
                print(f"✗ FAIL (max: {max_time:.2f}s > 5s threshold)")
                self.checks_failed += 1
                self.results.append({
                    "check": "Response Times",
                    "status": "FAIL",
                    "details": f"max: {max_time:.2f}s"
                })
                return False
        except Exception as e:
            print(f"✗ ERROR: {str(e)}")
            self.checks_failed += 1
            self.results.append({"check": "Response Times", "status": "ERROR", "details": str(e)})
            return False

    def check_callbacks_registered(self) -> bool:
        """Check 6: Input and tool callbacks are registered."""
        print("Check 6: Verifying callbacks...", end=" ")
        try:
            has_input_callback = hasattr(self.agent, "before_model_callback") and self.agent.before_model_callback is not None
            has_tool_callback = hasattr(self.agent, "before_tool_callback") and self.agent.before_tool_callback is not None

            if has_input_callback and has_tool_callback:
                print("✓ PASS (both callbacks registered)")
                self.checks_passed += 1
                self.results.append({"check": "Callbacks", "status": "PASS"})
                return True
            else:
                missing = []
                if not has_input_callback:
                    missing.append("before_model_callback")
                if not has_tool_callback:
                    missing.append("before_tool_callback")
                print(f"✗ FAIL (missing: {', '.join(missing)})")
                self.checks_failed += 1
                self.results.append({"check": "Callbacks", "status": "FAIL", "details": f"Missing: {missing}"})
                return False
        except Exception as e:
            print(f"✗ ERROR: {str(e)}")
            self.checks_failed += 1
            self.results.append({"check": "Callbacks", "status": "ERROR", "details": str(e)})
            return False

    async def run_all_checks(self) -> bool:
        """Run all validation checks and return True if all pass."""
        print("\n" + "="*60)
        print("TaskManager Pre-Deployment Validation")
        print("="*60 + "\n")

        # Run checks sequentially
        check1 = await self.check_local_evals()
        check2 = self.check_agent_structure()
        check3 = await self.check_firestore_connectivity()
        check4 = await self.check_error_handling()
        check5 = await self.check_response_times()
        check6 = self.check_callbacks_registered()

        return self.checks_failed == 0

    def print_summary(self):
        """Print validation report."""
        print(f"\n{'='*60}")
        print(f"Pre-Deployment Validation Report")
        print(f"{'='*60}")
        print(f"Passed: {self.checks_passed}/6")
        print(f"Failed: {self.checks_failed}/6")

        if self.checks_failed == 0:
            print("\n✓ Agent is ready for deployment!")
        else:
            print("\n✗ Fix the issues above before deploying")
            for result in self.results:
                if result.get("status") != "PASS":
                    print(f"  - {result['check']}: {result.get('details', '')}")

        print(f"{'='*60}\n")

if __name__ == "__main__":
    validator = PreDeploymentValidator(
        agent=root_agent,
        eval_dataset_path="tests/taskmanager_evals.json"
    )

    # Run validation
    success = asyncio.run(validator.run_all_checks())
    validator.print_summary()

    # Exit with appropriate code for CI/CD integration
    sys.exit(0 if success else 1)
```

**Key features:**
- **Check 1**: Runs actual evals, doesn't just assume they pass
- **Check 2**: Verifies all 6 tools are registered
- **Check 3**: Tests Firestore with real read/write operations
- **Check 4**: Tests error conditions (empty input, malformed input, long input)
- **Check 5**: Measures response times, confirms <5s threshold
- **Check 6**: Confirms callbacks exist and are not None

**How to use:**
```bash
# Make executable
chmod +x validate_before_deploy.py

# Run validation
python validate_before_deploy.py

# In CI/CD, use exit code for pass/fail
python validate_before_deploy.py && adk deploy agent_engine ... || echo "Deployment blocked by validation"
```

**Sample output:**
```
============================================================
TaskManager Pre-Deployment Validation
============================================================

Check 1: Running local eval cases... ✓ PASS (10/10)
Check 2: Validating agent structure... ✓ PASS (all 6 tools registered)
Check 3: Testing Firestore connectivity... ✓ PASS (Firestore read/write working)
Check 4: Testing error handling... ✓ PASS (all error cases handled)
Check 5: Testing response times... ✓ PASS (avg: 0.89s, max: 1.23s)
Check 6: Verifying callbacks... ✓ PASS (both callbacks registered)

============================================================
Pre-Deployment Validation Report
============================================================
Passed: 6/6
Failed: 0/6

✓ Agent is ready for deployment!
============================================================
```

</details>

---

## Generate Your Deployment Script

You've verified your agent locally and now need to automate the deployment process. Generate a complete deployment script that handles all pre-deployment checks, deployment to Vertex AI, and verification.

### Generate Deployment Automation

**Prompt for AI:**

```
Create a production-ready deployment script for a Vertex AI Agent Engine:

1. Pre-deployment validation:
   - Check all eval cases pass locally
   - Verify SessionService configured for production
   - Validate callbacks are registered and functional
   - Confirm GCP project setup (APIs enabled, permissions set)

2. Deploy command wrapper:
   - adk deploy agent_engine with proper config
   - Wait for deployment completion
   - Extract and save resource endpoint

3. Remote verification:
   - Run eval cases against live endpoint
   - Compare pass rates with local baseline
   - Confirm behavior matches (95%+ match required)

4. Rollback if remote evals fail:
   - Log failure details
   - Save backup of previous version
   - Provide rollback instructions

5. Safety gates:
   - Rate limiting check
   - Callback function verification
   - Error handling validation

Include comprehensive error handling, logging to Cloud Logging, and exit codes for CI/CD integration.
```

**Deploy and verify:**

```bash
# Run full deployment process
./deploy_to_vertex_ai.py --project=$GCP_PROJECT --staging-bucket=gs://your-bucket

# Expected output:
# ✓ All local eval cases pass (10/10)
# ✓ Deployment successful - Endpoint: projects/YOUR_PROJECT/locations/us-central1/reasoningEngines/engine-abc123
# ✓ Remote eval cases pass (10/10)
# ✓ Behavior matches local testing
# ✓ Ready for production traffic

# Manual verification (if needed)
adk eval ./taskmanager evals/taskmanager_evals.json --endpoint=projects/YOUR_PROJECT/locations/us-central1/reasoningEngines/engine-abc123
```

**What you're learning**: Deployment automation reduces human error and creates a repeatable, CI/CD-ready process. The script becomes your deployment safety net—it verifies nothing unexpected happened before production.

---

## Try With AI

Use your AI companion to deepen deployment expertise and address production concerns.

### Prompt 1: Troubleshoot Deployment Failure

```
I tried to deploy my agent to Vertex AI but got this error:

[paste error message from adk deploy]

Help me diagnose. What could cause this error?
Ask me questions about:
- My Google Cloud setup (API enablement, authentication)
- Agent structure (syntax errors, import issues)
- Environment (Python version, dependencies)
- Permissions (IAM roles, service accounts)
```

**What you're learning**: Deployment troubleshooting—understanding the chain from local development to cloud infrastructure. Production skills matter because deployment failures are expensive.

### Prompt 2: Design a Rollout Strategy

```
I'm deploying a customer-facing task agent to production. I'm worried
about what could go wrong. Help me design a safe rollout strategy.

Ask me about:
- How many users will this serve?
- What's the cost of failure (how bad if it breaks)?
- How much monitoring can I set up?
- What's our rollback plan if something breaks?

Then suggest a rollout plan with specific traffic percentages and
monitoring gates.
```

**What you're learning**: Production thinking—understanding that deployment is not a single moment, but a process of verification, monitoring, and gradual increase in user exposure. This is how enterprises operate.

### Prompt 3: Compare Deployment Options

```
I'm choosing where to deploy: Vertex AI Agent Engine vs Cloud Run.
The lesson recommends Agent Engine, but I want to understand tradeoffs.

Help me compare by asking about my requirements:
- Scale: How many concurrent users?
- Latency: How fast must responses be?
- Cost: What's my budget?
- Control: Do I need custom infrastructure?
- Monitoring: What observability do I need?

Then recommend which deployment option fits best and why.
```

**What you're learning**: Architecture decisions—understanding that deployment choice isn't technical trivia, it's strategic decision-making about cost, scale, and control. Different projects need different solutions.

### Safety Consideration in Production Deployments

Before routing real user traffic to your deployed agent: verify all eval cases pass on the remote endpoint (not just locally), enable monitoring and set up alerts for error spikes, test error conditions manually (empty input, invalid data, timeout scenarios), implement rate limiting to prevent quota overload, use gradual rollout (10% → 50% → 100%) for production traffic, and have a rollback plan ready in case behavior degrades. Deployment is not a release event—it's the start of production operation. Verification ensures your agent continues to work reliably under real-world conditions.
