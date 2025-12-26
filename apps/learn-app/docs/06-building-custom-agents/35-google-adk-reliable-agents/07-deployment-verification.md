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
