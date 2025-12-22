---
sidebar_position: 19
chapter: 50
lesson: 19
duration_minutes: 45
title: "AI-Assisted Kubernetes with kubectl-ai"
proficiency_level: B1
teaching_stage: 2
stage_name: "AI Collaboration"
stage_description: "Collaborating with AI to translate natural language into Kubernetes operations"
cognitive_load:
  concepts_count: 5
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Use kubectl-ai to generate Kubernetes manifests from natural language descriptions"
    bloom_level: "Apply"
  - id: LO2
    description: "Debug cluster issues through conversational dialogue with AI"
    bloom_level: "Analyze"
  - id: LO3
    description: "Iterate on generated configurations by expressing constraints and requirements"
    bloom_level: "Evaluate"
  - id: LO4
    description: "Apply kubectl foundation knowledge to validate AI-generated manifests"
    bloom_level: "Evaluate"
  - id: LO5
    description: "Recognize when AI-generated configurations need refinement based on production context"
    bloom_level: "Analyze"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.3 Programming"
  - objective_id: LO2
    competency_area: "5. Problem Solving"
    competency: "5.4 Using digital tools creatively"
  - objective_id: LO3
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
  - objective_id: LO4
    competency_area: "1. Information and Data Literacy"
    competency: "1.3 Managing data"
  - objective_id: LO5
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
---

# AI-Assisted Kubernetes with kubectl-ai

By Lesson 7, you've learned Kubernetes concepts deeply: control plane architecture, pod lifecycle, deployments with rolling updates, services, configuration injection, resource management. You can read manifests and understand why each field matters.

Now comes the efficiency problem: Writing Kubernetes manifests by hand is verbose. A deployment requires apiVersion, kind, metadata, spec, replicas, selectors, template specifications, resource requests, and health checks. That's 50+ lines for a simple deployment.

This is where kubectl-ai bridges the gap. Instead of hand-typing manifests, you describe what you want in natural language. kubectl-ai generates the YAML. You review it against your L1 knowledge, suggest improvements, and iterate—collaborating toward a production-ready manifest much faster than manual typing.

---

## Installing and Using kubectl-ai

kubectl-ai is a kubectl plugin that translates natural language commands to Kubernetes operations. It leverages LLM reasoning to understand intent and generate correct manifests.

### Installation

kubectl-ai extends kubectl through the plugin system. Install it:

```bash
# Install via pip (requires Python 3.8+)
pip install kubectl-ai

# Or via Homebrew (macOS/Linux)
brew install kubectl-ai

# Verify installation
kubectl ai --version
```

**Output:**
```
kubectl-ai version 0.2.0
```

Once installed, kubectl-ai integrates as a native kubectl plugin. You can invoke it with `kubectl ai` or `kubectl ai --help` to see available commands.

### How It Works: Three Interaction Patterns

kubectl-ai supports three core interaction patterns, each suited to different situations:

**Pattern 1: Generate Manifests from Description**

```bash
kubectl ai "Deploy nginx with 3 replicas, expose on port 80"
```

The plugin translates your description into a valid Deployment manifest, complete with selectors, resource requests, and best practices.

**Pattern 2: Generate Kubectl Commands from Intent**

```bash
kubectl ai "Show me all pods that are not in running state"
```

kubectl-ai converts this intent into `kubectl get pods --field-selector=status.phase!=Running`.

**Pattern 3: Explain Existing Manifests or Commands**

```bash
kubectl ai --explain "kubectl apply -f deployment.yaml"
```

The plugin explains what a kubectl command does, useful for understanding operational patterns.

This lesson focuses on Pattern 1 (manifest generation), which is where AI collaboration provides the most value: reducing manual typing while teaching you to evaluate quality.

---

## From Natural Language to Production Manifest: A Collaboration Journey

To show how kubectl-ai works in practice, let's walk through a realistic scenario: deploying your FastAPI agent from Chapter 49 to Kubernetes.

### Your Starting Point

You have:
- A Docker image `my-agent:1.0` pushed to Docker Hub
- Knowledge of your agent's resource requirements (256MB RAM, 100m CPU)
- A health check endpoint at `/health` that returns `{"status": "healthy"}`
- A requirement for 3 replicas in production

### Round 1: Initial Request to kubectl-ai

You start with a straightforward description:

**Your request:**
```
Generate a Kubernetes Deployment for my FastAPI agent.
Use image my-agent:1.0 from Docker Hub.
3 replicas.
Expose port 8000.
Name it my-agent-deployment.
```

**kubectl-ai generates:**

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-agent-deployment
  namespace: default
spec:
  replicas: 3
  selector:
    matchLabels:
      app: my-agent
  template:
    metadata:
      labels:
        app: my-agent
    spec:
      containers:
      - name: my-agent
        image: my-agent:1.0
        ports:
        - containerPort: 8000
```

**Your evaluation** (using L1 knowledge from Lessons 1-7):

You review this manifest against what you learned:

- ✓ The selector and labels match (lesson 3)
- ✓ ReplicaSet will ensure 3 pods (lesson 4)
- ✓ Container port is correct
- ✗ Missing: Resource requests/limits (should have defined in lesson 7)
- ✗ Missing: Health checks—the `/health` endpoint exists but isn't configured
- ✗ Missing: Image pull policy (what if the image isn't present?)

The manifest is functionally correct but incomplete for production. This is exactly the collaboration moment.

### Round 2: Teaching kubectl-ai Your Constraints

You describe what the initial manifest missed:

**Your refined request:**
```
Update the deployment. The agent uses 256MB RAM and 100m CPU.
Add a liveness probe on /health endpoint (port 8000, path /health).
Add a readiness probe on the same endpoint.
The image might not be on the node, so use imagePullPolicy: IfNotPresent.
```

**kubectl-ai generates:**

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-agent-deployment
  namespace: default
spec:
  replicas: 3
  selector:
    matchLabels:
      app: my-agent
  template:
    metadata:
      labels:
        app: my-agent
    spec:
      containers:
      - name: my-agent
        image: my-agent:1.0
        imagePullPolicy: IfNotPresent
        ports:
        - containerPort: 8000
        resources:
          requests:
            memory: "256Mi"
            cpu: "100m"
          limits:
            memory: "512Mi"
            cpu: "200m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 10
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5
```

**Your evaluation:**

Now you're comparing a more complete manifest against your L1 foundation:

- ✓ Resources defined with requests and limits (lesson 7)
- ✓ Probes configured with httpGet on the health endpoint (lesson 7)
- ✓ Image pull policy prevents problems if image isn't cached locally
- ✓ Delay and period values are reasonable for a startup-sensitive agent

BUT:

- ✗ The limits are 2x the requests (512Mi memory, 200m CPU). For a development agent, this might be excessive
- ✗ Initial delay of 10 seconds—what if your agent takes longer to start up initially?

### Round 3: Production Context Refinement

You have production experience: your agent sometimes takes 20+ seconds to initialize (it loads models). Also, the resource limits seem generous for a development cluster:

**Your request:**
```
Increase the liveness probe initialDelaySeconds to 20.
Reduce the memory limit to 384Mi (the agent doesn't use all 512Mi).
Keep the CPU request and limit—those look right.
Add a nice name for the container.
```

**kubectl-ai generates:**

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-agent-deployment
  namespace: default
spec:
  replicas: 3
  selector:
    matchLabels:
      app: my-agent
  template:
    metadata:
      labels:
        app: my-agent
    spec:
      containers:
      - name: fastapi-agent
        image: my-agent:1.0
        imagePullPolicy: IfNotPresent
        ports:
        - name: http
          containerPort: 8000
        resources:
          requests:
            memory: "256Mi"
            cpu: "100m"
          limits:
            memory: "384Mi"
            cpu: "200m"
        livenessProbe:
          httpGet:
            path: /health
            port: http
          initialDelaySeconds: 20
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5
```

**What changed** through iteration:

- Memory limit reduced from 512Mi to 384Mi (more realistic)
- Liveness probe delay increased to 20 seconds (your agent's startup time)
- Container named `fastapi-agent` instead of generic `my-agent`
- Port named `http` (allows referencing by name instead of number)

None of these changes were obvious from your initial description. But through dialogue—describing constraints and production experience—the manifest evolved toward a configuration that actually reflects how your agent behaves.

---

## Debugging with kubectl-ai: Collaborative Problem-Solving

kubectl-ai isn't just for generation. It's valuable for debugging too. When something goes wrong, you can describe the symptom and iterate on solutions.

### Scenario: Pod Not Starting

You deploy and pods are stuck in `CrashLoopBackOff`. You don't know why.

**Your prompt:**

```
My deployment is in CrashLoopBackOff.
The container starts but exits immediately.
How do I investigate this with kubectl commands?
What should I look for in the logs?
```

**kubectl-ai suggests:**

```bash
# Check pod status
kubectl describe pod <pod-name>

# Read logs from the crashed container
kubectl logs <pod-name> --previous

# Check for resource constraints causing OOM
kubectl describe node <node-name>

# Look for event details
kubectl get events --sort-by='.lastTimestamp'
```

You run `kubectl logs <pod-name> --previous` and see:

```
ImportError: No module named 'transformers'
```

Your Python environment is missing dependencies. This insight came from the collaborative debugging pattern: you described the symptom, kubectl-ai suggested diagnostic commands, and you got the information needed to fix the root cause.

### Scenario: Service Discovery Isn't Working

Your frontend can't reach your backend pods:

**Your description:**

```
My frontend pod is trying to reach the backend service.
It's using the DNS name my-backend-service.
But it times out trying to connect.
What could be wrong?
Generate commands to debug this.
```

**kubectl-ai generates diagnostic steps:**

```bash
# Verify the service exists
kubectl get svc my-backend-service

# Check if service has endpoints
kubectl get endpoints my-backend-service

# Try DNS resolution from a pod
kubectl run -it --rm debug --image=busybox -- nslookup my-backend-service

# Check if pods are actually running and healthy
kubectl get pods -l app=my-backend --show-labels

# Verify the port matches the service definition
kubectl describe svc my-backend-service
```

Running these steps, you discover: The backend service has zero endpoints. The label selector doesn't match any running pods. You add the correct labels, and service discovery works.

This is the core value of kubectl-ai for debugging: It helps you think through diagnostic steps without having to memorize kubectl command syntax.

---

## Critical Evaluation: Why Manual Knowledge Matters

The examples above show the collaborative pattern working. But they also highlight why your L1 foundation from Lessons 1-7 is essential.

### Example 1: Recognizing Over-Specification

kubectl-ai generates:

```yaml
resources:
  limits:
    memory: "512Mi"
    cpu: "500m"
```

Without L1 knowledge (from Lesson 7), you might accept these limits as correct. But you know from lesson 7:
- QoS tier "Guaranteed" requires requests == limits
- For a development agent, you probably want "Burstable" (requests < limits)
- 500m CPU is excessive for a FastAPI service doing inference, not compute

Your evaluation prevents a misconfigurations that would waste cluster resources.

### Example 2: Recognizing Missing Health Checks

kubectl-ai might generate:

```yaml
# No livenessProbe or readinessProbe
```

Without L1 knowledge (Lesson 7), you might deploy this. Kubernetes would run your pods, but:
- If a pod hangs (responds but doesn't process), Kubernetes doesn't know to restart it
- If a pod is starting, Kubernetes might send traffic before it's ready

Your L1 knowledge flags this as incomplete and you request the health checks.

### Example 3: Catching Image Pull Issues

kubectl-ai generates:

```yaml
imagePullPolicy: Always
```

This works for public images, but fails if:
- Your image is in a private registry (no credentials specified)
- You're testing locally with Minikube (image not available remotely)

Only knowing Kubernetes fundamentals (Lesson 3: image pull behavior) lets you catch and correct this.

**The pattern**: kubectl-ai generates manifests following general best practices. But you evaluate them through domain knowledge and production context. That evaluation catches issues before they fail in production.

---

## Iterative Refinement in Practice: Your Agent Deployment

Let's walk through what iteration looks like across multiple rounds:

**Round 1 Request:**
```
Create a Deployment for my agent (image: my-agent:1.0).
3 replicas.
Port 8000.
```

**kubectl-ai output:** Basic deployment (Lesson 4 level)

**Your feedback:** "Add resource limits and health checks because this is production."

**Round 2 Request:**
```
The agent needs 256MB RAM and 100m CPU.
Add health checks on /health endpoint.
```

**kubectl-ai output:** Deployment with resources and probes (Lesson 7 level)

**Your feedback:** "The initialization is slow—20 seconds before the agent is ready. Update the liveness delay. Also, the memory limit seems high—should be 384Mi max."

**Round 3 Request:**
```
Set liveness initialDelaySeconds to 20.
Set memory limit to 384Mi.
```

**kubectl-ai output:** Refined deployment

**Your evaluation:**
- ✓ Matches your constraints
- ✓ Production-appropriate for your use case
- ✓ Ready to deploy

The journey from Round 1 to Round 3 shows how collaborative iteration works:
1. Start with simplicity (Round 1)
2. Add constraints your domain knowledge recognizes (Round 2)
3. Refine based on actual behavior (Round 3)
4. Deploy with confidence

This is more efficient than hand-writing all 50+ lines while researching each field in the kubectl docs. But it requires your L1 foundation to evaluate quality.

---

## When to Use kubectl-ai (And When Not To)

### Perfect Use Cases

**1. Generating boilerplate from requirements**

```
"Deploy Redis with persistence, memory limit 2Gi, replicas 1"
```

Instead of hand-typing a StatefulSet, redis config, persistent volume, and service, describe what you need and iterate.

**2. Debugging unknown kubectl commands**

```
"I need to check why this pod keeps restarting. Show me the commands."
```

More efficient than searching documentation.

**3. Exploring alternatives**

```
"What's the difference between using a ConfigMap vs environment variables for configuration?"
```

Quick explanation with examples.

### When Manual Writing Is Better

**1. Complex architectural changes**

If you're redesigning a multi-service deployment, writing the spec by hand forces you to think through relationships. AI generation might miss architectural intent.

**2. Security-sensitive configurations**

Secrets management, RBAC policies, network policies. Review these line-by-line manually, not through AI suggestions.

**3. Teaching others**

When training team members, hand-written manifests with annotations teach better than AI-generated ones.

### The Balanced Approach

Use kubectl-ai for:
- Initial scaffolding
- Syntax generation
- Diagnostic commands

Then review, refine, and customize based on your domain knowledge. This combines AI efficiency with human judgment.

---

## Try With AI

**Setup**: You have a containerized FastAPI agent from Chapter 49 (image: `my-agent:1.0` on Docker Hub). You need to deploy it to Kubernetes with the following requirements:

- 2 replicas (development deployment)
- Port 8000
- Health check endpoint: `/api/health` (returns JSON)
- Memory allocation: 512MB request, 1GB limit
- CPU allocation: 250m request, 500m limit
- Image pull from Docker Hub (public image)
- Container should wait 15 seconds before liveness check (agent startup time)

### Part 1: Initial Generation

Ask kubectl-ai to generate the deployment manifest based on these requirements:

```
kubectl ai "Generate a Deployment manifest for my FastAPI agent.
Image: my-agent:1.0 from Docker Hub.
2 replicas.
Port 8000.
Health check on /api/health endpoint.
512MB memory request, 1GB limit.
250m CPU request, 500m CPU limit.
15 second startup delay for health checks."
```

### Part 2: Critical Review

Review the generated manifest. Use your L1 knowledge from Lessons 1-7 to evaluate:

- Does it have the right labels for service discovery?
- Are the resource requests and limits correctly specified?
- Do the probes have reasonable initial delay and period values?
- Is the image pull policy appropriate?
- Does it specify the health check port correctly?

Make note of anything that looks incomplete or incorrect.

### Part 3: Constraint Refinement

Based on your review, provide kubectl-ai with feedback. For example:

```
The manifest looks good overall. But:
- The memory limit should be 768Mi, not 1GB.
- Use imagePullPolicy: IfNotPresent for development.
- The readiness probe should respond faster (initialDelaySeconds 5).
```

Ask kubectl-ai to update the manifest with these constraints.

### Part 4: Validation Check

Compare the updated manifest against the original:

- What changed?
- Why did those changes make sense?
- Does the updated manifest match your actual agent requirements better?

### Part 5: Practical Deployment Decision

Looking at the final manifest, ask yourself:

- Could you deploy this to your cluster immediately?
- Would you modify anything else before applying it?
- What would you monitor after deployment (based on resource limits and probes)?
- If a pod crashed, where would you look first to understand why?

This is the practical thinking that complements AI generation—using your kubectl foundation to make confident production decisions.
