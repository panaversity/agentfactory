---
sidebar_position: 21
chapter: 50
lesson: 21
duration_minutes: 50
title: "Capstone: Deploy Your Part 6 Agent to Kubernetes"
proficiency_level: B1
teaching_stage: 4
stage_name: "Spec-Driven Integration"
stage_description: "Write deployment specification, generate manifests, deploy and validate"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Minimal"
learning_objectives:
  - id: LO1
    description: "Write a deployment specification that documents requirements before implementation"
    bloom_level: "Create"
  - id: LO2
    description: "Generate complete Kubernetes manifest suite from specification"
    bloom_level: "Apply"
  - id: LO3
    description: "Deploy containerized agent to Kubernetes cluster with configuration injection"
    bloom_level: "Apply"
  - id: LO4
    description: "Validate deployment success including health checks and external access"
    bloom_level: "Analyze"
  - id: LO5
    description: "Demonstrate self-healing by deleting Pod and verifying automatic recovery"
    bloom_level: "Analyze"
skills:
  kubernetes_deployment:
    proficiency: B1
digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO4
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO5
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
---

# Capstone: Deploy Your Part 6 Agent to Kubernetes

Your FastAPI agent from Chapter 49 is containerized and pushed to a registry. Now deploy it to a production-like environment using Kubernetes.

This capstone differs from earlier lessons: **you start with a specification, not a command**. Write out what you're deploying and why. Then have AI generate the manifests that implement your specification. Finally, deploy your agent, configure it with secrets and environment variables, and validate that Kubernetes is managing it correctly (scaling, self-healing, logging).

By the end, your agent will run on a Kubernetes cluster, surviving Pod crashes and responding to scaling demands—all orchestrated by Kubernetes' declarative model.

---

## The Specification-First Approach

Before you write a single line of YAML, specify what you're building.

A specification answers these questions:
- **What** are we deploying? (What's the container? Where is it?)
- **Why** are we deploying it this way? (What constraints drive our choices?)
- **How many** copies should run? (What's the replicas count based on?)
- **What configuration** does it need? (Environment variables, secrets, resource limits?)
- **How do we validate** it's working? (Health checks, access tests, what success looks like?)

Here's a template to guide your thinking:

```
## Deployment Specification: [Agent Name]

### Intent
[1-2 sentences: What problem does this solve? Why deploy to Kubernetes?]

### Container Details
- Image: [registry/image:tag]
- Port: [Which port does your agent listen on?]
- Image pull policy: [Always/IfNotPresent]

### Replicas & Scaling
- Desired replicas: [number]
- Why this count: [Load expectations? High availability?]

### Configuration
- Environment variables needed: [List all]
  - OPENAI_API_KEY: [From Secret]
  - Other vars: [From ConfigMap]
- Resource requests: [CPU/memory your agent needs]
- Resource limits: [Upper bounds to prevent resource contention]

### Health Checks
- Readiness probe: [What endpoint shows the agent is ready?]
- Liveness probe: [What endpoint proves the agent is still alive?]

### Networking
- Service type: [ClusterIP/NodePort/LoadBalancer]
- External access: [Do external clients need to reach this? How?]

### Success Criteria
- [ ] Deployment creates all replicas
- [ ] Service is accessible (internally or externally)
- [ ] Health checks pass
- [ ] Environment variables are injected correctly
- [ ] Pod recovery works (delete Pod, verify automatic restart)
```

This specification becomes the contract between you and Kubernetes. AI will read it and generate manifests that fulfill the contract.

---

## Writing Your Specification

Before proceeding to AI, write your specification in a text editor or document.

### Key Details for Your Part 6 Agent

**Container image**: This is the image you pushed in Chapter 49. Remember the format:
```
[registry]/[repository]:[tag]
```

Example: `ghcr.io/yourusername/part6-agent:latest`

**Port**: Your FastAPI agent listens on port 8000 by default (unless you configured differently in Chapter 49).

**Environment variables**: Your agent needs:
- `OPENAI_API_KEY`: Your OpenAI API key (should be in a Secret for security)
- Any other config from Chapter 49? (Database URLs, model names, API endpoints?)

**Readiness and liveness probes**: FastAPI's health check endpoint is typically `/health` or you can use the root endpoint `/` and check for HTTP 200.

**Replicas**: For learning purposes, 2-3 replicas is appropriate (shows redundancy). For production load, you'd scale higher.

**Resources**: A FastAPI agent with OpenAI calls is lightweight:
- Request: CPU 100m (100 millicores), Memory 256Mi
- Limit: CPU 500m, Memory 512Mi

**Service exposure**: For Minikube development, NodePort is practical. For cloud clusters, LoadBalancer or Ingress.

---

## Example Specification (For Reference)

Here's what a completed specification looks like:

```
## Deployment Specification: Part 6 AI Agent

### Intent
Deploy the Part 6 FastAPI agent to Kubernetes with automatic scaling and
self-healing. Agent should survive Pod crashes and scale to handle load
variations.

### Container Details
- Image: ghcr.io/yourusername/part6-agent:v1.0
- Port: 8000
- Image pull policy: Always (always fetch latest image)

### Replicas & Scaling
- Desired replicas: 3
- Why this count: High availability (can lose 1 Pod without service interruption)

### Configuration
- Environment variables needed:
  - OPENAI_API_KEY: From Secret named "agent-secrets"
  - LOG_LEVEL: From ConfigMap, set to "INFO"
  - API_TIMEOUT: From ConfigMap, set to "30s"
- Resource requests: CPU 100m, Memory 256Mi
- Resource limits: CPU 500m, Memory 512Mi

### Health Checks
- Readiness probe: GET /health (HTTP 200 means ready)
- Liveness probe: GET /health (HTTP 200 means alive)
- Both: initial delay 5 seconds, check every 10 seconds

### Networking
- Service type: NodePort
- External access: Yes—external clients access agent via service on port 30080

### Success Criteria
- [ ] Deployment creates 3 replicas
- [ ] Service is accessible via kubectl port-forward or NodePort
- [ ] Health checks pass
- [ ] OPENAI_API_KEY is injected correctly (from Secret)
- [ ] Pod recovery works (delete a Pod, verify new one starts automatically)
```

Use this as a model, then customize it for YOUR agent and deployment preferences.

---

## Generating Manifests from Specification

Once you've written your specification, you have two approaches:

**Approach 1: Manual (Educational)**
- Write each manifest yourself based on your specification
- Good for learning—you control every line
- Time-intensive

**Approach 2: AI-Assisted (Practical)**
- Copy your specification into a prompt
- Have AI generate the complete manifest suite
- Then review and validate the output

For this capstone, **use Approach 2**—this demonstrates the core AI-native workflow: Specification → AI Generation → Validation.

### The Prompt Structure for AI

When asking AI to generate manifests, provide:

1. **Your specification** (copy/paste)
2. **Context about your agent**: What does it do? What does it need?
3. **Deployment environment**: Are you using Minikube or a cloud cluster?
4. **Success criteria**: What must the manifests include to pass validation?

Here's a template:

```
I need to deploy a containerized AI agent to Kubernetes.

## Deployment Specification
[Paste your specification here]

## Agent Context
- Container image: [image]
- Port: [port]
- Environment needs: [list]

## Deployment Environment
[Minikube on localhost OR Cloud cluster (GKE/EKS/AKS)]

## Manifest Requirements
Generate the following:
1. ConfigMap: Store LOG_LEVEL and API_TIMEOUT
2. Secret: Store OPENAI_API_KEY
3. Deployment:
   - Image from my specification
   - Inject ConfigMap and Secret as environment variables
   - Include readiness and liveness probes
   - Set resource requests and limits
4. Service:
   - Type: NodePort (for Minikube)
   - Expose port 8000 from container
   - Target port 30080 (or similar)

Ensure all manifests use the same labels and selectors so the Service routes to the Deployment Pods.

Return YAML that I can immediately apply with `kubectl apply -f manifest.yaml`
```

---

## Deployment Steps

Once you have your manifests (either written by hand or generated by AI), deploy them to your Kubernetes cluster.

### Step 1: Verify Cluster Access

```bash
kubectl cluster-info
kubectl get nodes
```

If using Minikube, ensure it's running:
```bash
minikube start
```

### Step 2: Create Namespace (Optional but Recommended)

```bash
kubectl create namespace agent-app
kubectl config set-context --current --namespace=agent-app
```

This isolates your deployment and makes cleanup easier.

### Step 3: Apply Manifests

```bash
kubectl apply -f configmap.yaml
kubectl apply -f secret.yaml
kubectl apply -f deployment.yaml
kubectl apply -f service.yaml
```

Or apply all at once:
```bash
kubectl apply -f *.yaml
```

### Step 4: Verify Deployment

```bash
# Check Deployment status
kubectl get deployment

# Check Pods are running
kubectl get pods

# Check Service is created
kubectl get svc

# Detailed Pod info
kubectl describe pod [pod-name]
```

### Step 5: Test External Access

**For Minikube + NodePort**:
```bash
# Get the NodePort (e.g., 30080)
kubectl get svc agent-service -o jsonpath='{.spec.ports[0].nodePort}'

# Get Minikube IP
minikube ip

# Access your agent
curl http://[minikube-ip]:[nodeport]/health
```

**For kubectl port-forward** (alternative):
```bash
kubectl port-forward svc/agent-service 8000:8000
curl http://localhost:8000/health
```

**For cloud cluster + LoadBalancer**:
```bash
kubectl get svc agent-service
# Wait for EXTERNAL-IP to populate, then:
curl http://[external-ip]:8000/health
```

---

## Validation Checklist

Run through this checklist to confirm your deployment succeeded:

### Deployment Health

- [ ] **All Pods are Running**
  ```bash
  kubectl get pods
  # All should show STATUS: Running
  ```

- [ ] **Desired Replicas Match Actual**
  ```bash
  kubectl get deployment
  # DESIRED, CURRENT, READY should all be equal
  ```

- [ ] **Readiness Probes Pass**
  ```bash
  kubectl get pods
  # All Pods should show READY 1/1
  ```

### Configuration Injection

- [ ] **Environment Variables are Set**
  ```bash
  kubectl exec [pod-name] -- env | grep OPENAI_API_KEY
  # Should show the key is populated
  ```

- [ ] **ConfigMap Values are Injected**
  ```bash
  kubectl exec [pod-name] -- env | grep LOG_LEVEL
  # Should show your configured value
  ```

### Networking & Access

- [ ] **Service Exists and Routes to Pods**
  ```bash
  kubectl get endpoints agent-service
  # Should show IP addresses of Pods
  ```

- [ ] **Agent Responds to Health Check**
  ```bash
  curl http://[service-ip]:[port]/health
  # Should return HTTP 200
  ```

- [ ] **External Access Works** (NodePort or LoadBalancer)
  ```bash
  # Test from outside the cluster
  curl http://[external-ip]:[port]/health
  ```

### Self-Healing Verification

- [ ] **Pod Recovery Works**
  ```bash
  # Record initial Pod name
  kubectl get pods
  POD_NAME=$(kubectl get pods -o jsonpath='{.items[0].metadata.name}')

  # Delete a Pod
  kubectl delete pod $POD_NAME

  # Wait a few seconds, verify new Pod was created
  kubectl get pods
  # New Pod should be Running (different name)
  ```

- [ ] **Desired State Maintains Replicas**
  ```bash
  # Delete multiple Pods
  kubectl delete pod --all

  # Wait for recovery
  kubectl get pods
  # Should have recreated all replicas automatically
  ```

### Logging Verification

- [ ] **Logs are Accessible**
  ```bash
  kubectl logs [pod-name]
  # Should see your agent's startup logs
  ```

- [ ] **Recent Requests are Logged**
  ```bash
  # After making a request to your agent:
  kubectl logs [pod-name] --tail=20
  # Should see request in logs
  ```

---

## Try With AI

Use AI to help debug issues and explore advanced deployment scenarios. This section demonstrates the three-role collaboration that makes AI-native development effective.

### Part 1: Validate Your Manifests

**Ask AI**:
```
Review my Kubernetes manifests for correctness. I want to deploy a FastAPI
agent to Minikube. Check for:
- Correct label selectors (Service routes to Deployment Pods)
- Proper environment variable injection (ConfigMap and Secret)
- Appropriate resource requests/limits for a lightweight FastAPI app
- Valid readiness/liveness probe configuration

Here are my manifests:
[Paste your YAML files]

Are there any issues that would prevent successful deployment?
```

**What to evaluate**:
- Does AI catch obvious errors (wrong image name, mismatched selectors)?
- Are the suggestions aligned with your specification?
- Did you miss any configuration that AI suggests?

### Part 2: Troubleshoot Deployment Issues

If your deployment fails, use AI to diagnose:

**Ask AI**:
```
My Deployment isn't creating Pods. Here's what I see:

kubectl get deployment:
[Paste output]

kubectl describe deployment agent-deployment:
[Paste output]

kubectl get events:
[Paste output]

What's preventing the Pods from starting?
```

**What to evaluate**:
- AI correctly interprets kubectl output
- AI identifies the root cause (image pull error, resource constraints, etc.)
- AI's suggestion points to a specific fix you can try

**Then refine**: Based on AI's analysis, describe what you tried:
```
I applied your fix:
[Describe what you did]

But now I see this error:
[New error message]

What should I try next?
```

### Part 3: Validate Self-Healing Behavior

**Ask AI**:
```
I want to confirm Kubernetes is actually self-healing my Pods. Here's my
current deployment state:

kubectl get pods:
[Paste output showing 3 Pods]

Now I'm going to delete one Pod. Here's what happens:

[Delete Pod and immediately show output]

Then after 10 seconds:
[Show output again]

Did Kubernetes correctly recover and replace the deleted Pod?
```

**What to evaluate**:
- AI correctly interprets the before/after Pod states
- AI confirms that a new Pod was created (different name, recent age)
- AI explains why this proves self-healing works

### Part 4: Test External Access

**Ask AI**:
```
I'm trying to access my agent from outside the cluster. Here's what I tried:

minikube ip: [output]
kubectl get svc: [output showing NodePort]
curl http://[ip]:[port]/health: [response or error]

Why isn't my agent responding?
```

**What to evaluate**:
- AI identifies if port forwarding is needed vs NodePort vs LoadBalancer
- AI suggests debugging steps specific to your deployment environment
- AI's advice is actionable and tests the right thing

### Part 5: Final Validation Synthesis

**Ask AI**:
```
I've completed the deployment and run the validation checklist. Here's
what I confirmed:

[Paste results of your validation checks]

Based on this evidence, should I consider this capstone complete? What
would you look for as proof that deployment succeeded?
```

**What to evaluate**:
- AI acknowledges which checks you've completed successfully
- AI identifies any gaps remaining before the capstone is done
- AI confirms that your specification has been fully validated in practice

---

**Remember**: The goal of this capstone is not just to deploy your agent, but to understand how specification-first development works. Your specification is the contract. The manifests implement it. Kubernetes ensures the contract is maintained (desired state = observed state, always).

When self-healing works (Pod dies, new one starts), you're seeing the declarative model in action.

