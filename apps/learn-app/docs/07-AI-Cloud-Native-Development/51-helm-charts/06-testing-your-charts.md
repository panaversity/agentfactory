---
sidebar_position: 6
chapter: 51
lesson: 6
duration_minutes: 45
title: "Testing Your Charts"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Validate charts before production with lint, template, and test commands"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Use helm lint to catch structural errors"
    bloom_level: "Apply"
  - id: LO2
    description: "Debug template rendering with helm template --debug"
    bloom_level: "Apply"
  - id: LO3
    description: "Create test pods that verify deployment connectivity"
    bloom_level: "Create"
  - id: LO4
    description: "Execute chart tests with helm test"
    bloom_level: "Apply"
  - id: LO5
    description: "Understand when to use unit vs integration testing"
    bloom_level: "Understand"
---

# Testing Your Charts

In Lessons 1-5, you built a production-ready Helm chart: templates that render correctly, values that adapt to environments, and helper functions that eliminate boilerplate. But before deploying to production, you need confidence that the chart works as intended. This means catching errors early through validation, verifying template rendering produces correct manifests, and testing that deployed resources actually function.

This lesson teaches the testing safety net that stands between a working chart in development and a broken deployment in production. You'll use `helm lint` to validate chart structure, `helm template` to inspect rendered manifests, test pods to verify connectivity, and `helm test` to run validation against live deployments. Together, these tools catch the mistakes that would otherwise cost hours to debug in production.

---

## Concept 1: Validating Charts with helm lint

`helm lint` checks your chart for structural errors, missing required fields, and configuration problems. It runs before installation, catching issues early.

### Understanding helm lint Output

Create a chart with an intentional error to see how lint catches it:

```bash
# Create test chart
mkdir -p test-chart/templates
cd test-chart

# Create Chart.yaml with missing required field
cat > Chart.yaml <<'EOF'
name: test-chart
version: 1.0.0
# Note: description is missing—required field
EOF

# Create broken template
cat > templates/deployment.yaml <<'EOF'
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ .Values.appName }}
  # Invalid indentation in YAML
spec:
  replicas: {{ .Values.replicaCount | invalid_function }}
EOF

# Create values.yaml
cat > values.yaml <<'EOF'
appName: my-app
replicaCount: 3
EOF
```

**Output:**
```
$ helm lint .
==> Linting test-chart

[ERROR] Chart.yaml: Chart.metadata.description is required
[ERROR] templates/deployment.yaml: [ERROR] render error in "templates/deployment.yaml": template: test-chart/templates/deployment.yaml:6:34: executing "test-chart/templates/deployment.yaml" at <invalid_function>: function "invalid_function" not defined

Error: 1 chart(s) failed linting
```

### What helm lint Validates

| Check | Purpose |
|-------|---------|
| Chart.yaml structure | Required fields (name, version, description) |
| Template syntax | Valid Go templating, correct function names |
| YAML validity | Proper indentation, valid structure |
| Chart directory layout | `templates/`, `values.yaml` exist and have correct permissions |

### Fixing Errors: Add Required Description

```bash
cat > Chart.yaml <<'EOF'
name: test-chart
version: 1.0.0
description: A test chart for Helm testing
apiVersion: v1
appVersion: 1.0
EOF

cat > templates/deployment.yaml <<'EOF'
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ .Values.appName }}
spec:
  replicas: {{ .Values.replicaCount }}
EOF
```

**Output:**
```
$ helm lint .
==> Linting test-chart

1 chart(s) passed all tests
```

---

## Concept 2: Rendering Templates to Inspect Output

`helm template` renders your templates locally without connecting to a Kubernetes cluster. This shows exactly what manifests will be created.

### Basic Template Rendering

```bash
cd test-chart
helm template my-release .
```

**Output:**
```yaml
---
# Source: test-chart/templates/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-app
spec:
  replicas: 3
```

### Rendering with Environment-Specific Values

```bash
# Create values-prod.yaml
cat > values-prod.yaml <<'EOF'
appName: prod-agent
replicaCount: 5
EOF

# Render with production values
helm template my-release . -f values-prod.yaml
```

**Output:**
```yaml
---
# Source: test-chart/templates/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: prod-agent
spec:
  replicas: 5
```

### Why This Matters

Before deploying, you can verify:
- Variable substitution works correctly
- Conditionals produce expected output
- Resource definitions are valid Kubernetes YAML

---

## Concept 3: Debugging Template Rendering with --debug

When templates produce unexpected output, `helm template --debug` shows each rendering step with line numbers, making errors easy to locate.

### Broken Conditional Example

Create a template with a conditional that references a nonexistent value:

```bash
cat > templates/service.yaml <<'EOF'
{{- if .Values.serviceEnabled }}
apiVersion: v1
kind: Service
metadata:
  name: {{ .Values.serviceName }}
spec:
  type: {{ .Values.serviceType }}
{{- end }}
EOF
```

Without the flag, you see nothing if the value is missing. With `--debug`:

```bash
helm template my-release . --debug
```

**Output:**
```
---
# Source: test-chart/templates/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-app
spec:
  replicas: 3
# templates/service.yaml did not produce a resource (condition: if .Values.serviceEnabled is false)
```

### Using --debug for Line-by-Line Inspection

For more detailed inspection:

```bash
helm template my-release . --debug 2>&1 | grep -A 5 "service.yaml"
```

**Output shows** template variables, function evaluations, and which conditionals evaluated to true/false.

---

## Concept 4: Test Pods and the helm.sh/hook: test Annotation

Test pods are Kubernetes Pods that verify a deployed release works correctly. They're executed when you run `helm test` and removed afterward.

### Creating a Test Pod

Test pods use the `helm.sh/hook: test` annotation. Create a file `templates/test-pod.yaml`:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: {{ include "test-chart.fullname" . }}-test
  annotations:
    helm.sh/hook: test
    helm.sh/hook-delete-policy: before-hook-creation,hook-succeeded
spec:
  serviceAccountName: {{ include "test-chart.fullname" . }}
  containers:
  - name: test-agent-connectivity
    image: curlimages/curl:latest
    command:
    - sh
    - -c
    args:
    - |
      echo "Testing service connectivity..."
      curl -v http://{{ include "test-chart.fullname" . }}:8000/health
      if [ $? -eq 0 ]; then
        echo "Test PASSED: Service is reachable"
        exit 0
      else
        echo "Test FAILED: Service is not reachable"
        exit 1
      fi
  restartPolicy: Never
```

**Output (when running helm test):**
```
Pod test-chart-test created
Pod test-chart-test is still running...
Pod test-chart-test succeeded
Test suite for my-release PASSED
```

### Understanding Test Pod Annotations

| Annotation | Purpose |
|-----------|---------|
| `helm.sh/hook: test` | Marks this pod as a test resource |
| `helm.sh/hook-delete-policy: before-hook-creation` | Delete previous test pods before running new ones |
| `helm.sh/hook-delete-policy: hook-succeeded` | Delete test pod after it succeeds |

---

## Concept 5: Test Exit Codes—What Passes and What Fails

Test pods use exit codes to signal success (0) or failure (non-zero). Helm interprets these to report test results.

### Exit Code Semantics

```bash
# Test passes (exit 0)
apiVersion: v1
kind: Pod
metadata:
  name: test-pass
  annotations:
    helm.sh/hook: test
spec:
  containers:
  - name: test
    image: alpine:latest
    command:
    - sh
    - -c
    - |
      echo "Test passed"
      exit 0
  restartPolicy: Never
```

**Output:**
```
Pod test-pass created
Test pod test-pass succeeded
```

### Exit Code Semantics

```bash
# Test fails (exit non-zero)
apiVersion: v1
kind: Pod
metadata:
  name: test-fail
  annotations:
    helm.sh/hook: test
spec:
  containers:
  - name: test
    image: alpine:latest
    command:
    - sh
    - -c
    - |
      echo "Test failed: database unreachable"
      exit 1
  restartPolicy: Never
```

**Output:**
```
Pod test-fail created
Pod test-fail failed
Test suite for my-release FAILED
```

---

## Concept 6: Running Tests with helm test

`helm test` executes all test pods in a release and reports results. Run it after deploying with `helm install` or `helm upgrade`.

### Full Testing Workflow

```bash
# Step 1: Deploy the chart
helm install my-agent ./agent-chart --values agent-values.yaml
```

**Output:**
```
Release "my-agent" has been deployed.
NAME: my-agent
STATUS: deployed
REVISION: 1
```

```bash
# Step 2: Run tests against deployed release
helm test my-agent
```

**Output:**
```
Pod: my-agent-test-1
Pod: my-agent-test-1 succeeded
Pod: my-agent-test-2
Pod: my-agent-test-2 succeeded
Pod: my-agent-test-3
Pod: my-agent-test-3 succeeded
Pod: my-agent-connectivity-test
Pod: my-agent-connectivity-test succeeded

TEST SUITE: PASSED
```

### Testing with Timeout

If tests take longer than the default 300 seconds:

```bash
helm test my-agent --timeout 10m
```

**Output:**
```
Test pods running with 10m timeout...
All tests passed within timeout
```

---

## Concept 7: Unit Testing with helm-unittest Framework

For testing template logic (conditionals, loops, variable substitution), use helm-unittest. This tests chart templates without deploying to Kubernetes.

### Installing helm-unittest

```bash
helm plugin install https://github.com/helm-unit-test/helm-unittest
```

**Output:**
```
Installed helm-unittest to /Users/username/.helm/plugins/helm-unittest
```

### Writing a Unit Test

Create `tests/deployment_test.yaml`:

```yaml
suite: test deployment
templates:
  - deployment.yaml
tests:
  - name: deployment should have correct replica count
    set:
      replicaCount: 5
    asserts:
      - equal:
          path: spec.replicas
          value: 5

  - name: deployment should handle zero replicas
    set:
      replicaCount: 0
    asserts:
      - equal:
          path: spec.replicas
          value: 0

  - name: deployment should use correct image
    set:
      image:
        repository: myregistry/agent
        tag: v1.2.0
    asserts:
      - equal:
          path: spec.template.spec.containers[0].image
          value: myregistry/agent:v1.2.0
```

### Running Unit Tests

```bash
helm unittest ./agent-chart
```

**Output:**
```
suite: test deployment
 ✓ deployment should have correct replica count
 ✓ deployment should handle zero replicas
 ✓ deployment should use correct image

3 tests passed
```

---

## Concept 8: Integration Testing—Testing Deployed Resources

Integration tests verify that deployed resources (Deployments, Services, ConfigMaps) behave correctly in a live Kubernetes cluster.

### Integration Test Pattern

Create `templates/integration-test.yaml`:

```bash
cat > templates/integration-test.yaml <<'EOF'
apiVersion: v1
kind: Pod
metadata:
  name: {{ include "agent-chart.fullname" . }}-integration-test
  annotations:
    helm.sh/hook: test
    helm.sh/hook-delete-policy: before-hook-creation,hook-succeeded
spec:
  serviceAccountName: {{ include "agent-chart.fullname" . }}
  containers:
  - name: integration-test
    image: curlimages/curl:latest
    command:
    - sh
    - -c
    args:
    - |
      # Test 1: Verify Deployment has correct replicas
      REPLICAS=$(kubectl get deployment {{ include "agent-chart.fullname" . }} -o jsonpath='{.spec.replicas}')
      echo "Deployment has $REPLICAS replicas"
      if [ "$REPLICAS" != "{{ .Values.replicaCount }}" ]; then
        echo "FAILED: Expected {{ .Values.replicaCount }} replicas, got $REPLICAS"
        exit 1
      fi

      # Test 2: Verify Service exists and is accessible
      SERVICE_IP=$(kubectl get svc {{ include "agent-chart.fullname" . }} -o jsonpath='{.spec.clusterIP}')
      echo "Service IP: $SERVICE_IP"
      if [ -z "$SERVICE_IP" ]; then
        echo "FAILED: Service not found"
        exit 1
      fi

      # Test 3: Verify ConfigMap data
      CONFIG=$(kubectl get configmap {{ include "agent-chart.fullname" . }}-config -o jsonpath='{.data}')
      echo "ConfigMap data exists: $CONFIG"

      echo "All integration tests passed"
      exit 0
  restartPolicy: Never
EOF
```

**Output (when helm test runs):**
```
Pod agent-chart-integration-test created
Deployment has 3 replicas
Service IP: 10.0.0.15
ConfigMap data exists: map[model:gpt-4 logLevel:INFO]
All integration tests passed
Pod agent-chart-integration-test succeeded
```

---

## Decision Framework: Unit vs Integration Testing

| Dimension | Unit Testing (helm-unittest) | Integration Testing (Test Pods) |
|-----------|-------------------------------|--------------------------------|
| **What it tests** | Template logic (conditionals, loops, variable substitution) | Deployed resources behavior in live cluster |
| **Dependencies** | None (tests run locally) | Requires Kubernetes cluster, deployed release |
| **Speed** | Fast (seconds) | Slow (minutes—includes pod startup time) |
| **When to use** | Test chart configuration changes, template logic, edge cases | Verify deployment actually works, service connectivity |
| **Example** | "Does deployment have correct image?" | "Can the service reach the deployment?" |

**Decision rule**: Start with unit tests for template logic. Add integration tests only if deployment behavior needs verification (service connectivity, data persistence, etc.).

---

## Try With AI

### Setup

You'll test a sample Helm chart that deploys a simple agent API. Create a working directory:

```bash
mkdir helm-testing-practice
cd helm-testing-practice
```

### Part 1: Create a Chart and Run helm lint

Ask AI to create a Helm chart with intentional issues:

"Create a Helm chart for an agent service with:
- Chart.yaml with a missing description field (to trigger lint error)
- A Deployment template
- A Service template
- A values.yaml with image, replicas, and service type"

Apply the chart to your directory and run `helm lint` to verify it catches the error. Note how the error message guides you to fix it.

### Part 2: Inspect Template Rendering

Ask AI to render the chart with different values files:

"Now create a values-prod.yaml with different replica count (5 instead of 3) and nodePort service type. Show me what helm template outputs with and without this values file."

Compare the two outputs. What changed? Why would this difference matter in production?

### Part 3: Create a Test Pod

Ask AI to create a test pod that:
- Uses curl to verify the Service is accessible
- Returns exit code 0 if the service responds, exit code 1 if it doesn't
- Includes the `helm.sh/hook: test` annotation

Deploy the chart with `helm install` and run `helm test` to verify your test pod works.

### Part 4: Write a Unit Test

Ask AI to create a helm-unittest test file that:
- Verifies the Deployment has the correct number of replicas from values
- Verifies the image tag is correctly substituted
- Tests conditional logic (e.g., "if serviceEnabled is true, Service should exist")

Run `helm unittest` to verify your tests pass.

### Part 5: Reflection

Compare what each testing approach revealed:
- What did `helm lint` catch that you might have missed manually?
- What did `helm template` show that helped you understand the manifest?
- What did unit testing verify about template logic?
- What would integration testing verify that unit tests cannot?

