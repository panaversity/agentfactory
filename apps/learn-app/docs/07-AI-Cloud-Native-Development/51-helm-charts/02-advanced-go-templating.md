---
sidebar_position: 2
chapter: 51
lesson: 2
duration_minutes: 50
title: "Advanced Go Templating"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Master Go template syntax for dynamic Kubernetes manifests"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Use template variables to store intermediate values"
    bloom_level: "Apply"
  - id: LO2
    description: "Chain template functions with pipelines"
    bloom_level: "Apply"
  - id: LO3
    description: "Control template output with if/else conditionals"
    bloom_level: "Apply"
  - id: LO4
    description: "Iterate over lists and maps using range"
    bloom_level: "Apply"
  - id: LO5
    description: "Debug template rendering with helm template --debug"
    bloom_level: "Apply"
---

# Advanced Go Templating

In Chapter 50, you learned basic Helm syntax: `{{ .Values.replicaCount }}` inserts a value into your manifest. This is enough to deploy, but not enough to build production charts. You'll encounter situations where you need to store values, transform text, make decisions about what to render, and repeat blocks. That's where Go templating becomes essential.

Go templating is the foundation of every Helm chart. Master this, and you'll write charts that adapt to any environment, validate inputs, and eliminate boilerplate across dozens of Kubernetes manifests.

---

## What You'll Learn

This lesson covers 9 core templating concepts in 3 groups:

| Group | Concepts | What You'll Do |
|-------|----------|----------------|
| **Data Handling** | Variables, Pipelines, `with` blocks | Store values, transform text, simplify access |
| **Logic** | Operators, Conditionals, Iteration | Make decisions, repeat blocks |
| **Context** | Built-in variables, Whitespace, Named templates | Access release info, control formatting |

**Prerequisites**: You should be comfortable with:
- Basic Helm syntax (`{{ .Values.x }}`) from Chapter 50 Lesson 20
- YAML structure and indentation
- Running `helm template` and `helm install`

**Time estimate**: 45-60 minutes (work through concepts, then try exercises)

---

## The Problem: Dynamic vs Static Manifests

Consider a Deployment that needs environment variables for different use cases:

```yaml
# Static: hardcoded for one environment
spec:
  containers:
  - name: agent
    env:
    - name: LOG_LEVEL
      value: "INFO"
    - name: MODEL_NAME
      value: "gpt-4"
    - name: API_KEY
      valueFrom:
        secretKeyRef:
          name: agent-secrets
          key: api_key
```

In development, you might want just the first two variables. In production, you need both plus additional security constraints. In staging, you need different log levels.

Static manifests force you to maintain separate files. Go templating lets you write once:

```yaml
spec:
  containers:
  - name: agent
    env:
    - name: LOG_LEVEL
      value: {{ .Values.logLevel | quote }}
    - name: MODEL_NAME
      value: {{ .Values.modelName | quote }}
    {{- if .Values.apiKey }}
    - name: API_KEY
      valueFrom:
        secretKeyRef:
          name: agent-secrets
          key: api_key
    {{- end }}
```

Then specify different values files for each environment. This lesson teaches the template syntax that makes this possible.

---

## Concept 1: Template Variables with Assignment

Template variables store intermediate values so you don't repeat expressions. The syntax is `$varName := expression`.

### Basic Variable Assignment

```yaml
{{ $env := .Values.environment }}
spec:
  namespace: {{ $env }}-agents
```

**Output:**
```yaml
spec:
  namespace: dev-agents
```
(when `values.yaml` contains `environment: dev`)

### Using Variables in Conditionals

```yaml
{{ $replicaCount := .Values.replicaCount }}
{{- if ge $replicaCount 3 }}
spec:
  replicas: {{ $replicaCount }}
  affinity:
    podAntiAffinity:
      requiredDuringSchedulingIgnoredDuringExecution:
{{- else }}
spec:
  replicas: {{ $replicaCount }}
{{- end }}
```

**Output:**
```yaml
spec:
  replicas: 5
  affinity:
    podAntiAffinity:
      requiredDuringSchedulingIgnoredDuringExecution:
```
(when `replicaCount: 5`)

Why is this useful? Without variables, you'd repeat `.Values.replicaCount` multiple times. With variables, you assign once and reference `$replicaCount` throughout.

---

## Concept 2: Pipelines and Function Chaining

Pipelines use the `|` operator to pass output from one function to the next. Like shell pipes, they transform values step by step.

### Simple Pipeline

```yaml
name: {{ .Values.serviceName | upper }}
```

**Output:**
```yaml
name: MY-AGENT
```
(when `serviceName: task-api`)

The `upper` function transforms text to uppercase.

### Chained Pipeline

```yaml
image: {{ .Values.image.registry }}/{{ .Values.image.repository }}:{{ .Values.image.tag }}
```

becomes cleaner with pipelines:

```yaml
image: {{ .Values.image | printf "%s/%s:%s" .registry .repository .tag }}
```

But more commonly, you chain string functions:

```yaml
name: {{ .Values.serviceName | lower | quote }}
```

**Output:**
```yaml
name: "task-api"
```

Step by step:
1. `lower`: transforms to lowercase
2. `quote`: wraps in quotes

### Common Template Functions

```yaml
{{ .Values.env | upper }}              # Output: PRODUCTION
{{ .Values.name | quote }}            # Output: "agent-name"
{{ .Values.namespace | default "default" }}  # Output: default (if not set)
{{ .Values.timeout | required "timeout required" }}  # Fails if unset
{{ .Values.description | trim }}      # Removes whitespace
```

**Output:**
```yaml
PRODUCTION
"agent-name"
default
[error if timeout not provided]
```

### More Pipeline Examples

```yaml
# Indent YAML content (for nested structures)
spec:
  config: |
{{ .Values.configContent | indent 4 }}
```

The `indent 4` function adds 4 spaces to every line of output.

**Output:**
```yaml
spec:
  config: |
    key1: value1
    key2: value2
```

```yaml
# Convert to YAML (for complex objects)
spec:
  values: {{ .Values.complexObject | toYaml | indent 2 }}
```

**Output:**
```yaml
spec:
  values:
    field1: value1
    nested:
      field2: value2
```

---

## Concept 3: The `with` Block (Context Switching)

The `with` block changes the current context (`.`) to a nested value, reducing repetition.

### Without `with`

```yaml
spec:
  containers:
  - name: {{ .Values.container.name }}
    image: {{ .Values.container.image }}
    resources:
      limits:
        memory: {{ .Values.container.memory }}
        cpu: {{ .Values.container.cpu }}
```

### With `with` Block

```yaml
spec:
  containers:
  - {{- with .Values.container }}
    name: {{ .name }}
    image: {{ .image }}
    resources:
      limits:
        memory: {{ .memory }}
        cpu: {{ .cpu }}
    {{- end }}
```

**Output:**
```yaml
spec:
  containers:
  - name: agent
    image: myregistry/agent:v1.0.0
    resources:
      limits:
        memory: 1Gi
        cpu: 500m
```

Inside the `with` block, `.name` refers to `.Values.container.name`, and `.image` refers to `.Values.container.image`. This is cleaner when you have many nested properties.

### `with` and Conditionals

```yaml
{{- with .Values.ingress }}
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: {{ .name }}
spec:
  rules:
  - host: {{ .host }}
{{- end }}
```

This renders the entire Ingress block ONLY if `.Values.ingress` is defined.

---

### Checkpoint: Data Handling Complete

You've now learned the three core data handling patterns:

| Pattern | Syntax | Use When |
|---------|--------|----------|
| **Variables** | `$var := .Values.x` | You need to reuse a value multiple times |
| **Pipelines** | `value | func1 | func2` | You need to transform values (quote, uppercase, etc.) |
| **`with` blocks** | `{{- with .Values.x }}...{{- end }}` | You're accessing many properties from the same nested object |

**Quick self-check**: Can you...
- Store a value in a variable and use it in two places?
- Chain `upper` and `quote` together in a pipeline?
- Rewrite `.Values.container.name`, `.Values.container.image` using `with`?

If yes, continue to the Logic concepts. If not, re-read the examples above—these patterns appear in every production chart.

---

## Concept 4: Comparison Operators for Conditionals

Go templates use named comparison operators instead of symbols.

### Basic Operators

```yaml
{{- if eq .Values.environment "production" }}
# True if environment equals "production"
{{- end }}

{{- if ne .Values.logLevel "DEBUG" }}
# True if logLevel does NOT equal "DEBUG"
{{- end }}

{{- if gt .Values.replicaCount 3 }}
# True if replicaCount is greater than 3
{{- end }}

{{- if lt .Values.timeout 5 }}
# True if timeout is less than 5
{{- end }}

{{- if le .Values.maxConnections 100 }}
# True if maxConnections is less than or equal to 100
{{- end }}

{{- if ge .Values.minNodes 2 }}
# True if minNodes is greater than or equal to 2
{{- end }}
```

**Cheat sheet:**
- `eq`: equals
- `ne`: not equals
- `lt`: less than
- `le`: less than or equal
- `gt`: greater than
- `ge`: greater than or equal

### Logical Operators

```yaml
{{- if and (eq .Values.env "prod") (gt .Values.replicas 2) }}
# True if BOTH conditions are true
{{- end }}

{{- if or (eq .Values.env "prod") (eq .Values.env "staging") }}
# True if EITHER condition is true
{{- end }}

{{- if not (eq .Values.debug true) }}
# True if the condition is false
{{- end }}
```

**Output:**
```yaml
# Renders only if both conditions are met
spec:
  affinity:
    podAntiAffinity: required
```

---

## Concept 5: Conditional Blocks (`if`/`else if`/`else`)

Control what renders based on conditions.

### Simple `if`

```yaml
spec:
  {{- if .Values.persistence.enabled }}
  volumeMounts:
  - name: data
    mountPath: /data
  volumes:
  - name: data
    persistentVolumeClaim:
      claimName: {{ .Values.persistence.name }}
  {{- end }}
```

**Output:**
```yaml
spec:
  volumeMounts:
  - name: data
    mountPath: /data
  volumes:
  - name: data
    persistentVolumeClaim:
      claimName: agent-pvc
```
(when `persistence.enabled: true`)

Or nothing if `persistence.enabled: false` or not set.

### `if`/`else`

```yaml
spec:
  containers:
  - name: agent
    image: {{ .Values.image }}
    {{- if .Values.imagePullPolicy }}
    imagePullPolicy: {{ .Values.imagePullPolicy }}
    {{- else }}
    imagePullPolicy: IfNotPresent
    {{- end }}
```

**Output:**
```yaml
spec:
  containers:
  - name: agent
    image: myregistry/agent:v1.0.0
    imagePullPolicy: Always
```
(if `imagePullPolicy` is set, use it; otherwise use `IfNotPresent`)

### `if`/`else if`/`else`

```yaml
spec:
  containers:
  - name: agent
    resources:
      {{- if eq .Values.tier "production" }}
      limits:
        memory: 2Gi
        cpu: 1000m
      requests:
        memory: 1Gi
        cpu: 500m
      {{- else if eq .Values.tier "staging" }}
      limits:
        memory: 1Gi
        cpu: 500m
      requests:
        memory: 512Mi
        cpu: 250m
      {{- else }}
      limits:
        memory: 512Mi
        cpu: 250m
      requests:
        memory: 256Mi
        cpu: 100m
      {{- end }}
```

**Output (for tier: staging):**
```yaml
spec:
  containers:
  - name: agent
    resources:
      limits:
        memory: 1Gi
        cpu: 500m
      requests:
        memory: 512Mi
        cpu: 250m
```

---

## Concept 6: Iteration with `range`

The `range` function iterates over lists or maps.

### Iterating Over a List

```yaml
# values.yaml
ports:
  - name: http
    port: 8000
  - name: grpc
    port: 50051
```

```yaml
# template
spec:
  ports:
  {{- range .Values.ports }}
  - containerPort: {{ .port }}
    name: {{ .name }}
  {{- end }}
```

**Output:**
```yaml
spec:
  ports:
  - containerPort: 8000
    name: http
  - containerPort: 50051
    name: grpc
```

Inside the `range` block, `.` refers to each item (changing on each iteration).

### Iterating Over a Map

```yaml
# values.yaml
env:
  LOG_LEVEL: INFO
  MODEL_NAME: gpt-4
  ENVIRONMENT: production
```

```yaml
# template
spec:
  env:
  {{- range $key, $value := .Values.env }}
  - name: {{ $key }}
    value: {{ $value | quote }}
  {{- end }}
```

**Output:**
```yaml
spec:
  env:
  - name: LOG_LEVEL
    value: "INFO"
  - name: MODEL_NAME
    value: "gpt-4"
  - name: ENVIRONMENT
    value: "production"
```

With maps, use `$key, $value` to access both the key and value.

### `range` with Index

```yaml
{{- range $index, $port := .Values.ports }}
port-{{ $index }}: {{ $port }}
{{- end }}
```

**Output:**
```yaml
port-0: 8000
port-1: 50051
```

---

### Checkpoint: Logic Complete

You've now learned how to control what gets rendered:

| Pattern | Syntax | Use When |
|---------|--------|----------|
| **Conditionals** | `{{- if condition }}...{{- end }}` | Include/exclude entire blocks based on values |
| **Comparison** | `eq`, `ne`, `gt`, `lt`, `and`, `or` | Compare values in conditionals |
| **Iteration** | `{{- range .Values.list }}...{{- end }}` | Generate repeated blocks from arrays or maps |

**Common mistake**: Forgetting that Go uses `eq` not `==`. If your conditional isn't working, check your operator syntax.

**Quick self-check**: Can you...
- Conditionally render an Ingress only when `ingress.enabled` is true?
- Combine two conditions with `and`?
- Loop over a list of ports to generate container port entries?

The final three concepts (7-9) are about context and formatting—less critical for basic charts but essential for production.

---

## Concept 7: Built-in Variables

Helm provides special variables you can use inside templates.

### Release Information

```yaml
metadata:
  name: {{ .Release.Name }}
  namespace: {{ .Release.Namespace }}
```

`{{ .Release.Name }}` is the name you provide when installing: `helm install my-release ./my-chart`

Output would be: `metadata: name: my-release`

### Chart Information

```yaml
metadata:
  labels:
    chart: {{ .Chart.Name }}-{{ .Chart.Version }}
    app.kubernetes.io/managed-by: {{ .Release.Service }}
```

`{{ .Chart.Name }}` is from `Chart.yaml` (e.g., "task-api")
`{{ .Chart.Version }}` is from `Chart.yaml` (e.g., "1.0.0")
`{{ .Release.Service }}` is always "Helm" for Helm-deployed resources

**Output:**
```yaml
metadata:
  labels:
    chart: task-api-1.0.0
    app.kubernetes.io/managed-by: Helm
```

### Capabilities

```yaml
{{- if .Capabilities.APIVersions.Has "networking.k8s.io/v1" }}
# Use networking.k8s.io/v1 (Kubernetes 1.19+)
apiVersion: networking.k8s.io/v1
{{- else }}
# Fallback to older API version
apiVersion: networking.k8s.io/v1beta1
{{- end }}
```

`.Capabilities` lets you check what Kubernetes API versions are available on the target cluster.

---

## Concept 8: Whitespace Control

Template rendering often produces unwanted blank lines. The `-` character in `{{-` and `-}}` strips whitespace.

### Without Whitespace Control

```yaml
{{- if .Values.persistence.enabled }}
spec:
  volumes:
  - name: data
    persistentVolumeClaim:
      claimName: {{ .Values.persistence.name }}
  {{- end }}
```

The `{{-` removes spaces/newlines BEFORE the block.
The `-}}` removes spaces/newlines AFTER the block.

**Output (correct indentation):**
```yaml
spec:
  volumes:
  - name: data
    persistentVolumeClaim:
      claimName: agent-pvc
```

Without `{{-`, you'd get extra blank lines that break YAML indentation.

---

## Concept 9: Named Templates and Helpers

Named templates (also called "partials") let you define reusable template code. You'll explore this deeply in Lesson 2, but the basic syntax is:

```yaml
{{- define "task-api.labels" }}
labels:
  app: {{ .Chart.Name }}
  release: {{ .Release.Name }}
{{- end }}
```

Then include it elsewhere:

```yaml
metadata:
  {{- include "task-api.labels" . }}
```

**Output:**
```yaml
metadata:
  labels:
    app: task-api
    release: my-release
```

This eliminates copying labels across multiple manifests.

---

## Common Mistakes

Before trying the exercises, know these frequent errors:

| Mistake | Wrong | Correct |
|---------|-------|---------|
| Using `==` instead of `eq` | `{{- if .Values.env == "prod" }}` | `{{- if eq .Values.env "prod" }}` |
| Forgetting to quote strings | `value: {{ .Values.name }}` | `value: {{ .Values.name \| quote }}` |
| Missing whitespace control | `{{ if ... }}` (adds blank lines) | `{{- if ... }}` (clean output) |
| Wrong scope in `with` | `{{ .Release.Name }}` inside `with` | `{{ $.Release.Name }}` (use `$` to access root) |
| Forgetting `end` | `{{- if .Values.x }}...` | `{{- if .Values.x }}...{{- end }}` |

**Debugging tip**: When templates fail, run `helm template --debug ./my-chart` to see the exact error location and line number.

---

## Exercises

### Exercise 1.1: Dynamic Replica Count

Create a template that renders replica count based on environment tier:

```yaml
# values.yaml
tier: production
replicaCount:
  dev: 1
  staging: 2
  production: 3
```

Write a template block that sets `spec.replicas` based on the tier. Hint: Use a variable assignment and conditional.

**Solution:**
```yaml
spec:
  {{- $tier := .Values.tier }}
  {{- if eq $tier "production" }}
  replicas: {{ index .Values.replicaCount "production" }}
  {{- else if eq $tier "staging" }}
  replicas: {{ index .Values.replicaCount "staging" }}
  {{- else }}
  replicas: {{ index .Values.replicaCount "dev" }}
  {{- end }}
```

**Output (for tier: production):**
```yaml
spec:
  replicas: 3
```

### Exercise 1.2: Pipeline with Filters

Given environment variable values, transform and render them:

```yaml
# values.yaml
serviceName: task-api
environment: development
timeout: 30
```

Write a template that:
- Converts `serviceName` to uppercase and quotes it
- Converts `environment` to uppercase
- Adds a "seconds" suffix to `timeout`

**Solution:**
```yaml
env:
  SERVICE_NAME: {{ .Values.serviceName | upper | quote }}
  ENVIRONMENT: {{ .Values.environment | upper }}
  TIMEOUT_SECONDS: {{ .Values.timeout }}s
```

**Output:**
```yaml
env:
  SERVICE_NAME: "MY-AGENT"
  ENVIRONMENT: DEVELOPMENT
  TIMEOUT_SECONDS: 30s
```

### Exercise 1.3: Conditional Ingress Rendering

Render an Ingress ONLY if enabled, using a `with` block:

```yaml
# values.yaml
ingress:
  enabled: true
  host: agent.example.com
  tls: true
```

Write a template that renders the Ingress block conditionally.

**Solution:**
```yaml
{{- with .Values.ingress }}
{{- if .enabled }}
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: {{ .host | replace "." "-" }}
spec:
  rules:
  - host: {{ .host }}
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: agent-service
            port:
              number: 8000
  {{- if .tls }}
  tls:
  - hosts:
    - {{ .host }}
    secretName: {{ .host }}-tls
  {{- end }}
{{- end }}
{{- end }}
```

**Output (when enabled: true and tls: true):**
```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: agent-example-com
spec:
  rules:
  - host: agent.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: agent-service
            port:
              number: 8000
  tls:
  - hosts:
    - agent.example.com
    secretName: agent-example-com-tls
```

(Rendered nothing if `enabled: false`)

### Exercise 1.4: Iterating Over Environment Variables

Transform a map of environment variables into Kubernetes env blocks:

```yaml
# values.yaml
environmentVariables:
  LOG_LEVEL: INFO
  MODEL_NAME: gpt-4
  API_BASE: https://api.openai.com
  DEBUG: "false"
```

Write a `range` loop that creates env blocks with quoted values.

**Solution:**
```yaml
spec:
  containers:
  - name: agent
    env:
    {{- range $key, $value := .Values.environmentVariables }}
    - name: {{ $key }}
      value: {{ $value | quote }}
    {{- end }}
```

**Output:**
```yaml
spec:
  containers:
  - name: agent
    env:
    - name: LOG_LEVEL
      value: "INFO"
    - name: MODEL_NAME
      value: "gpt-4"
    - name: API_BASE
      value: "https://api.openai.com"
    - name: DEBUG
      value: "false"
```

### Exercise 1.5: Debugging a Template Error

You have a template with an error:

```yaml
spec:
  containers:
  - name: {{ .Values.name }}
    image: {{ .Values.image.repository | default }}:{{ .Values.image.tag }}
```

The `default` function is missing a value. Run `helm template` to debug.

**Command:**
```bash
helm template my-release ./my-chart
```

**Error Output:**
```
error: render error in "my-chart/templates/deployment.yaml":
template: my-chart/templates/deployment.yaml:5:29:
executing "my-chart/templates/deployment.yaml" at <.Values.image.repository | default>:
wrong number of args for default: want 2 got 1
```

**Fix:**
```yaml
image: {{ .Values.image.repository | default "nginx" }}:{{ .Values.image.tag }}
```

Use `helm template --debug` to see the full template rendering process.

---

## Try With AI

In this section, you'll use AI to explore template patterns you haven't seen before, evaluate its suggestions, refine them based on production constraints, and validate the results.

### Part 1: Initial Request

You're building a chart for a data pipeline that needs conditional authentication. Ask AI:

"I need a Helm template that includes a Secret for database credentials, but only when `.Values.database.enabled` is true. The Secret should also only be created when `.Values.auth.type` equals 'database'. Show me how to handle both conditions with a template."

### Part 2: Critical Evaluation

Review AI's response. Ask yourself:

- Does it use proper conditionals (`if` blocks)?
- Does it check BOTH conditions correctly (using `and` or nested `if`)?
- Is the YAML indentation correct inside the conditional block?
- Does it use `quote` to safely escape string values in the Secret data?

### Part 3: Constraint Teaching

If AI's response doesn't handle whitespace control (the `{{-` syntax), tell it:

"The template produces extra blank lines that break the YAML structure. Use `{{-` to strip leading whitespace from template tags. Also, make sure the Secret data values are base64-encoded (Kubernetes requirement)."

### Part 4: Refinement

Ask AI to validate its updated template:

"Now show me the rendered output when `.Values.database.enabled` is true and `.Values.auth.type` is 'database'. Also show what happens when one of those conditions is false."

### Part 5: Final Check

Compare the rendered outputs to expected Kubernetes manifests:

- Does the Secret appear ONLY when both conditions are met?
- Are the data values properly quoted?
- Does the YAML indentation match Kubernetes standards?
- Would `kubectl apply -f` accept this output without errors?

If all answers are yes, you've successfully learned to combine conditionals with AI's help.

---

**Next lesson:** Lesson 2 teaches named templates and helpers (like `_helpers.tpl`), which let you build reusable template code across an entire chart.
