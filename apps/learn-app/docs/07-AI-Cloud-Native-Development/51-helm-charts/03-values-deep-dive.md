---
sidebar_position: 3
chapter: 51
lesson: 3
duration_minutes: 45
title: "Values Deep Dive"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Master values hierarchy, schema validation, and multi-environment configuration"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain the values override precedence hierarchy"
    bloom_level: "Understand"
  - id: LO2
    description: "Create environment-specific values files for dev, staging, and production"
    bloom_level: "Create"
  - id: LO3
    description: "Use command-line --set overrides effectively"
    bloom_level: "Apply"
  - id: LO4
    description: "Create values.schema.json for input validation"
    bloom_level: "Create"
  - id: LO5
    description: "Design values.yaml structure for maintainability"
    bloom_level: "Analyze"
---

# Values Deep Dive

In Lesson 20 of Chapter 50, you learned that values.yaml contains default configuration and environment-specific files override those defaults. That was enough to deploy basic agents across environments.

But production deployments require more: clarity about which override takes precedence when multiple are specified, validation that required fields are present before rendering, and organizational patterns that keep complex values readable as your chart grows. This lesson teaches the complete values architecture that production charts depend on.

---

## The Values Override Precedence Hierarchy

When you deploy a Helm chart, values come from multiple sources. Understanding the order matters—if you override something and it doesn't work, you're probably at the wrong level of the hierarchy.

### The Hierarchy (Lowest to Highest Priority)

Here's what happens in order, with later sources overriding earlier ones:

1. **Chart defaults** (`values.yaml`)
2. **Parent chart values** (if this is a subchart)
3. **Values from `-f file.yaml` flags** (environment-specific files)
4. **Command-line `--set` overrides** (highest priority)

### Understanding Each Level

**Level 1: Chart Defaults (values.yaml)**

This is the baseline. Every configuration option your chart supports should have a sensible default here:

```yaml
# my-agent-chart/values.yaml
replicaCount: 3
image:
  repository: myregistry/my-agent
  tag: v1.0.0
  pullPolicy: IfNotPresent
service:
  type: ClusterIP
  port: 8000
resources:
  requests:
    memory: "512Mi"
    cpu: "250m"
  limits:
    memory: "1Gi"
    cpu: "500m"
agent:
  modelName: gpt-4
  logLevel: INFO
```

**Output:** This values.yaml is used when you run `helm install my-agent ./my-agent-chart` with no overrides.

When Helm renders templates, it replaces `{{ .Values.replicaCount }}` with `3`, `{{ .Values.image.tag }}` with `v1.0.0`, etc.

---

**Level 2: Environment-Specific Files (-f values-prod.yaml)**

When deploying to production, you don't modify values.yaml (it's version-controlled and shared). Instead, you create environment-specific files that override only the values that differ:

```yaml
# my-agent-chart/values-prod.yaml
replicaCount: 5
image:
  tag: v1.0.0
service:
  type: LoadBalancer
resources:
  limits:
    memory: "2Gi"
    cpu: "1000m"
agent:
  logLevel: WARN
```

**Output:** When you run:

```bash
helm install my-agent ./my-agent-chart -f values-prod.yaml
```

Helm merges these two YAML files. The result:

```yaml
# Effective values after merge
replicaCount: 5              # From values-prod.yaml
image:
  repository: myregistry/my-agent  # From values.yaml (kept)
  tag: v1.0.0               # From values-prod.yaml
  pullPolicy: IfNotPresent  # From values.yaml (kept)
service:
  type: LoadBalancer        # From values-prod.yaml
  port: 8000                # From values.yaml (kept)
resources:
  requests:
    memory: "512Mi"         # From values.yaml (kept)
    cpu: "250m"             # From values.yaml (kept)
  limits:
    memory: "2Gi"           # From values-prod.yaml
    cpu: "1000m"            # From values-prod.yaml
agent:
  modelName: gpt-4          # From values.yaml (kept)
  logLevel: WARN            # From values-prod.yaml
```

Key principle: Only specify values you're changing in environment files. Keep defaults in values.yaml.

---

**Level 3: Command-Line --set Overrides**

For one-off changes (emergency fixes, testing), you can override values directly via CLI:

```bash
helm install my-agent ./my-agent-chart \
  -f values-prod.yaml \
  --set image.tag=v1.0.1 \
  --set agent.logLevel=DEBUG
```

This runs at the highest priority. The effective values would have:

- `image.tag: v1.0.1` (from `--set`, overrides both values.yaml and values-prod.yaml)
- `agent.logLevel: DEBUG` (from `--set`, overrides values-prod.yaml's WARN)
- Everything else merged as normal

**Output:** Same structure as before, but with CLI overrides applied last.

---

### Practical: Demonstrating the Precedence Hierarchy

Let's verify the precedence with a real example:

```bash
# Step 1: Create three values files
mkdir -p helm-precedence && cd helm-precedence

# values.yaml (defaults)
cat > values.yaml << 'EOF'
replicas: 1
image: my-agent:v1.0.0
environment: development
EOF

# values-prod.yaml (environment override)
cat > values-prod.yaml << 'EOF'
replicas: 5
environment: production
EOF

# Step 2: Create a simple template
mkdir -p templates
cat > templates/config.yaml << 'EOF'
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
data:
  replicas: {{ .Values.replicas | quote }}
  image: {{ .Values.image | quote }}
  environment: {{ .Values.environment | quote }}
EOF

# Step 3: Create a Chart.yaml
cat > Chart.yaml << 'EOF'
apiVersion: v2
name: precedence-test
version: 0.1.0
EOF

# Step 4: Test the hierarchy with helm template
echo "=== Test 1: Default values only ==="
helm template test .

echo ""
echo "=== Test 2: With -f values-prod.yaml ==="
helm template test . -f values-prod.yaml

echo ""
echo "=== Test 3: With -f + --set ==="
helm template test . -f values-prod.yaml --set replicas=10
```

**Output (Test 1: Defaults):**

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
data:
  replicas: "1"
  image: "my-agent:v1.0.0"
  environment: "development"
```

**Output (Test 2: With -f values-prod.yaml):**

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
data:
  replicas: "5"
  image: "my-agent:v1.0.0"         # Unchanged from values.yaml
  environment: "production"        # Overridden in values-prod.yaml
```

**Output (Test 3: With -f + --set replicas=10):**

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
data:
  replicas: "10"                    # Overridden by --set
  image: "my-agent:v1.0.0"         # Unchanged
  environment: "production"        # From values-prod.yaml
```

The pattern is clear: `--set` beats `-f`, which beats values.yaml defaults.

---

## Designing values.yaml Structure: Flat vs Nested

As your chart grows, values.yaml can become unwieldy. Should you organize values hierarchically or keep them flat? The answer depends on cognitive load and reusability.

### The Flat Approach (Simple, Limited)

```yaml
# values.yaml - flat structure
agentName: my-agent
agentReplicas: 3
agentImage: myregistry/my-agent
agentImageTag: v1.0.0
agentLogLevel: INFO
databaseHost: postgres.default.svc.cluster.local
databasePort: 5432
databaseName: agents
redisHost: redis.default.svc.cluster.local
redisPort: 6379
redisPassword: ""
```

**Advantages:**
- Easy to understand (all values visible at once)
- No nested nesting to navigate

**Disadvantages:**
- Names become verbose (`agentName`, `agentReplicas`, `agentImage`)
- Hard to group related configuration (where does agent logging belong?)
- CLI overrides become tedious: `--set agentLogLevel=DEBUG`

---

### The Nested Approach (Organized, Scalable)

```yaml
# values.yaml - nested structure
agent:
  name: my-agent
  replicas: 3
  image:
    repository: myregistry/my-agent
    tag: v1.0.0
    pullPolicy: IfNotPresent
  config:
    modelName: gpt-4
    logLevel: INFO
    temperature: 0.7
  resources:
    requests:
      memory: "512Mi"
      cpu: "250m"
    limits:
      memory: "1Gi"
      cpu: "500m"

database:
  host: postgres.default.svc.cluster.local
  port: 5432
  name: agents
  username: postgres
  password: ""  # Set in production via --set or secrets

redis:
  host: redis.default.svc.cluster.local
  port: 6379
  password: ""
```

**Advantages:**
- Clear organization (all agent settings grouped, all database settings grouped)
- Template access is intuitive: `{{ .Values.agent.replicas }}`
- Environment overrides are cleaner:
  ```yaml
  # values-prod.yaml
  agent:
    replicas: 5
    config:
      logLevel: WARN
  ```
- CLI overrides use dot notation: `--set agent.logLevel=DEBUG`

**Disadvantages:**
- More hierarchy to navigate
- Accessing deep nested values in templates requires more typing

---

### Best Practice: Organize Around Concepts, Not Implementation

Use nesting to group values by **what they configure**, not by **where they're used**. For an AI agent chart:

```yaml
# Good: Organized by concept
agent:
  name: my-agent
  image: ...
  config:
    model: ...
    logging: ...

database:
  connection: ...
  credentials: ...

dependencies:
  redis: ...
  cache: ...
```

Not like this (organized by location in template):

```yaml
# Bad: Organized by template file
deployment:
  replicas: ...
  image: ...
service:
  port: ...
configMap:
  modelName: ...
secret:
  databasePassword: ...
```

The first approach is maintainable. The second scatters related values across different top-level keys.

---

## Environment-Specific Files: The Three-Tier Pattern

Production deployments typically use three environments: development (for testing), staging (for pre-production), and production. Each has different configurations—resource limits, replica counts, security settings, and external dependencies.

### The Three Files

**File 1: values.yaml (Development Defaults)**

This is the baseline—what most developers use locally. Minimal resources, one replica, verbose logging:

```yaml
# my-agent-chart/values.yaml
agent:
  name: my-agent
  replicas: 1
  image:
    tag: v1.0.0-dev
  config:
    logLevel: DEBUG
    modelName: gpt-3.5-turbo
  resources:
    requests:
      memory: "256Mi"
      cpu: "100m"
    limits:
      memory: "512Mi"
      cpu: "250m"

database:
  host: postgres.default.svc.cluster.local
  name: agents_dev
  username: postgres
  password: devpassword

redis:
  enabled: false
```

**Output:** When you run `helm install my-agent ./my-agent-chart`, you get:
- 1 replica
- Light resource requests
- Verbose logging (DEBUG)
- Development database

---

**File 2: values-staging.yaml**

Staging mirrors production more closely—multiple replicas, tighter resources, and real external services:

```yaml
# my-agent-chart/values-staging.yaml
agent:
  replicas: 2
  image:
    tag: v1.0.0
  config:
    logLevel: INFO
    modelName: gpt-4
  resources:
    requests:
      memory: "512Mi"
      cpu: "250m"
    limits:
      memory: "1Gi"
      cpu: "500m"

database:
  host: postgres-staging.production.svc.cluster.local
  name: agents_staging
  # password should come from Secret, not values file

redis:
  enabled: true
  host: redis-staging.production.svc.cluster.local
  password: ""  # Set via Secret
```

**Output:** When you run `helm install my-agent ./my-agent-chart -f values-staging.yaml`, you get:
- 2 replicas
- Realistic resource limits
- Production-like logging (INFO)
- Staging external services (separate DB, Redis enabled)

---

**File 3: values-prod.yaml**

Production maximizes availability and performance:

```yaml
# my-agent-chart/values-prod.yaml
agent:
  replicas: 5
  image:
    tag: v1.0.0
  config:
    logLevel: WARN
    modelName: gpt-4-turbo
  resources:
    requests:
      memory: "1Gi"
      cpu: "500m"
    limits:
      memory: "2Gi"
      cpu: "1000m"

database:
  host: postgres-prod.production.svc.cluster.local
  name: agents
  username: postgres
  # password must come from Secret in production

redis:
  enabled: true
  host: redis-prod.production.svc.cluster.local
  # password must come from Secret

# Service type changes for prod (LoadBalancer for external access)
service:
  type: LoadBalancer
```

**Output:** When you run `helm install my-agent ./my-agent-chart -f values-prod.yaml`, you get:
- 5 replicas for high availability
- Production-grade resource limits
- Minimal logging (WARN)
- Production external services
- External LoadBalancer access

---

### Using Environment Files in Practice

```bash
# Deploy to dev (uses values.yaml defaults)
helm install my-agent ./my-agent-chart

# Deploy to staging
helm install my-agent ./my-agent-chart -f values-staging.yaml

# Deploy to production
helm install my-agent ./my-agent-chart -f values-prod.yaml

# Upgrade staging to new version
helm upgrade my-agent ./my-agent-chart \
  -f values-staging.yaml \
  --set agent.image.tag=v1.0.1
```

Each command uses the same chart with different configurations. The chart stays in source control, environment files stay in source control, and Helm handles the merging.

---

## Command-Line --set Overrides

While environment files are preferred (they're version-controlled and documented), `--set` overrides are useful for debugging, quick tests, and emergency patches. Understanding the syntax is essential.

### Basic --set Syntax

```bash
helm install my-agent ./my-agent-chart \
  --set agent.name=custom-agent \
  --set agent.replicas=10 \
  --set agent.config.logLevel=DEBUG
```

Each `--set` takes the path to the value (using dot notation) and the new value. The chart is installed with those overrides applied.

**Output:** The effective values have:
- `agent.name: custom-agent`
- `agent.replicas: 10`
- `agent.config.logLevel: DEBUG`
- All other values from values.yaml unchanged

---

### --set-string (for String Values)

By default, `--set` tries to infer the type (number vs string). For values that look like numbers but should be strings, use `--set-string`:

```bash
# Wrong: This might be interpreted as a number
--set agent.apiKey=12345

# Right: Force it as a string
--set-string agent.apiKey=12345
```

**Why it matters:** If your template stores `{{ .Values.agent.apiKey }}` in an environment variable, it needs to be a string, not a number.

---

### --set-file (for Loading from Files)

When you have multi-line values (TLS certificates, JSON configs), loading from a file is cleaner:

```bash
# Create a config file
cat > agent-config.json << 'EOF'
{
  "model": "gpt-4",
  "temperature": 0.7,
  "max_tokens": 2000
}
EOF

# Use --set-file to load it
helm install my-agent ./my-agent-chart \
  --set-file agent.customConfig=agent-config.json
```

The file's contents become the value of `agent.customConfig`. In your template, you can access it as:

```yaml
- name: AGENT_CONFIG
  value: |
    {{ .Values.agent.customConfig }}
```

**Output:** The environment variable contains the entire JSON config from the file.

---

## Values Schema Validation: values.schema.json

As your chart grows, it's easy to make mistakes: typos in value names, wrong types (string vs number), missing required fields. The chart installs but doesn't work as expected, leading to debugging nightmares.

**values.schema.json** is a JSON Schema file that validates values before rendering. Helm refuses to render if required fields are missing or types don't match.

### Creating a Basic Schema

```json
{
  "$schema": "https://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": [
    "agent"
  ],
  "properties": {
    "agent": {
      "type": "object",
      "required": [
        "name",
        "replicas"
      ],
      "properties": {
        "name": {
          "type": "string",
          "description": "Name of the agent"
        },
        "replicas": {
          "type": "integer",
          "minimum": 1,
          "maximum": 100,
          "description": "Number of agent replicas"
        },
        "image": {
          "type": "object",
          "properties": {
            "repository": {
              "type": "string"
            },
            "tag": {
              "type": "string"
            },
            "pullPolicy": {
              "type": "string",
              "enum": [
                "Always",
                "IfNotPresent",
                "Never"
              ]
            }
          }
        },
        "config": {
          "type": "object",
          "properties": {
            "logLevel": {
              "type": "string",
              "enum": [
                "DEBUG",
                "INFO",
                "WARN",
                "ERROR"
              ],
              "description": "Agent logging level"
            },
            "modelName": {
              "type": "string",
              "description": "LLM model name (gpt-4, gpt-3.5-turbo, etc.)"
            }
          }
        },
        "resources": {
          "type": "object",
          "properties": {
            "requests": {
              "type": "object",
              "properties": {
                "memory": {
                  "type": "string",
                  "pattern": "^[0-9]+(Mi|Gi|Ki)$",
                  "description": "Memory request (e.g., 512Mi, 1Gi)"
                },
                "cpu": {
                  "type": "string",
                  "pattern": "^[0-9]+(m)?$",
                  "description": "CPU request (e.g., 250m, 1)"
                }
              }
            },
            "limits": {
              "type": "object",
              "properties": {
                "memory": {
                  "type": "string",
                  "pattern": "^[0-9]+(Mi|Gi|Ki)$"
                },
                "cpu": {
                  "type": "string",
                  "pattern": "^[0-9]+(m)?$"
                }
              }
            }
          }
        }
      }
    }
  }
}
```

Place this in `my-agent-chart/values.schema.json`.

---

### Schema Validation in Action

Now when you run helm with invalid values, it catches the error:

```bash
# Test 1: Missing required field
helm template test ./my-agent-chart \
  --set agent.replicas=abc

# Output (validation error):
# Error: failed to parse --set data: json: cannot unmarshal string into Go value of type int64
```

```bash
# Test 2: Invalid enum value
helm template test ./my-agent-chart \
  --set agent.config.logLevel=VERBOSE

# Output (validation error):
# Error: values.yaml does not conform to schema: [agent.config.logLevel: Unsupported value: "VERBOSE" (Enum: "DEBUG", "INFO", "WARN", "ERROR")]
```

```bash
# Test 3: Valid values pass
helm template test ./my-agent-chart \
  --set agent.replicas=5 \
  --set agent.config.logLevel=WARN

# Output: Renders successfully
```

---

## Nested Values and Access Patterns

As your chart grows, accessing deeply nested values in templates becomes essential. Understanding the dot notation and when to use bracket notation saves time.

### Dot Notation (Most Common)

```go
{{ .Values.agent.config.logLevel }}
{{ .Values.database.host }}
{{ .Values.redis.password }}
```

This is readable and works for most cases. The template engine navigates the YAML structure using dots.

---

### Accessing Values in Loops

When iterating over values, use the correct scope:

```yaml
# values.yaml
ports:
  - containerPort: 8000
    name: http
  - containerPort: 9000
    name: metrics
```

In your template:

```yaml
containers:
- name: agent
  ports:
  {{- range .Values.ports }}
  - containerPort: {{ .containerPort }}
    name: {{ .name }}
  {{- end }}
```

**Output:**

```yaml
containers:
- name: agent
  ports:
  - containerPort: 8000
    name: http
  - containerPort: 9000
    name: metrics
```

---

### With Blocks for Scope Management

When accessing nested values repeatedly, `with` simplifies templates:

```go
{{- with .Values.agent.config }}
logLevel: {{ .logLevel }}
modelName: {{ .modelName }}
temperature: {{ .temperature }}
{{- end }}
```

Instead of:

```go
logLevel: {{ .Values.agent.config.logLevel }}
modelName: {{ .Values.agent.config.modelName }}
temperature: {{ .Values.agent.config.temperature }}
```

The `with` block sets `.` to the nested object, reducing repetition.

**Output:** Same result, but cleaner template code.

---

## Best Practices for values.yaml Design

### 1. Sensible Defaults

Every value should have a default that allows the chart to deploy and function, even if suboptimally:

```yaml
# Good: Sensible defaults
agent:
  replicas: 1  # At least 1 replica
  image:
    repository: myregistry/my-agent
    tag: latest  # Latest image available
  config:
    logLevel: INFO  # Standard logging
```

Not like this (missing defaults):

```yaml
# Bad: No defaults, requires manual --set
agent:
  replicas: null
  image:
    repository: null
    tag: null
```

---

### 2. Documentation Comments

Use YAML comments in values.yaml to explain what each value does and valid options:

```yaml
agent:
  # Number of replicas (1-100)
  # For prod: 5-10; staging: 2; dev: 1
  replicas: 1

  image:
    # Container image repository
    repository: myregistry/my-agent
    # Image tag (version)
    # Use vX.Y.Z for releases, v1.0.0-dev for testing
    tag: v1.0.0-dev
    # Image pull policy
    # IfNotPresent: Use cached image if available
    # Always: Always pull latest image
    # Never: Only use cached images
    pullPolicy: IfNotPresent

  config:
    # Logging level: DEBUG, INFO, WARN, ERROR
    logLevel: INFO
    # LLM model: gpt-4, gpt-3.5-turbo, claude-opus, etc.
    modelName: gpt-4
```

**Output:** Developers know what each value does, valid options, and environment-specific recommendations without reading the chart code.

---

### 3. Grouping Related Values

Group values by concept and function:

```yaml
# Good: Grouped by concern
agent:
  replicas: 1
  image: ...
  config: ...
  resources: ...

database:
  host: ...
  credentials: ...

dependencies:
  redis: ...
  cache: ...
```

---

### 4. No Secrets in values.yaml

Never put passwords, API keys, or tokens in values.yaml. These should come from Kubernetes Secrets or external secret management:

```yaml
# Bad: Secrets in values
database:
  password: mypassword123
  apiKey: sk-123456789
```

Instead:

```yaml
# Good: Reference Secret, actual value elsewhere
database:
  password:
    valueFrom:
      secretKeyRef:
        name: db-credentials
        key: password
  apiKey:
    valueFrom:
      secretKeyRef:
        name: api-credentials
        key: apiKey
```

Or use `--set` with Secret values at deploy time:

```bash
helm install my-agent ./my-agent-chart \
  --set-string database.password=$(kubectl get secret db-credentials -o jsonpath='{.data.password}' | base64 -d)
```

---

## Exercises

### Exercise 3.1: Create values.yaml with Realistic AI Agent Defaults

Create a values.yaml for an AI agent chart with:
- Agent configuration (name, replicas, image)
- Model settings (model name, temperature, max tokens)
- Resource limits (reasonable for a CPU-constrained environment)
- Database connection (PostgreSQL)
- Logging configuration

**Solution:**

```yaml
# my-agent-chart/values.yaml
agent:
  name: my-ai-agent
  replicas: 1
  image:
    repository: myregistry/ai-agent
    tag: v1.0.0-dev
    pullPolicy: IfNotPresent

  config:
    modelName: gpt-3.5-turbo
    temperature: 0.7
    maxTokens: 2000
    logLevel: DEBUG

  resources:
    requests:
      memory: "512Mi"
      cpu: "250m"
    limits:
      memory: "1Gi"
      cpu: "500m"

database:
  host: postgres.default.svc.cluster.local
  port: 5432
  name: agents
  username: postgres
  password: devpassword

service:
  type: ClusterIP
  port: 8000
```

---

### Exercise 3.2: Create Environment-Specific Files

Create values-staging.yaml and values-prod.yaml that override the development defaults:

**Staging Requirements:**
- 2 replicas
- Real production database (postgres-staging.production.svc)
- INFO logging
- gpt-4 model

**Production Requirements:**
- 5 replicas
- Production database (postgres-prod.production.svc)
- WARN logging
- gpt-4-turbo model
- Higher resource limits

**Solution:**

```yaml
# my-agent-chart/values-staging.yaml
agent:
  replicas: 2
  config:
    modelName: gpt-4
    logLevel: INFO
  resources:
    requests:
      memory: "512Mi"
      cpu: "250m"
    limits:
      memory: "1Gi"
      cpu: "500m"

database:
  host: postgres-staging.production.svc.cluster.local
  name: agents_staging
```

```yaml
# my-agent-chart/values-prod.yaml
agent:
  replicas: 5
  config:
    modelName: gpt-4-turbo
    logLevel: WARN
  resources:
    requests:
      memory: "1Gi"
      cpu: "500m"
    limits:
      memory: "2Gi"
      cpu: "1000m"

database:
  host: postgres-prod.production.svc.cluster.local
  name: agents

service:
  type: LoadBalancer
```

---

### Exercise 3.3: Verify Override Precedence with helm template

Create a Helm chart and verify that values override in the correct order.

**Steps:**

```bash
# 1. Create chart directory
mkdir my-agent-chart && cd my-agent-chart

# 2. Create Chart.yaml
cat > Chart.yaml << 'EOF'
apiVersion: v2
name: my-agent
version: 1.0.0
EOF

# 3. Create values.yaml (defaults)
cat > values.yaml << 'EOF'
agent:
  name: default-agent
  replicas: 1
  logLevel: DEBUG
EOF

# 4. Create values-prod.yaml
cat > values-prod.yaml << 'EOF'
agent:
  replicas: 5
  logLevel: WARN
EOF

# 5. Create templates
mkdir templates
cat > templates/config.yaml << 'EOF'
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-config
data:
  name: {{ .Values.agent.name | quote }}
  replicas: {{ .Values.agent.replicas | quote }}
  logLevel: {{ .Values.agent.logLevel | quote }}
EOF

# 6. Test: Default values only
echo "=== Default values ==="
helm template my-agent .

# 7. Test: With -f values-prod.yaml
echo ""
echo "=== With -f values-prod.yaml ==="
helm template my-agent . -f values-prod.yaml

# 8. Test: With --set override
echo ""
echo "=== With --set agent.replicas=10 ==="
helm template my-agent . -f values-prod.yaml --set agent.replicas=10
```

**Output:**

Verify that:
- Default: replicas: "1", logLevel: "DEBUG"
- With -f: replicas: "5", logLevel: "WARN", name unchanged
- With --set: replicas: "10", logLevel: "WARN"

---

### Exercise 3.4: Create values.schema.json

Create a JSON Schema that validates:
- `agent.replicas` is a number between 1-100
- `agent.config.logLevel` is one of: DEBUG, INFO, WARN, ERROR
- `agent.config.modelName` is a string (required)
- `database.host` is a string (required)

**Solution:**

```json
{
  "$schema": "https://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "agent": {
      "type": "object",
      "properties": {
        "replicas": {
          "type": "integer",
          "minimum": 1,
          "maximum": 100,
          "description": "Number of agent replicas"
        },
        "config": {
          "type": "object",
          "required": [
            "modelName"
          ],
          "properties": {
            "logLevel": {
              "type": "string",
              "enum": [
                "DEBUG",
                "INFO",
                "WARN",
                "ERROR"
              ]
            },
            "modelName": {
              "type": "string"
            }
          }
        }
      }
    },
    "database": {
      "type": "object",
      "required": [
        "host"
      ],
      "properties": {
        "host": {
          "type": "string"
        }
      }
    }
  }
}
```

Save as `my-agent-chart/values.schema.json`.

---

### Exercise 3.5: Validate Invalid Values Against Schema

Test that Helm rejects invalid values according to schema:

```bash
# Test 1: Invalid replicas (string instead of number)
helm template my-agent . --set agent.replicas=abc
# Expected: Error about type mismatch

# Test 2: Invalid logLevel (not in enum)
helm template my-agent . --set agent.config.logLevel=VERBOSE
# Expected: Error about unsupported value

# Test 3: Missing required modelName
helm template my-agent . --set agent.config.modelName=null
# Expected: Error about missing required field

# Test 4: Valid values pass
helm template my-agent . \
  --set agent.replicas=5 \
  --set agent.config.logLevel=WARN \
  --set agent.config.modelName=gpt-4
# Expected: Renders successfully
```

---

## Try With AI

Now it's time to use your understanding of values hierarchy and override precedence to refine a real deployment scenario.

### Part 1: Initial Challenge

You're deploying your AI agent to three environments, but the current values setup has issues:

1. Environment files override too much (they repeat defaults they don't change)
2. Production database password is in values-prod.yaml (security issue)
3. No schema validation (invalid log levels are accepted)
4. Developers don't know which values control what (missing documentation)

Ask AI: "Help me redesign our Helm chart values files. We're deploying an AI agent to dev, staging, and production. Create a values.yaml with sensible defaults, values-staging.yaml and values-prod.yaml that only override what differs, and values.schema.json that validates the required fields. Include comments documenting each value."

### Part 2: Evaluate AI's Response

Review the AI's suggestions:
- Does values.yaml have sensible defaults for all fields?
- Do environment files only override what differs (not repeat everything)?
- Is values.schema.json complete (required fields, valid enums)?
- Are comments clear about what each value does and valid options?

### Part 3: Refine for Your Context

Tell AI: "I missed something in my requirements. The agent needs to support custom API endpoints for different providers (OpenAI, Anthropic, Ollama). Add that to the values structure and schema. Also, the staging environment should use the same replicas as production (for realistic load testing), so only the database connection differs."

### Part 4: Validate Your Understanding

Ask AI: "Walk me through the precedence hierarchy. If I run: `helm install my-agent ./my-agent-chart -f values-prod.yaml --set agent.replicas=3`, what will the final replica count be? Why?"

AI's answer should explain that `--set` has highest priority, so replicas will be 3 (not 5 from values-prod.yaml).

### Part 5: Final Check

Compare your final values structure to your learning objectives:
- Can you explain the override precedence hierarchy?
- Can you create environment-specific files that only override necessary values?
- Can you write `--set` overrides for one-off changes?
- Can you create a values.schema.json that prevents invalid deployments?
- Can you design a values.yaml structure that's maintainable as the chart grows?

If all are "yes," you're ready for Lesson 4 (Chart Dependencies). If any are "no," ask AI to clarify that specific concept.
