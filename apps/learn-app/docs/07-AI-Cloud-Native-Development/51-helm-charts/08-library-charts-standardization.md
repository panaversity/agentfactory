---
sidebar_position: 8
chapter: 51
lesson: 8
duration_minutes: 45
title: "Library Charts and Organizational Standardization"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Create library charts that enforce organizational patterns across teams"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Create library charts with type: library in Chart.yaml"
    bloom_level: "Create"
  - id: LO2
    description: "Explain why library charts cannot be installed directly"
    bloom_level: "Understand"
  - id: LO3
    description: "Design common templates for organizational consistency"
    bloom_level: "Create"
  - id: LO4
    description: "Consume library charts as dependencies in application charts"
    bloom_level: "Apply"
  - id: LO5
    description: "Override library defaults in application charts"
    bloom_level: "Apply"
---

# Library Charts and Organizational Standardization

In Lessons 1-7, you've built individual Helm charts that deploy applications with templating, hooks, and distribution strategies. But what happens when you have 10 microservices? Do you repeat the same labels, resource probes, and annotations in each chart? What if your security team requires specific SecurityContext defaults across all deployments?

This lesson introduces library charts—a Helm pattern that solves this through reusable templates. Instead of duplicating patterns across charts, you encode them once in a library chart that all application charts consume. Your organization then has a single source of truth for labels, probes, resource defaults, and organizational standards.

By the end of this lesson, you'll understand how library charts enforce consistency across your platform without forcing teams into rigid constraints.

---

## Concept 1: Library Chart Type

Library charts declare their purpose in `Chart.yaml` with `type: library`. This tells Helm that this chart exists to be included as a dependency, not to be installed directly.

### Creating a Library Chart

Start with the basic structure:

```bash
mkdir -p org-standards/templates

# Create Chart.yaml
cat > org-standards/Chart.yaml <<'EOF'
apiVersion: v2
name: org-standards
description: "Organizational standards for labels, probes, and security"
type: library
version: 1.0.0
EOF

# Create values.yaml with organizational defaults
cat > org-standards/values.yaml <<'EOF'
organization:
  name: "my-company"
  team: "platform"
  environment: "dev"

commonLabels:
  managed-by: "helm"
  org: "my-company"

commonAnnotations:
  doc-link: "https://internal.docs/deployments"

securityDefaults:
  runAsNonRoot: true
  readOnlyRootFilesystem: true
  allowPrivilegeEscalation: false
EOF

# Create empty templates directory
mkdir org-standards/templates
```

**Output:**
```
org-standards/
├── Chart.yaml
├── values.yaml
└── templates/

$ cat org-standards/Chart.yaml
apiVersion: v2
name: org-standards
description: "Organizational standards for labels, probes, and security"
type: library
version: 1.0.0
```

The `type: library` field distinguishes this chart from application charts (which have no type field or `type: application`).

---

## Concept 2: Why Library Charts Cannot Be Installed Directly

Helm refuses to install library charts directly. They contain only template definitions, not complete deployable resources.

### Testing the Restriction

Try installing the library chart:

```bash
cd org-standards

# Attempt to install library chart
helm install my-org . --dry-run=client
```

**Output:**
```
error: chart "org-standards" is a library chart. Library charts
are not installable. See https://helm.sh/docs/topics/library_charts/
for more information.
```

This restriction is intentional. Library charts have no `.Chart.Name` release—they're meant to provide helper functions that application charts use. Installing them directly would create an empty release with no resources, providing no value.

---

## Concept 3: Common Templates for Organizational Consistency

Create reusable templates in the library chart that enforce organizational patterns.

### Creating Labels Template

```bash
cat > org-standards/templates/_labels.tpl <<'EOF'
{{- define "org-standards.labels" -}}
app.kubernetes.io/name: {{ include "org-standards.name" . }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
org: {{ .Values.organization.name }}
team: {{ .Values.organization.team }}
env: {{ .Values.organization.environment }}
{{- end }}

{{- define "org-standards.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}
EOF
```

### Creating Common Annotations Template

```bash
cat > org-standards/templates/_annotations.tpl <<'EOF'
{{- define "org-standards.annotations" -}}
documentation: {{ .Values.commonAnnotations.doc-link }}
updated-at: {{ now | date "2006-01-02T15:04:05Z07:00" }}
{{- end }}
EOF
```

### Creating Probes Template

```bash
cat > org-standards/templates/_probes.tpl <<'EOF'
{{- define "org-standards.livenessProbe" -}}
livenessProbe:
  httpGet:
    path: /health
    port: 8080
  initialDelaySeconds: 30
  periodSeconds: 10
  timeoutSeconds: 5
  failureThreshold: 3
{{- end }}

{{- define "org-standards.readinessProbe" -}}
readinessProbe:
  httpGet:
    path: /ready
    port: 8080
  initialDelaySeconds: 10
  periodSeconds: 5
  timeoutSeconds: 3
  failureThreshold: 2
{{- end }}
EOF
```

**Output:**
```
org-standards/templates/
├── _annotations.tpl
├── _labels.tpl
└── _probes.tpl

$ ls -la org-standards/templates/
total 48
-rw-r--r--  1 user  staff  285 org-standards/templates/_annotations.tpl
-rw-r--r--  1 user  staff  412 org-standards/templates/_labels.tpl
-rw-r--r--  1 user  staff  518 org-standards/templates/_probes.tpl
```

These templates define labels, annotations, and probes that any application chart can use without reimplementation.

---

## Concept 4: Consuming Library Charts as Dependencies

Application charts declare library charts as dependencies in `Chart.yaml`.

### Create Application Chart with Library Dependency

```bash
mkdir -p my-api/templates

# Create Chart.yaml with library dependency
cat > my-api/Chart.yaml <<'EOF'
apiVersion: v2
name: my-api
description: "AI agent API service"
type: application
version: 1.0.0
appVersion: "1.0"
dependencies:
  - name: org-standards
    version: "1.0.0"
    repository: "file://../org-standards"
    condition: org-standards.enabled
EOF

# Create application values.yaml
cat > my-api/values.yaml <<'EOF'
appName: my-api
replicaCount: 2
image:
  repository: my-org/my-api
  tag: "1.0.0"

# Enable the library chart
org-standards:
  enabled: true
  organization:
    name: my-company
    team: platform
    environment: prod
EOF

# Update Helm dependencies
helm dependency update my-api/
```

**Output:**
```
Hang tight while we grab the latest from your chart repositories...
...Successfully got an update from the "file://../org-standards" repo
Saving 1 charts
Downloading org-standards from repo file://../org-standards
...Deleting outdated charts

$ ls -la my-api/charts/
total 32
drwxr-xr-x  4 user  staff  128 org-standards/
```

The `helm dependency update` command fetches the library chart and places it in `my-api/charts/org-standards/`.

---

## Concept 5: Using `include` to Pull Library Templates

Application charts use `include` function to reference templates from library charts.

### Creating Application Deployment Using Library Templates

```bash
cat > my-api/templates/deployment.yaml <<'EOF'
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ .Values.appName }}
  labels:
    {{- include "org-standards.labels" . | nindent 4 }}
  annotations:
    {{- include "org-standards.annotations" . | nindent 4 }}
spec:
  replicas: {{ .Values.replicaCount }}
  selector:
    matchLabels:
      app: {{ .Values.appName }}
  template:
    metadata:
      labels:
        app: {{ .Values.appName }}
        {{- include "org-standards.labels" . | nindent 8 }}
    spec:
      containers:
      - name: api
        image: "{{ .Values.image.repository }}:{{ .Values.image.tag }}"
        ports:
        - containerPort: 8080
        {{- include "org-standards.livenessProbe" . | nindent 8 }}
        {{- include "org-standards.readinessProbe" . | nindent 8 }}
        resources:
          requests:
            cpu: 100m
            memory: 128Mi
          limits:
            cpu: 500m
            memory: 512Mi
EOF
```

**Output:**
```
$ helm template my-api .
---
# Source: my-api/templates/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-api
  labels:
    app.kubernetes.io/name: my-api
    app.kubernetes.io/version: "1.0"
    app.kubernetes.io/managed-by: Helm
    org: my-company
    team: platform
    env: prod
  annotations:
    documentation: https://internal.docs/deployments
    updated-at: "2025-12-23T10:15:30Z"
spec:
  replicas: 2
  selector:
    matchLabels:
      app: my-api
  template:
    metadata:
      labels:
        app: my-api
        app.kubernetes.io/name: my-api
        app.kubernetes.io/version: "1.0"
        app.kubernetes.io/managed-by: Helm
        org: my-company
        team: platform
        env: prod
    spec:
      containers:
      - name: api
        image: "my-org/my-api:1.0.0"
        ports:
        - containerPort: 8080
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
          timeoutSeconds: 5
          failureThreshold: 3
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 10
          periodSeconds: 5
          timeoutSeconds: 3
          failureThreshold: 2
        resources:
          requests:
            cpu: 100m
            memory: 128Mi
          limits:
            cpu: 500m
            memory: 512Mi
```

The `include` function pulls templates from the library chart and injects them into the deployment, automatically inheriting organization labels and probes.

---

## Concept 6: Override Patterns for Application-Specific Values

Applications override library defaults through the values.yaml hierarchy while maintaining organizational standards.

### Creating Multiple Application Charts with Overrides

```bash
# Application 1: Web service
mkdir -p web-service/templates

cat > web-service/Chart.yaml <<'EOF'
apiVersion: v2
name: web-service
description: "Web frontend"
type: application
version: 1.0.0
dependencies:
  - name: org-standards
    version: "1.0.0"
    repository: "file://../org-standards"
EOF

cat > web-service/values.yaml <<'EOF'
appName: web-service
replicaCount: 3  # More replicas for web traffic

image:
  repository: my-org/web-service
  tag: "2.1.0"

org-standards:
  enabled: true
  organization:
    name: my-company
    team: frontend  # Override team
    environment: prod

  # Override default probes for web service
  customProbes:
    livenessProbe:
      httpGet:
        path: /api/health
        port: 8080
      initialDelaySeconds: 15  # Faster startup
EOF

# Application 2: Background worker
mkdir -p bg-worker/templates

cat > bg-worker/Chart.yaml <<'EOF'
apiVersion: v2
name: bg-worker
description: "Background job processor"
type: application
version: 1.0.0
dependencies:
  - name: org-standards
    version: "1.0.0"
    repository: "file://../org-standards"
EOF

cat > bg-worker/values.yaml <<'EOF'
appName: bg-worker
replicaCount: 2

image:
  repository: my-org/bg-worker
  tag: "1.5.0"

org-standards:
  enabled: true
  organization:
    name: my-company
    team: backend  # Override team
    environment: prod
EOF
```

**Output:**
```
# Web service inherits all org-standards labels/probes
$ helm template web-service . | grep -A 5 labels:
  labels:
    app.kubernetes.io/name: web-service
    org: my-company
    team: frontend  # Team overridden in values

# Background worker gets same labels
$ helm template bg-worker . | grep -A 5 labels:
  labels:
    app.kubernetes.io/name: bg-worker
    org: my-company
    team: backend  # Different team
```

Each application overrides organizational defaults for its context while maintaining consistency through the library chart.

---

## Concept 7: Enterprise Use Cases

Library charts solve real organizational problems: enforcing security policies, maintaining labeling conventions, and standardizing resource defaults across hundreds of deployments.

### Common Enterprise Library Chart Patterns

**Pattern 1: Security Enforcement**

```bash
cat > org-standards/templates/_security.tpl <<'EOF'
{{- define "org-standards.securityContext" -}}
securityContext:
  runAsNonRoot: true
  runAsUser: 1000
  fsGroup: 1000
  capabilities:
    drop:
      - ALL
  readOnlyRootFilesystem: true
{{- end }}
EOF
```

All deployments automatically get security hardening without individual configuration.

**Pattern 2: Compliance Labels**

```bash
cat > org-standards/values.yaml <<'EOF'
complianceLabels:
  audit-enabled: "true"
  pci-dss: "required"
  hipaa: "not-applicable"
  gdpr-processing: "true"
EOF
```

Platform teams enforce compliance requirements that auditors verify through labels.

**Pattern 3: Cost Attribution**

```bash
cat > org-standards/templates/_costAttribution.tpl <<'EOF'
{{- define "org-standards.costLabels" -}}
cost-center: {{ .Values.costCenter }}
project: {{ .Values.projectName }}
owning-team: {{ .Values.organization.team }}
{{- end }}
EOF
```

Finance teams track costs per team/project through labels injected by the library.

---

## Concept 8: Template Inheritance and Overrides

Templates in library charts can be extended or overridden by application charts, creating a flexible inheritance pattern.

### Base Probe Template with Override Points

```bash
cat > org-standards/templates/_probes-extended.tpl <<'EOF'
{{- define "org-standards.livenessProbe" -}}
livenessProbe:
  {{- if .Values.probes.liveness.custom }}
  {{- toYaml .Values.probes.liveness.custom | nindent 2 }}
  {{- else }}
  httpGet:
    path: {{ .Values.probes.liveness.path | default "/health" }}
    port: {{ .Values.probes.liveness.port | default 8080 }}
  initialDelaySeconds: {{ .Values.probes.liveness.initialDelay | default 30 }}
  periodSeconds: {{ .Values.probes.liveness.period | default 10 }}
  timeoutSeconds: {{ .Values.probes.liveness.timeout | default 5 }}
  failureThreshold: {{ .Values.probes.liveness.failureThreshold | default 3 }}
  {{- end }}
{{- end }}
EOF
```

### Application Chart Overrides Probe Behavior

```bash
cat > my-app/values.yaml <<'EOF'
# Standard org-standards usage
org-standards:
  enabled: true
  organization:
    name: my-company
    team: platform

# Override probe behavior for specific app
probes:
  liveness:
    path: /custom-health-check
    initialDelay: 60
    failureThreshold: 5
EOF
```

**Output:**
```
$ helm template my-app . | grep -A 10 "livenessProbe:"
livenessProbe:
  httpGet:
    path: /custom-health-check  # Overridden
    port: 8080
  initialDelaySeconds: 60  # Overridden
  periodSeconds: 10
  timeoutSeconds: 5
  failureThreshold: 5  # Overridden
```

The library chart defines structure and defaults, but applications customize behavior through values without modifying templates.

---

## Exercises

### Exercise 8.1: Create a Library Chart

Create your own library chart with organizational standards.

**Steps:**

1. Create directory: `mkdir -p org-lib/templates`
2. Create `Chart.yaml` with `type: library`
3. Create `values.yaml` with default labels and annotations
4. Create `_helpers.tpl` with common label template

**Validation:** `helm lint org-lib/` should show no errors

**Expected outcome:**
- Chart.yaml contains `type: library`
- values.yaml has default organization metadata
- _helpers.tpl defines reusable label template

---

### Exercise 8.2: Verify Library Chart Cannot Be Installed

Attempt to install the library chart and observe Helm's protection.

**Steps:**

1. Run: `helm install test org-lib/ --dry-run=client`
2. Observe the error message

**Expected error:**
```
error: chart "org-lib" is a library chart. Library charts are not installable.
```

---

### Exercise 8.3: Create Organizational Templates

Add security context, resource requests, and probe templates to your library.

**Steps:**

1. Create `_security.tpl` with SecurityContext template
2. Create `_resources.tpl` with resource requests/limits
3. Create `_probes.tpl` with liveness/readiness probes

**Validation:** `helm lint org-lib/` should succeed

---

### Exercise 8.4: Create Application Chart with Library Dependency

Build an application chart that declares the library as a dependency.

**Steps:**

1. Create app chart directory: `mkdir -p my-service/templates`
2. Add library dependency in `Chart.yaml`
3. Run: `helm dependency update my-service/`

**Validation:** `ls my-service/charts/` should show org-lib

---

### Exercise 8.5: Use `include` to Pull Library Templates

Create a deployment that uses library templates.

**Steps:**

1. Create `templates/deployment.yaml` in application chart
2. Use `include "org-lib.labels" .` for labels
3. Use `include "org-lib.livenessProbe" .` for probes
4. Run: `helm template my-service .`

**Validation:** Template output includes labels and probes from library

---

### Exercise 8.6: Override Library Defaults

Modify application values to override library probe defaults.

**Steps:**

1. Add probe customization to `values.yaml`
2. Modify `templates/deployment.yaml` to use conditional overrides
3. Run: `helm template my-service .` twice:
   - First with defaults
   - Then with custom values

**Validation:** Second template output uses custom probe paths/timings

---

## Try With AI

**Part 1: Design Library Chart Structure**

Ask AI: "I have 5 microservices that all need the same Kubernetes labels (organization, team, cost-center), security context (runAsNonRoot, readOnlyRootFilesystem), and health check probes. I want to avoid repeating this in each service's chart. How would I structure a library chart to enforce these standards across all services?"

**Part 2: Evaluate the Recommendation**

Review AI's response. Ask yourself:

- Does the recommendation include `type: library` in Chart.yaml?
- Does it separate reusable templates (helpers) from application logic?
- Can all 5 services consume this single library without modification?

**Part 3: Request Implementation Guidance**

Tell AI: "I need the library to provide templates for common labels, security context, and probes. The application charts should be able to override the probe paths (some use /health, others use /api/health-check). Walk me through how to structure the template conditionals so apps can override without duplicating code."

**Part 4: Test the Pattern**

Ask AI: "Show me how an application chart would declare this library as a dependency and use the library's label template. I want to see the Chart.yaml, values.yaml, and a sample deployment.yaml that pulls in the library templates."

**Part 5: Validate the Approach**

Ask yourself:

- Does the application chart dependency declaration match library chart naming?
- Can I see which values override organizational defaults?
- Is the `include` syntax correct for pulling templates from dependency charts?
- Would this pattern scale to 10+ applications without duplication?
