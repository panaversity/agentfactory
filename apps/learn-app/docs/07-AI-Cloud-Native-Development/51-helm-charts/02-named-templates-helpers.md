---
sidebar_position: 2
chapter: 51
lesson: 2
duration_minutes: 45
title: "Named Templates and Helpers"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Create reusable template partials with define and include"
cognitive_load:
  concepts_count: 7
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Define named templates using the define action"
    bloom_level: "Apply"
  - id: LO2
    description: "Create helper templates in _helpers.tpl"
    bloom_level: "Create"
  - id: LO3
    description: "Use include instead of template for better output handling"
    bloom_level: "Apply"
  - id: LO4
    description: "Understand scope and context passing in named templates"
    bloom_level: "Understand"
  - id: LO5
    description: "Apply naming conventions for organizational consistency"
    bloom_level: "Apply"
---

# Named Templates and Helpers

In Lesson 1, you learned Go templating basics: variables, pipelines, conditionals, ranges. You can now write templates that branch on conditions and loop through data.

But you discovered a problem: You're repeating the same labels (`app: myapp`, `version: 1.0`, `env: prod`) in your Deployment, Service, and ConfigMap. That's not a templating problem—that's a code repetition problem. You need a way to define template fragments once and reuse them everywhere.

This lesson introduces **named templates** and the **_helpers.tpl convention**—the DRY principle for Helm charts. You'll define common patterns once (labels, selectors, image pull secrets, annotations) and include them wherever needed.

---

## 1. Named Templates with `define`

In Lesson 1, every template you wrote was a full file that produced output. But templates can also define **functions** that output nothing on their own.

The `define` action creates a named template:

```yaml
{{ define "chartname.labels" }}
app: {{ .Chart.Name }}
version: {{ .Chart.AppVersion }}
environment: {{ .Values.environment }}
{{ end }}
```

**Key points:**
- `define` starts a named template block
- `chartname.labels` is the template's name (convention: `chart-name.component`)
- Content between `define` and `end` is stored, not immediately output
- The block produces no output when defined

**Output:**

When you define a template, nothing appears. The template is registered in memory, waiting to be called.

---

## 2. The _helpers.tpl File Convention

Helm has a naming convention: Templates starting with `_` (underscore) are **not rendered as standalone manifests**. They're utilities meant to be included elsewhere.

Create `templates/_helpers.tpl`:

```yaml
{{- define "myapp.labels" }}
app: {{ .Chart.Name }}
version: {{ .Chart.AppVersion }}
{{- end }}

{{- define "myapp.selector" }}
app: {{ .Chart.Name }}
{{- end }}

{{- define "myapp.imagePullSecrets" }}
{{- if .Values.imagePullSecrets }}
imagePullSecrets:
{{- range .Values.imagePullSecrets }}
- name: {{ . }}
{{- end }}
{{- end }}
{{- end }}
```

**Why _helpers.tpl?**
- Convention recognized by all Helm users (like `_config.scss` in CSS)
- Prevents accidental rendering (underscore files are skipped)
- Centralizes reusable fragments
- One place to find all helpers

**Output:**

No files are created. `_helpers.tpl` is never rendered. It only provides functions for other templates to include.

---

## 3. Including Templates with `include`

Now you call a named template using `include`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  labels:
    {{ include "myapp.labels" . | nindent 4 }}
spec:
  selector:
    matchLabels:
      {{ include "myapp.selector" . | nindent 6 }}
  template:
    metadata:
      labels:
        {{ include "myapp.labels" . | nindent 8 }}
    spec:
      containers:
      - name: app
        image: myapp:{{ .Values.version }}
```

**Syntax breakdown:**
- `include "myapp.labels" .` calls the named template, passing `.` (current context)
- `| nindent 4` pipes the output to `nindent` (indent and newline)
- `nindent N` indents output by N spaces, preserving YAML structure

**Output:**

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  labels:
    app: myapp
    version: 1.0
spec:
  selector:
    matchLabels:
      app: myapp
  template:
    metadata:
      labels:
        app: myapp
        version: 1.0
    spec:
      containers:
      - name: app
        image: myapp:1.0
```

---

## 4. Why `include` Replaces the Deprecated `template`

Helm originally provided `template` for calling named templates. **`template` is now deprecated** (removed in Helm 3.13+).

Why? The problem is output handling.

**Problem with `template`:**

```yaml
metadata:
  labels:
{{ template "myapp.labels" . }}
```

`template` doesn't support pipes. You can't control indentation. Output becomes:

```yaml
metadata:
  labels:
app: myapp
version: 1.0
```

Notice: No indentation. YAML is broken.

**Solution with `include`:**

```yaml
metadata:
  labels:
{{ include "myapp.labels" . | nindent 4 }}
```

`include` returns a string that you can pipe to `nindent`. Output:

```yaml
metadata:
  labels:
  app: myapp
  version: 1.0
```

Properly indented. Valid YAML.

**Output comparison:**

`template` (broken indentation):
```yaml
metadata:
  labels:
app: myapp
```

`include` (correct):
```yaml
metadata:
  labels:
  app: myapp
```

**Key difference:**
- `template`: Renders directly, no output control
- `include`: Returns string, composable with pipes

**Always use `include`.** Never use `template` for new code.

---

## 5. Scope Rules: What `.` Means Inside a Named Template

When you call a named template, you pass `.` as the current context. Inside the template, `.` refers to whatever you passed.

```yaml
{{- define "myapp.labels" }}
name: {{ .Chart.Name }}     # Chart name from passed context
env: {{ .Values.environment }} # Values from passed context
{{- end }}
```

When you call:

```yaml
{{ include "myapp.labels" . | nindent 4 }}
```

Inside `myapp.labels`, `.` is the root context (`.Chart`, `.Values`, etc. all work).

**Scope rule: Inside a named template, `.` refers to whatever you passed to include/template.**

Common mistake:

```yaml
{{- define "myapp.labels" }}
{{- range .Values.tags }}
tag: {{ . }}    # . is now the tag value (a string), not the root context
{{- end }}
{{- end }}
```

Inside the range, `.` becomes each tag string. You can't access `.Chart` anymore.

To access both root context and loop value:

```yaml
{{- define "myapp.labels" }}
{{- range .Values.tags }}
tag: {{ . }}
chart: {{ $.Chart.Name }}  # $ always refers to root context
{{- end }}
{{- end }}
```

**$ is a special variable**: Always refers to root context, no matter how deep you nest.

---

## 6. Common Patterns

### Pattern: Labels (Most Common)

```yaml
{{- define "myapp.labels" }}
app: {{ .Chart.Name }}
version: {{ .Chart.AppVersion }}
managed-by: {{ .Release.Service }}
release: {{ .Release.Name }}
{{- end }}
```

Used in every Kubernetes manifest.

### Pattern: Selectors

```yaml
{{- define "myapp.selector" }}
app: {{ .Chart.Name }}
{{- end }}
```

Used in Deployments, Services, StatefulSets.

### Pattern: Image Pull Secrets (Conditional)

```yaml
{{- define "myapp.imagePullSecrets" }}
{{- if .Values.imagePullSecrets }}
imagePullSecrets:
{{- range .Values.imagePullSecrets }}
- name: {{ . }}
{{- end }}
{{- end }}
{{- end }}
```

Only rendered if `imagePullSecrets` is defined.

### Pattern: Annotations (with defaults)

```yaml
{{- define "myapp.annotations" }}
annotations:
  description: {{ .Values.podAnnotations.description | default "AI Service" }}
  {{- if .Values.podAnnotations.custom }}
  custom: {{ .Values.podAnnotations.custom }}
  {{- end }}
{{- end }}
```

Combines defaults with conditional fields.

---

## 7. Naming Conventions

The standard Helm convention is: `chartname.component`

**Examples from real charts:**
- `postgresql.primary.fullname`
- `redis.fullname`
- `nginx.ingress.className`

**Your chart:** `myapp.labels`, `myapp.selector`, `myapp.imagePullSecrets`

**Why this pattern?**
1. **Namespacing**: Prevents collisions with subchart helpers
2. **Clarity**: `myapp.labels` immediately shows purpose
3. **Searchability**: `grep "myapp\." _helpers.tpl` finds all your helpers
4. **Consistency**: Everyone following convention means predictable code

---

## Exercises

### Exercise 2.1: Create _helpers.tpl with Common Labels

Create `templates/_helpers.tpl`:

```yaml
{{- define "aiagent.labels" }}
app: {{ .Chart.Name }}
version: {{ .Chart.AppVersion }}
environment: {{ .Values.environment | default "development" }}
managed-by: {{ .Release.Service }}
{{- end }}
```

Verify it's not rendered:

```bash
helm template my-agent ./chart
```

**Output:**

No YAML with `app: aiagent` appears. The template is defined but not rendered.

---

### Exercise 2.2: Include Labels in Multiple Manifests

Create `templates/deployment.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ .Release.Name }}-app
  labels:
    {{ include "aiagent.labels" . | nindent 4 }}
spec:
  selector:
    matchLabels:
      app: {{ .Chart.Name }}
  template:
    metadata:
      labels:
        {{ include "aiagent.labels" . | nindent 8 }}
    spec:
      containers:
      - name: app
        image: myapp:{{ .Values.version }}
```

Create `templates/service.yaml`:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: {{ .Release.Name }}-service
  labels:
    {{ include "aiagent.labels" . | nindent 4 }}
spec:
  selector:
    app: {{ .Chart.Name }}
  ports:
  - port: 80
    targetPort: 8000
```

Create `templates/configmap.yaml`:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: {{ .Release.Name }}-config
  labels:
    {{ include "aiagent.labels" . | nindent 4 }}
data:
  app.env: {{ .Values.environment }}
```

Render:

```bash
helm template my-agent ./chart
```

**Output:**

Three manifests (Deployment, Service, ConfigMap) all include the same labels from your helper.

---

### Exercise 2.3: Create Image Pull Secrets Helper with Conditional Logic

Update `templates/_helpers.tpl`:

```yaml
{{- define "aiagent.imagePullSecrets" }}
{{- if .Values.imagePullSecrets }}
imagePullSecrets:
{{- range .Values.imagePullSecrets }}
- name: {{ . }}
{{- end }}
{{- else }}
# No image pull secrets configured
{{- end }}
{{- end }}
```

Update `values.yaml`:

```yaml
environment: production
version: 1.0
imagePullSecrets:
  - dockerhub-secret
  - gcr-secret
```

Use in `templates/deployment.yaml`:

```yaml
spec:
  template:
    spec:
      {{ include "aiagent.imagePullSecrets" . | nindent 6 }}
      containers:
      - name: app
```

Render:

```bash
helm template my-agent ./chart
```

**Output:**

```yaml
spec:
  template:
    spec:
      imagePullSecrets:
      - name: dockerhub-secret
      - name: gcr-secret
      containers:
      - name: app
```

Change `values.yaml` to `imagePullSecrets: []` and render again:

**Output:**

```yaml
spec:
  template:
    spec:
      containers:
      - name: app
```

The `imagePullSecrets` section is omitted.

---

### Exercise 2.4: Demonstrate Why `include` Works Where `template` Fails

Create a test file using old `template` syntax:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  labels:
{{ template "aiagent.labels" . }}
```

Render:

```bash
helm template my-agent ./chart
```

**Output (BROKEN):**

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  labels:
app: aiagent
version: 1.0
environment: production
managed-by: Helm
```

YAML is invalid. Labels aren't indented under `labels:`.

Now use `include`:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  labels:
{{ include "aiagent.labels" . | nindent 4 }}
```

**Output (CORRECT):**

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  labels:
    app: aiagent
    version: 1.0
    environment: production
    managed-by: Helm
```

Valid YAML, proper indentation.

---

### Exercise 2.5: Refactor 3 Duplicate Sections into Single Include

You have three manifests with duplicate `metadata.labels`:

**Before:**

```yaml
# deployment.yaml
metadata:
  labels:
    app: myapp
    version: 1.0
    env: prod

# service.yaml
metadata:
  labels:
    app: myapp
    version: 1.0
    env: prod

# configmap.yaml
metadata:
  labels:
    app: myapp
    version: 1.0
    env: prod
```

**After:**

Create helper in `_helpers.tpl`:

```yaml
{{- define "myapp.labels" }}
app: myapp
version: 1.0
env: prod
{{- end }}
```

Use in all three:

```yaml
metadata:
  labels:
    {{ include "myapp.labels" . | nindent 4 }}
```

**Benefit:** Change labels once in `_helpers.tpl`. All three manifests automatically use the new labels.

---

## Try With AI

### Setup

You've created a chart with separate Deployment, Service, and ConfigMap. Each has duplicate labels, annotations, and image pull secret handling. Your job is to refactor common patterns into `_helpers.tpl` and simplify your manifests.

**Files to work with:**
- `templates/_helpers.tpl` (create this)
- `templates/deployment.yaml`
- `templates/service.yaml`
- `templates/configmap.yaml`
- `values.yaml`

### Part 1: Initial State (Current Duplications)

Show AI your current manifests:

```
I have three manifests with repeated labels and selectors:

[paste your current deployment.yaml, service.yaml, configmap.yaml]

What patterns would you identify as candidates for helpers?
```

### Part 2: Helper Design

Based on AI's suggestions, ask:

```
I want to create these helpers in _helpers.tpl:
1. labels (for metadata.labels in all manifests)
2. selector (for spec.selector.matchLabels in Deployment/Service)
3. imagePullSecrets (conditional, only if Values.imagePullSecrets exists)

For each helper, show me the define/end block and how I should call it with include and nindent.
```

### Part 3: Validation

Ask AI to help you validate:

```
After I add these helpers and refactor my manifests to use include,
how do I verify with `helm template` that:
1. Labels appear correctly indented in all manifests?
2. Selectors are only in Deployment and Service, not ConfigMap?
3. ImagePullSecrets appear when configured, disappear when empty?
```

### Part 4: Refinement

Show AI your refactored manifests:

```
Here's my refactored deployment.yaml using include:

[paste your refactored file]

Are there any places where I should add nindent, or any indentation issues?
```

### Part 5: Final Check

Verify your work:

```bash
helm template my-agent ./chart > rendered.yaml
cat rendered.yaml
```

Compare the output:
- Do all labels appear with correct indentation?
- Do selectors only appear where needed?
- Are image pull secrets included/excluded based on values.yaml?

Check your helpers are used everywhere they should be:

```bash
grep -r "include \"myapp\." templates/
```

Should show calls in deployment, service, configmap—not in _helpers.tpl definitions.

---
