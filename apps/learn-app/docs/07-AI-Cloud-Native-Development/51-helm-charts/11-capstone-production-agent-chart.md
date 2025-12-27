---
sidebar_position: 11
chapter: 51
lesson: 11
duration_minutes: 75
title: "Capstone: Production AI Agent Chart"
proficiency_level: B2
teaching_stage: 4
stage_name: "Spec-Driven Integration"
stage_description: "Create complete production-ready Helm chart synthesizing all chapter concepts"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Light"
learning_objectives:
  - id: LO1
    description: "Write a specification for a production Helm chart"
    bloom_level: "Create"
  - id: LO2
    description: "Compose chart with PostgreSQL and Redis dependencies"
    bloom_level: "Create"
  - id: LO3
    description: "Implement pre-upgrade hooks for database migrations"
    bloom_level: "Create"
  - id: LO4
    description: "Create multi-environment values files (dev/staging/prod)"
    bloom_level: "Create"
  - id: LO5
    description: "Validate chart with helm test and lint"
    bloom_level: "Apply"
---

# Capstone: Production AI Agent Chart

You've learned to template (Lesson 1), compose dependencies (Lesson 4), orchestrate with hooks (Lesson 5), test charts (Lesson 6), and distribute via OCI (Lesson 7). You understand library charts (Lesson 8) and have collaborated with AI on chart development (Lesson 9).

Now you'll synthesize everything into a single production-grade project: deploying a complete AI agent that talks to PostgreSQL for state and Redis for caching, with database migrations automated through hooks, and configuration tailored for dev/staging/production environments.

This is a **specification-first** capstone. You begin by writing a clear specification of what you're building before implementing anything. The specification becomes your contract with your implementation—and with AI if you choose to use it for validation or refinement.

---

## Part 1: Specification

Before writing any YAML, establish what you're building.

### Intent: What Are We Building?

You're creating a production-ready Helm chart that deploys an AI agent service with complete infrastructure:

- **AI Agent Container**: The application itself (could be your Part 6 AI service)
- **PostgreSQL Database**: Persistent state storage
- **Redis Cache**: In-memory caching for fast inference lookups
- **Database Schema Manager**: Automatic schema initialization and migrations
- **Multi-Environment Configuration**: Different resource levels for dev/staging/prod

### Success Criteria (Acceptance Tests)

Your chart succeeds when ALL of these are true:

**Criterion 1: Single `helm install` Deploys Complete Stack**
```
$ helm install task-api ./ai-agent-chart -f values-prod.yaml
Release task-api installed.
$ kubectl get all --selector=app=task-api
NAME                                  READY   STATUS    RESTARTS   AGE
pod/task-api-deployment-abc123        1/1     Running   0          5s
pod/task-api-postgres-0               1/1     Running   0          5s
pod/task-api-redis-0                  1/1     Running   0          5s
pod/db-migrate-pre-upgrade-abc123     0/1     Completed 0          4s
```
- Agent Pod running
- PostgreSQL StatefulSet running
- Redis running
- Pre-upgrade migration Job completed successfully

**Criterion 2: `helm test` Verifies Connectivity**
```
$ helm test task-api
NAME: task-api
LAST DEPLOYED: Thu Jan 23 14:30:00 2025
NAMESPACE: default
STATUS: deployed

TEST SUITE:     task-api-connection-test
Last Started:   Thu Jan 23 14:30:05 2025
Last Completed: Thu Jan 23 14:30:10 2025
Status:         PASSED
```
- Agent can connect to PostgreSQL
- Agent can connect to Redis
- Both dependencies report "healthy"

**Criterion 3: Multi-Environment Deployment Works**
```
# Deploy to dev with reduced resources
$ helm install agent-dev ./ai-agent-chart -f values-dev.yaml

# Deploy to staging with moderate resources
$ helm install agent-staging ./ai-agent-chart -f values-staging.yaml

# Deploy to prod with full resources
$ helm install agent-prod ./ai-agent-chart -f values-prod.yaml
```
Each deployment uses appropriate resource levels (dev: minimal, staging: moderate, prod: full).

### Requirements

Your chart MUST include:

**Configuration:**
- `Chart.yaml` with dependencies on PostgreSQL and Redis (Bitnami charts)
- `values.yaml` with production defaults
- `values-dev.yaml`, `values-staging.yaml`, `values-prod.yaml` for environment-specific overrides
- `values.schema.json` validation (at least for critical fields)

**Templates:**
- `templates/deployment.yaml` for Agent
- `templates/service.yaml` (ClusterIP for Agent)
- `templates/_helpers.tpl` with standard label macros
- `templates/configmap.yaml` for non-secret configuration
- `templates/secret.yaml` for sensitive data (database credentials)

**Lifecycle Management:**
- `templates/hooks/pre-upgrade-migration.yaml` Job to run database migrations
- Hook annotations with proper weights and delete policies

**Testing:**
- `templates/tests/test-connection.yaml` to verify Agent ↔ DB ↔ Cache connectivity

**Documentation:**
- Chart-level `README.md` with configuration options and usage examples

### Constraints

- All database migrations run BEFORE the deployment updates
- Secrets must NOT appear in ConfigMaps or unencrypted files
- Resource requests must scale appropriately per environment (dev: 256Mi/100m, staging: 512Mi/250m, prod: 1Gi/500m)
- Deployment must survive `helm upgrade` with zero downtime (PreparingUpdate strategy)
- PostgreSQL and Redis must be included as dependencies, NOT deployed externally

### Non-Goals

We are NOT building:

- TLS/HTTPS termination (that's a gateway concern in Chapter 54)
- Service mesh integration (Chapter 57)
- Full GitOps automation (Chapter 55)
- Monitoring dashboards (Chapter 56)
- Multi-region failover (Chapter 53)

This is a single-cluster, HTTP-based deployment focused on correct Helm patterns.

---

## Part 2: Chart Architecture

Before implementing, visualize the component relationships.

### Directory Structure

Your final chart will look like this:

```
ai-agent-chart/
├── Chart.yaml                                 # Chart metadata + dependencies
├── values.yaml                                # Base values (dev-like defaults)
├── values-dev.yaml                            # Dev environment overrides
├── values-staging.yaml                        # Staging environment overrides
├── values-prod.yaml                           # Production environment overrides
├── values.schema.json                         # Schema validation for critical fields
├── README.md                                  # Chart documentation
├── templates/
│   ├── deployment.yaml                        # Agent deployment
│   ├── service.yaml                           # Agent service (ClusterIP)
│   ├── configmap.yaml                         # Configuration (non-secrets)
│   ├── secret.yaml                            # Database credentials
│   ├── _helpers.tpl                           # Label/annotation macros
│   ├── hooks/
│   │   └── pre-upgrade-migration.yaml         # DB migration Job
│   └── tests/
│       └── test-connection.yaml               # Connectivity test Pod
└── charts/                                    # Auto-populated by Helm (don't edit)
    ├── postgresql-VERSION
    └── redis-VERSION
```

### Component Diagram

```
┌──────────────────────────────────────────────┐
│          Kubernetes Cluster                  │
│                                              │
│  ┌─────────────────┐   ┌─────────────────┐  │
│  │  AI Agent Pod   │   │  Agent Service  │  │
│  │  (Deployment)   │───│  (ClusterIP)    │  │
│  └────────┬────────┘   └─────────────────┘  │
│           │                                  │
│     ┌─────┼─────┬──────────┐                │
│     │           │          │                │
│  ┌──▼──┐    ┌──▼──┐   ┌───▼───┐            │
│  │ Pre-│    │Conf-│   │Secret  │            │
│  │ Mig │    │igMap│   │(DB creds)│         │
│  │Job  │    └─────┘   └─────────┘          │
│  └─────┘         (mounted to Pod)           │
│                                              │
│  ┌──────────────┐    ┌──────────────┐      │
│  │  PostgreSQL  │    │    Redis     │      │
│  │ StatefulSet  │    │  StatefulSet │      │
│  │  (dep)       │    │  (dep)       │      │
│  └──────────────┘    └──────────────┘      │
│                                              │
└──────────────────────────────────────────────┘
```

### How Components Relate

1. **Agent Deployment** starts the AI service container
2. **Pre-Upgrade Migration Hook** runs a database schema initialization Job BEFORE the deployment updates
3. **ConfigMap** provides environment-specific configuration to the Agent
4. **Secret** provides database credentials to the Agent (mounted as volume)
5. **PostgreSQL Dependency** is installed automatically (handles StatefulSet, PVC, Service)
6. **Redis Dependency** is installed automatically (handles StatefulSet, Service)
7. **Test Pod** verifies Agent can reach both PostgreSQL and Redis after deployment

---

## Part 3: Implementation

Now build the chart step by step.

### Step 1: Chart.yaml with Dependencies

Define what your chart includes and depends on:

```yaml
---
apiVersion: v2
name: ai-agent
description: "Production-ready Helm chart for AI agent with PostgreSQL and Redis"
type: application
version: 1.0.0
appVersion: "1.0.0"

dependencies:
  - name: postgresql
    version: "12.x.x"
    repository: "https://charts.bitnami.com/bitnami"
    condition: postgresql.enabled
  - name: redis
    version: "17.x.x"
    repository: "https://charts.bitnami.com/bitnami"
    condition: redis.enabled
```

**Output:** This metadata tells Helm:
- The chart is called `ai-agent`
- It depends on PostgreSQL 12.x and Redis 17.x from Bitnami
- Dependencies are only installed if `postgresql.enabled: true` and `redis.enabled: true`

Update dependencies:
```bash
$ helm dependency update ./ai-agent-chart
Hang tight while we grab the latest from your chart repositories...
...Successfully got an update from the "bitnami" chart repository
Update Complete. ⎈ Happy Helming!
```

**Output:** Helm downloads the PostgreSQL and Redis charts to `charts/` directory.

### Step 2: Base values.yaml

Create production-appropriate defaults:

```yaml
---
# Agent deployment configuration
agent:
  replicaCount: 1
  image:
    repository: "my-company/ai-agent"
    tag: "1.0.0"
    pullPolicy: IfNotPresent
  service:
    type: ClusterIP
    port: 8000
  resources:
    requests:
      cpu: "500m"
      memory: "512Mi"
    limits:
      cpu: "1000m"
      memory: "1Gi"
  env:
    LOG_LEVEL: "info"
    WORKER_THREADS: "4"

# PostgreSQL dependency configuration
postgresql:
  enabled: true
  auth:
    username: agent_user
    password: change-this-in-prod
    database: agent_db
  primary:
    persistence:
      enabled: true
      size: 10Gi
  metrics:
    enabled: true

# Redis dependency configuration
redis:
  enabled: true
  auth:
    enabled: true
    password: change-this-in-prod
  master:
    persistence:
      enabled: true
      size: 1Gi
  replica:
    replicaCount: 1

# Pre-upgrade hook configuration
migration:
  enabled: true
  image: "my-company/ai-agent-migrations:1.0.0"
  timeout: 300

# Schema validation
organizationLabels:
  team: "ai-platform"
  environment: "production"
```

**Output:** These values provide:
- Agent image and resource defaults (production-grade 512Mi/1Gi)
- PostgreSQL enabled with persistence
- Redis enabled with replication
- Migration image reference
- Standard organization labels

### Step 3: Environment-Specific Overrides

Create `values-dev.yaml`:

```yaml
---
# Development: minimal resources, relaxed settings
agent:
  replicaCount: 1
  resources:
    requests:
      cpu: "100m"
      memory: "256Mi"
    limits:
      cpu: "250m"
      memory: "512Mi"
  env:
    LOG_LEVEL: "debug"
    WORKER_THREADS: "1"

postgresql:
  auth:
    password: dev-password-ok
  primary:
    persistence:
      size: 1Gi

redis:
  auth:
    password: dev-password-ok
  master:
    persistence:
      size: 100Mi
  replica:
    replicaCount: 0

organizationLabels:
  environment: "dev"
```

**Output:** Dev environment uses:
- 100m CPU / 256Mi memory (1/5 of production)
- Debug logging
- 1 worker thread instead of 4
- No Redis replicas
- Smaller persistent volumes

Create `values-staging.yaml`:

```yaml
---
# Staging: moderate resources, matches production-like settings
agent:
  replicaCount: 2
  resources:
    requests:
      cpu: "250m"
      memory: "512Mi"
    limits:
      cpu: "750m"
      memory: "1Gi"
  env:
    LOG_LEVEL: "info"
    WORKER_THREADS: "2"

postgresql:
  auth:
    password: staging-password-change
  primary:
    persistence:
      size: 5Gi

redis:
  auth:
    password: staging-password-change
  replica:
    replicaCount: 1

organizationLabels:
  environment: "staging"
```

**Output:** Staging environment uses:
- 250m CPU / 512Mi memory (moderate)
- 2 replicas for HA testing
- Production-like configuration
- Medium storage volumes

Create `values-prod.yaml`:

```yaml
---
# Production: full resources, HA configuration
agent:
  replicaCount: 3
  resources:
    requests:
      cpu: "500m"
      memory: "1Gi"
    limits:
      cpu: "2000m"
      memory: "2Gi"
  env:
    LOG_LEVEL: "warn"
    WORKER_THREADS: "8"

postgresql:
  auth:
    password: prod-secret-from-sealed-secrets
  primary:
    persistence:
      size: 50Gi
  replica:
    replicaCount: 2

redis:
  auth:
    password: prod-secret-from-sealed-secrets
  master:
    persistence:
      size: 10Gi
  replica:
    replicaCount: 2

organizationLabels:
  environment: "production"
```

**Output:** Production environment uses:
- 500m CPU / 1Gi memory (full tier)
- 3 Agent replicas
- PostgreSQL replicas enabled
- Redis replicas enabled
- Large persistent volumes
- Reduced logging (warn level only)

### Step 4: Chart.yaml and Schema

Create `values.schema.json` for validation:

```json
{
  "$schema": "https://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "agent": {
      "type": "object",
      "properties": {
        "replicaCount": {
          "type": "integer",
          "minimum": 1,
          "maximum": 10
        },
        "resources": {
          "type": "object",
          "properties": {
            "requests": {
              "type": "object",
              "required": ["cpu", "memory"]
            }
          }
        }
      }
    },
    "postgresql": {
      "type": "object",
      "properties": {
        "enabled": { "type": "boolean" },
        "primary": {
          "properties": {
            "persistence": {
              "required": ["enabled"]
            }
          }
        }
      }
    }
  }
}
```

**Output:** This schema enforces:
- replicaCount between 1-10
- Resource requests for CPU and memory are required
- PostgreSQL must have persistence enabled
- Invalid configurations are caught during validation

### Step 5: Deployment Template

Create `templates/deployment.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ include "ai-agent.fullname" . }}
  labels:
    {{- include "ai-agent.labels" . | nindent 4 }}
spec:
  replicas: {{ .Values.agent.replicaCount }}
  selector:
    matchLabels:
      {{- include "ai-agent.selectorLabels" . | nindent 6 }}
  template:
    metadata:
      labels:
        {{- include "ai-agent.selectorLabels" . | nindent 8 }}
      annotations:
        checksum/config: {{ include (print $.Template.BasePath "/configmap.yaml") . | sha256sum }}
        checksum/secret: {{ include (print $.Template.BasePath "/secret.yaml") . | sha256sum }}
    spec:
      serviceAccountName: {{ include "ai-agent.serviceAccountName" . }}
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
        fsGroup: 1000
      containers:
      - name: agent
        image: "{{ .Values.agent.image.repository }}:{{ .Values.agent.image.tag }}"
        imagePullPolicy: {{ .Values.agent.image.pullPolicy }}
        ports:
        - name: http
          containerPort: 8000
          protocol: TCP
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: {{ include "ai-agent.fullname" . }}-secret
              key: database-url
        - name: REDIS_URL
          valueFrom:
            secretKeyRef:
              name: {{ include "ai-agent.fullname" . }}-secret
              key: redis-url
        {{- range $key, $value := .Values.agent.env }}
        - name: {{ $key }}
          value: "{{ $value }}"
        {{- end }}
        resources:
          {{- toYaml .Values.agent.resources | nindent 12 }}
        livenessProbe:
          httpGet:
            path: /health
            port: http
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: http
          initialDelaySeconds: 5
          periodSeconds: 5
```

**Output:** This creates:
- Deployment with configurable replicas
- Pod with agent container
- Database and Redis connection URLs from secrets
- Liveness/readiness probes
- Non-root security context
- Checksum annotations (triggers rollouts when config changes)

### Step 6: Service Template

Create `templates/service.yaml`:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: {{ include "ai-agent.fullname" . }}
  labels:
    {{- include "ai-agent.labels" . | nindent 4 }}
spec:
  type: {{ .Values.agent.service.type }}
  ports:
    - port: {{ .Values.agent.service.port }}
      targetPort: http
      protocol: TCP
      name: http
  selector:
    {{- include "ai-agent.selectorLabels" . | nindent 4 }}
```

**Output:** Creates a ClusterIP service exposing port 8000 to other pods in the cluster.

### Step 7: ConfigMap Template

Create `templates/configmap.yaml`:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: {{ include "ai-agent.fullname" . }}-config
  labels:
    {{- include "ai-agent.labels" . | nindent 4 }}
data:
  LOG_LEVEL: "{{ .Values.agent.env.LOG_LEVEL }}"
  WORKER_THREADS: "{{ .Values.agent.env.WORKER_THREADS }}"
  ORGANIZATION: "{{ .Values.organizationLabels.team }}"
  ENVIRONMENT: "{{ .Values.organizationLabels.environment }}"
```

**Output:** Non-sensitive configuration stored in ConfigMap (separate from secrets).

### Step 8: Secret Template

Create `templates/secret.yaml`:

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: {{ include "ai-agent.fullname" . }}-secret
  labels:
    {{- include "ai-agent.labels" . | nindent 4 }}
type: Opaque
data:
  database-url: {{ printf "postgresql://agent_user:%s@%s-postgresql:5432/agent_db" .Values.postgresql.auth.password (include "ai-agent.fullname" .) | b64enc | quote }}
  redis-url: {{ printf "redis://:%s@%s-redis-master:6379/0" .Values.redis.auth.password (include "ai-agent.fullname" .) | b64enc | quote }}
```

**Output:** Base64-encoded secrets for:
- PostgreSQL connection URL (constructed from dependency hostname)
- Redis connection URL (constructed from dependency hostname)

### Step 9: Helpers Template

Create `templates/_helpers.tpl`:

```gotemplate
{{- define "ai-agent.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}

{{- define "ai-agent.fullname" -}}
{{- if .Values.fullnameOverride }}
{{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- $name := default .Chart.Name .Values.nameOverride }}
{{- if contains $name .Release.Name }}
{{- .Release.Name | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- printf "%s-%s" .Release.Name $name | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}
{{- end }}

{{- define "ai-agent.chart" -}}
{{- printf "%s-%s" .Chart.Name .Chart.Version | replace "+" "_" | trunc 63 | trimSuffix "-" }}
{{- end }}

{{- define "ai-agent.labels" -}}
helm.sh/chart: {{ include "ai-agent.chart" . }}
{{ include "ai-agent.selectorLabels" . }}
{{- if .Chart.AppVersion }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
{{- end }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end }}

{{- define "ai-agent.selectorLabels" -}}
app.kubernetes.io/name: {{ include "ai-agent.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
{{- end }}

{{- define "ai-agent.serviceAccountName" -}}
{{- if .Values.serviceAccount.create }}
{{- default (include "ai-agent.fullname" .) .Values.serviceAccount.name }}
{{- else }}
{{- default "default" .Values.serviceAccount.name }}
{{- end }}
{{- end }}
```

**Output:** Helper functions for consistent naming and labels throughout all templates.

### Step 10: Pre-Upgrade Migration Hook

Create `templates/hooks/pre-upgrade-migration.yaml`:

```yaml
apiVersion: batch/v1
kind: Job
metadata:
  name: {{ include "ai-agent.fullname" . }}-pre-upgrade
  labels:
    {{- include "ai-agent.labels" . | nindent 4 }}
  annotations:
    "helm.sh/hook": pre-upgrade
    "helm.sh/hook-weight": "-5"
    "helm.sh/hook-delete-policy": before-hook-creation,hook-succeeded
spec:
  template:
    metadata:
      labels:
        {{- include "ai-agent.selectorLabels" . | nindent 8 }}
    spec:
      serviceAccountName: {{ include "ai-agent.serviceAccountName" . }}
      securityContext:
        runAsNonRoot: true
        runAsUser: 1000
      containers:
      - name: migrate
        image: "{{ .Values.migration.image }}"
        imagePullPolicy: IfNotPresent
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: {{ include "ai-agent.fullname" . }}-secret
              key: database-url
        command:
          - /bin/sh
          - -c
          - |
            echo "Running database migrations..."
            python -m alembic upgrade head
            echo "Migration completed successfully"
        resources:
          limits:
            memory: "256Mi"
            cpu: "100m"
      restartPolicy: Never
      backoffLimit: 3
  activeDeadlineSeconds: {{ .Values.migration.timeout }}
```

**Output:** When you run `helm upgrade`, this Job runs FIRST:
- Executes database migrations before the deployment updates
- Uses weight -5 to run first in sequence
- Deletes itself after success
- Supports retries (backoffLimit: 3)
- Timeout after 5 minutes

### Step 11: Connection Test

Create `templates/tests/test-connection.yaml`:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: {{ include "ai-agent.fullname" . }}-connection-test
  labels:
    {{- include "ai-agent.labels" . | nindent 4 }}
  annotations:
    "helm.sh/hook": test
spec:
  serviceAccountName: {{ include "ai-agent.serviceAccountName" . }}
  containers:
  - name: test
    image: "{{ .Values.agent.image.repository }}:{{ .Values.agent.image.tag }}"
    env:
    - name: DATABASE_URL
      valueFrom:
        secretKeyRef:
          name: {{ include "ai-agent.fullname" . }}-secret
          key: database-url
    - name: REDIS_URL
      valueFrom:
        secretKeyRef:
          name: {{ include "ai-agent.fullname" . }}-secret
          key: redis-url
    command:
      - /bin/sh
      - -c
      - |
        echo "Testing PostgreSQL connection..."
        python -c "from sqlalchemy import create_engine; engine = create_engine(os.environ['DATABASE_URL']); engine.execute('SELECT 1')"
        echo "PostgreSQL connection successful!"

        echo "Testing Redis connection..."
        python -c "import redis; r = redis.from_url(os.environ['REDIS_URL']); r.ping()"
        echo "Redis connection successful!"

        echo "All connectivity tests passed!"
  restartPolicy: Never
```

**Output:** When you run `helm test task-api`, this Pod verifies both database and cache are accessible.

### Step 12: Chart README

Create `README.md`:

```markdown
# AI Agent Helm Chart

A production-ready Helm chart for deploying AI agents with PostgreSQL and Redis.

## Features

- AI agent deployment with horizontal pod autoscaling
- PostgreSQL dependency for persistent state
- Redis dependency for caching
- Automatic database schema migrations (pre-upgrade hooks)
- Multi-environment support (dev/staging/prod)
- Health checks and readiness probes
- Deployment connectivity tests

## Installation

### Prerequisites

- Kubernetes 1.20+
- Helm 3.0+
- Bitnami Helm repository added

### Add Repository

```bash
helm repo add bitnami https://charts.bitnami.com/bitnami
helm repo update
```

### Deploy to Dev

```bash
helm install task-api ./ai-agent-chart -f values-dev.yaml --namespace dev --create-namespace
```

### Deploy to Production

```bash
helm install task-api ./ai-agent-chart -f values-prod.yaml --namespace prod --create-namespace
```

## Configuration

| Parameter | Description | Default |
|-----------|-------------|---------|
| `agent.replicaCount` | Number of agent replicas | `1` |
| `agent.image.repository` | Agent container image | `my-company/ai-agent` |
| `agent.image.tag` | Agent image tag | `1.0.0` |
| `agent.resources.requests.cpu` | CPU request | `500m` |
| `agent.resources.requests.memory` | Memory request | `512Mi` |
| `postgresql.enabled` | Enable PostgreSQL | `true` |
| `postgresql.auth.password` | Database password | `change-this-in-prod` |
| `redis.enabled` | Enable Redis | `true` |
| `redis.auth.password` | Redis password | `change-this-in-prod` |

## Usage

### Update Release

```bash
helm upgrade task-api ./ai-agent-chart -f values-prod.yaml
```

The pre-upgrade migration hook runs automatically before the upgrade proceeds.

### Test Connectivity

```bash
helm test task-api
```

This verifies Agent ↔ PostgreSQL and Agent ↔ Redis connectivity.

### Rollback Release

```bash
helm rollback task-api 1
```

### Delete Release

```bash
helm uninstall task-api
```

## Troubleshooting

### Migration Failed

Check migration job logs:
```bash
kubectl logs -l app=task-api,job=migration --tail=100
```

### Agent Pod Not Running

Check deployment:
```bash
kubectl describe deployment task-api
kubectl logs -l app=task-api
```

### Database Connection Errors

Verify secret:
```bash
kubectl get secret task-api-secret -o yaml
```

Verify PostgreSQL is running:
```bash
kubectl get pods -l app.kubernetes.io/name=postgresql
```
```

**Output:** Documentation covering installation, configuration options, usage patterns, and troubleshooting.

---

## Part 4: Validation

Verify your chart meets the specification before deployment.

### Check 1: Helm Lint

Validate chart syntax and best practices:

```bash
$ helm lint ./ai-agent-chart
==> Linting ./ai-agent-chart
[INFO] Chart.yaml: icon is recommended

1 chart(s) linted, 0 error(s)
```

**Output:** Chart passes linting (icon warning is optional).

### Check 2: Helm Template

Render templates without installing:

```bash
$ helm template task-api ./ai-agent-chart -f values-prod.yaml | head -50
---
# Source: ai-agent/templates/configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: task-api-config
  labels:
    helm.sh/chart: ai-agent-1.0.0
    app.kubernetes.io/name: ai-agent
    app.kubernetes.io/instance: task-api
    app.kubernetes.io/managed-by: Helm
data:
  LOG_LEVEL: "warn"
  WORKER_THREADS: "8"
---
# Source: ai-agent/templates/secret.yaml
apiVersion: v1
kind: Secret
metadata:
  name: task-api-secret
  labels:
    helm.sh/chart: ai-agent-1.0.0
```

**Output:** All YAML renders correctly with production values substituted.

### Check 3: Schema Validation

Validate values against schema:

```bash
$ helm template task-api ./ai-agent-chart -f values-invalid.yaml 2>&1 | grep -i error
error: values don't meet the schema requirements
```

**Output:** Invalid configurations are caught (if values-invalid.yaml had replicaCount: 50, this would fail).

### Check 4: Acceptance Criteria Verification

#### Criterion 1: Single `helm install` Deploys Complete Stack

```bash
$ helm install task-api ./ai-agent-chart -f values-prod.yaml --namespace test --create-namespace
NAME: task-api
LAST DEPLOYED: Thu Jan 23 14:35:00 2025
NAMESPACE: test
STATUS: deployed

$ kubectl get all -n test -l app.kubernetes.io/instance=task-api
NAME                           READY   STATUS    RESTARTS   AGE
pod/task-api-deployment-abc    1/1     Running   0          3s
pod/task-api-postgresql-0      1/1     Running   0          3s
pod/task-api-redis-0           1/1     Running   0          3s
pod/task-api-pre-upgrade-xyz   0/1     Completed 0          2s

NAME                  TYPE        CLUSTER-IP     EXTERNAL-IP   PORT(S)
service/task-api      ClusterIP   10.96.10.10    <none>        8000/TCP

NAME                           READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/task-api       1/1     1            1           3s

NAME                         READY   AGE
statefulset.apps/postgresql  1/1     3s
statefulset.apps/redis       1/1     3s
```

**Output:** ✓ Single `helm install` deployed all components:
- Agent Deployment running
- PostgreSQL StatefulSet running
- Redis StatefulSet running
- Pre-upgrade migration Job completed

#### Criterion 2: `helm test` Verifies Connectivity

```bash
$ helm test task-api -n test
NAME: task-api
LAST DEPLOYED: Thu Jan 23 14:35:00 2025
NAMESPACE: test
STATUS: deployed

TEST SUITE:     task-api-connection-test
Last Started:   Thu Jan 23 14:35:15 2025
Last Completed: Thu Jan 23 14:35:20 2025
Status:         PASSED

Details:
Test task-api-connection-test: PASSED
```

**Output:** ✓ Connectivity test passed:
- Agent can reach PostgreSQL
- Agent can reach Redis
- Both dependencies report healthy

Check test pod logs:

```bash
$ kubectl logs -n test task-api-connection-test
Testing PostgreSQL connection...
PostgreSQL connection successful!
Testing Redis connection...
Redis connection successful!
All connectivity tests passed!
```

**Output:** Both database and cache connectivity verified.

#### Criterion 3: Multi-Environment Deployment Works

Deploy dev:

```bash
$ helm install agent-dev ./ai-agent-chart -f values-dev.yaml --namespace dev --create-namespace
Release agent-dev installed.

$ kubectl get deployment agent-dev -n dev -o wide
NAME        READY   UP-TO-DATE   AVAILABLE   CPU        MEMORY
agent-dev   1/1     1            1           100m       256Mi
```

**Output:** Dev deployment uses 100m CPU / 256Mi memory.

Deploy staging:

```bash
$ helm install agent-staging ./ai-agent-chart -f values-staging.yaml --namespace staging --create-namespace
Release agent-staging installed.

$ kubectl get deployment agent-staging -n staging -o wide
NAME             READY   UP-TO-DATE   AVAILABLE   CPU        MEMORY
agent-staging    2/2     2            2           250m       512Mi
```

**Output:** Staging deployment uses 250m CPU / 512Mi memory with 2 replicas.

Deploy production:

```bash
$ helm install agent-prod ./ai-agent-chart -f values-prod.yaml --namespace prod --create-namespace
Release agent-prod installed.

$ kubectl get deployment agent-prod -n prod -o wide
NAME        READY   UP-TO-DATE   AVAILABLE   CPU        MEMORY
agent-prod  3/3     3            3           500m       1Gi
```

**Output:** Production deployment uses 500m CPU / 1Gi memory with 3 replicas.

Verify each environment has appropriate settings:

```bash
$ helm get values agent-prod -n prod | grep -A5 "agent:"
agent:
  replicaCount: 3
  env:
    LOG_LEVEL: warn
    WORKER_THREADS: '8'

$ helm get values agent-dev -n dev | grep -A5 "agent:"
agent:
  replicaCount: 1
  env:
    LOG_LEVEL: debug
    WORKER_THREADS: '1'
```

**Output:** ✓ Multi-environment configuration verified

### Summary: All Acceptance Criteria Met

- ✓ Criterion 1: Single `helm install` deployed complete stack with all components
- ✓ Criterion 2: `helm test` verified connectivity to both PostgreSQL and Redis
- ✓ Criterion 3: Multi-environment deployment works with appropriate resource levels

**Your chart meets the specification.**

---

## Try With AI

Now that you've built a production chart from specification, you can refine it further with AI collaboration. Your specification and implementation give you the foundation to evaluate AI suggestions critically.

### Setup

You'll use Claude or your preferred AI assistant to review and enhance your chart. Keep your specification and current implementation accessible.

### Prompts

**Part 1: Specification Review**

Ask AI to validate your specification against production Helm best practices:

```
I've written a specification for a production Helm chart that deploys an AI agent with PostgreSQL and Redis dependencies. Here's the specification:

[Paste your complete specification from Part 1]

Does this specification:
1. Include all necessary success criteria for a production deployment?
2. Have any missing requirements for security or reliability?
3. Match standard Helm patterns for dependency management?
4. Account for potential failure modes?

What would you add or change?
```

**Part 2: Implementation Review**

Ask AI to evaluate your chart against the spec:

```
Here's my implementation of that specification:
- Chart.yaml: [paste contents]
- values.yaml: [paste contents]
- deployment.yaml: [paste contents]
- [paste other critical templates]

Does my implementation:
1. Satisfy all acceptance criteria from the specification?
2. Follow Helm best practices?
3. Handle secrets securely?
4. Support multi-environment configuration correctly?

Are there any security or reliability gaps?
```

**Part 3: Edge Case Testing**

Ask AI to identify test scenarios you might have missed:

```
I've validated my chart against these acceptance criteria:
1. Single helm install deploys complete stack
2. helm test verifies connectivity
3. Multi-environment deployment works

What edge cases should I test?
- Network failures between services?
- Database migration failures during upgrade?
- Secret rotation?
- Pod evictions?
- Resource exhaustion scenarios?

Which of these are most critical for a production chart?
```

**Part 4: Production Hardening**

Ask AI for suggestions to make your chart more production-ready:

```
My chart currently includes:
- 3 replicas in production
- Resource requests and limits
- Health checks
- Pre-upgrade migrations

To make this chart production-grade, what would you recommend for:
1. Horizontal pod autoscaling?
2. Pod disruption budgets?
3. NetworkPolicies?
4. Monitoring and observability hooks?
5. Backup and disaster recovery patterns?

Which are most important before going live?
```

### Expected Insights

Through this collaboration, AI will likely suggest:

- **Missing resource management**: Pod Disruption Budgets (PDB) to survive cluster maintenance
- **Advanced deployment strategies**: Blue-green deployments or canary releases
- **Observability patterns**: Prometheus metrics and Grafana dashboards
- **Security enhancements**: Network policies and RBAC roles
- **Operational runbooks**: Procedures for common incidents (migration failures, pod evictions)

### Evaluate and Iterate

For each suggestion:
1. **Ask yourself**: Does this align with my specification? Is it in-scope?
2. **Implement selectively**: Add suggestions that improve the chart's ability to meet acceptance criteria
3. **Document decisions**: Record which suggestions you adopted and which you deferred (and why)

What emerges is a chart that's not just specification-compliant, but hardened for real production use.
