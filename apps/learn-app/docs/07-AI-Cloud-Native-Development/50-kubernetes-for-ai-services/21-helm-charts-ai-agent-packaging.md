---
sidebar_position: 21
chapter: 51
lesson: 21
duration_minutes: 45
title: "Helm Charts for AI Agent Packaging"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Learn Helm chart structure and release management"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Install Helm and use it to deploy public charts"
    bloom_level: "Apply"
  - id: LO2
    description: "Create custom Helm charts for AI agent deployments"
    bloom_level: "Create"
  - id: LO3
    description: "Use values.yaml to parameterize deployments across environments"
    bloom_level: "Apply"
  - id: LO4
    description: "Manage releases with upgrade, rollback, and uninstall operations"
    bloom_level: "Apply"
---

# Helm Charts for AI Agent Packaging

When you deploy your agent to Kubernetes using kubectl, you apply individual YAML files—one for the Deployment, one for the Service, one for the ConfigMap, maybe one for the Secret. That's 5-10 files for a single agent.

Now imagine managing this across three environments: dev, staging, production. Each environment needs different resource limits, different image tags, different replicas. You copy and modify the same files ten times, creating drift and bugs.

**Helm solves this problem.** Helm is the package manager for Kubernetes. Instead of managing individual YAML files, you create a **Helm chart**—a templated package that parameterizes your deployment. Change one value file, and your entire agent deployment adapts automatically.

This lesson teaches you to understand why Helm exists, install it, deploy a public chart, create your own custom chart, and manage releases across environments.

---

## Why Helm Exists: The Repetitive YAML Problem

Before Helm, deploying an application to Kubernetes meant writing YAML files by hand:

```yaml
# deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-agent
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
      - name: agent
        image: myregistry/my-agent:v1.0.0
        ports:
        - containerPort: 8000
        env:
        - name: MODEL_NAME
          value: "gpt-4"
        - name: LOG_LEVEL
          value: "INFO"
        resources:
          requests:
            memory: "512Mi"
            cpu: "250m"
          limits:
            memory: "1Gi"
            cpu: "500m"
```

```yaml
# service.yaml
apiVersion: v1
kind: Service
metadata:
  name: my-agent-service
spec:
  type: ClusterIP
  ports:
  - port: 80
    targetPort: 8000
  selector:
    app: my-agent
```

```yaml
# configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: my-agent-config
data:
  model_name: "gpt-4"
  log_level: "INFO"
```

This works for one environment. But in production, you need:

- **Development**: 1 replica, 256Mi memory, gpt-3.5-turbo
- **Staging**: 2 replicas, 512Mi memory, gpt-4
- **Production**: 5 replicas, 1Gi memory, gpt-4-turbo

Now you maintain three copies of the same files, edited manually. When you update the base deployment, you must remember to update all three copies. This is error-prone and doesn't scale.

**Helm templating solves this.** Instead of three copies, you write once with **placeholders**:

```yaml
replicas: {{ .Values.replicaCount }}
image: myregistry/my-agent:{{ .Values.image.tag }}
```

Then provide three values files—one for each environment:

```yaml
# values-dev.yaml
replicaCount: 1
image:
  tag: v1.0.0-dev
```

```yaml
# values-prod.yaml
replicaCount: 5
image:
  tag: v1.0.0
```

A single `helm install my-agent ./my-agent-chart -f values-prod.yaml` command deploys the entire stack to production with the correct configuration.

---

## Installing Helm

Helm is a CLI tool you install on your machine. It then communicates with your Kubernetes cluster to deploy charts.

::::os-tabs

::macos
```bash
brew install helm
```

**Output:**
```
==> Downloading https://ghcr.io/v2/homebrew/core/helm/manifests/3.14.0
==> Downloading https://ghcr.io/v2/homebrew/core/helm/manifests/3.14.0
==> Downloading https://ghcr.io/v2/homebrew/core/helm/blobs/sha256:abc...
==> Pouring helm--3.14.0.arm64_sonoma.bottle.tar.gz
==> Caveats
Bash completion has been installed to:
  /usr/local/etc/bash_completion.d
Zsh completion has been installed to:
  /opt/homebrew/share/zsh/site-functions
==> Summary
🍺  /opt/homebrew/bin/helm (3.14.0)
```

::linux
```bash
curl https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash
```

**Output:**
```
Downloading https://get.helm.sh/helm-v3.14.0-linux-amd64.tar.gz
Verifying checksum... Done.
Preparing to install helm into /usr/local/bin
helm installed into /usr/local/bin/helm
```

::windows
```powershell
choco install kubernetes-helm
```

Or using Scoop:
```powershell
scoop install helm
```

::::

### Verify Installation

```bash
helm version
```

**Output:**
```
version.BuildInfo{Version:"v3.14.0", GitCommit:"3fc9f4b2638e76077605842cc9038889c437f325", GitTreeState:"clean", GoVersion:"go1.22.0"}
```

Helm is now installed and ready to deploy charts.

---

## Understanding Helm Charts: The Package Structure

A Helm chart is a directory with a specific structure:

```
my-agent-chart/
├── Chart.yaml              # Chart metadata (name, version, description)
├── values.yaml             # Default configuration values
├── values-dev.yaml         # Development-specific overrides
├── values-prod.yaml        # Production-specific overrides
├── templates/              # Kubernetes manifests (Go templates)
│   ├── deployment.yaml     # Pod deployment template
│   ├── service.yaml        # Service template
│   ├── configmap.yaml      # ConfigMap template
│   └── helpers.tpl         # Template helper functions
└── README.md               # Chart documentation
```

Each file serves a purpose:

- **Chart.yaml**: Declares the chart's name, version, description. Think of it like package.json for Kubernetes.
- **values.yaml**: Default values for all placeholders. These are overridable per environment.
- **templates/**: Kubernetes YAML files with Go template syntax (`{{ .Values.replicas }}`).
- **values-dev.yaml, values-prod.yaml**: Environment-specific value overrides.

When you run `helm install`, Helm:
1. Reads Chart.yaml to understand the chart
2. Loads values.yaml (default values)
3. Merges in environment-specific values (e.g., values-prod.yaml)
4. Processes templates/ files, substituting values into placeholders
5. Applies the rendered YAML to Kubernetes

---

## Deploying a Public Chart: Bitnami Redis

Before creating your own chart, let's deploy a public chart to understand how Helm works.

The **Bitnami Helm repository** provides production-ready charts for common applications. Let's add it and install Redis:

### Step 1: Add the Bitnami Repository

```bash
helm repo add bitnami https://charts.bitnami.com/bitnami
```

**Output:**
```
"bitnami" has been added to your repositories
```

### Step 2: Update Repository Index

```bash
helm repo update
```

**Output:**
```
Hang tight while we grab the latest from your chart repositories...
...Successfully got an update from the "bitnami" chart repository
Update Complete. ⎉ Happy Helming!
```

### Step 3: Search for Redis Chart

```bash
helm search repo bitnami/redis
```

**Output:**
```
NAME                 	CHART VERSION	APP VERSION	DESCRIPTION
bitnami/redis        	18.6.0       	7.2.4      	Redis(TM) is an open source, advanced key-value ...
```

### Step 4: Install the Chart

```bash
helm install my-redis bitnami/redis \
  --set auth.enabled=false \
  --set replica.replicaCount=1 \
  --namespace default
```

**Output:**
```
NAME: my-redis
LAST DEPLOYED: Mon Dec 23 10:45:32 2024
NAMESPACE: default
STATUS: deployed
REVISION: 1
TEST SUITE: None
NOTES:
CHART NAME: redis
CHART VERSION: 18.6.0
APP VERSION: 7.2.4
** Please be patient while the chart is being deployed **

Redis&trade; can be accessed via port 6379 on the following DNS name from within your cluster:

    my-redis-master.default.svc.cluster.local

To get your Redis&trade; connection string:

    export REDIS_URL="redis://my-redis-master.default.svc.cluster.local:6379"

To connect to your Redis&trade; instance, run the following commands:

    kubectl run --namespace default redis-cli --rm --tty -i --restart='Never' -- redis-cli -h my-redis-master.default.svc.cluster.local -p 6379
```

### Step 5: Verify the Deployment

```bash
kubectl get pods -n default
```

**Output:**
```
NAME                READY   STATUS    RESTARTS   AGE
my-redis-master-0   1/1     Running   0          45s
```

```bash
helm list
```

**Output:**
```
NAME     	NAMESPACE	REVISION	UPDATED                 	STATUS  	CHART           	APP VERSION
my-redis 	default  	1        	2024-12-23 10:45:32 +0000 UTC	deployed	redis-18.6.0	7.2.4
```

You deployed a production-grade Redis instance with a single `helm install` command. Helm templated all the Redis manifests, applied them to Kubernetes, and created a **release** called `my-redis` that you can upgrade, downgrade, or rollback.

---

## Creating Your Own Helm Chart

Now you'll create a custom chart for your AI agent. Instead of managing raw YAML files, you'll use Helm's templating to create a reusable package.

### Step 1: Generate Chart Scaffold

Helm provides a generator to create the basic chart structure:

```bash
helm create my-agent-chart
```

**Output:**
```
Creating my-agent-chart
```

```bash
tree my-agent-chart
```

**Output:**
```
my-agent-chart/
├── Chart.yaml
├── charts
├── templates
│   ├── NOTES.txt
│   ├── _helpers.tpl
│   ├── deployment.yaml
│   ├── hpa.yaml
│   ├── ingress.yaml
│   ├── service.yaml
│   ├── serviceaccount.yaml
│   └── tests
│       └── test-connection.yaml
└── values.yaml
```

### Step 2: Examine Chart.yaml

```bash
cat my-agent-chart/Chart.yaml
```

**Output:**
```yaml
apiVersion: v2
name: my-agent-chart
description: A Helm chart for Kubernetes
type: application
version: 0.1.0
appVersion: "1.16"
```

This declares the chart metadata. Update it for your agent:

```yaml
apiVersion: v2
name: my-agent
description: "Helm chart for deploying AI agents on Kubernetes"
type: application
version: 0.1.0
appVersion: "1.0.0"
keywords:
  - ai
  - agent
  - kubernetes
maintainers:
  - name: "Your Name"
    email: "your.email@example.com"
```

### Step 3: Examine values.yaml

```bash
cat my-agent-chart/values.yaml
```

**Output:**
```yaml
replicaCount: 1

image:
  repository: nginx
  pullPolicy: IfNotPresent
  tag: ""

imagePullSecrets: []
nameOverride: ""
fullnameOverride: ""

serviceAccount:
  create: true
  annotations: {}
  name: ""

podAnnotations: {}

podSecurityContext: {}

securityContext: {}

service:
  type: ClusterIP
  port: 80

ingress:
  enabled: false
  className: ""
  annotations: {}
  hosts:
    - host: chart-example.local
      paths:
        - path: /
          pathType: ImplementationSpecific
  tls: []

resources:
  limits:
    cpu: 100m
    memory: 128Mi
  requests:
    cpu: 100m
    memory: 128Mi

autoscaling:
  enabled: false
  minReplicas: 1
  maxReplicas: 100
  targetCPUUtilizationPercentage: 80

nodeSelector: {}

tolerations: []

affinity: {}
```

This is the template of all values. Each value is referenced by name in templates (e.g., `{{ .Values.replicaCount }}`). Update values.yaml for your AI agent:

```yaml
replicaCount: 3

image:
  repository: myregistry.azurecr.io/my-agent
  pullPolicy: Always
  tag: "v1.0.0"

imagePullSecrets:
  - name: acr-secret

service:
  type: ClusterIP
  port: 8000
  targetPort: 8000

resources:
  limits:
    cpu: 500m
    memory: 1Gi
  requests:
    cpu: 250m
    memory: 512Mi

env:
  - name: MODEL_NAME
    value: "gpt-4"
  - name: LOG_LEVEL
    value: "INFO"
  - name: API_TIMEOUT
    value: "30"
```

### Step 4: Examine the Deployment Template

```bash
cat my-agent-chart/templates/deployment.yaml
```

**Output:**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ include "my-agent-chart.fullname" . }}
  labels:
    {{- include "my-agent-chart.labels" . | nindent 4 }}
spec:
  {{- if not .Values.autoscaling.enabled }}
  replicas: {{ .Values.replicaCount }}
  {{- end }}
  selector:
    matchLabels:
      {{- include "my-agent-chart.selectorLabels" . | nindent 6 }}
  template:
    metadata:
      {{- with .Values.podAnnotations }}
      annotations:
        {{- toYaml . | nindent 8 }}
      {{- end }}
      labels:
        {{- include "my-agent-chart.selectorLabels" . | nindent 8 }}
    spec:
      {{- with .Values.imagePullSecrets }}
      imagePullSecrets:
        {{- toYaml . | nindent 8 }}
      {{- end }}
      serviceAccountName: {{ include "my-agent-chart.serviceAccountName" . }}
      securityContext:
        {{- toYaml .Values.podSecurityContext | nindent 8 }}
      containers:
      - name: {{ .Chart.Name }}
        securityContext:
          {{- toYaml .Values.securityContext | nindent 12 }}
        image: "{{ .Values.image.repository }}:{{ .Values.image.tag | default .Chart.AppVersion }}"
        imagePullPolicy: {{ .Values.image.pullPolicy }}
        ports:
        - name: http
          containerPort: 8000
          protocol: TCP
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
          initialDelaySeconds: 10
          periodSeconds: 5
        resources:
          {{- toYaml .Values.resources | nindent 12 }}
        env:
        {{- if .Values.env }}
        {{- toYaml .Values.env | nindent 10 }}
        {{- end }}
      {{- with .Values.nodeSelector }}
      nodeSelector:
        {{- toYaml . | nindent 8 }}
      {{- end }}
      {{- with .Values.affinity }}
      affinity:
        {{- toYaml . | nindent 8 }}
      {{- end }}
      {{- with .Values.tolerations }}
      tolerations:
        {{- toYaml . | nindent 8 }}
      {{- end }}
```

This template uses Go template syntax:

- `{{ .Values.replicaCount }}` — Substitutes the value of `replicaCount` from values.yaml
- `{{ include "my-agent-chart.fullname" . }}` — Calls a helper function (defined in _helpers.tpl)
- `{{- if .Values.autoscaling.enabled }}` — Conditional: include this section only if autoscaling is enabled
- `{{ toYaml . | nindent 8 }}` — Converts a value to YAML and indents it 8 spaces

### Step 5: Test Template Rendering

Before deploying, preview what Helm will generate:

```bash
helm template my-release my-agent-chart
```

**Output:**
```yaml
---
# Source: my-agent-chart/templates/serviceaccount.yaml
apiVersion: v1
kind: ServiceAccount
metadata:
  name: my-release-my-agent-chart
  labels:
    helm.sh/chart: my-agent-chart-0.1.0
    app.kubernetes.io/name: my-agent-chart
    app.kubernetes.io/instance: my-release
    app.kubernetes.io/version: "1.0.0"
    app.kubernetes.io/managed-by: Helm
---
# Source: my-agent-chart/templates/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: my-release-my-agent-chart
  labels:
    helm.sh/chart: my-agent-chart-0.1.0
    app.kubernetes.io/name: my-agent-chart
    app.kubernetes.io/instance: my-release
    app.kubernetes.io/version: "1.0.0"
    app.kubernetes.io/managed-by: Helm
spec:
  type: ClusterIP
  ports:
    - port: 8000
      targetPort: http
      protocol: TCP
      name: http
  selector:
    app.kubernetes.io/name: my-agent-chart
    app.kubernetes.io/instance: my-release
---
# Source: my-agent-chart/templates/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-release-my-agent-chart
  labels:
    helm.sh/chart: my-agent-chart-0.1.0
    app.kubernetes.io/name: my-agent-chart
    app.kubernetes.io/instance: my-release
    app.kubernetes.io/version: "1.0.0"
    app.kubernetes.io/managed-by: Helm
spec:
  replicas: 3
  selector:
    matchLabels:
      app.kubernetes.io/name: my-agent-chart
      app.kubernetes.io/instance: my-release
  template:
    metadata:
      labels:
        helm.sh/chart: my-agent-chart-0.1.0
        app.kubernetes.io/name: my-agent-chart
        app.kubernetes.io/instance: my-release
        app.kubernetes.io/version: "1.0.0"
        app.kubernetes.io/managed-by: Helm
    spec:
      serviceAccountName: my-release-my-agent-chart
      containers:
      - name: my-agent-chart
        image: "myregistry.azurecr.io/my-agent:v1.0.0"
        imagePullPolicy: Always
        ports:
        - name: http
          containerPort: 8000
          protocol: TCP
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
          initialDelaySeconds: 10
          periodSeconds: 5
        resources:
          limits:
            cpu: 500m
            memory: 1Gi
          requests:
            cpu: 250m
            memory: 512Mi
        env:
        - name: MODEL_NAME
          value: "gpt-4"
        - name: LOG_LEVEL
          value: "INFO"
        - name: API_TIMEOUT
          value: "30"
```

Helm rendered the templates with values from values.yaml. The output is valid Kubernetes YAML.

---

## Environment-Specific Values: Dev vs Production

Now create environment-specific value files to customize deployments:

### values-dev.yaml

```yaml
replicaCount: 1

image:
  tag: "v1.0.0-dev"

resources:
  limits:
    cpu: 250m
    memory: 512Mi
  requests:
    cpu: 100m
    memory: 256Mi

env:
  - name: MODEL_NAME
    value: "gpt-3.5-turbo"
  - name: LOG_LEVEL
    value: "DEBUG"
  - name: API_TIMEOUT
    value: "60"
```

### values-prod.yaml

```yaml
replicaCount: 5

image:
  tag: "v1.0.0"

resources:
  limits:
    cpu: 1000m
    memory: 2Gi
  requests:
    cpu: 500m
    memory: 1Gi

env:
  - name: MODEL_NAME
    value: "gpt-4-turbo"
  - name: LOG_LEVEL
    value: "WARN"
  - name: API_TIMEOUT
    value: "30"

autoscaling:
  enabled: true
  minReplicas: 3
  maxReplicas: 10
  targetCPUUtilizationPercentage: 70
```

### Deploy to Development

```bash
helm install my-agent my-agent-chart -f my-agent-chart/values-dev.yaml
```

**Output:**
```
NAME: my-agent
LAST DEPLOYED: Mon Dec 23 11:05:22 2024
NAMESPACE: default
STATUS: deployed
REVISION: 1
```

Verify the deployment:

```bash
kubectl get deployment my-agent-my-agent-chart
```

**Output:**
```
NAME                      READY   UP-TO-DATE   AVAILABLE   AGE
my-agent-my-agent-chart   1/1     1            1           15s
```

Notice: 1 replica in dev (from values-dev.yaml).

### Deploy to Production

```bash
helm install my-agent-prod my-agent-chart -f my-agent-chart/values-prod.yaml
```

**Output:**
```
NAME: my-agent-prod
LAST DEPLOYED: Mon Dec 23 11:06:45 2024
NAMESPACE: default
STATUS: deployed
REVISION: 1
```

Verify:

```bash
kubectl get deployment my-agent-prod-my-agent-chart
```

**Output:**
```
NAME                          READY   UP-TO-DATE   AVAILABLE   AGE
my-agent-prod-my-agent-chart  5/5     5            5           18s
```

Notice: 5 replicas in production (from values-prod.yaml).

---

## Release Management: Upgrade, Rollback, Uninstall

A **release** is an instance of a chart deployed to your cluster. You manage releases with four operations: install, upgrade, rollback, and uninstall.

### Install (Create a Release)

```bash
helm install my-agent my-agent-chart -f my-agent-chart/values-prod.yaml
```

**Output:**
```
NAME: my-agent
LAST DEPLOYED: Mon Dec 23 11:15:22 2024
NAMESPACE: default
STATUS: deployed
REVISION: 1
```

### Upgrade (Update a Release)

You've pushed a new version of your agent image. Update the release to use it:

```bash
helm upgrade my-agent my-agent-chart \
  -f my-agent-chart/values-prod.yaml \
  --set image.tag="v1.1.0"
```

**Output:**
```
Release "my-agent" has been upgraded successfully
NAME: my-agent
LAST DEPLOYED: Mon Dec 23 11:16:33 2024
NAMESPACE: default
STATUS: deployed
REVISION: 2
```

Notice the revision changed from 1 to 2. Helm tracks every release change.

Verify the rollout:

```bash
kubectl rollout status deployment/my-agent-my-agent-chart
```

**Output:**
```
deployment "my-agent-my-agent-chart" successfully rolled out
```

### Check Release History

```bash
helm history my-agent
```

**Output:**
```
REVISION	UPDATED                 	STATUS    	CHART                  	APP VERSION	DESCRIPTION
1        	Mon Dec 23 11:15:22 2024	superseded	my-agent-chart-0.1.0	1.0.0      	Install complete
2        	Mon Dec 23 11:16:33 2024	deployed  	my-agent-chart-0.1.0	1.0.0      	Upgrade complete
```

### Rollback (Revert to a Previous Release)

The new version has a bug. Rollback to revision 1:

```bash
helm rollback my-agent 1
```

**Output:**
```
Rollback was a success
Release "my-agent" has been rolled back to revision 1
```

Verify:

```bash
helm history my-agent
```

**Output:**
```
REVISION	UPDATED                 	STATUS    	CHART                  	APP VERSION	DESCRIPTION
1        	Mon Dec 23 11:15:22 2024	superseded	my-agent-chart-0.1.0	1.0.0      	Install complete
2        	Mon Dec 23 11:16:33 2024	superseded	my-agent-chart-0.1.0	1.0.0      	Upgrade complete
3        	Mon Dec 23 11:17:45 2024	deployed  	my-agent-chart-0.1.0	1.0.0      	Rollback to 1
```

A new revision (3) was created that rolls back to revision 1's configuration. The agent is now running the old image again.

### Uninstall (Delete a Release)

```bash
helm uninstall my-agent
```

**Output:**
```
release "my-agent" uninstalled
```

All Kubernetes resources created by this release are deleted:

```bash
kubectl get deployment
```

**Output:**
```
No resources found in default namespace.
```

---

## What You've Learned

You now understand:

- **Why Helm exists**: It solves the repetitive YAML problem by templating deployments
- **How to install Helm**: On macOS, Linux, and verify installation
- **How to deploy public charts**: Using helm repo add, helm search, and helm install
- **Chart structure**: Chart.yaml, values.yaml, templates/, and helper functions
- **Go template syntax**: Substitution (`{{ .Values.replicaCount }}`), helpers, conditionals
- **Environment-specific values**: Creating separate values files for dev/prod deployments
- **Release management**: Install, upgrade, rollback, and uninstall operations

With these skills, you can package any Kubernetes application—including your AI agent—into a reusable, versioned Helm chart that deploys consistently across environments.

---

## Try With AI

Now you'll work with AI to create a multi-environment Helm configuration for your agent.

**Setup**: You have your AI agent container image pushed to a registry. You'll parameterize a Helm chart to support three environments (dev, staging, production) with different replicas, resources, and model configurations.

**Prompts**:

1. **Chart Creation with AI**:
   ```
   I have an AI agent image at myregistry.azurecr.io/my-agent:latest.
   Create a Helm chart called 'ai-agent' with:
   - Deployment for the agent (port 8000)
   - Service exposing the agent (ClusterIP)
   - ConfigMap for model configuration (MODEL_NAME, LOG_LEVEL)
   - Default values: 3 replicas, 500m CPU limit, 1Gi memory limit

   What does the Chart.yaml and values.yaml look like?
   ```

2. **Environment Values with AI**:
   ```
   Now create three values files for this chart:

   values-dev.yaml: 1 replica, 250m CPU, 512Mi memory, gpt-3.5-turbo
   values-staging.yaml: 2 replicas, 500m CPU, 1Gi memory, gpt-4
   values-prod.yaml: 5 replicas, 1Gi CPU, 2Gi memory, gpt-4-turbo

   Should I use separate files or one values file with conditionals?
   ```

3. **Refinement Based on Response**:
   Ask the AI to clarify:
   - How would you handle secret injection for API keys without storing them in values files?
   - Can I use Helm hooks to run database migrations before the agent starts?

**Expected**: You'll discover that environment-specific values files are cleaner than conditionals, and learn about Helm's secret management and hooks—features that deepen your chart's capabilities.

