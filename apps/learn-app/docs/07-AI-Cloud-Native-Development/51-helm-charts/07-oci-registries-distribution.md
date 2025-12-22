---
sidebar_position: 7
chapter: 51
lesson: 7
duration_minutes: 45
title: "OCI Registries and Distribution"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Publish and consume charts via OCI-compliant registries"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Package charts with helm package command"
    bloom_level: "Apply"
  - id: LO2
    description: "Push charts to OCI registries using helm push"
    bloom_level: "Apply"
  - id: LO3
    description: "Pull and install charts from OCI URLs"
    bloom_level: "Apply"
  - id: LO4
    description: "Configure Chart.yaml dependencies with OCI URLs"
    bloom_level: "Apply"
  - id: LO5
    description: "Implement chart versioning strategy aligned with semver"
    bloom_level: "Analyze"
---

# OCI Registries and Distribution

In Lessons 1-6, you built Helm charts locally: templating logic, dependency composition, hooks, and validation. These charts work perfectly in your development environment. But when you need to share charts across teams, use them in production, or compose them as dependencies in other charts, storing them as directories on your laptop doesn't scale.

Helm solved this by adopting OCI (Open Container Initiative) standards. The same registry that stores your Docker images—Docker Hub, Amazon ECR, Azure Container Registry—now stores your Helm charts. This unifies distribution: one registry for both container images AND charts, one authentication mechanism, one versioning strategy.

This lesson teaches you how to package charts into distributable artifacts, authenticate with registries, push and pull charts, consume charts from OCI URLs, and implement a chart versioning strategy aligned with semantic versioning. By the end, you'll publish your AI agent's Helm chart to a shared registry where teammates and production systems can discover and install it.

---

## Concept 1: OCI Basics — Why Helm Adopted OCI for Charts

Before you can push a chart to a registry, understand what OCI is and why Helm moved from Helm repositories (custom format) to OCI.

### The Problem: Two Distribution Systems

**Before OCI adoption (Helm 2-early Helm 3):**
- Container images → Docker registries (docker.io, gcr.io, etc.)
- Helm charts → Helm repositories (custom format, separate infrastructure)

Teams maintained two completely different distribution systems with different credentials, different APIs, different versioning semantics.

**After OCI adoption (Helm 3.7+):**
- Container images → OCI registries (same)
- Helm charts → OCI registries (same)

One registry. One authentication model. One API.

### What is OCI?

OCI (Open Container Initiative) is a standardized format for packaging and distributing artifacts. Originally designed for container images, OCI is now a general-purpose artifact format that can store anything: images, charts, software bills of materials (SBOMs), vulnerability scan results.

An OCI registry is any system that follows the OCI Distribution Spec:
- Docker Hub
- Amazon ECR (Elastic Container Registry)
- Azure Container Registry
- Google Artifact Registry
- Private registries (Harbor, Artifactory)
- Local registries (for development)

A Helm chart stored in OCI format is just an artifact in these registries, no different from a Docker image.

### How Helm Charts Fit into OCI

When you `helm package` a chart, Helm:

1. Creates a `.tgz` archive of your chart directory
2. Wraps it in OCI metadata (content type: `application/vnd.cncf.helm.chart.v1.tar+gzip`)
3. Stores it in the registry with a tag (like `myapp-chart:1.2.3`)

A Helm chart in a registry looks like this:

```
docker.io/myusername/myapp-chart:1.2.3
```

It's identical in structure to a Docker image reference:

```
docker.io/myusername/myapp-image:1.2.3
```

Same registry. Same tagging system. Same authentication.

---

## Concept 2: Packaging Charts — helm package

Before pushing to a registry, you must package your chart into a `.tgz` archive.

### Creating a Chart Archive

Start with an existing chart (or the example from Lesson 1):

```bash
# Navigate to your chart directory
cd path/to/myapp-chart

# View the chart structure
ls -la
# Output:
# Chart.yaml
# values.yaml
# templates/
# values-dev.yaml
# values-prod.yaml

# Package the chart
helm package .

# Output:
# Successfully packaged chart and saved it to: /Users/you/work/myapp-chart-1.2.3.tgz
```

**Output:**
```bash
$ cd myapp-chart && helm package .
Successfully packaged chart and saved it to: /Users/username/myapp-chart-1.2.3.tgz

$ ls -lh myapp-chart-1.2.3.tgz
-rw-r--r--  1 username  staff   4.2K Nov 27 10:30 myapp-chart-1.2.3.tgz
```

### What `helm package` Does

1. Validates the chart (runs `helm lint` internally)
2. Creates a `.tgz` archive containing:
   - Chart.yaml
   - values.yaml
   - templates/
   - All other chart files
3. Names the file: `{chart-name}-{version}.tgz`
4. Stores it in the current directory (unless `-d` specifies destination)

### Version Matters

The filename comes from `Chart.yaml`:

```yaml
---
apiVersion: v2
name: myapp-chart
version: 1.2.3
---
```

If you change version in Chart.yaml, `helm package` creates a new file:

```bash
# After changing Chart.yaml version to 1.3.0
helm package .
# Output: Successfully packaged chart and saved it to: myapp-chart-1.3.0.tgz
```

Each version is a separate artifact. This is how semantic versioning works: `1.2.3` represents major.minor.patch. When you fix a bug, bump patch (1.2.4). When you add backwards-compatible features, bump minor (1.3.0). When you make breaking changes, bump major (2.0.0).

---

## Concept 3: OCI URL Format — How to Reference Charts

When you push a chart to an OCI registry, you reference it using `oci://` URLs.

### The OCI URL Pattern

```
oci://registry-host/repository/chart-name:version
```

Breaking this down:

| Component | Example | Meaning |
|-----------|---------|---------|
| `oci://` | Protocol | Tells Helm this is an OCI artifact (not HTTP) |
| `registry-host` | `docker.io` | Registry domain |
| `repository` | `myusername/charts` | Path in registry (like Docker image paths) |
| `chart-name` | `myapp-chart` | Name of the Helm chart |
| `version` | `1.2.3` | Semantic version tag |

### Examples

**Public chart on Docker Hub:**
```
oci://docker.io/myusername/agent-chart:1.0.0
```

**Private chart on Amazon ECR:**
```
oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts/ml-pipeline:2.1.5
```

**Local registry for development:**
```
oci://localhost:5000/my-charts/scheduler:0.1.0
```

### Compare to HTTP URLs

Before OCI, Helm used HTTP repository URLs (still supported):

```
# OLD way (Helm repository)
https://charts.example.com
helm repo add myrepo https://charts.example.com
helm install myrelease myrepo/myapp-chart --version 1.2.3

# NEW way (OCI)
oci://docker.io/myusername/myapp-chart:1.2.3
helm install myrelease oci://docker.io/myusername/myapp-chart --version 1.2.3
```

The OCI approach is simpler: no separate repository infrastructure, same registry as your images.

---

## Concept 4: Authenticating with Registries — helm registry login

Before you push or pull charts from a private registry, authenticate using `helm registry login`.

### Logging In to Docker Hub

```bash
# Authenticate with Docker Hub
helm registry login docker.io

# Prompt appears:
# Username: your-docker-username
# Password: your-docker-token

# Successfully logged in to docker.io
```

**Output:**
```bash
$ helm registry login docker.io
Username: myusername
Password: ****
Login Succeeded

$ cat ~/.docker/config.json | grep -A5 "docker.io"
  "auths": {
    "docker.io": {
      "auth": "bXl1c2VybmFtZTpteS10b2tlbg=="
    }
  }
```

### Using Docker Credentials

If you've already logged in with the Docker CLI:

```bash
# You've already run: docker login -u myusername
# Helm reads the same credentials

helm push myapp-chart-1.2.3.tgz oci://docker.io/myusername/
# Uses your existing Docker credentials automatically
```

### Logging In to Private Registries

For Amazon ECR, Azure ACR, or other private registries:

```bash
# Amazon ECR
aws ecr get-login-password --region us-east-1 | helm registry login --username AWS --password-stdin 123456789.dkr.ecr.us-east-1.amazonaws.com

# Azure Container Registry
az acr login --name myregistry
helm registry login myregistry.azurecr.io

# Private registry (Artifactory, Harbor, etc.)
helm registry login artifactory.company.com
```

**Output:**
```bash
$ helm registry login 123456789.dkr.ecr.us-east-1.amazonaws.com
Username: AWS
Password: paste-ECR-token-here
Login Succeeded
```

### Credentials Storage

Helm stores credentials in `~/.docker/config.json` (same as Docker CLI). Each registry gets its own credential entry.

---

## Concept 5: Pushing Charts — helm push

Once authenticated, push your packaged chart to an OCI registry using `helm push`.

### Pushing to Docker Hub

```bash
# You've already packaged and authenticated
# Chart file: myapp-chart-1.2.3.tgz
# Authenticated with: helm registry login docker.io

# Push to your Docker Hub repository
helm push myapp-chart-1.2.3.tgz oci://docker.io/myusername/

# Output: Pushed: docker.io/myusername/myapp-chart:1.2.3
```

**Output:**
```bash
$ helm push myapp-chart-1.2.3.tgz oci://docker.io/myusername/
Pushed: docker.io/myusername/myapp-chart:1.2.3

$ # Verify it's in the registry (via Docker Hub web UI or CLI)
$ docker pull docker.io/myusername/myapp-chart:1.2.3
Error: manifest not found
# (Docker image protocol doesn't work; OCI charts aren't container images)

$ # But Helm can pull it
$ helm pull oci://docker.io/myusername/myapp-chart:1.2.3
Downloaded to myapp-chart-1.2.3.tgz
```

### Pushing to Private Registries

```bash
# Push to Amazon ECR
helm push agent-chart-2.0.1.tgz oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts/

# Output: Pushed: 123456789.dkr.ecr.us-east-1.amazonaws.com/charts/agent-chart:2.0.1

# Push to Azure ACR
helm push scheduler-chart-1.5.0.tgz oci://myregistry.azurecr.io/

# Output: Pushed: myregistry.azurecr.io/scheduler-chart:1.5.0
```

**Output:**
```bash
$ helm push agent-chart-2.0.1.tgz oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts/
Pushed: 123456789.dkr.ecr.us-east-1.amazonaws.com/charts/agent-chart:2.0.1

# Verify with AWS CLI
$ aws ecr describe-images --registry-id 123456789 --repository-name charts
{
  "imageDetails": [
    {
      "repositoryName": "charts",
      "imageTags": ["agent-chart:2.0.1"],
      "imageSizeInBytes": 4500,
      "pushedAt": 2025-11-27T10:30:00.000Z
    }
  ]
}
```

### Chart Naming Convention

When you push, Helm separates chart name from version:

```
helm push myapp-chart-1.2.3.tgz oci://docker.io/myusername/
# Pushes to: docker.io/myusername/myapp-chart:1.2.3

# NOT:
# docker.io/myusername/myapp-chart-1.2.3:latest
```

The version goes into the tag (after the colon), not the image name. This matches Docker convention:
- `docker.io/myusername/myapp-image:1.2.3` (Docker image)
- `docker.io/myusername/myapp-chart:1.2.3` (Helm chart)

---

## Concept 6: Pulling Charts — helm pull

Download charts from OCI registries using `helm pull`.

### Pulling from Docker Hub

```bash
# Pull a specific version
helm pull oci://docker.io/myusername/myapp-chart:1.2.3

# Output: Downloaded to myapp-chart-1.2.3.tgz

# List contents
tar -tzf myapp-chart-1.2.3.tgz | head
# Chart.yaml
# values.yaml
# templates/deployment.yaml
# templates/service.yaml
```

**Output:**
```bash
$ helm pull oci://docker.io/myusername/myapp-chart:1.2.3
Downloaded to myapp-chart-1.2.3.tgz

$ ls -lh myapp-chart-1.2.3.tgz
-rw-r--r--  1 username  staff   4.2K Nov 27 10:35 myapp-chart-1.2.3.tgz

$ tar -tzf myapp-chart-1.2.3.tgz | head -10
myapp-chart/Chart.yaml
myapp-chart/values.yaml
myapp-chart/templates/
myapp-chart/templates/deployment.yaml
myapp-chart/templates/service.yaml
myapp-chart/templates/_helpers.tpl
```

### Pulling from Private Registries

```bash
# Pull from ECR
helm pull oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts/agent-chart:2.0.1

# Pull from ACR (must be authenticated first)
helm registry login myregistry.azurecr.io
helm pull oci://myregistry.azurecr.io/scheduler-chart:1.5.0

# Output: Downloaded to agent-chart-2.0.1.tgz
```

**Output:**
```bash
$ helm pull oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts/agent-chart:2.0.1
Downloaded to agent-chart-2.0.1.tgz

$ tar -tzf agent-chart-2.0.1.tgz | grep -E "Chart|values" | head
agent-chart/Chart.yaml
agent-chart/values.yaml
```

### Extracting and Inspecting

```bash
# Extract the chart
tar -xzf myapp-chart-1.2.3.tgz

# Inspect values
cat myapp-chart/values.yaml

# Inspect templates
ls myapp-chart/templates/
```

**Output:**
```bash
$ tar -xzf myapp-chart-1.2.3.tgz

$ cat myapp-chart/values.yaml
replicaCount: 1
image:
  repository: myusername/myapp
  tag: "1.0.0"
  pullPolicy: IfNotPresent

$ ls -la myapp-chart/templates/
total 24
-rw-r--r--  6 username  staff  284 Nov 27 deployment.yaml
-rw-r--r--  6 username  staff  198 Nov 27 service.yaml
-rw-r--r--  6 username  staff  523 Nov 27 _helpers.tpl
```

---

## Concept 7: Semantic Versioning for Charts

Helm charts follow semantic versioning (semver): `MAJOR.MINOR.PATCH`.

### Understanding Semver for Charts

| Scenario | Version Change | Rationale |
|----------|----------------|-----------|
| Bug fix (values syntax, template indentation) | `1.2.3` → `1.2.4` | Patch — backwards compatible, internal improvement |
| New feature (add optional helper template) | `1.2.3` → `1.3.0` | Minor — backwards compatible, new functionality |
| New required field in values | `1.2.3` → `2.0.0` | Major — breaking change, existing values.yaml won't work |
| Change chart name | Any | Major — breaking change, existing deployments won't update |

### Practical Example: Versioning Your AI Agent Chart

```yaml
---
# myapp-chart/Chart.yaml
apiVersion: v2
name: agent-chart
version: 1.0.0
description: "AI agent deployment chart"
---
```

**Scenario 1: Fix indentation bug in service template**
```yaml
# No breaking change, existing installations unaffected
version: 1.0.1
```

**Scenario 2: Add optional monitoring sideca (optional feature)**
```yaml
# Backwards compatible, existing values work
version: 1.1.0
```

**Scenario 3: Add required field in values.yaml (for security context)**
```yaml
# Breaking change—existing values.yaml missing the field
version: 2.0.0

# deployments must upgrade with new values or provide the field
```

### Version Tags in Registries

```bash
# Your chart versions appear as registry tags
oci://docker.io/myusername/agent-chart:1.0.0
oci://docker.io/myusername/agent-chart:1.0.1
oci://docker.io/myusername/agent-chart:1.1.0
oci://docker.io/myusername/agent-chart:2.0.0

# Helm pulls the exact version requested
helm pull oci://docker.io/myusername/agent-chart:1.1.0
# Gets 1.1.0, not latest
```

**Output:**
```bash
$ # After pushing multiple versions
$ helm search repo oci://docker.io/myusername/agent-chart
# (search not directly supported for OCI, but you can list tags via registry CLI)

$ # With helm CLI >= 3.7
$ helm pull oci://docker.io/myusername/agent-chart:1.1.0
Downloaded to agent-chart-1.1.0.tgz

$ helm pull oci://docker.io/myusername/agent-chart:2.0.0
Downloaded to agent-chart-2.0.0.tgz
```

---

## Concept 8: Installing Charts Directly from OCI URLs

Instead of downloading then installing, install directly from OCI URLs.

### Installing from OCI

```bash
# Download, extract, and install in one command
helm install my-release oci://docker.io/myusername/myapp-chart:1.2.3

# Specify namespace
helm install my-release oci://docker.io/myusername/myapp-chart:1.2.3 \
  --namespace production \
  --create-namespace

# With custom values
helm install my-release oci://docker.io/myusername/myapp-chart:1.2.3 \
  --namespace production \
  --values custom-values.yaml
```

**Output:**
```bash
$ helm install agent oci://docker.io/myusername/agent-chart:1.0.0 \
    --namespace default \
    --wait

NAME: agent
NAMESPACE: default
STATUS: deployed
REVISION: 1
NOTES:
...

$ kubectl get pods
NAME                    READY   STATUS    RESTARTS   AGE
agent-7d4f8b9c2-xk9vz   1/1     Running   0          5s
```

### Upgrading from OCI URLs

```bash
# Upgrade to a new version
helm upgrade my-release oci://docker.io/myusername/myapp-chart:1.3.0

# Rollback if something goes wrong
helm rollback my-release
```

**Output:**
```bash
$ helm upgrade agent oci://docker.io/myusername/agent-chart:1.1.0

Release "agent" has been upgraded. Happy Helming!
NAME: agent
NAMESPACE: default
STATUS: deployed
REVISION: 2

$ helm history agent
REVISION  UPDATED                   STATUS    CHART             APP VERSION
1         Wed Nov 27 10:30:00 2025  deployed  agent-chart-1.0.0 1.0.0
2         Wed Nov 27 10:35:00 2025  deployed  agent-chart-1.1.0 1.1.0
```

---

## Concept 9: Using OCI URLs in Chart Dependencies

In Lesson 4 (Chart Dependencies), you used HTTP URLs to reference external charts. You can also reference them via OCI.

### Declaring OCI Dependencies in Chart.yaml

```yaml
---
# myapp-chart/Chart.yaml
apiVersion: v2
name: myapp-chart
version: 1.2.3
dependencies:
  - name: postgresql
    version: "12.x"
    repository: "oci://docker.io/bitnamicharts"
    condition: postgresql.enabled

  - name: redis
    version: "17.x"
    repository: "oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts"
    condition: redis.enabled
---
```

### Updating Dependencies with OCI URLs

```bash
# Helm authenticates and downloads from OCI registry
helm dependency update myapp-chart/

# Output:
# Getting updates for unmanaged Helm repositories...
# ...Successfully got an update from the "oci://docker.io/bitnamicharts" chart repository
# ...Successfully got an update from the "oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts" chart repository
# Update Complete. ⎈ Happy Helming!
```

**Output:**
```bash
$ helm dependency update myapp-chart/
Getting updates for unmanaged Helm repositories...
...Successfully got an update from the "oci://docker.io/bitnamicharts" chart repository
...Successfully got an update from the "oci://123456789.dkr.ecr.us-east-1.amazonaws.com/charts" chart repository
Update Complete. ⎈ Happy Helming!

$ ls myapp-chart/charts/
postgresql-12.1.5.tgz
redis-17.3.2.tgz

$ tar -tzf myapp-chart/charts/postgresql-12.1.5.tgz | head
postgresql/Chart.yaml
postgresql/values.yaml
postgresql/templates/
```

### Mixed HTTP and OCI Dependencies

You can mix HTTP (traditional Helm repos) and OCI in the same chart:

```yaml
---
dependencies:
  # HTTP repository (old style)
  - name: prometheus
    version: "15.x"
    repository: "https://prometheus-community.github.io/helm-charts"

  # OCI registry (new style)
  - name: postgresql
    version: "12.x"
    repository: "oci://docker.io/bitnamicharts"
---
```

**Output:**
```bash
$ helm dependency update myapp-chart/
Getting updates for unmanaged Helm repositories...
...Successfully got an update from the "https://prometheus-community.github.io/helm-charts" chart repository
...Successfully got an update from the "oci://docker.io/bitnamicharts" chart repository
Update Complete. ⎈ Happy Helming!

$ ls myapp-chart/charts/
postgresql-12.1.5.tgz
prometheus-15.2.1.tgz
```

---

## Exercise 1: Package Your Chart

Create a simple chart and package it:

```bash
# Create a minimal chart
mkdir -p my-agent-chart/templates
cd my-agent-chart

cat > Chart.yaml <<'EOF'
apiVersion: v2
name: my-agent-chart
version: 1.0.0
description: "Simple AI agent Helm chart"
EOF

cat > values.yaml <<'EOF'
replicaCount: 2
image:
  repository: myagent
  tag: "1.0.0"
EOF

cat > templates/deployment.yaml <<'EOF'
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ .Chart.Name }}
spec:
  replicas: {{ .Values.replicaCount }}
  template:
    spec:
      containers:
      - name: {{ .Chart.Name }}
        image: {{ .Values.image.repository }}:{{ .Values.image.tag }}
EOF

# Package it
helm package .
```

**Output:**
```bash
$ helm package .
Successfully packaged chart and saved it to: /Users/username/my-agent-chart-1.0.0.tgz

$ ls -lh my-agent-chart-1.0.0.tgz
-rw-r--r--  1 username  staff   1.2K Nov 27 11:00 my-agent-chart-1.0.0.tgz
```

---

## Exercise 2: Create a Docker Hub Account (Public Registry)

For learning, use a public registry to avoid authentication complexity:

1. Go to https://hub.docker.com
2. Click "Sign up"
3. Create account: username, email, password
4. Verify email
5. On your local machine:
   ```bash
   helm registry login docker.io
   # Use your Docker Hub username and password
   ```

**Output:**
```bash
$ helm registry login docker.io
Username: your-docker-username
Password:
Login Succeeded
```

---

## Exercise 3: Push Your Chart to OCI

Using the packaged chart and Docker Hub credentials:

```bash
# Push to Docker Hub
helm push my-agent-chart-1.0.0.tgz oci://docker.io/your-docker-username/

# The chart is now in the registry
# Reference: oci://docker.io/your-docker-username/my-agent-chart:1.0.0
```

**Output:**
```bash
$ helm push my-agent-chart-1.0.0.tgz oci://docker.io/your-docker-username/
Pushed: docker.io/your-docker-username/my-agent-chart:1.0.0
```

---

## Exercise 4: Pull Your Chart from OCI

Download your published chart:

```bash
# Create a new directory
mkdir pull-test
cd pull-test

# Pull the chart
helm pull oci://docker.io/your-docker-username/my-agent-chart:1.0.0

# Extract and inspect
tar -xzf my-agent-chart-1.0.0.tgz
cat my-agent-chart/Chart.yaml
```

**Output:**
```bash
$ helm pull oci://docker.io/your-docker-username/my-agent-chart:1.0.0
Downloaded to my-agent-chart-1.0.0.tgz

$ tar -xzf my-agent-chart-1.0.0.tgz

$ cat my-agent-chart/Chart.yaml
apiVersion: v2
name: my-agent-chart
version: 1.0.0
description: "Simple AI agent Helm chart"
```

---

## Exercise 5: Install Directly from OCI URL

Deploy using the OCI chart reference without downloading first:

```bash
# Install from OCI URL (requires Minikube/cluster running)
helm install my-agent oci://docker.io/your-docker-username/my-agent-chart:1.0.0 \
  --namespace default

# Check deployment
kubectl get deployments
kubectl get pods
```

**Output:**
```bash
$ helm install my-agent oci://docker.io/your-docker-username/my-agent-chart:1.0.0

NAME: my-agent
NAMESPACE: default
STATUS: deployed
REVISION: 1

$ kubectl get deployments
NAME       READY   UP-TO-DATE   AVAILABLE   AGE
my-agent   2/2     2            2           5s

$ kubectl get pods
NAME                        READY   STATUS    RESTARTS   AGE
my-agent-5c7d8f9b2-x4k9p   1/1     Running   0          5s
my-agent-5c7d8f9b2-y8m2q   1/1     Running   0          5s
```

---

## Exercise 6: Update Chart and Push a New Version

Modify your chart and publish version 1.1.0:

```bash
# In my-agent-chart directory, update Chart.yaml
sed -i '' 's/version: 1.0.0/version: 1.1.0/' Chart.yaml

# Update values.yaml to use image tag 1.1.0
sed -i '' 's/tag: "1.0.0"/tag: "1.1.0"/' values.yaml

# Package the new version
helm package .

# Push to registry
helm push my-agent-chart-1.1.0.tgz oci://docker.io/your-docker-username/

# Both versions now exist in the registry
# oci://docker.io/your-docker-username/my-agent-chart:1.0.0
# oci://docker.io/your-docker-username/my-agent-chart:1.1.0
```

**Output:**
```bash
$ helm package .
Successfully packaged chart and saved it to: my-agent-chart-1.1.0.tgz

$ helm push my-agent-chart-1.1.0.tgz oci://docker.io/your-docker-username/
Pushed: docker.io/your-docker-username/my-agent-chart:1.1.0
```

---

## Note: Local Registry for Private/Development Environments

If you don't have access to Docker Hub or prefer to keep charts private during development, you can run a local OCI registry:

```bash
# Run local registry on your machine
docker run -d -p 5000:5000 --name local-registry registry:2

# Push to local registry
helm push my-agent-chart-1.0.0.tgz oci://localhost:5000/

# Pull from local registry
helm pull oci://localhost:5000/my-agent-chart:1.0.0

# Install from local registry
helm install my-agent oci://localhost:5000/my-agent-chart:1.0.0

# Stop when done
docker stop local-registry
docker rm local-registry
```

**Output:**
```bash
$ docker run -d -p 5000:5000 --name local-registry registry:2
Unable to find image 'registry:2' locally
Pulling from library/registry
...
e83ee3a23efb: Already exists
Digest: sha256:...
Status: Downloaded newer image for registry:2
ef7b2e7b8c8f5d9a

$ helm push my-agent-chart-1.0.0.tgz oci://localhost:5000/
Pushed: localhost:5000/my-agent-chart:1.0.0

$ helm pull oci://localhost:5000/my-agent-chart:1.0.0
Downloaded to my-agent-chart-1.0.0.tgz
```

---

## Try With AI

### Part 1: Initial Request

Ask AI to explain the differences between pushing Helm charts to OCI registries versus traditional Helm repositories:

**Prompt:**
"Explain the key differences between distributing Helm charts via OCI registries (like Docker Hub) versus traditional Helm repositories. Why did Helm move to OCI standards?"

### Part 2: Critical Evaluation

Review AI's response. Ask yourself:

- Does it explain why OCI provides a simpler distribution model?
- Does it mention shared authentication and versioning?
- Does it clarify that OCI registries store both images AND charts?

### Part 3: Constraint Teaching

If AI missed important details, refine your question:

"Also explain how authentication works—what credentials do I need to push to Docker Hub for both images and Helm charts?"

### Part 4: Refinement

Ask AI to validate how you'd push a chart to a private Amazon ECR registry:

"Walk me through the steps to push a Helm chart to a private ECR repository in AWS. What commands do I run, and what credentials do I need?"

### Part 5: Final Check

Compare what you learned from AI to the concepts in this lesson:

- Can you explain why `oci://` URLs replace HTTP repository URLs?
- Can you describe the semantic versioning strategy for charts?
- Can you walk through the complete workflow: package → authenticate → push → pull → install?
- Did AI's explanations match the examples in this lesson?

