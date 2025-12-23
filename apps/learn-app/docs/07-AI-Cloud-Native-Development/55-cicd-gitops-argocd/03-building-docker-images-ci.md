---
sidebar_position: 3
title: "Building Docker Images in CI"
description: "Automate multi-platform Docker builds with GitHub Actions and registry push"
keywords: [docker, buildx, multi-platform, github actions, container registry, ghcr, ci]
---

# Building Docker Images in CI

In Lesson 2, you created GitHub Actions workflows that respond to code changes. Now we connect the build stage to actual container images. When you push code, GitHub Actions should automatically build your Docker image, tag it appropriately, and push it to a registry (Docker Hub, GitHub Container Registry, or similar).

The challenge: Your local machine might be Intel (amd64) while your cloud servers run ARM (arm64). A Docker image built on your Mac M1 may not run on your cloud Kubernetes cluster. This lesson teaches you how to build once and push images that work everywhere—using Docker buildx for multi-platform builds and GitHub Actions caching for speed.

## From Local Docker Build to Automated CI Builds

You've built Docker images locally:

```bash
docker build -t myagent:latest .
docker push myagent:latest
```

**Output:**
```
Sending build context to Docker daemon
Step 1/8 : FROM python:3.11-slim
 ---> abc123def456
Step 2/8 : WORKDIR /app
 ---> Running in xyz789
...
Successfully built abc123def456
Successfully tagged myagent:latest
The push refers to repository [docker.io/library/myagent]
latest: digest: sha256:abcd...xyz
```

This workflow is manual, local, and single-platform. Every developer builds on their own machine. If you're on Apple Silicon and your teammate is on Intel, the images might have subtle differences. And there's no audit trail.

**In CI, this becomes automated and declarative:**

1. You push code to GitHub
2. GitHub Actions detects the push
3. GitHub Actions builds the image (both amd64 and arm64)
4. GitHub Actions pushes to your registry
5. Your Kubernetes cluster pulls the multi-platform image

The image now has a clear source (Git commit), supports all architectures, and is reproducible.

## Why Multi-Platform Builds Matter

Let's make this concrete with three scenarios:

**Scenario 1: You develop on Mac M1, your team uses Intel**

Local build on your Mac:
```bash
docker build -t agent:latest .
# Creates image for linux/arm64
```

Output:
```
Created image: sha256:abc123 (linux/arm64)
```

When your teammate pulls and runs it on their Intel machine, Docker uses QEMU emulation, and everything runs at 1/10 speed. Not acceptable for production.

**Scenario 2: CI builds only for amd64, but Kubernetes schedules on ARM nodes**

A common setup: GitHub Actions runner is amd64, your cluster has some ARM nodes. You build an amd64 image, push it, Kubernetes tries to schedule it on ARM → ImagePullBackOff error. The cluster can't run the image.

**Scenario 3: You want to run your agent on both cloud servers and embedded devices**

You build once with both platforms:
```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t agent:latest \
  .
```

Output:
```
#0 building with "default" instance using docker driver
#1 [linux/amd64 internal] load build definition from Dockerfile
#1 transferring dockerfile: 65B done
#1 [linux/amd64 internal] load .dockerignore
...
#6 [linux/amd64] exporting to image
#6 sha256:image-digest-amd64
#6 [linux/arm64] exporting to image
#6 sha256:image-digest-arm64
```

Now one tag (agent:latest) points to a multi-platform image. Kubernetes on any architecture pulls the right variant automatically.

## Docker Buildx: The Multi-Platform Builder

Docker buildx is an extended build capability that uses QEMU (a machine emulator) to build for architectures different from your host.

### Why Buildx Instead of Regular Docker Build?

Regular `docker build` creates an image for your current architecture only:

```bash
docker build -t agent:latest .
```

Output:
```
# Builds only for your current architecture (e.g., linux/arm64 on Mac)
```

Docker buildx creates images for specified architectures:

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t agent:latest \
  .
```

Output:
```
#1 [linux/amd64] building
#1 [linux/arm64] building
#6 exporting to docker image format
# Builds both amd64 and arm64
```

### Setting Up Buildx in GitHub Actions

GitHub Actions runners already have buildx installed. You need to initialize the builder and enable QEMU emulation:

```yaml
name: Build Multi-Platform Image

on: push

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      # Set up QEMU for cross-platform building
      - uses: docker/setup-qemu-action@v3

      # Set up buildx (the builder itself)
      - uses: docker/setup-buildx-action@v3

      # Build the image
      - run: |
          docker buildx build \
            --platform linux/amd64,linux/arm64 \
            -t agent:latest \
            .
```

Output (when workflow runs):
```
GitHub Actions workflow triggered by push
Set up QEMU (emulator for cross-platform builds)
Set up buildx (multi-platform builder)
Building for linux/amd64 and linux/arm64
Both platforms built successfully
```

**What each step does:**

- **setup-qemu-action**: Installs QEMU so the runner can emulate ARM64 (if running on amd64)
- **setup-buildx-action**: Configures Docker's buildx plugin for advanced builds
- **docker buildx build**: Builds for multiple platforms in a single command

## Registry Authentication: Pushing Your Images

Building locally does nothing without pushing to a registry. You need to authenticate with your container registry (Docker Hub, GHCR, Quay, etc.).

### GitHub Container Registry (GHCR) Setup

GHCR is GitHub's registry, and GitHub Actions has built-in support. Here's how:

**Step 1: Create a GitHub Token**

Your workflow needs permission to push to GHCR. GitHub Actions provides an automatic token for this:

```yaml
- uses: docker/login-action@v3
  with:
    registry: ghcr.io
    username: ${{ github.actor }}
    password: ${{ secrets.GITHUB_TOKEN }}
```

Output (when this step runs):
```
Authenticating with ghcr.io
Username: octocat
Token: ****[hidden]****
Authentication successful
```

The `${{ secrets.GITHUB_TOKEN }}` is a built-in secret that GitHub automatically creates—no setup needed.

**Step 2: Use Authenticated Registry in Build**

Once authenticated, push to GHCR with your image name:

```yaml
- uses: docker/build-push-action@v6
  with:
    context: .
    platforms: linux/amd64,linux/arm64
    push: true
    tags: ghcr.io/${{ github.repository }}:latest
```

Output (when workflow runs):
```
Building image for linux/amd64 and linux/arm64
Pushing ghcr.io/octocat/agent:latest
Push completed
Digest: sha256:abcd...xyz
```

### Docker Hub Authentication

If using Docker Hub instead of GHCR:

**Step 1: Store credentials as GitHub Secrets**

In your repository settings (Settings → Secrets and Variables → Actions), add:
- `DOCKER_USERNAME`: Your Docker Hub username
- `DOCKER_PASSWORD`: Your Docker Hub access token (not password)

**Step 2: Authenticate in workflow**

```yaml
- uses: docker/login-action@v3
  with:
    registry: docker.io
    username: ${{ secrets.DOCKER_USERNAME }}
    password: ${{ secrets.DOCKER_PASSWORD }}
```

Output (when this step runs):
```
Authenticating with docker.io
Username: ****[hidden]****
Password: ****[hidden]****
Authentication successful
```

## Image Tagging Strategies

A single image needs multiple identities:

- **Latest stable**: `myagent:latest` (always points to latest stable build)
- **Commit SHA**: `myagent:abc1234` (immutable reference to specific code)
- **Semantic version**: `myagent:1.2.3` (release versions)
- **Branch**: `myagent:main` (latest from main branch)

### Strategy 1: Latest + SHA on Every Push

```yaml
- uses: docker/build-push-action@v6
  with:
    context: .
    platforms: linux/amd64,linux/arm64
    push: true
    tags: |
      ghcr.io/${{ github.repository }}:latest
      ghcr.io/${{ github.repository }}:${{ github.sha }}
```

Output (example with commit abc1234567...):
```
Pushing ghcr.io/octocat/agent:latest
Pushing ghcr.io/octocat/agent:abc12345
Push completed successfully
```

Now you have:
- `ghcr.io/octocat/agent:latest` → always the newest
- `ghcr.io/octocat/agent:abc12345` → exact commit immutable reference

### Strategy 2: Latest Only on Main Branch

You might want `latest` to only update when pushing to main:

```yaml
on:
  push:
    branches:
      - main

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: docker/setup-qemu-action@v3
      - uses: docker/setup-buildx-action@v3

      - uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - uses: docker/build-push-action@v6
        with:
          context: .
          platforms: linux/amd64,linux/arm64
          push: true
          tags: |
            ghcr.io/${{ github.repository }}:latest
            ghcr.io/${{ github.repository }}:${{ github.sha }}
```

Output (push to main branch):
```
Workflow triggered by push to main
Building for linux/amd64 and linux/arm64
Pushing ghcr.io/octocat/agent:latest
Pushing ghcr.io/octocat/agent:abc12345
Push completed
```

Output (push to feature branch):
```
Workflow triggered by push to feature-x
Only triggered by main branch, skipping this run
```

## Build Caching: Faster Rebuilds

Building a Docker image from scratch takes time (dependency downloads, compilation, etc.). GitHub Actions caching reuses layers between builds.

### Layer Caching Within a Build

Docker caches layers locally—if a layer's inputs haven't changed, it reuses the cached layer:

```dockerfile
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .
RUN python -m pytest
```

If you change only application code (line: `COPY . .`), Docker reuses the pip layer because requirements.txt is unchanged. This saves ~30 seconds per build.

### GitHub Actions Build Cache (Advanced)

For even faster multi-platform builds, export cache to a registry:

```yaml
- uses: docker/build-push-action@v6
  with:
    context: .
    platforms: linux/amd64,linux/arm64
    push: true
    tags: ghcr.io/${{ github.repository }}:latest
    cache-from: type=gha
    cache-to: type=gha,mode=max
```

Output (first build):
```
Building from scratch (no cache)
Build time: 180s
Cache saved for next build
```

Output (second build):
```
Using cache from previous build
Build time: 45s (75% faster!)
```

**What's happening:**

- **cache-from: type=gha**: Load cache from GitHub Actions cache backend
- **cache-to: type=gha,mode=max**: Save all cache layers (mode=max is comprehensive)

This is a GitHub-hosted cache that persists between workflow runs.

## Complete Workflow: Building Your FastAPI Agent

Let's put it all together. Here's a production-ready workflow for your Part 6 FastAPI agent:

```yaml
name: Build and Push Agent Image

on:
  push:
    branches:
      - main

jobs:
  build-and-push:
    runs-on: ubuntu-latest
    permissions:
      contents: read
      packages: write
    steps:
      - uses: actions/checkout@v4

      - name: Set up QEMU
        uses: docker/setup-qemu-action@v3

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3

      - name: Log in to Container Registry
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Extract metadata (tags and labels)
        id: meta
        uses: docker/metadata-action@v5
        with:
          images: ghcr.io/${{ github.repository }}
          tags: |
            type=ref,event=branch
            type=semver,pattern={{version}}
            type=semver,pattern={{major}}.{{minor}}
            type=sha,prefix={{branch}}-

      - name: Build and push
        uses: docker/build-push-action@v6
        with:
          context: .
          platforms: linux/amd64,linux/arm64
          push: true
          tags: ${{ steps.meta.outputs.tags }}
          labels: ${{ steps.meta.outputs.labels }}
          cache-from: type=gha
          cache-to: type=gha,mode=max
```

**Output (when you push to main):**

```
Build and Push Agent Image workflow triggered
Set up QEMU for cross-platform emulation
Set up Docker Buildx
Logged in to ghcr.io as octocat
Extracting metadata from commit
Building for linux/amd64
Building for linux/arm64
Pushing to ghcr.io/octocat/agent:main
Pushing to ghcr.io/octocat/agent:sha-abc1234
Build completed successfully
Image digest: sha256:xyz789
```

**What each section does:**

| Section | Purpose |
|---------|---------|
| `on: push` | Trigger on every push to main |
| `permissions` | Grant GitHub token access to packages (container registry) |
| `setup-qemu-action` | Enable ARM64 emulation on the amd64 runner |
| `setup-buildx-action` | Initialize multi-platform builder |
| `login-action` | Authenticate with GHCR |
| `metadata-action` | Generate semantic tags (branch, semver, SHA) |
| `build-push-action` | Build for both platforms and push |
| `cache-from/cache-to` | Save build cache between runs |

### Testing the Workflow

Push this workflow to your repository and make a commit to main:

```bash
git add .github/workflows/build-and-push.yml
git commit -m "Add multi-platform Docker build workflow"
git push origin main
```

Output (in GitHub Actions UI):
```
Build and Push Agent Image workflow triggered
Set up QEMU: Completed
Set up Docker Buildx: Completed
Authenticated with ghcr.io
Build for linux/amd64: Completed
Build for linux/arm64: Completed
Push to registry: Completed
Workflow Status: ✓ Success
```

Then verify the image exists in your registry:

```bash
docker pull ghcr.io/octocat/agent:main
```

Output:
```
main: Pulling from octocat/agent
sha256:abc... Pull complete
Status: Downloaded newer image for ghcr.io/octocat/agent:main
```

## Why This Matters for Your Agent

Your Part 6 FastAPI agent is no longer a local Docker file—it's a reproducible, versioned artifact in a registry. When you're ready to deploy to Kubernetes (Lessons 5-9), you pull this image with full confidence:

- **Multi-platform**: Works on Intel and ARM servers
- **Versioned**: Exact commit SHA is embedded in the image
- **Cached**: Rebuilds are fast for small changes
- **Auditable**: Every build is logged in GitHub Actions and traced to a Git commit

## Try With AI

Ask Claude: **"Optimize my GitHub Actions Docker build workflow. I'm currently building for linux/amd64 only, builds take 5 minutes, and I want to support ARM for Mac users."**

Before accepting the output, validate:
- Does it use `docker/setup-qemu-action` and `docker/setup-buildx-action`?
- Does it add `platforms: linux/amd64,linux/arm64` to the build step?
- Does it configure `cache-from: type=gha` and `cache-to: type=gha,mode=max` for caching?

Then iterate with: **"Show me how to tag images with both the commit SHA and 'latest' only on the main branch, using the metadata action."**

Look for:
- Conditional tagging (only `latest` on main)
- `docker/metadata-action` with semantic versioning
- Clear explanation of how tag patterns work

Finally ask: **"My builds are timing out at 20 minutes on ARM emulation. How do I split the build into separate jobs per platform instead of emulating?"**

This explores the tradeoff between single builds (simpler, uses emulation) and split builds (faster per-platform, requires coordination).
