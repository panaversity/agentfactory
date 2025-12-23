---
sidebar_position: 2
title: "GitHub Actions Fundamentals"
description: "Create automated workflows with triggers, jobs, steps, and secrets"
keywords: [github actions, workflow, yaml, triggers, jobs, steps, ci, automation]
---

# GitHub Actions Fundamentals

GitHub Actions is where the CI/CD pipeline gets executed. In Lesson 1, you learned the five stages: trigger → build → test → push → deploy. GitHub Actions automates all of this through workflows—YAML files that describe what happens when specific events occur (like pushing code).

Think of GitHub Actions as the orchestrator that listens for events ("code was pushed") and runs jobs in response. Each job contains steps, and each step either runs a command or uses a pre-built action. By the end of this lesson, you'll understand how to read and write workflow files, making automation tangible.

## What GitHub Actions Does (The Mental Model)

Before YAML, understand the flow:

1. **You push code** to a GitHub repository
2. **GitHub detects the push** and checks for workflow files
3. **GitHub Actions triggers a workflow** (a YAML file in `.github/workflows/`)
4. **The workflow runs jobs** (serial or parallel)
5. **Each job runs steps** (individual commands or actions)
6. **The results are reported** back to GitHub (passed/failed)

This replaces manual steps—no more SSH-ing into a server to rebuild code. Everything is declarative and version-controlled.

## Workflow File Location and Structure

All workflows live in `.github/workflows/` at the root of your repository. Each file is a YAML document describing a workflow.

Here's the minimal structure:

```yaml
name: My Workflow
on: push
jobs:
  my-job:
    runs-on: ubuntu-latest
    steps:
      - run: echo "Hello, GitHub Actions!"
```

**Output:**
```
My Workflow triggered by push event
my-job started on ubuntu-latest
Hello, GitHub Actions!
my-job completed successfully
```

This workflow:
- **name**: Labels the workflow (shown in GitHub UI)
- **on**: Trigger event(s) (when to run)
- **jobs**: Collection of jobs (parallel or sequential)
- **runs-on**: The runner (virtual machine) type
- **steps**: Individual steps (commands or actions)

## Triggers: When Does a Workflow Run?

The `on` field determines what events trigger your workflow. Here are the most common:

### Push Trigger
Runs when code is pushed to a branch:

```yaml
on: push
```

Or limit to specific branches:

```yaml
on:
  push:
    branches:
      - main
      - develop
```

**Use case**: Run tests and build on every push.

### Pull Request Trigger
Runs when a PR is opened or updated:

```yaml
on: pull_request
```

Limit to specific branches:

```yaml
on:
  pull_request:
    branches:
      - main
```

**Use case**: Validate code before merging—run tests to prevent broken code from reaching main.

### Schedule Trigger
Runs on a cron schedule (e.g., nightly):

```yaml
on:
  schedule:
    - cron: '0 2 * * *'  # 2 AM UTC daily
```

**Output** (when triggered):
```
Scheduled workflow triggered at 2025-12-24 02:00:00 UTC
```

**Use case**: Nightly security scans, dependency updates, data cleanup.

### Manual Trigger
Allows manual execution from GitHub UI:

```yaml
on: workflow_dispatch
```

**Use case**: Deploy specific versions manually without waiting for a commit.

### Combine Multiple Triggers
A workflow can respond to multiple events:

```yaml
on:
  push:
    branches: [main]
  pull_request:
    branches: [main]
  schedule:
    - cron: '0 2 * * *'
```

## Jobs and Execution Model

A workflow contains jobs. By default, jobs run in parallel. Use `needs` to create dependencies (job ordering).

### Job Structure

```yaml
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - run: echo "Building..."

  test:
    runs-on: ubuntu-latest
    needs: build
    steps:
      - run: echo "Testing..."
```

**Output:**
```
build: Building...
build completed
test: Testing...
test completed
```

Here, `test` depends on `build` because of `needs: build`. GitHub Actions waits for `build` to succeed before starting `test`.

### Available Runners

- `ubuntu-latest`: Linux (most common)
- `windows-latest`: Windows
- `macos-latest`: macOS

Each runner is a fresh virtual machine, so dependencies must be installed fresh each time.

## Steps: The Building Blocks

Each step in a job does one of two things:
1. **Run a command** (`run:`)
2. **Use an action** (`uses:`)

### Run Command

```yaml
steps:
  - run: python -m pip install -r requirements.txt
  - run: python -m pytest
```

**Output:**
```
Collecting flask==2.3.0
Successfully installed flask-2.3.0
collected 12 items
test_app.py::test_hello_world PASSED
```

### Use an Action

Actions are reusable steps published on GitHub Marketplace. For example, `actions/checkout@v4` clones your repository:

```yaml
steps:
  - uses: actions/checkout@v4
  - uses: actions/setup-python@v5
    with:
      python-version: '3.11'
```

**Output:**
```
Checking out repository at main
Python 3.11.7 installed successfully
```

Common actions for Python projects:
- `actions/checkout@v4`: Clone your repository
- `actions/setup-python@v5`: Install Python
- `actions/upload-artifact@v3`: Save build artifacts
- `docker/login-action@v2`: Log into container registry

## Secrets and Environment Variables

Never hardcode credentials (API keys, passwords, registry tokens) in workflows. GitHub provides two ways to store sensitive data:

### Secrets
Secrets are encrypted and only exposed to workflows. Define them in repository Settings → Secrets and Variables → Actions Secrets.

Use in workflows:

```yaml
steps:
  - name: Log into Docker Hub
    uses: docker/login-action@v2
    with:
      username: ${{ secrets.DOCKER_USERNAME }}
      password: ${{ secrets.DOCKER_PASSWORD }}
```

**Output:**
```
Logging into Docker Hub...
Login successful
```

The actual values are never shown in logs.

### Environment Variables
For non-sensitive configuration:

```yaml
env:
  REGISTRY: ghcr.io
  IMAGE_NAME: my-agent

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - run: echo "Building ${{ env.IMAGE_NAME }}"
```

**Output:**
```
Building my-agent
```

**Rule of thumb**: Secrets for credentials, environment variables for configuration.

## Matrix Builds: Test Multiple Versions

Matrix lets you run the same job with different parameter combinations. This is essential for testing across Python versions:

```yaml
jobs:
  test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: ['3.9', '3.10', '3.11', '3.12']
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: ${{ matrix.python-version }}
      - run: python -m pytest
```

**Output:**
```
test (3.9): Running tests on Python 3.9
test (3.9): PASSED
test (3.10): Running tests on Python 3.10
test (3.10): PASSED
test (3.11): Running tests on Python 3.11
test (3.11): PASSED
test (3.12): Running tests on Python 3.12
test (3.12): PASSED
```

GitHub Actions runs each Python version in parallel. If any fails, the workflow fails. This ensures your code works across supported versions.

## Connecting Jobs with Dependencies

Use `needs` to create explicit job dependencies. This is different from implicit parallelism:

```yaml
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - run: echo "Building..."

  test:
    runs-on: ubuntu-latest
    needs: build
    steps:
      - run: echo "Testing after build..."

  deploy:
    runs-on: ubuntu-latest
    needs: test
    steps:
      - run: echo "Deploying after test..."
```

**Output:**
```
build: Building...
build completed
test: Testing after build...
test completed
deploy: Deploying after test...
deploy completed
```

The workflow follows this sequence: build → test → deploy. If build fails, test and deploy never run.

## Complete Working Example: FastAPI CI Workflow

Here's a production-ready workflow for your Part 6 FastAPI agent:

```yaml
name: FastAPI Agent CI

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: my-agent

jobs:
  test:
    name: Test
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: ['3.10', '3.11', '3.12']

    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Set up Python ${{ matrix.python-version }}
        uses: actions/setup-python@v5
        with:
          python-version: ${{ matrix.python-version }}

      - name: Cache pip dependencies
        uses: actions/cache@v3
        with:
          path: ~/.cache/pip
          key: ${{ runner.os }}-pip-${{ hashFiles('requirements.txt') }}
          restore-keys: |
            ${{ runner.os }}-pip-

      - name: Install dependencies
        run: |
          python -m pip install --upgrade pip
          pip install -r requirements.txt
          pip install pytest pytest-cov

      - name: Run unit tests
        run: pytest tests/ -v --cov=app --cov-report=xml

      - name: Upload coverage to Codecov
        uses: codecov/codecov-action@v3
        with:
          file: ./coverage.xml
          flags: unittests
          fail_ci_if_error: true

  lint:
    name: Lint
    runs-on: ubuntu-latest

    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install linting tools
        run: |
          pip install flake8 black isort

      - name: Run black formatter check
        run: black --check app/

      - name: Run isort import check
        run: isort --check-only app/

      - name: Run flake8 linter
        run: flake8 app/ --count --select=E9,F63,F7,F82 --show-source --statistics

  build-and-push:
    name: Build and Push Image
    runs-on: ubuntu-latest
    needs: [test, lint]
    if: github.ref == 'refs/heads/main' && github.event_name == 'push'

    permissions:
      contents: read
      packages: write

    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v2

      - name: Log into registry
        uses: docker/login-action@v2
        with:
          registry: ${{ env.REGISTRY }}
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Extract metadata
        id: meta
        uses: docker/metadata-action@v4
        with:
          images: ${{ env.REGISTRY }}/${{ github.repository_owner }}/${{ env.IMAGE_NAME }}
          tags: |
            type=sha,prefix={{branch}}-
            type=ref,event=branch
            type=semver,pattern={{version}}

      - name: Build and push
        uses: docker/build-push-action@v4
        with:
          context: .
          push: true
          tags: ${{ steps.meta.outputs.tags }}
          labels: ${{ steps.meta.outputs.labels }}
          cache-from: type=gha
          cache-to: type=gha,mode=max
```

**Output:**
```
FastAPI Agent CI triggered by push to main
test (3.10): Running unit tests...
test (3.10): ✓ 42 passed, 0 failed
test (3.11): ✓ 42 passed, 0 failed
test (3.12): ✓ 42 passed, 0 failed
lint: Running black formatter...
lint: ✓ All files formatted correctly
lint: Running flake8...
lint: ✓ No linting errors
build-and-push: Building Docker image...
build-and-push: Pushing to ghcr.io/username/my-agent:main-abc1234
build-and-push: ✓ Image pushed successfully
Workflow completed: all jobs passed
```

**What this workflow does**:

1. **Triggers**: On push to main or develop, and on pull_request to main
2. **Test job**:
   - Checks out code
   - Tests on Python 3.10, 3.11, 3.12 in parallel
   - Installs dependencies (with caching)
   - Runs pytest with coverage
   - Uploads coverage report

3. **Lint job**:
   - Runs in parallel with test
   - Checks code formatting (black)
   - Checks import ordering (isort)
   - Runs linting (flake8)

4. **Build-and-push job**:
   - Depends on test and lint (runs after both pass)
   - Only runs on pushes to main (not PRs)
   - Builds Docker image
   - Pushes to container registry (GHCR)
   - Tags with commit SHA for traceability

The workflow fails fast: if tests fail, linting fails, or formatting is wrong, the build doesn't even happen. This prevents broken code from reaching your container registry.

## Understanding Workflow Runs in GitHub UI

After pushing code, navigate to your repository → Actions tab. You'll see workflow runs listed. Click a run to see:

- **Jobs**: Listed with status (passed/failed/skipped)
- **Logs**: Click a job to see full output
- **Artifacts**: Download any uploaded files (coverage reports, build logs)
- **Timing**: How long each job took

This makes debugging and understanding your CI/CD pipeline transparent.

## Try With AI

Ask Claude: "Generate a GitHub Actions workflow for my FastAPI project that runs on push to main, installs dependencies, runs pytest, and fails if tests don't pass. The project has a requirements.txt file."

Before accepting the output, verify:
- Does it use `actions/checkout@v4` to clone the repository?
- Does it use `actions/setup-python@v5` to install Python?
- Does it run `pip install -r requirements.txt`?
- Does it run pytest and fail if tests fail?
- Is there a `runs-on: ubuntu-latest`?

If the output is missing any of these, ask: "Add a step to cache pip dependencies using actions/cache so subsequent runs are faster."

Then iterate with: "Modify the workflow to also run on pull_request events to main. Add a linting step using flake8 that runs in parallel with tests."

As you review the AI-generated workflow, ask yourself: What event triggered this? What jobs run? What's the execution order? This deep understanding makes you a confident CI/CD engineer.
