---
sidebar_position: 7
chapter: 49
lesson: 7
duration_minutes: 50
title: "Security & Best Practices"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Security auditing teaches defensive containerization"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Run containers as non-root users with USER directive"
    bloom_level: "Apply"
  - id: LO2
    description: "Scan images for vulnerabilities with Docker Scout"
    bloom_level: "Apply"
  - id: LO3
    description: "Use Docker Hardened Images (DHI) for reduced attack surface"
    bloom_level: "Apply"
  - id: LO4
    description: "Manage secrets without baking them into images"
    bloom_level: "Apply"
  - id: LO5
    description: "Configure read-only root filesystems for immutability"
    bloom_level: "Apply"
  - id: LO6
    description: "Identify common security anti-patterns in Dockerfiles"
    bloom_level: "Analyze"
  - id: LO7
    description: "Apply principle of least privilege to container permissions"
    bloom_level: "Apply"
  - id: LO8
    description: "Use .dockerignore to prevent sensitive file leakage"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO4
    competency_area: "4. Safety"
    competency: "4.3 Protecting personal data and privacy"
  - objective_id: LO6
    competency_area: "4. Safety"
    competency: "4.2 Protecting personal health and well-being"
---

# Security & Best Practices

By default, Docker runs containers with significant security risks. Your agent runs as root (unrestricted system access). Secrets baked into images leak to anyone with image access. Packages installed during build bloat your image with unnecessary CVEs. The root filesystem is writable, allowing attackers to modify code mid-execution.

In this lesson, you'll audit a deliberately insecure Dockerfile, identify each vulnerability, and remediate it. Through hands-on fixes, you'll learn the principle of least privilege—not just as a concept, but as a practiced skill. By the end, your container will run with minimal permissions, no exposed secrets, and no unnecessary attack surface.

---

## The Insecure Dockerfile: Eight Vulnerabilities

Here's a production-like Dockerfile with realistic security mistakes:

```dockerfile
# INSECURE - DO NOT USE IN PRODUCTION
FROM python:3.12

# VIOLATION 1: No .dockerignore prevents secrets leakage
WORKDIR /app

# VIOLATION 2: Build secrets copied into image layers
COPY .env .
COPY credentials.json .
COPY config/prod-db-password.txt .

# VIOLATION 3: pip install as root
# VIOLATION 4: pip cache not cleaned (bloats image)
COPY requirements.txt .
RUN pip install -r requirements.txt

# VIOLATION 5: Entire source copied including tests, logs, etc
COPY . .

# VIOLATION 6: Runs as root (UID 0, unrestricted system access)
# No USER directive

# VIOLATION 7: No read-only filesystem protection
# VIOLATION 8: Base image (python:3.12) has 150+ unpatched CVEs
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0"]
```

Let's identify and fix each vulnerability:

---

## Vulnerability 1: No .dockerignore (Leaking Sensitive Files)

**The Problem**: Without `.dockerignore`, the `COPY . .` instruction copies ALL files into the image layer, including:

- `.env` (environment secrets)
- `credentials.json` (API keys)
- `.git/` (source history)
- `node_modules/` or `venv/` (unnecessary files)
- `*.log` (application logs)

Anyone with access to your image can extract these files.

**The Fix**: Create a `.dockerignore` file at your project root:

**File: .dockerignore**
```
# Environment and secrets
.env
.env.local
credentials.json
config/prod-*.txt
secrets/

# Version control
.git
.gitignore
.github/

# Development
venv/
.venv/
env/
node_modules/
.pytest_cache/
__pycache__/
*.pyc

# Dependency locks (use requirements.txt instead)
poetry.lock
package-lock.json

# IDE and OS
.vscode/
.idea/
.DS_Store
*.swp

# Temporary files
*.log
tmp/
build/
dist/
*.egg-info/

# Build artifacts
.docker/
docker-compose.override.yml
```

**Test it**:
```bash
# Without .dockerignore, the COPY includes everything
docker build -t myapp:unsafe .
docker run --rm myapp:unsafe ls -la /app

# Output shows .env, credentials.json present (UNSAFE)
```

---

## Vulnerability 2 & 3: Secrets Baked into Layers (Impossible to Remove)

**The Problem**: Once a secret is copied into a layer, it's part of the image forever—even if you delete it in a later layer, the original layer still contains it. Anyone with the image can extract it.

```dockerfile
COPY .env .                    # Layer contains secret
RUN rm .env                    # Later layer deletes reference, but secret is still in earlier layer
```

**The Fix**: Never copy secrets into images. Use build secrets (temporary, not in layers):

**File: Dockerfile (secure approach)**
```dockerfile
FROM docker/python:3.12-dhi AS builder

WORKDIR /app
COPY requirements.txt .

# --mount=type=secret makes secrets available DURING build only
# They don't persist in image layers
RUN --mount=type=secret=pip_token \
    pip install -r requirements.txt

FROM docker/python:3.12-dhi
WORKDIR /app
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY main.py .
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0"]
```

**Pass secrets at build time** (they don't enter the image):
```bash
docker build --secret pip_token=$MY_TOKEN_VALUE .
```

**At runtime, inject secrets as environment variables**:
```bash
# Load from .env file
docker run --env-file .env myapp:latest

# Or explicitly
docker run -e DATABASE_URL=postgres://... myapp:latest
```

**Output verification**:
```bash
# Build with secrets mounted
docker build --secret pip_token=$GITHUB_TOKEN -t myapp:secure .

# Inspect image - secrets are NOT in layers
docker inspect myapp:secure

# Runtime injection works
docker run --env-file /path/to/.env myapp:secure
```

---

## Vulnerability 4: Uncleaned Package Cache (Image Bloat)

**The Problem**: When you run `pip install`, it downloads packages and caches them. In the image layer, that cache is left behind, doubling the size:

```dockerfile
RUN pip install -r requirements.txt
# Cache is in /root/.cache (not removed)
```

**The Fix**: Clean the cache immediately after install:

```dockerfile
RUN pip install --no-cache-dir -r requirements.txt
```

**Test the difference**:
```bash
# Build WITH cache (bloated)
cat > Dockerfile.bloated << 'EOF'
FROM python:3.12
RUN pip install fastapi uvicorn pydantic requests
EOF
docker build -t myapp:bloated -f Dockerfile.bloated .

# Build WITHOUT cache (lean)
cat > Dockerfile.lean << 'EOF'
FROM python:3.12
RUN pip install --no-cache-dir fastapi uvicorn pydantic requests
EOF
docker build -t myapp:lean -f Dockerfile.lean .

# Compare sizes
docker images | grep myapp
```

**Output**:
```
REPOSITORY   TAG        SIZE
myapp        bloated    850MB
myapp        lean       420MB  # 50% smaller with --no-cache-dir
```

---

## Vulnerability 5: Copying Unnecessary Files (Attack Surface)

**The Problem**: `COPY . .` includes development files:

- Tests (pytest files, fixtures)
- Logs (application logs from development)
- Temporary files (git artifacts, Python cache)
- Credentials (accidentally committed .env files)

**The Fix**: Copy only what your application needs:

```dockerfile
# Copy only essentials
COPY main.py .
COPY models/ ./models/
COPY utils/ ./utils/

# Explicitly exclude development artifacts
# (combined with .dockerignore for defense-in-depth)
```

---

## Vulnerability 6: Running as Root (Unrestricted Access)

**The Problem**: By default, container processes run as root (UID 0). If your application is compromised, the attacker gains full system control.

```dockerfile
# No USER directive = runs as root
CMD ["python", "-m", "uvicorn", "main:app"]

# $ docker run myapp ps aux
# root    1  0.0  0.1 ...  python -m uvicorn
```

**The Fix**: Create a non-root user and switch to it:

```dockerfile
# Create user and group with fixed IDs (reproducible)
RUN groupadd -r appgroup && useradd -r -g appgroup appuser

# Change ownership of application files
COPY --chown=appuser:appgroup main.py .

# Switch to non-root user
USER appuser

# Now process runs with minimal privileges
CMD ["python", "-m", "uvicorn", "main:app"]
```

**Verify the change**:
```bash
docker build -t myapp:secure .
docker run --rm myapp:secure whoami

# Output: appuser (not root!)
```

---

## Vulnerability 7: Writable Root Filesystem (Mutable Attacks)

**The Problem**: By default, containers have a writable root filesystem. An attacker inside your container can:

- Modify application code while running
- Write malware to system directories
- Corrupt application files

**The Fix**: Run with `--read-only` flag:

```bash
docker run --read-only myapp:secure
```

**For persistent data**, mount a writable volume:

```bash
docker run --read-only -v /tmp -v /var/tmp myapp:secure
```

**In docker-compose.yml**:
```yaml
services:
  app:
    image: myapp:secure
    read_only: true
    tmpfs:
      - /tmp
      - /var/tmp
```

**Test it**:
```bash
# Try to modify the filesystem while container runs
docker run --rm myapp:secure touch /app/hacked.txt

# Output: Read-only file system error
# touch: cannot touch '/app/hacked.txt': Read-only file system
```

---

## Vulnerability 8: Using Bloated Base Images (Hundreds of CVEs)

**The Problem**: The standard `python:3.12` image includes tools you don't need (wget, curl, git, build-essential). Each tool is a potential vulnerability:

- `python:3.12`: 150+ known CVEs
- `python:3.12-slim`: 80+ known CVEs
- `docker/python:3.12-dhi`: 5 known CVEs (95% reduction)

**Docker Hardened Images (DHI)** are maintained by the Docker team specifically for security. They remove unnecessary packages and patch vulnerabilities aggressively.

**The Fix**: Switch to Docker Hardened Images:

```dockerfile
# Before (vulnerable)
FROM python:3.12

# After (hardened)
FROM docker/python:3.12-dhi
```

**Scan and compare**:
```bash
# Standard Python image
docker build -t myapp:standard -f Dockerfile.standard .
docker scout cves myapp:standard | grep "CVE-" | wc -l

# Hardened image
docker build -t myapp:dhi -f Dockerfile.dhi .
docker scout cves myapp:dhi | grep "CVE-" | wc -l
```

**Output**:
```
Standard image CVEs: 147
Hardened image CVEs: 4
```

**Find Hardened Images**: Visit [Docker Hub Trusted Content](https://hub.docker.com/u/docker) for official hardened images:
- `docker/python:3.12-dhi`
- `docker/node:20-dhi`
- `docker/golang:1.21-dhi`

---

## The Secure Dockerfile: All Fixes Applied

Combining all fixes, here's the secure version:

```dockerfile
# Stage 1: Builder (install dependencies)
FROM docker/python:3.12-dhi AS builder

WORKDIR /app

# Copy only requirements
COPY requirements.txt .

# Install with no-cache-dir (clean caches)
# Use build secrets for private package repos
RUN --mount=type=secret=pip_token \
    pip install --no-cache-dir -r requirements.txt

# Stage 2: Runtime (minimal, secure)
FROM docker/python:3.12-dhi

# Create non-root user
RUN groupadd -r appgroup && useradd -r -g appgroup appuser

WORKDIR /app

# Copy dependencies from builder
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages

# Copy only application code, set ownership
COPY --chown=appuser:appgroup main.py .
COPY --chown=appuser:appgroup models/ ./models/
COPY --chown=appuser:appgroup utils/ ./utils/

# Switch to non-root user
USER appuser

# Secrets injected at runtime, never baked into image
# Read-only filesystem enforced at docker run time
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0"]
```

**Build and verify**:
```bash
# Create .dockerignore (prevents secret leakage)
cat > .dockerignore << 'EOF'
.env
.env.local
credentials.json
config/prod-*.txt
secrets/
.git
node_modules/
__pycache__/
*.log
EOF

# Build (optionally with secrets)
docker build -t myapp:secure .

# Run with security hardening
docker run \
  --rm \
  --read-only \
  --tmpfs /tmp \
  --tmpfs /var/tmp \
  --env-file .env \
  -p 8000:8000 \
  myapp:secure

# Verify container runs as non-root
docker ps --format "{{.Names}}: {{.Command}}"

# Output shows process running under appuser, not root
```

---

## Docker Scout: Scanning for Vulnerabilities

Docker Scout is Docker's vulnerability scanner. It checks your image against CVE databases and reports findings.

**Install Docker Scout** (included with Docker Desktop):
```bash
docker scout --version
```

**Scan for CVEs**:
```bash
# Quick overview
docker scout quickview myapp:secure

# Detailed report
docker scout cves myapp:secure

# Compare with hardened base
docker scout compare myapp:secure --to docker/python:3.12-dhi
```

**Output of docker scout cves**:
```
  1 critical   fastapi 0.100.0 → 0.115.0   CVE-2024-xxxx Improper Input...
  3 high       python 3.12 (os)             CVE-2024-yyyy Command Inject...
  2 medium     requests 2.31.0 → 2.32.0    CVE-2024-zzzz...
```

**Fix vulnerabilities**:
```bash
# Update requirements.txt with patched versions
pip list --outdated

# Update package
pip install --upgrade fastapi

# Rebuild image
docker build -t myapp:patched .

# Verify fixes
docker scout cves myapp:patched
```

---

## Security Checklist: Before Shipping

Before pushing your container to a registry, verify:

- [ ] **Non-root user**: `USER appuser` in Dockerfile
- [ ] **No hardcoded secrets**: No `.env`, credentials, or API keys in image
- [ ] **.dockerignore created**: Prevents accidental secret leakage
- [ ] **Hardened base image**: Using `docker/` prefix images
- [ ] **Cache cleaned**: `--no-cache-dir` for pip, `--rm` for apt
- [ ] **Unnecessary files excluded**: Only `COPY` what's needed
- [ ] **Scout scan passed**: `docker scout cves` shows acceptable risk
- [ ] **Read-only test**: Verify with `docker run --read-only`

**Run the full security audit**:
```bash
# 1. Scan for CVEs
docker scout cves myapp:final

# 2. Check user
docker run --rm myapp:final whoami
# Should output: appuser

# 3. Test read-only
docker run --rm --read-only myapp:final sh -c "touch /test && echo FAIL || echo PASS"
# Should output: PASS

# 4. Verify no secrets in image
docker run --rm myapp:final grep -r "password\|secret\|api.key" . 2>/dev/null | wc -l
# Should output: 0
```

---

## Try With AI

The insecure Dockerfile you audited in this lesson was realistic—these vulnerabilities appear in production codebases. Docker Scanner and Scout are tools to catch them before deployment.

**Setup**: You have the insecure Dockerfile from this lesson and access to Docker Scout.

**Prompts to explore**:

1. **Analyze a specific CVE**:
   ```
   Ask AI: "I ran 'docker scout cves myapp:insecure' and got
   this output: [paste full output]. What's the severity of each CVE?
   Which are exploitable in my application context?"
   ```

2. **Understand a remediation**:
   ```
   Ask AI: "I want to use --mount=type=secret for build secrets.
   How do I pass a GitHub token to 'pip install' from a private
   repository without baking it into the image? Show the full docker build command."
   ```

3. **Validate your hardened image**:
   ```
   Ask AI: "Here's my production Dockerfile. Check it against these
   security principles: [paste the 8 vulnerabilities from this lesson].
   Does it violate any?"
   ```

4. **Compare base images**:
   ```
   Ask AI: "What's the actual difference between 'python:3.12' and
   'docker/python:3.12-dhi'? Are there cases where I should use
   standard images instead of hardened images?"
   ```

Safety note: Never share the actual content of `.env` files or API keys when asking AI for help. Show only sanitized examples (e.g., `DATABASE_URL=postgres://user:***@host:5432/db`).
