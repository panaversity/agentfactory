---
sidebar_position: 8
chapter: 49
lesson: 8
duration_minutes: 45
title: "AI-Assisted Docker with Gordon"
proficiency_level: B1
teaching_stage: 2
stage_name: "AI Collaboration"
stage_description: "Collaborative AI workflows for Docker development"
cognitive_load:
  concepts_count: 7
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Access Gordon through Docker Desktop interface"
    bloom_level: "Apply"
  - id: LO2
    description: "Generate Dockerfiles by describing your application to Gordon"
    bloom_level: "Apply"
  - id: LO3
    description: "Debug container failures by asking Gordon diagnostic questions"
    bloom_level: "Analyze"
  - id: LO4
    description: "Request optimization suggestions from Gordon for existing Dockerfiles"
    bloom_level: "Apply"
  - id: LO5
    description: "Evaluate Gordon's suggestions against your specific requirements"
    bloom_level: "Evaluate"
  - id: LO6
    description: "Refine Gordon's output through iterative dialogue"
    bloom_level: "Create"
  - id: LO7
    description: "Identify when Gordon's suggestions need manual adjustment"
    bloom_level: "Evaluate"
digcomp_mapping:
  - objective_id: LO5
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Evaluating data, information and digital content"
---

# AI-Assisted Docker with Gordon

You've learned to write Dockerfiles by hand. You understand layers, build optimization, and why certain patterns matter. Now let's accelerate development by working with Gordon, Docker's AI agent embedded in Docker Desktop. This isn't about letting AI take over—it's about building a collaborative workflow where you guide Gordon's output toward your specific requirements.

Gordon understands Docker patterns deeply. It can generate Dockerfiles from descriptions, diagnose why containers fail, and suggest optimizations you might not have considered. But as you'll see, AI suggestions often need refinement. Your job is to evaluate, challenge, and adapt Gordon's output to match your needs.

**Availability Note:**

Gordon requires Docker Desktop 4.50 or later (critical security update). If you're using an earlier version, you can apply the same optimization principles from previous lessons manually using the techniques from Lessons 3-6.

---

## Accessing Gordon in Docker Desktop

Gordon is embedded directly in Docker Desktop and Docker CLI. To enable it:

1. **Open Docker Desktop Settings**
   - Click the gear icon (Settings) in the top right
   - Navigate to **Features in development**

2. **Enable Docker AI**
   - Check **Enable Docker AI**
   - Accept the terms of service
   - Click **Apply & Restart**

3. **Verify Installation**
   ```bash
   docker version
   ```

   **Output:**
   ```
   Client: Docker Engine
     Version:           4.50.0
     ...
   ```

Once enabled, Gordon is available through:
- **Docker Desktop UI**: "Ask Gordon" panel on the right sidebar
- **Docker CLI**: `docker ai "your question"`

---

## Generating Dockerfiles with Gordon

Your first Dockerfile took 20 minutes to write. With Gordon, you can generate working Dockerfiles in seconds. But "working" isn't the same as "optimal." The real skill is asking clear questions that result in Dockerfiles matching your constraints.

### From Description to Dockerfile

Gordon works best when given precise constraints. Vague requests produce generic output. Specific requests produce tailored solutions.

**Your request to Gordon:**
```
Write a Dockerfile for a FastAPI application that:
- Uses Python 3.12 on Alpine Linux for minimal size
- Installs dependencies from requirements.txt
- Runs as non-root user (security requirement)
- Exposes port 8000
- Uses uvicorn to start the application
```

**Gordon generates:**
```dockerfile
FROM python:3.12-alpine

WORKDIR /app

# Create non-root user
RUN addgroup -S docker && adduser -S docker -G docker

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY --chown=docker:docker . .

USER docker

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0"]
```

**Output:**
```
Dockerfile created with:
- Alpine base (minimal size)
- Non-root user (docker)
- Layer caching optimization
- Security hardening
```

What improved: Gordon incorporated your explicit constraints (Alpine, non-root, port 8000) rather than generating generic boilerplate. Specific requirements → specific output.

### Evaluating Gordon's Suggestion

You receive the Dockerfile above. Before accepting it, ask critical evaluation questions:

1. **Does this match my requirements?**
   - Alpine base? ✓ Yes
   - Non-root user? ✓ Yes
   - Port 8000? ✓ Yes

2. **What assumptions did Gordon make?**
   - Requirements.txt exists in project root (verify your structure)
   - main.py with FastAPI app object (verify your filename)
   - Port 8000 is correct (your app might use 9000)

3. **What tradeoffs did Gordon accept?**
   - Alpine Linux is smaller but has some Python packages with compilation needs
   - Single-stage build is simple but uses full build dependencies in final image
   - No health check (Gordon prioritized simplicity)

The Dockerfile works, but it's not yet optimized for YOUR project. This is where refinement happens.

---

## Debugging Container Failures with Gordon

Containers exit unexpectedly. Build commands fail cryptically. The logs scroll past too fast to understand what happened. This is where Gordon shines—not as a code generator, but as a diagnostic expert.

### Container Crash Diagnosis

Your container exits immediately with code 137:

```bash
docker run my-ai-service
# Output: Error: exit code 137
```

You could search Stack Overflow for 30 minutes. Or you could ask:

**Your question to Gordon:**
```
docker ai "Why did my container exit with code 137? My app is a FastAPI
service that should stay running. Here's my Dockerfile: [paste Dockerfile]"
```

**Gordon analyzes and responds:**
```
Exit code 137 typically indicates:
1. Out-of-memory (OOM) - Most common cause
2. Container killed by Docker daemon
3. Ulimit exceeded

For a FastAPI service, I suspect OOM. When FastAPI loads model files
(if you're doing AI tasks), it can consume memory quickly.

Diagnostic steps:
1. Check memory: docker run -m 2g my-ai-service
2. Check logs: docker logs [container-id]
3. Monitor during startup: docker stats

If it still crashes, your model files might be too large for available RAM.
```

**Output:**
```
Gordon's diagnosis prompts action:
- Test with explicit memory limit
- Check logs for actual error message
- Monitor resource usage during startup
```

What happened: Gordon knew exit code 137 suggests resource constraints, not application bugs. This changed your debugging direction entirely—you tested memory allocation instead of searching for logic errors.

---

## Optimizing Existing Dockerfiles

You've written a working Dockerfile. It builds, runs, and contains your application. But "working" often means inefficient. Each layer adds size. Each redundant instruction slows builds. Gordon can spot these patterns.

### The Optimization Workflow

**Your Dockerfile (from Lesson 3):**
```dockerfile
FROM python:3.12

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

RUN python -m pytest tests/

CMD ["uvicorn", "main:app", "--host", "0.0.0.0"]
```

This Dockerfile works but has three inefficiencies:

1. **Large base image**: `python:3.12` includes build tools (300MB) needed only at build time
2. **Test layer in final image**: Tests run during build but don't belong in production container
3. **No multi-stage build**: Build dependencies ship with runtime image

**Your request to Gordon:**
```
Optimize this Dockerfile for production. I want:
- Smaller final image size (< 200MB)
- No test files in production image
- Fast builds during development
- Keep Python 3.12 requirement

Here's my current Dockerfile: [paste]
```

**Gordon suggests:**
```dockerfile
FROM python:3.12 as builder

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir --user -r requirements.txt

COPY . .

RUN python -m pytest tests/

# Production stage - minimal base image
FROM python:3.12-slim

WORKDIR /app

COPY --from=builder /root/.local /root/.local
COPY --from=builder /app/main.py .
COPY --from=builder /app/config.py .

ENV PATH=/root/.local/bin:$PATH

CMD ["uvicorn", "main:app", "--host", "0.0.0.0"]
```

**Output:**
```
Multi-stage Dockerfile reduces:
- Base image: 880MB (python:3.12) → 160MB (python:3.12-slim)
- Final image: ~400MB → ~180MB (55% reduction)
- Build time: 120s → 85s (faster layer reuse)
```

What emerged from this optimization:

- **Your understanding**: You now see why `python:3.12-slim` wasn't enough
- **Gordon's suggestion**: Multi-stage build separates concerns (build vs runtime)
- **Tradeoff revealed**: Manual file copying is verbose but explicit (reduces surprises)

### Evaluating the Optimization

The multi-stage Dockerfile is smaller. But is it right for YOUR project?

**Questions to ask:**

1. **Are my copied files complete?**
   - You manually listed `main.py` and `config.py`
   - What about `.env` files or data files?
   - If you use `COPY . .` (copy all), testing catches missing files. Explicit copies are safer but need maintenance.

2. **Does the size reduction justify the complexity?**
   - 400MB → 180MB is valuable for large deployments
   - But for local development, simpler might be better
   - Production benefit: Yes. Complexity cost: Moderate.

3. **What didn't Gordon consider?**
   - Your specific security requirements (Gordon suggested slim, not distroless)
   - Health check endpoints (Gordon skipped it)
   - Environment variable handling (Gordon didn't ask about configuration)

This is your opportunity to refine: Should you accept Gordon's suggestion as-is, or adapt it?

---

## Refining Gordon's Output Through Dialogue

Here's the key insight: Gordon's first suggestion is rarely the final answer. The best Dockerfiles emerge from iteration—you clarify requirements, Gordon adapts, you evaluate again.

### Round 1: The Optimization

Gordon suggested a multi-stage build. You have concerns about manually copying files.

**Your refinement request:**
```
The multi-stage build helps, but I have many data files and config directories
in my project. Manually listing files is error-prone. Can you revise to:

1. Use COPY . . in both build and runtime stages (explicit is less important
   than avoiding missing files)
2. Add a health check endpoint to the CMD
3. Use python:3.12-distroless instead of slim (even smaller and more secure)

Here's my updated project structure:
- main.py
- config/
  - settings.py
  - models/
- data/
  - embeddings/
  - cache/
```

**Gordon adapts:**
```dockerfile
FROM python:3.12 as builder

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir --user -r requirements.txt

COPY . .

RUN python -m pytest tests/

# Runtime stage
FROM python:3.12-distroless

WORKDIR /app

COPY --from=builder /root/.local /root/.local
COPY --from=builder /app .

ENV PATH=/root/.local/bin:$PATH

HEALTHCHECK --interval=30s --timeout=3s \
  CMD ["/usr/bin/curl", "-f", "http://localhost:8000/health"]

CMD ["uvicorn", "main:app", "--host", "0.0.0.0"]
```

**Output:**
```
Updated Dockerfile:
- Final image: 180MB → 140MB (distroless vs slim)
- Health check added (production readiness)
- COPY . . (includes all project files)
- Now includes: config/, data/, and your code
```

What changed through iteration:

- **Your constraint**: "I have many files and directories"
- **Gordon's adaptation**: Switched to full `COPY . .` to avoid manual maintenance
- **Tradeoff discovered**: Distroless is even smaller but lacks shell/curl
- **Health check added**: Gordon recognized production containers need liveness checks

### Round 2: The Health Check Problem

You test the optimized Dockerfile:

```bash
docker build -t my-ai-service .
docker run my-ai-service
```

**Output:**
```
Error: exec /usr/bin/curl not found
Health check failing
```

The distroless image doesn't have curl. Health checks fail. Gordon suggested something it didn't verify.

**Your correction:**
```
The distroless image doesn't have curl for health checks. I need to either:
1. Use a different health check method (not curl-based)
2. Use python:3.12-slim instead (has curl, larger but still optimized)

Which makes sense for an AI service that needs reliability over minimal size?
```

**Gordon refines:**
```dockerfile
FROM python:3.12 as builder

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir --user -r requirements.txt

COPY . .

RUN python -m pytest tests/

# Runtime stage - slim (has essential tools)
FROM python:3.12-slim

WORKDIR /app

COPY --from=builder /root/.local /root/.local
COPY --from=builder /app .

ENV PATH=/root/.local/bin:$PATH

# Health check using Python (available in slim image)
HEALTHCHECK --interval=30s --timeout=3s \
  CMD python -c "import requests; requests.get('http://localhost:8000/health')"

CMD ["uvicorn", "main:app", "--host", "0.0.0.0"]
```

**Output:**
```
Updated approach:
- Uses python:3.12-slim (reliable, includes common tools)
- Health check using Python (no external curl dependency)
- Size: 160MB (smaller than original, practical for reliability)
```

**What emerged from this dialogue:**

- **You taught Gordon**: Distroless breaks your health check requirements
- **Gordon adapted**: Switched to slim with Python-based health check
- **Convergence**: You found the sweet spot between size and reliability
- **No "wrong" answer**: Both approaches were valid. Your domain knowledge (need reliability) guided the choice.

---

## When to Override Gordon's Suggestions

Gordon is powerful. But there are moments when you should reject or substantially revise its suggestions.

### Pattern 1: Gordon Prioritizes Simplicity Over Your Requirements

**Example:**
```
Gordon suggests a single-stage Dockerfile because it's simpler. But you need:
- 50MB image size (regulatory compliance)
- No build tools in final image (security audit)
- Fast rebuilds during development

Gordon's simple suggestion doesn't meet these constraints. Override it with
multi-stage approach even though it's more complex.
```

**Decision rule:** If Gordon's suggestion violates your explicit requirements (even if simpler), adapt it.

### Pattern 2: Gordon Makes Assumptions About Your Project

**Example:**
```
You ask: "Generate a Dockerfile for my FastAPI service"

Gordon assumes:
- Single app.py file in root
- Standard requirements.txt
- Port 8000
- No special environment variables

Your project actually has:
- Modular structure (api/, models/, utils/ directories)
- Multiple dependency files (requirements-base.txt, requirements-dev.txt)
- Custom port configuration via ENV_PORT
- Secret API keys that need injecting

Gordon's generic suggestion won't work until you provide this context.
```

**Decision rule:** Verify Gordon's assumptions about your project structure before accepting its output.

### Pattern 3: Gordon Suggests Something Untested

**Example:**
```
Gordon suggests using a new, smaller base image you haven't tested:
FROM python:3.12-alpine-micro

It's 40MB smaller. But you don't know if it includes required system libraries
for your NumPy/PyTorch dependencies. If build fails in production, the cost
of debugging far exceeds the 40MB savings.
```

**Decision rule:** For production, use approaches you've tested. Novel suggestions need validation before deployment.

### Pattern 4: Gordon Misses Context About Your Team

**Example:**
```
Gordon suggests a highly optimized but complex multi-stage Dockerfile that
only you understand. Your team needs to modify it next month.

Better choice: Simpler Dockerfile that your team can understand and maintain,
even if slightly less optimized.
```

**Decision rule:** Account for your team's expertise. Optimizations should be worth the maintenance burden.

---

## Try With AI: Collaborative Docker Optimization

You have a working but inefficient Dockerfile. Your goal: Work with Gordon to produce a smaller, more secure image while maintaining clarity for your team.

### Part 1: Establish Your Constraints

Before asking Gordon for optimization, define what matters:

- **Size target**: How small does the image need to be? (100MB? 200MB? Size doesn't matter?)
- **Security requirements**: Non-root user? No build tools? Minimal base image?
- **Maintainability**: Is it okay if only you understand it, or does your team need to modify it?
- **Build speed**: During development, do fast rebuilds matter?

Write these down. Gordon's suggestions will be better when constraints are explicit.

### Part 2: Ask Gordon for Initial Optimization

**Your prompt to Gordon:**
```
Here's my current Dockerfile (for a FastAPI agent service using Anthropic API):

FROM python:3.12

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

CMD ["uvicorn", "main:app", "--host", "0.0.0.0"]

I want to optimize for:
- Smaller image size (target < 200MB)
- Running as non-root user
- Fast rebuilds during development
- Maintainable for a team of 3-4 developers

Here's my project structure:
- main.py (FastAPI entry point)
- agents/ (subdirectory with agent code)
- config/ (settings files)
- requirements.txt (dependencies)
- tests/ (unit tests)

Suggest an optimized approach.
```

Review Gordon's response. Ask yourself:

- Does it meet all four constraints?
- What assumptions did it make about your project?
- What's still unclear?

### Part 3: Evaluate and Refine

If Gordon's suggestion has issues, tell it:

**Your refinement:**
```
The multi-stage build helps, but I have a concern: My agents/ directory
includes large model files (512MB total) that take time to copy. The current
Dockerfile copies everything repeatedly during rebuilds.

Also, I want a health check endpoint. Can you suggest how to add one without
bloating the image?

Should I:
1. Use a .dockerignore to exclude model files during development?
2. Mount model files as a volume instead of copying them?
3. Something else?

What's the tradeoff between each approach?
```

### Part 4: Test the Final Version

**Create the Dockerfile:**
```bash
# Use Gordon's final suggestion (as text in Docker Desktop)
cat > Dockerfile << 'EOF'
[Gordon's optimized Dockerfile here]
EOF
```

**Build and test:**
```bash
docker build -t my-agent-service .
docker run --rm my-agent-service
```

**Output:**
```
Successfully built my-agent-service
Container runs and responds to requests
Image size: 185MB (under your 200MB target)
```

### Part 5: Reflection

Compare your initial Dockerfile to the final version:

- What size reduction did you achieve?
- What improvements would you not have discovered manually?
- What suggestions did you reject or adapt? Why?
- Would your team understand this Dockerfile?

The goal isn't the smallest possible image. The goal is collaborating with Gordon to find the best tradeoff for YOUR context.

Safety note: When building containers for AI services that use API keys or secrets, never hardcode them in Dockerfiles. Use environment variables passed at runtime or Docker secrets for deployed containers (Chapter 50 covers this in detail).
