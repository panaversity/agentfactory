---
sidebar_position: 49
title: "Chapter 49: Docker for AI Services"
description: "Containerize AI agents with Docker for portable, reproducible deployments"
---

# Chapter 49: Docker for AI Services

Your agent runs perfectly on your machine. But "works on my machine" doesn't ship products. Docker solves this by packaging your agent, its dependencies, and its runtime environment into a portable container that runs identically everywhere—your laptop, a teammate's machine, or a cloud server.

This chapter teaches containerization from first principles, with a focus on AI service patterns. Using the **Task API from Chapter 40** as your running example, you'll learn to write efficient Dockerfiles, optimize image sizes with multi-stage builds and UV, harden containers for production, and create a reusable skill that encodes your Docker expertise.

By the end, your FastAPI Task API will be a production-ready container pushed to a registry and ready for Kubernetes deployment.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand container fundamentals**: Images, containers, layers, and the Docker runtime
- **Write production Dockerfiles**: Multi-stage builds, UV package manager, layer caching, and size optimization
- **Debug containers effectively**: Logs, exec, inspect, port conflicts, and restart policies
- **Harden for production**: Environment variables, health checks, non-root users
- **Create reusable intelligence**: Transform Docker expertise into a skill that compounds across projects
- **Apply spec-driven workflow**: Write specifications before code for containerization projects

## Chapter Structure

| Lesson | Title | Layer | Focus |
|--------|-------|-------|-------|
| 1 | Docker Installation & Setup | L1 (Manual) | Platform setup, Docker Desktop, resource configuration |
| 2 | Container Fundamentals | L1 (Manual) | Images vs containers, Docker Hub, layers, lifecycle |
| 3 | Writing Your First Dockerfile | L1 (Manual) | Containerize Task API with FROM, WORKDIR, COPY, RUN, CMD |
| 4 | Container Lifecycle & Debugging | L1 (Manual) | Logs, exec, inspect, port conflicts, restart policies |
| 5 | Multi-Stage Builds & Optimization | L1 (Manual) | Build vs runtime stages, UV, slim/alpine, 70%+ size reduction |
| 6 | Production Hardening | L1 (Manual) | Environment variables, health checks, non-root users |
| 7 | Docker Image Builder Skill | L3 (Intelligence) | Create reusable Persona + Questions + Principles skill |
| 8 | Capstone: Containerize Your API | L4 (Spec-Driven) | Specification-first containerization of SQLModel Task API |

**Total Duration**: 6 hours (360 minutes)

## 4-Layer Teaching Progression

This chapter follows the **4-Layer Teaching Method**:

- **Lessons 1-6 (Layer 1)**: Build mental models manually—understand Docker before AI assists
- **Lesson 7 (Layer 3)**: Create reusable intelligence that compounds across future projects
- **Lesson 8 (Layer 4)**: Apply all lessons in spec-driven capstone project

## Running Example: Task API from Chapter 40

Throughout this chapter, you containerize the **In-Memory Task API** from Chapter 40 Lessons 1-5:

```python
# main.py - In-Memory Task API
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

app = FastAPI(title="Task API")

class Task(BaseModel):
    id: int | None = None
    title: str
    completed: bool = False

tasks: list[Task] = []
next_id = 1

@app.post("/tasks", response_model=Task)
def create_task(task: Task) -> Task:
    global next_id
    task.id = next_id
    next_id += 1
    tasks.append(task)
    return task

@app.get("/tasks", response_model=list[Task])
def list_tasks() -> list[Task]:
    return tasks

@app.get("/health")
def health_check() -> dict:
    return {"status": "healthy"}
```

This same API evolves through each lesson:
- **Lessons 3-5**: Containerize, debug, and optimize the in-memory version
- **Lesson 6**: Add production hardening (health checks, non-root user)
- **Lesson 7**: Encode patterns into reusable skill
- **Lesson 8 (Capstone)**: Containerize the **SQLModel + Neon version** from Chapter 40 Lesson 7

## Prerequisites

- **Chapter 40 completion**: The Task API exists—this is what you'll containerize
- **Part 6 completion**: FastAPI and Python fundamentals
- **Basic command-line familiarity**: Comfortable running terminal commands
- **No Docker experience required**: Lesson 1 covers installation from scratch

## Looking Ahead

This chapter produces:
1. A **production-ready container image** pushed to Docker Hub or GitHub Container Registry
2. A **reusable production-dockerfile skill** for future containerization work

Chapter 50 (Kubernetes) deploys your container to an orchestrated cluster, and Chapter 51 (Helm) packages it for repeatable deployments.
