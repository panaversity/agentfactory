---
sidebar_position: 49
title: "Chapter 49: Docker for AI Services"
description: "Containerize AI agents with Docker for portable, reproducible deployments"
---

# Chapter 49: Docker for AI Services

Your agent runs perfectly on your machine. But "works on my machine" doesn't ship products. Docker solves this by packaging your agent, its dependencies, and its runtime environment into a portable container that runs identically everywhere—your laptop, a teammate's machine, or a cloud server.

This chapter teaches containerization from first principles, with a focus on AI service patterns. You'll learn to write efficient Dockerfiles, optimize image sizes, handle Python/Node dependencies correctly, and use AI-assisted workflows with Gordon (Docker's AI assistant) to accelerate common tasks.

By the end, your FastAPI agent service from Part 6 will be a portable container ready for Kubernetes deployment.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand container fundamentals**: Images, containers, layers, and the Docker runtime
- **Write production Dockerfiles**: Multi-stage builds, layer caching, and size optimization
- **Handle AI service dependencies**: Python packages, model files, and environment configuration
- **Use Docker Compose**: Multi-container setups for local development (agent + database + redis)
- **Apply security best practices**: Non-root users, minimal base images, and secret handling
- **Leverage Gordon (Docker AI)**: AI-assisted Docker operations for common workflows
- **Create reusable skills**: Transform Docker expertise into organizational intelligence
- **Push to registries**: Docker Hub, GitHub Container Registry, and cloud registries

## Chapter Structure

| Lesson | Title | Layer | Focus |
|--------|-------|-------|-------|
| 1 | Docker Installation & Setup | L1 (Manual) | Platform setup, Docker Desktop, resource configuration |
| 2 | Container Fundamentals | L1 (Manual) | Images vs containers, Docker Hub, layers, lifecycle |
| 3 | Writing Your First Dockerfile | L1 (Manual) | FROM, WORKDIR, COPY, RUN, CMD, layer caching |
| 4 | Container Lifecycle & Debugging | L1 (Manual) | Logs, exec, inspect, port conflicts, restart policies |
| 5 | Multi-Stage Builds & Optimization | L1 (Manual) | Build vs runtime stages, UV, slim/alpine/distroless |
| 6 | Docker Compose for Development | L1 (Manual) | Multi-service orchestration, networks, volumes |
| 7 | Security & Best Practices | L1 (Manual) | Non-root users, Docker Scout, DHI, secrets |
| 8 | AI-Assisted Docker with Gordon | L2 (Collaboration) | Gordon workflows for generation, debugging, optimization |
| 9 | Capstone: Production-Ready Agent | L4 (Spec-Driven) | Specification-first containerization of FastAPI agent |
| 10 | Building the Production Dockerfile Skill | L3 (Intelligence) | Persona + Questions + Principles skill design |

## 4-Layer Teaching Progression

This chapter follows the **4-Layer Teaching Method**:

- **Lessons 1-7 (Layer 1)**: Build mental models manually before AI assistance
- **Lesson 8 (Layer 2)**: Collaborate with Gordon using Three Roles (invisible framework)
- **Lesson 9 (Layer 4)**: Apply all lessons in spec-driven capstone project
- **Lesson 10 (Layer 3)**: Create reusable intelligence that compounds across projects

## Prerequisites

- Part 6: A working FastAPI agent service to containerize
- Basic command-line familiarity
- Docker Desktop installed (Lesson 1 covers installation)

## Looking Ahead

This chapter produces a container image and a reusable Dockerfile skill. Chapter 50 (Kubernetes) deploys that container to an orchestrated cluster, and Chapter 51 (Helm) packages it for repeatable deployments.
