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
- **Push to registries**: Docker Hub, GitHub Container Registry, and cloud registries

## Chapter Structure

1. **Container Fundamentals** — Images vs containers, layers, and the build process
2. **Writing Your First Dockerfile** — Base images, COPY, RUN, and CMD for Python services
3. **Multi-Stage Builds** — Separate build and runtime stages for smaller images
4. **Dependency Management** — Python requirements, UV, and handling large packages
5. **Docker Compose for Development** — Multi-container local environments
6. **Security & Best Practices** — Non-root users, secrets, and vulnerability scanning
7. **AI-Assisted Docker with Gordon** — Using Docker's AI assistant for common tasks
8. **Capstone: Containerized Agent** — Package your Part 6 agent as a production-ready container

## Prerequisites

- Part 6: A working FastAPI agent service to containerize
- Basic command-line familiarity
- Docker Desktop installed

## Looking Ahead

This chapter produces a container image. Chapter 50 (Kubernetes) deploys that container to an orchestrated cluster, and Chapter 51 (Helm) packages it for repeatable deployments.
