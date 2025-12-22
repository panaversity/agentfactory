---
sidebar_position: 55
title: "Chapter 55: CI/CD Pipelines & GitOps with ArgoCD"
description: "Automate builds, tests, and deployments with GitHub Actions and ArgoCD"
---

# Chapter 55: CI/CD Pipelines & GitOps with ArgoCD

You've been deploying manually with kubectl and helm. That works for learning but not for teams. Every deployment should be automated, tested, and auditable. CI/CD (Continuous Integration/Continuous Deployment) automates the path from code commit to production.

This chapter teaches two complementary practices: CI pipelines with GitHub Actions (build, test, push images) and GitOps with ArgoCD (deploy by committing to a Git repo). Together, they create a fully automated, auditable deployment pipeline.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Design CI/CD pipelines**: Stages, jobs, and the build-test-deploy flow
- **Implement GitHub Actions**: Workflows for Python/Node projects
- **Build and push images**: Automated Docker builds on every commit
- **Run tests in CI**: Unit tests, integration tests, and quality gates
- **Understand GitOps principles**: Git as single source of truth for deployments
- **Deploy ArgoCD**: Installation and configuration on Kubernetes
- **Create ArgoCD Applications**: Sync Helm charts from Git to cluster
- **Implement deployment strategies**: Blue-green, canary, and progressive rollouts

## Chapter Structure

1. **CI/CD Concepts** — Continuous Integration, Delivery, and Deployment
2. **GitHub Actions Fundamentals** — Workflows, jobs, steps, and triggers
3. **Building Docker Images in CI** — Multi-platform builds and registry push
4. **Testing in CI** — Running tests, coverage, and quality gates
5. **GitOps Principles** — Declarative deployments and Git as truth
6. **ArgoCD Installation & Setup** — Deploying ArgoCD on Minikube
7. **ArgoCD Applications** — Syncing Helm charts and monitoring drift
8. **Deployment Strategies** — Rollouts, rollbacks, and progressive delivery
9. **Capstone: Automated Pipeline** — Full CI/CD from commit to deployed agent

## Prerequisites

- Chapter 51: Helm Charts (what ArgoCD deploys)
- Chapter 50: Kubernetes (where ArgoCD deploys)
- GitHub repository for your agent project

## Looking Ahead

Deployments are automated. The remaining chapters focus on operations: observability (Chapter 56), traffic management (Chapter 57), security (Chapter 58), and infrastructure provisioning (Chapter 59).
