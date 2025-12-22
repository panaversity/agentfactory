---
sidebar_position: 50
title: "Chapter 50: Kubernetes with Minikube"
description: "Deploy and orchestrate AI agents on local Kubernetes clusters"
---

# Chapter 50: Kubernetes with Minikube

Docker gives you portable containers. Kubernetes orchestrates them—handling deployment, scaling, networking, and self-healing automatically. When your agent container crashes, Kubernetes restarts it. When traffic spikes, Kubernetes scales it. When you push a new version, Kubernetes rolls it out safely.

This chapter teaches Kubernetes fundamentals using Minikube, a local K8s cluster that runs on your laptop. You'll learn the core concepts (pods, deployments, services) and practice with your containerized agent before moving to cloud clusters.

You'll also learn to use kubectl-ai for AI-assisted Kubernetes operations—generating manifests, debugging issues, and explaining cluster state with natural language.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand K8s architecture**: Control plane, nodes, and the declarative model
- **Deploy containers**: Pods, Deployments, and ReplicaSets
- **Expose services**: ClusterIP, NodePort, LoadBalancer, and Ingress
- **Configure applications**: ConfigMaps, Secrets, and environment variables
- **Manage resources**: CPU/memory requests and limits
- **Debug clusters**: kubectl commands, logs, and troubleshooting patterns
- **Use kubectl-ai**: AI-assisted manifest generation and cluster operations
- **Set up Minikube**: Local K8s cluster for development and learning

## Chapter Structure

1. **Kubernetes Architecture** — Control plane, worker nodes, and the API server
2. **Setting Up Minikube** — Installation, cluster creation, and dashboard access
3. **Pods & Deployments** — Running containers, replicas, and rollout strategies
4. **Services & Networking** — Exposing applications, DNS, and service discovery
5. **Configuration Management** — ConfigMaps, Secrets, and environment injection
6. **Resource Management** — Requests, limits, and quality of service
7. **Debugging & Troubleshooting** — Logs, events, exec, and common issues
8. **AI-Assisted K8s with kubectl-ai** — Natural language to manifests and cluster queries
9. **Capstone: Deploy Your Agent** — Run your containerized agent on Minikube with proper configuration

## Prerequisites

- Chapter 49: A containerized agent (Docker image)
- Docker Desktop running
- Basic YAML familiarity

## Looking Ahead

This chapter deploys containers manually with kubectl. Chapter 51 (Helm) packages deployments for repeatable, versioned releases across environments.
