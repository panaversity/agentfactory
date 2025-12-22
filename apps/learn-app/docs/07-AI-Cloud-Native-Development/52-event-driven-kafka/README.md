---
sidebar_position: 52
title: "Chapter 52: Event-Driven Architecture with Kafka"
description: "Build decoupled, scalable agent systems with Apache Kafka event streaming"
---

# Chapter 52: Event-Driven Architecture with Kafka

Request-response APIs work for simple interactions. But production agent systems need decoupling—when a task is created, a notification service should be triggered, an audit log should be written, and a recurring task engine should be notified. If these are direct API calls, one slow service blocks everything. If they're events on Kafka, each service consumes independently.

This chapter teaches Kafka fundamentals with a focus on agent communication patterns. You'll learn core concepts (topics, partitions, consumers), implement producers and consumers in Python, and design event-driven architectures for your agent services.

Understanding Kafka's raw concepts matters even if you later use Dapr's abstraction (Chapter 53-54). When something breaks in production, you need to debug the actual system, not just the abstraction.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand Kafka architecture**: Brokers, topics, partitions, and consumer groups
- **Explain message semantics**: At-least-once, at-most-once, and exactly-once delivery
- **Implement producers**: Publish events from your agent service
- **Implement consumers**: Subscribe to topics and process events
- **Design event schemas**: Message formats, versioning, and Schema Registry basics
- **Apply agent patterns**: Task events, notifications, audit logs, and real-time sync
- **Deploy Kafka locally**: Kafka on Docker/Minikube with Strimzi or Redpanda
- **Debug Kafka issues**: Consumer lag, rebalancing, and common problems

## Chapter Structure

1. **Event-Driven Architecture Concepts** — Why events? Coupling, scalability, and resilience
2. **Kafka Architecture** — Brokers, topics, partitions, and replication
3. **Producers Deep Dive** — Publishing events, partitioning strategies, and error handling
4. **Consumers Deep Dive** — Consumer groups, offsets, and rebalancing
5. **Message Schemas** — JSON, Avro, and schema evolution
6. **Agent Communication Patterns** — Task events, notifications, audit logs, real-time sync
7. **Local Kafka Setup** — Docker Compose and Strimzi operator on Minikube
8. **Capstone: Event-Driven Agent** — Add Kafka events to your agent for task notifications and audit logging

## Prerequisites

- Chapter 51: Helm Charts (for deploying Kafka)
- Python fundamentals (async patterns)
- Your agent service from Part 6

## Looking Ahead

This chapter teaches raw Kafka. Chapter 53 (Dapr) shows how to abstract Kafka behind Dapr's pub/sub, making your code portable across message brokers while retaining the concepts you learned here.
