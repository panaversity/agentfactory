"""
Course: Cloud Native AI: Docker, Kubernetes & DAPR for Agent Systems
Code: AI-400
Generated: 2025-11-10T00:00:00Z

Database-ready course definition.
Constitution: v3.1.2
"""

from datetime import datetime
from typing import Dict, List, Any


# Course metadata
COURSE_CODE = "AI-400"
COURSE_NAME = "Cloud Native AI: Docker, Kubernetes & DAPR for Agent Systems"
COURSE_INITIALS = "CNAI"


# Course definition (database-ready)
course: Dict[str, Any] = {
    "course_code": "AI-400",
    "course_name": "Cloud Native AI: Docker, Kubernetes & DAPR for Agent Systems",
    "course_initials": "CNAI",
    "course_description": """Master cloud-native infrastructure for AI agent systems through Context Engineering and Claude Code partnership. Learn Spec-Driven Development (SDD) to design Docker containers, Kubernetes orchestrations, and Dapr workflows—all generated from specifications. Deploy production-ready agent applications with observability, scaling, and cloud-agnostic patterns.""",
    "created_by": "db_admin",
    "updated_by": "db_admin",
    "course_outcomes": [
        "Apply Context Engineering to structure effective AI collaboration for infrastructure design",
        "Partner with Claude Code to generate production-ready cloud configurations from specifications",
        "Master Spec-Driven Development (SDD) to design infrastructure through clear intent, not manual YAML",
        "Containerize AI applications with Docker using AIDD and SDD for multi-stage builds and optimization",
        "Orchestrate agent systems on Kubernetes with AIDD and SDD using kubectl-ai and kagent",
        "Implement Dapr Core and Dapr Workflows for cloud-agnostic state, pub/sub, and long-running processes",
        "Build observable, scalable AI systems with OpenTelemetry, autoscaling, and automated CI/CD pipelines"
    ],
    "long_description": """The era of local-only AI development is over. Production deployment isn't about memorizing YAML syntax—it's about mastering Context Engineering to communicate infrastructure intent and partnering with Claude Code to generate Docker, Kubernetes, and Dapr configurations. Cloud Native AI represents the methodology shift from "manual DevOps" to "specification-driven infrastructure," where you design systems through clear specifications and AI handles implementation complexity.

This course grounds you in six essential pillars: Context Engineering for structuring AI collaboration, Claude Code as your infrastructure partner, Spec-Driven Development (SDD) for designing deployments, Docker with AIDD and SDD for containerization, Kubernetes with AIDD and SDD for orchestration (including kubectl-ai and kagent), and Dapr with workflows for cloud-agnostic communication and long-running processes. You'll learn to specify infrastructure requirements and let AI generate production-ready configurations—multi-stage Docker builds, Kubernetes manifests, Dapr components—while you focus on system design and validation.

You'll start with Context Engineering fundamentals, structuring infrastructure specifications for AI collaboration. Then Docker with SDD, containerizing FastAPI services through specifications. Next, Kubernetes with kubectl-ai and kagent, orchestrating agent systems while AI handles manifest complexity. You'll master Dapr Core and workflows for state management, pub/sub messaging, and durable execution patterns. Finally, you'll implement observability with OpenTelemetry, autoscaling, and CI/CD pipelines—all specification-driven.

By the end, you won't manually write Dockerfiles or Kubernetes YAML—you'll specify system requirements and validate AI-generated infrastructure. You'll deploy agent systems that scale across clouds, coordinate through Dapr abstractions, and adapt to demand automatically. This is professional cloud-native thinking: design through specifications, generate with AI, validate with confidence.""",
    "learning_modules": [
        {
            "module_id": 1,
            "module_name": "Foundations: Cloud Native Infrastructure for AI",
            "module_description": "Master Context Engineering to structure AI collaboration for infrastructure design. Partner with Claude Code to understand containerization (Docker), orchestration (Kubernetes), and cloud-agnostic abstractions (Dapr). Learn Spec-Driven Development (SDD) fundamentals: write specifications, AI generates infrastructure, you validate. Establish professional thinking patterns for production deployment, not manual configuration."
        },
        {
            "module_id": 2,
            "module_name": "Docker Fundamentals: Containerizing AI Applications",
            "module_description": "Containerize FastAPI services using Docker with AIDD and SDD. Specify requirements—multi-stage builds, Python dependencies, layer optimization—and Claude Code generates production Dockerfiles. Master container networking, health checks, and Docker Compose for local development. Focus on specification and validation, not Dockerfile syntax memorization."
        },
        {
            "module_id": 3,
            "module_name": "Kubernetes Basics: Orchestrating Agent Systems",
            "module_description": "Orchestrate agent systems on Kubernetes with AIDD and SDD using kubectl-ai and kagent. Specify deployment requirements—pods, services, ConfigMaps, StatefulSets—and Claude Code generates manifests. Master Kubernetes primitives through specifications, event-driven patterns with Kafka, and production-grade configurations while AI handles YAML complexity."
        },
        {
            "module_id": 4,
            "module_name": "DAPR Core: Cloud-Agnostic Abstractions",
            "module_description": "Implement Dapr Core and Dapr Workflows for cloud-agnostic communication and long-running processes. Specify requirements—state stores, pub/sub, service invocation, durable workflows—and Claude Code generates Dapr components. Master cloud-portable patterns: state works with any database, pub/sub with any broker, workflows for multi-step agent orchestration. Write once, deploy anywhere."
        },
        {
            "module_id": 5,
            "module_name": "Production Operations: Observability, Scaling & CI/CD",
            "module_description": "Build production-ready AI systems using SDD for operations and monitoring. Specify observability requirements—OpenTelemetry traces, metrics, cost dashboards—and Claude Code generates telemetry configurations. Master autoscaling, CI/CD with Testcontainers and GitHub Actions, Infrastructure-as-Code with Terraform. Design through specifications, validate with confidence, operate at scale."
        }
    ],
    "pre_requisite": [
        "AI-300 or equivalent (AI-Driven Development with Python and Agentic AI)",
        "Database fundamentals (Part 10: PostgreSQL, Graph, Vector databases)",
        "Basic command-line and terminal proficiency",
        "Understanding of client-server architecture"
    ],
    "media_link": "https://i.postimg.cc/XYLz3tSB/course-2.webp"
}


def get_course_dict() -> Dict[str, Any]:
    """Return complete course dictionary for database insertion."""
    return course.copy()


def get_course_code() -> str:
    """Return course code."""
    return course["course_code"]


def get_course_name() -> str:
    """Return course name."""
    return course["course_name"]


def get_course_outcomes() -> List[str]:
    """Return list of course learning outcomes."""
    return course["course_outcomes"].copy()


def get_learning_modules() -> List[Dict[str, Any]]:
    """Return list of learning modules."""
    return [m.copy() for m in course["learning_modules"]]


def get_prerequisites() -> List[str]:
    """Return list of prerequisites."""
    return course["pre_requisite"].copy()


def validate_course() -> Dict[str, bool]:
    """
    Validate course data structure.

    Returns:
        Dictionary with validation results
    """
    return {
        "has_code": bool(course.get("course_code")),
        "has_name": bool(course.get("course_name")),
        "has_outcomes": len(course.get("course_outcomes", [])) >= 4,
        "has_modules": len(course.get("learning_modules", [])) >= 4,
        "outcomes_are_list": isinstance(course.get("course_outcomes"), list),
        "modules_are_list": isinstance(course.get("learning_modules"), list),
    }


if __name__ == "__main__":
    print(f"Course: {COURSE_NAME}")
    print(f"Code: {COURSE_CODE}")
    print(f"Outcomes: {len(get_course_outcomes())} learning outcomes")
    print(f"Modules: {len(get_learning_modules())} learning modules")
    print(f"Prerequisites: {len(get_prerequisites())} items")
    print("\nValidation:", validate_course())
    print("\n✅ Course data ready for database insertion")
