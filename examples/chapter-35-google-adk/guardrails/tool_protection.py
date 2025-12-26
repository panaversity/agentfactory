"""
Tool Protection with before_tool_callback

Demonstrates patterns for validating and blocking tool calls BEFORE
execution. This is your second line of defense, protecting critical
operations and resources.

Key Pattern:
- Return Dict to BLOCK the tool call (returned as tool result)
- Return None to ALLOW the tool to execute normally

Security Rationale:
Tool callbacks prevent unauthorized access to sensitive operations,
validate arguments before execution, and enforce role-based access.
"""

import re
from typing import Optional, Dict, Any, Set

from google.adk.tools.base_tool import BaseTool
from google.adk.tools.tool_context import ToolContext


# =============================================================================
# Pattern 1: Protect Critical Resources (Admin Tools)
# =============================================================================

# Tools that require elevated privileges
ADMIN_TOOLS = {"delete_user", "reset_database", "modify_permissions", "delete_task"}
DANGEROUS_OPERATIONS = {"delete", "drop", "truncate", "remove_all"}


def protect_admin_tools(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext,
    admin_users: Set[str] | None = None
) -> Optional[Dict]:
    """
    Block non-admin users from executing admin-only tools.

    Security: Enforces principle of least privilege by restricting
    dangerous operations to authorized users only.

    Args:
        tool: The tool being called
        args: Arguments passed to the tool
        tool_context: Context with session state and user info
        admin_users: Set of user IDs with admin privileges

    Returns:
        Dict with error if blocked, None if allowed
    """
    if admin_users is None:
        admin_users = {"admin", "superuser"}

    # Check if this is an admin tool
    if tool.name not in ADMIN_TOOLS:
        return None  # Not an admin tool, allow

    # Get user from session state (set during authentication)
    current_user = tool_context.state.get("user_id", "anonymous")
    user_role = tool_context.state.get("user_role", "guest")

    # Check if user has admin privileges
    if current_user in admin_users or user_role == "admin":
        return None  # Admin user, allow

    # Block non-admin users
    return {
        "status": "error",
        "code": "UNAUTHORIZED",
        "message": f"Tool '{tool.name}' requires admin privileges. "
                   f"Current user '{current_user}' is not authorized.",
        "allowed_roles": ["admin"]
    }


# =============================================================================
# Pattern 2: Validate Tool Arguments
# =============================================================================

# Validation rules per tool
ARGUMENT_RULES = {
    "send_email": {
        "to": {
            "type": str,
            "required": True,
            "pattern": r"^[^@]+@[^@]+\.[^@]+$",  # Basic email pattern
            "blocked_domains": ["competitor.com", "spam.com"],
        },
        "subject": {
            "type": str,
            "required": True,
            "max_length": 200,
        },
        "body": {
            "type": str,
            "required": True,
            "max_length": 10000,
        }
    },
    "execute_query": {
        "query": {
            "type": str,
            "required": True,
            "blocked_patterns": [
                r"(?i)drop\s+table",
                r"(?i)delete\s+from.*where\s*1\s*=\s*1",
                r"(?i)truncate",
                r";\s*--",  # SQL injection pattern
            ]
        }
    },
    "file_write": {
        "path": {
            "type": str,
            "required": True,
            "allowed_prefixes": ["/tmp/", "/data/output/"],
            "blocked_patterns": [r"\.\."],  # Path traversal
        }
    }
}


def validate_tool_arguments(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext
) -> Optional[Dict]:
    """
    Validate tool arguments against defined rules before execution.

    Security: Prevents injection attacks, ensures data integrity,
    and enforces business rules on tool inputs.

    Args:
        tool: The tool being called
        args: Arguments to validate
        tool_context: Context (not used but required by signature)

    Returns:
        Dict with validation error if invalid, None if valid
    """
    rules = ARGUMENT_RULES.get(tool.name)
    if not rules:
        return None  # No rules defined, allow

    errors = []

    for arg_name, arg_rules in rules.items():
        value = args.get(arg_name)

        # Check required
        if arg_rules.get("required") and value is None:
            errors.append(f"Missing required argument: {arg_name}")
            continue

        if value is None:
            continue

        # Check type
        expected_type = arg_rules.get("type")
        if expected_type and not isinstance(value, expected_type):
            errors.append(f"Invalid type for {arg_name}: expected {expected_type.__name__}")
            continue

        # String-specific validations
        if isinstance(value, str):
            # Check max length
            max_length = arg_rules.get("max_length")
            if max_length and len(value) > max_length:
                errors.append(f"{arg_name} exceeds max length of {max_length}")

            # Check pattern
            pattern = arg_rules.get("pattern")
            if pattern and not re.match(pattern, value):
                errors.append(f"{arg_name} does not match required pattern")

            # Check blocked patterns
            for blocked in arg_rules.get("blocked_patterns", []):
                if re.search(blocked, value):
                    errors.append(f"{arg_name} contains blocked pattern (potential injection)")

            # Check blocked domains (for emails)
            blocked_domains = arg_rules.get("blocked_domains", [])
            for domain in blocked_domains:
                if domain in value.lower():
                    errors.append(f"{arg_name} contains blocked domain: {domain}")

            # Check allowed prefixes (for paths)
            allowed_prefixes = arg_rules.get("allowed_prefixes")
            if allowed_prefixes:
                if not any(value.startswith(prefix) for prefix in allowed_prefixes):
                    errors.append(f"{arg_name} must start with one of: {allowed_prefixes}")

    if errors:
        return {
            "status": "error",
            "code": "VALIDATION_ERROR",
            "message": "Argument validation failed",
            "errors": errors
        }

    return None  # All validations passed


# =============================================================================
# Pattern 3: Restrict Tools by User Role
# =============================================================================

# Role-based access control for tools
ROLE_PERMISSIONS = {
    "guest": {"list_items", "get_info", "search"},
    "user": {"list_items", "get_info", "search", "create_item", "update_item"},
    "moderator": {"list_items", "get_info", "search", "create_item", "update_item",
                  "flag_content", "review_reports"},
    "admin": {"*"},  # All tools
}


def restrict_tools_by_role(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext,
    role_permissions: Dict[str, Set[str]] | None = None
) -> Optional[Dict]:
    """
    Restrict tool access based on user role.

    Security: Implements role-based access control (RBAC) to ensure
    users can only access tools appropriate for their privilege level.

    Args:
        tool: The tool being called
        args: Tool arguments (not used but required)
        tool_context: Context with session state containing user role
        role_permissions: Custom role->tools mapping (optional)

    Returns:
        Dict with error if unauthorized, None if allowed
    """
    if role_permissions is None:
        role_permissions = ROLE_PERMISSIONS

    # Get user role from session state
    user_role = tool_context.state.get("user_role", "guest")

    # Get allowed tools for this role
    allowed_tools = role_permissions.get(user_role, set())

    # Admin can access everything
    if "*" in allowed_tools:
        return None

    # Check if tool is allowed
    if tool.name in allowed_tools:
        return None

    return {
        "status": "error",
        "code": "FORBIDDEN",
        "message": f"Role '{user_role}' is not authorized to use tool '{tool.name}'",
        "required_roles": [
            role for role, tools in role_permissions.items()
            if tool.name in tools or "*" in tools
        ]
    }


# =============================================================================
# Pattern 4: Selective Tool Blocking (Inspect and Decide)
# =============================================================================

def inspect_and_block_selective(
    tool: BaseTool,
    args: Dict[str, Any],
    tool_context: ToolContext
) -> Optional[Dict]:
    """
    Demonstrates how to inspect tool_call details and block selectively.

    This pattern is useful when blocking logic depends on:
    - Combination of tool + specific arguments
    - Dynamic conditions (time, load, feature flags)
    - Complex business rules

    Example scenarios blocked:
    - Deleting tasks with high priority
    - Sending emails outside business hours
    - Accessing premium features without subscription
    """

    # Scenario 1: Block deletion of high-priority items
    if tool.name == "delete_task":
        task_id = args.get("task_id")
        # In real code, you'd look up the task
        high_priority_tasks = tool_context.state.get("high_priority_tasks", [])
        if task_id in high_priority_tasks:
            return {
                "status": "error",
                "code": "PROTECTED_RESOURCE",
                "message": f"Cannot delete high-priority task {task_id}. "
                          f"Please lower priority first or contact admin."
            }

    # Scenario 2: Block certain operations on weekends
    if tool.name in {"deploy_production", "modify_infrastructure"}:
        import datetime
        if datetime.datetime.now().weekday() >= 5:  # Saturday=5, Sunday=6
            return {
                "status": "error",
                "code": "MAINTENANCE_WINDOW",
                "message": "Production modifications are not allowed on weekends. "
                          "Please schedule for a weekday."
            }

    # Scenario 3: Check feature flags / subscriptions
    if tool.name == "generate_report":
        report_type = args.get("type")
        subscription = tool_context.state.get("subscription_tier", "free")
        premium_reports = {"detailed_analytics", "custom_export", "api_usage"}

        if report_type in premium_reports and subscription == "free":
            return {
                "status": "error",
                "code": "PREMIUM_REQUIRED",
                "message": f"Report type '{report_type}' requires a premium subscription.",
                "upgrade_url": "https://example.com/upgrade"
            }

    return None  # Allow if no blocking conditions matched


# =============================================================================
# Combined Tool Protection Factory
# =============================================================================

def create_tool_protection_callback(
    protect_admin: bool = True,
    validate_args: bool = True,
    role_based: bool = True,
    custom_rules: bool = True,
    admin_users: Set[str] | None = None,
    role_permissions: Dict[str, Set[str]] | None = None,
):
    """
    Factory function to create a combined tool protection callback.

    Example:
        agent = Agent(
            name="secure_agent",
            model="gemini-2.5-flash",
            instruction="You are helpful.",
            tools=[...],
            before_tool_callback=create_tool_protection_callback(
                protect_admin=True,
                validate_args=True,
                role_based=True
            )
        )
    """
    def combined_callback(
        tool: BaseTool,
        args: Dict[str, Any],
        tool_context: ToolContext
    ) -> Optional[Dict]:
        # Check admin protection first
        if protect_admin:
            result = protect_admin_tools(tool, args, tool_context, admin_users)
            if result:
                return result

        # Check role-based access
        if role_based:
            result = restrict_tools_by_role(tool, args, tool_context, role_permissions)
            if result:
                return result

        # Validate arguments
        if validate_args:
            result = validate_tool_arguments(tool, args, tool_context)
            if result:
                return result

        # Apply custom rules
        if custom_rules:
            result = inspect_and_block_selective(tool, args, tool_context)
            if result:
                return result

        return None  # All checks passed

    return combined_callback


# =============================================================================
# Test Cases
# =============================================================================

if __name__ == "__main__":
    """
    Test cases demonstrating blocked vs allowed tool calls.
    Run: python tool_protection.py
    """
    from unittest.mock import MagicMock

    def create_mock_tool(name: str) -> BaseTool:
        """Helper to create mock tool."""
        tool = MagicMock(spec=BaseTool)
        tool.name = name
        return tool

    def create_mock_context(state: Dict[str, Any] = None) -> ToolContext:
        """Helper to create mock tool context."""
        context = MagicMock(spec=ToolContext)
        context.state = state or {}
        return context

    print("=" * 60)
    print("Testing Tool Protection Guardrails")
    print("=" * 60)

    # Test 1: Admin tool - regular user - should BLOCK
    print("\n1. Testing admin tool protection (regular user):")
    tool = create_mock_tool("delete_user")
    context = create_mock_context({"user_id": "john", "user_role": "user"})
    result = protect_admin_tools(tool, {}, context)
    print(f"   Tool: delete_user, User: john (role: user)")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is not None, "Should block non-admin"

    # Test 2: Admin tool - admin user - should ALLOW
    print("\n2. Testing admin tool protection (admin user):")
    context = create_mock_context({"user_id": "admin", "user_role": "admin"})
    result = protect_admin_tools(tool, {}, context)
    print(f"   Tool: delete_user, User: admin (role: admin)")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is None, "Should allow admin"

    # Test 3: SQL injection in query - should BLOCK
    print("\n3. Testing argument validation (SQL injection):")
    tool = create_mock_tool("execute_query")
    context = create_mock_context()
    args = {"query": "SELECT * FROM users; DROP TABLE users;--"}
    result = validate_tool_arguments(tool, args, context)
    print(f"   Query: 'SELECT * FROM users; DROP TABLE users;--'")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is not None, "Should block SQL injection"

    # Test 4: Valid email - should ALLOW
    print("\n4. Testing argument validation (valid email):")
    tool = create_mock_tool("send_email")
    args = {"to": "user@example.com", "subject": "Hello", "body": "Test message"}
    result = validate_tool_arguments(tool, args, context)
    print(f"   Email to: user@example.com")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is None, "Should allow valid email"

    # Test 5: Blocked domain - should BLOCK
    print("\n5. Testing argument validation (blocked domain):")
    args = {"to": "user@competitor.com", "subject": "Hi", "body": "Test"}
    result = validate_tool_arguments(tool, args, context)
    print(f"   Email to: user@competitor.com")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is not None, "Should block competitor domain"

    # Test 6: Role-based access - guest trying moderator tool
    print("\n6. Testing role-based access (guest -> moderator tool):")
    tool = create_mock_tool("flag_content")
    context = create_mock_context({"user_role": "guest"})
    result = restrict_tools_by_role(tool, {}, context)
    print(f"   Tool: flag_content, Role: guest")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is not None, "Should block guest from moderator tool"

    # Test 7: Role-based access - moderator using moderator tool
    print("\n7. Testing role-based access (moderator -> moderator tool):")
    context = create_mock_context({"user_role": "moderator"})
    result = restrict_tools_by_role(tool, {}, context)
    print(f"   Tool: flag_content, Role: moderator")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is None, "Should allow moderator"

    # Test 8: Path traversal attempt - should BLOCK
    print("\n8. Testing path traversal protection:")
    tool = create_mock_tool("file_write")
    args = {"path": "/tmp/../etc/passwd"}
    result = validate_tool_arguments(tool, args, context)
    print(f"   Path: /tmp/../etc/passwd")
    print(f"   Result: {'BLOCKED' if result else 'ALLOWED'}")
    assert result is not None, "Should block path traversal"

    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)
