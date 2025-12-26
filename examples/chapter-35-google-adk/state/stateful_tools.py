"""
Stateful Tools with ToolContext

Demonstrates how to access and modify session state from within tool functions.
ToolContext provides access to:
- state: Mutable dictionary persisted across turns
- agent_name: Current agent name
- invocation_id: Unique identifier for this tool call

Key patterns:
- Reading state within tools
- Writing state that persists across turns
- Cross-tool state sharing
- Building conversation history

Usage:
    export GOOGLE_API_KEY=your_api_key
    python stateful_tools.py
"""

import asyncio
from datetime import datetime
from typing import Any, Dict, List, Optional

from dotenv import load_dotenv
from google.adk import Runner
from google.adk.agents import Agent
from google.adk.sessions import InMemorySessionService
from google.adk.tools.tool_context import ToolContext
from google.genai import types

# Load environment variables
load_dotenv()


# =============================================================================
# STATEFUL TOOL EXAMPLES
# =============================================================================


def get_user_preference(key: str, tool_context: ToolContext) -> Dict[str, Any]:
    """
    Retrieve a user preference from session state.

    Args:
        key: The preference key to retrieve (e.g., 'theme', 'language', 'timezone')

    Returns:
        Dictionary with the preference value or indication it wasn't found.
    """
    preferences = tool_context.state.get("preferences", {})
    value = preferences.get(key)

    if value is not None:
        return {
            "status": "found",
            "key": key,
            "value": value,
        }
    else:
        return {
            "status": "not_found",
            "key": key,
            "available_keys": list(preferences.keys()),
        }


def set_user_preference(
    key: str, value: str, tool_context: ToolContext
) -> Dict[str, Any]:
    """
    Store a user preference in session state.

    Args:
        key: The preference key to set (e.g., 'theme', 'language')
        value: The value to store

    Returns:
        Confirmation of the saved preference.
    """
    # Initialize preferences dict if not exists
    if "preferences" not in tool_context.state:
        tool_context.state["preferences"] = {}

    # Store the preference
    tool_context.state["preferences"][key] = value

    return {
        "status": "saved",
        "key": key,
        "value": value,
        "all_preferences": tool_context.state["preferences"],
    }


def add_to_history(
    action: str, details: Optional[str], tool_context: ToolContext
) -> Dict[str, Any]:
    """
    Add an entry to the conversation/action history.

    This demonstrates appending to a list in session state.

    Args:
        action: The action type (e.g., 'search', 'purchase', 'view')
        details: Additional details about the action

    Returns:
        Confirmation with the updated history length.
    """
    # Initialize history if not exists
    if "history" not in tool_context.state:
        tool_context.state["history"] = []

    # Create history entry
    entry = {
        "action": action,
        "details": details,
        "timestamp": datetime.now().isoformat(),
        "agent": tool_context.agent_name,
    }

    # Append to history
    tool_context.state["history"].append(entry)

    return {
        "status": "recorded",
        "entry": entry,
        "total_entries": len(tool_context.state["history"]),
    }


def get_history(
    limit: Optional[int], tool_context: ToolContext
) -> Dict[str, Any]:
    """
    Retrieve action history from session state.

    Args:
        limit: Maximum number of entries to return. None for all entries.

    Returns:
        Recent history entries.
    """
    history: List[Dict] = tool_context.state.get("history", [])

    if limit and limit > 0:
        recent = history[-limit:]
    else:
        recent = history

    return {
        "total_entries": len(history),
        "returned_entries": len(recent),
        "history": recent,
    }


def increment_counter(counter_name: str, tool_context: ToolContext) -> Dict[str, Any]:
    """
    Increment a named counter in session state.

    Useful for tracking usage, rate limiting, or progression.

    Args:
        counter_name: Name of the counter to increment

    Returns:
        Updated counter value.
    """
    counters = tool_context.state.get("counters", {})
    current_value = counters.get(counter_name, 0)
    new_value = current_value + 1

    # Update state
    if "counters" not in tool_context.state:
        tool_context.state["counters"] = {}
    tool_context.state["counters"][counter_name] = new_value

    return {
        "counter": counter_name,
        "previous_value": current_value,
        "new_value": new_value,
    }


def get_session_summary(tool_context: ToolContext) -> Dict[str, Any]:
    """
    Generate a summary of the current session state.

    This tool reads from multiple state keys to provide an overview.

    Returns:
        Summary of all state data.
    """
    state = tool_context.state

    return {
        "preferences": state.get("preferences", {}),
        "history_count": len(state.get("history", [])),
        "counters": state.get("counters", {}),
        "custom_data_keys": [
            k
            for k in state.keys()
            if k not in ["preferences", "history", "counters"]
        ],
        "agent_name": tool_context.agent_name,
    }


# =============================================================================
# CROSS-TOOL STATE SHARING EXAMPLE
# =============================================================================


def add_to_cart(
    item: str, quantity: int, price: float, tool_context: ToolContext
) -> Dict[str, Any]:
    """
    Add an item to the shopping cart.

    Args:
        item: Name of the item
        quantity: Number of items
        price: Price per item

    Returns:
        Updated cart summary.
    """
    if "cart" not in tool_context.state:
        tool_context.state["cart"] = {"items": [], "total": 0.0}

    cart = tool_context.state["cart"]
    cart["items"].append(
        {"item": item, "quantity": quantity, "price": price, "subtotal": price * quantity}
    )
    cart["total"] = sum(i["subtotal"] for i in cart["items"])

    return {
        "status": "added",
        "item": item,
        "cart_items": len(cart["items"]),
        "cart_total": cart["total"],
    }


def apply_discount(code: str, tool_context: ToolContext) -> Dict[str, Any]:
    """
    Apply a discount code to the cart.

    This tool reads the cart state set by add_to_cart.

    Args:
        code: Discount code to apply

    Returns:
        Updated cart with discount.
    """
    cart = tool_context.state.get("cart", {"items": [], "total": 0.0})

    # Simulate discount codes
    discounts = {"SAVE10": 0.10, "SAVE20": 0.20, "HALF": 0.50}

    if code.upper() in discounts:
        discount_rate = discounts[code.upper()]
        original_total = cart["total"]
        discount_amount = original_total * discount_rate
        new_total = original_total - discount_amount

        # Update cart state
        cart["discount_code"] = code.upper()
        cart["discount_amount"] = discount_amount
        cart["total"] = new_total
        tool_context.state["cart"] = cart

        return {
            "status": "applied",
            "code": code.upper(),
            "discount_rate": f"{discount_rate * 100}%",
            "discount_amount": discount_amount,
            "original_total": original_total,
            "new_total": new_total,
        }
    else:
        return {
            "status": "invalid",
            "code": code,
            "message": "Unknown discount code",
        }


def checkout(tool_context: ToolContext) -> Dict[str, Any]:
    """
    Complete the checkout process.

    Reads cart state and clears it after successful checkout.

    Returns:
        Order confirmation.
    """
    cart = tool_context.state.get("cart", {"items": [], "total": 0.0})

    if not cart["items"]:
        return {"status": "error", "message": "Cart is empty"}

    # Generate order
    order = {
        "order_id": f"ORD-{datetime.now().strftime('%Y%m%d%H%M%S')}",
        "items": cart["items"],
        "subtotal": sum(i["subtotal"] for i in cart["items"]),
        "discount": cart.get("discount_amount", 0),
        "total": cart["total"],
        "status": "confirmed",
    }

    # Store order in history and clear cart
    if "orders" not in tool_context.state:
        tool_context.state["orders"] = []
    tool_context.state["orders"].append(order)

    # Clear cart
    tool_context.state["cart"] = {"items": [], "total": 0.0}

    return {
        "status": "success",
        "order": order,
    }


# =============================================================================
# AGENT FACTORY
# =============================================================================


def create_stateful_agent() -> Agent:
    """
    Create an agent with all stateful tools configured.

    Returns:
        Agent configured with preference, history, and shopping tools.
    """
    return Agent(
        name="stateful_assistant",
        model="gemini-2.5-flash",
        instruction="""You are a helpful assistant that remembers user preferences and tracks their actions.

You have access to these tools:
- get_user_preference / set_user_preference: Store and retrieve user preferences
- add_to_history / get_history: Track user actions
- increment_counter: Track usage counts
- get_session_summary: View all session data
- add_to_cart / apply_discount / checkout: Shopping cart operations

Always use these tools to maintain context across the conversation.
When a user expresses a preference, save it immediately.
When performing actions, log them to history.
""",
        tools=[
            get_user_preference,
            set_user_preference,
            add_to_history,
            get_history,
            increment_counter,
            get_session_summary,
            add_to_cart,
            apply_discount,
            checkout,
        ],
    )


# =============================================================================
# DEMONSTRATION
# =============================================================================


async def demonstrate_stateful_tools():
    """
    Demonstrate stateful tools preserving data across conversation turns.
    """
    agent = create_stateful_agent()
    session_service = InMemorySessionService()

    runner = Runner(
        app_name="stateful_demo",
        agent=agent,
        session_service=session_service,
    )

    # Create session with initial state
    session = await session_service.create_session(
        app_name="stateful_demo",
        user_id="demo_user",
        state={
            "preferences": {"theme": "dark"},  # Pre-existing preference
        },
    )

    print("\n" + "=" * 60)
    print("STATEFUL TOOLS DEMONSTRATION")
    print("=" * 60)

    # Conversation that builds up state across turns
    conversation = [
        "What's my current theme preference?",
        "Please set my language preference to English",
        "Add 2 wireless headphones at $49.99 each to my cart",
        "Apply discount code SAVE10",
        "Show me a summary of my session",
        "Complete my purchase",
    ]

    for i, user_msg in enumerate(conversation, 1):
        print(f"\n--- Turn {i} ---")
        print(f"User: {user_msg}")

        message = types.Content(
            role="user",
            parts=[types.Part(text=user_msg)],
        )

        response = ""
        async for event in runner.run_async(
            user_id="demo_user",
            session_id=session.id,
            new_message=message,
        ):
            if event.content and event.content.parts:
                for part in event.content.parts:
                    if part.text:
                        response += part.text

        print(f"Agent: {response[:300]}...")

    # Show final state
    final_session = await session_service.get_session(
        app_name="stateful_demo",
        user_id="demo_user",
        session_id=session.id,
    )
    print("\n" + "=" * 60)
    print("FINAL SESSION STATE")
    print("=" * 60)
    import json

    print(json.dumps(final_session.state, indent=2, default=str))


if __name__ == "__main__":
    asyncio.run(demonstrate_stateful_tools())
