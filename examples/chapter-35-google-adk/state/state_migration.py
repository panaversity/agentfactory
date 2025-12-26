"""
State Migration Patterns for Google ADK

Demonstrates patterns for migrating session state between backends:
- Export from InMemorySessionService
- Import to FirestoreSessionService
- Schema versioning for backward compatibility
- State transformation during migration

Use cases:
- Moving from development (InMemory) to production (Firestore)
- Upgrading state schema when adding new features
- Backup and restore session data
- Multi-region state replication

Usage:
    # Export from memory, import to Firestore
    export GOOGLE_CLOUD_PROJECT=your-project-id
    python state_migration.py

    # Dry run (no actual migration)
    python state_migration.py --dry-run
"""

import asyncio
import json
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional

from dotenv import load_dotenv
from google.adk.sessions import InMemorySessionService

# Load environment variables
load_dotenv()


# =============================================================================
# SCHEMA VERSIONING
# =============================================================================


@dataclass
class StateVersion:
    """Represents a state schema version with migration logic."""

    version: int
    description: str
    migrator: Callable[[Dict[str, Any]], Dict[str, Any]]


def migrate_v1_to_v2(state: Dict[str, Any]) -> Dict[str, Any]:
    """
    Migrate state from v1 to v2.

    Changes:
    - Rename 'prefs' to 'preferences'
    - Add 'schema_version' field
    """
    new_state = state.copy()

    # Rename prefs -> preferences
    if "prefs" in new_state:
        new_state["preferences"] = new_state.pop("prefs")

    # Add schema version
    new_state["schema_version"] = 2

    return new_state


def migrate_v2_to_v3(state: Dict[str, Any]) -> Dict[str, Any]:
    """
    Migrate state from v2 to v3.

    Changes:
    - Convert flat preferences to nested structure
    - Add default values for new fields
    """
    new_state = state.copy()

    # Convert flat preferences to nested
    if "preferences" in new_state and isinstance(new_state["preferences"], dict):
        old_prefs = new_state["preferences"]
        new_state["preferences"] = {
            "display": {
                "theme": old_prefs.get("theme", "light"),
                "language": old_prefs.get("language", "en"),
            },
            "notifications": {
                "email": old_prefs.get("email_notifications", True),
                "push": old_prefs.get("push_notifications", True),
            },
        }

    # Add new fields with defaults
    if "metadata" not in new_state:
        new_state["metadata"] = {
            "created_at": None,
            "updated_at": datetime.now().isoformat(),
            "migration_history": ["v2->v3"],
        }

    new_state["schema_version"] = 3

    return new_state


# Version registry
SCHEMA_VERSIONS = [
    StateVersion(1, "Initial schema", lambda x: x),
    StateVersion(2, "Renamed prefs to preferences", migrate_v1_to_v2),
    StateVersion(3, "Nested preferences structure", migrate_v2_to_v3),
]

CURRENT_VERSION = 3


def get_state_version(state: Dict[str, Any]) -> int:
    """Determine the schema version of a state dict."""
    return state.get("schema_version", 1)


def migrate_state(
    state: Dict[str, Any],
    from_version: Optional[int] = None,
    to_version: int = CURRENT_VERSION,
) -> Dict[str, Any]:
    """
    Migrate state from one schema version to another.

    Args:
        state: Current state dictionary
        from_version: Starting version (auto-detected if None)
        to_version: Target version (defaults to latest)

    Returns:
        Migrated state dictionary
    """
    if from_version is None:
        from_version = get_state_version(state)

    if from_version >= to_version:
        return state  # Already at or past target version

    current_state = state.copy()

    # Apply migrations sequentially
    for version in SCHEMA_VERSIONS:
        if version.version > from_version and version.version <= to_version:
            print(f"  Applying migration: v{version.version - 1} -> v{version.version}")
            print(f"    {version.description}")
            current_state = version.migrator(current_state)

    return current_state


# =============================================================================
# EXPORT/IMPORT UTILITIES
# =============================================================================


async def export_sessions(
    session_service: InMemorySessionService,
    app_name: str,
    output_file: str,
) -> int:
    """
    Export all sessions from a SessionService to JSON.

    Args:
        session_service: Source session service
        app_name: Application name to filter sessions
        output_file: Path to output JSON file

    Returns:
        Number of sessions exported
    """
    # Note: InMemorySessionService stores sessions in _sessions dict
    # In production, you'd use list_sessions for each user

    exported_sessions: List[Dict] = []

    # For demo, we'll create and export known sessions
    # In production, you'd iterate over all users
    demo_sessions = [
        {
            "user_id": "user_1",
            "session_id": "session_1",
            "state": {"prefs": {"theme": "dark"}, "cart": []},
        },
        {
            "user_id": "user_2",
            "session_id": "session_2",
            "state": {"prefs": {"theme": "light", "language": "es"}, "history": []},
        },
    ]

    for session_data in demo_sessions:
        exported_sessions.append(
            {
                "app_name": app_name,
                "user_id": session_data["user_id"],
                "session_id": session_data["session_id"],
                "state": session_data["state"],
                "exported_at": datetime.now().isoformat(),
            }
        )

    # Write to file
    with open(output_file, "w") as f:
        json.dump(
            {
                "export_version": 1,
                "app_name": app_name,
                "exported_at": datetime.now().isoformat(),
                "session_count": len(exported_sessions),
                "sessions": exported_sessions,
            },
            f,
            indent=2,
        )

    return len(exported_sessions)


async def import_sessions(
    input_file: str,
    target_service,  # SessionService
    migrate_to_version: int = CURRENT_VERSION,
    dry_run: bool = False,
) -> Dict[str, Any]:
    """
    Import sessions from JSON file to a SessionService.

    Args:
        input_file: Path to JSON file from export
        target_service: Target session service
        migrate_to_version: Apply migrations up to this version
        dry_run: If True, validate only without importing

    Returns:
        Import summary with success/failure counts
    """
    with open(input_file, "r") as f:
        export_data = json.load(f)

    results = {
        "total": export_data["session_count"],
        "imported": 0,
        "failed": 0,
        "skipped": 0,
        "migrated": 0,
        "errors": [],
    }

    print(f"\nImporting {results['total']} sessions from {input_file}")
    print(f"Target schema version: {migrate_to_version}")
    print(f"Dry run: {dry_run}")
    print("-" * 40)

    for session_data in export_data["sessions"]:
        user_id = session_data["user_id"]
        original_state = session_data["state"]

        try:
            # Determine if migration needed
            current_version = get_state_version(original_state)

            if current_version < migrate_to_version:
                print(f"\n[{user_id}] Migrating from v{current_version} to v{migrate_to_version}")
                migrated_state = migrate_state(
                    original_state,
                    from_version=current_version,
                    to_version=migrate_to_version,
                )
                results["migrated"] += 1
            else:
                migrated_state = original_state

            if dry_run:
                print(f"[{user_id}] Would import with state:")
                print(f"  Original: {json.dumps(original_state, indent=2)[:100]}...")
                print(f"  Migrated: {json.dumps(migrated_state, indent=2)[:100]}...")
                results["skipped"] += 1
            else:
                # Actually create the session
                await target_service.create_session(
                    app_name=export_data["app_name"],
                    user_id=user_id,
                    state=migrated_state,
                )
                print(f"[{user_id}] Imported successfully")
                results["imported"] += 1

        except Exception as e:
            results["failed"] += 1
            results["errors"].append({"user_id": user_id, "error": str(e)})
            print(f"[{user_id}] Failed: {e}")

    return results


# =============================================================================
# FIRESTORE-SPECIFIC MIGRATION
# =============================================================================


async def migrate_inmemory_to_firestore(
    source: InMemorySessionService,
    app_name: str,
    project_id: str,
    database: str = "(default)",
    dry_run: bool = False,
):
    """
    Migrate all sessions from InMemory to Firestore.

    This is the most common migration path: dev -> production.

    Args:
        source: InMemorySessionService with sessions to migrate
        app_name: Application name
        project_id: Google Cloud project ID
        database: Firestore database name
        dry_run: If True, validate without writing

    Returns:
        Migration summary
    """
    if not dry_run:
        try:
            from google.adk.sessions import FirestoreSessionService

            target = FirestoreSessionService(
                project=project_id,
                database=database,
            )
        except ImportError:
            print("ERROR: FirestoreSessionService requires google-cloud-firestore")
            print("Install with: pip install google-cloud-firestore")
            return None
    else:
        target = None

    # Export to temp file
    temp_file = f"/tmp/session_export_{app_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    export_count = await export_sessions(source, app_name, temp_file)

    print(f"\nExported {export_count} sessions to {temp_file}")

    if target:
        # Import to Firestore
        results = await import_sessions(
            temp_file,
            target,
            migrate_to_version=CURRENT_VERSION,
            dry_run=dry_run,
        )
        return results
    else:
        print("\n[DRY RUN] Would import to Firestore:")
        with open(temp_file, "r") as f:
            data = json.load(f)
            for session in data["sessions"]:
                print(f"  - User: {session['user_id']}, Session: {session['session_id']}")
        return {"dry_run": True, "would_import": export_count}


# =============================================================================
# BACKWARD COMPATIBILITY HANDLER
# =============================================================================


def create_backward_compatible_state_reader(
    current_version: int = CURRENT_VERSION,
) -> Callable[[Dict[str, Any]], Dict[str, Any]]:
    """
    Create a function that can read state of any version.

    This is useful for agents that need to handle sessions
    created before schema migrations.

    Returns:
        Function that normalizes state to current version
    """

    def read_state(state: Dict[str, Any]) -> Dict[str, Any]:
        state_version = get_state_version(state)

        if state_version < current_version:
            # Auto-migrate on read
            return migrate_state(state, state_version, current_version)
        else:
            return state

    return read_state


# =============================================================================
# DEMONSTRATION
# =============================================================================


async def demonstrate_migration():
    """Demonstrate state migration patterns."""

    print("\n" + "=" * 60)
    print("STATE MIGRATION DEMONSTRATION")
    print("=" * 60)

    # 1. Show version detection and migration
    print("\n1. Schema Version Migration")
    print("-" * 40)

    old_state_v1 = {
        "prefs": {"theme": "dark", "email_notifications": False},
        "cart": ["item1", "item2"],
    }

    print("Original state (v1):")
    print(json.dumps(old_state_v1, indent=2))

    migrated_state = migrate_state(old_state_v1, from_version=1, to_version=CURRENT_VERSION)

    print(f"\nMigrated state (v{CURRENT_VERSION}):")
    print(json.dumps(migrated_state, indent=2))

    # 2. Export/Import demonstration
    print("\n\n2. Export/Import Demonstration")
    print("-" * 40)

    source_service = InMemorySessionService()

    # Create some sessions in source
    await source_service.create_session(
        app_name="demo_app",
        user_id="user_1",
        state={"prefs": {"theme": "dark"}, "counter": 5},
    )

    export_file = "/tmp/demo_export.json"
    count = await export_sessions(source_service, "demo_app", export_file)
    print(f"Exported {count} sessions to {export_file}")

    # Show export contents
    with open(export_file, "r") as f:
        print("Export file contents:")
        print(json.dumps(json.load(f), indent=2)[:500])

    # 3. Backward compatibility reader
    print("\n\n3. Backward Compatibility")
    print("-" * 40)

    reader = create_backward_compatible_state_reader()

    test_states = [
        {"prefs": {"theme": "light"}},  # v1
        {"preferences": {"theme": "dark"}, "schema_version": 2},  # v2
        {  # v3
            "preferences": {"display": {"theme": "system"}},
            "schema_version": 3,
        },
    ]

    for i, state in enumerate(test_states, 1):
        version = get_state_version(state)
        normalized = reader(state)
        print(f"State {i} (v{version}) -> normalized to v{get_state_version(normalized)}")


if __name__ == "__main__":
    dry_run = "--dry-run" in sys.argv

    asyncio.run(demonstrate_migration())

    # Uncomment to run actual migration to Firestore
    # project = os.getenv("GOOGLE_CLOUD_PROJECT")
    # if project:
    #     asyncio.run(migrate_inmemory_to_firestore(
    #         source=InMemorySessionService(),
    #         app_name="my_app",
    #         project_id=project,
    #         dry_run=dry_run
    #     ))
