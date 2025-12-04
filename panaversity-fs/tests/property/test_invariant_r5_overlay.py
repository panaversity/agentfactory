"""Property-based tests for overlay exclusivity (T048).

Tests R5 invariant: User overlays are isolated - writes to user A's overlay
never affect user B's overlay or base content.

Uses hypothesis for property-based testing with composite strategies.
"""

import pytest
import json
from hypothesis import given, strategies as st, settings, HealthCheck
from panaversity_fs.tools.content import read_content, write_content, delete_content
from panaversity_fs.models import ReadContentInput, WriteContentInput, DeleteContentInput
from panaversity_fs.storage import get_operator


# Hypothesis settings - each test generates unique IDs so fixture state doesn't matter
HYPOTHESIS_SETTINGS = {
    "max_examples": 10,
    "deadline": None,
    "suppress_health_check": [HealthCheck.function_scoped_fixture]
}


# Strategies for generating test data
user_id_strategy = st.text(
    alphabet="abcdefghijklmnopqrstuvwxyz0123456789",
    min_size=5,
    max_size=12
).map(lambda s: f"user-{s}")

lesson_name_strategy = st.text(
    alphabet="abcdefghijklmnopqrstuvwxyz",
    min_size=3,
    max_size=10
).map(lambda s: f"01-{s}")

content_strategy = st.text(
    alphabet="abcdefghijklmnopqrstuvwxyz0123456789 \n#",
    min_size=10,
    max_size=100
).map(lambda s: f"# Lesson\n\n{s}")


# Composite strategy for generating two distinct users
@st.composite
def two_distinct_users(draw):
    """Generate two distinct user IDs."""
    user1 = draw(user_id_strategy)
    user2 = draw(user_id_strategy.filter(lambda u: u != user1))
    return user1, user2


# Composite strategy for generating two distinct lessons
@st.composite
def two_distinct_lessons(draw):
    """Generate two distinct lesson names."""
    lesson1 = draw(lesson_name_strategy)
    lesson2 = draw(lesson_name_strategy.filter(lambda l: l != lesson1))
    return lesson1, lesson2


class TestOverlayExclusivityR5:
    """Property tests for R5: Overlay exclusivity.

    R5 invariant: For any two users A and B:
    - User A's overlay writes never affect User B's overlay
    - User A's overlay writes never affect base content
    - User A reading with user_id returns overlay if exists, else base
    """

    @pytest.mark.asyncio
    @given(
        users=two_distinct_users(),
        content_a=content_strategy,
        content_b=content_strategy
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_user_overlays_are_isolated(self, setup_fs_backend, users, content_a, content_b, mock_context):
        """R5: User A's overlay is isolated from User B's overlay."""
        # Use valid NN-Name format (FR-007 schema)
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        user_a, user_b = users
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-isolation{unique_id}.md"

        op = get_operator()

        # Create base content
        base_content = "# Base Content\n\nOriginal base."
        await op.write(f"books/{book_id}/{path}", base_content.encode())

        # User A writes to their overlay
        write_a = WriteContentInput(
            book_id=book_id,
            path=path,
            content=content_a,
            user_id=user_a
        )
        result_a = await write_content(write_a, mock_context)
        assert "success" in result_a

        # User B writes to their overlay
        write_b = WriteContentInput(
            book_id=book_id,
            path=path,
            content=content_b,
            user_id=user_b
        )
        result_b = await write_content(write_b, mock_context)
        assert "success" in result_b

        # INVARIANT: User A reads their own overlay content
        read_a = ReadContentInput(book_id=book_id, path=path, user_id=user_a)
        result_read_a = await read_content(read_a, mock_context)
        data_a = json.loads(result_read_a)
        assert data_a["source"] == "overlay"
        # Compare stripped content to handle whitespace differences
        assert content_a.strip() in data_a["content"] or data_a["content"].strip() == content_a.strip()

        # INVARIANT: User B reads their own overlay content (different from A)
        read_b = ReadContentInput(book_id=book_id, path=path, user_id=user_b)
        result_read_b = await read_content(read_b, mock_context)
        data_b = json.loads(result_read_b)
        assert data_b["source"] == "overlay"
        # Compare stripped content to handle whitespace differences
        assert content_b.strip() in data_b["content"] or data_b["content"].strip() == content_b.strip()

        # INVARIANT: Base content is unchanged
        read_base = ReadContentInput(book_id=book_id, path=path)
        result_base = await read_content(read_base, mock_context)
        data_base = json.loads(result_base)
        assert "Base Content" in data_base["content"]

    @pytest.mark.asyncio
    @given(
        user=user_id_strategy,
        overlay_content=content_strategy
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_overlay_never_modifies_base(self, setup_fs_backend, user, overlay_content, mock_context):
        """R5: Overlay writes never modify base content."""
        # Use valid NN-Name format (FR-007 schema)
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-base-safe{unique_id}.md"

        op = get_operator()

        # Create base content with unique marker
        base_marker = f"BASE-MARKER-{unique_id}"
        base_content = f"# Base\n\n{base_marker}"
        await op.write(f"books/{book_id}/{path}", base_content.encode())

        # Get base hash before overlay write
        read_before = ReadContentInput(book_id=book_id, path=path)
        result_before = await read_content(read_before, mock_context)
        data_before = json.loads(result_before)
        hash_before = data_before["file_hash_sha256"]

        # Write to overlay
        write_params = WriteContentInput(
            book_id=book_id,
            path=path,
            content=overlay_content,
            user_id=user
        )
        await write_content(write_params, mock_context)

        # INVARIANT: Base content unchanged (same hash)
        read_after = ReadContentInput(book_id=book_id, path=path)
        result_after = await read_content(read_after, mock_context)
        data_after = json.loads(result_after)
        hash_after = data_after["file_hash_sha256"]

        assert hash_before == hash_after, "Base content hash changed after overlay write"
        assert base_marker in data_after["content"], "Base content modified by overlay write"

    @pytest.mark.asyncio
    @given(
        users=two_distinct_users(),
        lessons=two_distinct_lessons()
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_overlay_delete_isolation(self, setup_fs_backend, users, lessons, mock_context):
        """R5: Deleting User A's overlay doesn't affect User B's overlay."""
        # Use valid NN-Name format (FR-007 schema) - note: lesson already has 01- prefix
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        user_a, user_b = users
        lesson1, lesson2 = lessons
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/{lesson1}{unique_id}.md"

        op = get_operator()

        # Create base content
        await op.write(f"books/{book_id}/{path}", b"# Base Content")

        # Both users create overlays
        for user, content in [(user_a, "# A Content"), (user_b, "# B Content")]:
            write_params = WriteContentInput(
                book_id=book_id,
                path=path,
                content=content,
                user_id=user
            )
            await write_content(write_params, mock_context)

        # User A deletes their overlay
        delete_params = DeleteContentInput(
            book_id=book_id,
            path=path,
            user_id=user_a
        )
        result = await delete_content(delete_params, mock_context)
        assert "success" in result

        # INVARIANT: User B's overlay is unaffected
        read_b = ReadContentInput(book_id=book_id, path=path, user_id=user_b)
        result_b = await read_content(read_b, mock_context)
        data_b = json.loads(result_b)
        assert data_b["source"] == "overlay"
        assert "B Content" in data_b["content"]

        # INVARIANT: User A now falls back to base
        read_a = ReadContentInput(book_id=book_id, path=path, user_id=user_a)
        result_a = await read_content(read_a, mock_context)
        data_a = json.loads(result_a)
        assert data_a["source"] == "base"
        assert "Base Content" in data_a["content"]

    @pytest.mark.asyncio
    @given(
        user=user_id_strategy,
        content=content_strategy
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_overlay_fallback_to_base(self, setup_fs_backend, user, content, mock_context):
        """R5: Reading with user_id falls back to base when no overlay exists."""
        # Use valid NN-Name format (FR-007 schema)
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-fallback{unique_id}.md"

        op = get_operator()

        # Create only base content (no overlay)
        base_content = f"# Fallback Test\n\n{content}"
        await op.write(f"books/{book_id}/{path}", base_content.encode())

        # INVARIANT: Reading with user_id returns base content
        read_params = ReadContentInput(book_id=book_id, path=path, user_id=user)
        result = await read_content(read_params, mock_context)
        data = json.loads(result)

        assert data["source"] == "base"
        # Handle whitespace trimming in content
        assert content.strip() in data["content"] or data["content"].strip() == base_content.strip()

    @pytest.mark.asyncio
    @given(
        user=user_id_strategy,
        base_content=content_strategy,
        overlay_content=content_strategy
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_overlay_takes_precedence(self, setup_fs_backend, user, base_content, overlay_content, mock_context):
        """R5: When overlay exists, it takes precedence over base."""
        # Use valid NN-Name format (FR-007 schema)
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-precedence{unique_id}.md"

        op = get_operator()

        # Create base content
        await op.write(f"books/{book_id}/{path}", base_content.encode())

        # Create overlay
        write_params = WriteContentInput(
            book_id=book_id,
            path=path,
            content=overlay_content,
            user_id=user
        )
        await write_content(write_params, mock_context)

        # INVARIANT: Overlay takes precedence
        read_params = ReadContentInput(book_id=book_id, path=path, user_id=user)
        result = await read_content(read_params, mock_context)
        data = json.loads(result)

        assert data["source"] == "overlay"
        # Overlay content should be returned, not base (handle whitespace trimming)
        assert overlay_content.strip() in data["content"] or data["content"].strip() == overlay_content.strip()
