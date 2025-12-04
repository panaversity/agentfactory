"""Unit tests for personalization overlay support (FR-015, FR-016, FR-017, FR-018).

Tests overlay-first-then-base reading, overlay writes, and overlay deletes.
"""

import pytest
import json
from panaversity_fs.tools.content import read_content, write_content, delete_content
from panaversity_fs.models import ReadContentInput, WriteContentInput, DeleteContentInput
from panaversity_fs.storage import get_operator


class TestReadContentOverlay:
    """Test overlay support for read_content (FR-016)."""

    @pytest.mark.asyncio
    async def test_read_from_base_when_no_user_id(self, setup_fs_backend, mock_context):
        """Without user_id, reads from base namespace."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-base-only{unique_id}.md"

        # Create base content
        op = get_operator()
        await op.write(f"books/{book_id}/{path}", b"# Base Content")

        # Read without user_id
        params = ReadContentInput(book_id=book_id, path=path)
        result = await read_content(params, mock_context)
        data = json.loads(result)

        assert "# Base Content" in data["content"]
        assert "source" not in data  # No source field when no user_id

    @pytest.mark.asyncio
    async def test_read_from_overlay_when_exists(self, setup_fs_backend, mock_context):
        """With user_id and overlay exists, reads from overlay (FR-016)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-overlay{unique_id}.md"
        user_id = f"user-{unique_id}"

        op = get_operator()

        # Create base content
        await op.write(f"books/{book_id}/{path}", b"# Base Content")

        # Create overlay content
        overlay_path = f"books/{book_id}/users/{user_id}/{path}"
        await op.write(overlay_path, b"# User Personalized Content")

        # Read with user_id - should get overlay
        params = ReadContentInput(book_id=book_id, path=path, user_id=user_id)
        result = await read_content(params, mock_context)
        data = json.loads(result)

        assert "# User Personalized Content" in data["content"]
        assert data["source"] == "overlay"

    @pytest.mark.asyncio
    async def test_read_falls_back_to_base_when_no_overlay(self, setup_fs_backend, mock_context):
        """With user_id but no overlay, falls back to base (FR-016)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-fallback{unique_id}.md"
        user_id = f"user-{unique_id}"

        op = get_operator()

        # Create only base content (no overlay)
        await op.write(f"books/{book_id}/{path}", b"# Base Content Only")

        # Read with user_id - should fall back to base
        params = ReadContentInput(book_id=book_id, path=path, user_id=user_id)
        result = await read_content(params, mock_context)
        data = json.loads(result)

        assert "# Base Content Only" in data["content"]
        assert data["source"] == "base"

    @pytest.mark.asyncio
    async def test_read_returns_error_when_neither_exists(self, setup_fs_backend, mock_context):
        """With user_id but neither overlay nor base exists, returns error string."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-nonexistent{unique_id}.md"
        user_id = f"user-{unique_id}"

        # Read non-existent file
        params = ReadContentInput(book_id=book_id, path=path, user_id=user_id)
        result = await read_content(params, mock_context)

        # MCP tools return error messages as strings, not exceptions
        assert "Error reading content" in result


class TestWriteContentOverlay:
    """Test overlay support for write_content (FR-017)."""

    @pytest.mark.asyncio
    async def test_write_to_base_when_no_user_id(self, setup_fs_backend, mock_context):
        """Without user_id, writes to base namespace."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-base-write{unique_id}.md"

        # Write without user_id
        params = WriteContentInput(
            book_id=book_id,
            path=path,
            content="# Base Content"
        )
        result = await write_content(params, mock_context)
        data = json.loads(result)

        assert data["status"] == "success"
        assert data["mode"] == "created"
        assert "namespace" not in data  # No namespace field for base writes
        assert f"books/{book_id}/{path}" == data["path"]

    @pytest.mark.asyncio
    async def test_write_to_overlay_when_user_id_provided(self, setup_fs_backend, mock_context):
        """With user_id, writes to overlay namespace (FR-017)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-overlay-write{unique_id}.md"
        user_id = f"user-{unique_id}"

        # Write with user_id
        params = WriteContentInput(
            book_id=book_id,
            path=path,
            content="# User's Personalized Content",
            user_id=user_id
        )
        result = await write_content(params, mock_context)
        data = json.loads(result)

        assert data["status"] == "success"
        assert data["mode"] == "created"
        assert data["namespace"] == "overlay"
        assert f"users/{user_id}" in data["path"]

    @pytest.mark.asyncio
    async def test_overlay_write_does_not_modify_base(self, setup_fs_backend, mock_context):
        """Overlay write doesn't affect base content (FR-017)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-preserve-base{unique_id}.md"
        user_id = f"user-{unique_id}"

        op = get_operator()

        # Create base content first
        base_path = f"books/{book_id}/{path}"
        await op.write(base_path, b"# Original Base Content")

        # Write to overlay
        params = WriteContentInput(
            book_id=book_id,
            path=path,
            content="# User's Changes",
            user_id=user_id
        )
        await write_content(params, mock_context)

        # Verify base is unchanged
        base_content = await op.read(base_path)
        assert base_content == b"# Original Base Content"

        # Verify overlay has user's content
        overlay_path = f"books/{book_id}/users/{user_id}/{path}"
        overlay_content = await op.read(overlay_path)
        assert overlay_content == b"# User's Changes"

    @pytest.mark.asyncio
    async def test_overlay_update_requires_hash(self, setup_fs_backend, mock_context):
        """Updating overlay requires expected_hash (FR-004 applies to overlays)."""
        import uuid
        from panaversity_fs.errors import HashRequiredError
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-overlay-update{unique_id}.md"
        user_id = f"user-{unique_id}"

        # Create overlay
        params1 = WriteContentInput(
            book_id=book_id,
            path=path,
            content="# Initial Overlay",
            user_id=user_id
        )
        await write_content(params1, mock_context)

        # Try to update without hash
        params2 = WriteContentInput(
            book_id=book_id,
            path=path,
            content="# Updated Overlay",
            user_id=user_id
        )

        with pytest.raises(HashRequiredError):
            await write_content(params2, mock_context)


class TestDeleteContentOverlay:
    """Test overlay support for delete_content (FR-018)."""

    @pytest.mark.asyncio
    async def test_delete_base_when_no_user_id(self, setup_fs_backend, mock_context):
        """Without user_id, deletes from base namespace."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-delete-base{unique_id}.md"

        op = get_operator()

        # Create base content
        await op.write(f"books/{book_id}/{path}", b"# Base Content")

        # Delete without user_id
        params = DeleteContentInput(book_id=book_id, path=path)
        result = await delete_content(params, mock_context)
        data = json.loads(result)

        assert data["status"] == "success"
        assert data["existed"] is True
        assert "namespace" not in data

    @pytest.mark.asyncio
    async def test_delete_overlay_only_when_user_id_provided(self, setup_fs_backend, mock_context):
        """With user_id, only deletes overlay (FR-018)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-delete-overlay{unique_id}.md"
        user_id = f"user-{unique_id}"

        op = get_operator()

        # Create base content
        base_path = f"books/{book_id}/{path}"
        await op.write(base_path, b"# Base Content")

        # Create overlay content
        overlay_path = f"books/{book_id}/users/{user_id}/{path}"
        await op.write(overlay_path, b"# Overlay Content")

        # Delete with user_id - should only delete overlay
        params = DeleteContentInput(book_id=book_id, path=path, user_id=user_id)
        result = await delete_content(params, mock_context)
        data = json.loads(result)

        assert data["status"] == "success"
        assert data["existed"] is True
        assert data["namespace"] == "overlay"

        # Verify base is untouched
        base_content = await op.read(base_path)
        assert base_content == b"# Base Content"

        # Verify overlay is deleted
        try:
            await op.read(overlay_path)
            pytest.fail("Overlay should have been deleted")
        except Exception:
            pass  # Expected - overlay deleted

    @pytest.mark.asyncio
    async def test_delete_overlay_idempotent(self, setup_fs_backend, mock_context):
        """Deleting non-existent overlay is idempotent (FR-018)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-delete-idempotent{unique_id}.md"
        user_id = f"user-{unique_id}"

        # Delete non-existent overlay
        params = DeleteContentInput(book_id=book_id, path=path, user_id=user_id)
        result = await delete_content(params, mock_context)
        data = json.loads(result)

        assert data["status"] == "success"
        assert data["existed"] is False
        assert data["namespace"] == "overlay"

    @pytest.mark.asyncio
    async def test_delete_overlay_resets_to_base(self, setup_fs_backend, mock_context):
        """Deleting overlay allows reading base again (FR-018 reset behavior)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-reset{unique_id}.md"
        user_id = f"user-{unique_id}"

        op = get_operator()

        # Create base content
        await op.write(f"books/{book_id}/{path}", b"# Base Content")

        # Create overlay
        overlay_path = f"books/{book_id}/users/{user_id}/{path}"
        await op.write(overlay_path, b"# User Changes")

        # Read with user_id - should get overlay
        read_params = ReadContentInput(book_id=book_id, path=path, user_id=user_id)
        result1 = await read_content(read_params, mock_context)
        data1 = json.loads(result1)
        assert data1["source"] == "overlay"
        assert "# User Changes" in data1["content"]

        # Delete overlay
        delete_params = DeleteContentInput(book_id=book_id, path=path, user_id=user_id)
        await delete_content(delete_params, mock_context)

        # Read again with user_id - should fall back to base
        result2 = await read_content(read_params, mock_context)
        data2 = json.loads(result2)
        assert data2["source"] == "base"
        assert "# Base Content" in data2["content"]


class TestOverlayIsolation:
    """Test that different users have isolated overlays."""

    @pytest.mark.asyncio
    async def test_different_users_have_separate_overlays(self, setup_fs_backend, mock_context):
        """Different users' overlays are isolated from each other."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-multi-user{unique_id}.md"
        user_alice = f"alice-{unique_id}"
        user_bob = f"bob-{unique_id}"

        op = get_operator()

        # Create base content
        await op.write(f"books/{book_id}/{path}", b"# Base Content")

        # Alice creates her overlay
        alice_params = WriteContentInput(
            book_id=book_id,
            path=path,
            content="# Alice's Notes",
            user_id=user_alice
        )
        await write_content(alice_params, mock_context)

        # Bob creates his overlay
        bob_params = WriteContentInput(
            book_id=book_id,
            path=path,
            content="# Bob's Notes",
            user_id=user_bob
        )
        await write_content(bob_params, mock_context)

        # Read Alice's overlay
        alice_read = ReadContentInput(book_id=book_id, path=path, user_id=user_alice)
        result_alice = await read_content(alice_read, mock_context)
        data_alice = json.loads(result_alice)
        assert "# Alice's Notes" in data_alice["content"]
        assert data_alice["source"] == "overlay"

        # Read Bob's overlay
        bob_read = ReadContentInput(book_id=book_id, path=path, user_id=user_bob)
        result_bob = await read_content(bob_read, mock_context)
        data_bob = json.loads(result_bob)
        assert "# Bob's Notes" in data_bob["content"]
        assert data_bob["source"] == "overlay"

        # Read base (no user_id)
        base_read = ReadContentInput(book_id=book_id, path=path)
        result_base = await read_content(base_read, mock_context)
        data_base = json.loads(result_base)
        assert "# Base Content" in data_base["content"]

    @pytest.mark.asyncio
    async def test_delete_one_user_overlay_preserves_others(self, setup_fs_backend, mock_context):
        """Deleting one user's overlay doesn't affect other users."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        path = f"content/01-Part/01-Chapter/01-delete-isolation{unique_id}.md"
        user_alice = f"alice-{unique_id}"
        user_bob = f"bob-{unique_id}"

        op = get_operator()

        # Create base
        await op.write(f"books/{book_id}/{path}", b"# Base")

        # Create overlays for both users
        for user_id, content in [(user_alice, "Alice"), (user_bob, "Bob")]:
            params = WriteContentInput(
                book_id=book_id,
                path=path,
                content=f"# {content}'s Content",
                user_id=user_id
            )
            await write_content(params, mock_context)

        # Delete Alice's overlay
        delete_params = DeleteContentInput(book_id=book_id, path=path, user_id=user_alice)
        await delete_content(delete_params, mock_context)

        # Alice now sees base
        alice_read = ReadContentInput(book_id=book_id, path=path, user_id=user_alice)
        result_alice = await read_content(alice_read, mock_context)
        data_alice = json.loads(result_alice)
        assert data_alice["source"] == "base"

        # Bob still sees his overlay
        bob_read = ReadContentInput(book_id=book_id, path=path, user_id=user_bob)
        result_bob = await read_content(bob_read, mock_context)
        data_bob = json.loads(result_bob)
        assert data_bob["source"] == "overlay"
        assert "Bob" in data_bob["content"]
