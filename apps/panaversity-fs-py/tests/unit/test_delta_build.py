"""Unit tests for delta_build and plan_build tools (FR-025, FR-026, FR-027).

Tests incremental build detection via:
- delta_build: FileJournal timestamps
- plan_build: Manifest hash comparison with stored snapshots
"""

import pytest
import json
from datetime import datetime, timezone, timedelta
from panaversity_fs.tools.delta import delta_build, plan_build
from panaversity_fs.tools.content import write_content
from panaversity_fs.models import DeltaBuildInput, WriteContentInput, PlanBuildInput
from panaversity_fs.storage import get_operator


class TestDeltaBuildBasic:
    """Basic tests for delta_build tool."""

    @pytest.mark.asyncio
    async def test_delta_build_no_changes(self, setup_fs_backend, mock_context):
        """Returns empty when no changes since timestamp."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-{unique_id}"

        # Use future timestamp - nothing should be after it
        future_ts = (datetime.now(timezone.utc) + timedelta(days=1)).isoformat().replace('+00:00', 'Z')

        params = DeltaBuildInput(book_id=book_id, since=future_ts)
        result = await delta_build(params, mock_context)
        data = json.loads(result)

        assert data["changed_count"] == 0
        assert data["changed_files"] == []

    @pytest.mark.asyncio
    async def test_delta_build_detects_changes(self, setup_fs_backend, mock_context):
        """Detects files changed since timestamp."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-{unique_id}"

        # Get timestamp before creating content
        before_ts = datetime.now(timezone.utc)

        # Create content - use valid NN-Name format (FR-007 schema)
        write_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-lesson{unique_id}.md",
            content="# Test Lesson"
        )
        await write_content(write_params, mock_context)

        # Query for changes since before creation
        since_ts = (before_ts - timedelta(seconds=1)).isoformat().replace('+00:00', 'Z')

        delta_params = DeltaBuildInput(book_id=book_id, since=since_ts)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        assert data["changed_count"] >= 1
        assert len(data["changed_files"]) >= 1
        assert any(unique_id in f["path"] for f in data["changed_files"])

    @pytest.mark.asyncio
    async def test_delta_build_excludes_old_files(self, setup_fs_backend, mock_context):
        """Excludes files modified before timestamp."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-{unique_id}"

        # Create content - use valid NN-Name format (FR-007 schema)
        write_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-old{unique_id}.md",
            content="# Old Lesson"
        )
        await write_content(write_params, mock_context)

        # Use future timestamp
        future_ts = (datetime.now(timezone.utc) + timedelta(days=1)).isoformat().replace('+00:00', 'Z')

        delta_params = DeltaBuildInput(book_id=book_id, since=future_ts)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        # Should not include the old file
        assert data["changed_count"] == 0


class TestDeltaBuildWithContent:
    """Tests for include_content parameter."""

    @pytest.mark.asyncio
    async def test_delta_build_without_content(self, setup_fs_backend, mock_context):
        """Default: does not include file content."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-content-{unique_id}"

        # Get timestamp before
        before_ts = datetime.now(timezone.utc)

        # Create content - use valid NN-Name format (FR-007 schema)
        write_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-lesson{unique_id}.md",
            content="# Test Content Here"
        )
        await write_content(write_params, mock_context)

        since_ts = (before_ts - timedelta(seconds=1)).isoformat().replace('+00:00', 'Z')

        delta_params = DeltaBuildInput(book_id=book_id, since=since_ts, include_content=False)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        assert data["changed_count"] >= 1
        # Content should NOT be included
        for f in data["changed_files"]:
            assert "content" not in f

    @pytest.mark.asyncio
    async def test_delta_build_with_content(self, setup_fs_backend, mock_context):
        """include_content=True includes file content."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-content-{unique_id}"

        # Get timestamp before
        before_ts = datetime.now(timezone.utc)

        # Create content with known text - use valid NN-Name format (FR-007 schema)
        write_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-lesson{unique_id}.md",
            content="# Test Content Here"
        )
        await write_content(write_params, mock_context)

        since_ts = (before_ts - timedelta(seconds=1)).isoformat().replace('+00:00', 'Z')

        delta_params = DeltaBuildInput(book_id=book_id, since=since_ts, include_content=True)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        assert data["changed_count"] >= 1
        # Content SHOULD be included
        content_file = next((f for f in data["changed_files"] if unique_id in f["path"]), None)
        assert content_file is not None
        assert "content" in content_file
        assert "Test Content Here" in content_file["content"]


class TestDeltaBuildOverlay:
    """Tests for overlay support in delta_build."""

    @pytest.mark.asyncio
    async def test_delta_build_base_only_by_default(self, setup_fs_backend, mock_context):
        """Without user_id, only returns base changes."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-overlay-{unique_id}"
        user_id = f"user-{unique_id}"

        before_ts = datetime.now(timezone.utc)

        # Create base content - use valid NN-Name format (FR-007 schema)
        base_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-base{unique_id}.md",
            content="# Base Lesson"
        )
        await write_content(base_params, mock_context)

        # Create overlay content - use valid NN-Name format (FR-007 schema)
        overlay_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/02-overlay{unique_id}.md",
            content="# Overlay Lesson",
            user_id=user_id
        )
        await write_content(overlay_params, mock_context)

        since_ts = (before_ts - timedelta(seconds=1)).isoformat().replace('+00:00', 'Z')

        # Query without user_id
        delta_params = DeltaBuildInput(book_id=book_id, since=since_ts)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        # Should only have base changes
        assert all(f["namespace"] == "base" for f in data["changed_files"])
        assert any("01-base" in f["path"] for f in data["changed_files"])
        assert not any("02-overlay" in f["path"] for f in data["changed_files"])

    @pytest.mark.asyncio
    async def test_delta_build_with_overlay(self, setup_fs_backend, mock_context):
        """With user_id, returns both base and overlay changes."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-overlay-{unique_id}"
        user_id = f"user-{unique_id}"

        before_ts = datetime.now(timezone.utc)

        # Create base content - use valid NN-Name format (FR-007 schema)
        base_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-base{unique_id}.md",
            content="# Base Lesson"
        )
        await write_content(base_params, mock_context)

        # Create overlay content - use valid NN-Name format (FR-007 schema)
        overlay_params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/02-overlay{unique_id}.md",
            content="# Overlay Lesson",
            user_id=user_id
        )
        await write_content(overlay_params, mock_context)

        since_ts = (before_ts - timedelta(seconds=1)).isoformat().replace('+00:00', 'Z')

        # Query WITH user_id
        delta_params = DeltaBuildInput(book_id=book_id, since=since_ts, user_id=user_id)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        # Should have both base and overlay changes
        base_files = [f for f in data["changed_files"] if f["namespace"] == "base"]
        overlay_files = [f for f in data["changed_files"] if f["namespace"] == "overlay"]

        assert len(base_files) >= 1
        assert len(overlay_files) >= 1
        assert data["base_changes"] >= 1
        assert data["overlay_changes"] >= 1


class TestDeltaBuildTimestamp:
    """Tests for timestamp handling."""

    @pytest.mark.asyncio
    async def test_delta_build_valid_utc_timestamp(self, setup_fs_backend, mock_context):
        """Accepts valid UTC timestamp with Z suffix."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]

        params = DeltaBuildInput(
            book_id=f"delta-ts-{unique_id}",
            since="2025-01-01T00:00:00Z"
        )
        result = await delta_build(params, mock_context)
        data = json.loads(result)

        assert "changed_count" in data
        assert data["since"] == "2025-01-01T00:00:00Z"

    @pytest.mark.asyncio
    async def test_delta_build_valid_offset_timestamp(self, setup_fs_backend, mock_context):
        """Accepts valid timestamp with offset."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]

        params = DeltaBuildInput(
            book_id=f"delta-ts-{unique_id}",
            since="2025-01-01T00:00:00+00:00"
        )
        result = await delta_build(params, mock_context)
        data = json.loads(result)

        assert "changed_count" in data


class TestDeltaBuildMultipleChanges:
    """Tests for multiple file changes."""

    @pytest.mark.asyncio
    async def test_delta_build_multiple_files(self, setup_fs_backend, mock_context):
        """Tracks multiple changed files correctly."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-multi-{unique_id}"

        before_ts = datetime.now(timezone.utc)

        # Create multiple files - use valid NN-Name format (FR-007 schema)
        for i in range(3):
            params = WriteContentInput(
                book_id=book_id,
                path=f"content/01-Part/01-Chapter/{i:02d}-lesson{unique_id}.md",
                content=f"# Lesson {i}"
            )
            await write_content(params, mock_context)

        since_ts = (before_ts - timedelta(seconds=1)).isoformat().replace('+00:00', 'Z')

        delta_params = DeltaBuildInput(book_id=book_id, since=since_ts)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        assert data["changed_count"] == 3
        assert len(data["changed_files"]) == 3

    @pytest.mark.asyncio
    async def test_delta_build_returns_sha256(self, setup_fs_backend, mock_context):
        """Changed files include sha256 hash."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"delta-hash-{unique_id}"

        before_ts = datetime.now(timezone.utc)

        # Use valid NN-Name format (FR-007 schema)
        params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-lesson{unique_id}.md",
            content="# Lesson with hash"
        )
        await write_content(params, mock_context)

        since_ts = (before_ts - timedelta(seconds=1)).isoformat().replace('+00:00', 'Z')

        delta_params = DeltaBuildInput(book_id=book_id, since=since_ts)
        result = await delta_build(delta_params, mock_context)
        data = json.loads(result)

        # All files should have sha256
        for f in data["changed_files"]:
            assert "sha256" in f
            assert len(f["sha256"]) == 64  # SHA256 hex length


# =============================================================================
# plan_build Tests (FR-025, FR-026, FR-027)
# =============================================================================

class TestPlanBuildFirstBuild:
    """Tests for first build (no target hash) - FR-025 scenario 3."""

    @pytest.mark.asyncio
    async def test_plan_build_first_build_returns_all_files(self, setup_fs_backend, mock_context):
        """First build without target_manifest_hash returns all files."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-first-{unique_id}"

        # Create some content
        for i in range(3):
            params = WriteContentInput(
                book_id=book_id,
                path=f"content/01-Part/01-Chapter/{i:02d}-lesson{unique_id}.md",
                content=f"# Lesson {i}"
            )
            await write_content(params, mock_context)

        # Call plan_build without target hash
        plan_params = PlanBuildInput(book_id=book_id)
        result = await plan_build(plan_params, mock_context)
        data = json.loads(result)

        assert data["status"] == "changed"
        assert data["changed_count"] == 3
        assert data["total_files"] == 3
        assert len(data["files"]) == 3
        assert "manifest_hash" in data
        assert len(data["manifest_hash"]) == 64  # SHA256 hex length

    @pytest.mark.asyncio
    async def test_plan_build_empty_book(self, setup_fs_backend, mock_context):
        """First build on empty book returns empty files list."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-empty-{unique_id}"

        plan_params = PlanBuildInput(book_id=book_id)
        result = await plan_build(plan_params, mock_context)
        data = json.loads(result)

        # Empty book should still have changed status (first build)
        assert data["status"] == "changed"
        assert data["changed_count"] == 0
        assert data["total_files"] == 0
        assert data["files"] == []


class TestPlanBuildUnchanged:
    """Tests for unchanged state - FR-025 scenario 2."""

    @pytest.mark.asyncio
    async def test_plan_build_unchanged_same_hash(self, setup_fs_backend, mock_context):
        """Same manifest hash returns status=unchanged and empty files."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-unchanged-{unique_id}"

        # Create content
        params = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-lesson{unique_id}.md",
            content="# Lesson 1"
        )
        await write_content(params, mock_context)

        # First call to get current manifest hash
        plan_params = PlanBuildInput(book_id=book_id)
        result1 = await plan_build(plan_params, mock_context)
        data1 = json.loads(result1)
        manifest_hash = data1["manifest_hash"]

        # Second call with same manifest hash - should be unchanged
        plan_params2 = PlanBuildInput(book_id=book_id, target_manifest_hash=manifest_hash)
        result2 = await plan_build(plan_params2, mock_context)
        data2 = json.loads(result2)

        assert data2["status"] == "unchanged"
        assert data2["changed_count"] == 0
        assert data2["files"] == []
        assert data2["manifest_hash"] == manifest_hash


class TestPlanBuildDelta:
    """Tests for actual delta computation - FR-025 scenario 1, FR-026."""

    @pytest.mark.asyncio
    async def test_plan_build_detects_added_file(self, setup_fs_backend, mock_context):
        """Returns only added files when file is added after snapshot."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-add-{unique_id}"

        # Create initial content
        params1 = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-initial{unique_id}.md",
            content="# Initial Lesson"
        )
        await write_content(params1, mock_context)

        # Get initial manifest hash
        plan_params1 = PlanBuildInput(book_id=book_id)
        result1 = await plan_build(plan_params1, mock_context)
        data1 = json.loads(result1)
        initial_hash = data1["manifest_hash"]

        # Add a new file
        params2 = WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/02-added{unique_id}.md",
            content="# Added Lesson"
        )
        await write_content(params2, mock_context)

        # Plan build with initial hash - should show only the added file
        plan_params2 = PlanBuildInput(book_id=book_id, target_manifest_hash=initial_hash)
        result2 = await plan_build(plan_params2, mock_context)
        data2 = json.loads(result2)

        assert data2["status"] == "changed"
        assert data2["changed_count"] == 1  # Only the NEW file
        assert len(data2["files"]) == 1
        assert "02-added" in data2["files"][0]["path"]
        assert data2["files"][0]["change_type"] == "added"
        assert data2["files"][0]["target_hash"] is None  # New file has no target hash

    @pytest.mark.asyncio
    async def test_plan_build_detects_modified_file(self, setup_fs_backend, mock_context):
        """Returns only modified files with current and target hashes."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-mod-{unique_id}"

        # Create initial content
        file_path = f"content/01-Part/01-Chapter/01-lesson{unique_id}.md"
        params1 = WriteContentInput(
            book_id=book_id,
            path=file_path,
            content="# Original Content"
        )
        result = await write_content(params1, mock_context)
        original_data = json.loads(result)
        original_hash = original_data["file_hash"]

        # Get initial manifest hash
        plan_params1 = PlanBuildInput(book_id=book_id)
        result1 = await plan_build(plan_params1, mock_context)
        data1 = json.loads(result1)
        initial_manifest = data1["manifest_hash"]

        # Modify the file
        params2 = WriteContentInput(
            book_id=book_id,
            path=file_path,
            content="# Modified Content",
            expected_hash=original_hash
        )
        await write_content(params2, mock_context)

        # Plan build with initial manifest - should show the modified file
        plan_params2 = PlanBuildInput(book_id=book_id, target_manifest_hash=initial_manifest)
        result2 = await plan_build(plan_params2, mock_context)
        data2 = json.loads(result2)

        assert data2["status"] == "changed"
        assert data2["changed_count"] == 1
        assert len(data2["files"]) == 1
        modified_file = data2["files"][0]
        assert modified_file["change_type"] == "modified"
        assert modified_file["target_hash"] == original_hash  # FR-026: target_hash is present
        assert modified_file["current_hash"] != original_hash  # Hash changed

    @pytest.mark.asyncio
    async def test_plan_build_detects_deleted_file(self, setup_fs_backend, mock_context):
        """Returns deleted files with current_hash=None."""
        import uuid
        from panaversity_fs.tools.content import delete_content
        from panaversity_fs.models import DeleteContentInput

        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-del-{unique_id}"

        # Create initial content with two files
        file1_path = f"content/01-Part/01-Chapter/01-keep{unique_id}.md"
        file2_path = f"content/01-Part/01-Chapter/02-delete{unique_id}.md"

        await write_content(WriteContentInput(
            book_id=book_id,
            path=file1_path,
            content="# Keep This"
        ), mock_context)
        await write_content(WriteContentInput(
            book_id=book_id,
            path=file2_path,
            content="# Delete This"
        ), mock_context)

        # Get initial manifest hash
        plan_params1 = PlanBuildInput(book_id=book_id)
        result1 = await plan_build(plan_params1, mock_context)
        data1 = json.loads(result1)
        initial_manifest = data1["manifest_hash"]
        assert data1["total_files"] == 2

        # Delete one file
        delete_params = DeleteContentInput(book_id=book_id, path=file2_path)
        await delete_content(delete_params, mock_context)

        # Plan build with initial manifest - should show the deleted file
        plan_params2 = PlanBuildInput(book_id=book_id, target_manifest_hash=initial_manifest)
        result2 = await plan_build(plan_params2, mock_context)
        data2 = json.loads(result2)

        assert data2["status"] == "changed"
        assert data2["total_files"] == 1  # Only 1 file remains
        assert data2["changed_count"] == 1  # 1 change (deletion)

        deleted_file = data2["files"][0]
        assert deleted_file["change_type"] == "deleted"
        assert deleted_file["current_hash"] is None  # Deleted = no current hash
        assert deleted_file["target_hash"] is not None  # Has the old hash

    @pytest.mark.asyncio
    async def test_plan_build_multiple_changes(self, setup_fs_backend, mock_context):
        """Detects added, modified, and unchanged files correctly."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-multi-{unique_id}"

        # Create 3 initial files
        keep_path = f"content/01-Part/01-Chapter/01-keep{unique_id}.md"
        modify_path = f"content/01-Part/01-Chapter/02-modify{unique_id}.md"
        unchanged_path = f"content/01-Part/01-Chapter/03-unchanged{unique_id}.md"

        await write_content(WriteContentInput(
            book_id=book_id,
            path=keep_path,
            content="# Keep"
        ), mock_context)
        modify_result = await write_content(WriteContentInput(
            book_id=book_id,
            path=modify_path,
            content="# Modify Original"
        ), mock_context)
        await write_content(WriteContentInput(
            book_id=book_id,
            path=unchanged_path,
            content="# Unchanged"
        ), mock_context)

        modify_original_hash = json.loads(modify_result)["file_hash"]

        # Get initial manifest
        result1 = await plan_build(PlanBuildInput(book_id=book_id), mock_context)
        initial_manifest = json.loads(result1)["manifest_hash"]

        # Make changes: modify one file, add one file (leave one unchanged)
        await write_content(WriteContentInput(
            book_id=book_id,
            path=modify_path,
            content="# Modified Content",
            expected_hash=modify_original_hash
        ), mock_context)
        await write_content(WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/04-added{unique_id}.md",
            content="# Added"
        ), mock_context)

        # Plan build with initial manifest
        result2 = await plan_build(PlanBuildInput(
            book_id=book_id,
            target_manifest_hash=initial_manifest
        ), mock_context)
        data2 = json.loads(result2)

        assert data2["status"] == "changed"
        assert data2["changed_count"] == 2  # 1 modified + 1 added
        assert data2["total_files"] == 4  # Original 3 + 1 added

        # Check that unchanged file is NOT in the list
        changed_paths = [f["path"] for f in data2["files"]]
        assert not any("03-unchanged" in p for p in changed_paths)
        assert any("02-modify" in p for p in changed_paths)
        assert any("04-added" in p for p in changed_paths)


class TestPlanBuildManifestNotFound:
    """Tests for handling unknown target manifests."""

    @pytest.mark.asyncio
    async def test_plan_build_unknown_manifest_returns_warning(self, setup_fs_backend, mock_context):
        """Unknown target manifest returns all files with warning."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-unknown-{unique_id}"

        # Create content
        await write_content(WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-lesson{unique_id}.md",
            content="# Lesson"
        ), mock_context)

        # Use a fake manifest hash that doesn't exist
        fake_hash = "a" * 64

        plan_params = PlanBuildInput(book_id=book_id, target_manifest_hash=fake_hash)
        result = await plan_build(plan_params, mock_context)
        data = json.loads(result)

        # Should return all files with a warning
        assert data["status"] == "changed"
        assert data["changed_count"] == 1  # All files returned
        assert "warning" in data
        assert fake_hash in data["warning"]


class TestPlanBuildResponseFormat:
    """Tests for FR-026 response format compliance."""

    @pytest.mark.asyncio
    async def test_plan_build_response_has_required_fields(self, setup_fs_backend, mock_context):
        """Response contains all required FR-026 fields."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"plan-format-{unique_id}"

        await write_content(WriteContentInput(
            book_id=book_id,
            path=f"content/01-Part/01-Chapter/01-lesson{unique_id}.md",
            content="# Lesson"
        ), mock_context)

        result = await plan_build(PlanBuildInput(book_id=book_id), mock_context)
        data = json.loads(result)

        # Check required fields per FR-026
        assert "status" in data
        assert data["status"] in ["changed", "unchanged"]
        assert "manifest_hash" in data
        assert "files" in data
        assert isinstance(data["files"], list)
        assert "changed_count" in data
        assert "total_files" in data

        # Check file format
        if data["files"]:
            file_entry = data["files"][0]
            assert "path" in file_entry
            assert "current_hash" in file_entry
