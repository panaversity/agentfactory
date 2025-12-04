"""Storage abstraction using OpenDAL Python bindings.

Provides unified AsyncOperator interface for all storage backends:
- fs: Local filesystem (development)
- s3: AWS S3 / Cloudflare R2 (production)
- supabase: Supabase Storage
"""

import opendal
from panaversity_fs.config import get_config, Config


# Global operator instance (singleton pattern)
_operator: opendal.AsyncOperator | None = None


def _create_fs_operator(config: Config) -> opendal.AsyncOperator:
    """Create local filesystem operator.

    Args:
        config: Configuration instance.

    Returns:
        opendal.AsyncOperator: Configured filesystem operator.
    """
    return opendal.AsyncOperator("fs", root=config.storage_root)


def _create_s3_operator(config: Config) -> opendal.AsyncOperator:
    """Create S3/R2 operator.

    Args:
        config: Configuration instance.

    Returns:
        opendal.AsyncOperator: Configured S3 operator.

    Raises:
        ValueError: If required S3 configuration is missing.
    """
    if not config.s3_bucket:
        raise ValueError("S3_BUCKET required for s3 backend")
    if not config.s3_access_key_id or not config.s3_secret_access_key:
        raise ValueError("S3_ACCESS_KEY_ID and S3_SECRET_ACCESS_KEY required")

    operator_config = {
        "bucket": config.s3_bucket,
        "region": config.s3_region,
        "access_key_id": config.s3_access_key_id,
        "secret_access_key": config.s3_secret_access_key,
    }

    # Add endpoint for Cloudflare R2
    if config.s3_endpoint:
        operator_config["endpoint"] = config.s3_endpoint

    return opendal.AsyncOperator("s3", **operator_config)


def _create_supabase_operator(config: Config) -> opendal.AsyncOperator:
    """Create Supabase Storage operator.

    Args:
        config: Configuration instance.

    Returns:
        opendal.AsyncOperator: Configured Supabase operator.

    Raises:
        ValueError: If required Supabase configuration is missing.
    """
    if not config.supabase_bucket:
        raise ValueError("SUPABASE_BUCKET required for supabase backend")
    if not config.supabase_url or not config.supabase_service_role_key:
        raise ValueError("SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY required")

    return opendal.AsyncOperator(
        "supabase",
        bucket=config.supabase_bucket,
        endpoint=config.supabase_url,
        key=config.supabase_service_role_key
    )


def get_operator() -> opendal.AsyncOperator:
    """Get or create the global OpenDAL operator instance.

    Operator is configured based on Config.storage_backend:
    - "fs": Local filesystem (Config.storage_root)
    - "s3": AWS S3 or Cloudflare R2 (Config.s3_*)
    - "supabase": Supabase Storage (Config.supabase_*)

    Returns:
        opendal.AsyncOperator: Configured async operator for storage operations.

    Raises:
        ValueError: If storage backend configuration is invalid.

    Example:
        ```python
        op = get_operator()

        # Write file
        await op.write("books/test/lesson.md", b"# Hello")

        # Read file
        content = await op.read("books/test/lesson.md")

        # Get metadata
        metadata = await op.stat("books/test/lesson.md")
        print(f"Size: {metadata.content_length} bytes")

        # List directory
        entries = await op.list("books/test/")
        for entry in entries:
            print(entry.path)

        # Delete file
        await op.delete("books/test/lesson.md")
        ```
    """
    global _operator

    if _operator is None:
        config = get_config()

        if config.storage_backend == "fs":
            _operator = _create_fs_operator(config)
        elif config.storage_backend == "s3":
            _operator = _create_s3_operator(config)
        elif config.storage_backend == "supabase":
            _operator = _create_supabase_operator(config)
        else:
            raise ValueError(f"Unknown storage backend: {config.storage_backend}")

    return _operator


async def health_check() -> dict[str, str]:
    """Check storage backend health.

    Returns:
        dict: Health status {"status": "healthy"/"unhealthy", "backend": "fs"/"s3"/"supabase"}

    Example:
        ```python
        health = await health_check()
        # {"status": "healthy", "backend": "fs"}
        ```
    """
    try:
        config = get_config()
        op = get_operator()

        # Try a simple operation to verify backend is accessible
        # Write and immediately delete a test file
        test_path = ".health_check_test"
        await op.write(test_path, b"test")
        await op.delete(test_path)

        return {
            "status": "healthy",
            "backend": config.storage_backend
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "backend": config.storage_backend,
            "error": str(e)
        }
