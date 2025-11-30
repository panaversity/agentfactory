# Dependency Resolution Stage:
FROM python:3.13-slim AS requirements-stage
LABEL maintainer="devops-team"
WORKDIR /tmp

# Install uv package manager
RUN pip install --no-cache-dir uv==0.4.26

# Copy project files
COPY ./pyproject.toml ./uv.lock* /tmp/

# Export dependencies to requirements.txt using uv
RUN uv pip compile \
    --python-version 3.13 \
    pyproject.toml \
    --output-file requirements.txt

# Final Runtime Stage
FROM python:3.13-slim

# Set metadata
LABEL maintainer="devops-team" \
        description="Production-ready FastAPI application" \
        version="0.1.0"

# Set working directory
WORKDIR /code

# Security: Create non-root user before installing packages
RUN useradd --create-home --home-dir /home/appuser --uid 1000 appuser

# Copy requirements from requirements-stage
COPY --from=requirements-stage /tmp/requirements.txt /code/requirements.txt

# Install dependencies and clean up aggressively
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r /code/requirements.txt && \
    find /usr/local/lib/python3.13 -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true && \
    find /usr/local/lib/python3.13 -type f -name "*.py[cod]" -delete && \
    find /usr/local/lib/python3.13 -type f -name "*.dist-info/RECORD" -delete && \
    rm -rf /code/requirements.txt /tmp/* /var/tmp/* ~/.cache/pip

# Set environment variables
ENV PATH="/usr/local/bin:$PATH" \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH=/code

# Copy application code
# Copy all Python modules and main files (excludes tests, docs, etc. via .dockerignore)
COPY --chown=appuser:appuser . /code/

# Remove files that shouldn't be in production image
RUN rm -rf /code/tests /code/.pytest_cache /code/.venv /code/__pycache__ /code/*/__pycache__ /code/*/*/__pycache__ || true

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/health').read()" || exit 1

# Switch to non-root user for security
USER appuser

# Expose port
EXPOSE 8000

# Run application
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "2"]