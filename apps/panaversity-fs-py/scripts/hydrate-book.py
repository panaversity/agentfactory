#!/usr/bin/env python
"""Entry point for hydrate-book CLI.

Downloads content from PanaversityFS for Docusaurus builds.
Uses manifest-based delta detection for incremental builds.

Usage:
    python scripts/hydrate-book.py --book-id ai-native-python
    python scripts/hydrate-book.py --book-id ai-native-python --full-rebuild
    python scripts/hydrate-book.py --help
"""

import sys
from pathlib import Path

# Add scripts directory to path for imports
scripts_dir = Path(__file__).parent
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir.parent))

from scripts.hydrate.cli import main

if __name__ == "__main__":
    main()
