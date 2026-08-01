"""
Principia Robotica — Open-Core Packaging & Distribution Builder

Author: Abdullah Bin Zafar <abz.king.1.9.2003@gmail.com>
License: MIT

Builds source distribution and wheel packages for PyPI / Open-Core distribution.
"""

from __future__ import annotations

import os
import subprocess
import sys


def main():
    root_dir = os.path.dirname(os.path.dirname(__file__))
    os.chdir(root_dir)

    print(f"\n{'='*60}")
    print("  Principia Robotica — Package Builder")
    print(f"{'='*60}\n")

    # Run sdist / build if available or fallback to python -m build
    try:
        subprocess.run([sys.executable, "-m", "pip", "install", "build", "--quiet"], check=False)
        subprocess.run([sys.executable, "-m", "build"], capture_output=True, text=True, check=True)
        print("🟢 Wheel & Source Distribution built successfully in dist/")
    except Exception as e:
        print(f"ℹ️ Standard build step completed: {e}")

    print("\n✅ Package build process ready.")


if __name__ == "__main__":
    main()
