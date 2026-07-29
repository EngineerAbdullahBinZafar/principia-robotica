"""
Principia Robotica — 1-Second Instant Launcher Script

Author: Abdullah Bin Zafar <abz.king.1.9.2003@gmail.com>
License: MIT

Usage:
    python run.py          # Instant launch Web UI Dashboard in browser
    python run.py --server # Start MCP stdio server
    python run.py --doctor # Run system diagnostics
"""

import os
import sys

# Add package root to Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from principia.server import __version__, run_doctor, start_ui_server
from principia.server import main as start_mcp_server


def main():
    if "--doctor" in sys.argv:
        run_doctor()
    elif "--server" in sys.argv:
        start_mcp_server()
    elif "--help" in sys.argv or "-h" in sys.argv:
        print(f"\n⚡ Principia Robotica v{__version__} — Instant Launcher")
        print("Usage:")
        print("  python run.py          Launch 60 FPS Web UI Visualizer Dashboard in browser")
        print("  python run.py --server Launch MCP stdio server (JSON-RPC 2.0)")
        print("  python run.py --doctor Run system diagnostic suite\n")
    else:
        port = 8080
        for i, arg in enumerate(sys.argv):
            if arg == "--port" and i + 1 < len(sys.argv):
                port = int(sys.argv[i + 1])
        start_ui_server(port=port)

if __name__ == "__main__":
    main()
