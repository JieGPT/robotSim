"""Entry point — run robot validator demo.

Usage:
    python main.py                      # built-in demo with validation
    python main.py --yaml examples/mvp.yaml  # custom YAML scenario
    python main.py --visualize          # with viser 3D viewer
    python main.py --json > out.json    # JSON export
"""

from robot_validator.cli import app

if __name__ == "__main__":
    app()
