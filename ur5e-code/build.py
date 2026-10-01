#!/usr/bin/env python3
"""Rebuild the offline page from src/ur5e.html (Python standard library only)."""
import importlib.util
from pathlib import Path
import sys

sys.dont_write_bytecode = True
root = Path(__file__).resolve().parent
renderer_path = root / "tools" / "scripts" / "render.py"
spec = importlib.util.spec_from_file_location("ur5e_page_renderer", renderer_path)
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)
renderer.export_html(
    root / "src" / "ur5e.html",
    root / "index.html",
    title="UR5e · 六关节交互演示",
    force=True,
)
print("Built:", root / "index.html")
