# -*- coding: utf-8 -*-
"""Top-level package for ROSCO Repo."""
import importlib
import importlib.metadata
import re
from pathlib import Path

__author__ = """Nikhar J. Abbas and Daniel S. Zalkind"""
__email__ = 'daniel.zalkind@nrel.gov'

def _source_version():
    pyproject = Path(__file__).resolve().parents[2] / "pyproject.toml"
    if pyproject.exists():
        match = re.search(r'^version\s*=\s*"([^"]+)"', pyproject.read_text(), re.MULTILINE)
        if match:
            return match.group(1)
    return importlib.metadata.version("rosco")


__version__ = _source_version()

