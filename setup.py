# Minimal shim so that older pip/setuptools (e.g. JetPack 4.x's pip, which
# predates pyproject.toml/PEP 517 support) can install this package. All
# metadata lives in setup.cfg; modern tools use pyproject.toml.
from setuptools import setup

setup()
