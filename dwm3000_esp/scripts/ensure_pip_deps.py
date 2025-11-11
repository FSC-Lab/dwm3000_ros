#!/usr/bin/env python3

import importlib
import subprocess
import sys


def installed(module_name):
    try:
        importlib.import_module(module_name)
        return True
    except ImportError:
        return False


def pip_install(pkg):
    print(f"Installing {pkg} into {sys.executable} environment...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "--no-input", pkg])


# Map importable module -> pip package name
checks = {
    "google.protobuf": "protobuf",
    "grpc_tools": "grpcio-tools",
}

if __name__ == "__main__":
    for module, pip_pkg in checks.items():
        if not installed(module):
            pip_install(pip_pkg)
