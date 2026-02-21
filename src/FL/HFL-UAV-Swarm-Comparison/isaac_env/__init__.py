"""
Isaac Sim / OmniDrones Environment for UAV Task Offloading.

This package provides an OmniDrones-compatible environment that mirrors
the observation/action/reward structure of the pure-Python UAVSwarmEnv
but uses GPU-parallelized drone physics via NVIDIA Isaac Sim.

Modes:
    - Full Isaac Sim (requires OmniDrones + Isaac Lab installed)
    - Standalone (pure PyTorch, no Isaac Sim needed — for dev/testing)
"""

__version__ = "0.1.0"

import os
import sys

def _setup_isaac_paths():
    isaac_root = os.environ.get("ISAACSIM_PATH")
    if not isaac_root:
        return
        
    import isaacsim
    try:
        import omni
    except ImportError:
        # Create a dummy omni module if not exists
        class Dummy: pass
        omni = Dummy()
        omni.__path__ = []
        sys.modules["omni"] = omni

    search_dirs = ["exts", "extsDeprecated", "extscache"]
    for sdir in search_dirs:
        full_sdir = os.path.join(isaac_root, sdir)
        if not os.path.exists(full_sdir):
            continue
        for ext_name in os.listdir(full_sdir):
            ext_path = os.path.join(full_sdir, ext_name)
            if not os.path.isdir(ext_path):
                continue
            
            # Add 'omni' and 'isaacsim' subfolders to respective __path__
            for pkg_name in ["omni", "isaacsim"]:
                pkg_subfolder = os.path.join(ext_path, pkg_name)
                if os.path.exists(pkg_subfolder):
                    pkg_mod = sys.modules.get(pkg_name)
                    if pkg_mod and hasattr(pkg_mod, "__path__"):
                        if pkg_subfolder not in pkg_mod.__path__:
                            pkg_mod.__path__.append(pkg_subfolder)
            
            if os.path.exists(os.path.join(ext_path, "pip_prebundle")):
                path_to_add = os.path.join(ext_path, "pip_prebundle")
                if path_to_add not in sys.path:
                    sys.path.append(path_to_add)

_setup_isaac_paths()

from .uav_task_offloading import UAVTaskOffloadingEnv
