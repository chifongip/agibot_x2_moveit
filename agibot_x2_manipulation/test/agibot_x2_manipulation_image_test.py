import importlib.util
from pathlib import Path


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "best_effort_image_decompressor.py"
)


def load_decompressor_module():
    spec = importlib.util.spec_from_file_location("best_effort_image_decompressor", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
