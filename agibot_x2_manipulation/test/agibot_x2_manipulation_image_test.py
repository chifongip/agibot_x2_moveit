import importlib.util
from pathlib import Path


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "best_effort_image_decompressor.py"
)
RAW_THROTTLER_SCRIPT = Path(__file__).parents[1] / "scripts" / "raw_image_throttler.py"


def load_decompressor_module():
    spec = importlib.util.spec_from_file_location("best_effort_image_decompressor", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_raw_throttler_module():
    spec = importlib.util.spec_from_file_location("raw_image_throttler", RAW_THROTTLER_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
