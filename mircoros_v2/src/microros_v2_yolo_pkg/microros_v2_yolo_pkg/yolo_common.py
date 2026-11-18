#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Common utilities for microros_v2_yolo_pkg.

Target environment:
- Ubuntu 22.04
- ROS2 Humble
- CPU inference
- Ultralytics .pt models
"""

from pathlib import Path


WORKSPACE_ROOT = Path("/home/yahboom")

MODEL_DIR = WORKSPACE_ROOT / "MODELS" / "yolo"

DEFAULT_YOLO11N_MODEL = MODEL_DIR / "yolo11n.pt"
DEFAULT_YOLO26N_MODEL = MODEL_DIR / "yolo26n.pt"

RAW_IMAGE_DIR = MODEL_DIR / "assets"
OUTPUT_IMAGE_DIR = MODEL_DIR / "assets" / "output"

RAW_VIDEO_DIR = MODEL_DIR / "videos"
OUTPUT_VIDEO_DIR = MODEL_DIR / "videos" / "output"

MODEL_MAP = {
    "yolo11n": DEFAULT_YOLO11N_MODEL,
    "yolo11n.pt": DEFAULT_YOLO11N_MODEL,
    "yolo26n": DEFAULT_YOLO26N_MODEL,
    "yolo26n.pt": DEFAULT_YOLO26N_MODEL,
}


def as_bool(value) -> bool:
    """Convert ROS/string bool-like values to bool."""
    if isinstance(value, bool):
        return value

    if value is None:
        return False

    text = str(value).strip().lower()
    return text in ("1", "true", "yes", "y", "on")


def ensure_dir(path: Path) -> Path:
    """Create directory if it does not exist."""
    path = Path(path).expanduser()
    path.mkdir(parents=True, exist_ok=True)
    return path


def check_file_exists(path: Path, description: str = "file") -> Path:
    """Raise clear error if a file does not exist."""
    path = Path(path).expanduser()

    if not path.exists():
        raise FileNotFoundError(f"{description} does not exist: {path}")

    if not path.is_file():
        raise FileNotFoundError(f"{description} is not a file: {path}")

    return path


def resolve_model_path(model_name: str, model_path: str) -> Path:
    """
    Resolve final model path.

    Priority:
    1. Explicit model_path
    2. model_name in MODEL_MAP
    3. MODEL_DIR / model_name
    4. MODEL_DIR / (model_name + ".pt")
    """
    model_path = str(model_path).strip()
    model_name = str(model_name).strip()

    if model_path:
        return Path(model_path).expanduser()

    if model_name in MODEL_MAP:
        return MODEL_MAP[model_name]

    candidate = Path(model_name).expanduser()

    if candidate.is_absolute():
        return candidate

    if candidate.suffix:
        return MODEL_DIR / candidate.name

    return MODEL_DIR / f"{model_name}.pt"


def get_model_tag(model_path: Path) -> str:
    """
    Get model tag for output filename.

    Examples:
        /xxx/yolo11n.pt  -> yolo11n
        /xxx/yolo26n.pt  -> yolo26n
        /xxx/best.pt     -> best
        /xxx/model.onnx  -> model
    """
    model_path = Path(model_path).expanduser()
    tag = model_path.stem.strip()

    if not tag:
        tag = "unknown_model"

    return sanitize_filename_part(tag)


def sanitize_filename_part(text: str) -> str:
    """
    Sanitize text for filename part.

    Keep letters, numbers, '-', '_', '.'.
    Replace other characters with '_'.
    """
    text = str(text).strip()

    if not text:
        return "unknown"

    safe_chars = []

    for ch in text:
        if ch.isalnum() or ch in ("-", "_", "."):
            safe_chars.append(ch)
        else:
            safe_chars.append("_")

    safe = "".join(safe_chars).strip("._-")

    return safe if safe else "unknown"


def ensure_model_tag_in_output_path(output_path: Path, model_tag: str) -> Path:
    """
    Ensure output filename contains model tag.

    If output filename already contains the model tag, keep it unchanged.
    Otherwise append _{model_tag} before suffix.

    Examples:
        result.jpg + yolo11n -> result_yolo11n.jpg
        result_yolo11n.jpg + yolo11n -> result_yolo11n.jpg
    """
    output_path = Path(output_path).expanduser()
    model_tag = sanitize_filename_part(model_tag)

    stem = output_path.stem
    suffix = output_path.suffix

    if model_tag in stem:
        return output_path

    new_name = f"{stem}_{model_tag}{suffix}"
    return output_path.with_name(new_name)


def resolve_input_path(input_path: str, input_dir: str, file_name: str, default_file_name: str) -> Path:
    """
    Resolve input file path.

    Priority:
    1. Explicit full input_path
    2. input_dir + file_name
    3. input_dir + default_file_name
    """
    input_path = str(input_path).strip()
    file_name = str(file_name).strip()
    input_dir = Path(str(input_dir).strip()).expanduser()

    if input_path:
        return Path(input_path).expanduser()

    if file_name:
        return input_dir / file_name

    return input_dir / default_file_name


def build_output_path(
    input_path: Path,
    output_dir: str,
    model_path: Path,
    task_suffix: str,
    output_path: str = "",
    force_ext: str = "",
) -> Path:
    """
    Build output path and always mark selected model in filename.

    If output_path is set:
        result.jpg + yolo11n.pt -> result_yolo11n.jpg

    If output_path is not set:
        input_stem + "_" + model_tag + task_suffix + ext

    Examples:
        bus.jpg + yolo11n.pt + _detect
            -> bus_yolo11n_detect.jpg

        street.mp4 + yolo26n.pt + _detect
            -> street_yolo26n_detect.mp4
    """
    output_path = str(output_path).strip()
    input_path = Path(input_path).expanduser()
    model_tag = get_model_tag(model_path)

    if force_ext:
        ext = force_ext
    else:
        ext = input_path.suffix

    if not ext:
        ext = ".out"

    if not ext.startswith("."):
        ext = f".{ext}"

    if output_path:
        output = Path(output_path).expanduser()

        if output.suffix:
            output = output.with_suffix(ext)
        else:
            output = output / f"output_{model_tag}{task_suffix}{ext}"

        output = ensure_model_tag_in_output_path(output, model_tag)
        ensure_dir(output.parent)
        return output

    output_dir_path = ensure_dir(Path(output_dir).expanduser())

    filename = f"{input_path.stem}_{model_tag}{task_suffix}{ext}"
    output = output_dir_path / filename

    ensure_dir(output.parent)
    return output


def import_yolo():
    """
    Import Ultralytics YOLO with a clear error message.
    """
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise ImportError(
            "Failed to import ultralytics. Install it in your current Python environment:\n"
            "  pip install ultralytics\n"
        ) from exc

    return YOLO
