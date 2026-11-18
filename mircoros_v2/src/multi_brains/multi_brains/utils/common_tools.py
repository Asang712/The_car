"""Shared parsing, process, and localization helpers."""

import json
import os

import yaml


def _parse_action_object(candidate: str):
    """Parse a candidate JSON object with the expected Dify output fields."""
    try:
        result = json.loads(candidate)
    except (json.JSONDecodeError, TypeError, ValueError):
        return None
    if isinstance(result, dict) and ("action" in result or "response" in result):
        return result
    return None


def extract_json_content(raw_content):
    """Extract a Dify action JSON object from plain or fenced text.

    The JSON decoder is used for every fallback so braces and escaped quotes
    inside JSON strings are handled correctly.
    """
    if not isinstance(raw_content, str) or not raw_content.strip():
        return None

    direct_result = _parse_action_object(raw_content)
    if direct_result is not None:
        return direct_result

    fenced_parts = raw_content.split("```")
    for part_index in range(1, len(fenced_parts), 2):
        candidate = fenced_parts[part_index].strip()
        if candidate.lower().startswith("json"):
            candidate = candidate[4:].lstrip()
        fenced_result = _parse_action_object(candidate)
        if fenced_result is not None:
            return fenced_result

    decoder = json.JSONDecoder()
    for start_index, character in enumerate(raw_content):
        if character != "{":
            continue
        try:
            result, _ = decoder.raw_decode(raw_content[start_index:])
        except json.JSONDecodeError:
            continue
        if isinstance(result, dict) and ("action" in result or "response" in result):
            return result

    return None


def kill_process_tree(pid):
    """Kill a process and all descendants, ignoring an already-dead process."""
    import psutil

    try:
        parent = psutil.Process(pid)
        for child in parent.children(recursive=True):
            child.kill()
        parent.kill()
    except psutil.NoSuchProcess:
        pass


class LogTranslator:
    """Load and format localized log messages."""

    def __init__(self, language="zh"):
        self.language = language
        self.translations = {}

    def load_translations_file(self, language_dir: str) -> bool:
        """Load the configured language YAML file."""
        lang_file = os.path.join(language_dir, f"{self.language}.yaml")
        try:
            with open(lang_file, "r", encoding="utf-8") as stream:
                self.translations = yaml.safe_load(stream) or {}
            return True
        except (OSError, yaml.YAMLError):
            self.translations = {}
            return False

    def get_text(self, key: str, **kwargs) -> str:
        """Return one formatted translation, falling back to the key."""
        text = str(self.translations.get(key, key))
        try:
            return text.format(**kwargs)
        except (KeyError, ValueError):
            return text
