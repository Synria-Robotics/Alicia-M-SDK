"""Firmware version parsing and comparison helpers."""

from __future__ import annotations

import re
from typing import Optional, Tuple, Union

VersionTuple = Tuple[int, int, int]

_VERSION_RE = re.compile(r"^v?(\d+)(?:[._](\d+)[._](\d+))?$", re.IGNORECASE)


def parse_firmware_version(version: object) -> Optional[VersionTuple]:
    """Parse firmware versions such as 106, "1.0.6", and "v1.0.6"."""
    if version is None:
        return None

    text = str(version).strip()
    if not text:
        return None

    match = _VERSION_RE.match(text)
    if not match:
        return None

    major = int(match.group(1))
    minor_text = match.group(2)
    patch_text = match.group(3)
    if minor_text is not None and patch_text is not None:
        return major, int(minor_text), int(patch_text)

    # Compact firmware notation, e.g. 106 -> 1.0.6, 110 -> 1.1.0.
    value = major
    return value // 100, (value // 10) % 10, value % 10


def supports_min_version(
    version: object,
    minimum: Union[str, VersionTuple],
) -> bool:
    """Return True when version is parseable and greater than or equal to minimum."""
    parsed = parse_firmware_version(version)
    min_version = parse_firmware_version(minimum) if isinstance(minimum, str) else minimum
    if parsed is None or min_version is None:
        return False
    return parsed >= min_version
