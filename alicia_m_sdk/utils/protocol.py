"""Protocol display helpers shared by SDK internals and examples."""

from __future__ import annotations


def format_bytes(data: bytes) -> str:
    """Format bytes as an uppercase hexadecimal string."""
    return " ".join(f"{byte:02X}" for byte in data)
