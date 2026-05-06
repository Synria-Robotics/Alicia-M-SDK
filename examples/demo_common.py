"""Shared helpers for example scripts."""


def add_port_argument(parser):
    """Add the common Alicia-M serial port option."""
    parser.add_argument(
        "--port",
        type=str,
        default="",  # Set to your own port, default is auto-detect.
        help="Alicia-M serial port, for example COM37. Omit to auto-detect.",
    )
    return parser
