"""Shared helpers for example scripts."""


def add_port_argument(parser):
    """Add the common Alicia-M serial port option."""
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/tty.usbmodem5B8F0429481",
        help="Alicia-M serial port, for example COM37. Omit to auto-detect.",
    )
    return parser
