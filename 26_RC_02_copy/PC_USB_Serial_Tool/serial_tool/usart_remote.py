from __future__ import annotations

import struct
from typing import Iterable, Optional

from .protocol import USART_FRAME_DATA_LEN, pack_usart_remote_frame


def build_remote_data(
    mode: Optional[int] = None,
    arm_enable: int = 0,
    source_usb: int = 0,
    climb_enable: int = 0,
    climb_step: int = 0,
    climb_auto: int = 0,
    chassis: Iterable[float] = (0.0, 0.0, 0.0),
) -> bytes:
    data = bytearray(USART_FRAME_DATA_LEN)

    if mode is not None:
        if not 0 <= mode <= 7:
            raise ValueError("mode must be 0..7")
        data[mode] = 1

    data[8] = 1 if arm_enable else 0
    data[9] = 1 if source_usb else 0
    data[13] = 1 if climb_enable else 0
    data[14] = 1 if climb_step else 0
    data[15] = 1 if climb_auto else 0

    chassis_vals = list(chassis)
    if len(chassis_vals) != 3:
        raise ValueError("chassis needs 3 floats")

    data[16:28] = struct.pack("<3f", *chassis_vals)
    return bytes(data)


def build_remote_frame(**kwargs) -> bytes:
    return pack_usart_remote_frame(build_remote_data(**kwargs))
