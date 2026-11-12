#!/usr/bin/env python3
"""Bring up the SocketCAN interface via scripts/setup_can0.sh, so steering_node
and ros_can_bridge don't require a manual `sudo ip link set can0 up ...`
before launch when auto_configure_can is enabled."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory


def configure_can_interface(logger, channel, bitrate):
    script = os.path.join(
        get_package_share_directory('golfcart_low_level'), 'scripts', 'setup_can0.sh'
    )
    logger.info(f'Configuring {channel} via {script} (bitrate={bitrate})')
    try:
        subprocess.run(
            ['bash', script, channel, str(bitrate)],
            check=True,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip() if exc.stderr else str(exc)
        logger.warn(f'CAN interface setup failed: {stderr}')
    except subprocess.TimeoutExpired:
        logger.warn(
            f'CAN interface setup timed out after 15s (likely a sudo password '
            f'prompt with no TTY available under this launch); continuing '
            f'without reconfiguring {channel}. Set up passwordless sudo for '
            f'the commands in {script}, or bring the interface up manually '
            f'and set auto_configure_can:=false.'
        )
