"""Ctrl-C handling for nodes that should stop between callbacks rather than inside one.

With rclpy's default handlers, SIGINT shuts the context down and raises KeyboardInterrupt wherever the
node happens to be: inside a Matplotlib draw or while the executor is taking a message, which prints
tracebacks after an otherwise clean run. Here SIGINT and SIGTERM only set a flag that the node's spin
loop checks, so it finishes the current callback, leaves the loop and shuts down normally.
"""

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class StopFlag:
    def __init__(self) -> None:
        self.requested = False

    def __bool__(self) -> bool:
        return self.requested


def init_with_stop_flag(args=None) -> StopFlag:
    """rclpy.init without rclpy's signal handlers; returns a flag set on SIGINT or SIGTERM."""
    stop = StopFlag()

    def request_stop(*_):
        stop.requested = True

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    return stop
