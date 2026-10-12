"""Shared pytest configuration for the localization tests.

Tests marked `slow` are the end-to-end scenario runs (a toy map, a full trajectory, a filter
in the loop). `python3 -m pytest test -m "not slow"` runs only the fast step tests; CI runs
everything.
"""


def pytest_configure(config):
    config.addinivalue_line(
        'markers', 'slow: end-to-end scenario test (seconds each); deselect with -m "not slow"'
    )
