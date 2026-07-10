from unittest import TestCase

from krrood.ripple_down_rules.user_interface.ipython_custom_shell import IPythonShell


class ShellThatAlwaysFails:
    """
    Stands in for :class:`CustomInteractiveShell` and always raises on invocation, mimicking an
    embedded shell whose scripted/loaded expert answers are exhausted.
    """

    def __init__(self):
        self.call_count = 0

    def __call__(self):
        self.call_count += 1
        raise IndexError("scripted answers exhausted")


class TestIPythonShellRunFailsFastOnPersistentError(TestCase):
    """
    Regression test for :meth:`IPythonShell.run` retrying a failing embedded shell forever instead
    of surfacing the error.
    """

    def test_run_propagates_persistent_shell_error_without_retrying(self):
        shell = IPythonShell.__new__(IPythonShell)
        shell.shell = ShellThatAlwaysFails()

        with self.assertRaises(IndexError):
            shell.run()

        self.assertEqual(shell.shell.call_count, 1)
