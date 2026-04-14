import pytest
from mower.safety.lift_guard import LiftGuard


class TestLiftGuard:
    def _sensors(self, lift: bool) -> dict:
        return {'lift': lift, 'rain_adc': 0, 'encoder_ticks': 0}

    def test_no_lift_no_callback(self):
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(False))
        assert called == []

    def test_lift_fires_callback(self):
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(True))
        assert called == [1]

    def test_lift_only_fires_on_rising_edge(self):
        """Sustained lift (multiple True frames) fires on_lift exactly once."""
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(True))
        guard.check(self._sensors(True))
        guard.check(self._sensors(True))
        assert called == [1]

    def test_lift_fires_again_after_reset(self):
        """After deck lowered (False), a new lift fires the callback again."""
        guard = LiftGuard()
        called = []
        guard.on_lift = lambda: called.append(1)
        guard.check(self._sensors(True))    # rising edge → fires
        guard.check(self._sensors(False))   # deck lowered
        guard.check(self._sensors(True))    # rising edge again → fires
        assert called == [1, 1]

    def test_no_callback_if_not_assigned(self):
        guard = LiftGuard()
        guard.check(self._sensors(True))    # should not raise
