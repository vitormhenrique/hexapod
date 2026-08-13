"""Unit tests for fixture-based servo sign/trim fitting."""

from __future__ import annotations

from hexapod_protocol import config as cfg


def _ticks(sign: int, trim: int, servo_deg: float) -> int:
    offset = round((servo_deg - 180.0) * cfg.TICKS_PER_DEG)
    return cfg.SERVO_CENTER_TICK + trim + sign * offset


def test_identity_guesses_match_fixture_angles() -> None:
    assert cfg.identity_tick_for_servo_deg(180) == 2048
    assert cfg.identity_tick_for_servo_deg(135) == 2048 + round(-45 * cfg.TICKS_PER_DEG)
    assert cfg.identity_tick_for_servo_deg(225) == 2048 + round(45 * cfg.TICKS_PER_DEG)
    assert cfg.identity_angle_cdeg_for_tick(2048) == 0
    assert cfg.identity_angle_cdeg_for_tick(2048 + round(45 * cfg.TICKS_PER_DEG)) == 4500


def test_fit_positive_sign_and_trim() -> None:
    samples = [(d, _ticks(+1, -33, d)) for d in cfg.FIXTURE_SERVO_DEGREES]
    fit = cfg.fit_servo_sign_trim(samples)
    assert fit.ok
    assert fit.sign == 1
    assert fit.trim_ticks == -33
    assert fit.residual_ticks_rms < 0.5


def test_fit_negative_sign_and_trim() -> None:
    samples = [(d, _ticks(-1, 12, d)) for d in cfg.FIXTURE_SERVO_DEGREES]
    fit = cfg.fit_servo_sign_trim(samples)
    assert fit.ok
    assert fit.sign == -1
    assert fit.trim_ticks == 12
    assert fit.residual_ticks_rms < 0.5


def test_fit_requires_non_center_sample() -> None:
    fit = cfg.fit_servo_sign_trim([(180, 2048), (180, 2049)])
    assert not fit.ok
    assert "non-center" in fit.detail


def test_round_trip_with_angle_to_tick() -> None:
    """Fitted sign/trim must reproduce the captured ticks through angle_to_tick."""
    sign, trim = -1, 7
    samples = [(d, _ticks(sign, trim, d)) for d in cfg.FIXTURE_SERVO_DEGREES]
    fit = cfg.fit_servo_sign_trim(samples)
    assert fit.ok
    servo = cfg.ServoConfig(
        id=1, leg=0, joint=0, sign=fit.sign, trim_ticks=fit.trim_ticks,
        min_tick=0, max_tick=4095,
    )
    for servo_deg, tick in samples:
        rel = cfg.relative_deg_from_servo_deg(servo_deg) * cfg.DEG_TO_RAD
        assert cfg.angle_to_tick(servo, rel).tick == tick
