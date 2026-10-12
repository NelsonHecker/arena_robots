from __future__ import annotations

import math

import numpy as np
import pytest
from arena_robots.audio import dbfs_from_rms, gcc_phat, geometric_delays_s, rectangular


def _four_mic_delays(source: tuple[float, float, float]) -> np.ndarray:
    mics = rectangular(width_m=0.31, length_m=0.42, height_m=0.22, corner_inset_m=0.02)
    return geometric_delays_s(source, tuple(mic.position_m for mic in mics))


def _tone() -> np.ndarray:
    return np.sin(np.linspace(0.0, 4.0 * math.pi, 1600, endpoint=False)).astype(np.float32)


def test_independent_channels_keep_the_tdoa() -> None:
    rate = 16000
    delays = _four_mic_delays((0.0, 5.0, 0.22))
    pulse = np.hanning(64).astype(np.float32)
    audio = np.zeros((4, 512), dtype=np.float32)
    base = 80
    for channel, delay in enumerate(delays - delays.min()):
        start = base + round(float(delay) * rate)
        audio[channel, start : start + len(pulse)] = pulse * (1.0 - 0.08 * channel)
    assert not np.array_equal(audio[0], audio[1])
    estimate, confidence = gcc_phat(audio[1], audio[0], sample_rate_hz=rate, max_tau_s=0.002)
    assert estimate > 0.0
    assert abs(estimate - (delays[1] - delays[0])) < 1.0 / rate
    assert confidence > 0.0


def test_full_scale_sine_reads_zero_dbfs() -> None:
    assert dbfs_from_rms(float(np.sqrt(np.mean(_tone().astype(np.float64) ** 2)))) == pytest.approx(0.0, abs=1e-6)
