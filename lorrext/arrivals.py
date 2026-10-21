"""NumPy re-implementation of the arrival processes in ext/lorr_ext.h, used for
previews in the UI. Same model and parameters; a different random stream, so
counts match the simulator in distribution, not tick by tick."""

from __future__ import annotations

import numpy as np

from .core import Setting


def sample(s: Setting, T: int | None = None, seed: int | None = None) -> np.ndarray:
    """Arrivals per tick, shape (T,)."""
    T = T or s.steps
    rng = np.random.default_rng(s.seed if seed is None else seed)
    if s.arrival == "poisson":
        return rng.poisson(s.rate, T)
    if s.arrival == "mmpp":
        f = min(max(s.burst_fraction, 1e-6), 1 - 1e-6)
        p_exit = 1 / max(1.0, s.burst_length)
        p_enter = p_exit * f / (1 - f)
        hi = s.rate * s.burst_factor
        lo = max(0.0, (s.rate - f * hi) / (1 - f))
        burst = rng.random() < f
        out = np.zeros(T, dtype=int)
        for t in range(T):
            if t and rng.random() < (p_exit if burst else p_enter):
                burst = not burst
            out[t] = rng.poisson(hi if burst else lo)
        return out
    if s.arrival == "hawkes":
        n = min(max(s.branching, 0), 0.99)
        mu, alpha, exc = s.rate * (1 - n), s.branching * s.decay, 0.0
        out = np.zeros(T, dtype=int)
        for t in range(T):
            out[t] = rng.poisson(max(mu + exc, 1e-12))
            exc = exc * np.exp(-s.decay) + alpha * out[t]
        return out
    raise ValueError(f"no preview for arrival model {s.arrival!r}")


def service_samples(stop: dict, n: int = 20000, seed: int = 0) -> np.ndarray:
    """Draw service times (ticks) from one parsed errand slot, mirroring
    ServiceDist::draw in ext/lorr_ext.h (rounded, clamped)."""
    rng = np.random.default_rng(seed)
    d, mean, cv = stop["dist"], stop.get("mean", 0.0), stop.get("cv", 0.0)
    if d == "uniform":
        return rng.integers(stop["lo"], stop["hi"] + 1, n)
    if d == "fixed":
        x = np.full(n, mean)
    elif d == "exponential":
        x = rng.exponential(mean, n)
    elif d == "lognormal":
        s2 = np.log(1 + cv * cv)
        x = rng.lognormal(np.log(mean) - s2 / 2, np.sqrt(s2), n)
    elif d == "gamma":
        k = 1 / max(cv * cv, 1e-9)
        x = rng.gamma(k, mean / k, n)
    elif d == "normal":
        x = rng.normal(mean, mean * cv, n)
    else:
        raise ValueError(d)
    return np.clip(np.round(x), stop.get("min", 0.0), stop.get("max", 1e9)).astype(int)


def burst_stats(x: np.ndarray, window: int = 50) -> dict:
    w = np.convolve(x, np.ones(window, dtype=int), mode="valid")
    return {"mean_rate": float(x.mean()), f"max_in_{window}": int(w.max()),
            f"p99_in_{window}": float(np.percentile(w, 99)), "index_of_dispersion": float(w.var() / max(w.mean(), 1e-9))}
