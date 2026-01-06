"""Plot a typical "soft limiter" nonlinearity used as an alternative to u^3.

We plot (for p=2):
    g(u) = u^3 / (1 + (|u|/u_s)^2)

This behaves like u^3 near u=0, but for |u| >> u_s it becomes ~ u_s^2 * u,
so it grows only linearly (softer than cubic).
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt


def soft_limiter(u: np.ndarray, u_s: float, p: float = 2.0) -> np.ndarray:
    u_abs = np.abs(u)
    return (u**3) / (1.0 + (u_abs / u_s) ** p)


def main() -> None:
    u = np.linspace(-4.0, 4.0, 2001)

    plt.figure(figsize=(8.5, 5.5))
    plt.plot(u, u**3, color="0.35", linestyle="--", linewidth=2, label=r"$u^3$ (referência)")

    for u_s in (0.5, 1.0, 2.0):
        g = soft_limiter(u, u_s=u_s, p=2.0)
        plt.plot(u, g, linewidth=2, label=rf"$g(u)=\dfrac{{u^3}}{{1+(|u|/{u_s})^2}}$  (u_s={u_s})")

    plt.axhline(0.0, color="0.85", linewidth=1)
    plt.axvline(0.0, color="0.85", linewidth=1)

    plt.title("Limiter suave (p=2) vs cúbico")
    plt.xlabel("u")
    plt.ylabel("g(u)")
    plt.grid(True, alpha=0.25)
    plt.legend(loc="best")
    plt.tight_layout()

    out = "limiter_soft_p2.png"
    plt.savefig(out, dpi=160)
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()
