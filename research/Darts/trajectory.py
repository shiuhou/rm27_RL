"""Analytic trajectory of the dart's front reference point in field XYZ metres."""

from dataclasses import dataclass
import numpy as np


def vector(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError('Expected a finite 3-vector')
    return value


def basis(forward):
    """Right-handed frame with local +X forward; roll fixed relative to world up."""
    x = vector(forward)
    if np.linalg.norm(x) < 1e-12:
        raise ValueError('Undefined heading for zero velocity')
    x = x / np.linalg.norm(x)
    up = np.array([0., 0., 1.])
    if abs(x @ up) > .999:
        up = np.array([0., 1., 0.])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    return np.column_stack((x, y, np.cross(x, y)))


@dataclass
class BallisticPath:
    start: np.ndarray
    velocity: np.ndarray
    gravity: np.ndarray
    duration: float

    @classmethod
    def aimed(cls, start, target, duration, gravity=9.81):
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError('Flight time must be positive and finite')
        if not np.isfinite(gravity) or gravity < 0:
            raise ValueError('Gravity must be finite and nonnegative')
        start, target = vector(start), vector(target)
        if np.linalg.norm(target-start) < 1e-9:
            raise ValueError('Launch and target must differ')
        g = np.array([0., 0., -gravity])
        v = (target-start)/duration - .5*g*duration
        return cls(start, v, g, float(duration))

    def state(self, t):
        t = np.asarray(t, dtype=float)
        return (self.start + t[..., None]*self.velocity + .5*t[..., None]**2*self.gravity,
                self.velocity + t[..., None]*self.gravity)

    def samples(self, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Sample interval must be positive and finite')
        count = int(np.ceil(self.duration/dt))
        if count > 200000:
            raise ValueError('Too many trajectory samples')
        t = np.linspace(0, self.duration, count+1)
        p, v = self.state(t)
        return t, p, v

    def plane_hit(self, center, normal, width, height):
        """First inbound plane crossing inside the finite target rectangle.

        Returns None for a miss. The target's local +Z is the outward normal,
        local +X is horizontal, and local +Y points generally upward.
        """
        center, n = vector(center), vector(normal)
        if np.linalg.norm(n) < 1e-12 or width <= 0 or height <= 0:
            raise ValueError('Invalid target')
        n = n / np.linalg.norm(n)
        a, b, c = .5*(self.gravity@n), self.velocity@n, (self.start-center)@n
        roots = np.roots([a,b,c] if abs(a)>1e-12 else [b,c]) if abs(a)+abs(b)>1e-12 else []
        frame = target_frame(n)
        for root in sorted(float(x.real) for x in roots if abs(x.imag)<1e-9):
            if root < -1e-9 or root > self.duration+1e-9:
                continue
            p,v = self.state(max(0.,root))
            local = frame.T@(p-center)
            if v@n < 0 and abs(local[0]) <= width/2+1e-9 and abs(local[1]) <= height/2+1e-9:
                return dict(time_s=max(0.,root), position_m=p.tolist(),
                            normal_speed_m_s=float(-v@n), local_error_m=local.tolist())
        return None


def target_frame(normal):
    n = vector(normal)
    n = n / np.linalg.norm(n)
    x = np.cross([0.,0.,1.],n)
    if np.linalg.norm(x)<1e-9:
        x = np.array([1.,0.,0.])
    x /= np.linalg.norm(x)
    return np.column_stack((x,np.cross(n,x),n))
