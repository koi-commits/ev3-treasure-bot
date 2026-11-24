"""Odometry-frame rectangular boundary estimates; no simulator dependencies."""
try:
    from umath import cos, sin, pi
except ImportError:
    from math import cos, sin, pi


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def orient_axis(h, face):
    """Pick the axis variant that runs along the wall the robot just faced."""
    h = wrap180(h)
    target = wrap180(face + 90.0)
    if abs(wrap180(h - target)) <= abs(wrap180(h + 90.0 - target)):
        return h
    return wrap180(h + 90.0)


def _axis_score(points, heading, tol):
    """How many point pairs share a wall at this heading. Returns (matches, residual).

    A point on a wall has a fixed coordinate along the wall normal, so two hits on
    the same wall agree in exactly one of (u, v). Wrong headings match only by
    accident; the true arena axis makes all same-wall pairs match.
    """
    a = -heading * pi / 180.0
    c, s = cos(a), sin(a)
    us = [-s*x + c*y for (x, y) in points]
    vs = [c*x + s*y for (x, y) in points]
    matches = 0
    residual = 0.0
    for i in range(len(points)):
        for j in range(i):
            du = us[i] - us[j]
            dv = vs[i] - vs[j]
            if -tol <= du <= tol:
                matches += 1
                residual += du*du
            if -tol <= dv <= tol:
                matches += 1
                residual += dv*dv
    return matches, residual


def fit_axis(points, face=None, tol=12.0):
    """Arena axis (DriveBase heading) from white hits, even across two walls.

    Near a corner the probe hits two perpendicular walls, so a straight line fit
    fails. Any rectangle axis is valid; when `face` (a heading roughly toward a
    wall) is given, pick the axis along that wall rather than its normal.
    """
    if len(points) < 3:
        return None
    # Repeated hits at the same spot match at every heading; drop duplicates.
    unique = []
    for p in points:
        if all((p[0]-q[0])**2 + (p[1]-q[1])**2 > 64.0 for q in unique):
            unique.append(p)
    points = unique
    if len(points) < 3:
        return None
    best = None
    k = 0
    while k < 180:
        matches, residual = _axis_score(points, float(k), tol)
        if matches and (best is None or matches > best[0]
                        or (matches == best[0] and residual < best[1])):
            best = (matches, residual, float(k))
        k += 1
    if best is None:
        return None
    h = best[2]
    return orient_axis(h, face) if face is not None else h


class Boundary:
    def __init__(self, axis, width, height):
        self.axis = axis
        a = -axis * pi / 180
        self.axes = ((cos(a), sin(a)), (-sin(a), cos(a)))
        self.bounds = [[None, None], [None, None]]
        # Unknown opposing wall: use the shorter prior, never the longer one.
        self.prior = min(width, height)

    def axis_index(self, heading):
        """Which axis family a travel heading is most aligned with."""
        a = -heading * pi / 180
        fx, fy = cos(a), sin(a)
        c0 = self.axes[0][0]*fx + self.axes[0][1]*fy
        c1 = self.axes[1][0]*fx + self.axes[1][1]*fy
        return 0 if abs(c0) >= abs(c1) else 1

    def observe(self, point, heading):
        # A white hit lies on the wall the robot ran into. The wall normal is the
        # travel direction, so the more aligned arena axis identifies the wall
        # family; the other coordinate is just where along that wall we hit.
        # Recording both coordinates as extents would invent walls at the ends
        # of every observed wall and stop the plow in open floor.
        k = self.axis_index(heading)
        u, v = self.axes[k]
        value = u*point[0]+v*point[1]
        lo, hi = self.bounds[k]
        if value < 0 and (lo is None or value < lo):
            self.bounds[k][0] = value
        if value > 0 and (hi is None or value > hi):
            self.bounds[k][1] = value

    def limits(self, k, known_only=False):
        lo,hi = self.bounds[k]
        if known_only:
            return lo,hi
        if lo is None and hi is None:
            return -self.prior/2, self.prior/2
        if lo is None:
            lo = hi-self.prior
        if hi is None:
            hi = lo+self.prior
        return lo,hi

    def clearance(self, x, y, heading, front, half_width=56, known_only=False):
        """Forward distance until either plow-front corner reaches a wall.

        known_only=True ignores the prior-filled opposite wall: an unobserved
        wall is discovered by the color sensor instead of guessed, otherwise a
        conservative prior would end every lane inside the arena.
        """
        a = -heading*pi/180
        fx,fy = cos(a),sin(a)
        remaining = 1e9
        for k,(u,v) in enumerate(self.axes):
            speed = u*fx+v*fy
            if abs(speed) < 1e-6:
                continue
            lo,hi = self.limits(k, known_only)
            wall = hi if speed > 0 else lo
            if wall is None:
                continue
            for lateral in (-half_width, half_width):
                p = u*(x+front*fx-lateral*fy)+v*(y+front*fy+lateral*fx)
                remaining = min(remaining, (wall-p)/speed)
        return remaining
