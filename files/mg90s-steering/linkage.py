"""Steering linkage solver and optimiser for the MG90S front steering.

Plan view, vehicle coordinates: x to the right, y forward, servo shaft at the origin (z up). The servo turns a
crank with two tie-rod pins at (+-c, j); each tie rod runs to the end of its knuckle's steering arm (length a,
pointing rearward and inward by beta); the knuckles turn about kingpins at (+-K, yk). The rear axle sits L behind
the front axle, so true Ackermann wants cot(d_outer) - cot(d_inner) = 2K / L.

Run:  python linkage.py         (prints the chosen geometry and a table; --search runs the grid search)
"""
import math
import sys

K, YK = 40.0, 40.0      # kingpins (5 units out, 5 units ahead of the shaft)
L = 160.0               # wheelbase to the rear axle (20 units)
MIN_SERVO = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
PIN_CLEAR = 19.7 + 2.47 + 0.5   # crank pins must stay ahead of the servo holder front face (pin radius + margin)


def rot(p, ang):
    c, s = math.cos(ang), math.sin(ang)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def circle_meet(c0, r0, c1, r1, pick):
    """Intersections of two circles; pick(points) chooses one. None if they do not meet."""
    dx, dy = c1[0] - c0[0], c1[1] - c0[1]
    d = math.hypot(dx, dy)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    mx, my = c0[0] + a * dx / d, c0[1] + a * dy / d
    return pick([(mx + h * dy / d, my - h * dx / d), (mx - h * dy / d, my + h * dx / d)])


class Linkage:
    def __init__(self, a, beta_deg, c, j):
        self.a, self.beta, self.c, self.j = a, math.radians(beta_deg), c, j
        self.arm0 = {}
        self.rod = {}
        for s in (1, -1):
            kp = (s * K, YK)
            # arm end at neutral: rearward, turned inward by beta
            arm = (kp[0] - s * a * math.sin(self.beta), kp[1] - a * math.cos(self.beta))
            self.arm0[s] = arm
            jp = (s * c, j)
            self.rod[s] = math.hypot(arm[0] - jp[0], arm[1] - jp[1])

    def wheel_angle(self, s, theta):
        """Steer angle of knuckle s (+1 right, -1 left) for crank angle theta (rad, + = counter-clockwise).
        Positive = wheel turned to the left (counter-clockwise from above)."""
        jp = rot((s * self.c, self.j), theta)
        kp = (s * K, YK)
        arm0 = self.arm0[s]
        # arm end lies on the circle of radius a about the kingpin and rod length about the crank pin;
        # take the branch nearest the neutral arm end
        near = lambda pts: min(pts, key=lambda p: math.hypot(p[0] - arm0[0], p[1] - arm0[1]))  # noqa: E731
        arm = circle_meet(kp, self.a, jp, self.rod[s], near)
        if arm is None:
            return None, None
        ang0 = math.atan2(arm0[1] - kp[1], arm0[0] - kp[0])
        ang = math.atan2(arm[1] - kp[1], arm[0] - kp[0])
        d = (ang - ang0 + math.pi) % (2 * math.pi) - math.pi
        # transmission angle between tie rod and arm
        v1 = (arm[0] - kp[0], arm[1] - kp[1])
        v2 = (jp[0] - arm[0], jp[1] - arm[1])
        cosg = (v1[0] * v2[0] + v1[1] * v2[1]) / (math.hypot(*v1) * math.hypot(*v2))
        mu = math.degrees(math.acos(max(-1, min(1, cosg))))
        return math.degrees(d), min(mu, 180 - mu)

    def sweep(self, theta_max_deg, n=41):
        rows = []
        for i in range(n):
            th = math.radians(-theta_max_deg + 2 * theta_max_deg * i / (n - 1))
            dr, mr = self.wheel_angle(1, th)
            dl, ml = self.wheel_angle(-1, th)
            if dr is None or dl is None:
                return None
            jy = min(rot((s * self.c, self.j), th)[1] for s in (1, -1))
            rows.append((math.degrees(th), dl, dr, min(mr, ml), jy))
        return rows


def ackermann_ideal(d_inner):
    """Outer-wheel angle for true Ackermann given the inner one (deg, positive)."""
    if d_inner <= 0:
        return d_inner
    return math.degrees(math.atan(1 / (1 / math.tan(math.radians(d_inner)) + 2 * K / L)))


def score(link, target=32.0):
    """Find the crank angle giving `target` deg on the inner wheel; return (theta, worst Ackermann error over the
    range, min transmission angle, min crank-pin y), or None."""
    for th_max in [x * 0.5 for x in range(20, 141)]:
        rows = link.sweep(th_max, 61)
        if rows is None:
            return None
        inner = max(max(abs(r[1]), abs(r[2])) for r in rows)
        if inner >= target:
            err = 0.0
            for th, dl, dr, mu, jy in rows:
                if th == 0:
                    continue
                # the inner wheel is the one turned more; turning left (angles > 0) the left wheel is inner
                di, do = (dl, dr) if dl + dr > 0 else (-dr, -dl)
                if di <= 0:
                    return None
                err = max(err, abs(do - ackermann_ideal(di)))
            mu = min(r[3] for r in rows)
            jy = min(r[4] for r in rows)
            return th_max, err, mu, jy
    return None


def search():
    best = []
    for a in (14, 16, 18, 20, 22, 24, 26, 28, 30):
        for beta in [b * 1.0 for b in range(4, 31)]:
            for c in (4.0, 8.0, 12.0):
                for j in [x * 1.0 for x in range(22, 37)]:
                    link = Linkage(a, beta, c, j)
                    sc = score(link)
                    if sc is None:
                        continue
                    th, err, mu, jy = sc
                    if jy < PIN_CLEAR or mu < 35 or th > 45 or th < MIN_SERVO:
                        continue
                    best.append((err, a, beta, c, j, th, mu, jy))
    best.sort()
    for b in best[:15]:
        print("err %.2f  a %g beta %g c %g j %g  servo +-%.1f  mu %.0f  pin y %.1f" % b)


CHOSEN = dict(a=18, beta_deg=19, c=8.0, j=27.0)

if __name__ == "__main__":
    if "--search" in sys.argv:
        search()
        sys.exit()
    link = Linkage(**CHOSEN)
    print("geometry", CHOSEN, "rod lengths %.2f / %.2f" % (link.rod[1], link.rod[-1]))
    print("arm ends", {s: tuple(round(v, 2) for v in p) for s, p in link.arm0.items()})
    sc = score(link)
    print("servo +-%.1f deg for 32 deg inner, worst Ackermann error %.2f deg, min transmission %.0f, min pin y %.1f" % sc)
    for th, dl, dr, mu, jy in link.sweep(sc[0], 9):
        di, do = (dl, dr) if dl + dr > 0 else (-dr, -dl)
        print("servo %6.1f  left %6.2f  right %6.2f  ideal outer %6.2f" % (th, dl, dr, ackermann_ideal(di) if th else 0))
