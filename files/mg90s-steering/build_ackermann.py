"""Write ackermann_analysis.html: the Ackermann steering theory for the MG90S front steering, with figures.

Every number and curve comes from ../tools/linkage.py (the same solver the CAD, the checks and the animation use),
so the page follows the design if the linkage changes. Figures are inline SVG; no external libraries.
Run:  python build_ackermann.py
"""
import html
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.argv = sys.argv[:1]
sys.path.insert(0, str(HERE.parent / "tools"))
import linkage as lk  # noqa: E402

K, YK, L = lk.K, lk.YK, lk.L
W = 2 * K                       # kingpin spacing
WHEEL_X = 5.8 + 7.4 + 0.6 + 13  # wheel centre off the kingpin (steering_assembly.WHEEL_IN + half the tyre width)
REAR_Y = YK - L
LINK = lk.Linkage(**lk.CHOSEN)
TH_MAX = lk.score(LINK)[0]

C = {"bg": "#080e18", "card": "#131f2e", "deep": "#07111e", "fg": "#e2e8f0", "muted": "#94a3b8", "line": "#2d3f55",
     "blue": "#3b82f6", "blue2": "#93c5fd", "orange": "#fb923c", "green": "#4ade80", "grid": "#1c2a3c",
     "violet": "#a78bfa", "pink": "#f472b6", "yellow": "#facc15"}


def angles(servo_deg, link=LINK):
    """Turning left by servo_deg (>= 0): (inner = left, outer = right) wheel angles, transmission angles."""
    t = math.radians(-servo_deg)
    dl, ml = link.wheel_angle(-1, t)
    dr, mr = link.wheel_angle(1, t)
    return dl, dr, ml, mr


def ideal(di):
    return lk.ackermann_ideal(di)


# ---------- numbers ----------
DI_MAX, DO_MAX, MU_I, MU_O = angles(TH_MAX)
DO_IDEAL = ideal(DI_MAX)
X_IN = -K - L / math.tan(math.radians(DI_MAX))    # where the inner (left) wheel's axle line meets the rear axle line
X_OUT = K - L / math.tan(math.radians(DO_MAX))
R_MID = -(X_IN + X_OUT) / 2
ACK_PCT = (DI_MAX - DO_MAX) / (DI_MAX - DO_IDEAL) * 100
SAMPLES = [TH_MAX * i / 350 for i in range(1, 351)]
ERRS = [(angles(s)[1] - ideal(angles(s)[0]), angles(s)[0], s) for s in SAMPLES]
E_MAX, E_DI, E_SERVO = max(ERRS, key=lambda e: abs(e[0]))
MU_MIN = min(min(angles(s)[2], angles(s)[3]) for s in [0.0] + SAMPLES)
BETA_RULE = math.degrees(math.atan(K / L))
TRAP_Y = YK - K / math.tan(LINK.beta)              # where the two arm lines cross (chosen beta)
BETAS = [0.0, 8.0, BETA_RULE, 19.0, 24.0]
BETA_ROWS = []
for b in BETAS:
    lb = lk.Linkage(LINK.a, b, LINK.c, LINK.j)
    th, err, mu, jy = lk.score(lb)
    BETA_ROWS.append((b, lb, th, err, mu))


def f(x, n=1):
    return f"{x:.{n}f}"


def esc(s):
    return html.escape(s, quote=False)


# ---------- svg helpers ----------
def svg(w, h, body, label):
    return (f'<svg viewBox="0 0 {w} {h}" role="img" aria-label="{html.escape(label)}" '
            f'xmlns="http://www.w3.org/2000/svg">{body}</svg>')


def line(x1, y1, x2, y2, col, w=1.5, dash=None, extra=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return (f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{col}" stroke-width="{w}"'
            f'{d} stroke-linecap="round"{extra}/>')


def text(x, y, s, col=None, size=12, anchor="start", weight=400):
    return (f'<text x="{x:.2f}" y="{y:.2f}" fill="{col or C["fg"]}" font-size="{size}" text-anchor="{anchor}" '
            f'font-weight="{weight}">{esc(s)}</text>')


def circle(x, y, r, col, fill=None):
    return f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r}" stroke="{col}" stroke-width="1.5" fill="{fill or "none"}"/>'


def poly(pts, col, fill="none", w=1.5, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    p = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
    return f'<polygon points="{p}" stroke="{col}" stroke-width="{w}" fill="{fill}"{d}/>'


def arc(cx, cy, r, a0, a1, col, w=1.5):
    """Arc in screen coordinates, angles in degrees measured counter-clockwise from +x (math sense, y up)."""
    p0 = (cx + r * math.cos(math.radians(a0)), cy - r * math.sin(math.radians(a0)))
    p1 = (cx + r * math.cos(math.radians(a1)), cy - r * math.sin(math.radians(a1)))
    large = 1 if abs(a1 - a0) > 180 else 0
    sweep = 0 if a1 > a0 else 1
    return (f'<path d="M{p0[0]:.2f},{p0[1]:.2f} A{r},{r} 0 {large} {sweep} {p1[0]:.2f},{p1[1]:.2f}" '
            f'stroke="{col}" stroke-width="{w}" fill="none"/>')


class Plan:
    """Vehicle plan coordinates (mm, x right, y forward) -> screen."""

    def __init__(self, x0, x1, y0, y1, width, pad=24):
        self.x0, self.y1, self.pad = x0, y1, pad
        self.s = (width - 2 * pad) / (x1 - x0)
        self.w, self.h = width, (y1 - y0) * self.s + 2 * pad

    def __call__(self, x, y):
        return self.pad + (x - self.x0) * self.s, self.pad + (self.y1 - y) * self.s


def wheel_rect(P, cx, cy, delta_deg, length=56.0, width=26.0):
    """A front/rear wheel seen from above: length along its rolling direction (+y at delta 0)."""
    d = math.radians(delta_deg)
    ux, uy = -math.sin(d), math.cos(d)      # rolling direction
    vx, vy = math.cos(d), math.sin(d)       # axle direction
    pts = []
    for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        pts.append(P(cx + a * length / 2 * ux + b * width / 2 * vx, cy + a * length / 2 * uy + b * width / 2 * vy))
    return pts


# ---------- figure 1: why Ackermann (plan view at full left lock) ----------
def fig_geometry():
    P = Plan(-310, 95, REAR_Y - 40, YK + 45, 820)
    b = []
    # rear axle line and its extension to the turning centre
    b.append(line(*P(X_IN - 15, REAR_Y), *P(80, REAR_Y), C["muted"], 1.2, "6 5"))
    # chassis outline
    b.append(poly([P(-32, REAR_Y - 20), P(32, REAR_Y - 20), P(32, YK + 12), P(-32, YK + 12)], C["line"],
                  fill=C["card"]))
    # rear wheels (straight)
    for s in (1, -1):
        b.append(poly(wheel_rect(P, s * (K + WHEEL_X), REAR_Y, 0, 81.6, 15), C["muted"], fill="#0d1726"))
    # front axle (kingpin line)
    b.append(line(*P(-K, YK), *P(K, YK), C["line"], 2))
    for s, d, col in ((-1, DI_MAX, C["blue"]), (1, DO_MAX, C["orange"])):
        kx = s * K
        dr = math.radians(d)
        wc = (kx + s * WHEEL_X * math.cos(dr), YK + s * WHEEL_X * math.sin(dr))
        b.append(line(*P(kx, YK), *P(*wc), col, 2.5))
        b.append(poly(wheel_rect(P, wc[0], wc[1], d), col, fill=C["deep"], w=2))
        # axle line through the kingpin down to the rear axle line
        xr = kx - L / math.tan(dr)
        b.append(line(*P(*wc), *P(xr, REAR_Y), col, 1.2, "5 4"))
        b.append(circle(*P(kx, YK), 3.5, col, C["bg"]))
        # steering angle arc at the kingpin: from the vehicle's forward direction
        cx, cy = P(kx, YK)
        b.append(line(cx, cy, cx, cy - 46, C["muted"], 1, "3 3"))
        b.append(arc(cx, cy, 34, 90, 90 + d, col, 1.8))
    # labels for the angles
    lx, ly = P(-K, YK)
    b.append(text(lx - 32, ly - 40, f"δi = {f(DI_MAX)}°", C["blue2"], 13, "end", 600))
    rx, ry = P(K, YK)
    b.append(text(rx + 6, ry - 44, f"δo = {f(DO_MAX)}°", C["orange"], 13, "start", 600))
    # turning centre
    ox, oy = P(-R_MID, REAR_Y)
    b.append(circle(ox, oy, 6, C["green"], C["green"]))
    b.append(text(ox + 4, oy - 12, "轉彎中心 O", C["green"], 13, "start", 600))
    # dimensions: wheelbase L and kingpin spacing
    dx = P(62, 0)[0]
    b.append(line(dx, P(0, YK)[1], dx, P(0, REAR_Y)[1], C["muted"], 1))
    b.append(text(dx + 6, (P(0, YK)[1] + P(0, REAR_Y)[1]) / 2, f"軸距 L = {L:.0f}", C["muted"], 12))
    y = P(0, YK + 30)[1]
    b.append(line(P(-K, 0)[0], y, P(K, 0)[0], y, C["muted"], 1))
    b.append(text(P(0, 0)[0], y - 6, f"主銷距 w = {W:.0f}", C["muted"], 12, "middle"))
    # R
    my = P(0, REAR_Y)[1] + 34
    b.append(line(ox, my, P(0, REAR_Y)[0], my, C["green"], 1))
    b.append(text((ox + P(0, REAR_Y)[0]) / 2, my + 16, f"R ≈ {R_MID:.0f} mm（量到後軸中點）", C["green"], 12, "middle"))
    b.append(text(P(-K, YK)[0] - 8, P(-K, YK)[1] + 34, "內輪（左）", C["blue2"], 12, "end"))
    b.append(text(P(K, YK)[0] + 30, P(K, YK)[1] + 40, "外輪（右）", C["orange"], 12))
    return svg(P.w, P.h, "".join(b), "左打到底時的平面圖：兩前輪輪軸延長線與後軸線交於轉彎中心")


# ---------- figure 2: the classic trapezoid rule ----------
def fig_trapezoid():
    P = Plan(-70, 70, REAR_Y - 18, YK + 22, 420)
    b = []
    b.append(line(*P(-60, REAR_Y), *P(60, REAR_Y), C["muted"], 1.2, "6 5"))
    b.append(text(*P(58, REAR_Y + 4), "後軸", C["muted"], 11, "end"))
    b.append(line(*P(0, REAR_Y - 10), *P(0, YK + 15), C["line"], 1, "2 4"))
    b.append(line(*P(-K, YK), *P(K, YK), C["line"], 2))
    for beta, col, lab in ((BETA_RULE, C["green"], f"經驗法則 β = {BETA_RULE:.1f}°：交在後軸中點"),
                           (math.degrees(LINK.beta), C["orange"], f"本設計 β = {math.degrees(LINK.beta):.0f}°：交在 y = {TRAP_Y:.0f}")):
        yx = YK - K / math.tan(math.radians(beta))
        for s in (1, -1):
            b.append(line(*P(s * K, YK), *P(0, yx), col, 1.4, "5 4"))
            ax = s * K - s * LINK.a * math.sin(math.radians(beta))
            ay = YK - LINK.a * math.cos(math.radians(beta))
            b.append(line(*P(s * K, YK), *P(ax, ay), col, 4))
        b.append(circle(*P(0, yx), 4, col, col))
    for s in (1, -1):
        b.append(circle(*P(s * K, YK), 3.5, C["fg"], C["bg"]))
    b.append(text(*P(0, YK + 10), "主銷", C["muted"], 11, "middle"))
    yx_o = P(0, TRAP_Y)
    b.append(text(yx_o[0] + 8, yx_o[1] + 4, f"本設計 β = {math.degrees(LINK.beta):.0f}°", C["orange"], 12))
    yx_g = P(0, REAR_Y)
    b.append(text(yx_g[0] + 8, yx_g[1] - 8, f"β = {BETA_RULE:.1f}°（atan(K/L)）", C["green"], 12))
    return svg(P.w, P.h, "".join(b), "阿克曼梯形：轉向臂延長線的交點")


# ---------- figure 3: the actual linkage, neutral and full lock ----------
def fig_linkage():
    P = Plan(-58, 58, -9, 58, 620)
    b = []
    sat = [(-19.7, -9), (19.7, -9), (19.7, 19.7), (-19.7, 19.7)]  # servo holder, its front face at y 19.7
    b.append(poly([P(*p) for p in sat], C["line"], fill=C["card"]))
    b.append(text(*P(-18, -6), "伺服座（前緣 y = 19.7）", C["muted"], 11))
    for pose, alpha, dash in ((0.0, 1.0, None), (TH_MAX, 0.55, "5 4")):
        th = math.radians(-pose)
        crank = [lk.rot((s * LINK.c, LINK.j), th) for s in (1, -1)]
        op = f' opacity="{alpha}"'
        g = []
        g.append(line(*P(0, 0), *P(*crank[0]), C["yellow"], 4, dash))
        g.append(line(*P(0, 0), *P(*crank[1]), C["yellow"], 4, dash))
        g.append(line(*P(*crank[0]), *P(*crank[1]), C["yellow"], 4, dash))
        for s, col in ((1, C["orange"]), (-1, C["blue"])):
            d, _ = LINK.wheel_angle(s, th)
            kp = (s * K, YK)
            arm = lk.rot((LINK.arm0[s][0] - kp[0], LINK.arm0[s][1] - kp[1]), math.radians(d))
            arm = (arm[0] + kp[0], arm[1] + kp[1])
            jp = crank[0] if s == 1 else crank[1]
            g.append(line(*P(*jp), *P(*arm), C["pink"], 3, dash))
            g.append(line(*P(*kp), *P(*arm), col, 5, dash))
            g.append(circle(*P(*arm), 2.6, C["fg"], C["bg"]))
            g.append(circle(*P(*jp), 2.6, C["fg"], C["bg"]))
            g.append(circle(*P(*kp), 3.6, C["fg"], C["bg"]))
        b.append(f"<g{op}>{''.join(g)}</g>")
    b.append(circle(*P(0, 0), 4, C["violet"], C["violet"]))
    b.append(text(P(0, 0)[0] + 8, P(0, 0)[1] - 6, "伺服軸", C["violet"], 11))
    kpL, kpR = P(-K, YK), P(K, YK)
    b.append(text(kpL[0], kpL[1] - 10, "主銷 (−40, 40)", C["blue2"], 11, "middle"))
    b.append(text(kpR[0], kpR[1] - 10, "主銷 (40, 40)", C["orange"], 11, "middle"))
    jx, jy = P(LINK.c, LINK.j)
    b.append(text(jx + 6, jy - 8, f"曲柄銷 (±{LINK.c:.0f}, {LINK.j:.0f})", C["yellow"], 11))
    ax, ay = P(*LINK.arm0[1])
    b.append(text(ax - 2, ay + 18, f"轉向臂 a = {LINK.a:.0f}", C["orange"], 11, "middle"))
    rx, ry = P((LINK.arm0[1][0] + LINK.c) / 2, (LINK.arm0[1][1] + LINK.j) / 2)
    b.append(text(rx, ry + 18, f"拉桿 {LINK.rod[1]:.2f}", C["pink"], 11, "middle"))
    b.append(text(P(-56, 0)[0], P(0, 52)[1], "實線：直行　虛線：左打到底", C["muted"], 11))
    return svg(P.w, P.h, "".join(b), "伺服曲柄與兩支拉桿的連桿機構平面圖")


# ---------- charts ----------
class Chart:
    def __init__(self, xr, yr, w=620, h=380, xl="", yl="", ml=56, mr=18, mt=18, mb=48):
        self.xr, self.yr, self.w, self.h = xr, yr, w, h
        self.ml, self.mr, self.mt, self.mb = ml, mr, mt, mb
        self.xl, self.yl = xl, yl
        self.body = []

    def X(self, x):
        return self.ml + (x - self.xr[0]) / (self.xr[1] - self.xr[0]) * (self.w - self.ml - self.mr)

    def Y(self, y):
        return self.h - self.mb - (y - self.yr[0]) / (self.yr[1] - self.yr[0]) * (self.h - self.mt - self.mb)

    def axes(self, xt, yt, yfmt="{:g}"):
        b = []
        for x in xt:
            b.append(line(self.X(x), self.Y(self.yr[0]), self.X(x), self.Y(self.yr[1]), C["grid"], 1))
            b.append(text(self.X(x), self.Y(self.yr[0]) + 18, f"{x:g}", C["muted"], 11, "middle"))
        for y in yt:
            b.append(line(self.X(self.xr[0]), self.Y(y), self.X(self.xr[1]), self.Y(y), C["grid"], 1))
            b.append(text(self.X(self.xr[0]) - 8, self.Y(y) + 4, yfmt.format(y), C["muted"], 11, "end"))
        b.append(line(self.X(self.xr[0]), self.Y(self.yr[0]), self.X(self.xr[1]), self.Y(self.yr[0]), C["line"], 1.2))
        b.append(line(self.X(self.xr[0]), self.Y(self.yr[0]), self.X(self.xr[0]), self.Y(self.yr[1]), C["line"], 1.2))
        b.append(text((self.X(self.xr[0]) + self.X(self.xr[1])) / 2, self.h - 8, self.xl, C["muted"], 12, "middle"))
        b.append(f'<text transform="translate(14 {(self.Y(self.yr[0]) + self.Y(self.yr[1])) / 2:.1f}) rotate(-90)" '
                 f'fill="{C["muted"]}" font-size="12" text-anchor="middle">{esc(self.yl)}</text>')
        self.body = b + self.body

    def path(self, pts, col, w=2.2, dash=None):
        d = "M" + " L".join(f"{self.X(x):.2f},{self.Y(y):.2f}" for x, y in pts)
        da = f' stroke-dasharray="{dash}"' if dash else ""
        self.body.append(f'<path d="{d}" stroke="{col}" stroke-width="{w}" fill="none"{da} '
                         f'stroke-linejoin="round" stroke-linecap="round"/>')

    def dot(self, x, y, col, r=4):
        self.body.append(circle(self.X(x), self.Y(y), r, col, col))

    def label(self, x, y, s, col, anchor="start", dx=6, dy=-6, size=12):
        self.body.append(text(self.X(x) + dx, self.Y(y) + dy, s, col, size, anchor, 600))

    def legend(self, items, x, y):
        for i, (s, col, dash) in enumerate(items):
            yy = y + i * 20
            da = f' stroke-dasharray="{dash}"' if dash else ""
            self.body.append(f'<line x1="{x}" y1="{yy}" x2="{x + 26}" y2="{yy}" stroke="{col}" stroke-width="2.5"{da}/>')
            self.body.append(text(x + 32, yy + 4, s, C["fg"], 12))

    def svg(self, label):
        return svg(self.w, self.h, "".join(self.body), label)


def fig_outer_vs_inner():
    ch = Chart((0, 35), (0, 35), xl="內輪轉角 δi（°）", yl="外輪轉角 δo（°）")
    ch.axes(range(0, 36, 5), range(0, 36, 5))
    ch.path([(0, 0), (35, 35)], C["muted"], 1.5, "4 4")
    ch.path([(0, 0)] + [(d, ideal(d)) for d in [x * 0.25 for x in range(1, 141)]], C["green"], 2.2)
    pts = [(0.0, 0.0)] + [(angles(s)[0], angles(s)[1]) for s in SAMPLES]
    ch.path(pts, C["orange"], 2.6)
    ch.dot(DI_MAX, DO_MAX, C["orange"])
    ch.label(DI_MAX, DO_MAX, f"打到底 {f(DI_MAX)}° / {f(DO_MAX)}°", C["orange"], "end", -8, 22)
    ch.legend([("平行轉向 δo = δi（兩輪同角）", C["muted"], "4 4"), ("理想阿克曼 cot δo − cot δi = w / L", C["green"], None),
               ("本機構（連桿實際解）", C["orange"], None)], 72, 34)
    return ch.svg("外輪轉角對內輪轉角：本機構與理想阿克曼曲線")


def fig_error():
    ch = Chart((0, 33), (-1, 6), xl="內輪轉角 δi（°）", yl="外輪轉角誤差 δo − δo,理想（°）")
    ch.axes(range(0, 34, 5), range(-1, 7, 1))
    ch.path([(0, 0), (33, 0)], C["muted"], 1.2)
    cols = [C["muted"], C["violet"], C["green"], C["orange"], C["pink"]]
    items = []
    for (beta, lb, th, err, mu), col in zip(BETA_ROWS, cols):
        pts = [(0.0, 0.0)]
        for i in range(1, 121):
            s = th * i / 120
            a, o, _, _ = angles(s, lb)
            if a > 33:
                break
            pts.append((a, o - ideal(a)))
        chosen = abs(beta - math.degrees(LINK.beta)) < 1e-6
        ch.path(pts, col, 3.2 if chosen else 1.8)
        name = f"β = {beta:.1f}°（經驗法則）" if abs(beta - BETA_RULE) < 1e-6 else f"β = {beta:.0f}°" + ("（本設計）" if chosen else "")
        items.append((f"{name}　最大 {err:.2f}°", col, None))
    ch.legend(items, 72, 30)
    return ch.svg("不同轉向臂內傾角 β 的阿克曼誤差曲線")


def fig_servo():
    ch = Chart((0, 18), (0, 35), xl="伺服機轉角（°）", yl="輪子轉角（°）")
    ch.axes(range(0, 19, 2), range(0, 36, 5))
    ch.path([(0, 0)] + [(s, angles(s)[0]) for s in SAMPLES], C["blue"], 2.6)
    ch.path([(0, 0)] + [(s, angles(s)[1]) for s in SAMPLES], C["orange"], 2.6)
    ch.label(TH_MAX, DI_MAX, f"內輪 {f(DI_MAX)}°", C["blue2"], "end", -6, -8)
    ch.label(TH_MAX, DO_MAX, f"外輪 {f(DO_MAX)}°", C["orange"], "end", -6, 18)
    ch.legend([("內輪（左打時是左輪）", C["blue"], None), ("外輪", C["orange"], None)], 72, 34)
    return ch.svg("伺服機轉角與兩前輪轉角")


# ---------- table ----------
def table_rows():
    rows = []
    for s in [0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, TH_MAX]:
        di, do, mi, mo = angles(s)
        if s == 0:
            rows.append(f"<tr><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>{f(mi, 0)}°</td></tr>")
            continue
        oi = ideal(di)
        rows.append(f"<tr><td>{f(s)}</td><td>{f(di, 2)}</td><td>{f(do, 2)}</td><td>{f(oi, 2)}</td>"
                    f"<td>{do - oi:+.2f}</td><td>{f(min(mi, mo), 0)}°</td></tr>")
    return "".join(rows)


def beta_rows():
    out = []
    for beta, lb, th, err, mu in BETA_ROWS:
        chosen = abs(beta - math.degrees(LINK.beta)) < 1e-6
        name = f"{beta:.1f}°（經驗法則）" if abs(beta - BETA_RULE) < 1e-6 else f"{beta:.0f}°" + ("（本設計）" if chosen else "")
        cls = ' class="hl"' if chosen else ""
        out.append(f"<tr{cls}><td>{name}</td><td>±{th:.1f}°</td><td>{err:.2f}°</td><td>{mu:.0f}°</td></tr>")
    return "".join(out)


CSS = """
:root { --bg:#080e18; --card:#131f2e; --deep:#07111e; --fg:#e2e8f0; --muted:#94a3b8; --line:#2d3f55;
  --blue:#3b82f6; --blue-soft:#93c5fd; --orange:#fb923c; --green:#4ade80; color-scheme: dark; }
* { box-sizing: border-box; }
html { overflow-x: hidden; }
body { margin:0; background:var(--bg); color:var(--fg); line-height:1.7; font-size:16px;
  font-family: system-ui, -apple-system, "Segoe UI", "Microsoft JhengHei", "PingFang TC", "Noto Sans TC", sans-serif; }
a { color:var(--blue-soft); text-decoration:none; } a:hover { text-decoration:underline; }
.wrap { max-width: 920px; margin: 0 auto; padding: 0 16px; }
header { border-bottom:1px solid var(--line); background:linear-gradient(180deg,#0d1828,var(--bg)); }
header .wrap { padding-top: 28px; padding-bottom: 26px; }
.nav { display:flex; flex-wrap:wrap; gap:8px 16px; font-size:.9rem; margin-bottom: 14px; }
h1 { font-size: clamp(1.5rem, 4vw, 2.2rem); margin: 0; letter-spacing: .02em; }
.en { color: var(--muted); font-size: .95rem; }
.lead { color: var(--muted); margin: 12px 0 0; }
.kpis { display:grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 10px; margin-top: 20px; }
.kpi { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:10px 14px; }
.kpi b { display:block; font-size:1.35rem; font-variant-numeric: tabular-nums; }
.kpi span { color:var(--muted); font-size:.85rem; }
section { margin: 34px 0; }
h2 { font-size: 1.3rem; margin: 0 0 12px; padding-bottom: 6px; border-bottom: 1px solid var(--line); }
h2 small { color: var(--muted); font-weight: 400; font-size: .85rem; margin-left: 8px; }
ul, ol { padding-left: 1.3em; } li { margin: 6px 0; } li::marker { color: var(--blue); }
figure { margin: 18px 0; background: var(--card); border: 1px solid var(--line); border-radius: 12px; padding: 12px; }
figure svg { display:block; width:100%; height:auto; font-family: inherit; }
figcaption { color: var(--muted); font-size: .88rem; margin-top: 8px; }
.formula { background: var(--deep); border: 1px solid rgba(59,130,246,.35); color: #7dd3fc; border-radius: 8px;
  padding: 10px 14px; font-family: ui-monospace, "Cascadia Mono", Consolas, monospace; font-size: .95rem;
  overflow-x: auto; white-space: pre; margin: 10px 0; }
.callout { background:#0f2040; border:1px solid rgba(59,130,246,.3); color:#93c5fd; border-radius:8px; padding:10px 14px; }
.tablewrap { overflow-x:auto; border:1px solid var(--line); border-radius:10px; }
table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; font-size: .92rem; }
th, td { padding: 7px 12px; text-align: right; border-bottom: 1px solid var(--line); white-space: nowrap; }
th { background: var(--deep); color: var(--muted); font-weight: 600; }
th:first-child, td:first-child { text-align: left; }
tr:last-child td { border-bottom: 0; }
tr.hl td { background: rgba(251,146,60,.12); }
.two { display:grid; grid-template-columns: 1fr 1fr; gap: 16px; align-items: start; }
@media (max-width: 760px) { .two { grid-template-columns: 1fr; } }
footer { border-top:1px solid var(--line); color:var(--muted); font-size:.85rem; padding: 18px 0 40px; margin-top: 40px; }
"""


def build():
    deg = math.degrees(LINK.beta)
    body = f"""
<header><div class="wrap">
<div class="nav"><a href="../works/mg90s-steering.html">← MG90S 前輪轉向作品頁</a><a href="steering_animation.html">3D 動畫：組裝與轉向 ↗</a></div>
<h1>阿克曼轉向機構理論分析</h1>
<div class="en">Ackermann Steering Geometry · MG90S Servo Front Steering</div>
<p class="lead">轉彎時內外前輪該轉多少度、這組伺服曲柄＋拉桿機構實際轉多少度，兩者差多少、為什麼這樣選尺寸。所有數字與圖都由轉向機構的連桿程式 linkage.py 直接算出，和 CAD 模型、干涉檢查、3D 動畫用的是同一套幾何。</p>
<div class="kpis">
<div class="kpi"><b>{f(DI_MAX)}° / {f(DO_MAX)}°</b><span>打到底：內輪／外輪（伺服 ±{f(TH_MAX)}°）</span></div>
<div class="kpi"><b>{abs(E_MAX):.2f}°</b><span>全行程最大阿克曼誤差</span></div>
<div class="kpi"><b>{ACK_PCT:.0f} %</b><span>打到底時的阿克曼百分比</span></div>
<div class="kpi"><b>≈ {R_MID:.0f} mm</b><span>最小轉彎半徑（後軸中點）</span></div>
</div>
</div></header>
<main class="wrap">

<section>
<h2>1. 為什麼內外輪要轉不同角度<small>Ackermann Condition</small></h2>
<ul>
<li>車子低速轉彎、輪胎不打滑時，每個輪子都只能沿自己的滾動方向前進，所以<strong>四個輪子的輪軸延長線必須交在同一點</strong>：轉彎中心 O</li>
<li>後輪不轉向，輪軸固定在後軸線上，所以 O 一定落在<strong>後軸線的延長線</strong>上</li>
<li>內側前輪離 O 比較近、走比較小的圓，所以要轉得比較多：<strong>δi &gt; δo</strong></li>
<li>如果兩輪轉一樣的角度（平行轉向），兩條輪軸線交不到同一點，至少一個輪子會被拖著側滑，輪胎磨損、轉向變重，在玩具車上則是轉不順、抖動</li>
</ul>
<figure>{fig_geometry()}
<figcaption>圖 1　左打到底（伺服 −{f(TH_MAX)}°）的實際姿勢，比例照模型：主銷距 w = {W:.0f} mm、軸距 L = {L:.0f} mm。藍色內輪、橘色外輪的輪軸延長線（虛線）在後軸線上交會；兩條線的交點只差 {abs(X_IN - X_OUT):.1f} mm，就是第 4 節的阿克曼誤差。</figcaption></figure>
<p>由圖 1 的兩個直角三角形（R 量到後軸中點，K = w/2 是主銷到中線的距離）：</p>
<div class="formula">tan δi = L / (R − K)        tan δo = L / (R + K)

cot δo − cot δi = (R + K)/L − (R − K)/L = w / L = {W:.0f} / {L:.0f} = {W / L:.2f}</div>
<ul>
<li>這就是<strong>阿克曼條件</strong>：只要知道內輪角，外輪的理想角就定了，和轉彎半徑無關</li>
<li>例：內輪 {f(DI_MAX)}° 時，理想外輪 = atan(1 / (cot {f(DI_MAX)}° + {W / L:.2f})) = <strong>{f(DO_IDEAL, 2)}°</strong></li>
<li>轉彎半徑：R = L / tan δi + K；內輪 {f(DI_MAX)}° → R ≈ {L / math.tan(math.radians(DI_MAX)) + K:.0f} mm</li>
</ul>
</section>

<section>
<h2>2. 阿克曼梯形與經驗法則<small>Ackermann Trapezoid</small></h2>
<div class="two">
<div>
<ul>
<li>傳統做法：兩個轉向節各伸出一支轉向臂，用一根橫拉桿相連，四根桿件在平面上形成梯形</li>
<li>轉向臂往內斜 β，一側轉向時，內側轉向臂「轉得比較快」，內輪自然轉得比外輪多</li>
<li>經驗法則：兩支轉向臂延長線交在<strong>後軸中點</strong>，即 tan β = K / L，本車 β = <strong>{BETA_RULE:.1f}°</strong></li>
<li>這條法則只在小角度時近似成立，而且是針對「一根橫拉桿」的梯形；本機構是伺服曲柄＋兩支拉桿（分段式），最佳 β 不同，要用解析／數值方法找</li>
<li>本設計的 β = {deg:.0f}°，延長線交在 y = {TRAP_Y:.0f} mm，在後軸（y = {REAR_Y:.0f}）前方</li>
</ul>
</div>
<figure>{fig_trapezoid()}
<figcaption>圖 2　轉向臂延長線的交點：綠色是經驗法則（{BETA_RULE:.1f}°），橘色是本設計（{deg:.0f}°）。</figcaption></figure>
</div>
</section>

<section>
<h2>3. 本機構：伺服曲柄＋兩支拉桿<small>Split Tie-Rod Linkage</small></h2>
<ul>
<li>MG90S 輸出軸在車子中線上，曲柄有兩個銷，在軸前方 {LINK.j:.0f} mm、左右各 {LINK.c:.0f} mm</li>
<li>每一側是一組<strong>四連桿（Four-Bar Linkage）</strong>：伺服軸→曲柄銷（曲柄）、曲柄銷→轉向臂端（拉桿 {LINK.rod[1]:.2f} mm）、轉向臂端→主銷（轉向臂 a = {LINK.a:.0f} mm，內傾 β = {deg:.0f}°）、主銷→伺服軸（車架，固定）</li>
<li>左右兩組共用同一支曲柄，但幾何互為鏡像；曲柄往左轉時，左側四連桿和右側四連桿走在不同區段，這就是內外輪轉角不同的來源</li>
</ul>
<figure>{fig_linkage()}
<figcaption>圖 3　連桿平面圖（俯視，y 朝車頭）。黃色曲柄、粉色拉桿、藍／橘轉向臂；實線直行，虛線是伺服轉 {f(TH_MAX)}° 左打到底。</figcaption></figure>
<p>給定伺服角 θ，求輪子轉角 δ 的方法（程式 Linkage.wheel_angle）：</p>
<ol>
<li>曲柄銷位置 J(θ) = 旋轉 θ 後的 (±c, j)</li>
<li>轉向臂端 A 必須同時在「以主銷為圓心、半徑 a」與「以 J 為圓心、半徑 = 拉桿長」的兩個圓上 → <strong>兩圓交點</strong>，取靠近直行位置的那一個解</li>
<li>δ = 轉向臂從直行位置轉過的角度；同時算出拉桿與轉向臂的夾角，也就是<strong>傳力角（Transmission Angle）μ</strong></li>
</ol>
<div class="formula">|A − Kp| = a                 （A 在轉向臂圓上）
|A − J(θ)| = 拉桿長           （A 在拉桿圓上）
δ = ∠(A − Kp) − ∠(A0 − Kp)    （A0：直行時的轉向臂端）</div>
</section>

<section>
<h2>4. 結果：和理想阿克曼比較<small>Results</small></h2>
<figure>{fig_outer_vs_inner()}
<figcaption>圖 4　外輪對內輪轉角。橘色（本機構）幾乎貼著綠色（理想）；灰色虛線是兩輪同角的平行轉向，差距越轉越大。</figcaption></figure>
<div class="two">
<figure>{fig_servo()}
<figcaption>圖 5　伺服角與輪角。內輪曲線越轉越陡：伺服角相同時，內輪轉得比外輪多。</figcaption></figure>
<div>
<ul>
<li>伺服 ±{f(TH_MAX)}° 就從直行轉到打到底：內輪 <strong>{f(DI_MAX)}°</strong>、外輪 <strong>{f(DO_MAX)}°</strong>（理想 {f(DO_IDEAL, 2)}°）</li>
<li>全行程最大誤差 <strong>{abs(E_MAX):.2f}°</strong>，出現在內輪約 {f(E_DI)}°（伺服 {f(E_SERVO)}°）；外輪略多轉，偏向平行轉向一側</li>
<li>阿克曼百分比（實際內外角差 ÷ 理想內外角差）在打到底時 {ACK_PCT:.0f} %</li>
<li>打到底時兩條輪軸線在後軸線上相差 {abs(X_IN - X_OUT):.1f} mm，約是轉彎半徑的 {abs(X_IN - X_OUT) / R_MID * 100:.1f} %</li>
<li>傳力角最小 {MU_MIN:.0f}°（打到底的內側）；一般建議 ≥ 30～40°，越小越容易卡、伺服負載越大</li>
</ul>
</div>
</div>
<div class="tablewrap"><table>
<thead><tr><th>伺服角（°）</th><th>內輪 δi（°）</th><th>外輪 δo（°）</th><th>理想 δo（°）</th><th>誤差（°）</th><th>最小傳力角</th></tr></thead>
<tbody>{table_rows()}</tbody></table></div>
</section>

<section>
<h2>5. 尺寸怎麼選出來的<small>Design Search</small></h2>
<ul>
<li>固定條件：主銷在 (±{K:.0f}, {YK:.0f})、軸距 {L:.0f} mm（Technic 8 mm 格的整數倍）、內輪打到底 32°</li>
<li>掃描變數：轉向臂長 a（14～30 mm）、內傾角 β（4～30°）、曲柄銷橫距 c（4、8、12 mm）、曲柄銷前距 j（22～36 mm）</li>
<li>限制條件：曲柄銷整個行程都在伺服座前緣之前（不撞伺服座）、傳力角 ≥ 35°、伺服角 ≤ 45°</li>
<li>目標：全行程最大阿克曼誤差最小 → 選出 <strong>a = {LINK.a:.0f}、β = {deg:.0f}°、c = {LINK.c:.0f}、j = {LINK.j:.0f}</strong></li>
</ul>
<figure>{fig_error()}
<figcaption>圖 6　只改 β（其他尺寸不變、各自調整伺服行程讓內輪到 32°）的誤差曲線。β 太小接近平行轉向，外輪轉太多；β 太大則過度阿克曼。</figcaption></figure>
<div class="tablewrap"><table>
<thead><tr><th>轉向臂內傾 β</th><th>伺服行程</th><th>最大阿克曼誤差</th><th>最小傳力角</th></tr></thead>
<tbody>{beta_rows()}</tbody></table></div>
<ul>
<li>經驗法則 β = {BETA_RULE:.1f}° 的誤差是 {BETA_ROWS[2][3]:.2f}°，本設計 {deg:.0f}° 降到 {BETA_ROWS[3][3]:.2f}°</li>
<li>代價是傳力角變小（{BETA_ROWS[2][4]:.0f}° → {BETA_ROWS[3][4]:.0f}°）；β = 24° 誤差又變大、傳力角掉到 {BETA_ROWS[4][4]:.0f}°，所以 {deg:.0f}° 是兩者的平衡點</li>
</ul>
</section>

<section>
<h2>6. 實務上要注意的地方<small>Practical Notes</small></h2>
<ul>
<li><strong>0.8° 的理論誤差比實際誤差小很多</strong>：銷孔間隙、伺服齒輪背隙、轉向節晃動通常都超過 1°；理論誤差在這個量級時，阿克曼幾何已經不是限制</li>
<li>阿克曼條件假設低速、輪胎不側滑；車速高時輪胎有滑移角（Slip Angle），賽車常故意用較少的阿克曼，玩具車低速則越接近理想越好</li>
<li>轉向沒有機械擋塊，伺服機要在程式裡限制在 ±{f(TH_MAX)}°；超過時傳力角繼續變小，可能卡死或讓伺服過載</li>
<li>整車的後輪距（130 mm）和前輪距不同，不影響阿克曼條件：條件只和<strong>主銷距 w</strong> 與<strong>軸距 L</strong> 有關</li>
</ul>
<p class="callout">阿克曼條件：<strong>cot δo − cot δi = w / L = {W / L:.2f}</strong>　｜　本機構打到底 {f(DI_MAX)}° / {f(DO_MAX)}°，最大誤差 {abs(E_MAX):.2f}°</p>
</section>
</main>
<footer><div class="wrap">
<ul>
<li>資料來源：mg90s_steering/tools/linkage.py（連桿求解、阿克曼理想角、尺寸掃描）；主銷、轉向臂、曲柄尺寸與 build_steering.py 的 FreeCAD 參數一致</li>
<li>阿克曼條件推導：低速、輪胎不側滑的平面運動學假設</li>
<li>此頁由 mg90s_steering/analysis/build_ackermann.py 產生</li>
</ul>
</div></footer>
"""
    page = f"""<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<meta name="robots" content="noindex">
<title>阿克曼轉向機構理論分析｜MG90S 前輪轉向</title>
<style>{CSS}</style>
</head>
<body>{body}</body>
</html>
"""
    out = HERE / "ackermann_analysis.html"
    out.write_text(page, encoding="utf-8")
    print(f"wrote {out} ({out.stat().st_size / 1e3:.0f} KB)")
    print(f"full lock {DI_MAX:.2f}/{DO_MAX:.2f} ideal {DO_IDEAL:.2f}  max err {E_MAX:.3f} at di {E_DI:.1f}  "
          f"R {R_MID:.0f}  gap {X_IN - X_OUT:.2f}  ack {ACK_PCT:.0f}%  mu min {MU_MIN:.1f}")


if __name__ == "__main__":
    build()
