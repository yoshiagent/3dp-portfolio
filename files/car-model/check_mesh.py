"""Ring/pinion mesh and carrier sweep only (section 3 of check_car.py). Run from car_model/."""
import sys
sys.path.insert(0, "src")
import car_parts as c
from car_assembly import diff_parts
ch = c.chassis()
ok = True
for k in range(0, 25):
    spin = k * 3.75
    dp = diff_parts(spin)
    r = dp["ring gear"] & dp["pinion"]
    v = 0.0 if r is None else r.volume
    g = dp["ring gear"].distance_to(dp["pinion"])
    worst = min(dp[n].distance_to(ch) for n in ("carrier", "ring gear", "cross pin", "spider top", "spider bottom"))
    ok &= v < 1e-3 and worst > 0.5
    print(f"spin {spin:6.2f}  ring/pinion overlap {v:.3f}  gap {g:.3f}   carrier group to chassis {worst:.2f}")
print("PASS" if ok else "FAILED")
