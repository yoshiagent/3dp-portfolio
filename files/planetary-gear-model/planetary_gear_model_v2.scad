
// Parametric planetary gear teaching model - v2
// Units: mm. Open with OpenSCAD; edit parameters at the top.
// v2 changes (from the v1 fit check):
//  - true involute teeth (20 deg) with backlash and root clearance
//  - tooth counts 24/18/60 so three planets fit at 120 deg ((Zs+Zr)/3 = 28)
//  - solid base that carries the ring, screw holes that hit material
//  - clearance on planet bores, keyed D shaft driving the sun from the crank
//  - 0.5 mm hubs so gear faces do not rub on base or carrier
// Keep in sync with cad/src/planetary_v2_parts.py (used for the STL export and fit check).
$fn = 96;
part = "assembly"; // assembly, sun, planet, ring, carrier, base, crank, pin, shaft, shaft_print

// --- gears ---
m = 1.25;
pressure_angle = 20;
sun_teeth = 24;
planet_teeth = 18;
ring_teeth = sun_teeth + 2*planet_teeth; // 60
addendum = 1.0*m;
dedendum = 1.25*m;
backlash = 0.15;   // tooth thinning at pitch circle per gear; raise if gears bind
gear_thickness = 6;
axial_gap = 0.5;   // hub height between gear faces and base/carrier

// --- shafts and bores ---
shaft_d = 5;
shaft_flat = 0.5;  // depth of the D flat
key_clear = 0.15;  // radial clearance in D bores (sun, crank)
base_bearing_d = shaft_d + 0.4;
carrier_bore_d = shaft_d + 0.8;
pin_d = 4;
planet_bore_d = pin_d + 0.4;
carrier_pin_hole_d = pin_d + 0.1;
pin_head_d = 7;
pin_head_h = 2;
hub_r_sun = 4.5;
hub_r_planet = 4.0;

// --- frame ---
carrier_thickness = 4;
base_thickness = 5;
base_d = 120;
bearing_floor = 1;
ring_outer_r = 45;
ring_h = gear_thickness + axial_gap;
mount_r = 42;
ring_screw_d = 2.8;  // M3 self-tapping
base_screw_d = 3.4;  // M3 clearance
rim_clear = 0.2;
rim_w = 1.5;
rim_h = 3;

orbit = m*(sun_teeth + planet_teeth)/2;
planet_phase = 180/planet_teeth;
ring_phase = 180/ring_teeth;

gear_z = base_thickness;
carrier_z = gear_z + gear_thickness + 2*axial_gap;
crank_z = carrier_z + carrier_thickness + axial_gap;
pin_shank = carrier_z + carrier_thickness - (gear_z + axial_gap);
crank_hub_h = 5;
shaft_len = crank_z + crank_hub_h - bearing_floor;

// --- involute gear outline (tooth 0 on +X); negative bl widens teeth ---
function inv(a) = tan(a) - a*PI/180;
function pol(r, a) = [r*cos(a), r*sin(a)];
function gear_pts(z, add, ded, bl, n=8) =
    let(rp = m*z/2, rb = rp*cos(pressure_angle), ra = rp + add, rf = rp - ded,
        hp = (PI*m/2 - bl)/(2*rp),
        r0 = max(rf, rb),
        rs = [for (i=[0:n]) r0 + (ra - r0)*i/n])
    [for (k=[0:z-1]) let(c = k*360/z) each concat(
        rf < rb ? [pol(rf, c - th(rb, rb, hp))] : [],
        [for (r = rs) pol(r, c - th(r, rb, hp))],
        [for (j = [n:-1:0]) pol(rs[j], c + th(rs[j], rb, hp))],
        rf < rb ? [pol(rf, c + th(rb, rb, hp))] : [],
        [pol(rf, c + 180/z)])];
function th(r, rb, hp) = (hp + inv(pressure_angle) - inv(acos(rb/r)))*180/PI;

module gear_2d(z) polygon(gear_pts(z, addendum, dedendum, backlash));

module d_bore(clear, h) {
    r = shaft_d/2 + clear;
    difference() {
        translate([0,0,-0.1]) cylinder(h=h+0.2, r=r);
        translate([shaft_d/2 - shaft_flat + clear, -2*r, -1]) cube([2*r, 4*r, h+2]);
    }
}

module sun() {
    difference() {
        union() {
            cylinder(h=axial_gap, r=hub_r_sun);
            translate([0,0,axial_gap]) linear_extrude(height=gear_thickness) gear_2d(sun_teeth);
        }
        d_bore(key_clear, axial_gap + gear_thickness);
    }
}

module planet() {
    difference() {
        union() {
            cylinder(h=axial_gap, r=hub_r_planet);
            translate([0,0,axial_gap]) linear_extrude(height=gear_thickness) gear_2d(planet_teeth);
            translate([0,0,axial_gap+gear_thickness]) cylinder(h=axial_gap, r=hub_r_planet);
        }
        translate([0,0,-0.1]) cylinder(h=gear_thickness+2*axial_gap+0.2, d=planet_bore_d);
    }
}

module ring() {
    difference() {
        linear_extrude(height=ring_h) difference() {
            circle(r=ring_outer_r);
            rotate([0,0,ring_phase]) polygon(gear_pts(ring_teeth, dedendum, addendum, -backlash));
        }
        for (a=[0:120:240]) rotate([0,0,a])
            translate([mount_r,0,-0.1]) cylinder(h=ring_h+0.2, d=ring_screw_d);
    }
}

module carrier() {
    difference() {
        cylinder(h=carrier_thickness, r=orbit+5);
        translate([0,0,-0.1]) cylinder(h=carrier_thickness+0.2, d=carrier_bore_d);
        for (a=[0:120:240]) rotate([0,0,a])
            translate([orbit,0,-0.1]) cylinder(h=carrier_thickness+0.2, d=carrier_pin_hole_d);
    }
}

module base() {
    rim_r = ring_outer_r + rim_clear;
    difference() {
        union() {
            cylinder(h=base_thickness, d=base_d);
            translate([0,0,base_thickness]) difference() {
                cylinder(h=rim_h, r=rim_r+rim_w);
                translate([0,0,-0.1]) cylinder(h=rim_h+0.2, r=rim_r);
            }
        }
        translate([0,0,bearing_floor]) cylinder(h=base_thickness, d=base_bearing_d);
        for (a=[0:120:240]) rotate([0,0,a])
            translate([mount_r,0,-0.1]) cylinder(h=base_thickness+0.2, d=base_screw_d);
    }
}

module crank() {
    difference() {
        cylinder(h=crank_hub_h, d=18);
        d_bore(key_clear, crank_hub_h);
    }
    hull() {
        translate([7,0,2.5]) cylinder(h=5, d=5);
        translate([28,0,2.5]) cylinder(h=5, d=5);
    }
    translate([28,0,2.5]) cylinder(h=5, d=9);
    translate([28,0,7.5]) cylinder(h=12, d=5);
}

// print head-down
module pin() {
    cylinder(h=pin_head_h, d=pin_head_d);
    translate([0,0,pin_head_h]) cylinder(h=pin_shank, d=pin_d);
}

// D shaft along +Z, flat facing +X; print it lying on the flat
module shaft() {
    difference() {
        cylinder(h=shaft_len, d=shaft_d);
        translate([shaft_d/2 - shaft_flat, -shaft_d, -1]) cube([shaft_d, 2*shaft_d, shaft_len+2]);
    }
}

// shaft in print orientation: flat face down on the bed
module shaft_print() {
    translate([0,0,shaft_d/2 - shaft_flat]) rotate([0,90,0]) shaft();
}

module assembly() {
    color("dimgray") base();
    translate([0,0,gear_z]) color("orange") ring();
    translate([0,0,gear_z]) color("gold") sun();
    for (a=[0:120:240]) rotate([0,0,a]) translate([orbit,0,gear_z]) {
        color("deepskyblue") rotate([0,0,planet_phase]) planet();
        // pin head rests on the carrier top
        color("white") translate([0,0,carrier_z+carrier_thickness+pin_head_h-gear_z]) rotate([180,0,0]) pin();
    }
    translate([0,0,carrier_z]) color("limegreen") carrier();
    translate([0,0,bearing_floor]) color("silver") shaft();
    translate([0,0,crank_z]) color("red") crank();
}

if (part=="sun") sun();
else if (part=="planet") planet();
else if (part=="ring") ring();
else if (part=="carrier") carrier();
else if (part=="base") base();
else if (part=="crank") crank();
else if (part=="pin") pin();
else if (part=="shaft") shaft();
else if (part=="shaft_print") shaft_print();
else assembly();
