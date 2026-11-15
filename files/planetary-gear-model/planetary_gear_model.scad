
// Parametric planetary gear teaching model
// Units: mm. Open with OpenSCAD; edit parameters at the top.
// Gear geometry is intentionally simplified for FDM teaching prototype,
// not a precision involute gearbox.
$fn = 96;
part = "assembly"; // assembly, sun, planet, ring, carrier, base, crank, pin
m = 1.0;
sun_teeth = 20;
planet_teeth = 20;
ring_teeth = 60;
gear_thickness = 6;
backlash = 0.28; // increase if your printer makes gears tight
shaft_d = 5.2;
pin_d = 4.2;
carrier_thickness = 4;
base_thickness = 5;
outer_d = 120;

pitch_r_s = m*sun_teeth/2;
pitch_r_p = m*planet_teeth/2;
pitch_r_r = m*ring_teeth/2;
planet_orbit = pitch_r_s + pitch_r_p;
root_r_ext = m*(sun_teeth-2.5)/2;
outer_r_ext = m*(sun_teeth+2)/2;
root_r_planet = m*(planet_teeth-2.5)/2;
outer_r_planet = m*(planet_teeth+2)/2;
ring_inner_tip_r = pitch_r_r - m*1.25;
ring_inner_root_r = pitch_r_r + m*1.25;
ring_outer_r = 34.5;

module ext_gear(teeth=20, thick=6, bore=5.2) {
    pr = m*teeth/2;
    rr = m*(teeth-2.5)/2;
    ro = m*(teeth+2)/2;
    tooth_pitch = 360/teeth;
    difference() {
        union() {
            cylinder(h=thick, r=rr);
            for (i=[0:teeth-1]) rotate([0,0,i*tooth_pitch])
                linear_extrude(height=thick)
                    polygon(points=[
                        [rr*cos(-tooth_pitch*0.34), rr*sin(-tooth_pitch*0.34)],
                        [ro*cos(-tooth_pitch*0.19), ro*sin(-tooth_pitch*0.19)],
                        [ro*cos(tooth_pitch*0.19), ro*sin(tooth_pitch*0.19)],
                        [rr*cos(tooth_pitch*0.34), rr*sin(tooth_pitch*0.34)]
                    ]);
        }
        translate([0,0,-0.1]) cylinder(h=thick+0.2, d=bore);
    }
}
module sun() { ext_gear(sun_teeth, gear_thickness, shaft_d); }
module planet() { ext_gear(planet_teeth, gear_thickness, pin_d); }

module ring_2d() {
    // Internal tooth profile represented by a radial polygon with shallow tooth spaces.
    n=ring_teeth*8;
    pts = [for (i=[0:n-1]) let(a=i*360/n, phase=(i%8))
        [((phase==0 || phase==1 || phase==6 || phase==7) ? ring_inner_root_r : ring_inner_tip_r)*cos(a),
         ((phase==0 || phase==1 || phase==6 || phase==7) ? ring_inner_root_r : ring_inner_tip_r)*sin(a)]];
    difference() {
        circle(r=ring_outer_r);
        polygon(points=pts);
    }
}
module ring() {
    difference() {
        linear_extrude(height=gear_thickness) ring_2d();
        // Three mounting holes
        for (a=[0:120:240]) rotate([0,0,a])
            translate([32.0,0,-0.1]) cylinder(h=gear_thickness+0.2,d=3.4);
    }
}
module carrier() {
    difference() {
        union() {
            cylinder(h=carrier_thickness, r=planet_orbit+outer_r_planet*0.0+5);
            // three arms / pin bosses
            for (a=[0:120:240]) rotate([0,0,a])
                translate([planet_orbit,0,0]) cylinder(h=carrier_thickness,r=4.8);
        }
        translate([0,0,-0.1]) cylinder(h=carrier_thickness+0.2,d=shaft_d+0.6);
        for (a=[0:120:240]) rotate([0,0,a])
            translate([planet_orbit,0,-0.1]) cylinder(h=carrier_thickness+0.2,d=pin_d+0.3);
    }
}
module base() {
    difference() {
        union() {
            cylinder(h=base_thickness,d=outer_d);
            translate([0,0,base_thickness]) cylinder(h=3,r=ring_outer_r+1.5);
        }
        translate([0,0,-0.1]) cylinder(h=base_thickness+3.2,d=ring_outer_r*2+0.6);
        for (a=[0:120:240]) rotate([0,0,a])
            translate([32,0,-0.1]) cylinder(h=base_thickness+3.2,d=3.4);
    }
}
module crank() {
    union() {
        difference() {
            cylinder(h=5,d=18);
            translate([0,0,-0.1]) cylinder(h=5.2,d=shaft_d);
        }
        translate([0,0,0]) rotate([0,0,0])
            hull() {
                translate([7,0,2.5]) cylinder(h=5,d=5);
                translate([28,0,2.5]) cylinder(h=5,d=5);
            }
        translate([28,0,2.5]) cylinder(h=5,d=9);
        translate([28,0,7.5]) cylinder(h=12,d=5);
    }
}
module pin() {
    union() {
        cylinder(h=gear_thickness+carrier_thickness+1,d=pin_d);
        translate([0,0,gear_thickness+carrier_thickness+1]) cylinder(h=2,d=6);
    }
}
module assembly() {
    // base at z=0; ring sits on base raised rim; gears at z=base_thickness+3
    color("dimgray") base();
    translate([0,0,base_thickness+3]) color("orange") ring();
    translate([0,0,base_thickness+3]) color("gold") sun();
    for (a=[0:120:240]) rotate([0,0,a])
        translate([planet_orbit,0,base_thickness+3]) color("deepskyblue") planet();
    translate([0,0,base_thickness+3+gear_thickness]) color("limegreen") carrier();
    // crank placed on central shaft above carrier
    translate([0,0,base_thickness+3+gear_thickness+carrier_thickness]) color("red") crank();
}
if (part=="sun") sun();
else if (part=="planet") planet();
else if (part=="ring") ring();
else if (part=="carrier") carrier();
else if (part=="base") base();
else if (part=="crank") crank();
else if (part=="pin") pin();
else assembly();
