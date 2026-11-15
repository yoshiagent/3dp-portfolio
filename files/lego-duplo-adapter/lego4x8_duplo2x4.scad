// 4x8 標準 LEGO 磚，頂部改為 2x4 DUPLO 凸點（轉接磚）
// 底部：可卡在 LEGO 凸點上（4x8 標準底部、3x7 管柱）
// 頂部：2x4 DUPLO 空心凸點，可接 DUPLO 磚
// 原理：DUPLO 間距 16 mm = 2 × LEGO 間距 8 mm，故 LEGO 4x8 佔地正好等於 DUPLO 2x4
// 單位：mm

/* [LEGO 底部] */
pitch       = 8.0;    // LEGO 凸點間距
units_x     = 4;      // 4x8（須為偶數）
units_y     = 8;
play        = 0.1;    // 每側外緣內縮（標準 0.1）
brick_h     = 9.6;    // 標準磚高（不含凸點）
wall        = 1.2;    // 外牆厚度
top_t       = 1.0;    // 頂板厚度
tube_od     = 6.51;   // 底部管柱外徑
tube_id     = 4.8;    // 底部管柱內徑

/* [DUPLO 凸點] */
d_pitch     = 16.0;   // DUPLO 凸點間距
d_stud_od   = 9.55;   // DUPLO 凸點外徑（官方約 9.35，Bambu X2D 實測 9.55 最剛好）
d_stud_id   = 6.7;    // DUPLO 凸點內徑（空心）
d_stud_h    = 4.5;    // DUPLO 凸點高度

$fn = 96;

size_x = units_x * pitch - 2 * play;   // 31.8
size_y = units_y * pitch - 2 * play;   // 63.8
d_nx = units_x / 2;                    // 2
d_ny = units_y / 2;                    // 4

module shell() {
    difference() {
        translate([-size_x/2, -size_y/2, 0]) cube([size_x, size_y, brick_h]);
        translate([-size_x/2 + wall, -size_y/2 + wall, -0.01])
            cube([size_x - 2*wall, size_y - 2*wall, brick_h - top_t + 0.01]);
    }
}

// 底部管柱位於凸點之間：(units_x-1) x (units_y-1) = 3x7 支
module tubes() {
    for (i = [1 : units_x - 1], j = [1 : units_y - 1])
        translate([(i - units_x/2) * pitch, (j - units_y/2) * pitch, 0])
            difference() {
                cylinder(d = tube_od, h = brick_h - top_t + 0.01);
                translate([0, 0, -0.01]) cylinder(d = tube_id, h = brick_h);
            }
}

module duplo_studs() {
    for (i = [0 : d_nx - 1], j = [0 : d_ny - 1])
        translate([(i - (d_nx - 1)/2) * d_pitch, (j - (d_ny - 1)/2) * d_pitch, brick_h - 0.01])
            difference() {
                cylinder(d = d_stud_od, h = d_stud_h + 0.01);
                translate([0, 0, 0.01]) cylinder(d = d_stud_id, h = d_stud_h + 0.01);
            }
}

union() {
    shell();
    tubes();
    duplo_studs();
}
