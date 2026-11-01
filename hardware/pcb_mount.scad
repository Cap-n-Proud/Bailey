// Holders for Bailey's boards: no glue, no holes needed in the board.
//
// board_clamp()  ANY board, holes or not: give its size and it is redesigned.
//                The board slides its edge into a groove on one side and is
//                clamped on the other by a bar with two M3 screws (into heat-set
//                inserts). Firm against vibration, and adjustable (`keepouts`)
//                around connectors and parts at the edges. Under the board the
//                tray is open for pins, wires and air.
// imu_mount()    bracket that puts the IMU chip on the wheel axle's line,
//                adjustable in height (slots in the mounting plate) and
//                fore-aft (slots under the board), with a sight mark at the
//                chip's height to line it up with the axle centre.
// pcb_posts()    boards with mounting holes: posts with pilot holes.
// pcb_clip()     snap-in alternative to board_clamp (no screws, less firm).
// pico_socket()  the ESP32 plugged into female header strips.
//
// Measure each board with calipers: length, width, thickness, and where parts
// or connectors reach its edges. Print flat, as exported, PLA or PETG,
// 0.2 mm layers, no supports.
//
// Pick the part to export (F6, then File > Export > STL):
part = "esp32";   // esp32, esp32_bar, driver, driver_bar, buck, buck_bar, imu, preview

$fn = 40;

// ---------- Bailey's boards (measure yours and correct) ----------

// Waveshare ESP32-S3-Pico: gripped by its short ends; the long edges have the
// pins. The USB-C end is the clamp bar's side, with a gap for the plug; the
// gap on the other end clears the small part (C3) at the edge.
// gap 20: pins soldered pointing down, Dupont plugs on them, wires out of
// the open sides. With the pins pointing up (or wires soldered), gap 4.
esp32 = [ "l", 51, "w", 21, "t", 1.0, "grip", "short", "gap", 20, "ledge_inset", 3.5,
          "keep_fixed", [[0, 6]], "keep_bar", [[0, 12]] ];
// TMC2209 / TMC2226 (StepStick, 15.3 x 20.4 mm): gripped by the short ends,
// pins down under the board, Dupont plugs on them. A bare driver needs a 100 uF capacitor
// across VM and GND, close to it (the carrier boards have one).
driver = [ "l", 20.4, "w", 15.3, "t", 1.6, "grip", "short", "gap", 20, "ledge_inset", 3.2,
           "keep_fixed", [], "keep_bar", [] ];
// XL4015 buck converter: gripped by the long edges (its terminals are at the ends).
buck = [ "l", 52, "w", 26, "t", 1.6, "grip", "long", "gap", 4, "ledge_inset", 2,
         "keep_fixed", [], "keep_bar", [] ];

function p(b, k) = b[search([k], b)[0] + 1];
module clamp_for(b, which = "tray")
    board_clamp(l = p(b, "l"), w = p(b, "w"), t = p(b, "t"), grip = p(b, "grip"), gap = p(b, "gap"),
                ledge_inset = p(b, "ledge_inset"), keep_fixed = p(b, "keep_fixed"),
                keep_bar = p(b, "keep_bar"), which = which);

if (part == "esp32") clamp_for(esp32);
if (part == "esp32_bar") clamp_for(esp32, "bar");
if (part == "driver") clamp_for(driver);
if (part == "driver_bar") clamp_for(driver, "bar");
if (part == "buck") clamp_for(buck);
if (part == "buck_bar") clamp_for(buck, "bar");
// GY-521 (MPU-6050): 21 x 16 mm, two 3 mm holes along one long edge, the chip
// near the middle. Measure the holes and the chip's position on your board.
if (part == "imu")
    imu_mount(l = 21, w = 16, holes = [[-7.6, 5.4], [7.6, 5.4]], chip = [0, 0]);
if (part == "preview") {
    clamp_for(esp32, "assembly");
    translate([0, 45, 0]) clamp_for(driver, "assembly");
    translate([80, 20, 0]) imu_mount(l = 21, w = 16, holes = [[-7.6, 5.4], [7.6, 5.4]], chip = [0, 0]);
}

// ---------- universal clamp ----------

// l, w, t        board length, width, thickness
// grip           "short": hold the two short edges (pins along the long ones);
//                "long": hold the long edges
// gap            space under the board (solder joints, pin headers' plastic)
// lip            how far the groove and the bar reach over the board's top
// ledge          how far the board rests on the ledges underneath
// ledge_inset    no ledge this close to the corners (pin headers under them)
// keep_fixed     gaps cut through the groove side, [centre, width] along the
// keep_bar       gripped edge (centre 0 = middle): connectors, parts at the edge
// which          "tray", "bar" (print it upside down: flat side on the bed),
//                or "assembly" to look at both
module board_clamp(l, w, t = 1.6, grip = "short", gap = 4, lip = 1.0, ledge = 1.2, ledge_inset = 3,
                   keep_fixed = [], keep_bar = [], clear = 0.3, squeeze = 0.15, base = 2.4,
                   wall = 2.4, seat = 8, bar_t = 3, which = "tray") {
    span = grip == "short" ? max(l, w) : min(l, w);   // across, between the gripped edges
    edge = grip == "short" ? min(l, w) : max(l, w);   // along the gripped edges
    z0 = base + gap;                                  // board's underside
    top = z0 + t;                                     // board's top
    gy = edge / 2 + clear;                            // half the pocket along the edge
    cap = 2;                                          // end caps: stop the board sliding along the edge
    xf = -span / 2 - clear;                           // fixed side's inner face
    xb = span / 2 + clear;                            // bar side's inner face
    sy = edge >= 16 ? [-edge / 2 + 3.5, edge / 2 - 3.5] : [-(gy + cap) - 3.5, gy + cap + 3.5];  // bar screws
    outer_y = max(gy + cap + 6, max([for (y = sy) abs(y)]) + 4.5);   // room for ties and screws

    module keepout(list, x0, x1) for (k = list)
        translate([x0, k[0] - k[1] / 2, z0 - 3]) cube([x1 - x0, k[1], 50]);

    module tray() difference() {
        union() {
            // base: a frame around an open middle
            translate([xf - wall, -outer_y, 0]) cube([xb + seat - xf + wall, 2 * outer_y, base]);
            // fixed side: wall, groove, top lip
            translate([xf - wall, -gy - cap, 0]) cube([wall, 2 * (gy + cap), top + 0.1 + bar_t]);
            translate([xf, -gy - cap, top + 0.1]) cube([lip + clear, 2 * (gy + cap), bar_t]);
            // ledges the board rests on, both sides, clear of the corners
            for (x = [xf, xb - ledge - clear])
                translate([x, -gy + ledge_inset, 0]) cube([ledge + clear, 2 * (gy - ledge_inset), z0]);
            // bar seat: as high as the board's top, a little less so the bar clamps it
            translate([xb, -outer_y, 0]) cube([seat, 2 * outer_y, top - squeeze]);
            // end caps on both sides
            for (y = [-gy - cap, gy], x = [xf, xb - lip - clear])
                translate([x, y, 0]) cube([lip + clear, cap, z0 + (x == xf ? t + 0.1 + bar_t : t - squeeze)]);
        }
        // the groove
        translate([xf - 0.01, -gy, z0]) cube([lip + clear + 0.02, 2 * gy, t + 0.1]);
        // open middle
        translate([xf + ledge + clear + 0.5, -gy, -1]) cube([span - 2 * ledge - 1, 2 * gy, base + 2]);
        // inserts for the bar's screws (M3 heat-set: 4.0 mm, check the insert's datasheet)
        for (y = sy) translate([xb + seat / 2, y, top - squeeze - 5.5]) cylinder(d = 4.0, h = 6);
        // cable-tie slots and an M3 hole at both ends, to fix the tray to a shelf
        for (s = [-1, 1]) translate([0, s * (outer_y - 3), -1]) {
            translate([-6, -1, 0]) cube([3, 2, base + 2]);
            translate([3, -1, 0]) cube([3, 2, base + 2]);
            cylinder(d = 3.4, h = base + 2);
        }
        // an arch through the tall seat, between the screws: wires, and less plastic
        aw = 2 * min([for (y = sy) abs(y)]) - 8;
        if (aw > 4 && z0 - base > 8)
            translate([xb - ledge - clear - 1, 0, base]) rotate([90, 0, 90])
                linear_extrude(ledge + clear + seat + 2) arch(aw, z0 - base - 3);
        keepout(keep_fixed, xf - wall - 1, xf + lip + clear + 1);
        keepout(keep_bar, xb - lip - clear - 1, xb + seat + 1);
    }

    // The clamp bar: lies on the seat and reaches over the board's edge.
    module bar() difference() {
        translate([xb - lip - clear, -outer_y, top - squeeze]) cube([lip + clear + seat, 2 * outer_y, bar_t]);
        for (y = sy) translate([xb + seat / 2, y, 0]) cylinder(d = 3.4, h = 50);
        keepout(keep_bar, xb - lip - clear - 1, xb + seat + 1);
    }

    if (which == "tray") tray();
    if (which == "bar") translate([0, 0, top - squeeze + bar_t]) rotate([180, 0, 0]) bar();  // flat side down
    if (which == "assembly") {
        tray();
        color("orange") bar();
        color("green", 0.5) translate([-span / 2, -edge / 2, z0]) cube([span, edge, t]);   // the board
    }
}

// ---------- IMU on the axle line ----------

// The bracket screws to a vertical face behind the axle (the back of the
// base, or a side plate) through two vertical slots: slide it up or down
// until the sight mark lines up with the axle centre, then tighten. The
// board sits on posts on a platform reaching `reach` forward, with slots for
// sliding it fore-aft onto the axle line. Any orientation works for the
// firmware (imuAxis, imuInvert), as long as the board is square to the axle.
//
// l, w, holes   board size and hole centres (relative to the board's centre)
// chip          the IMU chip's centre on the board
// reach         mounting face to the axle line (e.g. half the base's depth)
// drop          slots' centre to the platform's top
// slot          adjustment range of each slot
// post          posts' height (room for the header's solder joints)
module imu_mount(l, w, holes, chip = [0, 0], t = 1.6, reach = 40, drop = 15, slot = 12,
                 post = 5, pilot = 2.6, plate = 3, slot_y = 8) {
    pw = max(l + 10, 30);                 // bracket width
    ph = drop + slot / 2 + 8;              // mounting plate height above the platform
    py = reach + w / 2 + 6;                // platform depth
    chip_z = plate + post + t + 0.5;       // chip height above the platform's underside
    difference() {
        union() {
            // mounting plate (vertical, against the face) and platform
            translate([-pw / 2, 0, 0]) cube([pw, plate, ph + plate]);
            translate([-pw / 2, 0, 0]) cube([pw, py, plate]);
            // gussets: keep the platform from flexing; they carry the sight marks
            for (x = [-pw / 2, pw / 2 - 2.4])
                translate([x, 0, 0]) rotate([90, 0, 90]) linear_extrude(2.4)
                    polygon([[0, 0], [py, 0], [py, chip_z + 1], [plate + 4, ph * 0.8], [0, ph * 0.8]]);
            // posts under the board's holes
            for (h = holes) translate([h[0] - chip[0], reach + h[1] - chip[1] - slot_y / 2, 0])
                hull() for (d = [0, slot_y]) translate([0, d, 0]) cylinder(d = 6, h = plate + post);
        }
        // vertical slots in the mounting plate (M3)
        for (x = [-pw / 4, pw / 4])
            hull() for (z = [-slot / 2, slot / 2]) translate([x, -1, plate + drop + z])
                rotate([-90, 0, 0]) cylinder(d = 3.4, h = plate + 2);
        // fore-aft slots through posts and platform: M2.5/M3 screw with a nut underneath
        for (h = holes) translate([h[0] - chip[0], reach + h[1] - chip[1] - slot_y / 2, -1])
            hull() for (d = [0, slot_y]) translate([0, d, 0]) cylinder(d = pilot + 0.6, h = plate + post + 2);
        // sight marks: a V notch on both gussets at the chip's height and on the axle line
        for (x = [-pw / 2 - 1, pw / 2 - 3.4])
            translate([x, reach, chip_z]) rotate([0, 90, 0]) cylinder(d = 2.5, h = 4.4, $fn = 4);
        // a window in the platform under the board (pins, air)
        translate([-l / 2 + 4 - chip[0], reach - w / 2 + 3 - chip[1], -1]) cube([max(l - 8, 1), max(w - 9, 1), plate + 2]);
    }
}

// ---------- holder for boards without holes ----------

// l, w, t     board length, width, thickness (mm)
// gap         space under the board for solder joints and pins
// clear       play around the board
// lip         how far each hook reaches over the board's edge
// hooks       hook positions along each long side, as fractions of l (-0.5..0.5)
// ears        M3 screw tabs at both ends
module pcb_clip(l, w, t = 1.6, gap = 6, clear = 0.3, lip = 0.8, hooks = [-0.25, 0.25],
                ears = true, base = 2.4, arm = 1.2, hook_w = 6, hook_h = 2.5) {
    L = l + 2 * clear;              // pocket
    W = w + 2 * clear;
    top = base + gap;               // board's underside
    wall = arm;
    difference() {
        union() {
            // base plate
            translate([-L / 2 - wall, -W / 2 - wall, 0]) cube([L + 2 * wall, W + 2 * wall, base]);
            if (ears) for (s = [-1, 1]) ear(s * (L / 2 + wall), base, W + 2 * wall);
            // corner pads the board rests on
            for (sx = [-1, 1], sy = [-1, 1])
                translate([sx * (L / 2 - 2) - 2, sy * (W / 2 - 1.5) - 1.5, 0]) cube([4, 3, top]);
            // end stops, as high as the board
            for (sx = [-1, 1])
                translate([sx * (L / 2 + wall / 2) - wall / 2, -min(8, W / 3) / 2, 0])
                    cube([wall, min(8, W / 3), top + t]);
            // flexible hooks on the long sides
            for (f = hooks, sy = [-1, 1])
                translate([f * l, sy * (W / 2 + arm / 2), 0])
                    rotate([0, 0, sy > 0 ? 0 : 180]) hook(top + t, arm, hook_w, lip + clear, hook_h);
        }
        // slots in the base for a cable tie, as an alternative to the screws
        for (sx = [-1, 1])
            translate([sx * L / 4 - 1.5, -W / 2 + 1, -1]) cube([3, W - 2, base + 2]);
    }
}

// One hook: an arm standing on the plate, with a sloped catch at the top that
// the board pushes outwards on its way down and then sits under. Built for the
// +y side (catch pointing towards -y).
module hook(height, arm, width, reach, hook_h) {
    translate([-width / 2, -arm / 2, 0]) cube([width, arm, height + hook_h]);
    // catch: flat underside at the board's top, slope above it
    translate([-width / 2, -arm / 2, height + 0.1])
        rotate([90, 0, 90]) linear_extrude(width)
            polygon([[0, 0], [-reach, 0], [0, hook_h]]);
}

// M3 screw tab at x (the plate's end), countersunk from the top.
module ear(x, base, w) {
    s = sign(x);
    difference() {
        translate([x + (s > 0 ? 0 : -9), -w / 2, 0]) cube([9, w, base]);
        translate([x + s * 4.5, 0, -1]) cylinder(d = 3.4, h = base + 2);
        translate([x + s * 4.5, 0, base - 1.6]) cylinder(d1 = 3.4, d2 = 6.6, h = 1.61);
    }
}

// ---------- posts for boards with holes ----------

// holes      hole centres relative to the board's centre
// pilot      hole in the post for a self-tapping screw: 1.7 for M2, 2.2 for
//            M2.5, 2.6 for M3 (or the heat-set insert's size)
module pcb_posts(l, w, holes, pilot = 2.2, gap = 6, post = 5.5, base = 2.4, ears = true) {
    difference() {
        union() {
            translate([-l / 2, -w / 2, 0]) cube([l, w, base]);
            if (ears) for (s = [-1, 1]) ear(s * l / 2, base, w);
            for (h = holes) translate([h[0], h[1], 0]) cylinder(d = post, h = base + gap);
        }
        for (h = holes) translate([h[0], h[1], 1]) cylinder(d = pilot, h = base + gap);
        // weight and airflow: a window between the posts
        translate([-l / 2 + 7, -w / 2 + 4, -1]) cube([l - 14, w - 8, base + 2]);
    }
}

// ---------- socket for the ESP32-S3-Pico ----------

// The board gets male pin headers soldered pointing down and plugs into two
// 1 x 20 female header strips, which press into the slots of this socket.
// With "stacking" female headers (long tails, 10-11 mm) the wires plug onto
// the tails underneath with Dupont connectors: nothing soldered to the
// ESP32's pins, and the board comes out by pulling it up. Ordinary female
// headers work too, with the wires soldered to their tails.
//
// pins, pitch, row   pins per side, pin spacing, distance between the rows
// hdr_w, hdr_h       female header body width and height (measure yours)
// legs               height of the end legs: room under the socket for the
//                    tails and the Dupont plugs
// leg_t, pillar      end leg thickness, and width of the arch's sides
//
// Both ends are open arches, for the wires and to save material. The arches
// are pointed (equilateral), so they print without support; the flat
// underside of the block between the legs is a bridge (or add support there).
module pico_socket(pins = 20, pitch = 2.54, row = 17.78, hdr_w = 2.54, hdr_h = 8.5,
                   legs = 22, wall = 2.5, fit = 0.15, base = 2.4, leg_t = 4, pillar = 3) {
    len = pins * pitch;
    sw = hdr_w + 2 * fit;                       // slot width: a press fit
    bw = row + hdr_w + 2 * wall;                // block width
    bl = len + 2 * wall;                        // block length
    difference() {
        union() {
            // the socket block, carried on two end legs
            translate([-bl / 2, -bw / 2, legs]) cube([bl, bw, hdr_h]);
            for (s = [-1, 1]) {
                translate([s > 0 ? bl / 2 - leg_t : -bl / 2, -bw / 2, 0]) cube([leg_t, bw, legs]);
                ear(s * bl / 2, base, bw);
            }
        }
        // the arches through both end legs
        for (s = [-1, 1])
            translate([s > 0 ? bl / 2 - leg_t - 1 : -bl / 2 - 1, 0, base])   // above the sill
                rotate([90, 0, 90]) linear_extrude(leg_t + 2) arch(bw - 2 * pillar, legs - 1.5 - base);
        for (s = [-1, 1]) {
            // header slots, through, with a ledge at the bottom the plastic rests on
            translate([-len / 2 - fit, s * row / 2 - sw / 2, legs + 1]) cube([len + 2 * fit, sw, hdr_h]);
            translate([-len / 2, s * row / 2 - 0.8, legs - 1]) cube([len, 1.6, 3]);
        }
        // window between the rows (lighter, and air under the ESP32)
        translate([-len / 2 + 3, -row / 2 + 3, legs - 1]) cube([len - 6, row - 6, hdr_h + 2]);
    }
}

// A pointed (equilateral) arch opening, span wide and height high, standing
// on y = 0: vertical sides, then two arcs meeting at the top. Its overhang
// never exceeds 30 degrees from vertical, so it prints without support.
module arch(span, height) {
    r = span;                                  // equilateral: radius = span
    rise = span * sqrt(3) / 2;                 // height of the pointed part
    side = max(height - rise, 0);
    translate([-span / 2, 0]) square([span, side + 0.01]);
    translate([0, side]) intersection() {
        translate([-span / 2, 0]) circle(r = r, $fn = 120);
        translate([span / 2, 0]) circle(r = r, $fn = 120);
        translate([-span / 2, 0]) square([span, rise]);
    }
}
