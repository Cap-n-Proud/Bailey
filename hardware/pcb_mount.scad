// Snap-in holders for Bailey's small boards: no glue, no holes in the board.
//
// pcb_clip()  boards without mounting holes (buck converter, ...): the board
//             rests on corner pads and clicks in under flexible hooks on its
//             long sides; end stops stop it sliding. The holder screws to a
//             shelf with two M3 screws (or a cable tie through the slots).
// pcb_posts() boards with mounting holes (GY-521, ...): posts with pilot
//             holes for small self-tapping screws.
// pico_socket() the ESP32-S3-Pico, which has no mounting holes: it plugs
//             into two female header strips held in a printed socket, like a
//             chip in its socket (see below).
//
// Measure the board with calipers (length, width, thickness) and check where
// its components sit at the edges: the hooks overlap the top of the board by
// `lip`, so put them (`hooks`, fractions of the length) where the edge is
// free. Print flat, base down, PLA or PETG, 0.2 mm layers, no supports.
// Stepper drivers: don't clip them, plug them into the carrier boards, which
// have screw holes and the filter capacitor the driver needs.
//
// Pick the part to export (F6, then File > Export > STL):
part = "buck";          // "buck", "esp32", "test"

$fn = 40;

// ---------- boards ----------

if (part == "buck")
    // XL4015-type buck converter (the blue one with two 220 uF capacitors).
    // These sizes are estimates from a photo: measure yours.
    pcb_clip(l = 52, w = 23.5, t = 1.6, hooks = [-0.15, 0.15]);

if (part == "esp32")
    // Waveshare ESP32-S3-Pico: 2 x 20 pins, 2.54 mm apart, rows 17.78 mm apart
    // (Raspberry Pi Pico layout), no mounting holes.
    pico_socket();

if (part == "test")
    // A short strip to check the fit and the hook's snap before printing the
    // whole holder: same width and thickness, 20 mm long.
    pcb_clip(l = 20, w = 23.5, t = 1.6, hooks = [0], ears = false);

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
module pico_socket(pins = 20, pitch = 2.54, row = 17.78, hdr_w = 2.54, hdr_h = 8.5,
                   legs = 22, wall = 2.5, fit = 0.15, base = 2.4) {
    len = pins * pitch;
    sw = hdr_w + 2 * fit;                       // slot width: a press fit
    bw = row + hdr_w + 2 * wall;                // block width
    bl = len + 2 * wall;                        // block length
    difference() {
        union() {
            // the socket block, carried on two end legs
            translate([-bl / 2, -bw / 2, legs]) cube([bl, bw, hdr_h]);
            for (s = [-1, 1]) {
                translate([s > 0 ? bl / 2 - 6 : -bl / 2, -bw / 2, 0]) cube([6, bw, legs]);
                ear(s * bl / 2, base, bw);
            }
        }
        for (s = [-1, 1]) {
            // header slots, through, with a ledge at the bottom the plastic rests on
            translate([-len / 2 - fit, s * row / 2 - sw / 2, legs + 1]) cube([len + 2 * fit, sw, hdr_h]);
            translate([-len / 2, s * row / 2 - 0.8, legs - 1]) cube([len, 1.6, 3]);
        }
        // window between the rows (lighter, and air under the ESP32)
        translate([-len / 2 + 3, -row / 2 + 3, legs - 1]) cube([len - 6, row - 6, hdr_h + 2]);
    }
}
