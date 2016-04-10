Zdim = 40;
Xdim = 60;
Ydim = 67;
thickness = 2;
supportDiam = 5;
buttonDiam = 16;
XT60X = 15.54;
XT60Y = 8.16;
XT60Z = 16.10;

textHV = "HVDC";
textLV = "5V";
$fn=80;

//--------------------------------
module box(Xdim, Ydim, Zdim, thickness, application)
    //Creates a box, dimensions are internal
    {
        difference() {

            cube([Xdim + thickness, Ydim + thickness, Zdim], center = true);
            translate([0, 0, thickness / 2]) cube([Xdim, Ydim, Zdim], center = true);

            if (application == "servo") {
                translate([-Xdim / 2, 0, -0.5 * Zdim]) cube([5, Ydim + 2 * thickness, Zdim / 2 + thickness], center = true);
                translate([+Xdim / 2, 0, -0.5 * Zdim]) cube([5, Ydim + 2 * thickness, Zdim / 2 + thickness], center = true);
                translate([+Xdim / 2, 0, 0]) cube([thickness / 2, 0.3 * Ydim, Zdim], center = true);
                translate([-Xdim / 2, 0, 0]) cube([thickness / 2, 0.3 * Ydim, Zdim], center = true);
            }

        }
    }



module support(shape, D1, H1, D1H, D2, H2, D2H, offsetval, tolerance) {
    difference() {
        union() {

                if (shape == "square") {
                    // translate([0,0,H1/2])
                    cube([D1, D1, H1], center = true);
                    translate([offsetval, 0, (H2 / 2 + (H2 + H1) / 3)]) cube([D2, D2, H2], center = true);

                } else {
                    cylinder(r = (D1 / 2), h = H1, center = false);
                    translate([offsetval, 0, H1]) cylinder(r = (D2 / 2), h = H2, center = false);
                }

                if (offsetval != 0) {
                    translate([0, 0, H1 - (H1 + H2) / 6])
                    hull() { //Connecition
                        if (shape == "square") {
                            translate([0, 0, (H1 + H2) / 9]) cube([D1, D1, (H1 + H2) / 3], center = true);
                            translate([offsetval, 0, (H1 + H2) / 9]) cube([D2, D2, (H1 + H2) / 3], center = true);

                        } else {
                            cylinder(r = (D1 / 2), h = (H1 + H2) / 3, center = false);
                            translate([offsetval, 0, 0]) cylinder(r = D2 / 2, h = (H1 + H2) / 3, center = false);
                        }
                    }
                }
            }
            //Holes
        cylinder(r = D1H / 2 + tolerance, h = 3 * H1, center = false);
        translate([offsetval, 0, H1 - (H1 + H2) / 2]) cylinder(r = D2H / 2 + tolerance, h = 3 * H2, center = false);
    }

}


module SUB_pillars() {
    translate([-Xdim / 2 + supportDiam / 2, -Ydim / 2 + supportDiam / 2, 0]) support("square", supportDiam, Zdim, 2.5, 0, 0, 0, 0, 0.2);
    translate([(Xdim / 2 - supportDiam / 2), (Ydim / 2 - supportDiam / 2), 0]) support("square", supportDiam, Zdim, 2.5, 0, 0, 0, 0, 0.2);
    translate([Xdim / 2 - supportDiam / 2, -Ydim / 2 + supportDiam / 2, 0]) support("square", supportDiam, Zdim, 2.5, 0, 0, 0, 0, 0.2);
    translate([-Xdim / 2 + supportDiam / 2, Ydim / 2 - supportDiam / 2, 0]) support("square", supportDiam, Zdim, 2.5, 0, 0, 0, 0, 0.2);
}

module SUB_regulatorSupport() {
    translate([0, 0, 0]) support("round", supportDiam, 8, 2, 0, 0, 0, 0, 0.2);
    translate([11, 33, 0]) support("round", supportDiam, 8, 2, 0, 0, 0, 0, 0.2);


}

module powerPlant() {
    difference() {
        box(Xdim, Ydim, Zdim, thickness, "box");
        //Button Holes
        translate([Xdim / 4, Ydim / 2, -buttonDiam / 2]) rotate([90, 0, 0]) {

            cylinder(r = buttonDiam / 2, h = 10, center = true);
            translate([0, buttonDiam + 3, 0]) cylinder(r = buttonDiam / 2, h = 10, center = true);
        }
    }
    SUB_pillars();


    translate([-Xdim / 4, -Ydim / 5, -Zdim / 2]) SUB_regulatorSupport();

    //XT60 holder
    translate([0, -Ydim / 2 + XT60Y / 2, 0]) box(XT60X, XT60Y, Zdim, thickness, "servo");


}

module labelVoltage(textL) {

    translate([0, 0, 0]) rotate([0, 0, -90]) linear_extrude(height = thickness / 2) text(textL, font = "Liberation Sans", size = 4);
    translate([4, 3, 0]) rotate([0, 0, -90]) linear_extrude(height = thickness / 2) text("+", font = "Liberation Sans", size = 2);
    translate([0, 3, 0]) rotate([0, 0, -90]) linear_extrude(height = thickness / 2) text("-", font = "Liberation Sans", size = 2);
}

module topLid() {

    translate([0, 0, 0])
    difference() {
        cube([Xdim + thickness, Ydim + thickness, thickness / 2], center = true);
        SUB_lidHoles();
    }
    translate([-Xdim / 4, -Ydim / 4, 0]) labelVoltage(textHV);
    translate([-Xdim / 4 - 12, -Ydim / 4, 0]) labelVoltage(textLV);

}


module SUB_lidHoles() {
    translate([-Xdim / 2 + supportDiam / 2, -Ydim / 2 + supportDiam / 2, -5]) cylinder(r = 2.5 / 2, h = 10);
    translate([(Xdim / 2 - supportDiam / 2), (Ydim / 2 - supportDiam / 2), -5]) cylinder(r = 2.5 / 2, h = 10);
    translate([Xdim / 2 - supportDiam / 2, -Ydim / 2 + supportDiam / 2, -5]) cylinder(r = 2.5 / 2, h = 10);
    translate([-Xdim / 2 + supportDiam / 2, Ydim / 2 - supportDiam / 2, -5]) cylinder(r = 2.5 / 2, h = 10);

    //Holes to power the lines, need to check the geomety. Fro now visually adjuste
     translate([-Xdim / 4 + 3, -Ydim / 6, -5]) cylinder(r = 4 / 2, h = 10);
     translate([-Xdim / 4 - 9, -Ydim / 6, -5]) cylinder(r = 4 / 2, h = 10);


    //XT60 hole
    translate([0, -Ydim / 2 + XT60Y / 2, 0]) cube([1 * XT60X, 1 * XT60Y, Zdim], center = true);

}

powerPlant();

//translate([0, 0, 0]) topLid();

