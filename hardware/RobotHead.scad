//Increase shell thickness
//Change slot for Raspberry 
//Change slot for camera (no need of pan

PIcameraDiam = 8;
PIcameraX = 25;
PIcameraY = 25;
// Raspberry Pi dimensions 85.60mm x 56mm x 21mm 
widht = 148;
depth = 120;
border = widht * 0.2;
T = 6;
2DOF_L = 40;
2DOF_H = 45;
USBH = 20;
USBL = 50 + 6;
FN = 50;
CameraModulePos = [0, 70, 20];
Tolerance= 0.5;
HSupportHoleDiam = 25;

$fn = 150;

//$fs = 0.01; // change me to 0.5 when doing final render for export

phi = 1.618033988749894848204586834;

//-----------------------------------
module HeadShell() {
	difference() {
		resize([widht + border, depth, depth]) sphere(1);
		//Create the inner shell
		resize([widht + border - T, depth - T, depth - T]) sphere(1);
	}
	//Endcaps
	intersection() {
		resize([widht + border, depth, depth ]) sphere(1);
		translate([widht / 2 - 3 / 2, 0, 0]) cube([3, 100, 100], center = true);

	}
	intersection() {
		resize([widht + border, depth, depth]) sphere(1);
		translate([-widht / 2 + 3 / 2, 0, 0]) cube([3, 100, 100], center = true);

	}
}
module H() {
	difference() {
		HeadShell();
		//2DOF slot
		translate(CameraModulePos) rotate([100, 0, 0]) resize([2DOF_L, 2DOF_H, 50]) cylinder(h = 30, r = 100);

		//Two cubes to cut the ends of the head
		translate([widht / 2, -widht / 2, -widht / 2]) cube(widht);
		translate([-3 / 2 * widht, -widht / 2, -widht / 2]) cube(widht);
		//Lower cut
		translate([-widht / 2, -widht / 2, -(widht + 40)]) cube(widht);
		//Holes for buttons
		translate([-widht / 4, -depth / 4, widht / 4]) rotate([45, 30, -45]) ButtonHoleL();
		translate([+widht / 4, -depth / 4, widht / 4]) rotate([45, 30, 0]) ButtonHoleL();

		//Raspberrry slot
		translate([-USBL / 2, -depth / 2, -10]) cube([USBL, 20, USBH]);
 
        translate([-1.1*USBL / 2, -depth / 2, -10]) cube([1.1*USBL, 20, 3 ]);
		//Simulates Raspberry PI dimensions
		//translate([-USBL/2+56,-depth/2,-10])rotate([0,0,90])cube([85.60,56,21]);
        
        //Hole for RGB LED
        translate([0, 0, depth/2]) rotate([0, 0, 0]) cylinder(h=10,r=5/2-Tolerance, center=true);
        
        //Hole for anchoring
		translate([widht / 2, 0, 0]) rotate([90, 180, 90]) cylinder(h=10,r=HSupportHoleDiam/2, center=true);
		//Microserv slot
        translate([-widht / 2 - 10, 0, 0]) rotate([90, 180, 90]) translate([-5.9 - 4.7 / 2, 0, 0]) microservo();

	}
    
    //This is the support fo  rthe raspberry slot
     intersection(){
            HeadShell();
            for (z = [-USBL/2: 2: USBL/2]) 
            {
                translate([z, -depth/2+1, 0])
                cube([0.4,20,USBH], center = true);
            }
            }
		
}
//-----------------------------------

module camera_supportOLD()

{
	translate(CameraModulePos - [0, +5, +6] - [0, 0, T + 12]) rotate([0, 0, 00]) box(13, 24.5, 17, 3);
	difference() {
		translate(CameraModulePos - [0, 0, T + 12]) box(2DOF_L + 2 * T, 2DOF_H, 30, T);

		difference() {
			translate(CameraModulePos - [0, 0, T + 12]) box(2DOF_L + 2 * T, 2DOF_H, 30, T);
			resize([widht + border - T, depth - T, depth - T]) sphere(1, $fn = FN);

		}
	}

}
//-----------------------------------

module box(Xdim, Ydim, Zdim, thickness)
//Creates a box, dimensions are internal
{
	difference() {
		//translate([10,0,-thickness])
		cube([Xdim + thickness, Ydim + thickness, Zdim], center = true);
		translate([0, 0, thickness / 2]) cube([Xdim, Ydim, Zdim], center = true);

		translate([-Xdim / 2, 0, -0.5 * Zdim]) cube([5, Ydim + 2 * thickness, Zdim / 2 + thickness], center = true);
		translate([+Xdim / 2, 0, -0.5 * Zdim]) cube([5, Ydim + 2 * thickness, Zdim / 2 + thickness], center = true);
		translate([+Xdim / 2, 0, 0]) cube([thickness / 2, 0.3 * Ydim, Zdim], center = true);
		translate([-Xdim / 2, 0, 0]) cube([thickness / 2, 0.3 * Ydim, Zdim], center = true);

	}
}

module support() {
	difference() {

		linear_extrude(height = 4, center = true, convexity = 10, twist = 0) polygon(points = [
			[0, 0],
			[0, 16],
			[4, 16]
		]);
		translate([2, 18, 0]) rotate([90, 0, 0]) cylinder(h = 15, d = 1.5);
		translate([0.5, 10, -6]) cube([4, 2, 12], center = false);
	}
}


//-----------------------------------

module CS()
translate(CameraModulePos + [6.25, -25, -32]) {
	box(24.5, 13, 17, T);
	translate([0, 12, 0]) cube([24.5 + T, 5 + T, 17], center = true);
}
//-----------------------------------



module camera_support()

{

	difference() {
		CS();

		difference() {
			CS();
			resize([widht + border - T, depth - T, depth - T]) sphere(1);

		}
	}

}

module ButtonHoleL() {

	cylinder(h = 20, r = 8);
}


module TiltServo() {
	//translate([widht/2-10,0,0])rotate([90,90,90])translate([-35.1,0,0])TiltServo();
	translate([0, -5, 0]) cylinder(h = 20, r = 2.5);
	translate([0, 5, 0]) cylinder(h = 20, r = 2.5);

	translate([50.5, -5, 0]) cylinder(h = 20, r = 2.5);
	translate([50.5, 5, 0]) cylinder(h = 20, r = 2.5);

	translate([35.1, 0, 0]) cylinder(h = 40, r = 5);
	translate([35.1, 0, 0]) cylinder(h = 20, r = 5);

	translate([5, -10.5, 0]) cube([41.50, 21, 20], center = false);
}






module SUB_bearing() {
	cylinder(h = 0.5, r = 25.5 / 2);
	translate([0, 0, 0.5]) cylinder(h = 1.5, r = 30 / 2);
}

module bearingServo() {
	difference() {
		SUB_bearing();
		cylinder(h = 5, r = 2);
	}
}

module bearingNoServo() {
	union() {
		SUB_bearing();
        
		translate([0,0,-8/2])difference(){
            cylinder(h = 12, r = HSupportHoleDiam/2-Tolerance, center=true);
            cylinder(h = 12, r = HSupportHoleDiam/2-Tolerance-4, center=true);
        }
	}
}



module microservo() {

	translate([0, 0, 0]) cylinder(h = 20, r = 1.2);
	translate([27.2, 0, 0]) cylinder(h = 20, r = 1.2);

	translate([5.9 + 4.7 / 2 + 6, 0, 0]) cylinder(h = 40, r = 7 / 2);

	translate([5.9 + 4.7 / 2, 0, 0]) cylinder(h = 40, r = 7);
}

/*difference(){
translate([-5,-10,-0])cube([40,20,2]);
microservo();
}*/


//camera_support();

/*difference(){
translate([-5,-10,0])cube([35,20,1], center=false);
microservo();
}*/

module PiSupport() {
	difference() {

		translate([-USBL / 2, -widht / 2, -USBH / 2 - 5]) cube([USBL, widht, 5]);
		translate([0, 0, -USBH / 2 - 2.5]) cube([USBL - 20, 100, 5], center = true);

		difference() {
			translate([-USBL / 2, -widht / 2, -USBH / 2 - 5]) cube([USBL, widht, 5]);
			resize([widht + border - T, depth - T, depth - T]) sphere(1);
		}
    translate([(USBL - 20)/2+10/2+2.75/2,-widht / 2 +85-58-3,-USBH / 2 -2]){
translate([0, +85-58, -2])cylinder(r=2.75/2-Tolerance, h=10, center=true);
translate([0, +85-58+58, -2])cylinder(r=2.75/2-Tolerance, h=10, center=true);
translate([0-49, +85-58, -2])cylinder(r=2.75/2-Tolerance, h=10, center=true);
translate([0-49, +85-58+58, -2])cylinder(r=2.75/2-Tolerance, h=10, center=true);
}
	}

}




//PiSupport();
//bearingServo();
//bearingNoServo();
 /*               translate([-1, 0, -5])
  cube([USBL,20,2], center = true);
for (z = [-USBL/2: 3: USBL/2]) 
            {
                translate([z, 0, 0])
                cube([0.6,20,10], center = true);
            }
   */


module SUB_PiCamHoles(T) {
// T is the tolerance for holes
        translate([PIcameraX/2-2, 0, PIcameraY/2-2])rotate([90,0,0])cylinder(r=1+T,h=10, center=true);
		translate([-(PIcameraX/2-2), 0, PIcameraY/2-2])rotate([90,0,0])cylinder(r=1+T,h=10, center=true);
		translate([PIcameraX/2-2, 0, (PIcameraY/2-2)-12.5])rotate([90,0,0])cylinder(r=1+T,h=10, center=true);
		translate([-(PIcameraX/2-2), 0, (PIcameraY/2-2)-12.5])rotate([90,0,0])cylinder(r=1+T,h=10, center=true); 
    
}    

module eye(){
resize([widht/4+10,0,+depth/4+15])
intersection()
{
difference() {
		HeadShell();
        
    }

translate(CameraModulePos) rotate([100, 0, 0]) resize([2DOF_L, 2DOF_H, 50]) cylinder(h = 30, r = 100);
}

//difference()

{
intersection()
{
difference() {
		resize([widht + border, depth, depth]) sphere(1);
		//Create the inner shell
		resize([widht + border , depth -20, depth - 20]) sphere(1);
        
    }

translate(CameraModulePos) rotate([100, 0, 0]) resize([2DOF_L, 2DOF_H, 50]) cylinder(h = 30, r = 100);
}
}


}
module PICam()
{
    
{
translate([0,-12,0])cube([PIcameraX + Tolerance, 20, PIcameraY + Tolerance], center = true);
translate([0, 0, -2.5]) 
rotate([90, 0, 0]) cube([PIcameraDiam + Tolerance, 10, PIcameraDiam + Tolerance], center = true);
    translate([0,-12,-PIcameraY/2])cube([PIcameraX - 5, 20, 8], center = true);
//		translate([0,0.9*R,0])
SUB_PiCamHoles(0.3);
}
}

/*difference()
{
eye();
translate([0,widht/4+20,15])rotate([12,0,0])PICam();
}
*/
//PICam();
bearingNoServo();