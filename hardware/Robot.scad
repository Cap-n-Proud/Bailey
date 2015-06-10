//Make pasage for cables larger

RobotY=220;
RobotX=80;
RobotZ=5;
BodyHoleR=20;
BodyHoleSpace=55;

plateX=150;
plateZ=3;
plateY=RobotX;
plateanchorX=RobotZ;
plateanchorY=15;
holeradius=3;
holespace=3*holeradius;
borderZ=30;

border=holespace+((plateX)/holespace-floor((plateX)/holespace))/2;
include <TextGenerator.scad>;

module put_pilogo(mt) {
    fn="raspberrypi_logo.dxf";
    // calculate the scale and which offset to use
    
    linear_extrude(height=mt) import(file=fn, scale=scx);
}


module put_Alogo(mt) {
    fn="arduino.dxf";
    // calculate the scale and which offset to use
    
    linear_extrude(height=mt) import(file=fn, scale=scx);
}

module put_OAlogo(mt) {
    fn="ohw-logo.dxf";
    // calculate the scale and which offset to use    
    linear_extrude(height=mt) import(file=fn, scale=scx);
}

module arduinoLogo(h,r) {

	scale([0.9,.7,1])union(){
		difference() {cylinder(h=h,r=r);cylinder(h=h,r=r-.25*r);}
		translate([1.8*r,0,0])difference() {cylinder(h=h,r=r);cylinder(h=h,r=r-.25*r);}

	}
translate([0,0,h/2])cube([0.6*r,0.2*r,h], center=true);
translate([2*r-0.4*r,0,h/2]){cube([0.6*r,0.2*r,h], center=true); rotate([0,0,90])cube([0.6*r,0.2*r,h], center=true);}
}

module plate(holes,baseF,baseB,text) 
{
difference(){linear_extrude(plateZ, center = true, convexity = 0, twist = 0)

	difference(){
		
		union(){square ([plateX, plateY], center=true);
			translate([-plateX/2-plateanchorX,plateY-RobotX/8-(7*(RobotX/8)-plateanchorY),0]) square([plateanchorX,plateanchorY], center = "True");
			
			translate([-plateX/2-plateanchorX,plateanchorY+RobotX/8-(7*(RobotX/8)-plateanchorY),0]) square([plateanchorX,plateanchorY], center = "True");
			
			translate([+plateX/2,plateY-RobotX/8-(7*(RobotX/8)-plateanchorY),0]) square([plateanchorX,plateanchorY], center = "True");
			
			translate([+plateX/2,plateanchorY+RobotX/8-(7*(RobotX/8)-plateanchorY),0]) square([plateanchorX,plateanchorY], center = "True");					
			
		}

//punchholes at the bottom of the plate
if (holes=="True")	{
for (j= [0 : holespace : plateY-1*border ]){
	for ( i = [0 : holespace : plateX-2*border ] ){
			translate([i+border-plateX/2+3,j+border-plateY/2, 0]) circle(holeradius);
	}
	}

}
	}			

//Passage for cables
translate([-plateX/2+5,j+border-plateY/2, 0]) cube([5,0.5*plateY,plateZ], center=true);
translate([plateX/2-5,j+border-plateY/2, 0]) cube([5,0.5*plateY,plateZ], center=true);

}
if (baseF=="True")
translate([-plateX/2,plateY/2-plateZ,-plateZ/2]) cube([plateX,plateZ,borderZ]);
if (baseB=="True")
translate([-plateX/2,-plateY/2,-plateZ/2]) cube([plateX,plateZ,borderZ]);

translate([-plateX/2,-plateY/2,5])rotate(90,[1,0,0]) scale([2,2,1]) drawtext(text);

}

module SensorsPlate(){
	IRLedD=3;
	
	plate("True","True","false","");
	difference(){
		translate([0,-plateY/2+10,borderZ-plateZ/2])
		rotate([180,0,0]) difference(){ 
			scale( v=[1,0.30,1] ) difference() {		
					cylinder( r = plateX/2, h=borderZ, center=false );
					cylinder( r = plateX/2-2*plateZ, h=borderZ, center=false );			
					}
		
				
		 translate([-plateX/2,-plateY/2+10,0])cube([plateX,0.35*plateX/2,plateY/2]);
		}
		//hole for the IR sensor
	 	translate([-plateX/2+12,-plateY/2+2,borderZ/4])rotate([100,0,-30]) translate([6,-3.48,0])cube([6,6.95,10]);
		translate([-8,-plateY/2-10,borderZ/4])rotate([100,0,0]) translate([6,-3.48,0])cube([6,6.95,10]);
		translate([+plateX/2-12,-plateY/2+2,borderZ/4])rotate([100,0,30]) translate([-12,-3.48,0])cube([6,6.95,10]);
	
		//holes for sonars and slots for sensors
		translate([0+28,-plateY/2-8,borderZ/2])rotate([90,0,0]) cylinder(r=16.8/2,h=8);
		translate([0-28,-plateY/2-8,borderZ/2])rotate([90,0,0]) cylinder(r=16.8/2,h=8);
		translate([0,-plateY/2-8,borderZ/2])rotate([90,0,0]) cube([3,1,5]);
		translate([-plateX/2+12,-plateY/2+2,borderZ/2])rotate([90,0,-30]) cube([3,1,5]);
		translate([+plateX/2-12,-plateY/2+2,borderZ/2])rotate([90,0,30]) cube([3,1,5]);

	translate([-plateX/2+12,-plateY/2+2,borderZ/4]) rotate([100,0,-30])cylinder(r=1.5+IRLedD,h=10);
translate([-8,-plateY/2-10,borderZ/4]) rotate([100,0,0])cylinder(r=1.5+IRLedD,h=10);
translate([+plateX/2-12,-plateY/2+2,borderZ/4]) rotate([100,0,30])cylinder(r=1.5+IRLedD,h=10);
	}

//Tubes to focus the IR beams
	translate([-plateX/2,-plateY/2-30,0]) difference (){cylinder(r=1+IRLedD,h=15); cylinder(r=IRLedD,h=15);}
	translate([-plateX/2+15,-plateY/2-30,0]) difference (){cylinder(r=1+IRLedD,h=15); cylinder(r=IRLedD,h=15);}
	translate([-plateX/2+30,-plateY/2-30,0]) difference (){cylinder(r=1+IRLedD,h=15); cylinder(r=IRLedD,h=15);}
//	translate([-8,-plateY/2-10,borderZ/4]) rotate([100,0,0])difference (){cylinder(r=1.2*IRLedD,h=8); cylinder(r=IRLedD,h=10);}
//	translate([+plateX/2-12,-plateY/2+2,borderZ/4]) rotate([100,0,30])difference (){cylinder(r=1.2*IRLedD,h=10); cylinder(r=IRLedD,h=8);}
}

module PowerPlate(){

	difference(){
	plate("True","True","True","Power");
	//fix for batteries
	translate([plateX/2-15,plateY/2+5,borderZ/2])translate([0,0,3])rotate([90,0,0])cylinder(h=10,r=2);	
	translate([-plateX/2+15,plateY/2+5,borderZ/2])translate([0,0,3])rotate([90,0,0])cylinder(h=10,r=2);	
	translate([plateX/2-15,-plateY/2+5,borderZ/2])translate([0,0,3])rotate([90,0,0])cylinder(h=10,r=2);	
	translate([-plateX/2+15,-plateY/2+5,borderZ/2])translate([0,0,3])rotate([90,0,0])cylinder(h=10,r=2);	
		
	}
		
	translate([plateX/2-15,plateY/2+4,borderZ/2])
		difference(){
			sphere(2);rotate([0,90,0])translate([0.7,0.6,0])cylinder(h=10,r=1.1, center=true);}	
	translate([-plateX/2+15,plateY/2+4,borderZ/2])
		difference(){
			sphere(2);rotate([0,90,0])translate([0.7,0.6,0])cylinder(h=10,r=1.1, center=true);}	
	translate([plateX/2-15,-plateY/2-1,borderZ/2])rotate([0,0,180])
		difference(){
			sphere(2);rotate([0,90,0])translate([0.7,0.6,0])cylinder(h=10,r=1.1, center=true);}	
	translate([-plateX/2+15,-plateY/2-1,borderZ/2])rotate([0,0,180])
		difference(){
			sphere(2);rotate([0,90,0])translate([0.7,0.6,0])cylinder(h=10,r=1.1, center=true);}	
	
	
	//mid section
		difference(){
			translate([-plateX/2,0,0]) cube([plateX,plateZ,borderZ]);
			for ( i = [1 : plateX/4 : plateX] ){
				translate([i+(-plateX/2)+borderZ/2,10,borderZ/2]) rotate([90,0,0]) cylinder(20,r=0.8*borderZ/2);
	
			}
		
	
	}


}

module BrainPlate()
{
	plate("False","True","True",""); 
	translate([-plateX/2,-plateY/2,borderZ/2]){
	color("green")rotate([90,0,0])
		translate([120,-2,0])scale(0.3)arduinoLogo(5,30);
		translate([5,-12,0])scale(0.1)put_pilogo(1.5/0.1);
		translate([80,-2,0])scale(0.1)put_OAlogo(1.5/0.1);}
}
	

module MotorBody()
{difference(){
	linear_extrude(RobotZ, center = true, convexity = 0, twist = 0){
		difference() {
//eed to redefine the motorbody as a fixed lenght 			
union(){translate([0,RobotX/2,0]) square([RobotX,220-RobotX-15]);translate([RobotX/2,40,0]) circle(RobotX/2);}
			translate([RobotX/2,40,0]) circle(26/2);
			translate([RobotX/2,40,0]) circle(3);
					
			//Motor holes
			translate([RobotX/2, 40, 0]){
			for ( i = [0 : 4] )
				{
				    rotate( (i * 360 / 4)+45)
				    //datasheet provides distance between holes, we need the radius from the shaft
					 translate([21.92, 0, 0])
				    circle(3);
				}}
			//Modules
			for ( i = [112 :BodyHoleSpace: RobotY] ){
				translate([RobotX/2,i,0]) circle(BodyHoleR);		
		//Slots for plates
				translate([RobotX/8,i-BodyHoleR-5,0]) square([1.15*plateanchorY,1.15*plateZ]);
translate([7*(RobotX/8)-plateanchorY,i-BodyHoleR-5,0]) square([plateanchorY,plateZ]);
				translate([RobotX/2,i-25-plateZ,0]) circle(3);	
				translate([RobotX/8,i,0]) circle(3);		
				translate([7*RobotX/8,i,0]) circle(3);		
			}
		}
		
	}
		//Connection with body extension
			translate([0,220-RobotX+2,0]) cube([RobotX,25,RobotZ/2]);
			translate([15,3*BodyHoleSpace-25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);		
			translate([RobotX-15,3*BodyHoleSpace-25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);	
}
}

module BodyExtension()
{ difference() {
	linear_extrude(RobotZ, center = true, convexity = 0, twist = 0){
		difference() {
			square([RobotX,RobotY]);
			for ( i = [0 :BodyHoleSpace: RobotY] ){
				translate([RobotX/2,i,0]) circle(BodyHoleR);		
				translate([RobotX/8,i-BodyHoleR-5,0]) square([1.15*plateanchorY,1.15*plateZ]);
				translate([7*(RobotX/8)-plateanchorY,i-BodyHoleR-5,0]) square([plateanchorY,plateZ]);
				translate([RobotX/2,i-25-plateZ,0]) circle(3);							translate([RobotX/8,i,0]) circle(3);		
				translate([7*RobotX/8,i,0]) circle(3);		
		
			}

translate([RobotX/8,0,0]) square([1.05*plateanchorY,1.05*plateZ]);
translate([7*(RobotX/8)-plateanchorY,0,0]) square([plateanchorY,plateZ]);
/*
translate([RobotX/8,RobotY-plateanchorY,0]) square([1.05*plateanchorY,1.05*plateZ]);
translate([7*(RobotX/8)-plateanchorY,0,0]) square([plateanchorY,plateZ]);

	*/	}
	
	}
			
//creates attachment for other vertical modules
			translate([0,RobotY-25,0]) cube([RobotX,25,RobotZ/2]);
			translate([15,RobotY-25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);	
			translate([RobotX-15,RobotY-25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);	

			translate([0,0,0]) cube([RobotX,25,RobotZ/2]);
			translate([15,25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);	
			translate([RobotX-15,25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);
			}
			 


}	

$fs = 0.01;

module PCBSupport(PCBHoleR){
	difference (){
		cylinder(h=plateZ+5,r=holeradius-.2);	
		cylinder(h=plateZ+2,r=holeradius/2);	
		cube([10,1,8], center=true);
	}
		translate([0,0,plateZ+5])cylinder(h=6,r=PCBHoleR);	

}

module PCBSupportStud(PCBHoleR){
	difference (){
		cylinder(h=plateZ+12,r=holeradius+.3);	
		cylinder(h=plateZ-5,r=holeradius/2);	
	}
		translate([0,0,plateZ+12])cylinder(h=6,r=PCBHoleR);	

}

//Draw a prism based on a 
//right angled triangle
//l - length of prism
//w - width of triangle
//h - height of triangle
module prism(l, w, h, he) {
	translate([0, l, 0]) rotate( a= [90, 0, 0]) 
	linear_extrude(height = he) polygon(points = [
		[0, 0],
		[w, 0],
		[0, h]
	], paths=[[0,1,2,0]]);
}



//creates attachment for other vertical modules
module base(){
	rotate([90,0,0]){
			translate([0,-(RobotY-25),-RobotX]){
				difference(){
					translate([0,RobotY-25,0]) cube([RobotX,25,RobotZ/2]);
					translate([15,RobotY-25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);	
					translate([RobotX-15,RobotY-25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);	
			}}
			translate([0,0,RobotX]){
				difference(){
				translate([0,0,0]) cube([RobotX,25,RobotZ/2]);
				translate([15,25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);	
				translate([RobotX-15,25/2,-RobotZ/2])	cylinder(h=2*RobotZ,r=3);}}}
difference()
{			translate([0,-RobotX-RobotZ/2,-25/2+7]){
				//cube([RobotX,2*(RobotX+0),10], center=false);
				cube([RobotX,2*(RobotX+RobotZ/4),6], center=false);

			}
	
translate([RobotX/2-31.24/2,0,0]) {
for (j= [0 : 6.35 : plateY-1*border ]){
	
			translate([0,j-plateY+border, -10]) cylinder(h=15,r = 2);
			translate([31.24,j-plateY+border, -10]) cylinder(h=15,r = 2);
			translate([0,-j+plateY-border, -10]) cylinder(h=15,r = 2);
			translate([31.24,-j+plateY-border, -10]) cylinder(h=15,r = 2);

	}	}translate([RobotX/2-31.24/2,0,0]) cube([4,15,20],center=true);
translate([RobotX/2+31.24/2,0,0]) cube([4,15,20],center=true);
}

}

//base();
//PCBSupportStud(1.2);
//permaproto drill holes 1.2mm 




//MotorBody();
//BodyExtension();
//SensorsPlate();

	//rotate([0,0,90])plate("True","True","True", " Motors");

	//translate([RobotX/2,-RobotX,0])rotate([0,0,90]) prism(3,30,25,RobotZ);