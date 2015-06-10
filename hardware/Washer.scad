module washer(d1,d2,z){
	difference(){
		cylinder(h=z,r=d1/2);
		cylinder(h=z,r=d2/2);
	}

}

$fs = 0.01;
washer(10,3.3,1);
//cylinder(h=1,r=10);