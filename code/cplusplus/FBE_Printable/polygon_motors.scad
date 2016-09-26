wallThickness = wallThickness;
resolution = resolution;
points = points;
motorMountLocs=motorMountLocs;
servoMountLocs=servoMountLocs;
isAssembled=isAssembled;
    
/*
resolution = 6;
wallThickness = 10;
points = [[801.96,0.20],[818.92,0.201],[829.53,18.57],[829.53,23.629],[818.92,41.995],[801.962,41.994],[788.159,18.08], [788.157,24.114]];
motorMountLocs=[];
servoMountLocs = [[50, 50], [20, 20]];
isAssembled=true;
*/

difference(){
	if (isAssembled) {
			translate([0,0,-wallThickness/2])
		   linear_extrude(height=wallThickness) polygon(points=points, $fn=resolution, twist=0);
	} else {
		linear_extrude(height=wallThickness) polygon(points=points, $fn=resolution);
	}
	for (point=motorMountLocs){
		stencil(point[0], point[1], point[2]);
	}

	if (!isAssembled) {
		for (point=servoMountLocs){
			holes(point[0], point[1], point[2]);
		}
	}
}


module stencil(x, y, angle){
	 translate([x, y, 0]) rotate([0, 0, angle]) union(){
// Turnigy TGY-1370A
translate([0, 0, 0]) cube([20.6, 9, 100], center=true);
translate([20.6/2.0 + 1.75, 0, 0])  cube([2.3, 2.3, 100], center=true);
translate([-20.6/2.0 - 1.75, 0, 0]) cube([2.3, 2.3, 100], center=true);
// FS90R
//translate([0, 0, 0]) cube([23.5, 12.2, 100], center=true);
//translate([23.5/2.0 + 2.5, 0, 0])  cube([2.3, 2.3, 100], center=true);
//translate([-23.5/2.0 - 2.5, 0, 0]) cube([2.3, 2.3, 100], center=true);
	}
}

module holes(x, y, angle){
	translate([x, y, 0]) rotate([0, 0, angle]) union(){
		translate([0.0, 0.0, 0])  cube([2.3, 2.3, 100], center=true);
		translate([0.0, 12.0, 0]) cube([2.3, 2.3, 100], center=true);
	}
}

