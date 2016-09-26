
wallThickness = wallThickness;
w = wallThickness;
length = length;
resolution = resolution;
r1s = r1s;

/*
wallThickness = 1;
w = wallThickness;
r1 = 2.5;
r1s = 4.0;
length = 10;
chunks = 4;
//resolution = 40;
//angle = -90+180;min(rot[2]/2,60);
//angle = (rot[2] - 0) * (60 - -60) / (180 - 0) + -60;
rot1 = [0,0,90];
rot2 = [0,0,90];
transL=[8,0,0];
transR=[-8,0,0];
delta1= 0;
delta2 = 0;
resolution=50;
*/

angle = 0;

/*******************************************************************/
ballRadius=2.4;
bumpRadius=0;
socketRadius=ballRadius+bumpRadius+1.2;
minD = 0.5;
h = 2.3;
legRadius=1;
legLength=4*ballRadius;//6.0;
//resolution = 20;

ballJoint();

module bumpySphere(offset) {
	sphere(r=ballRadius+offset, center=true, $fn=resolution);
	
	for(i = [[-10,6],[-50,4],[-90,1]]) {
		for(j = [0 : i[1]]) {
	   		rotate([0,i[0],j*360/i[1]])
			translate([ballRadius,0,0])
		    sphere(r=bumpRadius+offset, center=true, $fn=resolution*0.5);
		}
	}
}

module ball() {
union() {
	bumpySphere(0);
	rotate([180,0,0]) cylinder(r=legRadius,h=legLength,$fn=resolution);
	//translate([0,0,-legRadius-10+2]) cube([1,10,10],center=true);
}
}

module socket() {
/*
difference() {
	union() {
		translate([0,0,-h]) cylinder(r=socketRadius,h=legLength+h,$fn=resolution);
		//translate([0,0,-h+5]) cube([1,socketRadius*2,10],center=true);
	}
	intersection() {
		//color([1,0,0]) translate([-100,-100,-h]) cube([200,200,200]);
		color([1,0,0]) bumpySphere(minD);
	}
}
*/
difference() {
	union() {
		cylinder(r=socketRadius-0.5, h=legLength, $fn=resolution);
		sphere(r=socketRadius, center=true, $fn=resolution);
		//translate([0,0,5]) cube([1,10,10],center=true);
	}
	union() {
		bumpySphere(minD);
		rotate([0,180-angle,0]) translate([0,0,-h]) cylinder(r=ballRadius-minD, h=legLength, $fn=resolution);
		rotate([0,180,0]) translate([0,0,-h]) cylinder(r=legRadius+0.3, h=legLength, $fn=resolution);
		
		rotate([0, -2*angle/3, 0] ) translate([0, 0, -legLength-h]) cylinder(h = legLength+.3, r1 = 0, r2 = ballRadius, $fn=resolution);
		//rotate([0,180-angle/3,0]) translate([0,0,-h]) cylinder(r=legRadius+0.3, h=legLength, $fn=resolution);
		//rotate([0,180-2*angle/3,0]) translate([0,0,-h]) cylinder(r=legRadius+0.3, h=legLength, $fn=resolution);
		
	}
}


}

module ballJoint() {
translate([0,0,wallThickness/2.0]) union() {
	////// Ball Side:
	translate(transR) rotate(rot1+[0,0,-90]) translate([r1s,delta1,0]) rotate([0,90,0]) ball();

	////// Socket Side:
	translate(transL) rotate(rot2+[0,0,-90]) translate([-r1s,delta2,0]) rotate([0,90,0])	socket();
}
}


