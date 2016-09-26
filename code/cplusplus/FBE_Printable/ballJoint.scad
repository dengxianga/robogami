
wallThickness = wallThickness;
w = wallThickness;
length = length;
chunks = chunks;
trans = trans;
rot = rot;
resolution = resolution;

/*
wallThickness = 1;
w = wallThickness;
r1 = 2.5;
length = 10;
chunks = 4;
resolution = 40;
rot = [0,0,45];
*/

/*******************************************************************/
ballRadius=2.5;
bumpRadius=0;
socketRadius=ballRadius+bumpRadius+1;
minD = 0.6;
h = 2.3;
legRadius=1;
legLength=4*ballRadius;//6.0;
//resolution = 20;

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


module ballJoint() {
////// Ball Side:
union() {
	bumpySphere(0);
	rotate([180,0,0]) cylinder(r=legRadius,h=legLength,$fn=resolution);
	//translate([0,0,-legRadius-10+2]) cube([1,10,10],center=true);
}


////// Socket Side:
difference() {
	union() {
		translate([0,0,-h]) cylinder(r=socketRadius,h=legLength+h,$fn=resolution);
		//translate([0,0,-h+5]) cube([1,socketRadius*2,10],center=true);
	}
	intersection() {
		color([1,0,0]) translate([-100,-100,-h]) cube([200,200,200]);
		color([1,0,0]) bumpySphere(minD);
	}
}
}

translate([0,0,wallThickness/2.0]) translate(trans) rotate(rot+[0,0,90]) rotate([0,90,0]) ballJoint();
/*
for(k=[0 : 1]) {
	rotate([0,0,115+k*360./15.]) translate([ballRadius,0,legLength-h+0.5-0.1]) cube(0.5, center=true);
}
*/
/*******************************************************************/
/*

longR = (2*r1 - 0.6)/3.5;
r3 = 1.0;
r2 = 0.7;
trans = trans;
rot = rot;


cubex = r1/5;
cubey = r1/3;
cubex1 = 0.005;
cubey1 = wallThickness;
delta = 0.3;
cLen = (length-(chunks-1)*delta)/chunks;
beta=0.005; 
resolution = resolution;

cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);

module hinge(r1,r2,r3,length,chunks){
	rotate([-90,0,0])
	union(){
		//translate([-2*r1,0,0]) 
		left(r1,r2,r3,length,chunks);
		right(r1,r2,r3,length,chunks);
	}
	
}

module cropHinge(r1,r2,r3,length,chunks,points1,points2, wallThickness){
	difference(){
		translate(trans) rotate(rot) translate([0, -length/2, wallThickness/2.0]) hinge(r1,r2,r3,length,chunks);
		union(){
			translate([0,0,-2*r1])
			linear_extrude(height= 100*wallThickness) polygon(points = points1, $fn = resolution);
			translate([0,0,-2*r1])
			linear_extrude(height=100*wallThickness) polygon(points = points2, $fn = resolution);
		}
	}
}

*/

