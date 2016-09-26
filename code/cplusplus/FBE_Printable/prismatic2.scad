wallThickness = wallThickness;
r1 = r1;
r1s = r1s; // shrink amount = 1/2 gap width
length = length;
length1 = length1;
length2 = length2;
delta1 = delta1;
delta2 = delta2;

resolution = resolution;
points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;


rot1 = rot1;
rot2 = rot2;
transL = transL;
transR = transR;
inangle = inangle;

/*
wallThickness=1;
length=25;
length1 = 25;
length2 = 25;
delta2 = 0;
resolution=20;
r1 = 2.5;
r1s = r1 + 0.3;
rot1 = [0,0,180];
rot2 = [0,0,180];
transL = [0,r1s,0];
transR = [0,-r1s,0];
resolution = 100;
*/

minD = 0.3;
rod_diameter=r1;;
ridge_diameter=2*r1s+2*wallThickness;
ridge_width=wallThickness;
clamp_width=5;
widthFactor = 1.0;

cropPrismatic(r1,length,points1,points2, wallThickness);
//ridge();
//clamp();
//rod();

module test() {
			translate([0, clamp_width, rod_diameter/4]) rotate([0,0,0]) rotate([90,0,0]) cube([length, wallThickness/4, 2*clamp_width]);
			translate([0, clamp_width, -rod_diameter/4]) rotate([0,0,0]) rotate([90,0,0]) cube([length, wallThickness/4, 2*clamp_width]);

}

// caps at the end of the joint; enforce joint limits
module ridge() {
	rotate([90,0,0])
	 union() {
		difference() {
			cylinder(d=ridge_diameter,center=true,h=ridge_width,$fn=resolution);
			translate([ridge_diameter/2+rod_diameter/2, 0, 0]) cube([ridge_diameter*widthFactor, ridge_width+100, ridge_diameter+100], center=true);
		}
		linear_extrude(height = ridge_width, center = true) polygon([[0,ridge_diameter/2],[-ridge_diameter,0],[0,-ridge_diameter/2]]);
	}
}

// inner sliding rod
module rod(length, delta) {
	translate([0,delta,0]){
		difference(){
		union(){
		translate([ridge_diameter/4+minD,0,0]) cube([ridge_diameter/2+minD,clamp_width,wallThickness],center=true);
	
		rotate([90,0,0]) linear_extrude(height = clamp_width, center=true) polygon([[0,rod_diameter/2], [ridge_diameter/2, wallThickness/2], [ridge_diameter/2, -wallThickness/2], [0, -rod_diameter/2] ]);

			rotate([90,0,0]) cylinder(d=rod_diameter,center=true,h=clamp_width,$fn=resolution);
			}
			translate([rod_diameter, 0, rod_diameter/4]) rotate([90,0,0]) cube([r1 + rod_diameter, 2*minD, 2*clamp_width], center=true);
			translate([rod_diameter, 0, -rod_diameter/4]) rotate([90,0,0]) cube([r1 + rod_diameter, 2*minD, 2*clamp_width], center=true);
		}
	//translate([10/2+r1s+minD,0,0]) cube([10,length,wallThickness],center=true);
	}
}

// tube for rod to slide inside
module clamp(length, delta) {
	slat_length = rod_diameter/2 - minD;
	translate([0,delta,0]){
	//translate([-10/2-r1s,0,0]) cube([10,length,wallThickness],center=true);
	difference() {
		union() {
			rotate([90,0,0]) cylinder(d=ridge_diameter,center=true,h=length,$fn=resolution);
			translate([0,length/2,0]) rotate([90,0,0]) linear_extrude(height=length) polygon(points=[[0,ridge_diameter/2],[0,-ridge_diameter/2],[-ridge_diameter,0]],paths=[ [0,1,2] ]);
			//translate([-10/2-rod_diameter/2-minD,0,0]) cube([10,length-ridge_width*2-minD*2,wallThickness],center=true);
		}
		rotate([90,0,0]) cylinder(d=rod_diameter+minD*2,center=true,h=length+100,$fn=resolution);
		rotate([90,0,0]) linear_extrude(height = length+100, center=true) polygon([[0,rod_diameter/2+minD], [ridge_diameter/2, wallThickness/2+minD], [ridge_diameter/2, -wallThickness/2-minD], [0, -rod_diameter/2-minD] ]);
		translate([10/2,0,0]) cube([10,length+100,wallThickness+minD*2],center=true);
	}
	translate([0,-length/2+ridge_width/2,0]) ridge();
	translate([0,+length/2-ridge_width/2,0]) ridge();

	translate([ridge_diameter/2 - slat_length/2, 0, rod_diameter/4]) rotate([90,0,0]) cube([slat_length, minD, length], center=true);
	translate([ridge_diameter/2 - slat_length/2, 0, -rod_diameter/4]) rotate([90,0,0]) cube([slat_length, minD, length], center=true);


	translate([ridge_diameter/2 - minD, 0, rod_diameter/4]) rotate([90,0,0]) cube([minD*2, rod_diameter/8, length], center=true);
	translate([ridge_diameter/2 - minD, 0, -rod_diameter/4]) rotate([90,0,0]) cube([minD*2, rod_diameter/8, length], center=true);

	}
		
	
}


module cropPrismatic(r1, length, points1,points2, wallThickness){
	difference(){
		union() {
			translate([0,0, wallThickness/2.0]) translate(transR) rotate(rot1+[0,0,90]) translate([-r1s,0,0]) rod(length1, delta1);
			translate([0,0, wallThickness/2.0]) translate(transL)  rotate(rot2+[0,0,90]) translate([r1s,0,0]) clamp(length2, delta2);
		}
		union(){
			translate([0,0,-2*ridge_diameter])
			linear_extrude(height= 100*wallThickness) polygon(points = points1, $fn=resolution);
			translate([0,0,-2*ridge_diameter])
			linear_extrude(height=100*wallThickness) polygon(points = points2, $fn=resolution);
			linear_extrude(height= 100*wallThickness) polygon(points = points3, $fn=resolution);
			translate([0,0,-2*ridge_diameter])
			linear_extrude(height=100*wallThickness) polygon(points = points4, $fn=resolution);
		}
	}
}