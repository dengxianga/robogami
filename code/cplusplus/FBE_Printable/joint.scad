//3.6 1.6 2.4 1.6 3.6
length = length;
wallThickness = wallThickness;
r1 = 1.68065*wallThickness;
//r2 = 1.36521*wallThickness;
edgeThickness = 0.315*wallThickness;
jointDist = 0.83815*wallThickness;
alpha = 60;
trans = trans;
rot = rot;
points1 = points1;
points2 = points2;
resolution = resolution;

module edge(r1, length, wallThickness, alpha){
	rotate([0,0,90]) difference() {
		difference() {
		 rotate([0,90,0]) cylinder(r=r1, h=length, $fn=50);	
		 rotate([0,90,0]) cylinder(r=r1-edgeThickness, h=length, $fn=resolution);	
		}
	
		translate([-5/2, 0, 0]) union() {
			rotate([-90+(alpha-10),0,0]) cube([length+5, 2*r1, 2*r1], $fn=resolution);
			rotate([180-(alpha-10),0,0]) cube([length+5, 2*r1, 2*r1], $fn=resolution);
			translate([0,-r1, -r1]) cube([length+5, 2*r1, r1], $fn=resolution);
		}
	}
}

module cropEdge(r1,length,wallThickness, alpha,points1,points2){
	difference(){
		translate(trans) rotate(rot+[0,0,90]) translate([0, -length/2, 0]) edge(r1,length, wallThickness, alpha);
		union(){
			translate([0,0,-1])
			linear_extrude(height=wallThickness+2) polygon(points = points1, $fn=resolution);
			translate([0,0,-1])
			linear_extrude(height=wallThickness+2) polygon(points = points2, $fn=resolution);
		}
	}
}

cropEdge(r1,length,wallThickness, alpha,points1,points2);

