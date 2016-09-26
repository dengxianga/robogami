//3.6 1.6 2.4 1.6 3.6
length = 15;
wallThickness = wallThickness;
alpha = 60;
r1 = 1.68065*wallThickness;
//r2 = 1.36521*wallThickness;
edgeThickness = 0.315*wallThickness;
jointDist = 0.83815*wallThickness;
D = 0.68065*wallThickness;
S = 1.15792*wallThickness;
X = wallThickness/tan(alpha);
rot = rot;
trans = trans;
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
//edge(r1,length, wallThickness, alpha);

module slanted(length, wallThickness, alpha){
	linear_extrude(height=length)
	polygon([[-S/2,D],[-S/2-X,D],[-X-S/2,D+wallThickness]], $fn=resolution);
	
}

module slantedLeftEdge(length, wallThickness, alpha){
	rotate([90,0,0]) slanted(length, wallThickness, alpha);
}
module slantedRightEdge(length, wallThickness, alpha){
	mirror([-1,0,0]) slantedLeftEdge(length, wallThickness, alpha);
}

module fold(r1,length,wallThickness,alpha){
	difference(){
		translate([0,length,0]) union(){
			slantedLeftEdge(length, wallThickness, alpha);
			slantedRightEdge(length, wallThickness, alpha);
}
		edge(r1,length, wallThickness, alpha);
}
	
}
//slantedLeftEdge(length, wallThickness, alpha);
//slantedRightEdge(length, wallThickness, alpha);
//slanted(length,wallThickness, alpha);

//translate(trans) rotate(rot) fold(r1,length,wallThickness,alpha);

module cropFold(r1,length,wallThickness,alpha,points1,points2){
	difference(){
		translate(trans) rotate(rot+[0,0,90]) translate([0, -length/2, 0])fold(r1,length, wallThickness, alpha);
		union(){
			translate([0,0,-1])
			linear_extrude(height=wallThickness+2) polygon(points = points1, $fn=resolution);
			translate([0,0,-1])
			linear_extrude(height=wallThickness+2) polygon(points = points2, $fn=resolution);
		}
	}
}

cropFold(r1,length,wallThickness,alpha,points1,points2);