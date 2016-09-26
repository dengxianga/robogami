//3.6 1.6 2.4 1.6 3.6
length1 = length1;
length2 = length2;
wallThickness = wallThickness;
chunks = 15;
alpha = 60;
transL = transL;
rot1 = rot1;
rot2 = rot2;
transR = transR;
points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;


wallThickness=1.5;
length=20;
length1 = 20;
length2 = 20;
resolution=20;
r1 = 2.5;
r1s = r1 + 0.3;
rot1 = [0,0,0];
rot2 = [0,0,0];
transL = [0,0,0];
transR = [0,0,0];


module tab(l, w, a){
rotate([0,90,0])
linear_extrude(height=l) polygon ([[-w,0], [0,0], [-w,w/tan(a/2)]]);	}

module cropTab(length1, length2,wallThickness, alpha){
difference(){
		union(){
			translate(transR) rotate(rot1+[0,0,90])translate([-length1/2,-r1s, 0]) tab(length1, wallThickness, alpha);
			translate(transL) rotate(rot2+[0,0,90])translate([length2/2,r1s,, 0]) rotate([0,0,180]) tab(length2, wallThickness, alpha);
		}
		union(){
			linear_extrude(height=wallThickness+1) polygon(points = points1);
			linear_extrude(height=wallThickness+1) polygon(points = points2);
			linear_extrude(height=wallThickness+1) polygon(points = points3);
			linear_extrude(height=wallThickness+1) polygon(points = points4);
		}
	}
}

cropTab(length1, length2,wallThickness, alpha);

