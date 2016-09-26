 //3.6 1.6 2.4 1.6 3.6



space = 0.3;
length = length;
length1 = length1;
length2 = length2;
delta1 = delta1;
delta2 = delta2;
wallThickness = wallThickness;
chunks = max(floor(length/3.0), 5);
//chunks = chunks;

transL = transL;
transR = transR;
inangle = inangle;

rot1 = rot1;
rot2 = rot2;
points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;
resolution = resolution;

isAssembled = isAssembled;


/*
length = 25;
length1 = 25;
length2 = 25;
chunks = max(floor(length/3.0), 5);
delta1 = 0;
delta2 = 0;
wallThickness = 1;
rot1 = [0,0,0];
rot2 = [0,0,0];
transL = [0,-3,0];
transR = [0,3,0];
inangle = 90;
resolution = 100;
isAssembled = true;
*/

angle_signed = inangle - ((inangle > 0) ? ceil((inangle-180)/360) :  floor((inangle+180)/360)) * 360;

//angle_signed = 90+pow(-1,floor(inangle/180))*(inangle-floor(inangle/180)*180-90);
//angle_signed = inangle > 180 ? 180 : (inangle < -180 ? -180 : inangle);
angle = abs(angle_signed);

//leftTeeth(length, length1, delta1, wallThickness, chunks, angle);

if (angle_signed < 0) {
	if (isAssembled) {
		mirror([0,0,1])
		cropTeeth(length,length1, length2, delta1, delta2,wallThickness,chunks, angle);
	} else {
		translate([0,0,wallThickness/2])
		mirror([0,0,1])
		cropTeeth(length,length1, length2, delta1, delta2,wallThickness,chunks, angle);
	}
} else {
	cropTeeth(length,length1, length2, delta1, delta2,wallThickness,chunks, angle);
}

module leftTeeth(length, ltotal, d, wallThickness, chunks, angle) {
	makeTeeth(length, ltotal, d, wallThickness, chunks, angle, 0);
}

module rightTeeth(length, ltotal, d, wallThickness, chunks, angle) {
	mirror([-1,0,0])
	makeTeeth(length, ltotal, d, wallThickness, chunks, angle, 1);
}

module makeTeeth(length, ltotal, d, wallThickness,chunks, angle, offset) {

space = 0.3;
P = wallThickness*2.4;
w = wallThickness;
L = w/tan(angle/2)- space/2; 
dshift = d + (ltotal-length)/2.0;
difference(){
	translate([0,0,wallThickness])
	mirror([0,0,1])
	union(){
		translate([-1*(L - w/2 ),0,0])
		difference(){				
			for (i=[0:chunks-1]) {
				if (i%2==offset) {
					translate([0,(length/chunks)*i + dshift,0]) {
						union() {
							cube([L -w/2 , length/chunks, wallThickness], $fn = resolution);
							translate([0,0,wallThickness/2.0]) 
							rotate([-90,0,0]) 
							difference(){	
								cylinder(r=wallThickness*1/2, h=length/chunks, $fn = resolution);
								//translate([ 0, -wallThickness/2.0, 0 ])
								//cube( w , length/chunks , w);
							}
						}
					}
				}
			}
			//remove outside length
			translate([-100,ltotal,-w/2]) cube([L*2+wallThickness+200, length, w*2], $fn = resolution);
			translate([-100,-length,-w/2]) cube([L*2+wallThickness+200, length, w*2], $fn = resolution);
		}
		base(ltotal, wallThickness, angle);	
	}
	union(){
		translate([0,-2,-P+wallThickness/4])
			rotate([-90,0,0]) 
			cylinder(r=P, h=ltotal+4, $fn = resolution);
		translate([0,-2, P+3*wallThickness/4])
			rotate([-90,0,0]) 
			cylinder(r=P, h=ltotal+4, $fn = resolution);
	}
}
}

module base(length, wallThickness, alpha){
	x = wallThickness/tan(alpha);
	translate([0,length,0])
	rotate([90,0,0])
	linear_extrude(height = length)
	polygon(points = [[0,0],[-x,0],[0,wallThickness]], $fn = resolution);
}

module doveTeeth(length,length1, length2, delta1, delta2, wallThickness, chunks, angle){

			L = 3/2*wallThickness;
			alpha = 60;
			//translate(transL) rotate([0,0,90]) rotate(rot) 
			//translate([0, 0, -wallThickness]) 
			rotate([0,alpha, 0])  
			translate([0, -length2/2, 0]) 
			leftTeeth(length, length2, delta2, wallThickness, chunks, angle);
			rotate([0,-alpha, 0])  
			translate([0, -length1/2, 0]) 
			rightTeeth(length, length1, delta1,wallThickness, chunks,angle);

}

module cropTeeth(length,length1, length2, delta1, delta2, wallThickness, chunks, alpha){
	L = wallThickness/tan(angle/2)- space/2; 
	difference(){
		union(){
			if (isAssembled) {
				translate(transR) rotate([0,0,-90]) rotate(rot1)
				rotate([0,angle,0])
				translate([L-wallThickness/2, -length2/2, -wallThickness/2])
				leftTeeth(length, length2, delta2, wallThickness, chunks, alpha);
				translate(transR) rotate([0,0,-90]) rotate(rot1)
				translate([-L+wallThickness/2, -length1/2, -wallThickness/2])
				rightTeeth(length, length1, delta1,wallThickness, chunks, alpha);
			} else {
				translate(transL) rotate([0,0,-90]) rotate(rot2) translate([0, -length2/2, -wallThickness/4]) leftTeeth(length, length2, delta2, wallThickness, chunks, alpha);
				translate(transR) rotate([0,0,-90]) rotate(rot1) translate([0, -length1/2, -wallThickness/4]) rightTeeth(length, length1, delta1,wallThickness, chunks, alpha);
			}
		}
		union(){					
			translate([0,0,-1])linear_extrude(height=10*wallThickness) polygon(points = points1, $fn = resolution);
			translate([0,0,-1])linear_extrude(height=10*wallThickness) polygon(points = points2, $fn = resolution);
			translate([0,0,-1])linear_extrude(height=10*wallThickness) polygon(points = points3, $fn = resolution);
			translate([0,0,-1])linear_extrude(height=10*wallThickness) polygon(points = points4, $fn = resolution);		
		}
	}
}
