
wallThickness = wallThickness;
w = wallThickness;
r1 = r1;
r1s = r1s; // shrink amount = 1/2 gap width

length = length;
length1 = length1;
length2 = length2;
delta1 = delta1;
delta2 = delta2;

chunks = max(floor(length/6), 3);
//chunks = chunks;

//trans = trans;
//rot = rot;
rot1 = rot1;
rot2 = rot2;
transL = transL;
transR = transR;

inangle = inangle;
isAssembled = isAssembled;



//axle_width = 1.2; //get this to be a function of the values?
cubex = r1/5;
cubey = r1/3;
cubex1 = 0.005;
cubey1 = wallThickness;
delta = 0.4;
cLen = (length-(chunks-1)*delta)/chunks;
beta=0.005; 

// todo: make .5 a function of length
axle_width = delta+.4;
 
resolution = resolution;


/*
wallThickness=1;
length=50;
length1 = 50;
length2 = 50;
delta1 = 0;//-1.38;
delta2 = 0;
resolution=20;
r1 = 2.5;
r1s = r1 + 0.3;
mingap = 0.4;
rot1 = [0,0,0];
rot2 = [0,0,0];
transL = [0,-r1s,0];//[25.517,-2.8,0];
transR = [0,r1s,0];//[26.90,2.8,0];
inangle = -20;
*/

longR = (2*r1 - 0.6)/3.5;
r3 = r1 - longR;
r2 = r3 - 0.3;

points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;

angle = inangle - ((inangle > 0) ? ceil((inangle-180)/360) :  floor((inangle+180)/360)) * 360;
//angle = inangle > 180 ? 180 : (inangle < -180 ? -180 : inangle);
//angle = abs(angle_signed);



//translate(trans) rotate(rot) translate([0, -length/2, wallThickness]) //hinge(r1,r2,r3,length,chunks);
//hinge(r1,r2,r3,length,chunks);

if (angle < 0) {
	mirror([0,0,1])
	cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
} else {
	cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
}

//translate([0, -length/2, r1- wallThickness]) //hinge(r1,r2,r3,length,chunks);

//left(r1,r2,r3,length,length1,delta1,chunks);
//right(r1,r2,r3,length,chunks);

module hinge(r1,r2,r3,length,chunks){
	translate([0, 0, wallThickness/2]) 
	union(){
		//translate([-2*r1,0,0]) 
		if (isAssembled) {
			translate(transR) rotate(rot1+[0,0,90]) translate([-r1s,0,0])
			rotate([0,180+abs(angle),0])
			left(r1,r2,r3,length,length2, delta2, chunks);
		} else {
			translate(transL) rotate(rot2+[0,0,90]) translate([r1s,0,0])
			left(r1,r2,r3,length,length2, delta2, chunks);
		}
		translate(transR) rotate(rot1+[0,0,90]) translate([-r1s,0,0])
		right(r1,r2,r3,length,length1, -delta1,chunks);
		//translate([3, -1, 0]) translate(transR) translate([0, 0, cLen/2]) cube([2, 2, 2*(cLen + delta)]);
	}
	
}

module makeHinge(r1, r2, r3, length, ltotal, d, chunks, offset) {
	L = 2*r1s;
	dshift = d + (ltotal-length)/2.0;
	translate([0, -ltotal/2, 0]) rotate([-90,0,0]) {
	union(){
		/*translate([-cubex1 - cubex -r1+beta, -cubey1/2,0])
		cube([cubex1, cubey1, length],center = false, $fn = resolution);	*/
		difference() {
			for ( i = [0:chunks-1]){
				if(i%2==offset){
					translate([0,0,i*(cLen+delta)+dshift]){
						union(){
							difference(){
								union() {
								linear_extrude(height = cLen)
								polygon([[0,r1],[-2*r1,w],[-2*r1,0],[0,-r1]], $fn = resolution);
								cylinder(r=r1,h=cLen, $fn = resolution);
								}
								
								cylinder(r=r3,h=cLen, center=true,$fn = resolution);
							}
							translate([0, 0, cLen/2]) cylinder(r=r2, h = cLen/2 + axle_width, $fn = resolution, center=false);
						}	
					}
				}
			}
			//remove outside length
			translate([-100,-w/2-50,ltotal]) cube([L*2+wallThickness+200, w*2+100,length+100], $fn = resolution);
			translate([-100,-w/2-50,-length-100]) cube([L*2+wallThickness+200, w*2+100,length+100], $fn = resolution);
		}
	}	
	}
}

module left(r1,r2,r3,length,ltotal,d,chunks){
	makeHinge(r1, r2, r3, length, ltotal, d, chunks, 1);
}

module right(r1,r2,r3,length,ltotal,d,chunks){
	mirror([1,0,0])
	makeHinge(r1, r2, r3, length, ltotal, d, chunks, 0);
}

module cropHinge(r1,r2,r3,length,chunks,points1,points2, wallThickness){
	difference(){
		hinge(r1,r2,r3,length,chunks);
		union(){
			translate([0,0,-2*r1])
			linear_extrude(height= 100*wallThickness) polygon(points = points1, $fn = resolution);
			translate([0,0,-2*r1])
			linear_extrude(height=100*wallThickness) polygon(points = points2, $fn = resolution);
			translate([0,0,-2*r1])
			linear_extrude(height=100*wallThickness) polygon(points = points3, $fn = resolution);
			translate([0,0,-2*r1])
			linear_extrude(height=100*wallThickness) polygon(points = points4, $fn = resolution);
			
		}
	}
}

