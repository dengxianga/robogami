wallThickness = wallThickness;
w = wallThickness;
r1 = r1;
r1s = r1s;

isAssembled = isAssembled;

length = length;
chunks = max(floor(length/5.0), 5);
foldZOffset = foldZOffset;
//chunks = chunks;

resolution = resolution;

transL = transL;
transR = transR;
rot1 = rot1;
rot2 = rot2;

length1 = length1;
length2 = length2;
delta1 = delta1;
delta2 = delta2;

inangle = inangle;


/*
foldZOffset = -(0.68065);
wallThickness = 1;
w = wallThickness;
r1 = 1.5;
r1s = 2.8;
length = 20;
length1 = 20;
length2 = 20;
chunks = 10;
resolution = 10;
transL = [0,0,0];
transR = [0,0,0];
rot1 = [0,0,0];
rot2 = [0,0,0];
delta1 = 0;
delta2 = 0;
inangle = 40;
isAssembled = true;
*/

points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;

angle = inangle - ((inangle > 0) ? ceil((inangle-180)/360) :  floor((inangle+180)/360)) * 360;
//angle = inangle > 180 ? 180 : (inangle < -180 ? -180 : inangle);
//angle = ans(angle_signed);


longR = (2*r1 - 0.6)/3.5;
r3 = r1 - longR;
r2 = r3 - 0.3;

cubex = r1/5;
cubey = r1/3;
cubex1 = 0.005;
cubey1 = wallThickness;
delta = length/chunks/5;
cLen = (length-(chunks-1)*delta)/chunks;
beta=0.005; 


//translate(trans) rotate(rot) translate([0, -length/2, wallThickness]) hinge(r1,r2,r3,length,chunks);
//hinge(r1,r2,r3,length,chunks);
if (angle < 0) {
	if (isAssembled) {
		mirror([0,0,1])
		cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
	} else {
		translate([0,0,wallThickness]);
		mirror([0,0,1])
		cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
	}
} else {
	cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
}

//translate([0, -length/2, r1- wallThickness]) hinge(r1,r2,r3,length,chunks);

//left(r1,r2,r3,length,chunks);
//right(r1,r2,r3,length,chunks);

module hinge(r1,r2,r3,length,chunks){
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
		right(r1,r2,r3,length,length1, -delta1, chunks);
	}
	
}

module makeHinge(r1, r2, r3, length, ltotal, d, chunks, offset) {
	L = 2*r1s;
	dshift = d + (ltotal-length)/2.0;
	transAmount = isAssembled ? 0 : 1.75;
	translate([0, -ltotal/2, 0])
	translate([0,0,transAmount])
	rotate([-90,0,0])
	difference(){
		/*translate([-cubex1 - cubex -r1+beta, -cubey1/2,0])
		cube([cubex1, cubey1, length],center = false, $fn=resolution);	*/
		for ( i = [0:chunks-1]){
			if(i%2==offset){
				translate([0,0,i*(cLen+delta)+dshift]){
					union() {
						difference() {
							linear_extrude(height = cLen)
							if (isAssembled) {
								polygon([[0,r1],[-2*r1,w/2],[-2*r1,-w/2],[0,-r1]]);
							} else {
								polygon([[0,r1],[-2*r1,w],[-2*r1,0],[0,-r1]]);
							}
							translate([0,0,-1])
							cylinder(r=r3+0.01,h=100*cLen,center=true, $fn=resolution);
						}
						cylinder(r=r1,h=cLen, $fn=resolution);
					}
				}
			}
		}
		//remove outside length
		translate([-100,-w/2-50,ltotal]) cube([L*2+wallThickness+200, w*2+100,length+100], $fn = resolution);
		translate([-100,-w/2-50,-length-100]) cube([L*2+wallThickness+200, w*2+100,length+100], $fn = resolution);		
	}
}

module left(r1,r2,r3,length,l, d, chunks){
	makeHinge(r1, r2, r3, length, l, d, chunks, 1);
}

module right(r1,r2,r3,length,l, d, chunks){
	mirror([1,0,0])
	makeHinge(r1, r2, r3, length, l, d, chunks, 0);
}

module cropHinge(r1,r2,r3,length,chunks,points1,points2, wallThickness){
	difference(){
		if (isAssembled) {
			hinge(r1,r2,r3,length,chunks);
		} else {
			translate([0, 0, foldZOffset]) 
			hinge(r1,r2,r3,length,chunks);
		}
		union(){
			translate([0,0,-2*r1])
			linear_extrude(height= 100*wallThickness) polygon(points = points1, $fn=resolution);
			translate([0,0,-2*r1])
			linear_extrude(height=100*wallThickness) polygon(points = points2, $fn=resolution);
			translate([0,0,-2*r1])
			linear_extrude(height= 100*wallThickness) polygon(points = points3, $fn=resolution);
			translate([0,0,-2*r1])
			linear_extrude(height=100*wallThickness) polygon(points = points4, $fn=resolution);
		}
	}
}

