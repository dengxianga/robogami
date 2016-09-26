wallThickness = wallThickness;
w = wallThickness;
r1 = r1;
r1s = r1s;

isAssembled = isAssembled;

//NUB CONSTS
nub_length = 2.0;
nub_width = 1.0;
nub_thickness = 1.0;
thresh = 0.8;
//END NUB CONSTS

length = length;
chunks = (length > 25) ? floor(length/5.0) : ((length > 10) ? 5 : 4);
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
r1s = r1 + 0.3;
length = 9.546;
length1 = 9.546;
length2 = 9.5862;
chunks = 4;
resolution = 20;
transL = [5,-8.198,0];
transR = [4.94, -11.80, 0];
rot1 = [0,0,180];
rot2 = [0,0,-180];
delta1 = -0.95138;
delta2 = 0;
inangle = 60;
*/

points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;

nubangle = (inangle > 180)? 360 - inangle : inangle;

angle = inangle - ((inangle > 0) ? ceil((inangle-180)/360) :  floor((inangle+180)/360)) * 360;
//angle = inangle > 180 ? 180 : (inangle < -180 ? -180 : inangle);
//angle = abs(angle_signed);

longR = (2*r1 - 0.6)/3.5;
r3 = r1 - longR;
r2 = r3 - 0.2;

angleRad = r1s + 5;
interAngle = 0.4;
axisAngle = interAngle + 0.5;

cubex = r1/5;
cubey = r1/3;
cubex1 = 0.005;
cubey1 = wallThickness;
delta = length/chunks/5;
cLen = (length-(chunks-1)*delta)/chunks;
beta=0.005; 
axle_width = delta+min(0.3,cLen/2);

if (angle < 0) {
	if (isAssembled) {
		mirror([0,0,1])
		cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
	} else {
		translate([0,0,wallThickness])
		mirror([0,0,1])
		cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
	}
} else {
	cropHinge(r1,r2,r3,length,chunks,points1,points2,wallThickness);
}

//rotate([0,90,0]) cylinder(r=10, h =delta,center=true);

// left and right hinges
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

// basic hinge constructor
module makeHinge(r1, r2, r3, length, ltotal, d, chunks, offset) {
	L = 2*r1s;
	dshift = d + (ltotal-length)/2.0;
	transAmount = isAssembled ? 0 : 1.75;
	translate([0, -ltotal/2, transAmount]) rotate([-90,0,0])
	difference(){
		/*translate([-cubex1 - cubex -r1+beta, -cubey1/2,0])
		cube([cubex1, cubey1, length],center = false, $fn=resolution);	*/
		for ( i = [0:chunks-1]){
			if(i%2==offset){
				translate([0,0,i*(cLen+delta)+dshift]){
					union() {
						difference() {
							union(){
								linear_extrude(height = cLen)
								if (isAssembled) {
								polygon([[0,r1],[-2*r1,w/2],[-2*r1,-w/2],[0,-r1]], $fn=resolution);
								} else {
								polygon([[0,r1],[-2*r1,w],[-2*r1,0],[0,-r1]], $fn=resolution);
								}
								cylinder(r=r1,h=cLen, $fn=resolution);
                                //rotate([0, 0, nubangle]) translate([r1 - thresh, -wallThickness, cLen/2]) nubs();
                                //rotate([0, 0, nubangle]) translate([r1 - thresh, wallThickness, cLen/2]) nubs();
							}
						
							cylinder(r=r3+0.01,h=cLen,center=true, $fn =resolution);
                            
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

module makeAngle(pos) {
	translate(pos+[0,0,wallThickness/2]) {
		difference() {
			rotate([90,0,0]) cylinder(r=angleRad, h = wallThickness);
			rotate([90,0,0]) translate([0,0.75,-wallThickness/2]) cylinder(r=r1s+0.5, h = wallThickness*2);
			translate([0,-wallThickness/2,-angleRad]) cube([angleRad*2, 2*wallThickness, angleRad*2],center=true);
			rotate([0,-180+abs(angle),0]) translate([0,-wallThickness/2,-angleRad]) cube([angleRad*2, 2*wallThickness, angleRad*2],center=true);
		}		
	}
}

module left(r1,r2,r3,length,l, d, chunks){
	makeHinge(r1, r2, r3, length, l, d, chunks, 1);
	// TODO: fix values for nonzero delta
	//makeAngle([0,length/2+d,0]);
	//makeAngle([0,length/2+d-2*wallThickness-2*interAngle,0]);
	//makeAngle([0,-length/2+d+wallThickness*2+interAngle,0]);
}

module right(r1,r2,r3,length,l, d, chunks){
	mirror([1,0,0]) {
		makeHinge(r1, r2, r3, length, l, d, chunks, 0);
		//makeAngle([0,length/2+d-wallThickness-interAngle,0]);
		//makeAngle([0,-length/2+d+wallThickness,0]);
		//makeAngle([0,-length/2+d+3*wallThickness+2*interAngle,0]);
	}
}

// crop out areas that intersect with input faces
module cropHinge(r1,r2,r3,length,chunks,points1,points2, wallThickness){
	difference(){
		if (isAssembled) {
			hinge(r1,r2,r3,length,chunks);
		} else {
			translate([0, 0, foldZOffset])
			hinge(r1,r2,r3,length,chunks);
		}
		/*
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
		*/
	}
}

module nubs(){
     rotate([0, 0, 270]) linear_extrude(height = nub_thickness, center = true, convexity = 10, slices = 20, scale = 1.0) polygon( points=[[nub_width/2.0,0],[0.0, nub_length],[-nub_width/2.0,0.0]] );
    
}

