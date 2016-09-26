//3.6 1.6 2.4 1.6 3.6
length = length;
length1 = length1;
length2 = length2;
delta1 = delta1;
delta2 = delta2;
wallThickness = wallThickness;
//chunks = chunks;
chunks = max(floor(length/3.0), 5);
transL = transL;
rot1 = rot1;
rot2 = rot2;
transR = transR;
points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;
resolution = resolution;
inangle = inangle;
isAssembled = isAssembled;


/*
length = 20;
length1 = 20;
length2 = 20;
chunks = max(floor(length/3.0), 5);
delta1 = 0;
delta2 = 0;
wallThickness = 1;
rot1 = [0,0,0];
rot2 = [0,0,0];
transL = [0,5,0];
transR = [0,0,0];
inangle = 30;
resolution = 20;
*/


module outTeeth(l, ltotal, d, w, a, c){
	doveAngle = 20;
	friction = 0.00;
	del = 0.1;
	len = 2*w ;	
	s = len*tan(doveAngle);
	extra = 2*w*tan(doveAngle);
	wid = l/c + s;
	translate([0, 2*w,0])
	mirror([0,1,0])
	for (i=[0:c-1]) {
			if (i%2==1) {
				translate([(l/c)*i + d,0, 0])
				difference(){
					// the out part
					rotate([0,90,0])
					translate([0, 0, -friction])
					linear_extrude(height=l/c + s) 
					polygon ([[-2*w,0], [w,0], [0,2*w], [-w,2*w]],$fn=resolution);
					// the dove cut
					union(){
						translate([0, 0, -4])
						linear_extrude(w + 8) 
						polygon ([[0,0],[-del,0], [-del,len], 
									[len*tan(doveAngle),len]],$fn=resolution);
						translate([0, 0, -4])
						linear_extrude(w + 8) 
						polygon ([[wid,0], [wid +del,0], [wid + del,len ], 
									[wid -len*tan(doveAngle),len ]],$fn=resolution);
					}
				}
			}
		}
	
}

module inTeeth(l, ltotal, d, w, a, c){
doveAngle = 20;
	friction = 0.0;
	del = 0.1;
	len = 2*w ;
	s = len*tan(doveAngle);
	wid = l/c + s;
		translate([0, 2*w,0])
		mirror([0,1,0])
		for (i=[0:c-1]) {
			if (i%2==0) {
				translate([(l/c)*i + d + s,0, 0])
				translate([-s, 0, 0])
				difference(){
					// the out part
					rotate([0,90,0])
					linear_extrude(height=l/c + s) 
					polygon ([[-2*w,0], [w,0], [0,2*w], [-w,2*w]],$fn=resolution);
					// the dove cut
					union(){
						translate([0, 0, -4])
						linear_extrude(w + 8) 
						polygon ([[0,0],[-del,0], [-del,len], 
									[len*tan(doveAngle),len]],$fn=resolution);
						translate([0, 0, -4])
						linear_extrude(w + 8) 
						polygon ([[wid,0], [wid +del,0], [wid + del,len], 
									[wid -len*tan(doveAngle),len]],$fn=resolution);
					}
				}
			}
		}
	
}


module cropTab(length,length1, length2, delta1, delta2, wallThickness, alpha, chunks, transL, transR, rot ){
difference(){
		union(){
		//rotate([alpha, 0, 0])
	//translate([0, - wallThickness/tan(alpha/2), 0])
		
			//translate([0, - wallThickness/tan(alpha/2), -wallThickness])
			mirror([0,0,1])translate([0,0,- wallThickness])
			translate(transR) rotate([0,0,-90]) rotate(rot1)
			translate([-length1/2, 0, 0]) 
			outTeeth(length, length1, delta1, wallThickness, alpha, chunks);
			
			
	//translate([0, + wallThickness/tan(alpha/2) + 8, 0])
			//rotate([-alpha, 0, 0])
			if (isAssembled) {
				mirror([0,0,1])
				translate(transR) rotate([0,0,-90]) rotate(rot1)
				rotate([0,inangle,0])
				translate([0,0, -wallThickness])
				translate([-length2/2, 0, 0]) mirror([0,1,0]) 
				inTeeth(length, length2, delta2, wallThickness, alpha, chunks);
			} else {
				mirror([0,0,1])
				translate(transL) rotate([0,0,-90]) rotate(rot2)
				translate([0,0, -wallThickness])
				translate([-length2/2, 0, 0]) mirror([0,1,0]) 
				inTeeth(length, length2, delta2, wallThickness, alpha, chunks);
			}
		}
		union(){
			translate([0,0,-1])linear_extrude(height=wallThickness+2) polygon(points = points1,$fn=resolution);
			translate([0,0,-1])linear_extrude(height=wallThickness+2) polygon(points = points2,$fn=resolution);
			translate([0,0,-1])linear_extrude(height=wallThickness+2) polygon(points = points3,$fn=resolution);
			translate([0,0,-1])linear_extrude(height=wallThickness+2) polygon(points = points4,$fn=resolution);
		}
	}
}

cropTab(length, length1, length2, delta1, delta2, wallThickness, alpha, chunks,  transL, transR, rot);
