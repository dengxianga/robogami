
wallThickness = wallThickness;
r1s = r1s;
length = length;

points1 = points1;
points2 = points2;
points3 = points3;
points4 = points4;

resolution = resolution;


cropface(r1s,length,points1,points2,wallThickness);


module cropface(r1s,length,points1,points2, wallThickness){
		//translate(trans) rotate(rot+[0,0,90]) 
		linear_extrude(height=wallThickness) 
polygon(points=[ points1[0], points2[1], points2[0], points1[1] ], $fn=resolution);

		linear_extrude(height=wallThickness) 
polygon(points=[ points3[0], points4[1], points4[0], points3[1] ], $fn=resolution);
}

