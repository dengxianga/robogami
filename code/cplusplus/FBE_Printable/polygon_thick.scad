wallThickness = wallThickness;
resolution = resolution;
points = points;
isAssembled=isAssembled;

//wallThickness = 2;
//resolution = resolution;
//points = [[10, 10], [10, -10], [-10, -10], [-10, 10]];

//translate([0, 0, -wallThickness])

if (isAssembled){
	translate([0,0,-wallThickness/2]);
	linear_extrude(height=wallThickness) 
	polygon(points=points, $fn=resolution);

} else {
	linear_extrude(height=wallThickness) 
	polygon(points=points, $fn=resolution);
}