x = x;
y = y;
angle = angle;


module stencil(){
     translate([x, y, 0]) rotate([0, 0, angle]) union(){
translate([0, 0, 0]) cube([22, 10, 100], center=true);
translate([13.5, 0, 0])  cube([2, 2, 100], center=true);
translate([-13.5, 0, 0]) cube([2, 2, 100], center=true);
    }
}

stencil();