
// Rover 2 approximate printable concept model
// Units: mm. Designed as a starting point, not an exact reverse-engineering of the infographic.
$fn = 36;
part = "assembly";

module rounded_box(x,y,z,r=3) {
    // Simple robust rectangular solids; rounded corners are intentionally omitted for printability.
    cube([x,y,z], center=true);
}
module hole_z(d,h) { cylinder(d=d,h=h,center=true); }
module chassis() {
    difference() {
        translate([0,0,32]) cube([142,202,8],center=true);
        for (x=[-55,55], y=[-82,-52,52,82])
            translate([x,y,32]) cylinder(d=3.4,h=12,center=true);
        for (x=[-35,35], y=[-72,72])
            translate([x,y,32]) cylinder(d=4.2,h=12,center=true);
    }
}
module top_plate() {
    difference() {
        translate([0,0,49]) cube([132,174,5],center=true);
        for (x=[-48,48],y=[-72,-45,45,72])
            translate([x,y,49]) cylinder(d=3.4,h=8,center=true);
        for (x=[-20,20],y=[-20,20])
            translate([x,y,49]) cylinder(d=4.2,h=8,center=true);
    }
}
module side_panel() {
    difference() {
        translate([0,0,40]) cube([5,170,25],center=true);
        for (y=[-65,-30,30,65])
            translate([0,y,40]) rotate([0,90,0]) cylinder(d=3.2,h=8,center=true);
    }
}
module wheel() {
    difference() {
        rotate([0,90,0]) cylinder(d=54,h=22,center=true);
        rotate([0,90,0]) cylinder(d=6.2,h=26,center=true);
        // shallow tread grooves around circumference
        for (a=[0:30:330])
            rotate([a,0,0]) translate([0,0,26]) cube([26,2.4,3],center=true);
    }
}
module wheel_mount() {
    difference() {
        translate([0,0,0]) cube([30,30,22],center=true);
        translate([0,0,0]) cube([32,12,12],center=true);
        for (y=[-9,9])
            translate([0,y,0]) rotate([0,90,0]) cylinder(d=3.4,h=35,center=true);
        translate([0,0,0]) rotate([90,0,0]) cylinder(d=6.4,h=35,center=true);
    }
}
module front_sensor_mount() {
    difference() {
        translate([0,0,0]) cube([68,16,28],center=true);
        translate([0,0,2]) rotate([90,0,0]) cylinder(d=17,h=22,center=true);
        translate([-22,0,2]) rotate([90,0,0]) cylinder(d=3.2,h=22,center=true);
        translate([22,0,2]) rotate([90,0,0]) cylinder(d=3.2,h=22,center=true);
        translate([0,0,-9]) cube([48,18,5],center=true);
    }
}
module electronics_holder() {
    difference() {
        translate([0,0,0]) cube([72,62,18],center=true);
        translate([0,0,3]) cube([64,54,18],center=true);
        for (x=[-28,28],y=[-23,23])
            translate([x,y,0]) cylinder(d=3.2,h=25,center=true);
    }
}
module arm_base() {
    difference() {
        union() {
            translate([0,0,5]) cube([52,52,10],center=true);
            translate([0,0,20]) cylinder(d=38,h=20,center=true);
            translate([0,0,34]) cube([38,38,10],center=true);
        }
        for (x=[-18,18],y=[-18,18])
            translate([x,y,5]) cylinder(d=3.4,h=18,center=true);
        translate([0,0,20]) cylinder(d=12,h=45,center=true);
    }
}
module servo_block() {
    difference() {
        cube([40,20,40],center=true);
        cube([32,12,32],center=true);
        for (x=[-15,15],z=[-15,15])
            translate([x,0,z]) rotate([90,0,0]) cylinder(d=3.2,h=25,center=true);
    }
}
module arm_link() {
    difference() {
        union() {
            hull() {
                translate([0,-22,0]) rotate([90,0,0]) cylinder(d=24,h=12,center=true);
                translate([0,22,0]) rotate([90,0,0]) cylinder(d=20,h=12,center=true);
            }
            translate([0,0,0]) cube([18,42,12],center=true);
        }
        for (y=[-22,22])
            translate([0,y,0]) rotate([90,0,0]) cylinder(d=5.2,h=18,center=true);
    }
}
module gripper_palm() {
    difference() {
        cube([30,24,12],center=true);
        for (x=[-10,10])
            translate([x,0,0]) cylinder(d=3.2,h=16,center=true);
        translate([0,0,0]) cylinder(d=5.2,h=16,center=true);
    }
}
module gripper_finger() {
    difference() {
        union() {
            hull() {
                translate([0,-13,0]) cylinder(d=10,h=8,center=true);
                translate([0,13,0]) cylinder(d=6,h=8,center=true);
            }
            translate([0,0,0]) cube([8,26,8],center=true);
        }
        translate([0,-13,0]) cylinder(d=3.2,h=12,center=true);
    }
}
module arm_bracket() {
    difference() {
        union() {
            cube([34,22,12],center=true);
            translate([0,0,13]) cube([22,12,22],center=true);
        }
        for (x=[-12,12]) translate([x,0,0]) cylinder(d=3.2,h=18,center=true);
        translate([0,0,13]) rotate([90,0,0]) cylinder(d=5.2,h=18,center=true);
    }
}
module parts_assembly() {
    chassis();
    top_plate();
    // side panels
    translate([-69,0,40]) side_panel();
    translate([69,0,40]) side_panel();
    // four wheel mounts and wheels
    for (x=[-78,78], y=[-72,72]) {
        translate([x,y,22]) wheel_mount();
        translate([x+(x<0?-13:13),y,22]) wheel();
    }
    translate([0,101,42]) front_sensor_mount();
    translate([0,-22,58]) electronics_holder();
    translate([0,-20,65]) arm_base();
    // simplified articulated arm posed upright/forward
    translate([0,-20,91]) servo_block();
    translate([0,0,116]) rotate([25,0,0]) arm_link();
    translate([0,35,135]) rotate([-20,0,0]) servo_block();
    translate([0,52,155]) rotate([45,0,0]) arm_link();
    translate([0,74,171]) rotate([90,0,0]) gripper_palm();
    translate([-11,88,171]) rotate([0,0,10]) gripper_finger();
    translate([11,88,171]) rotate([0,0,-10]) gripper_finger();
}
if (part=="chassis") chassis();
else if (part=="top_plate") top_plate();
else if (part=="side_panel") side_panel();
else if (part=="wheel") wheel();
else if (part=="wheel_mount") wheel_mount();
else if (part=="front_sensor_mount") front_sensor_mount();
else if (part=="electronics_holder") electronics_holder();
else if (part=="arm_base") arm_base();
else if (part=="servo_block") servo_block();
else if (part=="arm_link") arm_link();
else if (part=="gripper_palm") gripper_palm();
else if (part=="gripper_finger") gripper_finger();
else if (part=="arm_bracket") arm_bracket();
else parts_assembly();
