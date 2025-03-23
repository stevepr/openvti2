


//Mounting Pegs for Arduino Mega

// one complete peg

module peg(){

$fn=60;
    
difference(){
cylinder(7,d=8);
cylinder(8,d=4.2);
}

rotate_extrude()

    translate([2.5,0,0]){
    intersection(){
    square(5);
    difference(){
    square(5,true);
    translate([2.5,2.5])circle(2.5);    
            }  
        }
    }
}

module peg_placement(){

translate([2,16,0])
peg();

translate([2,91,0])
peg();

translate([51,15,0])
peg();

translate([51,97,0])
peg();

translate([17.3,67,0])
peg();
    
translate([45.2,67,0])
peg();

}

translate([5,0.5,0])
color("red")peg_placement();

$fn=60;
module case1(){
    minkowski(){
    cube([63,115,3]);
    cylinder(60,d=5);
    }
}

// Inner case for cutout

module case_inner(){
    minkowski(){
    cube([59,111,3]);
    cylinder(62,d=4);
    }
}

module case2(){
difference(){
case1();
translate([2,2,3])
color("blue")case_inner();
    }
}

//posts for lid

module case3(){
case2();
difference(){
translate([3,3,55])
color("red")cylinder(6,d=8);

translate([3,3,55])
color("green")cylinder(15,d=4);
}

difference(){
translate([3,112,55])
color("red")cylinder(6,d=8);

translate([3,112,55])
color("green")cylinder(15,d=4);
}

difference(){
translate([60,112,55])
color("red")cylinder(6,d=8);

translate([60,112,55])
color("green")cylinder(15,d=4);
}

difference(){
translate([60,3,55])
color("red")cylinder(6,d=8);

translate([60,3,55])
color("green")cylinder(15,d=4);
    }
}
// Lid

module lid1(){
translate([0,0,70]){
minkowski(){
    cube([63,115,2]);
    cylinder(2,d=5);
        }
    }
}
//translate([0,0,0])
//lid1();

module inner_lid(){
translate([2,2,68]){
minkowski(){
    cube([58.7,110.7,1]);
    cylinder(1,d=4);
        }
    }
}

//color("yellow")inner_lid();

module total_lid(){
    lid1();
    inner_lid();
}

module combined_lid(){
    lid1();
    inner_lid();
}

//punch holes in lid

module lid_with_holes(){
difference(){
combined_lid();

translate([3,3,63])
color("green")cylinder(15,d=4);

translate([3,112,63])
color("green")cylinder(15,d=4);

translate([60,112,63])
color("green")cylinder(15,d=4);

translate([60,3,63])
color("green")cylinder(15,d=4);
    }
}

translate([0,0,5])
rotate([0,180,0])

//lid with restraint

translate([10,0,-70])
lid_with_holes();

translate([-30,73.5,6.7])
color("blue")lid_restraint();

translate([-30,68.5,6.7])
color("blue")lid_restraint2();

module holes(){
//USB Hole
translate([16,-47,-0.6])
color("blue")cube([18,13,12.5
    ]);

//Power Hole
translate([16,-15.5,-0.6])
color("red")cube([19,10,12]);

//LED Holes
$fn=60;
rotate([0,90,0])
translate([-15,-29,15])
color("red")cylinder(20,d1=6,d2=5,true);

rotate([0,90,0])
translate([-15,-20,15])
color("red")cylinder(20,d1=6,d2=5,true);
    
    
//RCA Holes

rotate([0,90,0])
translate([-37.5,-45,10])
color("green")cylinder(30,d1=14,d2=14,true);

rotate([0,90,0])
translate([-37.5,-14,10])
color("green")cylinder(30,d1=14,d2=14,true);

// GPS SMA Hole

rotate([0,90,0])
translate([-40,-28,-99])
color("green")cylinder(10,d=6.8);

}

module case4(){
difference(){
case3();
translate([60,20,10])
rotate([0,0,270])
holes();
    }
}

case4();

// positioning nubs
/*
//right 
translate([57.6,88,1.8])
rotate([0,10,0])
color("red")cube([5,3,15]);

//left
translate([0.2,88,1.3])
rotate([0,-10,0])
color("red")cube([5,3,15]);

//rear

translate([29,101.6,2.3])
rotate([-10,0,0])
color("red")cube([3,13,16]);
*/
//clicker

module clicker(){
translate([0,0,-1])
cube([15,5,3],true);

rotate([10,0,0])
translate([-5,,0,8])
cube([5,2,20],true);

rotate([10,0,0])
translate([5,,0,8])
cube([5,2,20],true);


module click_stop(){

difference(){
translate([0,-4,12])
rotate([75,0,0])    
cube([15,8,4],true);
    
 translate([0,-4.6,9.3])
    rotate([10,0,0])
color("blue")cube([17,5,4],true);   

translate([0,0.4,15])
    rotate([12,0,0])
color("blue")cube([17,5,4],true);   
}
}
//translate([0,0,5])
//click_stop();
}

//translate([30.5,108,3])
//color("green")clicker();


// lid restraint


module lid_restraint(){

$fn=60;
    
cylinder(39,d=7);

rotate_extrude()

    translate([3,0,0]){
    intersection(){
    square(10);
    difference(){
    square(10,true);
    translate([6,7])circle(6);    
            }  
        }
    }
}

module lid_restraint2(){

$fn=60;

translate([0,0,27])
cylinder(12,d=7);
    
cylinder(23,d=7);

rotate_extrude()

    translate([3,0,0]){
    intersection(){
    square(10);
    difference(){
    square(10,true);
    translate([6,7])circle(6);    
            }  
        }
    }
}

translate([-29,21,6.5])
color("red")cube([10,10,20]);

translate([-63,21,6.5])
color("red")cube([10,10,18]);






