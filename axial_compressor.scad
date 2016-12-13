// blade
//$fn=40;
//$fn=100;
$fn=180;
//$fn=360;

axle_radius = 2.56;
axlex_width = 2.05;

out_r = 30;
in_r = 5;
compression = 1.1;
stage_depth = 4;
stage_gap = 0.5;
rotor_fit = 0.5;
stator_fit = 0.2;
case_thickness = 0.7;

module axle_hole(depth) {
    cylinder(r=axle_radius, h=depth, center=true);
}

module axlex_hole(depth) {
    cube([2*axle_radius, axlex_width, depth], center=true);
    cube([axlex_width, 2*axle_radius, depth], center=true);
}

//difference() {cylinder(r=5.5,h=5,center=true);axlex_hole(6);}
//difference() {cylinder(r=5.5,h=5,center=true);axle_hole(6);}

module aerofoil(length, thickness, depth, radius) {
    r = radius-thickness;
    o = sqrt(r*r - depth*depth/4);
    translate([0,-o,depth/2])
    rotate([0,90,0])
    intersection() {
        difference() {
            cylinder(r=radius, h=length);
            cylinder(r=r, h=length);
        }
        translate([-depth/2, radius-5*thickness, 0])
        cube([depth,10*thickness,length]);
    }
}

//aerofoil(30,0.5,4,4);

module raw_blade(length, angle, thickness) {
    sloped_depth = (stage_depth + thickness/cos(angle))/cos(angle);
    translate([0,thickness/(2*cos(angle)),0])
    rotate([angle, 0, 0])
    translate([0,-0.4/cos(angle),-sloped_depth/2])
    aerofoil(length, thickness, sloped_depth, sloped_depth);
}

//raw_blade(30,4,70,0.3);   translate([0,-4,-10]) cube([1,1,20]); translate([0,3,-10]) cube([1,1,20]);

compress_limit = in_r + (out_r - in_r)*2.0/3.0;

// calculate blade limits at a stage boundary
function compression(stage) = 
    pow(compression, stage);

function compress(stage) = 
    let(m = compress_limit)
    let(n = out_r - compress_limit)
    let(c = compression(stage))
    let(x = (c*m - sqrt(c*c*m*m - 2*c*m*n + c*n*n)) / c)
    [
        compress_limit + x,
        compress_limit - 2*x,
    ];

//echo(compress(1));

// calculate change in position across a stage
function compress_pos(stage, pos) =
    let(in_size = compress(stage))
    let(out_size = compress(stage+1))
    let(x = (pos-in_size[0]) / (in_size[0]-in_size[1]))
    out_size[0] + x*(out_size[0]-out_size[1]);

module blade_trim(thickness, stage) {
    cut_cube = [(out_r+1)*2, (out_r+1)*2, thickness + 1];
    cut_trans = [-out_r-1, -out_r-1, stage_depth/2];
    in_size = compress(stage);
    out_size = compress(stage+1);

    translate(cut_trans) cube(cut_cube);
    
    rotate([180,0,0])
    translate(cut_trans) cube(cut_cube);

    difference() {
        cylinder(r=out_r+1, h=stage_depth, center=true);
        cylinder(r1=out_size[0], r2=in_size[0], h=stage_depth, center=true);
    }
}

//blade_trim(4,1,3);

module blade(angle, thickness, stage) {
    difference() {
        raw_blade(out_r, angle, thickness);
        blade_trim(thickness, stage);
    }
}

//blade(4, 75, 0.3, 0);
//mirror([0,1,0]) blade(4, 45, 0.3, 0);

// blade set
module blade_set(num, angle, thickness, support_spacing, support_thickness, stage) {
    m = angle < 0 ? [0,1,0] : [0,0,0];
    a = abs(angle);
    difference() {
        for (b=[0:num-1]) {
            rotate([0,0,360 * b / num])
            mirror(m)
            raw_blade(out_r, a, thickness);
        }
        blade_trim(thickness, stage);
    }

    in_size = compress(stage);
    out_size = compress(stage+1);

    if (support_spacing > 0) {
        in_width = in_size[0] - in_size[1];
        num_supports = floor(in_width / support_spacing);
        spacing = in_width / num_supports;
        for (r=[in_size[0]:-spacing:0]) {
            r_out = compress_pos(stage, r);
            difference() {
                cylinder(r1=r_out, r2=r, h=stage_depth, center=true);
                cylinder(r1=r_out-support_thickness, r2=r-support_thickness, h=stage_depth, center=true);
            }
        }
    }
}

module rotor_blades(num, angle, stage) {
    in_size = compress(stage);
    out_size = compress(stage+1);
    rotate([180,0,0])
    difference() {
        union() {
            blade_set(num, angle, 0.4, in_size[0], 0.5, stage);
            cylinder(r1=out_size[1], r2=in_size[1], h=stage_depth, center=true);
        }
        axlex_hole(stage_depth+1);
    }
}

rotor_blades(17, 45, 10);

function stator_step(stage) =
    min(compress(stage-1)[0], compress(stage)[0]+case_thickness/2);

module stator_blades(num, stage) {
    in_size = compress(stage);
    out_size = compress(stage+1);
    support_thickness = 0.5;
    step = stator_step(stage);
    rotate([180,0,0])
    difference() {
        union() {
            blade_set(num, -25, 0.4, out_r, support_thickness, stage);
            cylinder(r1=out_size[1], r2=in_size[1], h=stage_depth, center=true);
            difference() {
                cylinder(r=step, h=stage_depth/2);
                cylinder(r=in_size[0]-support_thickness, h=stage_depth/2);
            }
        }
        axle_hole(stage_depth+1);
    }
}

//for (i=[1:10]) translate([0,0,i*4]) stator_blades(1, 4, i);

//stator_blades(19, 3);

function case_outer_radius(stage) =
    let(s = compress(stage))
    s[0] + rotor_fit + case_thickness;

module rotor_case(stage) {
    in_radius = case_outer_radius(stage);
    difference() {
        cylinder(r = in_radius, h = stage_gap);
        cylinder(r = in_radius-case_thickness, h = stage_gap);
    }
    out_radius = case_outer_radius(stage+1);    
    translate([0,0,stage_gap])
    difference() {
        cylinder(r1 = in_radius, r2=out_radius, h = stage_depth);
        cylinder(r1 = in_radius-case_thickness, r2=out_radius-case_thickness, h = stage_depth);
    }
}

//rotor_case(0);

module stator_case(stage) {
    // the stage gap
    in_radius = case_outer_radius(stage);
    inner_step = stator_step(stage) + stator_fit;
    outer_step = max(in_radius, inner_step+case_thickness);
    difference() {
        cylinder(r1 = in_radius, r2=outer_step, h = stage_gap);
        cylinder(r1 = in_radius-case_thickness, r2=outer_step-case_thickness, h = stage_gap);
    }
    // stator step
    translate([0,0,stage_gap])
    difference() {
        cylinder(r = outer_step, h = stage_depth/2);
        cylinder(r = inner_step, h = stage_depth/2);
    }
    // stator narrows
    out_radius = case_outer_radius(stage+1);
    mid_radius = (in_radius + out_radius) / 2;
    inner_out = out_radius - case_thickness;
    inner_mid = mid_radius - rotor_fit - case_thickness + stator_fit;
    translate([0,0,stage_gap + stage_depth/2])
    difference() {
        cylinder(r1 = outer_step, r2=out_radius, h = stage_depth/2);
        cylinder(r1 = inner_mid, r2=inner_out, h = stage_depth/2);
    }
}

//stator_case(11);

case_depth = stage_depth + stage_gap;

module nozzle(length, radius, stage) {
    color("blue")
    if (length > 0) {
        in_radius = case_outer_radius(stage);
        cone_depth = (in_radius - radius)*2;
        difference() {
            cylinder(r1 = in_radius, r2=radius, h = cone_depth);
            cylinder(r1 = in_radius - case_thickness, r2=radius - case_thickness, h = cone_depth);
        }
        translate([0,0,cone_depth])
        difference() {
            cylinder(r = radius, h = length);
            cylinder(r = radius - case_thickness, h = length);
        }
    }
}

// nozzle(10, 6, 0);

module case(stages, nozzle_length=0, nozzle_radius=6, start_stage=0) {
    n_stages = stages-start_stage;
    echo("Compression =", compression(n_stages));
    translate([0,0,case_depth*stages])
    rotate([180,0,0]) {
        for (i=[start_stage:stages-1]) {
            translate([0,0,case_depth*i])
            if (i%2 == 0) {
                rotor_case(i);
            } else {
                stator_case(i);
            }
        }
        translate([0,0,case_depth*stages])
        nozzle(nozzle_length, nozzle_radius, stages);
    }
}

//case(12,10,16);

