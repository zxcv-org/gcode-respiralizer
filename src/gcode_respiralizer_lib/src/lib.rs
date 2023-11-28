use kiddo::{self, SquaredEuclidean};
use rand::Rng;
use regex::Regex;
use std::cell::RefCell;
use std::collections::{BTreeSet, HashSet, VecDeque};
use std::f32::consts::PI;
use std::fmt::Write as fmt_Write;
use std::fs;
use std::io::{self, BufRead, Write};
use std::ops;
use std::rc::Rc;
use std::time::Instant;

// ideas:
//   * if gt 90 degrees from original segment (and part of zig zag zig), then skip?
//   * if new segment is shorter than 0.050 ish, skip?
//   * reverse/reverse detection?, eliminate zag in zig zag zig
//   * eliminate a short segment if between two longer segments (same direction or not), and elimination doesn't mess up 0.0125 gcode resolution threshold, leaving a single vertex instead of a segment
//   ** when looking for point 2, constrain to find point 2 in opposite z direction from point 1, because nearly pointless to get a 2nd point in the same z direction - we want to terp if we can terp (maybe help with surface texture)
//     * still need at least 2 iterations, unless we want to basically drop point 1
//   * constrain choice of fine layers first; if 
//   * calculate an overall conversion factor between distance and extrusion amount, and keep a remainder after round-tripping to/from ascii / calculate with f64 per total distance travelled - doesn't work near build plate though...
//   * point 1 finds which z direction is trying to mess with us, point 2 finds the closest fine perimeter point in the z direction that isn't trying to mess with us, point 3 finds a replacement for point 1 with z closer to query point
//   * force closest two z values on opposite sides, but only if within threshold distance of each other... meh
//   * selecting point 1 and point 2 separately and interpolating can lead to pulling into corners when iterated, or not getting close enough to surface when not iterated - another reason to think about constraining layer above and layer below and then not iterating beyond that, or selecting point 2 with respect to point 1.
//   * force closest two z values on opposite sides, but only if within threshold distance of the query point
//     * keep z
//     * two closest points to query point may not be closest to each other; try it anyway
//     * nothing explicitly prevents going backwards; just rails
//   * leave bottom layers alone; maybe detectable in gcode, or maybe via env variables
// One problem at a time, based on worst problem(s) observed on prints:
//   1. The itty bitty segments seem to freak out klipper and/or the printer leading to printer pausing momentarily without any obvious reason why (unless missed some backwards ones)
//   2. Slight resampling / interference / moire pattern in shallow angle reflections
//   which is worse? 1
//   how to fix?
//     * check for zig/zag/zig direction reversals, emit as extra gcode comments (this will determine if the pausing is caused by reversals, or just a short segment, maybe at slightly different angle, or degenerately-calculated angle)

// This way we can switch to f64 easily for comparing memory and performance.
// Also possibly in future it could make sense to "newtype" these, if it seems
// like it'd help avoid mistakes.
pub use f32 as Mm;
pub use f32 as Factor;
pub use f32 as Radians;
pub use u32 as PointIndex;

// TODO: Make this a command-line argument, with this as the default, or with this scaled by the
// auto-detected coarse vase slicing layer height.
//
// This needs to be roughly big enough to capture the coarse vase-mode slicing points back onto the
// fine slicing perimeters, but not so big that we'll think the intro line is part of the model
// for example. This threshold limit does not apply for the first found closest point, which we'll
// use without any "other" point if this threshold is exceeded by any other-z point on the fine
// slicing perimeters. So it's not a disaster for this threshold to be exceeded.
//
// This needs to be small enough to permit "detaching" from the other z perimeter if the closest
// point perimeter is just doing it's own thing that's totally different than any nearby perimeter.
//
// The 2.0 is to allow for every-other-perimeter stacking (hypothetical clever model). The SQRT_2
// is to account for up to 45 degree mis-alignment of the next perimeter up or down when that
// perimeter is essentially a full layer height away. The 0.3 is roughly the max typical layer
// height with a 0.4mm nozzle, but really this part should be auto-detected.
const OTHER_Z_MAX_DISTANCE_DEFAULT : Mm = 2.0 * SQRT_2 * 0.3;
const REFINEMENT_MAX_ITERATIONS : u32 = 12;
const REFINEMENT_GOOD_ENOUGH_TO_STOP_UPDATE_DISTANCE : Mm = 0.001;

// To have 3 segments to catch zig-zag-zig, or to remove a tiny pointless segment between to other
// segments (while evaluating the reasonable-ness of doing so), we need 4 points.
const MAX_G1_BUFFERED : usize = 4;

// TODO: Get this from std when/if available there, and/or switch to const version of sqrt if/when
// that's a thing.
const FRAC_1_SQRT_3 : Mm = 0.5773502691896257;
const SQRT_2 : Mm = 1.4142135623730951;

type KdTree = kiddo::float::kdtree::KdTree<Mm, PointIndex, 3, 32, u32>;

// We split move segments implied by the gcode into sub-segments so that the kiddo::KdTree can help
// us find long segments that happen to pass near the query point. Making this value too small will
// use more memory without speeding things up. Making this value too large won't save much memory
// and will slow things down. An ok-ish value is ~4x the length of a typical gcode segment when
// some curves are being approximated, to have most gcode segments only need one sub-segment.
const MAX_SUBSEGMENT_LENGTH : Mm = 2.0;
// The KdTree complains if too many items have the "same position on one axis". TBD if we can just
// fudge the locations in the KdTree slightly, and expand our search by this amount as well, to
// avoid having to increase the buckets and presumably waste memory. Also, it's not immediately
// clear from the error message if this would complain about a bunch of points in a perfect plane
// or not, since those would have the same position on one axis - the bucket size being large
// enough for all the points in a flat axis-aligned vertical surface of a big model would mean the
// bucket size would have to be pretty huge. TBD if this fudge kills performance somehow, or leads
// to degenerate cases in KdTree code. The points we end up actually using for generating output
// points don't have this fudge radius applied.
const KD_TREE_FUDGE_RADIUS : Mm = 0.1;
// We fudge per component, since we don't care if we have cube-shaped fudge as long as we know the
// max distance to the corner of that fudge cube. So if an edge of the cube is this long, the 3D
// diagonal corner to far corner across a cube should be KD_TREE_FUDGE_RADIUS.
const KD_TREE_FUDGE_PER_COMPONENT : Mm = KD_TREE_FUDGE_RADIUS * FRAC_1_SQRT_3;

// output_filename can be the same string as coarse_reference_filename, so we
// can't overwrite output file until we're sure we have complete output
pub fn process_files(
    fine_reference_filename: &str,
    coarse_reference_filename: &str,
    output_filename: &str,
) {
    let _tmp_output_filename = output_filename.to_owned() + ".tmp";

    // We don't need to remember the gcode for the fine layers, only the paths.
    println!("reading fine layers...");
    let before_read_layers = Instant::now();
    let fine_layers = read_fine_layers(file_lines(fine_reference_filename).expect("file_lines failed"));
    let read_layers_elapsed = before_read_layers.elapsed();
    println!("done reading fine layers - points: {} segment_runs: {} kd_tree points: {} elapsed: {:.2?}", fine_layers.points.len(), fine_layers.segment_runs.len(), fine_layers.kd_tree.size(), read_layers_elapsed);

    let coarse_gcode_lines = file_lines(coarse_reference_filename).expect("file_lines failed");
    let buf_writer = io::BufWriter::with_capacity(8 * 1024, fs::File::create(output_filename).expect("fs::File::open failed"));

    let before_generate_output = Instant::now();
    generate_output(fine_layers, coarse_gcode_lines, buf_writer);
    let generate_output_elapsed = before_generate_output.elapsed();
    println!("done generating output - elapsed: {:.2?}", generate_output_elapsed);
}

fn file_lines(filename: &str) -> io::Result<io::Lines<io::BufReader<std::fs::File>>> {
    Ok(io::BufReader::with_capacity(64 * 1024, fs::File::open(filename).expect("fs::File::open failed")).lines())
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct Point {
    x: Mm,
    y: Mm,
    z: Mm,
}

#[derive(Debug, Default, Clone, Copy)]
struct Vec3 {
    x: Mm,
    y: Mm,
    z: Mm,
}

impl Vec3 {
    fn norm(&self) -> Mm {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
    fn dot(&self, rhs: Vec3) -> Mm {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
    fn cross(&self, rhs: Vec3) -> Vec3 {
        let a = *self;
        let b = rhs;
        Vec3 {x: a.y*b.z - a.z*b.y, y: a.x*b.z - a.z*b.x, z: a.x*b.y - a.y*b.x}
    }
}

impl ops::Add<Vec3> for Vec3 {
    type Output = Vec3;

    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3 { x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z }
    }
}

impl ops::Add<Vec3> for Point {
    type Output = Point;

    fn add(self, rhs: Vec3) -> Point {
        Point { x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z }
    }
}

impl ops::Sub<Vec3> for Vec3 {
    type Output = Vec3;

    fn sub(self, rhs: Vec3) -> Vec3 {
        Vec3 { x : self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z, }
    }
}

impl ops::Sub<Vec3> for Point {
    type Output = Point;

    fn sub(self, rhs: Vec3) -> Point {
        Point { x : self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z, }
    }
}

impl ops::Sub<Point> for Point {
    type Output = Vec3;

    fn sub(self, rhs: Point) -> Vec3 {
        Vec3 { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z, }
    }
}

impl ops::AddAssign<Vec3> for Vec3 {
    fn add_assign(&mut self, other: Vec3) {
        *self = Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        };
    }
}

impl ops::AddAssign<Vec3> for Point {
    fn add_assign(&mut self, other: Vec3) {
        *self = Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        };
    }
}

impl ops::SubAssign<Vec3> for Vec3 {
    fn sub_assign(&mut self, other: Vec3) {
        *self = Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        };
    }
}

impl ops::SubAssign<Vec3> for Point {
    fn sub_assign(&mut self, other: Vec3) {
        *self = Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        };
    }
}

impl ops::Div<Factor> for Vec3 {
    type Output = Vec3;

    fn div(self, rhs: Factor) -> Vec3 {
        Vec3{ x: self.x / rhs, y: self.y / rhs, z: self.z / rhs, }
    }
}

impl ops::DivAssign<Factor> for Vec3 {
    fn div_assign(&mut self, rhs: Factor) {
        *self = Vec3{x: self.x / rhs, y: self.y / rhs, z: self.z / rhs, }
    }
}

impl ops::Mul<Factor> for Vec3 {
    type Output = Vec3;

    fn mul(self, rhs: Factor) -> Vec3 {
        Vec3{ x: self.x * rhs, y: self.y * rhs, z: self.z * rhs, }
    }
}

impl ops::MulAssign<Factor> for Vec3 {
    fn mul_assign(&mut self, rhs: Factor) {
        *self = Vec3{x: self.x * rhs, y: self.y * rhs, z: self.z * rhs, }
    }
}

impl ops::Neg for Vec3 {
    type Output = Vec3;

    fn neg(self) -> Vec3 {
        Vec3{ x: -self.x, y: -self.y, z: -self.z, }
    }
}

#[derive(Debug, Default, PartialEq, Eq)]
struct SegmentRun {
    // eg. 0 means point 0 starts segment 0 of this SegmentRun
    start: PointIndex,
    // eg. 10 means the last segment starts at point index 9 and ends at point
    // index 10
    end: PointIndex,
}

impl PartialOrd for SegmentRun {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return Some(self.cmp(other));
    }
}

impl Ord for SegmentRun {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // We never make any that partially overlap or abut, so no need to handle those.
        assert!(self.start == other.start && self.end == other.end || self.end < other.start || self.start > other.end);
        if self.end < other.start {
            return std::cmp::Ordering::Less;
        }
        if self.start > other.end {
            return std::cmp::Ordering::Greater;
        }
        assert!(self.start == other.start && self.end == other.end);
        return std::cmp::Ordering::Equal;
    }
}

#[derive(Debug)]
pub struct Layers {
    // These are at least all the fine gcode points that have an extrusion to the point or away
    // from the point. Points which have no adjacent extrusion can be omitted since we don't need
    // any travel/travel points. We rely on all of retract/detract/extra detract to be 0.0 in
    // slicer settings. We detect and fail on negative extrusion so we'll notice retracts if user
    // forgets to disable.
    points: Vec<Point>,

    // Exactly all the contiguous runs of extruding moves, including the intro line.
    segment_runs: BTreeSet<SegmentRun>,

    // We use KdTree instead of ImmutableKdTree for these reasons:
    //   * No need to build a big slice with more entries than G1(s) in the fine
    //     gcode, in addition to the tree itself.
    //   * ImmutableKdTree can't have multiple entries that map to the same
    //     index, unlike KdTree which can.
    //
    // Each entry in kd_tree is the end point of a sub-segment with max length
    // MAX_SUBSEGMENT_LENGTH, with index into points (above). The kd_tree has no
    // filtering by layer aside from just the z distances increasing as we get
    // further away in z from a layer.
    kd_tree: KdTree,
}

#[derive(Debug, Default)]
struct GcodeState {
    // initially assumed 0,0,0 (ignoring homing which is assumed)
    loc: Point,
    // initially assumed false; verify true before adding a segment
    is_abs_xyz: bool,
    // initially assumed false; verify true before adding a segment
    is_rel_e: bool,
}

fn kd_fudge() -> Mm {
    rand::thread_rng().gen::<Mm>() * KD_TREE_FUDGE_PER_COMPONENT
}

pub struct G1LineContext<'a> {
    line: &'a str,
    old_loc: Point,
    g: &'a GcodeState,
    has_explicit_z: bool,
    opt_extrude: Option<Mm>,
    opt_f: Option<Mm>,
    opt_comment: Option<&'a str>,
}

pub struct GcodeLineHandler<'a> {
    pub g1_handler : &'a mut dyn FnMut(G1LineContext),
    pub default_handler : &'a mut dyn FnMut(&str),
}

pub fn process_lines(gcode_lines: io::Lines<io::BufReader<std::fs::File>>, line_handler: &mut GcodeLineHandler) {
    let g90_abs = Regex::new(r"^G90[^0-9a-zA-Z].*$").unwrap();
    let g91_rel = Regex::new(r"^G91[^0-9a-zA-Z].*$").unwrap();
    let m83_rel_e = Regex::new(r"^M83[^0-9a-zA-Z].*$").unwrap();
    // strip off the G1 and the comment
    let g1_move = Regex::new(r"^G1 (?<xyzef>[^;]+)(?<comment>;.*)?$").unwrap();
    let g1_sub_x = Regex::new(r"^.*X(?<X>[-0-9\.]+).*$").unwrap();
    let g1_sub_y = Regex::new(r"^.*Y(?<Y>[-0-9\.]+).*$").unwrap();
    let g1_sub_z = Regex::new(r"^.*Z(?<Z>[-0-9\.]+).*$").unwrap();
    let g1_sub_e = Regex::new(r"^.*E(?<E>[-0-9\.]+).*$").unwrap();
    let g1_sub_f = Regex::new(r"^.*F(?<F>[0-9\.]+).*$").unwrap();

    let mut line_number = 0u64;
    let mut g1_count = 0u64;
    let mut g = GcodeState::default();

    for line_result in gcode_lines {
        let line = line_result.expect("line fail");
        line_number += 1;
        if let Some(g1_captures) = g1_move.captures(&line) {
            // println!("G1 gcode: {}", line);
            let old_loc = g.loc.clone();
            let xyzef = g1_captures.name("xyzef").unwrap().as_str();
            let opt_comment = match g1_captures.name("comment") {
                Some(m) => Some(m.as_str()),
                None => None,
            };
            let opt_extrude : Option<Mm>;
            let opt_f : Option<Mm>;
            let has_explicit_z : bool;
            if let Some(x_captures) = g1_sub_x.captures(xyzef) {
                // println!("got x");
                g.loc.x = x_captures.name("X").unwrap().as_str().parse().unwrap();
            }
            if let Some(y_captures) = g1_sub_y.captures(xyzef) {
                // println!("got y");
                g.loc.y = y_captures.name("Y").unwrap().as_str().parse().unwrap();
            }
            if let Some(z_captures) = g1_sub_z.captures(xyzef) {
                // println!("got z");
                g.loc.z = z_captures.name("Z").unwrap().as_str().parse().unwrap();
                has_explicit_z = true;
            } else {
                has_explicit_z = false;
            }
            if let Some(e_captures) = g1_sub_e.captures(xyzef) {
                // println!("got e");
                let extrude = e_captures.name("E").unwrap().as_str().parse().unwrap();
                if extrude < 0.0 {
                    // This is only detecting retraction, but go ahead and complain about detract
                    // and extra detract as well since we don't have any detection for those but
                    // they might potentially also need to be turned off (not thinking too hard
                    // here about exactly how the settings work in prusaslicer since the UI could
                    // change anyway).
                    panic!("Please turn off retract, detract, and any extra detract, and re-generate fine vase-esque non-vase slicing, then try generating coarse slicing again.");
                }
                opt_extrude = Some(extrude);
            } else {
                opt_extrude = None;
            }
            if let Some(f_captures) = g1_sub_f.captures(xyzef) {
                opt_f = Some(f_captures.name("F").unwrap().as_str().parse().unwrap());
            } else {
                opt_f = None;
            }
            g1_count += 1;

            if !g.is_abs_xyz {
                panic!("G1 when !g.is_abs_xyz");
            }
            if !g.is_rel_e {
                panic!("G1 when !g.is_rel_e");
            }

            let g1_line_context = G1LineContext{line: &line, old_loc, g: &g, has_explicit_z, opt_extrude, opt_f, opt_comment};
            (line_handler.g1_handler)(g1_line_context);
            continue;
        }
        (line_handler.default_handler)(line.as_str());
        if !g.is_rel_e && m83_rel_e.is_match(&line) {
            g.is_rel_e = true;
            // println!("line: {} is_rel_e = true", line_number);
            continue;
        }
        if (!g.is_abs_xyz || g.is_rel_e) && g90_abs.is_match(&line) {
            g.is_abs_xyz = true;
            g.is_rel_e = false;
            // println!("line: {} is_abs_xyz = true; is_rel_e = false", line_number);
            continue;
        }
        if (g.is_abs_xyz || g.is_rel_e) && g91_rel.is_match(&line) {
            g.is_abs_xyz = false;
            g.is_rel_e = false;
            // println!("line: {} is_abs_xyz = false; is_rel_e = false", line_number);
            continue;
        }
    }

    println!("line_count: {}", line_number);
    println!("g1_count: {}", g1_count);
}

pub fn read_fine_layers(gcode_lines: io::Lines<io::BufReader<std::fs::File>>) -> Layers {
    let mut points: Vec<Point> = vec![];
    let mut segment_runs: BTreeSet<SegmentRun> = BTreeSet::new();
    let mut kd_tree: KdTree = KdTree::new();

    let mut cur_segment_run : Option<SegmentRun> = None;

    let mut g1_handler = |c: G1LineContext| {
        if !c.opt_extrude.is_some() {
            // println!("no extrude");
            if cur_segment_run.is_some() {
                segment_runs.insert(cur_segment_run.take().unwrap());
            }
            return;
        }

        match cur_segment_run.as_mut() {
            Some(run) => {
                run.end += 1;
            },
            None => {
                assert!(segment_runs.is_empty() || (segment_runs.last().unwrap().end as usize) < points.len());
                cur_segment_run = Some(SegmentRun { start: (points.len() as u32), end: ((points.len() + 1) as u32), });
                points.push(c.old_loc.clone());
            }
        };
        points.push(c.g.loc.clone());

        // We ensure that each point on the segment is within MAX_SUBSEGMENT_LENGTH of a point
        // added to kd_tree.

        let segment_delta = c.g.loc - c.old_loc;
        // dbg!(segment_delta);
        let segment_length = segment_delta.norm();
        // dbg!(segment_length);
        let segment_direction_unit = segment_delta / segment_length;
        // dbg!(segment_direction_unit);

        let mut i = 0u32;
        // the segment goes from point[point_index] to point[point_index + 1]
        let point_index = points.len() - 2;
        let segment_start_point = points[point_index];
        let segment_end_point = points[point_index + 1];
        if segment_start_point.z != segment_end_point.z {
            dbg!(segment_start_point);
            dbg!(segment_end_point);
            dbg!(c.opt_extrude);
            // We don't expect the fine vase-esque slicing to be extruding while changing z
            panic!("ensure fine vase-esque gcode slicing has vase mode un-checked");
        }
        // println!("start: {:?}", segment_start_point);
        loop {
            let new_point_distance_from_start = MAX_SUBSEGMENT_LENGTH * (i as Factor);
            // dbg!(new_point_distance_from_start);
            if new_point_distance_from_start > segment_length {
                break;
            }
            let new_point_clean = segment_start_point + segment_direction_unit * new_point_distance_from_start;
            let new_point_fudged = Point{ x: new_point_clean.x + kd_fudge(), y: new_point_clean.y + kd_fudge(), z: new_point_clean.z + kd_fudge(), };
            // dbg!(new_point_clean);
            // dbg!(new_point_fudged);
            kd_tree.add(&[new_point_fudged.x, new_point_fudged.y, new_point_fudged.z], point_index.try_into().unwrap());
            i += 1;
        }
    };

    let mut default_handler = |_: &str| {};

    let mut line_handler = GcodeLineHandler{g1_handler: &mut g1_handler, default_handler: &mut default_handler};

    process_lines(gcode_lines, &mut line_handler);

    Layers {
        points,
        segment_runs,
        kd_tree,
    }
}

fn clamp_point_to_segment(loc: Point, segment_start: Point, segment_end: Point) -> Point {
    let end_minus_start = segment_end - segment_start;
    let end_minus_start_norm = end_minus_start.norm();
    let end_minus_start_unit = end_minus_start / end_minus_start_norm;
    let loc_minus_start = loc - segment_start;
    let loc_perp_intersect_distance = loc_minus_start.dot(end_minus_start_unit);
    let clamped_loc_perp_intersect_distance = loc_perp_intersect_distance.clamp(0.0, end_minus_start_norm);
    let clamped_loc_perp_intersect = segment_start + end_minus_start_unit * clamped_loc_perp_intersect_distance;
    clamped_loc_perp_intersect
}

fn point_segment_distance(loc: Point, segment_start: Point, segment_end: Point) -> Mm {
    let clamped_loc_perp_intersect = clamp_point_to_segment(loc, segment_start, segment_end);
    let loc_minus_clamped_loc_perp_intersect = loc - clamped_loc_perp_intersect;
    let distance = loc_minus_clamped_loc_perp_intersect.norm();
    distance
}

fn point_segment_index_distance(layers: &Layers, loc: Point, segment_start_point_index: PointIndex) -> Mm {
    let segment_start = layers.points[segment_start_point_index as usize];
    let segment_end = layers.points[segment_start_point_index as usize + 1];
    let real_distance = point_segment_distance(loc, segment_start, segment_end);
    real_distance
}

fn clamp_point_to_segment_index(layers: &Layers, loc: Point, segment_start_point_index: PointIndex) -> Point {
    let segment_start = layers.points[segment_start_point_index as usize];
    let segment_end = layers.points[segment_start_point_index as usize + 1];
    clamp_point_to_segment(loc, segment_start, segment_end)
}

struct ExtrudingG1 {
    // XYZ
    point: Point,

    // The extrude value isn't extrude per distance, just how much to extrude moving to the point
    // from whatever the previous point is. The need for the previous point to re-compute extrude
    // is why MAX_G1_BUFFERED is 4 not 3.
    //
    // E
    extrude: Mm,

    // F
    opt_f: Option<Mm>,

    // ;onwards, without leading space
    opt_comment: Option<String>,
}

struct BufferedExtrudingG1 {
    g1: ExtrudingG1,
    lines_after_g1: Vec<String>,
}

// With the segment buffer, we can go back and change a segment without disturbing the other gcode
// around it.
struct ExtrudingG1Buffer {
    buf_writer: io::BufWriter<std::fs::File>,
    g1s: VecDeque<BufferedExtrudingG1>,
}

impl ExtrudingG1Buffer {
    fn new(buf_writer: io::BufWriter<std::fs::File>) -> ExtrudingG1Buffer {
        ExtrudingG1Buffer { buf_writer, g1s: VecDeque::new() }
    }
}

impl ExtrudingG1Buffer {
    fn flush(&mut self) {
        while !self.g1s.is_empty() {
            self.emit_first_g1();
        }
        self.buf_writer.flush().expect("flush failed");
    }

    fn queue_line(&mut self, line: &str) {
        if self.g1s.is_empty() {
            writeln!(self.buf_writer, "{}", line).expect("write failed");
            //println!("==== {}", line);
            return;
        }
        self.g1s.back_mut().unwrap().lines_after_g1.push(line.into());
    }

    fn queue_g1(&mut self, g1: ExtrudingG1) {
        self.g1s.push_back(BufferedExtrudingG1 { g1, lines_after_g1: vec!() });

        self.peephole_buffer();

        while self.g1s.len() > MAX_G1_BUFFERED {
            self.emit_first_g1();
        }
    }

    fn emit_first_g1(&mut self) {
        assert!(!self.g1s.is_empty());
        let buf_writer = &mut self.buf_writer;
        let g1 = self.g1s.pop_front().unwrap();

        //print!(">>>> ");
        write!(buf_writer, "G1 X{} Y{} Z{} E{}", g1.g1.point.x, g1.g1.point.y, g1.g1.point.z, g1.g1.extrude).expect("write failed");
        //print!("G1 X{} Y{} Z{} E{}", g1.g1.point.x, g1.g1.point.y, g1.g1.point.z, g1.g1.extrude);
        if let Some(f) = g1.g1.opt_f {
            write!(buf_writer, " F{}", f).expect("write failed");
            //print!(" F{}", f);
        }
        if let Some(comment) = &g1.g1.opt_comment {
            write!(buf_writer, " {}", comment).expect("write failed");
            //print!(" {}", comment);
        }
        writeln!(buf_writer).expect("write failed");
        //println!();
        for line in &g1.lines_after_g1 {
            writeln!(buf_writer, "{}", line).expect("write failed");
            //println!("==== {}", line);
        }
    }

    fn peephole_buffer(&mut self) {
        if self.g1s.len() < 4 {
            return;
        }

        // check for small segment that could cause the printer to momentarily pause for no good
        // reason (actual reason might be the small segment having such a low extrude that it makes
        // the printer hit extrude acceleration limit, or the small segment going backwards, or
        // the printer just freaking out about it - not sure exactly why it happens for some of
        // them)
        let first_norm : Mm;
        let last_norm : Mm;
        let middle : Vec3;
        {
            let g = &mut self.g1s;
            let g_len = g.len();
            let p0 = &g[g_len - 4].g1.point;
            let p1 = &g[g_len - 3].g1.point;
            let p2 = &g[g_len - 2].g1.point;
            let p3 = &g[g_len - 1].g1.point;
            // This isn't very sophisiticated, but ... seems reasonable to delete the middle segment if
            // all of these are true:
            //   * middle segment is shorter than 0.05mm
            //   * the first and last are longer than 0.5mm
            //   * the first and last segment are headed basically the same direction as each other
            //     (but not necessarily the same direction as the middle segment)
            //   * the middle segment's removal doesn't cause deviation more than 0.0125/2.0
            //     perpendicularly to first or last (analogous to surface normal).
            //
            // This could be done in 2D, but already have 3D Vec3 stuff so just using that.
            const MIDDLE_SHORT_ENOUGH_THRESH : Mm = 0.050;
            const FIRST_AND_LAST_LONG_ENOUGH_THRESH : Mm = 0.5;
            const GCODE_RESOLUTION_THRESH : Mm = 0.0125 / 2.0;
            const SAME_DIRECTION_THRESH : Radians = 10.0 * (2.0 * PI / 360.0);
            middle = *p2 - *p1;
            let middle_norm = middle.norm();
            if middle_norm > MIDDLE_SHORT_ENOUGH_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; middle_norm > MIDDLE_SHORT_ENOUGH_THRESH".into());
                return;
            }
            let first = *p1 - *p0;
            first_norm = first.norm();
            if first_norm < FIRST_AND_LAST_LONG_ENOUGH_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; first_norm < FIRST_AND_LAST_LONG_ENOUGH_THRESH".into());
                return;
            }
            let last = *p3 - *p2;
            last_norm = last.norm();
            if last_norm < FIRST_AND_LAST_LONG_ENOUGH_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; last_norm < FIRST_AND_LAST_LONG_ENOUGH_THRESH".into());
                return;
            }
            let first_unit = first / first_norm;
            let last_unit = last / last_norm;
            let first_last_radians = first_unit.dot(last_unit).acos();
            if first_last_radians > SAME_DIRECTION_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; first_last_radians > SAME_DIRECTION_THRESH".into());
                return;
            }
            let first_unit_cross_middle = first_unit.cross(middle);
            let first_unit_cross_middle_norm = first_unit_cross_middle.norm();
            if first_unit_cross_middle_norm / 2.0 > GCODE_RESOLUTION_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; first_unit_cross_middle_norm / 2.0 > GCODE_RESOLUTION_THRESH".into());
                return;
            }
            let last_unit_cross_middle = last_unit.cross(middle);
            let last_unit_cross_middle_norm = last_unit_cross_middle.norm();
            if last_unit_cross_middle_norm / 2.0 > GCODE_RESOLUTION_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; last_unit_cross_middle_norm / 2.0 > GCODE_RESOLUTION_THRESH".into());
                return;
            }
        }
        // Now we can remove the middle segment without doing too much damage, hopefully...
        let middle_point : Point;
        let first_extrude_per_distance : Factor;
        let last_extrude_per_distance : Factor;
        {
            let g = &self.g1s;
            let g_len = g.len();
            let s1 = &g[g_len - 3].g1;
            let s3 = &g[g_len - 1].g1;
            first_extrude_per_distance = s1.extrude / first_norm;
            last_extrude_per_distance = s3.extrude / last_norm;
            middle_point = s1.point + middle / 2.0;
        }
        {
            let g = &mut self.g1s;
            let mut removed = g.remove(g.len() - 2).unwrap();
            let fixup = g.get_mut(g.len() - 2).unwrap();
            fixup.lines_after_g1.push("; deleted segment here".into());
            fixup.g1.point = middle_point;
            fixup.lines_after_g1.append(&mut removed.lines_after_g1);
        }
        // Fix up extrude for first and last.
        {
            let g = &mut self.g1s;
            let p0 = &g.get(g.len() - 3).unwrap().g1.point;
            let s1 = &g[g.len() - 2].g1;
            let s2 = &g[g.len() - 1].g1;
            let first = s1.point - *p0;
            let last = s2.point - s1.point;
            g.get_mut(g.len() - 2).unwrap().g1.extrude = first_extrude_per_distance * first.norm();
            g.get_mut(g.len() - 1).unwrap().g1.extrude = last_extrude_per_distance * last.norm();
        }
    }
}

fn generate_output(fine_layers : Layers, coarse_gcode_lines: io::Lines<io::BufReader<std::fs::File>>, buf_writer: io::BufWriter<std::fs::File>) {
    let output_buffer = Rc::new(RefCell::new(ExtrudingG1Buffer::new(buf_writer)));
    let mut max_iterations_exceeded_count: u64 = 0;
    let mut good_enough_before_max_iterations_count: u64 = 0;
    let mut old_loc = Point::default();
    let mut g1_handler = |c: G1LineContext| {
        let mut output_buffer = output_buffer.borrow_mut();
        if c.g.loc == c.old_loc && !c.opt_extrude.is_some() {
            let mut new_line = String::new();
            match c.opt_f {
                None => {
                    write!(&mut new_line, "; same pos, no E, no F, removed: {}", c.line).expect("write failed");
                },
                Some(f) => {
                    write!(&mut new_line, "G1 F{} ; was same pos, no E, squelched: {}", f, c.line).expect("write failed");
                    if let Some(comment) = c.opt_comment {
                        write!(new_line, " {}", comment).expect("write failed");
                    }
                }
            }
            output_buffer.queue_line(&new_line);
            return;
        }
        if !c.opt_extrude.is_some() || !c.has_explicit_z {
            output_buffer.queue_line(c.line);
            // We don't worry about this point being non-corrected. Technically "wrong" but doesn't
            // matter enough to worry about.
            old_loc = c.g.loc;
            return;
        }
        //println!("<<<< {}", c.line);
        let mut point_so_far : Point = c.g.loc;
        let mut refinement_step_ordinal = 0;
        loop {
            let kd_query_point = &[point_so_far.x, point_so_far.y, point_so_far.z];
            // TODO: Consider biasing the distance metric toward same-z candidates, to encourage fixing xy in fewer iterations.
            let nearest_kd_neighbour = fine_layers.kd_tree.nearest_one::<kiddo::float::distance::SquaredEuclidean>(kd_query_point);
            let segment_start_point_index = nearest_kd_neighbour.item;

            let mut best_distance_so_far = point_segment_index_distance(&fine_layers, point_so_far, segment_start_point_index);
            let mut checked_segments : HashSet<PointIndex> = HashSet::new();
            checked_segments.insert(segment_start_point_index);
            let mut best_segment_so_far = segment_start_point_index;

            // To be sure that we've found the closest point on any segment, we have to expand the
            // search radius for kd_tree points to be large enough to account for both
            // MAX_SUBSEGMENT_LENGTH (in case the kd_tree point is MAX_SUBSEGMENT_LENGTH further away
            // worst-case) and KD_TREE_FUDGE_RADIUS (in case the kd_tree point got fudged away by the
            // max due to being diagonally point to furthest point across the fudge cube).

            let max_possible_distance_of_kd_tree_point_of_best_segment: Mm = best_distance_so_far + MAX_SUBSEGMENT_LENGTH + KD_TREE_FUDGE_RADIUS;
            let squared_euclidean_distance = max_possible_distance_of_kd_tree_point_of_best_segment * max_possible_distance_of_kd_tree_point_of_best_segment;
            // We'd use within_unsorted_iter(), except that seems to have a stack overflow (only
            // sometimes), so instead we use within_unsorted().
            //
            // TODO: post issue, preferably with repro I guess, like by setting constant rng seed.
            let neighbours = fine_layers.kd_tree.within_unsorted::<SquaredEuclidean>(kd_query_point, squared_euclidean_distance);
            let neighbours = neighbours.iter().map(|neighbour| {neighbour.item});

            // Can uncomment this if kd_tree seems possibly sus.
            //let mut neighbours : Vec<u32> = vec!();
            //for segment_run in &fine_layers.segment_runs {
            //    for segment_index in segment_run.start..segment_run.end {
            //        neighbours.push(segment_index);
            //    }
            //}

            for neighbour in neighbours {
                let candidate_segment_start_index = neighbour;

                // multiple sub-segments of the same segment can be returned
                if checked_segments.contains(&candidate_segment_start_index) {
                    // we already checked the actual segment, so we don't need to re-check it having
                    // found it again via a different sub-segment's kd_tree point
                    continue;
                }
                checked_segments.insert(candidate_segment_start_index);
    
                let candidate_distance = point_segment_index_distance(&fine_layers, point_so_far, candidate_segment_start_index);
                if candidate_distance > best_distance_so_far {
                    continue;
                }

                best_distance_so_far = candidate_distance;
                best_segment_so_far = candidate_segment_start_index;
            }
    
            let best_segment_1 = best_segment_so_far;
    
            // Now we want a best_segment_2, that is not in the same layer as best_segment_1. This
            // means we want to expand the search again, and filter out the z value of
            // best_segment_1. Unfortunately, the kd_tree doesn't let us keep expanding the sphere
            // and get points in increasing distance, nor does it let us query for the nearest
            // point passing a filter. Because we'll be connecting the closest point on
            // best_segment_1 with the closest point on best_segment_2 to construct a segment
            // ("construction segment"; not printed), and then using the point on that line that's
            // closest to the z value of loc, we only really care about the actual geometry of
            // best_segment_2 when the z value of best_segment_2 is in the opposite z direction
            // from best_segment_1, since if it's in the same z direction, we'll just end up
            // clamping to the best_segment_1 end of the constructed segment anyway (we don't
            // extrapolate beyond the ends of the construction segment since that could cause the
            // output gcode to move backwards for short distances over already-printed perimeter
            // when crossing a seam in the layer below.
            //
            // By limiting the search for best_segment_2 to a reasonable distance, and just using
            // best_segment_1 if we don't find a best_segment_2, we also make sure the intro line
            // is just the intro line, and not some sort of mess. It also might handle particularly
            // advanced designed-for-vase-mode isolated bridge-through-the-air-intentionally paths
            // better, though that's TBV.
            //
            // The case we really want to do well is when the z value of loc is between two fine
            // layers' z values, with the two fine layers each having a nearby point that's roughly
            // a fractional coarse layer height away from loc in z. At the "fudge line", the xy
            // distance can be larger, but that's what all this code is designed to fix after all.
            //
            // The threshold distance is a max distance from loc (not expanded by distance to
            // best_segment_1), but to find candidates we still need to expand beyond that
            // threshold distance similar to what we did for best_segment_1 above.

            let exclude_z = fine_layers.points[best_segment_so_far as usize].z;

            // Expanding the kd_tree distance threshold below doesn't imply that we won't enforce
            // the OTHER_Z_MAX_DISTANCE_DEFAULT when it comes to the actual distances found.
            let mut best_distance_so_far = OTHER_Z_MAX_DISTANCE_DEFAULT;
            // The best segment can remain None if we don't find anything that passes the filter
            // any closer than OTHER_Z_MAX_DISTANCE_DEFAULT.
            let mut best_segment_so_far : Option<PointIndex> = None;
            checked_segments.clear();

            let max_possible_distance_of_kd_tree_point_of_best_segment = OTHER_Z_MAX_DISTANCE_DEFAULT + MAX_SUBSEGMENT_LENGTH + KD_TREE_FUDGE_RADIUS;
            let squared_euclidean_distance = max_possible_distance_of_kd_tree_point_of_best_segment * max_possible_distance_of_kd_tree_point_of_best_segment;

            let neighbours = fine_layers.kd_tree.within_unsorted::<SquaredEuclidean>(kd_query_point, squared_euclidean_distance);
            let neighbours = neighbours.iter().map(|neighbour| {neighbour.item});

            // Can uncomment this if kd_tree seems possibly sus.
            //let mut neighbours : Vec<u32> = vec!();
            //for segment_run in &fine_layers.segment_runs {
            //    for segment_index in segment_run.start..segment_run.end {
            //        neighbours.push(segment_index);
            //    }
            //}

            for neighbour in neighbours {
                let candidate_segment_start_index = neighbour;

                if checked_segments.contains(&candidate_segment_start_index) {
                    continue;
                }
                checked_segments.insert(candidate_segment_start_index);

                let candidate_segment_z = fine_layers.points[candidate_segment_start_index as usize].z;
                // The z values are all parsed from the same ascii or copied, so == should work for
                // this since they should be bit-for-bit the same (or not the same z).
                if candidate_segment_z == exclude_z {
                    continue;
                }

                let candidate_distance = point_segment_index_distance(&fine_layers, point_so_far, candidate_segment_start_index);
                if candidate_distance > best_distance_so_far {
                    continue;
                }

                best_distance_so_far = candidate_distance;
                best_segment_so_far = Some(candidate_segment_start_index);
            }
            let best_segment_2 = best_segment_so_far;

            let best_point_1 = clamp_point_to_segment_index(&fine_layers, point_so_far, best_segment_1);
            let best_point_2 = best_segment_2.map(|segment_start_index| {
                clamp_point_to_segment_index(&fine_layers, point_so_far, segment_start_index)
            });

            let old_point_so_far = point_so_far;

            let loc_z = c.g.loc.z;
            if best_point_2.is_none() {
                point_so_far = Point{x: best_point_1.x, y: best_point_1.y, z: loc_z};
            } else {
                let best_segment_2 = best_segment_2.unwrap();
                let best_point_2 = best_point_2.unwrap();
                if best_point_1.z == best_point_2.z {
                    dbg!(best_segment_1);
                    dbg!(best_segment_2);
                    dbg!(best_point_1);
                    dbg!(best_point_2);
                    dbg!(fine_layers.points[best_segment_1 as usize]);
                    dbg!(fine_layers.points[best_segment_1 as usize + 1]);
                    dbg!(fine_layers.points[best_segment_2 as usize]);
                    dbg!(fine_layers.points[best_segment_2 as usize + 1]);
                }
                assert!(best_point_1.z != best_point_2.z, "{} {}", best_point_1.z, best_point_2.z);
                let low_z_point : Point;
                let high_z_point : Point;
                if best_point_1.z > best_point_2.z {
                    low_z_point = best_point_2;
                    high_z_point = best_point_1;
                } else {
                    low_z_point = best_point_1;
                    high_z_point = best_point_2;
                }
                if point_so_far.z >= high_z_point.z {
                    point_so_far = Point{x: high_z_point.x, y: high_z_point.y, z: loc_z};
                } else if point_so_far.z <= low_z_point.z {
                    point_so_far = Point{x: low_z_point.x, y: low_z_point.y, z: loc_z};
                } else {
                    // barycentric-ish
                    let high_z_factor = (loc_z - low_z_point.z) / (high_z_point.z - low_z_point.z);
                    let high_minus_low = high_z_point - low_z_point;                    
                    let tmp_point = low_z_point + high_minus_low * high_z_factor;
                    // should be very close to equal
                    assert!((tmp_point.z - loc_z).abs() < 0.01);
                    point_so_far = Point{x: tmp_point.x, y: tmp_point.y, z: loc_z};
                }
            }

            refinement_step_ordinal += 1;
            if refinement_step_ordinal >= REFINEMENT_MAX_ITERATIONS {
                max_iterations_exceeded_count += 1;
                break;
            }

            let update_vec = point_so_far - old_point_so_far;
            if update_vec.norm() < REFINEMENT_GOOD_ENOUGH_TO_STOP_UPDATE_DISTANCE {
                good_enough_before_max_iterations_count += 1;
                break;
            }
        }
        let new_point = point_so_far;

        let old_move_distance = (c.g.loc - c.old_loc).norm();
        let old_extrude = c.opt_extrude.unwrap();
        let new_move_distance = (new_point - old_loc).norm();
        let new_extrude = old_extrude * new_move_distance / old_move_distance;

        //dbg!(old_move_distance);
        //dbg!(old_extrude);
        //dbg!(new_move_distance);
        //dbg!(new_extrude);

        output_buffer.queue_g1(ExtrudingG1 { point: new_point, extrude: new_extrude, opt_f: c.opt_f, opt_comment: c.opt_comment.map(|s| {s.into()}) });

        old_loc = new_point;
    };

    let mut default_handler = |s: &str| {
        output_buffer.borrow_mut().queue_line(s);
    };

    let mut line_handler = GcodeLineHandler{g1_handler: &mut g1_handler, default_handler: &mut default_handler};

    process_lines(coarse_gcode_lines, &mut line_handler);

    output_buffer.borrow_mut().flush();

    println!("max_iterations_exceeded_count: {} good_enough_before_max_iterations_count: {}", max_iterations_exceeded_count, good_enough_before_max_iterations_count);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        panic!("no tests yet");
        //let result = 2 + 2;
        //assert_eq!(result, 4);
    }
}
