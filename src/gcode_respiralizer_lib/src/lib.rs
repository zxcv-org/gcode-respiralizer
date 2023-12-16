use kiddo::{self, SquaredEuclidean};
use merging_iterator::MergeIter;
use ordered_float::OrderedFloat;
use rand::Rng;
use regex::Regex;
use std::cell::{RefCell, RefMut};
use std::cmp::{min, Ordering};
use std::collections::{BTreeMap, HashSet, VecDeque};
use std::f32::consts::PI;
use std::fmt::Write as fmt_Write;
use std::fs;
use std::io::{self, BufRead, Write};
use std::ops;
use std::ops::Bound::Excluded;
use std::ops::Bound::Included;
use std::rc::{Rc, Weak};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::thread;
use std::time::Instant;

// ideas:
//   * if gt 90 degrees from original segment (and part of zig zag zig), then skip?
//   * if new segment is shorter than 0.050 ish, skip?
//   * reverse/reverse detection?, eliminate zag in zig zag zig
//   * eliminate a short segment if between two longer segments (same direction or not), and elimination doesn't mess up 0.0125 gcode resolution threshold, leaving a single vertex instead of a segment
//   * when looking for point 2, constrain to find point 2 in opposite z direction from point 1, because nearly pointless to get a 2nd point in the same z direction - we want to terp if we can terp (maybe help with surface texture)
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
//
// One problem at a time, based on worst problem(s) observed on prints:
//   1. The itty bitty segments seem to freak out klipper and/or the printer leading to printer pausing momentarily without any obvious reason why (unless missed some backwards ones)
//   2. Slight resampling / interference / moire pattern in shallow angle reflections
//   which is worse? 1
//   how to fix?
//     * check for zig/zag/zig direction reversals, emit as extra gcode comments (this will determine if the pausing is caused by reversals, or just a short segment, maybe at slightly different angle, or degenerately-calculated angle)

// memory use ideas
//   * could split up big kd_tree into kd_tree per fine layer, and only expand search to further z kd_tree(s) if closest point found so far is further than z distance of additional layer
//   * could spool kd_tree per fine layer(s) through memory, not keeping all at once

// Ideas for dealing with fudge seam where it creates pointless bridging (in contrast to point-ful bridging that's on purpose).
//   * Assumption:
//     * Each horizontally-sliced fine layer is authoritative in the sense that horizontally-sliced layers never do pointless bridging; all bridging in horizontally-sliced layers is pointful.
//   * Prerequisites:
//     * be able to iterate along fine perimeters, including the jump back around to 1st point of that perimeter when needed
//     * track prev output point's p1 and opt p2, keeping the fine perimeter segment index and the 0.0-norm value along that segment
//   * How to detect questionable bridging:
//     * prev output point uses set of fine z values that's entirely non-overlapping with set of fine z values used by new point (not yet an output point).
//     * along prev-new line, do binary-search-esque thing that tries to find (actually bracket) a small range along line that goes from old z closest to new z closest; the distance between found z points for each end of that range is > a jump thresh
//   * Try to find a better path.
//     * select closest fine point to prev output point, select that fine perimeter (this isn't the exact splice start yet, but it is the fine splice perimeter)
//     * LOOKAHEAD_DISTANCE - collect/iterate coarse segments up to and including this distance
//     * need to exclude looping all the way around, so ... threshold the z difference to < 1/2 layer height as well
//     * (potentially make/use kd_tree with just the one fine perimeter)
//     * lookahead along coarse slicing (along segments, not just the points), find the "best" pair of points on coarse and find slicing that are close enough (in xy distance) to each other
//       * while searching, can tighten down the search radius as we find best decent point along the way so far, and can have an initial search radius based on close-enough threshold
//       * if we find a pair of points below quite-good thresh, we can stop before LOOKAHEAD_DISTANCE
//       * end_provisional = best points on both fine and coarse (closest to each other)
//     * prev output point stays in output; this corresponds to the prev input point before the splice, this is beginning of splice z terp
//     * find closest point on selected z fine perimeter to the prev output point, including segment index and 0-norm value
//     * in loop, iterate forward on selected fine perimeter by small amounts (1/2 res could work, starting at 0) and "fix" the point using the usual p1 p2 method,
//       until a fixed point is found which is >= along any fine z perimeters in common with prev output point (up to both of them), or has no perimeters in common
//       in which case that's also "fine" in the "everything is fine" sense - just take that point and hope in that case
//       * fallback: if we get further along than end_provisional (or maybe than min(end_provisional, BEGIN_SPLICE_FAIL_DISTANCE)), don't splice after all
//     * now we have/know the first spliced-in input point (the first one that passes the check above)
//     * splice middle fine points, but with z delta from coarse terped over fine distance
//     * to select/make the end of the splice
//       * take the end_provisional fine point, with z from end_provisional coarse (end of z terp)
//       * "fix" the end_provisional fine point (with normal alg), to get up to two positions along 2 fine perimeters
//       * starting from end_provisional coarse point, step forward by small amounts along coarse segment(s), and "fix" the point, until fixed point is >= along any fine z perimeters in common with fixed end_provisional fine,
//         or has no perimeters in common, or is farther than END_SPLICE_FAIL_DISTANCE in which case panic
//   * splicing considerations
//     * non-G1 lines
//     * G1 lines that only change F
//     * accumulate these and just plop them all before the splice segments, mainly to keep layer change comments etc
//     * may cause some to happen slightly early but that's fine (for things like printing slower for smaller layers, changing fan at layer, etc)
//   * done splicing in a "better" path into the input points; process the new input points as normal

// This way we can switch to f64 easily for comparing memory and performance.
// Also possibly in future it could make sense to "newtype" these, if it seems
// like it'd help avoid mistakes.
pub use f32 as Mm;
pub use f32 as Factor;
pub use f32 as Radians;
pub use u32 as PointIndex;

const COARSE_LAYER_HEIGHT: Mm = 0.3;

// We give up if we don't find any fine slicing points within this distance. When this threshold is
// exceeded, it's likely to mean that the fine slicing and coarse slicing aren't aligned.
const FIRST_Z_MAX_DISTANCE_DEFAULT: Mm = 3.0;
// TODO: Make this a command-line argument, with this as the default, or with this scaled by the
// auto-detected coarse vase slicing layer height.
//
// This needs to be roughly big enough to capture the coarse vase-mode slicing points back onto the
// fine slicing perimeters, but not so big that we'll think the intro line is part of the model for
// example. This threshold limit does not apply for the first found closest point, which we'll use
// without any "other" point if this threshold is exceeded by any other-z point on the fine slicing
// perimeters. So it's not a disaster for this threshold to be exceeded.
//
// This needs to be small enough to permit "detaching" from the other z perimeter if the closest
// point perimeter is just doing it's own thing that's totally different than any nearby perimeter.
//
// The 2.0 is to allow for every-other-perimeter stacking (hypothetical clever model). The SQRT_2
// is to account for up to 45 degree mis-alignment of the next perimeter up or down when that
// perimeter is essentially two full layer heights away (should be plenty). The 0.3 is roughly the
// max typical layer height with a 0.4mm nozzle, but really this part should be auto-detected from
// the coarse slicing input file (though that's not trivial since layer height could vary, but
// hopefully not in the vase mode portion of the coarse slicing).
const OTHER_Z_MAX_DISTANCE_DEFAULT: Mm = 2.0 * SQRT_2 * COARSE_LAYER_HEIGHT;
const REFINEMENT_MAX_ITERATIONS: u32 = 12;
const REFINEMENT_GOOD_ENOUGH_TO_STOP_UPDATE_DISTANCE: Mm = 0.001;

// The acceptable distance away from the closest fine perimeter somewhere along an input coarse
// segment that jumps from one set of fine z layers to a disjoint set of fine z layers. Beyond this
// threshold, disjoint sets of z layers will trigger an attempt to splice in a portion of a fine
// perimeter to avoid the jump.
//
// If this is too small, we might splice more than necessary and may spend more time checking for
// jumps. If this is too large, we'll fail to splice away pointless jumps up to this * 2.
//
// We also use this as the threshold for finding a splice end position that's close enough to
// justify doing the splice (we'll try to get closer, but if we can get at least this close,
// essentially replacing a deviation away from fine layers by > this much, with deviation away from
// fine layers with the splice <= this much (when we jump back to coarse from the spliced-in fine
// perimeter chunk), that's good enough to justify splicing).
//
// The length of the shortest pointless single-extrusion bridging that we'll potentially splice
// away is ~2x this value. The max distance of any jump from fine back to coarse at the end of a
// splice is 1x this value (else we won't do the splice, and just follow the coarse slicing even if
// it jumps at the start).
const HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD: Mm = 0.5 * OTHER_Z_MAX_DISTANCE_DEFAULT;

// We're willing to look up to half way around the model for a good-enough splice end that's better
// than what the horizontal shelf jump that we're trying to avoid.
const HORIZONTAL_SHELF_SPLICE_MAX_SEARCH_Z_DELTA: Mm = 0.5 * COARSE_LAYER_HEIGHT;

// Once we've found a good-enough splice, we keep looking up to this much further along the coarse
// perimeter to see if we can find a better end of the splice (instead of just good enough). This
// doesn't allow exceeding HORIZONTAL_SHELF_SPLICE_MAX_SEARCH_Z_DELTA overall.
//
// This basically accounts for approaching the fine slicing perimeter at a shallower-than-90-degree
// angle, and still wanting to get to a better splice end point despite the shallow approach angle,
// up to a point. A "2% grade" (rise of 1 for every 50 forward) seems like it's very likely to be
// far enough to find a much better splice end, unless the model is pretty much desined to be
// adversarial to gcode-spiralizer, in which case we'll just go with the closest thing we find
// within this search distance from the found good enough point. If this is too low, that just
// means we potentially jump unnecessarily at the end of the splice by up to
// HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD, since that's the "good enough" distance at the end of
// the splice to justify doing the splice (the start of the splice was a jump of at least 2.0 *
// HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD).
const HORIZONTAL_SHELF_SPLICE_KEEP_LOOKING_AFTER_GOOD_ENOUGH_DISTANCE: Mm = 50.0 * HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD;

// If we get this close, we take the splice end at this distance as plenty good; no point in
// continuing to search, even for the remainder of
// HORIZONTAL_SHELF_SPLICE_KEEP_LOOKING_AFTER_GOOD_ENOUGH_DISTANCE.
const HORIZONTAL_SHELF_END_SPLICE_EXCELLENT_DISTANCE: Mm = GCODE_RESOLUTION;

// To have 3 segments to catch zig-zag-zig, or to remove a tiny pointless segment between to other
// segments (while evaluating the reasonable-ness of doing so), we need 4 points.
const MAX_G1_BUFFERED: usize = 4;

const REMAINDER_OF_CURRENT_SEGMENT: Mm = Mm::MAX / 2.0;

// TODO: Get this from std when/if available there, and/or switch to const version of sqrt if/when
// that's a thing.
const FRAC_1_SQRT_3: Mm = 0.5773502691896257;
const SQRT_2: Mm = 1.4142135623730951;

const GCODE_RESOLUTION: Mm = 0.0125;

type KdTree = kiddo::float::kdtree::KdTree<Mm, PointIndex, 3, 32, u32>;

// We split move segments implied by the gcode into sub-segments so that the kiddo::KdTree can help
// us find long segments that happen to pass near the query point. Making this value too small will
// use more memory without speeding things up. Making this value too large won't save much memory
// and will slow things down. An ok-ish value is ~4x the length of a typical gcode segment when
// some curves are being approximated, to have most gcode segments only need one sub-segment.
const MAX_SUBSEGMENT_LENGTH: Mm = 2.0;
// The KdTree complains if too many items have the "same position on one axis". TBD if we can just
// fudge the locations in the KdTree slightly, and expand our search by this amount as well, to
// avoid having to increase the buckets and presumably waste memory. Also, it's not immediately
// clear from the error message if this would complain about a bunch of points in a perfect plane
// or not, since those would have the same position on one axis - the bucket size being large
// enough for all the points in a flat axis-aligned vertical surface of a big model would mean the
// bucket size would have to be pretty huge. TBD if this fudge kills performance somehow, or leads
// to degenerate cases in KdTree code. The points we end up actually using for generating output
// points don't have this fudge radius applied.
const KD_TREE_FUDGE_RADIUS: Mm = 0.1;
// We fudge per component, since we don't care if we have cube-shaped fudge as long as we know the
// max distance to the corner of that fudge cube. So if an edge of the cube is this long, the 3D
// diagonal corner to far corner across a cube should be KD_TREE_FUDGE_RADIUS.
const KD_TREE_FUDGE_PER_COMPONENT: Mm = KD_TREE_FUDGE_RADIUS * FRAC_1_SQRT_3;

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
    let fine_layers =
        read_fine_layers(file_lines(fine_reference_filename).expect("file_lines failed"));
    // Maybe use multiple threads to parse and build KdTree(s) faster, since mostly independent per
    // z layer of the fine slicing, aside from breaking the input file into lines. Detecting a new
    // layer does require parsing Z though...
    let read_layers_elapsed = before_read_layers.elapsed();
    let kd_tree_points = fine_layers
        .layers
        .iter()
        .map(|(_z, layer)| layer.borrow().kd_tree.size())
        .fold(0, |accum, size| accum + size);
    println!(
        "done reading fine layers - layers: {} kd_tree points: {} elapsed: {:.2?}",
        fine_layers.layers.len(),
        kd_tree_points,
        read_layers_elapsed
    );

    let coarse_gcode_lines = file_lines(coarse_reference_filename).expect("file_lines failed");
    let buf_writer = io::BufWriter::with_capacity(
        8 * 1024,
        fs::File::create(output_filename).expect("fs::File::open failed"),
    );

    let before_generate_output = Instant::now();
    generate_output(fine_layers, coarse_gcode_lines, buf_writer);
    let generate_output_elapsed = before_generate_output.elapsed();
    println!(
        "done generating output - elapsed: {:.2?}",
        generate_output_elapsed
    );
}

fn file_lines(filename: &str) -> io::Result<io::Lines<io::BufReader<std::fs::File>>> {
    Ok(io::BufReader::with_capacity(
        64 * 1024,
        fs::File::open(filename).expect("fs::File::open failed"),
    )
    .lines())
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
        Vec3 {
            x: a.y * b.z - a.z * b.y,
            y: a.x * b.z - a.z * b.x,
            z: a.x * b.y - a.y * b.x,
        }
    }
}

impl ops::Add<Vec3> for Vec3 {
    type Output = Vec3;

    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl ops::Add<Vec3> for Point {
    type Output = Point;

    fn add(self, rhs: Vec3) -> Point {
        Point {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl ops::Sub<Vec3> for Vec3 {
    type Output = Vec3;

    fn sub(self, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl ops::Sub<Vec3> for Point {
    type Output = Point;

    fn sub(self, rhs: Vec3) -> Point {
        Point {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl ops::Sub<Point> for Point {
    type Output = Vec3;

    fn sub(self, rhs: Point) -> Vec3 {
        Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
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
        Vec3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl ops::DivAssign<Factor> for Vec3 {
    fn div_assign(&mut self, rhs: Factor) {
        *self = Vec3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl ops::Mul<Factor> for Vec3 {
    type Output = Vec3;

    fn mul(self, rhs: Factor) -> Vec3 {
        Vec3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl ops::MulAssign<Factor> for Vec3 {
    fn mul_assign(&mut self, rhs: Factor) {
        *self = Vec3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl ops::Neg for Vec3 {
    type Output = Vec3;

    fn neg(self) -> Vec3 {
        Vec3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

#[allow(dead_code)]
struct ClosestPointsBetweenSegmentsResult {
    // The minmum distance between a and b. This takes into account the fact that a and b are
    // segments, not lines.
    min_distance: Mm,
    // How far away from a_start along a is closest to any point of b.
    along_a_distance: Mm,
    // How far away from b_start along b is closest to any point of a.
    along_b_distance: Mm,
}

#[allow(dead_code)]
fn closest_points_between_segments(
    a_start: &Point,
    a_end: &Point,
    b_start: &Point,
    b_end: &Point,
) -> ClosestPointsBetweenSegmentsResult {
    let p1 = a_start;
    let p2 = a_end;
    let p3 = b_start;
    let p4 = b_end;

    let v21 = *p2 - *p1;
    let dot_21_21 = v21.dot(v21);
    let v43 = *p4 - *p3;
    let dot_43_43 = v43.dot(v43);
    let v31 = *p3 - *p1;
    let dot_31_21 = v31.dot(v21);
    let v41 = *p4 - *p1;
    let dot_41_21 = v41.dot(v21);
    let v43 = *p4 - *p3;
    let dot_43_21 = v43.dot(v21);
    let dot_43_31 = v43.dot(v31);
    let v32 = *p3 - *p2;
    let dot_43_32 = v43.dot(v32);

    let denom = dot_43_21 * dot_43_21 - dot_21_21 * dot_43_43;
    let s = (dot_43_21 * dot_43_31 - dot_31_21 * dot_43_43) / denom;
    let t = (dot_43_31 * dot_21_21 - dot_43_21 * dot_31_21) / denom;

    if s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0 {
        // Closest points are within the two segments, so the closest points are the same as the
        // closest points between two lines.
        let min_distance = ((*p1 + v21 * s) - (*p3 + v43 * t)).norm();
        let along_a_distance = v21.norm() * s;
        let along_b_distance = v43.norm() * t;
        return ClosestPointsBetweenSegmentsResult {
            min_distance,
            along_a_distance,
            along_b_distance,
        };
    }

    // Closest points are not within one or the other or both segments, so at this point we have 4
    // cases. We compute the distance of each of these cases and pick the one with lowest distance.

    let s_along_a_closest_to_b_start = (dot_31_21 / dot_21_21).clamp(0.0, 1.0);
    let s_along_a_closest_to_b_end = (dot_41_21 / dot_21_21).clamp(0.0, 1.0);
    let t_along_b_closest_to_a_start = (-dot_43_31 / dot_43_43).clamp(0.0, 1.0);
    let t_along_b_closest_to_a_end = (-dot_43_32 / dot_43_43).clamp(0.0, 1.0);

    let p_along_a_closest_to_b_start = *a_start + v21 * s_along_a_closest_to_b_start;
    let p_along_a_closest_to_b_end = *a_start + v21 * s_along_a_closest_to_b_end;
    let p_along_b_closest_to_a_start = *b_start + v43 * t_along_b_closest_to_a_start;
    let p_along_b_closest_to_a_end = *b_start + v43 * t_along_b_closest_to_a_end;

    let b_start_to_a_norm = (p_along_a_closest_to_b_start - *b_start).norm();
    let b_end_to_a_norm = (p_along_a_closest_to_b_end - *b_end).norm();
    let a_start_to_b_norm = (p_along_b_closest_to_a_start - *a_start).norm();
    let a_end_to_b_norm = (p_along_b_closest_to_a_end - *a_end).norm();

    struct CaseSpec {
        distance: Mm,
        along_a_distance: Mm,
        along_b_distance: Mm,
    }

    let cases = &[
        CaseSpec {
            distance: b_start_to_a_norm,
            along_a_distance: v21.norm() * s_along_a_closest_to_b_start,
            along_b_distance: 0.0,
        },
        CaseSpec {
            distance: b_end_to_a_norm,
            along_a_distance: v21.norm() * s_along_a_closest_to_b_end,
            along_b_distance: v43.norm(),
        },
        CaseSpec {
            distance: a_start_to_b_norm,
            along_a_distance: 0.0,
            along_b_distance: v43.norm() * t_along_b_closest_to_a_start,
        },
        CaseSpec {
            distance: a_end_to_b_norm,
            along_a_distance: v21.norm(),
            along_b_distance: v43.norm() * t_along_b_closest_to_a_end,
        },
    ];

    let mut remap = vec![0usize, 1usize, 2usize, 3usize];
    remap.sort_by(|a, b| OrderedFloat(cases[*a].distance).cmp(&OrderedFloat(cases[*b].distance)));

    let winner = &cases[remap[0]];

    ClosestPointsBetweenSegmentsResult {
        min_distance: winner.distance,
        along_a_distance: winner.along_a_distance,
        along_b_distance: winner.along_b_distance,
    }
}

// A fine sclicing layer.
#[derive(Debug)]
pub struct Layer {
    weak_self: Weak<RefCell<Layer>>,

    // All the points of a layer have the same z. The intro line is detected as having negative y,
    // not included in any layer, and the coarse gcode is passed through for any G1 that doesn't
    // include explicit Z, which passes through the intro line and any horizontally-sliced bottom
    // coarse layers, without reference to any fine layer.
    z: Mm,

    // These are all the points with Point.z == z, in fine gcode order. Each layer is assumed to be
    // a loop, so there is also a (fabricated/replaced/patched in/implicit) segment from
    // `points[points.len() - 1]` to `points[0]``.
    points: Vec<Point>,

    // cumulative length of segments up to the point with the same index; [0] is always 0.0
    cumulative_distance_up_to: Vec<Mm>,

    // We use KdTree instead of ImmutableKdTree for these reasons:
    //   * No need to build a big slice with more entries than G1(s) in the fine gcode, in addition
    //     to the tree itself.
    //   * ImmutableKdTree can't have multiple entries that map to the same index, unlike KdTree
    //     which can.
    //
    // Each entry in kd_tree is the end point of a sub-segment with max length
    // MAX_SUBSEGMENT_LENGTH, with index into points (above). The kd_tree has no filtering by layer
    // aside from just the z distances increasing as we get further away in z from a layer.
    kd_tree: KdTree,

    // Total length of all segments in the layer, including the segment from points[points.len() -
    // 1] to points[0]. This is used during comparison; comparison has to work across wrap.
    total_length: Mm,
}

impl Layer {
    fn point_min_distance(&self, query: &Point, within_distance: Mm) -> Option<DistanceAndCursor> {
        let mut checked_segments: HashSet<PointIndex> = HashSet::new();
        let mut min_distance_so_far = within_distance + 0.01;
        let mut min_distance_cursor_so_far: Option<LayerCursor> = None;
        let kd_query_point = &[query.x, query.y, query.z];
        let max_possible_distance_of_kd_tree_point_of_best_segment: Mm =
            min_distance_so_far + MAX_SUBSEGMENT_LENGTH + KD_TREE_FUDGE_RADIUS;
        let squared_euclidean_distance = max_possible_distance_of_kd_tree_point_of_best_segment
            * max_possible_distance_of_kd_tree_point_of_best_segment;
        // We'd use within_unsorted_iter(), except that seems to have a stack overflow (only
        // sometimes), so instead we use within_unsorted().
        //
        // TODO: post issue, preferably with repro I guess, like by setting constant rng seed.
        let neighbours = self
            .kd_tree
            .within_unsorted::<SquaredEuclidean>(kd_query_point, squared_euclidean_distance);
        for neighbour in neighbours {
            let candidate_segment_start_index = neighbour.item;

            // multiple sub-segments of the same segment can be returned
            if checked_segments.contains(&candidate_segment_start_index) {
                // we already checked the actual segment, so we don't need to re-check it having
                // found it again via a different sub-segment's kd_tree point
                continue;
            }
            checked_segments.insert(candidate_segment_start_index);

            let DistanceAndCursor {
                distance: candidate_distance,
                layer_cursor: candidate_cursor,
            } = self.point_segment_index_min_distance(query, candidate_segment_start_index);
            if candidate_distance >= min_distance_so_far {
                continue;
            }

            min_distance_so_far = candidate_distance;
            min_distance_cursor_so_far = Some(candidate_cursor);
        }
        if min_distance_so_far > within_distance {
            return None;
        }
        assert!(min_distance_cursor_so_far.is_some());
        Some(DistanceAndCursor {
            distance: min_distance_so_far,
            layer_cursor: min_distance_cursor_so_far.expect("bug?"),
        })
    }
}

#[allow(dead_code)]
struct FindClosestToSegmentResult {
    // The location in the layer which is closest to any part of the query segment.
    layer_cursor: LayerCursor,
    // The location along the query segment which is closest to any part of any segment of the
    // layer.
    distance_along_query_segment: Mm,
    // The distance from the point implied by layer_cursor to the point implied by the query
    // segment and distance_along_query_segment.
    separation_distance: Mm,
}

struct DistanceAndCursor {
    distance: Mm,
    layer_cursor: LayerCursor,
}

struct DistanceAndDistanceAlongSegment {
    distance: Mm,
    distance_along_segment: Mm,
}

fn point_segment_min_distance(
    point: &Point,
    segment_start: &Point,
    segment_end: &Point,
) -> DistanceAndDistanceAlongSegment {
    let clamped_loc_perp_intersect = clamp_point_to_segment(*point, *segment_start, *segment_end);
    let loc_minus_clamped_loc_perp_intersect = *point - clamped_loc_perp_intersect;
    let distance = loc_minus_clamped_loc_perp_intersect.norm();
    let along_segment = clamped_loc_perp_intersect - *segment_start;
    let distance_along_segment = along_segment.norm();
    DistanceAndDistanceAlongSegment {
        distance,
        distance_along_segment,
    }
}

impl Layer {
    // Returns the closest point in the layer to any point on the query segment, and returns how
    // far along the query segment achieves the minimum distance.
    #[allow(unused)]
    fn find_closest_to_segment(
        &self,
        query_start: &Point,
        query_end: &Point,
        within_distance: Mm,
    ) -> Option<FindClosestToSegmentResult> {
        let query_vec = (*query_end - *query_start);
        let query_vec_norm = query_vec.norm();
        let query_vec_unit = query_vec / query_vec_norm;
        let mut distance_along_query_vec = 0.0;
        let mut considered_segments = HashSet::new();
        let kd_max_distance = within_distance + 2.0 * MAX_SUBSEGMENT_LENGTH + KD_TREE_FUDGE_RADIUS;
        let mut min_distance_so_far = within_distance + 0.01;
        let mut min_distance_so_far_distance_along_coarse: Option<Mm> = None;
        let mut min_distance_so_far_distance_along_fine: Option<Mm> = None;
        let mut min_distance_so_far_segment_index: PointIndex = 0;
        while distance_along_query_vec <= query_vec_norm {
            let query_point = *query_start + query_vec_unit * distance_along_query_vec;
            distance_along_query_vec += MAX_SUBSEGMENT_LENGTH;
            let kd_query_point = &[query_point.x, query_point.y, query_point.z];
            let kd_max_distance_squared = kd_max_distance * kd_max_distance;
            let neighbours = self.kd_tree.within_unsorted::<SquaredEuclidean>(kd_query_point, kd_max_distance_squared);
            for neighbour in neighbours {
                let fine_segment_index = neighbour.item;
                
                if considered_segments.contains(&fine_segment_index) {
                    continue;
                }
                considered_segments.insert(fine_segment_index);

                let fine_segment_start = self.points[fine_segment_index as usize];
                let fine_segment_end = self.points[(fine_segment_index as usize + 1) % self.points.len()];
                let closest_points = closest_points_between_segments(query_start, query_end, &fine_segment_start, &fine_segment_end);
                if closest_points.min_distance < min_distance_so_far {
                    min_distance_so_far = closest_points.min_distance;
                    min_distance_so_far_distance_along_coarse = Some(closest_points.along_a_distance);
                    min_distance_so_far_distance_along_fine = Some(closest_points.along_b_distance);
                    min_distance_so_far_segment_index = fine_segment_index;
                }
            }
        }
        if min_distance_so_far > within_distance {
            return None;
        }
        assert!(min_distance_so_far_distance_along_coarse.is_some());
        assert!(min_distance_so_far_distance_along_fine.is_some());

        Some(FindClosestToSegmentResult{layer_cursor: LayerCursor { layer: self.weak_self.upgrade().unwrap(), segment_start_index: min_distance_so_far_segment_index, distance_along_segment: min_distance_so_far_distance_along_fine.unwrap()}, distance_along_query_segment: min_distance_so_far_distance_along_coarse.unwrap(), separation_distance: min_distance_so_far})
    }

    fn point_segment_index_min_distance(
        &self,
        query: &Point,
        segment_start_index: PointIndex,
    ) -> DistanceAndCursor {
        let segment_start = self.points[segment_start_index as usize];
        let segment_end_index = (segment_start_index as usize + 1) % self.points.len();
        let segment_end = self.points[segment_end_index as usize];
        let distance_and_distance_along_segment =
            point_segment_min_distance(query, &segment_start, &segment_end);
        DistanceAndCursor {
            distance: distance_and_distance_along_segment.distance,
            layer_cursor: LayerCursor {
                layer: self.weak_self.upgrade().expect("upgrade failed"),
                segment_start_index: segment_start_index,
                distance_along_segment: distance_and_distance_along_segment.distance_along_segment,
            },
        }
    }
}

// All the fine slicing layers. The intro line is excluded (detected via negative y).
#[derive(Debug)]
pub struct Layers {
    // TODO: Consider streaming this through using a reader thread and discard of fine layers that
    // are out of z distance threshold below the coarse z we're working on. For some larger fine
    // slicings this would reduce memory usage quite a bit.
    //
    // key: z of the layer; we use Rc because LayerCursor needs to refer to layers also
    layers: BTreeMap<OrderedFloat<Mm>, Rc<RefCell<Layer>>>,
}

impl Layers {
    // returns distance to the cursor and the cursor, of the closest point in any segment of any layer, to the query point
    fn point_min_distance(
        &self,
        query: &Point,
        within_distance: Mm,
        exclude_z: Option<Mm>,
    ) -> Option<DistanceAndCursor> {
        let mut min_distance_so_far = within_distance + 0.01;
        let mut min_distance_cursor_so_far: Option<LayerCursor> = None;
        // Workaround for MergeIter::with_custom_ordering only accepting a function pointer not a closure.
        struct ZLayerAndQuery {
            z: Mm,
            layer: Rc<RefCell<Layer>>,
            query_z: Mm,
        }
        let ge_iter = self
            .layers
            .range((
                Included(OrderedFloat(query.z)),
                Included(OrderedFloat(query.z + within_distance)),
            ))
            .map(|z_layer| ZLayerAndQuery {
                z: **z_layer.0,
                layer: z_layer.1.clone(),
                query_z: query.z,
            });
        let lt_iter = self
            .layers
            .range((
                Included(OrderedFloat(query.z - within_distance)),
                Excluded(OrderedFloat(query.z)),
            ))
            .rev()
            .map(|z_layer| ZLayerAndQuery {
                z: **z_layer.0,
                layer: z_layer.1.clone(),
                query_z: query.z,
            });
        let merged_by_distance = MergeIter::with_custom_ordering(lt_iter, ge_iter, |a, b| {
            (a.z - a.query_z).abs() < (b.z - b.query_z).abs()
        })
        .filter(|z_layer_and_query| {
            exclude_z.is_none() || z_layer_and_query.z != exclude_z.unwrap()
        });
        for z_layer_and_query in merged_by_distance {
            let layer = z_layer_and_query.layer.borrow();
            if (layer.z - query.z).abs() > min_distance_so_far {
                break;
            }
            
            let maybe_distance_and_cursor = layer.point_min_distance(query, within_distance);
            if maybe_distance_and_cursor.is_none() {
                continue;
            }
            let distance_and_cursor = maybe_distance_and_cursor.unwrap();
            if distance_and_cursor.distance >= min_distance_so_far {
                continue;
            }
            min_distance_so_far = distance_and_cursor.distance;
            min_distance_cursor_so_far = Some(distance_and_cursor.layer_cursor);
        }
        if min_distance_so_far > within_distance {
            return None;
        }
        assert!(min_distance_cursor_so_far.is_some());
        Some(DistanceAndCursor {
            distance: min_distance_so_far,
            layer_cursor: min_distance_cursor_so_far.expect("bug?"),
        })
    }
}

#[derive(Clone, Debug)]
pub struct LayerCursor {
    layer: Rc<RefCell<Layer>>,
    segment_start_index: PointIndex,
    distance_along_segment: Mm,
}

#[derive(Debug)]
pub struct CmpInternalResult {
    ordering: Ordering,
    distance_to: Mm,
}

impl LayerCursor {
    fn get_point(&self) -> Point {
        let layer = self.layer.borrow();
        let start = layer.points[self.segment_start_index as usize];
        let end = layer.points[(self.segment_start_index as usize + 1) % layer.points.len()];
        let end_minus_start = end - start;
        let end_minus_start_len = end_minus_start.norm();
        let end_minus_start_direction = end_minus_start / end_minus_start_len;
        start + end_minus_start_direction * self.distance_along_segment
    }

    // This will advance by at most max_distance along the perimeter, and will loop from the end
    // of the fine slicing perimeter back to the start of the fine slicing perimeter, treating the
    // end to start segment as a normal segment. Each call will advance by a non-zero positive
    // amount, and is guaranteed to not entirely skip any segments.
    fn advance_by_at_most(&mut self, max_distance: Mm) {
        let layer = self.layer.borrow();
        self.distance_along_segment += max_distance;
        let segment_start = layer.points[self.segment_start_index as usize];
        let segment_end_index = (self.segment_start_index as usize + 1) % layer.points.len();
        let segment_end = layer.points[segment_end_index as usize];
        let segment_length = (segment_end - segment_start).norm();
        if self.distance_along_segment > segment_length {
            self.segment_start_index = ((self.segment_start_index as usize + 1) % layer.points.len()).try_into().unwrap();
            // Since we don't ever want to skip an entire segment, we may as well start at 0.0
            // along the new segment.
            self.distance_along_segment = 0.0;
        }
    }

    // This compares two cursors in a way that doesn't have a discontinuity at the wrap. If two
    // cursors are farther than layer.total_length / 2.0 from each other, the ordering is flipped
    // since that means the cursors are really talking about the shorter part of the perimeter, by
    // definition.
    fn cmp_internal(&self, other: &LayerCursor) -> CmpInternalResult {
        // This isn't unsafe since we're not doing anything unsafe with the pointer value(s).
        assert!(self.layer.as_ptr() == other.layer.as_ptr());
        let layer = self.layer.borrow();
        let mut self_cumulative_distance = layer.cumulative_distance_up_to[self.segment_start_index as usize] + self.distance_along_segment;
        let mut other_cumulative_distance = layer.cumulative_distance_up_to[other.segment_start_index as usize] + other.distance_along_segment;
        let half_total_length = 0.5 * layer.total_length;
        //dbg!(layer.total_length);
        //dbg!(self_cumulative_distance);
        //dbg!(other_cumulative_distance);
        assert!(self_cumulative_distance < layer.total_length + 0.001);
        assert!(other_cumulative_distance < layer.total_length + 0.001);
        if self_cumulative_distance == other_cumulative_distance {
            return CmpInternalResult{ ordering: Ordering::Equal, distance_to: 0.0 };
        }
        let mut self_minus_other = self_cumulative_distance - other_cumulative_distance;
        if self_minus_other.abs() > half_total_length {
            if self_cumulative_distance < other_cumulative_distance {
                self_cumulative_distance += layer.total_length;
            } else {
                other_cumulative_distance += layer.total_length;
            }
            self_minus_other = self_cumulative_distance - other_cumulative_distance;
        }
        if self_minus_other > 0.0 {
            return CmpInternalResult{ ordering: Ordering::Greater, distance_to: -self_minus_other };
        } else {
            return CmpInternalResult{ ordering: Ordering::Less, distance_to: -self_minus_other };
        }
    }

    fn cmp(&self, other: &LayerCursor) -> Ordering {
        let cmp_internal = self.cmp_internal(other);
        cmp_internal.ordering
    }

    fn distance_to(&self, other: &LayerCursor) -> Mm {
        let cmp_internal = self.cmp_internal(other);
        cmp_internal.distance_to
    }
}

#[derive(Clone, Debug, Default)]
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

#[derive(Clone, Debug)]
pub struct G1LineContext {
    line: String,
    old_loc: Point,
    g: GcodeState,
    has_explicit_z: bool,
    opt_extrude: Option<Mm>,
    opt_f: Option<Mm>,
    opt_comment: Option<String>,
    is_spliced: bool,
}

pub trait GcodeLineHandler {
    fn handle_g1(&mut self, g1: G1LineContext);
    fn handle_default(&mut self, line: &str);
}

pub fn process_lines(
    gcode_lines: io::Lines<io::BufReader<std::fs::File>>,
    line_handler: &mut dyn GcodeLineHandler,
) {
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
                Some(m) => Some(m.as_str().into()),
                None => None,
            };
            let opt_extrude: Option<Mm>;
            let opt_f: Option<Mm>;
            let has_explicit_z: bool;
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

            let g1_line_context = G1LineContext {
                line,
                old_loc,
                g: g.clone(),
                has_explicit_z,
                opt_extrude,
                opt_f,
                opt_comment,
                is_spliced: false,
            };
            line_handler.handle_g1(g1_line_context);
            continue;
        }
        line_handler.handle_default(line.as_str());
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

// ~190ms before.
pub fn read_fine_layers(gcode_lines: io::Lines<io::BufReader<std::fs::File>>) -> Layers {
    struct FineLineHandler {
        layers: Layers,
    }
    impl GcodeLineHandler for FineLineHandler {
        fn handle_g1(&mut self, c: G1LineContext) {
            if !c.opt_extrude.is_some() {
                // println!("no extrude");
                return;
            }

            if c.g.loc.y < 0.0 {
                // intro line
                return;
            }

            if c.g.loc.z != c.old_loc.z {
                panic!("fine sliced gcode is changing z while extruding? - fine should be sliced with vase mode off, retraction/detraction/extra detraction off - see README.md");
            }

            let mut layer: RefMut<Layer>;
            if let Some(existing_layer) = self.layers.layers.get_mut(&OrderedFloat(c.g.loc.z)) {
                layer = existing_layer.borrow_mut();
            } else {
                let new_layer_rc = Rc::new(RefCell::new(Layer {
                    weak_self: Weak::default(),
                    z: c.g.loc.z,
                    points: vec![],
                    cumulative_distance_up_to: vec![],
                    kd_tree: KdTree::new(),
                    total_length: 0.0,
                }));
                new_layer_rc.borrow_mut().weak_self = Rc::downgrade(&new_layer_rc);
                let prev_layer = self
                    .layers
                    .layers
                    .insert(OrderedFloat(c.g.loc.z), new_layer_rc);
                assert!(prev_layer.is_none());
                layer = self
                    .layers
                    .layers
                    .get_mut(&OrderedFloat(c.g.loc.z))
                    .unwrap()
                    .borrow_mut();
                layer.points.push(c.old_loc.clone());
                layer.cumulative_distance_up_to.push(0.0);
            }
            layer.points.push(c.g.loc.clone());

            let distance_so_far = *layer.cumulative_distance_up_to.last().unwrap();
            let distance_to_add = (layer.points[layer.points.len() - 1] - layer.points[layer.points.len() - 2]).norm();
            layer.cumulative_distance_up_to.push(distance_so_far + distance_to_add);
            layer.total_length += distance_to_add;

            // We ensure that each point on the segment is within MAX_SUBSEGMENT_LENGTH plus
            // KD_TREE_FUDGE_RADIUS of a point added to kd_tree.

            let segment_delta = c.g.loc - c.old_loc;
            // dbg!(segment_delta);
            let segment_length = segment_delta.norm();
            // The slicer shouldn't extrude without moving, so this won't be a divide by zero. If it is
            // a divide by zero, that's a problem with the fine slicing. We just let the divide by zero
            // happen here if the slicer messed up.
            let segment_direction_unit = segment_delta / segment_length;
            // dbg!(segment_direction_unit);

            let points = &mut layer.points;
            // grab the just-inserted segment
            let point_index: PointIndex = (points.len() - 2).try_into().unwrap();
            let segment_start_point = points[point_index as usize];
            let segment_end_point = points[point_index as usize + 1];
            if segment_start_point.z != segment_end_point.z {
                dbg!(segment_start_point);
                dbg!(segment_end_point);
                dbg!(c.opt_extrude);
                // We don't expect the fine vase-esque slicing to be extruding while changing z
                panic!("ensure fine slicing has vase mode un-checked");
            }
            // println!("start: {:?}", segment_start_point);
            let mut i = 0u32;
            loop {
                let new_point_distance_from_start = MAX_SUBSEGMENT_LENGTH * (i as Factor);
                // dbg!(new_point_distance_from_start);
                if new_point_distance_from_start > segment_length {
                    break;
                }
                let new_point_clean =
                    segment_start_point + segment_direction_unit * new_point_distance_from_start;
                let new_point_fudged = Point {
                    x: new_point_clean.x + kd_fudge(),
                    y: new_point_clean.y + kd_fudge(),
                    z: new_point_clean.z + kd_fudge(),
                };
                // dbg!(new_point_clean);
                // dbg!(new_point_fudged);
                layer.kd_tree.add(
                    &[new_point_fudged.x, new_point_fudged.y, new_point_fudged.z],
                    point_index.try_into().unwrap(),
                );
                i += 1;
            }
        }

        fn handle_default(&mut self, _line: &str) {}
    }
    let mut line_handler = FineLineHandler {
        layers: Layers {
            layers: BTreeMap::new(),
        },
    };

    process_lines(gcode_lines, &mut line_handler);

    for (_layer_z, layer_rc) in &line_handler.layers.layers {
        let mut layer = layer_rc.borrow_mut();
        // This will panic if a layer has only 1 point, which is fine since nothing after this will
        // work properly with a 1 point layer anyway.
        layer.total_length += (layer.points[0] - layer.points[layer.points.len() - 1]).norm();
        assert!(layer.cumulative_distance_up_to.len() == layer.points.len());

        //drop(layer);
        //let layer = layer_rc.borrow();
        //let mut layer_cursor = LayerCursor{distance_along_segment: 0.0, layer: layer.weak_self.upgrade().unwrap(), segment_start_index: 0};
        //for _i in 0..100 {
        //    let cloned_cursor = layer_cursor.clone();
        //    layer_cursor.advance_by_at_most(GCODE_RESOLUTION);
        //    let cmp_internal = cloned_cursor.cmp_internal(&layer_cursor);
        //    dbg!(cmp_internal);
        //}
    }

    // yoink
    line_handler.layers
}

fn clamp_point_to_segment(loc: Point, segment_start: Point, segment_end: Point) -> Point {
    let end_minus_start = segment_end - segment_start;
    let end_minus_start_norm = end_minus_start.norm();
    let end_minus_start_unit = end_minus_start / end_minus_start_norm;
    let loc_minus_start = loc - segment_start;
    let loc_perp_intersect_distance = loc_minus_start.dot(end_minus_start_unit);
    let clamped_loc_perp_intersect_distance =
        loc_perp_intersect_distance.clamp(0.0, end_minus_start_norm);
    let clamped_loc_perp_intersect =
        segment_start + end_minus_start_unit * clamped_loc_perp_intersect_distance;
    clamped_loc_perp_intersect
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
        ExtrudingG1Buffer {
            buf_writer,
            g1s: VecDeque::new(),
        }
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
        self.g1s
            .back_mut()
            .unwrap()
            .lines_after_g1
            .push(line.into());
    }

    fn queue_g1(&mut self, g1: ExtrudingG1) {
        self.g1s.push_back(BufferedExtrudingG1 {
            g1,
            lines_after_g1: vec![],
        });

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
        write!(
            buf_writer,
            "G1 X{} Y{} Z{} E{}",
            g1.g1.point.x, g1.g1.point.y, g1.g1.point.z, g1.g1.extrude
        )
        .expect("write failed");
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
        let first_norm: Mm;
        let last_norm: Mm;
        let middle: Vec3;
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
            const MIDDLE_SHORT_ENOUGH_THRESH: Mm = 0.050;
            const FIRST_AND_LAST_LONG_ENOUGH_THRESH: Mm = 0.5;
            const GCODE_RESOLUTION_THRESH: Mm = 0.0125 / 2.0;
            const SAME_DIRECTION_THRESH: Radians = 10.0 * (2.0 * PI / 360.0);
            middle = *p2 - *p1;
            let middle_norm = middle.norm();
            if middle_norm > MIDDLE_SHORT_ENOUGH_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; middle_norm >
                // MIDDLE_SHORT_ENOUGH_THRESH".into());
                return;
            }
            let first = *p1 - *p0;
            first_norm = first.norm();
            if first_norm < FIRST_AND_LAST_LONG_ENOUGH_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; first_norm <
                // FIRST_AND_LAST_LONG_ENOUGH_THRESH".into());
                return;
            }
            let last = *p3 - *p2;
            last_norm = last.norm();
            if last_norm < FIRST_AND_LAST_LONG_ENOUGH_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; last_norm <
                // FIRST_AND_LAST_LONG_ENOUGH_THRESH".into());
                return;
            }
            let first_unit = first / first_norm;
            let last_unit = last / last_norm;
            let first_last_radians = first_unit.dot(last_unit).acos();
            if first_last_radians > SAME_DIRECTION_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; first_last_radians >
                // SAME_DIRECTION_THRESH".into());
                return;
            }
            let first_unit_cross_middle = first_unit.cross(middle);
            let first_unit_cross_middle_norm = first_unit_cross_middle.norm();
            if first_unit_cross_middle_norm / 2.0 > GCODE_RESOLUTION_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; first_unit_cross_middle_norm / 2.0
                // > GCODE_RESOLUTION_THRESH".into());
                return;
            }
            let last_unit_cross_middle = last_unit.cross(middle);
            let last_unit_cross_middle_norm = last_unit_cross_middle.norm();
            if last_unit_cross_middle_norm / 2.0 > GCODE_RESOLUTION_THRESH {
                // g.back_mut().unwrap().lines_after_g1.push("; last_unit_cross_middle_norm / 2.0 >
                // GCODE_RESOLUTION_THRESH".into());
                return;
            }
        }
        // Now we can remove the middle segment without doing too much damage, hopefully...
        let middle_point: Point;
        let first_extrude_per_distance: Factor;
        let last_extrude_per_distance: Factor;
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

#[derive(Debug)]
enum GcodeInputHandlerItem {
    G1(G1LineContext),
    Default(String),
}

// For peeking ahead, we're only interested in G1s, but we want to retain the other lines as well,
// with correct ordering, so that we can "splice" by removing and replacing a chunk of G1s without
// removing any fan or temperature related commands (or anything else that's not a G1). We only
// ever splice extruding G1s, not non-extruding G1s, so we don't have to worry about non-extruding
// G1s being removed / replaced.
#[derive(Clone)]
struct GcodeInputG1Item {
    // ordered front to back, before g1
    default_items_before: VecDeque<String>,
    // ordered after default_items_before
    g1: Option<G1LineContext>,
}

struct RxBuffer {
    rx: Receiver<GcodeInputHandlerItem>,
    // These can be previously-peeked items and/or replacment items, and will be processed first,
    // ordered front to back. We try to avoid buffering way more items here than needed, since
    // that just uses more memory for no good reason.
    first: VecDeque<GcodeInputG1Item>,
    weak_self: Weak<RefCell<RxBuffer>>,
}

impl RxBuffer {
    fn process_lines(this: &Rc<RefCell<RxBuffer>>, line_handler: &mut dyn GcodeLineHandler) {
        let mut handle_item = move |item: GcodeInputHandlerItem| match item {
            GcodeInputHandlerItem::G1(g1) => {
                line_handler.handle_g1(g1);
            }
            GcodeInputHandlerItem::Default(line) => {
                line_handler.handle_default(&line);
            }
        };
        loop {
            let maybe_item = this.borrow_mut().first.pop_front();
            if let Some(mut item) = maybe_item {
                // We re-wrap in GcodeInputHandlerItem in here mainly for clarity and consistency
                // of handling; the cost shouldn't be particularly high since this only runs for
                // items that were peeked or spliced in, which shouldn't happen particularly often.
                while let Some(default_before) = item.default_items_before.pop_front() {
                    handle_item(GcodeInputHandlerItem::Default(default_before));
                }
                if let Some(g1) = item.g1 {
                    handle_item(GcodeInputHandlerItem::G1(g1));
                }
                continue;
            }
            let recvd = this.borrow().rx.recv();
            match recvd {
                Ok(item) => {
                    handle_item(item);
                    continue;
                }
                Err(_e) => {
                    assert!(this.borrow().first.is_empty());
                    println!("input channel done");
                    return;
                }
            }
        }
    }

    // We rely on for correctness, but don't enforce, that process_lines is not called while the
    // cursor is active.
    fn peek_cursor(&mut self) -> PeekCursor {
        PeekCursor::new(self.weak_self.upgrade().unwrap())
    }
}

#[derive(Clone)]
struct PeekCursor {
    rx_buffer: Rc<RefCell<RxBuffer>>,
    next_index_in_first: usize,
    current_g1: Option<GcodeInputG1Item>,
    segment_start: Point,
    segment_end: Point,
    segment_vec: Vec3,
    segment_vec_norm: Mm,
    sum_segment_vec_norms_before_current: Mm,
    sum_segment_extrusion_before_current: Mm,
    distance_along_segment: Mm,
    hit_eof: bool,
}

impl PeekCursor {
    fn new(rx_buffer: Rc<RefCell<RxBuffer>>) -> PeekCursor {
        PeekCursor{rx_buffer, next_index_in_first: 0, current_g1: None, segment_start: Point::default(), segment_end: Point::default(), segment_vec: Vec3::default(), segment_vec_norm: 0.0, sum_segment_vec_norms_before_current: 0.0, sum_segment_extrusion_before_current: 0.0, distance_along_segment: 0.0, hit_eof: false}
    }

    // TODO: pick a better name
    fn get_segment_and_advance(&mut self) -> bool {
        // segment_vec_norm is initially 0.0
        self.sum_segment_vec_norms_before_current += self.segment_vec_norm;
        if let Some(g1_input_item) = &self.current_g1 {
            self.sum_segment_extrusion_before_current += g1_input_item.g1.as_ref().unwrap().opt_extrude.as_ref().unwrap();
        }
        let mut rx_buffer = self.rx_buffer.borrow_mut();
        if self.next_index_in_first == rx_buffer.first.len() {
            rx_buffer.first.push_back(GcodeInputG1Item{default_items_before: VecDeque::new(), g1: None});
            loop {
                match rx_buffer.rx.recv() {
                    Ok(item) => {
                        match item {
                            GcodeInputHandlerItem::Default(line) => {
                                let input_g1 = rx_buffer.first.back_mut().unwrap();
                                input_g1.default_items_before.push_back(line);
                                continue;
                            },
                            GcodeInputHandlerItem::G1(c) => {
                                if c.g.loc == c.old_loc && !c.opt_extrude.is_some() {
                                    // TODO: avoid duplicated / super-similar code
                                    let mut new_line = String::new();
                                    match c.opt_f {
                                        None => {
                                            write!(&mut new_line, "; same pos, no E, no F, removed (sp): {}", c.line)
                                                .expect("write failed");
                                        }
                                        Some(f) => {
                                            write!(
                                                &mut new_line,
                                                "G1 F{} ; was same pos, no E, squelched (sp): {}",
                                                f, c.line
                                            )
                                            .expect("write failed");
                                            if let Some(comment) = c.opt_comment {
                                                write!(new_line, " {}", comment).expect("write failed");
                                            }
                                        }
                                    }
                                    let input_g1 = rx_buffer.first.back_mut().unwrap();
                                    input_g1.default_items_before.push_back(new_line);
                                    continue;
                                }
                                let input_g1 = rx_buffer.first.back_mut().unwrap();
                                // TODO: avoid the clone here I guess
                                input_g1.g1 = Some(c.clone());
                                if c.opt_extrude.is_none() || !c.has_explicit_z {
                                    // We never iterate into or past a non-extruding segment (but
                                    // we do save the non-extruding g1 info into rx_buffer.first
                                    // since it does need to get processed normally; just not as
                                    // a PeekCursor g1)
                                    return false;
                                }
                                break;
                            }
                        }
                    },
                    Err(_e) => {
                        self.hit_eof = true;
                        // In this case it's possible the input_g1 has None g1, which is fine for
                        // the last item in first in the whole file; that item never gets iterated
                        // over by PeekCursor; PeekCursor clients only want to know about g1(s).
                        return false;
                    }
                }
            }
        }
        assert!(self.next_index_in_first < rx_buffer.first.len());
        self.current_g1 = Some(rx_buffer.first.get(self.next_index_in_first).unwrap().clone());
        assert!(self.current_g1.as_ref().unwrap().g1.as_ref().unwrap().opt_extrude.is_some());
        self.next_index_in_first += 1;
        let g1_line = self.current_g1.as_ref().unwrap().g1.as_ref().unwrap();
        self.segment_start = g1_line.old_loc;
        self.segment_end = g1_line.g.loc;
        self.segment_vec = self.segment_end - self.segment_start;
        self.segment_vec_norm = self.segment_vec.norm();
        self.distance_along_segment = 0.0;
        true
    }
    fn next_advance_by_at_most(&mut self, at_most: Mm) -> bool {
        if !self.current_g1.is_some() {
            if !self.get_segment_and_advance() {
                assert!(self.hit_eof);
                return false;
            }
            // 0.0 along segment 0
            return true;
        }
        assert!(!self.hit_eof);
        self.distance_along_segment += at_most;
        if self.distance_along_segment > self.segment_vec_norm {
            if !self.get_segment_and_advance() {
                return false;
            }
        }
        // 0.0 along a new segment, or within current segment
        true
    }
    fn get_point(&self) -> Point {
        assert!(self.current_g1.is_some());
        self.segment_start + (self.segment_vec / self.segment_vec_norm) * self.distance_along_segment
    }
    fn get_distance_since_creation(&self) -> Mm {
        self.sum_segment_vec_norms_before_current + self.distance_along_segment
    }
}

fn handler_channel(channel_capacity: usize) -> (Box<dyn GcodeLineHandler + Send>, Rc<RefCell<RxBuffer>>) {
    struct UpstreamCoarseLineHandler {
        tx: SyncSender<GcodeInputHandlerItem>,
    }
    impl GcodeLineHandler for UpstreamCoarseLineHandler {
        fn handle_g1(&mut self, g1: G1LineContext) {
            let item = GcodeInputHandlerItem::G1(g1);
            self.tx.send(item).expect("send failed");
        }

        fn handle_default(&mut self, line: &str) {
            self.tx
                .send(GcodeInputHandlerItem::Default(line.into()))
                .expect("send failed");
        }
    }

    let (tx, rx) = sync_channel(channel_capacity);

    let upstream_handler = Box::new(UpstreamCoarseLineHandler { tx });

    let rx_buffer = Rc::new(RefCell::new(RxBuffer {
        rx,
        first: VecDeque::new(),
        weak_self: Weak::new(),
    }));
    rx_buffer.borrow_mut().weak_self = Rc::downgrade(&rx_buffer);

    (upstream_handler, rx_buffer)
}

fn generate_output(
    fine_layers: Layers,
    coarse_gcode_lines: io::Lines<io::BufReader<std::fs::File>>,
    buf_writer: io::BufWriter<std::fs::File>,
) {
    struct DownstreamCoarseHandler {
        output_buffer: ExtrudingG1Buffer,
        max_iterations_exceeded_count: u64,
        good_enough_before_max_iterations_count: u64,
        old_loc: Point,
        old_p1_cursor: Option<LayerCursor>,
        old_p2_cursor: Option<LayerCursor>,
        old_input_loc: Option<Point>,
        fine_layers: Layers,
        rx_buffer: Rc<RefCell<RxBuffer>>,
    }
    struct ForcedPoint {
        point: Point,
        p1_cursor: LayerCursor,
        p2_cursor: Option<LayerCursor>,
    }
    impl DownstreamCoarseHandler {
        fn any_z_in_common(
            p1_z: Mm,
            p2_z: Option<Mm>,
            o_p1_z: Option<Mm>,
            o_p2_z: Option<Mm>,
        ) -> bool {
            let mut new_zs = vec![p1_z];
            if let Some(p2_z) = p2_z {
                new_zs.push(p2_z);
            }
            if let Some(o_p1_z) = o_p1_z {
                if new_zs.contains(&o_p1_z) {
                    return true;
                }
                if let Some(o_p2_z) = o_p2_z {
                    if new_zs.contains(&o_p2_z) {
                        return true;
                    }
                }
            }
            false
        }
        
        // All points in the coarse slicing are fairly close to a point on a fine slicing perimeter
        // (less so for points impacted more by the vase mode fudge line, but still fairly close).
        //
        // All reasonable coarse segments stay fairly close to a fine layer perimeter along their
        // entire length. In contrast, a coarse segment that jumps across a horizontal step between
        // layers in the fine slicing creates a situation where somewhere along the length of the
        // coarse segment, the closest point on any fine slicing perimeter is fairly far away. This
        // represents not just bridging, but pointless bridging where a single extrusion is all
        // alone jumping onto or off of a horizontal shelf. These segments are considered
        // unreasonable coarse segments (if this function returns true in addition to checks so far
        // in the caller).
        //
        // The function that converts from 0..1 along the segment to distance to nearest point on
        // any fine perimeter isn't a particularly convenient function to maximize; there's no
        // guarantee that the midpoint of the segment will be the farthest from any fine perimeter.
        // We use the midpoint as an initial guess however. If that's already far enough away from
        // any fine perimeter to return true, super. If not, we scan the line in small enough steps
        // that we won't estimate low on the max distance by more than 1/2 the
        // HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD threshold (at least that's the idea).
        //
        // Both a and b should be coarse points directly from the coarse slicing.
        fn segment_gets_far_from_fine(&self, a: Point, b: Point) -> bool {
            let segment_vec = b - a;
            let segment_norm = segment_vec.norm();
            // This assumes that both a and b are quite close to a point on a fine perimeter, which
            // should be a good assumption as long as fine slicing and coarse slicing were done
            // per instructions in README.md.
            if segment_norm < 2.0 * HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD {
                return false;
            }
            let first_guess = a + segment_vec * 0.5;
            if let None = self.fine_layers.point_min_distance(&first_guess, HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD, None) {
                return true;
            }
            let iter_direction = segment_vec / segment_norm;
            let mut iter_distance = 0.0;
            // Maybe worth checking if this loop is eating a lot of time. Hopefully the current
            // fuction doesn't get called much to begin with, at least when a model doesn't need
            // splicing in the first place.
            while iter_distance < segment_norm {
                let test_point = a + iter_direction * iter_distance;
                if let None = self.fine_layers.point_min_distance(&test_point, HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD, None) {
                    return true;
                }
                iter_distance += HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD / 2.0;
            }
            false
        }

        fn try_splicing(&mut self, start_search_z: Mm) -> bool {
            let mut fine_cursor = self.old_p1_cursor.as_ref().unwrap().clone();
            let splice_layer_rc = fine_cursor.layer.clone();
            let splice_layer = splice_layer_rc.borrow();

            // Maybe should ensure that this maps back to gt old_p1_cursor and gt old_p2_cursor by
            // advancing more if needed, but for now just advance by a little bit along p1 cursor
            // and hope it does ok wrt being gt old_p2_cursor also. The symptom of being slightly
            // wrong here will be quite minor compared to the single-extrusion pointless bridging,
            // but that's a low bar (ok for first rev though).
            fine_cursor.advance_by_at_most(GCODE_RESOLUTION);

            let mut first_bad_coarse_point_cursor = self.rx_buffer.borrow_mut().peek_cursor();
            first_bad_coarse_point_cursor.get_segment_and_advance();

            // Up to this point, we've been assuming that bad_coarse_point is far from
            // splice_layer; time to check.
            let first_bad_coarse_point = first_bad_coarse_point_cursor.get_point();
            if let Some(_) = splice_layer.point_min_distance(&first_bad_coarse_point, 2.0 * HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD) {
                println!("bad_coarse_point was already close to splice_layer; not splicing");
                return false;
            }

            let mut best_distance_to_fine_so_far = HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD + 0.01;
            let mut best_fine_cursor_so_far: Option<LayerCursor> = None;
            let mut best_coarse_cursor_so_far: Option<PeekCursor> = None;
            let mut distance_along_coarse_when_found_good_enough: Option<Mm> = None;
            let max_search_z = start_search_z + HORIZONTAL_SHELF_SPLICE_MAX_SEARCH_Z_DELTA;
            // The searching in this loop obeys the max search thresholds approximately, not
            // precisely. The small subtraction here prevents find_closest_to_segment from needing
            // to search again near the end of each SEARCH_GRANULARITY chunk of a long coarse
            // segment, since find_closest_to_segment iterates forward MAX_SUBSEGMENT_LENGTH at a
            // time internally. For vase models with very short perimeters, like the narrow part of
            // a narrow funnel, this value may need to be lower, if that model also has horizontal
            // step jumps in that area of the model. This value will also effectively become
            // slightly smaller due to flattening/projecting the z ramped coarse path onto the
            // plane at start_search_z (the z of the last coarse input point that mapped to an
            // output point (same z on input and output) prior to points of this (potential)
            // splice).
            const SEARCH_GRANULARITY: Mm = 5.0 * MAX_SUBSEGMENT_LENGTH - 0.01;

            // Now we need to look along the coarse segments for a place that's again close enough
            // to fine_cursor.layer. The coarse segments can be peeked from the RxBuffer.
            let mut peek_cursor = self.rx_buffer.borrow_mut().peek_cursor();
            loop {
                // first call will leave cursor at 0.0 along segment 0
                if !peek_cursor.next_advance_by_at_most(SEARCH_GRANULARITY) {
                    // hit EOF; maybe we have a splice end we can use already; break and see
                    break;
                }

                let cursor_point = peek_cursor.get_point();

                if let Some(distance_along_coarse_when_found_good_enough) = distance_along_coarse_when_found_good_enough {
                    if peek_cursor.get_distance_since_creation() > distance_along_coarse_when_found_good_enough + HORIZONTAL_SHELF_SPLICE_KEEP_LOOKING_AFTER_GOOD_ENOUGH_DISTANCE {
                        break;
                    }
                }

                let query_start_with_ramped_z = cursor_point;
                let remaining_len_in_3d = (peek_cursor.segment_end - query_start_with_ramped_z).norm();
                let query_len_in_3d: f32 = min(OrderedFloat(SEARCH_GRANULARITY), OrderedFloat(remaining_len_in_3d)).into();
                let query_end_with_ramped_z = query_start_with_ramped_z + (peek_cursor.segment_vec / peek_cursor.segment_vec_norm) * query_len_in_3d;

                // Potentially we could search the first part of SEARCH_GRANULARITY with z less
                // than or equal to max_search_z, but doesn't seem worth the extra code, especially
                // since reducing SEARCH_GRANULARITY is already mentioned as potentially needing to
                // be smaller for super-short-perimeter models that also have horizontal shelves.
                if query_end_with_ramped_z.z > max_search_z {
                    break;
                }

                let query_start = Point{z: start_search_z, ..query_start_with_ramped_z};
                let query_end = Point{z: start_search_z, ..query_end_with_ramped_z};
                let maybe_closest_segment = splice_layer.find_closest_to_segment(&query_start, &query_end, HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD);
                if maybe_closest_segment.is_none() {
                    continue;
                }
                let closest_segment = maybe_closest_segment.unwrap();

                if closest_segment.separation_distance >= best_distance_to_fine_so_far {
                    continue;
                }

                let mut found_best_so_far = false;
                let mut found_excellent = false;
                if closest_segment.separation_distance <= HORIZONTAL_SHELF_END_SPLICE_EXCELLENT_DISTANCE {
                    found_best_so_far = true;
                    found_excellent = true;
                }
                if best_distance_to_fine_so_far > HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD && closest_segment.separation_distance <= HORIZONTAL_SHELF_JUMP_DETECTION_THRESHOLD {
                    found_best_so_far = true;
                    distance_along_coarse_when_found_good_enough = Some(peek_cursor.get_distance_since_creation() + closest_segment.distance_along_query_segment);
                }
                if found_best_so_far {
                    best_distance_to_fine_so_far = closest_segment.separation_distance;
                    best_fine_cursor_so_far = Some(closest_segment.layer_cursor.clone());
                    let mut coarse_cursor = peek_cursor.clone();
                    coarse_cursor.distance_along_segment += closest_segment.distance_along_query_segment;
                    best_coarse_cursor_so_far = Some(coarse_cursor);
                }

                if found_excellent {
                    break;
                }
            }

            if best_coarse_cursor_so_far.is_none() {
                // This can happen if we hit EOF or non-extrusion before finding a good splice or
                // if we just didn't find a good splice end despite looking ahead the max distance.
                println!("no good-enough splice found despite disjoint fine z(s); not splicing");
                return false;
            }

            // TODO: check here that best_fine_cursor_so_far is "after" fine_cursor, accounting for
            // wrapping being totally fine. A reasonable-ish way to check is to see if the interval
            // contains less than half the total distance of the fine perimeter (similar to how
            // checking for "after" works when working in wrapped unsigned integer space). Models
            // for which that check wouldn't work "well enough" seem likely to be basically
            // adversarial. This TODO existing shouldn't cause nozzle crashes, so seems reasonable
            // enough to see how it goes without the check for now.

            let coarse_extrusion_per_distance = peek_cursor.sum_segment_extrusion_before_current / peek_cursor.sum_segment_vec_norms_before_current;
            let end_fine_cursor = best_fine_cursor_so_far.unwrap();
            let end_coarse_cursor = best_coarse_cursor_so_far.unwrap();

            // We fill in the z ramp values and full G1LineContext once we have the fine distance
            // over which to ramp.
            //
            // The self.old_input_loc is the last coarse input point before the spliced/synthesized
            // points. The current fine_cursor is the first fine point of the splice. The
            // end_fine_cursor is the last fine point of the splice. A little beyond
            // end_coarse_cursor is the first coarse point after the splice, but still synthesized.
            // The next coarse point after a little beyond end_coarse_cursor is the first coarse
            // input point that's after the splice, but the segment inbound to that point is fixed
            // up since it comes from the last synthesized point and needs extrusion fixed up. When
            // done splicing, the synthesized points are in front, followed by the coarse input
            // point with inbound segment fixed up. After that (in front or via channel) are
            // unmodified input points. Every synthesized point and the first unmodified coarse
            // point after the splice with fixed-up inbound segment are marked as part of the
            // splice for purposes of avoiding re-splicing over already-spliced before.
            //
            // This list contains synthesized points only, which means it doesn't include the first
            // unmodified coarse input point after the splice, despite it having fixed-up inbound
            // segment.
            let mut replacement_points: VecDeque<Point> = VecDeque::new();

            assert!(start_search_z == self.old_input_loc.unwrap().z);
            while fine_cursor.segment_start_index as usize != (end_fine_cursor.segment_start_index as usize + 1) % splice_layer.points.len() {
                let point = fine_cursor.get_point();
                replacement_points.push_back(Point{z: start_search_z, ..point});
                fine_cursor.advance_by_at_most(REMAINDER_OF_CURRENT_SEGMENT);
            }
            if end_fine_cursor.distance_along_segment != 0.0 {
                replacement_points.push_back(Point{z: start_search_z, ..end_fine_cursor.get_point()});
            }
            let mut just_beyond_end_coarse_cursor = end_coarse_cursor.clone();
            // Ideally we'd advance by enough here to get a point that's > the previous point on
            // all fine layers in common after this point and prev point go through the fixup
            // algorithm. For now we just advance a little bit and hope we don't get artifacts from
            // going backwards after fixup algorithm.
            if !just_beyond_end_coarse_cursor.next_advance_by_at_most(GCODE_RESOLUTION) {
                return false;
            }
            replacement_points.push_back(Point{z: start_search_z, ..just_beyond_end_coarse_cursor.get_point()});

            // ensure "first" has enough items before committing to splicing
            let mut ensure_first_items = just_beyond_end_coarse_cursor.clone();
            if !ensure_first_items.get_segment_and_advance() {
                return false;
            }
            drop(ensure_first_items);

            // The next point won't be replaced, so doesn't go in replacment_points, but it will
            // end up getting its incoming segment fixed up since that segment starts with the last
            // point we added just above.

            // Grab/rescue all the non-extrusion lines.
            let mut rescued_lines: VecDeque<String> = VecDeque::new();
            let mut index_in_first: usize = 0;
            while index_in_first < just_beyond_end_coarse_cursor.next_index_in_first {
                rescued_lines.append(&mut self.rx_buffer.borrow_mut().first[index_in_first].default_items_before);
                index_in_first += 1;
            }

            self.rx_buffer.borrow_mut().first.drain(..just_beyond_end_coarse_cursor.next_index_in_first);
            // These are no longer valid since they index into self.rx_buffer.first.
            drop(just_beyond_end_coarse_cursor);
            drop(end_coarse_cursor);
            drop(peek_cursor);

            // initially pointing at first unmodified point beyond splice, which needs its incoming
            // segment fixed up
            let mut first_unmodified_cursor = self.rx_buffer.borrow_mut().peek_cursor();
            if !first_unmodified_cursor.get_segment_and_advance() {
                panic!("impossible; already forced 'first' to have enough items above");
            }
            // first_unmodified_cursor still pointing at first unmodified point beyond splice,
            // now with info cached in fields of first_unmodified_cursor, so get_point can work
            //
            // don't need this to be mutable beyond this point
            let first_unmodified_cursor = first_unmodified_cursor;

            // Compute xy distance over which to ramp z. The first segment is self.old_input_loc to
            // replacement_points[0]. The last segment is
            // replacement_points[replacement_points.len() - 1] to rx_buffer.first[0].
            let mut total_z_ramp_xy_distance = 0.0;
            let mut prev_point = self.old_input_loc.unwrap();
            assert!(start_search_z == self.old_input_loc.unwrap().z);
            for point in &replacement_points {
                assert!(prev_point.z == start_search_z);
                assert!(point.z == start_search_z);
                total_z_ramp_xy_distance += (*point - prev_point).norm();
                prev_point = point.clone();
            }
            total_z_ramp_xy_distance += (Point{z: start_search_z, ..first_unmodified_cursor.current_g1.unwrap().g1.unwrap().g.loc} - replacement_points[replacement_points.len() - 1]).norm();
            let total_z_ramp_xy_distance = total_z_ramp_xy_distance;

            // Apply z ramp; AFAIK, this z ramp has the same caveats as typical z ramping slicings
            // (IIUC) when it comes to dealing with tricky models - in particular, this does
            // nothing to deal with layer-below z jumps due to completely different longer or
            // shorter path taken by the layer below, which would of course get complicated for
            // next layer above this one and so on. If we were going to address that "problem",
            // we'd need to address it globally, not just here. Such an approach might be more
            // plausible if implemented directly in the slicer, but it would get substantially
            // complicated - see README.md for any further thoughts on the overall topic.
            let mut z_ramp_xy_distance_so_far = 0.0;
            let mut prev_point_xy = self.old_input_loc.unwrap();
            let lower_z = prev_point_xy.z;
            let upper_z = self.rx_buffer.borrow().first[0].g1.as_ref().unwrap().g.loc.z;
            for point in &mut replacement_points {
                z_ramp_xy_distance_so_far += (*point - prev_point_xy).norm();
                prev_point_xy = *point;
                let z_for_point = lower_z + (upper_z - lower_z) * (z_ramp_xy_distance_so_far / total_z_ramp_xy_distance);
                *point = Point{z: z_for_point, ..*point};
            }

            // Now distances between points are all what they'll be in the synthetic input we're
            // creating. We convert the points into synthetic g1 entries now.

            // we'll keep the synthesized input entries separate from rx_buffer.front at first,
            // mainly to avoid confusion, since we still have the first unmodified point after the
            // splice in rx_buffer.front[0], and working forwards is less confusing than working
            // backwards
            let mut new_g1s: VecDeque<GcodeInputG1Item> = VecDeque::new();

            let mut prev_point = self.old_input_loc.unwrap();
            for point in replacement_points {
                let segment_norm = (point - prev_point).norm();
                let extrude = coarse_extrusion_per_distance * segment_norm;

                let mut line = String::from("");
                write!(line, "G1 X{} Y{} Z{} E{} ; splice", point.x, point.y, point.z, extrude).expect("write failed");
                new_g1s.push_back(GcodeInputG1Item { default_items_before: VecDeque::new(), g1: Some(G1LineContext { line, old_loc: prev_point, g: GcodeState { loc: point, is_abs_xyz: true, is_rel_e: true }, has_explicit_z: true, opt_extrude: Some(extrude), opt_f: None, opt_comment: Some(String::from("; splice")), is_spliced: true }) });

                prev_point = point;
            }

            // fix up inbound segment for rx_buffer.first[0] (first unmodified coarse point after
            // slice, but it's incoming segment needs fixup)

            {
                let mut rx_buffer = self.rx_buffer.borrow_mut();
                let coarse_after = rx_buffer.first[0].g1.as_mut().unwrap();
                let prev_point = new_g1s.back().unwrap().g1.as_ref().unwrap().g.loc;
                let segment_norm = (coarse_after.g.loc - prev_point).norm();
                let extrusion = coarse_extrusion_per_distance * segment_norm;
                let new_coarse_after = G1LineContext{old_loc: prev_point, opt_extrude: Some(extrusion), ..coarse_after.clone()};
                *coarse_after = new_coarse_after;
            }

            {
                let mut rx_buffer = self.rx_buffer.borrow_mut();
                new_g1s.append(&mut rx_buffer.first);
                rx_buffer.first = new_g1s;
                rx_buffer.first[0].default_items_before.append(&mut rescued_lines);
            }

            true
        }

        fn force_point(&mut self, input_point: Point) -> ForcedPoint {
            let mut point_so_far: Point = input_point;
            let mut refinement_step_ordinal = 0;
            let mut p1_cursor: Option<LayerCursor>;
            let mut p2_cursor: Option<LayerCursor>;
            loop {
                let p1_distance_and_cursor = self.fine_layers.point_min_distance(
                    &point_so_far, FIRST_Z_MAX_DISTANCE_DEFAULT, None)
                    .expect("p1 not found within FIRST_Z_MAX_DISTANCE_DEFAULT; fine slicing and coarse slicing not aligned?");

                // Now we want a p2_distance_and_cursor, that is not in the same layer as
                // p1_distance_and_cursor. This means we want to filter out the z value of
                // p1_distance_and_cursor. Unfortunately, the kd_tree doesn't let us keep expanding
                // the sphere and get points in increasing distance, nor does it let us query for
                // the nearest point passing a filter. Because we'll be connecting the closest
                // point p1_distance_and_cursor with the 2nd closest (with different z) point
                // p2_distance_and_cursor to construct a segment ("construction segment"; not
                // printed), and then using the point on that line that's closest to the z value of
                // loc, we only really care about the actual geometry of p2_distance_and_cursor
                // when the z value of p2_distance_and_cursor is in the opposite z direction from
                // p1_distance_and_cursor, since if it's in the same z direction, we'll just end up
                // clamping to the p1_distance_and_cursor end of the constructed segment anyway (we
                // don't extrapolate beyond the ends of the construction segment since that could
                // cause the output gcode to move backwards for short distances over
                // already-printed perimeter when crossing a seam in the layer below).
                //
                // By limiting the search for p2_distance_and_cursor to a reasonable distance, and
                // just using p1_distance_and_cursor if we don't find a p2_distance_and_cursor, we
                // might handle particularly advanced designed-for-vase-mode isolated
                // bridge-through-the-air-intentionally paths better, though that's TBV.
                //
                // The case we really want to do well is when the z value of loc is between two
                // fine layers' z values, with the two fine layers each having a nearby point
                // that's roughly a fractional coarse layer height away from loc in z. At the
                // "fudge line", the xy distance can be larger.
                //
                // The threshold distance is a max distance from loc (not expanded by distance to
                // p1_distance_and_cursor).

                let exclude_z = p1_distance_and_cursor.layer_cursor.layer.borrow().z;
                let p2_distance_and_cursor = self.fine_layers.point_min_distance(
                    &point_so_far,
                    OTHER_Z_MAX_DISTANCE_DEFAULT,
                    Some(exclude_z),
                );

                let best_point_1 = p1_distance_and_cursor.layer_cursor.get_point();
                let best_point_2 = p2_distance_and_cursor
                    .as_ref()
                    .map(|distance_and_cursor| distance_and_cursor.layer_cursor.get_point());

                p1_cursor = Some(p1_distance_and_cursor.layer_cursor);
                p2_cursor = p2_distance_and_cursor.map(|it| it.layer_cursor);

                let old_point_so_far = point_so_far;

                let loc_z = input_point.z;
                if best_point_2.is_none() {
                    point_so_far = Point {
                        x: best_point_1.x,
                        y: best_point_1.y,
                        z: loc_z,
                    };
                } else {
                    let best_point_2 = best_point_2.unwrap();
                    if best_point_1.z == best_point_2.z {
                        dbg!(best_point_1);
                        dbg!(best_point_2);
                    }
                    assert!(
                        best_point_1.z != best_point_2.z,
                        "{} {}",
                        best_point_1.z,
                        best_point_2.z
                    );
                    let low_z_point: Point;
                    let high_z_point: Point;
                    if best_point_1.z > best_point_2.z {
                        low_z_point = best_point_2;
                        high_z_point = best_point_1;
                    } else {
                        low_z_point = best_point_1;
                        high_z_point = best_point_2;
                    }
                    if point_so_far.z >= high_z_point.z {
                        point_so_far = Point {
                            x: high_z_point.x,
                            y: high_z_point.y,
                            z: loc_z,
                        };
                    } else if point_so_far.z <= low_z_point.z {
                        point_so_far = Point {
                            x: low_z_point.x,
                            y: low_z_point.y,
                            z: loc_z,
                        };
                    } else {
                        let high_z_factor =
                            (loc_z - low_z_point.z) / (high_z_point.z - low_z_point.z);
                        let high_minus_low = high_z_point - low_z_point;
                        let tmp_point = low_z_point + high_minus_low * high_z_factor;
                        // should be very close to equal
                        assert!((tmp_point.z - loc_z).abs() < 0.01);
                        point_so_far = Point {
                            x: tmp_point.x,
                            y: tmp_point.y,
                            z: loc_z,
                        };
                    }
                }

                refinement_step_ordinal += 1;
                if refinement_step_ordinal >= REFINEMENT_MAX_ITERATIONS {
                    self.max_iterations_exceeded_count += 1;
                    break;
                }

                let update_vec = point_so_far - old_point_so_far;
                if update_vec.norm() < REFINEMENT_GOOD_ENOUGH_TO_STOP_UPDATE_DISTANCE {
                    self.good_enough_before_max_iterations_count += 1;
                    break;
                }
            }
            let new_point = point_so_far;
            ForcedPoint { point: new_point, p1_cursor: p1_cursor.unwrap(), p2_cursor: p2_cursor }
        }
    }
    impl GcodeLineHandler for DownstreamCoarseHandler {
        fn handle_g1(&mut self, c: G1LineContext) {
            if c.g.loc == c.old_loc && !c.opt_extrude.is_some() {
                let mut new_line = String::new();
                match c.opt_f {
                    None => {
                        write!(&mut new_line, "; same pos, no E, no F, removed: {}", c.line)
                            .expect("write failed");
                    }
                    Some(f) => {
                        write!(
                            &mut new_line,
                            "G1 F{} ; was same pos, no E, squelched: {}",
                            f, c.line
                        )
                        .expect("write failed");
                        if let Some(comment) = c.opt_comment {
                            write!(new_line, " {}", comment).expect("write failed");
                        }
                    }
                }
                self.output_buffer.queue_line(&new_line);
                return;
            }
            if !c.opt_extrude.is_some() || !c.has_explicit_z {
                self.output_buffer.queue_line(c.line.as_str());
                // We don't worry about this point being non-corrected. Technically "wrong" but
                // doesn't matter enough to worry about.
                self.old_loc = c.g.loc;
                return;
            }
            //println!("<<<< {}", c.line);
            let input_loc = c.g.loc;
            let forced_point = self.force_point(input_loc);
            if !c.is_spliced && self.old_input_loc.is_some() && !Self::any_z_in_common(
                forced_point.p1_cursor.layer.borrow().z,
                forced_point.p2_cursor.as_ref().map(|c| c.layer.borrow().z),
                self.old_p1_cursor.as_ref().map(|c| c.layer.borrow().z),
                self.old_p2_cursor.as_ref().map(|c| c.layer.borrow().z),
            ) && self.segment_gets_far_from_fine(self.old_input_loc.unwrap(), input_loc) {
                if self.try_splicing(self.old_loc.z) {
                    // We spliced some of a fine perimeter into the pending input, in place of a
                    // problematic (horizontal step skip) portion of the coarse slicing. So we need
                    // to return to the caller so it can essentially retry with the bad coarse
                    // point (and some coarse points contiguously after that) replaced with a
                    // splice derived from fine layers. The spliced points are marked as spliced so
                    // we won't re-splice and potentially get stuck in an infinite loop.
                    return;
                }
            }

            let old_move_distance = (c.g.loc - c.old_loc).norm();
            let old_extrude = c.opt_extrude.unwrap();
            let extrude_per_distance = old_extrude / old_move_distance;
            let mut prev_point = self.old_loc;

            // Without this chunk of code, the first part of every coarse perimeter of the output
            // post-respiralizing looks more rough. There are points that are out of place, vs the
            // last part of every coarse perimeter which looks good. I believe the reason for this
            // is the offset between the input coarse slicing and the input fine slicing is maximal
            // at the start of each input coarse slicing perimeter (just after the fudged seam).
            //
            // The input coarse slicing points are essentially forced onto the input fine slicing
            // perimeters in xy. But the roughness results when the optimal placement of points
            // along the perimeter is different enough between the input fine slicing perimeter and
            // the input coarse slicing, that the input coarse slicing's points aren't placed in
            // important needed locations along the coarse perimeter. The force_point code doesn't
            // invent points, and doesn't optimize on a segment-to-segment level, so really doesn't
            // move points along the perimeter much; it mostly just moves points to the closest
            // reasonable place on the fine perimeters (roughly speaking, for clarity of
            // explanation). By bringing in the fine perimeter points (and only doing so at this
            // stage, after splicing has done whatever it's going to do), we benefit from the fine
            // slicing having a better fit of segments to the local shape of the perimeter, with
            // substantially less offset between the points being forced and the perimeters (up to
            // two) that we're forcing to.
            //
            // As mentioned below, we only use the closest fine perimeter for this, which maybe
            // could be improved upon by using both the closest and second-closest ("p1" and "p2")
            // when those have z on either side of the coarse z we're at. However, a concern with
            // using the second-closest is what to do in situations where the coarse perimeter will
            // be guiding us off away from second-closest - the concern is that by using both, we'd
            // increase the chance of outputting a point that creates a sorta zig-zag-zig (the zag
            // point being the one adversely impacted by using second-closest) when there should
            // just be a zig. For now, just using the closest fine perimeter for this seems to work
            // fine. Though of course the "closest" may be different for the previous point, so we
            // use the closest for this point, and see if we can find the same perimeter in the
            // prev point. If not, we bail on adding any of these in-between points from the
            // closest fine perimeter since at that point "in-between" isn't really well defined
            // enough, and probably we're better served by just letting the input coarse points
            // guide the output points until the situation becomes clear again. Keeping in mind
            // that we're already post-splicing at this point, so we shouldn't be getting any jumps
            // on/off of (less than roughly 1/2-perimeter-sized) horizontal steps.
            //
            // For now we use up to one old cursor if there's a shared layer, but not a 2nd even if
            // there are two shared layers. TODO: consider using both, though unclear how to order
            // them, or how deal with potentially getting misdirected by a second-closest if it's
            // diverging from closest. See longer paragraphs above.
            let mut old_cursor: Option<LayerCursor> = None;
            if let Some(old_p1_cursor) = self.old_p1_cursor.as_ref() {
                if forced_point.p1_cursor.layer.borrow().z == old_p1_cursor.layer.borrow().z {
                    old_cursor = Some(old_p1_cursor.clone());
                }
            }
            if let Some(old_p2_cursor) = self.old_p2_cursor.as_ref() {
                if forced_point.p1_cursor.layer.borrow().z == old_p2_cursor.layer.borrow().z {
                    assert!(old_cursor.is_none());
                    old_cursor = Some(old_p2_cursor.clone());
                }
            }
            if let Some(mut iter_old_to_new_p1_cursor) = old_cursor {
                let total_iter_distance = iter_old_to_new_p1_cursor.distance_to(&forced_point.p1_cursor);
                let low_z = prev_point.z;
                let high_z = forced_point.point.z;
                if total_iter_distance > 0.0 {
                    loop {
                        iter_old_to_new_p1_cursor.advance_by_at_most(REMAINDER_OF_CURRENT_SEGMENT);
                        let cmp_internal = iter_old_to_new_p1_cursor.cmp_internal(&forced_point.p1_cursor);
                        if cmp_internal.ordering != Ordering::Less {
                            break;
                        }
                        let iter_distance = total_iter_distance - cmp_internal.distance_to;
                        let z_ramped = low_z + (high_z - low_z) * (iter_distance / total_iter_distance);
                        let iter_forced_point = self.force_point(Point{z: z_ramped, ..iter_old_to_new_p1_cursor.get_point()});
                        let mut iter_forced_cursor: Option<&LayerCursor> = None;
                        if forced_point.p1_cursor.layer.borrow().z == iter_forced_point.p1_cursor.layer.borrow().z {
                            iter_forced_cursor = Some(&iter_forced_point.p1_cursor);
                        } else if let Some(iter_p2_cursor) = iter_forced_point.p2_cursor.as_ref() {
                            if forced_point.p1_cursor.layer.borrow().z == iter_p2_cursor.layer.borrow().z {
                                iter_forced_cursor = Some(iter_p2_cursor);
                            }
                        }
                        if iter_forced_cursor.is_none() {
                            break;
                        }
                        let iter_forced_cursor = iter_forced_cursor.unwrap();
                        if iter_forced_cursor.cmp(&forced_point.p1_cursor) != Ordering::Less {
                            break;
                        }
                        let extrude = extrude_per_distance * (iter_forced_point.point - prev_point).norm();
                        self.output_buffer.queue_g1(ExtrudingG1 { point: iter_forced_point.point, extrude, opt_f: None, opt_comment: Some(String::from("; iter")) });
                        prev_point = iter_forced_point.point;
                    }
                } else {
                    // TODO: negative values might be small backwards movements, in which case we
                    // could avoid that by tracking progress along a p1 cursor and p2 cursor to
                    // only output points that are moving forward on post-force_point fine
                    // perimeters. So far doesn't seem to be a huge issue, but some models might be
                    // impacted more (by not doing this yet).

                    //if total_iter_distance < 0.0 {
                    //    dbg!(total_iter_distance);
                    //}
                }
            }

            let new_move_distance = (forced_point.point - prev_point).norm();
            let new_extrude = old_extrude * new_move_distance / old_move_distance;

            //dbg!(old_move_distance);
            //dbg!(old_extrude);
            //dbg!(new_move_distance);
            //dbg!(new_extrude);

            self.output_buffer.queue_g1(ExtrudingG1 {
                point: forced_point.point,
                extrude: new_extrude,
                opt_f: c.opt_f,
                opt_comment: c.opt_comment.map(|s| s.into()),
            });

            self.old_loc = forced_point.point;
            self.old_p1_cursor = Some(forced_point.p1_cursor);
            self.old_p2_cursor = forced_point.p2_cursor;
            self.old_input_loc = Some(input_loc);
        }

        fn handle_default(&mut self, s: &str) {
            self.output_buffer.queue_line(s);
        }
    }

    let (mut tx_handler, rx_buffer) = handler_channel(512);

    let coarse_reader_thread = thread::spawn(move || {
        process_lines(coarse_gcode_lines, tx_handler.as_mut());
        println!("coarse reader thread done");
    });

    let mut coarse_handler = DownstreamCoarseHandler {
        output_buffer: ExtrudingG1Buffer::new(buf_writer),
        max_iterations_exceeded_count: 0,
        good_enough_before_max_iterations_count: 0,
        old_loc: Point::default(),
        old_p1_cursor: None,
        old_p2_cursor: None,
        old_input_loc: None,
        fine_layers,
        rx_buffer: rx_buffer.clone(),
    };

    RxBuffer::process_lines(&rx_buffer, &mut coarse_handler);
    coarse_handler.output_buffer.flush();

    coarse_reader_thread.join().expect("join failed");

    println!(
        "max_iterations_exceeded_count: {} good_enough_before_max_iterations_count: {}",
        coarse_handler.max_iterations_exceeded_count,
        coarse_handler.good_enough_before_max_iterations_count
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: Mm, b: Mm) {
        if (a - b).abs() < 0.00001 {
            return;
        }
        assert_eq!(a, b);
    }

    #[test]
    fn test_closest_points_between_segments() {
        // Case 1 - line-to-line distance
        let a_start = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let a_end = Point {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let b_start = Point {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let b_end = Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let result = closest_points_between_segments(&a_start, &a_end, &b_start, &b_end);
        let a_norm = (a_end - a_start).norm();
        let b_norm = (b_end - b_start).norm();
        let s = result.along_a_distance / a_norm;
        let t = result.along_b_distance / b_norm;
        assert_close(s, 3.0 / 4.0);
        assert_close(t, 1.0 / 4.0);
        assert_close(result.min_distance, 2.0_f32.sqrt() / 2.0);

        // Case 2 - point-to-line distance (because off end of segment a)
        let a_start = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let a_end = Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let b_start = Point {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let b_end = Point {
            x: 2.0,
            y: 3.0,
            z: 5.0,
        };
        let result = closest_points_between_segments(&a_start, &a_end, &b_start, &b_end);
        let a_norm = (a_end - a_start).norm();
        let b_norm = (b_end - b_start).norm();
        let s = result.along_a_distance / a_norm;
        let t = result.along_b_distance / b_norm;
        // off the end of line a, so needs to correctly use end of a, so clamp s to 1.0 (just 1.0
        // overall)
        assert_close(s, (5.0_f32 / 4.0_f32).clamp(0.0, 1.0));
        // (({1, 2, 3}-{1, 1, 1}).(({2, 3, 5}-{1, 1, 1})/norm({2, 3, 5}-{1, 1, 1})))/norm({2, 3,
        // 5}-{1, 1, 1})
        assert_close(t, 10.0 / 21.0);
        // closest points on Point(1, 2, 3) and Line((1, 1, 1), (2, 3, 5))
        assert_close(result.min_distance, (5.0_f32 / 21.0_f32).sqrt());
    }
}
