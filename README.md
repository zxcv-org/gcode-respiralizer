# gcode-respiralizer

The project/repo name is gcode-respiralizer (dash).

The binary name is gcode_respiralizer (underscore).

The lib name is gcode_respiralizer_lib (underscores).

## Goal / Purpose

Eliminate the vase mode "seam" that results when slicing in vase mode in
prusaslicer.

This "seam" isn't a normal retraction/detraction layer switch seam, but rather
the result of the slicer internally doing normal constant-z-per-layer slicing,
then ramping z within each layer's path and fudging to (sorta) connect the end
of one layer to the beginning of the next layer. This fudge is the vase mode
"seam".

It's worth noting that "real" vase mode spiral slicing is fraught with gotchas.
For example, selecting the slope of the spiral can increase the overall length
of the perimeter (until it stacks on top of itself and needs to be 1 layer
height up from where it started), which can then require a lower slope, but then
using that lower slope the perimeter is shorter, requiring a higher slope.
There's also the concern/requirement that vase mode models tend to use "steps"
with flat tops (or bottoms) to switch from a detour to a bridge that skips the
detour; a pure spiral would tend to "fall off" such steps, leading to a much
longer bridge that looks wrong and wasn't the model designer's intent. So any
algorithm that hopes to avoid extruding in thin air across the middle of a
detour that's about to disappear has to deviate from a pure spiral and
essentially choose to take the detour or not take the detour, even if the detour
and the non-detour short bridge fall at an inconvenient location.

It may be fair to say that there's not really any one "correct" vase mode
slicing algorithm. It depends on the goals.

Here are the goals for gcode-respiralizer:

* Eliminate (or at least greatly reduce) the vase mode "seam".
* Don't "jump off steps" / "jump onto steps" (see above).
* Do stuff in a way that'd be at least somewhat plausible for slic3r /
  prusaslicer to implement internally (though probably not in the same language,
  and almost certainly not at the same stage of slicing).
* Don't break aspects of current prusaslicer vase mode slicing that work (at
  least for non-hostile cases), such as the transition from flat build plate to
  z ramp regime, and the smoothness of the z ramp within the spiral regime, and
  the ability to have a few flat bottom layers before the spiral starts.

Because the slicer should really handle vase mode slicing better, the
gcode-respiralizer can be considered a prototype / temporary solution / test of
ideas. That said, a primary goal is to be reliable for typical vase mode models,
since that's a prerequisite for demonstrating that the slicer "should" do these
things instead of a gcode post-processor.

Ease-of-use is a lower-priority goal, but still a goal. Suggestions/issues
welcome.

## How-To

### Prusaslicer

1. Put path to gcode_respiralizer binary in the Print settings ->
   Post-processing steps
1. Set to vase mode, accept 1 perimeter etc.
1. Un-check vase mode (but leave 1 perimeter etc).
1. Set layer height and first layer height to 0.02mm (~1/10th of the layer
   height you actually want)
1. Slice (gcode-respiralizer will auto-detect that this is the finely-sliced
   file, and will save a copy for gcode-respiralizer to use later)
1. Set retract/detract/extra detract to all off / 0.0mm.
1. Set layer height to 0.2mm or 0.3mm (or whatever you actually want)
1. Set retract/detract/extra detract as desired (0.0mm typically wouldn't be a
   disaster for vase mode, but YMMV).
1. Check vase mode again.
1. Slice (gcode-respiralizer will auto-detect that this is the coarse-sliced
   file, and based on filename, will pair it up with the previously-saved
   finely-sliced file)
1. Open output in gcode analyzer, visually verify that it looks plausible.
1. Print, and hopefully verify that the vase mode "seam" is not present.

## Theory of Operation

Do a 1 perimeter normal (not vase) slice of the model using a very low layer
height like 0.002mm. Start by turning on vase mode, then turn it off again. Use
random seam placement. Don't use arcs. Call this gcode output "fine".

Without moving the object, do a vase mode slice of the model using the desired
layer height like 0.2mm or 0.3mm. Call this gcode output "coarse".

Let "paths" mean the intro line of fine, the intro line of coarse, the spiral
path of coarse, and the various single-layer paths of fine.

Points along the paths of coarse can be points that are directly in the gcode,
or points that are linearly / barycentric interpolated between two points from
the coarse gcode.

* For points along the paths of coarse (maintaining extrude-or-not consistency
  with coarse, and extrude-amount-per-distance of coarse):
  * Find the closest point on any path of fine.
  * Exclude that path (whole z value) of fine, and again find the closest point
    on any path of fine.
  * Use the z values of the two found points and the z value of the coarse point
    to establish where the coarse z point is in relation to the two found fine
    points.
    * The z value of the coarse point can be in between the z values of points
      along the two fine paths, or the z value of the coarse point can be
      outside that z range.
  * Apply the linear interpolation / extrapolation to replace the x, y values of
    the coarse point with the interpolated/extrapolated x, y value from the
    linear interpolation / extrapolation.
  * Output the x, y, z as a new coarse_out point, computing extrusion based on
    coarse extrusion-per-distance.

In this manner, as the coarse z height varies gradually, the x, y location of
the output coarse points will define a spiral that crosses through each fine
path from each fine z height, and between fine z heights, the output point will
be on a line that is drawn between the closest points drawn between the two
closest fine z height paths.

Overall, this should handle these:

* intro line
* first few layers where coarse jumps once per perimeter by decreasing amounts
  to get from 1st layer into spiral regime
* spiral regime
* prints that have detours that disappear in the next layer up via bridging
  which skips the detour, without jumping into the detour or jumping out of the
  detour with the coarse output path, and without any unfortunately handling of
  boundary condition when doing the last coarse loop of a detour
* boundary condition for extrusions without any more extrusions stacked directly
  above - we avoid "blurring" here by allowing the slope to be retained even if
  the coarse line is above both selected fine points on fine paths
* ignoring points on fine paths that "should be" irrelevant / ignored, via
  distance metric, and lack of any assumptions regarding any correspondences of
  gcode points from layer to layer (in coarse or fine).

Some details:

* Each fine path at specific z is not a perfect closed loop (this would not be
  an issue if the ideas in gcode-respiralizer were translated into code within
  slic3r / prusaslicer), since the core slicing at a given z would generate a
  closed loop (a closed non-self-intersecting polygon). For now we close the
  loop with an extra added line.
  * The motivation for adding the line is to avoid creating a situation where
    the output coarse path would go backwards a little, for example if there's a
    fine seam gap below the coarse z, and no fine seam gap slightly below that.
  * The motivation is mainly relevant for extrapolation where the coarse z is
    outside the range of z from the two fine paths - when the coarse z is
    between the two fine z values, the output path would "slow down" then "jump"
    a little, but not go backwards.
  * The alternate way to handle this is to not add the extra line, but constrain
    the linear situation to interpolation along a line segment connecting the
    two fine points, never an extrapolation. This could still result in the
    coarse path stopping (generating two points on top of each other, which
    could easily be filtered out), but it wouldn't go backwards.
* The coarse gcode vase "seam" is still corresponding to a layer switch in the
  coarse slicing, so is still a place where the coarse x, y can be
  discontinuous, if that "seam" jumps across a surface height discontinuity.
  * An optional mitigation of this can be to notice the distance of the coarse
    path from any fine path exceeding a threshold for an interval on the coarse
    path, and repairing that portion of the coarse path via linear interpolation
    of distance along a (single) fine path that is chosen to have the lowest
    starting + ending distance, using starting and ending coarse points of the
    interval to repair. The z value of the coarse output in the repaired
    interval would ramp z linearly per distance to match up with coarse z after
    the repaired interval.
* Why use the coarse slicing at all?
  * The coarse transition from first layer to spiral regime is a main
    motivation; not needing to explicitly generate that flat plate to spiral
    path transition strategy in gcode-respiralizer.
  * It's nice to keep the intro line intact without much hassle, and without any
    explicit handling (aside from tolerating multiple coarse paths, 1 for intro
    line and 1 for rest).
  * To at least some extent, having the coarse slicing drive the z lets the
    slicer do whatever is does (or doesn't) do to avoid extruding without any
    layer gap underneath. If the slicer does a terrible job ensuring there's a
    gap, then this algorithm won't fix that, but if the slicer always avoids
    having no gap even when successive coarse perimeters are very different,
    then this algorithm will follow the slicer's lead, just enough to keep the
    slicer's good work here. See also the transition from flat build plate to
    spiral regime.

## Posting issues

Please do post an issue if you run into a problem.

* Please make sure you followed the how-to precisely prior to posting the issue.
* Please include the model or a link to the model if possible.
* Please include a .3mf file for generating the fine horizontal slicing, and a
separate .3mf for generating the coarse spiral slicing.

If this sounds like too much work, just post the issue with whatever info you
can be bothered to include (and thanks!). However, issues which are missing info
needed for a local repro will be treated as lower priority and may be closed
after a while if I don't feel like there's enough info for me to reproduce the
issue myself.

## A note on Rust (rustlang)

As of this writing, I have a lot more experience with C++ than rust. Please feel
free to post a pull request with any specific suggestions (rust-related or not,
but I'm particularly interested in any rust tips/tricks).

I chose rust for this primarily to get more practice with rust, and because
writing it in C++ wouldn't have really made it any quicker/easier to integrate
the idea into slic3r/prusaslicer in the "correct" way (ie. not as a gcode
post-processor). The translation from gcode post-processor to a "correct" way of
achieving essentially the same thing as part of the slicer would/will require
substantial translation of the ideas to earlier stages of slicing/re-slicing
anyway (hopefully).

Any real integration into the slicer would be a whole separate step based on the
ideas, not the specific code.

## gcode-respiralizer endgame

I'm hoping this project will inspire "excellent" vase mode slicing in slic3r /
prusaslicer (regardless of whether this project itself achieves "excellent" vase
mode respiralizing), making this project redundant, at which point this project
can be considered "done". This project isn't intended to be a long-term project.
