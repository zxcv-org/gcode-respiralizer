# gcode-respiralizer

The project/repo name is gcode-respiralizer (dash).

The binary name is gcode_respiralizer (underscore).

The lib name is gcode_respiralizer_lib (underscores).

## WARNING

CAUTION CAUTION CAUTION

THIS CODE IS A PROTOTYPE AND THE GENERATED GCODE SHOULD BE SCRUTINIZED IN DETAIL
BEFORE ANY ATTEMPT TO PRINT THE GCODE

This code generates gcode, and this code may have bugs, resulting in bad gcode
output, which can be hazardous to your printer, and possibly hazardous to print
unattended in case of any bugs or unforeseen printing issues. While the author
has printed a few models after processing with this code and didn't hit any
significant issues, the author hasn't tested with your model, your printer, your
workflow, etc.

CAUTION CAUTION CAUTION

## Goal / Purpose

Eliminate the vase mode "seam" that results when slicing in vase mode in
prusaslicer.

See the "Detailed Goal / Purpose" section below if you like reading debian
governance docs cover to cover.

## How-To

### Prusaslicer

1. Turn off detect bridging perimeters (to slice faster and avoid bridging flow
   ratio in any part of the spiral). In my experience, with an extrusion width
   that isn't too much larger than the nozzle diameter, and prioritizing quality
   over speed, vase mode prints can bridge fine without these slicing features.
1. Switch from arachne to classic. Leave it classic for the rest of this (vase
   mode uses classic without indicating so in the UI; we need the non-vase
   slicing below to use classic also).
1. Skip this step
   * (TODO: / aspirational / skip for now) "Put path to gcode_respiralizer
     binary in the Print settings -> Post-processing steps" (doesn't work yet)
1. Set to vase mode, accept 1 perimeter etc.
1. Un-check vase mode (but leave 1 perimeter etc).
1. Set retract/detract/extra detract to all off / 0.0mm.
1. Set layer height and first layer height to 0.02mm (~1/10th of the layer
   height you actually want)
1. Position the model exactly where you want it on the build plate. From the
   next step onward, do not move the model on the plate.
1. Slice, and save the gcode file for use below.
   * (TODO: gcode-respiralizer will auto-detect that this is the finely-sliced
     file, and will save a copy for gcode-respiralizer to use later)
1. Set layer height to 0.3mm or 0.2mm (whichever you want the output gcode to
   be). Currently 0.3mm is recommended if you don't care which, but 0.2 should
   work. Layer heights > 0.3 aren't recommended yet.
   * (TODO: auto-detect and auto-tune thresholds based on detected layer
     height.)
1. Check vase mode again.
1. Slice, and save the gcode file for use below.
   * (TODO: gcode-respiralizer will auto-detect that this is the coarse-sliced
     file, and based on filename, will pair it up with the previously-saved
     finely-sliced file)
1. Run "gcode_spiralizer fine.gcode coarse.gcode OUTPUT.gcode". Wait a bit, or a
   while, or a longer while, depending. Look at your favorite task manager /
   system monitor / perfmon "disk" or "fs" tab or similar to see if it's still
   working and not just stuck in an infinite loop.
   * TODO: more incremental status to command line during fine perimeter loading
     and output generation.
1. Open output in gcode analyzer, visually verify that it looks plausible -
   sweep through the layers; check again - it is up to you to verify that you
   want to print this weirdly-generated gcode. In particular, there is no reason
   to believe that the generated gcode will absolutely never go backwards,
   extruding over what it's already printed. Thankfully due to the nature of the
   generated gcode, the z ramping would (probably) still be happening, so
   hopefully the extruder wouldn't skip, but BEWARE.
1. Print, and hopefully verify that the vase mode fudge "seam" is not present,
   and that there aren't any artifacts introduced by this gcode post-processor.

## Theory of Operation (currently somewhat stale)

Do a 1 perimeter normal (not vase) slice of the model using a very low layer
height like 0.03mm (1/10 of 0.3; not a typo). Start by turning on vase mode to
get mostly-correct settings, then turn it off again. Use random seam placement.
Don't use arcs. Call this gcode output "fine".

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

Less stale info:

Also keep points from the closest fine perimeter as a way to get points that are
more optimized for the model/fine geometry at the current z height, to avoid
issues with points from coarse slicing that aren't ideally placed for a place on
the model that's offset in z, for better surface just after (just to the right)
of where the fudged "seam" was.

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

## Detailed Goal / Purpose

This "seam" isn't a normal retraction/detraction layer switch seam, but rather
the result of the slicer internally doing normal constant-z-per-layer slicing,
then ramping z within each layer's path and fudging to (sorta) connect the end
of one layer to the beginning of the next layer. This fudge is the vase mode
"seam".

Fwiw, the first part of each vase perimeter/circuit is the part that has the
largest z offset between the model geometry and the slicer's vase mode output
points, so just to the right (looking in from outside, at a "normal" enough
model for reasonably-human-readable language to make any sense) of the fudged
"seam" is the most "wrong" with respect to model geometry, and arguably this
wrong-ness is what "causes" the fudged seam, since just before the fudged seam,
the slicers vase mode output is quite close to the model geometry (low z
offset). The fudged "seam" is effectively an artifact of the "wrongness" on the
right of the seam. Fixing the seam basicaly requires fixing the "wrongness" on
the right of the seam, and it makes sense to adopt "fixing the wrongness on the
right of the seam" as an explicit goal, mainly to have at least one goal that
cares about conformance to the model geometry.

It's worth noting that "real" vase mode spiral slicing is fraught with gotchas.
For example, selecting the slope of the spiral can increase the overall length
of the perimeter (until it stacks on top of itself and needs to be 1 layer
height up from where it started), which can then require a lower slope, but then
using that lower slope the perimeter is shorter, requiring a higher slope.
There's also the concern/requirement that vase mode models tend to use "steps"
with flat tops (or bottoms) to switch from a detour to a bridge that skips the
detour (or even vice-versa if the model prints in the air for artistic or
comedic effect); a pure spiral would tend to "fall off" such steps, leading to a
much longer bridge that looks wrong and wasn't the model designer's intent. So
any algorithm that hopes to avoid extruding in thin air across the middle of a
detour that's about to disappear has to deviate from a pure spiral and
essentially choose to take the detour or not take the detour, even if the detour
and the non-detour short bridge fall at an inconvenient location.

It may be fair to say that there's not really any one "correct" vase mode
slicing algorithm (aside from perhaps some sort of cost function minimizing
approach where the cost function has the lowest number of preference knobs it
can, while still addressing the vast majority of "reasonable" cases, and somehow
runs super fast; maybe a dynamic programming + A* + simulated annealing thing,
all running on a GPU - but then, could anyone other than the author (of such a
thing) hope to understand it?). Anyway, it depends on the goals.

Here are the goals for gcode-respiralizer:

* Eliminate (or at least greatly reduce) the vase mode "seam".
  * As an instrumental goal, reduce the "wrongness" on the right of that seam.
    * As a goal we get "for free" under the parent instrumental goal, avoid
      output that deviates a lot from the model.
* Don't "jump off steps" / "jump onto steps" (see above).
  * A jump across a horizontal step is considered "deviation" from the model, to
    reduce the degree to which this would otherwise tend to conflict with
    avoiding deviation from the model.
* Do stuff in a way that'd be at least somewhat plausible for slic3r /
  prusaslicer to implement internally (though probably not in the same language,
  and almost certainly not at the same stage of slicing).
* Don't break aspects of current prusaslicer vase mode slicing that work (at
  least for non-hostile cases), such as the transition from flat build plate to
  z ramp regime, and the smoothness of the z ramp within the spiral regime, and
  the ability to have a few flat bottom layers before the spiral starts, and fan
  speed changes, and print speed changes (this list isn't intended to be
  exhaustive, but others may have issues due to not being listed explicitly here
  and being overloooked as a result).
* Where print quality and print speed conflict (if ever), choose print quality.
  Example: more output points is ok if it helps quality, without worrying too
  much about potential stuttering at much higher speeds.
* Compatibilty: Work with prusaslicer output for at least legacy marlin and
  klipper (FW that a MK3S+ can run, since that's what I have locally). Avoid
  taking a dependency on firmware-specific aspects, to hopefully be/remain
  compatible with other slicer firmware settings also (but I have no way to test
  those locally).
* Compatibility: Work with the latest released version of prusaslicer, even if
  that version is non-final (alpha, beta, RC). When not a hassle, also work with
  latest final version (not alpha, beta, RC).
* Where it doesn't conflict with other goals, and doesn't conflict with the
  level of time commitment I'm willing to put into this, make the code not
  completely terrible. To date, this goal is mostly about the "where it doesn't
  conflict with" and it's more like "TODO: make the code not completely
  terrible". For example, there is only one test, and it's only covering a small
  part of the program. That's bad. There should be real tests. But see the next
  paragraph.
* Ease-of-use is a lower-priority goal, but still a goal. Suggestions/issues
  welcome, but even better would be pull requests that don't overlap with the
  ".plan" section at the bottom of this README.md.
* Try not to write a whole slicer (triangle processing, etc), at least for now.
* Be useful to users other than just me (but see next paragraph).

Because the slicer should really handle vase mode slicing better, the
gcode-respiralizer can be considered a prototype / temporary solution / test of
ideas. That said, a primary goal is to be reliable for typical vase mode models,
since that's a prerequisite for demonstrating that the slicer "should" do these
things (or analogous things, or better things) instead of a gcode
post-processor.
