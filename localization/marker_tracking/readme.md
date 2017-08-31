\defgroup markertracking Marker Tracking
\ingroup localization

This folder provides a library for detecting AR tags using ALVAR.

# Library

The library, in the src and include directories, provides a class
[MarkerCornerDetector](@ref marker_tracking::MarkerCornerDetector) which takes as input an image and a camera calibration.
It then returns the detected marker IDs and the locations of their detected corners
in the image.

# ROS Node

The ROS node takes images and outputs AR tag features.

## Inputs

* Camera Images (topic specified on command line, via `--image_topic`)
* AR Tag Specification (specified on command line, via `--artag_file`)

## Outputs

* `/localization/ar_tags/features`
* `/localization/ar_tags/registration`

# Overhead Tracking Node

This node is for use on spheresgoat in the granite lab. It reads images from the overhead
camera, detects an AR tag on the robot, and publishes the ground truth robot pose to
`/ground_truth`.

The node is configured in `geometry.config` and `overhead_tracker.config`.

# Marker file usage

## 2D definitions
AR markers are specified in a config file (Lua). A typical example is
`dock_markers_print`.

The marker file should define at least the following variables:
 - `ar_resolution` : defines the resolution (address space) of the marker
 - `ar_margin` : defines the outer black square thickness in cell size unit
 - `drawing_width`, `drawing_height` : width and height of the drawing (in drawing unit)
 - `markers` : table of markers specs with:
   - `id` : id of the markers
   - `size` : size of the outer square defining the markers
   - `pos` : table of x / y coordinate of the maker (drawing units)
   - `white_margin` : optional argument specifying a white margin to add around the marker (if not printed on white sheet)

The marker positions are defined in the regular SVG coordinate system: X positive to the right, Y positive down.

Since the SVG default drawing origin is the top left corner of the virtual sheet, it is possible to define an offset to place the markers at a specific location on the sheet:
  - `drawing_xoffset`, `drawing_yoffset` : x / y offset to apply to all drawing elements.

Optional variables are:
  - `drawing_unit` : default is 'mm'
  - `labels` : table of labels
  - `text_font` : define the font to use for labels (drawing wise)
  - `holes` : table of holes markers
  - `black_background` : bool to specify that the sheet should be black_background

## 3D definitions

The coordinates of the AR markers are defined in a config file (Lua). A typical
example is `granite_lab_markers.config`.

The config file needs to define one variable:
  - `markers_world` : table of marker specs
    - `id` : id of the marker
    - `top_left` : 3D coordinate of the marker top left corner
    - `top_right` : 3D coordinate of the marker top left corner
    - `bottom_left` : 3D coordinate of the marker top left corner

Currently `granite_lab_markers.config` uses the `matrix.lua` transform to
compute the 3D coordinates of the AR markers in the world frame from their
2D definition on the dock.

# Generate dock targets

`<ars_src_dir>` Top of the Astrobee Robotic Software (ARS) source tree
`<ars_build_dir>` Directory where the software has been compiled, alternatively, it could be the install path of ARS.

Make sure the environment variable `ASTROBEE_CONFIG_DIR` as been correctly set to `<ars_src_dir>/astrobee/config` (or install path).
```
<ars_build_dir>/bin/generate_svg_markers dock_markers_print.config target2print.svg
```

# Others

## Important config files
  - `dock_markers_specs.config` : specification of the markers in 2D
  - `dock_markers_print.config` : includes dock_markers_specs, and adds printing features (offset on paper, labels, etc.)
  - `granite_lab_markers.config` : includes dock_makers_specs, and generate coordinates of the markers in 3D from the dock position and orientation. The variable `markers_world` contains the same list of markers than `markers`, but with the computed 3D coordinates.

## Old XML format and new "config" format

The AR markers were originally described with an XML file. These files can still be processed with the `marker_tracking/src/arxmlio.cc` parser using `LoadARTagLocation`.

The new system standardize on the Lua config file, and at the same time offers much more flexibilty since positions can be programatically computed. The new parser is in `marker_tracking/src/arconfigio.cc` and use the function `LoadARTagsConfig`.

A converter (also acting as a simple test program) is available: `marker_config2xml`.
