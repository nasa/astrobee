\page theia_map Building a map with Theia

Theia (https://github.com/sweeneychris/TheiaSfM) is a package for
global structure-from-motion (SfM). It may be faster and require less
user effort than the method used in Astrobee (see \ref map_building)
which consists of creating submaps using incremental SfM on sequences
of images that overlap with their immediate neighbors, followed by
merging the submaps. Theia uses "cascade matching" to handle
non-sequential image acquisitions.

The Theia approach was tested on about 700 images acquired at
different times and it did well. It is currently studied as an
alternative to Astrobee's approach.

# Install Theia's prerequisites

It is suggested to use ``conda`` to install Theia's dependencies. The
conda toolset does not require root access and creates a set of
consistent libraries at a custom location in your home directory,
which are largely independent of your particular Linux system.

For the record, the instructions below were tested on Ubuntu 18.04.

Fetch and install ``conda`` from:

    https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html

Restart your shell if it is suggested to do so. 

Create and activate the environment:

    conda create -n theia   
    conda activate theia

Run the following command to install some packages and GCC 11:

    conda install -c conda-forge cmake                \
      gcc_linux-64==11.1.0 gxx_linux-64==11.1.0       \
      lapack blas eigen==3.3.7 suitesparse rapidjson  \
      glog gflags rocksdb OpenImageIO                 \
      ceres-solver=1.14.0=h0948850_10
 
The Ceres package can be quite particular about the version of Eigen
it uses, and some versions of Ceres are not built with ``suitesparse``,
which is a sparse solver that is needed for best performance, so some
care is needed with choosing the versions of the packages.

# Fetch and build Theia

    git clone git@github.com:sweeneychris/TheiaSfM.git
    cd TheiaSfM
    git checkout d2112f1 # this version was tested

Edit the file:

    applications/CMakeLists.txt

and comment out all the lines regarding OpenGL, GLEW, GLUT, and 
view_reconstruction. That visualizer logic is not easy to compile
and is not needed.

Run ``which cmake`` to ensure its version in the ``theia`` environemnt
installed earlier is used. Otherwise run again:

    conda activate theia

Do:

    mkdir build
    cd build
    cmake ..
    make -j 20

This will create the executables ``build_reconstruction`` and
``export_to_nvm_file`` in the ``bin`` subdirectory of ``build``. 

# Hide the conda environment

The conda environment set up earlier will confuse the lookup the
dependencies for the Astrobee libraries. Hence remove the block of
lines starting and ending with ``conda initialize`` which conda added
to your ~/.bashrc, then close and reopen your terminal. Ensure that
the ``env`` command shows no mention of conda.

# Set up the environment for Theia and Astrobee

Add the Theia tools to your path as:

    export PATH=/path/to/TheiaSfM/build/bin:$PATH

Set up the environment for Astrobee, including the robot name. These
should be adjusted as needed:

    export ASTROBEE_BUILD_PATH=$HOME/astrobee
    export ASTROBEE_SOURCE_PATH=$ASTROBEE_BUILD_PATH/src
    source $ASTROBEE_BUILD_PATH/devel/setup.bash
    export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
    export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
    export ASTROBEE_WORLD=iss
    export ASTROBEE_ROBOT=bumble

Add some Astrobee tools to the path as well:

    export PATH=$ASTROBEE_BUILD_PATH/devel/lib/sparse_mapping:$PATH

Please note that if the robot name, as specified in
``ASTROBEE_ROBOT``, is incorrect, you will get poor results.

# Prepare the data

The data preparation is the same as used with the usual Astrobee
map-building software documented in \ref map_building. To summarize,
the steps are as follows.

Extract the data from the bag:

    $ASTROBEE_BUILD_PATH/devel/lib/localization_node/extract_image_bag \
      bagfile.bag -use_timestamp_as_image_name                         \
      -image_topic /hw/cam_nav -output_directory image_dir

If the images were recorded with the image sampler, the nav_cam image
topic needs to be changed to:

    /mgt/img_sampler/nav_cam/image_record

The ``-use_timestamp_as_image_name`` option is not strictly necessary,
but it is helpful if a lot of datasets needs to be processed
jointly. With it, the image filename is the timestamp (in
double-precision seconds since epoch), which provides for a rather
unique name, as compared to using the image index in the bag without
that option.

Partition image files into sequences:
    rosrun sparse_mapping partition_image_sequences image_directory_name config_path

Partitions a sequentially ordered set of image files into valid, rotation, and invalid sequences. During bundle adjustment, it is useful to avoid adding pure rotation sequences initially as these cause errors for monocular systems. The resulting sequences can be individually bundle-adjusted and merged as described later, generally starting with the valid (non-rotation) sequences and optionally adding rotations at the end once enough matches exist in the map. See 'rosrun sparse_mapping partition_image_sequences -h' for more usage details, options, and instructions.

Remove low movement images:
    rosrun sparse_mapping remove_low_movement_images image_directory_name

This will delete subsequent images with low movement from that directory to improve mapping performance and accuracy. 

These are non-reversible operations, so they should be invoked on a copy
of the images.

If possible, the robot should have some translation motion (in addition to any rotation) when
the data is acquired.

Removing low movement images and initially only adding valid movement images helps the accuracy of bundle adjustment, which struggles to optimize camera poses with small or no translation changes.

Put the selected images in a list:

    ls image_dir/*jpg > image_list.txt
 
# Run the Astrobee wrapper around the Theia tools

    python $ASTROBEE_SOURCE_PATH/localization/sparse_mapping/tools/build_theia_map.py \
       --output_map theia.map --work_dir theia_work --image_list image_list.txt

This will take care of preparing everything Theia needs, will run it,
and will export the resulting map to Astrobee's expected format. 

It is suggested to first run this tool on a small subset of the data,
perhaps made up of 10 images. A 700-image dataset may take perhaps 6
hours on a machine with a couple of dozen cores and use up perhaps 20
GB of RAM.

The obtained map can be examined with ``nvm_visualize``, as described
in \ref sparsemapping.

The work directory can be deleted later.

# Command line options

This tool has the following command-line options:

    --theia_flags: The flags to pass to Theia. If not specified, use
      localization/sparse_mapping/theia_flags.txt in the Astrobee repo.
    --image_list: The list of distorted (original) nav cam images to
      use, one per line.
    --output_map: The resulting output map.
    --skip_rebuilding: Do not rebuild the map after importing it from
      Theia.
    --work_dir: A temporary work directory to be deleted by the user
      later.
    --keep_undistorted_images: Do not replace the undistorted images
      Theia used with the original distorted ones in the sparse map
      imported from Theia. This is for testing purposes.
    --help: Show this help message and exit.

# Next steps

This map will need to be registered and visualized as described in
\ref map_building.

That page also has information for how the map can be rebuilt to use
BRISK features, and how it can be validated for localization by
playing a bag against it.

The \ref import_map program is used by this tool to import a sparse
map from Theia's format.
