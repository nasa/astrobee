\page theia_map Building a map with Theia

Theia (https://github.com/sweeneychris/TheiaSfM) is a package for
global structure-from-motion (SfM). It may be faster and require less
user effort than the method used in Astrobee which consists of
creating submaps using incremental SfM on sequences of images that
overlap with their immediate neighbors, followed by merging the
submaps. Theia uses "cascade matching" to handle non-sequential image
acquisitions.

The Theia approach was tested on about 700 images acquired at
different times and it did well. It is currently studied as an
alternative to Astrobee's approach.

# Install Theia's prerequisites

Fetch and install ``conda`` from:

    https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html

Restart your shell if it is suggested to do so. 

Create and activate the environment:

    conda create -n theia   
    conda activate theia

Run the following command to install some packages and GCC 11:

  conda install -c conda-forge cmake                \
     gcc_linux-64==11.1.0 gxx_linux-64==11.1.0      \
     lapack blas eigen==3.3.7 suitesparse rapidjson \
     glog gflags rocksdb OpenImageIO                \
     ceres-solver=1.14.0=h0948850_10 
 
The Ceres package can be quite particular about the version of Eigen
it uses, and some versions of Ceres are not built with suitesparse
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
installed earlier is used. Otherwise run again ``conda activate
theia``.  Do:

  mkdir build
  cd build
  cmake ..
  make -j 20

This will create the executables ``build_reconstruction`` and
``export_to_nvm_file`` in the ``bin`` subdirectory of ``build``. That
directory needs to be added to the PATH.

# Run the Astrobee wrapper around the Theia tools

The conda environment set up earlier will confuse the lookup the
dependencies for the Astrobee libraries. Hence the lines ``conda`` added
to one's ``.bashrc`` should be removed, the bash shell restarted, and
one should ensure that the ``env`` command has no mentions of conda.

Set the environment. The following lines should be adjusted as needed,
especially the robot name:

  export ASTROBEE_SOURCE_PATH=$HOME/projects/astrobee/src
  source $ASTROBEE_SOURCE_PATH/../devel/setup.bash
  export ASTROBEE_RESOURCE_DIR=$ASTROBEE_SOURCE_PATH/astrobee/resources
  export ASTROBEE_CONFIG_DIR=$ASTROBEE_SOURCE_PATH/astrobee/config
  export ASTROBEE_WORLD=iss
  export ASTROBEE_ROBOT=bumble

  python $ASTROBEE_SOURCE_PATH/localization/sparse_mapping/tools/build_theia_map.py \
     --output_map theia.map --work_dir theia_work --image_list image_list.txt

This will take care of preparing everything Theia needs, will run it,
and will export the resulting map to Astrobee's expected format. This
map will need to be registered and visualized as described in other
documentation. The work directory can be deleted later.

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

# Auxiliary import_map tool

This tool is used to import a map from the NVM format, which Theia
exports to. These operations are done automatically by the
``build_theia_map.py'' tool. This documentation is provided for
reference only.
 
An NVM map exported by Theia (or some other SfM tool) can be saved as
an Astrobee sparse map with the command::

    astrobee/devel/lib/sparse_mapping/import_map                             \
      -undistorted_camera_params "wid_x wid_y focal_len opt_ctr_x opt_ctr_y" \
      <undistorted images>                                                   \
      -input_map map.nvm -output_map map.map
 
This assumes that the images were acquired with the nav camera of the
robot given by $ASTROBEE_ROBOT and undistorted with the Astrobee
program ``undistort_image``. The undistorted camera parameters to use
should be as printed on the screen by ``undistort_image``.

If desired to replace on importing the undistorted images with the
original distorted ones, as it is usually expected of a sparse map,
the above command should be called instead as::
  
    astrobee/devel/lib/sparse_mapping/import_map \
      -undistorted_images_list undist_list.txt   \
      -distorted_images_list dist_list.txt       \
      -input_map map.nvm -output_map map.map

Here, the files undist_list.txt and dist_list.txt must have one image
per line and be in one-to-one correspondence. It is important that
both undistorted and distorted images be specified, as the former are
needed to look up camera poses and other data in the .nvm file before
being replaced with the distorted ones.
