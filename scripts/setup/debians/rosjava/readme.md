# Java Generator for ROS Noetic

These scripts are meant to patch and repackage the Java Generator for ROS
Noetic.

Generated Debians include:

- ros-noetic-rosjava-build-tools
- ros-noetic-rosjava-bootstrap
- ros-noetic-genjava
- ros-noetic-rosjava-messages

These are based on official ros-kinetic versions. Minimal patches have been
applied to make them compatible with Astrobee usage.

These Debians are not meant for general rosjava use. They are made with
Astrobee-specific needs in mind.

## Notes

### Building multiple times

Depending on the system, some of these Debians may not be able to compile again
if their package is installed. If you need to recompile these Debians for some
reason, please remove them before:

```shell
sudo apt remove ros-noetic-rosjava-messages ros-noetic-genjava ros-noetic-rosjava-bootstrap ros-noetic-rosjava-build-tools
```

### Build-tools and Python setuptools

The build-tools Debian may have issues compiling if the pip3 setuptools
package does not match the default Ubuntu 20 version.

**If pip3 is not installed in your system this may not apply to you.**

```shell
# Versions should match (default: 45.2.0)
pip3 list | grep setuptools
apt show python3-setuptools
```

If they don't match, adjust the pip3 version. Example:

```shell
pip3 install setuptools==45.2.0 && pip3 install -U testresources
```