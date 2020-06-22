\page imagesampler Image Sampler

## Image Sampler Node

The image sampler subscribes to an image topic, then republishes the
images at a different rate and resolution.

Two steps are required to start using the image sampler:


### Configure the image sampler for the desired camera

```
rosservice call /mgt/img_sampler/{nav_cam,dock_cam}/configure \
                  <mode> <framerate> <width> <height> <bitrate>
```
  - `mode` : parameters apply to RECORDING=1, or STREAMING=2 or BOTH=3
  - `framerate` : rate in Hz to publis
  - `with`, `height` : dimension in pixels of the image to publish
  - `bitrate` : only applies to SciCam, defines the image quality

Example to configure the nav_cam to publish half size images every 2 seconds (both for streaming and recording):
```
rosservice call /mgt/img_sampler/nav_cam/configure 0 0.5 640 480 0
```

See [Configure Camera Service](../communications/ff_msgs/srv/ConfigureCamera.srv).

### Enable the image sampler for the desired camera

```
rosservice call /mgt/img_sampler/{nav_cam.dock_cam}/enable <mode> <flag>
```
  - `mode` : parameters apply to RECORDING=1, or STREAMING=2 or BOTH=3
  - `flag` : **true** to enable and **false** to disable the service

Example to turn on streaming only of the nav_cam (using previous camera configuration):
```
rosservice call /mgt/img_sampler/nav_cam/enable 2 true
```

See [EnableCamera Service](../../communications/ff_msgs/srv/EnableCamera.srv).


Then, the node will publish an image at the desired resolution and rate,
with the topic name `<node name>/image`.

The image to read is specified through the command line argument `--input_topic`.
