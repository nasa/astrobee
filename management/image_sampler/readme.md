\defgroup imagesampler Image Sampler
\ingroup management

The image sampler subscribes to an image topic, then republishes the
images at a different rate and resolution.

To start using the image sampler, you must call the `<node name>/configure` service,
with the following fields:

| Field | Type  | Description |
|:-------------|:------------- |:------------|
| enable | bool | Enables or disables the image sampling. |
| width | int | The width of the output image. |
| height | int | The height of the output image. |
| rate | float | The rate to publish the output image, in Hz. |

Then, the node will publish an image at the desired resolution and rate,
with the topic name `<node name>/image`.

The image to read is specified through the command line argument `--input_topic`.
