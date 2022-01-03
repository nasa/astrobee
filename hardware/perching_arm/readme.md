\page perching_arm Perching arm

## Re-initialize service

In case we forget to power on the arm bay before starting fsw, it is
possible to re-initialize the perching arm, trying to re-connect with:

    rosservice call /hw/arm/enable_arm "enabled: true"