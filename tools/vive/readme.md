# Configure udev and user permissions

Add a file called /et/udev/rules.d/80-htc-vive.rules with the following:

    KERNEL=="hidraw*", ATTRS{idVendor}=="0bb4", MODE="0666"
    KERNEL=="hidraw*", ATTRS{idVendor}=="28de", MODE="0666"
    KERNEL=="hidraw*", ATTRS{idVendor}=="0424", MODE="0666"

Ensure that your user is added to the plugdev Linux group.