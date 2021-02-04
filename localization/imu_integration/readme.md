\page imuintegration Imu Integration

# Package Overview
The imu integration package integrates imu measurements using the gtsam PreintegratedCombinedMeasurements (pim) class.  As measurements are added to the pim, their acceleration and angular velocities are integrated after removing imu biases and uncertainties are propogated accordingly (_Dellaert, The new Imu Factor, https://github.com/borglab/gtsam/blob/develop/doc/ImuFactor.pdf_).  For efficieny and particulary for use in an optimization procedure, the pim uses the initial biases and velocity estimate and assumes these are constant for the integration procedure.  The pim also assumes gravity is constantly oriented for the integration procedure, which can lead to accumulated errors if there are large orientation changes before reinitializing the pim.  Therefore there is a tradeoff of efficiency and accuracy, where the most accurate integration would reset the pim after each measurement integration.  

## Important Classes
# ImuIntegrator
Maintains a history of measurements and allows for the create of pims using different sets of measurements.

# LatestImuIntegrator
Adds on to the ImuIntegrator by keeping track of the latest integrated measurement and allowing for the integration of the latest measurements up to a certain time.

# Filters
There are various filters provided that enable filtering of imu data before it is added to an imu integrator.  Filter params are provided in their header files, and vary by filter order and cutoff frequency.  Each butterworth filter is a lowpass filter.
