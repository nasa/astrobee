\page imubiastester Imu Bias Tester 

## ImuBiasTester
The imu bias tester provides a way to evaluate the accuracy of imu biases without having bias groundtruth.  It works by relying on some sort of localization groundtruth instead.  The bias tester integrates imu measurements but updates biases as localization estimates are provided.  Thus, if the biases are perfectly estimated by the localizer, the integrated imu measurements should perfectly match localization groundtruth. The nodelet subscribes to the imu and localization state and published a imu bias pose. 

# Inputs
* `/hw/imu`
* `graph_loc/state`

# Outputs
* `imu_bias_tester/pose`
