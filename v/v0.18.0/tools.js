var tools =
[
    [ "Bag Processing", "bag_processing.html", [
      [ "Package Overview", "bag_processing.html#autotoc_md588", null ],
      [ "Usage Instructions", "bag_processing.html#autotoc_md589", null ],
      [ "Scripts", "bag_processing.html#autotoc_md590", [
        [ "apply_histogram_equalization_to_images", "bag_processing.html#autotoc_md591", null ],
        [ "check_bag_for_gaps", "bag_processing.html#autotoc_md592", null ],
        [ "clock_skew", "bag_processing.html#autotoc_md593", null ],
        [ "csv_join", "bag_processing.html#autotoc_md594", null ],
        [ "get_msg_stats", "bag_processing.html#autotoc_md595", null ],
        [ "rosbag_debayer", "bag_processing.html#autotoc_md596", null ],
        [ "rosbag_detect_bad_topics", "bag_processing.html#autotoc_md597", null ],
        [ "rosbag_fix_all", "bag_processing.html#autotoc_md598", null ],
        [ "rosbag_merge", "bag_processing.html#autotoc_md599", null ],
        [ "rosbag_rewrite_types", "bag_processing.html#autotoc_md600", null ],
        [ "rosbag_sample", "bag_processing.html#autotoc_md601", null ],
        [ "rosbag_splice", "bag_processing.html#autotoc_md602", null ],
        [ "rosbag_topic_filter", "bag_processing.html#autotoc_md603", null ],
        [ "rosbag_trim", "bag_processing.html#autotoc_md604", null ],
        [ "rosbag_verify", "bag_processing.html#autotoc_md605", null ]
      ] ],
      [ "Utilities", "bag_processing.html#autotoc_md606", [
        [ "utilities/bmr_renumber_enum", "bag_processing.html#autotoc_md607", null ]
      ] ],
      [ "Resources", "bag_processing.html#autotoc_md608", null ]
    ] ],
    [ "Calibration", "calibration.html", [
      [ "Package Overview", "calibration.html#autotoc_md609", null ],
      [ "Camera Target Based Intrinsics Calibration", "calibration.html#autotoc_md610", [
        [ "Example Usage", "calibration.html#autotoc_md611", [
          [ "Generate target detections from bagfiles", "calibration.html#autotoc_md612", null ],
          [ "View target detection coverage in image space", "calibration.html#autotoc_md613", null ],
          [ "Calibrate", "calibration.html#autotoc_md614", [
            [ "System requirements and installation", "picoflexx_python.html#autotoc_md223", null ],
            [ "Usage", "picoflexx_python.html#autotoc_md224", null ],
            [ "Getting valid xyz coefficients", "picoflexx_python.html#autotoc_md225", null ],
            [ "Calibration Parameters", "calibration.html#autotoc_md615", null ],
            [ "Run Calibration", "calibration.html#autotoc_md616", null ],
            [ "Calibration Output", "calibration.html#autotoc_md617", null ],
            [ "Judging Calibration Results", "calibration.html#autotoc_md618", null ]
          ] ]
        ] ]
      ] ],
      [ "Usage Instructions", "calibration.html#autotoc_md622", null ],
      [ "Tools", "calibration.html#autotoc_md623", [
        [ "create_undistorted_images", "calibration.html#autotoc_md624", null ],
        [ "run_camera_target_based_intrinsics_calibrator", "calibration.html#autotoc_md625", null ]
      ] ],
      [ "Scripts", "calibration.html#autotoc_md626", [
        [ "calibrate_intrinsics_and_save_results.py", "calibration.html#autotoc_md627", null ],
        [ "copy_calibration_params_to_config.py", "calibration.html#autotoc_md628", null ],
        [ "get_bags_with_topic.py", "calibration.html#autotoc_md629", null ],
        [ "make_error_histograms.py", "calibration.html#autotoc_md630", null ],
        [ "save_images_with_target_detections.py", "calibration.html#autotoc_md631", null ],
        [ "view_all_detections.py", "calibration.html#autotoc_md632", null ]
      ] ]
    ] ],
    [ "GNC Visualizer", "gncvisualizer.html", "gncvisualizer" ],
    [ "IMU Bias Tester", "imubiastester.html", [
      [ "Usage", "imubiastester.html#autotoc_md651", null ],
      [ "Inputs", "imubiastester.html#autotoc_md652", null ],
      [ "Outputs", "imubiastester.html#autotoc_md653", null ]
    ] ],
    [ "Localization Analysis", "localizationanalysis.html", [
      [ "Package Overview", "localizationanalysis.html#autotoc_md654", [
        [ "ROS Mode", "gncvisualizer.html#autotoc_md633", null ],
        [ "DDS Mode", "gncvisualizer.html#autotoc_md634", [
          [ "On-orbit activities", "gncvisualizer.html#autotoc_md635", null ]
        ] ],
        [ "Dependencies", "gncvisualizer.html#autotoc_md636", [
          [ "If used along with the Astrobee Robot Software", "gncvisualizer.html#autotoc_md637", null ],
          [ "If using as a standalone tool", "gncvisualizer.html#autotoc_md638", null ]
        ] ],
        [ "Installing dependencies", "gncvisualizer.html#autotoc_md639", [
          [ "Installing Python", "gncvisualizer.html#autotoc_md640", null ],
          [ "Installing PIP", "gncvisualizer.html#autotoc_md641", null ],
          [ "Installing the RTI connector (DDS Only)", "gncvisualizer.html#autotoc_md642", null ],
          [ "Installing QT in standalone mode", "gncvisualizer.html#autotoc_md643", null ]
        ] ],
        [ "Platform support", "gncvisualizer.html#autotoc_md644", null ],
        [ "ImuBiasTester", "imubiastester.html#autotoc_md650", null ],
        [ "Usage Instructions", "localizationanalysis.html#autotoc_md655", null ]
      ] ],
      [ "Tools", "localizationanalysis.html#autotoc_md656", [
        [ "<tt>convert_depth_msg</tt>", "localizationanalysis.html#autotoc_md657", null ],
        [ "<tt>run_bag_imu_filterer</tt>", "localizationanalysis.html#autotoc_md658", null ],
        [ "<tt>run_depth_odometry_adder</tt>", "localizationanalysis.html#autotoc_md659", null ],
        [ "<tt>run_graph_bag</tt>", "localizationanalysis.html#autotoc_md660", null ],
        [ "<tt>run_imu_bias_tester_adder</tt>", "localizationanalysis.html#autotoc_md661", null ],
        [ "<tt>run_sparse_mapping_pose_adder</tt>", "localizationanalysis.html#autotoc_md662", null ]
      ] ],
      [ "Scripts", "localizationanalysis.html#autotoc_md663", [
        [ "<tt>bag_and_parameter_sweep</tt>", "localizationanalysis.html#autotoc_md664", null ],
        [ "<tt>bag_sweep</tt>", "localizationanalysis.html#autotoc_md665", null ],
        [ "<tt>depth_odometry_parameter_sweep</tt>", "localizationanalysis.html#autotoc_md666", null ],
        [ "<tt>get_average_opt_and_update_times</tt>", "localizationanalysis.html#autotoc_md667", null ],
        [ "<tt>groundtruth_sweep</tt>", "localizationanalysis.html#autotoc_md668", null ],
        [ "<tt>imu_analyzer</tt>", "localizationanalysis.html#autotoc_md669", null ],
        [ "<tt>make_groundtruth</tt>", "localizationanalysis.html#autotoc_md670", null ],
        [ "<tt>make_map</tt>", "localizationanalysis.html#autotoc_md671", null ],
        [ "<tt>parameter_sweep</tt>", "localizationanalysis.html#autotoc_md672", null ],
        [ "<tt>plot_all_results</tt>", "localizationanalysis.html#autotoc_md673", null ],
        [ "<tt>plot_results</tt>", "localizationanalysis.html#autotoc_md674", null ],
        [ "<tt>run_graph_bag_and_plot_results</tt>", "localizationanalysis.html#autotoc_md675", null ]
      ] ]
    ] ],
    [ "Localization Rviz Plugins", "localizationrvizplugins.html", [
      [ "Package Overview", "localizationrvizplugins.html#autotoc_md676", [
        [ "Usage", "localizationrvizplugins.html#autotoc_md677", null ],
        [ "Plugins", "localizationrvizplugins.html#autotoc_md678", null ],
        [ "Depth Odometry Display", "localizationrvizplugins.html#autotoc_md679", null ],
        [ "Imu Augmentor Display", "localizationrvizplugins.html#autotoc_md680", null ],
        [ "Localization Graph Display", "localizationrvizplugins.html#autotoc_md681", null ],
        [ "Localization Graph Panel", "localizationrvizplugins.html#autotoc_md682", null ],
        [ "Pose Display", "localizationrvizplugins.html#autotoc_md683", null ],
        [ "Sparse Mapping Display", "localizationrvizplugins.html#autotoc_md684", null ]
      ] ]
    ] ],
    [ "DDS Profile and Types", "dds_profile.html", null ],
    [ "Interactive Marker Teleop", "interactive_marker_teleop.html", null ],
    [ "Performance Tester", "performance_tester.html", null ]
];