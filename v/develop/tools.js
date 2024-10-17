var tools =
[
    [ "Bag Processing", "bag_processing.html", [
      [ "Package Overview", "bag_processing.html#autotoc_md617", null ],
      [ "Usage Instructions", "bag_processing.html#autotoc_md618", null ],
      [ "Scripts", "bag_processing.html#autotoc_md619", [
        [ "apply_histogram_equalization_to_images", "bag_processing.html#autotoc_md620", null ],
        [ "check_bag_for_gaps", "bag_processing.html#autotoc_md621", null ],
        [ "clock_skew", "bag_processing.html#autotoc_md622", null ],
        [ "csv_join", "bag_processing.html#autotoc_md623", null ],
        [ "get_msg_stats", "bag_processing.html#autotoc_md624", null ],
        [ "rosbag_debayer", "bag_processing.html#autotoc_md625", null ],
        [ "rosbag_detect_bad_topics", "bag_processing.html#autotoc_md626", null ],
        [ "rosbag_fix_all", "bag_processing.html#autotoc_md627", null ],
        [ "rosbag_merge", "bag_processing.html#autotoc_md628", null ],
        [ "rosbag_rewrite_types", "bag_processing.html#autotoc_md629", null ],
        [ "rosbag_sample", "bag_processing.html#autotoc_md630", null ],
        [ "rosbag_splice", "bag_processing.html#autotoc_md631", null ],
        [ "rosbag_topic_filter", "bag_processing.html#autotoc_md632", null ],
        [ "rosbag_trim", "bag_processing.html#autotoc_md633", null ],
        [ "rosbag_verify", "bag_processing.html#autotoc_md634", null ],
        [ "rosbag_recover", "bag_processing.html#autotoc_md635", null ]
      ] ],
      [ "Utilities", "bag_processing.html#autotoc_md636", [
        [ "utilities/bmr_renumber_enum", "bag_processing.html#autotoc_md637", null ]
      ] ],
      [ "Resources", "bag_processing.html#autotoc_md638", null ]
    ] ],
    [ "Calibration", "calibration.html", [
      [ "Package Overview", "calibration.html#autotoc_md639", null ],
      [ "Camera Target Based Intrinsics Calibration", "calibration.html#autotoc_md640", [
        [ "Example Usage", "calibration.html#autotoc_md641", [
          [ "Generate target detections from bagfiles", "calibration.html#autotoc_md642", null ],
          [ "View target detection coverage in image space", "calibration.html#autotoc_md643", null ],
          [ "Calibrate", "calibration.html#autotoc_md644", [
            [ "System requirements and installation", "picoflexx_python.html#autotoc_md225", null ],
            [ "Usage", "picoflexx_python.html#autotoc_md226", null ],
            [ "Getting valid xyz coefficients", "picoflexx_python.html#autotoc_md227", null ],
            [ "Calibration Parameters", "calibration.html#autotoc_md645", null ],
            [ "Run Calibration", "calibration.html#autotoc_md646", null ],
            [ "Calibration Output", "calibration.html#autotoc_md647", null ],
            [ "Judging Calibration Results", "calibration.html#autotoc_md648", null ]
          ] ]
        ] ]
      ] ],
      [ "Usage Instructions", "calibration.html#autotoc_md652", null ],
      [ "Tools", "calibration.html#autotoc_md653", [
        [ "create_undistorted_images", "calibration.html#autotoc_md654", null ],
        [ "run_camera_target_based_intrinsics_calibrator", "calibration.html#autotoc_md655", null ]
      ] ],
      [ "Scripts", "calibration.html#autotoc_md656", [
        [ "calibrate_intrinsics_and_save_results.py", "calibration.html#autotoc_md657", null ],
        [ "copy_calibration_params_to_config.py", "calibration.html#autotoc_md658", null ],
        [ "get_bags_with_topic.py", "calibration.html#autotoc_md659", null ],
        [ "make_error_histograms.py", "calibration.html#autotoc_md660", null ],
        [ "save_images_with_target_detections.py", "calibration.html#autotoc_md661", null ],
        [ "view_all_detections.py", "calibration.html#autotoc_md662", null ]
      ] ]
    ] ],
    [ "GNC Visualizer", "gncvisualizer.html", "gncvisualizer" ],
    [ "Localization Analysis", "localizationanalysis.html", [
      [ "Package Overview", "localizationanalysis.html#autotoc_md680", [
        [ "ROS Mode", "gncvisualizer.html#autotoc_md663", null ],
        [ "DDS Mode", "gncvisualizer.html#autotoc_md664", [
          [ "On-orbit activities", "gncvisualizer.html#autotoc_md665", null ]
        ] ],
        [ "Dependencies", "gncvisualizer.html#autotoc_md666", [
          [ "If used along with the Astrobee Robot Software", "gncvisualizer.html#autotoc_md667", null ],
          [ "If using as a standalone tool", "gncvisualizer.html#autotoc_md668", null ]
        ] ],
        [ "Installing dependencies", "gncvisualizer.html#autotoc_md669", [
          [ "Installing Python", "gncvisualizer.html#autotoc_md670", null ],
          [ "Installing PIP", "gncvisualizer.html#autotoc_md671", null ],
          [ "Installing the RTI connector (DDS Only)", "gncvisualizer.html#autotoc_md672", null ],
          [ "Installing QT in standalone mode", "gncvisualizer.html#autotoc_md673", null ]
        ] ],
        [ "Platform support", "gncvisualizer.html#autotoc_md674", null ],
        [ "Usage Instructions", "localizationanalysis.html#autotoc_md681", null ]
      ] ],
      [ "Tools", "localizationanalysis.html#autotoc_md682", [
        [ "<tt>convert_depth_msg</tt>", "localizationanalysis.html#autotoc_md683", null ],
        [ "<tt>run_bag_imu_filterer</tt>", "localizationanalysis.html#autotoc_md684", null ],
        [ "<tt>run_depth_odometry_adder</tt>", "localizationanalysis.html#autotoc_md685", null ],
        [ "<tt>run_offline_replay</tt>", "localizationanalysis.html#autotoc_md686", null ],
        [ "<tt>run_imu_bias_tester_adder</tt>", "localizationanalysis.html#autotoc_md687", null ],
        [ "<tt>run_sparse_mapping_pose_adder</tt>", "localizationanalysis.html#autotoc_md688", null ]
      ] ],
      [ "Scripts", "localizationanalysis.html#autotoc_md689", [
        [ "<tt>bag_and_parameter_sweep</tt>", "localizationanalysis.html#autotoc_md690", null ],
        [ "<tt>bag_sweep</tt>", "localizationanalysis.html#autotoc_md691", null ],
        [ "<tt>depth_odometry_parameter_sweep</tt>", "localizationanalysis.html#autotoc_md692", null ],
        [ "<tt>get_average_opt_and_update_times</tt>", "localizationanalysis.html#autotoc_md693", null ],
        [ "<tt>groundtruth_sweep</tt>", "localizationanalysis.html#autotoc_md694", null ],
        [ "<tt>imu_analyzer</tt>", "localizationanalysis.html#autotoc_md695", null ],
        [ "<tt>make_groundtruth</tt>", "localizationanalysis.html#autotoc_md696", null ],
        [ "<tt>make_map</tt>", "localizationanalysis.html#autotoc_md697", null ],
        [ "<tt>parameter_sweep</tt>", "localizationanalysis.html#autotoc_md698", null ],
        [ "<tt>plot_all_results</tt>", "localizationanalysis.html#autotoc_md699", null ],
        [ "<tt>plot_results</tt>", "localizationanalysis.html#autotoc_md700", null ],
        [ "<tt>run_offline_replay_and_plot_results</tt>", "localizationanalysis.html#autotoc_md701", null ]
      ] ]
    ] ],
    [ "Localization Rviz Plugins", "localizationrvizplugins.html", [
      [ "Package Overview", "localizationrvizplugins.html#autotoc_md702", [
        [ "Usage", "localizationrvizplugins.html#autotoc_md703", null ],
        [ "Plugins", "localizationrvizplugins.html#autotoc_md704", null ],
        [ "Depth Odometry Display", "localizationrvizplugins.html#autotoc_md705", null ],
        [ "Imu Augmentor Display", "localizationrvizplugins.html#autotoc_md706", null ],
        [ "Localization Coverage Display", "localizationrvizplugins.html#autotoc_md707", null ],
        [ "Localization Graph Display", "localizationrvizplugins.html#autotoc_md708", null ],
        [ "Localization Graph Panel", "localizationrvizplugins.html#autotoc_md709", null ],
        [ "Pose Display", "localizationrvizplugins.html#autotoc_md710", null ],
        [ "Sparse Mapping Display", "localizationrvizplugins.html#autotoc_md711", null ]
      ] ]
    ] ],
    [ "DDS Profile and Types", "dds_profile.html", null ],
    [ "Interactive Marker Teleop", "interactive_marker_teleop.html", null ],
    [ "Performance Tester", "performance_tester.html", null ]
];