// main function
#include <ff_common/init.h>
#include <localization_analysis/map_matcher.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp> 


namespace po = boost::program_options;
namespace lc = localization_common;




int main(int argc, char** argv) {
    std::string output_bagfile;
    std::string image_topic;
    std::string robot_config_file;
    std::string world;
    std::string config_path_prefix;
    std::string save_noloc_imgs;
    po::options_description desc("Matches images to provided map and saves matches features and poses to a new bag file");
    desc.add_options()("help,h", "produce help message")("bagfile,b", po::value<std::string>()->required(),
                                                        "Input bagfile containing image messages.")(
        "map-file,m", po::value<std::string>()->required(), "Map file")(
        "config-path,c", po::value<std::string>()->required(), "Path to config directory.")(
        "image-topic,i", po::value<std::string>(&image_topic)->default_value("mgt/img_sampler/nav_cam/image_record"),
        "Image topic")("robot-config-file,r",
                    po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
                    "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name")(
        "output-bagfile,o", po::value<std::string>(&output_bagfile)->default_value(""),
        "Output bagfile, defaults to input_bag + _map_matches.bag")(
        "config-path-prefix,p", po::value<std::string>(&config_path_prefix)->default_value(""), "Config path prefix")
        ("save-noloc-imgs,s", po::value<std::string>(&save_noloc_imgs)->default_value("")->implicit_value(""),
        "Save non-localized images to a bag, defaults to input_bag + _nonloc_imgs.bag");
    po::positional_options_description p;
    p.add("bagfile", 1);
    p.add("map-file", 1);
    p.add("config-path", 1);
    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        if (vm.count("help") || (argc <= 1)) {
        std::cout << desc << "\n";
        return 1;
        }
        po::notify(vm);
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    const std::string input_bag = vm["bagfile"].as<std::string>();
    const std::string map_file = vm["map-file"].as<std::string>();
    const std::string config_path = vm["config-path"].as<std::string>();

    // Only pass program name to free flyer so that boost command line options
    // are ignored when parsing gflags.
    int ff_argc = 1;
    ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

    if (!boost::filesystem::exists(input_bag)) {
        LogFatal("Bagfile " << input_bag << " not found.");
    }

    if (!boost::filesystem::exists(map_file)) {
        LogFatal("Map file " << map_file << " not found.");
    }

    boost::filesystem::path input_bag_path(input_bag);
    if (vm["output-bagfile"].defaulted()) {
        boost::filesystem::path output_bag_path =
        boost::filesystem::current_path() / boost::filesystem::path(input_bag_path.stem().string() + "_map_matches.bag");
        output_bagfile = output_bag_path.string();
    }

    if (!vm["save-noloc-imgs"].defaulted() && save_noloc_imgs.empty()) {
        save_noloc_imgs =
        boost::filesystem::current_path().string() + "/" + input_bag_path.stem().string() + "_nonloc_imgs.bag";
    }
    lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
    config_reader::ConfigReader config;
    localization_analysis::MapMatcher map_matcher(input_bag, map_file, image_topic, output_bagfile, config_path_prefix,
                                                    save_noloc_imgs);

    // Load the JPG image file
    std::string filename = "/srv/novus_1/amoravar/data/images/latest_map_imgs/2020-09-24/1599237349.9658780.jpg";//"/home/lmao/Documents/1595953751.0356236.jpg";*/ "/home/lmao/Documents/1595953736.4697478.jpg";
    std::cout<<filename<<std::endl;

    // test query the database
    // std::string image = "/home/lmao/Documents/1595953751.0356236.jpg";
    cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    if (image.empty()) {
        ROS_ERROR_STREAM("Failed to load image: " << filename);
        return 1;
    }

    // Convert the OpenCV image to sensor_msgs::ImageConstPtr
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    // Print the image message details
    ROS_INFO("Image message created:");
    ROS_INFO_STREAM("Width: " << image_msg->width);
    ROS_INFO_STREAM("Height: " << image_msg->height);
    ROS_INFO_STREAM("Encoding: " << image_msg->encoding);
    ROS_INFO_STREAM("Step: " << image_msg->step);

    ff_msgs::VisualLandmarks vl_msg;


    // Perform further processing with the image message...
    if (map_matcher.GenerateVLFeatures(image_msg, vl_msg)) {
        ROS_INFO("VL features generated.");
    } else {
        ROS_ERROR("Failed to generate VL features.");
    }

    return 0;
}
