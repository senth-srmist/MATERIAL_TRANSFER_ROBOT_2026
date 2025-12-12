#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace fs = std::filesystem;

class ArucoMarkerGenerator : public rclcpp::Node
{
public:
  ArucoMarkerGenerator() : Node("aruco_marker_generator")
  {
    // Declare parameters with defaults
    this->declare_parameter<int>("marker_count", 1);
    this->declare_parameter<double>("marker_size", 1.0);
    this->declare_parameter<std::string>("unit", "ft");
    this->declare_parameter<std::string>("package_name", "zed_aruco_localization");
    
    // Get parameters
    int marker_count = this->get_parameter("marker_count").as_int();
    double marker_size = this->get_parameter("marker_size").as_double();
    std::string unit = this->get_parameter("unit").as_string();
    std::string package_name = this->get_parameter("package_name").as_string();
    
    // Get package path and construct absolute paths
    std::string package_path = getPackagePath(package_name);
    std::string markers_dir = package_path + "/markers";
    std::string config_file = package_path + "/config/aruco_loc.yaml";
    
    // Fixed parameters
    int image_size = 400;
    std::string format = "png";
    
    // Generate sequential IDs starting from 0
    std::vector<long> marker_ids;
    for (int i = 0; i < marker_count; i++) {
      marker_ids.push_back(i);
    }
    
    RCLCPP_INFO(this->get_logger(), "Package path: %s", package_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Generating %d ArUco markers...", marker_count);
    RCLCPP_INFO(this->get_logger(), "Marker size: %.2f %s", marker_size, unit.c_str());
    RCLCPP_INFO(this->get_logger(), "IDs: 0 to %d", marker_count - 1);
    
    // Create markers directory if it doesn't exist
    fs::create_directories(markers_dir);
    
    // Generate markers
    generateMarkers(marker_ids, image_size, format, markers_dir);
    
    // Update config file
    updateConfig(config_file, marker_count, marker_size, marker_ids);
    
    RCLCPP_INFO(this->get_logger(), "✓ Marker generation complete!");
    RCLCPP_INFO(this->get_logger(), "✓ Config file updated: %s", config_file.c_str());
  }

private:
  std::string getPackagePath(const std::string& package_name)
  {
    // Try to find the package in the workspace
    std::string cmd = "ros2 pkg prefix " + package_name + " 2>/dev/null";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute ros2 pkg prefix command");
      return "";
    }
    
    char buffer[512];
    std::string result = "";
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      result = buffer;
      // Remove trailing newline
      result.erase(result.find_last_not_of(" \n\r\t") + 1);
    }
    pclose(pipe);
    
    if (result.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Package %s not found", package_name.c_str());
      return "";
    }
    
    // Go from install/package_name to src/package_name
    // Typical path: /workspace/ros_ws/install/zed_aruco_localization
    // We want: /workspace/ros_ws/src/zed_aruco_localization
    size_t install_pos = result.find("/install/");
    if (install_pos != std::string::npos) {
      result = result.substr(0, install_pos) + "/src/" + package_name;
    }
    
    return result;
  }
  
  void generateMarkers(const std::vector<long>& marker_ids, int image_size, 
                       const std::string& format, const std::string& markers_dir)
  {
    // Create ArUco dictionary (6x6_1000)
    cv::Ptr<cv::aruco::Dictionary> dictionary = 
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    
    for (long marker_id : marker_ids) {
      cv::Mat marker_image;
      
      // Use drawMarker for OpenCV 4.x compatibility
      cv::aruco::drawMarker(dictionary, marker_id, image_size, marker_image, 1);
      
      std::stringstream filename;
      filename << markers_dir << "/6x6_1000-" << marker_id << "." << format;
      
      if (format == "png" || format == "jpg") {
        cv::imwrite(filename.str(), marker_image);
        RCLCPP_INFO(this->get_logger(), "  Generated: %s", filename.str().c_str());
      } else if (format == "svg") {
        saveAsSVG(marker_image, filename.str(), marker_id);
        RCLCPP_INFO(this->get_logger(), "  Generated: %s", filename.str().c_str());
      }
    }
  }
  
  void saveAsSVG(const cv::Mat& marker_img, const std::string& filepath, int marker_id)
  {
    int size = marker_img.rows;
    
    std::ofstream svg_file(filepath);
    svg_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg_file << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" ";
    svg_file << "viewBox=\"0 0 " << size << " " << size << "\" ";
    svg_file << "width=\"" << size << "\" height=\"" << size << "\">\n";
    svg_file << "  <rect width=\"" << size << "\" height=\"" << size << "\" fill=\"white\"/>\n";
    
    // Add black squares for marker pixels
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (marker_img.at<uchar>(i, j) == 0) {  // Black pixel
          svg_file << "  <rect x=\"" << j << "\" y=\"" << i 
                   << "\" width=\"1\" height=\"1\" fill=\"black\"/>\n";
        }
      }
    }
    
    svg_file << "</svg>\n";
    svg_file.close();
  }
  
  void updateConfig(const std::string& config_file, int marker_count, 
                    double marker_size, const std::vector<long>& marker_ids)
  {
    try {
      YAML::Node config = YAML::LoadFile(config_file);
      
      // Update general parameters
      config["/**"]["ros__parameters"]["general"]["marker_count"] = marker_count;
      config["/**"]["ros__parameters"]["general"]["marker_size"] = marker_size;
      
      // Remove old marker definitions (keep only marker_000 as template)
      YAML::Node params = config["/**"]["ros__parameters"];
      std::vector<std::string> keys_to_remove;
      for (auto it = params.begin(); it != params.end(); ++it) {
        std::string key = it->first.as<std::string>();
        if (key.find("marker_") == 0 && key != "marker_000") {
          keys_to_remove.push_back(key);
        }
      }
      for (const auto& key : keys_to_remove) {
        params.remove(key);
      }
      
      // Add new marker definitions
      for (size_t i = 0; i < marker_ids.size(); i++) {
        std::stringstream marker_key;
        marker_key << "marker_" << std::setfill('0') << std::setw(3) << i;
        
        YAML::Node marker;
        marker["aruco_id"] = static_cast<int>(marker_ids[i]);
        marker["position"] = std::vector<double>{0.0, 0.0, 0.0};
        marker["orientation"] = std::vector<double>{0.0, 0.0, 0.0};
        
        params[marker_key.str()] = marker;
      }
      
      // Update the config with modified params
      config["/**"]["ros__parameters"] = params;
      
      // Write updated config
      std::ofstream fout(config_file);
      fout << "# config/aruco_loc.yaml\n";
      fout << "# Parameters for ArUco localization\n";
      fout << "# Auto-generated - Edit with caution\n\n";
      fout << "---\n";
      fout << config;
      fout.close();
      
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "YAML Error: %s", e.what());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoMarkerGenerator>();
  rclcpp::shutdown();
  return 0;
}
