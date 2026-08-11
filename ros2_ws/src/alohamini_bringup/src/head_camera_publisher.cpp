#include <algorithm>
#include <chrono>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace
{

namespace fs = std::filesystem;

bool ends_with(const std::string &value, const std::string &suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool is_video_node(const fs::path &path)
{
  const std::string name = path.filename().string();
  return name.size() > 5 && name.compare(0, 5, "video") == 0 &&
         std::all_of(
    name.begin() + 5, name.end(),
    [](char value) { return std::isdigit(static_cast<unsigned char>(value)) != 0; });
}

int video_node_number(const fs::path &path)
{
  try {
    return std::stoi(path.filename().string().substr(5));
  } catch (const std::exception &) {
    return 0;
  }
}

std::vector<std::string> camera_device_candidates(const std::string &device)
{
  std::vector<std::string> candidates;
  std::set<std::string> seen;

  const auto add_candidate = [&candidates, &seen](const std::string &candidate) {
      if (!candidate.empty() && seen.insert(candidate).second) {
        candidates.push_back(candidate);
      }
    };

  std::error_code ec;
  const fs::path configured_path(device);
  if (fs::exists(configured_path, ec)) {
    add_candidate(device);
    const fs::path resolved = fs::weakly_canonical(configured_path, ec);
    if (!ec && !resolved.empty()) {
      add_candidate(resolved.string());
    }
  }

  // Docker may expose /dev/video2 but omit the host's /dev/am_camera_* symlink.
  // The V4L2 name is available in the shared sysfs tree and is more stable than
  // relying on a hard-coded video number.
  const std::string camera_name = configured_path.filename().string();
  if (camera_name.compare(0, 10, "am_camera_") == 0) {
    std::vector<fs::path> video_nodes;
    fs::directory_iterator dev_iterator("/dev", ec);
    if (!ec) {
      for (const auto &entry : dev_iterator) {
        if (entry.is_character_file(ec) && is_video_node(entry.path())) {
          video_nodes.push_back(entry.path());
        }
      }
    }
    std::sort(
      video_nodes.begin(), video_nodes.end(),
      [](const fs::path &left, const fs::path &right) {
        return video_node_number(left) < video_node_number(right);
      });

    for (const auto &video_node : video_nodes) {
      std::ifstream name_file(
        "/sys/class/video4linux/" + video_node.filename().string() + "/name");
      std::string v4l2_name;
      if (name_file && std::getline(name_file, v4l2_name) &&
        (v4l2_name == camera_name || v4l2_name.rfind(camera_name + ":", 0) == 0))
      {
        add_candidate(video_node.string());
      }
    }
  }

  // Keep retrying the configured path if the camera is hot-plugged after startup.
  if (candidates.empty()) {
    add_candidate(device);
  }

  return candidates;
}

}  // namespace

class HeadCameraPublisher : public rclcpp::Node
{
public:
  HeadCameraPublisher()
  : Node("head_camera_publisher")
  {
    image_topic_base_ = declare_parameter<std::string>("image_topic", "/head_camera/image_raw");
    device_ = declare_parameter<std::string>("device", "/dev/am_camera_forward");
    frame_id_ = declare_parameter<std::string>("frame_id", "head_camera");
    fps_ = std::max(1.0, declare_parameter<double>("fps", 20.0));
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    jpeg_quality_ = static_cast<int>(
      std::clamp<long>(
        declare_parameter<long>("jpeg_quality", 80L), 1L, 100L));
    reconnect_period_sec_ = std::max(0.5, declare_parameter<double>("reconnect_period_sec", 2.0));

    publish_topic_ = ends_with(image_topic_base_, "/compressed")
                       ? image_topic_base_
                       : image_topic_base_ + "/compressed";

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    // RViz's image_transport subscriber uses Reliable QoS by default.  A
    // Best-Effort publisher would be incompatible with that subscriber, so
    // keep the queue bounded but use Reliable for the compressed stream.
    qos.reliable();
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(publish_topic_, qos);

    RCLCPP_INFO(
      get_logger(),
      "Head camera publisher ready: device=%s base_topic=%s compressed_topic=%s fps=%.1f",
      device_.c_str(), image_topic_base_.c_str(), publish_topic_.c_str(), fps_);

    open_camera();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / fps_));
    timer_ = create_wall_timer(period, std::bind(&HeadCameraPublisher::tick, this));
  }

private:
  void open_camera()
  {
    last_open_attempt_ = now();
    capture_.release();

    const auto candidates = camera_device_candidates(device_);
    for (const auto &candidate : candidates) {
      // A V4L2 device must be opened through V4L2.  Falling back to CAP_ANY
      // makes OpenCV try GStreamer and the image-sequence backend on /dev/*,
      // producing noisy but unhelpful errors whenever a camera is temporarily
      // unavailable (for example, while another process owns it).
      const bool is_device_path = candidate.rfind("/dev/", 0) == 0;
      if (capture_.open(candidate, cv::CAP_V4L2) ||
        (!is_device_path && capture_.open(candidate, cv::CAP_ANY)))
      {
        active_device_ = candidate;
        break;
      }
    }

    if (!capture_.isOpened()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to open head camera device %s; retrying every %.1f s",
        device_.c_str(), reconnect_period_sec_);
      return;
    }

    if (width_ > 0) {
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    }
    if (height_ > 0) {
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    }
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    capture_.set(cv::CAP_PROP_FPS, fps_);

    const double actual_width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actual_height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actual_fps = capture_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(
      get_logger(),
      "Opened %s (requested %s) at %.0fx%.0f @ %.1f fps",
      active_device_.c_str(), device_.c_str(), actual_width, actual_height, actual_fps);
  }

  void tick()
  {
    const auto stamp = now();

    if (!capture_.isOpened()) {
      if ((stamp - last_open_attempt_).seconds() >= reconnect_period_sec_) {
        open_camera();
      }
      return;
    }

    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to read a frame from %s; reopening",
        device_.c_str());
      capture_.release();
      last_open_attempt_ = stamp;
      return;
    }

    if (width_ > 0 && height_ > 0 && (frame.cols != width_ || frame.rows != height_)) {
      cv::resize(frame, frame, cv::Size(width_, height_), 0.0, 0.0, cv::INTER_AREA);
    }

    std::vector<uchar> encoded;
    const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", frame, encoded, encode_params)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to JPEG encode a frame from %s",
        device_.c_str());
      return;
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data.assign(encoded.begin(), encoded.end());
    publisher_->publish(msg);
  }

  std::string image_topic_base_;
  std::string publish_topic_;
  std::string device_;
  std::string active_device_;
  std::string frame_id_;
  double fps_{20.0};
  double reconnect_period_sec_{2.0};
  int width_{640};
  int height_{480};
  int jpeg_quality_{80};
  cv::VideoCapture capture_;
  rclcpp::Time last_open_attempt_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadCameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
