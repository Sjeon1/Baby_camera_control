#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class RtspImagePublisher : public rclcpp::Node {
 public:
  RtspImagePublisher() : Node("rtsp_image_publisher") {
    declare_parameter<std::string>("rtsp_url", "");
    declare_parameter<std::string>("host", "");
    declare_parameter<std::string>("username", "");
    declare_parameter<std::string>("password", "");
    declare_parameter<std::string>("stream_path", "stream1");
    declare_parameter<std::string>("frame_id", "camera_optical_frame");
    declare_parameter<std::string>("image_topic", "/baby_cam/image_raw");
    declare_parameter<double>("reconnect_delay_sec", 2.0);
    declare_parameter<double>("publish_rate_hz", 30.0);
    declare_parameter<bool>("low_latency_ffmpeg", true);
    declare_parameter<int>("capture_buffer_size", 1);
    declare_parameter<double>("output_scale", 1.0);
    declare_parameter<int>("max_output_width", 0);

    rtsp_url_ = get_rtsp_url();
    frame_id_ = get_parameter("frame_id").as_string();
    const auto image_topic = get_parameter("image_topic").as_string();
    reconnect_delay_sec_ = get_parameter("reconnect_delay_sec").as_double();
    const auto publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    low_latency_ffmpeg_ = get_parameter("low_latency_ffmpeg").as_bool();
    capture_buffer_size_ = static_cast<int>(
        get_parameter("capture_buffer_size").as_int());
    output_scale_ = clamp(get_parameter("output_scale").as_double(), 0.1, 1.0);
    max_output_width_ = static_cast<int>(
        get_parameter("max_output_width").as_int());

    if (rtsp_url_.empty()) {
      throw std::runtime_error("Parameter 'rtsp_url' must be set.");
    }
    if (publish_rate_hz <= 0.0) {
      throw std::runtime_error(
          "Parameter 'publish_rate_hz' must be greater than 0.");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz)),
        std::bind(&RtspImagePublisher::publish_frame, this));

    RCLCPP_INFO(get_logger(), "Publishing RTSP frames on %s",
                image_topic.c_str());
    connect();
  }

 private:
  std::string get_rtsp_url() {
    const auto rtsp_url = get_parameter("rtsp_url").as_string();
    if (!rtsp_url.empty()) {
      return rtsp_url;
    }

    const auto host = get_parameter("host").as_string();
    const auto username = get_parameter("username").as_string();
    const auto password = get_parameter("password").as_string();
    auto stream_path = get_parameter("stream_path").as_string();
    while (!stream_path.empty() && stream_path.front() == '/') {
      stream_path.erase(stream_path.begin());
    }

    if (host.empty() || username.empty() || password.empty()) {
      throw std::runtime_error(
          "Set either 'rtsp_url' or RTSP host, username, and password.");
    }
    return "rtsp://" + username + ":" + password + "@" + host + "/" +
           stream_path;
  }

  bool connect() {
    close_capture();
    if (low_latency_ffmpeg_ && std::getenv("OPENCV_FFMPEG_CAPTURE_OPTIONS") == nullptr) {
      setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
             "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0",
             0);
    }

    capture_.open(rtsp_url_, cv::CAP_FFMPEG);
    if (capture_buffer_size_ > 0) {
      capture_.set(cv::CAP_PROP_BUFFERSIZE, capture_buffer_size_);
    }
    if (!capture_.isOpened()) {
      RCLCPP_WARN(get_logger(), "Could not open RTSP stream. Retrying in %.2fs",
                  reconnect_delay_sec_);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Connected to RTSP stream");
    return true;
  }

  void publish_frame() {
    if (!capture_.isOpened()) {
      maybe_reconnect();
      return;
    }

    cv::Mat frame;
    const auto read_start = std::chrono::steady_clock::now();
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN(get_logger(), "Frame read failed; reconnecting to RTSP stream");
      close_capture();
      maybe_reconnect();
      return;
    }
    const double read_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - read_start).count();
    if (read_ms > 100.0) {
      RCLCPP_WARN(get_logger(), "Frame read blocked for %.0f ms", read_ms);
    }

    resize_frame(frame);
    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    header.frame_id = frame_id_;
    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    publisher_->publish(*msg);
  }

  void maybe_reconnect() {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_time_) {
      return;
    }
    next_reconnect_time_ =
        now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(reconnect_delay_sec_));
    connect();
  }

  void resize_frame(cv::Mat& frame) const {
    auto scale = output_scale_;
    if (max_output_width_ > 0 &&
        static_cast<double>(frame.cols) * scale > max_output_width_) {
      scale = static_cast<double>(max_output_width_) / frame.cols;
    }
    if (scale >= 1.0) {
      return;
    }

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    frame = std::move(resized);
  }

  void close_capture() {
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

  static double clamp(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(maximum, value));
  }

  std::string rtsp_url_;
  std::string frame_id_;
  double reconnect_delay_sec_{2.0};
  bool low_latency_ffmpeg_{true};
  int capture_buffer_size_{1};
  double output_scale_{1.0};
  int max_output_width_{0};
  cv::VideoCapture capture_;
  std::chrono::steady_clock::time_point next_reconnect_time_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RtspImagePublisher>());
  } catch (const std::exception& exc) {
    RCLCPP_ERROR(rclcpp::get_logger("rtsp_image_publisher"), "%s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
