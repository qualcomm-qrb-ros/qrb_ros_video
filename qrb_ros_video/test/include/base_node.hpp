/*
**************************************************************************************************
* Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#ifndef QRB_ROS_VIDEO_TEST_BASE_NODE_HPP_
#define QRB_ROS_VIDEO_TEST_BASE_NODE_HPP_

#include <gst/gst.h>
#include <gst/pbutils/pbutils.h>

#include <cctype>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace video
{

/**
 * @brief Base node template class for GStreamer-based ROS nodes
 */
class BaseNode : public rclcpp::Node
{
public:
  explicit BaseNode(const std::string & node_name, const rclcpp::NodeOptions & options)
    : Node(node_name, options)
    , format_(this->declare_parameter("format", "h264"))
    , width_(this->declare_parameter("width", "640"))
    , height_(this->declare_parameter("height", "480"))
    , framerate_(this->declare_parameter("framerate", "30/1"))
    , log_level_(this->declare_parameter("log-level", "warning"))
    , url_(this->declare_parameter("url", ""))
  {
    // Initialize GStreamer
    if (!gst_is_initialized()) {
      gst_init(nullptr, nullptr);
    }

    auto pos = framerate_.find_first_of('/');
    std::string denominator = "1";
    if (pos != std::string::npos) {
      denominator = framerate_.substr(pos + 1);
    }
    std::string numerator = framerate_.substr(0, pos);
    if (std::stof(denominator) != 0) {
      fps_ = std::stof(numerator) / std::stof(denominator);
    }

    // Set ROS Node log level
    auto log_level = rclcpp::Logger::Level::Warn;
    if (log_level_ == "info") {
      log_level = rclcpp::Logger::Level::Info;
    } else if (log_level_ == "debug") {
      log_level = rclcpp::Logger::Level::Debug;
    }

    get_logger().set_level(log_level);
  }

  virtual ~BaseNode() = default;

protected:
  /**
   * @brief Setup the GStreamer pipeline
   */
  virtual void setup_pipeline() = 0;

  // Common logging utility
  void log_pipeline_creation(const std::string & pipeline_str)
  {
    RCLCPP_WARN(this->get_logger(), "Creating pipeline: %s", pipeline_str.c_str());
  }

  bool discover_pipeline()
  {
    if (url_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No URL provided for file discovery");
      return false;
    }

    auto fallback_with_format = [this](const std::string & reason) {
      // Parse extension from URL path (ignore URI scheme and query string)
      std::string path = url_;
      auto query_pos = path.find_first_of("?#");
      if (query_pos != std::string::npos) {
        path = path.substr(0, query_pos);
      }
      auto slash_pos = path.find_last_of('/');
      std::string filename = (slash_pos == std::string::npos) ? path : path.substr(slash_pos + 1);
      auto dot_pos = filename.find_last_of('.');
      std::string extension = dot_pos == std::string::npos ? "" : filename.substr(dot_pos);
      for (char & ch : extension) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
      }

      // If discovery fails but file extension is mp4, switch container path to mp4 and keep user
      // codec hint in pixel_format_.
      if (extension == ".mp4") {
        pixel_format_ = format_;
        format_ = "mp4";
        RCLCPP_WARN(this->get_logger(),
            "Discovery failed (%s), fallback by extension '.mp4'. format=%s pixel_format=%s",
            reason.c_str(), format_.c_str(), pixel_format_.c_str());
        return true;
      }

      pixel_format_ = format_;
      RCLCPP_WARN(this->get_logger(),
          "Discovery failed (%s), fallback to configured format. format=%s pixel_format=%s",
          reason.c_str(), format_.c_str(), pixel_format_.c_str());
      return true;
    };

    RCLCPP_INFO(this->get_logger(), "Discovering video info from URI: %s", url_.c_str());

    // Create a proper URI if file path is provided
    std::string uri = url_;
    if (uri.find("://") == std::string::npos) {
      // Assume it's a file path and convert to URI
      uri = "file://" + uri;
    }

    // Create a new discoverer with 2-second timeout
    GstDiscoverer * discoverer = gst_discoverer_new(2 * GST_SECOND, nullptr);
    if (!discoverer) {
      return fallback_with_format("failed to create GstDiscoverer");
    }

    // Attempt to discover the URI
    GError * error = nullptr;
    GstDiscovererInfo * info = gst_discoverer_discover_uri(discoverer, uri.c_str(), &error);

    if (error) {
      std::string error_msg = error->message;
      g_error_free(error);
      gst_object_unref(discoverer);
      return fallback_with_format(error_msg);
    }

    if (!info) {
      gst_object_unref(discoverer);
      return fallback_with_format("discoverer returned null info");
    }

    // Check if discovery result is valid
    GstDiscovererResult result = gst_discoverer_info_get_result(info);
    if (result != GST_DISCOVERER_OK && result != GST_DISCOVERER_MISSING_PLUGINS) {
      gst_discoverer_info_unref(info);
      gst_object_unref(discoverer);
      return fallback_with_format("discoverer returned non-OK result");
    }

    // Extract video streams information
    GList * list = gst_discoverer_info_get_video_streams(info);
    if (!list) {
      gst_discoverer_info_unref(info);
      gst_object_unref(discoverer);
      return fallback_with_format("no video streams found");
    }

    bool found_stream = false;
    // Extract information from the first video stream
    for (GList * item = list; item != nullptr; item = item->next) {
      GstDiscovererStreamInfo * stream_info = GST_DISCOVERER_STREAM_INFO(item->data);
      if (GST_IS_DISCOVERER_VIDEO_INFO(stream_info)) {
        GstDiscovererVideoInfo * video_info = GST_DISCOVERER_VIDEO_INFO(stream_info);

        // Get codec information from caps
        GstCaps * caps = gst_discoverer_stream_info_get_caps(stream_info);
        if (caps) {
          gchar * caps_str = gst_caps_to_string(caps);
          RCLCPP_INFO(this->get_logger(), "Video caps: %s", caps_str);
          g_free(caps_str);

          // For mp4 input, detect coded stream format into pixel_format_
          if (format_ == "mp4") {
            const GstStructure * structure = gst_caps_get_structure(caps, 0);
            if (structure) {
              const gchar * codec_name = gst_structure_get_name(structure);
              if (codec_name) {
                if (g_str_has_suffix(codec_name, "h264")) {
                  pixel_format_ = "h264";
                } else if (g_str_has_suffix(codec_name, "h265") ||
                           g_str_has_suffix(codec_name, "hevc")) {
                  pixel_format_ = "h265";
                } else {
                  pixel_format_ = codec_name;
                }
                RCLCPP_INFO(this->get_logger(), "Detected codec: %s", pixel_format_.c_str());
              }
            }
          }
          gst_caps_unref(caps);
        }

        // Get resolution and framerate
        width_ = std::to_string(gst_discoverer_video_info_get_width(video_info));
        height_ = std::to_string(gst_discoverer_video_info_get_height(video_info));

        gint fps_num = gst_discoverer_video_info_get_framerate_num(video_info);
        gint fps_denom = gst_discoverer_video_info_get_framerate_denom(video_info);

        // Avoid division by zero
        if (fps_denom > 0 && fps_num > 0) {
          auto framerate = std::to_string(fps_num) + "/" + std::to_string(fps_denom);
          auto fps = static_cast<float>(fps_num) / static_cast<float>(fps_denom);
          if (fps != fps_) {
            RCLCPP_WARN(
                this->get_logger(), "Detected framerate: %s (%f fps)", framerate.c_str(), fps);
          }
        }

        RCLCPP_INFO(this->get_logger(), "Video properties: %sx%s @ %s fps", width_.c_str(),
            height_.c_str(), framerate_.c_str());

        found_stream = true;
        break;  // We only need information from the first video stream
      }
    }

    g_list_free(list);
    gst_discoverer_info_unref(info);
    gst_object_unref(discoverer);

    if (!found_stream) {
      return fallback_with_format("no valid video stream found");
    }

    RCLCPP_INFO(this->get_logger(), "Successfully discovered video information");
    return true;
  }

  // Common error handling for pipeline creation
  bool create_pipeline(const std::string & pipeline_str)
  {
    GError * error = nullptr;

    log_pipeline_creation(pipeline_str);

    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
      g_error_free(error);
      return false;
    }

    return true;
  }

  std::string format_;
  std::string pixel_format_;
  std::string width_;
  std::string height_;
  std::string framerate_;
  std::string log_level_;
  std::string url_;
  float fps_ = 30.0f;
  GstElement * pipeline_ = nullptr;
};

}  // namespace video
}  // namespace qrb_ros

#endif  // QRB_ROS_VIDEO_TEST_BASE_NODE_HPP_
