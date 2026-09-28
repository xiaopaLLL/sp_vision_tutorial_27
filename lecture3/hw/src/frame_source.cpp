#include "frame_source.hpp"

#include <algorithm>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <thread>

namespace
{
bool isImage(const std::filesystem::path & path)
{
  const auto extension = path.extension().string();
  return extension == ".jpg" || extension == ".jpeg" || extension == ".png" || extension == ".ppm";
}
}  // namespace

ImageSequenceSource::ImageSequenceSource(std::filesystem::path directory, int producer_delay_ms)
: producer_delay_ms_(producer_delay_ms)
{
  if (!std::filesystem::is_directory(directory)) {
    throw std::runtime_error("input directory does not exist: " + directory.string());
  }
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && isImage(entry.path())) {
      paths_.push_back(entry.path());
    }
  }
  std::sort(paths_.begin(), paths_.end());
  if (paths_.empty()) {
    throw std::runtime_error("input directory contains no images");
  }
}

bool ImageSequenceSource::next(Frame & frame)
{
  if (next_index_ >= paths_.size()) {
    return false;
  }

  const cv::Mat raw = cv::imread(paths_[next_index_].string(), cv::IMREAD_COLOR);
  if (raw.empty()) {
    throw std::runtime_error("failed to read: " + paths_[next_index_].string());
  }
  raw.copyTo(buffer_);  // Simulates a camera-owned reusable buffer.

  frame.id = static_cast<int>(next_index_++);
  frame.expected_checksum = checksum(buffer_);

  // DONE: The camera will reuse its internal buffer. Make sure this frame
  // remains valid after the next call to next().
  frame.image = buffer_.clone();
  // frame.image = buffer_;

  if (producer_delay_ms_ > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(producer_delay_ms_));
  }
  return true;
}
