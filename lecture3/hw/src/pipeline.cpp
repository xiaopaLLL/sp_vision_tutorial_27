#include "pipeline.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <opencv2/imgcodecs.hpp>

namespace
{
    std::mutex output_mutex;

    // A simple thread-safe log output helper function
    void logLine(std::ostream &output, const std::string &message)
    {
        std::lock_guard<std::mutex> lock(output_mutex);
        output << message << '\n';
    }
}

Pipeline::Pipeline(std::unique_ptr<FrameSource> source, PipelineConfig config)
    : source_(std::move(source)), config_(std::move(config))
{
    if (!source_)
    {
        throw std::invalid_argument("Pipeline requires a frame source");
    }
    if (config_.worker_count < 2)
    {
        throw std::invalid_argument("worker_count must be at least 2");
    }
}

Pipeline::~Pipeline()
{
    // DONE: Make sure Pipeline never destroys running threads.
    wait();
}

void Pipeline::start()
{
    std::filesystem::create_directories(config_.output_directory);
    workers_.reserve(static_cast<std::size_t>(config_.worker_count));
    for (int i = 0; i < config_.worker_count; ++i)
    {
        workers_.emplace_back([this, i]
                              { workerLoop(i); });
    }
    producer_ = std::thread([this]
                            { producerLoop(); });
}

void Pipeline::wait()
{
    if (producer_.joinable())
    {
        producer_.join();
    }
    for (auto &worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
}

StatisticsSnapshot Pipeline::statistics() const
{
    return statistics_.snapshot();
}

void Pipeline::producerLoop()
{
    Frame frame;
    while (source_->next(frame))
    {
        statistics_.onProduced();
        logLine(std::cout, "[Producer] frame " + std::to_string(frame.id));

        // What's the best way to write this?
        queue_.push(std::move(frame));
        // queue_.push(frame);
    }
    queue_.close();
}

void Pipeline::workerLoop(int worker_id)
{
    Frame frame;
    while (queue_.pop(frame))
    {
        if (config_.worker_delay_ms > 0)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.worker_delay_ms));
        }

        if (checksum(frame.image) != frame.expected_checksum)
        {
            statistics_.onCorrupted();
            logLine(std::cerr,
                    "[Worker " + std::to_string(worker_id) + "] ERROR: frame " +
                        std::to_string(frame.id) + " data changed before processing");
            continue;
        }

        logLine(std::cout,
                "[Worker " + std::to_string(worker_id) + "] processing frame " +
                    std::to_string(frame.id));
        const cv::Mat output = processor_.process(frame);
        statistics_.onProcessed();

        std::ostringstream filename;
        filename << std::setw(3) << std::setfill('0') << frame.id << ".jpg";
        const auto name = filename.str();
        if (cv::imwrite((config_.output_directory / name).string(), output))
        {
            statistics_.onSaved();
        }
    }
}
