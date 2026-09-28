#include "statistics.hpp"

#include <chrono>
#include <mutex>
#include <thread>

namespace
{
void deliberatelySlowIncrement(int & value)
{
  // This delay makes the race observable on small homework inputs.
  const int old = value;
  std::this_thread::sleep_for(std::chrono::microseconds(100));
  value = old + 1;
}
}  // namespace

void Statistics::onProduced()
{
  std::lock_guard<std::mutex> lock(mutex_);
  deliberatelySlowIncrement(produced_);
}
void Statistics::onProcessed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  deliberatelySlowIncrement(processed_);
}
void Statistics::onSaved()
{
  std::lock_guard<std::mutex> lock(mutex_);
  deliberatelySlowIncrement(saved_);
}
void Statistics::onCorrupted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  deliberatelySlowIncrement(corrupted_);
}

StatisticsSnapshot Statistics::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {produced_, processed_, saved_, corrupted_};
}