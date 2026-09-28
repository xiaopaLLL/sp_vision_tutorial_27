#pragma once

#include <mutex>

struct StatisticsSnapshot
{
  int produced = 0;
  int processed = 0;
  int saved = 0;
  int corrupted = 0;
};

class Statistics
{
public:
  void onProduced();
  void onProcessed();
  void onSaved();
  void onCorrupted();
  StatisticsSnapshot snapshot() const;

private:
  // DONE: This object is shared by multiple worker threads.
  mutable std::mutex mutex_;
  int produced_ = 0;
  int processed_ = 0;
  int saved_ = 0;
  int corrupted_ = 0;
};
