#include "ArmorerPhys/timer.h"

#include <chrono>  // NOLINT
#include <iomanip>
#include <unordered_map>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

// add more advanced log in the future

namespace aphys {

std::string mus_to_string(double mus) {
  std::stringstream ss;
  ss << std::right << std::setw(12);
  ss << std::fixed << std::setprecision(3);
  if (mus < 1e2)
    ss << mus << "  us";
  else if (mus < 1e5)
    ss << mus / 1e3 << "  ms";
  else
    ss << mus / 1e6 << "  s";
  return ss.str();
}

std::string mus_to_string(double mus, std::string format) {
  if (format == "") return mus_to_string(mus);
  std::stringstream ss;
  ss << std::right << std::setw(12);
  ss << std::fixed << std::setprecision(3);
  if (format == "us")
    ss << mus << "  us";
  else if (format == "ms")
    ss << mus / 1e3 << "  ms";
  else if (format == "s")
    ss << mus / 1e6 << "  s";
  return ss.str();
}

struct SingleTimer::Impl {
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  std::chrono::duration<int64_t, std::nano> duration;
  std::vector<std::chrono::duration<int64_t, std::nano>> durations;
};

SingleTimer::SingleTimer() : impl(new Impl()) {}

SingleTimer::~SingleTimer() = default;

void SingleTimer::start() {
  impl->start_time = std::chrono::high_resolution_clock::now();
}

void SingleTimer::pause_and_record() {
  auto pause_time = std::chrono::high_resolution_clock::now();
  impl->duration += pause_time - impl->start_time;
  impl->durations.push_back(pause_time - impl->start_time);
  impl->start_time = pause_time;
}

void SingleTimer::pause() {
  impl->duration +=
      std::chrono::high_resolution_clock::now() - impl->start_time;
  impl->start_time = std::chrono::high_resolution_clock::now();
}

void SingleTimer::reset() {
  impl->duration = std::chrono::high_resolution_clock::duration::zero();
  impl->durations.clear();
}

double SingleTimer::get_duration_in_microsecond() {
  return std::chrono::duration_cast<std::chrono::microseconds>(impl->duration)
      .count();
}

std::string SingleTimer::to_string(std::string format) {
  return mus_to_string(get_duration_in_microsecond(), format);
}

Timer* Timer::getInstance() {
  if (!instance) {
    instance.reset(new Timer());
  }
  return instance.get();
}

void Timer::start(const std::string& name) {
  if (timers.find(name) == timers.end()) {
    SingleTimer new_st;
    timers[name] = new_st;
    max_length = std::max(max_length, static_cast<int>(name.size()));
  }
  timers[name].start();
}

void Timer::pause(const std::string& name) {
  if (timers.find(name) == timers.end()) return;
  timers[name].pause();
}

void Timer::reset(const std::string& name) {
  if (timers.find(name) == timers.end()) return;
  timers[name].reset();
}

void Timer::print(const std::string& name) {
  if (timers.find(name) == timers.end()) return;
  std::cout << std::left;
  std::cout << '[' << std::setw(max_length + 5) << name << ']'
            << timers[name].to_string() << std::endl;
}

void Timer::print_all(std::string format) {
  std::cout << std::left;
  for (auto& timer : timers) {
    std::cout << '[' << std::setw(max_length + 5) << timer.first << "]      "
              << timer.second.to_string(format) << std::endl;
  }
}

void Timer::reset_all() {
  for (auto& timer : timers) {
    timer.second.reset();
  }
}

std::unique_ptr<Timer> Timer::instance = nullptr;

}  // namespace aphys
