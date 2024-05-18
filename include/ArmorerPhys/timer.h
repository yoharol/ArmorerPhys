#ifndef ARMORER_PHYS_TIMER_H_
#define ARMORER_PHYS_TIMER_H_

#include <memory>
#include <unordered_map>
#include <string>

namespace aphys {

// ! set ms or second mode

struct SingleTimer {
 private:
  struct Impl;
  std::shared_ptr<Impl> impl;

 public:
  SingleTimer();
  ~SingleTimer();
  void start();
  void pause();
  void pause_and_record();
  void reset();
  double get_duration_in_microsecond();
  std::string to_string(std::string format = "");
};

struct Timer {
 private:
  static std::unique_ptr<Timer> instance;
  std::unordered_map<std::string, SingleTimer> timers;
  int max_length = 0;
  Timer() {}

 public:
  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;
  static Timer* getInstance();

  void start(const std::string& name);
  void pause(const std::string& name);
  void reset(const std::string& name);
  void print(const std::string& name);
  void print_all(const std::string format = "");
  void reset_all();
};

}  // namespace aphys

#endif  // ARMORER_PHYS_TIMER_H_
