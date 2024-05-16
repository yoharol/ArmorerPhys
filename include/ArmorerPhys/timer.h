#ifndef ARMORER_PHYS_TIMER_H_
#define ARMORER_PHYS_TIMER_H_

#include <memory>

struct Timer {
 private:
  static std::unique_ptr<Timer> instance;

  Timer() {}

 public:
  Timer(const Timer&) = delete;
  Timer& operator=(const Timer&) = delete;

  static Timer* getInstance() {
    if (!instance) {
      instance.reset(new Timer());
    }
    return instance.get();
  }
};

std::unique_ptr<Timer> Timer::instance = nullptr;

#endif  // ARMORER_PHYS_TIMER_H_
