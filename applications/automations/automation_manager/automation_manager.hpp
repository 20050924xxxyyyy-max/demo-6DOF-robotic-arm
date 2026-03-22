#ifndef AUTOMATION_MANAGER_HPP
#define AUTOMATION_MANAGER_HPP

#include "automations/automation/automation.hpp"

class AutomationManager
{
public:
  bool idle() const;
  void load(Automation * automation);
  void run();
  void quit();
  bool is(Automation * automation);
  Automation * automation_ = nullptr;

private:
  // Automation * automation_ = nullptr;
};

#endif  // AUTOMATION_MANAGER_HPP