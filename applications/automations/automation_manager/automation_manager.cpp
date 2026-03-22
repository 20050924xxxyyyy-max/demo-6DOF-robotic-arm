#include "automation_manager.hpp"

bool AutomationManager::idle() const { return automation_ == nullptr; }

void AutomationManager::load(Automation * automation)
{
  if (automation_ != nullptr) automation_->quit();

  automation_ = automation;
  automation_->quit();
}

void AutomationManager::run()
{
  if (automation_ == nullptr) return;

  if (automation_->run()) {
    automation_->quit();
    automation_ = nullptr;
  }
}

void AutomationManager::quit()
{
  if (automation_ == nullptr) return;

  automation_->quit();
  automation_ = nullptr;
}

bool AutomationManager::is(Automation * automation) { return (automation_ == automation); }