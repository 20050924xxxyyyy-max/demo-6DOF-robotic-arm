#include "automation.hpp"

Call::Call(std::function<void()> func) : func_(func) {}

bool Call::run()
{
  func_();
  return true;
}

Wait::Wait(std::function<bool()> func) : func_(func) {}

bool Wait::run() { return func_(); }

bool Sequence::run()
{
  if (i_ < automations_.size() && automations_[i_]->run()) i_++;
  return i_ == automations_.size();
}

void Sequence::quit()
{
  i_ = 0;
  for (auto & automation : automations_) automation->quit();
}

bool Parallel::run()
{
  bool done = true;
  for (auto & automation : automations_) done &= automation->run();
  return done;
}

void Parallel::quit()
{
  for (auto & automation : automations_) automation->quit();
}