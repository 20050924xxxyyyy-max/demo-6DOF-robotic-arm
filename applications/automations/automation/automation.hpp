#ifndef AUTOMATION_HPP
#define AUTOMATION_HPP

#include <functional>
#include <memory>
#include <vector>

class Automation
{
public:
  virtual bool run() = 0;
  virtual void quit() {};
};

class Call : public Automation
{
public:
  Call(std::function<void()> func);

  bool run() override;

private:
  std::function<void()> func_;
};

class Wait : public Automation
{
public:
  Wait(std::function<bool()> func);

  bool run() override;

private:
  std::function<bool()> func_;
};

template <std::size_t N>
class Count : public Automation
{
  static_assert(N > 0);

public:
  bool run() override
  {
    count_++;
    return count_ == N;
  }
  void quit() override { count_ = 0; }

private:
  std::size_t count_ = 0;
};

class Sequence : public Automation
{
public:
  template <typename... Automations>
  Sequence(Automations... automations)
  {
    (automations_.push_back(std::make_shared<Automations>(automations)), ...);
  }

  bool run() override;
  void quit() override;

private:
  std::size_t i_ = 0;
  std::vector<std::shared_ptr<Automation>> automations_;
};

class Parallel : public Automation
{
public:
  template <typename... Automations>
  Parallel(Automations... automations)
  {
    (automations_.push_back(std::make_shared<Automations>(automations)), ...);
  }

  bool run() override;
  void quit() override;

private:
  std::vector<std::shared_ptr<Automation>> automations_;
};

#endif  // AUTOMATION_HPP