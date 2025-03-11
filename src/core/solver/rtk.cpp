#include "solution/rtk.hpp"

using namespace navp::solution;

class MyRtk : public navp::solution::RtkServer {
 public:
  using RtkServer::RtkServer;

  virtual ~MyRtk() noexcept = default;

 protected:
  virtual void before_action() override { Task::before_action(); }

  virtual void after_action() override { Task::after_action(); }

  virtual void action() override {
    
  }
};

navp::i32 main() { return 0; }