#pragma once

#include <memory>
#include <string_view>

#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::sensors::gnss {
class Error;
class Model;

class NAVP_EXPORT Model {
 public:
  Model();
  virtual ~Model() = 0;

  virtual std::string_view name() = 0;
};

class NAVP_EXPORT Error {
 public:
  Error();
  virtual ~Error() = 0;

  virtual std::string_view name() = 0;

  virtual void set_model(std::unique_ptr<Model> model);

  virtual f64 evalute() = 0;

 protected:
  std::unique_ptr<Model> model_;
};

}  // namespace navp::sensors::gnss