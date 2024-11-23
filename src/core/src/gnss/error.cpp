#include "sensors/gnss/error.hpp"

namespace navp::sensors::gnss {

Error::Error() = default;

Error::~Error() = default;

void Error::set_model(std::unique_ptr<Model> model) {
    model_ = std::move(model);
}

}