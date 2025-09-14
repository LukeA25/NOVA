// PorcupineHandler.hpp
#pragma once
#include <pv_porcupine.h>
#include <string>

namespace nova {

class PorcupineHandler {
public:
    PorcupineHandler(const std::string &model_path, const std::string &keyword_path);
    ~PorcupineHandler();

    bool process_frame(const int16_t *frame);  // returns true if wake word detected

private:
    pv_porcupine_t *handle_;
};

}
