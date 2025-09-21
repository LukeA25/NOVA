#pragma once
#include <string>
#include "pv_porcupine.h"
#include <vector>

namespace nova {

class PorcupineHandler {
public:
    PorcupineHandler(const std::string &access_key,
                     const std::string &model_path,
                     const std::string &keyword_path,
                     float sensitivity = 0.5f);

    ~PorcupineHandler();

    int process_frame(const int16_t *pcm);

private:
    pv_porcupine_t *handle_;
};

} // namespace nova
