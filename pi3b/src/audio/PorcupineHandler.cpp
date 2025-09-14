// PorcupineHandler.cpp
#include "nova/PorcupineHandler.hpp"
#include <stdexcept>

using namespace nova;

PorcupineHandler::PorcupineHandler(const std::string &model_path, const std::string &keyword_path) {
    const char *keyword_paths[] = { keyword_path.c_str() };
    pv_status_t status = pv_porcupine_init(
        model_path.c_str(), 1, keyword_paths, nullptr, &handle_
    );
    if (status != PV_STATUS_SUCCESS) {
        throw std::runtime_error("Failed to init Porcupine");
    }
}

PorcupineHandler::~PorcupineHandler() {
    if (handle_) {
        pv_porcupine_delete(handle_);
    }
}

bool PorcupineHandler::process_frame(const int16_t *frame) {
    int32_t result = 0;
    pv_status_t status = pv_porcupine_process(handle_, frame, &result);
    return (status == PV_STATUS_SUCCESS && result == 1);
}
