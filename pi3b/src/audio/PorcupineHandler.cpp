#include "nova/audio/PorcupineHandler.hpp"
#include <iostream>

namespace nova {

PorcupineHandler::PorcupineHandler(const std::string &access_key,
                                   const std::string &model_path,
                                   const std::string &keyword_path,
                                   float sensitivity) {
    const char *keyword_paths[] = { keyword_path.c_str() };
    float sensitivities[] = { sensitivity };

    pv_status_t status = pv_porcupine_init(
        access_key.c_str(),          // access key (from Picovoice Console)
        model_path.c_str(),          // path to porcupine_params.pv
        1,                           // number of keywords
        keyword_paths,               // array of keyword paths
        sensitivities,               // array of sensitivities
        &handle_);                   // out handle

    if (status != PV_STATUS_SUCCESS) {
        std::cerr << "Failed to init Porcupine, status " << status << std::endl;
        handle_ = nullptr;
    }
}

PorcupineHandler::~PorcupineHandler() {
    if (handle_ != nullptr) {
        pv_porcupine_delete(handle_);
        handle_ = nullptr;
    }
}

int PorcupineHandler::process_frame(const int16_t *pcm) {
    if (!handle_) return false;

    int32_t keyword_index = -1;
    pv_status_t status = pv_porcupine_process(handle_, pcm, &keyword_index);

    if (status != PV_STATUS_SUCCESS) {
        std::cerr << "Porcupine process failed, status " << status << std::endl;
        return false;
    }

    return keyword_index;
}

} // namespace nova
