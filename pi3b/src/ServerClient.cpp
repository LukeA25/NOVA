#include "nova/ServerClient.hpp"
#include <curl/curl.h>

namespace {
    size_t write_cb(void* contents, size_t size, size_t nmemb, void* userp) {
        auto* out = static_cast<std::string*>(userp);
        out->append(static_cast<char*>(contents), size * nmemb);
        return size * nmemb;
    }
}

ServerClient::ServerClient(const std::string& url) : m_url(url) {}

bool ServerClient::upload_wav(const std::string& path, std::string& response_out) {
    CURL* curl = curl_easy_init();
    if (!curl) throw std::runtime_error("curl init failed");

    curl_mime* form = curl_mime_init(curl);
    curl_mimepart* field = curl_mime_addpart(form);
    curl_mime_name(field, "file");
    curl_mime_filedata(field, path.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, m_url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_out);

    CURLcode res = curl_easy_perform(curl);

    curl_mime_free(form);
    curl_easy_cleanup(curl);

    return (res == CURLE_OK);
}
