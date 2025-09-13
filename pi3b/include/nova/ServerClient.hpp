#pragma once
#include <string>

class ServerClient {
public:
    explicit ServerClient(const std::string& url);
    bool upload_wav(const std::string& path, std::string& response_out);

private:
    std::string m_url;
};
