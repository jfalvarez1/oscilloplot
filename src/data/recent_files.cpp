#include "recent_files.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>

namespace oscilloplot {

namespace {

// Windows paths are case-insensitive; comparing case-sensitively there would
// let the same file appear twice under different capitalisation.
bool samePath(const std::string& a, const std::string& b) {
#if defined(_WIN32)
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        char ca = a[i], cb = b[i];
        if (ca == '/') ca = '\\';
        if (cb == '/') cb = '\\';
        if (std::tolower(static_cast<unsigned char>(ca)) !=
            std::tolower(static_cast<unsigned char>(cb))) {
            return false;
        }
    }
    return true;
#else
    return a == b;
#endif
}

bool fileExists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

} // namespace

void RecentFiles::add(const std::string& path) {
    if (path.empty()) return;

    remove(path);
    m_entries.insert(m_entries.begin(), path);
    if (m_entries.size() > MAX_ENTRIES) {
        m_entries.resize(MAX_ENTRIES);
    }
}

void RecentFiles::remove(const std::string& path) {
    m_entries.erase(
        std::remove_if(m_entries.begin(), m_entries.end(),
                       [&](const std::string& e) { return samePath(e, path); }),
        m_entries.end());
}

size_t RecentFiles::pruneMissing() {
    const size_t before = m_entries.size();
    m_entries.erase(
        std::remove_if(m_entries.begin(), m_entries.end(),
                       [](const std::string& e) { return !fileExists(e); }),
        m_entries.end());
    return before - m_entries.size();
}

bool RecentFiles::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    for (const auto& e : m_entries) f << e << "\n";
    return f.good();
}

bool RecentFiles::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    m_entries.clear();
    std::string line;
    while (std::getline(f, line) && m_entries.size() < MAX_ENTRIES) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
            line.pop_back();
        }
        if (!line.empty()) m_entries.push_back(line);
    }
    return true;
}

std::string RecentFiles::displayName(const std::string& path) {
    size_t slash = path.find_last_of("/\\");
    return (slash == std::string::npos) ? path : path.substr(slash + 1);
}

} // namespace oscilloplot
