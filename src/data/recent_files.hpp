#pragma once

#include <string>
#include <vector>

namespace oscilloplot {

//==============================================================================
// Most-recently-used file list, persisted next to the app's other settings.
//
// Newest first, de-duplicated case-insensitively on Windows paths, capped at
// MAX_ENTRIES. Missing files are dropped when the list is loaded so the menu
// never offers something that no longer exists.
//==============================================================================

class RecentFiles {
public:
    static constexpr size_t MAX_ENTRIES = 5;

    // Move `path` to the front, inserting it if new, and trim to MAX_ENTRIES.
    void add(const std::string& path);

    void remove(const std::string& path);
    void clear() { m_entries.clear(); }

    const std::vector<std::string>& entries() const { return m_entries; }
    bool empty() const { return m_entries.empty(); }

    // Drop entries whose file no longer exists. Returns how many were removed.
    size_t pruneMissing();

    bool save(const std::string& path) const;
    bool load(const std::string& path);

    // Display form: file name only, since the menu is narrow.
    static std::string displayName(const std::string& path);

private:
    std::vector<std::string> m_entries;
};

} // namespace oscilloplot
