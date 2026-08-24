#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace oscilloplot {

//==============================================================================
// Bounded undo/redo history over whole-state snapshots.
//
// Editors here hold small state (a stroke list, a step list, 16 pad cells), so
// snapshotting the whole thing is simpler and less error-prone than modelling
// individual operations, and cheap enough at this size.
//
// Usage: call push(state) BEFORE mutating, then undo()/redo() return the state
// to restore, or nullptr when there is nothing to go back to.
//==============================================================================

template <typename State>
class UndoStack {
public:
    explicit UndoStack(size_t maxDepth = 32) : m_maxDepth(maxDepth) {}

    // Record the state as it is *before* an edit.
    void push(const State& state) {
        // A new edit invalidates anything that was undone past this point.
        m_redo.clear();
        m_undo.push_back(state);
        if (m_undo.size() > m_maxDepth) {
            m_undo.erase(m_undo.begin());
        }
    }

    bool canUndo() const { return !m_undo.empty(); }
    bool canRedo() const { return !m_redo.empty(); }

    // `current` is the live state, saved so redo can return to it.
    // Returns nullptr if there is nothing to undo.
    const State* undo(const State& current) {
        if (m_undo.empty()) return nullptr;
        m_redo.push_back(current);
        m_last = std::move(m_undo.back());
        m_undo.pop_back();
        return &m_last;
    }

    const State* redo(const State& current) {
        if (m_redo.empty()) return nullptr;
        m_undo.push_back(current);
        m_last = std::move(m_redo.back());
        m_redo.pop_back();
        return &m_last;
    }

    void clear() {
        m_undo.clear();
        m_redo.clear();
    }

    size_t undoDepth() const { return m_undo.size(); }
    size_t redoDepth() const { return m_redo.size(); }

private:
    std::vector<State> m_undo;
    std::vector<State> m_redo;
    State m_last{};              // Storage backing the pointer returned above
    size_t m_maxDepth;
};

} // namespace oscilloplot
