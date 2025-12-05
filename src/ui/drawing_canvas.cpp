#include "drawing_canvas.hpp"
#include <imgui.h>

namespace oscilloplot {

void DrawingCanvas::render() {
    if (!m_isOpen) return;

    ImGui::Begin("Drawing Canvas", &m_isOpen);

    ImVec2 canvasSize = ImGui::GetContentRegionAvail();
    ImVec2 canvasPos = ImGui::GetCursorScreenPos();

    // Draw canvas background
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRectFilled(canvasPos,
        ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y),
        IM_COL32(20, 20, 25, 255));

    // Handle mouse input
    ImGui::InvisibleButton("canvas", canvasSize);
    bool isHovered = ImGui::IsItemHovered();

    if (isHovered && ImGui::IsMouseDown(0)) {
        ImVec2 mousePos = ImGui::GetMousePos();

        // Convert to normalized coordinates (-1 to 1)
        float nx = (mousePos.x - canvasPos.x) / canvasSize.x * 2.0f - 1.0f;
        float ny = -((mousePos.y - canvasPos.y) / canvasSize.y * 2.0f - 1.0f); // Flip Y

        m_pattern.push_back(nx, ny);
        m_isDrawing = true;
    } else {
        m_isDrawing = false;
    }

    // Draw the pattern
    if (m_pattern.size() > 1) {
        for (size_t i = 1; i < m_pattern.size(); ++i) {
            float x1 = (m_pattern.x[i-1] + 1.0f) / 2.0f * canvasSize.x + canvasPos.x;
            float y1 = (-m_pattern.y[i-1] + 1.0f) / 2.0f * canvasSize.y + canvasPos.y;
            float x2 = (m_pattern.x[i] + 1.0f) / 2.0f * canvasSize.x + canvasPos.x;
            float y2 = (-m_pattern.y[i] + 1.0f) / 2.0f * canvasSize.y + canvasPos.y;

            drawList->AddLine(ImVec2(x1, y1), ImVec2(x2, y2),
                IM_COL32(0, 255, 100, 255), 2.0f);
        }
    }

    // Controls
    if (ImGui::Button("Clear")) {
        clearPattern();
    }
    ImGui::SameLine();
    ImGui::Text("Points: %zu", m_pattern.size());

    ImGui::End();
}

void DrawingCanvas::clearPattern() {
    m_pattern.clear();
}

} // namespace oscilloplot
