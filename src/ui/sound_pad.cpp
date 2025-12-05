#include "sound_pad.hpp"
#include <imgui.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {

void SoundPad::render() {
    if (!m_isOpen) return;

    ImGui::Begin("Sound Pad", &m_isOpen);

    ImGui::SliderInt("Rows", &m_gridRows, 2, 8);
    ImGui::SliderInt("Columns", &m_gridCols, 2, 8);

    ImVec2 buttonSize(60, 60);
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    for (int row = 0; row < m_gridRows; ++row) {
        for (int col = 0; col < m_gridCols; ++col) {
            int cellId = row * m_gridCols + col;

            if (col > 0) ImGui::SameLine();

            ImGui::PushID(cellId);

            ImVec2 pos = ImGui::GetCursorScreenPos();
            bool pressed = ImGui::Button("##pad", buttonSize);

            // Draw cell content
            float freq = 1.0f + cellId * 0.5f;
            ImU32 color = (m_activeCell == cellId)
                ? IM_COL32(0, 255, 100, 255)
                : IM_COL32(50, 50, 60, 255);

            drawList->AddRectFilled(pos,
                ImVec2(pos.x + buttonSize.x, pos.y + buttonSize.y),
                color, 4.0f);

            // Show frequency label
            char label[16];
            snprintf(label, sizeof(label), "%.1f", freq);
            drawList->AddText(ImVec2(pos.x + 5, pos.y + 5),
                IM_COL32(200, 200, 200, 255), label);

            if (pressed) {
                m_activeCell = cellId;

                // Generate pattern based on cell
                int points = 100;
                m_pattern.resize(points);

                for (int i = 0; i < points; ++i) {
                    float t = static_cast<float>(i) / points * 2.0f * M_PI;
                    m_pattern.x[i] = std::cos(freq * t);
                    m_pattern.y[i] = std::sin(freq * t * (1.0f + cellId * 0.1f));
                }
            }

            ImGui::PopID();
        }
    }

    ImGui::Separator();
    ImGui::Text("Active: %d", m_activeCell);
    ImGui::Text("Pattern: %zu points", m_pattern.size());

    ImGui::End();
}

} // namespace oscilloplot
