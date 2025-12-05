#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>

namespace oscilloplot {

//==============================================================================
// Stroke-Based 3D Font for Oscilloscope Display
//==============================================================================
// Each character is defined as a series of strokes (pen movements).
// A stroke is a connected series of points. Multiple strokes per character
// allow for disconnected elements (like the dot on 'i' or crossbar on 'A').
//
// Coordinates are normalized: X in [0, 1], Y in [0, 1]
// Origin (0,0) is bottom-left of character cell.
//==============================================================================

struct Point2D {
    float x, y;
    Point2D(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
};

struct CharacterDef {
    std::vector<std::vector<Point2D>> strokes;  // Multiple strokes per character
    float width = 1.0f;  // Character width (for spacing)
};

class StrokeFont {
public:
    StrokeFont() {
        initializeFont();
    }

    // Get character definition (returns empty if not found)
    const CharacterDef& getChar(char c) const {
        auto it = m_chars.find(c);
        if (it != m_chars.end()) {
            return it->second;
        }
        return m_emptyChar;
    }

    // Check if character is defined
    bool hasChar(char c) const {
        return m_chars.find(c) != m_chars.end();
    }

    // Get character spacing
    float getSpacing() const { return 0.2f; }

private:
    std::unordered_map<char, CharacterDef> m_chars;
    CharacterDef m_emptyChar;

    void initializeFont() {
        // Define a technical/block letter stroke font
        // All characters fit in a 1.0 x 1.0 box (width x height)
        // Some characters are narrower (like 'I', 'l', '1')

        //======================================================================
        // UPPERCASE LETTERS
        //======================================================================

        // A
        m_chars['A'] = {{
            {{0.0f, 0.0f}, {0.5f, 1.0f}, {1.0f, 0.0f}},  // Main shape
            {{0.2f, 0.4f}, {0.8f, 0.4f}}  // Crossbar
        }, 1.0f};

        // B
        m_chars['B'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}, {0.7f, 1.0f}, {0.9f, 0.85f}, {0.9f, 0.65f},
             {0.7f, 0.5f}, {0.0f, 0.5f}},
            {{0.7f, 0.5f}, {0.9f, 0.35f}, {0.9f, 0.15f}, {0.7f, 0.0f}, {0.0f, 0.0f}}
        }, 1.0f};

        // C
        m_chars['C'] = {{
            {{1.0f, 0.2f}, {0.8f, 0.0f}, {0.2f, 0.0f}, {0.0f, 0.2f}, {0.0f, 0.8f},
             {0.2f, 1.0f}, {0.8f, 1.0f}, {1.0f, 0.8f}}
        }, 1.0f};

        // D
        m_chars['D'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}, {0.6f, 1.0f}, {0.9f, 0.8f}, {1.0f, 0.5f},
             {0.9f, 0.2f}, {0.6f, 0.0f}, {0.0f, 0.0f}}
        }, 1.0f};

        // E
        m_chars['E'] = {{
            {{1.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}},
            {{0.0f, 0.5f}, {0.8f, 0.5f}}
        }, 1.0f};

        // F
        m_chars['F'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f}},
            {{0.0f, 0.5f}, {0.8f, 0.5f}}
        }, 1.0f};

        // G
        m_chars['G'] = {{
            {{1.0f, 0.8f}, {0.8f, 1.0f}, {0.2f, 1.0f}, {0.0f, 0.8f}, {0.0f, 0.2f},
             {0.2f, 0.0f}, {0.8f, 0.0f}, {1.0f, 0.2f}, {1.0f, 0.5f}, {0.5f, 0.5f}}
        }, 1.0f};

        // H
        m_chars['H'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}},
            {{1.0f, 0.0f}, {1.0f, 1.0f}},
            {{0.0f, 0.5f}, {1.0f, 0.5f}}
        }, 1.0f};

        // I
        m_chars['I'] = {{
            {{0.0f, 0.0f}, {0.6f, 0.0f}},
            {{0.3f, 0.0f}, {0.3f, 1.0f}},
            {{0.0f, 1.0f}, {0.6f, 1.0f}}
        }, 0.6f};

        // J
        m_chars['J'] = {{
            {{0.0f, 0.2f}, {0.2f, 0.0f}, {0.6f, 0.0f}, {0.8f, 0.2f}, {0.8f, 1.0f}},
            {{0.4f, 1.0f}, {1.0f, 1.0f}}
        }, 1.0f};

        // K
        m_chars['K'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}},
            {{1.0f, 1.0f}, {0.0f, 0.4f}, {1.0f, 0.0f}}
        }, 1.0f};

        // L
        m_chars['L'] = {{
            {{0.0f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f}}
        }, 1.0f};

        // M
        m_chars['M'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}, {0.5f, 0.5f}, {1.0f, 1.0f}, {1.0f, 0.0f}}
        }, 1.0f};

        // N
        m_chars['N'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}}
        }, 1.0f};

        // O
        m_chars['O'] = {{
            {{0.2f, 0.0f}, {0.8f, 0.0f}, {1.0f, 0.2f}, {1.0f, 0.8f}, {0.8f, 1.0f},
             {0.2f, 1.0f}, {0.0f, 0.8f}, {0.0f, 0.2f}, {0.2f, 0.0f}}
        }, 1.0f};

        // P
        m_chars['P'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}, {0.7f, 1.0f}, {0.9f, 0.85f}, {0.9f, 0.65f},
             {0.7f, 0.5f}, {0.0f, 0.5f}}
        }, 1.0f};

        // Q
        m_chars['Q'] = {{
            {{0.2f, 0.0f}, {0.8f, 0.0f}, {1.0f, 0.2f}, {1.0f, 0.8f}, {0.8f, 1.0f},
             {0.2f, 1.0f}, {0.0f, 0.8f}, {0.0f, 0.2f}, {0.2f, 0.0f}},
            {{0.6f, 0.3f}, {1.0f, 0.0f}}  // Tail
        }, 1.0f};

        // R
        m_chars['R'] = {{
            {{0.0f, 0.0f}, {0.0f, 1.0f}, {0.7f, 1.0f}, {0.9f, 0.85f}, {0.9f, 0.65f},
             {0.7f, 0.5f}, {0.0f, 0.5f}},
            {{0.5f, 0.5f}, {1.0f, 0.0f}}  // Leg
        }, 1.0f};

        // S
        m_chars['S'] = {{
            {{1.0f, 0.85f}, {0.8f, 1.0f}, {0.2f, 1.0f}, {0.0f, 0.85f}, {0.0f, 0.65f},
             {0.2f, 0.5f}, {0.8f, 0.5f}, {1.0f, 0.35f}, {1.0f, 0.15f}, {0.8f, 0.0f},
             {0.2f, 0.0f}, {0.0f, 0.15f}}
        }, 1.0f};

        // T
        m_chars['T'] = {{
            {{0.0f, 1.0f}, {1.0f, 1.0f}},
            {{0.5f, 1.0f}, {0.5f, 0.0f}}
        }, 1.0f};

        // U
        m_chars['U'] = {{
            {{0.0f, 1.0f}, {0.0f, 0.2f}, {0.2f, 0.0f}, {0.8f, 0.0f}, {1.0f, 0.2f}, {1.0f, 1.0f}}
        }, 1.0f};

        // V
        m_chars['V'] = {{
            {{0.0f, 1.0f}, {0.5f, 0.0f}, {1.0f, 1.0f}}
        }, 1.0f};

        // W
        m_chars['W'] = {{
            {{0.0f, 1.0f}, {0.25f, 0.0f}, {0.5f, 0.5f}, {0.75f, 0.0f}, {1.0f, 1.0f}}
        }, 1.0f};

        // X
        m_chars['X'] = {{
            {{0.0f, 0.0f}, {1.0f, 1.0f}},
            {{0.0f, 1.0f}, {1.0f, 0.0f}}
        }, 1.0f};

        // Y
        m_chars['Y'] = {{
            {{0.0f, 1.0f}, {0.5f, 0.5f}, {1.0f, 1.0f}},
            {{0.5f, 0.5f}, {0.5f, 0.0f}}
        }, 1.0f};

        // Z
        m_chars['Z'] = {{
            {{0.0f, 1.0f}, {1.0f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f}}
        }, 1.0f};

        //======================================================================
        // LOWERCASE LETTERS (simplified - same as uppercase but smaller region)
        //======================================================================

        // For simplicity, map lowercase to uppercase
        for (char c = 'a'; c <= 'z'; ++c) {
            m_chars[c] = m_chars[c - 32];  // Copy from uppercase
        }

        //======================================================================
        // NUMBERS
        //======================================================================

        // 0
        m_chars['0'] = {{
            {{0.2f, 0.0f}, {0.8f, 0.0f}, {1.0f, 0.2f}, {1.0f, 0.8f}, {0.8f, 1.0f},
             {0.2f, 1.0f}, {0.0f, 0.8f}, {0.0f, 0.2f}, {0.2f, 0.0f}},
            {{0.2f, 0.2f}, {0.8f, 0.8f}}  // Diagonal slash
        }, 1.0f};

        // 1
        m_chars['1'] = {{
            {{0.2f, 0.8f}, {0.5f, 1.0f}, {0.5f, 0.0f}},
            {{0.2f, 0.0f}, {0.8f, 0.0f}}
        }, 0.7f};

        // 2
        m_chars['2'] = {{
            {{0.0f, 0.8f}, {0.2f, 1.0f}, {0.8f, 1.0f}, {1.0f, 0.8f}, {1.0f, 0.6f},
             {0.0f, 0.0f}, {1.0f, 0.0f}}
        }, 1.0f};

        // 3
        m_chars['3'] = {{
            {{0.0f, 0.85f}, {0.2f, 1.0f}, {0.8f, 1.0f}, {1.0f, 0.85f}, {1.0f, 0.65f},
             {0.8f, 0.5f}, {0.4f, 0.5f}},
            {{0.8f, 0.5f}, {1.0f, 0.35f}, {1.0f, 0.15f}, {0.8f, 0.0f}, {0.2f, 0.0f}, {0.0f, 0.15f}}
        }, 1.0f};

        // 4
        m_chars['4'] = {{
            {{0.8f, 0.0f}, {0.8f, 1.0f}, {0.0f, 0.3f}, {1.0f, 0.3f}}
        }, 1.0f};

        // 5
        m_chars['5'] = {{
            {{1.0f, 1.0f}, {0.0f, 1.0f}, {0.0f, 0.55f}, {0.7f, 0.55f}, {1.0f, 0.4f},
             {1.0f, 0.15f}, {0.8f, 0.0f}, {0.2f, 0.0f}, {0.0f, 0.15f}}
        }, 1.0f};

        // 6
        m_chars['6'] = {{
            {{0.8f, 1.0f}, {0.2f, 1.0f}, {0.0f, 0.8f}, {0.0f, 0.2f}, {0.2f, 0.0f},
             {0.8f, 0.0f}, {1.0f, 0.2f}, {1.0f, 0.4f}, {0.8f, 0.55f}, {0.0f, 0.55f}}
        }, 1.0f};

        // 7
        m_chars['7'] = {{
            {{0.0f, 1.0f}, {1.0f, 1.0f}, {0.3f, 0.0f}}
        }, 1.0f};

        // 8
        m_chars['8'] = {{
            {{0.5f, 0.5f}, {0.2f, 0.5f}, {0.0f, 0.65f}, {0.0f, 0.85f}, {0.2f, 1.0f},
             {0.8f, 1.0f}, {1.0f, 0.85f}, {1.0f, 0.65f}, {0.8f, 0.5f}, {0.5f, 0.5f}},
            {{0.5f, 0.5f}, {0.2f, 0.5f}, {0.0f, 0.35f}, {0.0f, 0.15f}, {0.2f, 0.0f},
             {0.8f, 0.0f}, {1.0f, 0.15f}, {1.0f, 0.35f}, {0.8f, 0.5f}}
        }, 1.0f};

        // 9
        m_chars['9'] = {{
            {{0.2f, 0.0f}, {0.8f, 0.0f}, {1.0f, 0.2f}, {1.0f, 0.8f}, {0.8f, 1.0f},
             {0.2f, 1.0f}, {0.0f, 0.8f}, {0.0f, 0.6f}, {0.2f, 0.45f}, {1.0f, 0.45f}}
        }, 1.0f};

        //======================================================================
        // PUNCTUATION & SYMBOLS
        //======================================================================

        // Space
        m_chars[' '] = {{}, 0.5f};

        // Period .
        m_chars['.'] = {{
            {{0.2f, 0.0f}, {0.4f, 0.0f}, {0.4f, 0.15f}, {0.2f, 0.15f}, {0.2f, 0.0f}}
        }, 0.5f};

        // Comma ,
        m_chars[','] = {{
            {{0.3f, 0.15f}, {0.3f, 0.0f}, {0.1f, -0.15f}}
        }, 0.5f};

        // Exclamation !
        m_chars['!'] = {{
            {{0.3f, 0.3f}, {0.3f, 1.0f}},
            {{0.2f, 0.0f}, {0.4f, 0.0f}, {0.4f, 0.12f}, {0.2f, 0.12f}, {0.2f, 0.0f}}
        }, 0.5f};

        // Question ?
        m_chars['?'] = {{
            {{0.0f, 0.8f}, {0.2f, 1.0f}, {0.8f, 1.0f}, {1.0f, 0.8f}, {1.0f, 0.6f},
             {0.5f, 0.4f}, {0.5f, 0.25f}},
            {{0.4f, 0.0f}, {0.6f, 0.0f}, {0.6f, 0.12f}, {0.4f, 0.12f}, {0.4f, 0.0f}}
        }, 1.0f};

        // Hyphen -
        m_chars['-'] = {{
            {{0.1f, 0.5f}, {0.9f, 0.5f}}
        }, 0.8f};

        // Plus +
        m_chars['+'] = {{
            {{0.1f, 0.5f}, {0.9f, 0.5f}},
            {{0.5f, 0.2f}, {0.5f, 0.8f}}
        }, 1.0f};

        // Equals =
        m_chars['='] = {{
            {{0.1f, 0.35f}, {0.9f, 0.35f}},
            {{0.1f, 0.65f}, {0.9f, 0.65f}}
        }, 1.0f};

        // Asterisk *
        m_chars['*'] = {{
            {{0.5f, 0.3f}, {0.5f, 0.9f}},
            {{0.2f, 0.45f}, {0.8f, 0.75f}},
            {{0.2f, 0.75f}, {0.8f, 0.45f}}
        }, 0.8f};

        // Slash /
        m_chars['/'] = {{
            {{0.0f, 0.0f}, {1.0f, 1.0f}}
        }, 0.8f};

        // Colon :
        m_chars[':'] = {{
            {{0.25f, 0.25f}, {0.35f, 0.25f}, {0.35f, 0.35f}, {0.25f, 0.35f}, {0.25f, 0.25f}},
            {{0.25f, 0.65f}, {0.35f, 0.65f}, {0.35f, 0.75f}, {0.25f, 0.75f}, {0.25f, 0.65f}}
        }, 0.5f};

        // At @
        m_chars['@'] = {{
            {{0.7f, 0.3f}, {0.5f, 0.2f}, {0.4f, 0.3f}, {0.4f, 0.5f}, {0.5f, 0.6f},
             {0.7f, 0.6f}, {0.8f, 0.5f}, {0.8f, 0.2f}, {0.9f, 0.1f}, {1.0f, 0.2f},
             {1.0f, 0.8f}, {0.8f, 1.0f}, {0.2f, 1.0f}, {0.0f, 0.8f}, {0.0f, 0.2f},
             {0.2f, 0.0f}, {0.9f, 0.0f}}
        }, 1.2f};

        // Hash #
        m_chars['#'] = {{
            {{0.2f, 0.0f}, {0.35f, 1.0f}},
            {{0.65f, 0.0f}, {0.8f, 1.0f}},
            {{0.0f, 0.35f}, {1.0f, 0.35f}},
            {{0.0f, 0.65f}, {1.0f, 0.65f}}
        }, 1.0f};

        // Heart <3
        m_chars['<'] = {{
            {{0.8f, 0.5f}, {0.2f, 1.0f}, {0.2f, 0.0f}, {0.8f, 0.5f}}
        }, 0.8f};

        // Greater >
        m_chars['>'] = {{
            {{0.2f, 1.0f}, {0.8f, 0.5f}, {0.2f, 0.0f}}
        }, 0.8f};

        // Parentheses
        m_chars['('] = {{
            {{0.6f, 1.0f}, {0.3f, 0.8f}, {0.2f, 0.5f}, {0.3f, 0.2f}, {0.6f, 0.0f}}
        }, 0.5f};

        m_chars[')'] = {{
            {{0.2f, 1.0f}, {0.5f, 0.8f}, {0.6f, 0.5f}, {0.5f, 0.2f}, {0.2f, 0.0f}}
        }, 0.5f};

        // Brackets
        m_chars['['] = {{
            {{0.6f, 1.0f}, {0.2f, 1.0f}, {0.2f, 0.0f}, {0.6f, 0.0f}}
        }, 0.5f};

        m_chars[']'] = {{
            {{0.2f, 1.0f}, {0.6f, 1.0f}, {0.6f, 0.0f}, {0.2f, 0.0f}}
        }, 0.5f};

        // Underscore _
        m_chars['_'] = {{
            {{0.0f, 0.0f}, {1.0f, 0.0f}}
        }, 1.0f};

        // Apostrophe '
        m_chars['\''] = {{
            {{0.3f, 0.8f}, {0.3f, 1.0f}}
        }, 0.4f};

        // Quote "
        m_chars['"'] = {{
            {{0.2f, 0.8f}, {0.2f, 1.0f}},
            {{0.5f, 0.8f}, {0.5f, 1.0f}}
        }, 0.6f};

        // Ampersand &
        m_chars['&'] = {{
            {{1.0f, 0.0f}, {0.3f, 0.5f}, {0.3f, 0.7f}, {0.4f, 0.85f}, {0.6f, 0.85f},
             {0.7f, 0.7f}, {0.7f, 0.55f}, {0.0f, 0.0f}, {0.3f, 0.0f}, {0.5f, 0.15f},
             {1.0f, 0.5f}}
        }, 1.0f};
    }
};

//==============================================================================
// Text to 3D Points Converter
//==============================================================================

class Text3DGenerator {
public:
    // Generate 3D points for a text string
    // depth: extrusion depth (0 = flat 2D text)
    // connectFrontBack: whether to add edges connecting front and back faces
    static void generateText3D(
        const std::string& text,
        float charHeight,
        float depth,
        bool connectFrontBack,
        std::vector<float>& outX,
        std::vector<float>& outY,
        std::vector<size_t>& strokeStarts  // Track stroke boundaries
    ) {
        static StrokeFont font;

        outX.clear();
        outY.clear();
        strokeStarts.clear();

        float cursorX = 0.0f;
        float totalWidth = calculateTextWidth(text, charHeight);
        float startX = -totalWidth / 2.0f;  // Center the text

        cursorX = startX;

        for (char c : text) {
            const CharacterDef& charDef = font.getChar(c);

            for (const auto& stroke : charDef.strokes) {
                if (stroke.empty()) continue;

                // Record stroke start
                strokeStarts.push_back(outX.size());

                if (depth > 0.001f && connectFrontBack) {
                    // 3D extrusion: draw front face, back face, and connecting edges

                    // Front face (z = -depth/2)
                    for (const auto& pt : stroke) {
                        float x = cursorX + pt.x * charHeight;
                        float y = (pt.y - 0.5f) * charHeight;  // Center vertically
                        outX.push_back(x);
                        outY.push_back(y);
                    }

                    // Mark new stroke for back face
                    strokeStarts.push_back(outX.size());

                    // Back face (z = +depth/2) - we'll handle Z in the rotation later
                    // For now, offset slightly in X to simulate depth perspective
                    float depthOffset = depth * 0.1f;  // Perspective effect
                    for (const auto& pt : stroke) {
                        float x = cursorX + pt.x * charHeight + depthOffset;
                        float y = (pt.y - 0.5f) * charHeight + depthOffset * 0.5f;
                        outX.push_back(x);
                        outY.push_back(y);
                    }

                    // Connecting edges (vertical lines between front and back)
                    for (size_t i = 0; i < stroke.size(); i += std::max(size_t(1), stroke.size() / 4)) {
                        strokeStarts.push_back(outX.size());
                        float x1 = cursorX + stroke[i].x * charHeight;
                        float y1 = (stroke[i].y - 0.5f) * charHeight;
                        float x2 = x1 + depthOffset;
                        float y2 = y1 + depthOffset * 0.5f;
                        outX.push_back(x1);
                        outY.push_back(y1);
                        outX.push_back(x2);
                        outY.push_back(y2);
                    }
                } else {
                    // Flat 2D text
                    for (const auto& pt : stroke) {
                        float x = cursorX + pt.x * charHeight;
                        float y = (pt.y - 0.5f) * charHeight;  // Center vertically
                        outX.push_back(x);
                        outY.push_back(y);
                    }
                }
            }

            cursorX += (charDef.width + font.getSpacing()) * charHeight;
        }

        // Normalize to fit in -1 to 1 range
        if (!outX.empty()) {
            float maxExtent = 0.0f;
            for (size_t i = 0; i < outX.size(); ++i) {
                maxExtent = std::max(maxExtent, std::abs(outX[i]));
                maxExtent = std::max(maxExtent, std::abs(outY[i]));
            }
            if (maxExtent > 0.9f) {
                float scale = 0.9f / maxExtent;
                for (size_t i = 0; i < outX.size(); ++i) {
                    outX[i] *= scale;
                    outY[i] *= scale;
                }
            }
        }
    }

    // Calculate total width of text string
    static float calculateTextWidth(const std::string& text, float charHeight) {
        static StrokeFont font;
        float width = 0.0f;
        for (size_t i = 0; i < text.size(); ++i) {
            const CharacterDef& charDef = font.getChar(text[i]);
            width += charDef.width * charHeight;
            if (i < text.size() - 1) {
                width += font.getSpacing() * charHeight;
            }
        }
        return width;
    }
};

} // namespace oscilloplot
