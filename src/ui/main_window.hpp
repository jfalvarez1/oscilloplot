#pragma once

namespace oscilloplot {

class App;

class MainWindow {
public:
    void render(App& app);

private:
    float m_zoom = 1.0f;
    bool m_showGrid = true;
};

} // namespace oscilloplot
