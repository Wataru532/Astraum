// F3 debug screen: performance, system, temperatures and the full state of the active vessel
#pragma once
struct App;

namespace dbg {
void noteLog(const char* line); // picks the OpenGL device information out of the raylib log
void frame(float dt);           // record the frame time (every frame, so the graph is ready)
void draw(App& app);            // the overlay
} // namespace dbg
