#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include "raylib.h"
#include "sim.hpp"
#include "scenario.hpp"

using namespace crowd;

static Color color_for_goal(float x, float y, float gx, float gy)
{
    // boja po smjeru kretanja (hue = ugao), da se vidi antipodalna struktura
    float ang = std::atan2(gy - y, gx - x);            // [-pi, pi]
    float hue = (ang + 3.14159265f) * (180.f / 3.14159265f);
    return ColorFromHSV(hue, 0.75f, 0.95f);
}

int main(int argc, char** argv)
{
    std::size_t n = (argc > 1) ? std::strtoull(argv[1], nullptr, 10) : 500;

    Params p;
    const Scenario sc = make_antipodal_disk(n, p);
    Sim sim(sc.agents, sc.world, p);

    const int W = 1280, H = 800;
    InitWindow(W, H, "crowd_viz");
    SetTargetFPS(60);

    // kamera: sredina svijeta u sredini ekrana, zoom da cijeli svijet stane
    Camera2D cam{};
    cam.offset = {W / 2.f, H / 2.f};
    cam.target = {(sc.world.min_x + sc.world.max_x) / 2.f,
                  (sc.world.min_y + sc.world.max_y) / 2.f};
    const float ext = std::max(sc.world.max_x - sc.world.min_x,
                               sc.world.max_y - sc.world.min_y);
    cam.zoom = 0.9f * std::min(W, H) / ext;
    const Camera2D cam0 = cam;

    bool paused = false, show_goals = false;
    int steps_per_frame = 1;

    while (!WindowShouldClose())
    {
        // ---- ulaz ----
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_G)) show_goals = !show_goals;
        if (IsKeyPressed(KEY_UP))   steps_per_frame = std::min(steps_per_frame * 2, 64);
        if (IsKeyPressed(KEY_DOWN)) steps_per_frame = std::max(steps_per_frame / 2, 1);
        if (IsKeyPressed(KEY_HOME)) cam = cam0;
        if (IsKeyPressed(KEY_R))    { sim = Sim(sc.agents, sc.world, p); paused = true; }

        // pan: lijevo dugme miša
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 d = GetMouseDelta();
            cam.target.x -= d.x / cam.zoom;
            cam.target.y -= d.y / cam.zoom;
        }
        // zoom prema pokazivacu
        float wheel = GetMouseWheelMove();
        if (wheel != 0.f)
        {
            Vector2 before = GetScreenToWorld2D(GetMousePosition(), cam);
            cam.zoom = std::clamp(cam.zoom * (1.f + 0.15f * wheel), 0.5f, 200.f);
            Vector2 after = GetScreenToWorld2D(GetMousePosition(), cam);
            cam.target.x += before.x - after.x;
            cam.target.y += before.y - after.y;
        }

        // ---- simulacija (odvojena od crtanja) ----
        if (!paused)
            for (int k = 0; k < steps_per_frame; ++k) sim.step();
        else if (IsKeyPressed(KEY_RIGHT))
            sim.step();

        // ---- crtanje ----
        const Agents& a = sim.state();
        BeginDrawing();
        ClearBackground({18, 18, 24, 255});
        BeginMode2D(cam);

        DrawCircleLinesV({0.f, 0.f}, 0.1f, GRAY);   // ishodiste
        for (std::size_t i = 0; i < a.size(); ++i)
        {
            Color c = color_for_goal(sc.agents.x[i], sc.agents.y[i],
                                     a.gx[i], a.gy[i]);
            if (show_goals)
                DrawLineV({a.x[i], a.y[i]}, {a.gx[i], a.gy[i]}, Fade(c, 0.15f));
            DrawCircleV({a.x[i], a.y[i]}, p.radius, c);
        }
        EndMode2D();

        DrawText(TextFormat("N=%zu  korak=%zu  na cilju=%zu  korak/frame=%d  %s",
                            a.size(), sim.steps(), sim.reached(),
                            steps_per_frame, paused ? "[PAUZA]" : ""),
                 10, 10, 20, RAYWHITE);
        DrawText("SPACE pauza | RIGHT korak | UP/DOWN brzina | G ciljevi | R reset | HOME kamera | tocak zoom | LMB pan",
                 10, H - 24, 16, GRAY);
        DrawFPS(W - 90, 10);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}