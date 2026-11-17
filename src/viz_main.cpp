#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include "map.hpp"
#include "metrics.hpp"
#include "raylib.h"
#include "sim.hpp"
#include "scenario.hpp"
#include "navfield.hpp"

using namespace crowd;

static Color color_for_goal(float x, float y, float gx, float gy)
{
    float ang = std::atan2(gy - y, gx - x);
    float hue = (ang + 3.14159265f) * (180.f / 3.14159265f);
    return ColorFromHSV(hue, 0.75f, 0.95f);
}

static Scenario make_head_on(const Params& p, float y_offset)
{
    Scenario sc = make_antipodal_disk(2, p);
    sc.agents.x[0] = -5.f; sc.agents.y[0] = 0.f;
    sc.agents.gx[0] = 5.f; sc.agents.gy[0] = 0.f;
    sc.agents.x[1] = 5.f;  sc.agents.y[1] = y_offset;
    sc.agents.gx[1] = -5.f; sc.agents.gy[1] = y_offset;
    sc.agents.vx[0] = p.v_max;  sc.agents.vy[0] = 0.f;
    sc.agents.vx[1] = -p.v_max; sc.agents.vy[1] = 0.f;
    return sc;
}

int main(int argc, char** argv)
{
    Params p;

    // --map <fajl>        : samo prikaz mape (korak 1)
    // --sim <fajl> [N]    : simulacija na mapi (korak 2)
    Map map;
    bool map_mode = false, map_sim = false;
    if (argc > 2 && (std::string(argv[1]) == "--map" || std::string(argv[1]) == "--sim"))
    {
        std::string err;
        if (!Map::load(argv[2], map, err))
        {
            std::fprintf(stderr, "Greska pri ucitavanju mape: %s\n", err.c_str());
            return 1;
        }
        map_mode = std::string(argv[1]) == "--map";
        map_sim = !map_mode;
    }
    const bool show_map = map_mode || map_sim;

    const bool head_on = (!show_map && argc > 1 && std::string(argv[1]) == "headon");
    const std::size_t n = map_mode ? 1
        : map_sim ? ((argc > 3) ? std::strtoull(argv[3], nullptr, 10) : 30)
        : head_on ? 2
        : (argc > 1) ? std::strtoull(argv[1], nullptr, 10) : 500;

    // u map modu je Sim samo placeholder (1 agent), ne crta se i ne pokrece
    const Scenario sc = map_sim ? make_map_scenario(map, n, p)
        : head_on ? make_head_on(p, 0.0f)
        : make_antipodal_disk(n, p);
    const Map* mp = map_sim ? &map : nullptr;
    Sim sim(sc.agents, sc.world, p, mp);

    // prozor promjenjive velicine, odmah maksimiziran; F11 = fullscreen bez okvira
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1280, 800, "crowd_viz");
    MaximizeWindow();
    SetTargetFPS(60);

    float min_x = sc.world.min_x, max_x = sc.world.max_x;
    float min_y = sc.world.min_y, max_y = sc.world.max_y;
    if (show_map)
    {
        min_x = 0.f; min_y = 0.f;
        max_x = map.widthWorld(); max_y = map.heightWorld();
    }

    // kamera koja uklapa cijeli svijet u trenutnu velicinu prozora
    auto fit_camera = [&]() {
        const float sw = static_cast<float>(GetScreenWidth());
        const float sh = static_cast<float>(GetScreenHeight());
        Camera2D c{};
        c.offset = {sw / 2.f, sh / 2.f};
        c.target = {(min_x + max_x) / 2.f, (min_y + max_y) / 2.f};
        c.zoom = 0.92f * std::min(sw / (max_x - min_x), sh / (max_y - min_y));
        return c;
    };
    Camera2D cam = fit_camera();
    bool cam_user = false;   // korisnik je pomjerao/zumirao -> ne refitaj na resize

    bool paused = false, show_goals = false, show_segments = true, show_field = false;
    bool show_markers = false;   // C: markeri ciljeva + oznaka agenata koji su stigli
    int steps_per_frame = 1;
    long long steps_done = 0;   // broj odradjenih koraka simulacije (za HUD)
    long long finish_step = -1; // korak u kojem su svi stigli na cilj (-1 = jos nisu)

    // T: tragovi putanja, kruzni bafer zadnjih TRAIL_LEN pozicija po agentu
    bool show_trails = false;
    constexpr int TRAIL_LEN = 40;
    std::vector<Vector2> trail(sim.state().size() * TRAIL_LEN);
    int trail_head = 0, trail_count = 0;
    auto record_trail = [&]() {
        const Agents& s = sim.state();
        for (std::size_t i = 0; i < s.size(); ++i)
            trail[i * TRAIL_LEN + trail_head] = {s.x[i], s.y[i]};
        trail_head = (trail_head + 1) % TRAIL_LEN;
        trail_count = std::min(trail_count + 1, TRAIL_LEN);
    };
    record_trail();

    while (!WindowShouldClose())
    {
        // ---- ulaz ----
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_G)) show_goals = !show_goals;
        if (IsKeyPressed(KEY_C)) show_markers = !show_markers;
        if (IsKeyPressed(KEY_L)) show_segments = !show_segments;
        if (IsKeyPressed(KEY_F)) show_field = !show_field;
        if (IsKeyPressed(KEY_UP))   steps_per_frame = std::min(steps_per_frame * 2, 64);
        if (IsKeyPressed(KEY_DOWN)) steps_per_frame = std::max(steps_per_frame / 2, 1);
        if (IsKeyPressed(KEY_HOME)) { cam = fit_camera(); cam_user = false; }
        if (IsKeyPressed(KEY_R))    { sim = Sim(sc.agents, sc.world, p, mp); paused = true; steps_done = 0; finish_step = -1; trail_count = 0; record_trail(); }
        if (IsKeyPressed(KEY_T))    show_trails = !show_trails;
        if (IsKeyPressed(KEY_F11))  ToggleBorderlessWindowed();

        if (IsWindowResized())
        {
            if (cam_user)
                cam.offset = {GetScreenWidth() / 2.f, GetScreenHeight() / 2.f};
            else
                cam = fit_camera();
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 d = GetMouseDelta();
            cam.target.x -= d.x / cam.zoom;
            cam.target.y -= d.y / cam.zoom;
            if (d.x != 0.f || d.y != 0.f) cam_user = true;
        }
        float wheel = GetMouseWheelMove();
        if (wheel != 0.f)
        {
            cam_user = true;
            Vector2 before = GetScreenToWorld2D(GetMousePosition(), cam);
            cam.zoom = std::clamp(cam.zoom * (1.f + 0.15f * wheel), 0.5f, 200.f);
            Vector2 after = GetScreenToWorld2D(GetMousePosition(), cam);
            cam.target.x += before.x - after.x;
            cam.target.y += before.y - after.y;
        }

        // ---- simulacija; u map modu se ne pokrece ----
        if (!map_mode)
        {
            int steps = !paused ? steps_per_frame : (IsKeyPressed(KEY_RIGHT) ? 1 : 0);
            for (int k = 0; k < steps; ++k)
            {
                sim.step();
                ++steps_done;
                record_trail();
                // svi stigli prvi put -> automatska pauza, vrijeme ostaje zamrznuto
                if (finish_step < 0 && goalReachedCount(sim.state(), 0.25f) == sim.state().size())
                {
                    finish_step = steps_done;
                    paused = true;
                    break;
                }
            }
        }

        // ---- crtanje ----
        const Agents& a = sim.state();
        BeginDrawing();
        ClearBackground({18, 18, 24, 255});
        BeginMode2D(cam);

        if (show_map)
        {
            for (int cy = 0; cy < map.h; ++cy)
                for (int cx = 0; cx < map.w; ++cx)
                {
                    const std::size_t i = static_cast<std::size_t>(cy) * map.w + cx;
                    Color c = {45, 47, 58, 255};
                    if (map.wall[i])  c = {20, 20, 26, 255};
                    DrawRectangleV({cx * map.cell, cy * map.cell},
                                   {map.cell, map.cell}, c);
                }
            if (show_segments)
                for (const Segment& s : map.segments)
                    DrawLineV({s.a.x, s.a.y}, {s.b.x, s.b.y}, YELLOW);

            // strelice navigacijskog polja za cilj agenta 0
            if (map_sim && show_field && a.size() > 0)
            {
                const NavField& f = sim.nav().at(sim.agent_field(0));
                for (int cy = 0; cy < map.h; ++cy)
                    for (int cx = 0; cx < map.w; ++cx)
                    {
                        const std::size_t i = static_cast<std::size_t>(cy) * map.w + cx;
                        if (f.cost[i] >= 1e29f) continue;
                        const float px = (cx + 0.5f) * map.cell, py = (cy + 0.5f) * map.cell;
                        DrawLineV({px, py}, {px + f.dirx[i] * 0.4f * map.cell,
                                             py + f.diry[i] * 0.4f * map.cell}, SKYBLUE);
                    }
            }
        }
        else
        {
            DrawCircleLinesV({0.f, 0.f}, 0.1f, GRAY);   // ishodiste
        }

        if (!map_mode)
        {
            const float r = p.radius;

            // tragovi putanja: od najstarije (providno) do najnovije pozicije
            if (show_trails && trail_count > 1)
                for (std::size_t i = 0; i < a.size(); ++i)
                {
                    Color c = color_for_goal(sc.agents.x[i], sc.agents.y[i],
                                             a.gx[i], a.gy[i]);
                    const int start = (trail_head - trail_count + TRAIL_LEN) % TRAIL_LEN;
                    for (int k = 1; k < trail_count; ++k)
                    {
                        const Vector2 p0 = trail[i * TRAIL_LEN + (start + k - 1) % TRAIL_LEN];
                        const Vector2 p1 = trail[i * TRAIL_LEN + (start + k) % TRAIL_LEN];
                        const float t = static_cast<float>(k) / trail_count;
                        DrawLineEx(p0, p1, 0.25f * r, Fade(c, 0.5f * t));
                    }
                }

            // markeri ciljeva (ispod agenata): prsten + krstic u boji agenta
            if (show_markers)
                for (std::size_t i = 0; i < a.size(); ++i)
                {
                    Color c = color_for_goal(sc.agents.x[i], sc.agents.y[i],
                                             a.gx[i], a.gy[i]);
                    const Vector2 g = {a.gx[i], a.gy[i]};
                    DrawRing(g, 0.55f * r, 0.75f * r, 0.f, 360.f, 24, Fade(c, 0.8f));
                    DrawLineEx({g.x - 0.4f * r, g.y}, {g.x + 0.4f * r, g.y}, 0.12f * r, Fade(c, 0.8f));
                    DrawLineEx({g.x, g.y - 0.4f * r}, {g.x, g.y + 0.4f * r}, 0.12f * r, Fade(c, 0.8f));
                }

            for (std::size_t i = 0; i < a.size(); ++i)
            {
                Color c = color_for_goal(sc.agents.x[i], sc.agents.y[i],
                                         a.gx[i], a.gy[i]);
                const float dx = a.gx[i] - a.x[i], dy = a.gy[i] - a.y[i];
                const bool arrived = dx * dx + dy * dy < 0.25f * 0.25f;
                if (show_goals && !arrived)
                    DrawLineV({a.x[i], a.y[i]}, {a.gx[i], a.gy[i]}, Fade(c, 0.15f));
                if (arrived && show_markers)
                {
                    // stigao: zatamnjen agent sa bijelim obrubom
                    DrawCircleV({a.x[i], a.y[i]}, r, Fade(c, 0.35f));
                    DrawRing({a.x[i], a.y[i]}, 0.85f * r, r, 0.f, 360.f, 24, Fade(RAYWHITE, 0.7f));
                }
                else
                    DrawCircleV({a.x[i], a.y[i]}, r, c);
            }
        }
        EndMode2D();

        // ---- HUD ----
        const int sw = GetScreenWidth(), sh = GetScreenHeight();
        const Color bar_bg = {10, 10, 14, 200};

        // donja traka: [tipka] opis; ukljucene opcije svijetle, iskljucene sive,
        // obicne komande (bez stanja) srednje sive. Prelama se ako ne stane.
        struct KeyItem { const char* key; const char* label; int state; }; // -1 = komanda
        auto draw_key_bar = [&](const std::vector<KeyItem>& items) {
            const int fs = 16, pad = 8, gap = 18, line_h = fs + 6;
            int lines = 1, x = pad;
            for (const KeyItem& it : items)
            {
                const int wdt = MeasureText(TextFormat("[%s] %s", it.key, it.label), fs);
                if (x + wdt > sw - pad && x > pad) { ++lines; x = pad; }
                x += wdt + gap;
            }
            const int bar_h = lines * line_h + pad;
            DrawRectangle(0, sh - bar_h, sw, bar_h, bar_bg);
            x = pad;
            int y = sh - bar_h + pad / 2 + 3;
            for (const KeyItem& it : items)
            {
                const char* txt = TextFormat("[%s] %s", it.key, it.label);
                const int wdt = MeasureText(txt, fs);
                if (x + wdt > sw - pad && x > pad) { x = pad; y += line_h; }
                const Color col = it.state == 1 ? Color{255, 214, 90, 255}
                                : it.state == 0 ? Color{95, 95, 105, 255}
                                                : Color{160, 160, 170, 255};
                DrawText(txt, x, y, fs, col);
                x += wdt + gap;
            }
        };

        if (map_mode)
        {
            DrawRectangle(0, 0, sw, 36, bar_bg);
            DrawText(TextFormat("mapa %dx%d   segmenata %zu   %d FPS",
                                map.w, map.h, map.segments.size(), GetFPS()),
                     10, 8, 20, RAYWHITE);
            draw_key_bar({{"L", "konture zidova", show_segments ? 1 : 0},
                          {"HOME", "kamera", -1}, {"F11", "fullscreen", -1},
                          {"tocak", "zoom", -1}, {"LMB", "pan", -1}});
        }
        else
        {
            const std::size_t reached = goalReachedCount(a, 0.25f);
            DrawRectangle(0, 0, sw, 58, bar_bg);
            DrawText(TextFormat("t = %.1f s     korak %lld     brzina x%d",
                                steps_done * p.dt, steps_done, steps_per_frame),
                     10, 8, 20, RAYWHITE);
            DrawText(TextFormat("N=%zu    na cilju %zu/%zu    preklapanja %zu    min udalj. %.2f    %d FPS",
                                a.size(), reached, a.size(), overlapCount(a, p.radius),
                                minPairDistance(a), GetFPS()),
                     10, 33, 18, Color{180, 180, 190, 255});
            if (finish_step >= 0)
            {
                const char* dtxt = TextFormat("SVI NA CILJU za %.1f s", finish_step * p.dt);
                const int dw = MeasureText(dtxt, 22);
                DrawText(dtxt, sw - dw - 16, 18, 22, Color{120, 220, 120, 255});
            }
            else if (paused)
            {
                const char* ptxt = "|| PAUZA";
                const int pw = MeasureText(ptxt, 24);
                DrawText(ptxt, sw - pw - 16, 17, 24, Color{255, 200, 40, 255});
            }
            draw_key_bar({{"SPACE", "pauza", -1}, {"RIGHT", "korak", -1},
                          {"UP/DOWN", "brzina", -1},
                          {"G", "linije", show_goals ? 1 : 0},
                          {"C", "ciljevi", show_markers ? 1 : 0},
                          {"T", "tragovi", show_trails ? 1 : 0},
                          {"F", "polje", show_field ? 1 : 0},
                          {"L", "zidovi", show_segments ? 1 : 0},
                          {"R", "reset", -1}, {"HOME", "kamera", -1},
                          {"F11", "fullscreen", -1}, {"tocak", "zoom", -1},
                          {"LMB", "pan", -1}});
        }
        EndDrawing();
    }
    CloseWindow();
    return 0;
}