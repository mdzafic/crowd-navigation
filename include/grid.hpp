#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "scenario.hpp"
#include <cstdlib>

namespace crowd
{
#ifdef CROWD_DIAG
    extern unsigned long long g_cand, g_inradius, g_inserted;
#endif

    // Uniformna mreza sa celijama stranice R, gradjena counting sortom.
    //   cell_start[c] .. cell_start[c+1]-1  =  pozicije u sorted_ids za celiju c
    //   sorted_ids[k]                       =  ID agenta
    // Svi baferi se alociraju jednom (u konstruktoru), build() ih samo prepisuje.
    class Grid
    {
    public:
        Grid(const World& world, float cell_size, std::size_t max_agents, int subdiv = 1);

        void build(const float* x, const float* y, std::size_t n);

        int width()  const noexcept { return W_; }
        int height() const noexcept { return H_; }
        float cell_size() const noexcept { return R_; }

        int cell_x(float x) const noexcept { return clamp_x(static_cast<int>(std::floor((x - min_x_) * inv_R_))); }
        int cell_y(float y) const noexcept { return clamp_y(static_cast<int>(std::floor((y - min_y_) * inv_R_))); }
        int cell_index(int cx, int cy) const noexcept { return cy * W_ + cx; }

        // Poziva f(j) za svakog kandidata j (ukljucujuci i samog agenta)
        // iz (2*reach+1)^2 celija oko pozicije (x,y).
        // Optimizacija: tri susjedne celije u istom redu su uzastopne u
        // memoriji, pa su to samo 3 raspona umjesto 9.
        template <class F>
        void for_each_candidate(float x, float y, F&& f) const
        {
            const int cx = cell_x(x), cy = cell_y(y);
            const int x0 = std::max(cx - reach_, 0), x1 = std::min(cx + reach_, W_ - 1);
            const int y0 = std::max(cy - reach_, 0), y1 = std::min(cy + reach_, H_ - 1);
            for (int yy = y0; yy <= y1; ++yy)
            {
                const std::int32_t b = cell_start_[yy * W_ + x0];
                const std::int32_t e = cell_start_[yy * W_ + x1 + 1];
                for (std::int32_t k = b; k < e; ++k) f(sorted_ids_[k]);
            }
        }

        // Poziva f(j) samo za agente j != self koji su stvarno unutar
        // radijusa. Mreza daje kandidate, a ovaj filter provjerava udaljenost.
        template <class F>
        void for_each_neighbor(std::size_t self, const float* x, const float* y,
                               float radius, F&& f) const
        {
            const float xi = x[self], yi = y[self];
            const float radius2 = radius * radius;
            for_each_candidate(xi, yi, [&](std::int32_t j) {
                if (static_cast<std::size_t>(j) == self) return;
                const float dx = x[j] - xi, dy = y[j] - yi;
                if (dx * dx + dy * dy <= radius2) f(j);
            });
        }

        // k najblizih susjeda agenta `self` unutar `radius` (bez samog agenta).
        // Izlaz: ids[0..count) i d2s[0..count), sortirano rastuce po (d2, pozicija
        // u sorted_ids) -- to je TACNO isti poredak kao stari "insert_neighbor nad
        // kandidatima red-po-red", pa je rezultat bit-identican.
        //
        // Zasto je brze (ideja iz RVO2 kd-stabla):
        //  1) celije se obilaze od najblize ka najdaljoj (prsten po prsten), pa se
        //     top-k brzo napuni stvarno bliskim agentima;
        //  2) cim je top-k pun, prag se smanji sa R^2 na d2 k-tog susjeda, pa se
        //     cijele celije cija je donja granica udaljenosti > prag preskacu,
        //     a vecina preostalih kandidata se odbaci jednim (predvidivim) poredjenjem;
        //  3) pozicije se citaju iz sx_/sy_ (kopija sortirana po celijama) --
        //     uzastopna memorija, bez indirekcije x[sorted_ids[k]].
        std::size_t k_nearest(std::size_t self, float radius, std::size_t k,
                              std::int32_t* ids, float* d2s) const
        {
            if (k == 0) return 0;
            const float xi = sx_[pos_of_[self]], yi = sy_[pos_of_[self]];
            const std::int32_t self_pos = pos_of_[self];
            const int cx = cell_x(xi), cy = cell_y(yi);

            // Kvadrat donje granice udaljenosti po osi, za svaku kolonu/red prozora.
            // Rubne celije su "beskonacne" prema van (tamo zavrse clampani agenti).
            // Mala rezerva (slack) stiti od zaokruzivanja kod granice celije.
            constexpr float INF = 3.0e38f;
            float bx[2 * MAX_REACH + 1], by[2 * MAX_REACH + 1];
            auto axis_bound = [&](float p, float lo_world, int c, int cmax) {
                const float lo = c == 0 ? -INF : lo_world + static_cast<float>(c) * R_;
                const float hi = c == cmax ? INF : lo_world + static_cast<float>(c + 1) * R_;
                float d = std::max({ lo - p, p - hi, 0.0f });
                d = std::max(d - slack_, 0.0f);
                return d * d;
            };
            for (int o = -reach_; o <= reach_; ++o)
            {
                const int gx = cx + o, gy = cy + o;
                bx[o + reach_] = (gx < 0 || gx >= W_) ? INF : axis_bound(xi, min_x_, gx, W_ - 1);
                by[o + reach_] = (gy < 0 || gy >= H_) ? INF : axis_bound(yi, min_y_, gy, H_ - 1);
            }

            float thr = radius * radius;   // prag: R^2 dok top-k nije pun, zatim d2 k-tog
            std::size_t count = 0;

            for (int r = 0; r <= reach_; ++r)
            {
              // Svaka celija prstena r ima |dx| == r ili |dy| == r, a granice rastu
              // s udaljenoscu, pa je ovo donja granica za prsten r i sve dalje.
              if (r > 0 && std::min({ bx[reach_ - r], bx[reach_ + r],
                                      by[reach_ - r], by[reach_ + r] }) > thr)
                  break;
              for (int oi = ring_begin_[r]; oi < ring_begin_[r + 1]; ++oi)
              {
                const Offset off = order_[oi];
                const float b = bx[off.dx + reach_] + by[off.dy + reach_];
                if (b > thr) continue;
                const int c = (cy + off.dy) * W_ + (cx + off.dx);
                const std::int32_t e = cell_start_[c + 1];
                for (std::int32_t p = cell_start_[c]; p < e; ++p)
                {
                    const float dx = sx_[p] - xi, dy = sy_[p] - yi;
                    const float d2 = dx * dx + dy * dy;
#ifdef CROWD_DIAG
                    ++g_cand;
#endif
                    // Pun top-k: prihvati samo ako je (d2,p) < (d2 k-tog, p k-tog).
                    // Nepun: prihvati sve u radijusu. Oba slucaja: d2 <= thr.
                    if (d2 > thr) continue;
                    if (count == k && !(d2 < d2s[k - 1] || p < ids[k - 1])) continue;
                    if (p == self_pos) continue;
#ifdef CROWD_DIAG
                    ++g_inserted;
#endif
                    std::size_t q = count < k ? count++ : k - 1;
                    while (q > 0 && (d2 < d2s[q - 1] || (d2 == d2s[q - 1] && p < ids[q - 1])))
                    {
                        d2s[q] = d2s[q - 1];
                        ids[q] = ids[q - 1];
                        --q;
                    }
                    d2s[q] = d2;
                    ids[q] = p;
                    if (count == k) thr = d2s[k - 1];
                }
              }
            }
            for (std::size_t t = 0; t < count; ++t) ids[t] = sorted_ids_[ids[t]];
            return count;
        }

        const std::vector<std::int32_t>& cell_start() const noexcept { return cell_start_; }
        const std::vector<std::int32_t>& sorted_ids() const noexcept { return sorted_ids_; }

        static constexpr int MAX_REACH = 8;

    private:
        int clamp_x(int c) const noexcept { return c < 0 ? 0 : (c >= W_ ? W_ - 1 : c); }
        int clamp_y(int c) const noexcept { return c < 0 ? 0 : (c >= H_ ? H_ - 1 : c); }

        struct Offset { std::int8_t dx, dy; };

        float min_x_, min_y_, R_, inv_R_;
        float slack_;
        int W_, H_;
        int reach_;
        std::vector<Offset> order_;             // pomaci celija, od najblizeg ka najdaljem
        std::vector<int> ring_begin_;           // order_[ring_begin_[r] .. ring_begin_[r+1]) = prsten r
        std::vector<std::int32_t> cell_of_;     // [n]     celija svakog agenta
        std::vector<std::int32_t> cell_start_;  // [W*H+1] prefix sum
        std::vector<std::int32_t> cursor_;      // [W*H]   radni bafer za scatter
        std::vector<std::int32_t> sorted_ids_;  // [n]     pozicija -> ID agenta
        std::vector<std::int32_t> pos_of_;      // [n]     ID agenta -> pozicija
        std::vector<float> sx_, sy_;            // [n]     pozicije u redoslijedu sorted_ids_
    };
}
