// Likelihood-field particle filter cost on the K26 A53 (measure, don't guess).
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#ifdef _OPENMP
#include <omp.h>
#endif
struct P { float x, y, th, w; };
int main() {
  const int W = 300, H = 200; const float res = 0.05f;          // ~15 m x 10 m track map
  std::vector<float> loglik(W * H);                              // precomputed log p(z | d) per cell
  std::mt19937 rng(1); std::uniform_real_distribution<float> u01(0.f, 1.f);
  for (auto &v : loglik) v = -u01(rng) * 4.f;
  const int beam_sets[] = {108, 270, 1081};
  const int particle_sets[] = {1000, 2000, 4000, 8000};
  for (int B : beam_sets) {
    std::vector<float> bx(B), by(B);
    for (int i = 0; i < B; ++i) { float a = -2.356f + 4.712f * i / (B - 1); float r = 0.5f + 4.f * u01(rng); bx[i] = r * std::cos(a); by[i] = r * std::sin(a); }
    for (int N : particle_sets) {
      std::vector<P> ps(N), tmp(N);
      for (auto &p : ps) p = {2.f + 10.f * u01(rng), 2.f + 6.f * u01(rng), 6.283f * u01(rng), 1.f / N};
      std::normal_distribution<float> nz(0.f, 0.02f);
      const int iters = 60; double t_total = 0;
      for (int it = 0; it < iters; ++it) {
        auto t0 = std::chrono::steady_clock::now();
        for (auto &p : ps) { p.x += 0.1f + nz(rng); p.y += nz(rng); p.th += 0.5f * nz(rng); }       // motion model
        float maxl = -1e30f;
        #pragma omp parallel for reduction(max:maxl) schedule(static)
        for (int k = 0; k < N; ++k) {                                                              // sensor model
          P &p = ps[k]; const float c = std::cos(p.th), s = std::sin(p.th); float acc = 0.f;
          for (int i = 0; i < B; ++i) {
            int gx = (int)((p.x + c * bx[i] - s * by[i]) * (1.f / res));
            int gy = (int)((p.y + s * bx[i] + c * by[i]) * (1.f / res));
            gx = gx < 0 ? 0 : (gx >= W ? W - 1 : gx); gy = gy < 0 ? 0 : (gy >= H ? H - 1 : gy);
            acc += loglik[gy * W + gx];
          }
          p.w = acc; maxl = std::max(maxl, acc);
        }
        double sum = 0; for (auto &p : ps) { p.w = std::exp((p.w - maxl) * 0.05f); sum += p.w; }
        const double step = sum / N; double cum = ps[0].w, r = u01(rng) * step; int j = 0;         // low-variance resampling
        for (int k = 0; k < N; ++k) { double tgt = r + k * step; while (tgt > cum && j < N - 1) cum += ps[++j].w; tmp[k] = ps[j]; }
        ps.swap(tmp);
        t_total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      }
      const double ms = t_total / iters;
      std::printf("beams=%4d particles=%5d : %7.2f ms/update  -> %5.1f%% of %s at 40 Hz\n", B, N, ms, ms * 40.0 / 10.0,
#ifdef _OPENMP
                  "the used cores"
#else
                  "one core"
#endif
      );
    }
  }
}
