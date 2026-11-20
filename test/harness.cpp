// PC-Test: spielt FRAM-Zeilen aus dem HA-CSV durch den PresenceDetector und
// schreibt pro abgeschlossener Sekunde: sekunde,p,person,vehicle
// Aufruf: harness <FRAM.csv> <out.csv>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include "../components/kld7/presence_detector.h"

using esphome::kld7::PresenceDetector;

static bool parse_time(const char* s, double& out) {
  int Y, M, D, h, m; double sec;
  if (sscanf(s, " %d-%d-%d %d:%d:%lf", &Y, &M, &D, &h, &m, &sec) != 6) return false;
  struct tm t = {};
  t.tm_year = Y - 1900; t.tm_mon = M - 1; t.tm_mday = D; t.tm_hour = h; t.tm_min = m; t.tm_sec = 0;
  out = (double) timegm(&t) + sec;
  return true;
}

int main(int argc, char** argv) {
  if (argc < 3) { fprintf(stderr, "usage: harness FRAM.csv out.csv\n"); return 1; }
  std::ifstream in(argv[1]);
  FILE* out = fopen(argv[2], "w");
  fprintf(out, "t,p,person,vehicle\n");
  PresenceDetector det;
  std::string line;
  bool started = false;
  uint32_t last_sec = 0;
  long frames = 0;
  while (std::getline(in, line)) {
    if (line.compare(0, 5, "FRAM;") != 0) continue;
    // Felder durch ';' trennen
    std::vector<std::string> f;
    size_t pos = 0;
    while (true) {
      size_t q = line.find(';', pos);
      if (q == std::string::npos) { f.push_back(line.substr(pos)); break; }
      f.push_back(line.substr(pos, q - pos));
      pos = q + 1;
    }
    if (f.size() < 12) continue;
    double t;
    if (!parse_time(f[1].c_str(), t)) continue;
    uint32_t sec = (uint32_t) std::floor(t);
    if (!started) { started = true; last_sec = sec; det.tick(sec); }
    // Jede Sekunde einzeln abschliessen, damit wir pro Sekunde ausgeben koennen.
#ifdef JUMP
    if (last_sec < sec) {
      det.tick(sec); last_sec = sec;
      fprintf(out, "%u,%.17g,%d,%d\n", sec - 1, det.probability(), det.person() ? 1 : 0, det.vehicle() ? 1 : 0);
    }
#else
    while (last_sec < sec) {
      last_sec++;
      det.tick(last_sec);
      fprintf(out, "%u,%.17g,%d,%d\n", last_sec - 1, det.probability(), det.person() ? 1 : 0, det.vehicle() ? 1 : 0);
    }
#endif
    det.begin_frame(sec);
    // Targets: [[d,s,a,m],...]
    const char* p = f[11].c_str();
    while ((p = strchr(p, '[')) != nullptr) {
      if (p[1] == '[') { p++; continue; }
      int d; double s, a, m;
      if (sscanf(p, "[%d,%lf,%lf,%lf]", &d, &s, &a, &m) == 4) {
        det.add_point((uint16_t) d, (float) s, (float) a, (float) m);
      }
      p++;
    }
    det.end_frame(true);
    frames++;
  }
  last_sec++;
  det.tick(last_sec);
  fprintf(out, "%u,%.17g,%d,%d\n", last_sec - 1, det.probability(), det.person() ? 1 : 0, det.vehicle() ? 1 : 0);
  fclose(out);
  fprintf(stderr, "%ld Frames\n", frames);
  return 0;
}
