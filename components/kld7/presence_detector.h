#pragma once
// Echtzeit-Erkennung "KLD7 Person" und "KLD7 Fahrzeug" aus PDAT-Rohzielen.
//
// Herkunft: Analyse der FRAM-Logs vom 10.-19.11.2026 gegen Hue Sued
// (siehe Projekt-Doc "KLD7-Projektstand.md", Abschnitt "Echtzeit-Detektor v0").
// Die Merkmalsberechnung entspricht exakt dem Python-Original (sec.parquet /
// sec2.parquet). Jede Aenderung hier muss dort nachgezogen werden, sonst passt
// das Modell (presence_model.h) nicht mehr.
//
// Reine C++-Klasse ohne ESPHome-Abhaengigkeit, damit sie auf dem PC gegen die
// Logs getestet werden kann. Alles liegt im RAM, nichts wird im Flash gespeichert.
//
// Zeitraster: ganze Sekunden. Punkte eines Frames zaehlen zur Sekunde, in der
// der Frame ankommt. Nach Ablauf einer Sekunde wird mit den letzten 3 (bzw. 6
// und 10) Sekunden entschieden.

#include <cmath>
#include <cstdint>
#include "presence_model.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace esphome {
namespace kld7 {

class PresenceDetector {
 public:
  // --- Parameter (aus der Analyse, nicht ohne neue Auswertung aendern) ---
  // Personenzone (Meter, x quer, y vom Sensor weg) und Geschwindigkeit (km/h).
  static constexpr double ZONE_Y_MIN = 2.5, ZONE_Y_MAX = 10.5;
  static constexpr double ZONE_X_MIN = -7.0, ZONE_X_MAX = 3.0;
  static constexpr double ZONE_SPEED_MIN = 0.8, ZONE_SPEED_MAX = 10.0;
  static constexpr double FAST_SPEED = 15.0;      // "schnell" = Fahrzeug
  static constexpr double FAST_NEAR_Y = 9.0;      // schnelle Punkte vor der Strasse
  static constexpr double STREET_Y = 10.5;        // langsam, aber jenseits der Zone
  static constexpr double MAG_HIGH = 35.0;        // Zonenpunkt mit hoher Magnitude
  static constexpr int FAST_BURST = 5;            // schnelle Punkte/s fuer since_fast
  static constexpr int SINCE_FAST_MAX = 600;
  // Hysterese Person
  static constexpr double PERSON_ON = 0.5;
  static constexpr double PERSON_OFF = 0.25;
  static constexpr int PERSON_OFF_SECONDS = 4;
  // Hysterese Fahrzeug: schnelle Punkte in der aktuellen und der vorigen Sekunde
  static constexpr int VEHICLE_ON_POINTS = 8;
  static constexpr int VEHICLE_OFF_SECONDS = 2;

  // Einen Frame melden. has_content = der Frame hatte PDAT-Ziele oder eine
  // TDAT-Detection (genau die Frames, die auch als HA-Ereignis geloggt werden).
  void begin_frame(uint32_t second) {
    advance_(second);
    frame_zone_ = false;
  }
  // d in cm, s in km/h, a in Grad (wie geloggt, also nach invert_angle), m in dB.
  void add_point(uint16_t d, float s_raw, float a_raw, float m_raw) {
    // Auf 2 Nachkommastellen runden: genau so stehen die Werte im Log.
    double s = std::round((double) s_raw * 100.0) / 100.0;
    double a = std::round((double) a_raw * 100.0) / 100.0;
    double m = std::round((double) m_raw * 100.0) / 100.0;
    double rad = a * M_PI / 180.0;
    double x = d * std::sin(rad) / 100.0;
    double y = d * std::cos(rad) / 100.0;
    double as = std::fabs(s);
    Bin& b = cur_;
    b.nall++;
    if (as >= FAST_SPEED) {
      b.nf++;
      if (y < FAST_NEAR_Y) b.nfz++;
    }
    if (as >= ZONE_SPEED_MIN && as < FAST_SPEED && y > STREET_Y) b.nstreet++;
    bool zone = y >= ZONE_Y_MIN && y <= ZONE_Y_MAX && x >= ZONE_X_MIN && x <= ZONE_X_MAX &&
                as >= ZONE_SPEED_MIN && as <= ZONE_SPEED_MAX;
    if (zone) {
      b.nz++;
      b.sx += x;
      b.sxx += x * x;
      b.sy += y;
      b.sm += m;
      if (m > b.mmax) b.mmax = m;
      if (x < b.xmin) b.xmin = x;
      if (x > b.xmax) b.xmax = x;
      if (m >= MAG_HIGH) b.nzh++;
      frame_zone_ = true;
    }
  }
  void end_frame(bool has_content) {
    if (!has_content) return;
    cur_.nfr++;
    if (frame_zone_) cur_.nzf++;
  }
  // Regelmaessig aufrufen (z.B. aus loop()). Gibt true zurueck, wenn sich ein
  // Zustand oder die Wahrscheinlichkeit geaendert hat.
  bool tick(uint32_t second) { return advance_(second); }

  bool person() const { return person_; }
  bool vehicle() const { return vehicle_; }
  double probability() const { return p_; }
  const double* features() const { return feat_; }
  uint32_t last_second() const { return have_second_ ? cur_second_ - 1 : 0; }

 private:
  struct Bin {
    int nall = 0, nf = 0, nfz = 0, nz = 0, nzf = 0, nfr = 0, nzh = 0, nstreet = 0;
    double sx = 0, sxx = 0, sy = 0, sm = 0, mmax = 0, xmin = 99, xmax = -99;
  };
  static constexpr int HIST = 10;
  Bin hist_[HIST];        // hist_[0] = zuletzt abgeschlossene Sekunde
  Bin cur_;
  bool frame_zone_ = false;
  bool have_second_ = false;
  uint32_t cur_second_ = 0;
  int since_fast_ = SINCE_FAST_MAX;
  int prev_nf_ = 0;       // schnelle Punkte der vorigen Sekunde (Fahrzeug)
  bool person_ = false, vehicle_ = false;
  int person_off_count_ = 0, vehicle_off_count_ = 0;
  double p_ = 0;
  double feat_[presence_model::NUM_FEATURES] = {0};

  bool advance_(uint32_t second) {
    if (!have_second_) {
      have_second_ = true;
      cur_second_ = second;
      return false;
    }
    if (second <= cur_second_) return false;  // gleiche Sekunde (oder Uhr zurueck)
    bool changed = false;
    // Nach langer Pause (z.B. Neustart der Zeitbasis) nicht tausende leere
    // Sekunden durchrechnen: 10 leere Sekunden setzen alle Fenster zurueck.
    uint32_t steps = second - cur_second_;
    uint32_t skipped = 0;
    if (steps > HIST + PERSON_OFF_SECONDS + 2) {
      skipped = steps - (HIST + PERSON_OFF_SECONDS + 2);
      steps = HIST + PERSON_OFF_SECONDS + 2;
    }
    for (uint32_t k = 0; k < steps; k++) {
      changed |= close_second_();
    }
    // Uebersprungene leere Sekunden aendern nur noch since_fast.
    if (skipped > 0) {
      uint32_t s = (uint32_t) since_fast_ + skipped;
      since_fast_ = s > (uint32_t) SINCE_FAST_MAX ? SINCE_FAST_MAX : (int) s;
    }
    cur_second_ = second;
    return changed;
  }

  bool close_second_() {
    // Abgeschlossene Sekunde nach vorne schieben.
    for (int i = HIST - 1; i > 0; i--) hist_[i] = hist_[i - 1];
    hist_[0] = cur_;
    cur_ = Bin();
    const Bin& b0 = hist_[0];

    // since_fast: Sekunden seit der letzten Sekunde mit >= FAST_BURST schnellen Punkten
    if (b0.nf >= FAST_BURST) since_fast_ = 0;
    else if (since_fast_ < SINCE_FAST_MAX) since_fast_++;

    // 3-Sekunden-Summen
    int nz = 0, nzf = 0, nfr = 0, nall = 0, nf = 0, nfz = 0, nzh = 0, nstreet = 0;
    double sx = 0, sxx = 0, sy = 0, sm = 0, mmax = 0, xmin = 99, xmax = -99;
    for (int i = 0; i < 3; i++) {
      const Bin& b = hist_[i];
      nz += b.nz; nzf += b.nzf; nfr += b.nfr; nall += b.nall; nf += b.nf; nfz += b.nfz;
      nzh += b.nzh; nstreet += b.nstreet;
      sx += b.sx; sxx += b.sxx; sy += b.sy; sm += b.sm;
      if (b.mmax > mmax) mmax = b.mmax;
      if (b.xmin < xmin) xmin = b.xmin;
      if (b.xmax > xmax) xmax = b.xmax;
    }
    int nf10 = 0, nz6 = 0, nz_act6 = 0;
    for (int i = 0; i < 10; i++) nf10 += hist_[i].nf;
    for (int i = 0; i < 6; i++) {
      nz6 += hist_[i].nz;
      if (hist_[i].nz >= 3) nz_act6++;
    }
    double xm = 0, xsd = 0, ym = 0, mm = 0;
    if (nz > 0) {
      xm = sx / nz;
      double var = sxx / nz - xm * xm;
      xsd = std::sqrt(var > 0 ? var : 0);
      ym = sy / nz;
      mm = sm / nz;
    }
    double xr = xmax - xmin;
    if (xr < 0) xr = 0;
    double* f = feat_;
    f[0] = nz; f[1] = nzf; f[2] = nfr; f[3] = nall; f[4] = nf; f[5] = nfz; f[6] = mmax;
    f[7] = xr; f[8] = xm; f[9] = xsd; f[10] = ym; f[11] = mm;
    f[12] = nall > 0 ? (double) nz / nall : 0;
    f[13] = nfr > 0 ? (double) nzf / nfr : 0;
    f[14] = nf10; f[15] = nf10 - nf; f[16] = since_fast_; f[17] = nz6; f[18] = nz_act6;
    f[19] = nzh; f[20] = nstreet;

    double p = nall > 0 ? presence_model::predict(f) : 0.0;
    bool changed = (p != p_);
    p_ = p;

    // Hysterese Person
    if (!person_) {
      if (p >= PERSON_ON) { person_ = true; person_off_count_ = 0; changed = true; }
    } else {
      person_off_count_ = (p < PERSON_OFF) ? person_off_count_ + 1 : 0;
      if (person_off_count_ >= PERSON_OFF_SECONDS) { person_ = false; changed = true; }
    }
    // Hysterese Fahrzeug
    int f2 = b0.nf + prev_nf_;
    prev_nf_ = b0.nf;
    if (!vehicle_) {
      if (f2 >= VEHICLE_ON_POINTS) { vehicle_ = true; vehicle_off_count_ = 0; changed = true; }
    } else {
      vehicle_off_count_ = (f2 < 1) ? vehicle_off_count_ + 1 : 0;
      if (vehicle_off_count_ >= VEHICLE_OFF_SECONDS) { vehicle_ = false; changed = true; }
    }
    return changed;
  }
};

}  // namespace kld7
}  // namespace esphome
