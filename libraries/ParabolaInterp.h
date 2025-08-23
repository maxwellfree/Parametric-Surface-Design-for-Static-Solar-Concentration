#pragma once
#include <Arduino.h>
#include <SD.h>
#include <math.h>

// Librería de interpolación para perfil parabólico visto de frente.
// Dos métodos: IDW8 (compatibilidad) y SPLINE (nuevo).

class ParabolaInterp {
public:
  enum Method : uint8_t { IDW8 = 0, SPLINE = 1 };

  // Parámetros de compilación/memoria (ajusta si hace falta):
  static const uint16_t kMaxPointsCloud = 200; // máximo de puntos a leer del strip
  static const uint16_t kMaxKnots       = 120; // máximo de nodos de spline (<= kMaxPointsCloud)
  static const float    kDegToRad;

  ParabolaInterp();

  void setMethod(Method m) { method_ = m; }
  void setPointCloudFile(const char* fname) { strncpy(fileName_, fname, sizeof(fileName_)-1); }
  void setStripHalfWidth(float eps) { epsStrip_ = eps; }            // |y'| <= eps en el plano perfil
  void setZref(float zref) { Zref_ = zref; }                        // corrección de referencia
  void setIMUDeg(float pitch_deg, float roll_deg);                  // inclinaciones (grados)
  void setTargetsDeg(float theta_az_deg, float phi_pol_deg);        // servo azimut/polar (grados)

  // Carga strip, corrige IMU, rota por -theta, proyecta (u,w), y prepara el modelo (IDW/Spline).
  // Devuelve false si hay error de SD o no hay suficientes puntos.
  bool buildStripAndFit(uint16_t max_points_to_read = kMaxPointsCloud);

  // Predice el alcance s_pred del rayo en el plano del perfil (y el u* si se pide).
  bool predictRange(float& s_pred, float* u_star_out = nullptr);

  // Para IDW8 (compatibilidad): expone los 8 vecinos si quieres revisarlos
  struct Punto { int x, y, z; float distancia; };
  const Punto* get8Nearest() const { return nearest8_; }

private:
  // --- estado ---
  Method  method_ = SPLINE;
  char    fileName_[16] = "coor.dat";
  float   epsStrip_ = 5.0f;      // en mismas unidades que y' (mm o cm según tus datos)
  float   Zref_     = 0.0f;

  // IMU y objetivos
  float pitch_rad_ = 0.0f, roll_rad_ = 0.0f;
  float theta_rad_ = 0.0f, phi_rad_ = 0.0f;

  // Datos del strip: (u,w)
  uint16_t M_ = 0;
  float u_[kMaxPointsCloud];
  float w_[kMaxPointsCloud];

  // Para IDW8
  Punto nearest8_[8];

  // Para SPLINE: nodos (uS,wS) (ordenados) y coeficientes tridiagonales
  uint16_t KS_ = 0;                // nº de nodos en la spline
  float uS_[kMaxKnots];
  float wS_[kMaxKnots];
  // coeficientes de segunda derivada (spline natural): m[i] ≈ S''(uS[i])
  float m2_[kMaxKnots];

  // --- helpers ---
  static void rotZ(float ang, float x, float y, float& xr, float& yr);
  void applyIMURotation(float x, float y, float z, float& xo, float& yo, float& zo) const;

  bool readStripFromSD(uint16_t max_points_to_read);
  void compute8NearestByAngular(const float* xu, const float* yw, uint16_t n, float& dummy); // rellena nearest8_

  // SPLINE: construcción y evaluación
  bool buildNaturalCubicSpline();               // rellena m2_
  bool evalSpline(float uu, float& Sw, float* S1 = nullptr, float* S2 = nullptr) const;
  int  locateInterval(float uu) const;          // búsqueda binaria

  // Proyección rayo–curva por Newton salvaguardado
  bool rayIntersectSpline(float& s_pred, float* u_star_out);

  // Utilidades
  static float clampf(float v, float a, float b) { return (v<a)?a:((v>b)?b:v); }
};

