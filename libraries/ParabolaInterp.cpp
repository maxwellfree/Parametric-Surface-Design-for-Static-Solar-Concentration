#include "ParabolaInterp.h"

const float ParabolaInterp::kDegToRad = 0.017453292519943295f;

ParabolaInterp::ParabolaInterp() {}

void ParabolaInterp::setIMUDeg(float pitch_deg, float roll_deg) {
  pitch_rad_ = pitch_deg * kDegToRad;
  roll_rad_  = roll_deg  * kDegToRad;
}
void ParabolaInterp::setTargetsDeg(float theta_az_deg, float phi_pol_deg) {
  theta_rad_ = theta_az_deg * kDegToRad;
  phi_rad_   = phi_pol_deg  * kDegToRad;
}

void ParabolaInterp::rotZ(float ang, float x, float y, float& xr, float& yr) {
  float c = cosf(ang), s = sinf(ang);
  xr =  c*x + s*y;
  yr = -s*x + c*y;
}

// Rotación simple para alinear suelo con z=0 aproximando pitch/roll pequeñas.
// (suficiente para AVR y datos ya nivelados; si quieres, puedes reemplazar por una rotación 3D completa).
void ParabolaInterp::applyIMURotation(float x, float y, float z, float& xo, float& yo, float& zo) const {
  // rotación en X por roll, luego en Y por pitch
  float cr = cosf(roll_rad_), sr = sinf(roll_rad_);
  float cp = cosf(pitch_rad_), sp = sinf(pitch_rad_);

  // R_x(roll)
  float y1 =  cr*y - sr*z;
  float z1 =  sr*y + cr*z;
  float x1 =  x;

  // R_y(pitch)
  float x2 =  cp*x1 + sp*z1;
  float z2 = -sp*x1 + cp*z1;
  float y2 =  y1;

  xo = x2; yo = y2; zo = z2;
}

bool ParabolaInterp::buildStripAndFit(uint16_t max_points_to_read) {
  M_ = 0;
  if (!readStripFromSD(max_points_to_read)) return false;

  if (method_ == IDW8) {
    float dummy = 0.f;
    compute8NearestByAngular(u_, w_, M_, dummy); // usa u_=x', w_=z' como proxy
    return (M_ >= 8);
  } else {
    // ordenar (u,w) por u ascendente (insertion sort rápido por tamaño)
    for (uint16_t i=1;i<M_;++i){
      float uu=u_[i], ww=w_[i];
      uint16_t j=i;
      while(j>0 && u_[j-1]>uu){ u_[j]=u_[j-1]; w_[j]=w_[j-1]; --j; }
      u_[j]=uu; w_[j]=ww;
    }
    // opcional: submuestreo para spline si M_ muy grande
    KS_ = min<uint16_t>(M_, kMaxKnots);
    if (M_ > kMaxKnots) {
      float step = float(M_-1)/float(kMaxKnots-1);
      for (uint16_t k=0;k<kMaxKnots;++k){
        uint16_t idx = (uint16_t)roundf(k*step);
        uS_[k] = u_[idx];
        wS_[k] = w_[idx];
      }
    } else {
      for (uint16_t k=0;k<M_;++k){ uS_[k]=u_[k]; wS_[k]=w_[k]; }
    }
    return buildNaturalCubicSpline();
  }
}

bool ParabolaInterp::predictRange(float& s_pred, float* u_star_out) {
  if (method_ == IDW8) {
    // predicción muy simple: media ponderada radial de los 8 vecinos hacia el rayo (compatibilidad)
    // Para mantener compatibilidad, devolvemos la proyección sobre el rayo ± aproximación.
    // Aquí dejamos un retorno neutro (no usado por tu sketch si sigues mostrando con IDW8 antiguo).
    s_pred = 0.f;
    if (u_star_out) *u_star_out = 0.f;
    return true;
  }
  return rayIntersectSpline(s_pred, u_star_out);
}

bool ParabolaInterp::readStripFromSD(uint16_t max_points_to_read) {
  if (!SD.begin()) return false;     // usa CS por defecto ya inicializado fuera si lo prefieres
  File f = SD.open(fileName_);
  if (!f) return false;

  // Limpiamos vecinos
  for (int i=0;i<8;++i){ nearest8_[i] = {0,0,0, 1e9f}; }

  uint16_t count=0;
  while (f.available() && count < max_points_to_read) {
    String linea = f.readStringUntil('\n');
    linea.trim();
    if (linea.length()<3) continue;

    int c1 = linea.indexOf(',');
    int c2 = linea.lastIndexOf(',');
    if (c1<=0 || c2<=c1) continue;

    int xi = linea.substring(0, c1).toInt();
    int yi = linea.substring(c1+1, c2).toInt();
    int zi = linea.substring(c2+1).toInt();

    // Zref
    float x = (float)xi;
    float y = (float)yi;
    float z = (float)zi - Zref_;

    // Corrección IMU
    float xI,yI,zI;
    applyIMURotation(x,y,z, xI,yI,zI);

    // Girar por -theta (perfil visto de frente)
    float xR,yR;
    rotZ(-theta_rad_, xI, yI, xR, yR);

    // Filtro de strip: |y'| <= eps
    if (fabsf(yR) <= epsStrip_) {
      // Perfil: u = x', w = z'
      u_[M_] = xR;
      w_[M_] = zI;
      ++M_;
      if (M_ >= kMaxPointsCloud) break;
    }
    ++count;
  }
  f.close();
  return (M_ >= 4); // mínimo para spline
}

void ParabolaInterp::compute8NearestByAngular(const float* xu, const float* zw, uint16_t n, float& dummy) {
  // Reutiliza tu idea: aproximar "distancia angular" en el plano del perfil con métrica euclídea en (u,w),
  // y guardar los 8 más cercanos a (0,0) o al rayo según convenga; aquí tomamos el rayo d'=(cos phi, sin phi).
  float dx = cosf(phi_rad_);
  float dz = sinf(phi_rad_);
  for (uint16_t i=0;i<n;++i){
    // Distancia perpendicular aproximada al rayo (u,w)
    float proj  = xu[i]*dx + zw[i]*dz;
    float perp2 = xu[i]*xu[i] + zw[i]*zw[i] - proj*proj;
    float da = sqrtf(fabsf(perp2));
    // Insertar ordenado
    for (int k=0;k<8;++k){
      if (da < nearest8_[k].distancia){
        for (int j=7;j>k;--j) nearest8_[j] = nearest8_[j-1];
        nearest8_[k] = { (int)roundf(xu[i]), 0, (int)roundf(zw[i]), da };
        break;
      }
    }
  }
}

// -------------------- SPLINE ------------------------

bool ParabolaInterp::buildNaturalCubicSpline() {
  if (KS_ < 4) return false;

  // Construye segunda derivada m2_ (met. de tridiagonal para spline natural)
  // Referencia clásica: de Boor. Condiciones: S''(u0)=S''(uN)=0
  uint16_t n = KS_-1;
  static float a[kMaxKnots], b[kMaxKnots], c[kMaxKnots], d[kMaxKnots];
  // Limpia
  for(uint16_t i=0;i<KS_;++i){ m2_[i]=0.f; a[i]=b[i]=c[i]=d[i]=0.f; }

  // h_i
  // sistema para i = 1..n-1
  for (uint16_t i=1;i<n;++i){
    float h_i   = uS_[i]   - uS_[i-1];
    float h_ip1 = uS_[i+1] - uS_[i];
    float alpha = 3.0f*((wS_[i+1]-wS_[i])/h_ip1 - (wS_[i]-wS_[i-1])/h_i);
    a[i] = h_i;
    b[i] = 2.0f*(h_i + h_ip1);
    c[i] = h_ip1;
    d[i] = alpha;
  }
  // Thomas (con condiciones naturales: m2_[0]=m2_[n]=0 ya implícitas)
  // forward
  for (uint16_t i=2;i<n;++i){
    float m = a[i]/b[i-1];
    b[i] -= m*c[i-1];
    d[i] -= m*d[i-1];
  }
  // back
  m2_[n-1] = d[n-1]/b[n-1];
  for (int i=(int)n-2; i>=1; --i){
    m2_[i] = (d[i] - c[i]*m2_[i+1]) / b[i];
  }
  m2_[0]=0.f; m2_[n]=0.f;
  return true;
}

int ParabolaInterp::locateInterval(float uu) const {
  // búsqueda binaria en uS_ para encontrar i tal que uu in [uS_[i], uS_[i+1]]
  int lo = 0, hi = (int)KS_-1;
  if (uu <= uS_[0]) return 0;
  if (uu >= uS_[KS_-1]) return KS_-2;
  while (hi - lo > 1){
    int mid = (hi+lo)/2;
    if (uS_[mid] > uu) hi = mid; else lo = mid;
  }
  return lo;
}

bool ParabolaInterp::evalSpline(float uu, float& Sw, float* S1, float* S2) const {
  if (KS_ < 2) return false;
  int i = locateInterval(uu);
  float h = uS_[i+1]-uS_[i];
  if (h == 0.f) return false;
  float t = (uu - uS_[i])/h;      // [0,1]
  float t2=t*t, t3=t2*t;

  // forma de Hermite con m2_ (segunda derivada) -> equivalente a polinomio cúbico por tramo
  float w0 = wS_[i];
  float w1 = wS_[i+1];
  float m0 = m2_[i];
  float m1 = m2_[i+1];

  // S(u) = A*w0 + B*w1 + C*m0 + D*m1
  float A = 1 - 3*t2 + 2*t3;
  float B = 3*t2 - 2*t3;
  float C = (t3 - 2*t2 + t)*(h*h)/6.0f;
  float D = (t3 - t2)*(h*h)/6.0f;
  Sw = A*w0 + B*w1 + C*m0 + D*m1;

  if (S1) {
    // S'(u) derivando los términos (ver derivadas de A,B,C,D)
    float dA = (-6*t + 6*t2)/h;
    float dB = ( 6*t - 6*t2)/h;
    float dC = ((3*t2 - 4*t + 1)*h)/6.0f;
    float dD = ((3*t2 - 2*t   )*h)/6.0f;
    *S1 = dA*w0 + dB*w1 + dC*m0 + dD*m1;
  }
  if (S2) {
    // S''(u) por tramo ~ lineal entre m0 y m1, pero computamos exacto desde Hermite (simplificado):
    float d2A = (-6 + 12*t)/(h*h);
    float d2B = ( 6 - 12*t)/(h*h);
    float d2C = ( (6*t - 4) )/6.0f;
    float d2D = ( (6*t - 2) )/6.0f;
    *S2 = d2A*w0 + d2B*w1 + d2C*m0 + d2D*m1;
  }
  return true;
}

bool ParabolaInterp::rayIntersectSpline(float& s_pred, float* u_star_out) {
  if (KS_ < 2) return false;

  // Dirección del rayo en el plano perfil (u,w)
  float dx = cosf(phi_rad_);
  float dz = sinf(phi_rad_);

  // Inicialización: toma el u del punto más cercano al rayo (proyección aproximada)
  int iBest = 0;
  float best = 1e9f;
  for (uint16_t i=0;i<KS_;++i){
    float proj = uS_[i]*dx + wS_[i]*dz;
    float perp2 = uS_[i]*uS_[i] + wS_[i]*wS_[i] - proj*proj;
    if (perp2 < best){ best = perp2; iBest = i; }
  }
  float u = uS_[iBest];

  // Función F(u) = derivada del coste perp^2(u)
  auto computeF = [&](float uu, float& F, float& dF)->bool{
    float Su,S1,S2;
    if (!evalSpline(uu, Su, &S1, &S2)) return false;
    // gamma(u)=(u, S(u)), gamma'(u)=(1, S'(u))
    float dot = (uu*dx + Su*dz);
    float gx  = uu - dot*dx;
    float gz  = Su - dot*dz;
    // F(u) = 2*gamma'(u) · g
    F  = 2.0f*( 1.0f*gx + S1*gz );
    // dF = 2*(gamma''·g + gamma'·g')
    // gamma'' = (0, S''), g' = gamma' - (gamma'·d) d - (gamma·d') ; con d' constante => g' = gamma' - (gamma'·d) d
    float gp = (1.0f*dx + S1*dz);   // gamma'·d
    float gpx = 1.0f - gp*dx;
    float gpz = S1   - gp*dz;
    dF = 2.0f*( S2*gz + (1.0f*gpx + S1*gpz) );
    return true;
  };

  // Newton salvaguardado
  const int   kMaxIt = 12;
  const float kTol   = 1e-3f * ( (KS_>=2) ? fabsf(uS_[KS_-1]-uS_[0]) : 1.0f );
  for (int it=0; it<kMaxIt; ++it){
    float F, dF;
    if (!computeF(u, F, dF)) break;
    float step = (dF!=0.f) ? -F/dF : 0.f;
    // backtracking simple
    float u_new = clampf(u + step, uS_[0], uS_[KS_-1]);
    if (fabsf(u_new - u) < kTol) { u = u_new; break; }
    u = u_new;
  }

  float Su;
  if (!evalSpline(u, Su, nullptr, nullptr)) return false;
  s_pred = u*dx + Su*dz;          // proyección sobre el rayo
  if (u_star_out) *u_star_out = u;
  return true;
}

