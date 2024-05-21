/*------------------------------------------------------------------------------
 * ephemeris.c : satellite ephemeris and clock functions
 *
 *          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
 *
 * References :
 *     [1] IS-GPS-200K, Navstar GPS Space Segment/Navigation User Interfaces,
 *         May 6, 2019
 *     [2] Global Navigation Satellite System GLONASS, Interface Control Document
 *         Navigational radiosignal In bands L1, L2, (Version 5.1), 2008
 *     [3] RTCA/DO-229C, Minimum operational performance standards for global
 *         positioning system/wide area augmentation system airborne equipment,
 *         RTCA inc, November 28, 2001
 *     [4] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
 *         Code Biases, URA
 *     [5] RTCM Paper 012-2009-SC104-528, January 28, 2009 (previous ver of [4])
 *     [6] RTCM Paper 012-2009-SC104-582, February 2, 2010 (previous ver of [4])
 *     [7] European GNSS (Galileo) Open Service Signal In Space Interface Control
 *         Document, Issue 1.3, December, 2016
 *     [8] Quasi-Zenith Satellite System Interface Specification Satellite
 *         Positioning, Navigation and Timing Service (IS-QZSS-PNT-003), Cabinet
 *         Office, November 5, 2018
 *     [9] BeiDou navigation satellite system signal in space interface control
 *         document open service signal B1I (version 3.0), China Satellite
 *         Navigation office, February, 2019
 *     [10] RTCM Standard 10403.3, Differential GNSS (Global Navigation
 *         Satellite Systems) Services - version 3, October 7, 2016
 *
 * Version : $Revision:$ $Date:$
 * History : 2010/07/28 1.1  moved from rtkcmn.c
 *                           added api:
 *                               eph2clk(),geph2clk(),seph2clk(),satantoff()
 *                               satposs()
 *                           changed api:
 *                               eph2pos(),geph2pos(),satpos()
 *                           deleted api:
 *                               satposv(),satposiode()
 *           2010/08/26 1.2  add ephemeris option EPHOPT_LEX
 *           2010/09/09 1.3  fix problem when precise clock outage
 *           2011/01/12 1.4  add api alm2pos()
 *                           change api satpos(),satposs()
 *                           enable valid unhealthy satellites and output status
 *                           fix bug on exception by GLONASS ephem computation
 *           2013/01/10 1.5  support BeiDou (compass)
 *                           use newton's method to solve kepler eq.
 *                           update SSR correction algorithm
 *           2013/03/20 1.6  fix problem on SSR clock relativistic correction
 *           2013/09/01 1.7  support negative pseudorange
 *                           fix bug on variance in case of URA SSR = 63
 *           2013/11/11 1.8  change constant MAXAGESSR 70.0 -> 90.0
 *           2014/10/24 1.9  fix bug on return of var_uraeph() if ura<0||15<ura
 *           2014/12/07 1.10 modify MAXDTOE for QZSS,GAL and bds
 *                           test max number of iteration for Kepler
 *           2015/08/26 1.11 update RTOL_ELPLER 1E-14 -> 1E-13
 *                           set MAX_ITER_KEPLER for alm2pos()
 *           2017/04/11 1.12 fix bug on max number of obs data in satposs()
 *           2018/10/10 1.13 update reference [7]
 *                           support URA value in var_uraeph() for Galileo
 *                           test eph->flag to recognize BeiDou geo
 *                           add api satseleph() for ephemeris selection
 *           2020/11/30 1.14 update references [1],[2],[8],[9] and [10]
 *                           add API getseleph()
 *                           rename API satseleph() as setseleph()
 *                           support NavIC/IRNSS by API satpos() and satposs()
 *                           support BDS C59-63 as GEO satellites in eph2pos()
 *                           default selection of I/NAV for Galileo ephemeris
 *                           no support EPHOPT_LEX by API satpos() and satposs()
 *                           unselect Galileo ephemeris with AOD<=0 in seleph()
 *                           fix bug on clock iteration in eph2clk(), geph2clk()
 *                           fix bug on clock reference time in satpos_ssr()
 *                           fix bug on wrong value with ura=15 in var_ura()
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Constants and macros ------------------------------------------------------*/

#define SQR(x) ((x) * (x))

#define RE_GLO 6378136.0L      /* Radius of earth (m)            ref [2] */
#define MU_GPS 3.9860050E14L   /* Gravitational constant         ref [1] */
#define MU_GLO 3.9860044E14L   /* Gravitational constant         ref [2] */
#define MU_GAL 3.986004418E14L /* Earth gravitational constant   ref [7] */
#define MU_CMP 3.986004418E14L /* Earth gravitational constant   ref [9] */
#define J2_GLO 1.0826257E-3L   /* 2nd zonal harmonic of geopot   ref [2] */

#define OMGE_GLO 7.292115E-5L     /* Earth angular velocity (rad/s) ref [2] */
#define OMGE_GAL 7.2921151467E-5L /* Earth angular velocity (rad/s) ref [7] */
#define OMGE_CMP 7.292115E-5L     /* Earth angular velocity (rad/s) ref [9] */

#define SIN_5 -0.08715574274765817356L /* sinl(-5.0 deg) */
#define COS_5 0.9961946980917455323L   /* cosl(-5.0 deg) */

#define ERREPH_GLO 5.0L    /* Error of GLONASS ephemeris (m) */
#define TSTEP 60.0L        /* Integration step GLONASS ephemeris (s) */
#define RTOL_KEPLER 1E-18L /* Relative tolerance for Kepler equation */

#define DEFURASSR 0.15L             /* Default accuracy of SSR corr (m) */
#define MAXECORSSR 10.0L            /* Max orbit correction of SSR (m) */
#define MAXCCORSSR (1E-6L * CLIGHT) /* Max clock correction of SSR (m) */
#define MAXAGESSR 90.0L             /* Max age of SSR orbit and clock (s) */
#define MAXAGESSR_HRCLK 10.0L       /* Max age of SSR high-rate clock (s) */
#define STD_BRDCCLK 30.0L           /* Error of broadcast clock (m) */
#define STD_GAL_NAPA 500.0L         /* Error of Galileo ephemeris for NAPA (m) */

#define MAX_ITER_KEPLER 30 /* Max number of iteration of Kepler */

/* Ephemeris selections ------------------------------------------------------*/
static int eph_sel[] = {/* GPS,GLO,GAL,QZS,BDS,IRN,SBS */
                        0, 0, 0, 0, 0, 0, 0};

/* Variance by URA ephemeris -------------------------------------------------*/
static long double var_uraeph(int sys, int ura) {
  const long double ura_value[] = {2.4L,  3.4L,   4.85L,  6.85L,  9.65L,   13.65L,  24.0L,  48.0L,
                                   96.0L, 192.0L, 384.0L, 768.0L, 1536.0L, 3072.0L, 6144.0L};
  if (sys == SYS_GAL) { /* Galileo sisa (ref [7] 5.1.11) */
    if (ura <= 49) return SQR(ura * 0.01L);
    if (ura <= 74) return SQR(0.5L + (ura - 50) * 0.02L);
    if (ura <= 99) return SQR(1.0L + (ura - 75) * 0.04L);
    if (ura <= 125) return SQR(2.0L + (ura - 100) * 0.16L);
    return SQR(STD_GAL_NAPA);
  } else { /* GPS URA (ref [1] 20.3.3.3.1.1) */
    return ura < 0 || 14 < ura ? SQR(6144.0L) : SQR(ura_value[ura]);
  }
}
/* Variance by URA SSR (ref [10] table 3.3-1 DF389) --------------------------*/
static long double var_urassr(int ura) {
  if (ura <= 0) return SQR(DEFURASSR);
  if (ura >= 63) return SQR(5.4665L);
  long double std = (powl(3.0L, (ura >> 3) & 7) * (1.0L + (ura & 7) / 4.0L) - 1.0L) * 1E-3L;
  return SQR(std);
}
/* Almanac to satellite position and clock bias --------------------------------
 * Compute satellite position and clock bias with almanac (GPS, Galileo, QZSS)
 * Args   : gtime_t time     I   time (GPST)
 *          alm_t *alm       I   almanac
 *          long double *rs       O   satellite position (ECEF) {x,y,z} (m)
 *          long double *dts      O   satellite clock bias (s)
 * Return : none
 * Notes  : see ref [1],[7],[8]
 *----------------------------------------------------------------------------*/
extern void alm2pos(gtime_t time, const alm_t *alm, long double *rs, long double *dts) {
  char tstr[40];
  trace(4, "alm2pos : time=%s sat=%2d\n", time2str(time, tstr, 3), alm->sat);

  long double tk = timediff(time, alm->toa);

  if (alm->A <= 0.0L) {
    rs[0] = rs[1] = rs[2] = *dts = 0.0L;
    return;
  }
  long double mu = satsys(alm->sat, NULL) == SYS_GAL ? MU_GAL : MU_GPS;

  long double M = alm->M0 + sqrtl(mu / (alm->A * alm->A * alm->A)) * tk;
  int n;
  long double Ek = 0.0L, E = M;
  for (n = 0; fabsl(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++) {
    Ek = E;
    E -= (E - alm->e * sinl(E) - M) / (1.0L - alm->e * cosl(E));
  }
  if (n >= MAX_ITER_KEPLER) {
    trace(2, "alm2pos: kepler iteration overflow sat=%2d\n", alm->sat);
  }
  long double sinE = sinl(E);
  long double cosE = cosl(E);
  long double u = atan2l(sqrtl(1.0L - alm->e * alm->e) * sinE, cosE - alm->e) + alm->omg;
  long double r = alm->A * (1.0L - alm->e * cosE);
  long double i = alm->i0;
  long double O = alm->OMG0 + (alm->OMGd - OMGE) * tk - OMGE * alm->toas;
  long double x = r * cosl(u);
  long double y = r * sinl(u);
  long double sinO = sinl(O);
  long double cosO = cosl(O);
  long double cosi = cosl(i);
  rs[0] = x * cosO - y * cosi * sinO;
  rs[1] = x * sinO + y * cosi * cosO;
  rs[2] = y * sinl(i);
  *dts = alm->f0 + alm->f1 * tk;
}
/* Broadcast ephemeris to satellite clock bias ---------------------------------
 * Compute satellite clock bias with broadcast ephemeris (GPS, Galileo, QZSS)
 * Args   : gtime_t time     I   time by satellite clock (GPST)
 *          eph_t *eph       I   broadcast ephemeris
 * Return : satellite clock bias (s) without relativity correction
 * Notes  : see ref [1],[7],[8]
 *          satellite clock does not include relativity correction and tdg
 *----------------------------------------------------------------------------*/
extern long double eph2clk(gtime_t time, const eph_t *eph) {
  char tstr[40];
  trace(4, "eph2clk : time=%s sat=%2d\n", time2str(time, tstr, 3), eph->sat);

  long double t = timediff(time, eph->toc), ts = t;

  for (int i = 0; i < 2; i++) {
    t = ts - (eph->f0 + eph->f1 * t + eph->f2 * t * t);
  }
  trace(4, "ephclk: t=%.12Lf ts=%.12Lf dts=%.12Lf f0=%.12Lf f1=%.9Lf f2=%.9Lf\n", t, ts,
        eph->f0 + eph->f1 * t + eph->f2 * t * t, eph->f0, eph->f1, eph->f2);

  return eph->f0 + eph->f1 * t + eph->f2 * t * t;
}
/* Broadcast ephemeris to satellite position and clock bias --------------------
 * Compute satellite position and clock bias with broadcast ephemeris (GPS,
 * Galileo, QZSS)
 * Args   : gtime_t time     I   time (GPST)
 *          eph_t *eph       I   broadcast ephemeris
 *          long double *rs       O   satellite position (ECEF) {x,y,z} (m)
 *          long double *dts      O   satellite clock bias (s)
 *          long double *var      O   satellite position and clock variance (m^2)
 * Return : none
 * Notes  : see ref [1],[7],[8]
 *          satellite clock includes relativity correction without code bias
 *          (tgd or bgd)
 *----------------------------------------------------------------------------*/
extern void eph2pos(gtime_t time, const eph_t *eph, long double *rs, long double *dts,
                    long double *var) {
  char tstr[40];
  trace(4, "eph2pos : time=%s sat=%2d\n", time2str(time, tstr, 3), eph->sat);

  if (eph->A <= 0.0L) {
    rs[0] = rs[1] = rs[2] = *dts = *var = 0.0L;
    return;
  }
  long double tk = timediff(time, eph->toe);

  long double mu, omge;
  int prn;
  int sys = satsys(eph->sat, &prn);
  switch (sys) {
    case SYS_GAL:
      mu = MU_GAL;
      omge = OMGE_GAL;
      break;
    case SYS_CMP:
      mu = MU_CMP;
      omge = OMGE_CMP;
      break;
    default:
      mu = MU_GPS;
      omge = OMGE;
      break;
  }
  long double M = eph->M0 + (sqrtl(mu / (eph->A * eph->A * eph->A)) + eph->deln) * tk;

  int n;
  long double E = M, Ek = 0.0L;
  for (n = 0; fabsl(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++) {
    Ek = E;
    E -= (E - eph->e * sinl(E) - M) / (1.0L - eph->e * cosl(E));
  }
  if (n >= MAX_ITER_KEPLER) {
    trace(2, "eph2pos: kepler iteration overflow sat=%2d\n", eph->sat);
  }
  long double sinE = sinl(E);
  long double cosE = cosl(E);

  trace(4, "kepler: sat=%2d e=%8.5Lf n=%2d del=%10.3Le\n", eph->sat, eph->e, n, E - Ek);

  long double u = atan2l(sqrtl(1.0L - eph->e * eph->e) * sinE, cosE - eph->e) + eph->omg;
  long double r = eph->A * (1.0L - eph->e * cosE);
  long double i = eph->i0 + eph->idot * tk;
  long double sin2u = sinl(2.0L * u);
  long double cos2u = cosl(2.0L * u);
  u += eph->cus * sin2u + eph->cuc * cos2u;
  r += eph->crs * sin2u + eph->crc * cos2u;
  i += eph->cis * sin2u + eph->cic * cos2u;
  long double x = r * cosl(u);
  long double y = r * sinl(u);
  long double cosi = cosl(i);

  /* BeiDou geo satellite */
  if (sys == SYS_CMP && (prn <= 5 || prn >= 59)) { /* Ref [9] table 4-1 */
    long double O = eph->OMG0 + eph->OMGd * tk - omge * eph->toes;
    long double sinO = sinl(O);
    long double cosO = cosl(O);
    long double xg = x * cosO - y * cosi * sinO;
    long double yg = x * sinO + y * cosi * cosO;
    long double zg = y * sinl(i);
    long double sino = sinl(omge * tk);
    long double coso = cosl(omge * tk);
    rs[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
    rs[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
    rs[2] = -yg * SIN_5 + zg * COS_5;
  } else {
    long double O = eph->OMG0 + (eph->OMGd - omge) * tk - omge * eph->toes;
    long double sinO = sinl(O);
    long double cosO = cosl(O);
    rs[0] = x * cosO - y * cosi * sinO;
    rs[1] = x * sinO + y * cosi * cosO;
    rs[2] = y * sinl(i);
  }
  tk = timediff(time, eph->toc);
  *dts = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;

  /* Relativity correction */
  *dts -= 2.0L * sqrtl(mu * eph->A) * eph->e * sinE / SQR(CLIGHT);

  /* Position and clock error variance */
  *var = var_uraeph(sys, eph->sva);
  trace(4, "eph2pos: sat=%d, dts=%.10Lf rs=%.4Lf %.4Lf %.4Lf var=%.3Lf\n", eph->sat, *dts, rs[0],
        rs[1], rs[2], *var);
}
/* GLONASS orbit differential equations --------------------------------------*/
static void deq(const long double *x, long double *xdot, const long double *acc) {
  long double r2 = dot3(x, x);
  if (r2 <= 0.0L) {
    xdot[0] = xdot[1] = xdot[2] = xdot[3] = xdot[4] = xdot[5] = 0.0L;
    return;
  }
  /* Ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
  long double r3 = r2 * sqrtl(r2);
  long double a = 1.5L * J2_GLO * MU_GLO * SQR(RE_GLO) / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
  long double b = 5.0L * x[2] * x[2] / r2;                        /* 5*z^2/r^2 */
  long double c = -MU_GLO / r3 - a * (1.0L - b);                  /* -mu/r^3-a(1-b) */
  xdot[0] = x[3];
  xdot[1] = x[4];
  xdot[2] = x[5];
  long double omg2 = SQR(OMGE_GLO);
  xdot[3] = (c + omg2) * x[0] + 2.0L * OMGE_GLO * x[4] + acc[0];
  xdot[4] = (c + omg2) * x[1] - 2.0L * OMGE_GLO * x[3] + acc[1];
  xdot[5] = (c - 2.0L * a) * x[2] + acc[2];
}
/* GLONASS position and velocity by numerical integration --------------------*/
static void glorbit(long double t, long double *x, const long double *acc) {
  long double k1[6];
  deq(x, k1, acc);
  long double w[6];
  for (int i = 0; i < 6; i++) w[i] = x[i] + k1[i] * t / 2.0L;
  long double k2[6];
  deq(w, k2, acc);
  for (int i = 0; i < 6; i++) w[i] = x[i] + k2[i] * t / 2.0L;
  long double k3[6];
  deq(w, k3, acc);
  for (int i = 0; i < 6; i++) w[i] = x[i] + k3[i] * t;
  long double k4[6];
  deq(w, k4, acc);
  for (int i = 0; i < 6; i++) x[i] += (k1[i] + 2.0L * k2[i] + 2.0L * k3[i] + k4[i]) * t / 6.0L;
}
/* GLONASS ephemeris to satellite clock bias -----------------------------------
 * Compute satellite clock bias with GLONASS ephemeris
 * Args   : gtime_t time     I   time by satellite clock (GPST)
 *          geph_t *geph     I   GLONASS ephemeris
 * Return : satellite clock bias (s)
 * Notes  : see ref [2]
 *----------------------------------------------------------------------------*/
extern long double geph2clk(gtime_t time, const geph_t *geph) {
  char tstr[40];
  trace(4, "geph2clk: time=%s sat=%2d\n", time2str(time, tstr, 3), geph->sat);

  long double t = timediff(time, geph->toe), ts = t;

  for (int i = 0; i < 2; i++) {
    t = ts - (-geph->taun + geph->gamn * t);
  }
  trace(4, "geph2clk: t=%.12Lf ts=%.12Lf taun=%.12Lf gamn=%.12Lf\n", t, ts, geph->taun, geph->gamn);
  return -geph->taun + geph->gamn * t;
}
/* GLONASS ephemeris to satellite position and clock bias ----------------------
 * Compute satellite position and clock bias with GLONASS ephemeris
 * Args   : gtime_t time     I   time (GPST)
 *          geph_t *geph     I   GLONASS ephemeris
 *          long double *rs       O   satellite position {x,y,z} (ECEF) (m)
 *          long double *dts      O   satellite clock bias (s)
 *          long double *var      O   satellite position and clock variance (m^2)
 * Return : none
 * Notes  : see ref [2]
 *----------------------------------------------------------------------------*/
extern void geph2pos(gtime_t time, const geph_t *geph, long double *rs, long double *dts,
                     long double *var) {
  char tstr[40];
  trace(4, "geph2pos: time=%s sat=%2d\n", time2str(time, tstr, 3), geph->sat);

  long double t = timediff(time, geph->toe);
  *dts = -geph->taun + geph->gamn * t;
  trace(4, "geph2pos: sat=%d\n", geph->sat);

  long double x[6];
  for (int i = 0; i < 3; i++) {
    x[i] = geph->pos[i];
    x[i + 3] = geph->vel[i];
  }
  long double tt = t < 0.0L ? -TSTEP : TSTEP;
  for (; fabsl(t) > 1E-9L; t -= tt) {
    if (fabsl(t) < TSTEP) tt = t;
    glorbit(tt, x, geph->acc);
  }
  for (int i = 0; i < 3; i++) rs[i] = x[i];

  *var = SQR(ERREPH_GLO);
}
/* SBAS ephemeris to satellite clock bias --------------------------------------
 * Compute satellite clock bias with SBAS ephemeris
 * Args   : gtime_t time     I   time by satellite clock (GPST)
 *          seph_t *seph     I   SBAS ephemeris
 * Return : satellite clock bias (s)
 * Notes  : see ref [3]
 *----------------------------------------------------------------------------*/
extern long double seph2clk(gtime_t time, const seph_t *seph) {
  char tstr[40];
  trace(4, "seph2clk: time=%s sat=%2d\n", time2str(time, tstr, 3), seph->sat);

  long double t = timediff(time, seph->t0);

  for (int i = 0; i < 2; i++) {
    t -= seph->af0 + seph->af1 * t;
  }
  return seph->af0 + seph->af1 * t;
}
/* SBAS ephemeris to satellite position and clock bias -------------------------
 * Compute satellite position and clock bias with SBAS ephemeris
 * Args   : gtime_t time     I   time (GPST)
 *          seph_t  *seph    I   SBAS ephemeris
 *          long double  *rs      O   satellite position {x,y,z} (ECEF) (m)
 *          long double  *dts     O   satellite clock bias (s)
 *          long double  *var     O   satellite position and clock variance (m^2)
 * Return : none
 * Notes  : see ref [3]
 *----------------------------------------------------------------------------*/
extern void seph2pos(gtime_t time, const seph_t *seph, long double *rs, long double *dts,
                     long double *var) {
  char tstr[40];
  trace(4, "seph2pos: time=%s sat=%2d\n", time2str(time, tstr, 3), seph->sat);

  long double t = timediff(time, seph->t0);

  for (int i = 0; i < 3; i++) {
    rs[i] = seph->pos[i] + seph->vel[i] * t + seph->acc[i] * t * t / 2.0L;
  }
  *dts = seph->af0 + seph->af1 * t;

  *var = var_uraeph(SYS_SBS, seph->sva);
}
/* Select ephemeris ----------------------------------------------------------*/
static eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav) {
  char tstr[40];
  trace(4, "seleph  : time=%s sat=%2d iode=%d\n", time2str(time, tstr, 3), sat, iode);

  int sys = satsys(sat, NULL), sel = 0;
  long double tmax;
  switch (sys) {
    case SYS_GPS:
      tmax = MAXDTOE + 1.0L;
      sel = eph_sel[0];
      break;
    case SYS_GAL:
      tmax = MAXDTOE_GAL;
      sel = eph_sel[2];
      break;
    case SYS_QZS:
      tmax = MAXDTOE_QZS + 1.0L;
      sel = eph_sel[3];
      break;
    case SYS_CMP:
      tmax = MAXDTOE_CMP + 1.0L;
      sel = eph_sel[4];
      break;
    case SYS_IRN:
      tmax = MAXDTOE_IRN + 1.0L;
      sel = eph_sel[5];
      break;
    default:
      tmax = MAXDTOE + 1.0L;
      break;
  }
  long double tmin = tmax + 1.0L;

  int j = -1;
  for (int i = 0; i < nav->n[sat - 1]; i++) {
    /* Skip empty entries for which the sat is zero. */
    if (nav->eph[sat - 1][i].sat != sat) continue;
    if (iode >= 0 && nav->eph[sat - 1][i].iode != iode) continue;
    if (sys == SYS_GAL) {
      sel = getseleph(SYS_GAL);
      /* This code is from 2.4.3 b34 but does not seem to be fully supported,
         so for now I have dropped back to the b33 code */
      /* If (sel==0&&!(nav->eph[i].code&(1<<9))) continue; */            /* I/NAV */
      /*if (sel==1&&!(nav->eph[i].code&(1<<8))) continue; */             /* F/NAV */
      if (sel == 1 && !(nav->eph[sat - 1][i].code & (1 << 9))) continue; /* I/NAV */
      if (sel == 2 && !(nav->eph[sat - 1][i].code & (1 << 8))) continue; /* F/NAV */
      if (timediff(nav->eph[sat - 1][i].toe, time) >= 0.0L) continue;    /* AOD<=0 */
    }
    long double t = fabsl(timediff(nav->eph[sat - 1][i].toe, time));
    if (t > tmax) continue;
    if (iode >= 0) return nav->eph[sat - 1] + i;
    if (t <= tmin) {
      j = i;
      tmin = t;
    } /* Toe closest to time */
  }
  if (iode >= 0 || j < 0) {
    trace(2, "no broadcast ephemeris: %s sat=%2d iode=%3d\n", time2str(time, tstr, 0), sat, iode);
    return NULL;
  }
  trace(4, "seleph: sat=%d dt=%.0Lf\n", sat, tmin);
  return nav->eph[sat - 1] + j;
}
/* Select GLONASS ephemeris --------------------------------------------------*/
static geph_t *selgeph(gtime_t time, int sat, int iode, const nav_t *nav) {
  char tstr[40];
  trace(4, "selgeph : time=%s sat=%2d iode=%2d\n", time2str(time, tstr, 3), sat, iode);

  int prn;
  if (satsys(sat, &prn) != SYS_GLO) return NULL;

  long double tmax = MAXDTOE_GLO, tmin = tmax + 1.0L;
  int j = -1;
  for (int i = 0; i < nav->ng[prn - 1]; i++) {
    if (nav->geph[prn - 1][i].sat != sat) continue;
    if (iode >= 0 && nav->geph[prn - 1][i].iode != iode) continue;
    long double t = fabsl(timediff(nav->geph[prn - 1][i].toe, time));
    if (t > tmax) continue;
    if (iode >= 0) return nav->geph[prn - 1] + i;
    if (t <= tmin) {
      j = i;
      tmin = t;
    } /* Toe closest to time */
  }
  if (iode >= 0 || j < 0) {
    trace(3, "no glonass ephemeris  : %s sat=%2d iode=%2d\n", time2str(time, tstr, 0), sat, iode);
    return NULL;
  }
  trace(4, "selgeph: sat=%d dt=%.0Lf\n", sat, tmin);
  return nav->geph[prn - 1] + j;
}
/* Select SBAS ephemeris -----------------------------------------------------*/
static seph_t *selseph(gtime_t time, int sat, const nav_t *nav) {
  char tstr[40];
  trace(4, "selseph : time=%s sat=%2d\n", time2str(time, tstr, 3), sat);

  int prn;
  if (satsys(sat, &prn) != SYS_SBS) return NULL;
  int k = prn - MINPRNSBS;

  long double tmax = MAXDTOE_SBS, tmin = tmax + 1.0L;
  int j = -1;
  for (int i = 0; i < nav->ns[k]; i++) {
    if (nav->seph[k][i].sat != sat) continue;
    long double t = fabsl(timediff(nav->seph[k][i].t0, time));
    if (t > tmax) continue;
    if (t <= tmin) {
      j = i;
      tmin = t;
    } /* Toe closest to time */
  }
  if (j < 0) {
    trace(3, "no sbas ephemeris     : %s sat=%2d\n", time2str(time, tstr, 0), sat);
    return NULL;
  }
  return nav->seph[k] + j;
}
/* Satellite clock with broadcast ephemeris ----------------------------------*/
static bool ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav, long double *dts) {
  char tstr[40];
  trace(4, "ephclk  : time=%s sat=%2d\n", time2str(time, tstr, 3), sat);

  int sys = satsys(sat, NULL);

  if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP || sys == SYS_IRN) {
    const eph_t *eph = seleph(teph, sat, -1, nav);
    if (!eph) return false;
    *dts = eph2clk(time, eph);
    return true;
  } else if (sys == SYS_GLO) {
    const geph_t *geph = selgeph(teph, sat, -1, nav);
    if (!geph) return false;
    if (fabsl(geph->taun) > 1) return 0; /* Reject invalid data to prevent fp error */
    *dts = geph2clk(time, geph);
    return true;
  } else if (sys == SYS_SBS) {
    const seph_t *seph = selseph(teph, sat, nav);
    if (!seph) return false;
    *dts = seph2clk(time, seph);
    return true;
  }

  return false;
}
/* Satellite position and clock by broadcast ephemeris -----------------------*/
static bool ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav, int iode, long double *rs,
                   long double *dts, long double *var, int *svh) {
  char tstr[40];
  trace(4, "ephpos  : time=%s sat=%2d iode=%d\n", time2str(time, tstr, 3), sat, iode);

  int sys = satsys(sat, NULL);

  *svh = -1;

  long double rst[3], dtst[1], tt = 1E-3L;
  if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP || sys == SYS_IRN) {
    const eph_t *eph = seleph(teph, sat, iode, nav);
    if (!eph) return false;
    eph2pos(time, eph, rs, dts, var);
    time = timeadd(time, tt);
    eph2pos(time, eph, rst, dtst, var);
    *svh = eph->svh;
  } else if (sys == SYS_GLO) {
    const geph_t *geph = selgeph(teph, sat, iode, nav);
    if (!geph) return false;
    geph2pos(time, geph, rs, dts, var);
    time = timeadd(time, tt);
    geph2pos(time, geph, rst, dtst, var);
    *svh = geph->svh;
  } else if (sys == SYS_SBS) {
    const seph_t *seph = selseph(teph, sat, nav);
    if (!seph) return false;
    seph2pos(time, seph, rs, dts, var);
    time = timeadd(time, tt);
    seph2pos(time, seph, rst, dtst, var);
    *svh = seph->svh;
  } else
    return false;

  /* Satellite velocity and clock drift by differential approx */
  for (int i = 0; i < 3; i++) rs[i + 3] = (rst[i] - rs[i]) / tt;
  dts[1] = (dtst[0] - dts[0]) / tt;

  return true;
}
/* Satellite position and clock with SBAS correction -------------------------*/
static bool satpos_sbas(gtime_t time, gtime_t teph, int sat, const nav_t *nav, long double *rs,
                        long double *dts, long double *var, int *svh) {
  char tstr[40];
  trace(4, "satpos_sbas: time=%s sat=%2d\n", time2str(time, tstr, 3), sat);

  /* Search SBAS satellite correction */
  const sbssatp_t *sbs = NULL;
  int i;
  for (i = 0; i < nav->sbssat.nsat; i++) {
    sbs = nav->sbssat.sat + i;
    if (sbs->sat == sat) break;
  }
  if (i >= nav->sbssat.nsat) {
    trace(2, "no sbas, use brdcast: %s sat=%2d\n", time2str(time, tstr, 0), sat);
    if (!ephpos(time, teph, sat, nav, -1, rs, dts, var, svh)) return false;
    /* *svh=-1; */ /* Use broadcast if no SBAS */
    return true;
  }
  /* Satellite position and clock by broadcast ephemeris */
  if (!ephpos(time, teph, sat, nav, sbs->lcorr.iode, rs, dts, var, svh)) return false;

  /* SBAS satellite correction (long term and fast) */
  if (sbssatcorr(time, sat, nav, rs, dts, var)) return true;
  *svh = -1;
  return false;
}
/* Satellite position and clock with SSR correction --------------------------*/
static bool satpos_ssr(gtime_t time, gtime_t teph, int sat, const nav_t *nav, int opt,
                       long double *rs, long double *dts, long double *var, int *svh) {
  char tstr[40];
  trace(4, "satpos_ssr: time=%s sat=%2d\n", time2str(time, tstr, 3), sat);

  const ssr_t *ssr = nav->ssr + sat - 1;

  if (!ssr->t0[0].time) {
    trace(2, "no ssr orbit correction: %s sat=%2d\n", time2str(time, tstr, 0), sat);
    return false;
  }
  if (!ssr->t0[1].time) {
    trace(2, "no ssr clock correction: %s sat=%2d\n", time2str(time, tstr, 0), sat);
    return false;
  }
  /* Inconsistency between orbit and clock correction */
  if (ssr->iod[0] != ssr->iod[1]) {
    trace(2, "inconsist ssr correction: %s sat=%2d iod=%d %d\n", time2str(time, tstr, 0), sat,
          ssr->iod[0], ssr->iod[1]);
    *svh = -1;
    return false;
  }
  long double t1 = timediff(time, ssr->t0[0]);
  long double t2 = timediff(time, ssr->t0[1]);
  long double t3 = timediff(time, ssr->t0[2]);

  /* SSR orbit and clock correction (ref [4]) */
  if (fabsl(t1) > MAXAGESSR || fabsl(t2) > MAXAGESSR) {
    trace(2, "age of ssr error: %s sat=%2d t=%.0Lf %.0Lf\n", time2str(time, tstr, 0), sat, t1, t2);
    *svh = -1;
    return false;
  }
  if (ssr->udi[0] >= 1.0L) t1 -= ssr->udi[0] / 2.0L;
  if (ssr->udi[1] >= 1.0L) t2 -= ssr->udi[1] / 2.0L;

  long double deph[3];
  for (int i = 0; i < 3; i++) deph[i] = ssr->deph[i] + ssr->ddeph[i] * t1;
  long double dclk = ssr->dclk[0] + ssr->dclk[1] * t2 + ssr->dclk[2] * t2 * t2;

  /* SSR highrate clock correction (ref [4]) */
  if (ssr->iod[0] == ssr->iod[2] && ssr->t0[2].time && fabsl(t3) < MAXAGESSR_HRCLK) {
    dclk += ssr->hrclk;
  }
  if (norm(deph, 3) > MAXECORSSR || fabsl(dclk) > MAXCCORSSR) {
    trace(3, "invalid ssr correction: %s deph=%.1Lf dclk=%.1Lf\n", time2str(time, tstr, 0),
          norm(deph, 3), dclk);
    *svh = -1;
    return false;
  }
  /* Satellite position and clock by broadcast ephemeris */
  if (!ephpos(time, teph, sat, nav, ssr->iode, rs, dts, var, svh)) return false;

  /* Satellite clock for GPS, Galileo and QZSS */
  int sys = satsys(sat, NULL);
  if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP) {
    const eph_t *eph = seleph(teph, sat, ssr->iode, nav);
    if (!eph) return false;

    /* Satellite clock by clock parameters */
    long double tk = timediff(time, eph->toc);
    dts[0] = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;
    dts[1] = eph->f1 + 2.0L * eph->f2 * tk;

    /* Relativity correction */
    dts[0] -= 2.0L * dot3(rs, rs + 3) / CLIGHT / CLIGHT;
  }
  /* Radial-along-cross directions in ECEF */
  long double ea[3];
  if (!normv3(rs + 3, ea)) return false;
  long double rc[3];
  cross3(rs, rs + 3, rc);
  long double ec[3];
  if (!normv3(rc, ec)) {
    *svh = -1;
    return false;
  }
  long double er[3];
  cross3(ea, ec, er);

  /* Satellite antenna offset correction */
  long double dant[3] = {0};
  if (opt) {
    satantoff(time, rs, sat, nav, dant);
  }
  for (int i = 0; i < 3; i++) {
    rs[i] += -(er[i] * deph[0] + ea[i] * deph[1] + ec[i] * deph[2]) + dant[i];
  }
  /* t_corr = t_sv - (dts(brdc) + dclk(SSR) / CLIGHT) (ref [10] eq.3.12-7) */
  dts[0] += dclk / CLIGHT;

  /* Variance by SSR URA */
  *var = var_urassr(ssr->ura);

  trace(5,
        "satpos_ssr: %s sat=%2d deph=%6.3Lf %6.3Lf %6.3Lf er=%6.3Lf %6.3Lf %6.3Lf dclk=%6.3Lf "
        "var=%6.3Lf\n",
        time2str(time, tstr, 2), sat, deph[0], deph[1], deph[2], er[0], er[1], er[2], dclk, *var);

  return true;
}
/* Satellite position and clock ------------------------------------------------
 * Compute satellite position, velocity and clock
 * Args   : gtime_t time     I   time (GPST)
 *          gtime_t teph     I   time to select ephemeris (GPST)
 *          int    sat       I   satellite number
 *          int    ephopt    I   ephemeris option (EPHOPT_???)
 *          nav_t  *nav      I   navigation data
 *          long double *rs       O   sat position and velocity (ECEF)
 *                               {x,y,z,vx,vy,vz} (m|m/s)
 *          long double *dts      O   sat clock {bias,drift} (s|s/s)
 *          long double *var      O   sat position and clock error variance (m^2)
 *          int    *svh      O   sat health flag (-1:correction not available)
 * Return : status (true:ok,false:error)
 * Notes  : satellite position is referenced to antenna phase center
 *          satellite clock does not include code bias correction (tgd or bgd)
 *----------------------------------------------------------------------------*/
extern bool satpos(gtime_t time, gtime_t teph, int sat, int ephopt, const nav_t *nav,
                   long double *rs, long double *dts, long double *var, int *svh) {
  char tstr[40];
  trace(4, "satpos  : time=%s sat=%2d ephopt=%d\n", time2str(time, tstr, 3), sat, ephopt);

  *svh = 0;

  switch (ephopt) {
    case EPHOPT_BRDC:
      return ephpos(time, teph, sat, nav, -1, rs, dts, var, svh);
    case EPHOPT_SBAS:
      return satpos_sbas(time, teph, sat, nav, rs, dts, var, svh);
    case EPHOPT_SSRAPC:
      return satpos_ssr(time, teph, sat, nav, 0, rs, dts, var, svh);
    case EPHOPT_SSRCOM:
      return satpos_ssr(time, teph, sat, nav, 1, rs, dts, var, svh);
    case EPHOPT_PREC:
      if (!peph2pos(time, sat, nav, 1, rs, dts, var))
        break;
      else
        return true;
  }
  *svh = -1;
  return false;
}
/* Satellite positions and clocks ----------------------------------------------
 * Compute satellite positions, velocities and clocks
 * Args   : gtime_t teph     I   time to select ephemeris (GPST)
 *          obsd_t *obs      I   observation data
 *          int    n         I   number of observation data
 *          nav_t  *nav      I   navigation data
 *          int    ephopt    I   ephemeris option (EPHOPT_???)
 *          long double *rs       O   satellite positions and velocities (ECEF)
 *          long double *dts      O   satellite clocks
 *          long double *var      O   sat position and clock error variances (m^2)
 *          int    *svh      O   sat health flag (-1:correction not available)
 * Return : none
 * Notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
 *          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
 *          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
 *          var[i]        = obs[i] sat position and clock error variance (m^2)
 *          svh[i]        = obs[i] sat health flag
 *          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
 *          satellite position and clock are values at signal transmission time
 *          satellite position is referenced to antenna phase center
 *          satellite clock does not include code bias correction (tgd or bgd)
 *          any pseudorange and broadcast ephemeris are always needed to get
 *          signal transmission time
 *----------------------------------------------------------------------------*/
extern void satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav, int ephopt,
                    long double *rs, long double *dts, long double *var, int *svh) {
  gtime_t time[2 * MAXOBS] = {{0}};

  char tstr[40];
  trace(3, "satposs : teph=%s n=%d ephopt=%d\n", time2str(teph, tstr, 3), n, ephopt);

  for (int i = 0; i < n && i < 2 * MAXOBS; i++) {
    for (int j = 0; j < 6; j++) rs[j + i * 6] = 0.0L;
    for (int j = 0; j < 2; j++) dts[j + i * 2] = 0.0L;
    var[i] = 0.0L;
    svh[i] = 0;

    /* Search any pseudorange */
    int j;
    long double pr = 0.0L;
    for (j = 0; j < NFREQ; j++) {
      pr = obs[i].P[j];
      if (pr != 0.0L) break;
    }

    if (j >= NFREQ) {
      trace(2, "no pseudorange %s sat=%2d\n", time2str(obs[i].time, tstr, 3), obs[i].sat);
      continue;
    }
    /* Transmission time by satellite clock */
    time[i] = timeadd(obs[i].time, -pr / CLIGHT);

    /* Satellite clock bias by broadcast ephemeris */
    long double dt;
    if (!ephclk(time[i], teph, obs[i].sat, nav, &dt)) {
      trace(3, "no broadcast clock %s sat=%2d\n", time2str(time[i], tstr, 3), obs[i].sat);
      continue;
    }
    time[i] = timeadd(time[i], -dt);

    /* Satellite position and clock at transmission time */
    if (!satpos(time[i], teph, obs[i].sat, ephopt, nav, rs + i * 6, dts + i * 2, var + i,
                svh + i)) {
      trace(3, "no ephemeris %s sat=%2d\n", time2str(time[i], tstr, 3), obs[i].sat);
      continue;
    }
    /* If no precise clock available, use broadcast clock instead */
    if (dts[i * 2] == 0.0L) {
      if (!ephclk(time[i], teph, obs[i].sat, nav, dts + i * 2)) continue;
      dts[1 + i * 2] = 0.0L;
      *var = SQR(STD_BRDCCLK);
    }
    trace(4,
          "satposs: %d,time=%.9Lf dt=%.9Lf pr=%.3Lf rs=%13.3Lf %13.3Lf %13.3Lf dts=%12.3Lf "
          "var=%7.3Lf\n",
          obs[i].sat, time[i].sec, dt, pr, rs[i * 6], rs[1 + i * 6], rs[2 + i * 6],
          dts[i * 2] * 1E9L, var[i]);
  }
  for (int i = 0; i < n && i < 2 * MAXOBS; i++) {
    trace(4, "%s sat=%2d rs=%13.3Lf %13.3Lf %13.3Lf dts=%12.3Lf var=%7.3Lf svh=%02X\n",
          time2str(time[i], tstr, 9), obs[i].sat, rs[i * 6], rs[1 + i * 6], rs[2 + i * 6],
          dts[i * 2] * 1E9L, var[i], svh[i]);
  }
}
/* Set selected satellite ephemeris --------------------------------------------
 * Set selected satellite ephemeris for multiple ones like LNAV - CNAV, I/NAV -
 * F/NAV. Call it before calling satpos(),satposs() to use unselected one.
 * Args   : int    sys       I   satellite system (SYS_???)
 *          int    sel       I   selection of ephemeris
 *                                 GPS,QZS : 0:LNAV ,1:CNAV  (default: LNAV)
 *  b33 and demo5 b34:             GAL: 0:any,1:I/NAV,2:F/NAV
 *  2.4.3 b34 but not functional?  GAL     : 0:I/NAV,1:F/NAV (default: I/NAV)
 *                                 others : undefined
 * Return : none
 *----------------------------------------------------------------------------*/
extern void setseleph(int sys, int sel) {
  switch (sys) {
    case SYS_GPS:
      eph_sel[0] = sel;
      break;
    case SYS_GLO:
      eph_sel[1] = sel;
      break;
    case SYS_GAL:
      eph_sel[2] = sel;
      break;
    case SYS_QZS:
      eph_sel[3] = sel;
      break;
    case SYS_CMP:
      eph_sel[4] = sel;
      break;
    case SYS_IRN:
      eph_sel[5] = sel;
      break;
    case SYS_SBS:
      eph_sel[6] = sel;
      break;
  }
}
/* Get selected satellite ephemeris --------------------------------------------
 * Get the selected satellite ephemeris.
 * Args   : int    sys       I   satellite system (SYS_???)
 * Return : selected ephemeris
 *            refer setseleph()
 *----------------------------------------------------------------------------*/
extern int getseleph(int sys) {
  switch (sys) {
    case SYS_GPS:
      return eph_sel[0];
    case SYS_GLO:
      return eph_sel[1];
    case SYS_GAL:
      return eph_sel[2];
    case SYS_QZS:
      return eph_sel[3];
    case SYS_CMP:
      return eph_sel[4];
    case SYS_IRN:
      return eph_sel[5];
    case SYS_SBS:
      return eph_sel[6];
  }
  return 0;
}
