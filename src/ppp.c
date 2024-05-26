/*------------------------------------------------------------------------------
 * ppp.c : precise point positioning
 *
 *          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
 *
 * Options : -DIERS_MODEL  use IERS tide model
 *           -DOUTSTAT_AMB output ambiguity parameters to solution status
 *
 * References :
 *    [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
 *    [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
 *        2003, November 2003
 *    [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
 *        Space Technology Library, 2004
 *    [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
 *        May 2009
 *    [5] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
 *        Code Biases, URA
 *    [6] MacMillan et al., Atmospheric gradients and the VLBI terrestrial and
 *        celestial reference frames, Geophys. Res. Let., 1997
 *    [7] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
 *         Conventions (2010), 2010
 *    [8] J.Kouba, A simplified yaw-attitude model for eclipsing GPS satellites,
 *        GPS Solutions, 13:1-12, 2009
 *    [9] F.Dilssner, GPS IIF-1 satellite antenna phase center and attitude
 *        modeling, InsideGNSS, September, 2010
 *    [10] F.Dilssner, The GLONASS-M satellite yaw-attitude model, Advances in
 *        Space Research, 2010
 *    [11] IGS MGEX (http://igs.org/mgex)
 *
 * Version : $Revision:$ $Date:$
 * History : 2010/07/20 1.0  new
 *                           added api:
 *                               tidedisp()
 *           2010/12/11 1.1  enable exclusion of eclipsing satellite
 *           2012/02/01 1.2  add gps-glonass h/w bias correction
 *                           move windupcorr() to rtkcmn.c
 *           2013/03/11 1.3  add otl and pole tides corrections
 *                           involve IERS model with -DIERS_MODEL
 *                           change initial variances
 *                           suppress acos domain error
 *           2013/09/01 1.4  pole tide model by IERS 2010
 *                           add mode of ionosphere model off
 *           2014/05/23 1.5  add output of trop gradient in solution status
 *           2014/10/13 1.6  fix bug on P0(a[3]) computation in tide_oload()
 *                           fix bug on m2 computation in tide_pole()
 *           2015/03/19 1.7  fix bug on ionosphere correction for GLO and BDS
 *           2015/05/10 1.8  add function to detect slip by MW-LC jump
 *                           fix ppp solution problem with large clock variance
 *           2015/06/08 1.9  add precise satellite yaw-models
 *                           cope with day-boundary problem of satellite clock
 *           2015/07/31 1.10 fix bug on nan-solution without GLONASS nav-data
 *                           pppoutsolsat() -> pppoutstat()
 *           2015/11/13 1.11 add L5-receiver-dcb estimation
 *                           merge post-residual validation by rnx2rtkp_test
 *                           support support option opt->pppopt=-GAP_RESION=nnnn
 *           2016/01/22 1.12 delete support for yaw-model bug
 *                           add support for URA of ephemeris
 *           2018/10/10 1.13 support api change of satexclude()
 *           2020/11/30 1.14 use sat2freq() to get carrier frequency
 *                           use E1-E5b for Galileo iono-free LC
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

#define SQR(x) ((x) * (x))
#define SQRT(x) ((x) <= 0.0L || (x) != (x) ? 0.0L : sqrtl(x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define ROUND(x) (int)floorl((x) + 0.5L)

#define MAX_ITER 8        /* Max number of iterations */
#define MAX_STD_FIX 0.15L /* Max std-dev (3d) to fix solution */
#define MIN_NSAT_SOL 4    /* Min satellite number for solution */
#define THRES_REJECT 4.0L /* Reject threshold of posfit-res (sigma) */

#define THRES_MW_JUMP 10.0L

#define VAR_POS SQR(60.0L)    /* Init variance receiver position (m^2) */
#define VAR_VEL SQR(10.0L)    /* Init variance of receiver vel ((m/s)^2) */
#define VAR_ACC SQR(10.0L)    /* Init variance of receiver acc ((m/ss)^2) */
#define VAR_CLK SQR(60.0L)    /* Init variance receiver clock (m^2) */
#define VAR_ZTD SQR(0.6L)     /* Init variance ztd (m^2) */
#define VAR_GRA SQR(0.01L)    /* Init variance gradient (m^2) */
#define VAR_DCB SQR(30.0L)    /* Init variance dcb (m^2) */
#define VAR_BIAS SQR(60.0L)   /* Init variance phase-bias (m^2) */
#define VAR_IONO SQR(60.0L)   /* Init variance iono-delay */
#define VAR_GLO_IFB SQR(0.6L) /* Variance of GLONASS ifb */

#define ERR_SAAS 0.3L  /* Saastamoinen model error std (m) */
#define ERR_BRDCI 0.5L /* Broadcast iono model error factor */
#define ERR_CBIAS 0.3L /* Code bias error std (m) */
#define REL_HUMI 0.7L  /* Relative humidity for saastamoinen model */
#define GAP_RESION 120 /* Default gap to reset ionos parameters (ep) */

#define EFACT_GPS_L5 10.0L /* Error factor of GPS/QZS L5 */

#define MUDOT_GPS (0.00836L * D2R) /* Average angular velocity GPS (rad/s) */
#define MUDOT_GLO (0.00888L * D2R) /* Average angular velocity GLO (rad/s) */
#define EPS0_GPS (13.5L * D2R)     /* Max shadow crossing angle GPS (rad) */
#define EPS0_GLO (14.2L * D2R)     /* Max shadow crossing angle GLO (rad) */
#define T_POSTSHADOW 1800.0L       /* Post-shadow recovery time (s) */
#define QZS_EC_BETA 20.0L          /* Max beta angle for QZSS Ec (deg) */

/* Number and index of states */
#define NF(opt) ((opt)->ionoopt == IONOOPT_IFLC ? 1 : (opt)->nf)
#define NP(opt) ((opt)->dynamics ? 9 : 3)
#define NC(opt) (NSYS)
#define NT(opt) ((opt)->tropopt < TROPOPT_EST ? 0 : ((opt)->tropopt == TROPOPT_EST ? 1 : 3))
#define NI(opt) ((opt)->ionoopt == IONOOPT_EST ? MAXSAT : 0)
#define ND(opt) ((opt)->nf >= 3 ? 1 : 0)
#define NR(opt) (NP(opt) + NC(opt) + NT(opt) + NI(opt) + ND(opt))
#define NB(opt) (NF(opt) * MAXSAT)
#define NX(opt) (NR(opt) + NB(opt))
#define IC(s, opt) (NP(opt) + (s))
#define IT(opt) (NP(opt) + NC(opt))
#define II(s, opt) (NP(opt) + NC(opt) + NT(opt) + (s)-1)
#define ID(opt) (NP(opt) + NC(opt) + NT(opt) + NI(opt))
#define IB(s, f, opt) (NR(opt) + MAXSAT * (f) + (s)-1)

/* Standard deviation of state -----------------------------------------------*/
static long double STD(rtk_t *rtk, int i) {
  if (rtk->sol.stat == SOLQ_FIX) return SQRT(rtk->Pa[i + i * rtk->nx]);
  return SQRT(rtk->P[i + i * rtk->nx]);
}
/* Write solution status for PPP -----------------------------------------------
 * Note   : The output is appended to the buffer which must be nul terminated.
 */
extern void pppoutstat(rtk_t *rtk, char *buff, size_t size) {
  if (!rtk->sol.stat) return;

  trace(3, "pppoutstat:\n");

  int week;
  long double tow = time2gpst(rtk->sol.time, &week);

  const long double *x = rtk->sol.stat == SOLQ_FIX ? rtk->xa : rtk->x;

  /* Receiver position */
  rtkcatprintf(buff, size, "$POS,%d,%.3Lf,%d,%.4Lf,%.4Lf,%.4Lf,%.4Lf,%.4Lf,%.4Lf\n", week, tow,
               rtk->sol.stat, x[0], x[1], x[2], STD(rtk, 0), STD(rtk, 1), STD(rtk, 2));

  /* Receiver velocity and acceleration */
  if (rtk->opt.dynamics) {
    long double pos[3];
    ecef2pos(rtk->sol.rr, pos);
    long double vel[3];
    ecef2enu(pos, rtk->x + 3, vel);
    long double acc[3];
    ecef2enu(pos, rtk->x + 6, acc);
    rtkcatprintf(buff, size,
                 "$VELACC,%d,%.3Lf,%d,%.4Lf,%.4Lf,%.4Lf,%.5Lf,%.5Lf,%.5Lf,%.4Lf,%.4Lf,"
                 "%.4Lf,%.5Lf,%.5Lf,%.5Lf\n",
                 week, tow, rtk->sol.stat, vel[0], vel[1], vel[2], acc[0], acc[1], acc[2], 0.0L,
                 0.0L, 0.0L, 0.0L, 0.0L, 0.0L);
  }
  /* Receiver clocks */

  {
    int i = IC(0, &rtk->opt);
    rtkcatprintf(buff, size,
                 "$CLK,%d,%.3Lf,%d,%d,%.3Lf,%.3Lf,%.3Lf,%.3Lf,%.3Lf,%.3Lf,%.3Lf,%.3Lf\n", week, tow,
                 rtk->sol.stat, 1, x[i] * 1E9L / CLIGHT, x[i + 1] * 1E9L / CLIGHT,
                 x[i + 2] * 1E9L / CLIGHT, x[i + 3] * 1E9L / CLIGHT, STD(rtk, i) * 1E9L / CLIGHT,
                 STD(rtk, i + 1) * 1E9L / CLIGHT, STD(rtk, i + 2) * 1E9L / CLIGHT,
                 STD(rtk, i + 2) * 1E9L / CLIGHT);
  }

  /* Tropospheric parameters */
  if (rtk->opt.tropopt == TROPOPT_EST || rtk->opt.tropopt == TROPOPT_ESTG) {
    int i = IT(&rtk->opt);
    rtkcatprintf(buff, size, "$TROP,%d,%.3Lf,%d,%d,%.4Lf,%.4Lf\n", week, tow, rtk->sol.stat, 1,
                 x[i], STD(rtk, i));
  }
  if (rtk->opt.tropopt == TROPOPT_ESTG) {
    int i = IT(&rtk->opt);
    rtkcatprintf(buff, size, "$TRPG,%d,%.3Lf,%d,%d,%.5Lf,%.5Lf,%.5Lf,%.5Lf\n", week, tow,
                 rtk->sol.stat, 1, x[i + 1], x[i + 2], STD(rtk, i + 1), STD(rtk, i + 2));
  }
  /* Ionosphere parameters */
  if (rtk->opt.ionoopt == IONOOPT_EST) {
    for (int i = 0; i < MAXSAT; i++) {
      const ssat_t *ssat = rtk->ssat + i;
      if (!ssat->vs) continue;
      int j = II(i + 1, &rtk->opt);
      if (rtk->x[j] == 0.0L) continue;
      char id[8];
      satno2id(i + 1, id);
      rtkcatprintf(buff, size, "$ION,%d,%.3Lf,%d,%s,%.1Lf,%.1Lf,%.4Lf,%.4Lf\n", week, tow,
                   rtk->sol.stat, id, rtk->ssat[i].azel[0] * R2D, rtk->ssat[i].azel[1] * R2D, x[j],
                   STD(rtk, j));
    }
  }
#ifdef OUTSTAT_AMB
  /* Ambiguity parameters */
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < NF(&rtk->opt); j++) {
      int k = IB(i + 1, j, &rtk->opt);
      if (rtk->x[k] == 0.0L) continue;
      char id[8];
      satno2id(i + 1, id);
      rtkcatprintf(buff, size, "$AMB,%d,%.3Lf,%d,%s,%d,%.4Lf,%.4Lf\n", week, tow, rtk->sol.stat, id,
                   j + 1, x[k], STD(rtk, k));
    }
#endif
}
/* Exclude meas of eclipsing satellite (block IIA) ---------------------------*/
static void testeclipse(const obsd_t *obs, int n, const nav_t *nav, long double *rs) {
  trace(3, "testeclipse:\n");

  /* Unit vector of sun direction (ECEF) */
  long double rsun[3];
  const long double erpv[5] = {0};
  sunmoonpos(gpst2utc(obs[0].time), erpv, rsun, NULL, NULL);
  long double esun[3];
  normv3(rsun, esun);

  for (int i = 0; i < n; i++) {
    const char *type = nav->pcvs[obs[i].sat - 1].type;

    long double r = norm(rs + i * 6, 3);
    if (r <= 0.0L) continue;

    /* Only block IIA */
    if (*type && !strstr(type, "BLOCK IIA")) continue;

    /* Sun-earth-satellite angle */
    long double cosa = dot3(rs + i * 6, esun) / r;
    cosa = cosa < -1.0L ? -1.0L : (cosa > 1.0L ? 1.0L : cosa);
    long double ang = acosl(cosa);

    /* Test eclipse */
    if (ang < PI / 2.0L || r * sinl(ang) > RE_WGS84) continue;

    char tstr[40];
    trace(3, "eclipsing sat excluded %s sat=%2d\n", time2str(obs[0].time, tstr, 0), obs[i].sat);

    for (int j = 0; j < 3; j++) rs[j + i * 6] = 0.0L;
  }
}
/* Nominal yaw-angle ---------------------------------------------------------*/
static long double yaw_nominal(long double beta, long double mu) {
  if (fabsl(beta) < 1E-12L && fabsl(mu) < 1E-12L) return PI;
  return atan2l(-tanl(beta), sinl(mu)) + PI;
}
/* Yaw-angle of satellite ----------------------------------------------------*/
static bool yaw_angle(int sat, const char *type, int opt, long double beta, long double mu,
                      long double *yaw) {
  *yaw = yaw_nominal(beta, mu);
  return true;
}
/* Satellite attitude model --------------------------------------------------*/
static bool sat_yaw(gtime_t time, int sat, const char *type, int opt, const long double *rs,
                    long double *exs, long double *eys) {
  const long double erpv[5] = {0};
  long double rsun[3];
  sunmoonpos(gpst2utc(time), erpv, rsun, NULL, NULL);

  /* Beta and orbit angle */
  long double ri[6];
  matcpy(ri, rs, 6, 1);
  ri[3] -= OMGE * ri[1];
  ri[4] += OMGE * ri[0];
  long double n[3];
  cross3(ri, ri + 3, n);
  long double p[3];
  cross3(rsun, n, p);
  long double es[3], esun[3], en[3], ep[3];
  if (!normv3(rs, es) || !normv3(rsun, esun) || !normv3(n, en) || !normv3(p, ep)) return false;
  long double beta = PI / 2.0L - acosl(dot3(esun, en));
  long double E = acosl(dot3(es, ep));
  long double mu = PI / 2.0L + (dot3(es, esun) <= 0 ? -E : E);
  if (mu < -PI / 2.0L)
    mu += 2.0L * PI;
  else if (mu >= PI / 2.0L)
    mu -= 2.0L * PI;

  /* Yaw-angle of satellite */
  long double yaw;
  if (!yaw_angle(sat, type, opt, beta, mu, &yaw)) return false;

  /* Satellite fixed x,y-vector */
  long double ex[3];
  cross3(en, es, ex);
  long double cosy = cosl(yaw);
  long double siny = sinl(yaw);
  for (int i = 0; i < 3; i++) {
    exs[i] = -siny * en[i] + cosy * ex[i];
    eys[i] = -cosy * en[i] - siny * ex[i];
  }
  return true;
}
/* Phase windup model --------------------------------------------------------*/
static bool model_phw(gtime_t time, int sat, const char *type, int opt, const long double *rs,
                      const long double *rr, long double *phw) {
  if (opt <= 0) return true; /* No phase windup */

  /* Satellite yaw attitude model */
  long double exs[3], eys[3];
  if (!sat_yaw(time, sat, type, opt, rs, exs, eys)) return false;

  /* Unit vector satellite to receiver */
  long double r[3];
  for (int i = 0; i < 3; i++) r[i] = rr[i] - rs[i];

  long double ek[3];
  if (!normv3(r, ek)) return false;

  /* Unit vectors of receiver antenna */
  long double pos[3];
  ecef2pos(rr, pos);
  long double E[9];
  xyz2enu(pos, E);
  long double exr[3], eyr[3];
  exr[0] = E[1];
  exr[1] = E[4];
  exr[2] = E[7]; /* x = north */
  eyr[0] = -E[0];
  eyr[1] = -E[3];
  eyr[2] = -E[6]; /* y = west  */

  /* Phase windup effect */
  long double eks[3];
  cross3(ek, eys, eks);
  long double ekr[3];
  cross3(ek, eyr, ekr);
  long double dr[3], ds[3];
  for (int i = 0; i < 3; i++) {
    ds[i] = exs[i] - ek[i] * dot3(ek, exs) - eks[i];
    dr[i] = exr[i] - ek[i] * dot3(ek, exr) + ekr[i];
  }
  long double cosp = dot3(ds, dr) / norm(ds, 3) / norm(dr, 3);
  if (cosp < -1.0L)
    cosp = -1.0L;
  else if (cosp > 1.0L)
    cosp = 1.0L;
  long double ph = acosl(cosp) / 2.0L / PI;
  long double drs[3];
  cross3(ds, dr, drs);
  if (dot3(ek, drs) < 0.0L) ph = -ph;

  *phw = ph + floorl(*phw - ph + 0.5L); /* In cycle */
  return true;
}

/* Measurement error variance ------------------------------------------------*/
static long double varerr(int sat, int sys, long double el, long double snr_rover, int f,
                          const prcopt_t *opt, const obsd_t *obs) {
  int frq = f / 2;
  int code = f % 2; /* 0=phase, 1=code */

  /* Firstly establish some factors that will scale the variance */

  /* System error factor */
  long double sys_fact;
  switch (sys) {
    case SYS_GPS:
      sys_fact = EFACT_GPS;
      break;
    case SYS_GLO:
      sys_fact = EFACT_GLO;
      break;
    case SYS_GAL:
      sys_fact = EFACT_GAL;
      break;
    case SYS_SBS:
      sys_fact = EFACT_SBS;
      break;
    case SYS_QZS:
      sys_fact = EFACT_QZS;
      break;
    case SYS_CMP:
      sys_fact = EFACT_CMP;
      break;
    case SYS_IRN:
      sys_fact = EFACT_IRN;
      break;
    default:
      sys_fact = EFACT_GPS;
      break;
  }

    /* System/frequency factors */
    /* The rtkpos varerr function does not use this factor.
       Define there, or undefine here, to be consistent. */
#define VAR_GPS_QZS_L5_FACT
#ifdef VAR_GPS_QZS_L5_FACT
  /* GPS/QZS L5 error factor */
  if (sys == SYS_GPS || sys == SYS_QZS) {
    if (frq == 2) sys_fact *= EFACT_GPS_L5;
  }
#endif

  /* Code/phase/frequency factor */
  long double code_freq_fact = opt->eratio[frq];
  /* The rtkpos varerr function does not implement this guard.
     Define there to be consistent. */
#define VAR_GUARD_ERATIO
#ifdef VAR_GUARD_ERATIO
  /* Guard against a configuration eratio being zero, or less */
  if (code_freq_fact <= 0.0L) code_freq_fact = opt->eratio[0];
#endif
  /* Increased variance for pseudoranges */
  if (!code) {
    /* The rtkpos varerr function use the ratio.
       #define VAR_PHASE_FREQ_RATIO to be consistent */
#ifdef VAR_PHASE_FREQ_RATIO
    /* Phase: adjust variance between freqs */
    code_freq_fact /= opt->eratio[0];
#else
    code_freq_fact = 1.0L;
#endif
  }

  /* IONOOPT IFLC factor */
  long double iflc_fact = (opt->ionoopt == IONOOPT_IFLC) ? 3.0L : 1.0L;

  /* Variance using an additive model */

  /* Base term */
  long double a = opt->err[1];
  long double var = SQR(a);

  /* Satellite elevation term */
  /* The pntpos varerr function limits the elevation.
     #define VAR_MIN_EL in radians to be consistent. */
#ifdef VAR_MIN_EL
  if (el < VAR_MIN_EL) el = VAR_MIN_EL;
#endif
  long double b = opt->err[2];
  /* The pntpos varerr function scales the elevation variance by 1/sin(el)
     Undefine VAR_SQR_SINEL to be consistent. */
#define VAR_SQR_SINEL
#ifdef VAR_SQR_SINEL
  var += SQR(b / sinl(el));
#else
  var += SQR(b) / sinl(el);
#endif

  /* Add the SNR term, if not zero */
  if (opt->err[6] > 0.0L) {
    long double d = opt->err[6];
    /* #define VAR_SNR_NO_MAX to not have the SNR curve relative to the maximum SNR */
#ifndef VAR_SNR_NO_MAX
    long double snr_max = opt->err[5];
    var += SQR(d) * powl(10, 0.1L * MAX(snr_max - snr_rover, 0));
#else
    var += SQR(d) * powl(10, -0.1L * snr_rover);
#endif
  }

  /* Scale the above terms */
  var *= SQR(sys_fact * code_freq_fact);

  /* Add the receiver std estimate
     TODO perhaps move after scaling below? */
  if (opt->err[7] > 0.0L) {
    long double e = opt->err[7];
    if (code)
      var += SQR(e) * SQR(0.01L * (1 << (obs->Pstd[frq] + 5))); /* 0.01*2^(n+5) */
    else
      var += SQR(e) * SQR(obs->Lstd[frq] * 0.004L * 0.2L); /* 0.004 cycles -> m */
  }

  /* Scale the above terms */
  var *= SQR(iflc_fact);

  return var;
}

/* Initialize state and covariance -------------------------------------------*/
static inline void initx(rtk_t *rtk, long double xi, long double var, int i) {
  rtk->x[i] = xi;
  for (int j = 0; j < rtk->nx; j++) rtk->P[i + j * rtk->nx] = 0.0L;
  for (int j = 0; j < rtk->nx; j++) rtk->P[j + i * rtk->nx] = 0.0L;
  rtk->P[i + i * rtk->nx] = var;
}
/* Geometry-free phase measurement -------------------------------------------*/
static long double gfmeas(const obsd_t *obs, const nav_t *nav) {
  long double freq1 = sat2freq(obs->sat, obs->code[0], nav);
  long double freq2 = sat2freq(obs->sat, obs->code[1], nav);
  if (freq1 == 0.0L || freq2 == 0.0L || obs->L[0] == 0.0L || obs->L[1] == 0.0L) return 0.0L;
  return (obs->L[0] / freq1 - obs->L[1] / freq2) * CLIGHT;
}
/* Melbourne-Wubbena linear combination --------------------------------------*/
static long double mwmeas(const obsd_t *obs, const nav_t *nav) {
  long double freq1 = sat2freq(obs->sat, obs->code[0], nav);
  long double freq2 = sat2freq(obs->sat, obs->code[1], nav);

  if (freq1 == 0.0L || freq2 == 0.0L || obs->L[0] == 0.0L || obs->L[1] == 0.0L ||
      obs->P[0] == 0.0L || obs->P[1] == 0.0L)
    return 0.0L;
  trace(3, "mwmeas: %12.1Lf %12.1Lf %15.3Lf %15.3Lf %15.3Lf %15.3Lf %d %d\n", freq1, freq2,
        obs->L[0], obs->L[1], obs->P[0], obs->P[1], obs->code[0], obs->code[1]);
  return (obs->L[0] - obs->L[1]) * CLIGHT / (freq1 - freq2) -
         (freq1 * obs->P[0] + freq2 * obs->P[1]) / (freq1 + freq2);
}
/* Antenna corrected measurements --------------------------------------------*/
static void corr_meas(const obsd_t *obs, const nav_t *nav, const long double *azel,
                      const prcopt_t *opt, const long double *dantr, const long double *dants,
                      long double phw, long double *L, long double *P, long double *Lc,
                      long double *Pc) {
  long double freq[NFREQ] = {0};
  int sys = satsys(obs->sat, NULL);

  for (int i = 0; i < opt->nf; i++) {
    L[i] = P[i] = 0.0L;
    /* Skip if low SNR or missing observations */
    freq[i] = sat2freq(obs->sat, obs->code[i], nav);
    if (freq[i] == 0.0L || obs->L[i] == 0.0L || obs->P[i] == 0.0L) continue;
    if (testsnr(0, 0, azel[1], obs->SNR[i] * SNR_UNIT, &opt->snrmask)) continue;

    /* Antenna phase center and phase windup correction */
    L[i] = obs->L[i] * CLIGHT / freq[i] - dants[i] - dantr[i] - phw * CLIGHT / freq[i];
    P[i] = obs->P[i] - dants[i] - dantr[i];

    if (opt->sateph == EPHOPT_SSRAPC || opt->sateph == EPHOPT_SSRCOM) {
      /* Select SSR code correction based on code */
      int ix = 0;
      if (sys == SYS_GPS)
        ix = (i == 0 ? CODE_L1W - 1 : CODE_L2W - 1);
      else if (sys == SYS_GLO)
        ix = (i == 0 ? CODE_L1P - 1 : CODE_L2P - 1);
      else if (sys == SYS_GAL)
        ix = (i == 0 ? CODE_L1X - 1 : CODE_L7X - 1);
      /* Apply SSR correction */
      P[i] += (nav->ssr[obs->sat - 1].cbias[obs->code[i] - 1] - nav->ssr[obs->sat - 1].cbias[ix]);
    } else { /* Apply code bias corrections from file */
      int frq;
      if (sys == SYS_GAL && (i == 1 || i == 2))
        frq = 3 - i; /* GAL biases are L1/L5 */
      else
        frq = i; /* Other biases are L1/L2 */
      if (frq >= MAX_CODE_BIAS_FREQS)
        continue; /* Only 2 freqs per system supported in code bias table */
      int bias_ix = code2bias_ix(sys, obs->code[i]);        /* Look up bias index in table */
      if (bias_ix > 0) {                                    /*  0=ref code */
        P[i] += nav->cbias[obs->sat - 1][frq][bias_ix - 1]; /* Code bias */
      }
    }
  }
  /* Choose freqs for iono-free LC */
  *Lc = *Pc = 0.0L;
  int frq2 = L[1] == 0 ? 2 : 1; /* If L[1]==0, try L[2] */
  if (freq[0] == 0.0L || freq[frq2] == 0.0L) return;
  long double C1 = SQR(freq[0]) / (SQR(freq[0]) - SQR(freq[frq2]));
  long double C2 = -SQR(freq[frq2]) / (SQR(freq[0]) - SQR(freq[frq2]));

  if (L[0] != 0.0L && L[frq2] != 0.0L) *Lc = C1 * L[0] + C2 * L[frq2];
  if (P[0] != 0.0L && P[frq2] != 0.0L) *Pc = C1 * P[0] + C2 * P[frq2];
}
/* Detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int n) {
  int nf = rtk->opt.nf;

  trace(3, "detslp_ll: n=%d\n", n);

  for (int i = 0; i < n && i < MAXOBS; i++)
    for (int j = 0; j < nf; j++) {
      if (obs[i].L[j] == 0.0L || !(obs[i].LLI[j] & 3)) continue;

      trace(3, "detslp_ll: slip detected sat=%2d f=%d\n", obs[i].sat, j + 1);

      rtk->ssat[obs[i].sat - 1].slip[j] = 1;
    }
}
/* Detect cycle slip by geometry free phase jump -----------------------------*/
static void detslp_gf(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav) {
  trace(4, "detslp_gf: n=%d\n", n);

  for (int i = 0; i < n && i < MAXOBS; i++) {
    long double g1 = gfmeas(obs + i, nav);
    if (g1 == 0.0L) continue;

    long double g0 = rtk->ssat[obs[i].sat - 1].gf[0];
    rtk->ssat[obs[i].sat - 1].gf[0] = g1;

    trace(4, "detslip_gf: sat=%2d gf0=%8.3Lf gf1=%8.3Lf\n", obs[i].sat, g0, g1);

    if (g0 != 0.0L && fabsl(g1 - g0) > rtk->opt.thresslip) {
      trace(3, "detslip_gf: slip detected sat=%2d gf=%8.3Lf->%8.3Lf\n", obs[i].sat, g0, g1);

      for (int j = 0; j < rtk->opt.nf; j++) rtk->ssat[obs[i].sat - 1].slip[j] |= 1;
    }
  }
}
/* Detect slip by Melbourne-Wubbena linear combination jump ------------------*/
static void detslp_mw(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav) {
  trace(4, "detslp_mw: n=%d\n", n);

  for (int i = 0; i < n && i < MAXOBS; i++) {
    long double w1 = mwmeas(obs + i, nav);
    if (w1 == 0.0L) continue;

    long double w0 = rtk->ssat[obs[i].sat - 1].mw[0];
    rtk->ssat[obs[i].sat - 1].mw[0] = w1;

    trace(4, "detslip_mw: sat=%2d mw0=%8.3Lf mw1=%8.3Lf\n", obs[i].sat, w0, w1);

    if (w0 != 0.0L && fabsl(w1 - w0) > THRES_MW_JUMP) {
      trace(3, "detslip_mw: slip detected sat=%2d mw=%8.3Lf->%8.3Lf\n", obs[i].sat, w0, w1);

      for (int j = 0; j < rtk->opt.nf; j++) rtk->ssat[obs[i].sat - 1].slip[j] |= 1;
    }
  }
}
/* Temporal update of position -----------------------------------------------*/
static void udpos_ppp(rtk_t *rtk) {
  trace(3, "udpos_ppp:\n");

  /* Fixed mode */
  if (rtk->opt.mode == PMODE_PPP_FIXED) {
    for (int i = 0; i < 3; i++) initx(rtk, rtk->opt.ru[i], 1E-8L, i);
    return;
  }
  /* Initialize position for first epoch */
  if (norm(rtk->x, 3) <= 0.0L) {
    for (int i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
    if (rtk->opt.dynamics) {
      for (int i = 3; i < 6; i++) initx(rtk, rtk->sol.rr[i], VAR_VEL, i);
      for (int i = 6; i < 9; i++) initx(rtk, 1E-6L, VAR_ACC, i);
    }
  }
  /* Static ppp mode */
  if (rtk->opt.mode == PMODE_PPP_STATIC) {
    for (int i = 0; i < 3; i++) {
      rtk->P[i * (1 + rtk->nx)] += SQR(rtk->opt.prn[5]) * fabsl(rtk->tt);
    }
    return;
  }
  /* Kinematic mode without dynamics */
  if (!rtk->opt.dynamics) {
    for (int i = 0; i < 3; i++) {
      initx(rtk, rtk->sol.rr[i], VAR_POS, i);
    }
    return;
  }
  /* Check variance of estimated position */
  long double var = 0.0L;
  for (int i = 0; i < 3; i++) var += rtk->P[i + i * rtk->nx];
  var /= 3.0L;

  if (var > VAR_POS) {
    /* Reset position with large variance */
    for (int i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
    for (int i = 3; i < 6; i++) initx(rtk, rtk->sol.rr[i], VAR_VEL, i);
    for (int i = 6; i < 9; i++) initx(rtk, 1E-6L, VAR_ACC, i);
    trace(2, "reset rtk position due to large variance: var=%.3Lf\n", var);
    return;
  }
  /* Generate valid state index */
  int *ix = imat(rtk->nx, 1);
  int nx = 0;
  for (int i = 0; i < rtk->nx; i++) {
    if (i < 9 || (rtk->x[i] != 0.0L && rtk->P[i + i * rtk->nx] > 0.0L)) ix[nx++] = i;
  }
  /* State transition of position/velocity/acceleration */
  long double *F = eye(nx), *P = mat(nx, nx), *FP = mat(nx, nx), *x = mat(nx, 1), *xp = mat(nx, 1);

  for (int i = 0; i < 6; i++) {
    F[i + (i + 3) * nx] = rtk->tt;
  }
  /* Include accel terms if filter is converged */
  if (var < rtk->opt.thresar[1]) {
    for (int i = 0; i < 3; i++) {
      F[i + (i + 6) * nx] = SQR(rtk->tt) / 2.0L;
    }
  } else
    trace(3, "pos var too high for accel term: %.4Lf,%.4Lf\n", var, rtk->opt.thresar[1]);
  for (int i = 0; i < nx; i++) {
    x[i] = rtk->x[ix[i]];
    for (int j = 0; j < nx; j++) {
      P[i + j * nx] = rtk->P[ix[i] + ix[j] * rtk->nx];
    }
  }
  /* x=F*x, P=F*P*F+Q */
  matmul("NN", nx, 1, nx, F, x, xp);
  matmul("NN", nx, nx, nx, F, P, FP);
  matmul("NT", nx, nx, nx, FP, F, P);

  for (int i = 0; i < nx; i++) {
    rtk->x[ix[i]] = xp[i];
    for (int j = 0; j < nx; j++) {
      rtk->P[ix[i] + ix[j] * rtk->nx] = P[i + j * nx];
    }
  }
  /* Process noise added to only acceleration */
  long double Q[9] = {0};
  Q[0] = Q[4] = SQR(rtk->opt.prn[3]) * fabsl(rtk->tt);
  Q[8] = SQR(rtk->opt.prn[4]) * fabsl(rtk->tt);
  long double pos[3];
  ecef2pos(rtk->x, pos);
  long double Qv[9];
  covecef(pos, Q, Qv);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      rtk->P[i + 6 + (j + 6) * rtk->nx] += Qv[i + j * 3];
    }
  free(ix);
  free(F);
  free(P);
  free(FP);
  free(x);
  free(xp);
}
/* Temporal update of clock --------------------------------------------------*/
static void udclk_ppp(rtk_t *rtk) {
  trace(3, "udclk_ppp:\n");

  /* Initialize every epoch for clock (white noise) */
  for (int i = 0; i < NSYS; i++) {
    long double dtr;
    if (rtk->opt.sateph == EPHOPT_PREC) {
      /* Time of prec ephemeris is based GPST */
      /* Neglect receiver inter-system bias  */
      dtr = rtk->sol.dtr[0];
    } else {
      dtr = i == 0 ? rtk->sol.dtr[0] : rtk->sol.dtr[0] + rtk->sol.dtr[i];
    }
    initx(rtk, CLIGHT * dtr, VAR_CLK, IC(i, &rtk->opt));
  }
}
/* Temporal update of tropospheric parameters --------------------------------*/
static void udtrop_ppp(rtk_t *rtk) {
  int i = IT(&rtk->opt);

  trace(3, "udtrop_ppp:\n");

  if (rtk->x[i] == 0.0L) {
    long double pos[3];
    ecef2pos(rtk->sol.rr, pos);
    const long double azel[] = {0.0L, PI / 2.0L};
    long double var;
    long double ztd = sbstropcorr(rtk->sol.time, pos, azel, &var);
    initx(rtk, ztd, var, i);

    if (rtk->opt.tropopt >= TROPOPT_ESTG) {
      for (int j = i + 1; j < i + 3; j++) initx(rtk, 1E-6L, VAR_GRA, j);
    }
  } else {
    rtk->P[i + i * rtk->nx] += SQR(rtk->opt.prn[2]) * fabsl(rtk->tt);

    if (rtk->opt.tropopt >= TROPOPT_ESTG) {
      for (int j = i + 1; j < i + 3; j++) {
        rtk->P[j + j * rtk->nx] += SQR(rtk->opt.prn[2] * 0.1L) * fabsl(rtk->tt);
      }
    }
  }
}
/* Temporal update of ionospheric parameters ---------------------------------*/
static void udiono_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav) {
  trace(3, "udiono_ppp:\n");

  int gap_resion = GAP_RESION;
  const char *p = strstr(rtk->opt.pppopt, "-GAP_RESION=");
  if (p) {
    sscanf(p, "-GAP_RESION=%d", &gap_resion);
  }
  /* Reset ionosphere delay estimate if outage too long */
  for (int i = 0; i < MAXSAT; i++) {
    int j = II(i + 1, &rtk->opt);
    if (rtk->x[j] != 0.0L && (int)rtk->ssat[i].outc[0] > gap_resion) {
      rtk->x[j] = 0.0L;
    }
  }
  for (int i = 0; i < n; i++) {
    int sat = obs[i].sat;
    int j = II(sat, &rtk->opt);
    if (rtk->x[j] == 0.0L) {
      /* Initialize ionosphere delay estimates if zero */
      int f2 = seliflc(rtk->opt.nf, satsys(sat, NULL));
      long double freq1 = sat2freq(sat, obs[i].code[0], nav);
      long double freq2 = sat2freq(sat, obs[i].code[f2], nav);
      if (obs[i].P[0] == 0.0L || obs[i].P[f2] == 0.0L || freq1 == 0.0L || freq2 == 0.0L) {
        continue;
      }
      /* Use pseudorange difference adjusted by freq for initial estimate */
      long double ion = (obs[i].P[0] - obs[i].P[f2]) / (SQR(FREQL1 / freq1) - SQR(FREQL1 / freq2));
      long double pos[3];
      ecef2pos(rtk->sol.rr, pos);
      const long double *azel = rtk->ssat[sat - 1].azel;
      /* Adjust delay estimate by path length */
      ion /= ionmapf(pos, azel);
      initx(rtk, ion, VAR_IONO, j);
      trace(4, "ion init: sat=%d ion=%.4Lf\n", sat, ion);
    } else {
      long double sinel = sinl(MAX(rtk->ssat[sat - 1].azel[1], 5.0L * D2R));
      /* Update variance of delay state */
      rtk->P[j + j * rtk->nx] += SQR(rtk->opt.prn[1] / sinel) * fabsl(rtk->tt);
    }
  }
}
/* Temporal update of L5-receiver-dcb parameters -----------------------------*/
static void uddcb_ppp(rtk_t *rtk) {
  trace(3, "uddcb_ppp:\n");

  int i = ID(&rtk->opt);
  if (rtk->x[i] == 0.0L) {
    initx(rtk, 1E-6L, VAR_DCB, i);
  }
}
/* Temporal update of phase biases -------------------------------------------*/
static void udbias_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav) {
  trace(3, "udbias  : n=%d\n", n);

  /* Handle day-boundary clock jump */
  int clk_jump = 0;
  if (rtk->opt.posopt[5]) {
    clk_jump = ROUND(time2gpst(obs[0].time, NULL) * 10) % 864000 == 0;
  }
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < rtk->opt.nf; j++) {
      rtk->ssat[i].slip[j] = 0;
    }
  /* Detect cycle slip by LLI */
  detslp_ll(rtk, obs, n);

  /* Detect cycle slip by geometry-free phase jump */
  detslp_gf(rtk, obs, n, nav);

  /* Detect slip by Melbourne-Wubbena linear combination jump */
  detslp_mw(rtk, obs, n, nav);

  long double pos[3] = {0};
  ecef2pos(rtk->sol.rr, pos);

  long double offset = 0.0L, bias[MAXOBS];
  int slip[MAXOBS] = {0};
  for (int f = 0; f < NF(&rtk->opt); f++) {
    /* Reset phase-bias if expire obs outage counter */
    for (int i = 0; i < MAXSAT; i++) {
      if (++rtk->ssat[i].outc[f] > (uint32_t)rtk->opt.maxout || rtk->opt.modear == ARMODE_INST ||
          clk_jump) {
        initx(rtk, 0.0L, 0.0L, IB(i + 1, f, &rtk->opt));
      }
    }
    int k = 0;
    for (int i = 0; i < n && i < MAXOBS; i++) {
      int sat = obs[i].sat;
      int j = IB(sat, f, &rtk->opt);
      const long double dantr[NFREQ] = {0}, dants[NFREQ] = {0};
      long double L[NFREQ], P[NFREQ], Lc, Pc;
      corr_meas(obs + i, nav, rtk->ssat[sat - 1].azel, &rtk->opt, dantr, dants, 0.0L, L, P, &Lc,
                &Pc);

      bias[i] = 0.0L;

      if (rtk->opt.ionoopt == IONOOPT_IFLC) {
        bias[i] = Lc - Pc;
        slip[i] = rtk->ssat[sat - 1].slip[0] || rtk->ssat[sat - 1].slip[1];
      } else if (L[f] != 0.0L && P[f] != 0.0L) {
        long double freq1 = sat2freq(sat, obs[i].code[0], nav);
        long double freq2 = sat2freq(sat, obs[i].code[f], nav);
        slip[i] = rtk->ssat[sat - 1].slip[f];
        long double ion;
        if (f == 0 || obs[i].P[0] == 0.0L || obs[i].P[f] == 0.0L || freq1 == 0.0L || freq2 == 0.0L)
          ion = 0;
        else
          ion = (obs[i].P[0] - obs[i].P[f]) / (1.0L - SQR(freq1 / freq2));
        bias[i] = L[f] - P[f] + 2.0L * ion * SQR(freq1 / freq2);
      }
      if (rtk->x[j] == 0.0L || slip[i] || bias[i] == 0.0L) continue;

      offset += bias[i] - rtk->x[j];
      k++;
    }
    /* Correct phase-code jump to ensure phase-code coherence */
    if (k >= 2 && fabsl(offset / k) > 0.0005L * CLIGHT) {
      for (int i = 0; i < MAXSAT; i++) {
        int j = IB(i + 1, f, &rtk->opt);
        if (rtk->x[j] != 0.0L) rtk->x[j] += offset / k;
      }
      char tstr[40];
      trace(2, "phase-code jump corrected: %s n=%2d dt=%12.9Lfs\n",
            time2str(rtk->sol.time, tstr, 0), k, offset / k / CLIGHT);
    }
    for (int i = 0; i < n && i < MAXOBS; i++) {
      int sat = obs[i].sat;
      int j = IB(sat, f, &rtk->opt);

      rtk->P[j + j * rtk->nx] += SQR(rtk->opt.prn[0]) * fabsl(rtk->tt);

      if (bias[i] == 0.0L || (rtk->x[j] != 0.0L && !slip[i])) continue;

      /* Reinitialize phase-bias if detecting cycle slip */
      initx(rtk, bias[i], VAR_BIAS, IB(sat, f, &rtk->opt));

      /* Reset fix flags */
      for (int m = 0; m < MAXSAT; m++) rtk->ambc[sat - 1].flags[m] = 0;

      trace(3, "udbias_ppp: sat=%2d bias=%.3Lf\n", sat, bias[i]);
    }
  }
}
/* Temporal update of states -------------------------------------------------*/
static void udstate_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav) {
  trace(3, "udstate_ppp: n=%d\n", n);

  /* Temporal update of position */
  udpos_ppp(rtk);

  /* Temporal update of clock */
  udclk_ppp(rtk);

  /* Temporal update of tropospheric parameters */
  if (rtk->opt.tropopt == TROPOPT_EST || rtk->opt.tropopt == TROPOPT_ESTG) {
    udtrop_ppp(rtk);
  }
  /* Temporal update of ionospheric parameters */
  if (rtk->opt.ionoopt == IONOOPT_EST) {
    udiono_ppp(rtk, obs, n, nav);
  }
  /* Temporal update of L5-receiver-dcb parameters */
  if (rtk->opt.nf >= 3) {
    uddcb_ppp(rtk);
  }
  /* Temporal update of phase-bias */
  udbias_ppp(rtk, obs, n, nav);
}
/* Satellite antenna phase center variation ----------------------------------*/
static void satantpcv(const long double *rs, const long double *rr, const pcv_t *pcv,
                      long double *dant) {
  long double ru[3], rz[3];
  for (int i = 0; i < 3; i++) {
    ru[i] = rr[i] - rs[i];
    rz[i] = -rs[i];
  }
  long double eu[3], ez[3];
  if (!normv3(ru, eu) || !normv3(rz, ez)) return;

  long double cosa = dot3(eu, ez);
  cosa = cosa < -1.0L ? -1.0L : (cosa > 1.0L ? 1.0L : cosa);
  long double nadir = acosl(cosa);

  antmodel_s(pcv, nadir, dant);
}
/* Precise tropospheric model ------------------------------------------------*/
static long double trop_model_prec(gtime_t time, const long double *pos, const long double *azel,
                                   const long double *x, long double *dtdx, long double *var) {
  const long double zazel[] = {0.0L, PI / 2.0L};

  /* Zenith hydrostatic delay */
  long double zhd = tropmodel(time, pos, zazel, 0.0L);

  /* Mapping function */
  long double m_w;
  long double m_h = tropmapf(time, pos, azel, &m_w);

  if (azel[1] > 0.0L) {
    /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
    long double cotz = 1.0L / tanl(azel[1]);
    long double grad_n = m_w * cotz * cosl(azel[0]);
    long double grad_e = m_w * cotz * sinl(azel[0]);
    m_w += grad_n * x[1] + grad_e * x[2];
    dtdx[1] = grad_n * (x[0] - zhd);
    dtdx[2] = grad_e * (x[0] - zhd);
  }
  dtdx[0] = m_w;
  *var = SQR(0.01L);
  return m_h * zhd + m_w * (x[0] - zhd);
}
/* Tropospheric model --------------------------------------------------------*/
static bool model_trop(gtime_t time, const long double *pos, const long double *azel,
                       const prcopt_t *opt, const long double *x, long double *dtdx,
                       const nav_t *nav, long double *dtrp, long double *var) {
  if (opt->tropopt == TROPOPT_SAAS) {
    *dtrp = tropmodel(time, pos, azel, REL_HUMI);
    *var = SQR(ERR_SAAS);
    return true;
  }
  if (opt->tropopt == TROPOPT_SBAS) {
    *dtrp = sbstropcorr(time, pos, azel, var);
    return true;
  }
  if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG) {
    long double trp[3] = {0};
    matcpy(trp, x + IT(opt), opt->tropopt == TROPOPT_EST ? 1 : 3, 1);
    *dtrp = trop_model_prec(time, pos, azel, trp, dtdx, var);
    return true;
  }
  return false;
}
/* Ionospheric model ---------------------------------------------------------*/
static bool model_iono(gtime_t time, const long double *pos, const long double *azel,
                       const prcopt_t *opt, int sat, const long double *x, const nav_t *nav,
                       long double *dion, long double *var) {
  if (opt->ionoopt == IONOOPT_SBAS) {
    return sbsioncorr(time, nav, pos, azel, dion, var);
  }
  if (opt->ionoopt == IONOOPT_TEC) {
    return iontec(time, nav, pos, azel, 1, dion, var);
  }
  if (opt->ionoopt == IONOOPT_BRDC) {
    *dion = ionmodel(time, nav->ion_gps, pos, azel);
    *var = SQR(*dion * ERR_BRDCI);
    return true;
  }
  if (opt->ionoopt == IONOOPT_EST) {
    /* Estimated delay is a vertical delay, apply the mapping function. */
    *dion = x[II(sat, opt)] * ionmapf(pos, azel);
    *var = 0.0L;
    return true;
  }
  if (opt->ionoopt == IONOOPT_IFLC) {
    *dion = *var = 0.0L;
    return true;
  }
  return false;
}
static inline void cvwrite(long double *V, int nc, const int *xi, int i, long double v) {
  int ii = xi[i];
  if (ii >= nc) return;
  V[ii] = v;
}
/* Phase and code residuals --------------------------------------------------*/
static int ppp_res(int post, const obsd_t *obs, int n, const long double *rs,
                   const long double *dts, const long double *var_rs, const int *svh,
                   const long double *dr, int *exc, const nav_t *nav, const long double *x,
                   rtk_t *rtk, long double *v, long double *Hc, int nc, const int *xi,
                   long double *R, long double *azel) {
  char str[40];
  time2str(obs[0].time, str, 2);

  prcopt_t *opt = &rtk->opt;
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < opt->nf; j++) rtk->ssat[i].vsat[j] = 0;

  long double rr[3];
  for (int i = 0; i < 3; i++) rr[i] = x[i] + dr[i];
  long double pos[3];
  ecef2pos(rr, pos);

  long double var[MAXOBS * 2];
  long double ve[MAXOBS * 2 * NFREQ] = {0};
  int ne = 0, obsi[MAXOBS * 2 * NFREQ] = {0}, frqi[MAXOBS * 2 * NFREQ];
  int nv = 0;
  for (int i = 0; i < n && i < MAXOBS; i++) {
    int sat = obs[i].sat;

    long double e[3];
    long double r = geodist(rs + i * 6, rr, e);
    if (r <= 0.0L || satazel(pos, e, azel + i * 2) < opt->elmin) {
      exc[i] = 1;
      continue;
    }
    int sys = satsys(sat, NULL);
    if (!sys || !rtk->ssat[sat - 1].vs || satexclude(sat, var_rs[i], svh[i], opt) || exc[i]) {
      exc[i] = 1;
      continue;
    }
    /* Tropospheric and ionospheric model */
    long double dtrp = 0.0L, dion = 0.0L, vart = 0.0L, vari = 0.0L;
    long double dtdx[3];
    if (!model_trop(obs[i].time, pos, azel + i * 2, opt, x, dtdx, nav, &dtrp, &vart) ||
        !model_iono(obs[i].time, pos, azel + i * 2, opt, sat, x, nav, &dion, &vari)) {
      continue;
    }
    /* Satellite and receiver antenna model */
    long double dants[NFREQ] = {0};
    if (opt->posopt[0]) satantpcv(rs + i * 6, rr, nav->pcvs + sat - 1, dants);
    long double dantr[NFREQ] = {0};
    antmodel(opt->pcvr, opt->antdel[0], azel + i * 2, opt->posopt[1], dantr);

    /* Phase windup model */
    if (!model_phw(rtk->sol.time, sat, nav->pcvs[sat - 1].type, opt->posopt[2] ? 2 : 0, rs + i * 6,
                   rr, &rtk->ssat[sat - 1].phw)) {
      continue;
    }
    /* Corrected phase and code measurements */
    long double L[NFREQ], P[NFREQ];
    long double Lc, Pc;
    corr_meas(obs + i, nav, azel + i * 2, &rtk->opt, dantr, dants, rtk->ssat[sat - 1].phw, L, P,
              &Lc, &Pc);

    /* Stack phase and code residuals {L1,P1,L2,P2,...} */
    for (int j = 0; j < 2 * NF(opt); j++) {
      long double C = 0.0L;
      long double *Hi = NULL;
      long double dcb = 0.0L, bias = 0.0L;
      int code = j % 2; /* 0=phase, 1=code */
      int frq = j / 2;

      long double y;
      if (opt->ionoopt == IONOOPT_IFLC) {
        y = code == 0 ? Lc : Pc;
        if (y == 0.0L) continue;
      } else {
        y = code == 0 ? L[frq] : P[frq];
        if (y == 0.0L) continue;

        int freq = sat2freq(sat, obs[i].code[frq], nav);
        if (freq == 0.0L) continue;
        /* The iono paths have already applied a slant factor. */
        C = SQR(FREQL1 / freq) * (code == 0 ? -1.0L : 1.0L);
      }
      if (Hc) {
        Hi = Hc + nv * nc;
        for (int k = 0; k < 3; k++) Hi[k] = -e[k];
        for (int k = 3; k < nc; k++) Hi[k] = 0.0L;
      }

      /* Receiver clock */
      int k;
      switch (sys) {
        case SYS_GLO:
          k = 1;
          break;
        case SYS_GAL:
          k = 2;
          break;
        case SYS_CMP:
          k = 3;
          break;
        case SYS_IRN:
          k = 4;
          break;
        default:
          k = 0;
          break;
      }
      long double cdtr = x[IC(k, opt)];
      if (Hc) {
        cvwrite(Hi, nc, xi, IC(k, opt), 1.0L);

        if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG) {
          for (int k2 = 0; k2 < (opt->tropopt >= TROPOPT_ESTG ? 3 : 1); k2++) {
            cvwrite(Hi, nc, xi, IT(opt) + k2, dtdx[k2]);
          }
        }
      }
      if (opt->ionoopt == IONOOPT_EST) {
        if (rtk->x[II(sat, opt)] == 0.0L) continue;
        /* The vertical iono delay is estimated, but the residual is
         * in the direction of the slant, so apply the slat factor
         * mapping function. */
        if (Hc) cvwrite(Hi, nc, xi, II(sat, opt), C * ionmapf(pos, azel + i * 2));
      }
      if (frq == 2 && code == 1) { /* L5-receiver-dcb */
        dcb += rtk->x[ID(opt)];
        if (Hc) cvwrite(Hi, nc, xi, ID(opt), 1.0L);
      }
      if (code == 0) { /* Phase bias */
        bias = x[IB(sat, frq, opt)];
        if (bias == 0.0L) continue;
        if (Hc) cvwrite(Hi, nc, xi, IB(sat, frq, opt), 1.0L);
      }
      /* Residual */
      long double res = y - (r + cdtr - CLIGHT * dts[i * 2] + dtrp + C * dion + dcb + bias);
      if (v) v[nv] = res;

      if (code == 0)
        rtk->ssat[sat - 1].resc[frq] = res; /* Carrier phase */
      else
        rtk->ssat[sat - 1].resp[frq] = res; /* Pseudorange */

      /* Variance */
      var[nv] = varerr(sat, sys, azel[1 + i * 2], SNR_UNIT * rtk->ssat[sat - 1].snr_rover[frq], j,
                       opt, obs + i);
      var[nv] += vart + SQR(C) * vari + var_rs[i];
      if (sys == SYS_GLO && code == 1) var[nv] += VAR_GLO_IFB;

      trace(3, "%s sat=%2d %s%d res=%9.4Lf sig=%9.4Lf el=%4.1Lf\n", str, sat, code ? "P" : "L",
            frq + 1, res, sqrtl(var[nv]), azel[1 + i * 2] * R2D);

      /* Reject satellite by pre-fit residuals */
      if (!post && opt->maxinno[code] > 0.0L && fabsl(res) > opt->maxinno[code]) {
        trace(2, "Outlier (%d) rejected %s sat=%2d %s%d res=%9.4Lf el=%4.1Lf\n", post, str, sat,
              code ? "P" : "L", frq + 1, res, azel[1 + i * 2] * R2D);
        exc[i] = 1;
        rtk->ssat[sat - 1].rejc[frq]++;
        continue;
      }
      /* Record large post-fit residuals */
      if (post && fabsl(res) > sqrtl(var[nv]) * THRES_REJECT) {
        obsi[ne] = i;
        frqi[ne] = j;
        ve[ne] = res;
        ne++;
      }
      if (code == 0) rtk->ssat[sat - 1].vsat[frq] = 1;
      nv++;
    }
  }
  /* Reject satellite with large and max post-fit residual */
  int stat = 1;
  if (post && ne > 0) {
    long double vmax = ve[0];
    int maxobs = obsi[0], maxfrq = frqi[0], rej = 0;
    for (int j = 1; j < ne; j++) {
      if (fabsl(vmax) >= fabsl(ve[j])) continue;
      vmax = ve[j];
      maxobs = obsi[j];
      maxfrq = frqi[j];
      rej = j;
    }
    int sat = obs[maxobs].sat;
    trace(2, "Outlier (%d) rejected %s sat=%2d %s%d res=%9.4Lf el=%4.1Lf\n", post, str, sat,
          maxfrq % 2 ? "P" : "L", maxfrq / 2 + 1, vmax, azel[1 + maxobs * 2] * R2D);
    exc[maxobs] = 1;
    rtk->ssat[sat - 1].rejc[maxfrq % 2]++;
    stat = 0;
    ve[rej] = 0;
  }
  if (R) {
    for (int j = 0; j < nv; j++)
      for (int i = 0; i < nv; i++) R[i + j * nv] = 0.0L;
    for (int i = 0; i < nv; i++) R[i + i * nv] = var[i];
  }
  return post ? stat : nv;
}
/* Number of estimated states ------------------------------------------------*/
extern int pppnx(const prcopt_t *opt) { return NX(opt); }
/* Update solution status ----------------------------------------------------*/
static void update_stat(rtk_t *rtk, const obsd_t *obs, int n, int stat) {
  const prcopt_t *opt = &rtk->opt;

  /* Test # of valid satellites */
  rtk->sol.ns = 0;
  for (int i = 0; i < n && i < MAXOBS; i++) {
    for (int j = 0; j < opt->nf; j++) {
      if (!rtk->ssat[obs[i].sat - 1].vsat[j]) continue;
      rtk->ssat[obs[i].sat - 1].lock[j]++;
      rtk->ssat[obs[i].sat - 1].outc[j] = 0;
      if (j == 0) rtk->sol.ns++;
    }
  }
  rtk->sol.stat = rtk->sol.ns < MIN_NSAT_SOL ? SOLQ_NONE : stat;

  if (rtk->sol.stat == SOLQ_FIX) {
    for (int i = 0; i < 3; i++) {
      rtk->sol.rr[i] = rtk->xa[i];
      rtk->sol.qr[i] = rtk->Pa[i + i * rtk->na];
    }
    rtk->sol.qr[3] = rtk->Pa[1];
    rtk->sol.qr[4] = rtk->Pa[1 + 2 * rtk->na];
    rtk->sol.qr[5] = rtk->Pa[2];
  } else {
    for (int i = 0; i < 3; i++) {
      rtk->sol.rr[i] = rtk->x[i];
      rtk->sol.qr[i] = rtk->P[i + i * rtk->nx];
    }
    rtk->sol.qr[3] = rtk->P[1];
    rtk->sol.qr[4] = rtk->P[2 + rtk->nx];
    rtk->sol.qr[5] = rtk->P[2];

    if (rtk->opt.dynamics) { /* Velocity and covariance */
      for (int i = 3; i < 6; i++) {
        rtk->sol.rr[i] = rtk->x[i];
        rtk->sol.qv[i - 3] = rtk->P[i + i * rtk->nx];
      }
      rtk->sol.qv[3] = rtk->P[4 + 3 * rtk->nx];
      rtk->sol.qv[4] = rtk->P[5 + 4 * rtk->nx];
      rtk->sol.qv[5] = rtk->P[5 + 3 * rtk->nx];
    }
  }
  rtk->sol.dtr[0] = rtk->x[IC(0, opt)];                      /* GPS */
  rtk->sol.dtr[1] = rtk->x[IC(1, opt)] - rtk->x[IC(0, opt)]; /* GLO-GPS */
  rtk->sol.dtr[2] = rtk->x[IC(2, opt)] - rtk->x[IC(0, opt)]; /* GAL-GPS */
  rtk->sol.dtr[3] = rtk->x[IC(3, opt)] - rtk->x[IC(0, opt)]; /* BDS-GPS */

  for (int i = 0; i < n && i < MAXOBS; i++)
    for (int j = 0; j < opt->nf; j++) {
      rtk->ssat[obs[i].sat - 1].snr_rover[j] = obs[i].SNR[j];
      rtk->ssat[obs[i].sat - 1].snr_base[j] = 0;
    }
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < opt->nf; j++) {
      if (rtk->ssat[i].slip[j] & 3) rtk->ssat[i].slipc[j]++;
      if (rtk->ssat[i].fix[j] == 2 && stat != SOLQ_FIX) rtk->ssat[i].fix[j] = 1;
    }
}
/* Test hold ambiguity -------------------------------------------------------*/
static bool test_hold_amb(rtk_t *rtk) {
  /* No fix-and-hold mode */
  if (rtk->opt.modear != ARMODE_FIXHOLD) return false;

  /* Reset # of continuous fixed if new ambiguity introduced */
  int stat = 0;
  for (int i = 0; i < MAXSAT; i++) {
    if (rtk->ssat[i].fix[0] != 2 && rtk->ssat[i].fix[1] != 2) continue;
    for (int j = 0; j < MAXSAT; j++) {
      if (rtk->ssat[j].fix[0] != 2 && rtk->ssat[j].fix[1] != 2) continue;
      if (!rtk->ambc[j].flags[i] || !rtk->ambc[i].flags[j]) stat = 1;
      rtk->ambc[j].flags[i] = rtk->ambc[i].flags[j] = 1;
    }
  }
  if (stat) {
    rtk->nfix = 0;
    return false;
  }
  /* Test # of continuous fixed */
  return ++rtk->nfix >= rtk->opt.minfix;
}
/* Precise point positioning -------------------------------------------------*/
extern void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav) {
  char str[40];
  time2str(obs[0].time, str, 2);
  int nx = rtk->nx;
  trace(3, "pppos   : time=%s nx=%d n=%d\n", str, nx, n);

  const prcopt_t *opt = &rtk->opt;
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < opt->nf; j++) rtk->ssat[i].fix[j] = 0;
  for (int i = 0; i < n && i < MAXOBS; i++)
    for (int j = 0; j < opt->nf; j++) {
      rtk->ssat[obs[i].sat - 1].snr_rover[j] = obs[i].SNR[j];
      rtk->ssat[obs[i].sat - 1].snr_base[j] = 0;
    }

  /* Temporal update of ekf states */
  udstate_ppp(rtk, obs, n, nav);

  /* Satellite positions and clocks */
  long double *rs = mat(6, n), *dts = mat(2, n), *var = mat(1, n);
  int svh[MAXOBS];
  satposs(obs[0].time, obs, n, nav, rtk->opt.sateph, rs, dts, var, svh);

  /* Exclude measurements of eclipsing satellite (block IIA) */
  if (rtk->opt.posopt[3]) {
    testeclipse(obs, n, nav, rs);
  }
  /* Earth tides correction */
  long double dr[3] = {0};
  if (opt->tidecorr) {
    tidedisp(gpst2utc(obs[0].time), rtk->x, opt->tidecorr == 1 ? 1 : 7, &nav->erp, opt->odisp[0],
             dr);
  }
  int nv = n * rtk->opt.nf * 2 + MAXSAT + 3;
  long double *xp = mat(nx, 1), *v = mat(nv, 1), *R = mat(nv, nv);

  long double *x = rtk->x, *P = rtk->P;
  matcpy(xp, x, nx, 1);

  /* Create list of non-zero states */
  int *ix = imat(nx, 1), *xi = imat(nx, 1);
  int nc;
  for (int i = nc = 0; i < nx; i++) {
    if (i < 9 || (x[i] != 0.0L && P[i + i * nx] > 0.0L)) {
      xi[i] = nc;
      ix[nc++] = i;
    } else
      xi[i] = 0xfffffff;
  }
  /* Compress array by removing zero elements to save computation time */
  long double *xc = mat(nc, 1), *xpc = mat(nc, 1), *Pc = mat(nc, nc), *Ppc = mat(nc, nc),
              *Hc = mat(nc, nv);
  for (int i = 0; i < nc; i++) xc[i] = xpc[i] = x[ix[i]];
  for (int j = 0; j < nc; j++)
    for (int i = 0; i < nc; i++) Pc[i + j * nc] = P[ix[i] + ix[j] * nx];

  long double *azel = zeros(2, n);
  int exc[MAXOBS] = {0}, i;
  enum solq stat = SOLQ_SINGLE;
  for (i = 0; i < MAX_ITER; i++) {
    /* Prefit residuals */
    nv = ppp_res(0, obs, n, rs, dts, var, svh, dr, exc, nav, xp, rtk, v, Hc, nc, xi, R, azel);
    if (!nv) {
      trace(2, "%s ppp (%d) no valid obs data\n", str, i + 1);
      break;
    }
    /* Measurement update of ekf states */
    /*  Do kalman filter state update on compressed arrays */
    int info = filter_(xc, Pc, Hc, v, R, nc, nv, Ppc);
    if (info) {
      trace(2, "%s ppp (%d) filter error info=%d\n", str, i + 1, info);
      break;
    }
    /* Copy values from compressed array xc back to full array xp */
    for (int k = 0; k < nc; k++) xp[ix[k]] = xc[k];

    /* Postfit residuals */
    if (ppp_res(i + 1, obs, n, rs, dts, var, svh, dr, exc, nav, xp, rtk, NULL, NULL, 0, NULL, NULL,
                azel)) {
      /* Copy values from compressed arrays back to full arrays */
      for (int k = 0; k < nc; k++) x[ix[k]] = xc[k];
      for (int j = 0; j < nc; j++)
        for (int k = 0; k < nc; k++) P[ix[k] + ix[j] * nx] = Ppc[k + j * nc];
      stat = SOLQ_PPP;
      break;
    }

    /* Restore xp and xc */
    for (int k = 0; k < nc; k++) xp[ix[k]] = xc[k] = xpc[k];
  }
  free(ix);
  free(xi);
  free(xc);
  free(xpc);
  free(Pc);
  free(Ppc);
  free(Hc);
  free(v);
  free(R);
  if (i >= MAX_ITER) {
    trace(2, "%s ppp (%d) iteration overflows\n", str, i);
  }
  if (stat == SOLQ_PPP) {
    long double *Pp = mat(nx, nx);
    matcpy(Pp, P, nx, nx);

    if (ppp_ar(rtk, obs, n, exc, nav, azel, xp, Pp) &&
        ppp_res(9, obs, n, rs, dts, var, svh, dr, exc, nav, xp, rtk, NULL, NULL, 0, NULL, NULL,
                azel)) {
      matcpy(rtk->xa, xp, nx, 1);
      matcpy(rtk->Pa, Pp, nx, nx);

      long double std[3];
      for (int i2 = 0; i2 < 3; i2++) std[i2] = sqrtl(Pp[i2 + i2 * nx]);
      if (norm(std, 3) < MAX_STD_FIX) stat = SOLQ_FIX;
    } else {
      rtk->nfix = 0;
    }
    /* Update solution status */
    update_stat(rtk, obs, n, stat);

    if (stat == SOLQ_FIX && test_hold_amb(rtk)) {
      matcpy(rtk->x, xp, nx, 1);
      matcpy(rtk->P, Pp, nx, nx);
      trace(2, "%s hold ambiguity\n", str);
      rtk->nfix = 0;
    }
    free(Pp);
  }
  free(rs);
  free(dts);
  free(var);
  free(azel);
  free(xp);
}
