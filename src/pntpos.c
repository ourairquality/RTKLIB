/*------------------------------------------------------------------------------
 * pntpos.c : standard positioning
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision:$ $Date:$
 * History : 2010/07/28 1.0  moved from rtkcmn.c
 *                           changed api:
 *                               pntpos()
 *                           deleted api:
 *                               pntvel()
 *           2011/01/12 1.1  add option to include unhealthy satellite
 *                           reject duplicated observation data
 *                           changed api: ionocorr()
 *           2011/11/08 1.2  enable snr mask for single-mode (rtklib_2.4.1_p3)
 *           2012/12/25 1.3  add variable snr mask
 *           2014/05/26 1.4  support Galileo and BeiDou
 *           2015/03/19 1.5  fix bug on ionosphere correction for GLO and BDS
 *           2018/10/10 1.6  support api change of satexclude()
 *           2020/11/30 1.7  support NavIC/IRNSS in pntpos()
 *                           no support IONOOPT_LEX option in ioncorr()
 *                           improve handling of TGD correction for each system
 *                           use E1-E5b for Galileo dual-freq iono-correction
 *                           use API sat2freq() to get carrier frequency
 *                           add output of velocity estimation error in estvel()
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Constants/macros ----------------------------------------------------------*/

#define SQR(x) ((x) * (x))
#define MAX(x, y) ((x) >= (y) ? (x) : (y))

#define QZSDT /* Enable GPS-QZS time offset estimation */
#ifdef QZSDT
#define NX (4 + 5) /* # of estimated parameters */
#else
#define NX (4 + 4) /* # of estimated parameters */
#endif
#define MAXITR 10               /* Max number of iteration for point pos */
#define ERR_ION 5.0L            /* Ionospheric delay Std (m) */
#define ERR_TROP 3.0L           /* Tropspheric delay Std (m) */
#define ERR_SAAS 0.3L           /* Saastamoinen model error Std (m) */
#define ERR_BRDCI 0.5L          /* Broadcast ionosphere model error factor */
#define ERR_CBIAS 0.3L          /* Code bias error Std (m) */
#define REL_HUMI 0.7L           /* Relative humidity for Saastamoinen model */
#define VAR_MIN_EL (5.0L * D2R) /* Min elevation for measurement error (rad) */
#define MAX_GDOP 30             /* Max gdop for valid solution  */

/* Pseudorange measurement error variance ------------------------------------*/
static long double varerr(int sat, int sys, long double el, long double snr_rover,
                          const prcopt_t *opt, const obsd_t *obs) {
  int frq = 0; /* TODO */

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
      /* The rtkpos and ppp varerr functions include EFACT_GAL.
         #define VAR_USE_EFACT_GAL to be consistent. */
#ifdef VAR_USE_EFACT_GAL
    case SYS_GAL:
      sys_fact = EFACT_GAL;
      break;
#endif
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

  /* Frequency factor */
  long double freq_fact = opt->eratio[frq];

  /* IONOOPT IFLC factor */
  long double iflc_fact = (opt->ionoopt == IONOOPT_IFLC) ? 3.0L : 1.0L;

  /* Variance using an additive model */

  /* Base term */
  long double a = opt->err[1];
  long double var = SQR(a);

  /* Satellite elevation term */
  /* The rtkpos and ppp varerr functions do not limit the elevation.
     #define VAR_MIN_EL in rtkpos and ppp, or undefine above, to be consistent. */
#ifdef VAR_MIN_EL
  if (el < VAR_MIN_EL) el = VAR_MIN_EL;
#endif
  long double b = opt->err[2];
  /* The rtkpos and ppp varerr functions scale the elevation variance by 1/sin(el)^2
     #define VAR_SQR_SINEL to be consistent. */
#ifdef VAR_SQR_SINEL
  var += SQR(b / sinl(el));
#else
  var += SQR(b) / sinl(el);
#endif

  /* Add the SNR term, if not zero */
  long double d = opt->err[6];
  if (d > 0.0L) {
    /* #define VAR_SNR_NO_MAX to not have the SNR curve relative to the maximum SNR */
#ifndef VAR_SNR_NO_MAX
    long double snr_max = opt->err[5];
    var += SQR(d) * powl(10, 0.1L * MAX(snr_max - snr_rover, 0));
#else
    var += SQR(d) * powl(10, -0.1L * snr_rover);
#endif
  }

  /* Scale the above terms */
  /* The rtkpos and ppp varerr functions do not scale the rcv std by the system factor,
     #define VAR_NOT_SCALE_RCV_STD_SYS_FACT to be consistent. */
#ifndef VAR_NOT_SCALE_RCV_STD_SYS_FACT
  var *= SQR(freq_fact);
#else
  var *= SQR(sys_fact * freq_fact);
#endif

  /* Add the receiver std estimate */
  long double e = opt->err[7];
  if (e > 0.0L) {
    var += SQR(e) * SQR(0.01L * (1 << (obs->Pstd[frq] + 5))); /* 0.01*2^(n+5) */
  }

  /* Scale the above terms */
#ifndef VAR_NOT_SCALE_RCV_STD_SYS_FACT
  var *= SQR(sys_fact * iflc_fact);
#else
  var *= SQR(iflc_fact);
#endif

  return var;
}

/* Get group delay parameter (m) ---------------------------------------------*/
static long double gettgd(int sat, const nav_t *nav, int type) {
  int prn, sys = satsys(sat, &prn);

  if (sys == SYS_GLO) {
    /* Search for the first non-empty entry */
    int i;
    for (i = 0; i < nav->ng[prn - 1]; i++) {
      if (nav->geph[prn - 1][i].sat == sat) break;
    }
    return (i >= nav->ng[prn - 1]) ? 0.0L : -nav->geph[prn - 1][i].dtaun * CLIGHT;
  } else {
    int i;
    for (i = 0; i < nav->n[sat - 1]; i++) {
      if (nav->eph[sat - 1][i].sat == sat) break;
    }
    return (i >= nav->n[sat - 1]) ? 0.0L : nav->eph[sat - 1][i].tgd[type] * CLIGHT;
  }
}
/* Test SNR mask -------------------------------------------------------------*/
static bool snrmask(const obsd_t *obs, const long double *azel, const prcopt_t *opt) {
  if (testsnr(0, 0, azel[1], obs->SNR[0] * SNR_UNIT, &opt->snrmask)) {
    return false;
  }
  if (opt->ionoopt == IONOOPT_IFLC) {
    int f2 = seliflc(opt->nf, satsys(obs->sat, NULL));
    if (testsnr(0, f2, azel[1], obs->SNR[f2] * SNR_UNIT, &opt->snrmask)) return false;
  }
  return true;
}
/* Iono-free or "pseudo iono-free" pseudorange with code bias correction -----*/
static long double prange(const obsd_t *obs, const nav_t *nav, const prcopt_t *opt,
                          long double *var) {
  int sat = obs->sat;
  int sys = satsys(sat, NULL);
  long double P1 = obs->P[0];
  int f2 = seliflc(opt->nf, satsys(obs->sat, NULL));
  long double P2 = obs->P[f2];
  *var = 0.0L;

  if (P1 == 0.0L || (opt->ionoopt == IONOOPT_IFLC && P2 == 0.0L)) return 0.0L;
  int bias_ix = code2bias_ix(sys, obs->code[0]); /* L1 code bias */
  if (bias_ix > 0) {                             /* 0=ref code */
    P1 += nav->cbias[sat - 1][0][bias_ix - 1];
  }
  /* GPS code biases are L1/L2, Galileo are L1/L5 */
  if (sys == SYS_GAL && f2 == 1) {
    /* Skip code bias, no GAL L2 bias available */
  } else { /* Apply L2 or L5 code bias */
    bias_ix = code2bias_ix(sys, obs->code[f2]);
    if (bias_ix > 0) {                           /* 0=ref code */
      P2 += nav->cbias[sat - 1][1][bias_ix - 1]; /* L2 or L5 code bias */
    }
  }
  if (opt->ionoopt == IONOOPT_IFLC) { /* Dual-frequency */

    if (sys == SYS_GPS || sys == SYS_QZS) { /* L1-L2 or L1-L5 */
      long double gamma = f2 == 1 ? SQR(FREQL1 / FREQL2) : SQR(FREQL1 / FREQL5);
      return (P2 - gamma * P1) / (1.0L - gamma);
    } else if (sys == SYS_GLO) { /* G1-G2 or G1-G3 */
      long double gamma = f2 == 1 ? SQR(FREQ1_GLO / FREQ2_GLO) : SQR(FREQ1_GLO / FREQ3_GLO);
      return (P2 - gamma * P1) / (1.0L - gamma);
    } else if (sys == SYS_GAL) { /* E1-E5b, E1-E5a */
      long double gamma = f2 == 1 ? SQR(FREQL1 / FREQE5b) : SQR(FREQL1 / FREQL5);
      if (f2 == 1 && getseleph(SYS_GAL)) {               /* F/NAV */
        P2 -= gettgd(sat, nav, 0) - gettgd(sat, nav, 1); /* BGD_E5aE5b */
      }
      return (P2 - gamma * P1) / (1.0L - gamma);
    } else if (sys == SYS_CMP) { /* B1-B2 */
      long double gamma = SQR(((obs->code[0] == CODE_L2I) ? FREQ1_CMP : FREQL1) / FREQ2_CMP);
      long double b1;
      if (obs->code[0] == CODE_L2I)
        b1 = gettgd(sat, nav, 0); /* TGD_B1I */
      else if (obs->code[0] == CODE_L1P)
        b1 = gettgd(sat, nav, 2); /* TGD_B1Cp */
      else
        b1 = gettgd(sat, nav, 2) + gettgd(sat, nav, 4); /* TGD_B1Cp+ISC_B1Cd */
      long double b2 = gettgd(sat, nav, 1);             /* TGD_B2I/B2bI (m) */
      return ((P2 - gamma * P1) - (b2 - gamma * b1)) / (1.0L - gamma);
    } else if (sys == SYS_IRN) { /* L5-S */
      long double gamma = SQR(FREQL5 / FREQs);
      return (P2 - gamma * P1) / (1.0L - gamma);
    }
  } else { /* Single-freq (L1/E1/B1) */
    *var = SQR(ERR_CBIAS);

    if (sys == SYS_GPS || sys == SYS_QZS) { /* L1 */
      long double b1 = gettgd(sat, nav, 0); /* TGD (m) */
      return P1 - b1;
    } else if (sys == SYS_GLO) { /* G1 */
      long double gamma = SQR(FREQ1_GLO / FREQ2_GLO);
      long double b1 = gettgd(sat, nav, 0); /* -dtaun (m) */
      return P1 - b1 / (gamma - 1.0L);
    } else if (sys == SYS_GAL) { /* E1 */
      long double b1;
      if (getseleph(SYS_GAL))
        b1 = gettgd(sat, nav, 0); /* BGD_E1E5a */
      else
        b1 = gettgd(sat, nav, 1); /* BGD_E1E5b */
      return P1 - b1;
    } else if (sys == SYS_CMP) { /* B1I/B1Cp/B1Cd */
      long double b1;
      if (obs->code[0] == CODE_L2I)
        b1 = gettgd(sat, nav, 0); /* TGD_B1I */
      else if (obs->code[0] == CODE_L1P)
        b1 = gettgd(sat, nav, 2); /* TGD_B1Cp */
      else
        b1 = gettgd(sat, nav, 2) + gettgd(sat, nav, 4); /* TGD_B1Cp+ISC_B1Cd */
      return P1 - b1;
    } else if (sys == SYS_IRN) { /* L5 */
      long double gamma = SQR(FREQs / FREQL5);
      long double b1 = gettgd(sat, nav, 0); /* TGD (m) */
      return P1 - gamma * b1;
    }
  }
  return P1;
}
/* Ionospheric correction ------------------------------------------------------
 * Compute ionospheric correction
 * Args   : gtime_t time     I   time
 *          nav_t  *nav      I   navigation data
 *          int    sat       I   satellite number
 *          long double *pos      I   receiver position {lat,lon,h} (rad|m)
 *          long double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
 *          long double *ion      O   ionospheric delay (L1) (m)
 *          long double *var      O   ionospheric delay (L1) variance (m^2)
 * Return : status(true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool ionocorr(gtime_t time, const nav_t *nav, int sat, const long double *pos,
                     const long double *azel, int ionoopt, long double *ion, long double *var) {
  int err = 0;

  char tstr[40];
  trace(4, "ionocorr: time=%s opt=%d sat=%2d pos=%.3Lf %.3Lf azel=%.3Lf %.3Lf\n",
        time2str(time, tstr, 3), ionoopt, sat, pos[0] * R2D, pos[1] * R2D, azel[0] * R2D,
        azel[1] * R2D);

  /* SBAS ionosphere model */
  if (ionoopt == IONOOPT_SBAS) {
    if (sbsioncorr(time, nav, pos, azel, ion, var)) return false;
    err = 1;
  }
  /* IONEX TEC model */
  if (ionoopt == IONOOPT_TEC) {
    if (iontec(time, nav, pos, azel, 1, ion, var)) return true;
    err = 1;
  }
  /* QZSS broadcast ionosphere model */
  if (ionoopt == IONOOPT_QZS && norm(nav->ion_qzs, 8) > 0.0L) {
    *ion = ionmodel(time, nav->ion_qzs, pos, azel);
    *var = SQR(*ion * ERR_BRDCI);
    return true;
  }
  /* GPS broadcast ionosphere model */
  if (ionoopt == IONOOPT_BRDC || err == 1) {
    *ion = ionmodel(time, nav->ion_gps, pos, azel);
    *var = SQR(*ion * ERR_BRDCI);
    return true;
  }
  *ion = 0.0L;
  *var = ionoopt == IONOOPT_OFF ? SQR(ERR_ION) : 0.0L;
  return true;
}
/* Tropospheric correction -----------------------------------------------------
 * Compute tropospheric correction
 * Args   : gtime_t time     I   time
 *          nav_t  *nav      I   navigation data
 *          long double *pos      I   receiver position {lat,lon,h} (rad|m)
 *          long double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          int    tropopt   I   tropospheric correction option (TROPOPT_???)
 *          long double *trp      O   tropospheric delay (m)
 *          long double *var      O   tropospheric delay variance (m^2)
 * Return : status(true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool tropcorr(gtime_t time, const nav_t *nav, const long double *pos,
                     const long double *azel, int tropopt, long double *trp, long double *var) {
  char tstr[40];
  trace(4, "tropcorr: time=%s opt=%d pos=%.3Lf %.3Lf azel=%.3Lf %.3Lf\n", time2str(time, tstr, 3),
        tropopt, pos[0] * R2D, pos[1] * R2D, azel[0] * R2D, azel[1] * R2D);

  /* Saastamoinen model */
  if (tropopt == TROPOPT_SAAS || tropopt == TROPOPT_EST || tropopt == TROPOPT_ESTG) {
    *trp = tropmodel(time, pos, azel, REL_HUMI);
    *var = SQR(ERR_SAAS / (sinl(azel[1]) + 0.1L));
    return true;
  }
  /* SBAS (MOPS) troposphere model */
  if (tropopt == TROPOPT_SBAS) {
    *trp = sbstropcorr(time, pos, azel, var);
    return true;
  }
  /* No correction */
  *trp = 0.0L;
  *var = tropopt == TROPOPT_OFF ? SQR(ERR_TROP) : 0.0L;
  return true;
}
/* Pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, const obsd_t *obs, int n, const long double *rs,
                   const long double *dts, const long double *vare, const int *svh,
                   const nav_t *nav, const long double *x, const prcopt_t *opt, const ssat_t *ssat,
                   long double *v, long double *H, long double *var, long double *azel, int *vsat,
                   long double *resp, int *ns) {
  long double rr[3];
  for (int i = 0; i < 3; i++) rr[i] = x[i];
  long double dtr = x[3];

  long double pos[3];
  ecef2pos(rr, pos);
  trace(3, "rescode: rr=%.3Lf %.3Lf %.3Lf\n", rr[0], rr[1], rr[2]);

  int mask[NX - 3] = {0};
  long double snr_max = opt->err[5], snr_rover;
  int nv = 0;
  *ns = 0;
  for (int i = 0; i < n && i < MAXOBS; i++) {
    vsat[i] = 0;
    azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0L;
    gtime_t time = obs[i].time;
    int sat = obs[i].sat;
    int sys = satsys(sat, NULL);
    if (!sys) continue;

    /* Reject duplicated observation data */
    if (i < n - 1 && i < MAXOBS - 1 && sat == obs[i + 1].sat) {
      char tstr[40];
      trace(2, "duplicated obs data %s sat=%d\n", time2str(time, tstr, 3), sat);
      i++;
      continue;
    }
    /* Excluded satellite? */
    if (satexclude(sat, vare[i], svh[i], opt)) continue;

    /* Geometric distance and elevation mask*/
    long double e[3];
    long double r = geodist(rs + i * 6, rr, e);
    if (r <= 0.0L) continue;
    if (satazel(pos, e, azel + i * 2) < opt->elmin) continue;

    long double dion = 0.0L, vion = 0.0L, dtrp = 0.0L, vtrp = 0.0L;
    if (iter > 0) {
      /* Test SNR mask */
      if (!snrmask(obs + i, azel + i * 2, opt)) continue;

      /* Ionospheric correction */
      if (!ionocorr(time, nav, sat, pos, azel + i * 2, opt->ionoopt, &dion, &vion)) {
        continue;
      }
      long double freq = sat2freq(sat, obs[i].code[0], nav);
      if (freq == 0.0L) continue;
      /* Convert from FREQL1 to freq */
      dion *= SQR(FREQL1 / freq);
      vion *= SQR(SQR(FREQL1 / freq));

      /* Tropospheric correction */
      if (!tropcorr(time, nav, pos, azel + i * 2, opt->tropopt, &dtrp, &vtrp)) {
        continue;
      }
    }
    /* Pseudorange with code bias correction */
    long double vmeas;
    long double P = prange(obs + i, nav, opt, &vmeas);
    if (P == 0.0L) continue;

    /* Pseudorange residual */
    v[nv] = P - (r + dtr - CLIGHT * dts[i * 2] + dion + dtrp);
    trace(4, "sat=%d: v=%.3Lf P=%.3Lf r=%.3Lf dtr=%.6LF dts=%.6LF dion=%.3Lf dtrp=%.3Lf\n", sat,
          v[nv], P, r, dtr, dts[i * 2], dion, dtrp);

    /* Design matrix */
    for (int j = 0; j < NX; j++) {
      H[j + nv * NX] = j < 3 ? -e[j] : (j == 3 ? 1.0L : 0.0L);
    }
    /* Time system offset and receiver bias correction */
    if (sys == SYS_GLO) {
      v[nv] -= x[4];
      H[4 + nv * NX] = 1.0L;
      mask[1] = 1;
    } else if (sys == SYS_GAL) {
      v[nv] -= x[5];
      H[5 + nv * NX] = 1.0L;
      mask[2] = 1;
    } else if (sys == SYS_CMP) {
      v[nv] -= x[6];
      H[6 + nv * NX] = 1.0L;
      mask[3] = 1;
    } else if (sys == SYS_IRN) {
      v[nv] -= x[7];
      H[7 + nv * NX] = 1.0L;
      mask[4] = 1;
    }
#ifdef QZSDT
    else if (sys == SYS_QZS) {
      v[nv] -= x[8];
      H[8 + nv * NX] = 1.0L;
      mask[5] = 1;
    }
#endif
    else
      mask[0] = 1;

    vsat[i] = 1;
    resp[i] = v[nv];
    (*ns)++;

    /* Variance of pseudorange error */
    var[nv] = vare[i] + vmeas + vion + vtrp;
    snr_rover = ssat ? SNR_UNIT * ssat[sat - 1].snr_rover[0] : snr_max;
    var[nv++] += varerr(sat, sys, azel[1 + i * 2], snr_rover, opt, &obs[i]);
    trace(4, "sat=%2d azel=%5.1LF %4.1LF res=%7.3Lf sig=%5.3Lf\n", obs[i].sat, azel[i * 2] * R2D,
          azel[1 + i * 2] * R2D, resp[i], sqrtl(var[nv - 1]));
  }
  /* Constraint to avoid rank-deficient */
  for (int i = 0; i < NX - 3; i++) {
    if (mask[i]) continue;
    v[nv] = 0.0L;
    for (int j = 0; j < NX; j++) H[j + nv * NX] = j == i + 3 ? 1.0L : 0.0L;
    var[nv++] = 0.01L;
  }
  return nv;
}
/* Validate solution ---------------------------------------------------------*/
static bool valsol(const long double *azel, const int *vsat, int n, const prcopt_t *opt,
                   const long double *v, int nv, int nx, char *msg, size_t msize) {
  trace(3, "valsol  : n=%d nv=%d\n", n, nv);

  /* Chi-square validation of residuals */
  long double vv = dot(v, v, nv);
  if (nv > nx && vv > chisqr[nv - nx - 1]) {
    rtkcatprintf(msg, msize, "%sWarning: large chi-square error nv=%d vv=%.1LF cs=%.1LF",
                 strlen(msg) > 0 ? "; " : "", nv, vv, chisqr[nv - nx - 1]);
    /* return 0; */ /* Threshold too strict for all use cases, report error but continue on */
  }
  /* Large GDOP check */
  int ns = 0;
  long double azels[MAXOBS * 2];
  for (int i = 0; i < n; i++) {
    if (!vsat[i]) continue;
    azels[ns * 2] = azel[i * 2];
    azels[1 + ns * 2] = azel[1 + i * 2];
    ns++;
  }
  long double dop[4];
  dops(ns, azels, opt->elmin, dop);
  if (dop[0] <= 0.0L || dop[0] > MAX_GDOP) {
    rtkcatprintf(msg, msize, "%sgdop error nv=%d gdop=%.1LF", strlen(msg) > 0 ? "; " : "", nv,
                 dop[0]);
    return false;
  }
  return true;
}
/* Estimate receiver position ------------------------------------------------*/
static bool estpos(const obsd_t *obs, int n, const long double *rs, const long double *dts,
                   const long double *vare, const int *svh, const nav_t *nav, const prcopt_t *opt,
                   const ssat_t *ssat, sol_t *sol, long double *azel, int *vsat, long double *resp,
                   char *msg, size_t msize) {
  trace(3, "estpos  : n=%d\n", n);

  long double *v = mat(n + NX - 3, 1);
  long double *H = mat(NX, n + NX - 3);
  long double *var = mat(n + NX - 3, 1);

  long double x[NX] = {0};
  for (int i = 0; i < 3; i++) x[i] = sol->rr[i];

  int i = 0;
  for (; i < MAXITR; i++) {
    /* Pseudorange residuals (m) */
    int ns;
    int nv =
        rescode(i, obs, n, rs, dts, vare, svh, nav, x, opt, ssat, v, H, var, azel, vsat, resp, &ns);

    if (nv < NX) {
      rtkcatprintf(msg, msize, "%slack of valid sats ns=%d", strlen(msg) > 0 ? "; " : "", nv);
      break;
    }
    /* Weight by variance (lsq uses sqrt of weight */
    for (int j = 0; j < nv; j++) {
      long double sig = sqrtl(var[j]);
      v[j] /= sig;
      for (int k = 0; k < NX; k++) H[k + j * NX] /= sig;
    }
    /* Least square estimation */
    long double dx[NX], Q[NX * NX];
    int info = lsq(H, v, NX, nv, dx, Q);
    if (info) {
      rtkcatprintf(msg, msize, "%slsq error info=%d", strlen(msg) > 0 ? "; " : "", info);
      break;
    }
    for (int j = 0; j < NX; j++) {
      x[j] += dx[j];
    }
    if (norm(dx, NX) < 1E-4L) {
      sol->type = 0;
      sol->time = timeadd(obs[0].time, -x[3] / CLIGHT);
      sol->dtr[0] = x[3] / CLIGHT; /* Receiver clock bias (s) */
      sol->dtr[1] = x[4] / CLIGHT; /* GLO-GPS time offset (s) */
      sol->dtr[2] = x[5] / CLIGHT; /* GAL-GPS time offset (s) */
      sol->dtr[3] = x[6] / CLIGHT; /* BDS-GPS time offset (s) */
      sol->dtr[4] = x[7] / CLIGHT; /* IRN-GPS time offset (s) */
#ifdef QZSDT
      sol->dtr[5] = x[8] / CLIGHT; /* QZS-GPS time offset (s) */
#endif
      for (int j = 0; j < 6; j++) sol->rr[j] = j < 3 ? x[j] : 0.0L;
      for (int j = 0; j < 3; j++) sol->qr[j] = Q[j + j * NX];
      sol->qr[3] = Q[1];      /* Cov xy */
      sol->qr[4] = Q[2 + NX]; /* Cov yz */
      sol->qr[5] = Q[2];      /* Cov zx */
      sol->ns = (uint8_t)ns;
      sol->age = sol->ratio = 0.0L;

      /* Validate solution */
      bool stat = valsol(azel, vsat, n, opt, v, nv, NX, msg, msize);
      if (stat) {
        sol->stat = opt->sateph == EPHOPT_SBAS ? SOLQ_SBAS : SOLQ_SINGLE;
      }
      free(v);
      free(H);
      free(var);
      return stat;
    }
  }
  if (i >= MAXITR) {
    rtkcatprintf(msg, msize, "%siteration divergent i=%d", strlen(msg) > 0 ? "; " : "", i);
  }

  free(v);
  free(H);
  free(var);
  return false;
}
/* RAIM FDE (failure detection and exclusion) --------------------------------*/
static bool raim_fde(const obsd_t *obs, int n, const long double *rs, const long double *dts,
                     const long double *vare, const int *svh, const nav_t *nav, const prcopt_t *opt,
                     const ssat_t *ssat, sol_t *sol, long double *azel, int *vsat,
                     long double *resp, char *msg, size_t msize) {
  char tstr[40];
  trace(3, "raim_fde: %s n=%2d\n", time2str(obs[0].time, tstr, 0), n);

  obsd_t *obs_e = (obsd_t *)malloc(sizeof(obsd_t) * n);
  if (!obs_e) return false;
  long double *rs_e = mat(6, n);
  long double *dts_e = mat(2, n);
  long double *vare_e = mat(1, n);
  long double *azel_e = zeros(2, n);
  int *svh_e = imat(1, n);
  int *vsat_e = imat(1, n);
  long double *resp_e = mat(1, n);

  long double rms = 100.0L;
  int sat = 0;
  bool stat = 0;
  for (int i = 0; i < n; i++) {
    /* Satellite exclusion */
    for (int j = 0, k = 0; j < n; j++) {
      if (j == i) continue;
      obs_e[k] = obs[j];
      matcpy(rs_e + 6 * k, rs + 6 * j, 6, 1);
      matcpy(dts_e + 2 * k, dts + 2 * j, 2, 1);
      vare_e[k] = vare[j];
      svh_e[k++] = svh[j];
    }
    char msg_e[128];
    msg_e[0] = '\0';
    /* Estimate receiver position without a satellite */
    sol_t sol_e = {{0}};
    if (!estpos(obs_e, n - 1, rs_e, dts_e, vare_e, svh_e, nav, opt, ssat, &sol_e, azel_e, vsat_e,
                resp_e, msg_e, sizeof(msg_e))) {
      trace(3, "raim_fde: exsat=%2d (%s)\n", obs[i].sat, msg_e);
      continue;
    }
    int nvsat = 0;
    long double rms_e = 0.0L;
    for (int j = 0; j < n - 1; j++) {
      if (!vsat_e[j]) continue;
      rms_e += SQR(resp_e[j]);
      nvsat++;
    }
    if (nvsat < 5) {
      trace(3, "raim_fde: exsat=%2d lack of satellites nvsat=%2d\n", obs[i].sat, nvsat);
      continue;
    }
    rms_e = sqrtl(rms_e / nvsat);

    trace(3, "raim_fde: exsat=%2d rms=%8.3Lf\n", obs[i].sat, rms_e);

    if (rms_e > rms) continue;

    /* Save result */
    for (int j = 0, k = 0; j < n; j++) {
      if (j == i) continue;
      matcpy(azel + 2 * j, azel_e + 2 * k, 2, 1);
      vsat[j] = vsat_e[k];
      resp[j] = resp_e[k++];
    }
    stat = 1;
    sol_e.eventime = sol->eventime;
    *sol = sol_e;
    sat = obs[i].sat;
    rms = rms_e;
    vsat[i] = 0;
    rtkcatprintf(msg, msize, "%s%s", strlen(msg) > 0 ? "; " : "", msg_e);
  }
#ifdef TRACE
  if (stat) {
    time2str(obs[0].time, tstr, 2);
    char name[8];
    satno2id(sat, name);
    trace(2, "%s: %s excluded by raim\n", tstr + 11, name);
  }
#endif
  free(obs_e);
  free(rs_e);
  free(dts_e);
  free(vare_e);
  free(azel_e);
  free(svh_e);
  free(vsat_e);
  free(resp_e);
  return stat;
}
/* Range rate residuals ------------------------------------------------------*/
static int resdop(const obsd_t *obs, int n, const long double *rs, const long double *dts,
                  const nav_t *nav, const long double *rr, const long double *x,
                  const long double *azel, const int *vsat, long double err, long double *v,
                  long double *H) {
  trace(3, "resdop  : n=%d\n", n);

  long double pos[3];
  ecef2pos(rr, pos);
  long double E[9];
  xyz2enu(pos, E);

  int nv = 0;
  for (int i = 0; i < n && i < MAXOBS; i++) {
    long double freq = sat2freq(obs[i].sat, obs[i].code[0], nav);

    if (obs[i].D[0] == 0.0L || freq == 0.0L || !vsat[i] || norm(rs + 3 + i * 6, 3) <= 0.0L) {
      continue;
    }
    /* LOS (line-of-sight) vector in ECEF */
    long double cosel = cosl(azel[1 + i * 2]);
    long double a[3];
    a[0] = sinl(azel[i * 2]) * cosel;
    a[1] = cosl(azel[i * 2]) * cosel;
    a[2] = sinl(azel[1 + i * 2]);
    long double e[3];
    matmul("TN", 3, 1, 3, E, a, e);

    /* Satellite velocity relative to receiver in ECEF */
    long double vs[3];
    for (int j = 0; j < 3; j++) {
      vs[j] = rs[j + 3 + i * 6] - x[j];
    }
    /* Range rate with earth rotation correction */
    long double rate = dot3(vs, e) + OMGE / CLIGHT *
                                         (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] -
                                          rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);

    /* Std of range rate error (m/s) */
    long double sig = (err <= 0.0L) ? 1.0L : err * CLIGHT / freq;

    /* Range rate residual (m/s) */
    v[nv] = (-obs[i].D[0] * CLIGHT / freq - (rate + x[3] - CLIGHT * dts[1 + i * 2])) / sig;

    /* Design matrix */
    for (int j = 0; j < 4; j++) {
      H[j + nv * 4] = ((j < 3) ? -e[j] : 1.0L) / sig;
    }
    nv++;
  }
  return nv;
}
/* Estimate receiver velocity ------------------------------------------------*/
static void estvel(const obsd_t *obs, int n, const long double *rs, const long double *dts,
                   const nav_t *nav, const prcopt_t *opt, sol_t *sol, const long double *azel,
                   const int *vsat) {
  long double err = opt->err[4]; /* Doppler error (Hz) */

  long double *v = mat(n, 1);
  long double *H = mat(4, n);

  long double x[4] = {0};

  for (int i = 0; i < MAXITR; i++) {
    /* Range rate residuals (m/s) */
    int nv = resdop(obs, n, rs, dts, nav, sol->rr, x, azel, vsat, err, v, H);
    if (nv < 4) break;

    /* Least square estimation */
    long double dx[4], Q[16];
    if (lsq(H, v, 4, nv, dx, Q)) break;

    for (int j = 0; j < 4; j++) x[j] += dx[j];

    if (norm(dx, 4) < 1E-6L) {
      trace(3, "estvel : vx=%.3Lf vy=%.3Lf vz=%.3Lf, n=%d\n", x[0], x[1], x[2], n);
      matcpy(sol->rr + 3, x, 3, 1);
      sol->qv[0] = Q[0];  /* xx */
      sol->qv[1] = Q[5];  /* yy */
      sol->qv[2] = Q[10]; /* zz */
      sol->qv[3] = Q[1];  /* xy */
      sol->qv[4] = Q[6];  /* yz */
      sol->qv[5] = Q[2];  /* zx */
      break;
    }
  }
  free(v);
  free(H);
}
/* Single-point positioning ----------------------------------------------------
 * Compute receiver position, velocity, clock bias by single-point positioning
 * With pseudorange and doppler observables
 * Args   : obsd_t *obs      I   observation data
 *          int    n         I   number of observation data
 *          nav_t  *nav      I   navigation data
 *          prcopt_t *opt    I   processing options
 *          sol_t  *sol      IO  solution
 *          long double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
 *          ssat_t *ssat     IO  satellite status              (NULL: no output)
 *          char   *msg      O   error message for error exit
 *          size_t msize     I   error message buffer size
 * Return : status(true:ok,false:error)
 * Note   : Message output is appended to msg which must be nul terminated.
 *----------------------------------------------------------------------------*/
extern bool pntpos(const obsd_t *obs, int n, const nav_t *nav, const prcopt_t *opt, sol_t *sol,
                   long double *azel, ssat_t *ssat, char *msg, size_t msize) {
  char tstr[40];
  trace(3, "pntpos  : tobs=%s n=%d\n", time2str(obs[0].time, tstr, 3), n);

  sol->stat = SOLQ_NONE;

  if (n <= 0) {
    rtkstrcat(msg, msize, "no observation data");
    return false;
  }
  sol->time = obs[0].time;
  sol->eventime = obs[0].eventime;

  long double *rs = mat(6, n);
  long double *dts = mat(2, n);
  long double *var = mat(1, n);
  long double *azel_ = zeros(2, n);
  long double *resp = mat(1, n);

  if (ssat) {
    for (int i = 0; i < MAXSAT; i++) {
      ssat[i].snr_rover[0] = 0;
      ssat[i].snr_base[0] = 0;
    }
    for (int i = 0; i < n; i++) ssat[obs[i].sat - 1].snr_rover[0] = obs[i].SNR[0];
  }

  prcopt_t opt_ = *opt;
  if (opt_.mode != PMODE_SINGLE) { /* For precise positioning */
    opt_.ionoopt = IONOOPT_BRDC;
    opt_.tropopt = TROPOPT_SAAS;
  }
  /* Satellite positions, velocities and clocks */
  int svh[MAXOBS];
  satposs(sol->time, obs, n, nav, opt_.sateph, rs, dts, var, svh);

  /* Estimate receiver position and time with pseudorange */
  int vsat[MAXOBS] = {0};
  bool stat =
      estpos(obs, n, rs, dts, var, svh, nav, &opt_, ssat, sol, azel_, vsat, resp, msg, msize);

  /* RAIM FDE */
  if (!stat && n >= 6 && opt->posopt[4]) {
    stat =
        raim_fde(obs, n, rs, dts, var, svh, nav, &opt_, ssat, sol, azel_, vsat, resp, msg, msize);
  }
  /* Estimate receiver velocity with Doppler */
  if (stat) {
    estvel(obs, n, rs, dts, nav, &opt_, sol, azel_, vsat);
  }
  if (azel) {
    for (int i = 0; i < n * 2; i++) azel[i] = azel_[i];
  }
  if (ssat) {
    for (int i = 0; i < MAXSAT; i++) {
      ssat[i].vs = 0;
      ssat[i].azel[0] = ssat[i].azel[1] = 0.0L;
      ssat[i].resp[0] = ssat[i].resc[0] = 0.0L;
    }
    for (int i = 0; i < n; i++) {
      ssat[obs[i].sat - 1].azel[0] = azel_[i * 2];
      ssat[obs[i].sat - 1].azel[1] = azel_[1 + i * 2];
      if (!vsat[i]) continue;
      ssat[obs[i].sat - 1].vs = 1;
      ssat[obs[i].sat - 1].resp[0] = resp[i];
    }
  }
  free(rs);
  free(dts);
  free(var);
  free(azel_);
  free(resp);
  return stat;
}
