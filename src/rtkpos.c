/*------------------------------------------------------------------------------
 * rtkpos.c : Precise positioning
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * History : 2007/01/12 1.0  new
 *           2007/03/13 1.1  add slip detection by LLI flag
 *           2007/04/18 1.2  add antenna pcv correction
 *                           change rtkpos argin
 *           2008/07/18 1.3  refactored
 *           2009/01/02 1.4  modify rtk positioning api
 *           2009/03/09 1.5  support GLONASS, gallileo and QZS
 *           2009/08/27 1.6  fix bug on numerical exception
 *           2009/09/03 1.7  add check of valid satellite number
 *                           add check time sync for moving-base
 *           2009/11/23 1.8  add api rtkopenstat(),rtkclosestat()
 *                           add receiver h/w bias estimation
 *                           add solution status output
 *           2010/04/04 1.9  support ppp-kinematic and ppp-static modes
 *                           support earth tide correction
 *                           changed api:
 *                               rtkpos()
 *           2010/09/07 1.10 add elevation mask to hold ambiguity
 *           2012/02/01 1.11 add extended receiver error model
 *                           add GLONASS interchannel bias correction
 *                           add slip detection by L1-L5 gf jump
 *                           output snr of rover receiver in residuals
 *           2013/03/10 1.12 add otl and pole tides corrections
 *           2014/05/26 1.13 support BeiDou and Galileo
 *                           add output of gal-gps and bds-gps time offset
 *           2014/05/28 1.14 fix bug on memory exception with many sys and freq
 *           2014/08/26 1.15 add function to swap sol-stat file with keywords
 *           2014/10/21 1.16 fix bug on BeiDou amb-res with pos2-bdsarmode=0
 *           2014/11/08 1.17 fix bug on ar-degradation by unhealthy satellites
 *           2015/03/23 1.18 residuals referenced to reference satellite
 *           2015/05/20 1.19 no output solution status file with Q=0
 *           2015/07/22 1.20 fix bug on base station position setting
 *           2016/07/30 1.21 suppress single solution if !prcopt.outsingle
 *                           fix bug on slip detection of backward filter
 *           2016/08/20 1.22 fix bug on ddres() function
 *           2018/10/10 1.13 support api change of satexclude()
 *           2018/12/15 1.14 disable ambiguity resolution for gps-qzss
 *           2019/08/19 1.15 fix bug on return value of resamb_LAMBDA()
 *           2020/11/30 1.16 support of NavIC/IRNSS in API rtkpos()
 *                           add detecting cycle slips by L1-Lx GF phase jump
 *                           delete GLONASS IFB correction in ddres()
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include <stdarg.h>

#include "rtklib.h"

/* Algorithm configuration -------------------------------------------------- */
#define STD_PREC_VAR_THRESH 0 /* Pos variance threshold to skip standard precision */
                              /* Solution: 0   = run every epoch, */
                              /*           0.5 = skip except for first*/

/* Constants/macros ----------------------------------------------------------*/

#define SQR(x) ((x) * (x))
#define SQRTL(x) ((x) <= 0.0L || (x) != (x) ? 0.0L : sqrtl(x))
#define MIN(x, y) ((x) <= (y) ? (x) : (y))
#define MAX(x, y) ((x) >= (y) ? (x) : (y))
#define ROUND(x) (int)floorl((x) + 0.5L)

#define VAR_POS SQR(30.0L)     /* Initial variance of receiver pos (m^2) */
#define VAR_POS_FIX SQR(1e-4L) /* Initial variance of fixed receiver pos (m^2) */
#define VAR_VEL SQR(10.0L)     /* Initial variance of receiver vel ((m/s)^2) */
#define VAR_ACC SQR(10.0L)     /* Initial variance of receiver acc ((m/ss)^2) */
#define VAR_GRA SQR(0.001L)    /* Initial variance of gradient (m^2) */
#define INIT_ZWD 0.15L         /* Initial zwd (m) */

#define GAP_RESION 120 /* Gap to reset ionosphere parameters (epochs) */

#define TTOL_MOVEB (1.0L + 2L * DTTOL)
/* Time sync tolerance for moving-baseline (s) */

/* Number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */
#define NF(opt) ((opt)->ionoopt == IONOOPT_IFLC ? 1 : (opt)->nf)
#define NP(opt) ((opt)->dynamics == 0 ? 3 : 9)
#define NI(opt) ((opt)->ionoopt != IONOOPT_EST ? 0 : MAXSAT)
#define NT(opt) ((opt)->tropopt < TROPOPT_EST ? 0 : ((opt)->tropopt < TROPOPT_ESTG ? 2 : 6))
#define NL(opt) ((opt)->glomodear != GLO_ARMODE_AUTOCAL ? 0 : NFREQGLO)
#define NB(opt) ((opt)->mode <= PMODE_DGPS ? 0 : MAXSAT * NF(opt))
#define NR(opt) (NP(opt) + NI(opt) + NT(opt) + NL(opt))
#define NX(opt) (NR(opt) + NB(opt))

/* State variable index */
#define II(s, opt) (NP(opt) + (s)-1)                       /* Ionos (s:satellite no) */
#define IT(r, opt) (NP(opt) + NI(opt) + NT(opt) / 2 * (r)) /* Tropos (r:0=rov,1:ref) */
#define IL(f, opt) (NP(opt) + NI(opt) + NT(opt) + (f))     /* Receiver h/w bias */
#define IB(s, f, opt) (NR(opt) + MAXSAT * (f) + (s)-1)     /* Phase bias (s:satno,f:freq) */

/* Poly coeffs used to adjust AR ratio by # of sats, derived by fitting to  example from:
   https://www.tudelft.nl/citg/over-faculteit/afdelingen/geoscience-remote-sensing/research/lambda/lambda
 */
static long double ar_poly_coeffs[3][5] = {
    {-1.94058448e-01L, -7.79023476e+00L, 1.24231120e+02L, -4.03126050e+02L, 3.50413202e+02L},
    {6.42237302e-01L, -8.39813962e+00L, 2.92107285e+01L, -2.37577308e+01L, -1.14307128e+00L},
    {-2.22600390e-02L, 3.23169103e-01L, -1.39837429e+00L, 2.19282996e+00L, -5.34583971e-02L}};

/* Global variables ----------------------------------------------------------*/
static int statlevel = 0;           /* Rtk status output level (0:off) */
static FILE *fp_stat = NULL;        /* Rtk status file pointer */
static char file_stat[FNSIZE] = ""; /* Rtk status file original path */
static gtime_t time_stat = {0};     /* Rtk status file time */

/* Open solution status file ---------------------------------------------------
 * Open solution status file and set output level
 * Args   : char     *file   I   rtk status file
 *          int      level   I   rtk status level (0: off)
 * Return : status (true:ok,false:error)
 * Notes  : file can constain time keywords (%Y,%y,%m...) defined in reppath().
 *          The time to replace keywords is based on UTC of CPU time.
 * Output : solution status file record format
 *
 *   $POS,week,tow,stat,posx,posy,posz,posxf,posyf,poszf
 *          week/tow : GPS week no/time of week (s)
 *          stat     : solution status
 *          posx/posy/posz    : position x/y/z ECEF (m) float
 *          posxf/posyf/poszf : position x/y/z ECEF (m) fixed
 *
 *   $VELACC,week,tow,stat,vele,veln,velu,acce,accn,accu,velef,velnf,veluf,accef,accnf,accuf
 *          week/tow : GPS week no/time of week (s)
 *          stat     : solution status
 *          vele/veln/velu    : velocity e/n/u (m/s) float
 *          acce/accn/accu    : acceleration e/n/u (m/s^2) float
 *          velef/velnf/veluf : velocity e/n/u (m/s) fixed
 *          accef/accnf/accuf : acceleration e/n/u (m/s^2) fixed
 *
 *   $CLK,week,tow,stat,clk1,clk2,clk3,clk4,clk5,clk6
 *          week/tow : GPS week no/time of week (s)
 *          stat     : solution status
 *          clk1     : receiver clock bias GPS (ns)
 *          clk2     : receiver clock bias GLO-GPS (ns)
 *          clk3     : receiver clock bias GAL-GPS (ns)
 *          clk4     : receiver clock bias BDS-GPS (ns)
 *          clk5     : receiver clock bias IRN-GPS (ns)
 *          clk6     : receiver clock bias QZS-GPS (ns)
 *
 *   $ION,week,tow,stat,sat,az,el,ion,ion-fixed
 *          week/tow : GPS week no/time of week (s)
 *          stat     : solution status
 *          sat      : satellite id
 *          az/el    : azimuth/elevation angle(deg)
 *          ion      : vertical ionospheric delay L1 (m) float
 *          ion-fixed: vertical ionospheric delay L1 (m) fixed
 *
 *   $TROP,week,tow,stat,rcv,ztd,ztdf
 *          week/tow : GPS week no/time of week (s)
 *          stat     : solution status
 *          rcv      : receiver (1:rover,2:base station)
 *          ztd      : zenith total delay (m) float
 *          ztdf     : zenith total delay (m) fixed
 *
 *   $HWBIAS,week,tow,stat,frq,bias,biasf
 *          week/tow : GPS week no/time of week (s)
 *          stat     : solution status
 *          frq      : frequency (1:L1,2:L2,...)
 *          bias     : h/w bias coefficient (m/MHz) float
 *          biasf    : h/w bias coefficient (m/MHz) fixed
 *
 *   $SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc,icbias,bias,bias_var,lambda
 *          week/tow : GPS week no/time of week (s)
 *          sat/frq  : satellite id/frequency (1:L1,2:L2,...)
 *          az/el    : azimuth/elevation angle (deg)
 *          resp     : pseudorange residual (m)
 *          resc     : carrier-phase residual (m)
 *          vsat     : valid data flag (0:invalid,1:valid)
 *          snr      : signal strength (dbHz)
 *          fix      : ambiguity flag  (0:no data,1:float,2:fixed,3:hold,4:ppp)
 *          slip     : cycle-slip flag (bit1:slip,bit2:parity unknown)
 *          lock     : carrier-lock count
 *          outc     : data outage count
 *          slipc    : cycle-slip count
 *          rejc     : data reject (outlier) count
 *          icbias   : interchannel bias (GLONASS)
 *          bias     : phase bias
 *          bias_var : variance of phase bias
 *          lambda   : wavelength
 *
 *----------------------------------------------------------------------------*/
extern bool rtkopenstat(const char *file, int level) {
  trace(3, "rtkopenstat: file=%s level=%d\n", file, level);

  if (level <= 0) return false;

  gtime_t time = utc2gpst(timeget());
  char path[FNSIZE];
  reppath(file, path, sizeof(path), time, "", "");

  fp_stat = fopen(path, "w");
  if (!fp_stat) {
    trace(1, "rtkopenstat: file open error path=%s\n", path);
    return false;
  }
  rtkstrcpy(file_stat, sizeof(file_stat), file);
  time_stat = time;
  statlevel = level;
  return true;
}
/* Close solution status file --------------------------------------------------
 * Close solution status file
 * Args   : none
 * Return : none
 *----------------------------------------------------------------------------*/
extern void rtkclosestat(void) {
  trace(3, "rtkclosestat:\n");

  if (fp_stat) fclose(fp_stat);
  fp_stat = NULL;
  file_stat[0] = '\0';
  statlevel = 0;
}
/* Write solution status to buffer ---------------------------------------------
 * Note The output is appended to the buffer which must be nul terminated.
 */
extern void rtkoutstat(rtk_t *rtk, int level, char *buff, size_t size) {
  if (level <= 0 || rtk->sol.stat == SOLQ_NONE) {
    return;
  }

  int est = rtk->opt.mode >= PMODE_DGPS;
  int nf = NF(&rtk->opt);
  int nfreq = est ? nf : 1;
  int week;
  long double tow = time2gpst(rtk->sol.time, &week);

  if (rtk->opt.mode >= PMODE_PPP_KINEMA) {
    /* Write ppp solution status to buffer */
    pppoutstat(rtk, buff, size);
  } else {
    /* Receiver position */
    if (est) {
      long double xa[3];
      for (int i = 0; i < 3; i++) xa[i] = i < rtk->na ? rtk->xa[i] : 0.0L;
      rtkcatprintf(buff, size, "$POS,%d,%.3Lf,%d,%.4Lf,%.4Lf,%.4Lf,%.4Lf,%.4Lf,%.4Lf\n", week, tow,
                   rtk->sol.stat, rtk->x[0], rtk->x[1], rtk->x[2], xa[0], xa[1], xa[2]);
    } else {
      rtkcatprintf(buff, size, "$POS,%d,%.3Lf,%d,%.4Lf,%.4Lf,%.4Lf,%.4Lf,%.4Lf,%.4Lf\n", week, tow,
                   rtk->sol.stat, rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], 0.0L, 0.0L, 0.0L);
    }
    /* Receiver velocity and acceleration */
    if (est && rtk->opt.dynamics) {
      long double pos[3];
      ecef2pos(rtk->sol.rr, pos);
      long double vel[3];
      ecef2enu(pos, rtk->x + 3, vel);
      long double acc[3];
      ecef2enu(pos, rtk->x + 6, acc);
      long double vela[3] = {0};
      if (rtk->na >= 6) ecef2enu(pos, rtk->xa + 3, vela);
      long double acca[3] = {0};
      if (rtk->na >= 9) ecef2enu(pos, rtk->xa + 6, acca);
      rtkcatprintf(
          buff, size,
          "$VELACC,%d,%.3Lf,%d,%.4Lf,%.4Lf,%.4Lf,%.5Lf,%.5Lf,%.5Lf,%.4Lf,%.4Lf,%.4Lf,%.5Lf,%"
          ".5Lf,%.5Lf\n",
          week, tow, rtk->sol.stat, vel[0], vel[1], vel[2], acc[0], acc[1], acc[2], vela[0],
          vela[1], vela[2], acca[0], acca[1], acca[2]);
    } else {
      long double pos[3];
      ecef2pos(rtk->sol.rr, pos);
      long double vel[3];
      ecef2enu(pos, rtk->sol.rr + 3, vel);
      rtkcatprintf(
          buff, size,
          "$VELACC,%d,%.3Lf,%d,%.4Lf,%.4Lf,%.4Lf,%.5Lf,%.5Lf,%.5Lf,%.4Lf,%.4Lf,%.4Lf,%.5Lf,%"
          ".5Lf,%.5Lf\n",
          week, tow, rtk->sol.stat, vel[0], vel[1], vel[2], 0.0L, 0.0L, 0.0L, 0.0L, 0.0L, 0.0L,
          0.0L, 0.0L, 0.0L);
    }
    /* Receiver clocks */
    rtkcatprintf(buff, size, "$CLK,%d,%.3Lf,%d,%d,%.3Lf,%.3Lf,%.3Lf,%.3Lf,%.3Lf,%.3Lf\n", week, tow,
                 rtk->sol.stat, 1, rtk->sol.dtr[0] * 1E9L, rtk->sol.dtr[1] * 1E9L,
                 rtk->sol.dtr[2] * 1E9L, rtk->sol.dtr[3] * 1E9L, rtk->sol.dtr[4] * 1E9L,
                 rtk->sol.dtr[5] * 1E9L);

    /* Ionospheric parameters */
    if (est && rtk->opt.ionoopt == IONOOPT_EST) {
      for (int i = 0; i < MAXSAT; i++) {
        ssat_t *ssat = rtk->ssat + i;
        if (!ssat->vs) continue;
        char id[8];
        satno2id(i + 1, id);
        int j = II(i + 1, &rtk->opt);
        long double xa = j < rtk->na ? rtk->xa[j] : 0.0L;
        rtkcatprintf(buff, size, "$ION,%d,%.3Lf,%d,%s,%.1Lf,%.1Lf,%.4Lf,%.4Lf\n", week, tow,
                     rtk->sol.stat, id, ssat->azel[0] * R2D, ssat->azel[1] * R2D, rtk->x[j], xa);
      }
    }
    /* Tropospheric parameters */
    if (est && (rtk->opt.tropopt >= TROPOPT_EST)) {
      for (int i = 0; i < 2; i++) {
        int j = IT(i, &rtk->opt);
        long double xa = j < rtk->na ? rtk->xa[j] : 0.0L;
        rtkcatprintf(buff, size, "$TROP,%d,%.3Lf,%d,%d,%.4Lf,%.4Lf\n", week, tow, rtk->sol.stat,
                     i + 1, rtk->x[j], xa);
      }
    }
    /* Receiver h/w bias */
    if (est && rtk->opt.glomodear == GLO_ARMODE_AUTOCAL) {
      for (int i = 0; i < nfreq; i++) {
        int j = IL(i, &rtk->opt);
        long double xa = j < rtk->na ? rtk->xa[j] : 0.0L;
        rtkcatprintf(buff, size, "$HWBIAS,%d,%.3Lf,%d,%d,%.4Lf,%.4Lf\n", week, tow, rtk->sol.stat,
                     i + 1, rtk->x[j], xa);
      }
    }
  }

  if (level <= 1) return;

  /* Write residuals and status */
  for (int i = 0; i < MAXSAT; i++) {
    ssat_t *ssat = rtk->ssat + i;
    if (!ssat->vs) continue;
    char id[8];
    satno2id(i + 1, id);
    for (int j = 0; j < nfreq; j++) {
      int k = IB(i + 1, j, &rtk->opt);
      rtkcatprintf(
          buff, size,
          "$SAT,%d,%.3Lf,%s,%d,%.1Lf,%.1Lf,%.4Lf,%.4Lf,%d,%.2Lf,%d,%d,%d,%u,%u,%u,%.2Lf,%.6Lf,%"
          ".5Lf\n",
          week, tow, id, j + 1, ssat->azel[0] * R2D, ssat->azel[1] * R2D, ssat->resp[j],
          ssat->resc[j], ssat->vsat[j], ssat->snr_rover[j] * SNR_UNIT, ssat->fix[j],
          ssat->slip[j] & 3, ssat->lock[j], ssat->outc[j], ssat->slipc[j], ssat->rejc[j],
          k < rtk->nx ? rtk->x[k] : 0, k < rtk->nx ? rtk->P[k + k * rtk->nx] : 0, ssat->icbias[j]);
    }
  }

  if (level <= 1) return;

  /* Write residuals and status */
  for (int i = 0; i < MAXSAT; i++) {
    ssat_t *ssat = rtk->ssat + i;
    if (!ssat->vs) continue;
    char id[8];
    satno2id(i + 1, id);
    for (int j = 0; j < nfreq; j++) {
      int k = IB(i + 1, j, &rtk->opt);
      rtkcatprintf(
          buff, size,
          "$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.2f,%d,%d,%d,%u,%u,%u,%.2f,%.6f,%.5lf\n",
          week, tow, id, j + 1, ssat->azel[0] * R2D, ssat->azel[1] * R2D, ssat->resp[j],
          ssat->resc[j], ssat->vsat[j], ssat->snr_rover[j] * SNR_UNIT, ssat->fix[j],
          ssat->slip[j] & 3, ssat->lock[j], ssat->outc[j], ssat->slipc[j], ssat->rejc[j],
          k < rtk->nx ? rtk->x[k] : 0, k < rtk->nx ? rtk->P[k + k * rtk->nx] : 0, ssat->icbias[j]);
    }
  }
}
/* Swap solution status file -------------------------------------------------*/
static void swapsolstat(void) {
  gtime_t time = utc2gpst(timeget());
  if ((int)(time2gpst(time, NULL) / INT_SWAP_STAT) ==
      (int)(time2gpst(time_stat, NULL) / INT_SWAP_STAT)) {
    return;
  }
  time_stat = time;

  char path[FNSIZE];
  if (!reppath(file_stat, path, sizeof(path), time, "", "")) {
    return;
  }
  if (fp_stat) fclose(fp_stat);

  fp_stat = fopen(path, "w");
  if (!fp_stat) {
    trace(2, "swapsolstat: file open error path=%s\n", path);
    return;
  }
  trace(3, "swapsolstat: path=%s\n", path);
}
/* Output solution status ----------------------------------------------------*/
static void outsolstat(rtk_t *rtk, const nav_t *nav) {
  if (statlevel <= 0 || !fp_stat || !rtk->sol.stat) return;

  trace(3, "outsolstat:\n");

  /* Swap solution status file */
  swapsolstat();

  /* Write solution status */
  char buff[MAXSOLMSG + 1];
  buff[0] = '\0';
  rtkoutstat(rtk, statlevel, buff, sizeof(buff));
  fputs(buff, fp_stat);
}
/* Save error message --------------------------------------------------------*/
static void errmsg(rtk_t *rtk, const char *format, ...) {
  char tstr[40];
  time2str(rtk->sol.time, tstr, 2);
  char buff[256];
  rtksnprintf(buff, sizeof(buff), "%s: ", tstr + 11);
  size_t len = strlen(buff);
  va_list ap;
  va_start(ap, format);
  vsnprintf(buff + len, sizeof(buff) - len, format, ap);
  va_end(ap);
  len = strlen(buff);
  len = (int)len < MAXERRMSG - rtk->neb ? len : (size_t)MAXERRMSG - rtk->neb;
  memcpy(rtk->errbuf + rtk->neb, buff, len);
  rtk->neb += len;
  trace(2, "%s", buff);
}
/* Single-differenced observable ---------------------------------------------*/
static long double sdobs(const obsd_t *obs, int i, int j, int k) {
  long double pi = (k < NFREQ) ? obs[i].L[k] : obs[i].P[k - NFREQ];
  long double pj = (k < NFREQ) ? obs[j].L[k] : obs[j].P[k - NFREQ];
  return pi == 0.0L || pj == 0.0L ? 0.0L : pi - pj;
}
/* Single-differenced geometry-free linear combination of phase --------------*/
static long double gfobs(const obsd_t *obs, int i, int j, int k, const nav_t *nav) {
  long double freq1 = sat2freq(obs[i].sat, obs[i].code[0], nav);
  long double freq2 = sat2freq(obs[i].sat, obs[i].code[k], nav);
  long double L1 = sdobs(obs, i, j, 0);
  long double L2 = sdobs(obs, i, j, k);
  if (freq1 == 0.0L || freq2 == 0.0L || L1 == 0.0L || L2 == 0.0L) return 0.0L;
  return L1 * CLIGHT / freq1 - L2 * CLIGHT / freq2;
}

/* Single-differenced measurement error variance -----------------------------*/
static long double varerr(int sat, int sys, long double el, long double snr_rover,
                          long double snr_base, long double bl, long double dt, int f,
                          const prcopt_t *opt, const obsd_t *obs) {
  int nf = NF(opt), frq = f % nf;
  int code = f < nf ? 0 : 1; /* 0=phase, 1=code */

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
    /* The ppp varerr function applies this factor.
       #define VAR_GPS_QZS_L5_FACT to be consistent. */
#ifdef VAR_GPS_QZS_L5_FACT
  /* GPS/QZS L5 error factor */
  if (sys == SYS_GPS || sys == SYS_QZS) {
    if (frq == 2) sys_fact *= EFACT_GPS_L5;
  }
#endif

  /* Code/phase/frequency factor */
  long double code_freq_fact = opt->eratio[frq];
  /* The ppp varerr function implements this guard.
     #define VAR_GUARD_ERATIO to be consistent. */
#ifdef VAR_GUARD_ERATIO
  /* Guard against a configuration eratio being zero, or less */
  if (code_freq_fact <= 0.0L) code_freq_fact = opt->eratio[0];
#endif
  /* Increased variance for pseudoranges */
  if (!code) {
    /* The ppp and pntpos varerr functions use 1.0 rather than the ratio. */
#define VAR_PHASE_FREQ_RATIO
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
  /* The pntpos varerr function scales the elevation variance by 1/sinl(el)
     Undefine VAR_SQR_SINEL to be consistent. */
#define VAR_SQR_SINEL
#ifdef VAR_SQR_SINEL
  var += SQR(b / sinl(el));
#else
  var += SQR(b) / sinl(el);
#endif

  /* Scale the above terms */
  var *= 2.0L;

  /* Add the SNR term, if not zero */
  long double d = opt->err[6];
  if (d > 0.0L) {
    /* #define VAR_SNR_NO_MAX to not have the SNR curve relative to the maximum SNR */
#ifndef VAR_SNR_NO_MAX
    long double snr_max = opt->err[5];
    var += SQR(d) * (powl(10L, 0.1L * MAX(snr_max - snr_rover, 0)) +
                     powl(10L, 0.1L * MAX(snr_max - snr_base, 0)));
#else
    var += SQR(d) * (powl(10L, -0.1L * snr_rover) + powl(10L, -0.1L * snr_base));
#endif
  }

  /* Scale the above terms */
  var *= SQR(sys_fact * code_freq_fact);

  /* Add the receiver std estimate */
  long double e = opt->err[7];
  if (e > 0.0L) {
    if (code)
      var += SQR(e) * SQR(0.01L * (1 << (obs->Pstd[frq] + 5))); /* 0.01*2^(n+5) */
    else
      var += SQR(e) * SQR(obs->Lstd[frq] * 0.004L * 0.2L); /* 0.004 cycles -> m */
  }

  /* Baseline term */
  /* TODO would the baseline contribution be affected by the use of IFLC, if
     not then perhaps move below the scaling by the IFLC factor? */
  long double c = opt->err[3] * bl / 1E4L;
  var += 2.0L * SQR(c);

  /* TODO The upstream code did not scale the clock error by the IFLC
     factor.  The use of IFLC might not affect the random clock drift, so
     perhaps add the clock term after scaling. */

  /* Add the clock term */
  var += SQR(CLIGHT * opt->sclkstab * dt);

  /* Scale the above terms */
  var *= SQR(iflc_fact);

  return var;
}

/* Baseline length -----------------------------------------------------------*/
static long double baseline(const long double *ru, const long double *rb, long double *dr) {
  for (int i = 0; i < 3; i++) dr[i] = ru[i] - rb[i];
  return norm(dr, 3);
}
/* Initialize state and covariance -------------------------------------------*/
static inline void initx(rtk_t *rtk, long double xi, long double var, int i) {
  rtk->x[i] = xi;
  for (int j = 0; j < rtk->nx; j++) rtk->P[i + j * rtk->nx] = 0.0L;
  for (int j = 0; j < rtk->nx; j++) rtk->P[j + i * rtk->nx] = 0.0L;
  rtk->P[i + i * rtk->nx] = var;
}
/* Select common satellites between rover and reference station --------------*/
static int selsat(const obsd_t *obs, const long double *azel, int nu, int nr, const prcopt_t *opt,
                  int *sat, int *iu, int *ir) {
  trace(3, "selsat  : nu=%d nr=%d\n", nu, nr);

  int k = 0;
  for (int i = 0, j = nu; i < nu && j < nu + nr; i++, j++) {
    if (obs[i].sat < obs[j].sat)
      j--;
    else if (obs[i].sat > obs[j].sat)
      i--;
    else if (azel[1 + j * 2] >= opt->elmin) { /* Elevation at base station */
      sat[k] = obs[i].sat;
      iu[k] = i;
      ir[k++] = j;
      trace(4, "(%2d) sat=%3d iu=%2d ir=%2d\n", k - 1, obs[i].sat, i, j);
    }
  }
  return k;
}
/* Temporal update of position/velocity/acceleration -------------------------*/
static void udpos(rtk_t *rtk, long double tt) {
  trace(3, "udpos   : tt=%.3Lf\n", tt);

  /* Fixed mode */
  if (rtk->opt.mode == PMODE_FIXED) {
    for (int i = 0; i < 3; i++) initx(rtk, rtk->opt.ru[i], VAR_POS_FIX, i);
    return;
  }
  /* Initialize position for first epoch */
  if (norm(rtk->x, 3) <= 0.0L) {
    trace(3, "rr_init=");
    tracemat(3, rtk->sol.rr, 1, 6, 15, 6);
    for (int i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
    if (rtk->opt.dynamics) {
      for (int i = 3; i < 6; i++) initx(rtk, rtk->sol.rr[i], VAR_VEL, i);
      for (int i = 6; i < 9; i++) initx(rtk, 1E-6L, VAR_ACC, i);
    }
  }
  /* Static mode */
  if (rtk->opt.mode == PMODE_STATIC || rtk->opt.mode == PMODE_STATIC_START) return;

  /* Kinmatic mode without dynamics */
  if (!rtk->opt.dynamics) {
    for (int i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
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
    /*    TODO:  The b34 code causes issues so use b33 code for now */
    if (i < 9 || (rtk->x[i] != 0.0L && rtk->P[i + i * rtk->nx] > 0.0L)) ix[nx++] = i;
  }
  /* State transition of position/velocity/acceleration */
  long double *F = eye(nx), *P = mat(nx, nx), *FP = mat(nx, nx), *x = mat(nx, 1), *xp = mat(nx, 1);

  for (int i = 0; i < 6; i++) {
    F[i + (i + 3) * nx] = tt;
  }
  /* Include accel terms if filter is converged */
  if (var < rtk->opt.thresar[1]) {
    for (int i = 0; i < 3; i++) {
      F[i + (i + 6) * nx] = (tt >= 0 ? 1 : -1) * SQR(tt) / 2.0L;
    }
  } else
    trace(3, "pos var too high for accel term: %.4Lf\n", var);
  for (int i = 0; i < nx; i++) {
    x[i] = rtk->x[ix[i]];
    for (int j = 0; j < nx; j++) {
      P[i + j * nx] = rtk->P[ix[i] + ix[j] * rtk->nx];
    }
  }
  /* x=F*x, P=F*P*F' */
  matmul("NN", nx, 1, nx, F, x, xp);
  matmul("NN", nx, nx, nx, F, P, FP);
  matmul("NT", nx, nx, nx, FP, F, P);

  for (int i = 0; i < nx; i++) {
    rtk->x[ix[i]] = xp[i];
    for (int j = 0; j < nx; j++) {
      rtk->P[ix[i] + ix[j] * rtk->nx] = P[i + j * nx];
    }
  }
  /* Process noise added to only acceleration  P=P+Q */
  long double Q[9] = {0};
  Q[0] = Q[4] = SQR(rtk->opt.prn[3]) * fabsl(tt);
  Q[8] = SQR(rtk->opt.prn[4]) * fabsl(tt);
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
/* Temporal update of ionospheric parameters ---------------------------------*/
static void udion(rtk_t *rtk, long double tt, long double bl, const int *sat, int ns) {
  trace(3, "udion   : tt=%.3Lf bl=%.0Lf ns=%d\n", tt, bl, ns);

  /* Reset ionospheric delays for sats with long outages */
  for (int i = 1; i <= MAXSAT; i++) {
    int j = II(i, &rtk->opt);
    if (rtk->x[j] != 0.0L && rtk->ssat[i - 1].outc[0] > GAP_RESION &&
        rtk->ssat[i - 1].outc[1] > GAP_RESION)
      rtk->x[j] = 0.0L;
  }
  for (int i = 0; i < ns; i++) {
    int j = II(sat[i], &rtk->opt);

    if (rtk->x[j] == 0.0L) {
      /* Initialize ionospheric delay state */
      initx(rtk, 1E-6L, SQR(rtk->opt.std[1] * bl / 1E4L), j);
    } else {
      /* Elevation dependent factor of process noise */
      long double el = rtk->ssat[sat[i] - 1].azel[1];
      long double fact = cosl(el);
      rtk->P[j + j * rtk->nx] += SQR(rtk->opt.prn[1] * bl / 1E4L * fact) * fabsl(tt);
    }
  }
}
/* Temporal update of tropospheric parameters --------------------------------*/
static void udtrop(rtk_t *rtk, long double tt, long double bl) {
  trace(3, "udtrop  : tt=%.3Lf\n", tt);

  for (int i = 0; i < 2; i++) {
    int j = IT(i, &rtk->opt);

    if (rtk->x[j] == 0.0L) {
      initx(rtk, INIT_ZWD, SQR(rtk->opt.std[2]), j); /* Initial zwd */

      if (rtk->opt.tropopt >= TROPOPT_ESTG) {
        for (int k = 0; k < 2; k++) initx(rtk, 1E-6L, VAR_GRA, ++j);
      }
    } else {
      rtk->P[j + j * rtk->nx] += SQR(rtk->opt.prn[2]) * fabsl(tt);

      if (rtk->opt.tropopt >= TROPOPT_ESTG) {
        for (int k = 0; k < 2; k++) {
          rtk->P[++j * (1 + rtk->nx)] += SQR(rtk->opt.prn[2] * 0.3L) * fabsl(tt);
        }
      }
    }
  }
}
/* Temporal update of receiver h/w biases ------------------------------------*/
static void udrcvbias(rtk_t *rtk, long double tt) {
  trace(3, "udrcvbias: tt=%.3Lf\n", tt);

  for (int i = 0; i < NFREQGLO; i++) {
    int j = IL(i, &rtk->opt);

    if (rtk->x[j] == 0.0L) {
      /* Add small offset to avoid initializing with zero */
      initx(rtk, rtk->opt.thresar[2] + 1e-6L, rtk->opt.thresar[3], j);
    }
    /* Hold to fixed solution */
    else if (rtk->nfix >= rtk->opt.minfix) {
      initx(rtk, rtk->xa[j], rtk->Pa[j + j * rtk->na], j);
    } else {
      rtk->P[j + j * rtk->nx] += SQR(rtk->opt.thresar[4]) * fabsl(tt);
    }
  }
}
/* Detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv) {
  trace(4, "detslp_ll: i=%d rcv=%d\n", i, rcv);

  int sat = obs[i].sat;

  for (int f = 0; f < rtk->opt.nf; f++) {
    if ((obs[i].L[f] == 0.0L && obs[i].LLI[f] == 0) ||
        fabsl(timediff(obs[i].time, rtk->ssat[sat - 1].pt[rcv - 1][f])) < DTTOL) {
      continue;
    }
    /* Restore previous LLI */
    uint32_t LLI;
    if (rcv == 1)
      LLI = getbitu(&rtk->ssat[sat - 1].slip[f], 1, 0, 2); /* Rover */
    else
      LLI = getbitu(&rtk->ssat[sat - 1].slip[f], 1, 2, 2); /* Base  */

    /* Detect slip by cycle slip flag in LLI */
    uint32_t slip;
    if (rtk->tt >= 0.0L) { /* Forward */
      if (obs[i].LLI[f] & 1) {
        errmsg(rtk, "slip detected forward (sat=%2d rcv=%d F=%d LLI=%x)\n", sat, rcv, f + 1,
               obs[i].LLI[f]);
      }
      slip = obs[i].LLI[f];
    } else { /* Backward */
      if (LLI & 1) {
        errmsg(rtk, "slip detected backward (sat=%2d rcv=%d F=%d LLI=%x)\n", sat, rcv, f + 1, LLI);
      }
      slip = LLI;
    }
    /* Detect slip by parity unknown flag transition in LLI */
    if (((LLI & 2) && !(obs[i].LLI[f] & 2)) || (!(LLI & 2) && (obs[i].LLI[f] & 2))) {
      errmsg(rtk, "slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x->%x)\n", sat, rcv, f + 1, LLI,
             obs[i].LLI[f]);
      slip |= 1;
    }
    /* Save current LLI */
    if (rcv == 1)
      setbitu(&rtk->ssat[sat - 1].slip[f], 1, 0, 2, obs[i].LLI[f]);
    else
      setbitu(&rtk->ssat[sat - 1].slip[f], 1, 2, 2, obs[i].LLI[f]);

    /* Save slip and half-cycle valid flag */
    rtk->ssat[sat - 1].slip[f] |= (uint8_t)slip;
    rtk->ssat[sat - 1].half[f] = (obs[i].LLI[f] & 2) ? 0 : 1;
  }
}
/* Detect cycle slip by geometry free phase jump -----------------------------*/
static void detslp_gf(rtk_t *rtk, const obsd_t *obs, int i, int j, const nav_t *nav) {
  trace(4, "detslp_gf: i=%d j=%d\n", i, j);

  int sat = obs[i].sat;

  /* Skip check if slip already detected or check disabled*/
  if (rtk->opt.thresslip == 0) return;
  for (int k = 0; k < rtk->opt.nf; k++)
    if (rtk->ssat[sat - 1].slip[k] & 1) return;

  for (int k = 1; k < rtk->opt.nf; k++) {
    /* Calc SD geomotry free LC of phase between freq0 and freqk */
    long double gf1 = gfobs(obs, i, j, k, nav);
    if (gf1 == 0.0L) continue;

    long double gf0 = rtk->ssat[sat - 1].gf[k - 1]; /* Retrieve previous gf */
    rtk->ssat[sat - 1].gf[k - 1] = gf1;             /* Save current gf for next epoch */

    if (gf0 != 0.0L && fabsl(gf1 - gf0) > rtk->opt.thresslip) {
      rtk->ssat[sat - 1].slip[0] |= 1;
      rtk->ssat[sat - 1].slip[k] |= 1;
      errmsg(rtk, "slip detected GF jump (sat=%2d L1-L%d dGF=%.3Lf)\n", sat, k + 1, gf0 - gf1);
    }
  }
}
/* Detect cycle slip by doppler and phase difference -------------------------*/
static void detslp_dop(rtk_t *rtk, const obsd_t *obs, const int *ix, int ns, int rcv,
                       const nav_t *nav) {
  trace(4, "detslp_dop: rcv=%d\n", rcv);
  if (rtk->opt.thresdop <= 0) return; /* Skip test if doppler thresh <= 0 */

  /* Calculate doppler differences for all sats and freqs */
  int nf = rtk->opt.nf, ndop = 0;
  long double mean_dop = 0;
  long double dopdif[MAXSAT][NFREQ], tt[MAXSAT][NFREQ];
  for (int i = 0; i < ns; i++) {
    int ii = ix[i];
    int sat = obs[ii].sat;

    for (int f = 0; f < nf; f++) {
      dopdif[i][f] = 0;
      tt[i][f] = 0.0L;
      if (obs[ii].L[f] == 0.0L || obs[ii].D[f] == 0.0L || rtk->ssat[sat - 1].ph[rcv - 1][f] == 0.0L)
        continue;
      if (fabsl(tt[i][f] = timediff(obs[ii].time, rtk->ssat[sat - 1].pt[rcv - 1][f])) < DTTOL)
        continue;

      /* Calc phase difference and doppler x time (cycle) */
      long double dph = (obs[ii].L[f] - rtk->ssat[sat - 1].ph[rcv - 1][f]) / tt[i][f];
      long double dpt = -obs[ii].D[f];
      dopdif[i][f] = dph - dpt;

      /* If not outlier, use this to calculate mean */
      if (fabsl(dopdif[i][f]) < 3 * rtk->opt.thresdop) {
        mean_dop += dopdif[i][f];
        ndop++;
      }
    }
  }
  /* Calc mean doppler diff, most likely due to clock error */
  if (ndop == 0) return; /* Unable to calc mean doppler, usually very large clock err */
  mean_dop = mean_dop / ndop;

  /* Set slip if doppler difference with mean removed exceeds threshold */
  for (int i = 0; i < ns; i++) {
    int sat = obs[ix[i]].sat;

    for (int f = 0; f < nf; f++) {
      if (dopdif[i][f] == 0.0L) continue;
      if (fabsl(dopdif[i][f] - mean_dop) > rtk->opt.thresdop) {
        rtk->ssat[sat - 1].slip[f] |= 1;
        errmsg(rtk, "slip detected doppler (sat=%2d rcv=%d dL%d=%.3Lf off=%.3Lf tt=%.2Lf)\n", sat,
               rcv, f + 1, dopdif[i][f] - mean_dop, mean_dop, tt[i][f]);
      }
    }
  }
}
/* Temporal update of phase biases -------------------------------------------*/
static void udbias(rtk_t *rtk, long double tt, const obsd_t *obs, const int *sat, const int *iu,
                   const int *ir, int ns, const nav_t *nav) {
  trace(3, "udbias  : tt=%.3Lf ns=%d\n", tt, ns);

  /* Clear cycle slips */
  for (int i = 0; i < ns; i++) {
    for (int k = 0; k < rtk->opt.nf; k++) rtk->ssat[sat[i] - 1].slip[k] &= 0xFC;
  }

  /* Detect cycle slip by doppler and phase difference */
  detslp_dop(rtk, obs, iu, ns, 1, nav);
  detslp_dop(rtk, obs, ir, ns, 2, nav);

  int nf = NF(&rtk->opt);
  for (int i = 0; i < ns; i++) {
    /* Detect cycle slip by LLI */
    detslp_ll(rtk, obs, iu[i], 1);
    detslp_ll(rtk, obs, ir[i], 2);

    /* Detect cycle slip by geometry-free phase jump */
    detslp_gf(rtk, obs, iu[i], ir[i], nav);

    /* Update half-cycle valid flag */
    for (int k = 0; k < nf; k++) {
      rtk->ssat[sat[i] - 1].half[k] = !((obs[iu[i]].LLI[k] & 2) || (obs[ir[i]].LLI[k] & 2));
    }
  }
  for (int k = 0; k < nf; k++) {
    /* Reset phase-bias if instantaneous AR or expire obs outage counter */
    for (int i = 1; i <= MAXSAT; i++) {
      int reset = ++rtk->ssat[i - 1].outc[k] > (uint32_t)rtk->opt.maxout;

      if (rtk->opt.modear == ARMODE_INST && rtk->x[IB(i, k, &rtk->opt)] != 0.0L) {
        initx(rtk, 0.0L, 0.0L, IB(i, k, &rtk->opt));
      } else if (reset && rtk->x[IB(i, k, &rtk->opt)] != 0.0L) {
        initx(rtk, 0.0L, 0.0L, IB(i, k, &rtk->opt));
        trace(3, "udbias : obs outage counter overflow (sat=%3d L%d n=%d)\n", i, k + 1,
              rtk->ssat[i - 1].outc[k]);
        rtk->ssat[i - 1].outc[k] = 0;
      }
      if (rtk->opt.modear != ARMODE_INST && reset) {
        rtk->ssat[i - 1].lock[k] = -rtk->opt.minlock;
      }
    }
    /* Update phase bias noise and check for cycle slips */
    for (int i = 0; i < ns; i++) {
      int j = IB(sat[i], k, &rtk->opt);
      rtk->P[j + j * rtk->nx] += rtk->opt.prn[0] * rtk->opt.prn[0] * fabsl(tt);
      int slip = rtk->ssat[sat[i] - 1].slip[k];
      int rejc = rtk->ssat[sat[i] - 1].rejc[k];
      if (rtk->opt.ionoopt == IONOOPT_IFLC) {
        int f2 = seliflc(rtk->opt.nf, rtk->ssat[sat[i] - 1].sys);
        slip |= rtk->ssat[sat[i] - 1].slip[f2];
      }
      if (rtk->opt.modear == ARMODE_INST || (!(slip & 1) && rejc < 2)) continue;
      /* Reset phase-bias state if detecting cycle slip or outlier */
      rtk->x[j] = 0.0L;
      rtk->ssat[sat[i] - 1].rejc[k] = 0;
      rtk->ssat[sat[i] - 1].lock[k] = -rtk->opt.minlock;
      /* Retain icbiases for GLONASS sats */
      if (rtk->ssat[sat[i] - 1].sys != SYS_GLO) rtk->ssat[sat[i] - 1].icbias[k] = 0;
    }
    long double *bias = zeros(ns, 1);

    /* Estimate approximate phase-bias by delta phase - delta code */
    int j = 0;
    long double offset = 0.0L;
    for (int i = 0; i < ns; i++) {
      if (rtk->opt.ionoopt != IONOOPT_IFLC) {
        /* Phase diff between rover and base in cycles */
        long double cp = sdobs(obs, iu[i], ir[i], k); /* Cycle */
        /* Pseudorange diff between rover and base in meters */
        long double pr = sdobs(obs, iu[i], ir[i], k + NFREQ);
        long double freqi = sat2freq(sat[i], obs[iu[i]].code[k], nav);
        if (cp == 0.0L || pr == 0.0L || freqi == 0.0L) continue;
        /* Estimate bias in cycles */
        bias[i] = cp - pr * freqi / CLIGHT;
      } else { /* Use ionosphere free calc with 2 freqs */
        int f2 = seliflc(rtk->opt.nf, rtk->ssat[sat[i] - 1].sys);
        long double cp1 = sdobs(obs, iu[i], ir[i], 0);
        long double cp2 = sdobs(obs, iu[i], ir[i], f2);
        long double pr1 = sdobs(obs, iu[i], ir[i], NFREQ);
        long double pr2 = sdobs(obs, iu[i], ir[i], NFREQ + f2);
        long double freq1 = sat2freq(sat[i], obs[iu[i]].code[0], nav);
        long double freq2 = sat2freq(sat[i], obs[iu[i]].code[f2], nav);
        if (cp1 == 0.0L || cp2 == 0.0L || pr1 == 0.0L || pr2 == 0.0L || freq1 <= 0.0L ||
            freq2 <= 0.0L)
          continue;

        long double C1 = SQR(freq1) / (SQR(freq1) - SQR(freq2));
        long double C2 = -SQR(freq2) / (SQR(freq1) - SQR(freq2));
        /* Estimate bias in meters */
        bias[i] = (C1 * cp1 * CLIGHT / freq1 + C2 * cp2 * CLIGHT / freq2) - (C1 * pr1 + C2 * pr2);
      }
      if (rtk->x[IB(sat[i], k, &rtk->opt)] != 0.0L) {
        offset += bias[i] - rtk->x[IB(sat[i], k, &rtk->opt)];
        j++;
      }
    }
    /* Correct phase-bias offset to ensure phase-code coherency */
    if (j > 0) {
      for (int i = 1; i <= MAXSAT; i++) {
        if (rtk->x[IB(i, k, &rtk->opt)] != 0.0L) rtk->x[IB(i, k, &rtk->opt)] += offset / j;
      }
    }
    /* Set initial states of phase-bias */
    for (int i = 0; i < ns; i++) {
      if (bias[i] == 0.0L || rtk->x[IB(sat[i], k, &rtk->opt)] != 0.0L) continue;
      initx(rtk, bias[i], SQR(rtk->opt.std[0]), IB(sat[i], k, &rtk->opt));
      trace(3, "     sat=%3d, F=%d: init phase=%.3Lf\n", sat[i], k + 1, bias[i]);
      rtk->ssat[sat[i] - 1].lock[k] = -rtk->opt.minlock;
    }
    free(bias);
  }
}
/* Temporal update of states -------------------------------------------------*/
static void udstate(rtk_t *rtk, const obsd_t *obs, const int *sat, const int *iu, const int *ir,
                    int ns, const nav_t *nav) {
  trace(3, "udstate : ns=%d\n", ns);

  long double tt = rtk->tt;

  /* Temporal update of position/velocity/acceleration */
  udpos(rtk, tt);

  /* Temporal update of ionospheric parameters */
  if (rtk->opt.ionoopt == IONOOPT_EST || rtk->opt.tropopt >= TROPOPT_EST) {
    long double dr[3], bl = baseline(rtk->x, rtk->rb, dr);
    if (rtk->opt.ionoopt == IONOOPT_EST) {
      udion(rtk, tt, bl, sat, ns);
    }
    /* Temporal update of tropospheric parameters */
    if (rtk->opt.tropopt >= TROPOPT_EST) {
      udtrop(rtk, tt, bl);
    }
  }
  /* Temporal update of receiver h/w bias */
  if (rtk->opt.glomodear == GLO_ARMODE_AUTOCAL && (rtk->opt.navsys & SYS_GLO)) {
    udrcvbias(rtk, tt);
  }
  /* Temporal update of phase-bias */
  if (rtk->opt.mode > PMODE_DGPS) {
    udbias(rtk, tt, obs, sat, iu, ir, ns, nav);
  }
}
/* UD (undifferenced) phase/code residual for satellite ----------------------*/
static void zdres_sat(int base, long double r, const obsd_t *obs, const nav_t *nav,
                      const long double *azel, const long double *dant, const prcopt_t *opt,
                      long double *y, long double *freq) {
  int nf = NF(opt);
  if (opt->ionoopt == IONOOPT_IFLC) { /* Iono-free linear combination */
    long double freq1 = sat2freq(obs->sat, obs->code[0], nav);
    int f2 = seliflc(opt->nf, satsys(obs->sat, NULL));
    long double freq2 = sat2freq(obs->sat, obs->code[f2], nav);

    if (freq1 == 0.0L || freq2 == 0.0L) return;

    if (testsnr(base, 0, azel[1], obs->SNR[0] * SNR_UNIT, &opt->snrmask) ||
        testsnr(base, f2, azel[1], obs->SNR[f2] * SNR_UNIT, &opt->snrmask))
      return;

    long double C1 = SQR(freq1) / (SQR(freq1) - SQR(freq2));
    long double C2 = -SQR(freq2) / (SQR(freq1) - SQR(freq2));
    long double dant_if = C1 * dant[0] + C2 * dant[f2];

    if (obs->L[0] != 0.0L && obs->L[f2] != 0.0L) {
      y[0] = C1 * obs->L[0] * CLIGHT / freq1 + C2 * obs->L[f2] * CLIGHT / freq2 - r - dant_if;
    }
    if (obs->P[0] != 0.0L && obs->P[f2] != 0.0L) {
      y[nf] = C1 * obs->P[0] + C2 * obs->P[f2] - r - dant_if;
    }
    freq[0] = 1.0L;
  } else {
    for (int i = 0; i < nf; i++) {
      if ((freq[i] = sat2freq(obs->sat, obs->code[i], nav)) == 0.0L) continue;

      /* Check SNR mask */
      if (testsnr(base, i, azel[1], obs->SNR[i] * SNR_UNIT, &opt->snrmask)) {
        continue;
      }
      /* Residuals = observable - estimated range */
      if (obs->L[i] != 0.0L) y[i] = obs->L[i] * CLIGHT / freq[i] - r - dant[i];
      if (obs->P[i] != 0.0L) y[i + nf] = obs->P[i] - r - dant[i];
      trace(4, "zdres_sat: %d: L=%.6Lf P=%.6Lf r=%.6Lf f=%.0Lf\n", obs->sat, obs->L[i], obs->P[i],
            r, freq[i]);
    }
  }
}
/* Undifferenced phase/code residuals ------------------------------------------
    Calculate zero diff residuals [observed pseudorange - range]
        output is in y[0:nu-1], only shared input with base is nav
 Args:  I   base:  1=base,0=rover
        I   obs  = sat observations
        I   n    = # of sats
        I   rs [(0:2)+i*6]= sat position {x,y,z} (m)
        I   dts[(0:1)+i*2]= sat clock {bias,drift} (s|s/s)
        I   var  = variance of ephemeris
        I   svh  = sat health flags
        I   nav  = sat nav data
        I   rr   = rcvr pos (x,y,z)
        I   opt  = options
        O   y[(0:1)+i*2] = zero diff residuals {phase,code} (m)
        O   e    = line of sight unit vectors to sats
        O   azel = [az, el] to sats                                           */
static bool zdres(int base, const obsd_t *obs, int n, const long double *rs, const long double *dts,
                  const long double *var, const int *svh, const nav_t *nav, const long double *rr,
                  const prcopt_t *opt, long double *y, long double *e, long double *azel,
                  long double *freq) {
  trace(3, "zdres   : n=%d rr=%.2Lf %.2Lf %.2Lf\n", n, rr[0], rr[1], rr[2]);

  int nf = NF(opt);

  /* Init residuals to zero */
  for (int i = 0; i < n * nf * 2; i++) y[i] = 0.0L;

  if (norm(rr, 3) <= 0.0L) return false; /* No receiver position */

  /* rr_ = local copy of rcvr pos */
  long double rr_[3];
  for (int i = 0; i < 3; i++) rr_[i] = rr[i];

  /* Adjust rcvr pos for earth tide correction */
  if (opt->tidecorr) {
    long double disp[3];
    tidedisp(gpst2utc(obs[0].time), rr_, opt->tidecorr, &nav->erp, opt->odisp[base], disp);
    for (int i = 0; i < 3; i++) rr_[i] += disp[i];
  }
  /* Translate rcvr pos from ECEF to geodetic */
  long double pos[3];
  ecef2pos(rr_, pos);

  /* Loop through satellites */
  for (int i = 0; i < n; i++) {
    /* Compute geometric-range and azimuth/elevation angle */
    long double r = geodist(rs + i * 6, rr_, e + i * 3);
    if (r <= 0.0L) continue;
    if (satazel(pos, e + i * 3, azel + i * 2) < opt->elmin) continue;

    /* Excluded satellite? */
    if (satexclude(obs[i].sat, var[i], svh[i], opt)) continue;

    /* Adjust range for satellite clock-bias */
    r += -CLIGHT * dts[i * 2];

    /* Adjust range for troposphere delay model (hydrostatic) */
    const long double zazel[] = {0.0L, 90.0L * D2R};
    long double zhd = tropmodel(obs[0].time, pos, zazel, 0.0L);
    long double mapfh = tropmapf(obs[i].time, pos, azel + i * 2, NULL);
    r += mapfh * zhd;

    /* Calc receiver antenna phase center correction */
    long double dant[NFREQ];
    antmodel(opt->pcvr + base, opt->antdel[base], azel + i * 2, opt->posopt[1], dant);

    /* Calc undifferenced phase/code residual for satellite */
    trace(4, "sat=%d r=%.6Lf c*dts=%.6Lf zhd=%.6Lf map=%.6Lf\n", obs[i].sat, r, CLIGHT * dts[i * 2],
          zhd, mapfh);
    zdres_sat(base, r, obs + i, nav, azel + i * 2, dant, opt, y + i * nf * 2, freq + i * nf);
  }
  trace(4, "rr_=%.3Lf %.3Lf %.3Lf\n", rr_[0], rr_[1], rr_[2]);
  trace(4, "pos=%.9Lf %.9Lf %.3Lf\n", pos[0] * R2D, pos[1] * R2D, pos[2]);
  for (int i = 0; i < n; i++) {
    if ((obs[i].L[0] == 0 && obs[i].L[1] == 0 && obs[i].L[2] == 0) || base == 0) continue;
    trace(3, "sat=%2d rs=%13.3Lf %13.3Lf %13.3Lf dts=%13.10Lf az=%6.1Lf el=%5.1Lf\n", obs[i].sat,
          rs[i * 6], rs[1 + i * 6], rs[2 + i * 6], dts[i * 2], azel[i * 2] * R2D,
          azel[1 + i * 2] * R2D);
  }
  trace(3, "y=\n");
  tracemat(3, y, nf * 2, n, 13, 3);

  return true;
}
/* Test valid observation data -----------------------------------------------*/
static bool validobs(int i, int j, int f, int nf, const long double *y) {
  /* Check for valid residuals */
  return y[f + i * nf * 2] != 0.0L && y[f + j * nf * 2] != 0.0L;
}
/* Long Double-differenced measurement error covariance ------------------------
 *
 *   nb[n]:  # of sat pairs in group
 *   n:      # of groups (2 for each system, phase and code)
 *   Ri[nv]: variances of first sats in long double diff pairs
 *   Rj[nv]: variances of 2nd sats in long double diff pairs
 *   nv:     total # of sat pairs
 *   R[nv][nv]:  long double diff measurement err covariance matrix       */
static void ddcov(const int *nb, int n, const long double *Ri, const long double *Rj, int nv,
                  long double *R) {
  trace(4, "ddcov   : n=%d\n", n);

  for (int i = 0; i < nv * nv; i++) R[i] = 0.0L;

  for (int b = 0, k = 0; b < n; k += nb[b++]) { /* Loop through each system */
    for (int i = 0; i < nb[b]; i++)
      for (int j = 0; j < nb[b]; j++) {
        R[k + i + (k + j) * nv] = Ri[k + i] + (i == j ? Rj[k + i] : 0.0L);
      }
  }
  trace(5, "R=\n");
  tracemat(5, R, nv, nv, 8, 6);
}

/* Compressed vector write */
static inline void cvwrite(long double *V, int nc, const int *xi, int i, long double v) {
  int ii = xi[i];
  if (ii >= nc) return;
  V[ii] = v;
}

/* Read from a compressed matrix otherwise from an uncompressed shadow matrix.
    A   - uncompressed matrix (n x n)
    n   - number of rows and columns in A
    Ac  - compacted copy of A (nc x nx)
    nc  - number of rows and columns in Ac
    xi  - vector mapping row or column index in A to index in Ac
    i,j - row, column index into A
   Note: if i,j exists in Ac then reads the value from Ac, otherwise read from A. */
static inline long double cmatread(const long double *A, int n, const long double *Ac, int nc,
                                   const int *xi, int i, int j) {
  int ii = xi[i], jj = xi[j];
  if (ii >= nc || jj > nc) return A[i + j * n];
  return Ac[ii + jj * nc];
}

/* Baseline length constraint ------------------------------------------------*/
static bool constbl(rtk_t *rtk, const long double *x, const long double *P, const long double *Pc,
                    int nc, const int *xi, long double *v, long double *Hc, long double *Ri,
                    long double *Rj, int index) {
  const long double thres = 0.1L; /* Threshold for nonlinearity (v.2.3.0) */

  trace(4, "constbl : \n");

  /* Time-adjusted baseline vector and length */
  long double xb[3], b[3];
  for (int i = 0; i < 3; i++) {
    xb[i] = rtk->rb[i];
    b[i] = x[i] - xb[i];
  }
  long double bb = norm(b, 3);

  /* Approximate variance of solution */
  long double var = 0.0L;
  if (P) {
    for (int i = 0; i < 3; i++) var += cmatread(P, rtk->nx, Pc, nc, xi, i, i);
    var /= 3.0L;
  }
  /* Check nonlinearity */
  if (var > SQR(thres * bb)) {
    trace(3, "constbl : pos variance large (bb=%.3Lf var=%.3Lf)\n", bb, var);
    /* return false; */ /* Threshold too strict for all use cases, report error but continue on
                         */
  }
  /* Constraint to baseline length */
  v[index] = rtk->opt.baseline[0] - bb;
  if (Hc) {
    for (int i = 0; i < 3; i++) cvwrite(Hc + index * nc, nc, xi, i, b[i] / bb);
  }
  Ri[index] = 0.0L;
  Rj[index] = SQR(rtk->opt.baseline[1]);

  trace(3, "constbl : baseline len   v=%13.3Lf R=%8.6Lf\n", v[index], Rj[index]);

  return true;
}
/* Precise tropospheric model ------------------------------------------------*/
static long double prectrop(gtime_t time, const long double *pos, int r, const long double *azel,
                            const prcopt_t *opt, const long double *x, long double *dtdx) {
  /* Wet mapping function */
  long double m_w = 0.0L;
  tropmapf(time, pos, azel, &m_w);

  int i = IT(r, opt);
  if (opt->tropopt >= TROPOPT_ESTG && azel[1] > 0.0L) {
    /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
    long double cotz = 1.0L / tanl(azel[1]);
    long double grad_n = m_w * cotz * cosl(azel[0]);
    long double grad_e = m_w * cotz * sinl(azel[0]);
    m_w += grad_n * x[i + 1] + grad_e * x[i + 2];
    dtdx[1] = grad_n * x[i];
    dtdx[2] = grad_e * x[i];
  } else
    dtdx[1] = dtdx[2] = 0.0L;
  dtdx[0] = m_w;
  return m_w * x[i];
}
/* Test satellite system (m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN) ---------*/
static inline bool test_sys(int sys, int m) {
  const int im[] = {-1,  /* Undefined */
                    0,   /* GPS */
                    0,   /* SBS */
                    1,   /* GLO */
                    2,   /* GAL */
                    4,   /* QZS */
                    3,   /* CMP */
                    5,   /* IRN */
                    -1}; /* LEO */
  return m == im[sys2no(sys)];
}
/* Long Double-differenced residuals and partial derivatives  ------------------
        O rtk->ssat[i].resp[j] = residual pseudorange error
        O rtk->ssat[i].resc[j] = residual carrier phase error
        I rtk->rb= base location
        I dt = time diff between base and rover observations
        I x = rover pos & vel and sat phase biases (float solution)
        I P = error covariance matrix of float states
        I Pc = compacted P matrix, shadows P.
        I sat = list of common sats
        I y = zero diff residuals (code and phase, base and rover)
        I e = line of sight unit vectors to sats
        I azel = [az, el] to sats
        I iu,ir = user and ref indices to sats
        I ns = # of sats
        O v = long double diff innovations (measurement-model) (phase and code)
        O Hc = linearized translation from innovations to states (az/el to sats)
        O R = measurement error covariances
        O vflg = bit encoded list of sats used for each long double diff  */
static int ddres(rtk_t *rtk, const obsd_t *obs, long double dt, const long double *x,
                 const long double *P, const long double *Pc, int nc, const int *xi, const int *sat,
                 const long double *y, const long double *e, const long double *azel,
                 const long double *freq, const int *iu, const int *ir, int ns, long double *v,
                 long double *Hc, long double *R, int *vflg) {
  trace(3, "ddres   : dt=%.4Lf ns=%d\n", dt, ns);

  /* bl=distance from base to rover, dr=x,y,z components */
  long double dr[3], bl = baseline(x, rtk->rb, dr);
  /* Translate ECEF pos to geodetic pos */
  long double posu[3];
  ecef2pos(x, posu);
  long double posr[3];
  ecef2pos(rtk->rb, posr);

  /* Zero out residual phase and code biases for all satellites */
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < NFREQ; j++) {
      rtk->ssat[i].resp[j] = rtk->ssat[i].resc[j] = 0.0L;
    }
  /* Compute factors of ionospheric and tropospheric delay
         - only used if kalman filter contains states for ION and TROP delays
         usually insignificant for short baselines (<10km)*/
  long double *im = mat(ns, 1);
  long double *tropu = mat(ns, 1), *tropr = mat(ns, 1), *dtdxu = mat(ns, 3), *dtdxr = mat(ns, 3);
  prcopt_t *opt = &rtk->opt;
  for (int i = 0; i < ns; i++) {
    if (opt->ionoopt == IONOOPT_EST) {
      im[i] = (ionmapf(posu, azel + iu[i] * 2) + ionmapf(posr, azel + ir[i] * 2)) / 2.0L;
    }
    if (opt->tropopt >= TROPOPT_EST) {
      tropu[i] = prectrop(rtk->sol.time, posu, 0, azel + iu[i] * 2, opt, x, dtdxu + i * 3);
      tropr[i] = prectrop(rtk->sol.time, posr, 1, azel + ir[i] * 2, opt, x, dtdxr + i * 3);
    }
  }
  /* Step through sat systems: m=0:gps/SBS,1:glo,2:gal,3:bds 4:qzs 5:irn*/
  int nv = 0, nb[NFREQ * NSYS * 2 + 2] = {0}, b = 0, nf = NF(opt);
  long double *Ri = mat(ns * nf * 2 + 2, 1), *Rj = mat(ns * nf * 2 + 2, 1);
  for (int m = 0; m < 6; m++) {
    /* Step through phases/codes */
    for (int f = opt->mode > PMODE_DGPS ? 0 : nf; f < nf * 2; f++) {
      int frq = f % nf, code = f < nf ? 0 : 1, i = -1;

      /* Find reference satellite with highest elevation, set to i */
      for (int j = 0; j < ns; j++) {
        int sysi = rtk->ssat[sat[j] - 1].sys;
        if (!test_sys(sysi, m) || sysi == SYS_SBS) continue;
        if (!validobs(iu[j], ir[j], f, nf, y)) continue;
        /* Skip sat with slip unless no other valid sat */
        if (i >= 0 && rtk->ssat[sat[j] - 1].slip[frq] & LLI_SLIP) continue;
        if (i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) i = j;
      }
      if (i < 0) continue;

      /* Calculate long double differences of residuals (code/phase) for each sat */
      for (int j = 0; j < ns; j++) {
        long double *Hi = NULL;

        if (i == j) continue; /* Skip ref sat */
        int sysi = rtk->ssat[sat[i] - 1].sys;
        int sysj = rtk->ssat[sat[j] - 1].sys;
        long double freqi = freq[frq + iu[i] * nf];
        long double freqj = freq[frq + iu[j] * nf];
        if (freqi <= 0.0L || freqj <= 0.0L) continue;
        if (!test_sys(sysj, m)) continue;
        if (!validobs(iu[j], ir[j], f, nf, y)) continue;

        if (Hc) {
          Hi = Hc + nv * nc;
          for (int k = 0; k < nc; k++) Hi[k] = 0.0L;
        }

        /* Long Double-differenced measurements from 2 receivers and 2 sats in meters */
        v[nv] = (y[f + iu[i] * nf * 2] - y[f + ir[i] * nf * 2]) -
                (y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);

        /* Partial derivatives by rover position, combine unit vectors from two sats */
        if (Hc) {
          /* Translation of innovation to position states */
          for (int k = 0; k < 3; k++) {
            cvwrite(Hi, nc, xi, k, -e[k + iu[i] * 3] + e[k + iu[j] * 3]);
          }
        }
        if (opt->ionoopt == IONOOPT_EST) {
          /* Adjust long double-differenced measurements by long double-differenced ionospheric
           * delay term */
          long double didxi = (code ? -1.0L : 1.0L) * im[i] * SQR(FREQL1 / freqi);
          long double didxj = (code ? -1.0L : 1.0L) * im[j] * SQR(FREQL1 / freqj);
          v[nv] -= didxi * x[II(sat[i], opt)] - didxj * x[II(sat[j], opt)];
          if (Hc) {
            cvwrite(Hi, nc, xi, II(sat[i], opt), didxi);
            cvwrite(Hi, nc, xi, II(sat[j], opt), -didxi);
          }
        }
        if (opt->tropopt >= TROPOPT_EST) {
          /* Adjust long double-differenced measurements by long double-differenced tropospheric
           * delay term */
          v[nv] -= (tropu[i] - tropu[j]) - (tropr[i] - tropr[j]);
          if (!Hc) continue;
          for (int k = 0; k < (opt->tropopt < TROPOPT_ESTG ? 1 : 3); k++) {
            cvwrite(Hi, nc, xi, IT(0, opt) + k, (dtdxu[k + i * 3] - dtdxu[k + j * 3]));
            cvwrite(Hi, nc, xi, IT(1, opt) + k, -(dtdxr[k + i * 3] - dtdxr[k + j * 3]));
          }
        }
        int ii = IB(sat[i], frq, opt);
        int jj = IB(sat[j], frq, opt);
        if (!code) {
          /* Adjust phase residual by long double-differenced phase-bias term,
                IB=look up index by sat&freq */
          if (opt->ionoopt != IONOOPT_IFLC) {
            /* Phase-bias states are single-differenced so need to difference them */
            v[nv] -= CLIGHT / freqi * x[ii] - CLIGHT / freqj * x[jj];
            if (Hc) {
              cvwrite(Hi, nc, xi, ii, CLIGHT / freqi);
              cvwrite(Hi, nc, xi, jj, -CLIGHT / freqj);
            }
          } else {
            v[nv] -= x[ii] - x[jj];
            if (Hc) {
              cvwrite(Hi, nc, xi, ii, 1.0L);
              cvwrite(Hi, nc, xi, jj, -1.0L);
            }
          }
        }

        /* Adjust long double-difference for GLONASS sats */
        if (sysi == SYS_GLO && sysj == SYS_GLO) {
          if (rtk->opt.glomodear == GLO_ARMODE_AUTOCAL && frq < NFREQGLO) {
            /* Auto-cal method */
            long double df = (freqi - freqj) / (f == 0 ? DFRQ1_GLO : DFRQ2_GLO);
            v[nv] -= df * x[IL(frq, opt)];
            if (Hc) cvwrite(Hi, nc, xi, IL(frq, opt), df);
          } else if (rtk->opt.glomodear == GLO_ARMODE_FIXHOLD && frq < NFREQGLO) {
            /* Fix-and-hold method */
            long double icb = rtk->ssat[sat[i] - 1].icbias[frq] * CLIGHT / freqi -
                              rtk->ssat[sat[j] - 1].icbias[frq] * CLIGHT / freqj;
            v[nv] -= icb;
          }
        }

        /* Adjust long double-difference for SBAS sats */
        if (sysj == SYS_SBS && sysi == SYS_GPS) {
          if (rtk->opt.glomodear == GLO_ARMODE_FIXHOLD && frq < NFREQ) {
            /* Fix-and-hold method */
            long double icb = rtk->ssat[sat[i] - 1].icbias[frq] * CLIGHT / freqi -
                              rtk->ssat[sat[j] - 1].icbias[frq] * CLIGHT / freqj;
            v[nv] -= icb;
          }
        }

        /* Save residuals */
        if (code)
          rtk->ssat[sat[j] - 1].resp[frq] = v[nv]; /* Pseudorange */
        else
          rtk->ssat[sat[j] - 1].resc[frq] = v[nv]; /* Carrier phase */

        /* Open up outlier threshold if one of the phase biases was just initialized */
        long double threshadj =
            (cmatread(P, rtk->nx, Pc, nc, xi, ii, ii) == SQR(rtk->opt.std[0])) ||
                    (cmatread(P, rtk->nx, Pc, nc, xi, jj, jj) == SQR(rtk->opt.std[0]))
                ? 10
                : 1;

        /* If residual too large, flag as outlier */
        if (fabsl(v[nv]) > opt->maxinno[code] * threshadj) {
          rtk->ssat[sat[j] - 1].vsat[frq] = 0;
          rtk->ssat[sat[j] - 1].rejc[frq]++;
          errmsg(rtk, "outlier rejected (sat=%3d-%3d %s%d v=%.3Lf)\n", sat[i], sat[j],
                 code ? "P" : "L", frq + 1, v[nv]);
          continue;
        }

        /* Single-differenced measurement error variances (m) */
        Ri[nv] = varerr(
            sat[i], sysi, azel[1 + iu[i] * 2], SNR_UNIT * rtk->ssat[sat[i] - 1].snr_rover[frq],
            SNR_UNIT * rtk->ssat[sat[i] - 1].snr_base[frq], bl, dt, f, opt, &obs[iu[i]]);
        Rj[nv] = varerr(
            sat[j], sysj, azel[1 + iu[j] * 2], SNR_UNIT * rtk->ssat[sat[j] - 1].snr_rover[frq],
            SNR_UNIT * rtk->ssat[sat[j] - 1].snr_base[frq], bl, dt, f, opt, &obs[iu[j]]);
        /* Increase variance if half cycle flags set */
        if (!code && (obs[iu[i]].LLI[frq] & LLI_HALFC)) Ri[nv] += 0.01L;
        if (!code && (obs[iu[j]].LLI[frq] & LLI_HALFC)) Rj[nv] += 0.01L;

        /* Set valid data flags */
        if (opt->mode > PMODE_DGPS) {
          if (!code) rtk->ssat[sat[i] - 1].vsat[frq] = rtk->ssat[sat[j] - 1].vsat[frq] = 1;
        } else {
          rtk->ssat[sat[i] - 1].vsat[frq] = rtk->ssat[sat[j] - 1].vsat[frq] = 1;
        }

#ifdef TRACE
        long double icb;
        if (rtk->opt.glomodear == GLO_ARMODE_AUTOCAL)
          icb = x[IL(frq, opt)];
        else
          icb = rtk->ssat[sat[i] - 1].icbias[frq] * CLIGHT / freqi -
                rtk->ssat[sat[j] - 1].icbias[frq] * CLIGHT / freqj;
        jj = IB(sat[j], frq, &rtk->opt);
        trace(3,
              "sat=%3d-%3d %s%d v=%13.3Lf R=%9.6Lf %9.6Lf icb=%9.3Lf lock=%5d x=%9.3Lf P=%.3Lf\n",
              sat[i], sat[j], code ? "P" : "L", frq + 1, v[nv], Ri[nv], Rj[nv], icb,
              rtk->ssat[sat[j] - 1].lock[frq], x[jj], cmatread(P, rtk->nx, Pc, nc, xi, jj, jj));
#endif

        vflg[nv++] = (sat[i] << 16) | (sat[j] << 8) | ((code ? 1 : 0) << 4) | (frq);
        nb[b]++;
      }
      b++;
    }
  } /* End of system loop */

  /* Baseline length constraint, for fixed distance between base and rover */
  if (rtk->opt.baseline[0] > 0.0L) {
    if (constbl(rtk, x, P, Pc, nc, xi, v, Hc, Ri, Rj, nv)) {
      vflg[nv++] = 3 << 4;
      nb[b++]++;
    }
  }
  if (Hc) {
    trace(5, "Hc=\n");
    tracemat(5, Hc, nc, nv, 7, 4);
  }

  /* Long Double-differenced measurement error covariance */
  ddcov(nb, b, Ri, Rj, nv, R);

  free(Ri);
  free(Rj);
  free(im);
  free(tropu);
  free(tropr);
  free(dtdxu);
  free(dtdxr);

  return nv;
}

/* Time-interpolation of residuals (for post-processing solutions) -------------
        time = rover time stamp
        obs = pointer to first base observation for this epoch
        y = pointer to base obs errors */
static long double intpres(gtime_t time, const obsd_t *obs, int n, const nav_t *nav, rtk_t *rtk,
                           long double *y) {
  static obsd_t obsb[MAXOBS];
  static long double yb[MAXOBS * NFREQ * 2], rs[MAXOBS * 6], dts[MAXOBS * 2], var[MAXOBS];
  static long double e[MAXOBS * 3], azel[MAXOBS * 2], freq[MAXOBS * NFREQ];
  static int nb = 0, svh[MAXOBS * 2];

  long double tt =
      timediff(time, obs[0].time); /* Time delta between rover obs and current base obs */
  trace(3, "intpres : n=%d tt=%.1Lf, epoch=%d\n", n, tt, rtk->epoch);
  /* Use current base obs if first epoch or delta time between rover obs and
     current base obs very small */
  if (nb == 0 || rtk->epoch == 0 || fabsl(tt) < DTTOL) {
    nb = n;
    for (int i = 0; i < n; i++) obsb[i] = obs[i]; /* Current base obs -> previous base obs */
    return tt;
  }
  /* Use current base obs if delta time between rover obs and previous base obs too large
     or same as between current base and rover */
  long double ttb =
      timediff(time, obsb[0].time); /* Time delta between rover obs and previous base obs */

  prcopt_t *opt = &rtk->opt;
  if (fabsl(ttb) > opt->maxtdiff * 2.0L || ttb == tt) return tt;

  /* Calculate sat positions for previous base obs */
  satposs(time, obsb, nb, nav, opt->sateph, rs, dts, var, svh);

  /* Calculate [measured pseudorange - range] for previous base obs */
  if (!zdres(1, obsb, nb, rs, dts, var, svh, nav, rtk->rb, opt, yb, e, azel, freq)) {
    return tt;
  }
  /* Interpolate previous and current base obs */
  int nf = NF(opt);
  for (int i = 0; i < n; i++) {
    /* Align previous sat to current sat */
    int j = 0;
    for (; j < nb; j++)
      if (obsb[j].sat == obs[i].sat) break;
    if (j >= nb) continue;
    /* p=ptr to current obs error, q=ptr to prev obs error,
       tt = delta time between rover and current base obs,
       ttb = delta time between rover and previous base obs */
    long double *p = y + i * nf * 2, *q = yb + j * nf * 2;
    for (int k = 0; k < nf * 2; k++, p++, q++) {
      if (*p == 0.0L || *q == 0.0L || (obs[i].LLI[k % nf] & LLI_SLIP) ||
          (obsb[j].LLI[k % nf] & LLI_SLIP))
        *p = 0.0L;
      else
        /* Calculate interpolated values */
        *p = (ttb * (*p) - tt * (*q)) / (ttb - tt);
    }
  }
  return fabsl(ttb) < fabsl(tt) ? ttb : tt;
}
/* Index for single to long double-difference transformation matrix (D') -----*/
static int ddidx(rtk_t *rtk, int *ix, int gps, int glo, int sbs) {
  trace(3, "ddidx: gps=%d/%d glo=%d/%d sbs=%d\n", gps, rtk->opt.gpsmodear, glo, rtk->opt.glomodear,
        sbs);

  /* Clear fix flag for all sats (1=float, 2=fix) */
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < NFREQ; j++) {
      rtk->ssat[i].fix[j] = 0;
    }

  int nb = 0, na = rtk->na, nf = NF(&rtk->opt);
  long double fix[MAXSAT], ref[MAXSAT];
  for (int m = 0; m < 6; m++) { /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */

    /* Skip if ambiguity resolution turned off for this sys */
    int nofix = (m == 0 && gps == 0) || (m == 1 && glo == 0) || (m == 3 && rtk->opt.bdsmodear == 0);

    /* Step through freqs */
    for (int f = 0, k = na; f < nf; f++, k += MAXSAT) {
      /* Look for first valid sat (i=state index, i-k=sat index) */
      int i = k;
      for (; i < k + MAXSAT; i++) {
        /* Skip if sat not active */
        if (rtk->x[i] == 0.0L || !test_sys(rtk->ssat[i - k].sys, m) || !rtk->ssat[i - k].vsat[f]) {
          continue;
        }
        /* Set sat to use for fixing ambiguity if meets criteria */
        if (rtk->ssat[i - k].lock[f] >= 0 && !(rtk->ssat[i - k].slip[f] & 2) &&
            rtk->ssat[i - k].azel[1] >= rtk->opt.elmaskar && !nofix) {
          rtk->ssat[i - k].fix[f] = 2; /* Fix */
          break;                       /* Break out of loop if find good sat */
        }
        /* Else don't use this sat for fixing ambiguity */
        else
          rtk->ssat[i - k].fix[f] = 1;
      }
      if (i >= k + MAXSAT || rtk->ssat[i - k].fix[f] != 2) continue; /* No good sat found */
      /* Step through all sats (j=state index, j-k=sat index, i-k=first good sat) */
      int n = 0;
      for (int j = k; j < k + MAXSAT; j++) {
        if (i == j || rtk->x[j] == 0.0L || !test_sys(rtk->ssat[j - k].sys, m) ||
            !rtk->ssat[j - k].vsat[f]) {
          continue;
        }
        if (sbs == 0 && satsys(j - k + 1, NULL) == SYS_SBS) continue;
        if (rtk->ssat[j - k].lock[f] >= 0 && !(rtk->ssat[j - k].slip[f] & 2) &&
            rtk->ssat[j - k].vsat[f] && rtk->ssat[j - k].azel[1] >= rtk->opt.elmaskar && !nofix) {
          /* Set D coeffs to subtract sat j from sat i */
          ix[nb * 2] = i;     /* State index of ref bias */
          ix[nb * 2 + 1] = j; /* State index of target bias */
          /* Inc # of sats used for fix */
          ref[nb] = i - k + 1;
          fix[nb++] = j - k + 1;
          rtk->ssat[j - k].fix[f] = 2; /* Fix */
          n++;                         /* Count # of sat pairs for this freq/constellation */
        }
        /* Else don't use this sat for fixing ambiguity */
        else
          rtk->ssat[j - k].fix[f] = 1;
      }
      /* Don't use ref sat if no sat pairs */
      if (n == 0) rtk->ssat[i - k].fix[f] = 1;
    }
  }

  if (nb > 0) {
    trace(3, "refSats=");
    tracemat(3, ref, 1, nb, 7, 0);
    trace(3, "fixSats=");
    tracemat(3, fix, 1, nb, 7, 0);
  }
  return nb;
}
/* Translate long double diff fixed phase-bias values to single diff fix phase-bias values */
static void restamb(rtk_t *rtk, const long double *bias, int nb, long double *xa) {
  trace(3, "restamb :\n");

  /* Init all fixed states to float state values */
  for (int i = 0; i < rtk->nx; i++) xa[i] = rtk->x[i];

  /* Overwrite non phase-bias states with fixed values */
  for (int i = 0; i < rtk->na; i++) xa[i] = rtk->xa[i];

  int index[MAXSAT] = {0}, nv = 0, nf = NF(&rtk->opt);
  for (int m = 0; m < 6; m++)
    for (int f = 0; f < nf; f++) {
      int n = 0;
      for (int i = 0; i < MAXSAT; i++) {
        if (!test_sys(rtk->ssat[i].sys, m) || rtk->ssat[i].fix[f] != 2) {
          continue;
        }
        index[n++] = IB(i + 1, f, &rtk->opt);
      }
      if (n < 2) continue;

      xa[index[0]] = rtk->x[index[0]];

      for (int i = 1; i < n; i++) {
        xa[index[i]] = xa[index[0]] - bias[nv++];
      }
    }
}
/* Hold integer ambiguity ----------------------------------------------------*/
static void holdamb(rtk_t *rtk, const long double *xa) {
  trace(3, "holdamb :\n");

  /* Pre-calculate the size nv. Needs to match the loop below. */
  int nv = 0, nf = NF(&rtk->opt);
  for (int m = 0; m < 6; m++)
    for (int f = 0; f < nf; f++) {
      int n = 0;
      for (int i = 0; i < MAXSAT; i++) {
        if (!test_sys(rtk->ssat[i].sys, m) || rtk->ssat[i].fix[f] != 2 ||
            rtk->ssat[i].azel[1] < rtk->opt.elmaskhold) {
          continue;
        }
        n++;
      }
      nv += n < 1 ? 0 : n - 1;
    }

  /* Return if less than min sats for hold (skip if fix&hold for GLONASS only) */
  if (rtk->opt.modear == ARMODE_FIXHOLD && nv < rtk->opt.minholdsats) {
    trace(3, "holdamb: not enough sats to hold ambiguity\n");
    return;
  }

  /* Create list of non-zero states */
  int nx = rtk->nx, *ix = imat(nx, 1), *xi = imat(nx, 1), nc = 0;
  long double *x = rtk->x, *P = rtk->P;
  for (int i = 0; i < nx; i++) {
    if (x[i] != 0.0L && P[i + i * nx] > 0.0L) {
      xi[i] = nc;
      ix[nc++] = i;
    } else
      xi[i] = 0xfffffff;
  }

  long double *Hc = zeros(nc, nv), *v = mat(nv, 1);
  int nv2 = 0, index[MAXSAT];
  for (int m = 0; m < 6; m++)
    for (int f = 0; f < nf; f++) {
      int n = 0;
      for (int i = 0; i < MAXSAT; i++) {
        if (!test_sys(rtk->ssat[i].sys, m) || rtk->ssat[i].fix[f] != 2 ||
            rtk->ssat[i].azel[1] < rtk->opt.elmaskhold) {
          continue;
        }
        index[n++] = IB(i + 1, f, &rtk->opt);
        rtk->ssat[i].fix[f] = 3; /* Hold */
      }
      /* Use ambiguity resolution results to generate a set of pseudo-innovations
         to feed to kalman filter based on error between fixed and float solutions */
      for (int i = 1; i < n; i++) {
        /* Phase-biases are single diff, so subtract errors to get
           long double diff: v(nv)=err(i)-err(0) */
        v[nv2] = (xa[index[0]] - xa[index[i]]) - (rtk->x[index[0]] - rtk->x[index[i]]);
        cvwrite(Hc + nv2 * nc, nc, xi, index[0], 1.0L);
        cvwrite(Hc + nv2 * nc, nc, xi, index[i], -1.0L);
        nv2++;
      }
    }

  rtk->holdamb = 1; /* Set flag to indicate hold has occurred */
  long double *R = zeros(nv, nv);
  for (int i = 0; i < nv; i++) R[i + i * nv] = rtk->opt.varholdamb;

  /* Update states with constraints */

  /* Compress array by removing zero elements to save computation time */
  long double *xc = mat(nc, 1), *Pc = mat(nc, nc), *Ppc = mat(nc, nc);
  for (int i = 0; i < nc; i++) xc[i] = x[ix[i]];
  for (int j = 0; j < nc; j++)
    for (int i = 0; i < nc; i++) Pc[i + j * nc] = P[ix[i] + ix[j] * nx];

  /* Do kalman filter state update on compressed arrays */
  int info = filter_(xc, Pc, Hc, v, R, nc, nv, Ppc);
  if (!info) {
    /* Copy values from compressed arrays back to full arrays */
    for (int i = 0; i < nc; i++) x[ix[i]] = xc[i];
    for (int j = 0; j < nc; j++)
      for (int i = 0; i < nc; i++) P[ix[i] + ix[j] * nx] = Ppc[i + j * nc];
  } else {
    errmsg(rtk, "filter error (info=%d)\n", info);
  }
  free(ix);
  free(xi);
  free(xc);
  free(Pc);
  free(Ppc);
  free(Hc);
  free(R);
  free(v);

  /* Skip GLONASS/SBS icbias update if not enabled  */
  if (rtk->opt.glomodear != GLO_ARMODE_FIXHOLD) return;

  /* Move fractional part of bias from phase-bias into ic bias for GLONASS sats (both in cycles)
   */
  for (int f = 0; f < nf; f++) {
    int i = -1;
    for (int j = 0, nv = 0; j < MAXSAT; j++) {
      /* Check if valid GLONASS sat */
      if (test_sys(rtk->ssat[j].sys, 1) && rtk->ssat[j].vsat[f] && rtk->ssat[j].lock[f] >= 0) {
        if (i < 0) {
          i = j; /* Use first valid sat for reference sat */
          index[nv++] = j;
        } else { /* Adjust the rest */
          /* Find phase-bias difference */
          long double dd = rtk->x[IB(j + 1, f, &rtk->opt)] - rtk->x[IB(i + 1, f, &rtk->opt)];
          dd = rtk->opt.gainholdamb *
               (dd - ROUND(dd)); /* Throwout integer part of answer and multiply by filter gain */
          rtk->x[IB(j + 1, f, &rtk->opt)] -= dd; /* Remove fractional part from phase bias */
          rtk->ssat[j].icbias[f] += dd;          /* and move to IC bias */
          index[nv++] = j;
        }
      }
    }
  }
  /* Move fractional part of bias from phase-bias into ic bias for SBAS sats (both in cycles) */
  for (int f = 0; f < nf; f++) {
    int i = -1;
    for (int j = 0, nv = 0; j < MAXSAT; j++) {
      /* Check if valid GPS/SBS sat */
      if (test_sys(rtk->ssat[j].sys, 0) && rtk->ssat[j].vsat[f] && rtk->ssat[j].lock[f] >= 0) {
        if (i < 0) {
          i = j; /* Use first valid GPS sat for reference sat */
          index[nv++] = j;
        } else { /* Adjust the SBS sats */
          if (rtk->ssat[j].sys != SYS_SBS) continue;
          /* Find phase-bias difference */
          long double dd = rtk->x[IB(j + 1, f, &rtk->opt)] - rtk->x[IB(i + 1, f, &rtk->opt)];
          dd = rtk->opt.gainholdamb *
               (dd - ROUND(dd)); /* Throwout integer part of answer and multiply by filter gain */
          rtk->x[IB(j + 1, f, &rtk->opt)] -= dd; /* Remove fractional part from phase bias diff */
          rtk->ssat[j].icbias[f] += dd;          /* and move to IC bias */
          index[nv++] = j;
        }
      }
    }
  }
}
/* Resolve integer ambiguity by LAMBDA ---------------------------------------*/
static int resamb_LAMBDA(rtk_t *rtk, long double *bias, long double *xa, int gps, int glo,
                         int sbs) {
  int nx = rtk->nx;
  trace(3, "resamb_LAMBDA : nx=%d\n", nx);

  rtk->sol.ratio = 0.0L;
  rtk->nb_ar = 0;
  /* Create index of single to long double-difference transformation matrix (D')
        used to translate phase biases to long double difference */
  int *ix = imat(nx, 2);
  prcopt_t *opt = &rtk->opt;
  int nb = ddidx(rtk, ix, gps, glo, sbs);
  if (nb < (opt->minfixsats - 1)) { /* Nb is sat pairs */
    errmsg(rtk, "not enough valid long double-differences\n");
    free(ix);
    return -1; /* Flag abort */
  }
  rtk->nb_ar = nb;
  /* nx=# of float states, na=# of fixed states, nb=# of long double-diff phase biases */
  int na = rtk->na;
  long double *y = mat(nb, 1), *DP = mat(nb, nx - na), *b = mat(nb, 2), *db = mat(nb, 1);
  long double *Qb = mat(nb, nb), *Qab = mat(na, nb), *QQ = mat(na, nb);

  /* Phase-bias covariance (Qb) and real-parameters to bias covariance (Qab) */
  /* y=D*xc, Qb=D*Qc*D', Qab=Qac*D' */
  for (int i = 0; i < nb; i++) {
    y[i] = rtk->x[ix[i * 2]] - rtk->x[ix[i * 2 + 1]];
  }
  for (int j = 0; j < nx - na; j++)
    for (int i = 0; i < nb; i++) {
      DP[i + j * nb] = rtk->P[ix[i * 2] + (na + j) * nx] - rtk->P[ix[i * 2 + 1] + (na + j) * nx];
    }
  for (int j = 0; j < nb; j++)
    for (int i = 0; i < nb; i++) {
      Qb[i + j * nb] = DP[i + (ix[j * 2] - na) * nb] - DP[i + (ix[j * 2 + 1] - na) * nb];
    }
  for (int j = 0; j < nb; j++)
    for (int i = 0; i < na; i++) {
      Qab[i + j * na] = rtk->P[i + ix[j * 2] * nx] - rtk->P[i + ix[j * 2 + 1] * nx];
    }
#ifdef TRACE
  long double QQb[MAXSAT];
  for (int i = 0; i < nb; i++) QQb[i] = 1000 * Qb[i + i * nb];

  trace(3, "N(0)=     ");
  tracemat(3, y, 1, nb, 7, 2);
  trace(3, "Qb*1000=  ");
  tracemat(3, QQb, 1, nb, 7, 4);
#endif

  /* Lambda/mlambda integer least-square estimation */
  /* Return best integer solutions */
  /* b are best integer solutions, s are residuals */
  long double s[2];
  int info = lambda(nb, 2, y, Qb, b, s);
  if (!info) {
    trace(3, "N(1)=     ");
    tracemat(3, b, 1, nb, 7, 2);
    trace(3, "N(2)=     ");
    tracemat(3, b + nb, 1, nb, 7, 2);

    rtk->sol.ratio = s[0] > 0 ? (s[1] / s[0]) : 0.0L;
    if (rtk->sol.ratio > 999.9L) rtk->sol.ratio = 999.9L;

    /* Adjust AR ratio based on # of sats, unless minAR==maxAR */
    if (opt->thresar[5] != opt->thresar[6]) {
      int nb1 = nb < 50 ? nb : 50; /* Poly only fitted for upto 50 sat pairs */
      /* Generate poly coeffs based on nominal AR ratio */
      long double coeff[3];
      for (int i = 0; i < 3; i++) {
        coeff[i] = ar_poly_coeffs[i][0];
        for (int j = 1; j < 5; j++) coeff[i] = coeff[i] * opt->thresar[0] + ar_poly_coeffs[i][j];
      }
      /* Generate adjusted AR ratio based on # of sat pairs */
      rtk->sol.thres = coeff[0];
      for (int i = 1; i < 3; i++) {
        rtk->sol.thres = rtk->sol.thres * 1 / (nb1 + 1) + coeff[i];
      }
      rtk->sol.thres = MIN(MAX(rtk->sol.thres, opt->thresar[5]), opt->thresar[6]);
    } else
      rtk->sol.thres = opt->thresar[0];
    /* Validation by popular ratio-test of residuals*/
    if (s[0] <= 0.0L || s[1] / s[0] >= rtk->sol.thres) {
      /* Init non phase-bias states and covariances with float solution values */
      /* Transform float to fixed solution (xa=x-Qab*Qb\(b0-b)) */
      for (int i = 0; i < na; i++) {
        rtk->xa[i] = rtk->x[i];
        for (int j = 0; j < na; j++) rtk->Pa[i + j * na] = rtk->P[i + j * nx];
      }
      /* y = differences between float and fixed dd phase-biases
         bias = fixed dd phase-biases   */
      for (int i = 0; i < nb; i++) {
        bias[i] = b[i];
        y[i] -= b[i];
      }
      /* Adjust non phase-bias states and covariances using fixed solution values */
      if (!matinv(Qb, nb)) { /* Returns 0 if inverse successful */
        /* rtk->xa = rtk->x-Qab*Qb^-1*(b0-b) */
        matmul("NN", nb, 1, nb, Qb, y, db);         /* db = Qb^-1*(b0-b) */
        matmulm("NN", na, 1, nb, Qab, db, rtk->xa); /* rtk->xa = rtk->x-Qab*db */

        /* rtk->Pa=rtk->P-Qab*Qb^-1*Qab') */
        /* Covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
        matmul("NN", na, nb, nb, Qab, Qb, QQ);       /* QQ = Qab*Qb^-1 */
        matmulm("NT", na, na, nb, QQ, Qab, rtk->Pa); /* Rtk->Pa = rtk->P-QQ*Qab' */

        trace(3, "resamb : validation ok (nb=%d ratio=%.2Lf thresh=%.2Lf s=%.2Lf/%.2Lf)\n", nb,
              s[0] == 0.0L ? 0.0L : s[1] / s[0], rtk->sol.thres, s[0], s[1]);

        /* Translate long double diff fixed phase-bias values to single diff
        fix phase-bias values, result in xa */
        restamb(rtk, bias, nb, xa);
      } else
        nb = 0;
    } else { /* Validation failed */
      errmsg(rtk, "ambiguity validation failed (nb=%d ratio=%.2Lf thresh=%.2Lf s=%.2Lf/%.2Lf)\n",
             nb, s[1] / s[0], rtk->sol.thres, s[0], s[1]);
      nb = 0;
    }
  } else {
    errmsg(rtk, "lambda error (info=%d)\n", info);
    nb = 0;
  }
  free(ix);
  free(y);
  free(DP);
  free(b);
  free(db);
  free(Qb);
  free(Qab);
  free(QQ);

  return nb; /* Number of ambiguities */
}

/* Resolve integer ambiguity by LAMBDA using partial fix techniques and multiple attempts
 * -----------------------*/
static int manage_amb_LAMBDA(rtk_t *rtk, long double *bias, long double *xa, const int *sat, int nf,
                             int ns) {
  /* Calc position variance, will skip AR if too high to avoid false fix */
  long double posvar = 0;
  for (int i = 0; i < 3; i++) posvar += rtk->P[i + i * rtk->nx];
  posvar /= 3.0L; /* Maintain compatibility with previous code */

  trace(3, "posvar=%.6Lf\n", posvar);
  trace(3, "prevRatios= %.3Lf %.3Lf\n", rtk->sol.prev_ratio1, rtk->sol.prev_ratio2);
  trace(3, "num ambiguities used last AR: %d\n", rtk->nb_ar);

  /* Skip AR if don't meet criteria */
  if (rtk->opt.mode <= PMODE_DGPS || rtk->opt.modear == ARMODE_OFF || rtk->opt.thresar[0] < 1.0L ||
      posvar > rtk->opt.thresar[1]) {
    trace(3, "Skip AR\n");
    rtk->sol.ratio = 0.0L;
    rtk->sol.prev_ratio1 = rtk->sol.prev_ratio2 = 0.0L;
    rtk->nb_ar = 0;
    return 0;
  }
  /* If no fix on previous sample and enough sats, exclude next sat in list */
  int lockc[NFREQ], excflag = 0, arsats[MAXOBS] = {0};
  if (rtk->sol.prev_ratio2 < rtk->sol.thres && rtk->nb_ar >= rtk->opt.mindropsats) {
    /* Find and count sats used last time for AR */
    int ar = 0;
    for (int f = 0; f < nf; f++)
      for (int i = 0; i < ns; i++)
        if (rtk->ssat[sat[i] - 1].vsat[f] && rtk->ssat[sat[i] - 1].lock[f] >= 0 &&
            rtk->ssat[sat[i] - 1].azel[1] >= rtk->opt.elmin) {
          arsats[ar++] = i;
        }
    if (rtk->excsat < ar) {
      int i = sat[arsats[rtk->excsat]];
      for (int f = 0; f < nf; f++) {
        lockc[f] = rtk->ssat[i - 1].lock[f]; /* Save lock count */
        /* Remove sat from AR long enough to enable hold if stays fixed */
        rtk->ssat[i - 1].lock[f] = -rtk->nb_ar;
      }
      trace(3, "AR: exclude sat %d\n", i);
      excflag = 1;
    } else
      rtk->excsat = 0; /* Exclude none and reset to beginning of list */
  }

  /* For inital ambiguity resolution attempt, include all enabled sats */
  int gps1 = 1; /* Always enable GPS for initial pass */
  int glo1 = (rtk->opt.navsys & SYS_GLO)
                 ? (((rtk->opt.glomodear == GLO_ARMODE_FIXHOLD) && !rtk->holdamb) ? 0 : 1)
                 : 0;
  int sbas1 = (rtk->opt.navsys & SYS_GLO) ? glo1 : ((rtk->opt.navsys & SYS_SBS) ? 1 : 0);
  /* First attempt to resolve ambiguities */
  int nb = resamb_LAMBDA(rtk, bias, xa, gps1, glo1, sbas1);
  long double ratio1 = rtk->sol.ratio;
  /* Reject bad satellites if AR filtering enabled */
  if (rtk->opt.arfilter) {
    int rerun = 0;
    /* If results are much poorer than previous epoch or dropped below ar ratio thresh, remove
     * new sats */
    if (nb >= 0 && rtk->sol.prev_ratio2 >= rtk->sol.thres &&
        ((rtk->sol.ratio < rtk->sol.thres) || (rtk->sol.ratio < rtk->opt.thresar[0] * 1.1L &&
                                               rtk->sol.ratio < rtk->sol.prev_ratio1 / 2.0L))) {
      trace(3, "low ratio: check for new sat\n");
      for (int i = 0, dly = 2; i < ns; i++)
        for (int f = 0; f < nf; f++) {
          if (rtk->ssat[sat[i] - 1].fix[f] != 2) continue;
          /* Check for new sats */
          if (rtk->ssat[sat[i] - 1].lock[f] == 0) {
            trace(3, "remove sat %d:%d lock=%d\n", sat[i], f, rtk->ssat[sat[i] - 1].lock[f]);
            rtk->ssat[sat[i] - 1].lock[f] =
                -rtk->opt.minlock - dly; /* Delay use of this sat with stagger */
            dly += 2;                    /* Stagger next try of new sats */
            rerun = 1;
          }
        }
    }
    /* Rerun if filter removed any sats */
    if (rerun) {
      trace(3, "rerun AR with new sats removed\n");
      /* Try again with new sats removed */
      nb = resamb_LAMBDA(rtk, bias, xa, gps1, glo1, sbas1);
    }
  }
  rtk->sol.prev_ratio1 = ratio1;

  /* If fix-and-hold gloarmode enabled, re-run AR with final GPS/GLO settings if differ from above
   */
  if ((rtk->opt.navsys & SYS_GLO) && rtk->opt.glomodear == GLO_ARMODE_FIXHOLD &&
      rtk->sol.ratio < rtk->sol.thres) {
    /* Turn off gpsmode if not enabled and got good fix (used for debug and eval only) */
    int gps2 = rtk->opt.gpsmodear == 0 && rtk->sol.ratio >= rtk->sol.thres ? 0 : 1;
    int glo2 = 0, sbas2 = 0;
    /* If modes changed since initial AR run or haven't run yet,re-run with new modes */
    if (glo1 != glo2 || gps1 != gps2) nb = resamb_LAMBDA(rtk, bias, xa, gps2, glo2, sbas2);
  }
  /* Restore excluded sat if still no fix or significant increase in ar ratio */
  if (excflag && (rtk->sol.ratio < rtk->sol.thres) &&
      (rtk->sol.ratio < (1.5L * rtk->sol.prev_ratio2))) {
    int i = sat[arsats[rtk->excsat++]];
    for (int f = 0; f < nf; f++) rtk->ssat[i - 1].lock[f] = lockc[f];
    trace(3, "AR: restore sat %d\n", i);
  }

  rtk->sol.prev_ratio1 = ratio1 > 0 ? ratio1 : rtk->sol.ratio;
  rtk->sol.prev_ratio2 = rtk->sol.ratio;

  return nb;
}

/* Validation of solution ----------------------------------------------------*/
static bool valpos(rtk_t *rtk, const long double *v, const long double *R, const int *vflg, int nv,
                   long double thres) {
  bool stat = true;

  trace(3, "valpos  : nv=%d thres=%.1Lf\n", nv, thres);

  /* Post-fit residual test */
  long double fact = thres * thres;
  for (int i = 0; i < nv; i++) {
    if (v[i] * v[i] <= fact * R[i + i * nv]) continue;
    int sat1 = (vflg[i] >> 16) & 0xFF;
    int sat2 = (vflg[i] >> 8) & 0xFF;
    int type = (vflg[i] >> 4) & 0xF;
    int freq = vflg[i] & 0xF;
    char *stype = type == 0 ? "L" : (type == 1 ? "P" : "C");
    errmsg(rtk, "large residual (sat=%2d-%2d %s%d v=%6.3Lf sig=%.3Lf)\n", sat1, sat2, stype,
           freq + 1, v[i], SQRTL(R[i + i * nv]));
  }
  return stat;
}
/* Relpos() relative positioning -----------------------------------------------
 *  Args:  rtk      IO      GPS solution structure
           obs      I       satellite observations
           nu       I       # of user observations (rover)
           nr       I       # of ref observations  (base)
           nav      I       satellite navigation data
 */
static bool relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr, const nav_t *nav) {
  prcopt_t *opt = &rtk->opt;
  enum solq stat = opt->mode <= PMODE_DGPS ? SOLQ_DGPS : SOLQ_FLOAT;
  int nf = opt->ionoopt == IONOOPT_IFLC ? 1 : opt->nf;

  /* Time diff between base and rover observations */
  gtime_t time = obs[0].time;
  long double dt = timediff(time, obs[nu].time);
  trace(3, "relpos  : dt=%.3Lf nu=%d nr=%d\n", dt, nu, nr);

  /* Define local matrices, n=total observations, base + rover */
  /* Init satellite status arrays */
  for (int i = 0; i < MAXSAT; i++) {
    rtk->ssat[i].sys = satsys(i + 1, NULL); /* GNSS system */
    for (int j = 0; j < NFREQ; j++) {
      rtk->ssat[i].vsat[j] = 0; /* Valid satellite */
      rtk->ssat[i].snr_rover[j] = 0;
      rtk->ssat[i].snr_base[j] = 0;
    }
  }
  /* Compute satellite positions, velocities and clocks for base and rover */
  int n = nu + nr;
  long double *rs = mat(6, n);  /* Range to satellites */
  long double *dts = mat(2, n); /* Satellite clock biases */
  long double *var = mat(1, n), *y = mat(nf * 2, n), *e = mat(3, n);
  int svh[MAXOBS * 2];
  satposs(time, obs, n, nav, opt->sateph, rs, dts, var, svh);

  /* Calculate [range - measured pseudorange] for base station (phase and code)
       output is in y[nu:nu+nr], see call for rover below for more details */
  trace(3, "base station:\n");
  long double *azel = zeros(2, n); /* [az, el] */
  long double *freq = zeros(nf, n);
  if (!zdres(1, obs + nu, nr, rs + nu * 6, dts + nu * 2, var + nu, svh + nu, nav, rtk->rb, opt,
             y + nu * nf * 2, e + nu * 3, azel + nu * 2, freq + nu * nf)) {
    errmsg(rtk, "initial base station position error\n");

    free(rs);
    free(dts);
    free(var);
    free(y);
    free(e);
    free(azel);
    free(freq);
    return false;
  }
  /* Time-interpolation of base residuals (for post-processing)  */
  if (opt->intpref) {
    dt = intpres(time, obs + nu, nr, nav, rtk, y + nu * nf * 2);
  }
  /* Select common satellites between rover and base-station */
  int sat[MAXSAT], iu[MAXSAT], ir[MAXSAT];
  int ns = selsat(obs, azel, nu, nr, opt, sat, iu, ir);
  if (ns <= 0) {
    errmsg(rtk, "no common satellite\n");

    free(rs);
    free(dts);
    free(var);
    free(y);
    free(e);
    free(azel);
    free(freq);
    return false;
  }
  /* Update kalman filter states (pos,vel,acc,ionosp, troposp, sat phase biases) */
  long double *x = rtk->x;
  trace(4, "before udstate: x=");
  tracemat(4, x, 1, NR(opt), 13, 4);
  udstate(rtk, obs, sat, iu, ir, ns, nav);
  trace(4, "after udstate x=");
  tracemat(4, x, 1, NR(opt), 13, 4);

  for (int i = 0; i < ns; i++)
    for (int j = 0; j < nf; j++) {
      /* SNR of base and rover receiver */
      rtk->ssat[sat[i] - 1].snr_rover[j] = obs[iu[i]].SNR[j];
      rtk->ssat[sat[i] - 1].snr_base[j] = obs[ir[i]].SNR[j];
    }

  /* Backup rtk->x to xp, in case of rollback */
  int nx = rtk->nx;
  long double *xp = mat(nx, 1);
  matcpy(xp, x, nx, 1);

  int ny = ns * nf * 2 + 2; /* Max */
  long double *v = mat(ny, 1), *R = mat(ny, ny), *bias = mat(nx, 1);

  trace(3, "rover:  dt=%.3Lf\n", dt);

  long double *P = rtk->P;

  /* Create list of non-zero states */
  int *ix = imat(nx, 1), *xi = imat(nx, 1), nc = 0;
  /* Always include the first 3 elements, so the compressed array can still
   * be passed as a vector of the 3 axis */
  for (int i = 0; i < 3; i++) {
    xi[i] = nc;
    ix[nc++] = i;
  }
  for (int i = 3; i < nx; i++) {
    if (x[i] != 0.0L && P[i + i * nx] > 0.0L) {
      xi[i] = nc;
      ix[nc++] = i;
    } else {
      xi[i] = 0xfffffff; /* Invalid value >= nc */
    }
  }
  /* Compress array by removing zero elements to save computation time */
  long double *xc = mat(nc, 1), *Pc = mat(nc, nc), *Ppc = mat(nc, nc);
  for (int i = 0; i < nc; i++) xc[i] = x[ix[i]];
  for (int j = 0; j < nc; j++)
    for (int i = 0; i < nc; i++) Pc[i + j * nc] = P[ix[i] + ix[j] * nx];

  /* Know nc<=nx now, allocate H with this known number of rows and the max possible columns. */
  long double *Hc = mat(nc, ny);

  for (int i = 0; i < opt->niter; i++) {
    /* Calculate zero diff residuals [range - measured pseudorange] for rover (phase and code)
        output is in y[0:nu-1], only shared input with base is nav
            obs  = sat observations
            nu   = # of sats
            rs   = range to sats
            dts  = sat clock biases (rover)
            svh  = sat health flags
            nav  = sat nav data
            x    = kalman states
            opt  = options
            y    = zero diff residuals (code and phase)
            e    = line of sight unit vectors to sats
            azel = [az, el] to sats                                   */
    if (!zdres(0, obs, nu, rs, dts, var, svh, nav, x, opt, y, e, azel, freq)) {
      errmsg(rtk, "rover initial position error\n");
      stat = SOLQ_NONE;
      break;
    }
    /* Calculate long double-differenced residuals and create state matrix from sat angles
            O rtk->ssat[i].resp[j] = residual pseudorange error
            O rtk->ssat[i].resc[j] = residual carrier phase error
            I dt = time diff between base and rover observations
            I Pp = covariance matrix of float solution
            I sat = list of common sats
            I iu,ir = user and ref indices to sats
            I ns = # of sats
            O v = long double diff residuals (phase and code)
            O H = partial derivatives
            O R = long double diff measurement error covariances
            O vflg = list of sats used for dd  */
    int vflg[MAXOBS * NFREQ * 2 + 1];
    int nv =
        ddres(rtk, obs, dt, x, P, Pc, nc, xi, sat, y, e, azel, freq, iu, ir, ns, v, Hc, R, vflg);
    if (nv < 4) {
      errmsg(rtk, "not enough long double-differenced residual, n=%d\n", nv);
      stat = SOLQ_NONE;
      break;
    }
    /* Kalman filter measurement update, updates x,y,z,sat phase biases, etc
            K=P*H*(H'*P*H+R)^-1
            xp=x+K*v
            Pp=(I-K*H')*P                  */
    trace(3, "before filter x=");
    tracemat(3, x, 1, 9, 13, 6);
    /*  Do kalman filter state update on compressed arrays */
    int info = filter_(xc, Pc, Hc, v, R, nc, nv, Ppc);
    if (info) {
      errmsg(rtk, "filter error (info=%d)\n", info);
      stat = SOLQ_NONE;
      break;
    }
    /* Copy values from compressed vector xc back to full vector rtk->x */
    for (int ic = 0; ic < nc; ic++) x[ix[ic]] = xc[ic];
    long double *Pt = Pc;
    Pc = Ppc;
    Ppc = Pt; /* Swap Pc and Ppc */
    trace(3, "after filter x=");
    tracemat(3, x, 1, 9, 13, 6);
    trace(4, "x(%d)=", i + 1);
    tracemat(4, x, 1, NR(opt), 13, 4);
  }
  free(xc);
  free(Ppc);
  free(Hc);
  /* Calc zero diff residuals again after kalman filter update */
  if (stat != SOLQ_NONE && zdres(0, obs, nu, rs, dts, var, svh, nav, x, opt, y, e, azel, freq)) {
    /* Calc double diff residuals again after kalman filter update for float solution */
    int vflg[MAXOBS * NFREQ * 2 + 1];
    int nv =
        ddres(rtk, obs, dt, x, P, Pc, nc, xi, sat, y, e, azel, freq, iu, ir, ns, v, NULL, R, vflg);

    /* Validation of float solution, always returns 1, msg to trace file if large residual */
    if (valpos(rtk, v, R, vflg, nv, 4.0L)) {
      /* Copy values from compressed array Pc back to full array rtk->P. */
      for (int j = 0; j < nc; j++)
        for (int i = 0; i < nc; i++) P[ix[i] + ix[j] * nx] = Pc[i + j * nc];
      /* The rtk->x vector is written in place and restored below from
       * xp if this path is not taken. */
      free(xp);
      xp = NULL;

      /* Update valid satellite status for ambiguity control */
      rtk->sol.ns = 0;
      for (int i = 0; i < ns; i++)
        for (int f = 0; f < nf; f++) {
          if (!rtk->ssat[sat[i] - 1].vsat[f]) continue;
          rtk->ssat[sat[i] - 1].outc[f] = 0;
          if (f == 0) rtk->sol.ns++; /* Valid satellite count by L1 */
        }
      /* Too few valid phases */
      if (rtk->sol.ns < 4) stat = SOLQ_DGPS;
    } else
      stat = SOLQ_NONE;
  }

  free(ix);

  if (xp) {
    /* Restore rtk->x from xp */
    matcpy(rtk->x, xp, nx, 1);
    free(xp);
  }

  /* Resolve integer ambiguity by LAMBDA */
  if (stat == SOLQ_FLOAT) {
    /* Initialize xa to zero */
    long double *xa = mat(nx, 1);

    /* If valid fixed solution, process it */
    if (manage_amb_LAMBDA(rtk, bias, xa, sat, nf, ns) > 1) {
      /* Find zero-diff residuals for fixed solution */
      if (zdres(0, obs, nu, rs, dts, var, svh, nav, xa, opt, y, e, azel, freq)) {
        /* Post-fit residuals for fixed solution (xa includes fixed phase biases, rtk->xa
         * does not) */
        int vflg[MAXOBS * NFREQ * 2 + 1];
        int nv = ddres(rtk, obs, dt, xa, P, Pc, nc, xi, sat, y, e, azel, freq, iu, ir, ns, v, NULL,
                       R, vflg);

        /* Validation of fixed solution, always returns valid */
        if (valpos(rtk, v, R, vflg, nv, 4.0L)) {
          /* Hold integer ambiguity if meet minfix count */
          if (++rtk->nfix >= rtk->opt.minfix) {
            if (rtk->opt.modear == ARMODE_FIXHOLD || rtk->opt.glomodear == GLO_ARMODE_FIXHOLD)
              holdamb(rtk, xa);
            /* Switch to kinematic after qualify for hold if in static-start mode */
            if (rtk->opt.mode == PMODE_STATIC_START) {
              rtk->opt.mode = PMODE_KINEMA;
              trace(3, "Fix and hold complete: switch to kinematic mode\n");
            }
          }
          stat = SOLQ_FIX;
        }
      }
    }
    free(xa);
  }

  free(xi);
  free(Pc);

  /* Save solution status (fixed or float) */
  if (stat == SOLQ_FIX) {
    for (int i = 0; i < 3; i++) {
      rtk->sol.rr[i] = rtk->xa[i];
      rtk->sol.qr[i] = rtk->Pa[i + i * rtk->na];
    }
    rtk->sol.qr[3] = rtk->Pa[1];
    rtk->sol.qr[4] = rtk->Pa[1 + 2 * rtk->na];
    rtk->sol.qr[5] = rtk->Pa[2];

    if (rtk->opt.dynamics) { /* Velocity and covariance */
      for (int i = 3; i < 6; i++) {
        rtk->sol.rr[i] = rtk->xa[i];
        rtk->sol.qv[i - 3] = rtk->Pa[i + i * rtk->na];
      }
      rtk->sol.qv[3] = rtk->Pa[4 + 3 * rtk->na];
      rtk->sol.qv[4] = rtk->Pa[5 + 4 * rtk->na];
      rtk->sol.qv[5] = rtk->Pa[5 + 3 * rtk->na];
    }
  } else { /* Float solution */
    for (int i = 0; i < 3; i++) {
      rtk->sol.rr[i] = rtk->x[i];
      rtk->sol.qr[i] = rtk->P[i + i * nx];
    }
    rtk->sol.qr[3] = rtk->P[1];
    rtk->sol.qr[4] = rtk->P[1 + 2 * nx];
    rtk->sol.qr[5] = rtk->P[2];

    if (rtk->opt.dynamics) { /* Velocity and covariance */
      for (int i = 3; i < 6; i++) {
        rtk->sol.rr[i] = rtk->x[i];
        rtk->sol.qv[i - 3] = rtk->P[i + i * nx];
      }
      rtk->sol.qv[3] = rtk->P[4 + 3 * nx];
      rtk->sol.qv[4] = rtk->P[5 + 4 * nx];
      rtk->sol.qv[5] = rtk->P[5 + 3 * nx];
    }
    rtk->nfix = 0;
  }
  trace(3, "sol_rr= ");
  tracemat(3, rtk->sol.rr, 1, 6, 15, 3);
  /* Save phase measurements */
  for (int i = 0; i < n; i++)
    for (int j = 0; j < nf; j++) {
      if (obs[i].L[j] == 0.0L) continue;
      rtk->ssat[obs[i].sat - 1].pt[obs[i].rcv - 1][j] = obs[i].time;
      rtk->ssat[obs[i].sat - 1].ph[obs[i].rcv - 1][j] = obs[i].L[j];
    }
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < nf; j++) {
      /* Don't lose track of which sats were used to try and resolve the ambiguities */
      /* if (rtk->ssat[i].fix[j]==2&&stat!=SOLQ_FIX) rtk->ssat[i].fix[j]=1; */
      if (rtk->ssat[i].slip[j] & 1) rtk->ssat[i].slipc[j]++;
      /* Inc lock count if this sat used for good fix */
      if (!rtk->ssat[i].vsat[j]) continue;
      if (rtk->ssat[i].lock[j] < 0 || (rtk->nfix > 0 && rtk->ssat[i].fix[j] >= 2))
        rtk->ssat[i].lock[j]++;
    }
  free(rs);
  free(dts);
  free(var);
  free(y);
  free(e);
  free(azel);
  free(freq);
  free(v);
  free(R);
  free(bias);

  if (stat != SOLQ_NONE) rtk->sol.stat = stat;

  return stat != SOLQ_NONE;
}
/* Initialize RTK control ------------------------------------------------------
 * Initialize RTK control struct
 * Args   : rtk_t    *rtk    IO  TKk control/result struct
 *          prcopt_t *opt    I   positioning options (see rtklib.h)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void rtkinit(rtk_t *rtk, const prcopt_t *opt) {
  trace(3, "rtkinit :\n");

  sol_t sol0 = {{0}};
  rtk->sol = sol0;
  for (int i = 0; i < 6; i++) rtk->rb[i] = 0.0L;
  rtk->nx = opt->mode <= PMODE_FIXED ? NX(opt) : pppnx(opt);
  rtk->na = opt->mode <= PMODE_FIXED ? NR(opt) : pppnx(opt);
  rtk->tt = 0.0L;
  rtk->epoch = 0;
  rtk->x = zeros(rtk->nx, 1);
  rtk->P = zeros(rtk->nx, rtk->nx);
  rtk->xa = zeros(rtk->na, 1);
  rtk->Pa = zeros(rtk->na, rtk->na);
  rtk->nfix = rtk->neb = 0;
  ambc_t ambc0 = {{{0}}};
  ssat_t ssat0 = {0};
  for (int i = 0; i < MAXSAT; i++) {
    rtk->ambc[i] = ambc0;
    rtk->ssat[i] = ssat0;
  }
  rtk->holdamb = 0;
  rtk->excsat = 0;
  rtk->nb_ar = 0;
  for (int i = 0; i < MAXERRMSG; i++) rtk->errbuf[i] = 0;
  rtk->opt = *opt;
  rtk->initial_mode = rtk->opt.mode;
  rtk->sol.thres = opt->thresar[0];
}
/* Free rtk control ------------------------------------------------------------
 * Free memory for rtk control struct
 * Args   : rtk_t    *rtk    IO  rtk control/result struct
 * Return : none
 *----------------------------------------------------------------------------*/
extern void rtkfree(rtk_t *rtk) {
  trace(3, "rtkfree :\n");

  rtk->nx = rtk->na = 0;
  free(rtk->x);
  rtk->x = NULL;
  free(rtk->P);
  rtk->P = NULL;
  free(rtk->xa);
  rtk->xa = NULL;
  free(rtk->Pa);
  rtk->Pa = NULL;
}
/* Precise positioning ---------------------------------------------------------
 * Input observation data and navigation message, compute rover position by
 * Precise positioning
 * Args   : rtk_t *rtk       IO  RTK control/result struct
 *            rtk->sol       IO  solution
 *                .time      O   solution time
 *                .rr[]      IO  rover position/velocity
 *                               (I:fixed mode,O:single mode)
 *                .dtr[0]    O   receiver clock bias (s)
 *                .dtr[1-5]  O   receiver GLO/GAL/BDS/IRN/QZS-GPS time offset (s)
 *                .Qr[]      O   rover position covarinace
 *                .stat      O   solution status (SOLQ_???)
 *                .ns        O   number of valid satellites
 *                .age       O   age of differential (s)
 *                .ratio     O   ratio factor for ambiguity validation
 *            rtk->rb[]      IO  base station position/velocity
 *                               (I:relative mode,O:moving-base mode)
 *            rtk->nx        I   number of all states
 *            rtk->na        I   number of integer states
 *            rtk->ns        O   number of valid satellites in use
 *            rtk->tt        O   time difference between current and previous (s)
 *            rtk->x[]       IO  float states pre-filter and post-filter
 *            rtk->P[]       IO  float covariance pre-filter and post-filter
 *            rtk->xa[]      O   fixed states after AR
 *            rtk->Pa[]      O   fixed covariance after AR
 *            rtk->ssat[s]   IO  satellite {s+1} status
 *                .sys       O   system (SYS_???)
 *                .az   [r]  O   azimuth angle   (rad) (r=0:rover,1:base)
 *                .el   [r]  O   elevation angle (rad) (r=0:rover,1:base)
 *                .vs   [r]  O   data valid single     (r=0:rover,1:base)
 *                .resp [f]  O   freq(f+1) pseudorange residual (m)
 *                .resc [f]  O   freq(f+1) carrier-phase residual (m)
 *                .vsat [f]  O   freq(f+1) data vaild (0:invalid,1:valid)
 *                .fix  [f]  O   freq(f+1) ambiguity flag
 *                               (0:nodata,1:float,2:fix,3:hold)
 *                .slip [f]  O   freq(f+1) cycle slip flag
 *                               (bit8-7:rcv1 LLI, bit6-5:rcv2 LLI,
 *                                bit2:parity unknown, bit1:slip)
 *                .lock [f]  IO  freq(f+1) carrier lock count
 *                .outc [f]  IO  freq(f+1) carrier outage count
 *                .slipc[f]  IO  freq(f+1) cycle slip count
 *                .rejc [f]  IO  freq(f+1) data reject count
 *                .gf        IO  geometry-free phase (L1-L2 or L1-L5) (m)
 *            rtk->nfix      IO  number of continuous fixes of ambiguity
 *            rtk->neb       IO  bytes of error message buffer
 *            rtk->errbuf    IO  error message buffer
 *            rtk->tstr      O   time string for debug
 *            rtk->opt       I   processing options
 *          obsd_t *obs      I   observation data for an epoch
 *                               obs[i].rcv=1:rover,2:reference
 *                               sorted by receiver and satellte
 *          int    n         I   number of observation data
 *          nav_t  *nav      I   navigation messages
 * Return : status (true:valid solution,false:no solution)
 * Notes  : before calling function, base station position rtk->sol.rb[] should
 *          be properly set for relative mode except for moving-baseline
 *----------------------------------------------------------------------------*/
extern bool rtkpos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav) {
  char tstr[40];
  trace(3, "rtkpos  : time=%s n=%d\n", time2str(obs[0].time, tstr, 3), n);
  trace(4, "obs=\n");
  traceobs(4, obs, n);
  /*trace(5,"nav=\n"); tracenav(5,nav);*/

  /* Set base station position */
  prcopt_t *opt = &rtk->opt;
  if (opt->refpos <= POSOPT_RINEX && opt->mode != PMODE_SINGLE && opt->mode != PMODE_MOVEB) {
    for (int i = 0; i < 6; i++) rtk->rb[i] = i < 3 ? opt->rb[i] : 0.0L;
  }
  /* Count rover/base station observations */
  int nu;
  for (nu = 0; nu < n && obs[nu].rcv == 1; nu++)
    ;
  int nr;
  for (nr = 0; nu + nr < n && obs[nu + nr].rcv == 2; nr++)
    ;

  gtime_t time = rtk->sol.time; /* Previous epoch */

  /* Rover position and time by single point positioning, skip if
   position variance smaller than threshold */
  if (rtk->P[0] == 0 || rtk->P[0] > STD_PREC_VAR_THRESH) {
    char msg[128] = "";
    msg[0] = '\0';
    if (!pntpos(obs, nu, nav, &rtk->opt, &rtk->sol, NULL, rtk->ssat, msg, sizeof(msg))) {
      errmsg(rtk, "point pos error (%s)\n", msg);

      if (!rtk->opt.dynamics) {
        outsolstat(rtk, nav);
        return false;
      }
    }
  } else
    rtk->sol.time = obs[0].time;
  if (time.time != 0) rtk->tt = timediff(rtk->sol.time, time);

  /* Return to static start if long delay without rover data */
  if (fabsl(rtk->tt) > 300 && rtk->initial_mode == PMODE_STATIC_START) {
    rtk->opt.mode = PMODE_STATIC_START;
    for (int i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
    if (rtk->opt.dynamics) {
      for (int i = 3; i < 6; i++) initx(rtk, 1E-6L, VAR_VEL, i);
      for (int i = 6; i < 9; i++) initx(rtk, 1E-6L, VAR_ACC, i);
    }
    trace(3, "No data for > 5 min: switch back to static mode:\n");
  }

  /* Single point positioning */
  if (opt->mode == PMODE_SINGLE) {
    outsolstat(rtk, nav);
    return true;
  }
  /* Suppress output of single solution */
  if (!opt->outsingle) {
    rtk->sol.stat = SOLQ_NONE;
  }
  /* Precise point positioning */
  if (opt->mode >= PMODE_PPP_KINEMA) {
    pppos(rtk, obs, nu, nav);
    outsolstat(rtk, nav);
    return true;
  }
  /* Check number of data of base station and age of differential */
  if (nr == 0) {
    errmsg(rtk, "no base station observation data for rtk\n");
    outsolstat(rtk, nav);
    return true;
  }
  if (opt->mode == PMODE_MOVEB) { /*  Moving baseline */
    /* Estimate position/velocity of base station,
       skip if position varinace below threshold*/
    sol_t solb = {{0}};
    if (rtk->P[0] == 0 || rtk->P[0] > STD_PREC_VAR_THRESH) {
      char msg[128] = "";
      msg[0] = '\0';
      if (!pntpos(obs + nu, nr, nav, &rtk->opt, &solb, NULL, NULL, msg, sizeof(msg))) {
        errmsg(rtk, "base station position error (%s)\n", msg);
        return false;
      }
      /* If base position uninitialized, use full position */
      if (fabsl(rtk->rb[0]) < 0.1L)
        for (int i = 0; i < 3; i++) rtk->rb[i] = solb.rr[i];
      /* Else filter base position to reduce noise from single precision solution */
      else
        for (int i = 0; i < 3; i++) {
          rtk->rb[i] = 0.95L * rtk->rb[i] + 0.05L * solb.rr[i];
          rtk->rb[i + 3] = 0; /* Set velocity to zero */
        }
    } else
      solb.time = obs[nu].time;
    trace(3, "basex= %.3Lf %.3Lf\n", rtk->rb[0], solb.rr[0]);

    rtk->sol.age = timediff(rtk->sol.time, solb.time);

    if (fabsl(rtk->sol.age) > MIN(TTOL_MOVEB, opt->maxtdiff)) {
      errmsg(rtk, "time sync error for moving-base (age=%.1Lf)\n", rtk->sol.age);
      return false;
    }

    /* Time-synchronized position of base station */
    /* Single position velocity solution too noisy to be helpful */
    /*for (i=0;i<3;i++) rtk->rb[i]+=rtk->rb[i+3]*rtk->sol.age; */

    trace(3, "base pos: ");
    tracemat(3, rtk->rb, 1, 3, 13, 4);
  } else {
    rtk->sol.age = timediff(obs[0].time, obs[nu].time);

    if (fabsl(rtk->sol.age) > opt->maxtdiff) {
      errmsg(rtk, "age of differential error (age=%.1Lf)\n", rtk->sol.age);
      outsolstat(rtk, nav);
      return true;
    }
  }
  /* Relative potitioning */
  relpos(rtk, obs, nu, nr, nav);
  rtk->epoch++;
  outsolstat(rtk, nav);

  return true;
}
