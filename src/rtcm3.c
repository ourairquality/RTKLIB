/*------------------------------------------------------------------------------
 * rtcm3.c : RTCM ver.3 message decorder functions
 *
 *          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
 *
 * References :
 *     see rtcm.c
 *
 * Version : $Revision:$ $Date:$
 * History : 2012/05/14 1.0  separated from rtcm.c
 *           2012/12/12 1.1  support GAL/QZS ephemeris, GAL/QZS SSR, MSM
 *                           add station id consistency test for obs data
 *           2012/12/25 1.2  change compass MSM id table
 *           2013/01/31 1.3  change signal id by the latest draft (ref [13])
 *           2013/02/23 1.4  change reference for RTCM 3 message (ref [14])
 *           2013/05/19 1.5  GPST -> BDT of time-tag in BeiDou MSM message
 *           2014/05/02 1.6  fix bug on dropping last field of SSR message
 *                           comply with RTCM 3.2 with amendment 1/2 (ref[15])
 *                           delete MT 1046 according to ref [15]
 *           2014/09/14 1.7  add receiver option -RT_INP
 *           2014/12/06 1.8  support SBAS/BeiDou SSR messages (ref [16])
 *           2015/03/22 1.9  add handling of iodcrc for BeiDou/SBAS SSR messages
 *           2015/04/27 1.10 support phase bias messages (MT2065-2070)
 *           2015/09/07 1.11 add message count of MT 2000-2099
 *           2015/10/21 1.12 add MT1046 support for IGS MGEX
 *                           fix bug on decode of SSR 3/7 (code/phase bias)
 *           2015/12/04 1.13 add MT63 BeiDou ephemeris (RTCM draft)
 *                           fix bug on SSR 3 message decoding (#321)
 *           2016/01/22 1.14 fix bug on L2C code in MT1004 (#131)
 *           2016/08/20 1.15 fix bug on loss-of-lock detection in MSM 6/7 (#134)
 *           2016/09/20 1.16 fix bug on MT1045 Galileo week rollover
 *           2016/10/09 1.17 support MT1029 unicode text string
 *           2017/04/11 1.18 fix bug on unchange-test of BeiDou ephemeris
 *                           fix bug on week number in Galileo ephemeris struct
 *           2018/10/10 1.19 merge changes for 2.4.2 p13
 *                           fix problem on eph.code for Galileo ephemeris
 *                           change mt for SSR 7 phase biases
 *                           add RTCM option -GALINAV, -GALFNAV
 *           2018/11/05 1.20 fix problem on invalid time in message monitor
 *           2019/05/10 1.21 save Galileo E5b data to obs index 2
 *           2020/11/30 1.22 support MT1230 GLONASS code-phase biases
 *                           support MT1131-1137,1041 (NavIC MSM and ephemeris)
 *                           support MT4076 IGS SSR
 *                           update MSM signal ID table (ref [17])
 *                           update SSR signal and tracking mode ID table
 *                           add week adjustment in MT1019,1044,1045,1046,1042
 *                           use API code2idx() to get freq-index
 *                           use API code2freq() to get carrier frequency
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Constants -----------------------------------------------------------------*/

#define PRUNIT_GPS 299792.458     /* RTCM ver.3 unit of GPS pseudorange (m) */
#define PRUNIT_GLO 599584.916     /* RTCM ver.3 unit of GLONASS pseudorange (m) */
#define RANGE_MS (CLIGHT * 0.001) /* Range in 1 ms */

#define P2_10 0.0009765625          /* 2^-10 */
#define P2_28 3.725290298461914E-09 /* 2^-28 */
#define P2_34 5.820766091346740E-11 /* 2^-34 */
#define P2_41 4.547473508864641E-13 /* 2^-41 */
#define P2_46 1.421085471520200E-14 /* 2^-46 */
#define P2_59 1.734723475976810E-18 /* 2^-59 */
#define P2_66 1.355252715606880E-20 /* 2^-66 */

/* Type definition -----------------------------------------------------------*/

typedef struct {        /* Multi-signal-message header type */
  uint8_t iod;          /* Issue of data station */
  uint8_t time_s;       /* Cumulative session transmitting time */
  uint8_t clk_str;      /* Clock steering indicator */
  uint8_t clk_ext;      /* External clock indicator */
  uint8_t smooth;       /* Divergence free smoothing indicator */
  uint8_t tint_s;       /* Soothing interval */
  uint8_t nsat, nsig;   /* Number of satellites/signals */
  uint8_t sats[64];     /* Satellites */
  uint8_t sigs[32];     /* Signals */
  uint8_t cellmask[64]; /* Cell mask */
} msm_h_t;

/* MSM signal ID table -------------------------------------------------------*/
const char *msm_sig_gps[32] = {
    /* GPS: ref [17] table 3.5-91 */
    "", "1C", "1P", "1W", "",   "",   "",   "2C", "2P", "2W", "",   "",   /*  1-12 */
    "", "",   "2S", "2L", "2X", "",   "",   "",   "",   "5I", "5Q", "5X", /* 13-24 */
    "", "",   "",   "",   "",   "1S", "1L", "1X"                          /* 25-32 */
};
const char *msm_sig_glo[32] = {
    /* GLONASS: ref [17] table 3.5-96 */
    "", "1C", "1P", "", "", "", "", "2C", "2P", "", "", "", "", "", "", "",
    "", "",   "",   "", "", "", "", "",   "",   "", "", "", "", "", "", ""};
const char *msm_sig_gal[32] = {
    /* Galileo: ref [17] table 3.5-99 */
    "", "1C", "1A", "1B", "1X", "1Z", "",   "6C", "6A", "6B", "6X", "6Z", "", "7I", "7Q", "7X",
    "", "8I", "8Q", "8X", "",   "5I", "5Q", "5X", "",   "",   "",   "",   "", "",   "",   ""};
const char *msm_sig_qzs[32] = {
    /* QZSS: ref [17] table 3.5-105 */
    "",   "1C", "", "", "", "",   "",   "",   "6S", "6L", "6X", "", "", "",   "2S", "2L",
    "2X", "",   "", "", "", "5I", "5Q", "5X", "",   "",   "",   "", "", "1S", "1L", "1X"};
const char *msm_sig_sbs[32] = {
    /* SBAS: ref [17] table 3.5-102 */
    "", "1C", "", "", "", "",   "",   "",   "", "", "", "", "", "", "", "",
    "", "",   "", "", "", "5I", "5Q", "5X", "", "", "", "", "", "", "", ""};
const char *msm_sig_cmp[32] = {
    /* BeiDou: ref [17] table 3.5-108 */
    "", "2I", "2Q", "2X", "", "",   "",   "6I", "6Q", "6X", "", "", "", "7I", "7Q", "7X",
    "", "",   "",   "",   "", "5D", "5P", "5X", "7D", "",   "", "", "", "1D", "1P", "1X"};
const char *msm_sig_irn[32] = {
    /* NavIC/IRNSS: ref [17] table 3.5-108.3 */
    "", "", "", "", "", "",   "", "", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "5A", "", "", "", "", "", "", "", "", "", ""};
/* SSR signal and tracking mode IDs ------------------------------------------*/
const uint8_t ssr_sig_gps[32] = {CODE_L1C, CODE_L1P, CODE_L1W, CODE_L1S, CODE_L1L, CODE_L2C,
                                 CODE_L2D, CODE_L2S, CODE_L2L, CODE_L2X, CODE_L2P, CODE_L2W,
                                 0,        0,        CODE_L5I, CODE_L5Q};
const uint8_t ssr_sig_glo[32] = {CODE_L1C, CODE_L1P, CODE_L2C, CODE_L2P, CODE_L4A,
                                 CODE_L4B, CODE_L6A, CODE_L6B, CODE_L3I, CODE_L3Q};
const uint8_t ssr_sig_gal[32] = {CODE_L1A, CODE_L1B, CODE_L1C, 0,        0,       CODE_L5I,
                                 CODE_L5Q, 0,        CODE_L7I, CODE_L7Q, 0,       CODE_L8I,
                                 CODE_L8Q, 0,        CODE_L6A, CODE_L6B, CODE_L6C};
const uint8_t ssr_sig_qzs[32] = {CODE_L1C, CODE_L1S, CODE_L1L, CODE_L2S, CODE_L2L, 0,
                                 CODE_L5I, CODE_L5Q, 0,        CODE_L6S, CODE_L6L, 0,
                                 0,        0,        0,        0,        0,        CODE_L6E};
const uint8_t ssr_sig_cmp[32] = {
    CODE_L2I, CODE_L2Q, 0,        CODE_L6I, CODE_L6Q, 0, CODE_L7I, CODE_L7Q, 0, CODE_L1D, CODE_L1P,
    0,        CODE_L5D, CODE_L5P, 0,        CODE_L1A, 0, 0,        CODE_L6A};
const uint8_t ssr_sig_sbs[32] = {CODE_L1C, CODE_L5I, CODE_L5Q};
/* SSR update intervals ------------------------------------------------------*/
static const double ssrudint[16] = {1,   2,   5,   10,  15,   30,   60,   120,
                                    240, 300, 600, 900, 1800, 3600, 7200, 10800};

static uint32_t rtcm_getbitu(const rtcm_t *rtcm, unsigned pos, unsigned len) {
  return getbitu(rtcm->buff, sizeof(rtcm->buff), pos, len);
}
static int32_t rtcm_getbits(const rtcm_t *rtcm, unsigned pos, unsigned len) {
  return getbits(rtcm->buff, sizeof(rtcm->buff), pos, len);
}
/* Get sign-magnitude bits ---------------------------------------------------*/
static double rtcm_getbitg(const rtcm_t *rtcm, int pos, int len) {
  double value = rtcm_getbitu(rtcm, pos + 1, len - 1);
  return rtcm_getbitu(rtcm, pos, 1) ? -value : value;
}
/* Adjust weekly rollover of GPS time ----------------------------------------*/
static void adjweek(rtcm_t *rtcm, double tow) {
  /* If no time, get cpu time */
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  int week;
  double tow_p = time2gpst(rtcm->time, &week);
  if (tow < tow_p - 302400.0)
    tow += 604800.0;
  else if (tow > tow_p + 302400.0)
    tow -= 604800.0;
  rtcm->time = gpst2time(week, tow);
}
/* Adjust weekly rollover of BDS time ----------------------------------------*/
static int adjbdtweek(int week) {
  int w;
  (void)time2bdt(gpst2bdt(utc2gpst(timeget())), &w);
  if (w < 1) w = 1; /* Use 2006/1/1 if time is earlier than 2006/1/1 */
  return week + (w - week + 512) / 1024 * 1024;
}
/* Adjust daily rollover of GLONASS time -------------------------------------*/
static void adjday_glot(rtcm_t *rtcm, double tod) {
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  gtime_t time = timeadd(gpst2utc(rtcm->time), 10800.0); /* GLONASS time */
  int week;
  double tow = time2gpst(time, &week);
  double tod_p = fmod(tow, 86400.0);
  tow -= tod_p;
  if (tod < tod_p - 43200.0)
    tod += 86400.0;
  else if (tod > tod_p + 43200.0)
    tod -= 86400.0;
  time = gpst2time(week, tow + tod);
  rtcm->time = utc2gpst(timeadd(time, -10800.0));
}
/* Adjust carrier-phase rollover ---------------------------------------------*/
static double adjcp(rtcm_t *rtcm, int sat, int idx, double cp) {
  if (rtcm->cp[sat - 1][idx] == 0.0)
    ;
  else if (cp < rtcm->cp[sat - 1][idx] - 750.0)
    cp += 1500.0;
  else if (cp > rtcm->cp[sat - 1][idx] + 750.0)
    cp -= 1500.0;
  rtcm->cp[sat - 1][idx] = cp;
  return cp;
}
/* Loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_t *rtcm, int sat, int idx, int lock) {
  int lli = (!lock && !rtcm->lock[sat - 1][idx]) || lock < rtcm->lock[sat - 1][idx];
  rtcm->lock[sat - 1][idx] = (uint16_t)lock;
  return lli;
}
/* S/N ratio -----------------------------------------------------------------*/
static uint16_t snratio(double snr) {
  return (uint16_t)(snr <= 0.0 || 100.0 <= snr ? 0.0 : snr / SNR_UNIT + 0.5);
}
/* Get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat) {
  int i = 0;
  for (; i < obs->n; i++) {
    if (obs->data[i].sat == sat) return i; /* Field already exists */
  }
  if (i >= MAXOBS) return -1; /* Overflow */

  /* Add new field */
  obs->data[i].time = time;
  obs->data[i].sat = sat;
  for (int j = 0; j < NFREQ + NEXOBS; j++) {
    obs->data[i].L[j] = obs->data[i].P[j] = 0.0;
    obs->data[i].D[j] = 0.0;
    obs->data[i].SNR[j] = obs->data[i].LLI[j] = obs->data[i].code[j] = 0;
  }
  obs->n++;
  return i;
}
/* Test station ID consistency -----------------------------------------------*/
static bool test_staid(rtcm_t *rtcm, int staid) {
  /* Test station id option */
  char *p = strstr(rtcm->opt, "-STA=");
  if (p) {
    int id;
    if (sscanf(p, "-STA=%d", &id) == 1) {
      if (staid != id) return false;
    }
  }
  /* Save station id */
  if (rtcm->staid == 0 || rtcm->obsflag) {
    rtcm->staid = staid;
  } else if (staid != rtcm->staid) {
    int type = rtcm_getbitu(rtcm, 24, 12);
    trace(2, "rtcm3 %d staid invalid id=%d %d\n", type, staid, rtcm->staid);

    /* Reset station id if station id error */
    rtcm->staid = 0;
    return false;
  }
  return true;
}
/* Decode type 1001-1004 message header --------------------------------------*/
static int decode_head1001(rtcm_t *rtcm, int *sync) {
  int i = 24;
  int type = rtcm_getbitu(rtcm, i, 12);
  i += 12;

  if (i + 52 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12;
  double tow = rtcm_getbitu(rtcm, i, 30) * 0.001;
  i += 30;
  *sync = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  int nsat = rtcm_getbitu(rtcm, i, 5);
  /* Test station ID */
  if (!test_staid(rtcm, staid)) return -1;

  adjweek(rtcm, tow);

  char tstr[40];
  time2str(rtcm->time, tstr, 2);
  trace(4, "decode_head1001: time=%s nsat=%d sync=%d\n", tstr, nsat, *sync);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d %s nsat=%2d sync=%d", staid,
                 tstr, nsat, *sync);
  }
  return nsat;
}
/* Decode type 1001: L1-only GPS RTK observation -----------------------------*/
static int decode_type1001(rtcm_t *rtcm) {
  int sync;
  if (decode_head1001(rtcm, &sync) < 0) return -1;
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 1002: extended L1-only GPS RTK observables --------------------*/
static int decode_type1002(rtcm_t *rtcm) {
  int sync;
  int nsat = decode_head1001(rtcm, &sync);
  if (nsat < 0) return -1;

  int i = 24 + 64;
  for (int j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 74 <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, 6);
    i += 6;
    int code = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    double pr1 = rtcm_getbitu(rtcm, i, 24);
    i += 24;
    int ppr1 = rtcm_getbits(rtcm, i, 20);
    i += 20;
    int lock1 = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    int amb = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    double cnr1 = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    int sys;
    if (prn < 40) {
      sys = SYS_GPS;
    } else {
      sys = SYS_SBS;
      prn += 80;
    }
    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 1002 satellite number error: prn=%d\n", prn);
      continue;
    }
    double tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    int index = obsindex(&rtcm->obs, rtcm->time, sat);
    if (index < 0) continue;
    pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
    rtcm->obs.data[index].P[0] = pr1;

    if (ppr1 != (int)0xFFF80000) {
      double freq = FREQL1;
      double cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 * freq / CLIGHT);
      rtcm->obs.data[index].L[0] = pr1 * freq / CLIGHT + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code ? CODE_L1P : CODE_L1C;
  }
  return sync ? 0 : 1;
}
/* Decode type 1003: L1&L2 GPS rtk observables -------------------------------*/
static int decode_type1003(rtcm_t *rtcm) {
  int sync;
  if (decode_head1001(rtcm, &sync) < 0) return -1;
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 1004: extended L1&L2 GPS RTK observables ----------------------*/
static int decode_type1004(rtcm_t *rtcm) {
  int sync;
  int nsat = decode_head1001(rtcm, &sync);
  if (nsat < 0) return -1;

  int i = 24 + 64;
  for (int j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 125 <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, 6);
    i += 6;
    int code1 = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    double pr1 = rtcm_getbitu(rtcm, i, 24);
    i += 24;
    int ppr1 = rtcm_getbits(rtcm, i, 20);
    i += 20;
    int lock1 = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    int amb = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    double cnr1 = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    int code2 = rtcm_getbitu(rtcm, i, 2);
    i += 2;
    int pr21 = rtcm_getbits(rtcm, i, 14);
    i += 14;
    double ppr2 = rtcm_getbits(rtcm, i, 20);
    i += 20;
    int lock2 = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    double cnr2 = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    int sys;
    if (prn < 40) {
      sys = SYS_GPS;
    } else {
      sys = SYS_SBS;
      prn += 80;
    }
    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 1004 satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    double tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    int index = obsindex(&rtcm->obs, rtcm->time, sat);
    if (index < 0) continue;
    pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
    rtcm->obs.data[index].P[0] = pr1;

    const double freq[2] = {FREQL1, FREQL2};
    if (ppr1 != (int)0xFFF80000) {
      double cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 * freq[0] / CLIGHT);
      rtcm->obs.data[index].L[0] = pr1 * freq[0] / CLIGHT + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

    if (pr21 != (int)0xFFFFE000) {
      rtcm->obs.data[index].P[1] = pr1 + pr21 * 0.02;
    }
    if (ppr2 != (int)0xFFF80000) {
      double cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 * freq[1] / CLIGHT);
      rtcm->obs.data[index].L[1] = pr1 * freq[1] / CLIGHT + cp2;
    }
    rtcm->obs.data[index].LLI[1] = lossoflock(rtcm, sat, 1, lock2);
    rtcm->obs.data[index].SNR[1] = snratio(cnr2 * 0.25);
    const int L2codes[] = {CODE_L2X, CODE_L2P, CODE_L2D, CODE_L2W};
    rtcm->obs.data[index].code[1] = L2codes[code2];
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Get signed 38bit field ----------------------------------------------------*/
static double rtcm_getbits_38(const rtcm_t *rtcm, int pos) {
  return (double)rtcm_getbits(rtcm, pos, 32) * 64.0 + rtcm_getbitu(rtcm, pos + 32, 6);
}
/* Decode type 1005: stationary RTK reference station ARP --------------------*/
static int decode_type1005(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 140 != rtcm->len * 8) {
    trace(2, "rtcm3 1005 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12;
  int itrf = rtcm_getbitu(rtcm, i, 6);
  i += 6 + 4;
  double rr[3];
  rr[0] = rtcm_getbits_38(rtcm, i);
  i += 38 + 2;
  rr[1] = rtcm_getbits_38(rtcm, i);
  i += 38 + 2;
  rr[2] = rtcm_getbits_38(rtcm, i);
  if (rtcm->outtype) {
    double re[3];
    for (int j = 0; j < 3; j++) re[j] = rr[j] * 0.0001;
    double pos[3];
    ecef2pos(re, pos);
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d pos=%.8f %.8f %.3f", staid,
                 pos[0] * R2D, pos[1] * R2D, pos[2]);
  }
  /* Test station id */
  if (!test_staid(rtcm, staid)) return -1;

  rtksnprintf(rtcm->sta.name, sizeof(rtcm->sta.name), "%04d", staid);
  rtcm->sta.deltype = 0; /* XYZ */
  for (int j = 0; j < 3; j++) {
    rtcm->sta.pos[j] = rr[j] * 0.0001;
    rtcm->sta.del[j] = 0.0;
  }
  rtcm->sta.hgt = 0.0;
  rtcm->sta.itrf = itrf;
  return 5;
}
/* Decode type 1006: stationary RTK reference station ARP with height --------*/
static int decode_type1006(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 156 > rtcm->len * 8) {
    trace(2, "rtcm3 1006 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12;
  int itrf = rtcm_getbitu(rtcm, i, 6);
  i += 6 + 4;
  double rr[3];
  rr[0] = rtcm_getbits_38(rtcm, i);
  i += 38 + 2;
  rr[1] = rtcm_getbits_38(rtcm, i);
  i += 38 + 2;
  rr[2] = rtcm_getbits_38(rtcm, i);
  i += 38;
  double anth = rtcm_getbitu(rtcm, i, 16);
  if (rtcm->outtype) {
    double re[3], pos[3];
    for (int j = 0; j < 3; j++) re[j] = rr[j] * 0.0001;
    ecef2pos(re, pos);
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d pos=%.8f %.8f %.3f anth=%.3f",
                 staid, pos[0] * R2D, pos[1] * R2D, pos[2], anth * 0.0001);
  }
  /* Test station id */
  if (!test_staid(rtcm, staid)) return -1;

  rtksnprintf(rtcm->sta.name, sizeof(rtcm->sta.name), "%04d", staid);
  rtcm->sta.deltype = 1; /* XYZ */
  for (int j = 0; j < 3; j++) {
    rtcm->sta.pos[j] = rr[j] * 0.0001;
    rtcm->sta.del[j] = 0.0;
  }
  rtcm->sta.hgt = anth * 0.0001;
  rtcm->sta.itrf = itrf;
  return 5;
}
/* Decode type 1007: antenna descriptor --------------------------------------*/
static int decode_type1007(rtcm_t *rtcm) {
  int i = 24 + 12;
  int n = rtcm_getbitu(rtcm, i + 12, 8);

  if (i + 28 + 8 * n > rtcm->len * 8) {
    trace(2, "rtcm3 1007 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12 + 8;
  char des[32] = "";
  for (int j = 0; j < n && j < 31; j++) {
    des[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  int setup = rtcm_getbitu(rtcm, i, 8);
  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d", staid);
  }
  /* Test station ID */
  if (!test_staid(rtcm, staid)) return -1;

  rtksnprintf(rtcm->sta.name, sizeof(rtcm->sta.name), "%04d", staid);
  rtkesubstrcpy(rtcm->sta.antdes, sizeof(rtcm->sta.antdes), des, 0, n);
  rtcm->sta.antsetup = setup;
  rtcm->sta.antsno[0] = '\0';
  return 5;
}
/* Decode type 1008: antenna descriptor & serial number ----------------------*/
static int decode_type1008(rtcm_t *rtcm) {
  int i = 24 + 12;
  int n = rtcm_getbitu(rtcm, i + 12, 8);
  int m = rtcm_getbitu(rtcm, i + 28 + 8 * n, 8);

  if (i + 36 + 8 * (n + m) > rtcm->len * 8) {
    trace(2, "rtcm3 1008 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12 + 8;
  char des[32] = "";
  for (int j = 0; j < n && j < 31; j++) {
    des[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  int setup = rtcm_getbitu(rtcm, i, 8);
  i += 8 + 8;
  char sno[32] = "";
  for (int j = 0; j < m && j < 31; j++) {
    sno[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d", staid);
  }
  /* Test station ID */
  if (!test_staid(rtcm, staid)) return -1;

  rtksnprintf(rtcm->sta.name, sizeof(rtcm->sta.name), "%04d", staid);
  rtkesubstrcpy(rtcm->sta.antdes, sizeof(rtcm->sta.antdes), des, 0, n);
  rtcm->sta.antsetup = setup;
  rtkesubstrcpy(rtcm->sta.antsno, sizeof(rtcm->sta.antsno), sno, 0, m);
  return 5;
}
/* Decode type 1009-1012 message header --------------------------------------*/
static int decode_head1009(rtcm_t *rtcm, int *sync) {
  int i = 24;
  int type = rtcm_getbitu(rtcm, i, 12);
  i += 12;

  if (i + 49 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12;
  double tod = rtcm_getbitu(rtcm, i, 27) * 0.001;
  i += 27; /* Sec in a day */
  *sync = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  int nsat = rtcm_getbitu(rtcm, i, 5);
  /* Test station ID */
  if (!test_staid(rtcm, staid)) return -1;

  adjday_glot(rtcm, tod);

  char tstr[40];
  time2str(rtcm->time, tstr, 2);
  trace(4, "decode_head1009: time=%s nsat=%d sync=%d\n", tstr, nsat, *sync);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d %s nsat=%2d sync=%d", staid,
                 tstr, nsat, *sync);
  }
  return nsat;
}
/* Decode type 1009: L1-only GLONASS rtk observables -------------------------*/
static int decode_type1009(rtcm_t *rtcm) {
  int sync;
  if (decode_head1009(rtcm, &sync) < 0) return -1;
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 1010: extended L1-only GLONASS rtk observables ----------------*/
static int decode_type1010(rtcm_t *rtcm) {
  int i = 24 + 61;
  int sync;
  int nsat = decode_head1009(rtcm, &sync);
  if (nsat < 0) return -1;

  for (int j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 79 <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, 6);
    i += 6;
    int code = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int fcn = rtcm_getbitu(rtcm, i, 5);
    i += 5; /* Fcn+7 */
    double pr1 = rtcm_getbitu(rtcm, i, 25);
    i += 25;
    int ppr1 = rtcm_getbits(rtcm, i, 20);
    i += 20;
    int lock1 = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    int amb = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    double cnr1 = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    int sys = SYS_GLO;
    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 1010 satellite number error: prn=%d\n", prn);
      continue;
    }
    if (!rtcm->nav.glo_fcn[prn - 1]) {
      rtcm->nav.glo_fcn[prn - 1] = fcn - 7 + 8; /* fcn+8 */
    }
    double tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    int index = obsindex(&rtcm->obs, rtcm->time, sat);
    if (index < 0) continue;
    pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
    rtcm->obs.data[index].P[0] = pr1;

    if (ppr1 != (int)0xFFF80000) {
      double freq1 = code2freq(SYS_GLO, CODE_L1C, fcn - 7);
      double cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 * freq1 / CLIGHT);
      rtcm->obs.data[index].L[0] = pr1 * freq1 / CLIGHT + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code ? CODE_L1P : CODE_L1C;
  }
  return sync ? 0 : 1;
}
/* Decode type 1011: L1&L2 GLONASS RTK observables ---------------------------*/
static int decode_type1011(rtcm_t *rtcm) {
  int sync;
  if (decode_head1009(rtcm, &sync) < 0) return -1;
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 1012: extended L1&L2 GLONASS RTK observables ------------------*/
static int decode_type1012(rtcm_t *rtcm) {
  int i = 24 + 61;
  int sync;
  int nsat = decode_head1009(rtcm, &sync);
  if (nsat < 0) return -1;

  for (int j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 130 <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, 6);
    i += 6;
    int code1 = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int fcn = rtcm_getbitu(rtcm, i, 5);
    i += 5; /* fcn+7 */
    double pr1 = rtcm_getbitu(rtcm, i, 25);
    i += 25;
    int ppr1 = rtcm_getbits(rtcm, i, 20);
    i += 20;
    int lock1 = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    int amb = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    double cnr1 = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    int code2 = rtcm_getbitu(rtcm, i, 2);
    i += 2;
    int pr21 = rtcm_getbits(rtcm, i, 14);
    i += 14;
    double ppr2 = rtcm_getbits(rtcm, i, 20);
    i += 20;
    int lock2 = rtcm_getbitu(rtcm, i, 7);
    i += 7;
    double cnr2 = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    int sys = SYS_GLO;
    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 1012 satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    if (!rtcm->nav.glo_fcn[prn - 1]) {
      rtcm->nav.glo_fcn[prn - 1] = fcn - 7 + 8; /* fcn+8 */
    }
    double tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    int index = obsindex(&rtcm->obs, rtcm->time, sat);
    if (index < 0) continue;
    pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
    rtcm->obs.data[index].P[0] = pr1;

    if (ppr1 != (int)0xFFF80000) {
      double freq1 = code2freq(SYS_GLO, CODE_L1C, fcn - 7);
      double cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 * freq1 / CLIGHT);
      rtcm->obs.data[index].L[0] = pr1 * freq1 / CLIGHT + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

    if (pr21 != (int)0xFFFFE000) {
      rtcm->obs.data[index].P[1] = pr1 + pr21 * 0.02;
    }
    if (ppr2 != (int)0xFFF80000) {
      double freq2 = code2freq(SYS_GLO, CODE_L2C, fcn - 7);
      double cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 * freq2 / CLIGHT);
      rtcm->obs.data[index].L[1] = pr1 * freq2 / CLIGHT + cp2;
    }
    rtcm->obs.data[index].LLI[1] = lossoflock(rtcm, sat, 1, lock2);
    rtcm->obs.data[index].SNR[1] = snratio(cnr2 * 0.25);
    rtcm->obs.data[index].code[1] = code2 ? CODE_L2P : CODE_L2C;
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 1013: system parameters ---------------------------------------*/
static int decode_type1013(rtcm_t *rtcm) { return 0; }
/* Decode type 1019: GPS ephemerides -----------------------------------------*/
static int decode_type1019(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 476 > rtcm->len * 8) {
    trace(2, "rtcm3 1019 length error: len=%d\n", rtcm->len);
    return -1;
  }
  eph_t eph = {0};
  int prn = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  int week = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.sva = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  eph.code = rtcm_getbitu(rtcm, i, 2);
  i += 2;
  eph.idot = rtcm_getbits(rtcm, i, 14) * P2_43 * SC2RAD;
  i += 14;
  eph.iode = rtcm_getbitu(rtcm, i, 8);
  i += 8;
  double toc = rtcm_getbitu(rtcm, i, 16) * 16.0;
  i += 16;
  eph.f2 = rtcm_getbits(rtcm, i, 8) * P2_55;
  i += 8;
  eph.f1 = rtcm_getbits(rtcm, i, 16) * P2_43;
  i += 16;
  eph.f0 = rtcm_getbits(rtcm, i, 22) * P2_31;
  i += 22;
  eph.iodc = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.crs = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.deln = rtcm_getbits(rtcm, i, 16) * P2_43 * SC2RAD;
  i += 16;
  eph.M0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cuc = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.e = rtcm_getbitu(rtcm, i, 32) * P2_33;
  i += 32;
  eph.cus = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  double sqrtA = rtcm_getbitu(rtcm, i, 32) * P2_19;
  i += 32;
  eph.toes = rtcm_getbitu(rtcm, i, 16) * 16.0;
  i += 16;
  eph.cic = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.OMG0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cis = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.i0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.crc = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.omg = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.OMGd = rtcm_getbits(rtcm, i, 24) * P2_43 * SC2RAD;
  i += 24;
  eph.tgd[0] = rtcm_getbits(rtcm, i, 8) * P2_31;
  i += 8;
  eph.svh = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  eph.flag = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  eph.fit = rtcm_getbitu(rtcm, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */
  int sys = SYS_GPS;
  if (prn >= 40) {
    sys = SYS_SBS;
    prn += 80;
  }
  trace(4, "decode_type1019: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X", prn, eph.iode,
                 eph.iodc, week, eph.toes, toc, eph.svh);
  }
  int sat = satno(sys, prn);
  if (!sat) {
    trace(2, "rtcm3 1019 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjgpsweek(week);
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  double tt = timediff(gpst2time(eph.week, eph.toes), rtcm->time);
  if (tt < -302400.0)
    eph.week++;
  else if (tt >= 302400.0)
    eph.week--;
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1][0].iode) return 0; /* Unchanged */
  }
  rtcm->nav.eph[sat - 1][0] = eph;
  rtcm->ephsat = sat;
  rtcm->ephset = 0;
  return 2;
}
/* Decode type 1020: GLONASS ephemerides -------------------------------------*/
static int decode_type1020(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 348 > rtcm->len * 8) {
    trace(2, "rtcm3 1020 length error: len=%d\n", rtcm->len);
    return -1;
  }
  geph_t geph = {0};
  int prn = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  geph.frq = rtcm_getbitu(rtcm, i, 5) - 7;
  i += 5 + 2 + 2;
  double tk_h = rtcm_getbitu(rtcm, i, 5);
  i += 5;
  double tk_m = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  double tk_s = rtcm_getbitu(rtcm, i, 1) * 30.0;
  i += 1;
  int bn = rtcm_getbitu(rtcm, i, 1);
  i += 1 + 1;
  int tb = rtcm_getbitu(rtcm, i, 7);
  i += 7;
  geph.vel[0] = rtcm_getbitg(rtcm, i, 24) * P2_20 * 1E3;
  i += 24;
  geph.pos[0] = rtcm_getbitg(rtcm, i, 27) * P2_11 * 1E3;
  i += 27;
  geph.acc[0] = rtcm_getbitg(rtcm, i, 5) * P2_30 * 1E3;
  i += 5;
  geph.vel[1] = rtcm_getbitg(rtcm, i, 24) * P2_20 * 1E3;
  i += 24;
  geph.pos[1] = rtcm_getbitg(rtcm, i, 27) * P2_11 * 1E3;
  i += 27;
  geph.acc[1] = rtcm_getbitg(rtcm, i, 5) * P2_30 * 1E3;
  i += 5;
  geph.vel[2] = rtcm_getbitg(rtcm, i, 24) * P2_20 * 1E3;
  i += 24;
  geph.pos[2] = rtcm_getbitg(rtcm, i, 27) * P2_11 * 1E3;
  i += 27;
  geph.acc[2] = rtcm_getbitg(rtcm, i, 5) * P2_30 * 1E3;
  i += 5 + 1;
  geph.gamn = rtcm_getbitg(rtcm, i, 11) * P2_40;
  i += 11 + 3;
  geph.taun = rtcm_getbitg(rtcm, i, 22) * P2_30;
  i += 22;
  geph.dtaun = rtcm_getbitg(rtcm, i, 5) * P2_30;
  i += 5;
  geph.age = rtcm_getbitu(rtcm, i, 5);
  int sys = SYS_GLO;
  int sat = satno(sys, prn);
  if (!sat) {
    trace(2, "rtcm3 1020 satellite number error: prn=%d\n", prn);
    return -1;
  }
  trace(4, "decode_type1020: prn=%d tk=%02.0f:%02.0f:%02.0f\n", prn, tk_h, tk_m, tk_s);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " prn=%2d tk=%02.0f:%02.0f:%02.0f frq=%2d bn=%d tb=%d", prn, tk_h, tk_m, tk_s,
                 geph.frq, bn, tb);
  }
  geph.sat = sat;
  geph.svh = bn;
  geph.iode = tb & 0x7F;
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  int week;
  double tow = time2gpst(gpst2utc(rtcm->time), &week);
  double tod = fmod(tow, 86400.0);
  tow -= tod;
  double tof = tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0; /* Lt->utc */
  if (tof < tod - 43200.0)
    tof += 86400.0;
  else if (tof > tod + 43200.0)
    tof -= 86400.0;
  geph.tof = utc2gpst(gpst2time(week, tow + tof));
  double toe = tb * 900.0 - 10800.0; /* Lt->utc */
  if (toe < tod - 43200.0)
    toe += 86400.0;
  else if (toe > tod + 43200.0)
    toe -= 86400.0;
  geph.toe = utc2gpst(gpst2time(week, tow + toe)); /* Utc->gpst */

  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (fabs(timediff(geph.toe, rtcm->nav.geph[prn - 1][0].toe)) < 1.0 &&
        geph.svh == rtcm->nav.geph[prn - 1][0].svh)
      return 0; /* Unchanged */
  }
  rtcm->nav.geph[prn - 1][0] = geph;
  rtcm->ephsat = sat;
  rtcm->ephset = 0;
  return 2;
}
/* Decode type 1021: helmert/abridged molodenski -----------------------------*/
static int decode_type1021(rtcm_t *rtcm) {
  trace(2, "rtcm3 1021: not supported message\n");
  return 0;
}
/* Decode type 1022: Moledenski-Badekas transfromation -----------------------*/
static int decode_type1022(rtcm_t *rtcm) {
  trace(2, "rtcm3 1022: not supported message\n");
  return 0;
}
/* Decode type 1023: residual, ellipsoidal grid representation ---------------*/
static int decode_type1023(rtcm_t *rtcm) {
  trace(2, "rtcm3 1023: not supported message\n");
  return 0;
}
/* Decode type 1024: residual, plane grid representation ---------------------*/
static int decode_type1024(rtcm_t *rtcm) {
  trace(2, "rtcm3 1024: not supported message\n");
  return 0;
}
/* Decode type 1025: projection (types except LCC2SP,OM) ---------------------*/
static int decode_type1025(rtcm_t *rtcm) {
  trace(2, "rtcm3 1025: not supported message\n");
  return 0;
}
/* Decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -----*/
static int decode_type1026(rtcm_t *rtcm) {
  trace(2, "rtcm3 1026: not supported message\n");
  return 0;
}
/* Decode type 1027: projection (type OM - oblique mercator) -----------------*/
static int decode_type1027(rtcm_t *rtcm) {
  trace(2, "rtcm3 1027: not supported message\n");
  return 0;
}
/* Decode type 1029: UNICODE text string -------------------------------------*/
static int decode_type1029(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 60 > rtcm->len * 8) {
    trace(2, "rtcm3 1029 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12;
  int mjd = rtcm_getbitu(rtcm, i, 16);
  i += 16;
  int tod = rtcm_getbitu(rtcm, i, 17);
  i += 17;
  int nchar = rtcm_getbitu(rtcm, i, 7);
  i += 7;
  int cunit = rtcm_getbitu(rtcm, i, 8);
  i += 8;
  if (i + nchar * 8 > rtcm->len * 8) {
    trace(2, "rtcm3 1029 length error: len=%d nchar=%d\n", rtcm->len, nchar);
    return -1;
  }
  int j = 0;
  for (; j < nchar && j < 126; j++) {
    rtcm->msg[j] = rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  rtcm->msg[j] = '\0';

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d text=%s", staid, rtcm->msg);
  }
  return 0;
}
/* Decode type 1030: network RTK residual ------------------------------------*/
static int decode_type1030(rtcm_t *rtcm) {
  trace(2, "rtcm3 1030: not supported message\n");
  return 0;
}
/* Decode type 1031: GLONASS network RTK residual ----------------------------*/
static int decode_type1031(rtcm_t *rtcm) {
  trace(2, "rtcm3 1031: not supported message\n");
  return 0;
}
/* Decode type 1032: physical reference station position information ---------*/
static int decode_type1032(rtcm_t *rtcm) {
  trace(2, "rtcm3 1032: not supported message\n");
  return 0;
}
/* Decode type 1033: receiver and antenna descriptor -------------------------*/
static int decode_type1033(rtcm_t *rtcm) {
  int i = 24 + 12;
  int n = rtcm_getbitu(rtcm, i + 12, 8);
  int m = rtcm_getbitu(rtcm, i + 28 + 8 * n, 8);
  int n1 = rtcm_getbitu(rtcm, i + 36 + 8 * (n + m), 8);
  int n2 = rtcm_getbitu(rtcm, i + 44 + 8 * (n + m + n1), 8);
  int n3 = rtcm_getbitu(rtcm, i + 52 + 8 * (n + m + n1 + n2), 8);
  if (i + 60 + 8 * (n + m + n1 + n2 + n3) > rtcm->len * 8) {
    trace(2, "rtcm3 1033 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12 + 8;
  char des[32] = "";
  for (int j = 0; j < n && j < 31; j++) {
    des[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  int setup = rtcm_getbitu(rtcm, i, 8);
  i += 8 + 8;
  char sno[32] = "";
  for (int j = 0; j < m && j < 31; j++) {
    sno[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  i += 8;
  char rec[32] = "";
  for (int j = 0; j < n1 && j < 31; j++) {
    rec[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  i += 8;
  char ver[32] = "";
  for (int j = 0; j < n2 && j < 31; j++) {
    ver[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  i += 8;
  char rsn[32] = "";
  for (int j = 0; j < n3 && j < 31; j++) {
    rsn[j] = (char)rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d", staid);
  }
  /* Test station id */
  if (!test_staid(rtcm, staid)) return -1;

  rtksnprintf(rtcm->sta.name, sizeof(rtcm->sta.name), "%04d", staid);
  rtkesubstrcpy(rtcm->sta.antdes, sizeof(rtcm->sta.antdes), des, 0, n);
  rtcm->sta.antsetup = setup;
  rtkesubstrcpy(rtcm->sta.antsno, sizeof(rtcm->sta.antsno), sno, 0, m);
  rtkesubstrcpy(rtcm->sta.rectype, sizeof(rtcm->sta.rectype), rec, 0, n1);
  rtkesubstrcpy(rtcm->sta.recver, sizeof(rtcm->sta.recver), ver, 0, n2);
  rtkesubstrcpy(rtcm->sta.recsno, sizeof(rtcm->sta.recsno), rsn, 0, n3);

  trace(3, "rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n", des, sno, rec, ver, rsn);
  return 5;
}
/* Decode type 1034: GPS network FKP gradient --------------------------------*/
static int decode_type1034(rtcm_t *rtcm) {
  trace(2, "rtcm3 1034: not supported message\n");
  return 0;
}
/* Decode type 1035: GLONASS network FKP gradient ----------------------------*/
static int decode_type1035(rtcm_t *rtcm) {
  trace(2, "rtcm3 1035: not supported message\n");
  return 0;
}
/* Decode type 1037: GLONASS network RTK ionospheric correction difference ---*/
static int decode_type1037(rtcm_t *rtcm) {
  trace(2, "rtcm3 1037: not supported message\n");
  return 0;
}
/* Decode type 1038: GLONASS network RTK geometic correction difference ------*/
static int decode_type1038(rtcm_t *rtcm) {
  trace(2, "rtcm3 1038: not supported message\n");
  return 0;
}
/* Decode type 1039: GLONASS network RTK combined correction difference ------*/
static int decode_type1039(rtcm_t *rtcm) {
  trace(2, "rtcm3 1039: not supported message\n");
  return 0;
}
/* Decode type 1041: NavIC/IRNSS ephemerides ---------------------------------*/
static int decode_type1041(rtcm_t *rtcm) {
  int i = 24 + 12, sys = SYS_IRN;
  if (i + 482 - 12 > rtcm->len * 8) {
    trace(2, "rtcm3 1041 length error: len=%d\n", rtcm->len);
    return -1;
  }
  eph_t eph = {0};
  int prn = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  int week = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.f0 = rtcm_getbits(rtcm, i, 22) * P2_31;
  i += 22;
  eph.f1 = rtcm_getbits(rtcm, i, 16) * P2_43;
  i += 16;
  eph.f2 = rtcm_getbits(rtcm, i, 8) * P2_55;
  i += 8;
  eph.sva = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  double toc = rtcm_getbitu(rtcm, i, 16) * 16.0;
  i += 16;
  eph.tgd[0] = rtcm_getbits(rtcm, i, 8) * P2_31;
  i += 8;
  eph.deln = rtcm_getbits(rtcm, i, 22) * P2_41 * SC2RAD;
  i += 22;
  eph.iode = rtcm_getbitu(rtcm, i, 8);
  i += 8 + 10; /* IODEC */
  eph.svh = rtcm_getbitu(rtcm, i, 2);
  i += 2; /* L5+Sflag */
  eph.cuc = rtcm_getbits(rtcm, i, 15) * P2_28;
  i += 15;
  eph.cus = rtcm_getbits(rtcm, i, 15) * P2_28;
  i += 15;
  eph.cic = rtcm_getbits(rtcm, i, 15) * P2_28;
  i += 15;
  eph.cis = rtcm_getbits(rtcm, i, 15) * P2_28;
  i += 15;
  eph.crc = rtcm_getbits(rtcm, i, 15) * 0.0625;
  i += 15;
  eph.crs = rtcm_getbits(rtcm, i, 15) * 0.0625;
  i += 15;
  eph.idot = rtcm_getbits(rtcm, i, 14) * P2_43 * SC2RAD;
  i += 14;
  eph.M0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.toes = rtcm_getbitu(rtcm, i, 16) * 16.0;
  i += 16;
  eph.e = rtcm_getbitu(rtcm, i, 32) * P2_33;
  i += 32;
  double sqrtA = rtcm_getbitu(rtcm, i, 32) * P2_19;
  i += 32;
  eph.OMG0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.omg = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.OMGd = rtcm_getbits(rtcm, i, 22) * P2_41 * SC2RAD;
  i += 22;
  eph.i0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  trace(4, "decode_type1041: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X", prn, eph.iode, week,
                 eph.toes, toc, eph.svh);
  }
  int sat = satno(sys, prn);
  if (!sat) {
    trace(2, "rtcm3 1041 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjgpsweek(week);
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  double tt = timediff(gpst2time(eph.week, eph.toes), rtcm->time);
  if (tt < -302400.0)
    eph.week++;
  else if (tt >= 302400.0)
    eph.week--;
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  eph.iodc = eph.iode;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1][0].iode) return 0; /* Unchanged */
  }
  rtcm->nav.eph[sat - 1][0] = eph;
  rtcm->ephsat = sat;
  rtcm->ephset = 0;
  return 2;
}
/* Decode type 1044: QZSS ephemerides ----------------------------------------*/
static int decode_type1044(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 473 > rtcm->len * 8) {
    trace(2, "rtcm3 1044 length error: len=%d\n", rtcm->len);
    return -1;
  }
  eph_t eph = {0};
  int prn = rtcm_getbitu(rtcm, i, 4) + 192;
  i += 4;
  double toc = rtcm_getbitu(rtcm, i, 16) * 16.0;
  i += 16;
  eph.f2 = rtcm_getbits(rtcm, i, 8) * P2_55;
  i += 8;
  eph.f1 = rtcm_getbits(rtcm, i, 16) * P2_43;
  i += 16;
  eph.f0 = rtcm_getbits(rtcm, i, 22) * P2_31;
  i += 22;
  eph.iode = rtcm_getbitu(rtcm, i, 8);
  i += 8;
  eph.crs = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.deln = rtcm_getbits(rtcm, i, 16) * P2_43 * SC2RAD;
  i += 16;
  eph.M0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cuc = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.e = rtcm_getbitu(rtcm, i, 32) * P2_33;
  i += 32;
  eph.cus = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  double sqrtA = rtcm_getbitu(rtcm, i, 32) * P2_19;
  i += 32;
  eph.toes = rtcm_getbitu(rtcm, i, 16) * 16.0;
  i += 16;
  eph.cic = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.OMG0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cis = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.i0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.crc = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.omg = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.OMGd = rtcm_getbits(rtcm, i, 24) * P2_43 * SC2RAD;
  i += 24;
  eph.idot = rtcm_getbits(rtcm, i, 14) * P2_43 * SC2RAD;
  i += 14;
  eph.code = rtcm_getbitu(rtcm, i, 2);
  i += 2;
  int week = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.sva = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  eph.svh = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  eph.tgd[0] = rtcm_getbits(rtcm, i, 8) * P2_31;
  i += 8;
  eph.iodc = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.fit = rtcm_getbitu(rtcm, i, 1) ? 0.0 : 2.0; /* 0:2hr,1:>2hr */
  trace(4, "decode_type1044: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " prn=%3d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X", prn, eph.iode,
                 eph.iodc, week, eph.toes, toc, eph.svh);
  }
  int sys = SYS_QZS;
  int sat = satno(sys, prn);
  if (!sat) {
    trace(2, "rtcm3 1044 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjgpsweek(week);
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  double tt = timediff(gpst2time(eph.week, eph.toes), rtcm->time);
  if (tt < -302400.0)
    eph.week++;
  else if (tt >= 302400.0)
    eph.week--;
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  eph.flag = 1; /* Fixed to 1 */
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1][0].iode && eph.iodc == rtcm->nav.eph[sat - 1][0].iodc)
      return 0; /* Unchanged */
  }
  rtcm->nav.eph[sat - 1][0] = eph;
  rtcm->ephsat = sat;
  rtcm->ephset = 0;
  return 2;
}
/* Decode type 1045: Galileo F/NAV satellite ephemerides ---------------------*/
static int decode_type1045(rtcm_t *rtcm) {
  if (strstr(rtcm->opt, "-GALINAV")) return 0;
  int i = 24 + 12;
  if (i + 484 > rtcm->len * 8) {
    trace(2, "rtcm3 1045 length error: len=%d\n", rtcm->len);
    return -1;
  }
  eph_t eph = {0};
  int prn = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  int week = rtcm_getbitu(rtcm, i, 12);
  i += 12; /* Gst-week */
  eph.iode = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.sva = rtcm_getbitu(rtcm, i, 8);
  i += 8;
  eph.idot = rtcm_getbits(rtcm, i, 14) * P2_43 * SC2RAD;
  i += 14;
  double toc = rtcm_getbitu(rtcm, i, 14) * 60.0;
  i += 14;
  eph.f2 = rtcm_getbits(rtcm, i, 6) * P2_59;
  i += 6;
  eph.f1 = rtcm_getbits(rtcm, i, 21) * P2_46;
  i += 21;
  eph.f0 = rtcm_getbits(rtcm, i, 31) * P2_34;
  i += 31;
  eph.crs = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.deln = rtcm_getbits(rtcm, i, 16) * P2_43 * SC2RAD;
  i += 16;
  eph.M0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cuc = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.e = rtcm_getbitu(rtcm, i, 32) * P2_33;
  i += 32;
  eph.cus = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  double sqrtA = rtcm_getbitu(rtcm, i, 32) * P2_19;
  i += 32;
  eph.toes = rtcm_getbitu(rtcm, i, 14) * 60.0;
  i += 14;
  eph.cic = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.OMG0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cis = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.i0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.crc = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.omg = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.OMGd = rtcm_getbits(rtcm, i, 24) * P2_43 * SC2RAD;
  i += 24;
  eph.tgd[0] = rtcm_getbits(rtcm, i, 10) * P2_32;
  i += 10; /* E5a/E1 */
  int e5a_hs = rtcm_getbitu(rtcm, i, 2);
  i += 2; /* OSHS */
  int e5a_dvs = rtcm_getbitu(rtcm, i, 1);
  i += 1; /* OSDVS */
  int rsv = rtcm_getbitu(rtcm, i, 7);
  trace(4, "decode_type1045: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d", prn, eph.iode, week,
                 eph.toes, toc, e5a_hs, e5a_dvs);
  }
  int sys = SYS_GAL;
  int sat = satno(sys, prn);
  if (!sat) {
    trace(2, "rtcm3 1045 satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (strstr(rtcm->opt, "-GALINAV")) {
    return 0;
  }
  eph.sat = sat;
  eph.week = week + 1024; /* Gal-week = gst-week + 1024 */
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  double tt = timediff(gpst2time(eph.week, eph.toes), rtcm->time);
  if (tt < -302400.0)
    eph.week++;
  else if (tt >= 302400.0)
    eph.week--;
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  eph.svh = (e5a_hs << 4) + (e5a_dvs << 3);
  eph.code = (1 << 1) + (1 << 8); /* Data source = F/NAV+E5a */
  eph.iodc = eph.iode;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1][1].iode) return 0; /* Unchanged */
  }
  rtcm->nav.eph[sat - 1][1] = eph;
  rtcm->ephsat = sat;
  rtcm->ephset = 1; /* F/NAV */
  return 2;
}
/* Decode type 1046: Galileo I/NAV satellite ephemerides ---------------------*/
static int decode_type1046(rtcm_t *rtcm) {
  if (strstr(rtcm->opt, "-GALFNAV")) return 0;
  int i = 24 + 12;
  if (i + 492 > rtcm->len * 8) {
    trace(2, "rtcm3 1046 length error: len=%d\n", rtcm->len);
    return -1;
  }
  eph_t eph = {0};
  int prn = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  int week = rtcm_getbitu(rtcm, i, 12);
  i += 12;
  eph.iode = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.sva = rtcm_getbitu(rtcm, i, 8);
  i += 8;
  eph.idot = rtcm_getbits(rtcm, i, 14) * P2_43 * SC2RAD;
  i += 14;
  double toc = rtcm_getbitu(rtcm, i, 14) * 60.0;
  i += 14;
  eph.f2 = rtcm_getbits(rtcm, i, 6) * P2_59;
  i += 6;
  eph.f1 = rtcm_getbits(rtcm, i, 21) * P2_46;
  i += 21;
  eph.f0 = rtcm_getbits(rtcm, i, 31) * P2_34;
  i += 31;
  eph.crs = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.deln = rtcm_getbits(rtcm, i, 16) * P2_43 * SC2RAD;
  i += 16;
  eph.M0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cuc = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.e = rtcm_getbitu(rtcm, i, 32) * P2_33;
  i += 32;
  eph.cus = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  double sqrtA = rtcm_getbitu(rtcm, i, 32) * P2_19;
  i += 32;
  eph.toes = rtcm_getbitu(rtcm, i, 14) * 60.0;
  i += 14;
  eph.cic = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.OMG0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cis = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.i0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.crc = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.omg = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.OMGd = rtcm_getbits(rtcm, i, 24) * P2_43 * SC2RAD;
  i += 24;
  eph.tgd[0] = rtcm_getbits(rtcm, i, 10) * P2_32;
  i += 10; /* E5a/E1 */
  eph.tgd[1] = rtcm_getbits(rtcm, i, 10) * P2_32;
  i += 10; /* E5b/E1 */
  int e5b_hs = rtcm_getbitu(rtcm, i, 2);
  i += 2; /* E5b OSHS */
  int e5b_dvs = rtcm_getbitu(rtcm, i, 1);
  i += 1; /* E5b OSDVS */
  int e1_hs = rtcm_getbitu(rtcm, i, 2);
  i += 2; /* E1 OSHS */
  int e1_dvs = rtcm_getbitu(rtcm, i, 1);
  i += 1; /* E1 OSDVS */
  trace(4, "decode_type1046: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d %d dvs=%d %d", prn, eph.iode,
                 week, eph.toes, toc, e5b_hs, e1_hs, e5b_dvs, e1_dvs);
  }
  int sys = SYS_GAL;
  int sat = satno(sys, prn);
  if (!sat) {
    trace(2, "rtcm3 1046 satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (strstr(rtcm->opt, "-GALFNAV")) {
    return 0;
  }
  eph.sat = sat;
  eph.week = week + 1024; /* Gal-week = gst-week + 1024 */
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  double tt = timediff(gpst2time(eph.week, eph.toes), rtcm->time);
  if (tt < -302400.0)
    eph.week++;
  else if (tt >= 302400.0)
    eph.week--;
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  eph.svh = (e5b_hs << 7) + (e5b_dvs << 6) + (e1_hs << 1) + (e1_dvs << 0);
  eph.code = (1 << 0) + (1 << 2) + (1 << 9); /* Data source = I/NAV+E1+E5b */
  eph.iodc = eph.iode;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1][0].iode) return 0; /* Unchanged */
  }
  rtcm->nav.eph[sat - 1][0] = eph;
  rtcm->ephsat = sat;
  rtcm->ephset = 0; /* I/NAV */
  return 2;
}
/* Decode type 1042/63: BeiDou ephemerides -----------------------------------*/
static int decode_type1042(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 499 > rtcm->len * 8) {
    trace(2, "rtcm3 1042 length error: len=%d\n", rtcm->len);
    return -1;
  }
  eph_t eph = {0};
  int prn = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  int week = rtcm_getbitu(rtcm, i, 13);
  i += 13;
  eph.sva = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  eph.idot = rtcm_getbits(rtcm, i, 14) * P2_43 * SC2RAD;
  i += 14;
  eph.iode = rtcm_getbitu(rtcm, i, 5);
  i += 5; /* AODE */
  double toc = rtcm_getbitu(rtcm, i, 17) * 8.0;
  i += 17;
  eph.f2 = rtcm_getbits(rtcm, i, 11) * P2_66;
  i += 11;
  eph.f1 = rtcm_getbits(rtcm, i, 22) * P2_50;
  i += 22;
  eph.f0 = rtcm_getbits(rtcm, i, 24) * P2_33;
  i += 24;
  eph.iodc = rtcm_getbitu(rtcm, i, 5);
  i += 5; /* AODC */
  eph.crs = rtcm_getbits(rtcm, i, 18) * P2_6;
  i += 18;
  eph.deln = rtcm_getbits(rtcm, i, 16) * P2_43 * SC2RAD;
  i += 16;
  eph.M0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cuc = rtcm_getbits(rtcm, i, 18) * P2_31;
  i += 18;
  eph.e = rtcm_getbitu(rtcm, i, 32) * P2_33;
  i += 32;
  eph.cus = rtcm_getbits(rtcm, i, 18) * P2_31;
  i += 18;
  double sqrtA = rtcm_getbitu(rtcm, i, 32) * P2_19;
  i += 32;
  eph.toes = rtcm_getbitu(rtcm, i, 17) * 8.0;
  i += 17;
  eph.cic = rtcm_getbits(rtcm, i, 18) * P2_31;
  i += 18;
  eph.OMG0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cis = rtcm_getbits(rtcm, i, 18) * P2_31;
  i += 18;
  eph.i0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.crc = rtcm_getbits(rtcm, i, 18) * P2_6;
  i += 18;
  eph.omg = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.OMGd = rtcm_getbits(rtcm, i, 24) * P2_43 * SC2RAD;
  i += 24;
  eph.tgd[0] = rtcm_getbits(rtcm, i, 10) * 1E-10;
  i += 10;
  eph.tgd[1] = rtcm_getbits(rtcm, i, 10) * 1E-10;
  i += 10;
  eph.svh = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  trace(4, "decode_type1042: prn=%d iode=%d toe=%.0f\n", prn, eph.iode, eph.toes);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X", prn, eph.iode,
                 eph.iodc, week, eph.toes, toc, eph.svh);
  }
  int sys = SYS_CMP;
  int sat = satno(sys, prn);
  if (!sat) {
    trace(2, "rtcm3 1042 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjbdtweek(week);
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  double tt = timediff(bdt2gpst(bdt2time(eph.week, eph.toes)), rtcm->time);
  if (tt < -302400.0)
    eph.week++;
  else if (tt >= 302400.0)
    eph.week--;
  eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* BDT -> GPST */
  eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* BDT -> GPST */
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (timediff(eph.toe, rtcm->nav.eph[sat - 1][0].toe) == 0.0 &&
        eph.iode == rtcm->nav.eph[sat - 1][0].iode && eph.iodc == rtcm->nav.eph[sat - 1][0].iodc)
      return 0; /* Unchanged */
  }
  rtcm->nav.eph[sat - 1][0] = eph;
  rtcm->ephset = 0;
  rtcm->ephsat = sat;
  return 2;
}
/* Decode SSR message epoch time ---------------------------------------------*/
static int decode_ssr_epoch(rtcm_t *rtcm, int sys, int subtype) {
  int i = 24 + 12;
  if (subtype == 0) { /* RTCM SSR */
    if (sys == SYS_GLO) {
      double tod = rtcm_getbitu(rtcm, i, 17);
      i += 17;
      adjday_glot(rtcm, tod);
    } else {
      double tow = rtcm_getbitu(rtcm, i, 20);
      i += 20;
      adjweek(rtcm, tow);
    }
  } else { /* IGS SSR */
    i += 3 + 8;
    double tow = rtcm_getbitu(rtcm, i, 20);
    i += 20;
    adjweek(rtcm, tow);
  }
  return i;
}
/* Decode SSR 1,4 message header ---------------------------------------------*/
static int decode_ssr1_head(rtcm_t *rtcm, int sys, int subtype, int *sync, int *iod, double *udint,
                            int *refd, int *hsize) {
  int i = 24 + 12, ns;
  if (subtype == 0) { /* RTCM SSR */
    ns = (sys == SYS_QZS) ? 4 : 6;
    if (i + ((sys == SYS_GLO) ? 53 : 50 + ns) > rtcm->len * 8) return -1;
  } else { /* IGS SSR */
    ns = 6;
    if (i + 3 + 8 + 50 + ns > rtcm->len * 8) return -1;
  }
  i = decode_ssr_epoch(rtcm, sys, subtype);
  int udi = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  *sync = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  if (subtype == 0) { /* RTCM SSR */
    *refd = rtcm_getbitu(rtcm, i, 1);
    i += 1; /* Satellite ref datum */
  }
  *iod = rtcm_getbitu(rtcm, i, 4);
  i += 4; /* IOD SSR */
  int provid = rtcm_getbitu(rtcm, i, 16);
  i += 16; /* Provider ID */
  int solid = rtcm_getbitu(rtcm, i, 4);
  i += 4;            /* Solution ID */
  if (subtype > 0) { /* IGS SSR */
    *refd = rtcm_getbitu(rtcm, i, 1);
    i += 1; /* Global/regional CRS indicator */
  }
  int nsat = rtcm_getbitu(rtcm, i, ns);
  i += ns;
  *udint = ssrudint[udi];

  char tstr[40];
  time2str(rtcm->time, tstr, 2);
  trace(4,
        "decode_ssr1_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d"
        " provid=%d solid=%d\n",
        tstr, sys, subtype, nsat, *sync, *iod, provid, solid);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " %s nsat=%2d iod=%2d udi=%2d sync=%d", tstr,
                 nsat, *iod, udi, *sync);
  }
  *hsize = i;
  return nsat;
}
/* Decode SSR 2,3,5,6 message header -----------------------------------------*/
static int decode_ssr2_head(rtcm_t *rtcm, int sys, int subtype, int *sync, int *iod, double *udint,
                            int *hsize) {
  int i = 24 + 12, ns;
  if (subtype == 0) { /* RTCM SSR */
    ns = (sys == SYS_QZS) ? 4 : 6;
    if (i + ((sys == SYS_GLO) ? 52 : 49 + ns) > rtcm->len * 8) return -1;
  } else {
    ns = 6;
    if (i + 3 + 8 + 49 + ns > rtcm->len * 8) return -1;
  }
  i = decode_ssr_epoch(rtcm, sys, subtype);
  int udi = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  *sync = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  *iod = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  int provid = rtcm_getbitu(rtcm, i, 16);
  i += 16; /* Provider ID */
  int solid = rtcm_getbitu(rtcm, i, 4);
  i += 4; /* Solution ID */
  int nsat = rtcm_getbitu(rtcm, i, ns);
  i += ns;
  *udint = ssrudint[udi];

  char tstr[40];
  time2str(rtcm->time, tstr, 2);
  trace(4,
        "decode_ssr2_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d"
        " provid=%d solid=%d\n",
        tstr, sys, subtype, nsat, *sync, *iod, provid, solid);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " %s nsat=%2d iod=%2d udi=%2d sync=%d", tstr,
                 nsat, *iod, udi, *sync);
  }
  *hsize = i;
  return nsat;
}
/* Decode SSR 1: orbit corrections -------------------------------------------*/
static int decode_ssr1(rtcm_t *rtcm, int sys, int subtype) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  int sync, iod, refd = 0, i;
  double udint;
  int nsat = decode_ssr1_head(rtcm, sys, subtype, &sync, &iod, &udint, &refd, &i);
  if (nsat < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int np, ni, nj, offp;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      ni = 10;
      nj = 0;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      ni = 8;
      nj = 0;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      ni = 10;
      nj = 24;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      ni = 9;
      nj = 24;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    ni = 8;
    nj = 0;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  for (int j = 0; j < nsat && i + 121 + np + ni + nj <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, np) + offp;
    i += np;
    int iode = rtcm_getbitu(rtcm, i, ni);
    i += ni;
    int iodcrc = rtcm_getbitu(rtcm, i, nj);
    i += nj;
    double deph[3];
    deph[0] = rtcm_getbits(rtcm, i, 22) * 1E-4;
    i += 22;
    deph[1] = rtcm_getbits(rtcm, i, 20) * 4E-4;
    i += 20;
    deph[2] = rtcm_getbits(rtcm, i, 20) * 4E-4;
    i += 20;
    double ddeph[3];
    ddeph[0] = rtcm_getbits(rtcm, i, 21) * 1E-6;
    i += 21;
    ddeph[1] = rtcm_getbits(rtcm, i, 19) * 4E-6;
    i += 19;
    ddeph[2] = rtcm_getbits(rtcm, i, 19) * 4E-6;
    i += 19;

    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[0] = rtcm->time;
    rtcm->ssr[sat - 1].udi[0] = udint;
    rtcm->ssr[sat - 1].iod[0] = iod;
    rtcm->ssr[sat - 1].iode = iode;     /* SBAS/BDS: toe/t0 modulo */
    rtcm->ssr[sat - 1].iodcrc = iodcrc; /* SBAS/BDS: IOD CRC */
    rtcm->ssr[sat - 1].refd = refd;

    for (int k = 0; k < 3; k++) {
      rtcm->ssr[sat - 1].deph[k] = deph[k];
      rtcm->ssr[sat - 1].ddeph[k] = ddeph[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* Decode SSR 2: clock corrections -------------------------------------------*/
static int decode_ssr2(rtcm_t *rtcm, int sys, int subtype) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  double udint;
  int sync, iod, i;
  int nsat = decode_ssr2_head(rtcm, sys, subtype, &sync, &iod, &udint, &i);
  if (nsat < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int np, offp;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  for (int j = 0; j < nsat && i + 70 + np <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, np) + offp;
    i += np;
    double dclk[3];
    dclk[0] = rtcm_getbits(rtcm, i, 22) * 1E-4;
    i += 22;
    dclk[1] = rtcm_getbits(rtcm, i, 21) * 1E-6;
    i += 21;
    dclk[2] = rtcm_getbits(rtcm, i, 27) * 2E-8;
    i += 27;

    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[1] = rtcm->time;
    rtcm->ssr[sat - 1].udi[1] = udint;
    rtcm->ssr[sat - 1].iod[1] = iod;

    for (int k = 0; k < 3; k++) {
      rtcm->ssr[sat - 1].dclk[k] = dclk[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* Decode SSR 3: satellite code biases ---------------------------------------*/
static int decode_ssr3(rtcm_t *rtcm, int sys, int subtype) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  double udint;
  int sync, iod, i;
  int nsat = decode_ssr2_head(rtcm, sys, subtype, &sync, &iod, &udint, &i);
  if (nsat < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int np, offp;
  const uint8_t *sigs;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      sigs = ssr_sig_gps;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      sigs = ssr_sig_glo;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      sigs = ssr_sig_gal;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      sigs = ssr_sig_qzs;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      sigs = ssr_sig_cmp;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      sigs = ssr_sig_sbs;
      break;
    default:
      return sync ? 0 : 10;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  for (int j = 0; j < nsat && i + 5 + np <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, np) + offp;
    i += np;
    int nbias = rtcm_getbitu(rtcm, i, 5);
    i += 5;

    double cbias[MAXCODE];
    for (int k = 0; k < MAXCODE; k++) cbias[k] = 0.0;
    for (int k = 0; k < nbias && i + 19 <= rtcm->len * 8; k++) {
      int mode = rtcm_getbitu(rtcm, i, 5);
      i += 5;
      double bias = rtcm_getbits(rtcm, i, 14) * 0.01;
      i += 14;
      if (sigs[mode]) {
        cbias[sigs[mode] - 1] = (float)bias;
      } else {
        trace(2, "rtcm3 %d not supported mode: mode=%d\n", type, mode);
      }
    }
    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[4] = rtcm->time;
    rtcm->ssr[sat - 1].udi[4] = udint;
    rtcm->ssr[sat - 1].iod[4] = iod;

    for (int k = 0; k < MAXCODE; k++) {
      rtcm->ssr[sat - 1].cbias[k] = (float)cbias[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* Decode SSR 4: combined orbit and clock corrections ------------------------*/
static int decode_ssr4(rtcm_t *rtcm, int sys, int subtype) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  double udint;
  int sync, iod, refd = 0, i;
  int nsat = decode_ssr1_head(rtcm, sys, subtype, &sync, &iod, &udint, &refd, &i);
  if (nsat < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int np, ni, nj, offp;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      ni = 10;
      nj = 0;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      ni = 8;
      nj = 0;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      ni = 10;
      nj = 24;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      ni = 9;
      nj = 24;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    ni = 8;
    nj = 0;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  for (int j = 0; j < nsat && i + 191 + np + ni + nj <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, np) + offp;
    i += np;
    int iode = rtcm_getbitu(rtcm, i, ni);
    i += ni;
    int iodcrc = rtcm_getbitu(rtcm, i, nj);
    i += nj;
    double deph[3];
    deph[0] = rtcm_getbits(rtcm, i, 22) * 1E-4;
    i += 22;
    deph[1] = rtcm_getbits(rtcm, i, 20) * 4E-4;
    i += 20;
    deph[2] = rtcm_getbits(rtcm, i, 20) * 4E-4;
    i += 20;
    double ddeph[3];
    ddeph[0] = rtcm_getbits(rtcm, i, 21) * 1E-6;
    i += 21;
    ddeph[1] = rtcm_getbits(rtcm, i, 19) * 4E-6;
    i += 19;
    ddeph[2] = rtcm_getbits(rtcm, i, 19) * 4E-6;
    i += 19;

    double dclk[3];
    dclk[0] = rtcm_getbits(rtcm, i, 22) * 1E-4;
    i += 22;
    dclk[1] = rtcm_getbits(rtcm, i, 21) * 1E-6;
    i += 21;
    dclk[2] = rtcm_getbits(rtcm, i, 27) * 2E-8;
    i += 27;

    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[0] = rtcm->ssr[sat - 1].t0[1] = rtcm->time;
    rtcm->ssr[sat - 1].udi[0] = rtcm->ssr[sat - 1].udi[1] = udint;
    rtcm->ssr[sat - 1].iod[0] = rtcm->ssr[sat - 1].iod[1] = iod;
    rtcm->ssr[sat - 1].iode = iode;
    rtcm->ssr[sat - 1].iodcrc = iodcrc;
    rtcm->ssr[sat - 1].refd = refd;

    for (int k = 0; k < 3; k++) {
      rtcm->ssr[sat - 1].deph[k] = deph[k];
      rtcm->ssr[sat - 1].ddeph[k] = ddeph[k];
      rtcm->ssr[sat - 1].dclk[k] = dclk[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* Decode SSR 5: URA ---------------------------------------------------------*/
static int decode_ssr5(rtcm_t *rtcm, int sys, int subtype) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  double udint;
  int sync, iod, i;
  int nsat = decode_ssr2_head(rtcm, sys, subtype, &sync, &iod, &udint, &i);
  if (nsat < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int np, offp;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  for (int j = 0; j < nsat && i + 6 + np <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, np) + offp;
    i += np;
    int ura = rtcm_getbitu(rtcm, i, 6);
    i += 6;

    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[3] = rtcm->time;
    rtcm->ssr[sat - 1].udi[3] = udint;
    rtcm->ssr[sat - 1].iod[3] = iod;
    rtcm->ssr[sat - 1].ura = ura;
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* Decode SSR 6: high rate clock correction ----------------------------------*/
static int decode_ssr6(rtcm_t *rtcm, int sys, int subtype) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  double udint;
  int sync, iod, i;
  int nsat = decode_ssr2_head(rtcm, sys, subtype, &sync, &iod, &udint, &i);
  if (nsat < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int np, offp;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  for (int j = 0; j < nsat && i + 22 + np <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, np) + offp;
    i += np;
    double hrclk = rtcm_getbits(rtcm, i, 22) * 1E-4;
    i += 22;

    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[2] = rtcm->time;
    rtcm->ssr[sat - 1].udi[2] = udint;
    rtcm->ssr[sat - 1].iod[2] = iod;
    rtcm->ssr[sat - 1].hrclk = hrclk;
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* Decode SSR 7 message header -----------------------------------------------*/
static int decode_ssr7_head(rtcm_t *rtcm, int sys, int subtype, int *sync, int *iod, double *udint,
                            int *dispe, int *mw, int *hsize) {
  int i = 24 + 12, ns;
  if (subtype == 0) { /* RTCM SSR */
    ns = (sys == SYS_QZS) ? 4 : 6;
    if (i + ((sys == SYS_GLO) ? 54 : 51 + ns) > rtcm->len * 8) return -1;
  } else { /* IGS SSR */
    ns = 6;
    if (i + 3 + 8 + 51 + ns > rtcm->len * 8) return -1;
  }
  i = decode_ssr_epoch(rtcm, sys, subtype);
  int udi = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  *sync = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  *iod = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  int provid = rtcm_getbitu(rtcm, i, 16);
  i += 16; /* Provider ID */
  int solid = rtcm_getbitu(rtcm, i, 4);
  i += 4; /* Solution ID */
  *dispe = rtcm_getbitu(rtcm, i, 1);
  i += 1; /* Dispersive bias consistency ind */
  *mw = rtcm_getbitu(rtcm, i, 1);
  i += 1; /* MW consistency indicator */
  int nsat = rtcm_getbitu(rtcm, i, ns);
  i += ns;
  *udint = ssrudint[udi];

  char tstr[40];
  time2str(rtcm->time, tstr, 2);
  trace(4,
        "decode_ssr7_head: time=%s sys=%d subtype=%d nsat=%d sync=%d iod=%d"
        " provid=%d solid=%d\n",
        tstr, sys, subtype, nsat, *sync, *iod, provid, solid);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " %s nsat=%2d iod=%2d udi=%2d sync=%d", tstr,
                 nsat, *iod, udi, *sync);
  }
  *hsize = i;
  return nsat;
}
/* Decode SSR 7: phase bias --------------------------------------------------*/
static int decode_ssr7(rtcm_t *rtcm, int sys, int subtype) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  double udint;
  int sync, iod, dispe, mw, i;
  int nsat = decode_ssr7_head(rtcm, sys, subtype, &sync, &iod, &udint, &dispe, &mw, &i);
  if (nsat < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int np, offp;
  const uint8_t *sigs;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      sigs = ssr_sig_gps;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      sigs = ssr_sig_glo;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      sigs = ssr_sig_gal;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      sigs = ssr_sig_qzs;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      sigs = ssr_sig_cmp;
      break;
    default:
      return sync ? 0 : 10;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  for (int j = 0; j < nsat && i + 5 + 17 + np <= rtcm->len * 8; j++) {
    int prn = rtcm_getbitu(rtcm, i, np) + offp;
    i += np;
    int nbias = rtcm_getbitu(rtcm, i, 5);
    i += 5;
    double yaw_ang = rtcm_getbitu(rtcm, i, 9);
    i += 9;
    double yaw_rate = rtcm_getbits(rtcm, i, 8);
    i += 8;

    double pbias[MAXCODE], stdpb[MAXCODE];
    for (int k = 0; k < MAXCODE; k++) pbias[k] = stdpb[k] = 0.0;
    for (int k = 0; k < nbias && i + ((subtype == 0) ? 49 : 32) <= rtcm->len * 8; k++) {
      int mode = rtcm_getbitu(rtcm, i, 5);
      i += 5;
      int sii = rtcm_getbitu(rtcm, i, 1);
      i += 1; /* Integer-indicator */
      int swl = rtcm_getbitu(rtcm, i, 2);
      i += 2; /* WL integer-indicator */
      int sdc = rtcm_getbitu(rtcm, i, 4);
      i += 4; /* Discontinuity counter */
      double bias = rtcm_getbits(rtcm, i, 20);
      i += 20; /* Phase bias (m) */
      double std = 0.0;
      if (subtype == 0) {
        std = rtcm_getbitu(rtcm, i, 17);
        i += 17; /* Phase bias std-dev (m) */
      }
      if (sigs[mode]) {
        pbias[sigs[mode] - 1] = bias * 0.0001; /* (m) */
        stdpb[sigs[mode] - 1] = std * 0.0001;  /* (m) */
      } else {
        trace(2, "rtcm3 %d not supported mode: mode=%d\n", type, mode);
      }
    }
    int sat = satno(sys, prn);
    if (!sat) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[5] = rtcm->time;
    rtcm->ssr[sat - 1].udi[5] = udint;
    rtcm->ssr[sat - 1].iod[5] = iod;
    rtcm->ssr[sat - 1].yaw_ang = yaw_ang / 256.0 * 180.0;    /* (deg) */
    rtcm->ssr[sat - 1].yaw_rate = yaw_rate / 8192.0 * 180.0; /* (deg/s) */

    for (int k = 0; k < MAXCODE; k++) {
      rtcm->ssr[sat - 1].pbias[k] = pbias[k];
      rtcm->ssr[sat - 1].stdpb[k] = (float)stdpb[k];
    }
  }
  return 20;
}
/* Get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const uint8_t *code, int n, const char *opt, int *idx) {
  /* Test code priority */
  int pri_h[8] = {0}, index[8] = {0}, ex[32] = {0};
  for (int i = 0; i < n; i++) {
    if (!code[i]) continue;

    if (idx[i] >= NFREQ) { /* Save as extended signal if idx >= NFREQ */
      ex[i] = 1;
      continue;
    }
    /* Code priority */
    int pri = getcodepri(sys, code[i], opt);

    /* Select highest priority signal */
    if (pri > pri_h[idx[i]]) {
      if (index[idx[i]]) ex[index[idx[i]] - 1] = 1;
      pri_h[idx[i]] = pri;
      index[idx[i]] = i + 1;
    } else
      ex[i] = 1;
  }
  /* Signal index in obs data */
  for (int i = 0, nex = 0; i < n; i++) {
    if (ex[i] == 0)
      ;
    else if (nex < NEXOBS)
      idx[i] = NFREQ + nex++;
    else { /* No space in obs data */
      trace(2, "rtcm msm: no space in obs data sys=%d code=%d\n", sys, code[i]);
      idx[i] = -1;
    }
#ifdef RTK_DISABLED /* For debug */
    trace(2, "sig pos: sys=%d code=%d ex=%d idx=%d\n", sys, code[i], ex[i], idx[i]);
#endif
  }
}
/* Save obs data in MSM message ----------------------------------------------*/
static void save_msm_obs(rtcm_t *rtcm, int sys, msm_h_t *h, const double *r, const double *pr,
                         const double *cp, const double *rr, const double *rrf, const double *cnr,
                         const int *lock, const int *ex, const int *half) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  char *msm_type = "", *q = NULL;
  switch (sys) {
    case SYS_GPS:
      msm_type = q = rtcm->msmtype[0];
      break;
    case SYS_GLO:
      msm_type = q = rtcm->msmtype[1];
      break;
    case SYS_GAL:
      msm_type = q = rtcm->msmtype[2];
      break;
    case SYS_QZS:
      msm_type = q = rtcm->msmtype[3];
      break;
    case SYS_SBS:
      msm_type = q = rtcm->msmtype[4];
      break;
    case SYS_CMP:
      msm_type = q = rtcm->msmtype[5];
      break;
    case SYS_IRN:
      msm_type = q = rtcm->msmtype[6];
      break;
  }
  /* Id to signal */
  const char *sig[32];
  uint8_t code[32];
  int idx[32];
  for (int i = 0; i < h->nsig; i++) {
    switch (sys) {
      case SYS_GPS:
        sig[i] = msm_sig_gps[h->sigs[i] - 1];
        break;
      case SYS_GLO:
        sig[i] = msm_sig_glo[h->sigs[i] - 1];
        break;
      case SYS_GAL:
        sig[i] = msm_sig_gal[h->sigs[i] - 1];
        break;
      case SYS_QZS:
        sig[i] = msm_sig_qzs[h->sigs[i] - 1];
        break;
      case SYS_SBS:
        sig[i] = msm_sig_sbs[h->sigs[i] - 1];
        break;
      case SYS_CMP:
        sig[i] = msm_sig_cmp[h->sigs[i] - 1];
        break;
      case SYS_IRN:
        sig[i] = msm_sig_irn[h->sigs[i] - 1];
        break;
      default:
        sig[i] = "";
        break;
    }
    /* Signal to RINEX obs type */
    code[i] = obs2code(sig[i]);
    idx[i] = code2idx(sys, code[i]);

    if (code[i] != CODE_NONE) {
      if (q) rtksnprintf(q, sizeof(rtcm->msmtype[0]), "L%s%s", sig[i], i < h->nsig - 1 ? "," : "");
    } else {
      if (q)
        rtksnprintf(q, sizeof(rtcm->msmtype[0]), "(%d)%s", h->sigs[i], i < h->nsig - 1 ? "," : "");

      trace(2, "rtcm3 %d: unknown signal id=%2d\n", type, h->sigs[i]);
    }
  }
  trace(3, "rtcm3 %d: signals=%s\n", type, msm_type);

  /* Get signal index */
  sigindex(sys, code, h->nsig, rtcm->opt, idx);

  int index = 0;
  for (int i = 0, j = 0; i < h->nsat; i++) {
    int prn = h->sats[i];
    if (sys == SYS_QZS)
      prn += MINPRNQZS - 1;
    else if (sys == SYS_SBS)
      prn += MINPRNSBS - 1;

    int sat = satno(sys, prn);
    if (sat) {
      double tt = timediff(rtcm->obs.data[0].time, rtcm->time);
      if (rtcm->obsflag || fabs(tt) > 1E-9) {
        rtcm->obs.n = rtcm->obsflag = 0;
      }
      index = obsindex(&rtcm->obs, rtcm->time, sat);
    } else {
      trace(2, "rtcm3 %d satellite error: prn=%d\n", type, prn);
    }
    int fcn = 0;
    if (sys == SYS_GLO) {
      fcn = -8; /* No GLONASS fcn info */
      if (ex && ex[i] <= 13) {
        fcn = ex[i] - 7;
        if (!rtcm->nav.glo_fcn[prn - 1]) {
          rtcm->nav.glo_fcn[prn - 1] = fcn + 8; /* fcn+8 */
        }
      } else if (rtcm->nav.geph[prn - 1][0].sat == sat) {
        fcn = rtcm->nav.geph[prn - 1][0].frq;
      } else if (rtcm->nav.glo_fcn[prn - 1] > 0) {
        fcn = rtcm->nav.glo_fcn[prn - 1] - 8;
      }
    }
    for (int k = 0; k < h->nsig; k++) {
      if (!h->cellmask[k + i * h->nsig]) continue;

      if (sat && index >= 0 && idx[k] >= 0) {
        double freq = fcn < -7 ? 0.0 : code2freq(sys, code[k], fcn);

        /* Pseudorange (m) */
        if (r[i] != 0.0 && pr[j] > -1E12) {
          rtcm->obs.data[index].P[idx[k]] = r[i] + pr[j];
        }
        /* Carrier-phase (cycle) */
        if (r[i] != 0.0 && cp[j] > -1E12) {
          rtcm->obs.data[index].L[idx[k]] = (r[i] + cp[j]) * freq / CLIGHT;
        }
        /* Doppler (hz) */
        if (rr && rrf && rrf[j] > -1E12) {
          rtcm->obs.data[index].D[idx[k]] = (float)(-(rr[i] + rrf[j]) * freq / CLIGHT);
        }
        rtcm->obs.data[index].LLI[idx[k]] =
            lossoflock(rtcm, sat, idx[k], lock[j]) + (half[j] ? 2 : 0);
        rtcm->obs.data[index].SNR[idx[k]] = (uint16_t)(cnr[j] / SNR_UNIT + 0.5);
        rtcm->obs.data[index].code[idx[k]] = code[k];
      }
      j++;
    }
  }
}
/* Decode type MSM message header --------------------------------------------*/
static int decode_msm_head(rtcm_t *rtcm, int sys, int *sync, int *iod, msm_h_t *h, int *hsize) {
  int i = 24;
  int type = rtcm_getbitu(rtcm, i, 12);
  i += 12;

  msm_h_t h0 = {0};
  *h = h0;
  if (i + 157 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12;

  if (sys == SYS_GLO) {
    int dow = rtcm_getbitu(rtcm, i, 3);
    i += 3;
    double tod = rtcm_getbitu(rtcm, i, 27) * 0.001;
    i += 27;
    adjday_glot(rtcm, tod);
  } else if (sys == SYS_CMP) {
    double tow = rtcm_getbitu(rtcm, i, 30) * 0.001;
    i += 30;
    tow += 14.0; /* BDT -> GPST */
    adjweek(rtcm, tow);
  } else {
    double tow = rtcm_getbitu(rtcm, i, 30) * 0.001;
    i += 30;
    adjweek(rtcm, tow);
  }
  *sync = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  *iod = rtcm_getbitu(rtcm, i, 3);
  i += 3;
  h->time_s = rtcm_getbitu(rtcm, i, 7);
  i += 7;
  h->clk_str = rtcm_getbitu(rtcm, i, 2);
  i += 2;
  h->clk_ext = rtcm_getbitu(rtcm, i, 2);
  i += 2;
  h->smooth = rtcm_getbitu(rtcm, i, 1);
  i += 1;
  h->tint_s = rtcm_getbitu(rtcm, i, 3);
  i += 3;
  for (int j = 1; j <= 64; j++) {
    int mask = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    if (mask) h->sats[h->nsat++] = j;
  }
  for (int j = 1; j <= 32; j++) {
    int mask = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    if (mask) h->sigs[h->nsig++] = j;
  }
  /* Test station id */
  if (!test_staid(rtcm, staid)) return -1;

  if (h->nsat * h->nsig > 64) {
    trace(2, "rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n", type, h->nsat, h->nsig);
    return -1;
  }
  if (i + h->nsat * h->nsig > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: len=%d nsat=%d nsig=%d\n", type, rtcm->len, h->nsat, h->nsig);
    return -1;
  }
  int ncell = 0;
  for (int j = 0; j < h->nsat * h->nsig; j++) {
    h->cellmask[j] = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    if (h->cellmask[j]) ncell++;
  }
  *hsize = i;

  char tstr[40];
  time2str(rtcm->time, tstr, 2);
  trace(4, "decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
        tstr, sys, staid, h->nsat, h->nsig, *sync, *iod, ncell);

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                 " staid=%4d %s nsat=%2d nsig=%2d iod=%2d ncell=%2d sync=%d", staid, tstr, h->nsat,
                 h->nsig, *iod, ncell, *sync);
  }
  return ncell;
}
/* Decode unsupported MSM message --------------------------------------------*/
static int decode_msm0(rtcm_t *rtcm, int sys) {
  msm_h_t h = {0};
  int sync, iod, i;
  if (decode_msm_head(rtcm, sys, &sync, &iod, &h, &i) < 0) return -1;
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
static int decode_msm4(rtcm_t *rtcm, int sys) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  /* Decode MSM header */
  msm_h_t h = {0};
  int sync, iod, i;
  int ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i);
  if (ncell < 0) return -1;

  if (i + h.nsat * 18 + ncell * 48 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat, ncell, rtcm->len);
    rtcm->obsflag = !sync; /* Header ok, so return sync bit */
    return sync ? 0 : 1;
  }
  double r[64];
  for (int j = 0; j < h.nsat; j++) r[j] = 0.0;
  double pr[64], cp[64];
  for (int j = 0; j < ncell; j++) pr[j] = cp[j] = -1E16;

  /* Decode satellite data */
  for (int j = 0; j < h.nsat; j++) { /* Range */
    int rng = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    if (rng != 255) r[j] = rng * RANGE_MS;
  }
  for (int j = 0; j < h.nsat; j++) {
    int rng_m = rtcm_getbitu(rtcm, i, 10);
    i += 10;
    if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
  }
  /* Decode signal data */
  for (int j = 0; j < ncell; j++) { /* Pseudorange */
    int prv = rtcm_getbits(rtcm, i, 15);
    i += 15;
    if (prv != -16384) pr[j] = prv * P2_24 * RANGE_MS;
  }
  for (int j = 0; j < ncell; j++) { /* Phaserange */
    int cpv = rtcm_getbits(rtcm, i, 22);
    i += 22;
    if (cpv != -2097152) cp[j] = cpv * P2_29 * RANGE_MS;
  }
  int lock[64];
  for (int j = 0; j < ncell; j++) { /* Lock time */
    lock[j] = rtcm_getbitu(rtcm, i, 4);
    i += 4;
  }
  int half[64];
  for (int j = 0; j < ncell; j++) { /* Half-cycle ambiguity */
    half[j] = rtcm_getbitu(rtcm, i, 1);
    i += 1;
  }
  double cnr[64];
  for (int j = 0; j < ncell; j++) { /* Cnr */
    cnr[j] = rtcm_getbitu(rtcm, i, 6) * 1.0;
    i += 6;
  }
  /* Save obs data in MSM message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode MSM 5: full pseudorange, phaserange, phaserangerate and CNR --------*/
static int decode_msm5(rtcm_t *rtcm, int sys) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  /* Decode MSM header */
  msm_h_t h = {0};
  int sync, iod, i;
  int ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i);
  if (ncell < 0) return -1;

  if (i + h.nsat * 36 + ncell * 63 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat, ncell, rtcm->len);
    rtcm->obsflag = !sync; /* Header ok, so return sync bit */
    return sync ? 0 : 1;
  }
  int ex[64];
  double r[64], rr[64];
  for (int j = 0; j < h.nsat; j++) {
    r[j] = rr[j] = 0.0;
    ex[j] = 15;
  }
  double pr[64], cp[64], rrf[64];
  for (int j = 0; j < ncell; j++) pr[j] = cp[j] = rrf[j] = -1E16;

  /* Decode satellite data */
  for (int j = 0; j < h.nsat; j++) { /* Range */
    int rng = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    if (rng != 255) r[j] = rng * RANGE_MS;
  }
  for (int j = 0; j < h.nsat; j++) { /* Extended info */
    ex[j] = rtcm_getbitu(rtcm, i, 4);
    i += 4;
  }
  for (int j = 0; j < h.nsat; j++) {
    int rng_m = rtcm_getbitu(rtcm, i, 10);
    i += 10;
    if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
  }
  for (int j = 0; j < h.nsat; j++) { /* Phaserangerate */
    int rate = rtcm_getbits(rtcm, i, 14);
    i += 14;
    if (rate != -8192) rr[j] = rate * 1.0;
  }
  /* Decode signal data */
  for (int j = 0; j < ncell; j++) { /* Pseudorange */
    int prv = rtcm_getbits(rtcm, i, 15);
    i += 15;
    if (prv != -16384) pr[j] = prv * P2_24 * RANGE_MS;
  }
  for (int j = 0; j < ncell; j++) { /* Phaserange */
    int cpv = rtcm_getbits(rtcm, i, 22);
    i += 22;
    if (cpv != -2097152) cp[j] = cpv * P2_29 * RANGE_MS;
  }
  int lock[64];
  for (int j = 0; j < ncell; j++) { /* Lock time */
    lock[j] = rtcm_getbitu(rtcm, i, 4);
    i += 4;
  }
  int half[64];
  for (int j = 0; j < ncell; j++) { /* Half-cycle ambiguity */
    half[j] = rtcm_getbitu(rtcm, i, 1);
    i += 1;
  }
  double cnr[64];
  for (int j = 0; j < ncell; j++) { /* Cnr */
    cnr[j] = rtcm_getbitu(rtcm, i, 6) * 1.0;
    i += 6;
  }
  for (int j = 0; j < ncell; j++) { /* Phaserangerate */
    int rrv = rtcm_getbits(rtcm, i, 15);
    i += 15;
    if (rrv != -16384) rrf[j] = rrv * 0.0001;
  }
  /* Save obs data in MSM message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode MSM 6: full pseudorange and phaserange plus CNR (high-res) ---------*/
static int decode_msm6(rtcm_t *rtcm, int sys) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  /* Decode MSM header */
  msm_h_t h = {0};
  int sync, iod, i;
  int ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i);
  if (ncell < 0) return -1;

  if (i + h.nsat * 18 + ncell * 65 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat, ncell, rtcm->len);
    rtcm->obsflag = !sync; /* Header ok, so return sync bit */
    return sync ? 0 : 1;
  }
  double r[64], pr[64], cp[64];
  for (int j = 0; j < h.nsat; j++) r[j] = 0.0;
  for (int j = 0; j < ncell; j++) pr[j] = cp[j] = -1E16;

  /* Decode satellite data */
  for (int j = 0; j < h.nsat; j++) { /* Range */
    int rng = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    if (rng != 255) r[j] = rng * RANGE_MS;
  }
  for (int j = 0; j < h.nsat; j++) {
    int rng_m = rtcm_getbitu(rtcm, i, 10);
    i += 10;
    if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
  }
  /* Decode signal data */
  for (int j = 0; j < ncell; j++) { /* Pseudorange */
    int prv = rtcm_getbits(rtcm, i, 20);
    i += 20;
    if (prv != -524288) pr[j] = prv * P2_29 * RANGE_MS;
  }
  for (int j = 0; j < ncell; j++) { /* Phaserange */
    int cpv = rtcm_getbits(rtcm, i, 24);
    i += 24;
    if (cpv != -8388608) cp[j] = cpv * P2_31 * RANGE_MS;
  }
  int lock[64];
  for (int j = 0; j < ncell; j++) { /* Lock time */
    lock[j] = rtcm_getbitu(rtcm, i, 10);
    i += 10;
  }
  int half[64];
  for (int j = 0; j < ncell; j++) { /* Half-cycle ambiguity */
    half[j] = rtcm_getbitu(rtcm, i, 1);
    i += 1;
  }
  double cnr[64];
  for (int j = 0; j < ncell; j++) { /* Cnr */
    cnr[j] = rtcm_getbitu(rtcm, i, 10) * 0.0625;
    i += 10;
  }
  /* Save obs data in MSM message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode MSM 7: full pseudorange, phaserange, phaserangerate and CNR (h-res) */
static int decode_msm7(rtcm_t *rtcm, int sys) {
  int type = rtcm_getbitu(rtcm, 24, 12);

  /* Decode MSM header */
  msm_h_t h = {0};
  int sync, iod, i;
  int ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i);
  if (ncell < 0) return -1;

  if (i + h.nsat * 36 + ncell * 80 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat, ncell, rtcm->len);
    rtcm->obsflag = !sync; /* Header ok, so return sync bit */
    return sync ? 0 : 1;
  }
  int ex[64];
  double r[64], rr[64];
  for (int j = 0; j < h.nsat; j++) {
    r[j] = rr[j] = 0.0;
    ex[j] = 15;
  }
  double pr[64], cp[64], rrf[64];
  for (int j = 0; j < ncell; j++) pr[j] = cp[j] = rrf[j] = -1E16;

  /* Decode satellite data */
  for (int j = 0; j < h.nsat; j++) { /* Range */
    int rng = rtcm_getbitu(rtcm, i, 8);
    i += 8;
    if (rng != 255) r[j] = rng * RANGE_MS;
  }
  for (int j = 0; j < h.nsat; j++) { /* Extended info */
    ex[j] = rtcm_getbitu(rtcm, i, 4);
    i += 4;
  }
  for (int j = 0; j < h.nsat; j++) {
    int rng_m = rtcm_getbitu(rtcm, i, 10);
    i += 10;
    if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
  }
  for (int j = 0; j < h.nsat; j++) { /* Phaserangerate */
    int rate = rtcm_getbits(rtcm, i, 14);
    i += 14;
    if (rate != -8192) {
      rr[j] = rate * 1.0;
      if (strstr(rtcm->opt, "-INVPRR")) rr[j] = -rr[j];
    }
  }
  /* Decode signal data */
  for (int j = 0; j < ncell; j++) { /* Pseudorange */
    int prv = rtcm_getbits(rtcm, i, 20);
    i += 20;
    if (prv != -524288) pr[j] = prv * P2_29 * RANGE_MS;
  }
  for (int j = 0; j < ncell; j++) { /* Phaserange */
    int cpv = rtcm_getbits(rtcm, i, 24);
    i += 24;
    if (cpv != -8388608) cp[j] = cpv * P2_31 * RANGE_MS;
  }
  int lock[64];
  for (int j = 0; j < ncell; j++) { /* Lock time */
    lock[j] = rtcm_getbitu(rtcm, i, 10);
    i += 10;
  }
  int half[64];
  for (int j = 0; j < ncell; j++) { /* Half-cycle amiguity */
    half[j] = rtcm_getbitu(rtcm, i, 1);
    i += 1;
  }
  double cnr[64];
  for (int j = 0; j < ncell; j++) { /* Cnr */
    cnr[j] = rtcm_getbitu(rtcm, i, 10) * 0.0625;
    i += 10;
  }
  for (int j = 0; j < ncell; j++) { /* Phaserangerate */
    int rrv = rtcm_getbits(rtcm, i, 15);
    i += 15;
    if (rrv != -16384) {
      rrf[j] = rrv * 0.0001;
      if (strstr(rtcm->opt, "-INVPRR")) rrf[j] = -rrf[j];
    }
  }
  /* Save obs data in MSM message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
static int decode_type1230(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 20 >= rtcm->len * 8) {
    trace(2, "rtcm3 1230: length error len=%d\n", rtcm->len);
    return -1;
  }
  int staid = rtcm_getbitu(rtcm, i, 12);
  i += 12;
  int align = rtcm_getbitu(rtcm, i, 1);
  i += 1 + 3;
  int mask = rtcm_getbitu(rtcm, i, 4);
  i += 4;

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " staid=%4d align=%d mask=0x%X", staid,
                 align, mask);
  }
  /* Test station ID */
  if (!test_staid(rtcm, staid)) return -1;

  rtcm->sta.glo_cp_align = align;
  for (int j = 0; j < 4; j++) {
    rtcm->sta.glo_cp_bias[j] = 0.0;
  }
  for (int j = 0; j < 4 && i + 16 <= rtcm->len * 8; j++) {
    if (!(mask & (1 << (3 - j)))) continue;
    int bias = rtcm_getbits(rtcm, i, 16);
    i += 16;
    if (bias != -32768) {
      rtcm->sta.glo_cp_bias[j] = bias * 0.02;
    }
  }
  return 5;
}
/* Decode type 4073: proprietary message Mitsubishi Electric -----------------*/
static int decode_type4073(rtcm_t *rtcm) {
  int i = 24 + 12;
  int subtype = rtcm_getbitu(rtcm, i, 4);
  i += 4;

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " subtype=%d", subtype);
  }
  trace(2, "rtcm3 4073: unsupported message subtype=%d\n", subtype);
  return 0;
}
/* Decode type 4076: proprietary message IGS ---------------------------------*/
static int decode_type4076(rtcm_t *rtcm) {
  int i = 24 + 12;
  if (i + 3 + 8 >= rtcm->len * 8) {
    trace(2, "rtcm3 4076: length error len=%d\n", rtcm->len);
    return -1;
  }
  int ver = rtcm_getbitu(rtcm, i, 3);
  i += 3;
  int subtype = rtcm_getbitu(rtcm, i, 8);
  i += 8;

  if (rtcm->outtype) {
    rtkcatprintf(rtcm->msgtype, sizeof(rtcm->msgtype), " ver=%d subtype=%3d", ver, subtype);
  }
  switch (subtype) {
    case 21:
      return decode_ssr1(rtcm, SYS_GPS, subtype);
    case 22:
      return decode_ssr2(rtcm, SYS_GPS, subtype);
    case 23:
      return decode_ssr4(rtcm, SYS_GPS, subtype);
    case 24:
      return decode_ssr6(rtcm, SYS_GPS, subtype);
    case 25:
      return decode_ssr3(rtcm, SYS_GPS, subtype);
    case 26:
      return decode_ssr7(rtcm, SYS_GPS, subtype);
    case 27:
      return decode_ssr5(rtcm, SYS_GPS, subtype);
    case 41:
      return decode_ssr1(rtcm, SYS_GLO, subtype);
    case 42:
      return decode_ssr2(rtcm, SYS_GLO, subtype);
    case 43:
      return decode_ssr4(rtcm, SYS_GLO, subtype);
    case 44:
      return decode_ssr6(rtcm, SYS_GLO, subtype);
    case 45:
      return decode_ssr3(rtcm, SYS_GLO, subtype);
    case 46:
      return decode_ssr7(rtcm, SYS_GLO, subtype);
    case 47:
      return decode_ssr5(rtcm, SYS_GLO, subtype);
    case 61:
      return decode_ssr1(rtcm, SYS_GAL, subtype);
    case 62:
      return decode_ssr2(rtcm, SYS_GAL, subtype);
    case 63:
      return decode_ssr4(rtcm, SYS_GAL, subtype);
    case 64:
      return decode_ssr6(rtcm, SYS_GAL, subtype);
    case 65:
      return decode_ssr3(rtcm, SYS_GAL, subtype);
    case 66:
      return decode_ssr7(rtcm, SYS_GAL, subtype);
    case 67:
      return decode_ssr5(rtcm, SYS_GAL, subtype);
    case 81:
      return decode_ssr1(rtcm, SYS_QZS, subtype);
    case 82:
      return decode_ssr2(rtcm, SYS_QZS, subtype);
    case 83:
      return decode_ssr4(rtcm, SYS_QZS, subtype);
    case 84:
      return decode_ssr6(rtcm, SYS_QZS, subtype);
    case 85:
      return decode_ssr3(rtcm, SYS_QZS, subtype);
    case 86:
      return decode_ssr7(rtcm, SYS_QZS, subtype);
    case 87:
      return decode_ssr5(rtcm, SYS_QZS, subtype);
    case 101:
      return decode_ssr1(rtcm, SYS_CMP, subtype);
    case 102:
      return decode_ssr2(rtcm, SYS_CMP, subtype);
    case 103:
      return decode_ssr4(rtcm, SYS_CMP, subtype);
    case 104:
      return decode_ssr6(rtcm, SYS_CMP, subtype);
    case 105:
      return decode_ssr3(rtcm, SYS_CMP, subtype);
    case 106:
      return decode_ssr7(rtcm, SYS_CMP, subtype);
    case 107:
      return decode_ssr5(rtcm, SYS_CMP, subtype);
    case 121:
      return decode_ssr1(rtcm, SYS_SBS, subtype);
    case 122:
      return decode_ssr2(rtcm, SYS_SBS, subtype);
    case 123:
      return decode_ssr4(rtcm, SYS_SBS, subtype);
    case 124:
      return decode_ssr6(rtcm, SYS_SBS, subtype);
    case 125:
      return decode_ssr3(rtcm, SYS_SBS, subtype);
    case 126:
      return decode_ssr7(rtcm, SYS_SBS, subtype);
    case 127:
      return decode_ssr5(rtcm, SYS_SBS, subtype);
  }
  trace(2, "rtcm3 4076: unsupported message subtype=%d\n", subtype);
  return 0;
}
/* Decode RTCM ver.3 message -------------------------------------------------*/
extern int decode_rtcm3(rtcm_t *rtcm) {
  int type = rtcm_getbitu(rtcm, 24, 12);
  trace(3, "decode_rtcm3: len=%3d type=%d\n", rtcm->len, type);

  if (rtcm->outtype) {
    rtksnprintf(rtcm->msgtype, sizeof(rtcm->msgtype), "RTCM %4d (%4d):", type, rtcm->len);
  }
  /* Real-time input option */
  if (strstr(rtcm->opt, "-RT_INP")) {
    int week;
    double tow = time2gpst(utc2gpst(timeget()), &week);
    rtcm->time = gpst2time(week, floor(tow));
  }
  int ret = 0;
  switch (type) {
    case 1001:
      ret = decode_type1001(rtcm);
      break; /* Not supported */
    case 1002:
      ret = decode_type1002(rtcm);
      break;
    case 1003:
      ret = decode_type1003(rtcm);
      break; /* Not supported */
    case 1004:
      ret = decode_type1004(rtcm);
      break;
    case 1005:
      ret = decode_type1005(rtcm);
      break;
    case 1006:
      ret = decode_type1006(rtcm);
      break;
    case 1007:
      ret = decode_type1007(rtcm);
      break;
    case 1008:
      ret = decode_type1008(rtcm);
      break;
    case 1009:
      ret = decode_type1009(rtcm);
      break; /* Not supported */
    case 1010:
      ret = decode_type1010(rtcm);
      break;
    case 1011:
      ret = decode_type1011(rtcm);
      break; /* Not supported */
    case 1012:
      ret = decode_type1012(rtcm);
      break;
    case 1013:
      ret = decode_type1013(rtcm);
      break; /* Not supported */
    case 1019:
      ret = decode_type1019(rtcm);
      break;
    case 1020:
      ret = decode_type1020(rtcm);
      break;
    case 1021:
      ret = decode_type1021(rtcm);
      break; /* Not supported */
    case 1022:
      ret = decode_type1022(rtcm);
      break; /* Not supported */
    case 1023:
      ret = decode_type1023(rtcm);
      break; /* Not supported */
    case 1024:
      ret = decode_type1024(rtcm);
      break; /* Not supported */
    case 1025:
      ret = decode_type1025(rtcm);
      break; /* Not supported */
    case 1026:
      ret = decode_type1026(rtcm);
      break; /* Not supported */
    case 1027:
      ret = decode_type1027(rtcm);
      break; /* Not supported */
    case 1029:
      ret = decode_type1029(rtcm);
      break;
    case 1030:
      ret = decode_type1030(rtcm);
      break; /* Not supported */
    case 1031:
      ret = decode_type1031(rtcm);
      break; /* Not supported */
    case 1032:
      ret = decode_type1032(rtcm);
      break; /* Not supported */
    case 1033:
      ret = decode_type1033(rtcm);
      break;
    case 1034:
      ret = decode_type1034(rtcm);
      break; /* Not supported */
    case 1035:
      ret = decode_type1035(rtcm);
      break; /* Not supported */
    case 1037:
      ret = decode_type1037(rtcm);
      break; /* Not supported */
    case 1038:
      ret = decode_type1038(rtcm);
      break; /* Not supported */
    case 1039:
      ret = decode_type1039(rtcm);
      break; /* Not supported */
    case 1041:
      ret = decode_type1041(rtcm);
      break;
    case 1044:
      ret = decode_type1044(rtcm);
      break;
    case 1045:
      ret = decode_type1045(rtcm);
      break;
    case 1046:
      ret = decode_type1046(rtcm);
      break;
    case 63:
      ret = decode_type1042(rtcm);
      break; /* RTCM draft */
    case 1042:
      ret = decode_type1042(rtcm);
      break;
    case 1057:
      ret = decode_ssr1(rtcm, SYS_GPS, 0);
      break;
    case 1058:
      ret = decode_ssr2(rtcm, SYS_GPS, 0);
      break;
    case 1059:
      ret = decode_ssr3(rtcm, SYS_GPS, 0);
      break;
    case 1060:
      ret = decode_ssr4(rtcm, SYS_GPS, 0);
      break;
    case 1061:
      ret = decode_ssr5(rtcm, SYS_GPS, 0);
      break;
    case 1062:
      ret = decode_ssr6(rtcm, SYS_GPS, 0);
      break;
    case 1063:
      ret = decode_ssr1(rtcm, SYS_GLO, 0);
      break;
    case 1064:
      ret = decode_ssr2(rtcm, SYS_GLO, 0);
      break;
    case 1065:
      ret = decode_ssr3(rtcm, SYS_GLO, 0);
      break;
    case 1066:
      ret = decode_ssr4(rtcm, SYS_GLO, 0);
      break;
    case 1067:
      ret = decode_ssr5(rtcm, SYS_GLO, 0);
      break;
    case 1068:
      ret = decode_ssr6(rtcm, SYS_GLO, 0);
      break;
    case 1071:
      ret = decode_msm0(rtcm, SYS_GPS);
      break; /* Not supported */
    case 1072:
      ret = decode_msm0(rtcm, SYS_GPS);
      break; /* Not supported */
    case 1073:
      ret = decode_msm0(rtcm, SYS_GPS);
      break; /* Not supported */
    case 1074:
      ret = decode_msm4(rtcm, SYS_GPS);
      break;
    case 1075:
      ret = decode_msm5(rtcm, SYS_GPS);
      break;
    case 1076:
      ret = decode_msm6(rtcm, SYS_GPS);
      break;
    case 1077:
      ret = decode_msm7(rtcm, SYS_GPS);
      break;
    case 1081:
      ret = decode_msm0(rtcm, SYS_GLO);
      break; /* Not supported */
    case 1082:
      ret = decode_msm0(rtcm, SYS_GLO);
      break; /* Not supported */
    case 1083:
      ret = decode_msm0(rtcm, SYS_GLO);
      break; /* Not supported */
    case 1084:
      ret = decode_msm4(rtcm, SYS_GLO);
      break;
    case 1085:
      ret = decode_msm5(rtcm, SYS_GLO);
      break;
    case 1086:
      ret = decode_msm6(rtcm, SYS_GLO);
      break;
    case 1087:
      ret = decode_msm7(rtcm, SYS_GLO);
      break;
    case 1091:
      ret = decode_msm0(rtcm, SYS_GAL);
      break; /* Not supported */
    case 1092:
      ret = decode_msm0(rtcm, SYS_GAL);
      break; /* Not supported */
    case 1093:
      ret = decode_msm0(rtcm, SYS_GAL);
      break; /* Not supported */
    case 1094:
      ret = decode_msm4(rtcm, SYS_GAL);
      break;
    case 1095:
      ret = decode_msm5(rtcm, SYS_GAL);
      break;
    case 1096:
      ret = decode_msm6(rtcm, SYS_GAL);
      break;
    case 1097:
      ret = decode_msm7(rtcm, SYS_GAL);
      break;
    case 1101:
      ret = decode_msm0(rtcm, SYS_SBS);
      break; /* Not supported */
    case 1102:
      ret = decode_msm0(rtcm, SYS_SBS);
      break; /* Not supported */
    case 1103:
      ret = decode_msm0(rtcm, SYS_SBS);
      break; /* Not supported */
    case 1104:
      ret = decode_msm4(rtcm, SYS_SBS);
      break;
    case 1105:
      ret = decode_msm5(rtcm, SYS_SBS);
      break;
    case 1106:
      ret = decode_msm6(rtcm, SYS_SBS);
      break;
    case 1107:
      ret = decode_msm7(rtcm, SYS_SBS);
      break;
    case 1111:
      ret = decode_msm0(rtcm, SYS_QZS);
      break; /* Not supported */
    case 1112:
      ret = decode_msm0(rtcm, SYS_QZS);
      break; /* Not supported */
    case 1113:
      ret = decode_msm0(rtcm, SYS_QZS);
      break; /* Not supported */
    case 1114:
      ret = decode_msm4(rtcm, SYS_QZS);
      break;
    case 1115:
      ret = decode_msm5(rtcm, SYS_QZS);
      break;
    case 1116:
      ret = decode_msm6(rtcm, SYS_QZS);
      break;
    case 1117:
      ret = decode_msm7(rtcm, SYS_QZS);
      break;
    case 1121:
      ret = decode_msm0(rtcm, SYS_CMP);
      break; /* Not supported */
    case 1122:
      ret = decode_msm0(rtcm, SYS_CMP);
      break; /* Not supported */
    case 1123:
      ret = decode_msm0(rtcm, SYS_CMP);
      break; /* Not supported */
    case 1124:
      ret = decode_msm4(rtcm, SYS_CMP);
      break;
    case 1125:
      ret = decode_msm5(rtcm, SYS_CMP);
      break;
    case 1126:
      ret = decode_msm6(rtcm, SYS_CMP);
      break;
    case 1127:
      ret = decode_msm7(rtcm, SYS_CMP);
      break;
    case 1131:
      ret = decode_msm0(rtcm, SYS_IRN);
      break; /* Not supported */
    case 1132:
      ret = decode_msm0(rtcm, SYS_IRN);
      break; /* Not supported */
    case 1133:
      ret = decode_msm0(rtcm, SYS_IRN);
      break; /* Not supported */
    case 1134:
      ret = decode_msm4(rtcm, SYS_IRN);
      break;
    case 1135:
      ret = decode_msm5(rtcm, SYS_IRN);
      break;
    case 1136:
      ret = decode_msm6(rtcm, SYS_IRN);
      break;
    case 1137:
      ret = decode_msm7(rtcm, SYS_IRN);
      break;
    case 1230:
      ret = decode_type1230(rtcm);
      break;
    case 1240:
      ret = decode_ssr1(rtcm, SYS_GAL, 0);
      break; /* Draft */
    case 1241:
      ret = decode_ssr2(rtcm, SYS_GAL, 0);
      break; /* Draft */
    case 1242:
      ret = decode_ssr3(rtcm, SYS_GAL, 0);
      break; /* Draft */
    case 1243:
      ret = decode_ssr4(rtcm, SYS_GAL, 0);
      break; /* Draft */
    case 1244:
      ret = decode_ssr5(rtcm, SYS_GAL, 0);
      break; /* Draft */
    case 1245:
      ret = decode_ssr6(rtcm, SYS_GAL, 0);
      break; /* Draft */
    case 1246:
      ret = decode_ssr1(rtcm, SYS_QZS, 0);
      break; /* Draft */
    case 1247:
      ret = decode_ssr2(rtcm, SYS_QZS, 0);
      break; /* Draft */
    case 1248:
      ret = decode_ssr3(rtcm, SYS_QZS, 0);
      break; /* Draft */
    case 1249:
      ret = decode_ssr4(rtcm, SYS_QZS, 0);
      break; /* Draft */
    case 1250:
      ret = decode_ssr5(rtcm, SYS_QZS, 0);
      break; /* Draft */
    case 1251:
      ret = decode_ssr6(rtcm, SYS_QZS, 0);
      break; /* Draft */
    case 1252:
      ret = decode_ssr1(rtcm, SYS_SBS, 0);
      break; /* Draft */
    case 1253:
      ret = decode_ssr2(rtcm, SYS_SBS, 0);
      break; /* Draft */
    case 1254:
      ret = decode_ssr3(rtcm, SYS_SBS, 0);
      break; /* Draft */
    case 1255:
      ret = decode_ssr4(rtcm, SYS_SBS, 0);
      break; /* Draft */
    case 1256:
      ret = decode_ssr5(rtcm, SYS_SBS, 0);
      break; /* Draft */
    case 1257:
      ret = decode_ssr6(rtcm, SYS_SBS, 0);
      break; /* Draft */
    case 1258:
      ret = decode_ssr1(rtcm, SYS_CMP, 0);
      break; /* Draft */
    case 1259:
      ret = decode_ssr2(rtcm, SYS_CMP, 0);
      break; /* Draft */
    case 1260:
      ret = decode_ssr3(rtcm, SYS_CMP, 0);
      break; /* Draft */
    case 1261:
      ret = decode_ssr4(rtcm, SYS_CMP, 0);
      break; /* Draft */
    case 1262:
      ret = decode_ssr5(rtcm, SYS_CMP, 0);
      break; /* Draft */
    case 1263:
      ret = decode_ssr6(rtcm, SYS_CMP, 0);
      break; /* Draft */
    case 11:
      ret = decode_ssr7(rtcm, SYS_GPS, 0);
      break; /* Tentative */
    case 12:
      ret = decode_ssr7(rtcm, SYS_GAL, 0);
      break; /* Tentative */
    case 13:
      ret = decode_ssr7(rtcm, SYS_QZS, 0);
      break; /* Tentative */
    case 14:
      ret = decode_ssr7(rtcm, SYS_CMP, 0);
      break; /* Tentative */
    case 4073:
      ret = decode_type4073(rtcm);
      break;
    case 4076:
      ret = decode_type4076(rtcm);
      break;
  }
  if (ret >= 0) {
    if (1001 <= type && type <= 1299)
      rtcm->nmsg3[type - 1000]++; /*   1-299 */
    else if (4070 <= type && type <= 4099)
      rtcm->nmsg3[type - 3770]++; /* 300-329 */
    else
      rtcm->nmsg3[0]++; /* Other */
  }
  return ret;
}
