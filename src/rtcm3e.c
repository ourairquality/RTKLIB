/*------------------------------------------------------------------------------
 * rtcm3e.c : RTCM ver.3 message encoder functions
 *
 *          Copyright (C) 2012-2020 by T.TAKASU, All rights reserved.
 *
 * References :
 *     see rtcm.c
 *
 * Version : $Revision:$ $Date:$
 * History : 2012/12/05 1.0  new
 *           2012/12/16 1.1  fix bug on SSR high rate clock correction
 *           2012/12/24 1.2  fix bug on MSM carrier-phase offset correction
 *                           fix bug on SBAS sat id in 1001-1004
 *                           fix bug on carrier-phase in 1001-1004,1009-1012
 *           2012/12/28 1.3  fix bug on compass carrier wave length
 *           2013/01/18 1.4  fix bug on SSR message generation
 *           2013/05/11 1.5  change type of arg value of setbig()
 *           2013/05/19 1.5  GPST -> BDT of time-tag in BeiDou MSM message
 *           2013/04/27 1.7  comply with RTCM 3.2 with amendment 1/2 (ref[15])
 *                           delete MT 1046 according to ref [15]
 *           2014/05/15 1.8  set NT field in MT 1020 GLONASS ephemeris
 *           2014/12/06 1.9  support SBAS/BeiDou SSR messages (ref [16])
 *                           fix bug on invalid staid in QZSS SSR messages
 *           2015/03/22 1.9  add handling of iodcrc for BeiDou/SBAS SSR messages
 *           2015/08/03 1.10 fix bug on wrong udint and iod in SSR 7.
 *                           support RTCM SSR fcb message mt 2065-2069.
 *           2015/09/07 1.11 add message count of MT 2000-2099
 *           2015/10/21 1.12 add MT1046 support for IGS MGEX
 *           2015/12/04 1.13 add MT63 BeiDou ephemeris (RTCM draft)
 *                           fix bug on MSM message generation of BeiDou
 *                           fix bug on SSR 3 message generation (#321)
 *           2016/06/12 1.14 fix bug on segmentation fault by generating msm1
 *           2016/09/20 1.15 fix bug on MT1045 Galileo week rollover
 *           2017/04/11 1.16 fix bug on gst-week in MT1045/1046
 *           2018/10/10 1.17 merge changes for 2.4.2 p13
 *                           change mt for SSR 7 phase biases
 *           2019/05/10 1.21 save Galileo E5b data to obs index 2
 *           2020/11/30 1.22 support MT1230 GLONASS code-phase biases
 *                           support MT1131-1137,1041 (NavIC MSM and ephemeris)
 *                           support MT4076 IGS SSR
 *                           fixed invalid delta clock C2 value for SSR 2 and 4
 *                           delete SSR signal and tracking mode ID table
 *                           use API code2idx() to get freq-index
 *                           use API code2freq() to get carrier frequency
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Constants and macros ------------------------------------------------------*/

#define PRUNIT_GPS 299792.458L           /* RTCM 3 unit of GPS pseudorange (m) */
#define PRUNIT_GLO 599584.916L           /* RTCM 3 unit of GLO pseudorange (m) */
#define RANGE_MS (CLIGHT * 0.001L)       /* Range in 1 ms */
#define P2_10 0.0009765625L              /* 2^-10 */
#define P2_28 3.7252902984619140625E-09L /* 2^-28 */
#define P2_34 5.8207660913467407227E-11L /* 2^-34 */
#define P2_41 4.5474735088646411896E-13L /* 2^-41 */
#define P2_46 1.4210854715202003717E-14L /* 2^-46 */
#define P2_59 1.7347234759768070944E-18L /* 2^-59 */
#define P2_66 1.3552527156068805425E-20L /* 2^-66 */

#define ROUND(x) ((int)floorl((x) + 0.5L))
#define ROUND_U(x) ((uint32_t)floorl((x) + 0.5L))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

/* MSM signal ID table -------------------------------------------------------*/
extern const char *msm_sig_gps[32];
extern const char *msm_sig_glo[32];
extern const char *msm_sig_gal[32];
extern const char *msm_sig_qzs[32];
extern const char *msm_sig_sbs[32];
extern const char *msm_sig_cmp[32];
extern const char *msm_sig_irn[32];

/* SSR signal and tracking mode IDs ------------------------------------------*/
extern const uint8_t ssr_sig_gps[32];
extern const uint8_t ssr_sig_glo[32];
extern const uint8_t ssr_sig_gal[32];
extern const uint8_t ssr_sig_qzs[32];
extern const uint8_t ssr_sig_cmp[32];
extern const uint8_t ssr_sig_sbs[32];

/* SSR update intervals ------------------------------------------------------*/
static const long double ssrudint[16] = {1,   2,   5,   10,  15,   30,   60,   120,
                                         240, 300, 600, 900, 1800, 3600, 7200, 10800};

static void rtcm_setbitu(rtcm_t *rtcm, unsigned pos, unsigned len, uint32_t data) {
  setbitu(rtcm->buff, sizeof(rtcm->buff), pos, len, data);
}
static void rtcm_setbits(rtcm_t *rtcm, unsigned pos, unsigned len, int32_t data) {
  setbits(rtcm->buff, sizeof(rtcm->buff), pos, len, data);
}

/* Set sign-magnitude bits ---------------------------------------------------*/
static void rtcm_setbitg(rtcm_t *rtcm, int pos, int len, int32_t value) {
  rtcm_setbitu(rtcm, pos, 1, value < 0 ? 1 : 0);
  rtcm_setbitu(rtcm, pos + 1, len - 1, value < 0 ? -value : value);
}
/* Set signed 38 bit field ---------------------------------------------------*/
static void rtcm_set38bits(rtcm_t *rtcm, int pos, long double value) {
  int word_h = (int)floorl(value / 64.0L);
  uint32_t word_l = (uint32_t)(value - word_h * 64.0L);
  rtcm_setbits(rtcm, pos, 32, word_h);
  rtcm_setbitu(rtcm, pos + 32, 6, word_l);
}
/* Lock time -----------------------------------------------------------------*/
static int locktime(gtime_t time, gtime_t *lltime, uint8_t LLI) {
  if (!lltime->time || (LLI & 1)) *lltime = time;
  return (int)timediff(time, *lltime);
}
/* Lock time in long double --------------------------------------------------*/
static long double locktime_d(gtime_t time, gtime_t *lltime, uint8_t LLI) {
  if (!lltime->time || (LLI & 1)) *lltime = time;
  return timediff(time, *lltime);
}
/* GLONASS frequency channel number in RTCM (FCN+7,-1:error) -----------------*/
static int fcn_glo(int sat, rtcm_t *rtcm) {
  int prn;
  if (satsys(sat, &prn) != SYS_GLO) {
    return -1;
  }
  if (rtcm->nav.geph[prn - 1][0].sat == sat) {
    return rtcm->nav.geph[prn - 1][0].frq + 7;
  }
  if (rtcm->nav.glo_fcn[prn - 1] > 0) { /* Fcn+8 (0: no data) */
    return rtcm->nav.glo_fcn[prn - 1] - 8 + 7;
  }
  return -1;
}
/* Lock time indicator (ref [17] table 3.4-2) --------------------------------*/
static int to_lock(int lock) {
  if (lock < 0) return 0;
  if (lock < 24) return lock;
  if (lock < 72) return (lock + 24) / 2;
  if (lock < 168) return (lock + 120) / 4;
  if (lock < 360) return (lock + 408) / 8;
  if (lock < 744) return (lock + 1176) / 16;
  if (lock < 937) return (lock + 3096) / 32;
  return 127;
}
/* MSM lock time indicator (ref [17] table 3.5-74) ---------------------------*/
static int to_msm_lock(long double lock) {
  if (lock < 0.032L) return 0;
  if (lock < 0.064L) return 1;
  if (lock < 0.128L) return 2;
  if (lock < 0.256L) return 3;
  if (lock < 0.512L) return 4;
  if (lock < 1.024L) return 5;
  if (lock < 2.048L) return 6;
  if (lock < 4.096L) return 7;
  if (lock < 8.192L) return 8;
  if (lock < 16.384L) return 9;
  if (lock < 32.768L) return 10;
  if (lock < 65.536L) return 11;
  if (lock < 131.072L) return 12;
  if (lock < 262.144L) return 13;
  if (lock < 524.288L) return 14;
  return 15;
}
/* MSM lock time indicator with extended-resolution (ref [17] table 3.5-76) --*/
static int to_msm_lock_ex(long double lock) {
  int lock_ms = (int)(lock * 1000.0L);

  if (lock < 0.0L) return 0;
  if (lock < 0.064L) return lock_ms;
  if (lock < 0.128L) return (lock_ms + 64) / 2;
  if (lock < 0.256L) return (lock_ms + 256) / 4;
  if (lock < 0.512L) return (lock_ms + 768) / 8;
  if (lock < 1.024L) return (lock_ms + 2048) / 16;
  if (lock < 2.048L) return (lock_ms + 5120) / 32;
  if (lock < 4.096L) return (lock_ms + 12288) / 64;
  if (lock < 8.192L) return (lock_ms + 28672) / 128;
  if (lock < 16.384L) return (lock_ms + 65536) / 256;
  if (lock < 32.768L) return (lock_ms + 147456) / 512;
  if (lock < 65.536L) return (lock_ms + 327680) / 1024;
  if (lock < 131.072L) return (lock_ms + 720896) / 2048;
  if (lock < 262.144L) return (lock_ms + 1572864) / 4096;
  if (lock < 524.288L) return (lock_ms + 3407872) / 8192;
  if (lock < 1048.576L) return (lock_ms + 7340032) / 16384;
  if (lock < 2097.152L) return (lock_ms + 15728640) / 32768;
  if (lock < 4194.304L) return (lock_ms + 33554432) / 65536;
  if (lock < 8388.608L) return (lock_ms + 71303168) / 131072;
  if (lock < 16777.216L) return (lock_ms + 150994944) / 262144;
  if (lock < 33554.432L) return (lock_ms + 318767104) / 524288;
  if (lock < 67108.864L) return (lock_ms + 671088640) / 1048576;
  return 704;
}
/* L1 code indicator GPS -----------------------------------------------------*/
static int to_code1_gps(uint8_t code) {
  switch (code) {
    case CODE_L1C:
      return 0; /* L1 C/A */
    case CODE_L1P:
    case CODE_L1W:
    case CODE_L1Y:
    case CODE_L1N:
      return 1; /* L1 P(Y) direct */
  }
  return 0;
}
/* L2 code indicator GPS -----------------------------------------------------*/
static int to_code2_gps(uint8_t code) {
  switch (code) {
    case CODE_L2C:
    case CODE_L2S:
    case CODE_L2L:
    case CODE_L2X:
      return 0; /* L2 C/A or L2C */
    case CODE_L2P:
    case CODE_L2Y:
      return 1; /* L2 P(Y) direct */
    case CODE_L2D:
      return 2; /* L2 P(Y) cross-correlated */
    case CODE_L2W:
    case CODE_L2N:
      return 3; /* L2 correlated P/Y */
  }
  return 0;
}
/* L1 code indicator GLONASS -------------------------------------------------*/
static int to_code1_glo(uint8_t code) {
  switch (code) {
    case CODE_L1C:
      return 0; /* L1 C/A */
    case CODE_L1P:
      return 1; /* L1 P */
  }
  return 0;
}
/* L2 code indicator GLONASS -------------------------------------------------*/
static int to_code2_glo(uint8_t code) {
  switch (code) {
    case CODE_L2C:
      return 0; /* L2 C/A */
    case CODE_L2P:
      return 1; /* L2 P */
  }
  return 0;
}
/* Carrier-phase - pseudorange in cycle --------------------------------------*/
static long double cp_pr(long double cp, long double pr_cyc) {
  return fmodl(cp - pr_cyc + 750.0L, 1500.0L) - 750.0L;
}
/* Generate obs field data GPS -----------------------------------------------*/
static void gen_obs_gps(rtcm_t *rtcm, const obsd_t *data, int *code1, int *pr1, int *ppr1,
                        int *lock1, int *amb, int *cnr1, int *code2, int *pr21, int *ppr2,
                        int *lock2, int *cnr2) {
  long double lam1 = CLIGHT / FREQL1;
  long double lam2 = CLIGHT / FREQL2;
  *pr1 = *amb = 0;
  if (ppr1) *ppr1 = 0xFFF80000; /* Invalid values */
  if (pr21) *pr21 = 0xFFFFE000;
  if (ppr2) *ppr2 = 0xFFF80000;

  /* L1 peudorange */
  long double pr1c = 0.0L;
  if (data->P[0] != 0.0L && data->code[0]) {
    *amb = (int)floorl(data->P[0] / PRUNIT_GPS);
    *pr1 = ROUND((data->P[0] - *amb * PRUNIT_GPS) / 0.02L);
    pr1c = *pr1 * 0.02L + *amb * PRUNIT_GPS;
  }
  /* L1 phaserange - L1 pseudorange */
  if (data->P[0] != 0.0L && data->L[0] != 0.0L && data->code[0]) {
    long double ppr = cp_pr(data->L[0], pr1c / lam1);
    if (ppr1) *ppr1 = ROUND(ppr * lam1 / 0.0005L);
  }
  /* L2 -L1 pseudorange */
  if (data->P[0] != 0.0L && data->P[1] != 0.0L && data->code[0] && data->code[1] &&
      fabsl(data->P[1] - pr1c) <= 163.82L) {
    if (pr21) *pr21 = ROUND((data->P[1] - pr1c) / 0.02L);
  }
  /* L2 phaserange - L1 pseudorange */
  if (data->P[0] != 0.0L && data->L[1] != 0.0L && data->code[0] && data->code[1]) {
    long double ppr = cp_pr(data->L[1], pr1c / lam2);
    if (ppr2) *ppr2 = ROUND(ppr * lam2 / 0.0005L);
  }
  int lt1 = locktime(data->time, rtcm->lltime[data->sat - 1], data->LLI[0]);
  int lt2 = locktime(data->time, rtcm->lltime[data->sat - 1] + 1, data->LLI[1]);

  if (lock1) *lock1 = to_lock(lt1);
  if (lock2) *lock2 = to_lock(lt2);
  if (cnr1) *cnr1 = ROUND(data->SNR[0] * SNR_UNIT / 0.25L);
  if (cnr2) *cnr2 = ROUND(data->SNR[1] * SNR_UNIT / 0.25L);
  if (code1) *code1 = to_code1_gps(data->code[0]);
  if (code2) *code2 = to_code2_gps(data->code[1]);
}
/* Generate obs field data GLONASS -------------------------------------------*/
static void gen_obs_glo(rtcm_t *rtcm, const obsd_t *data, int fcn, int *code1, int *pr1, int *ppr1,
                        int *lock1, int *amb, int *cnr1, int *code2, int *pr21, int *ppr2,
                        int *lock2, int *cnr2) {
  long double lam1 = 0.0L, lam2 = 0.0L;
  if (fcn >= 0) { /* Fcn+7 */
    lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO * (fcn - 7));
    lam2 = CLIGHT / (FREQ2_GLO + DFRQ2_GLO * (fcn - 7));
  }
  *pr1 = *amb = 0;
  if (ppr1) *ppr1 = 0xFFF80000; /* Invalid values */
  if (pr21) *pr21 = 0xFFFFE000;
  if (ppr2) *ppr2 = 0xFFF80000;

  /* L1 pseudorange */
  long double pr1c = 0.0L;
  if (data->P[0] != 0.0L) {
    *amb = (int)floorl(data->P[0] / PRUNIT_GLO);
    *pr1 = ROUND((data->P[0] - *amb * PRUNIT_GLO) / 0.02L);
    pr1c = *pr1 * 0.02L + *amb * PRUNIT_GLO;
  }
  /* L1 phaserange - L1 pseudorange */
  if (data->P[0] != 0.0L && data->L[0] != 0.0L && data->code[0] && lam1 > 0.0L) {
    long double ppr = cp_pr(data->L[0], pr1c / lam1);
    if (ppr1) *ppr1 = ROUND(ppr * lam1 / 0.0005L);
  }
  /* L2 -L1 pseudorange */
  if (data->P[0] != 0.0L && data->P[1] != 0.0L && data->code[0] && data->code[1] &&
      fabsl(data->P[1] - pr1c) <= 163.82L) {
    if (pr21) *pr21 = ROUND((data->P[1] - pr1c) / 0.02L);
  }
  /* L2 phaserange - L1 pseudorange */
  if (data->P[0] != 0.0L && data->L[1] != 0.0L && data->code[0] && data->code[1] && lam2 > 0.0L) {
    long double ppr = cp_pr(data->L[1], pr1c / lam2);
    if (ppr2) *ppr2 = ROUND(ppr * lam2 / 0.0005L);
  }
  int lt1 = locktime(data->time, rtcm->lltime[data->sat - 1], data->LLI[0]);
  int lt2 = locktime(data->time, rtcm->lltime[data->sat - 1] + 1, data->LLI[1]);

  if (lock1) *lock1 = to_lock(lt1);
  if (lock2) *lock2 = to_lock(lt2);
  if (cnr1) *cnr1 = ROUND(data->SNR[0] * SNR_UNIT / 0.25L);
  if (cnr2) *cnr2 = ROUND(data->SNR[1] * SNR_UNIT / 0.25L);
  if (code1) *code1 = to_code1_glo(data->code[0]);
  if (code2) *code2 = to_code2_glo(data->code[1]);
}
/* Encode RTCM header --------------------------------------------------------*/
static int encode_head(int type, rtcm_t *rtcm, int sys, int sync, int nsat) {
  trace(4, "encode_head: type=%d sync=%d sys=%d nsat=%d\n", type, sync, sys, nsat);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, type);
  i += 12; /* Message no */
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12; /* Ref station id */

  if (sys == SYS_GLO) {
    int week;
    long double tow = time2gpst(timeadd(gpst2utc(rtcm->time), 10800.0L), &week);
    int epoch = ROUND(fmodl(tow, 86400.0L) / 0.001L);
    rtcm_setbitu(rtcm, i, 27, epoch);
    i += 27; /* GLONASS epoch time */
  } else {
    int week;
    long double tow = time2gpst(rtcm->time, &week);
    int epoch = ROUND(tow / 0.001L);
    rtcm_setbitu(rtcm, i, 30, epoch);
    i += 30; /* GPS epoch time */
  }
  rtcm_setbitu(rtcm, i, 1, sync);
  i += 1; /* Synchronous GNSS flag */
  rtcm_setbitu(rtcm, i, 5, nsat);
  i += 5; /* No of satellites */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Smoothing indicator */
  rtcm_setbitu(rtcm, i, 3, 0);
  i += 3; /* Smoothing interval */
  return i;
}
/* Encode type 1001: basic L1-only GPS RTK observables -----------------------*/
static bool encode_type1001(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1001: sync=%d\n", sync);

  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1001, rtcm, SYS_GPS, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;

    if (sys == SYS_SBS) prn -= 80; /* 40-58: SBAS 120-138 */

    /* Generate obs field data GPS */
    int code1, pr1, ppr1, lock1, amb;
    gen_obs_gps(rtcm, rtcm->obs.data + j, &code1, &pr1, &ppr1, &lock1, &amb, NULL, NULL, NULL, NULL,
                NULL, NULL);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 24, pr1);
    i += 24;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1002: extended L1-only GPS RTK observables --------------------*/
static bool encode_type1002(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1002: sync=%d\n", sync);

  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1002, rtcm, SYS_GPS, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;

    if (sys == SYS_SBS) prn -= 80; /* 40-58: SBAS 120-138 */

    /* Generate obs field data GPS */
    int code1, pr1, ppr1, lock1, amb, cnr1;

    gen_obs_gps(rtcm, rtcm->obs.data + j, &code1, &pr1, &ppr1, &lock1, &amb, &cnr1, NULL, NULL,
                NULL, NULL, NULL);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 24, pr1);
    i += 24;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
    rtcm_setbitu(rtcm, i, 8, amb);
    i += 8;
    rtcm_setbitu(rtcm, i, 8, cnr1);
    i += 8;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1003: basic L1&L2 GPS RTK observables -------------------------*/
static bool encode_type1003(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1003: sync=%d\n", sync);

  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1003, rtcm, SYS_GPS, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;

    if (sys == SYS_SBS) prn -= 80; /* 40-58: SBAS 120-138 */

    /* Generate obs field data GPS */
    int code1, pr1, ppr1, lock1, amb, code2, pr21, ppr2, lock2;
    gen_obs_gps(rtcm, rtcm->obs.data + j, &code1, &pr1, &ppr1, &lock1, &amb, NULL, &code2, &pr21,
                &ppr2, &lock2, NULL);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 24, pr1);
    i += 24;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
    rtcm_setbitu(rtcm, i, 2, code2);
    i += 2;
    rtcm_setbits(rtcm, i, 14, pr21);
    i += 14;
    rtcm_setbits(rtcm, i, 20, ppr2);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock2);
    i += 7;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1004: extended L1&L2 GPS RTK observables ----------------------*/
static bool encode_type1004(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1004: sync=%d\n", sync);

  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1004, rtcm, SYS_GPS, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int prn;
    int sys = satsys(rtcm->obs.data[j].sat, &prn);
    if (!(sys & (SYS_GPS | SYS_SBS))) continue;

    if (sys == SYS_SBS) prn -= 80; /* 40-58: SBAS 120-138 */

    /* Generate obs field data GPS */
    int code1, pr1, ppr1, lock1, amb, cnr1, code2, pr21, ppr2, lock2, cnr2;
    gen_obs_gps(rtcm, rtcm->obs.data + j, &code1, &pr1, &ppr1, &lock1, &amb, &cnr1, &code2, &pr21,
                &ppr2, &lock2, &cnr2);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 24, pr1);
    i += 24;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
    rtcm_setbitu(rtcm, i, 8, amb);
    i += 8;
    rtcm_setbitu(rtcm, i, 8, cnr1);
    i += 8;
    rtcm_setbitu(rtcm, i, 2, code2);
    i += 2;
    rtcm_setbits(rtcm, i, 14, pr21);
    i += 14;
    rtcm_setbits(rtcm, i, 20, ppr2);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock2);
    i += 7;
    rtcm_setbitu(rtcm, i, 8, cnr2);
    i += 8;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1005: stationary RTK reference station ARP --------------------*/
static bool encode_type1005(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1005: sync=%d\n", sync);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1005);
  i += 12; /* Message no */
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12; /* Ref station id */
  rtcm_setbitu(rtcm, i, 6, 0);
  i += 6; /* ITRF realization year */
  rtcm_setbitu(rtcm, i, 1, 1);
  i += 1; /* GPS indicator */
  rtcm_setbitu(rtcm, i, 1, 1);
  i += 1; /* GLONASS indicator */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Galileo indicator */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Ref station indicator */
  rtcm_set38bits(rtcm, i, rtcm->sta.pos[0] / 0.0001L);
  i += 38; /* Antenna ref point ecef-x */
  rtcm_setbitu(rtcm, i, 1, 1);
  i += 1; /* Oscillator indicator */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Reserved */
  rtcm_set38bits(rtcm, i, rtcm->sta.pos[1] / 0.0001L);
  i += 38; /* Antenna ref point ecef-y */
  rtcm_setbitu(rtcm, i, 2, 0);
  i += 2; /* Quarter cycle indicator */
  rtcm_set38bits(rtcm, i, rtcm->sta.pos[2] / 0.0001L);
  i += 38; /* Antenna ref point ecef-z */
  rtcm->nbit = i;
  return true;
}
/* Encode type 1006: stationary RTK reference station ARP with height --------*/
static bool encode_type1006(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1006: sync=%d\n", sync);

  int hgt = 0;
  if (0.0L <= rtcm->sta.hgt && rtcm->sta.hgt <= 6.5535L) {
    hgt = ROUND(rtcm->sta.hgt / 0.0001L);
  } else {
    trace(2, "antenna height error: h=%.4Lf\n", rtcm->sta.hgt);
  }
  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1006);
  i += 12; /* Message no */
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12; /* Ref station id */
  rtcm_setbitu(rtcm, i, 6, 0);
  i += 6; /* ITRF realization year */
  rtcm_setbitu(rtcm, i, 1, 1);
  i += 1; /* GPS indicator */
  rtcm_setbitu(rtcm, i, 1, 1);
  i += 1; /* GLONASS indicator */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Galileo indicator */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Ref station indicator */
  rtcm_set38bits(rtcm, i, rtcm->sta.pos[0] / 0.0001L);
  i += 38; /* Antenna ref point ecef-x */
  rtcm_setbitu(rtcm, i, 1, 1);
  i += 1; /* Oscillator indicator */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Reserved */
  rtcm_set38bits(rtcm, i, rtcm->sta.pos[1] / 0.0001L);
  i += 38; /* Antenna ref point ecef-y */
  rtcm_setbitu(rtcm, i, 2, 0);
  i += 2; /* Quarter cycle indicator */
  rtcm_set38bits(rtcm, i, rtcm->sta.pos[2] / 0.0001L);
  i += 38; /* Antenna ref point ecef-z */
  rtcm_setbitu(rtcm, i, 16, hgt);
  i += 16; /* Antenna height */
  rtcm->nbit = i;
  return true;
}
/* Encode type 1007: antenna descriptor --------------------------------------*/
static bool encode_type1007(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1007: sync=%d\n", sync);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1007);
  i += 12; /* Message no */
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12; /* Ref station id */

  /* Antenna descriptor */
  int n = MIN((int)strlen(rtcm->sta.antdes), 31);
  rtcm_setbitu(rtcm, i, 8, n);
  i += 8;
  for (int j = 0; j < n; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.antdes[j]);
    i += 8;
  }
  int antsetup = rtcm->sta.antsetup;
  rtcm_setbitu(rtcm, i, 8, antsetup);
  i += 8; /* Antetnna setup id */
  rtcm->nbit = i;
  return true;
}
/* Encode type 1008: antenna descriptor & serial number ----------------------*/
static bool encode_type1008(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1008: sync=%d\n", sync);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1008);
  i += 12; /* Message no */
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12; /* Ref station id */

  /* Antenna descriptor */
  int n = MIN((int)strlen(rtcm->sta.antdes), 31);
  rtcm_setbitu(rtcm, i, 8, n);
  i += 8;
  for (int j = 0; j < n; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.antdes[j]);
    i += 8;
  }
  int antsetup = rtcm->sta.antsetup;
  rtcm_setbitu(rtcm, i, 8, antsetup);
  i += 8; /* Antenna setup id */

  /* Antenna serial number */
  int m = MIN((int)strlen(rtcm->sta.antsno), 31);
  rtcm_setbitu(rtcm, i, 8, m);
  i += 8;
  for (int j = 0; j < m; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.antsno[j]);
    i += 8;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1009: basic L1-only GLONASS RTK observables -------------------*/
static bool encode_type1009(rtcm_t *rtcm, int sync) {
  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    int fcn = fcn_glo(sat, rtcm);
    if (fcn < 0) continue; /* Fcn+7 */
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1009, rtcm, SYS_GLO, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    int fcn = fcn_glo(sat, rtcm);
    if (fcn < 0) continue; /* Fcn+7 */

    /* Generate obs field data GLONASS */
    int code1, pr1, ppr1, lock1, amb;
    gen_obs_glo(rtcm, rtcm->obs.data + j, fcn, &code1, &pr1, &ppr1, &lock1, &amb, NULL, NULL, NULL,
                NULL, NULL, NULL);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 5, fcn);
    i += 5; /* Fcn+7 */
    rtcm_setbitu(rtcm, i, 25, pr1);
    i += 25;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1010: extended L1-only GLONASS RTK observables ----------------*/
static bool encode_type1010(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1010: sync=%d\n", sync);

  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    int fcn = fcn_glo(sat, rtcm);
    if (fcn < 0) continue; /* Fcn+7 */
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1010, rtcm, SYS_GLO, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    int fcn = fcn_glo(sat, rtcm);
    if (fcn < 0) continue; /* Fcn+7 */

    /* Generate obs field data GLONASS */
    int code1, pr1, ppr1, lock1, amb, cnr1;
    gen_obs_glo(rtcm, rtcm->obs.data + j, fcn, &code1, &pr1, &ppr1, &lock1, &amb, &cnr1, NULL, NULL,
                NULL, NULL, NULL);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 5, fcn);
    i += 5; /* Fcn+7 */
    rtcm_setbitu(rtcm, i, 25, pr1);
    i += 25;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
    rtcm_setbitu(rtcm, i, 7, amb);
    i += 7;
    rtcm_setbitu(rtcm, i, 8, cnr1);
    i += 8;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1011: basic  L1&L2 GLONASS RTK observables --------------------*/
static bool encode_type1011(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1011: sync=%d\n", sync);

  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    int fcn = fcn_glo(sat, rtcm);
    if (fcn < 0) continue; /* Fcn+7 */
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1011, rtcm, SYS_GLO, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    int fcn = fcn_glo(sat, rtcm);
    if (fcn < 0) continue; /* Fcn+7 */

    /* Generate obs field data GLONASS */
    int code1, pr1, ppr1, lock1, amb, code2, pr21, ppr2, lock2;
    gen_obs_glo(rtcm, rtcm->obs.data + j, fcn, &code1, &pr1, &ppr1, &lock1, &amb, NULL, &code2,
                &pr21, &ppr2, &lock2, NULL);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 5, fcn);
    i += 5; /* Fcn+7 */
    rtcm_setbitu(rtcm, i, 25, pr1);
    i += 25;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
    rtcm_setbitu(rtcm, i, 2, code2);
    i += 2;
    rtcm_setbits(rtcm, i, 14, pr21);
    i += 14;
    rtcm_setbits(rtcm, i, 20, ppr2);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock2);
    i += 7;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1012: extended L1&L2 GLONASS RTK observables ------------------*/
static bool encode_type1012(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1012: sync=%d\n", sync);

  int nsat = 0;
  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    if (fcn_glo(sat, rtcm) < 0) continue; /* Fcn+7 */
    nsat++;
  }
  /* Encode header */
  int i = encode_head(1012, rtcm, SYS_GLO, sync, nsat);

  for (int j = 0; j < rtcm->obs.n && nsat < MAXOBS; j++) {
    int sat = rtcm->obs.data[j].sat;
    int prn;
    if (satsys(sat, &prn) != SYS_GLO) continue;
    int fcn = fcn_glo(sat, rtcm);
    if (fcn < 0) continue; /* Fcn+7 */

    /* Generate obs field data GLONASS */
    int code1, pr1, ppr1, lock1, amb, cnr1, code2, pr21, ppr2, lock2, cnr2;
    gen_obs_glo(rtcm, rtcm->obs.data + j, fcn, &code1, &pr1, &ppr1, &lock1, &amb, &cnr1, &code2,
                &pr21, &ppr2, &lock2, &cnr2);

    rtcm_setbitu(rtcm, i, 6, prn);
    i += 6;
    rtcm_setbitu(rtcm, i, 1, code1);
    i += 1;
    rtcm_setbitu(rtcm, i, 5, fcn);
    i += 5; /* Fcn+7 */
    rtcm_setbitu(rtcm, i, 25, pr1);
    i += 25;
    rtcm_setbits(rtcm, i, 20, ppr1);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock1);
    i += 7;
    rtcm_setbitu(rtcm, i, 7, amb);
    i += 7;
    rtcm_setbitu(rtcm, i, 8, cnr1);
    i += 8;
    rtcm_setbitu(rtcm, i, 2, code2);
    i += 2;
    rtcm_setbits(rtcm, i, 14, pr21);
    i += 14;
    rtcm_setbits(rtcm, i, 20, ppr2);
    i += 20;
    rtcm_setbitu(rtcm, i, 7, lock2);
    i += 7;
    rtcm_setbitu(rtcm, i, 8, cnr2);
    i += 8;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1019: GPS ephemerides -----------------------------------------*/
static bool encode_type1019(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1019: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_GPS) return false;
  eph_t *eph = rtcm->nav.eph[rtcm->ephsat - 1];
  if (eph->sat != rtcm->ephsat) return false;
  int week = eph->week % 1024;
  int toe = ROUND(eph->toes / 16.0L);
  int toc = ROUND(time2gpst(eph->toc, NULL) / 16.0L);
  uint32_t sqrtA = ROUND_U(sqrtl(eph->A) / P2_19);
  uint32_t e = ROUND_U(eph->e / P2_33);
  int i0 = ROUND(eph->i0 / P2_31 / SC2RAD);
  int OMG0 = ROUND(eph->OMG0 / P2_31 / SC2RAD);
  int omg = ROUND(eph->omg / P2_31 / SC2RAD);
  int M0 = ROUND(eph->M0 / P2_31 / SC2RAD);
  int deln = ROUND(eph->deln / P2_43 / SC2RAD);
  int idot = ROUND(eph->idot / P2_43 / SC2RAD);
  int OMGd = ROUND(eph->OMGd / P2_43 / SC2RAD);
  int crs = ROUND(eph->crs / P2_5);
  int crc = ROUND(eph->crc / P2_5);
  int cus = ROUND(eph->cus / P2_29);
  int cuc = ROUND(eph->cuc / P2_29);
  int cis = ROUND(eph->cis / P2_29);
  int cic = ROUND(eph->cic / P2_29);
  int af0 = ROUND(eph->f0 / P2_31);
  int af1 = ROUND(eph->f1 / P2_43);
  int af2 = ROUND(eph->f2 / P2_55);
  int tgd = ROUND(eph->tgd[0] / P2_31);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1019);
  i += 12;
  rtcm_setbitu(rtcm, i, 6, prn);
  i += 6;
  rtcm_setbitu(rtcm, i, 10, week);
  i += 10;
  rtcm_setbitu(rtcm, i, 4, eph->sva);
  i += 4;
  rtcm_setbitu(rtcm, i, 2, eph->code);
  i += 2;
  rtcm_setbits(rtcm, i, 14, idot);
  i += 14;
  rtcm_setbitu(rtcm, i, 8, eph->iode);
  i += 8;
  rtcm_setbitu(rtcm, i, 16, toc);
  i += 16;
  rtcm_setbits(rtcm, i, 8, af2);
  i += 8;
  rtcm_setbits(rtcm, i, 16, af1);
  i += 16;
  rtcm_setbits(rtcm, i, 22, af0);
  i += 22;
  rtcm_setbitu(rtcm, i, 10, eph->iodc);
  i += 10;
  rtcm_setbits(rtcm, i, 16, crs);
  i += 16;
  rtcm_setbits(rtcm, i, 16, deln);
  i += 16;
  rtcm_setbits(rtcm, i, 32, M0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cuc);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, e);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cus);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, sqrtA);
  i += 32;
  rtcm_setbitu(rtcm, i, 16, toe);
  i += 16;
  rtcm_setbits(rtcm, i, 16, cic);
  i += 16;
  rtcm_setbits(rtcm, i, 32, OMG0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cis);
  i += 16;
  rtcm_setbits(rtcm, i, 32, i0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, crc);
  i += 16;
  rtcm_setbits(rtcm, i, 32, omg);
  i += 32;
  rtcm_setbits(rtcm, i, 24, OMGd);
  i += 24;
  rtcm_setbits(rtcm, i, 8, tgd);
  i += 8;
  rtcm_setbitu(rtcm, i, 6, eph->svh);
  i += 6;
  rtcm_setbitu(rtcm, i, 1, eph->flag);
  i += 1;
  rtcm_setbitu(rtcm, i, 1, eph->fit > 0.0L ? 0 : 1);
  i += 1;
  rtcm->nbit = i;
  return true;
}
/* Encode type 1020: GLONASS ephemerides -------------------------------------*/
static bool encode_type1020(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1020: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_GLO) return false;
  geph_t *geph = rtcm->nav.geph[prn - 1];
  if (geph->sat != rtcm->ephsat) return false;
  int fcn = geph->frq + 7;

  /* Time of frame within day (utc(su) + 3 hr) */
  gtime_t time = timeadd(gpst2utc(geph->tof), 10800.0L);
  long double ep[6];
  time2epoch(time, ep);
  int tk_h = (int)ep[3];
  int tk_m = (int)ep[4];
  int tk_s = ROUND(ep[5] / 30.0L);

  /* # of days since jan 1 in leap year */
  ep[0] = floorl(ep[0] / 4.0L) * 4.0L;
  ep[1] = ep[2] = 1.0L;
  ep[3] = ep[4] = ep[5] = 0.0L;
  int NT = (int)floorl(timediff(time, epoch2time(ep)) / 86400.0L + 1.0L);

  /* Index of time interval within day (utc(su) + 3 hr) */
  time = timeadd(gpst2utc(geph->toe), 10800.0L);
  time2epoch(time, ep);
  int tb = ROUND((ep[3] * 3600.0L + ep[4] * 60.0L + ep[5]) / 900.0L);

  int pos[3], vel[3], acc[3];
  for (int j = 0; j < 3; j++) {
    pos[j] = ROUND(geph->pos[j] / P2_11 / 1E3L);
    vel[j] = ROUND(geph->vel[j] / P2_20 / 1E3L);
    acc[j] = ROUND(geph->acc[j] / P2_30 / 1E3L);
  }
  int gamn = ROUND(geph->gamn / P2_40);
  int taun = ROUND(geph->taun / P2_30);
  int dtaun = ROUND(geph->dtaun / P2_30);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1020);
  i += 12;
  rtcm_setbitu(rtcm, i, 6, prn);
  i += 6;
  rtcm_setbitu(rtcm, i, 5, fcn);
  i += 5;
  rtcm_setbitu(rtcm, i, 4, 0);
  i += 4; /* Almanac health,P1 */
  rtcm_setbitu(rtcm, i, 5, tk_h);
  i += 5;
  rtcm_setbitu(rtcm, i, 6, tk_m);
  i += 6;
  rtcm_setbitu(rtcm, i, 1, tk_s);
  i += 1;
  rtcm_setbitu(rtcm, i, 1, geph->svh);
  i += 1; /* Bn */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* P2 */
  rtcm_setbitu(rtcm, i, 7, tb);
  i += 7;
  rtcm_setbitg(rtcm, i, 24, vel[0]);
  i += 24;
  rtcm_setbitg(rtcm, i, 27, pos[0]);
  i += 27;
  rtcm_setbitg(rtcm, i, 5, acc[0]);
  i += 5;
  rtcm_setbitg(rtcm, i, 24, vel[1]);
  i += 24;
  rtcm_setbitg(rtcm, i, 27, pos[1]);
  i += 27;
  rtcm_setbitg(rtcm, i, 5, acc[1]);
  i += 5;
  rtcm_setbitg(rtcm, i, 24, vel[2]);
  i += 24;
  rtcm_setbitg(rtcm, i, 27, pos[2]);
  i += 27;
  rtcm_setbitg(rtcm, i, 5, acc[2]);
  i += 5;
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* P3 */
  rtcm_setbitg(rtcm, i, 11, gamn);
  i += 11;
  rtcm_setbitu(rtcm, i, 3, 0);
  i += 3; /* P,ln */
  rtcm_setbitg(rtcm, i, 22, taun);
  i += 22;
  rtcm_setbitg(rtcm, i, 5, dtaun);
  i += 5;
  rtcm_setbitu(rtcm, i, 5, geph->age);
  i += 5; /* En */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* P4 */
  rtcm_setbitu(rtcm, i, 4, 0);
  i += 4; /* FT */
  rtcm_setbitu(rtcm, i, 11, NT);
  i += 11;
  rtcm_setbitu(rtcm, i, 2, 0);
  i += 2; /* M */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Flag for additional data */
  rtcm_setbitu(rtcm, i, 11, 0);
  i += 11; /* NA */
  rtcm_setbitu(rtcm, i, 32, 0);
  i += 32; /* Tauc */
  rtcm_setbitu(rtcm, i, 5, 0);
  i += 5; /* N4 */
  rtcm_setbitu(rtcm, i, 22, 0);
  i += 22; /* Taugps */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Ln */
  rtcm_setbitu(rtcm, i, 7, 0);
  i += 7;
  rtcm->nbit = i;
  return true;
}
/* Encode type 1033: receiver and antenna descriptor -------------------------*/
static bool encode_type1033(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1033: sync=%d\n", sync);

  int antsetup = rtcm->sta.antsetup;
  int n = MIN((int)strlen(rtcm->sta.antdes), 31);
  int m = MIN((int)strlen(rtcm->sta.antsno), 31);
  int I = MIN((int)strlen(rtcm->sta.rectype), 31);
  int J = MIN((int)strlen(rtcm->sta.recver), 31);
  int K = MIN((int)strlen(rtcm->sta.recsno), 31);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1033);
  i += 12;
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12;

  rtcm_setbitu(rtcm, i, 8, n);
  i += 8;
  for (int j = 0; j < n; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.antdes[j]);
    i += 8;
  }
  rtcm_setbitu(rtcm, i, 8, antsetup);
  i += 8;

  rtcm_setbitu(rtcm, i, 8, m);
  i += 8;
  for (int j = 0; j < m; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.antsno[j]);
    i += 8;
  }
  rtcm_setbitu(rtcm, i, 8, I);
  i += 8;
  for (int j = 0; j < I; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.rectype[j]);
    i += 8;
  }
  rtcm_setbitu(rtcm, i, 8, J);
  i += 8;
  for (int j = 0; j < J; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.recver[j]);
    i += 8;
  }
  rtcm_setbitu(rtcm, i, 8, K);
  i += 8;
  for (int j = 0; j < K; j++) {
    rtcm_setbitu(rtcm, i, 8, rtcm->sta.recsno[j]);
    i += 8;
  }
  rtcm->nbit = i;
  return true;
}
/* Encode type 1041: NavIC/IRNSS ephemerides ---------------------------------*/
static bool encode_type1041(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1041: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_IRN) return false;
  eph_t *eph = rtcm->nav.eph[rtcm->ephsat - 1];
  if (eph->sat != rtcm->ephsat) return false;
  int week = eph->week % 1024;
  int toe = ROUND(eph->toes / 16.0L);
  int toc = ROUND(time2gpst(eph->toc, NULL) / 16.0L);
  uint32_t sqrtA = ROUND_U(sqrtl(eph->A) / P2_19);
  uint32_t e = ROUND_U(eph->e / P2_33);
  int i0 = ROUND(eph->i0 / P2_31 / SC2RAD);
  int OMG0 = ROUND(eph->OMG0 / P2_31 / SC2RAD);
  int omg = ROUND(eph->omg / P2_31 / SC2RAD);
  int M0 = ROUND(eph->M0 / P2_31 / SC2RAD);
  int deln = ROUND(eph->deln / P2_41 / SC2RAD);
  int idot = ROUND(eph->idot / P2_43 / SC2RAD);
  int OMGd = ROUND(eph->OMGd / P2_41 / SC2RAD);
  int crs = ROUND(eph->crs / 0.0625L);
  int crc = ROUND(eph->crc / 0.0625L);
  int cus = ROUND(eph->cus / P2_28);
  int cuc = ROUND(eph->cuc / P2_28);
  int cis = ROUND(eph->cis / P2_28);
  int cic = ROUND(eph->cic / P2_28);
  int af0 = ROUND(eph->f0 / P2_31);
  int af1 = ROUND(eph->f1 / P2_43);
  int af2 = ROUND(eph->f2 / P2_55);
  int tgd = ROUND(eph->tgd[0] / P2_31);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1041);
  i += 12;
  rtcm_setbitu(rtcm, i, 6, prn);
  i += 6;
  rtcm_setbitu(rtcm, i, 10, week);
  i += 10;
  rtcm_setbits(rtcm, i, 22, af0);
  i += 22;
  rtcm_setbits(rtcm, i, 16, af1);
  i += 16;
  rtcm_setbits(rtcm, i, 8, af2);
  i += 8;
  rtcm_setbitu(rtcm, i, 4, eph->sva);
  i += 4;
  rtcm_setbitu(rtcm, i, 16, toc);
  i += 16;
  rtcm_setbits(rtcm, i, 8, tgd);
  i += 8;
  rtcm_setbits(rtcm, i, 22, deln);
  i += 22;
  rtcm_setbitu(rtcm, i, 8, eph->iode);
  i += 8 + 10; /* IODEC */
  rtcm_setbitu(rtcm, i, 2, eph->svh);
  i += 2; /* L5+Sflag */
  rtcm_setbits(rtcm, i, 15, cuc);
  i += 15;
  rtcm_setbits(rtcm, i, 15, cus);
  i += 15;
  rtcm_setbits(rtcm, i, 15, cic);
  i += 15;
  rtcm_setbits(rtcm, i, 15, cis);
  i += 15;
  rtcm_setbits(rtcm, i, 15, crc);
  i += 15;
  rtcm_setbits(rtcm, i, 15, crs);
  i += 15;
  rtcm_setbits(rtcm, i, 14, idot);
  i += 14;
  rtcm_setbits(rtcm, i, 32, M0);
  i += 32;
  rtcm_setbitu(rtcm, i, 16, toe);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, e);
  i += 32;
  rtcm_setbitu(rtcm, i, 32, sqrtA);
  i += 32;
  rtcm_setbits(rtcm, i, 32, OMG0);
  i += 32;
  rtcm_setbits(rtcm, i, 32, omg);
  i += 32;
  rtcm_setbits(rtcm, i, 22, OMGd);
  i += 22;
  rtcm_setbits(rtcm, i, 32, i0);
  i += 32 + 4;
  rtcm->nbit = i;
  return true;
}
/* Encode type 1044: QZSS ephemerides ----------------------------------------*/
static bool encode_type1044(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1044: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_QZS) return false;
  eph_t *eph = rtcm->nav.eph[rtcm->ephsat - 1];
  if (eph->sat != rtcm->ephsat) return false;
  int week = eph->week % 1024;
  int toe = ROUND(eph->toes / 16.0L);
  int toc = ROUND(time2gpst(eph->toc, NULL) / 16.0L);
  uint32_t sqrtA = ROUND_U(sqrtl(eph->A) / P2_19);
  uint32_t e = ROUND_U(eph->e / P2_33);
  int i0 = ROUND(eph->i0 / P2_31 / SC2RAD);
  int OMG0 = ROUND(eph->OMG0 / P2_31 / SC2RAD);
  int omg = ROUND(eph->omg / P2_31 / SC2RAD);
  int M0 = ROUND(eph->M0 / P2_31 / SC2RAD);
  int deln = ROUND(eph->deln / P2_43 / SC2RAD);
  int idot = ROUND(eph->idot / P2_43 / SC2RAD);
  int OMGd = ROUND(eph->OMGd / P2_43 / SC2RAD);
  int crs = ROUND(eph->crs / P2_5);
  int crc = ROUND(eph->crc / P2_5);
  int cus = ROUND(eph->cus / P2_29);
  int cuc = ROUND(eph->cuc / P2_29);
  int cis = ROUND(eph->cis / P2_29);
  int cic = ROUND(eph->cic / P2_29);
  int af0 = ROUND(eph->f0 / P2_31);
  int af1 = ROUND(eph->f1 / P2_43);
  int af2 = ROUND(eph->f2 / P2_55);
  int tgd = ROUND(eph->tgd[0] / P2_31);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1044);
  i += 12;
  rtcm_setbitu(rtcm, i, 4, prn - 192);
  i += 4;
  rtcm_setbitu(rtcm, i, 16, toc);
  i += 16;
  rtcm_setbits(rtcm, i, 8, af2);
  i += 8;
  rtcm_setbits(rtcm, i, 16, af1);
  i += 16;
  rtcm_setbits(rtcm, i, 22, af0);
  i += 22;
  rtcm_setbitu(rtcm, i, 8, eph->iode);
  i += 8;
  rtcm_setbits(rtcm, i, 16, crs);
  i += 16;
  rtcm_setbits(rtcm, i, 16, deln);
  i += 16;
  rtcm_setbits(rtcm, i, 32, M0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cuc);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, e);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cus);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, sqrtA);
  i += 32;
  rtcm_setbitu(rtcm, i, 16, toe);
  i += 16;
  rtcm_setbits(rtcm, i, 16, cic);
  i += 16;
  rtcm_setbits(rtcm, i, 32, OMG0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cis);
  i += 16;
  rtcm_setbits(rtcm, i, 32, i0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, crc);
  i += 16;
  rtcm_setbits(rtcm, i, 32, omg);
  i += 32;
  rtcm_setbits(rtcm, i, 24, OMGd);
  i += 24;
  rtcm_setbits(rtcm, i, 14, idot);
  i += 14;
  rtcm_setbitu(rtcm, i, 2, eph->code);
  i += 2;
  rtcm_setbitu(rtcm, i, 10, week);
  i += 10;
  rtcm_setbitu(rtcm, i, 4, eph->sva);
  i += 4;
  rtcm_setbitu(rtcm, i, 6, eph->svh);
  i += 6;
  rtcm_setbits(rtcm, i, 8, tgd);
  i += 8;
  rtcm_setbitu(rtcm, i, 10, eph->iodc);
  i += 10;
  rtcm_setbitu(rtcm, i, 1, eph->fit == 2.0L ? 0 : 1);
  i += 1;
  rtcm->nbit = i;
  return true;
}
/* Encode type 1045: Galileo F/NAV satellite ephemerides ---------------------*/
static bool encode_type1045(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1045: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_GAL) return false;
  eph_t *eph = rtcm->nav.eph[rtcm->ephsat - 1] + 1; /* F/NAV */
  if (eph->sat != rtcm->ephsat) return false;
  int week = (eph->week - 1024) % 4096; /* Gst-week = gal-week - 1024 */
  int toe = ROUND(eph->toes / 60.0L);
  int toc = ROUND(time2gpst(eph->toc, NULL) / 60.0L);
  uint32_t sqrtA = ROUND_U(sqrtl(eph->A) / P2_19);
  uint32_t e = ROUND_U(eph->e / P2_33);
  int i0 = ROUND(eph->i0 / P2_31 / SC2RAD);
  int OMG0 = ROUND(eph->OMG0 / P2_31 / SC2RAD);
  int omg = ROUND(eph->omg / P2_31 / SC2RAD);
  int M0 = ROUND(eph->M0 / P2_31 / SC2RAD);
  int deln = ROUND(eph->deln / P2_43 / SC2RAD);
  int idot = ROUND(eph->idot / P2_43 / SC2RAD);
  int OMGd = ROUND(eph->OMGd / P2_43 / SC2RAD);
  int crs = ROUND(eph->crs / P2_5);
  int crc = ROUND(eph->crc / P2_5);
  int cus = ROUND(eph->cus / P2_29);
  int cuc = ROUND(eph->cuc / P2_29);
  int cis = ROUND(eph->cis / P2_29);
  int cic = ROUND(eph->cic / P2_29);
  int af0 = ROUND(eph->f0 / P2_34);
  int af1 = ROUND(eph->f1 / P2_46);
  int af2 = ROUND(eph->f2 / P2_59);
  int bgd1 = ROUND(eph->tgd[0] / P2_32); /* E5a/E1 */
  int bgd2 = ROUND(eph->tgd[1] / P2_32); /* E5b/E1 */
  int oshs = (eph->svh >> 4) & 3;        /* E5a SVH */
  int osdvs = (eph->svh >> 3) & 1;       /* E5a DVS */
  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1045);
  i += 12;
  rtcm_setbitu(rtcm, i, 6, prn);
  i += 6;
  rtcm_setbitu(rtcm, i, 12, week);
  i += 12;
  rtcm_setbitu(rtcm, i, 10, eph->iode);
  i += 10;
  rtcm_setbitu(rtcm, i, 8, eph->sva);
  i += 8;
  rtcm_setbits(rtcm, i, 14, idot);
  i += 14;
  rtcm_setbitu(rtcm, i, 14, toc);
  i += 14;
  rtcm_setbits(rtcm, i, 6, af2);
  i += 6;
  rtcm_setbits(rtcm, i, 21, af1);
  i += 21;
  rtcm_setbits(rtcm, i, 31, af0);
  i += 31;
  rtcm_setbits(rtcm, i, 16, crs);
  i += 16;
  rtcm_setbits(rtcm, i, 16, deln);
  i += 16;
  rtcm_setbits(rtcm, i, 32, M0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cuc);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, e);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cus);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, sqrtA);
  i += 32;
  rtcm_setbitu(rtcm, i, 14, toe);
  i += 14;
  rtcm_setbits(rtcm, i, 16, cic);
  i += 16;
  rtcm_setbits(rtcm, i, 32, OMG0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cis);
  i += 16;
  rtcm_setbits(rtcm, i, 32, i0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, crc);
  i += 16;
  rtcm_setbits(rtcm, i, 32, omg);
  i += 32;
  rtcm_setbits(rtcm, i, 24, OMGd);
  i += 24;
  rtcm_setbits(rtcm, i, 10, bgd1);
  i += 10;
  rtcm_setbitu(rtcm, i, 2, oshs);
  i += 2; /* E5a SVH */
  rtcm_setbitu(rtcm, i, 1, osdvs);
  i += 1; /* E5a DVS */
  rtcm_setbitu(rtcm, i, 7, 0);
  i += 7; /* Reserved */
  rtcm->nbit = i;
  return true;
}
/* Encode type 1046: Galileo I/NAV satellite ephemerides ---------------------*/
static bool encode_type1046(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1046: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_GAL) return false;
  eph_t *eph = rtcm->nav.eph[rtcm->ephsat - 1]; /* I/NAV */
  if (eph->sat != rtcm->ephsat) return false;
  int week = (eph->week - 1024) % 4096; /* Gst-week = gal-week - 1024 */
  int toe = ROUND(eph->toes / 60.0L);
  int toc = ROUND(time2gpst(eph->toc, NULL) / 60.0L);
  uint32_t sqrtA = ROUND_U(sqrtl(eph->A) / P2_19);
  uint32_t e = ROUND_U(eph->e / P2_33);
  int i0 = ROUND(eph->i0 / P2_31 / SC2RAD);
  int OMG0 = ROUND(eph->OMG0 / P2_31 / SC2RAD);
  int omg = ROUND(eph->omg / P2_31 / SC2RAD);
  int M0 = ROUND(eph->M0 / P2_31 / SC2RAD);
  int deln = ROUND(eph->deln / P2_43 / SC2RAD);
  int idot = ROUND(eph->idot / P2_43 / SC2RAD);
  int OMGd = ROUND(eph->OMGd / P2_43 / SC2RAD);
  int crs = ROUND(eph->crs / P2_5);
  int crc = ROUND(eph->crc / P2_5);
  int cus = ROUND(eph->cus / P2_29);
  int cuc = ROUND(eph->cuc / P2_29);
  int cis = ROUND(eph->cis / P2_29);
  int cic = ROUND(eph->cic / P2_29);
  int af0 = ROUND(eph->f0 / P2_34);
  int af1 = ROUND(eph->f1 / P2_46);
  int af2 = ROUND(eph->f2 / P2_59);
  int bgd1 = ROUND(eph->tgd[0] / P2_32); /* E5a/E1 */
  int bgd2 = ROUND(eph->tgd[1] / P2_32); /* E5b/E1 */
  int oshs1 = (eph->svh >> 7) & 3;       /* E5b SVH */
  int osdvs1 = (eph->svh >> 6) & 1;      /* E5b DVS */
  int oshs2 = (eph->svh >> 1) & 3;       /* E1 SVH */
  int osdvs2 = (eph->svh >> 0) & 1;      /* E1 DVS */
  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1046);
  i += 12;
  rtcm_setbitu(rtcm, i, 6, prn);
  i += 6;
  rtcm_setbitu(rtcm, i, 12, week);
  i += 12;
  rtcm_setbitu(rtcm, i, 10, eph->iode);
  i += 10;
  rtcm_setbitu(rtcm, i, 8, eph->sva);
  i += 8;
  rtcm_setbits(rtcm, i, 14, idot);
  i += 14;
  rtcm_setbitu(rtcm, i, 14, toc);
  i += 14;
  rtcm_setbits(rtcm, i, 6, af2);
  i += 6;
  rtcm_setbits(rtcm, i, 21, af1);
  i += 21;
  rtcm_setbits(rtcm, i, 31, af0);
  i += 31;
  rtcm_setbits(rtcm, i, 16, crs);
  i += 16;
  rtcm_setbits(rtcm, i, 16, deln);
  i += 16;
  rtcm_setbits(rtcm, i, 32, M0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cuc);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, e);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cus);
  i += 16;
  rtcm_setbitu(rtcm, i, 32, sqrtA);
  i += 32;
  rtcm_setbitu(rtcm, i, 14, toe);
  i += 14;
  rtcm_setbits(rtcm, i, 16, cic);
  i += 16;
  rtcm_setbits(rtcm, i, 32, OMG0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, cis);
  i += 16;
  rtcm_setbits(rtcm, i, 32, i0);
  i += 32;
  rtcm_setbits(rtcm, i, 16, crc);
  i += 16;
  rtcm_setbits(rtcm, i, 32, omg);
  i += 32;
  rtcm_setbits(rtcm, i, 24, OMGd);
  i += 24;
  rtcm_setbits(rtcm, i, 10, bgd1);
  i += 10;
  rtcm_setbits(rtcm, i, 10, bgd2);
  i += 10;
  rtcm_setbitu(rtcm, i, 2, oshs1);
  i += 2; /* E5b SVH */
  rtcm_setbitu(rtcm, i, 1, osdvs1);
  i += 1; /* E5b DVS */
  rtcm_setbitu(rtcm, i, 2, oshs2);
  i += 2; /* E1 SVH */
  rtcm_setbitu(rtcm, i, 1, osdvs2);
  i += 1; /* E1 DVS */
  rtcm->nbit = i;
  return true;
}
/* Encode type 1042: BeiDou ephemerides --------------------------------------*/
static bool encode_type1042(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1042: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_CMP) return false;
  eph_t *eph = rtcm->nav.eph[rtcm->ephsat - 1];
  if (eph->sat != rtcm->ephsat) return false;
  int week = eph->week % 8192;
  int toe = ROUND(eph->toes / 8.0L);
  int toc = ROUND(time2bdt(gpst2bdt(eph->toc), NULL) / 8.0L); /* GPST -> BDT */
  uint32_t sqrtA = ROUND_U(sqrtl(eph->A) / P2_19);
  uint32_t e = ROUND_U(eph->e / P2_33);
  int i0 = ROUND(eph->i0 / P2_31 / SC2RAD);
  int OMG0 = ROUND(eph->OMG0 / P2_31 / SC2RAD);
  int omg = ROUND(eph->omg / P2_31 / SC2RAD);
  int M0 = ROUND(eph->M0 / P2_31 / SC2RAD);
  int deln = ROUND(eph->deln / P2_43 / SC2RAD);
  int idot = ROUND(eph->idot / P2_43 / SC2RAD);
  int OMGd = ROUND(eph->OMGd / P2_43 / SC2RAD);
  int crs = ROUND(eph->crs / P2_6);
  int crc = ROUND(eph->crc / P2_6);
  int cus = ROUND(eph->cus / P2_31);
  int cuc = ROUND(eph->cuc / P2_31);
  int cis = ROUND(eph->cis / P2_31);
  int cic = ROUND(eph->cic / P2_31);
  int af0 = ROUND(eph->f0 / P2_33);
  int af1 = ROUND(eph->f1 / P2_50);
  int af2 = ROUND(eph->f2 / P2_66);
  int tgd1 = ROUND(eph->tgd[0] / 1E-10L);
  int tgd2 = ROUND(eph->tgd[1] / 1E-10L);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1042);
  i += 12;
  rtcm_setbitu(rtcm, i, 6, prn);
  i += 6;
  rtcm_setbitu(rtcm, i, 13, week);
  i += 13;
  rtcm_setbitu(rtcm, i, 4, eph->sva);
  i += 4;
  rtcm_setbits(rtcm, i, 14, idot);
  i += 14;
  rtcm_setbitu(rtcm, i, 5, eph->iode);
  i += 5;
  rtcm_setbitu(rtcm, i, 17, toc);
  i += 17;
  rtcm_setbits(rtcm, i, 11, af2);
  i += 11;
  rtcm_setbits(rtcm, i, 22, af1);
  i += 22;
  rtcm_setbits(rtcm, i, 24, af0);
  i += 24;
  rtcm_setbitu(rtcm, i, 5, eph->iodc);
  i += 5;
  rtcm_setbits(rtcm, i, 18, crs);
  i += 18;
  rtcm_setbits(rtcm, i, 16, deln);
  i += 16;
  rtcm_setbits(rtcm, i, 32, M0);
  i += 32;
  rtcm_setbits(rtcm, i, 18, cuc);
  i += 18;
  rtcm_setbitu(rtcm, i, 32, e);
  i += 32;
  rtcm_setbits(rtcm, i, 18, cus);
  i += 18;
  rtcm_setbitu(rtcm, i, 32, sqrtA);
  i += 32;
  rtcm_setbitu(rtcm, i, 17, toe);
  i += 17;
  rtcm_setbits(rtcm, i, 18, cic);
  i += 18;
  rtcm_setbits(rtcm, i, 32, OMG0);
  i += 32;
  rtcm_setbits(rtcm, i, 18, cis);
  i += 18;
  rtcm_setbits(rtcm, i, 32, i0);
  i += 32;
  rtcm_setbits(rtcm, i, 18, crc);
  i += 18;
  rtcm_setbits(rtcm, i, 32, omg);
  i += 32;
  rtcm_setbits(rtcm, i, 24, OMGd);
  i += 24;
  rtcm_setbits(rtcm, i, 10, tgd1);
  i += 10;
  rtcm_setbits(rtcm, i, 10, tgd2);
  i += 10;
  rtcm_setbitu(rtcm, i, 1, eph->svh);
  i += 1;
  rtcm->nbit = i;
  return true;
}
/* Encode type 63: BeiDou ephemerides (RTCM draft) ---------------------------*/
static bool encode_type63(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type63: sync=%d\n", sync);

  int prn;
  if (satsys(rtcm->ephsat, &prn) != SYS_CMP) return false;
  eph_t *eph = rtcm->nav.eph[rtcm->ephsat - 1];
  if (eph->sat != rtcm->ephsat) return false;
  int week = eph->week % 8192;
  int toe = ROUND(eph->toes / 8.0L);
  int toc = ROUND(time2bdt(gpst2bdt(eph->toc), NULL) / 8.0L); /* GPST -> BDT */
  uint32_t sqrtA = ROUND_U(sqrtl(eph->A) / P2_19);
  uint32_t e = ROUND_U(eph->e / P2_33);
  int i0 = ROUND(eph->i0 / P2_31 / SC2RAD);
  int OMG0 = ROUND(eph->OMG0 / P2_31 / SC2RAD);
  int omg = ROUND(eph->omg / P2_31 / SC2RAD);
  int M0 = ROUND(eph->M0 / P2_31 / SC2RAD);
  int deln = ROUND(eph->deln / P2_43 / SC2RAD);
  int idot = ROUND(eph->idot / P2_43 / SC2RAD);
  int OMGd = ROUND(eph->OMGd / P2_43 / SC2RAD);
  int crs = ROUND(eph->crs / P2_6);
  int crc = ROUND(eph->crc / P2_6);
  int cus = ROUND(eph->cus / P2_31);
  int cuc = ROUND(eph->cuc / P2_31);
  int cis = ROUND(eph->cis / P2_31);
  int cic = ROUND(eph->cic / P2_31);
  int af0 = ROUND(eph->f0 / P2_33);
  int af1 = ROUND(eph->f1 / P2_50);
  int af2 = ROUND(eph->f2 / P2_66);
  int tgd1 = ROUND(eph->tgd[0] / 1E-10L);
  int tgd2 = ROUND(eph->tgd[1] / 1E-10L);

  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 63);
  i += 12;
  rtcm_setbitu(rtcm, i, 6, prn);
  i += 6;
  rtcm_setbitu(rtcm, i, 13, week);
  i += 13;
  rtcm_setbitu(rtcm, i, 4, eph->sva);
  i += 4;
  rtcm_setbits(rtcm, i, 14, idot);
  i += 14;
  rtcm_setbitu(rtcm, i, 5, eph->iode);
  i += 5;
  rtcm_setbitu(rtcm, i, 17, toc);
  i += 17;
  rtcm_setbits(rtcm, i, 11, af2);
  i += 11;
  rtcm_setbits(rtcm, i, 22, af1);
  i += 22;
  rtcm_setbits(rtcm, i, 24, af0);
  i += 24;
  rtcm_setbitu(rtcm, i, 5, eph->iodc);
  i += 5;
  rtcm_setbits(rtcm, i, 18, crs);
  i += 18;
  rtcm_setbits(rtcm, i, 16, deln);
  i += 16;
  rtcm_setbits(rtcm, i, 32, M0);
  i += 32;
  rtcm_setbits(rtcm, i, 18, cuc);
  i += 18;
  rtcm_setbitu(rtcm, i, 32, e);
  i += 32;
  rtcm_setbits(rtcm, i, 18, cus);
  i += 18;
  rtcm_setbitu(rtcm, i, 32, sqrtA);
  i += 32;
  rtcm_setbitu(rtcm, i, 17, toe);
  i += 17;
  rtcm_setbits(rtcm, i, 18, cic);
  i += 18;
  rtcm_setbits(rtcm, i, 32, OMG0);
  i += 32;
  rtcm_setbits(rtcm, i, 18, cis);
  i += 18;
  rtcm_setbits(rtcm, i, 32, i0);
  i += 32;
  rtcm_setbits(rtcm, i, 18, crc);
  i += 18;
  rtcm_setbits(rtcm, i, 32, omg);
  i += 32;
  rtcm_setbits(rtcm, i, 24, OMGd);
  i += 24;
  rtcm_setbits(rtcm, i, 10, tgd1);
  i += 10;
  rtcm_setbits(rtcm, i, 10, tgd2);
  i += 10;
  rtcm_setbitu(rtcm, i, 1, eph->svh);
  i += 1;
  rtcm->nbit = i;
  return true;
}
/* Encode SSR header ---------------------------------------------------------*/
static int encode_ssr_head(int type, rtcm_t *rtcm, int sys, int subtype, int nsat, int sync,
                           int iod, long double udint, int refd, int provid, int solid) {
  trace(4,
        "encode_ssr_head: type=%d sys=%d subtype=%d nsat=%d sync=%d iod=%d "
        "udint=%.0Lf\n",
        type, sys, subtype, nsat, sync, iod, udint);

  int i = 24, ns;
  if (subtype == 0) { /* RTCM SSR */
    ns = (sys == SYS_QZS) ? 4 : 6;
    int msgno;
    switch (sys) {
      case SYS_GPS:
        msgno = (type == 7) ? 11 : 1056 + type;
        break;
      case SYS_GLO:
        msgno = (type == 7) ? 0 : 1062 + type;
        break;
      case SYS_GAL:
        msgno = (type == 7) ? 12 : 1239 + type;
        break; /* Draft */
      case SYS_QZS:
        msgno = (type == 7) ? 13 : 1245 + type;
        break; /* Draft */
      case SYS_CMP:
        msgno = (type == 7) ? 14 : 1257 + type;
        break; /* Draft */
      case SYS_SBS:
        msgno = (type == 7) ? 0 : 1251 + type;
        break; /* Draft */
      default:
        return 0;
    }
    if (msgno == 0) {
      return 0;
    }
    rtcm_setbitu(rtcm, i, 12, msgno);
    i += 12; /* Message type */

    if (sys == SYS_GLO) {
      int week;
      long double tow = time2gpst(timeadd(gpst2utc(rtcm->time), 10800.0L), &week);
      int epoch = ROUND(tow) % 86400;
      rtcm_setbitu(rtcm, i, 17, epoch);
      i += 17; /* GLONASS epoch time */
    } else {
      int week;
      long double tow = time2gpst(rtcm->time, &week);
      int epoch = ROUND(tow) % 604800;
      rtcm_setbitu(rtcm, i, 20, epoch);
      i += 20; /* GPS epoch time */
    }
  } else { /* IGS SSR */
    ns = 6;
    int week;
    long double tow = time2gpst(rtcm->time, &week);
    int epoch = ROUND(tow) % 604800;
    rtcm_setbitu(rtcm, i, 12, 4076);
    i += 12; /* Message type */
    rtcm_setbitu(rtcm, i, 3, 1);
    i += 3; /* Version */
    rtcm_setbitu(rtcm, i, 8, subtype);
    i += 8; /* Subtype */
    rtcm_setbitu(rtcm, i, 20, epoch);
    i += 20; /* SSR epoch time */
  }
  int udi;
  for (udi = 0; udi < 15; udi++) {
    if (ssrudint[udi] >= udint) break;
  }
  rtcm_setbitu(rtcm, i, 4, udi);
  i += 4; /* Update interval */
  rtcm_setbitu(rtcm, i, 1, sync);
  i += 1; /* Multiple message indicator */
  if (subtype == 0 && (type == 1 || type == 4)) {
    rtcm_setbitu(rtcm, i, 1, refd);
    i += 1; /* Satellite ref datum */
  }
  rtcm_setbitu(rtcm, i, 4, iod);
  i += 4; /* IOD SSR */
  rtcm_setbitu(rtcm, i, 16, provid);
  i += 16; /* Provider ID */
  rtcm_setbitu(rtcm, i, 4, solid);
  i += 4; /* Solution ID */
  if (subtype > 0 && (type == 1 || type == 4)) {
    rtcm_setbitu(rtcm, i, 1, refd);
    i += 1; /* Global/regional CRS indicator */
  }
  if (type == 7) {
    rtcm_setbitu(rtcm, i, 1, 0);
    i += 1; /* Dispersive bias consistency ind */
    rtcm_setbitu(rtcm, i, 1, 0);
    i += 1; /* MW consistency indicator */
  }
  rtcm_setbitu(rtcm, i, ns, nsat);
  i += ns; /* No of satellites */
  return i;
}
/* SSR signal and tracking mode IDs ------------------------------------------*/
static const int codes_gps[32] = {CODE_L1C, CODE_L1P, CODE_L1W, CODE_L1S, CODE_L1L, CODE_L2C,
                                  CODE_L2D, CODE_L2S, CODE_L2L, CODE_L2X, CODE_L2P, CODE_L2W,
                                  0,        0,        CODE_L5I, CODE_L5Q};
static const int codes_glo[32] = {CODE_L1C, CODE_L1P, CODE_L2C, CODE_L2P, CODE_L4A,
                                  CODE_L4B, CODE_L6A, CODE_L6B, CODE_L3I, CODE_L3Q};
static const int codes_gal[32] = {CODE_L1A, CODE_L1B, CODE_L1C, 0,        0,       CODE_L5I,
                                  CODE_L5Q, 0,        CODE_L7I, CODE_L7Q, 0,       CODE_L8I,
                                  CODE_L8Q, 0,        CODE_L6A, CODE_L6B, CODE_L6C};
static const int codes_qzs[32] = {CODE_L1C, CODE_L1S, CODE_L1L, CODE_L2S, CODE_L2L, 0,
                                  CODE_L5I, CODE_L5Q, 0,        CODE_L6S, CODE_L6L, 0,
                                  0,        0,        0,        0,        0,        CODE_L6E};
static const int codes_bds[32] = {
    CODE_L2I, CODE_L2Q, 0,        CODE_L6I, CODE_L6Q, 0, CODE_L7I, CODE_L7Q, 0, CODE_L1D, CODE_L1P,
    0,        CODE_L5D, CODE_L5P, 0,        CODE_L1A, 0, 0,        CODE_L6A};
static const int codes_sbs[32] = {CODE_L1C, CODE_L5I, CODE_L5Q};
/* Encode SSR 1: orbit corrections -------------------------------------------*/
static bool encode_ssr1(rtcm_t *rtcm, int sys, int subtype, int sync) {
  trace(3, "encode_ssr1: sys=%d subtype=%d sync=%d\n", sys, subtype, sync);

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
      return false;
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
  /* Number of satellites */
  int nsat = 0;
  long double udint = 0.0L;
  int iod = 0, refd = 0;
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    nsat++;
    udint = rtcm->ssr[j].udi[0];
    iod = rtcm->ssr[j].iod[0];
    refd = rtcm->ssr[j].refd;
  }
  /* Encode SSR header */
  int i = encode_ssr_head(1, rtcm, sys, subtype, nsat, sync, iod, udint, refd, 0, 0);

  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;

    int iode = rtcm->ssr[j].iode;     /* SBAS/BDS: toe/t0 modulo */
    int iodcrc = rtcm->ssr[j].iodcrc; /* SBAS/BDS: IOD CRC */

    if (subtype > 0) { /* IGS SSR */
      iode &= 0xFF;
    }
    int deph[3];
    deph[0] = ROUND(rtcm->ssr[j].deph[0] / 1E-4L);
    deph[1] = ROUND(rtcm->ssr[j].deph[1] / 4E-4L);
    deph[2] = ROUND(rtcm->ssr[j].deph[2] / 4E-4L);
    int ddeph[3];
    ddeph[0] = ROUND(rtcm->ssr[j].ddeph[0] / 1E-6L);
    ddeph[1] = ROUND(rtcm->ssr[j].ddeph[1] / 4E-6L);
    ddeph[2] = ROUND(rtcm->ssr[j].ddeph[2] / 4E-6L);

    rtcm_setbitu(rtcm, i, np, prn - offp);
    i += np; /* Satellite ID */
    rtcm_setbitu(rtcm, i, ni, iode);
    i += ni; /* IODE */
    rtcm_setbitu(rtcm, i, nj, iodcrc);
    i += nj; /* IODCRC */
    rtcm_setbits(rtcm, i, 22, deph[0]);
    i += 22; /* Delta radial */
    rtcm_setbits(rtcm, i, 20, deph[1]);
    i += 20; /* Delta along-track */
    rtcm_setbits(rtcm, i, 20, deph[2]);
    i += 20; /* Delta cross-track */
    rtcm_setbits(rtcm, i, 21, ddeph[0]);
    i += 21; /* Dot delta radial */
    rtcm_setbits(rtcm, i, 19, ddeph[1]);
    i += 19; /* Dot delta along-track */
    rtcm_setbits(rtcm, i, 19, ddeph[2]);
    i += 19; /* Dot delta cross-track */
  }
  rtcm->nbit = i;
  return true;
}
/* Encode SSR 2: clock corrections -------------------------------------------*/
static bool encode_ssr2(rtcm_t *rtcm, int sys, int subtype, int sync) {
  trace(3, "encode_ssr2: sys=%d subtype=%d sync=%d\n", sys, subtype, sync);

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
      return false;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  /* Number of satellites */
  int nsat = 0;
  long double udint = 0.0L;
  int iod = 0;
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    nsat++;
    udint = rtcm->ssr[j].udi[1];
    iod = rtcm->ssr[j].iod[1];
  }
  /* Encode SSR header */
  int i = encode_ssr_head(2, rtcm, sys, subtype, nsat, sync, iod, udint, 0, 0, 0);

  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;

    int dclk[3];
    dclk[0] = ROUND(rtcm->ssr[j].dclk[0] / 1E-4L);
    dclk[1] = ROUND(rtcm->ssr[j].dclk[1] / 1E-6L);
    dclk[2] = ROUND(rtcm->ssr[j].dclk[2] / 2E-8L);

    rtcm_setbitu(rtcm, i, np, prn - offp);
    i += np; /* Satellite ID */
    rtcm_setbits(rtcm, i, 22, dclk[0]);
    i += 22; /* Delta clock C0 */
    rtcm_setbits(rtcm, i, 21, dclk[1]);
    i += 21; /* Delta clock C1 */
    rtcm_setbits(rtcm, i, 27, dclk[2]);
    i += 27; /* Delta clock C2 */
  }
  rtcm->nbit = i;
  return true;
}
/* Encode SSR 3: satellite code biases ---------------------------------------*/
static bool encode_ssr3(rtcm_t *rtcm, int sys, int subtype, int sync) {
  trace(3, "encode_ssr3: sys=%d subtype=%d sync=%d\n", sys, subtype, sync);

  int np, offp;
  const int *codes;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      codes = codes_gps;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      codes = codes_glo;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      codes = codes_gal;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      codes = codes_qzs;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      codes = codes_bds;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      codes = codes_sbs;
      break;
    default:
      return false;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  /* Number of satellites */
  int nsat = 0, iod = 0;
  long double udint = 0.0L;
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    nsat++;
    udint = rtcm->ssr[j].udi[4];
    iod = rtcm->ssr[j].iod[4];
  }
  /* Encode SSR header */
  int i = encode_ssr_head(3, rtcm, sys, subtype, nsat, sync, iod, udint, 0, 0, 0);

  int code[MAXCODE], bias[MAXCODE];
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;

    int nbias = 0;
    for (int k = 0; k < 32; k++) {
      if (!codes[k] || rtcm->ssr[j].cbias[codes[k] - 1] == 0.0L) continue;
      code[nbias] = k;
      bias[nbias++] = ROUND(rtcm->ssr[j].cbias[codes[k] - 1] / 0.01L);
    }
    rtcm_setbitu(rtcm, i, np, prn - offp);
    i += np; /* Satellite ID */
    rtcm_setbitu(rtcm, i, 5, nbias);
    i += 5; /* Number of code biases */

    for (int k = 0; k < nbias; k++) {
      rtcm_setbitu(rtcm, i, 5, code[k]);
      i += 5; /* Signal indicator */
      rtcm_setbits(rtcm, i, 14, bias[k]);
      i += 14; /* Code bias */
    }
  }
  rtcm->nbit = i;
  return true;
}
/* Encode SSR 4: combined orbit and clock corrections ------------------------*/
static bool encode_ssr4(rtcm_t *rtcm, int sys, int subtype, int sync) {
  trace(3, "encode_ssr4: sys=%d subtype=%d sync=%d\n", sys, subtype, sync);

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
      return false;
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
  /* Number of satellites */
  int nsat = 0, iod = 0, refd = 0;
  long double udint = 0.0L;
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    nsat++;
    udint = rtcm->ssr[j].udi[0];
    iod = rtcm->ssr[j].iod[0];
    refd = rtcm->ssr[j].refd;
  }
  /* Encode SSR header */
  int i = encode_ssr_head(4, rtcm, sys, subtype, nsat, sync, iod, udint, refd, 0, 0);

  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;

    int iode = rtcm->ssr[j].iode;
    int iodcrc = rtcm->ssr[j].iodcrc;

    if (subtype > 0) { /* IGS SSR */
      iode &= 0xFF;
    }
    int deph[3];
    deph[0] = ROUND(rtcm->ssr[j].deph[0] / 1E-4L);
    deph[1] = ROUND(rtcm->ssr[j].deph[1] / 4E-4L);
    deph[2] = ROUND(rtcm->ssr[j].deph[2] / 4E-4L);
    int ddeph[3];
    ddeph[0] = ROUND(rtcm->ssr[j].ddeph[0] / 1E-6L);
    ddeph[1] = ROUND(rtcm->ssr[j].ddeph[1] / 4E-6L);
    ddeph[2] = ROUND(rtcm->ssr[j].ddeph[2] / 4E-6L);
    int dclk[3];
    dclk[0] = ROUND(rtcm->ssr[j].dclk[0] / 1E-4L);
    dclk[1] = ROUND(rtcm->ssr[j].dclk[1] / 1E-6L);
    dclk[2] = ROUND(rtcm->ssr[j].dclk[2] / 2E-8L);

    rtcm_setbitu(rtcm, i, np, prn - offp);
    i += np; /* Satellite ID */
    rtcm_setbitu(rtcm, i, ni, iode);
    i += ni; /* IODE */
    rtcm_setbitu(rtcm, i, nj, iodcrc);
    i += nj; /* IODCRC */
    rtcm_setbits(rtcm, i, 22, deph[0]);
    i += 22; /* Delta raidal */
    rtcm_setbits(rtcm, i, 20, deph[1]);
    i += 20; /* Delta along-track */
    rtcm_setbits(rtcm, i, 20, deph[2]);
    i += 20; /* Delta cross-track */
    rtcm_setbits(rtcm, i, 21, ddeph[0]);
    i += 21; /* Dot delta radial */
    rtcm_setbits(rtcm, i, 19, ddeph[1]);
    i += 19; /* Dot delta along-track */
    rtcm_setbits(rtcm, i, 19, ddeph[2]);
    i += 19; /* Dot delta cross-track */
    rtcm_setbits(rtcm, i, 22, dclk[0]);
    i += 22; /* Delta clock C0 */
    rtcm_setbits(rtcm, i, 21, dclk[1]);
    i += 21; /* Delta clock C1 */
    rtcm_setbits(rtcm, i, 27, dclk[2]);
    i += 27; /* Delta clock C2 */
  }
  rtcm->nbit = i;
  return true;
}
/* Encode SSR 5: URA ---------------------------------------------------------*/
static bool encode_ssr5(rtcm_t *rtcm, int sys, int subtype, int sync) {
  trace(3, "encode_ssr5: sys=%d subtype=%d sync=%d\n", sys, subtype, sync);

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
      return false;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  /* Number of satellites */
  int nsat = 0, iod = 0;
  long double udint = 0.0L;
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    nsat++;
    udint = rtcm->ssr[j].udi[3];
    iod = rtcm->ssr[j].iod[3];
  }
  /* Encode SSR header */
  int i = encode_ssr_head(5, rtcm, sys, subtype, nsat, sync, iod, udint, 0, 0, 0);

  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    int ura = rtcm->ssr[j].ura;
    rtcm_setbitu(rtcm, i, np, prn - offp);
    i += np; /* Satellite id */
    rtcm_setbitu(rtcm, i, 6, ura);
    i += 6; /* SSR URA */
  }
  rtcm->nbit = i;
  return true;
}
/* Encode SSR 6: high rate clock correction ----------------------------------*/
static bool encode_ssr6(rtcm_t *rtcm, int sys, int subtype, int sync) {
  trace(3, "encode_ssr6: sys=%d subtype=%d sync=%d\n", sys, subtype, sync);

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
      return false;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  /* Number of satellites */
  int nsat = 0, iod = 0;
  long double udint = 0.0L;
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    nsat++;
    udint = rtcm->ssr[j].udi[2];
    iod = rtcm->ssr[j].iod[2];
  }
  /* Encode SSR header */
  int i = encode_ssr_head(6, rtcm, sys, subtype, nsat, sync, iod, udint, 0, 0, 0);

  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;

    int hrclk = ROUND(rtcm->ssr[j].hrclk / 1E-4L);

    rtcm_setbitu(rtcm, i, np, prn - offp);
    i += np; /* Satellite ID */
    rtcm_setbits(rtcm, i, 22, hrclk);
    i += 22; /* High rate clock corr */
  }
  rtcm->nbit = i;
  return true;
}
/* Encode SSR 7: satellite phase biases --------------------------------------*/
static bool encode_ssr7(rtcm_t *rtcm, int sys, int subtype, int sync) {
  trace(3, "encode_ssr7: sys=%d subtype=%d sync=%d\n", sys, subtype, sync);

  int np, offp;
  const int *codes;
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      codes = codes_gps;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      codes = codes_glo;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      codes = codes_gal;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      codes = codes_qzs;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      codes = codes_bds;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      codes = codes_sbs;
      break;
    default:
      return false;
  }
  if (subtype > 0) { /* IGS SSR */
    np = 6;
    if (sys == SYS_CMP)
      offp = 0;
    else if (sys == SYS_SBS)
      offp = 119;
  }
  /* Number of satellites */
  int nsat = 0, iod = 0;
  long double udint = 0.0L;
  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;
    nsat++;
    udint = rtcm->ssr[j].udi[5];
    iod = rtcm->ssr[j].iod[5];
  }
  /* Encode SSR header */
  int i = encode_ssr_head(7, rtcm, sys, subtype, nsat, sync, iod, udint, 0, 0, 0);

  for (int j = 0; j < MAXSAT; j++) {
    int prn;
    if (satsys(j + 1, &prn) != sys || !rtcm->ssr[j].update) continue;

    int nbias = 0;
    int code[MAXCODE], pbias[MAXCODE], stdpb[MAXCODE];
    for (int k = 0; k < 32; k++) {
      if (!codes[k] || rtcm->ssr[j].pbias[codes[k] - 1] == 0.0L) continue;
      code[nbias] = k;
      pbias[nbias] = ROUND(rtcm->ssr[j].pbias[codes[k] - 1] / 0.0001L);
      stdpb[nbias++] = ROUND(rtcm->ssr[j].stdpb[codes[k] - 1] / 0.0001L);
    }
    int yaw_ang = ROUND(rtcm->ssr[j].yaw_ang / 180.0L * 256.0L);
    int yaw_rate = ROUND(rtcm->ssr[j].yaw_rate / 180.0L * 8192.0L);
    rtcm_setbitu(rtcm, i, np, prn - offp);
    i += np; /* Satellite ID */
    rtcm_setbitu(rtcm, i, 5, nbias);
    i += 5; /* Number of code biases */
    rtcm_setbitu(rtcm, i, 9, yaw_ang);
    i += 9; /* Yaw angle */
    rtcm_setbits(rtcm, i, 8, yaw_rate);
    i += 8; /* Yaw rate */

    for (int k = 0; k < nbias; k++) {
      rtcm_setbitu(rtcm, i, 5, code[k]);
      i += 5; /* Signal indicator */
      rtcm_setbitu(rtcm, i, 1, 0);
      i += 1; /* Integer-indicator */
      rtcm_setbitu(rtcm, i, 2, 0);
      i += 2; /* WL integer-indicator */
      rtcm_setbitu(rtcm, i, 4, 0);
      i += 4; /* Discont counter */
      rtcm_setbits(rtcm, i, 20, pbias[k]);
      i += 20; /* Phase bias */
      if (subtype == 0) {
        rtcm_setbits(rtcm, i, 17, stdpb[k]);
        i += 17; /* Std-dev ph-bias */
      }
    }
  }
  rtcm->nbit = i;
  return true;
}
/* Satellite no to MSM satellite ID ------------------------------------------*/
static int to_satid(int sys, int sat) {
  int prn;
  if (satsys(sat, &prn) != sys) return 0;

  if (sys == SYS_QZS)
    prn -= MINPRNQZS - 1;
  else if (sys == SYS_SBS)
    prn -= MINPRNSBS - 1;

  return prn;
}
/* Observation code to MSM signal ID -----------------------------------------*/
static int to_sigid(int sys, uint8_t code) {
  /* Signal conversion for undefined signal by RTCM */
  if (sys == SYS_GPS) {
    if (code == CODE_L1Y)
      code = CODE_L1P;
    else if (code == CODE_L1M)
      code = CODE_L1P;
    else if (code == CODE_L1N)
      code = CODE_L1P;
    else if (code == CODE_L2D)
      code = CODE_L2P;
    else if (code == CODE_L2Y)
      code = CODE_L2P;
    else if (code == CODE_L2M)
      code = CODE_L2P;
    else if (code == CODE_L2N)
      code = CODE_L2P;
  }
  char *sig = code2obs(code);
  if (!*sig) return 0;

  const char **msm_sig;
  switch (sys) {
    case SYS_GPS:
      msm_sig = msm_sig_gps;
      break;
    case SYS_GLO:
      msm_sig = msm_sig_glo;
      break;
    case SYS_GAL:
      msm_sig = msm_sig_gal;
      break;
    case SYS_QZS:
      msm_sig = msm_sig_qzs;
      break;
    case SYS_SBS:
      msm_sig = msm_sig_sbs;
      break;
    case SYS_CMP:
      msm_sig = msm_sig_cmp;
      break;
    case SYS_IRN:
      msm_sig = msm_sig_irn;
      break;
    default:
      return 0;
  }
  for (int i = 0; i < 32; i++) {
    if (!strcmp(sig, msm_sig[i])) return i + 1;
  }
  return 0;
}
/* Generate MSM satellite, signal and cell index -----------------------------*/
static void gen_msm_index(rtcm_t *rtcm, int sys, int *nsat, int *nsig, int *ncell, uint8_t *sat_ind,
                          uint8_t *sig_ind, uint8_t *cell_ind) {
  *nsat = *nsig = *ncell = 0;

  /* Generate satellite and signal index */
  for (int i = 0; i < rtcm->obs.n; i++) {
    int sat = to_satid(sys, rtcm->obs.data[i].sat);
    if (!sat) continue;

    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      int sig = to_sigid(sys, rtcm->obs.data[i].code[j]);
      if (!sig) continue;

      sat_ind[sat - 1] = sig_ind[sig - 1] = 1;
    }
  }
  for (int i = 0; i < 64; i++) {
    if (sat_ind[i]) sat_ind[i] = ++(*nsat);
  }
  for (int i = 0; i < 32; i++) {
    if (sig_ind[i]) sig_ind[i] = ++(*nsig);
  }
  /* Generate cell index */
  for (int i = 0; i < rtcm->obs.n; i++) {
    int sat = to_satid(sys, rtcm->obs.data[i].sat);
    if (!sat) continue;

    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      int sig = to_sigid(sys, rtcm->obs.data[i].code[j]);
      if (!sig) continue;

      int cell = sig_ind[sig - 1] - 1 + (sat_ind[sat - 1] - 1) * (*nsig);
      cell_ind[cell] = 1;
    }
  }
  for (int i = 0; i < *nsat * (*nsig); i++) {
    if (cell_ind[i] && *ncell < 64) cell_ind[i] = ++(*ncell);
  }
}
/* Generate MSM satellite data fields ----------------------------------------*/
static void gen_msm_sat(rtcm_t *rtcm, int sys, int nsat, const uint8_t *sat_ind, long double *rrng,
                        long double *rrate, uint8_t *info) {
  for (int i = 0; i < 64; i++) rrng[i] = rrate[i] = 0.0L;

  for (int i = 0; i < rtcm->obs.n; i++) {
    obsd_t *data = rtcm->obs.data + i;
    int fcn = fcn_glo(data->sat, rtcm); /* Fcn+7 */

    int sat = to_satid(sys, data->sat);
    if (!sat) continue;

    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      int sig = to_sigid(sys, data->code[j]);
      if (!sig) continue;
      int k = sat_ind[sat - 1] - 1;
      long double freq = code2freq(sys, data->code[j], fcn - 7);

      /* Rough range (ms) and rough phase-range-rate (m/s) */
      if (rrng[k] == 0.0L && data->P[j] != 0.0L) {
        rrng[k] = ROUND(data->P[j] / RANGE_MS / P2_10) * RANGE_MS * P2_10;
      }
      if (rrate[k] == 0.0L && data->D[j] != 0.0L && freq > 0.0L) {
        rrate[k] = ROUND(-data->D[j] * CLIGHT / freq) * 1.0L;
      }
      /* Extended satellite info */
      if (info) info[k] = sys != SYS_GLO ? 0 : (fcn < 0 ? 15 : fcn);
    }
  }
}
/* Generate MSM signal data fields -------------------------------------------*/
static void gen_msm_sig(rtcm_t *rtcm, int sys, int nsat, int nsig, int ncell,
                        const uint8_t *sat_ind, const uint8_t *sig_ind, const uint8_t *cell_ind,
                        const long double *rrng, const long double *rrate, long double *psrng,
                        long double *phrng, long double *rate, long double *lock, uint8_t *half,
                        long double *cnr) {
  for (int i = 0; i < ncell; i++) {
    if (psrng) psrng[i] = 0.0L;
    if (phrng) phrng[i] = 0.0L;
    if (rate) rate[i] = 0.0L;
  }
  for (int i = 0; i < rtcm->obs.n; i++) {
    obsd_t *data = rtcm->obs.data + i;
    int fcn = fcn_glo(data->sat, rtcm); /* Fcn+7 */

    int sat = to_satid(sys, data->sat);
    if (!sat) continue;

    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      int sig = to_sigid(sys, data->code[j]);
      if (!sig) continue;

      int k = sat_ind[sat - 1] - 1;
      int cell = cell_ind[sig_ind[sig - 1] - 1 + k * nsig];
      if (cell >= 64) continue;

      long double freq = code2freq(sys, data->code[j], fcn - 7);
      long double lambda = freq == 0.0L ? 0.0L : CLIGHT / freq;
      long double psrng_s = data->P[j] == 0.0L ? 0.0L : data->P[j] - rrng[k];
      long double phrng_s =
          data->L[j] == 0.0L || lambda <= 0.0L ? 0.0L : data->L[j] * lambda - rrng[k];
      long double rate_s =
          data->D[j] == 0.0L || lambda <= 0.0L ? 0.0L : -data->D[j] * lambda - rrate[k];

      /* Subtract phase - psudorange integer cycle offset */
      int LLI = data->LLI[j];
      if ((LLI & 1) || fabsl(phrng_s - rtcm->cp[data->sat - 1][j]) > 1171.0L) {
        rtcm->cp[data->sat - 1][j] = ROUND(phrng_s / lambda) * lambda;
        LLI |= 1;
      }
      phrng_s -= rtcm->cp[data->sat - 1][j];

      long double lt = locktime_d(data->time, rtcm->lltime[data->sat - 1] + j, LLI);

      if (psrng && psrng_s != 0.0L) psrng[cell - 1] = psrng_s;
      if (phrng && phrng_s != 0.0L) phrng[cell - 1] = phrng_s;
      if (rate && rate_s != 0.0L) rate[cell - 1] = rate_s;
      if (lock) lock[cell - 1] = lt;
      if (half) half[cell - 1] = (data->LLI[j] & 2) ? 1 : 0;
      if (cnr) cnr[cell - 1] = data->SNR[j] * SNR_UNIT;
    }
  }
}
/* Encode MSM header ---------------------------------------------------------*/
static int encode_msm_head(int type, rtcm_t *rtcm, int sys, int sync, int *nsat, int *ncell,
                           long double *rrng, long double *rrate, uint8_t *info, long double *psrng,
                           long double *phrng, long double *rate, long double *lock, uint8_t *half,
                           long double *cnr) {
  switch (sys) {
    case SYS_GPS:
      type += 1070;
      break;
    case SYS_GLO:
      type += 1080;
      break;
    case SYS_GAL:
      type += 1090;
      break;
    case SYS_QZS:
      type += 1110;
      break;
    case SYS_SBS:
      type += 1100;
      break;
    case SYS_CMP:
      type += 1120;
      break;
    case SYS_IRN:
      type += 1130;
      break;
    default:
      return 0;
  }
  /* Generate MSM satellite, signal and cell index */
  int nsig = 0;
  uint8_t sat_ind[64] = {0}, sig_ind[32] = {0}, cell_ind[32 * 64] = {0};
  gen_msm_index(rtcm, sys, nsat, &nsig, ncell, sat_ind, sig_ind, cell_ind);

  uint32_t epoch;
  if (sys == SYS_GLO) {
    /* GLONASS time (dow + tod-ms) */
    long double tow = time2gpst(timeadd(gpst2utc(rtcm->time), 10800.0L), NULL);
    uint32_t dow = (uint32_t)(tow / 86400.0L);
    epoch = (dow << 27) + ROUND_U(fmodl(tow, 86400.0L) * 1E3L);
  } else if (sys == SYS_CMP) {
    /* BDS time (tow-ms) */
    epoch = ROUND_U(time2gpst(gpst2bdt(rtcm->time), NULL) * 1E3L);
  } else {
    /* GPS, QZSS, Galileo and IRNSS time (tow-ms) */
    epoch = ROUND_U(time2gpst(rtcm->time, NULL) * 1E3L);
  }
  int i = 24;
  /* Encode MSM header (ref [15] table 3.5-78) */
  rtcm_setbitu(rtcm, i, 12, type);
  i += 12; /* Message number */
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12; /* Reference station id */
  rtcm_setbitu(rtcm, i, 30, epoch);
  i += 30; /* Epoch time */
  rtcm_setbitu(rtcm, i, 1, sync);
  i += 1; /* Multiple message bit */
  rtcm_setbitu(rtcm, i, 3, rtcm->seqno);
  i += 3; /* Issue of data station */
  rtcm_setbitu(rtcm, i, 7, 0);
  i += 7; /* Reserved */
  rtcm_setbitu(rtcm, i, 2, 0);
  i += 2; /* Clock streering indicator */
  rtcm_setbitu(rtcm, i, 2, 0);
  i += 2; /* External clock indicator */
  rtcm_setbitu(rtcm, i, 1, 0);
  i += 1; /* Smoothing indicator */
  rtcm_setbitu(rtcm, i, 3, 0);
  i += 3; /* Smoothing interval */

  /* Satellite mask */
  for (int j = 0; j < 64; j++) {
    rtcm_setbitu(rtcm, i, 1, sat_ind[j] ? 1 : 0);
    i += 1;
  }
  /* Signal mask */
  for (int j = 0; j < 32; j++) {
    rtcm_setbitu(rtcm, i, 1, sig_ind[j] ? 1 : 0);
    i += 1;
  }
  /* Cell mask */
  for (int j = 0; j < *nsat * nsig && j < 64; j++) {
    rtcm_setbitu(rtcm, i, 1, cell_ind[j] ? 1 : 0);
    i += 1;
  }
  /* Generate MSM satellite data fields */
  gen_msm_sat(rtcm, sys, *nsat, sat_ind, rrng, rrate, info);

  /* Generate MSM signal data fields */
  gen_msm_sig(rtcm, sys, *nsat, nsig, *ncell, sat_ind, sig_ind, cell_ind, rrng, rrate, psrng, phrng,
              rate, lock, half, cnr);

  return i;
}
/* Encode rough range integer ms ---------------------------------------------*/
static int encode_msm_int_rrng(rtcm_t *rtcm, int i, const long double *rrng, int nsat) {
  for (int j = 0; j < nsat; j++) {
    uint32_t int_ms;
    if (rrng[j] == 0.0L) {
      int_ms = 255;
    } else if (rrng[j] < 0.0L || rrng[j] > RANGE_MS * 255.0L) {
      char tstr[40];
      trace(2, "msm rough range overflow %s rrng=%.3Lf\n", time2str(rtcm->time, tstr, 0), rrng[j]);
      int_ms = 255;
    } else {
      int_ms = ROUND_U(rrng[j] / RANGE_MS / P2_10) >> 10;
    }
    rtcm_setbitu(rtcm, i, 8, int_ms);
    i += 8;
  }
  return i;
}
/* Encode rough range modulo 1 ms --------------------------------------------*/
static int encode_msm_mod_rrng(rtcm_t *rtcm, int i, const long double *rrng, int nsat) {
  for (int j = 0; j < nsat; j++) {
    uint32_t mod_ms;
    if (rrng[j] <= 0.0L || rrng[j] > RANGE_MS * 255.0L) {
      mod_ms = 0;
    } else {
      mod_ms = ROUND_U(rrng[j] / RANGE_MS / P2_10) & 0x3FFu;
    }
    rtcm_setbitu(rtcm, i, 10, mod_ms);
    i += 10;
  }
  return i;
}
/* Encode extended satellite info --------------------------------------------*/
static int encode_msm_info(rtcm_t *rtcm, int i, const uint8_t *info, int nsat) {
  for (int j = 0; j < nsat; j++) {
    rtcm_setbitu(rtcm, i, 4, info[j]);
    i += 4;
  }
  return i;
}
/* Encode rough phase-range-rate ---------------------------------------------*/
static int encode_msm_rrate(rtcm_t *rtcm, int i, const long double *rrate, int nsat) {
  for (int j = 0; j < nsat; j++) {
    int rrate_val;
    if (fabsl(rrate[j]) > 8191.0L) {
      char tstr[40];
      trace(2, "msm rough phase-range-rate overflow %s rrate=%.4Lf\n",
            time2str(rtcm->time, tstr, 0), rrate[j]);
      rrate_val = -8192;
    } else {
      rrate_val = ROUND(rrate[j] / 1.0L);
    }
    rtcm_setbits(rtcm, i, 14, rrate_val);
    i += 14;
  }
  return i;
}
/* Encode fine pseudorange ---------------------------------------------------*/
static int encode_msm_psrng(rtcm_t *rtcm, int i, const long double *psrng, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int psrng_val;
    if (psrng[j] == 0.0L) {
      psrng_val = -16384;
    } else if (fabsl(psrng[j]) > 292.7L) {
      char tstr[40];
      trace(2, "msm fine pseudorange overflow %s psrng=%.3Lf\n", time2str(rtcm->time, tstr, 0),
            psrng[j]);
      psrng_val = -16384;
    } else {
      psrng_val = ROUND(psrng[j] / RANGE_MS / P2_24);
    }
    rtcm_setbits(rtcm, i, 15, psrng_val);
    i += 15;
  }
  return i;
}
/* Encode fine pseudorange with extended resolution --------------------------*/
static int encode_msm_psrng_ex(rtcm_t *rtcm, int i, const long double *psrng, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int psrng_val;
    if (psrng[j] == 0.0L) {
      psrng_val = -524288;
    } else if (fabsl(psrng[j]) > 292.7L) {
      char tstr[40];
      trace(2, "msm fine pseudorange ext overflow %s psrng=%.3Lf\n", time2str(rtcm->time, tstr, 0),
            psrng[j]);
      psrng_val = -524288;
    } else {
      psrng_val = ROUND(psrng[j] / RANGE_MS / P2_29);
    }
    rtcm_setbits(rtcm, i, 20, psrng_val);
    i += 20;
  }
  return i;
}
/* Encode fine phase-range ---------------------------------------------------*/
static int encode_msm_phrng(rtcm_t *rtcm, int i, const long double *phrng, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int phrng_val;
    if (phrng[j] == 0.0L) {
      phrng_val = -2097152;
    } else if (fabsl(phrng[j]) > 1171.0L) {
      char tstr[40];
      trace(2, "msm fine phase-range overflow %s phrng=%.3Lf\n", time2str(rtcm->time, tstr, 0),
            phrng[j]);
      phrng_val = -2097152;
    } else {
      phrng_val = ROUND(phrng[j] / RANGE_MS / P2_29);
    }
    rtcm_setbits(rtcm, i, 22, phrng_val);
    i += 22;
  }
  return i;
}
/* Encode fine phase-range with extended resolution --------------------------*/
static int encode_msm_phrng_ex(rtcm_t *rtcm, int i, const long double *phrng, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int phrng_val;
    if (phrng[j] == 0.0L) {
      phrng_val = -8388608;
    } else if (fabsl(phrng[j]) > 1171.0L) {
      char tstr[40];
      trace(2, "msm fine phase-range ext overflow %s phrng=%.3Lf\n", time2str(rtcm->time, tstr, 0),
            phrng[j]);
      phrng_val = -8388608;
    } else {
      phrng_val = ROUND(phrng[j] / RANGE_MS / P2_31);
    }
    rtcm_setbits(rtcm, i, 24, phrng_val);
    i += 24;
  }
  return i;
}
/* Encode lock-time indicator ------------------------------------------------*/
static int encode_msm_lock(rtcm_t *rtcm, int i, const long double *lock, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int lock_val = to_msm_lock(lock[j]);
    rtcm_setbitu(rtcm, i, 4, lock_val);
    i += 4;
  }
  return i;
}
/* Encode lock-time indicator with extended range and resolution -------------*/
static int encode_msm_lock_ex(rtcm_t *rtcm, int i, const long double *lock, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int lock_val = to_msm_lock_ex(lock[j]);
    rtcm_setbitu(rtcm, i, 10, lock_val);
    i += 10;
  }
  return i;
}
/* Encode half-cycle-ambiguity indicator -------------------------------------*/
static int encode_msm_half_amb(rtcm_t *rtcm, int i, const uint8_t *half, int ncell) {
  for (int j = 0; j < ncell; j++) {
    rtcm_setbitu(rtcm, i, 1, half[j]);
    i += 1;
  }
  return i;
}
/* Encode signal CNR ---------------------------------------------------------*/
static int encode_msm_cnr(rtcm_t *rtcm, int i, const long double *cnr, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int cnr_val = ROUND(cnr[j] / 1.0L);
    rtcm_setbitu(rtcm, i, 6, cnr_val);
    i += 6;
  }
  return i;
}
/* Encode signal CNR with extended resolution --------------------------------*/
static int encode_msm_cnr_ex(rtcm_t *rtcm, int i, const long double *cnr, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int cnr_val = ROUND(cnr[j] / 0.0625L);
    rtcm_setbitu(rtcm, i, 10, cnr_val);
    i += 10;
  }
  return i;
}
/* Encode fine phase-range-rate ----------------------------------------------*/
static int encode_msm_rate(rtcm_t *rtcm, int i, const long double *rate, int ncell) {
  for (int j = 0; j < ncell; j++) {
    int rate_val;
    if (rate[j] == 0.0L) {
      rate_val = -16384;
    } else if (fabsl(rate[j]) > 1.6384L) {
      char tstr[40];
      trace(2, "msm fine phase-range-rate overflow %s rate=%.3Lf\n", time2str(rtcm->time, tstr, 0),
            rate[j]);
      rate_val = -16384;
    } else {
      rate_val = ROUND(rate[j] / 0.0001L);
    }
    rtcm_setbitu(rtcm, i, 15, rate_val);
    i += 15;
  }
  return i;
}
/* Encode MSM 1: compact pseudorange -----------------------------------------*/
static bool encode_msm1(rtcm_t *rtcm, int sys, int sync) {
  trace(3, "encode_msm1: sys=%d sync=%d\n", sys, sync);

  /* Encode MSM header */
  int nsat, ncell;
  long double rrng[64], rrate[64], psrng[64];
  int i = encode_msm_head(1, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, NULL, psrng, NULL, NULL,
                          NULL, NULL, NULL);
  if (!i) {
    return false;
  }
  /* Encode MSM satellite data */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* Rough range modulo 1 ms */

  /* Encode MSM signal data */
  i = encode_msm_psrng(rtcm, i, psrng, ncell); /* Fine pseudorange */

  rtcm->nbit = i;
  return true;
}
/* Encode MSM 2: compact phaserange ------------------------------------------*/
static bool encode_msm2(rtcm_t *rtcm, int sys, int sync) {
  trace(3, "encode_msm2: sys=%d sync=%d\n", sys, sync);

  /* Encode MSM header */
  int nsat, ncell;
  long double rrng[64], rrate[64], phrng[64], lock[64];
  uint8_t half[64];
  int i = encode_msm_head(2, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, NULL, NULL, phrng, NULL,
                          lock, half, NULL);
  if (!i) {
    return false;
  }
  /* Encode MSM satellite data */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* Rough range modulo 1 ms */

  /* Encode MSM signal data */
  i = encode_msm_phrng(rtcm, i, phrng, ncell);   /* Fine phase-range */
  i = encode_msm_lock(rtcm, i, lock, ncell);     /* Lock-time indicator */
  i = encode_msm_half_amb(rtcm, i, half, ncell); /* Half-cycle-amb indicator */

  rtcm->nbit = i;
  return true;
}
/* Encode MSM 3: compact pseudorange and phaserange --------------------------*/
static bool encode_msm3(rtcm_t *rtcm, int sys, int sync) {
  trace(3, "encode_msm3: sys=%d sync=%d\n", sys, sync);

  /* Encode MSM header */
  int nsat, ncell;
  long double rrng[64], rrate[64], psrng[64], phrng[64], lock[64];
  uint8_t half[64];
  int i = encode_msm_head(3, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, NULL, psrng, phrng, NULL,
                          lock, half, NULL);
  if (!i) {
    return false;
  }
  /* Encode MSM satellite data */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* Rough range modulo 1 ms */

  /* Encode MSM signal data */
  i = encode_msm_psrng(rtcm, i, psrng, ncell);   /* Fine pseudorange */
  i = encode_msm_phrng(rtcm, i, phrng, ncell);   /* Fine phase-range */
  i = encode_msm_lock(rtcm, i, lock, ncell);     /* Lock-time indicator */
  i = encode_msm_half_amb(rtcm, i, half, ncell); /* Half-cycle-amb indicator */

  rtcm->nbit = i;
  return true;
}
/* Encode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
static bool encode_msm4(rtcm_t *rtcm, int sys, int sync) {
  trace(3, "encode_msm4: sys=%d sync=%d\n", sys, sync);

  /* Encode MSM header */
  int nsat, ncell;
  long double rrng[64], rrate[64], psrng[64], phrng[64], lock[64];
  uint8_t half[64];
  long double cnr[64];
  int i = encode_msm_head(4, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, NULL, psrng, phrng, NULL,
                          lock, half, cnr);
  if (!i) {
    return false;
  }
  /* Encode MSM satellite data */
  i = encode_msm_int_rrng(rtcm, i, rrng, nsat); /* Rough range integer ms */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* Rough range modulo 1 ms */

  /* Encode MSM signal data */
  i = encode_msm_psrng(rtcm, i, psrng, ncell);   /* Fine pseudorange */
  i = encode_msm_phrng(rtcm, i, phrng, ncell);   /* Fine phase-range */
  i = encode_msm_lock(rtcm, i, lock, ncell);     /* Lock-time indicator */
  i = encode_msm_half_amb(rtcm, i, half, ncell); /* Half-cycle-amb indicator */
  i = encode_msm_cnr(rtcm, i, cnr, ncell);       /* Signal cnr */
  rtcm->nbit = i;
  return true;
}
/* Encode MSM 5: full pseudorange, phaserange, phaserangerate and CNR --------*/
static bool encode_msm5(rtcm_t *rtcm, int sys, int sync) {
  trace(3, "encode_msm5: sys=%d sync=%d\n", sys, sync);

  /* Encode MSM header */
  int nsat, ncell;
  long double rrng[64], rrate[64], psrng[64], phrng[64], rate[64], lock[64];
  uint8_t info[64], half[64];
  long double cnr[64];
  int i = encode_msm_head(5, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, info, psrng, phrng, rate,
                          lock, half, cnr);
  if (!i) {
    return false;
  }
  /* Encode MSM satellite data */
  i = encode_msm_int_rrng(rtcm, i, rrng, nsat); /* Rough range integer ms */
  i = encode_msm_info(rtcm, i, info, nsat);     /* Extended satellite info */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* Rough range modulo 1 ms */
  i = encode_msm_rrate(rtcm, i, rrate, nsat);   /* Rough phase-range-rate */

  /* Encode MSM signal data */
  i = encode_msm_psrng(rtcm, i, psrng, ncell);   /* Fine pseudorange */
  i = encode_msm_phrng(rtcm, i, phrng, ncell);   /* Fine phase-range */
  i = encode_msm_lock(rtcm, i, lock, ncell);     /* Lock-time indicator */
  i = encode_msm_half_amb(rtcm, i, half, ncell); /* Half-cycle-amb indicator */
  i = encode_msm_cnr(rtcm, i, cnr, ncell);       /* Signal cnr */
  i = encode_msm_rate(rtcm, i, rate, ncell);     /* Fine phase-range-rate */
  rtcm->nbit = i;
  return true;
}
/* Encode MSM 6: full pseudorange and phaserange plus CNR (high-res) ---------*/
static bool encode_msm6(rtcm_t *rtcm, int sys, int sync) {
  trace(3, "encode_msm6: sys=%d sync=%d\n", sys, sync);

  /* Encode MSM header */
  int nsat, ncell;
  long double rrng[64], rrate[64], psrng[64], phrng[64], lock[64];
  uint8_t half[64];
  long double cnr[64];
  int i = encode_msm_head(6, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, NULL, psrng, phrng, NULL,
                          lock, half, cnr);
  if (!i) {
    return false;
  }
  /* Encode MSM satellite data */
  i = encode_msm_int_rrng(rtcm, i, rrng, nsat); /* Rough range integer ms */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* Rough range modulo 1 ms */

  /* Encode MSM signal data */
  i = encode_msm_psrng_ex(rtcm, i, psrng, ncell); /* Fine pseudorange ext */
  i = encode_msm_phrng_ex(rtcm, i, phrng, ncell); /* Fine phase-range ext */
  i = encode_msm_lock_ex(rtcm, i, lock, ncell);   /* Lock-time indicator ext */
  i = encode_msm_half_amb(rtcm, i, half, ncell);  /* Half-cycle-amb indicator */
  i = encode_msm_cnr_ex(rtcm, i, cnr, ncell);     /* Signal cnr ext */
  rtcm->nbit = i;
  return true;
}
/* Encode MSM 7: full pseudorange, phaserange, phaserangerate and CNR (h-res) */
static bool encode_msm7(rtcm_t *rtcm, int sys, int sync) {
  trace(3, "encode_msm7: sys=%d sync=%d\n", sys, sync);

  /* Encode MSM header */
  int nsat, ncell;
  long double rrng[64], rrate[64], psrng[64], phrng[64], rate[64], lock[64];
  uint8_t info[64], half[64];
  long double cnr[64];
  int i = encode_msm_head(7, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, info, psrng, phrng, rate,
                          lock, half, cnr);
  if (!i) return false;

  /* Encode MSM satellite data */
  i = encode_msm_int_rrng(rtcm, i, rrng, nsat); /* Rough range integer ms */
  i = encode_msm_info(rtcm, i, info, nsat);     /* Extended satellite info */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat); /* Rough range modulo 1 ms */
  i = encode_msm_rrate(rtcm, i, rrate, nsat);   /* Rough phase-range-rate */

  /* Encode MSM signal data */
  i = encode_msm_psrng_ex(rtcm, i, psrng, ncell); /* Fine pseudorange ext */
  i = encode_msm_phrng_ex(rtcm, i, phrng, ncell); /* Fine phase-range ext */
  i = encode_msm_lock_ex(rtcm, i, lock, ncell);   /* Lock-time indicator ext */
  i = encode_msm_half_amb(rtcm, i, half, ncell);  /* Half-cycle-amb indicator */
  i = encode_msm_cnr_ex(rtcm, i, cnr, ncell);     /* Signal cnr ext */
  i = encode_msm_rate(rtcm, i, rate, ncell);      /* Fine phase-range-rate */
  rtcm->nbit = i;
  return true;
}
/* Encode type 1230: GLONASS L1 and L2 code-phase biases ---------------------*/
static bool encode_type1230(rtcm_t *rtcm, int sync) {
  trace(3, "encode_type1230: sync=%d\n", sync);

  int align = rtcm->sta.glo_cp_align;

  int bias[4];
  for (int j = 0; j < 4; j++) {
    bias[j] = ROUND(rtcm->sta.glo_cp_bias[j] / 0.02L);
    if (bias[j] <= -32768 || bias[j] > 32767) {
      bias[j] = -32768; /* Invalid value */
    }
  }
  int i = 24;
  rtcm_setbitu(rtcm, i, 12, 1230);
  i += 12; /* Message no */
  rtcm_setbitu(rtcm, i, 12, rtcm->staid);
  i += 12; /* Station ID */
  rtcm_setbitu(rtcm, i, 1, align);
  i += 1; /* GLO code-phase bias ind */
  rtcm_setbitu(rtcm, i, 3, 0);
  i += 3; /* Reserved */
  int mask = 15;
  rtcm_setbitu(rtcm, i, 4, mask);
  i += 4; /* GLO FDMA signals mask */
  rtcm_setbits(rtcm, i, 16, bias[0]);
  i += 16; /* GLO C1 code-phase bias */
  rtcm_setbits(rtcm, i, 16, bias[1]);
  i += 16; /* GLO P1 code-phase bias */
  rtcm_setbits(rtcm, i, 16, bias[2]);
  i += 16; /* GLO C2 code-phase bias */
  rtcm_setbits(rtcm, i, 16, bias[3]);
  i += 16; /* GLO P2 code-phase bias */
  rtcm->nbit = i;
  return true;
}
/* Encode type 4073: proprietary message Mitsubishi Electric -----------------*/
static bool encode_type4073(rtcm_t *rtcm, int subtype, int sync) {
  trace(2, "rtcm3 4073: unsupported message subtype=%d\n", subtype);
  return false;
}
/* Encode type 4076: proprietary message IGS ---------------------------------*/
static bool encode_type4076(rtcm_t *rtcm, int subtype, int sync) {
  switch (subtype) {
    case 21:
      return encode_ssr1(rtcm, SYS_GPS, subtype, sync);
    case 22:
      return encode_ssr2(rtcm, SYS_GPS, subtype, sync);
    case 23:
      return encode_ssr4(rtcm, SYS_GPS, subtype, sync);
    case 24:
      return encode_ssr6(rtcm, SYS_GPS, subtype, sync);
    case 25:
      return encode_ssr3(rtcm, SYS_GPS, subtype, sync);
    case 26:
      return encode_ssr7(rtcm, SYS_GPS, subtype, sync);
    case 27:
      return encode_ssr5(rtcm, SYS_GPS, subtype, sync);
    case 41:
      return encode_ssr1(rtcm, SYS_GLO, subtype, sync);
    case 42:
      return encode_ssr2(rtcm, SYS_GLO, subtype, sync);
    case 43:
      return encode_ssr4(rtcm, SYS_GLO, subtype, sync);
    case 44:
      return encode_ssr6(rtcm, SYS_GLO, subtype, sync);
    case 45:
      return encode_ssr3(rtcm, SYS_GLO, subtype, sync);
    case 46:
      return encode_ssr7(rtcm, SYS_GLO, subtype, sync);
    case 47:
      return encode_ssr5(rtcm, SYS_GLO, subtype, sync);
    case 61:
      return encode_ssr1(rtcm, SYS_GAL, subtype, sync);
    case 62:
      return encode_ssr2(rtcm, SYS_GAL, subtype, sync);
    case 63:
      return encode_ssr4(rtcm, SYS_GAL, subtype, sync);
    case 64:
      return encode_ssr6(rtcm, SYS_GAL, subtype, sync);
    case 65:
      return encode_ssr3(rtcm, SYS_GAL, subtype, sync);
    case 66:
      return encode_ssr7(rtcm, SYS_GAL, subtype, sync);
    case 67:
      return encode_ssr5(rtcm, SYS_GAL, subtype, sync);
    case 81:
      return encode_ssr1(rtcm, SYS_QZS, subtype, sync);
    case 82:
      return encode_ssr2(rtcm, SYS_QZS, subtype, sync);
    case 83:
      return encode_ssr4(rtcm, SYS_QZS, subtype, sync);
    case 84:
      return encode_ssr6(rtcm, SYS_QZS, subtype, sync);
    case 85:
      return encode_ssr3(rtcm, SYS_QZS, subtype, sync);
    case 86:
      return encode_ssr7(rtcm, SYS_QZS, subtype, sync);
    case 87:
      return encode_ssr5(rtcm, SYS_QZS, subtype, sync);
    case 101:
      return encode_ssr1(rtcm, SYS_CMP, subtype, sync);
    case 102:
      return encode_ssr2(rtcm, SYS_CMP, subtype, sync);
    case 103:
      return encode_ssr4(rtcm, SYS_CMP, subtype, sync);
    case 104:
      return encode_ssr6(rtcm, SYS_CMP, subtype, sync);
    case 105:
      return encode_ssr3(rtcm, SYS_CMP, subtype, sync);
    case 106:
      return encode_ssr7(rtcm, SYS_CMP, subtype, sync);
    case 107:
      return encode_ssr5(rtcm, SYS_CMP, subtype, sync);
    case 121:
      return encode_ssr1(rtcm, SYS_SBS, subtype, sync);
    case 122:
      return encode_ssr2(rtcm, SYS_SBS, subtype, sync);
    case 123:
      return encode_ssr4(rtcm, SYS_SBS, subtype, sync);
    case 124:
      return encode_ssr6(rtcm, SYS_SBS, subtype, sync);
    case 125:
      return encode_ssr3(rtcm, SYS_SBS, subtype, sync);
    case 126:
      return encode_ssr7(rtcm, SYS_SBS, subtype, sync);
    case 127:
      return encode_ssr5(rtcm, SYS_SBS, subtype, sync);
  }
  trace(2, "rtcm3 4076: unsupported message subtype=%d\n", subtype);
  return false;
}
/* Encode RTCM ver.3 message -------------------------------------------------*/
extern bool encode_rtcm3(rtcm_t *rtcm, int type, int subtype, int sync) {
  trace(0, "encode_rtcm3: type=%d subtype=%d sync=%d\n", type, subtype, sync);

  bool ret = false;
  switch (type) {
    case 1001:
      ret = encode_type1001(rtcm, sync);
      break;
    case 1002:
      ret = encode_type1002(rtcm, sync);
      break;
    case 1003:
      ret = encode_type1003(rtcm, sync);
      break;
    case 1004:
      ret = encode_type1004(rtcm, sync);
      break;
    case 1005:
      ret = encode_type1005(rtcm, sync);
      break;
    case 1006:
      ret = encode_type1006(rtcm, sync);
      break;
    case 1007:
      ret = encode_type1007(rtcm, sync);
      break;
    case 1008:
      ret = encode_type1008(rtcm, sync);
      break;
    case 1009:
      ret = encode_type1009(rtcm, sync);
      break;
    case 1010:
      ret = encode_type1010(rtcm, sync);
      break;
    case 1011:
      ret = encode_type1011(rtcm, sync);
      break;
    case 1012:
      ret = encode_type1012(rtcm, sync);
      break;
    case 1019:
      ret = encode_type1019(rtcm, sync);
      break;
    case 1020:
      ret = encode_type1020(rtcm, sync);
      break;
    case 1033:
      ret = encode_type1033(rtcm, sync);
      break;
    case 1041:
      ret = encode_type1041(rtcm, sync);
      break;
    case 1042:
      ret = encode_type1042(rtcm, sync);
      break;
    case 1044:
      ret = encode_type1044(rtcm, sync);
      break;
    case 1045:
      ret = encode_type1045(rtcm, sync);
      break;
    case 1046:
      ret = encode_type1046(rtcm, sync);
      break;
    case 63:
      ret = encode_type63(rtcm, sync);
      break; /* Draft */
    case 1057:
      ret = encode_ssr1(rtcm, SYS_GPS, 0, sync);
      break;
    case 1058:
      ret = encode_ssr2(rtcm, SYS_GPS, 0, sync);
      break;
    case 1059:
      ret = encode_ssr3(rtcm, SYS_GPS, 0, sync);
      break;
    case 1060:
      ret = encode_ssr4(rtcm, SYS_GPS, 0, sync);
      break;
    case 1061:
      ret = encode_ssr5(rtcm, SYS_GPS, 0, sync);
      break;
    case 1062:
      ret = encode_ssr6(rtcm, SYS_GPS, 0, sync);
      break;
    case 1063:
      ret = encode_ssr1(rtcm, SYS_GLO, 0, sync);
      break;
    case 1064:
      ret = encode_ssr2(rtcm, SYS_GLO, 0, sync);
      break;
    case 1065:
      ret = encode_ssr3(rtcm, SYS_GLO, 0, sync);
      break;
    case 1066:
      ret = encode_ssr4(rtcm, SYS_GLO, 0, sync);
      break;
    case 1067:
      ret = encode_ssr5(rtcm, SYS_GLO, 0, sync);
      break;
    case 1068:
      ret = encode_ssr6(rtcm, SYS_GLO, 0, sync);
      break;
    case 1071:
      ret = encode_msm1(rtcm, SYS_GPS, sync);
      break;
    case 1072:
      ret = encode_msm2(rtcm, SYS_GPS, sync);
      break;
    case 1073:
      ret = encode_msm3(rtcm, SYS_GPS, sync);
      break;
    case 1074:
      ret = encode_msm4(rtcm, SYS_GPS, sync);
      break;
    case 1075:
      ret = encode_msm5(rtcm, SYS_GPS, sync);
      break;
    case 1076:
      ret = encode_msm6(rtcm, SYS_GPS, sync);
      break;
    case 1077:
      ret = encode_msm7(rtcm, SYS_GPS, sync);
      break;
    case 1081:
      ret = encode_msm1(rtcm, SYS_GLO, sync);
      break;
    case 1082:
      ret = encode_msm2(rtcm, SYS_GLO, sync);
      break;
    case 1083:
      ret = encode_msm3(rtcm, SYS_GLO, sync);
      break;
    case 1084:
      ret = encode_msm4(rtcm, SYS_GLO, sync);
      break;
    case 1085:
      ret = encode_msm5(rtcm, SYS_GLO, sync);
      break;
    case 1086:
      ret = encode_msm6(rtcm, SYS_GLO, sync);
      break;
    case 1087:
      ret = encode_msm7(rtcm, SYS_GLO, sync);
      break;
    case 1091:
      ret = encode_msm1(rtcm, SYS_GAL, sync);
      break;
    case 1092:
      ret = encode_msm2(rtcm, SYS_GAL, sync);
      break;
    case 1093:
      ret = encode_msm3(rtcm, SYS_GAL, sync);
      break;
    case 1094:
      ret = encode_msm4(rtcm, SYS_GAL, sync);
      break;
    case 1095:
      ret = encode_msm5(rtcm, SYS_GAL, sync);
      break;
    case 1096:
      ret = encode_msm6(rtcm, SYS_GAL, sync);
      break;
    case 1097:
      ret = encode_msm7(rtcm, SYS_GAL, sync);
      break;
    case 1101:
      ret = encode_msm1(rtcm, SYS_SBS, sync);
      break;
    case 1102:
      ret = encode_msm2(rtcm, SYS_SBS, sync);
      break;
    case 1103:
      ret = encode_msm3(rtcm, SYS_SBS, sync);
      break;
    case 1104:
      ret = encode_msm4(rtcm, SYS_SBS, sync);
      break;
    case 1105:
      ret = encode_msm5(rtcm, SYS_SBS, sync);
      break;
    case 1106:
      ret = encode_msm6(rtcm, SYS_SBS, sync);
      break;
    case 1107:
      ret = encode_msm7(rtcm, SYS_SBS, sync);
      break;
    case 1111:
      ret = encode_msm1(rtcm, SYS_QZS, sync);
      break;
    case 1112:
      ret = encode_msm2(rtcm, SYS_QZS, sync);
      break;
    case 1113:
      ret = encode_msm3(rtcm, SYS_QZS, sync);
      break;
    case 1114:
      ret = encode_msm4(rtcm, SYS_QZS, sync);
      break;
    case 1115:
      ret = encode_msm5(rtcm, SYS_QZS, sync);
      break;
    case 1116:
      ret = encode_msm6(rtcm, SYS_QZS, sync);
      break;
    case 1117:
      ret = encode_msm7(rtcm, SYS_QZS, sync);
      break;
    case 1121:
      ret = encode_msm1(rtcm, SYS_CMP, sync);
      break;
    case 1122:
      ret = encode_msm2(rtcm, SYS_CMP, sync);
      break;
    case 1123:
      ret = encode_msm3(rtcm, SYS_CMP, sync);
      break;
    case 1124:
      ret = encode_msm4(rtcm, SYS_CMP, sync);
      break;
    case 1125:
      ret = encode_msm5(rtcm, SYS_CMP, sync);
      break;
    case 1126:
      ret = encode_msm6(rtcm, SYS_CMP, sync);
      break;
    case 1127:
      ret = encode_msm7(rtcm, SYS_CMP, sync);
      break;
    case 1131:
      ret = encode_msm1(rtcm, SYS_IRN, sync);
      break;
    case 1132:
      ret = encode_msm2(rtcm, SYS_IRN, sync);
      break;
    case 1133:
      ret = encode_msm3(rtcm, SYS_IRN, sync);
      break;
    case 1134:
      ret = encode_msm4(rtcm, SYS_IRN, sync);
      break;
    case 1135:
      ret = encode_msm5(rtcm, SYS_IRN, sync);
      break;
    case 1136:
      ret = encode_msm6(rtcm, SYS_IRN, sync);
      break;
    case 1137:
      ret = encode_msm7(rtcm, SYS_IRN, sync);
      break;
    case 1230:
      ret = encode_type1230(rtcm, sync);
      break;
    case 1240:
      ret = encode_ssr1(rtcm, SYS_GAL, 0, sync);
      break; /* Draft */
    case 1241:
      ret = encode_ssr2(rtcm, SYS_GAL, 0, sync);
      break; /* Draft */
    case 1242:
      ret = encode_ssr3(rtcm, SYS_GAL, 0, sync);
      break; /* Draft */
    case 1243:
      ret = encode_ssr4(rtcm, SYS_GAL, 0, sync);
      break; /* Draft */
    case 1244:
      ret = encode_ssr5(rtcm, SYS_GAL, 0, sync);
      break; /* Draft */
    case 1245:
      ret = encode_ssr6(rtcm, SYS_GAL, 0, sync);
      break; /* Draft */
    case 1246:
      ret = encode_ssr1(rtcm, SYS_QZS, 0, sync);
      break; /* Draft */
    case 1247:
      ret = encode_ssr2(rtcm, SYS_QZS, 0, sync);
      break; /* Draft */
    case 1248:
      ret = encode_ssr3(rtcm, SYS_QZS, 0, sync);
      break; /* Draft */
    case 1249:
      ret = encode_ssr4(rtcm, SYS_QZS, 0, sync);
      break; /* Draft */
    case 1250:
      ret = encode_ssr5(rtcm, SYS_QZS, 0, sync);
      break; /* Draft */
    case 1251:
      ret = encode_ssr6(rtcm, SYS_QZS, 0, sync);
      break; /* Draft */
    case 1252:
      ret = encode_ssr1(rtcm, SYS_SBS, 0, sync);
      break; /* Draft */
    case 1253:
      ret = encode_ssr2(rtcm, SYS_SBS, 0, sync);
      break; /* Draft */
    case 1254:
      ret = encode_ssr3(rtcm, SYS_SBS, 0, sync);
      break; /* Draft */
    case 1255:
      ret = encode_ssr4(rtcm, SYS_SBS, 0, sync);
      break; /* Draft */
    case 1256:
      ret = encode_ssr5(rtcm, SYS_SBS, 0, sync);
      break; /* Draft */
    case 1257:
      ret = encode_ssr6(rtcm, SYS_SBS, 0, sync);
      break; /* Draft */
    case 1258:
      ret = encode_ssr1(rtcm, SYS_CMP, 0, sync);
      break; /* Draft */
    case 1259:
      ret = encode_ssr2(rtcm, SYS_CMP, 0, sync);
      break; /* Draft */
    case 1260:
      ret = encode_ssr3(rtcm, SYS_CMP, 0, sync);
      break; /* Draft */
    case 1261:
      ret = encode_ssr4(rtcm, SYS_CMP, 0, sync);
      break; /* Draft */
    case 1262:
      ret = encode_ssr5(rtcm, SYS_CMP, 0, sync);
      break; /* Draft */
    case 1263:
      ret = encode_ssr6(rtcm, SYS_CMP, 0, sync);
      break; /* Draft */
    case 11:
      ret = encode_ssr7(rtcm, SYS_GPS, 0, sync);
      break; /* Tentative */
    case 12:
      ret = encode_ssr7(rtcm, SYS_GAL, 0, sync);
      break; /* Tentative */
    case 13:
      ret = encode_ssr7(rtcm, SYS_QZS, 0, sync);
      break; /* Tentative */
    case 14:
      ret = encode_ssr7(rtcm, SYS_CMP, 0, sync);
      break; /* Tentative */
    case 4073:
      ret = encode_type4073(rtcm, subtype, sync);
      break;
    case 4076:
      ret = encode_type4076(rtcm, subtype, sync);
      break;
  }
  if (ret > 0) {
    if (1001 <= type && type <= 1299)
      rtcm->nmsg3[type - 1000]++; /*   1-299 */
    else if (4070 <= type && type <= 4099)
      rtcm->nmsg3[type - 3770]++; /* 300-329 */
    else
      rtcm->nmsg3[0]++; /* Other */
  }
  return ret;
}
