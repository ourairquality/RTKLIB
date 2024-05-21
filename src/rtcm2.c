/*------------------------------------------------------------------------------
 * rtcm2.c : RTCM ver.2 message functions
 *
 *          Copyright (C) 2009-2014 by T.TAKASU, All rights reserved.
 *
 * References :
 *     see rtcm.c
 *
 * Version : $Revision:$ $Date:$
 * History : 2011/11/28 1.0  separated from rtcm.c
 *           2014/10/21 1.1  fix problem on week rollover in RTCM 2 type 14
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Adjust hourly rollover of RTCM 2 time -------------------------------------*/
static void adjhour(rtcm_t *rtcm, long double zcnt) {
  /* If no time, get cpu time */
  if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
  int week;
  long double tow = time2gpst(rtcm->time, &week);
  long double hour = floorl(tow / 3600.0L);
  long double sec = tow - hour * 3600.0L;
  if (zcnt < sec - 1800.0L)
    zcnt += 3600.0L;
  else if (zcnt > sec + 1800.0L)
    zcnt -= 3600.0L;
  rtcm->time = gpst2time(week, hour * 3600 + zcnt);
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
  for (int j = 0; j < NFREQ; j++) {
    obs->data[i].L[j] = obs->data[i].P[j] = 0.0L;
    obs->data[i].D[j] = 0.0L;
    obs->data[i].SNR[j] = obs->data[i].LLI[j] = obs->data[i].code[j] = 0;
  }
  obs->n++;
  return i;
}

static uint32_t rtcm_getbitu(const rtcm_t *rtcm, unsigned pos, unsigned len) {
  return getbitu(rtcm->buff, sizeof(rtcm->buff), pos, len);
}
static int32_t rtcm_getbits(const rtcm_t *rtcm, unsigned pos, unsigned len) {
  return getbits(rtcm->buff, sizeof(rtcm->buff), pos, len);
}

/* Decode type 1/9: differential GPS correction/partial correction set -------*/
static int decode_type1(rtcm_t *rtcm) {
  trace(4, "decode_type1: len=%d\n", rtcm->len);

  int i = 48;
  while (i + 40 <= rtcm->len * 8) {
    int fact = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int udre = rtcm_getbitu(rtcm, i, 2);
    i += 2;
    int prn = rtcm_getbitu(rtcm, i, 5);
    i += 5;
    long double prc = rtcm_getbits(rtcm, i, 16);
    i += 16;
    long double rrc = rtcm_getbits(rtcm, i, 8);
    i += 8;
    int iod = rtcm_getbits(rtcm, i, 8);
    i += 8;
    if (prn == 0) prn = 32;
    if (prc == 0x80000000 || rrc == 0xFFFF8000) {
      trace(2, "rtcm2 1 prc/rrc indicates satellite problem: prn=%d\n", prn);
      continue;
    }
    if (rtcm->dgps) {
      int sat = satno(SYS_GPS, prn);
      rtcm->dgps[sat - 1].t0 = rtcm->time;
      rtcm->dgps[sat - 1].prc = prc * (fact ? 0.32L : 0.02L);
      rtcm->dgps[sat - 1].rrc = rrc * (fact ? 0.032L : 0.002L);
      rtcm->dgps[sat - 1].iod = iod;
      rtcm->dgps[sat - 1].udre = udre;
    }
  }
  return 7;
}
/* Decode type 3: reference station parameter --------------------------------*/
static int decode_type3(rtcm_t *rtcm) {
  trace(4, "decode_type3: len=%d\n", rtcm->len);

  int i = 48;
  if (i + 96 > rtcm->len * 8) {
    trace(2, "rtcm2 3 length error: len=%d\n", rtcm->len);
    return -1;
  }
  rtcm->sta.pos[0] = rtcm_getbits(rtcm, i, 32) * 0.01L;
  i += 32;
  rtcm->sta.pos[1] = rtcm_getbits(rtcm, i, 32) * 0.01L;
  i += 32;
  rtcm->sta.pos[2] = rtcm_getbits(rtcm, i, 32) * 0.01L;
  return 5;
}
/* Decode type 14: GPS time of week ------------------------------------------*/
static int decode_type14(rtcm_t *rtcm) {
  trace(4, "decode_type14: len=%d\n", rtcm->len);

  int i = 48;
  long double zcnt = rtcm_getbitu(rtcm, 24, 13);
  if (i + 24 > rtcm->len * 8) {
    trace(2, "rtcm2 14 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int week = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  int hour = rtcm_getbitu(rtcm, i, 8);
  i += 8;
  int leaps = rtcm_getbitu(rtcm, i, 6);
  week = adjgpsweek(week);
  rtcm->time = gpst2time(week, hour * 3600.0L + zcnt * 0.6L);
  rtcm->nav.utc_gps[4] = leaps;
  return 6;
}
/* Decode type 16: GPS special message ---------------------------------------*/
static int decode_type16(rtcm_t *rtcm) {
  trace(4, "decode_type16: len=%d\n", rtcm->len);

  int i = 48, n = 0;
  while (i + 8 <= rtcm->len * 8 && n < 90) {
    rtcm->msg[n++] = rtcm_getbitu(rtcm, i, 8);
    i += 8;
  }
  rtcm->msg[n] = '\0';

  trace(3, "rtcm2 16 message: %s\n", rtcm->msg);
  return 9;
}
/* Decode type 17: GPS ephemerides -------------------------------------------*/
static int decode_type17(rtcm_t *rtcm) {
  trace(4, "decode_type17: len=%d\n", rtcm->len);

  int i = 48;
  if (i + 480 > rtcm->len * 8) {
    trace(2, "rtcm2 17 length error: len=%d\n", rtcm->len);
    return -1;
  }
  eph_t eph = {0};
  int week = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.idot = rtcm_getbits(rtcm, i, 14) * P2_43 * SC2RAD;
  i += 14;
  eph.iode = rtcm_getbitu(rtcm, i, 8);
  i += 8;
  long double toc = rtcm_getbitu(rtcm, i, 16) * 16.0L;
  i += 16;
  eph.f1 = rtcm_getbits(rtcm, i, 16) * P2_43;
  i += 16;
  eph.f2 = rtcm_getbits(rtcm, i, 8) * P2_55;
  i += 8;
  eph.crs = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.deln = rtcm_getbits(rtcm, i, 16) * P2_43 * SC2RAD;
  i += 16;
  eph.cuc = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.e = rtcm_getbitu(rtcm, i, 32) * P2_33;
  i += 32;
  eph.cus = rtcm_getbits(rtcm, i, 16);
  i += 16;
  long double sqrtA = rtcm_getbitu(rtcm, i, 32) * P2_19;
  i += 32;
  eph.toes = rtcm_getbitu(rtcm, i, 16);
  i += 16;
  eph.OMG0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cic = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.i0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.cis = rtcm_getbits(rtcm, i, 16) * P2_29;
  i += 16;
  eph.omg = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.crc = rtcm_getbits(rtcm, i, 16) * P2_5;
  i += 16;
  eph.OMGd = rtcm_getbits(rtcm, i, 24) * P2_43 * SC2RAD;
  i += 24;
  eph.M0 = rtcm_getbits(rtcm, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph.iodc = rtcm_getbitu(rtcm, i, 10);
  i += 10;
  eph.f0 = rtcm_getbits(rtcm, i, 22) * P2_31;
  i += 22;
  int prn = rtcm_getbitu(rtcm, i, 5);
  i += 5 + 3;
  eph.tgd[0] = rtcm_getbits(rtcm, i, 8) * P2_31;
  i += 8;
  eph.code = rtcm_getbitu(rtcm, i, 2);
  i += 2;
  eph.sva = rtcm_getbitu(rtcm, i, 4);
  i += 4;
  eph.svh = rtcm_getbitu(rtcm, i, 6);
  i += 6;
  eph.flag = rtcm_getbitu(rtcm, i, 1);
  if (prn == 0) prn = 32;
  int sat = satno(SYS_GPS, prn);
  eph.sat = sat;
  eph.week = adjgpsweek(week);
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  rtcm->nav.eph[sat - 1][0] = eph;
  rtcm->ephset = 0;
  rtcm->ephsat = sat;
  return 2;
}
/* Decode type 18: rtk uncorrected carrier-phase -----------------------------*/
static int decode_type18(rtcm_t *rtcm) {
  trace(4, "decode_type18: len=%d\n", rtcm->len);

  int i = 48;
  if (i + 24 > rtcm->len * 8) {
    trace(2, "rtcm2 18 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int freq = rtcm_getbitu(rtcm, i, 2);
  i += 2 + 2;
  long double usec = rtcm_getbitu(rtcm, i, 20);
  i += 20;
  if (freq & 0x1) {
    trace(2, "rtcm2 18 not supported frequency: freq=%d\n", freq);
    return -1;
  }
  freq >>= 1;

  int sync = 1;
  while (i + 48 <= rtcm->len * 8 && rtcm->obs.n < MAXOBS) {
    sync = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int code = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int sys = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int prn = rtcm_getbitu(rtcm, i, 5);
    i += 5 + 3;
    int loss = rtcm_getbitu(rtcm, i, 5);
    i += 5;
    long double cp = rtcm_getbits(rtcm, i, 32);
    i += 32;
    if (prn == 0) prn = 32;
    int sat = satno(sys ? SYS_GLO : SYS_GPS, prn);
    if (!sat) {
      trace(2, "rtcm2 18 satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    gtime_t time = timeadd(rtcm->time, usec * 1E-6L);
    if (sys) time = utc2gpst(time); /* Convert GLONASS time to GPST */

    long double tt = timediff(rtcm->obs.data[0].time, time);
    if (rtcm->obsflag || fabsl(tt) > 1E-9L) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    int index = obsindex(&rtcm->obs, time, sat);
    if (index >= 0) {
      rtcm->obs.data[index].L[freq] = -cp / 256.0L;
      rtcm->obs.data[index].LLI[freq] = rtcm->loss[sat - 1][freq] != loss;
      rtcm->obs.data[index].code[freq] =
          !freq ? (code ? CODE_L1P : CODE_L1C) : (code ? CODE_L2P : CODE_L2C);
      rtcm->loss[sat - 1][freq] = loss;
    }
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 19: rtk uncorrected pseudorange -------------------------------*/
static int decode_type19(rtcm_t *rtcm) {
  trace(4, "decode_type19: len=%d\n", rtcm->len);

  int i = 48;
  if (i + 24 > rtcm->len * 8) {
    trace(2, "rtcm2 19 length error: len=%d\n", rtcm->len);
    return -1;
  }
  int freq = rtcm_getbitu(rtcm, i, 2);
  i += 2 + 2;
  long double usec = rtcm_getbitu(rtcm, i, 20);
  i += 20;
  if (freq & 0x1) {
    trace(2, "rtcm2 19 not supported frequency: freq=%d\n", freq);
    return -1;
  }
  freq >>= 1;

  int sync = 1;
  while (i + 48 <= rtcm->len * 8 && rtcm->obs.n < MAXOBS) {
    sync = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int code = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int sys = rtcm_getbitu(rtcm, i, 1);
    i += 1;
    int prn = rtcm_getbitu(rtcm, i, 5);
    i += 5 + 8;
    long double pr = rtcm_getbitu(rtcm, i, 32);
    i += 32;
    if (prn == 0) prn = 32;
    int sat = satno(sys ? SYS_GLO : SYS_GPS, prn);
    if (!sat) {
      trace(2, "rtcm2 19 satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    gtime_t time = timeadd(rtcm->time, usec * 1E-6L);
    if (sys) time = utc2gpst(time); /* Convert GLONASS time to GPST */

    long double tt = timediff(rtcm->obs.data[0].time, time);
    if (rtcm->obsflag || fabsl(tt) > 1E-9L) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    int index = obsindex(&rtcm->obs, time, sat);
    if (index >= 0) {
      rtcm->obs.data[index].P[freq] = pr * 0.02L;
      rtcm->obs.data[index].code[freq] =
          !freq ? (code ? CODE_L1P : CODE_L1C) : (code ? CODE_L2P : CODE_L2C);
    }
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* Decode type 22: extended reference station parameter ----------------------*/
static int decode_type22(rtcm_t *rtcm) {
  trace(4, "decode_type22: len=%d\n", rtcm->len);

  int i = 48;
  if (i + 24 > rtcm->len * 8) {
    trace(2, "rtcm2 22 length error: len=%d\n", rtcm->len);
    return -1;
  }
  long double del[2][3] = {{0}};
  del[0][0] = rtcm_getbits(rtcm, i, 8) / 25600.0L;
  i += 8;
  del[0][1] = rtcm_getbits(rtcm, i, 8) / 25600.0L;
  i += 8;
  del[0][2] = rtcm_getbits(rtcm, i, 8) / 25600.0L;
  i += 8;
  long double hgt = 0.0L;
  if (i + 24 <= rtcm->len * 8) {
    i += 5;
    int noh = rtcm_getbits(rtcm, i, 1);
    i += 1;
    hgt = noh ? 0.0L : rtcm_getbitu(rtcm, i, 18) / 25600.0L;
    i += 18;
  }
  /* TODO should this path be taken if the above was not read? */
  if (i + 24 <= rtcm->len * 8) {
    del[1][0] = rtcm_getbits(rtcm, i, 8) / 1600.0L;
    i += 8;
    del[1][1] = rtcm_getbits(rtcm, i, 8) / 1600.0L;
    i += 8;
    del[1][2] = rtcm_getbits(rtcm, i, 8) / 1600.0L;
  }
  rtcm->sta.deltype = 1; /* XYZ */
  for (int j = 0; j < 3; j++) rtcm->sta.del[j] = del[0][j];
  rtcm->sta.hgt = hgt;
  return 5;
}
/* Decode type 23: antenna type definition record ----------------------------*/
static int decode_type23(rtcm_t *rtcm) { return 0; }
/* Decode type 24: antenna reference point (arp) -----------------------------*/
static int decode_type24(rtcm_t *rtcm) { return 0; }
/* Decode type 31: differential GLONASS correction ---------------------------*/
static int decode_type31(rtcm_t *rtcm) { return 0; }
/* Decode type 32: differential GLONASS reference station parameters ---------*/
static int decode_type32(rtcm_t *rtcm) { return 0; }
/* Decode type 34: GLONASS partial differential correction set ---------------*/
static int decode_type34(rtcm_t *rtcm) { return 0; }
/* Decode type 36: GLONASS special message -----------------------------------*/
static int decode_type36(rtcm_t *rtcm) { return 0; }
/* Decode type 37: GNSS system time offset -----------------------------------*/
static int decode_type37(rtcm_t *rtcm) { return 0; }
/* Decode type 59: proprietary message ---------------------------------------*/
static int decode_type59(rtcm_t *rtcm) { return 0; }
/* Decode RTCM ver.2 message -------------------------------------------------*/
extern int decode_rtcm2(rtcm_t *rtcm) {
  int type = rtcm_getbitu(rtcm, 8, 6);
  trace(3, "decode_rtcm2: type=%2d len=%3d\n", type, rtcm->len);

  long double zcnt = rtcm_getbitu(rtcm, 24, 13) * 0.6L;
  if (zcnt >= 3600.0L) {
    trace(2, "rtcm2 modified z-count error: zcnt=%.1Lf\n", zcnt);
    return -1;
  }
  adjhour(rtcm, zcnt);
  int staid = rtcm_getbitu(rtcm, 14, 10);
  int seqno = rtcm_getbitu(rtcm, 37, 3);
  int stah = rtcm_getbitu(rtcm, 45, 3);
  if (seqno - rtcm->seqno != 1 && seqno - rtcm->seqno != -7) {
    trace(2, "rtcm2 message outage: seqno=%d->%d\n", rtcm->seqno, seqno);
  }
  rtcm->seqno = seqno;
  rtcm->stah = stah;

  if (rtcm->outtype) {
    rtksnprintf(rtcm->msgtype, sizeof(rtcm->msgtype),
                "RTCM %2d (%4d) zcnt=%7.1Lf staid=%3d seqno=%d", type, rtcm->len, zcnt, staid,
                seqno);
  }
  if (type == 3 || type == 22 || type == 23 || type == 24) {
    if (rtcm->staid != 0 && staid != rtcm->staid) {
      trace(2, "rtcm2 station id changed: %d->%d\n", rtcm->staid, staid);
    }
    rtcm->staid = staid;
  }
  if (rtcm->staid != 0 && staid != rtcm->staid) {
    trace(2, "rtcm2 station id invalid: %d %d\n", staid, rtcm->staid);
    return -1;
  }
  int ret = 0;
  switch (type) {
    case 1:
      ret = decode_type1(rtcm);
      break;
    case 3:
      ret = decode_type3(rtcm);
      break;
    case 9:
      ret = decode_type1(rtcm);
      break;
    case 14:
      ret = decode_type14(rtcm);
      break;
    case 16:
      ret = decode_type16(rtcm);
      break;
    case 17:
      ret = decode_type17(rtcm);
      break;
    case 18:
      ret = decode_type18(rtcm);
      break;
    case 19:
      ret = decode_type19(rtcm);
      break;
    case 22:
      ret = decode_type22(rtcm);
      break;
    case 23:
      ret = decode_type23(rtcm);
      break; /* Not supported */
    case 24:
      ret = decode_type24(rtcm);
      break; /* Not supported */
    case 31:
      ret = decode_type31(rtcm);
      break; /* Not supported */
    case 32:
      ret = decode_type32(rtcm);
      break; /* Not supported */
    case 34:
      ret = decode_type34(rtcm);
      break; /* Not supported */
    case 36:
      ret = decode_type36(rtcm);
      break; /* Not supported */
    case 37:
      ret = decode_type37(rtcm);
      break; /* Not supported */
    case 59:
      ret = decode_type59(rtcm);
      break; /* Not supported */
  }
  if (ret >= 0) {
    if (1 <= type && type <= 99)
      rtcm->nmsg2[type]++;
    else
      rtcm->nmsg2[0]++;
  }
  return ret;
}
