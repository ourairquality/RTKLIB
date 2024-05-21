/*------------------------------------------------------------------------------
 * nvs.c : NVS receiver dependent functions
 *
 *    Copyright (C) 2012-2016 by M.BAVARO and T.TAKASU, All rights reserved.
 *    Copyright (C) 2014-2020 by T.TAKASU, All rights reserved.
 *
 *     [1] Description of BINR messages which is used by RC program for RINEX
 *         files accumulation, NVS
 *     [2] NAVIS Navis Standard Interface Protocol BINR, NVS
 *
 * Version : $Revision:$ $Date:$
 * History : 2012/01/30 1.0  first version by M.BAVARO
 *           2012/11/08 1.1  modified by T.TAKASU
 *           2013/02/23 1.2  fix memory access violation problem on arm
 *           2013/04/24 1.3  fix bug on cycle-slip detection
 *                           add range check of GPS ephemeris week
 *           2013/09/01 1.4  add check error of week, time jump, obs data range
 *           2014/08/26 1.5  fix bug on iode in GLONASS ephemeris
 *           2016/01/26 1.6  fix bug on unrecognized meas data (#130)
 *           2017/04/11 1.7  (char *) -> (signed char *)
 *           2020/07/10 1.8  suppress warnings
 *           2020/11/30 1.9  use integer type in stdint.h
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include "rtklib.h"

#define NVSSYNC 0x10   /* Nvs message sync code 1 */
#define NVSENDMSG 0x03 /* Nvs message sync code 1 */
#define NVSCFG 0x06    /* Nvs message cfg-??? */

#define ID_XF5RAW 0xf5  /* Nvs msg id: raw measurement data */
#define ID_X4AIONO 0x4a /* Nvs msg id: GPS ionospheric data */
#define ID_X4BTIME 0x4b /* Nvs msg id: GPS/GLONASS/UTC timescale data */
#define ID_XF7EPH 0xf7  /* Nvs msg id: subframe buffer */
#define ID_XE5BIT 0xe5  /* Nvs msg id: bit information */

#define ID_XD7ADVANCED 0xd7 /* */
#define ID_X02RATEPVT 0x02  /* */
#define ID_XF4RATERAW 0xf4  /* */
#define ID_XD7SMOOTH 0xd7   /* */
#define ID_XD5BIT 0xd5      /* */

/* Get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
static uint16_t U2(uint8_t *p) {
  uint16_t u;
  memcpy(&u, p, 2);
  return u;
}
static uint32_t U4(uint8_t *p) {
  uint32_t u;
  memcpy(&u, p, 4);
  return u;
}
static int16_t I2(uint8_t *p) {
  int16_t i;
  memcpy(&i, p, 2);
  return i;
}
static int32_t I4(uint8_t *p) {
  int32_t i;
  memcpy(&i, p, 4);
  return i;
}
static float R4(uint8_t *p) {
  float r;
  memcpy(&r, p, 4);
  return r;
}
static double R8(uint8_t *p) {
  double r;
  memcpy(&r, p, 8);
  return r;
}

/* URA values (ref [3] 20.3.3.3.1.1) -----------------------------------------*/
static const long double ura_eph[] = {2.4L,    3.4L,    4.85L,   6.85L,  9.65L,  13.65L,
                                      24.0L,   48.0L,   96.0L,   192.0L, 384.0L, 768.0L,
                                      1536.0L, 3072.0L, 6144.0L, 0.0L};
/* URA value (m) to URA index ------------------------------------------------*/
static int uraindex(long double value) {
  int i;
  for (i = 0; i < 15; i++)
    if (ura_eph[i] >= value) break;
  return i;
}
/* Decode NVS xf5-raw: raw measurement data ----------------------------------*/
static int decode_xf5raw(raw_t *raw) {
  gtime_t time;
  long double tadj = 0.0L, toff = 0.0L, tn;
  int dTowInt;
  long double dTowUTC, dTowGPS, dTowFrac, L1, P1, D1;
  long double gpsutcTimescale;
  uint8_t rcvTimeScaleCorr, sys, carrNo;
  int i, j, prn, sat, n = 0, nsat, week;
  uint8_t *p = raw->buff + 2;
  char *q, tstr[40], flag;

  trace(4, "decode_xf5raw: len=%d\n", raw->len);

  /* Time tag adjustment option (-TADJ) */
  if ((q = strstr(raw->opt, "-tadj"))) {
    sscanf(q, "-TADJ=%Lf", &tadj);
  }
  dTowUTC = (long double)R8(p);
  week = U2(p + 8);
  gpsutcTimescale = (long double)R8(p + 10);
  /* GlonassutcTimescale = (long double)R8(p+18); */
  rcvTimeScaleCorr = I1(p + 26);

  /* Check GPS week range */
  if (week >= 4096) {
    trace(2, "nvs xf5raw obs week error: week=%d\n", week);
    return -1;
  }
  week = adjgpsweek(week);

  if ((raw->len - 31) % 30) {
    /* Message length is not correct: there could be an error in the stream */
    trace(2, "nvs xf5raw len=%d seems not be correct\n", raw->len);
    return -1;
  }
  nsat = (raw->len - 31) / 30;

  dTowGPS = dTowUTC + gpsutcTimescale;

  /* Tweak pseudoranges to allow RINEX to represent the NVS time of measure */
  dTowInt = 10.0L * floorl((dTowGPS / 10.0L) + 0.5L);
  dTowFrac = dTowGPS - (long double)dTowInt;
  time = gpst2time(week, dTowInt * 0.001L);

  /* Time tag adjustment */
  if (tadj > 0.0L) {
    tn = time2gpst(time, &week) / tadj;
    toff = (tn - floorl(tn + 0.5L)) * tadj;
    time = timeadd(time, -toff);
  }
  /* Check time tag jump and output warning */
  if (raw->time.time && fabsl(timediff(time, raw->time)) > 86400.0L) {
    time2str(time, tstr, 3);
    trace(2, "nvs xf5raw time tag jump warning: time=%s\n", tstr);
  }
  if (fabsl(timediff(time, raw->time)) <= 1e-3L) {
    time2str(time, tstr, 3);
    trace(2, "nvs xf5raw time tag duplicated: time=%s\n", tstr);
    return 0;
  }
  for (i = 0, p += 27; (i < nsat) && (n < MAXOBS); i++, p += 30) {
    raw->obs.data[n].time = time;
    sys = (U1(p) == 1) ? SYS_GLO : ((U1(p) == 2) ? SYS_GPS : ((U1(p) == 4) ? SYS_SBS : SYS_NONE));
    prn = U1(p + 1);
    if (sys == SYS_SBS) prn += 120; /* Correct this */
    if (!(sat = satno(sys, prn))) {
      trace(2, "nvs xf5raw satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    carrNo = I1(p + 2);
    L1 = (long double)R8(p + 4);
    P1 = (long double)R8(p + 12);
    D1 = (long double)R8(p + 20);

    /* Check range error */
    if (L1 < -1E10L || L1 > 1E10L || P1 < -1E10L || P1 > 1E10L || D1 < -1E5L || D1 > 1E5L) {
      trace(2, "nvs xf5raw obs range error: sat=%2d L1=%12.5e P1=%12.5e D1=%12.5e\n", sat, L1, P1,
            D1);
      continue;
    }
    raw->obs.data[n].SNR[0] = (uint16_t)(I1(p + 3) / SNR_UNIT + 0.5L);
    if (sys == SYS_GLO) {
      raw->obs.data[n].L[0] = L1 - toff * (FREQ1_GLO + DFRQ1_GLO * carrNo);
    } else {
      raw->obs.data[n].L[0] = L1 - toff * FREQL1;
    }
    raw->obs.data[n].P[0] =
        (P1 - dTowFrac) * CLIGHT * 0.001L - toff * CLIGHT; /* In ms, needs to be converted */
    raw->obs.data[n].D[0] = D1;

    /* Set LLI if meas flag 4 (carrier phase present) off -> on */
    flag = U1(p + 28);
    raw->obs.data[n].LLI[0] = (flag & 0x08) && !(raw->halfc[sat - 1][0] & 0x08) ? 1 : 0;
    raw->halfc[sat - 1][0] = flag;

    raw->obs.data[n].code[0] = CODE_L1C;
    raw->obs.data[n].sat = sat;

    for (j = 1; j < NFREQ + NEXOBS; j++) {
      raw->obs.data[n].L[j] = raw->obs.data[n].P[j] = 0.0L;
      raw->obs.data[n].D[j] = 0.0L;
      raw->obs.data[n].SNR[j] = raw->obs.data[n].LLI[j] = 0;
      raw->obs.data[n].code[j] = CODE_NONE;
    }
    n++;
  }
  raw->time = time;
  raw->obs.n = n;
  return 1;
}
/* Decode ephemeris ----------------------------------------------------------*/
static int decode_gpsephem(int sat, raw_t *raw) {
  eph_t eph = {0};
  uint8_t *puiTmp = (raw->buff) + 2;
  uint16_t week;
  long double toc;

  trace(4, "decode_ephem: sat=%2d\n", sat);

  eph.crs = (long double)R4(&puiTmp[2]);
  eph.deln = (long double)R4(&puiTmp[6]) * 1e+3L;
  eph.M0 = (long double)R8(&puiTmp[10]);
  eph.cuc = (long double)R4(&puiTmp[18]);
  eph.e = (long double)R8(&puiTmp[22]);
  eph.cus = (long double)R4(&puiTmp[30]);
  eph.A = powl((long double)R8(&puiTmp[34]), 2);
  eph.toes = (long double)R8(&puiTmp[42]) * 1e-3L;
  eph.cic = (long double)R4(&puiTmp[50]);
  eph.OMG0 = (long double)R8(&puiTmp[54]);
  eph.cis = (long double)R4(&puiTmp[62]);
  eph.i0 = (long double)R8(&puiTmp[66]);
  eph.crc = (long double)R4(&puiTmp[74]);
  eph.omg = (long double)R8(&puiTmp[78]);
  eph.OMGd = (long double)R8(&puiTmp[86]) * 1e+3L;
  eph.idot = (long double)R8(&puiTmp[94]) * 1e+3L;
  eph.tgd[0] = (long double)R4(&puiTmp[102]) * 1e-3L;
  toc = (long double)R8(&puiTmp[106]) * 1e-3L;
  eph.f2 = (long double)R4(&puiTmp[114]) * 1e+3L;
  eph.f1 = (long double)R4(&puiTmp[118]);
  eph.f0 = (long double)R4(&puiTmp[122]) * 1e-3L;
  eph.sva = uraindex(I2(&puiTmp[126]));
  eph.iode = I2(&puiTmp[128]);
  eph.iodc = I2(&puiTmp[130]);
  eph.code = I2(&puiTmp[132]);
  eph.flag = I2(&puiTmp[134]);
  week = I2(&puiTmp[136]);
  eph.fit = 0;

  if (week >= 4096) {
    trace(2, "nvs gps ephemeris week error: sat=%2d week=%d\n", sat, week);
    return -1;
  }
  eph.week = adjgpsweek(week);
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = raw->time;

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1][0].iode) return 0; /* Unchanged */
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1][0] = eph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}
/* Adjust daily rollover of time ---------------------------------------------*/
static gtime_t adjday(gtime_t time, long double tod) {
  long double ep[6], tod_p;
  time2epoch(time, ep);
  tod_p = ep[3] * 3600.0L + ep[4] * 60.0L + ep[5];
  if (tod < tod_p - 43200.0L)
    tod += 86400.0L;
  else if (tod > tod_p + 43200.0L)
    tod -= 86400.0L;
  ep[3] = ep[4] = ep[5] = 0.0L;
  return timeadd(epoch2time(ep), tod);
}
/* Decode gloephem -----------------------------------------------------------*/
static int decode_gloephem(int sat, raw_t *raw) {
  geph_t geph = {0};
  uint8_t *p = (raw->buff) + 2;
  int prn, tk, tb;

  if (raw->len >= 93) {
    prn = I1(p + 1);
    geph.frq = I1(p + 2);
    geph.pos[0] = (long double)R8(p + 3);
    geph.pos[1] = (long double)R8(p + 11);
    geph.pos[2] = (long double)R8(p + 19);
    geph.vel[0] = (long double)R8(p + 27) * 1e+3L;
    geph.vel[1] = (long double)R8(p + 35) * 1e+3L;
    geph.vel[2] = (long double)R8(p + 43) * 1e+3L;
    geph.acc[0] = (long double)R8(p + 51) * 1e+6L;
    geph.acc[1] = (long double)R8(p + 59) * 1e+6L;
    geph.acc[2] = (long double)R8(p + 67) * 1e+6L;
    tb = (long double)R8(p + 75) * 1e-3L;
    tk = tb;
    geph.gamn = (long double)R4(p + 83);
    geph.taun = (long double)R4(p + 87) * 1e-3L;
    geph.age = I2(p + 91);
  } else {
    trace(2, "nvs NE length error: len=%d\n", raw->len);
    return -1;
  }
  if (!(geph.sat = satno(SYS_GLO, prn))) {
    trace(2, "nvs NE satellite error: prn=%d\n", prn);
    return -1;
  }
  if (raw->time.time == 0) return 0;

  geph.iode = (tb / 900) & 0x7F;
  geph.toe = utc2gpst(adjday(raw->time, tb - 10800.0L));
  geph.tof = utc2gpst(adjday(raw->time, tk - 10800.0L));
#ifdef RTK_DISABLED
  /* Check illegal ephemeris by toe */
  long double tt = timediff(raw->time, geph.toe);
  if (fabsl(tt) > 3600.0) {
    trace(3, "nvs NE illegal toe: prn=%2d tt=%6.0Lf\n", prn, tt);
    return 0;
  }
#endif
#ifdef RTK_DISABLED
  /* Check illegal ephemeris by frequency number consistency */
  if (raw->nav.geph[prn - MINPRNGLO][0].toe.time &&
      geph.frq != raw->nav.geph[prn - MINPRNGLO][0].frq) {
    trace(2, "nvs NE illegal freq change: prn=%2d frq=%2d->%2d\n", prn,
          raw->nav.geph[prn - MINPRNGLO][0].frq, geph.frq);
    return -1;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (fabsl(timediff(geph.toe, raw->nav.geph[prn - MINPRNGLO][0].toe)) < 1.0L &&
        geph.svh == raw->nav.geph[prn - MINPRNGLO][0].svh)
      return 0;
  }
#endif
  raw->nav.geph[prn - 1][0] = geph;
  raw->ephsat = geph.sat;
  raw->ephset = 0;

  return 2;
}
/* Decode NVS ephemerides in clear -------------------------------------------*/
static int decode_xf7eph(raw_t *raw) {
  int prn, sat, sys;
  uint8_t *p = raw->buff;

  trace(4, "decode_xf7eph: len=%d\n", raw->len);

  if ((raw->len) < 93) {
    trace(2, "nvs xf7eph length error: len=%d\n", raw->len);
    return -1;
  }
  sys = (U1(p + 2) == 1) ? SYS_GPS : ((U1(p + 2) == 2) ? SYS_GLO : SYS_NONE);
  prn = U1(p + 3);
  if (!(sat = satno(sys == 1 ? SYS_GPS : SYS_GLO, prn))) {
    trace(2, "nvs xf7eph satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (sys == SYS_GPS) {
    return decode_gpsephem(sat, raw);
  } else if (sys == SYS_GLO) {
    return decode_gloephem(sat, raw);
  }
  return 0;
}
/* Decode NVS rxm-sfrb: subframe buffer --------------------------------------*/
static int decode_xe5bit(raw_t *raw) {
  int prn;
  int iBlkStartIdx, iExpLen, iIdx;
  uint32_t words[10];
  uint8_t uiDataBlocks, uiDataType;
  uint8_t *p = raw->buff;

  trace(4, "decode_xe5bit: len=%d\n", raw->len);

  p += 2; /* Discard preamble and message identifier */
  uiDataBlocks = U1(p);

  if (uiDataBlocks >= 16) {
    trace(2, "nvs xf5bit message error: data blocks %u\n", uiDataBlocks);
    return -1;
  }
  iBlkStartIdx = 1;
  for (iIdx = 0; iIdx < uiDataBlocks; iIdx++) {
    iExpLen = (iBlkStartIdx + 10);
    if ((raw->len) < iExpLen) {
      trace(2, "nvs xf5bit message too short (expected at least %d)\n", iExpLen);
      return -1;
    }
    uiDataType = U1(p + iBlkStartIdx + 1);

    switch (uiDataType) {
      case 1: /* GLONASS */
        iBlkStartIdx += 19;
        break;
      case 2: /* GPS */
        iBlkStartIdx += 47;
        break;
      case 4: /* SBAS */
        prn = U1(p + (iBlkStartIdx + 2)) + 120;

        /* Sat = satno(SYS_SBS, prn); */
        /* Sys = satsys(sat,&prn); */
        memset(words, 0, 10 * sizeof(uint32_t));
        for (iIdx = 0, iBlkStartIdx += 7; iIdx < 10; iIdx++, iBlkStartIdx += 4) {
          words[iIdx] = U4(p + iBlkStartIdx);
        }
        words[7] >>= 6;
        return sbsdecodemsg(raw->time, prn, words, &raw->sbsmsg) ? 3 : 0;
      default:
        trace(2, "nvs xf5bit SNS type unknown (got %d)\n", uiDataType);
        return -1;
    }
  }
  return 0;
}
/* Decode NVS x4aiono --------------------------------------------------------*/
static int decode_x4aiono(raw_t *raw) {
  uint8_t *p = raw->buff + 2;

  trace(4, "decode_x4aiono: len=%d\n", raw->len);

  raw->nav.ion_gps[0] = (long double)R4(p);
  raw->nav.ion_gps[1] = (long double)R4(p + 4);
  raw->nav.ion_gps[2] = (long double)R4(p + 8);
  raw->nav.ion_gps[3] = (long double)R4(p + 12);
  raw->nav.ion_gps[4] = (long double)R4(p + 16);
  raw->nav.ion_gps[5] = (long double)R4(p + 20);
  raw->nav.ion_gps[6] = (long double)R4(p + 24);
  raw->nav.ion_gps[7] = (long double)R4(p + 28);

  return 9;
}
/* Decode NVS x4btime --------------------------------------------------------*/
static int decode_x4btime(raw_t *raw) {
  uint8_t *p = raw->buff + 2;

  trace(4, "decode_x4btime: len=%d\n", raw->len);

  raw->nav.utc_gps[1] = (long double)R8(p);
  raw->nav.utc_gps[0] = (long double)R8(p + 8);
  raw->nav.utc_gps[2] = I4(p + 16);
  raw->nav.utc_gps[3] = I2(p + 20);
  raw->nav.utc_gps[4] = I1(p + 22);

  return 9;
}
/* Decode NVS raw message ----------------------------------------------------*/
static int decode_nvs(raw_t *raw) {
  int type = U1(raw->buff + 1);

  trace(3, "decode_nvs: type=%02x len=%d\n", type, raw->len);

  rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "NVS: type=%2d len=%3d", type, raw->len);

  switch (type) {
    case ID_XF5RAW:
      return decode_xf5raw(raw);
    case ID_XF7EPH:
      return decode_xf7eph(raw);
    case ID_XE5BIT:
      return decode_xe5bit(raw);
    case ID_X4AIONO:
      return decode_x4aiono(raw);
    case ID_X4BTIME:
      return decode_x4btime(raw);
    default:
      break;
  }
  return 0;
}
/* Input NVS raw message from stream -------------------------------------------
 * Fetch next NVS raw data and input a message from stream
 * Args   : raw_t *raw   IO    receiver raw data control struct
 *          uint8_t data I     stream data (1 byte)
 * Return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 3: input SBAS message,
 *                  9: input ion/UTC parameter)
 *
 * Notes  : to specify input options, set raw->opt to the following option
 *          strings separated by spaces.
 *
 *          -EPHALL    : input all ephemerides
 *          -TADJ=tint : adjust time tags to multiples of tint (sec)
 *
 *----------------------------------------------------------------------------*/
extern int input_nvs(raw_t *raw, uint8_t data) {
  trace(5, "input_nvs: data=%02x\n", data);

  /* Synchronize frame */
  if ((raw->nbyte == 0) && (data == NVSSYNC)) {
    /* Search a 0x10 */
    raw->buff[0] = data;
    raw->nbyte = 1;
    return 0;
  }
  if ((raw->nbyte == 1) && (data != NVSSYNC) && (data != NVSENDMSG)) {
    /* Discard long double 0x10 and 0x10 0x03 at beginning of frame */
    raw->buff[1] = data;
    raw->nbyte = 2;
    raw->flag = 0;
    return 0;
  }
  /* This is all done to discard a long double 0x10 */
  if (data == NVSSYNC) raw->flag = (raw->flag + 1) % 2;
  if ((data != NVSSYNC) || (raw->flag)) {
    /* Store the new byte */
    raw->buff[(raw->nbyte++)] = data;
  }
  /* Detect ending sequence */
  if ((data == NVSENDMSG) && (raw->flag)) {
    raw->len = raw->nbyte;
    raw->nbyte = 0;

    /* Decode NVS raw message */
    return decode_nvs(raw);
  }
  if (raw->nbyte == MAXRAWLEN) {
    trace(2, "nvs message size error: len=%d\n", raw->nbyte);
    raw->nbyte = 0;
    return -1;
  }
  return 0;
}
/* Input NVS raw message from file ---------------------------------------------
 * Fetch next NVS raw data and input a message from file
 * Args   : raw_t  *raw  IO    receiver raw data control struct
 *          FILE   *fp   I     file pointer
 * Return : status(-2: end of file, -1...9: same as above)
 *----------------------------------------------------------------------------*/
extern int input_nvsf(raw_t *raw, FILE *fp) {
  int i, data, odd = 0;

  trace(4, "input_nvsf:\n");

  /* Synchronize frame */
  for (i = 0;; i++) {
    if ((data = fgetc(fp)) == EOF) return -2;

    /* Search a 0x10 */
    if (data == NVSSYNC) {
      /* Store the frame begin */
      raw->buff[0] = data;
      if ((data = fgetc(fp)) == EOF) return -2;

      /* Discard long double 0x10 and 0x10 0x03 */
      if ((data != NVSSYNC) && (data != NVSENDMSG)) {
        raw->buff[1] = data;
        break;
      }
    }
    if (i >= 4096) return 0;
  }
  raw->nbyte = 2;
  for (i = 0;; i++) {
    if ((data = fgetc(fp)) == EOF) return -2;
    if (data == NVSSYNC) odd = (odd + 1) % 2;
    if ((data != NVSSYNC) || odd) {
      /* Store the new byte */
      raw->buff[(raw->nbyte++)] = data;
    }
    /* Detect ending sequence */
    if ((data == NVSENDMSG) && odd) break;
    if (i >= 4096) return 0;
  }
  raw->len = raw->nbyte;
  if ((raw->len) > MAXRAWLEN) {
    trace(2, "nvs length error: len=%d\n", raw->len);
    return -1;
  }
  /* Decode nvs raw message */
  return decode_nvs(raw);
}
/* Generate NVS binary message -------------------------------------------------
 * Generate NVS binary message from message string
 * Args   : char  *msg   I      message string
 *            "RESTART  [arg...]" system reset
 *            "CFG-SERI [arg...]" configure serial port property
 *            "CFG-FMT  [arg...]" configure output message format
 *            "CFG-RATE [arg...]" configure binary measurement output rates
 *          uint8_t *buff O binary message
 * Return : length of binary message (0: error)
 * Note   : see reference [1][2] for details.
 *----------------------------------------------------------------------------*/
extern int gen_nvs(const char *msg, uint8_t *buff, size_t size) {
  /* TODO respect the buff size */
  char mbuff[1024], *args[32], *p;
  uint32_t byte;
  int iRate, n, narg = 0;
  uint8_t ui100Ms;

  trace(4, "gen_nvs: msg=%s\n", msg);

  rtkstrcpy(mbuff, sizeof(mbuff), msg);
  char *r;
  for (p = strtok_r(mbuff, " ", &r); p && narg < 32; p = strtok_r(NULL, " ", &r)) {
    args[narg++] = p;
  }
  if (narg < 1) {
    return 0;
  }
  size_t len = 0;
  buff[len++] = NVSSYNC; /* DLE */

  if (!strcmp(args[0], "CFG-PVTRATE")) {
    buff[len++] = ID_XD7ADVANCED;
    buff[len++] = ID_X02RATEPVT;
    if (narg > 1) {
      iRate = atoi(args[1]);
      buff[len++] = (uint8_t)iRate;
    }
  } else if (!strcmp(args[0], "CFG-RAWRATE")) {
    buff[len++] = ID_XF4RATERAW;
    if (narg > 1) {
      iRate = atoi(args[1]);
      switch (iRate) {
        case 2:
          ui100Ms = 5;
          break;
        case 5:
          ui100Ms = 2;
          break;
        case 10:
          ui100Ms = 1;
          break;
        default:
          ui100Ms = 10;
          break;
      }
      buff[len++] = ui100Ms;
    }
  } else if (!strcmp(args[0], "CFG-SMOOTH")) {
    buff[len++] = ID_XD7SMOOTH;
    buff[len++] = 0x03;
    buff[len++] = 0x01;
    buff[len++] = 0x00;
  } else if (!strcmp(args[0], "CFG-BINR")) {
    for (n = 1; (n < narg); n++) {
      if (sscanf(args[n], "%2x", &byte)) buff[len++] = (uint8_t)byte;
    }
  } else
    return 0;

  buff[len++] = 0x10; /* ETX */
  buff[len] = 0x03;   /* DLE */
  return (int)len;
}
