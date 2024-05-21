/*------------------------------------------------------------------------------
 * skytraq.c : skytraq receiver dependent functions
 *
 *          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
 *
 * Reference :
 *     [1] Skytraq, Application Note AN0023 Binary Message of SkyTraq Venus 6
 *         GPS Receiver, ver 1.4.8, August 21, 2008
 *     [2] Skytraq, Application Note AN0024 Raw Measurement Binary Message
 *         Extension of SkyTraq Venus 6 GPS Receiver, ver 0.5, October 9, 2009
 *     [3] Skytraq, Application Note AN0024G2 Binary Message of SkyTraq Venus 7
 *         GLONASS/GPS Receiver (Raw Measurement F/W), ver 1.4.26, April 26, 2012
 *     [4] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
 *         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.29,
 *         April 3, 2014
 *     [5] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
 *         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.31,
 *         August 12, 2014
 *     [6] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
 *         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.32,
 *         Sep 26, 2016
 *     [7] Skytraq, Application Note AN0039 Binary Messages of Raw Measurement
 *         Data Extension of SkyTraq Phoneix GNSS Receiver, ver.1.4.39,
 *         Dec 30, 2020
 *
 * Notes   :
 *     The byte order of S1315F raw message is big-endian inconsistent to [1].
 *
 * Version : $Revision:$
 * History : 2009/10/10 1.0 new
 *           2009/11/08 1.1 flip carrier-phase polarity for F/W 1.8.23-20091106
 *           2011/05/27 1.2 add almanac decoding
 *                          fix problem with ARM compiler
 *           2011/07/01 1.3 suppress warning
 *           2013/03/10 1.5 change option -invcp to -INVCP
 *           2014/11/09 1.6 support GLONASS, QZSS and BeiDou
 *           2016/10/09 1.7 support F/W version specified as ref [6]
 *           2017/04/11 1.8 (char *) -> (signed char *)
 *           2017/05/08 1.9 fix bug on decoding extended raw meas v.1 (0xE5)
 *                          fix bug on encoding CFG-BIN message (0x1E)
 *                          add decode of ack/nack to request msg (0x83/0x84)
 *           2020/10/30 1.10 add adjustment of GPS week by cpu time
 *                           CODE_L1I -> CODE_L2I for BDS
 *                           use integer type in stdint.h
 *                           suppress warnings
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include "rtklib.h"

#define STQSYNC1 0xA0 /* Skytraq binary sync code 1 */
#define STQSYNC2 0xA1 /* Skytraq binary sync code 2 */

#define ID_STQTIME 0xDC  /* Skytraq message id: measurement epoch */
#define ID_STQRAW 0xDD   /* Skytraq message id: raw measurement */
#define ID_STQSVCH 0xDE  /* Skytraq message id: SV and channel status */
#define ID_STQSTAT 0xDF  /* Skytraq message id: navigation status */
#define ID_STQGPS 0xE0   /* Skytraq message id: GPS/QZS subframe */
#define ID_STQGLO 0xE1   /* Skytraq message id: GLONASS string */
#define ID_STQBDSD1 0xE2 /* Skytraq message id: BeiDou d1 subframe */
#define ID_STQBDSD2 0xE3 /* Skytraq message id: BeiDou d2 subframe */
#define ID_STQRAWX 0xE5  /* Skytraq message id: extended raw meas v.1 */
#define ID_STQGENE 0xE6  /* Skytraq message id: general subframe data */
#define ID_STQGLOE 0x5C  /* Skytraq message id: GLONASS ephemeris */
#define ID_STQACK 0x83   /* Skytraq message id: ack to request msg */
#define ID_STQNACK 0x84  /* Skytraq message id: nack to request msg */

#define ID_RESTART 0x01   /* Skytraq message id: system restart */
#define ID_CFGSERI 0x05   /* Skytraq message id: configure serial port */
#define ID_CFGFMT 0x09    /* Skytraq message id: configure message format */
#define ID_CFGRATE 0x12   /* Skytraq message id: configure message rate */
#define ID_CFGBIN 0x1E    /* Skytraq message id: configure binary message */
#define ID_GETGLOEPH 0x5B /* Skytraq message id: get GLONASS ephemeris */

/* Extract field (big-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))

static uint16_t U2(uint8_t *p) {
  uint16_t value;
  uint8_t *q = (uint8_t *)&value + 1;
  int i;
  for (i = 0; i < 2; i++) *q-- = *p++;
  return value;
}
static uint32_t U4(uint8_t *p) {
  uint32_t value;
  uint8_t *q = (uint8_t *)&value + 3;
  int i;
  for (i = 0; i < 4; i++) *q-- = *p++;
  return value;
}
static float R4(uint8_t *p) {
  float value;
  uint8_t *q = (uint8_t *)&value + 3;
  int i;
  for (i = 0; i < 4; i++) *q-- = *p++;
  return value;
}
static double R8(uint8_t *p) {
  double value;
  uint8_t *q = (uint8_t *)&value + 7;
  int i;
  for (i = 0; i < 8; i++) *q-- = *p++;
  return value;
}
/* Checksum ------------------------------------------------------------------*/
static uint8_t checksum(uint8_t *buff, int size) {
  uint8_t cs = 0;
  for (int i = 4; i < size - 3; i++) {
    cs ^= buff[i];
  }
  return cs;
}
/* 8-bit week -> full week ---------------------------------------------------*/
static void adj_utcweek(gtime_t time, long double *utc) {
  int week;

  if (utc[3] >= 256.0L) return;
  time2gpst(time, &week);
  utc[3] += week / 256 * 256;
  if (utc[3] < week - 128)
    utc[3] += 256.0L;
  else if (utc[3] > week + 128)
    utc[3] -= 256.0L;
}
/* GNSSId to system  ---------------------------------------------------------*/
static int sky_sys(int gnssid) {
  switch (gnssid) {
    case 0:
      return SYS_GPS;
    case 1:
      return SYS_SBS;
    case 2:
      return SYS_GLO;
    case 3:
      return SYS_GAL;
    case 4:
      return SYS_QZS;
    case 5:
      return SYS_CMP;
    case 6:
      return SYS_IRN;
  }
  return 0;
}
/* UBX SigId to signal (ref [5] 1.5.4) ---------------------------------------*/
static int sky_sig(int sys, int signal_type) {
  if (sys == SYS_GPS) {
    switch (signal_type) {
      case 1:
        return CODE_L1X;
      case 2:
        return CODE_L2X;
      case 4:
        return CODE_L5X;
      default:
        return CODE_L1C;
    }
  } else if (sys == SYS_SBS) {
    return CODE_L1C;
  } else if (sys == SYS_GLO) {
    switch (signal_type) {
      case 2:
        return CODE_L2C;
      case 4:
        return CODE_L3X;
      default:
        return CODE_L1C;
    }
  } else if (sys == SYS_GAL) {
    switch (signal_type) {
      case 4:
        return CODE_L5X;
      case 5:
        return CODE_L7X;
      case 6:
        return CODE_L6X;
      default:
        return CODE_L1C;
    }
  } else if (sys == SYS_QZS) {
    switch (signal_type) {
      case 1:
        return CODE_L1X;
      case 2:
        return CODE_L2X;
      case 4:
        return CODE_L5X;
      case 6:
        return CODE_L6X;
      default:
        return CODE_L1C;
    }
  } else if (sys == SYS_CMP) { /* BeiDou */
    switch (signal_type) {
      case 1:
        return CODE_L1X;
      case 4:
        return CODE_L5X;
      case 5:
        return CODE_L7I;
      case 7:
        return CODE_L6I;
      default:
        return CODE_L2I;
    }
  } else {
    trace(2, "stq rawx gnss type error: type=%d\n", sys);
    return (CODE_NONE);
  }
}
/* Decode skytraq measurement epoch (0xDC) -----------------------------------*/
static int decode_stqtime(raw_t *raw) {
  uint8_t *p = raw->buff + 4;
  long double tow;
  int week;

  trace(4, "decode_stqtime: len=%d\n", raw->len);

  raw->iod = U1(p + 1);
  week = U2(p + 2);
  week = adjgpsweek(week);
  tow = U4(p + 4) * 0.001L;
  raw->time = gpst2time(week, tow);

  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ EPOCH (%4d): iod=%d week=%d tow=%.3Lf",
                raw->len, raw->iod, week, tow);
  }
  return 0;
}
/* Decode skytraq raw measurement (0xDD) -------------------------------------*/
static int decode_stqraw(raw_t *raw) {
  uint8_t *p = raw->buff + 4, ind;
  long double pr1, cp1;
  int i, j, iod, prn, sys, sat, n = 0, nsat;

  trace(4, "decode_stqraw: len=%d\n", raw->len);

  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ RAW   (%4d): nsat=%d", raw->len,
                U1(p + 2));
  }
  iod = U1(p + 1);
  if (iod != raw->iod) { /* Need preceding measurement epoch (0xDC) */
    trace(2, "stq raw iod error: iod=%d %d\n", iod, raw->iod);
    return -1;
  }
  nsat = U1(p + 2);
  if (raw->len < 8 + 23 * nsat) {
    trace(2, "stq raw length error: len=%d nsat=%d\n", raw->len, nsat);
    return -1;
  }
  for (i = 0, p += 3; i < nsat && i < MAXOBS; i++, p += 23) {
    prn = U1(p);

    if (MINPRNGPS <= prn && prn <= MAXPRNGPS) {
      sys = SYS_GPS;
    } else if (MINPRNGLO <= prn - 64 && prn - 64 <= MAXPRNGLO) {
      sys = SYS_GLO;
      prn -= 64;
    } else if (MINPRNQZS <= prn && prn <= MAXPRNQZS) {
      sys = SYS_QZS;
    } else if (MINPRNCMP <= prn - 200 && prn - 200 <= MAXPRNCMP) {
      sys = SYS_CMP;
      prn -= 200;
    } else {
      trace(2, "stq raw satellite number error: prn=%d\n", prn);
      continue;
    }
    if (!(sat = satno(sys, prn))) {
      trace(2, "stq raw satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    ind = U1(p + 22);
    pr1 = !(ind & 1) ? 0.0L : (long double)R8(p + 2);
    cp1 = !(ind & 4) ? 0.0L : (long double)R8(p + 10);
    cp1 -= floorl((cp1 + 1E9L) / 2E9L) * 2E9L; /* -10^9 < cp1 < 10^9 */

    raw->obs.data[n].P[0] = pr1;
    raw->obs.data[n].L[0] = cp1;
    raw->obs.data[n].D[0] = !(ind & 2) ? 0.0L : (long double)R4(p + 18);
    raw->obs.data[n].SNR[0] = (uint16_t)(U1(p + 1) / SNR_UNIT + 0.5L);
    raw->obs.data[n].LLI[0] = 0;
    raw->obs.data[n].code[0] = sys == SYS_CMP ? CODE_L2I : CODE_L1C;

    raw->lockt[sat - 1][0] = ind & 8 ? 1 : 0; /* Cycle slip */

    if (raw->obs.data[n].L[0] != 0.0L) {
      raw->obs.data[n].LLI[0] = (uint8_t)raw->lockt[sat - 1][0];
      raw->lockt[sat - 1][0] = 0;
    }
    /* Receiver dependent options */
    if (strstr(raw->opt, "-INVCP")) {
      raw->obs.data[n].L[0] *= -1.0L;
    }
    raw->obs.data[n].time = raw->time;
    raw->obs.data[n].sat = sat;

    for (j = 1; j < NFREQ + NEXOBS; j++) {
      raw->obs.data[n].L[j] = raw->obs.data[n].P[j] = 0.0L;
      raw->obs.data[n].D[j] = 0.0L;
      raw->obs.data[n].SNR[j] = raw->obs.data[n].LLI[j] = 0;
      raw->obs.data[n].code[j] = CODE_NONE;
    }
    n++;
  }
  raw->obs.n = n;
  return n > 0 ? 1 : 0;
}
/* Decode skytraq extended raw measurement data v.1 (0xE5) -------------------*/
static int decode_stqrawx(raw_t *raw) {
  uint8_t *p = raw->buff + 4, ind;
  long double tow, peri, pr1, cp1;
  int i, j, k, ver, week, nsat, sys, sig, prn, sat, n = 0, idx;

  trace(4, "decode_stqraw: len=%d\n", raw->len);

  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ RAWX  (%4d): nsat=%2d", raw->len,
                U1(p + 13));
  }
  ver = U1(p + 1);
  raw->iod = U1(p + 2);
  week = U2(p + 3);
  week = adjgpsweek(week);
  tow = U4(p + 5) * 0.001L;
  raw->time = gpst2time(week, tow);
  peri = U2(p + 9) * 0.001L;
  nsat = U1(p + 13);
  if (raw->len < 19 + 31 * nsat) {
    trace(2, "stq raw length error: len=%d nsat=%d\n", raw->len, nsat);
    return -1;
  }
  for (i = 0, p += 14; i < nsat && i < MAXOBS; i++, p += 31) {
    sys = sky_sys(U1(p) & 0xF);
    sig = sky_sig(sys, (U1(p) >> 4) & 0xF);
    idx = code2idx(sys, sig);
    prn = U1(p + 1);
    if (!(sat = satno(sys, prn))) {
      trace(2, "stq raw satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    /* Set GLONASS freq channel number */
    if (sys == SYS_GLO) {
      raw->nav.geph[prn - 1][0].frq = (int)(U1(p + 2) & 0xF) - 7;
    }
    ind = U2(p + 27);
    pr1 = !(ind & 1) ? 0.0L : (long double)R8(p + 4);
    cp1 = !(ind & 4) ? 0.0L : (long double)R8(p + 12);
    cp1 -= floorl((cp1 + 1E9L) / 2E9L) * 2E9L; /* -10^9 < cp1 < 10^9 */

    for (j = 0; j < n; j++) {
      if (raw->obs.data[j].sat == sat) break;
    }
    if (j >= n) {
      raw->obs.data[n].time = raw->time;
      raw->obs.data[n].sat = sat;
      raw->obs.data[n].rcv = 0;
      for (k = 0; k < NFREQ + NEXOBS; k++) {
        raw->obs.data[n].L[k] = raw->obs.data[n].P[k] = 0.0L;
        raw->obs.data[n].D[k] = 0.0L;
        raw->obs.data[n].SNR[k] = raw->obs.data[n].LLI[k] = 0;
        raw->obs.data[n].code[k] = CODE_NONE;
      }
      n++;
    }
    raw->obs.data[j].P[idx] = pr1;
    raw->obs.data[j].L[idx] = cp1;
    raw->obs.data[j].D[idx] = !(ind & 2) ? 0.0L : (long double)R4(p + 20);
    raw->obs.data[j].SNR[idx] = (uint16_t)(U1(p + 3) / SNR_UNIT + 0.5L);
    raw->obs.data[j].LLI[idx] = 0;
    raw->obs.data[j].code[idx] = sig;

    raw->lockt[sat - 1][idx] = ind & 8 ? 1 : 0; /* Cycle slip */

    if (raw->obs.data[j].L[idx] != 0.0L) {
      raw->obs.data[j].LLI[idx] = (uint8_t)raw->lockt[sat - 1][idx];
      raw->lockt[sat - 1][idx] = 0;
    }
    /* Receiver dependent options */
    if (strstr(raw->opt, "-INVCP")) {
      raw->obs.data[n].L[idx] *= -1.0L;
    }
    raw->obs.data[n].time = raw->time;
    raw->obs.data[n].sat = sat;
  }
  raw->obs.n = n;
  return n > 0 ? 1 : 0;
}
/* Decode Galileo ephemeris (0xE6) -------------------------------------------*/
static int decode_stqgene(raw_t *raw) {
  eph_t eph = {0};
  int i, j, prn, sat, sys;
  int part1, page1, part2, page2, type;
  long double ion[4] = {0}, utc[8] = {0};
  uint8_t *p = raw->buff + 4, buff[32], crc_buff[26] = {0};

  trace(4, "decode_stqgene: len=%d\n", raw->len);

  if (raw->len < 44) {
    trace(2, "stq gene string length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U1(p + 3);
  sys = sky_sys(U1(p + 2) & 0xF);
  if (!(sat = satno(sys, prn))) {
    trace(2, "stq raw satellite number error: sys=%d prn=%d\n", sys, prn);
    return 0;
  }
  if (sys != SYS_GAL) {
    trace(2, "stq sys not supported: sys=%d\n", sys);
    return 0;
  }
  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ GENE (%4d): prn=%2d", raw->len, prn);
  }
  for (i = 0; i < 8; i++, p += 4) {
    setbitu(buff, sizeof(buff), 32 * i, 32, U4(p + 5));
  }
  part1 = getbitu(buff, sizeof(buff), 0, 1);
  page1 = getbitu(buff, sizeof(buff), 1, 1);
  part2 = getbitu(buff, sizeof(buff), 128, 1);
  page2 = getbitu(buff, sizeof(buff), 129, 1);

  if (part1 != 0 || part2 != 1) {
    trace(3, "ubx rxmsfrbx enav page even/odd error: sat=%d\n", sat);
    return -1;
  }
  if (page1 == 1 || page2 == 1) return 0; /* Alert page */

  /* Test crc (4(pad) + 114 + 82 bits) */
  for (i = 0, j = 4; i < 15; i++, j += 8)
    setbitu(crc_buff, sizeof(crc_buff), j, 8, getbitu(buff, sizeof(buff), i * 8, 8));
  for (i = 0, j = 118; i < 11; i++, j += 8)
    setbitu(crc_buff, sizeof(crc_buff), j, 8, getbitu(buff, sizeof(buff), i * 8 + 128, 8));
  if (rtk_crc24q(crc_buff, sizeof(crc_buff), 25) != getbitu(buff, sizeof(buff), 128 + 82, 24)) {
    trace(2, "ubx rxmsfrbx enav crc error: sat=%d\n", sat);
    return -1;
  }
  type = getbitu(buff, sizeof(buff), 2, 6); /* Word type */

  if (type > 6) return 0;

  /* Save 128 (112:even+16:odd) bits word */
  for (i = 0, j = 2; i < 14; i++, j += 8) {
    raw->subfrm[sat - 1][type * 16 + i] = getbitu(buff, sizeof(buff), j, 8);
  }
  for (i = 14, j = 130; i < 16; i++, j += 8) {
    raw->subfrm[sat - 1][type * 16 + i] = getbitu(buff, sizeof(buff), j, 8);
  }
  if (type != 5) return 0;
  if (!decode_gal_inav(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), &eph, ion, utc)) return 0;

  if (eph.sat != sat) {
    trace(2, "skytraq enav satellite error: sat=%d %d\n", sat, eph.sat);
    return -1;
  }
  eph.code |= (1 << 0); /* Data source: E1 */

  adj_utcweek(raw->time, utc);
  matcpy(raw->nav.ion_gal, ion, 4, 1);
  matcpy(raw->nav.utc_gal, utc, 8, 1);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1][0].iode &&
        timediff(eph.toe, raw->nav.eph[sat - 1][0].toe) == 0.0L &&
        timediff(eph.toc, raw->nav.eph[sat - 1][0].toc) == 0.0L)
      return 0;
  }
  raw->nav.eph[sat - 1][0] = eph;
  raw->ephsat = sat;
  raw->ephset = 0; /* 0:I/NAV */
  return 2;
}
/* Save GPS/QZSS subframe ----------------------------------------------------*/
static int save_subfrm_gps(int sat, raw_t *raw) {
  uint8_t *p = raw->buff + 7, *q;
  int i, id;

  trace(4, "save_subfrm_gps: sat=%2d\n", sat);

  /* Check navigation subframe preamble */
  if (p[0] != 0x8B) {
    trace(2, "stq subframe preamble error: 0x%02X\n", p[0]);
    return 0;
  }
  id = (p[5] >> 2) & 0x7;

  /* Check subframe id */
  if (id < 1 || 5 < id) {
    trace(2, "stq subframe id error: id=%d\n", id);
    return 0;
  }
  q = raw->subfrm[sat - 1] + (id - 1) * 30;

  for (i = 0; i < 30; i++) q[i] = p[i];

  return id;
}
/* Decode ephemeris ----------------------------------------------------------*/
static int decode_ephem(int sat, raw_t *raw) {
  eph_t eph = {0};

  trace(4, "decode_ephem: sat=%2d\n", sat);

  if (!decode_frame(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), &eph, NULL, NULL, NULL)) return 0;

  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1][0].iode && eph.iodc == raw->nav.eph[sat - 1][0].iodc)
      return 0; /* Unchanged */
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1][0] = eph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}
/* Decode almanac and ion/UTC ------------------------------------------------*/
static int decode_alm1(int sat, raw_t *raw) {
  int sys = satsys(sat, NULL);

  trace(4, "decode_alm1 : sat=%2d\n", sat);

  if (sys == SYS_GPS) {
    decode_frame(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), NULL, raw->nav.alm, raw->nav.ion_gps,
                 raw->nav.utc_gps);
    adj_utcweek(raw->time, raw->nav.utc_gps);
  } else if (sys == SYS_QZS) {
    decode_frame(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), NULL, raw->nav.alm, raw->nav.ion_qzs,
                 raw->nav.utc_qzs);
    adj_utcweek(raw->time, raw->nav.utc_qzs);
  }
  return 9;
}
/* Decode almanac ------------------------------------------------------------*/
static int decode_alm2(int sat, raw_t *raw) {
  int sys = satsys(sat, NULL);

  trace(4, "decode_alm2 : sat=%2d\n", sat);

  if (sys == SYS_GPS) {
    decode_frame(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), NULL, raw->nav.alm, NULL, NULL);
  } else if (sys == SYS_QZS) {
    decode_frame(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), NULL, raw->nav.alm, raw->nav.ion_qzs,
                 raw->nav.utc_qzs);
    adj_utcweek(raw->time, raw->nav.utc_qzs);
  }
  return 0;
}
/* Decode GPS/QZSS subframe (0xE0) -------------------------------------------*/
static int decode_stqgps(raw_t *raw) {
  int prn, sat, id;
  uint8_t *p = raw->buff + 4;

  trace(4, "decode_stqgps: len=%d\n", raw->len);

  if (raw->len < 40) {
    trace(2, "stq gps/qzss subframe length error: len=%d\n", raw->len);
    return -1;
  }
  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ GPSSF (%4d): prn=%2d id=%d", raw->len,
                U1(p + 1), (p[8] >> 2) & 0x7);
  }
  prn = U1(p + 1);
  if (!(sat = satno(MINPRNQZS <= prn && prn <= MAXPRNQZS ? SYS_QZS : SYS_GPS, prn))) {
    trace(2, "stq gps/qzss subframe satellite number error: prn=%d\n", prn);
    return -1;
  }
  id = save_subfrm_gps(sat, raw);
  if (id == 3) return decode_ephem(sat, raw);
  if (id == 4) return decode_alm1(sat, raw);
  if (id == 5) return decode_alm2(sat, raw);
  return 0;
}
/* Decode GLONASS string (0xE1) ----------------------------------------------*/
static int decode_stqglo(raw_t *raw) {
  geph_t geph = {0};
  int i, prn, sat, m;
  uint8_t *p = raw->buff + 4;

  trace(4, "decode_stqglo: len=%d\n", raw->len);

  if (raw->len < 19) {
    trace(2, "stq glo string length error: len=%d\n", raw->len);
    return -1;
  }
  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ GLSTR (%4d): prn=%2d no=%d", raw->len,
                U1(p + 1) - 64, U1(p + 2));
  }
  prn = U1(p + 1) - 64;
  if (!(sat = satno(SYS_GLO, prn))) {
    trace(2, "stq glo string satellite number error: prn=%d\n", prn);
    return -1;
  }
  m = U1(p + 2); /* String number */
  if (m < 1 || 4 < m) {
    return 0; /* Non-immediate info and almanac */
  }
  setbitu(raw->subfrm[sat - 1] + (m - 1) * 10, sizeof(raw->subfrm[0]) - (m - 1) * 10, 1, 4, m);
  for (i = 0; i < 9; i++) {
    setbitu(raw->subfrm[sat - 1] + (m - 1) * 10, sizeof(raw->subfrm[0]) - (m - 1) * 10, 5 + i * 8,
            8, p[3 + i]);
  }
  if (m != 4) return 0;

  /* Decode GLONASS ephemeris strings */
  geph.tof = raw->time;
  if (!decode_glostr(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), &geph, NULL) || geph.sat != sat)
    return 0;

  if (!strstr(raw->opt, "-EPHALL")) {
    if (geph.iode == raw->nav.geph[prn - 1][0].iode) return 0; /* Unchanged */
  }
  /* Keep freq channel number */
  geph.frq = raw->nav.geph[prn - 1][0].frq;
  raw->nav.geph[prn - 1][0] = geph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}
/* Decode GLONASS string (requested) (0x5C) ----------------------------------*/
static int decode_stqgloe(raw_t *raw) {
  int prn, sat;
  uint8_t *p = raw->buff + 4;

  trace(4, "decode_stqgloe: len=%d\n", raw->len);

  if (raw->len < 50) {
    trace(2, "stq glo string length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U1(p + 1);
  if (!(sat = satno(SYS_GLO, prn))) {
    trace(2, "stq gloe string satellite number error: prn=%d\n", prn);
    return -1;
  }
  /* Set frequency channel number */
  raw->nav.geph[prn - 1][0].frq = I1(p + 2);

  return 0;
}
/* Decode BeiDou subframe (0xE2,0xE3) ----------------------------------------*/
static int decode_stqbds(raw_t *raw) {
  eph_t eph = {0};
  uint32_t word;
  int i, j = 0, id, pgn, prn, sat;
  uint8_t *p = raw->buff + 4;

  trace(4, "decode_stqbds: len=%d\n", raw->len);

  if (raw->len < 38) {
    trace(2, "stq bds subframe length error: len=%d\n", raw->len);
    return -1;
  }
  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ BDSSF (%4d): prn=%2d id=%d", raw->len,
                U1(p + 1) - 200, U1(p + 2));
  }
  prn = U1(p + 1) - 200;
  if (!(sat = satno(SYS_CMP, prn))) {
    trace(2, "stq bds subframe satellite number error: prn=%d\n", prn);
    return -1;
  }
  id = U1(p + 2); /* Subframe id */
  if (id < 1 || 5 < id) {
    trace(2, "stq bds subframe id error: prn=%2d\n", prn);
    return -1;
  }
  if (prn > 5) { /* IGSO/MEO */
    word = getbitu(p + 3, sizeof(raw->buff) - (p + 3 - raw->buff), j, 26) << 4;
    j += 26;
    setbitu(raw->subfrm[sat - 1] + (id - 1) * 38, sizeof(raw->subfrm[0]) - (id - 1) * 38, 0, 30,
            word);

    for (i = 1; i < 10; i++) {
      word = getbitu(p + 3, sizeof(raw->buff) - (p + 3 - raw->buff), j, 22) << 8;
      j += 22;
      setbitu(raw->subfrm[sat - 1] + (id - 1) * 38, sizeof(raw->subfrm[0]) - (id - 1) * 38, i * 30,
              30, word);
    }
    if (id != 3) return 0;
    if (!decode_bds_d1(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), &eph, NULL, NULL)) return 0;
  } else { /* GEO */
    if (id != 1) return 0;

    pgn = getbitu(p + 3, sizeof(raw->buff) - (p + 3 - raw->buff), 26 + 12, 4); /* Page number */
    if (pgn < 1 || 10 < pgn) {
      trace(2, "stq bds subframe page number error: prn=%2d pgn=%d\n", prn, pgn);
      return -1;
    }
    word = getbitu(p + 3, sizeof(raw->buff) - (p + 3 - raw->buff), j, 26) << 4;
    j += 26;
    setbitu(raw->subfrm[sat - 1] + (pgn - 1) * 38, sizeof(raw->subfrm[0]) - (pgn - 1) * 38, 0, 30,
            word);

    for (i = 1; i < 10; i++) {
      word = getbitu(p + 3, sizeof(raw->buff) - (p + 3 - raw->buff), j, 22) << 8;
      j += 22;
      setbitu(raw->subfrm[sat - 1] + (pgn - 1) * 38, sizeof(raw->subfrm[0]) - (pgn - 1) * 38,
              i * 30, 30, word);
    }
    if (pgn != 10) return 0;
    if (!decode_bds_d2(raw->subfrm[sat - 1], sizeof(raw->subfrm[0]), &eph, NULL)) return 0;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (timediff(eph.toe, raw->nav.eph[sat - 1][0].toe) == 0.0L) return 0; /* Unchanged */
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1][0] = eph;
  raw->ephsat = sat;
  raw->ephset = 0;
  return 2;
}
/* Decode ack to request msg (0x83) ------------------------------------------*/
static int decode_stqack(raw_t *raw) {
  uint8_t *p = raw->buff + 4;

  trace(4, "decode_stqack: len=%d\n", raw->len);

  if (raw->len < 9) {
    trace(2, "stq ack length error: len=%d\n", raw->len);
    return -1;
  }
  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ ACK   (%4d): msg=0x%02X", raw->len,
                U1(p + 1));
  }
  return 0;
}
/* Decode nack to request msg (0x84) -----------------------------------------*/
static int decode_stqnack(raw_t *raw) {
  uint8_t *p = raw->buff + 4;

  trace(4, "decode_stqnack: len=%d\n", raw->len);

  if (raw->len < 9) {
    trace(2, "stq nack length error: len=%d\n", raw->len);
    return -1;
  }
  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ NACK  (%4d): msg=0x%02X", raw->len,
                U1(p + 1));
  }
  return 0;
}
/* Decode skytraq message ----------------------------------------------------*/
static int decode_stq(raw_t *raw) {
  int type = U1(raw->buff + 4);
  uint8_t cs, *p = raw->buff + raw->len - 3;

  trace(3, "decode_stq: type=%02x len=%d\n", type, raw->len);

  /* Checksum */
  cs = checksum(raw->buff, raw->len);

  if (cs != *p || *(p + 1) != 0x0D || *(p + 2) != 0x0A) {
    trace(2, "stq checksum error: type=%02X cs=%02X tail=%02X%02X%02X\n", type, cs, *p, *(p + 1),
          *(p + 2));
    return -1;
  }
  if (raw->outtype) {
    rtksnprintf(raw->msgtype, sizeof(raw->msgtype), "SKYTRAQ 0x%02X  (%4d):", type, raw->len);
  }
  switch (type) {
    case ID_STQTIME:
      return decode_stqtime(raw);
    case ID_STQRAW:
      return decode_stqraw(raw);
    case ID_STQRAWX:
      return decode_stqrawx(raw);
    case ID_STQGPS:
      return decode_stqgps(raw);
    case ID_STQGLO:
      return decode_stqglo(raw);
    case ID_STQGLOE:
      return decode_stqgloe(raw);
    case ID_STQGENE:
      return decode_stqgene(raw);
    case ID_STQBDSD1:
      return decode_stqbds(raw);
    case ID_STQBDSD2:
      return decode_stqbds(raw);
    /* Case ID_STQGENE : return decode_stqgene(raw); */
    case ID_STQACK:
      return decode_stqack(raw);
    case ID_STQNACK:
      return decode_stqnack(raw);
  }
  return 0;
}
/* Sync code -----------------------------------------------------------------*/
static int sync_stq(uint8_t *buff, uint8_t data) {
  buff[0] = buff[1];
  buff[1] = data;
  return buff[0] == STQSYNC1 && buff[1] == STQSYNC2;
}
/* Input skytraq raw message from stream ---------------------------------------
 * Fetch next skytraq raw data and input a mesasge from stream
 * Args   : raw_t *raw       IO  receiver raw data control struct
 *          uint8_t data     I   stream data (1 byte)
 * Return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 3: input SBAS message,
 *                  9: input ion/UTC parameter)
 *
 * Notes  : to specify input options, set raw->opt to the following option
 *          strings separated by spaces.
 *
 *          -INVCP     : inverse polarity of carrier-phase
 *
 *----------------------------------------------------------------------------*/
extern int input_stq(raw_t *raw, uint8_t data) {
  trace(5, "input_stq: data=%02x\n", data);

  /* Synchronize frame */
  if (raw->nbyte == 0) {
    if (!sync_stq(raw->buff, data)) return 0;
    raw->nbyte = 2;
    return 0;
  }
  raw->buff[raw->nbyte++] = data;

  if (raw->nbyte == 4) {
    if ((raw->len = U2(raw->buff + 2) + 7) > MAXRAWLEN) {
      trace(2, "stq message length error: len=%d\n", raw->len);
      raw->nbyte = 0;
      return -1;
    }
  }
  if (raw->nbyte < 4 || raw->nbyte < raw->len) return 0;
  raw->nbyte = 0;

  /* Decode skytraq raw message */
  return decode_stq(raw);
}
/* Input skytraq raw message from file -----------------------------------------
 * Fetch next skytraq raw data and input a message from file
 * Args   : raw_t  *raw      IO  receiver raw data control struct
 *          FILE   *fp       I   file pointer
 * Return : status(-2: end of file, -1...9: same as above)
 *----------------------------------------------------------------------------*/
extern int input_stqf(raw_t *raw, FILE *fp) {
  int i, data;

  trace(4, "input_stqf:\n");

  /* Synchronize frame */
  if (raw->nbyte == 0) {
    for (i = 0;; i++) {
      if ((data = fgetc(fp)) == EOF) return -2;
      if (sync_stq(raw->buff, (uint8_t)data)) break;
      if (i >= 4096) return 0;
    }
  }
  if (fread(raw->buff + 2, 1, 2, fp) < 2) return -2;
  raw->nbyte = 4;

  if ((raw->len = U2(raw->buff + 2) + 7) > MAXRAWLEN) {
    trace(2, "stq message length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }
  if (fread(raw->buff + 4, 1, raw->len - 4, fp) < (size_t)(raw->len - 4)) return -2;
  raw->nbyte = 0;

  /* Decode skytraq raw message */
  return decode_stq(raw);
}
/* Generate skytraq binary message ---------------------------------------------
 * Generate skytraq binary message from message string
 * Args   : char  *msg       I   message string
 *            "RESTART  [arg...]" system restart
 *            "CFG-SERI [arg...]" configure serial port propperty
 *            "CFG-FMT  [arg...]" configure output message format
 *            "CFG-RATE [arg...]" configure binary measurement output rates
 *            "CFG-BIN  [arg...]" configure general binary
 *            "GET-GLOEPH [slot]" get GLONASS ephemeris for freq channel number
 *          uint8_t *buff O binary message
 * Return : length of binary message (0: error)
 * Note   : see reference [1][2][3][4] for details.
 *----------------------------------------------------------------------------*/
extern int gen_stq(const char *msg, uint8_t *buff, size_t size) {
  /* TODO respect the buff size */
  const char *hz[] = {"1Hz", "2Hz", "4Hz", "5Hz", "10Hz", "20Hz", ""};
  char mbuff[1024], *args[32], *p;
  int i, narg = 0;

  trace(4, "gen_stq: msg=%s\n", msg);

  rtkstrcpy(mbuff, sizeof(mbuff), msg);
  char *r;
  for (p = strtok_r(mbuff, " ", &r); p && narg < 32; p = strtok_r(NULL, " ", &r)) {
    args[narg++] = p;
  }
  if (narg < 1) {
    return 0;
  }
  size_t len = 0;
  buff[len++] = STQSYNC1;
  buff[len++] = STQSYNC2;
  if (!strcmp(args[0], "RESTART")) {
    buff[len++] = 0;
    buff[len++] = 15;
    buff[len++] = ID_RESTART;
    buff[len++] = narg > 2 ? (uint8_t)atoi(args[1]) : 0;
    for (i = 1; i < 15; i++) buff[len++] = 0; /* Set all 0 */
  } else if (!strcmp(args[0], "CFG-SERI")) {
    buff[len++] = 0;
    buff[len++] = 4;
    buff[len++] = ID_CFGSERI;
    for (i = 1; i < 4; i++) buff[len++] = narg > i + 1 ? (uint8_t)atoi(args[i]) : 0;
  } else if (!strcmp(args[0], "CFG-FMT")) {
    buff[len++] = 0;
    buff[len++] = 3;
    buff[len++] = ID_CFGFMT;
    for (i = 1; i < 3; i++) buff[len++] = narg > i + 1 ? (uint8_t)atoi(args[i]) : 0;
  } else if (!strcmp(args[0], "CFG-RATE")) {
    buff[len++] = 0;
    buff[len++] = 8;
    buff[len++] = ID_CFGRATE;
    if (narg > 2) {
      for (i = 0; *hz[i]; i++)
        if (!strcmp(args[1], hz[i])) break;
      if (*hz[i])
        buff[len++] = i;
      else
        buff[len++] = (uint8_t)atoi(args[1]);
    } else
      buff[len++] = 0;
    for (i = 2; i < 8; i++) buff[len++] = narg > i + 1 ? (uint8_t)atoi(args[i]) : 0;
  } else if (!strcmp(args[0], "CFG-BIN")) {
    buff[len++] = 0;
    buff[len++] = 9; /* F/W 1.4.32 */
    buff[len++] = ID_CFGBIN;
    if (narg > 2) {
      for (i = 0; *hz[i]; i++)
        if (!strcmp(args[1], hz[i])) break;
      if (*hz[i])
        buff[len++] = i;
      else
        buff[len++] = (uint8_t)atoi(args[1]);
    } else
      buff[len++] = 0;
    for (i = 2; i < 9; i++) buff[len++] = narg > i + 1 ? (uint8_t)atoi(args[i]) : 0;
  } else if (!strcmp(args[0], "GET-GLOEPH")) {
    buff[len++] = 0;
    buff[len++] = 2;
    buff[len++] = ID_GETGLOEPH;
    buff[len++] = narg >= 2 ? (uint8_t)atoi(args[1]) : 0;
  } else
    return 0;

  uint8_t cs = checksum(buff, len + 3);
  buff[len++] = cs;
  buff[len++] = 0x0D;
  buff[len] = 0x0A;

  trace(4, "gen_stq: buff=\n");
  traceb(4, buff, size, len);
  return (int)len;
}
