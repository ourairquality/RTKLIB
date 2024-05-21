/*------------------------------------------------------------------------------
 * sbas.c : SBAS functions
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Option : -DRRCENA  enable rrc correction
 *
 * References :
 *     [1] RTCA/DO-229C, Minimum operational performanc standards for global
 *         positioning system/wide area augmentation system airborne equipment,
 *         RTCA inc, November 28, 2001
 *     [2] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
 *         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
 *         July 31, 2009
 *
 * Version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * History : 2007/10/14 1.0  new
 *           2009/01/24 1.1  modify sbspntpos() api
 *                           improve fast/ion correction update
 *           2009/04/08 1.2  move function crc24q() to rcvlog.c
 *                           support GLONASS, Galileo and QZSS
 *           2009/06/08 1.3  modify sbsupdatestat()
 *                           delete sbssatpos()
 *           2009/12/12 1.4  support GLONASS
 *           2010/01/22 1.5  support EMS (egnos message service) format
 *           2010/06/10 1.6  added api:
 *                               sbssatcorr(),sbstropcorr(),sbsioncorr(),
 *                               sbsupdatecorr()
 *                           changed api:
 *                               sbsreadmsgt(),sbsreadmsg()
 *                           deleted api:
 *                               sbspntpos(),sbsupdatestat()
 *           2010/08/16 1.7  not reject udre==14 or give==15 correction message
 *                           (2.4.0_p4)
 *           2011/01/15 1.8  use api ionppp()
 *                           add prn mask of QZSS for QZSS L1SAIF
 *           2016/07/29 1.9  crc24q() -> rtk_crc24q()
 *           2020/11/30 1.10 use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Constants -----------------------------------------------------------------*/

#define WEEKOFFSET 1024 /* GPS week offset for NovAtel OEM-3 */

/* SBAS igp definition -------------------------------------------------------*/
static const int16_t x1[] = {-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0,
                             5,   10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  65,  75, 85},
                     x2[] = {-55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0,
                             5,   10,  15,  20,  25,  30,  35,  40,  45,  50,  55},
                     x3[] = {-75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0,
                             5,   10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  65,  75},
                     x4[] = {-85, -75, -65, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5,
                             0,   5,   10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  65,  75},
                     x5[] = {-180, -175, -170, -165, -160, -155, -150, -145, -140, -135, -130, -125,
                             -120, -115, -110, -105, -100, -95,  -90,  -85,  -80,  -75,  -70,  -65,
                             -60,  -55,  -50,  -45,  -40,  -35,  -30,  -25,  -20,  -15,  -10,  -5,
                             0,    5,    10,   15,   20,   25,   30,   35,   40,   45,   50,   55,
                             60,   65,   70,   75,   80,   85,   90,   95,   100,  105,  110,  115,
                             120,  125,  130,  135,  140,  145,  150,  155,  160,  165,  170,  175},
                     x6[] = {-180, -170, -160, -150, -140, -130, -120, -110, -100, -90, -80, -70,
                             -60,  -50,  -40,  -30,  -20,  -10,  0,    10,   20,   30,  40,  50,
                             60,   70,   80,   90,   100,  110,  120,  130,  140,  150, 160, 170},
                     x7[] = {-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150},
                     x8[] = {-170, -140, -110, -80, -50, -20, 10, 40, 70, 100, 130, 160};

EXPORT const sbsigpband_t igpband1[9][8] = {/* Band 0-8 */
                                            {{-180, x1, 1, 28},
                                             {-175, x2, 29, 51},
                                             {-170, x3, 52, 78},
                                             {-165, x2, 79, 101},
                                             {-160, x3, 102, 128},
                                             {-155, x2, 129, 151},
                                             {-150, x3, 152, 178},
                                             {-145, x2, 179, 201}},
                                            {{-140, x4, 1, 28},
                                             {-135, x2, 29, 51},
                                             {-130, x3, 52, 78},
                                             {-125, x2, 79, 101},
                                             {-120, x3, 102, 128},
                                             {-115, x2, 129, 151},
                                             {-110, x3, 152, 178},
                                             {-105, x2, 179, 201}},
                                            {{-100, x3, 1, 27},
                                             {-95, x2, 28, 50},
                                             {-90, x1, 51, 78},
                                             {-85, x2, 79, 101},
                                             {-80, x3, 102, 128},
                                             {-75, x2, 129, 151},
                                             {-70, x3, 152, 178},
                                             {-65, x2, 179, 201}},
                                            {{-60, x3, 1, 27},
                                             {-55, x2, 28, 50},
                                             {-50, x4, 51, 78},
                                             {-45, x2, 79, 101},
                                             {-40, x3, 102, 128},
                                             {-35, x2, 129, 151},
                                             {-30, x3, 152, 178},
                                             {-25, x2, 179, 201}},
                                            {{-20, x3, 1, 27},
                                             {-15, x2, 28, 50},
                                             {-10, x3, 51, 77},
                                             {-5, x2, 78, 100},
                                             {0, x1, 101, 128},
                                             {5, x2, 129, 151},
                                             {10, x3, 152, 178},
                                             {15, x2, 179, 201}},
                                            {{20, x3, 1, 27},
                                             {25, x2, 28, 50},
                                             {30, x3, 51, 77},
                                             {35, x2, 78, 100},
                                             {40, x4, 101, 128},
                                             {45, x2, 129, 151},
                                             {50, x3, 152, 178},
                                             {55, x2, 179, 201}},
                                            {{60, x3, 1, 27},
                                             {65, x2, 28, 50},
                                             {70, x3, 51, 77},
                                             {75, x2, 78, 100},
                                             {80, x3, 101, 127},
                                             {85, x2, 128, 150},
                                             {90, x1, 151, 178},
                                             {95, x2, 179, 201}},
                                            {{100, x3, 1, 27},
                                             {105, x2, 28, 50},
                                             {110, x3, 51, 77},
                                             {115, x2, 78, 100},
                                             {120, x3, 101, 127},
                                             {125, x2, 128, 150},
                                             {130, x4, 151, 178},
                                             {135, x2, 179, 201}},
                                            {{140, x3, 1, 27},
                                             {145, x2, 28, 50},
                                             {150, x3, 51, 77},
                                             {155, x2, 78, 100},
                                             {160, x3, 101, 127},
                                             {165, x2, 128, 150},
                                             {170, x3, 151, 177},
                                             {175, x2, 178, 200}}};
EXPORT const sbsigpband_t igpband2[2][5] = {/* Band 9-10 */
                                            {{60, x5, 1, 72},
                                             {65, x6, 73, 108},
                                             {70, x6, 109, 144},
                                             {75, x6, 145, 180},
                                             {85, x7, 181, 192}},
                                            {{-60, x5, 1, 72},
                                             {-65, x6, 73, 108},
                                             {-70, x6, 109, 144},
                                             {-75, x6, 145, 180},
                                             {-85, x8, 181, 192}}};
/* Extract field from line ---------------------------------------------------*/
static int getfield(char *buff, size_t start, int pos) {
  int pi = start;
  for (pos--; pos > 0; pos--, pi++) {
    pi = strchri(buff, pi, ',');
    if (pi < 0) return -1;
  }
  return pi;
}
/* Variance of fast correction (udre=UDRE+1) ---------------------------------*/
static long double varfcorr(int udre) {
  const long double var[14] = {0.052L,  0.0924L, 0.1444L, 0.283L,  0.4678L,  0.8315L,   1.2992L,
                               1.8709L, 2.5465L, 3.326L,  5.1968L, 20.7870L, 230.9661L, 2078.695L};
  return 0 < udre && udre <= 14 ? var[udre - 1] : 0.0L;
}
/* Variance of ionosphere correction (give=GIVEI+1) --------------------------*/
static long double varicorr(int give) {
  const long double var[15] = {0.0084L, 0.0333L, 0.0749L, 0.1331L, 0.2079L,
                               0.2994L, 0.4075L, 0.5322L, 0.6735L, 0.8315L,
                               1.1974L, 1.8709L, 3.326L,  20.787L, 187.0826L};
  return 0 < give && give <= 15 ? var[give - 1] : 0.0L;
}
/* Fast correction degradation -----------------------------------------------*/
static long double degfcorr(int ai) {
  const long double degf[16] = {0.00000L, 0.00005L, 0.00009L, 0.00012L, 0.00015L, 0.00020L,
                                0.00030L, 0.00045L, 0.00060L, 0.00090L, 0.00150L, 0.00210L,
                                0.00270L, 0.00330L, 0.00460L, 0.00580L};
  return 0 < ai && ai <= 15 ? degf[ai] : 0.0058L;
}

static uint32_t sbsmsg_getbitu(const sbsmsg_t *msg, unsigned pos, unsigned len) {
  return getbitu(msg->msg, sizeof(msg->msg), pos, len);
}
static int32_t sbsmsg_getbits(const sbsmsg_t *msg, unsigned pos, unsigned len) {
  return getbits(msg->msg, sizeof(msg->msg), pos, len);
}

/* Decode type 1: prn masks --------------------------------------------------*/
static bool decode_sbstype1(const sbsmsg_t *msg, sbssat_t *sbssat) {
  trace(4, "decode_sbstype1:\n");

  int n = 0;
  for (int i = 1; i <= 210 && n < MAXSAT; i++) {
    if (sbsmsg_getbitu(msg, 13 + i, 1)) {
      int sat;
      if (i <= 37)
        sat = satno(SYS_GPS, i); /*   0- 37: GPS */
      else if (i <= 61)
        sat = satno(SYS_GLO, i - 37); /*  38- 61: GLONASS */
      else if (i <= 119)
        sat = 0; /*  62-119: future GNSS */
      else if (i <= 138)
        sat = satno(SYS_SBS, i); /* 120-138: geo/waas */
      else if (i <= 182)
        sat = 0; /* 139-182: reserved */
      else if (i <= 192)
        sat = satno(SYS_SBS, i + 10); /* 183-192: QZSS ref [2] */
      else if (i <= 202)
        sat = satno(SYS_QZS, i); /* 193-202: QZSS ref [2] */
      else
        sat = 0; /* 203-   : reserved */
      sbssat->sat[n++].sat = sat;
    }
  }
  sbssat->iodp = sbsmsg_getbitu(msg, 224, 2);
  sbssat->nsat = n;

  trace(5, "decode_sbstype1: nprn=%d iodp=%d\n", n, sbssat->iodp);
  return true;
}
/* Decode type 2-5,0: fast corrections ---------------------------------------*/
static bool decode_sbstype2(const sbsmsg_t *msg, sbssat_t *sbssat) {
  trace(4, "decode_sbstype2:\n");

  if (sbssat->iodp != (int)sbsmsg_getbitu(msg, 16, 2)) return false;

  int type = sbsmsg_getbitu(msg, 8, 6);
  int iodf = sbsmsg_getbitu(msg, 14, 2);

  for (int i = 0; i < 13; i++) {
    int j = 13 * ((type == 0 ? 2 : type) - 2) + i;
    if (j >= sbssat->nsat) break;
    int udre = sbsmsg_getbitu(msg, 174 + 4 * i, 4);
    gtime_t t0 = sbssat->sat[j].fcorr.t0;
    long double prc = sbssat->sat[j].fcorr.prc;
    sbssat->sat[j].fcorr.t0 = gpst2time(msg->week, msg->tow);
    sbssat->sat[j].fcorr.prc = sbsmsg_getbits(msg, 18 + i * 12, 12) * 0.125L;
    sbssat->sat[j].fcorr.udre = udre + 1;
    long double dt = timediff(sbssat->sat[j].fcorr.t0, t0);
    if (t0.time == 0 || dt <= 0.0L || 18.0L < dt || sbssat->sat[j].fcorr.ai == 0) {
      sbssat->sat[j].fcorr.rrc = 0.0L;
      sbssat->sat[j].fcorr.dt = 0.0L;
    } else {
      sbssat->sat[j].fcorr.rrc = (sbssat->sat[j].fcorr.prc - prc) / dt;
      sbssat->sat[j].fcorr.dt = dt;
    }
    sbssat->sat[j].fcorr.iodf = iodf;
  }
  trace(5, "decode_sbstype2: type=%d iodf=%d\n", type, iodf);
  return true;
}
/* Decode type 6: integrity info ---------------------------------------------*/
static bool decode_sbstype6(const sbsmsg_t *msg, sbssat_t *sbssat) {
  trace(4, "decode_sbstype6:\n");

  int iodf[4];
  for (int i = 0; i < 4; i++) {
    iodf[i] = sbsmsg_getbitu(msg, 14 + i * 2, 2);
  }
  /* Limited to 51 to avoid overflow of iodf[] */
  for (int i = 0; i < sbssat->nsat && i <= 51; i++) {
    if (sbssat->sat[i].fcorr.iodf != iodf[i / 13]) continue;
    int udre = sbsmsg_getbitu(msg, 22 + i * 4, 4);
    sbssat->sat[i].fcorr.udre = udre + 1;
  }
  trace(5, "decode_sbstype6: iodf=%d %d %d %d\n", iodf[0], iodf[1], iodf[2], iodf[3]);
  return true;
}
/* Decode type 7: fast correction degradation factor -------------------------*/
static bool decode_sbstype7(const sbsmsg_t *msg, sbssat_t *sbssat) {
  trace(4, "decode_sbstype7\n");

  if (sbssat->iodp != (int)sbsmsg_getbitu(msg, 18, 2)) return false;

  sbssat->tlat = sbsmsg_getbitu(msg, 14, 4);

  for (int i = 0; i < sbssat->nsat && i < MAXSAT; i++) {
    sbssat->sat[i].fcorr.ai = sbsmsg_getbitu(msg, 22 + i * 4, 4);
  }
  return true;
}
/* Decode type 9: geo navigation message -------------------------------------*/
static bool decode_sbstype9(const sbsmsg_t *msg, nav_t *nav) {
  trace(4, "decode_sbstype9:\n");

  int sat = satno(SYS_SBS, msg->prn);
  if (!sat) {
    trace(2, "invalid prn in sbas type 9: prn=%3d\n", msg->prn);
    return false;
  }
  int t = (int)sbsmsg_getbitu(msg, 22, 13) * 16 - (int)msg->tow % 86400;
  if (t <= -43200)
    t += 86400;
  else if (t > 43200)
    t -= 86400;
  seph_t seph = {0};
  seph.sat = sat;
  seph.t0 = gpst2time(msg->week, msg->tow + t);
  seph.tof = gpst2time(msg->week, msg->tow);
  seph.sva = sbsmsg_getbitu(msg, 35, 4);
  seph.svh = seph.sva == 15 ? 1 : 0; /* Unhealthy if ura==15 */

  seph.pos[0] = sbsmsg_getbits(msg, 39, 30) * 0.08L;
  seph.pos[1] = sbsmsg_getbits(msg, 69, 30) * 0.08L;
  seph.pos[2] = sbsmsg_getbits(msg, 99, 25) * 0.4L;
  seph.vel[0] = sbsmsg_getbits(msg, 124, 17) * 0.000625L;
  seph.vel[1] = sbsmsg_getbits(msg, 141, 17) * 0.000625L;
  seph.vel[2] = sbsmsg_getbits(msg, 158, 18) * 0.004L;
  seph.acc[0] = sbsmsg_getbits(msg, 176, 10) * 0.0000125L;
  seph.acc[1] = sbsmsg_getbits(msg, 186, 10) * 0.0000125L;
  seph.acc[2] = sbsmsg_getbits(msg, 196, 10) * 0.0000625L;

  seph.af0 = sbsmsg_getbits(msg, 206, 12) * P2_31;
  seph.af1 = sbsmsg_getbits(msg, 218, 8) * P2_39 / 2.0L;

  int i = msg->prn - MINPRNSBS;
  if (!nav->seph[i] || fabsl(timediff(nav->seph[i][0].t0, seph.t0)) < 1E-3L) {
    /* Not change */
    return false;
  }
  nav->seph[i][1] = nav->seph[i][0]; /* Previous */
  nav->seph[i][0] = seph;            /* Current */

  trace(5, "decode_sbstype9: prn=%d\n", msg->prn);
  return true;
}
/* Decode type 18: ionospheric grid point masks ------------------------------*/
static bool decode_sbstype18(const sbsmsg_t *msg, sbsion_t *sbsion) {
  trace(4, "decode_sbstype18:\n");

  int band = sbsmsg_getbitu(msg, 18, 4);
  const sbsigpband_t *p;
  int m;
  if (0 <= band && band <= 8) {
    p = igpband1[band];
    m = 8;
  } else if (9 <= band && band <= 10) {
    p = igpband2[band - 9];
    m = 5;
  } else
    return false;

  sbsion[band].iodi = (int16_t)sbsmsg_getbitu(msg, 22, 2);

  int n = 0;
  for (int i = 1; i <= 201; i++) {
    if (!sbsmsg_getbitu(msg, 23 + i, 1)) continue;
    for (int j = 0; j < m; j++) {
      if (i < p[j].bits || p[j].bite < i) continue;
      sbsion[band].igp[n].lat = band <= 8 ? p[j].y[i - p[j].bits] : p[j].x;
      sbsion[band].igp[n++].lon = band <= 8 ? p[j].x : p[j].y[i - p[j].bits];
      break;
    }
  }
  sbsion[band].nigp = n;

  trace(5, "decode_sbstype18: band=%d nigp=%d\n", band, n);
  return true;
}
/* Decode half long term correction (vel code=0) -----------------------------*/
static bool decode_longcorr0(const sbsmsg_t *msg, int p, sbssat_t *sbssat) {
  trace(4, "decode_longcorr0:\n");

  int n = sbsmsg_getbitu(msg, p, 6);
  if (n == 0 || n > MAXSAT) return false;

  sbssat->sat[n - 1].lcorr.iode = sbsmsg_getbitu(msg, p + 6, 8);

  for (int i = 0; i < 3; i++) {
    sbssat->sat[n - 1].lcorr.dpos[i] = sbsmsg_getbits(msg, p + 14 + 9 * i, 9) * 0.125L;
    sbssat->sat[n - 1].lcorr.dvel[i] = 0.0L;
  }
  sbssat->sat[n - 1].lcorr.daf0 = sbsmsg_getbits(msg, p + 41, 10) * P2_31;
  sbssat->sat[n - 1].lcorr.daf1 = 0.0L;
  sbssat->sat[n - 1].lcorr.t0 = gpst2time(msg->week, msg->tow);

  trace(5, "decode_longcorr0:sat=%2d\n", sbssat->sat[n - 1].sat);
  return true;
}
/* Decode half long term correction (vel code=1) -----------------------------*/
static bool decode_longcorr1(const sbsmsg_t *msg, int p, sbssat_t *sbssat) {
  trace(4, "decode_longcorr1:\n");

  int n = sbsmsg_getbitu(msg, p, 6);
  if (n == 0 || n > MAXSAT) return false;

  sbssat->sat[n - 1].lcorr.iode = sbsmsg_getbitu(msg, p + 6, 8);

  for (int i = 0; i < 3; i++) {
    sbssat->sat[n - 1].lcorr.dpos[i] = sbsmsg_getbits(msg, p + 14 + i * 11, 11) * 0.125L;
    sbssat->sat[n - 1].lcorr.dvel[i] = sbsmsg_getbits(msg, p + 58 + i * 8, 8) * P2_11;
  }
  sbssat->sat[n - 1].lcorr.daf0 = sbsmsg_getbits(msg, p + 47, 11) * P2_31;
  sbssat->sat[n - 1].lcorr.daf1 = sbsmsg_getbits(msg, p + 82, 8) * P2_39;
  int t = (int)sbsmsg_getbitu(msg, p + 90, 13) * 16 - (int)msg->tow % 86400;
  if (t <= -43200)
    t += 86400;
  else if (t > 43200)
    t -= 86400;
  sbssat->sat[n - 1].lcorr.t0 = gpst2time(msg->week, msg->tow + t);

  trace(5, "decode_longcorr1: sat=%2d\n", sbssat->sat[n - 1].sat);
  return true;
}
/* Decode half long term correction ------------------------------------------*/
static bool decode_longcorrh(const sbsmsg_t *msg, int p, sbssat_t *sbssat) {
  trace(4, "decode_longcorrh:\n");

  if (sbsmsg_getbitu(msg, p, 1) == 0) { /* Vel code=0 */
    if (sbssat->iodp == (int)sbsmsg_getbitu(msg, p + 103, 2)) {
      return decode_longcorr0(msg, p + 1, sbssat) && decode_longcorr0(msg, p + 52, sbssat);
    }
  } else if (sbssat->iodp == (int)sbsmsg_getbitu(msg, p + 104, 2)) {
    return decode_longcorr1(msg, p + 1, sbssat);
  }
  return false;
}
/* Decode type 24: mixed fast/long term correction ---------------------------*/
static bool decode_sbstype24(const sbsmsg_t *msg, sbssat_t *sbssat) {
  trace(4, "decode_sbstype24:\n");

  if (sbssat->iodp != (int)sbsmsg_getbitu(msg, 110, 2)) return false; /* Check IODP */

  int blk = sbsmsg_getbitu(msg, 112, 2);
  int iodf = sbsmsg_getbitu(msg, 114, 2);

  for (int i = 0; i < 6; i++) {
    int j = 13 * blk + i;
    if (j >= sbssat->nsat) break;
    int udre = sbsmsg_getbitu(msg, 86 + 4 * i, 4);

    sbssat->sat[j].fcorr.t0 = gpst2time(msg->week, msg->tow);
    sbssat->sat[j].fcorr.prc = sbsmsg_getbits(msg, 14 + i * 12, 12) * 0.125L;
    sbssat->sat[j].fcorr.udre = udre + 1;
    sbssat->sat[j].fcorr.iodf = iodf;
  }
  return decode_longcorrh(msg, 120, sbssat);
}
/* Decode type 25: long term satellite error correction ----------------------*/
static bool decode_sbstype25(const sbsmsg_t *msg, sbssat_t *sbssat) {
  trace(4, "decode_sbstype25:\n");

  return decode_longcorrh(msg, 14, sbssat) && decode_longcorrh(msg, 120, sbssat);
}
/* Decode type 26: ionospheric deley corrections -----------------------------*/
static bool decode_sbstype26(const sbsmsg_t *msg, sbsion_t *sbsion) {
  trace(4, "decode_sbstype26:\n");

  int band = sbsmsg_getbitu(msg, 14, 4);
  if (band > MAXBAND || sbsion[band].iodi != (int)sbsmsg_getbitu(msg, 217, 2)) return false;

  int block = sbsmsg_getbitu(msg, 18, 4);

  for (int i = 0; i < 15; i++) {
    int j = block * 15 + i;
    if (j >= sbsion[band].nigp) continue;
    int give = sbsmsg_getbitu(msg, 22 + i * 13 + 9, 4);

    int delay = sbsmsg_getbitu(msg, 22 + i * 13, 9);
    sbsion[band].igp[j].t0 = gpst2time(msg->week, msg->tow);
    sbsion[band].igp[j].delay = delay == 0x1FF ? 0.0L : delay * 0.125L;
    sbsion[band].igp[j].give = give + 1;

    if (sbsion[band].igp[j].give >= 16) {
      sbsion[band].igp[j].give = 0;
    }
  }
  trace(5, "decode_sbstype26: band=%d block=%d\n", band, block);
  return true;
}
/* Update SBAS corrections -----------------------------------------------------
 * Update SBAS correction parameters in navigation data with a SBAS message
 * Args   : sbsmg_t  *msg    I   SBAS message
 *          nav_t    *nav    IO  navigation data
 * Return : message type (-1: error or not supported type)
 * Notes  : nav->seph must point to seph[NSATSBS][2] (array of seph_t)
 *               seph[prn-MINPRNSBS+1][0]       : sat prn current epehmeris
 *               seph[prn-MINPRNSBS+1][1]       : sat prn previous epehmeris
 *----------------------------------------------------------------------------*/
extern int sbsupdatecorr(const sbsmsg_t *msg, nav_t *nav) {
  int type = sbsmsg_getbitu(msg, 8, 6);
  trace(3, "sbsupdatecorr: type=%d\n", type);

  if (msg->week == 0) return -1;

  bool stat = false;
  switch (type) {
    case 0:
      stat = decode_sbstype2(msg, &nav->sbssat);
      break;
    case 1:
      stat = decode_sbstype1(msg, &nav->sbssat);
      break;
    case 2:
    case 3:
    case 4:
    case 5:
      stat = decode_sbstype2(msg, &nav->sbssat);
      break;
    case 6:
      stat = decode_sbstype6(msg, &nav->sbssat);
      break;
    case 7:
      stat = decode_sbstype7(msg, &nav->sbssat);
      break;
    case 9:
      stat = decode_sbstype9(msg, nav);
      break;
    case 18:
      stat = decode_sbstype18(msg, nav->sbsion);
      break;
    case 24:
      stat = decode_sbstype24(msg, &nav->sbssat);
      break;
    case 25:
      stat = decode_sbstype25(msg, &nav->sbssat);
      break;
    case 26:
      stat = decode_sbstype26(msg, nav->sbsion);
      break;
    case 63:
      break; /* Null message */

      /*default: trace(2,"unsupported SBAS message: type=%d\n",type); break;*/
  }
  return stat ? type : -1;
}
/* Read SBAS log file --------------------------------------------------------*/
static void readmsgs(const char *file, int sel, gtime_t ts, gtime_t te, sbs_t *sbs) {
  trace(3, "readmsgs: file=%s sel=%d\n", file, sel);

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "sbas message file open error: %s\n", file);
    return;
  }
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    int week, prn;
    long double tow;
    long double ep[6] = {0};
    int msg;
    int pi;
    if (sscanf(buff, "%d %Lf %d", &week, &tow, &prn) == 3 && (pi = strstri(buff, 0, ": ")) >= 0) {
      pi += 2; /* RTRLIB form */
    } else if (sscanf(buff, "%d %Lf %Lf %Lf %Lf %Lf %Lf %d", &prn, ep, ep + 1, ep + 2, ep + 3,
                      ep + 4, ep + 5, &msg) == 8) {
      /* EMS (EGNOS Message Service) form */
      ep[0] += ep[0] < 70.0L ? 2000.0L : 1900.0L;
      tow = time2gpst(epoch2time(ep), &week);
      pi = msg >= 10 ? 25 : 24;
    } else if (!strncmp(buff, "#RAWWAASFRAMEA", 14)) { /* NovAtel OEM4/V */
      pi = getfield(buff, 0, 6);
      if (pi < 0) continue;
      if (sscanf(buff + pi, "%d,%Lf", &week, &tow) < 2) continue;
      pi = strchri(buff, pi + 1, ';');
      if (pi < 0) continue;
      pi++;
      int ch;
      if (sscanf(buff + pi, "%d,%d", &ch, &prn) < 2) continue;
      pi = getfield(buff, pi, 4);
      if (pi < 0) continue;
    } else if (!strncmp(buff, "$FRMA", 5)) { /* NovAtel OEM3 */
      pi = getfield(buff, 0, 2);
      if (pi < 0) continue;
      if (sscanf(buff + pi, "%d,%Lf,%d", &week, &tow, &prn) < 3) continue;
      pi = getfield(buff, pi, 6);
      if (pi < 0) continue;
      if (week < WEEKOFFSET) week += WEEKOFFSET;
    } else
      continue;

    if (sel != 0 && sel != prn) continue;

    gtime_t time = gpst2time(week, tow);

    if (!screent(time, ts, te, 0.0L)) continue;

    if (sbs->n >= sbs->nmax) {
      sbs->nmax = sbs->nmax == 0 ? 1024 : sbs->nmax * 2;
      sbsmsg_t *sbs_msgs = (sbsmsg_t *)realloc(sbs->msgs, sbs->nmax * sizeof(sbsmsg_t));
      if (!sbs_msgs) {
        trace(1, "readsbsmsg malloc error: nmax=%d\n", sbs->nmax);
        free(sbs->msgs);
        sbs->msgs = NULL;
        sbs->n = sbs->nmax = 0;
        return;
      }
      sbs->msgs = sbs_msgs;
    }
    sbs->msgs[sbs->n].week = week;
    sbs->msgs[sbs->n].tow = (int)(tow + 0.5L);
    sbs->msgs[sbs->n].prn = prn;
    for (int i = 0; i < 29; i++) sbs->msgs[sbs->n].msg[i] = 0;
    for (int i = 0; buff[pi - 1] && buff[pi] && i < 29; pi += 2, i++) {
      uint32_t b;
      if (sscanf(buff + pi, "%2X", &b) == 1) sbs->msgs[sbs->n].msg[i] = (uint8_t)b;
    }
    sbs->msgs[sbs->n++].msg[28] &= 0xC0;
  }
  fclose(fp);
}
/* Compare SBAS messages -----------------------------------------------------*/
static int cmpmsgs(const void *p1, const void *p2) {
  const sbsmsg_t *q1 = (sbsmsg_t *)p1, *q2 = (sbsmsg_t *)p2;
  return q1->week != q2->week
             ? q1->week - q2->week
             : (q1->tow < q2->tow ? -1 : (q1->tow > q2->tow ? 1 : q1->prn - q2->prn));
}
/* Read SBAS message file ------------------------------------------------------
 * Read SBAS message file
 * Args   : char     *file   I   SBAS message file (wind-card * is expanded)
 *          int      sel     I   SBAS satellite prn number selection (0:all)
 *         (gtime_t  ts      I   start time)
 *         (gtime_t  te      I   end time  )
 *          sbs_t    *sbs    IO  SBAS messages
 * Return : number of SBAS messages
 * Notes  : SBAS message are appended and sorted. before calling the funciton,
 *          sbs->n, sbs->nmax and sbs->msgs must be set properly. (initially
 *          sbs->n=sbs->nmax=0, sbs->msgs=NULL)
 *          only the following file extentions after wild card expanded are valid
 *          to read. others are skipped
 *          .sbs, .SBS, .ems, .EMS
 *----------------------------------------------------------------------------*/
extern int sbsreadmsgt(const char *file, int sel, gtime_t ts, gtime_t te, sbs_t *sbs) {
  trace(3, "sbsreadmsgt: file=%s sel=%d\n", file, sel);

  char *efiles[MAXEXFILE] = {0};
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(efiles[i] = (char *)malloc(FNSIZE))) {
      for (i--; i >= 0; i--) free(efiles[i]);
      return 0;
    }
  }
  /* Expand wild card in file path */
  int n = expath(file, efiles, FNSIZE, MAXEXFILE);

  for (int i = 0; i < n; i++) {
    const char *ext = strrchr(efiles[i], '.');
    if (!ext) continue;
    if (strcmp(ext, ".sbs") && strcmp(ext, ".SBS") && strcmp(ext, ".ems") && strcmp(ext, ".EMS"))
      continue;

    readmsgs(efiles[i], sel, ts, te, sbs);
  }
  for (int i = 0; i < MAXEXFILE; i++) free(efiles[i]);

  /* Sort messages */
  if (sbs->n > 0) {
    qsort(sbs->msgs, sbs->n, sizeof(sbsmsg_t), cmpmsgs);
  }
  return sbs->n;
}
extern int sbsreadmsg(const char *file, int sel, sbs_t *sbs) {
  gtime_t ts = {0}, te = {0};

  trace(3, "sbsreadmsg: file=%s sel=%d\n", file, sel);

  return sbsreadmsgt(file, sel, ts, te, sbs);
}
/* Output SBAS messages --------------------------------------------------------
 * Output SBAS message record to output file in RTKLIB SBAS log format
 * Args   : FILE   *fp       I   output file pointer
 *          sbsmsg_t *sbsmsg I   SBAS messages
 * Return : none
 *----------------------------------------------------------------------------*/
extern void sbsoutmsg(FILE *fp, sbsmsg_t *sbsmsg) {
  trace(4, "sbsoutmsg:\n");

  int prn = sbsmsg->prn, type = sbsmsg->msg[1] >> 2;
  fprintf(fp, "%4d %6d %3d %2d : ", sbsmsg->week, sbsmsg->tow, prn, type);
  for (int i = 0; i < 29; i++) fprintf(fp, "%02X", sbsmsg->msg[i]);
  fprintf(fp, "\n");
}
/* Search igps ---------------------------------------------------------------*/
static void searchigp(gtime_t time, const long double *pos, const sbsion_t *ion,
                      const sbsigp_t **igp, long double *x, long double *y) {
  trace(4, "searchigp: pos=%.3Lf %.3Lf\n", pos[0] * R2D, pos[1] * R2D);

  long double lat = pos[0] * R2D, lon = pos[1] * R2D;
  int latp[2], lonp[4];
  if (lon >= 180.0L) lon -= 360.0L;
  if (-55.0L <= lat && lat < 55.0L) {
    latp[0] = (int)floorl(lat / 5.0L) * 5;
    latp[1] = latp[0] + 5;
    lonp[0] = lonp[1] = (int)floorl(lon / 5.0L) * 5;
    lonp[2] = lonp[3] = lonp[0] + 5;
    *x = (lon - lonp[0]) / 5.0L;
    *y = (lat - latp[0]) / 5.0L;
  } else {
    latp[0] = (int)floorl((lat - 5.0L) / 10.0L) * 10 + 5;
    latp[1] = latp[0] + 10;
    lonp[0] = lonp[1] = (int)floorl(lon / 10.0L) * 10;
    lonp[2] = lonp[3] = lonp[0] + 10;
    *x = (lon - lonp[0]) / 10.0L;
    *y = (lat - latp[0]) / 10.0L;
    if (75.0L <= lat && lat < 85.0L) {
      lonp[1] = (int)floorl(lon / 90.0L) * 90;
      lonp[3] = lonp[1] + 90;
    } else if (-85.0L <= lat && lat < -75.0L) {
      lonp[0] = (int)floorl((lon - 50.0L) / 90.0L) * 90 + 40;
      lonp[2] = lonp[0] + 90;
    } else if (lat >= 85.0L) {
      for (int i = 0; i < 4; i++) lonp[i] = (int)floorl(lon / 90.0L) * 90;
    } else if (lat < -85.0L) {
      for (int i = 0; i < 4; i++) lonp[i] = (int)floorl((lon - 50.0L) / 90.0L) * 90 + 40;
    }
  }
  for (int i = 0; i < 4; i++)
    if (lonp[i] == 180) lonp[i] = -180;
  for (int i = 0; i <= MAXBAND; i++) {
    for (const sbsigp_t *p = ion[i].igp; p < ion[i].igp + ion[i].nigp; p++) {
      if (p->t0.time == 0) continue;
      if (p->lat == latp[0] && p->lon == lonp[0] && p->give > 0)
        igp[0] = p;
      else if (p->lat == latp[1] && p->lon == lonp[1] && p->give > 0)
        igp[1] = p;
      else if (p->lat == latp[0] && p->lon == lonp[2] && p->give > 0)
        igp[2] = p;
      else if (p->lat == latp[1] && p->lon == lonp[3] && p->give > 0)
        igp[3] = p;
      if (igp[0] && igp[1] && igp[2] && igp[3]) return;
    }
  }
}
/* SBAS ionospheric delay correction -------------------------------------------
 * Compute SBAS ionosphric delay correction
 * Args   : gtime_t  time    I   time
 *          nav_t    *nav    I   navigation data
 *          long double   *pos    I   receiver position {lat,lon,height} (rad/m)
 *          long double   *azel   I   satellite azimuth/elavation angle (rad)
 *          long double   *delay  O   slant ionospheric delay (L1) (m)
 *          long double   *var    O   variance of ionospheric delay (m^2)
 * Return : status (true:ok,false:no correction)
 * Notes  : before calling the function, SBAS ionosphere correction parameters
 *          in navigation data (nav->sbsion) must be set by calling
 *          sbsupdatecorr()
 *----------------------------------------------------------------------------*/
extern bool sbsioncorr(gtime_t time, const nav_t *nav, const long double *pos,
                       const long double *azel, long double *delay, long double *var) {
  trace(4, "sbsioncorr: pos=%.3Lf %.3Lf azel=%.3Lf %.3Lf\n", pos[0] * R2D, pos[1] * R2D,
        azel[0] * R2D, azel[1] * R2D);

  *delay = *var = 0.0L;
  if (pos[2] < -100.0L || azel[1] <= 0) return true;

  /* IPP (ionospheric pierce point) position */
  const long double re = 6378.1363L, hion = 350.0L;
  long double posp[2];
  long double fp = ionppp(pos, azel, re, hion, posp);

  /* Search igps around IPP */
  const sbsigp_t *igp[4] = {0}; /* {ws,wn,es,en} */
  long double x = 0.0L, y = 0.0L;
  searchigp(time, posp, nav->sbsion, igp, &x, &y);

  /* Weight of igps */
  int err = 0;
  long double w[4] = {0};
  if (igp[0] && igp[1] && igp[2] && igp[3]) {
    w[0] = (1.0L - x) * (1.0L - y);
    w[1] = (1.0L - x) * y;
    w[2] = x * (1.0L - y);
    w[3] = x * y;
  } else if (igp[0] && igp[1] && igp[2]) {
    w[1] = y;
    w[2] = x;
    if ((w[0] = 1.0L - w[1] - w[2]) < 0.0L) err = 1;
  } else if (igp[0] && igp[2] && igp[3]) {
    w[0] = 1.0L - x;
    w[3] = y;
    if ((w[2] = 1.0L - w[0] - w[3]) < 0.0L) err = 1;
  } else if (igp[0] && igp[1] && igp[3]) {
    w[0] = 1.0L - y;
    w[3] = x;
    if ((w[1] = 1.0L - w[0] - w[3]) < 0.0L) err = 1;
  } else if (igp[1] && igp[2] && igp[3]) {
    w[1] = 1.0L - x;
    w[2] = 1.0L - y;
    if ((w[3] = 1.0L - w[1] - w[2]) < 0.0L) err = 1;
  } else
    err = 1;

  if (err) {
    trace(2, "no sbas iono correction: lat=%3.0Lf lon=%4.0Lf\n", posp[0] * R2D, posp[1] * R2D);
    return false;
  }
  for (int i = 0; i < 4; i++) {
    if (!igp[i]) continue;
    long double t = timediff(time, igp[i]->t0);
    *delay += w[i] * igp[i]->delay;
    *var += w[i] * varicorr(igp[i]->give) * 9E-8L * fabsl(t);
  }
  *delay *= fp;
  *var *= fp * fp;

  trace(5, "sbsioncorr: dion=%7.2Lf sig=%7.2Lf\n", *delay, sqrtl(*var));
  return true;
}
/* Get meterological parameters ----------------------------------------------*/
static void getmet(long double lat, long double *met) {
  static const long double metprm[][10] = {
      /* lat=15,30,45,60,75 */
      {1013.25L, 299.65L, 26.31L, 6.30E-3L, 2.77L, 0.00L, 0.00L, 0.00L, 0.00E-3L, 0.00L},
      {1017.25L, 294.15L, 21.79L, 6.05E-3L, 3.15L, -3.75L, 7.00L, 8.85L, 0.25E-3L, 0.33L},
      {1015.75L, 283.15L, 11.66L, 5.58E-3L, 2.57L, -2.25L, 11.00L, 7.24L, 0.32E-3L, 0.46L},
      {1011.75L, 272.15L, 6.78L, 5.39E-3L, 1.81L, -1.75L, 15.00L, 5.36L, 0.81E-3L, 0.74L},
      {1013.00L, 263.65L, 4.11L, 4.53E-3L, 1.55L, -0.50L, 14.50L, 3.39L, 0.62E-3L, 0.30L}};
  lat = fabsl(lat);
  if (lat <= 15.0L)
    for (int i = 0; i < 10; i++) met[i] = metprm[0][i];
  else if (lat >= 75.0L)
    for (int i = 0; i < 10; i++) met[i] = metprm[4][i];
  else {
    int j = (int)(lat / 15.0L);
    long double a = (lat - j * 15.0L) / 15.0L;
    for (int i = 0; i < 10; i++) met[i] = (1.0L - a) * metprm[j - 1][i] + a * metprm[j][i];
  }
}
/* Tropospheric delay correction -----------------------------------------------
 * Compute SBAS tropospheric delay correction (mops model)
 * Args   : gtime_t time     I   time
 *          long double   *pos    I   receiver position {lat,lon,height} (rad/m)
 *          long double   *azel   I   satellite azimuth/elavation (rad)
 *          long double   *var    O   variance of troposphric error (m^2)
 * Return : slant tropospheric delay (m)
 *----------------------------------------------------------------------------*/
extern long double sbstropcorr(gtime_t time, const long double *pos, const long double *azel,
                               long double *var) {
  trace(4, "sbstropcorr: pos=%.3Lf %.3Lf azel=%.3Lf %.3Lf\n", pos[0] * R2D, pos[1] * R2D,
        azel[0] * R2D, azel[1] * R2D);

  if (pos[2] < -100.0L || 10000.0L < pos[2] || azel[1] <= 0) {
    *var = 0.0L;
    return 0.0L;
  }
  static long double pos_[3] = {0}, zh = 0.0L, zw = 0.0L;
  const long double k1 = 77.604L, k2 = 382000.0L, rd = 287.054L, gm = 9.784L, g = 9.80665L;
  long double sinel = sinl(azel[1]), h = pos[2];
  if (zh == 0.0L || fabsl(pos[0] - pos_[0]) > 1E-7L || fabsl(pos[1] - pos_[1]) > 1E-7L ||
      fabsl(pos[2] - pos_[2]) > 1.0L) {
    long double met[10];
    getmet(pos[0] * R2D, met);
    long double c =
        cosl(2.0L * PI * (time2doy(time) - (pos[0] >= 0.0L ? 28.0L : 211.0L)) / 365.25L);
    for (int i = 0; i < 5; i++) met[i] -= met[i + 5] * c;
    zh = 1E-6L * k1 * rd * met[0] / gm;
    zw = 1E-6L * k2 * rd / (gm * (met[4] + 1.0L) - met[3] * rd) * met[2] / met[1];
    zh *= powl(1.0L - met[3] * h / met[1], g / (rd * met[3]));
    zw *= powl(1.0L - met[3] * h / met[1], (met[4] + 1.0L) * g / (rd * met[3]) - 1.0L);
    for (int i = 0; i < 3; i++) pos_[i] = pos[i];
  }
  long double m = 1.001L / sqrtl(0.002001L + sinel * sinel);
  *var = 0.12L * 0.12L * m * m;
  return (zh + zw) * m;
}
/* Long term correction ------------------------------------------------------*/
static bool sbslongcorr(gtime_t time, int sat, const sbssat_t *sbssat, long double *drs,
                        long double *ddts) {
  trace(3, "sbslongcorr: sat=%2d\n", sat);

  for (const sbssatp_t *p = sbssat->sat; p < sbssat->sat + sbssat->nsat; p++) {
    if (p->sat != sat || p->lcorr.t0.time == 0) continue;
    long double t = timediff(time, p->lcorr.t0);
    if (fabsl(t) > MAXSBSAGEL) {
      char tstr[40];
      trace(2, "sbas long-term correction expired: %s sat=%2d t=%5.0Lf\n", time2str(time, tstr, 0),
            sat, t);
      return false;
    }
    for (int i = 0; i < 3; i++) drs[i] = p->lcorr.dpos[i] + p->lcorr.dvel[i] * t;
    *ddts = p->lcorr.daf0 + p->lcorr.daf1 * t;

    trace(5, "sbslongcorr: sat=%2d drs=%7.2Lf%7.2Lf%7.2Lf ddts=%7.2Lf\n", sat, drs[0], drs[1],
          drs[2], *ddts * CLIGHT);

    return true;
  }
  /* If SBAS satellite without correction, no correction applied */
  if (satsys(sat, NULL) == SYS_SBS) return true;

  char tstr[40];
  trace(2, "no sbas long-term correction: %s sat=%2d\n", time2str(time, tstr, 0), sat);
  return false;
}
/* Fast correction -----------------------------------------------------------*/
static bool sbsfastcorr(gtime_t time, int sat, const sbssat_t *sbssat, long double *prc,
                        long double *var) {
  trace(3, "sbsfastcorr: sat=%2d\n", sat);

  for (const sbssatp_t *p = sbssat->sat; p < sbssat->sat + sbssat->nsat; p++) {
    if (p->sat != sat) continue;
    if (p->fcorr.t0.time == 0) break;
    long double t = timediff(time, p->fcorr.t0) + sbssat->tlat;

    /* Expire age of correction or UDRE==14 (not monitored) */
    if (fabsl(t) > MAXSBSAGEF || p->fcorr.udre >= 15) continue;
    *prc = p->fcorr.prc;
#ifdef RRCENA
    if (p->fcorr.ai > 0 && fabsl(t) <= 8.0 * p->fcorr.dt) {
      *prc += p->fcorr.rrc * t;
    }
#endif
    *var = varfcorr(p->fcorr.udre) + degfcorr(p->fcorr.ai) * t * t / 2.0L;

    trace(5, "sbsfastcorr: sat=%3d prc=%7.2Lf sig=%7.2Lf t=%5.0Lf\n", sat, *prc, sqrtl(*var), t);
    return true;
  }
  char tstr[40];
  trace(2, "no sbas fast correction: %s sat=%2d\n", time2str(time, tstr, 0), sat);
  return false;
}
/* SBAS satellite ephemeris and clock correction -------------------------------
 * Correct satellite position and clock bias with SBAS satellite corrections
 * Args   : gtime_t time     I   reception time
 *          int    sat       I   satellite
 *          nav_t  *nav      I   navigation data
 *          long double *rs       IO  sat position and corrected {x,y,z} (ECEF) (m)
 *          long double *dts      IO  sat clock bias and corrected (s)
 *          long double *var      O   sat position and clock variance (m^2)
 * Return : status (true:ok,false:no correction)
 * Notes  : before calling the function, SBAS satellite correction parameters
 *          in navigation data (nav->sbssat) must be set by calling
 *          sbsupdatecorr().
 *          satellite clock correction include long-term correction and fast
 *          correction.
 *          SBAS clock correction is usually based on L1C/A code. TGD or DCB has
 *          to be considered for other codes
 *----------------------------------------------------------------------------*/
extern bool sbssatcorr(gtime_t time, int sat, const nav_t *nav, long double *rs, long double *dts,
                       long double *var) {
  trace(3, "sbssatcorr : sat=%2d\n", sat);

  /* SBAS long term corrections */
  long double drs[3] = {0}, dclk = 0.0L;
  if (!sbslongcorr(time, sat, &nav->sbssat, drs, &dclk)) {
    return false;
  }
  /* SBAS fast corrections */
  long double prc = 0.0L;
  if (!sbsfastcorr(time, sat, &nav->sbssat, &prc, var)) {
    return false;
  }
  for (int i = 0; i < 3; i++) rs[i] += drs[i];

  dts[0] += dclk + prc / CLIGHT;

  trace(4, "sbssatcorr: sat=%2d drs=%6.3Lf %6.3Lf %6.3Lf dclk=%.3Lf %.3Lf var=%.3Lf\n", sat, drs[0],
        drs[1], drs[2], dclk, prc / CLIGHT, *var);

  return true;
}
/* Decode SBAS message ---------------------------------------------------------
 * Decode SBAS message frame words and check crc
 * Args   : gtime_t time     I   reception time
 *          int    prn       I   SBAS satellite prn number
 *          uint32_t *word   I   message frame words (24bit x 10)
 *          sbsmsg_t *sbsmsg O   SBAS message
 * Return : status (true:ok,false:crc error)
 *----------------------------------------------------------------------------*/
extern bool sbsdecodemsg(gtime_t time, int prn, const uint32_t *words, sbsmsg_t *sbsmsg) {
  trace(5, "sbsdecodemsg: prn=%d\n", prn);

  if (time.time == 0) return false;
  long double tow = time2gpst(time, &sbsmsg->week);
  sbsmsg->tow = (int)(tow + DTTOL);
  sbsmsg->prn = prn;
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 4; j++) {
      sbsmsg->msg[i * 4 + j] = (uint8_t)(words[i] >> ((3 - j) * 8));
    }
  sbsmsg->msg[28] = (uint8_t)(words[7] >> 18) & 0xC0;
  uint8_t f[29];
  for (int i = 28; i > 0; i--) f[i] = (sbsmsg->msg[i] >> 6) + (sbsmsg->msg[i - 1] << 2);
  f[0] = sbsmsg->msg[0] >> 6;

  return rtk_crc24q(f, sizeof(f), 29) == (words[7] & 0xFFFFFF); /* Check crc */
}
