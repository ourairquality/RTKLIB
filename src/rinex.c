/*------------------------------------------------------------------------------
 * rinex.c : RINEX functions
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Reference :
 *     [1] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
 *         Version 2.11, December 10, 2007
 *     [2] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
 *         Version 3.00, November 28, 2007
 *     [3] IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
 *         7 March, 2006
 *     [4] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
 *         Version 2.12, June 23, 2009
 *     [5] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
 *         Version 3.01, June 22, 2009
 *     [6] J.Ray and W.Gurtner, RINEX extensions to handle clock information
 *         version 3.02, September 2, 2010
 *     [7] RINEX The Receiver Independent Exchange Format Version 3.02,
 *         International GNSS Service (IGS), RINEX Working Group and Radio
 *         Technical Commission for Maritime Services Special Committee 104
 *         (RTCM-SC104), December 10, 2012
 *     [8] RINEX The Receiver Independent Exchange Format Version 3.03,
 *         International GNSS Service (IGS), RINEX Working Group and Radio
 *         Technical Commission for Maritime Services Special Committee 104
 *         (RTCM-SC104), July 14, 2015
 *     [9] RINEX The Receiver Independent Exchange Format Version 3.04,
 *         International GNSS Service (IGS), RINEX Working Group and Radio
 *         Technical Commission for Maritime Services Special Committee 104
 *         (RTCM-SC104), November 23, 2018
 *
 * Version : $Revision:$
 * History : 2006/01/16 1.0  new
 *           2007/03/14 1.1  read P1 if no obstype of C1
 *           2007/04/27 1.2  add readrnxt() function
 *           2007/05/25 1.3  add support of file path with wild-card (*)
 *                           add support of compressed files
 *           2007/11/02 1.4  support SBAS/geo satellite
 *                           support doppler observables
 *                           support RINEX bug of week handover
 *                           add RINEX obs/nav output functions
 *           2008/06/16 1.5  export readrnxf(), add compress()
 *                           separate sortobs(), uniqeph(), screent()
 *           2008/10/28 1.6  fix bug on reading RINEX obs header types of observ
 *           2009/04/09 1.7  support RINEX 2.11
 *                           change api of outrnxobsh(),outrnxobsb(),outrnxnavb()
 *           2009/06/02 1.8  add api outrnxgnavb()
 *           2009/08/15 1.9  support GLONASS
 *                           add slip save/restore functions
 *           2010/03/03 1.10 fix bug of array access by disabled satellite
 *           2010/07/21 1.11 support RINEX ver.2.12, 3.00
 *                           support RINEX extension for QZSS
 *                           support geo navigation messages
 *                           added api:
 *                               setrnxcodepri(),outrnxhnavh(),outrnxhnavb(),
 *                           changed api:
 *                               readrnx(),readrnxt(),outrnxnavh(),outrnxgnavh()
 *           2010/05/29 1.12 fix bug on skipping invalid satellite data
 *                           fix bug on frequency number overflow
 *                           output P1 instead of C1 if rnxopt.rcvopt=-L1P
 *                           output C2 instead of P2 if rnxopt.rcvopt=-L2C
 *                           change api:
 *                               outrnxgnavh(),outrnxhnavh(),readrnx(),
 *                               readrnxt()
 *                           add api:
 *                               outrnxlnavh(), outrnxqnav()
 *                           move uniqeph(),uniqgeph,uniqseph()
 *           2010/08/19 1.13 suppress warning
 *           2012/03/01 1.14 add function to read cnes widelane fcb in rnxclk
 *                           support compass RINEX nav
 *                           change api: setcodepri()
 *           2012/10/17 1.15 support ver.2.12, ver.3.01
 *                           add api init_rnxctr(),free_rnxctr(),open_rnxctr(),
 *                           input_rnxctr()
 *                           change api readrnxt(),readrnx()
 *                           delete api setrnxcodepri()
 *                           fix bug on message frame time in v.3 GLONASS nav
 *           2013/02/09 1.16 add reading geph.iode derived from toe
 *           2013/02/23 1.17 support RINEX 3.02 (ref [7])
 *                           change api outrnxobsh()
 *                           add api outrnxcnavh()
 *                           fix bug on output of fit interval
 *           2013/05/08 1.18 fix bug on reading GLO and geo nav in RINEX 3
 *           2013/09/01 1.19 fix bug on reading Galileo "C1" in RINEX 2.12
 *           2013/12/16 1.20 reject C1 for 2.12
 *           2014/05/26 1.21 fix bug on reading GPS "C2" in RINEX 2.11 or 2.12
 *                           fix problem on type incompatibility
 *                           support BeiDou
 *           2014/08/29 1.22 fix bug on reading GPS "C2" in RINEX 2.11 or 2.12
 *           2014/10/20 1.23 recognize "C2" in 2.12 as "C2W" instead of "C2D"
 *           2014/12/07 1.24 add read RINEX option -SYS=...
 *           2016/07/01 1.25 support RINEX 3.03
 *                           support IRNSS
 *           2016/09/17 1.26 fix bug on fit interval in QZSS RINEX nav
 *                           URA output value compliant to RINEX 3.03
 *           2016/10/10 1.27 add api outrnxinavh()
 *           2018/10/10 1.28 support Galileo sisa value for RINEX nav output
 *                           fix bug on handling BeiDou B1 code in RINEX 3.03
 *           2019/08/19 1.29 support Galileo sisa index for RINEX nav input
 *           2020/11/30 1.30 support RINEX 3.04 (ref [9])
 *                           support phase shift in RINEX options rnxopt_t
 *                           support high-resolution (16bit) C/N0 in obsd_t
 *                           support dual sets of ephemerides in RINEX control
 *                             (for Galileo I/NAV and F/NAV)
 *                           no support RINEX 2 NAV extensions (QZS and BDS)
 *                           no support CNES/GRG clock extension in comments
 *                           fix bug on segfault to read NavIC/IRNSS OBS data
 *                           fix bug on segfault with # obs data >= MAXOBS
 *                           fix bug on reading/writing GLONASS slot/frq # lines
 *                           fix bug on reading SBAS UTC parameters in RINEX nav
 *                           fix bug on saving slip info in extended OBS slots
 *                           add iono/UTC param. in separated output RINEX NAV
 *                           zero-padded satellite number (e.g. "G 1" -> "G01")
 *                           zero-padded month/date/hour/min/sec
 *                           use exponent letter D instead of E for RINEX NAV
 *                           use API code2idx() to get frequency index
 *                           use integer types in stdint.h
 *                           suppress warnings
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Constants/macros ----------------------------------------------------------*/

#define SQR(x) ((x) * (x))

#define NAVEXP "D"                      /* Exponent letter in RINEX NAV */
#define NUMSYS 7                        /* Number of systems */
#define MAXRNXLEN (16 * MAXOBSTYPE + 4) /* Max RINEX record length */
#define MAXPOSHEAD 1024                 /* Max head line position */
#define MINFREQ_GLO -7                  /* Min frequency number GLONASS */
#define MAXFREQ_GLO 13                  /* Max frequency number GLONASS */
#define NINCOBS 262144                  /* Incremental number of obs data */

static const int navsys[] = {/* Satellite systems */
                             SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_SBS, SYS_CMP, SYS_IRN, 0};
static const char syscodes[] = "GREJSCI"; /* Satellite system codes */

static const char obscodes[] = "CLDS"; /* Observation type codes */

static const long double ura_eph[] = {/* RAa values (ref [3] 20.3.3.3.1.1) */
                                      2.4L,    3.4L,    4.85L,   6.85L,  9.65L,  13.65L,
                                      24.0L,   48.0L,   96.0L,   192.0L, 384.0L, 768.0L,
                                      1536.0L, 3072.0L, 6144.0L, 0.0L};
static const long double ura_nominal[] = {/* URA nominal values */
                                          2.0L,    2.8L,    4.0L,    5.7L,   8.0L,   11.3L,
                                          16.0L,   32.0L,   64.0L,   128.0L, 256.0L, 512.0L,
                                          1024.0L, 2048.0L, 4096.0L, 8192.0L};
/* Type definition -----------------------------------------------------------*/
typedef struct {                 /* Signal index type */
  int n;                         /* Number of index */
  int idx[MAXOBSTYPE];           /* Signal freq-index */
  int pos[MAXOBSTYPE];           /* Signal index in obs data (-1:no) */
  uint8_t pri[MAXOBSTYPE];       /* Signal priority (15-0) */
  uint8_t type[MAXOBSTYPE];      /* Type (0:C,1:L,2:D,3:S) */
  uint8_t code[MAXOBSTYPE];      /* Obs-code (CODE_L??) */
  long double shift[MAXOBSTYPE]; /* Phase shift (cycle) */
} sigind_t;

/* Adjust time considering week handover -------------------------------------*/
static gtime_t adjweek(gtime_t t, gtime_t t0) {
  long double tt = timediff(t, t0);
  if (tt < -302400.0L) return timeadd(t, 604800.0L);
  if (tt > 302400.0L) return timeadd(t, -604800.0L);
  return t;
}
/* Adjust time considering week handover -------------------------------------*/
static gtime_t adjday(gtime_t t, gtime_t t0) {
  long double tt = timediff(t, t0);
  if (tt < -43200.0L) return timeadd(t, 86400.0L);
  if (tt > 43200.0L) return timeadd(t, -86400.0L);
  return t;
}
/* Time string for ver.3 (yyyymmdd hhmmss UTC) -------------------------------*/
static void timestr_rnx(char str[32]) {
  gtime_t time;
  long double ep[6];
  time = timeget();
  time.sec = 0.0L;
  time2epoch(time, ep);
  rtksnprintf(str, 32, "%04.0Lf%02.0Lf%02.0Lf %02.0Lf%02.0Lf%02.0Lf UTC", ep[0], ep[1], ep[2],
              ep[3], ep[4], ep[5]);
}
/* Satellite to satellite code -----------------------------------------------*/
static bool sat2code(int sat, char code[8]) {
  int prn;
  switch (satsys(sat, &prn)) {
    case SYS_GPS:
      rtksnprintf(code, 8, "G%02d", prn - MINPRNGPS + 1);
      break;
    case SYS_GLO:
      rtksnprintf(code, 8, "R%02d", prn - MINPRNGLO + 1);
      break;
    case SYS_GAL:
      rtksnprintf(code, 8, "E%02d", prn - MINPRNGAL + 1);
      break;
    case SYS_SBS:
      rtksnprintf(code, 8, "S%02d", prn - 100);
      break;
    case SYS_QZS:
      rtksnprintf(code, 8, "J%02d", prn - MINPRNQZS + 1);
      break;
    case SYS_CMP:
      rtksnprintf(code, 8, "C%02d", prn - MINPRNCMP + 1);
      break;
    case SYS_IRN:
      rtksnprintf(code, 8, "I%02d", prn - MINPRNIRN + 1);
      break;
    default:
      return false;
  }
  return true;
}
/* URA index to URA nominal value (m) ----------------------------------------*/
static long double uravalue(int sva) { return 0 <= sva && sva < 15 ? ura_nominal[sva] : 8192.0L; }
/* URA value (m) to URA index ------------------------------------------------*/
static int uraindex(long double value) {
  int i;
  for (i = 0; i < 15; i++)
    if (ura_eph[i] >= value) break;
  return i;
}
/* Galileo SISA index to SISA nominal value (m) ------------------------------*/
static long double sisa_value(int sisa) {
  if (sisa < 0) return -1.0L;
  if (sisa <= 49) return sisa * 0.01L;
  if (sisa <= 74) return 0.5L + (sisa - 50) * 0.02L;
  if (sisa <= 99) return 1.0L + (sisa - 75) * 0.04L;
  if (sisa <= 125) return 2.0L + (sisa - 100) * 0.16L;
  return -1.0L; /* Unknown or NAPA */
}
/* Galileo SISA value (m) to SISA index --------------------------------------*/
static int sisa_index(long double value) {
  /*
   * kudos to https://core.ac.uk/download/pdf/328854682.pdf for this ...
   * Signal-in-Space Accuracy : SISA flag is a prediction at 1-sigma standard deviation of the
   * quality of the transmitted signal.The flag can take values from 0 to 255. The transmitted
   * standard index is 107 that corresponds to a SISA value of 3.12m.If the prediction is not
   * available, the transmitted index is 255 and corresponds to No Accurate Prediction
   * Available(NAPA).NAPA is an indicator of a potential anomalous signal-in-space[6].Please
   * notice that SISA flag refers to the dual-frequency signal combinations.
   */
  if (value < 0.0L || value > 6.0L)
    return 255; /* Unknown or NAPA */
  else if (value <= 0.49L)
    return (int)roundl(value / 0.01L);
  else if (value <= 0.98L)
    return (int)roundl((value - 0.5L) / 0.02L) + 50;
  else if (value <= 1.96L)
    return (int)roundl((value - 1.0L) / 0.04L) + 75;
  return (int)roundl((value - 2.0L) / 0.16L) + 100;
}
/* Initialize station parameter ----------------------------------------------*/
static void init_sta(sta_t *sta) {
  *sta->name = '\0';
  *sta->marker = '\0';
  *sta->antdes = '\0';
  *sta->antsno = '\0';
  *sta->rectype = '\0';
  *sta->recver = '\0';
  *sta->recsno = '\0';
  sta->antsetup = sta->itrf = sta->deltype = 0;
  for (int i = 0; i < 3; i++) sta->pos[i] = 0.0L;
  for (int i = 0; i < 3; i++) sta->del[i] = 0.0L;
  sta->hgt = 0.0L;
}
/*------------------------------------------------------------------------------
 * Input RINEX functions
 *----------------------------------------------------------------------------*/

/* Convert RINEX obs-type ver.2 -> ver.3 -------------------------------------*/
static void convcode(long double ver, int sys, const char *str, char type[4]) {
  rtkstrcpy(type, 4, "   ");

  if (!strcmp(str, "P1")) { /* Ver.2.11 GPS L1PY,GLO L2P */
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c1W", 'C');
    else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c1P", 'C');
  } else if (!strcmp(str, "P2")) { /* Ver.2.11 GPS L2PY,GLO L2P */
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c2W", 'C');
    else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c2P", 'C');
  } else if (!strcmp(str, "C1")) { /* Ver.2.11 GPS L1C,GLO L1C/A */
    if (ver >= 2.12L)
      ; /* Reject C1 for 2.12 */
    else if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c1C", 'C');
    else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c1C", 'C');
    else if (sys == SYS_GAL)
      rtksnprintf(type, 4, "%c1X", 'C'); /* Ver.2.12 */
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c1C", 'C');
    else if (sys == SYS_SBS)
      rtksnprintf(type, 4, "%c1C", 'C');
  } else if (!strcmp(str, "C2")) {
    if (sys == SYS_GPS) {
      if (ver >= 2.12L)
        rtksnprintf(type, 4, "%c2W", 'C'); /* L2P(Y) */
      else
        rtksnprintf(type, 4, "%c2X", 'C'); /* L2C */
    } else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c2C", 'C');
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c2X", 'C');
    else if (sys == SYS_CMP)
      rtksnprintf(type, 4, "%c2X", 'C');      /* Ver.2.12 B1_2 */
  } else if (ver >= 2.12L && str[1] == 'A') { /* Ver.2.12 L1C/A */
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c1C", str[0]);
    else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c1C", str[0]);
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c1C", str[0]);
    else if (sys == SYS_SBS)
      rtksnprintf(type, 4, "%c1C", str[0]);
  } else if (ver >= 2.12L && str[1] == 'B') { /* Ver.2.12 GPS L1C */
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c1X", str[0]);
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c1X", str[0]);
  } else if (ver >= 2.12L && str[1] == 'C') { /* Ver.2.12 GPS L2C */
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c2X", str[0]);
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c2X", str[0]);
  } else if (ver >= 2.12L && str[1] == 'D') { /* Ver.2.12 GLO L2C/A */
    if (sys == SYS_GLO) rtksnprintf(type, 4, "%c2C", str[0]);
  } else if (ver >= 2.12L && str[1] == '1') { /* Ver.2.12 GPS L1PY,GLO L1P */
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c1W", str[0]);
    else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c1P", str[0]);
    else if (sys == SYS_GAL)
      rtksnprintf(type, 4, "%c1X", str[0]); /* Tentative */
    else if (sys == SYS_CMP)
      rtksnprintf(type, 4, "%c2X", str[0]); /* Extension */
  } else if (ver < 2.12L && str[1] == '1') {
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c1C", str[0]);
    else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c1C", str[0]);
    else if (sys == SYS_GAL)
      rtksnprintf(type, 4, "%c1X", str[0]); /* Tentative */
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c1C", str[0]);
    else if (sys == SYS_SBS)
      rtksnprintf(type, 4, "%c1C", str[0]);
  } else if (str[1] == '2') {
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c2W", str[0]);
    else if (sys == SYS_GLO)
      rtksnprintf(type, 4, "%c2P", str[0]);
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c2X", str[0]);
    else if (sys == SYS_CMP)
      rtksnprintf(type, 4, "%c2X", str[0]); /* Ver.2.12 B1_2 */
  } else if (str[1] == '5') {
    if (sys == SYS_GPS)
      rtksnprintf(type, 4, "%c5X", str[0]);
    else if (sys == SYS_GAL)
      rtksnprintf(type, 4, "%c5X", str[0]);
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c5X", str[0]);
    else if (sys == SYS_SBS)
      rtksnprintf(type, 4, "%c5X", str[0]);
  } else if (str[1] == '6') {
    if (sys == SYS_GAL)
      rtksnprintf(type, 4, "%c6X", str[0]);
    else if (sys == SYS_QZS)
      rtksnprintf(type, 4, "%c6X", str[0]);
    else if (sys == SYS_CMP)
      rtksnprintf(type, 4, "%c6X", str[0]); /* Ver.2.12 B3 */
  } else if (str[1] == '7') {
    if (sys == SYS_GAL)
      rtksnprintf(type, 4, "%c7X", str[0]);
    else if (sys == SYS_CMP)
      rtksnprintf(type, 4, "%c7X", str[0]); /* Ver.2.12 B2b */
  } else if (str[1] == '8') {
    if (sys == SYS_GAL) rtksnprintf(type, 4, "%c8X", str[0]);
  }
  trace(3, "convcode: ver=%.2Lf sys=%2d type= %s -> %s\n", ver, sys, str, type);
}
/* Decode RINEX observation data file header ---------------------------------*/
static void decode_obsh(FILE *fp, char *buff, long double ver, int *tsys,
                        char tobs[][MAXOBSTYPE][4], nav_t *nav, sta_t *sta) {
  /* Default codes for unknown code */
  const char frqcodes[] = "1256789";
  const char *defcodes[] = {
      "CWX    ", /* GPS: L125____ */
      "CCXX X ", /* GLO: L1234_6_ */
      "CXXXXX ", /* GAL: L125678_ */
      "CXXX   ", /* QZS: L1256___ */
      "C X    ", /* SBS: L1_5____ */
      "XIXIIX ", /* BDS: L125678_ */
      "  A   A"  /* IRN: L__5___9 */
  };

  trace(4, "decode_obsh: ver=%.2Lf\n", ver);

  char *label = buff + 60;
  if (strstr(label, "MARKER NAME")) {
    if (sta) rtksetstr(sta->name, sizeof(sta->name), buff, 0, 60);
  } else if (strstr(label, "MARKER NUMBER")) { /* Opt */
    if (sta) rtksetstr(sta->marker, sizeof(sta->marker), buff, 0, 20);
  } else if (strstr(label, "MARKER TYPE"))
    ; /* Ver.3 */
  else if (strstr(label, "OBSERVER / AGENCY"))
    ;
  else if (strstr(label, "REC # / TYPE / VERS")) {
    if (sta) {
      rtksetstr(sta->recsno, sizeof(sta->recsno), buff, 0, 20);
      rtksetstr(sta->rectype, sizeof(sta->rectype), buff, 20, 40);
      rtksetstr(sta->recver, sizeof(sta->recver), buff, 40, 60);
    }
  } else if (strstr(label, "ANT # / TYPE")) {
    if (sta) {
      rtksetstr(sta->antsno, sizeof(sta->antsno), buff, 0, 20);
      rtksetstr(sta->antdes, sizeof(sta->antdes), buff, 20, 40);
    }
  } else if (strstr(label, "APPROX POSITION XYZ")) {
    if (sta) {
      for (int i = 0, j = 0; i < 3; i++, j += 14) sta->pos[i] = str2num(buff, j, 14);
    }
  } else if (strstr(label, "ANTENNA: DELTA H/E/N")) {
    if (sta) {
      long double del[3];
      for (int i = 0, j = 0; i < 3; i++, j += 14) del[i] = str2num(buff, j, 14);
      sta->del[2] = del[0]; /* h */
      sta->del[0] = del[1]; /* e */
      sta->del[1] = del[2]; /* n */
    }
  } else if (strstr(label, "ANTENNA: DELTA X/Y/Z"))
    ; /* Opt ver.3 */
  else if (strstr(label, "ANTENNA: PHASECENTER"))
    ; /* Opt ver.3 */
  else if (strstr(label, "ANTENNA: B.SIGHT XYZ"))
    ; /* Opt ver.3 */
  else if (strstr(label, "ANTENNA: ZERODIR AZI"))
    ; /* Opt ver.3 */
  else if (strstr(label, "ANTENNA: ZERODIR XYZ"))
    ; /* Opt ver.3 */
  else if (strstr(label, "CENTER OF MASS: XYZ"))
    ;                                              /* Opt ver.3 */
  else if (strstr(label, "SYS / # / OBS TYPES")) { /* Ver.3 */
    const char *p = strchr(syscodes, buff[0]);
    if (!p) {
      trace(2, "invalid system code: sys=%c\n", buff[0]);
      return;
    }
    int i = (int)(p - syscodes);
    int n = (int)str2num(buff, 3, 3);
    int nt = 0, k = 7;
    for (int j = 0; j < n; j++, k += 4) {
      if (k > 58) {
        if (!fgets(buff, MAXRNXLEN, fp)) break;
        k = 7;
      }
      if (nt < MAXOBSTYPE - 1) rtksetstr(tobs[i][nt++], sizeof(tobs[0][0]), buff, k, k + 3);
    }
    *tobs[i][nt] = '\0';

    /* Change BDS B1 code: 3.02 */
    if (i == 5 && fabsl(ver - 3.02L) < 1e-3L) {
      for (int j = 0; j < nt; j++)
        if (tobs[i][j][1] == '1') tobs[i][j][1] = '2';
    }
    /* Uncomment this code to convert unknown codes to defaults */
    /* For (int j=0;j<nt;j++) {
        if (tobs[i][j][2]) continue;
        if (!(p=strchr(frqcodes,tobs[i][j][1]))) continue;
        tobs[i][j][2]=defcodes[i][(int)(p-frqcodes)];
        trace(2,"set default for unknown code: sys=%c code=%s\n",buff[0],
              tobs[i][j]);
    }  */
  } else if (strstr(label, "WAVELENGTH FACT L1/2"))
    ;                                              /* Opt ver.2 */
  else if (strstr(label, "# / TYPES OF OBSERV")) { /* Ver.2 */
    int n = (int)str2num(buff, 0, 6);
    int nt = 0;
    for (int i = 0, j = 10; i < n; i++, j += 6) {
      if (j > 58) {
        if (!fgets(buff, MAXRNXLEN, fp)) break;
        j = 10;
      }
      if (nt >= MAXOBSTYPE - 1) continue;
      if (ver <= 2.99L) {
        char str[4];
        rtksetstr(str, sizeof(str), buff, j, j + 2);
        convcode(ver, SYS_GPS, str, tobs[0][nt]);
        convcode(ver, SYS_GLO, str, tobs[1][nt]);
        convcode(ver, SYS_GAL, str, tobs[2][nt]);
        convcode(ver, SYS_QZS, str, tobs[3][nt]);
        convcode(ver, SYS_SBS, str, tobs[4][nt]);
        convcode(ver, SYS_CMP, str, tobs[5][nt]);
      }
      nt++;
    }
    *tobs[0][nt] = '\0';
  } else if (strstr(label, "SIGNAL STRENGTH UNIT"))
    ; /* Opt ver.3 */
  else if (strstr(label, "INTERVAL"))
    ; /* Opt */
  else if (strstr(label, "TIME OF FIRST OBS")) {
    if (!strncmp(buff + 48, "GPS", 3))
      *tsys = TSYS_GPS;
    else if (!strncmp(buff + 48, "GLO", 3))
      *tsys = TSYS_UTC;
    else if (!strncmp(buff + 48, "GAL", 3))
      *tsys = TSYS_GAL;
    else if (!strncmp(buff + 48, "QZS", 3))
      *tsys = TSYS_QZS; /* Ver.3.02 */
    else if (!strncmp(buff + 48, "BDT", 3))
      *tsys = TSYS_CMP; /* Ver.3.02 */
    else if (!strncmp(buff + 48, "IRN", 3))
      *tsys = TSYS_IRN; /* Ver.3.03 */
  } else if (strstr(label, "TIME OF LAST OBS"))
    ; /* Opt */
  else if (strstr(label, "RCV CLOCK OFFS APPL"))
    ; /* Opt */
  else if (strstr(label, "SYS / DCBS APPLIED"))
    ; /* Opt ver.3 */
  else if (strstr(label, "SYS / PCVS APPLIED"))
    ; /* Opt ver.3 */
  else if (strstr(label, "SYS / SCALE FACTOR"))
    ; /* Opt ver.3 */
  else if (strstr(label, "SYS / PHASE SHIFTS"))
    ;                                               /* Ver.3.01 */
  else if (strstr(label, "GLONASS SLOT / FRQ #")) { /* Ver.3.02 */
    for (int i = 0; i < 8; i++) {
      if (buff[4 + i * 7] != 'R') continue;
      int prn = (int)str2num(buff, 5 + i * 7, 2);
      int fcn = (int)str2num(buff, 8 + i * 7, 2);
      if (prn < 1 || prn > MAXPRNGLO || fcn < -7 || fcn > 6) continue;
      if (nav) nav->glo_fcn[prn - 1] = fcn + 8;
    }
  } else if (strstr(label, "GLONASS COD/PHS/BIS")) { /* Ver.3.02 */
    if (sta) {
      sta->glo_cp_bias[0] = str2num(buff, 5, 8);
      sta->glo_cp_bias[1] = str2num(buff, 18, 8);
      sta->glo_cp_bias[2] = str2num(buff, 31, 8);
      sta->glo_cp_bias[3] = str2num(buff, 44, 8);
    }
  } else if (strstr(label, "LEAP SECONDS")) { /* Opt */
    if (nav) {
      nav->utc_gps[4] = str2num(buff, 0, 6);
      nav->utc_gps[7] = str2num(buff, 6, 6);
      nav->utc_gps[5] = str2num(buff, 12, 6);
      nav->utc_gps[6] = str2num(buff, 18, 6);
    }
  } else if (strstr(label, "# OF SATELLITES")) { /* Opt */
    /* Skip */;
  } else if (strstr(label, "PRN / # OF OBS")) { /* Opt */
    /* Skip */;
  }
}
/* Decode RINEX NAV header ---------------------------------------------------*/
static void decode_navh(char *buff, nav_t *nav) {
  trace(4, "decode_navh:\n");

  char *label = buff + 60;
  if (strstr(label, "ION ALPHA")) { /* Opt ver.2 */
    if (nav) {
      for (int i = 0, j = 2; i < 4; i++, j += 12) nav->ion_gps[i] = str2num(buff, j, 12);
    }
  } else if (strstr(label, "ION BETA")) { /* Opt ver.2 */
    if (nav) {
      for (int i = 0, j = 2; i < 4; i++, j += 12) nav->ion_gps[i + 4] = str2num(buff, j, 12);
    }
  } else if (strstr(label, "DELTA-UTC: A0,A1,T,W")) { /* Opt ver.2 */
    if (nav) {
      int i, j;
      for (i = 0, j = 3; i < 2; i++, j += 19) nav->utc_gps[i] = str2num(buff, j, 19);
      for (; i < 4; i++, j += 9) nav->utc_gps[i] = str2num(buff, j, 9);
    }
  } else if (strstr(label, "IONOSPHERIC CORR")) { /* Opt ver.3 */
    if (nav) {
      if (!strncmp(buff, "GPSA", 4)) {
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_gps[i] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "GPSB", 4)) {
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_gps[i + 4] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "GAL", 3)) {
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_gal[i] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "QZSA", 4)) { /* v.3.02 */
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_qzs[i] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "QZSB", 4)) { /* v.3.02 */
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_qzs[i + 4] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "BDSA", 4)) { /* v.3.02 */
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_cmp[i] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "BDSB", 4)) { /* v.3.02 */
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_cmp[i + 4] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "IRNA", 4)) { /* v.3.03 */
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_irn[i] = str2num(buff, j, 12);
      } else if (!strncmp(buff, "IRNB", 4)) { /* v.3.03 */
        for (int i = 0, j = 5; i < 4; i++, j += 12) nav->ion_irn[i + 4] = str2num(buff, j, 12);
      }
    }
  } else if (strstr(label, "TIME SYSTEM CORR")) { /* Opt ver.3 */
    if (nav) {
      if (!strncmp(buff, "GPUT", 4)) {
        nav->utc_gps[0] = str2num(buff, 5, 17);
        nav->utc_gps[1] = str2num(buff, 22, 16);
        nav->utc_gps[2] = str2num(buff, 38, 7);
        nav->utc_gps[3] = str2num(buff, 45, 5);
      } else if (!strncmp(buff, "GLUT", 4)) {
        nav->utc_glo[0] = -str2num(buff, 5, 17); /* tau_C */
      } else if (!strncmp(buff, "GLGP", 4)) {
        nav->utc_glo[1] = str2num(buff, 5, 17); /* tau_GPS */
      } else if (!strncmp(buff, "GAUT", 4)) {   /* v.3.02 */
        nav->utc_gal[0] = str2num(buff, 5, 17);
        nav->utc_gal[1] = str2num(buff, 22, 16);
        nav->utc_gal[2] = str2num(buff, 38, 7);
        nav->utc_gal[3] = str2num(buff, 45, 5);
      } else if (!strncmp(buff, "QZUT", 4)) { /* v.3.02 */
        nav->utc_qzs[0] = str2num(buff, 5, 17);
        nav->utc_qzs[1] = str2num(buff, 22, 16);
        nav->utc_qzs[2] = str2num(buff, 38, 7);
        nav->utc_qzs[3] = str2num(buff, 45, 5);
      } else if (!strncmp(buff, "BDUT", 4)) { /* v.3.02 */
        nav->utc_cmp[0] = str2num(buff, 5, 17);
        nav->utc_cmp[1] = str2num(buff, 22, 16);
        nav->utc_cmp[2] = str2num(buff, 38, 7);
        nav->utc_cmp[3] = str2num(buff, 45, 5);
      } else if (!strncmp(buff, "SBUT", 4)) { /* v.3.02 */
        nav->utc_sbs[0] = str2num(buff, 5, 17);
        nav->utc_sbs[1] = str2num(buff, 22, 16);
        nav->utc_sbs[2] = str2num(buff, 38, 7);
        nav->utc_sbs[3] = str2num(buff, 45, 5);
      } else if (!strncmp(buff, "IRUT", 4)) { /* v.3.03 */
        nav->utc_irn[0] = str2num(buff, 5, 17);
        nav->utc_irn[1] = str2num(buff, 22, 16);
        nav->utc_irn[2] = str2num(buff, 38, 7);
        nav->utc_irn[3] = str2num(buff, 45, 5);
        nav->utc_irn[8] = 0.0L; /* A2 */
      }
    }
  } else if (strstr(label, "LEAP SECONDS")) { /* Opt */
    if (nav) {
      nav->utc_gps[4] = str2num(buff, 0, 6);
      nav->utc_gps[7] = str2num(buff, 6, 6);
      nav->utc_gps[5] = str2num(buff, 12, 6);
      nav->utc_gps[6] = str2num(buff, 18, 6);
    }
  }
}
/* Decode GNAV header --------------------------------------------------------*/
static void decode_gnavh(char *buff, nav_t *nav) {
  trace(4, "decode_gnavh:\n");

  char *label = buff + 60;
  if (strstr(label, "CORR TO SYSTEM TIME")) {
  } /* Opt */
  else if (strstr(label, "LEAP SECONDS")) {
  } /* Opt */
}
/* Decode GEO NAV header -----------------------------------------------------*/
static void decode_hnavh(char *buff, nav_t *nav) {
  trace(4, "decode_hnavh:\n");

  char *label = buff + 60;
  if (strstr(label, "CORR TO SYSTEM TIME")) {
  } /* Opt */
  else if (strstr(label, "D-UTC A0,A1,T,W,S,U")) {
  } /* Opt */
  else if (strstr(label, "LEAP SECONDS")) {
  } /* Opt */
}
/* Read RINEX file header ----------------------------------------------------*/
static bool readrnxh(FILE *fp, long double *ver, char *type, int *sys, int *tsys,
                     char tobs[][MAXOBSTYPE][4], nav_t *nav, sta_t *sta, int flag) {
  trace(3, "readrnxh:\n");

  *ver = 2.10L;
  *type = ' ';
  *sys = SYS_GPS;
  *tsys = TSYS_GPS;

  int i = 0;
  char buff[MAXRNXLEN], *label = buff + 60;
  while (fgets(buff, MAXRNXLEN, fp)) {
    if (strlen(buff) <= 60) {
      continue;
    } else if (strstr(label, "RINEX VERSION / TYPE")) {
      *ver = str2num(buff, 0, 9);
      /* Format change for clock files >=3.04 */
      *type = (*ver < 3.04L || flag == 0) ? *(buff + 20) : *(buff + 21);

      /* Satellite system */
      switch (*(buff + 40)) {
        case ' ':
        case 'G':
          *sys = SYS_GPS;
          *tsys = TSYS_GPS;
          break;
        case 'R':
          *sys = SYS_GLO;
          *tsys = TSYS_UTC;
          break;
        case 'E':
          *sys = SYS_GAL;
          *tsys = TSYS_GAL;
          break; /* v.2.12 */
        case 'S':
          *sys = SYS_SBS;
          *tsys = TSYS_GPS;
          break;
        case 'J':
          *sys = SYS_QZS;
          *tsys = TSYS_QZS;
          break; /* v.3.02 */
        case 'C':
          *sys = SYS_CMP;
          *tsys = TSYS_CMP;
          break; /* v.2.12 */
        case 'I':
          *sys = SYS_IRN;
          *tsys = TSYS_IRN;
          break; /* v.3.03 */
        case 'M':
          *sys = SYS_NONE;
          *tsys = TSYS_GPS;
          break; /* Mixed */
        default:
          trace(2, "not supported satellite system: %c\n", *(buff + 40));
          break;
      }
      continue;
    } else if (strstr(label, "PGM / RUN BY / DATE")) {
      continue;
    } else if (strstr(label, "COMMENT")) {
      continue;
    }
    switch (*type) { /* File type */
      case 'O':
        decode_obsh(fp, buff, *ver, tsys, tobs, nav, sta);
        break;
      case 'N':
        decode_navh(buff, nav);
        break;
      case 'G':
        decode_gnavh(buff, nav);
        break;
      case 'H':
        decode_hnavh(buff, nav);
        break;
      case 'J':
        decode_navh(buff, nav);
        break; /* Extension */
      case 'L':
        decode_navh(buff, nav);
        break; /* Extension */
    }
    if (strstr(label, "END OF HEADER")) return true;

    if (++i >= MAXPOSHEAD && *type == ' ') break; /* No RINEX file */
  }
  return false;
}
/* Decode observation epoch --------------------------------------------------*/
static int decode_obsepoch(FILE *fp, char *buff, long double ver, gtime_t *time, int *flag,
                           int *sats) {
  trace(4, "decode_obsepoch: ver=%.2Lf\n", ver);

  int n;
  if (ver <= 2.99L) { /* Ver.2 */
    /* Epoch flag: 3:new site,4:header info,5:external event */
    *flag = (int)str2num(buff, 28, 1);

    /* Handle external event */
    if (*flag == 5) {
      str2time(buff, 0, 26, time);
    }

    n = (int)str2num(buff, 29, 3);
    if (n <= 0) return 0;

    if (3 <= *flag && *flag <= 5) return n;

    if (str2time(buff, 0, 26, time)) {
      trace(2, "rinex obs invalid epoch: epoch=%26.26s\n", buff);
      return 0;
    }
    for (int i = 0, j = 32; i < n; i++, j += 3) {
      if (j >= 68) {
        if (!fgets(buff, MAXRNXLEN, fp)) break;
        j = 32;
      }
      if (i < MAXOBS) {
        char satid[8] = {'\0'};
        rtkesubstrcpy(satid, sizeof(satid), buff, j, j + 3);
        sats[i] = satid2no(satid);
      }
    }
  } else { /* Ver.3 */
    *flag = (int)str2num(buff, 31, 1);

    /* Handle external event */
    if (*flag == 5) {
      str2time(buff, 1, 28, time);
    }

    n = (int)str2num(buff, 32, 3);
    if (n <= 0) return 0;

    if (3 <= *flag && *flag <= 5) return n;

    if (buff[0] != '>' || str2time(buff, 1, 28, time)) {
      trace(2, "rinex obs invalid epoch: epoch=%29.29s\n", buff);
      return 0;
    }
  }
  char tstr[40];
  trace(4, "decode_obsepoch: time=%s flag=%d\n", time2str(*time, tstr, 3), *flag);
  return n;
}
/* Decode observation data ---------------------------------------------------*/
static bool decode_obsdata(FILE *fp, char *buff, long double ver, int mask, sigind_t *index,
                           obsd_t *obs) {
  trace(4, "decode_obsdata: ver=%.2Lf\n", ver);

  char satid[8] = "";
  if (ver > 2.99L) { /* Ver.3 */
    rtksnprintf(satid, 8, "%.3s", buff);
    obs->sat = (uint8_t)satid2no(satid);
  }
  int stat = 1;
  if (!obs->sat) {
    trace(4, "decode_obsdata: unsupported sat sat=%s\n", satid);
    stat = 0;
  } else if (!(satsys(obs->sat, NULL) & mask)) {
    stat = 0;
  }
  /* Read observation data fields */
  sigind_t *ind;
  switch (satsys(obs->sat, NULL)) {
    case SYS_GLO:
      ind = index + 1;
      break;
    case SYS_GAL:
      ind = index + 2;
      break;
    case SYS_QZS:
      ind = index + 3;
      break;
    case SYS_SBS:
      ind = index + 4;
      break;
    case SYS_CMP:
      ind = index + 5;
      break;
    case SYS_IRN:
      ind = index + 6;
      break;
    default:
      ind = index;
      break;
  }
  long double val[MAXOBSTYPE] = {0};
  uint8_t lli[MAXOBSTYPE] = {0}, std[MAXOBSTYPE] = {0};
  for (int i = 0, j = ver <= 2.99L ? 0 : 3; i < ind->n; i++, j += 16) {
    if (ver <= 2.99L && j >= 80) { /* Ver.2 */
      if (!fgets(buff, MAXRNXLEN, fp)) break;
      j = 0;
    }
    if (stat) {
      val[i] = str2num(buff, j, 14) + ind->shift[i];
      lli[i] = (uint8_t)str2num(buff, j + 14, 1) & 3;
      /* Measurement std from receiver */
      std[i] = (uint8_t)str2num(buff, j + 15, 1);
    }
  }
  if (!stat) return false;

  for (int i = 0; i < NFREQ + NEXOBS; i++) {
    obs->P[i] = obs->L[i] = 0.0L;
    obs->D[i] = 0.0L;
    obs->SNR[i] = obs->LLI[i] = obs->Lstd[i] = obs->Pstd[i] = obs->code[i] = 0;
  }
  /* Assign position in observation data */
  int n = 0, m = 0, q = 0, k[16], l[16], r[16], p[MAXOBSTYPE];
  for (int i = 0; i < ind->n; i++) {
    p[i] = (ver <= 2.11L) ? ind->idx[i] : ind->pos[i];

    if (ind->type[i] == 0 && p[i] == 0) k[n++] = i; /* C1? index */
    if (ind->type[i] == 0 && p[i] == 1) l[m++] = i; /* C2? index */
    if (ind->type[i] == 0 && p[i] == 2) r[q++] = i; /* C3? index */
  }

  /* If multiple codes (C1/P1,C2/P2), select higher priority */
  if (ver <= 2.11L) {
    if (n >= 2) {
      if (val[k[0]] == 0.0L && val[k[1]] == 0.0L) {
        p[k[0]] = -1;
        p[k[1]] = -1;
      } else if (val[k[0]] != 0.0L && val[k[1]] == 0.0L) {
        p[k[0]] = 0;
        p[k[1]] = -1;
      } else if (val[k[0]] == 0.0L && val[k[1]] != 0.0L) {
        p[k[0]] = -1;
        p[k[1]] = 0;
      } else if (ind->pri[k[1]] > ind->pri[k[0]]) {
        p[k[1]] = 0;
        p[k[0]] = NEXOBS < 1 ? -1 : NFREQ;
      } else {
        p[k[0]] = 0;
        p[k[1]] = NEXOBS < 1 ? -1 : NFREQ;
      }
    }
    if (m >= 2) {
      if (val[l[0]] == 0.0L && val[l[1]] == 0.0L) {
        p[l[0]] = -1;
        p[l[1]] = -1;
      } else if (val[l[0]] != 0.0L && val[l[1]] == 0.0L) {
        p[l[0]] = 1;
        p[l[1]] = -1;
      } else if (val[l[0]] == 0.0L && val[l[1]] != 0.0L) {
        p[l[0]] = -1;
        p[l[1]] = 1;
      } else if (ind->pri[l[1]] > ind->pri[l[0]]) {
        p[l[1]] = 1;
        p[l[0]] = NEXOBS < 2 ? -1 : NFREQ + 1;
      } else {
        p[l[0]] = 1;
        p[l[1]] = NEXOBS < 2 ? -1 : NFREQ + 1;
      }
    }
    if (q >= 2) {
      if (val[r[0]] == 0.0L && val[r[1]] == 0.0L) {
        p[r[0]] = -1;
        p[r[1]] = -1;
      } else if (val[r[0]] != 0.0L && val[r[1]] == 0.0L) {
        p[r[0]] = 2;
        p[r[1]] = -1;
      } else if (val[r[0]] == 0.0L && val[r[1]] != 0.0L) {
        p[r[0]] = -1;
        p[r[1]] = 2;
      } else if (ind->pri[r[1]] > ind->pri[r[0]]) {
        p[r[1]] = 2;
        p[r[0]] = NEXOBS < 3 ? -1 : NFREQ + 2;
      } else {
        p[r[0]] = 2;
        p[r[1]] = NEXOBS < 3 ? -1 : NFREQ + 2;
      }
    }
  }

  /* Save observation data */
  for (int i = 0; i < ind->n; i++) {
    if (p[i] < 0 || (val[i] == 0.0L && lli[i] == 0)) continue;
    switch (ind->type[i]) {
      case 0:
        obs->P[p[i]] = val[i];
        obs->code[p[i]] = ind->code[i];
        obs->Pstd[p[i]] = std[i] > 0 ? std[i] : 0;
        break;
      case 1:
        obs->L[p[i]] = val[i];
        obs->LLI[p[i]] = lli[i];
        obs->Lstd[p[i]] = std[i] > 0 ? std[i] : 0;
        break;
      case 2:
        obs->D[p[i]] = val[i];
        break;
      case 3:
        obs->SNR[p[i]] = (uint16_t)(val[i] / SNR_UNIT + 0.5L);
        break;
    }
    trace(4, "obs: i=%d f=%d P=%14.3Lf L=%14.3Lf LLI=%d code=%d\n", i, p[i], obs->P[p[i]],
          obs->L[p[i]], obs->LLI[p[i]], obs->code[p[i]]);
  }
  char tstr[40];
  trace(4, "decode_obsdata: time=%s sat=%2d\n", time2str(obs->time, tstr, 0), obs->sat);
  return true;
}
/* Save cycle slips ----------------------------------------------------------*/
static void saveslips(uint8_t slips[][NFREQ + NEXOBS], const obsd_t *data) {
  for (int i = 0; i < NFREQ + NEXOBS; i++) {
    if (data->LLI[i] & 1) slips[data->sat - 1][i] |= LLI_SLIP;
  }
}
/* Restore cycle slips -------------------------------------------------------*/
static void restslips(uint8_t slips[][NFREQ + NEXOBS], obsd_t *data) {
  for (int i = 0; i < NFREQ + NEXOBS; i++) {
    if (slips[data->sat - 1][i] & 1) data->LLI[i] |= LLI_SLIP;
    slips[data->sat - 1][i] = 0;
  }
}
/* Add observation data ------------------------------------------------------*/
static int addobsdata(obs_t *obs, const obsd_t *data) {
  if (obs->nmax <= obs->n) {
    if (obs->nmax <= 0)
      obs->nmax = NINCOBS;
    else
      obs->nmax *= 2;
    obsd_t *obs_data = (obsd_t *)realloc(obs->data, sizeof(obsd_t) * obs->nmax);
    if (!obs_data) {
      trace(1, "addobsdata: malloc error n=%dx%d\n", sizeof(obsd_t), obs->nmax);
      free(obs->data);
      obs->data = NULL;
      obs->n = obs->nmax = 0;
      return -1;
    }
    obs->data = obs_data;
  }
  obs->data[obs->n++] = *data;
  return 1;
}
/* Set system mask -----------------------------------------------------------*/
static int set_sysmask(const char *opt) {
  const char *p = strstr(opt, "-SYS=");
  if (!p) return SYS_ALL;

  int mask = SYS_NONE;
  for (p += 5; *p && *p != ' '; p++) {
    switch (*p) {
      case 'G':
        mask |= SYS_GPS;
        break;
      case 'R':
        mask |= SYS_GLO;
        break;
      case 'E':
        mask |= SYS_GAL;
        break;
      case 'J':
        mask |= SYS_QZS;
        break;
      case 'C':
        mask |= SYS_CMP;
        break;
      case 'I':
        mask |= SYS_IRN;
        break;
      case 'S':
        mask |= SYS_SBS;
        break;
    }
  }
  return mask;
}
/* Set signal index ----------------------------------------------------------*/
static void set_index(long double ver, int sys, const char *opt, char tobs[MAXOBSTYPE][4],
                      sigind_t *ind) {
  int n = 0;
  for (int i = 0; *tobs[i]; i++, n++) {
    ind->code[i] = obs2code(tobs[i] + 1);
    const char *p = strchr(obscodes, tobs[i][0]);
    ind->type[i] = p ? (int)(p - obscodes) : 0;
    ind->idx[i] = code2idx(sys, ind->code[i]);
    ind->pri[i] = getcodepri(sys, ind->code[i], opt);
    ind->pos[i] = -1;
  }
  /* Parse phase shift options */
  const char *optstr = "";
  switch (sys) {
    case SYS_GPS:
      optstr = "-GL%2s=%Lf";
      break;
    case SYS_GLO:
      optstr = "-RL%2s=%Lf";
      break;
    case SYS_GAL:
      optstr = "-EL%2s=%Lf";
      break;
    case SYS_QZS:
      optstr = "-JL%2s=%Lf";
      break;
    case SYS_SBS:
      optstr = "-SL%2s=%Lf";
      break;
    case SYS_CMP:
      optstr = "-CL%2s=%Lf";
      break;
    case SYS_IRN:
      optstr = "-IL%2s=%Lf";
      break;
  }
  for (const char *p = opt; p && (p = strchr(p, '-')); p++) {
    char str[8];
    long double shift;
    if (sscanf(p, optstr, str, &shift) < 2) continue;
    for (int i = 0; i < n; i++) {
      if (strcmp(code2obs(ind->code[i]), str)) continue;
      ind->shift[i] = shift;
      trace(2, "phase shift: sys=%2d tobs=%s shift=%.3Lf\n", sys, tobs[i], shift);
    }
  }
  /* Assign index for highest priority code */
  for (int i = 0; i < NFREQ; i++) {
    int k = -1;
    for (int j = 0; j < n; j++) {
      if (ind->idx[j] == i && ind->pri[j] && (k < 0 || ind->pri[j] > ind->pri[k])) {
        k = j;
      }
    }
    if (k < 0) continue;

    for (int j = 0; j < n; j++) {
      if (ind->code[j] == ind->code[k]) ind->pos[j] = i;
    }
  }
  /* Assign index of extended observation data */
  for (int i = 0; i < NEXOBS; i++) {
    int j;
    for (j = 0; j < n; j++) {
      if (ind->code[j] && ind->pri[j] && ind->pos[j] < 0) break;
    }
    if (j >= n) break;

    for (int k = 0; k < n; k++) {
      if (ind->code[k] == ind->code[j]) ind->pos[k] = NFREQ + i;
    }
  }
  /* List rejected observation types */
  for (int i = 0; i < n; i++) {
    if (!ind->code[i] || !ind->pri[i] || ind->pos[i] >= 0) continue;
    trace(4, "reject obs type: sys=%2d, obs=%s\n", sys, tobs[i]);
  }
  ind->n = n;

#ifdef RTK_DISABLED /* For debug */
  for (int i = 0; i < n; i++) {
    trace(2, "set_index: sys=%2d,tobs=%s code=%2d pri=%2d idx=%d pos=%d shift=%5.2Lf\n", sys,
          tobs[i], ind->code[i], ind->pri[i], ind->idx[i], ind->pos[i], ind->shift[i]);
  }
#endif
}
/* Read RINEX observation data body ------------------------------------------*/
static int readrnxobsb(FILE *fp, const char *opt, long double ver, int *tsys,
                       char tobs[][MAXOBSTYPE][4], int *flag, obsd_t *data, sta_t *sta) {
  /* Set system mask */
  int mask = set_sysmask(opt);

  /* Set signal index */
  int nsys = NUMSYS;
  sigind_t index[NUMSYS] = {{0}};
  if (nsys >= 1) set_index(ver, SYS_GPS, opt, tobs[0], index);
  if (nsys >= 2) set_index(ver, SYS_GLO, opt, tobs[1], index + 1);
  if (nsys >= 3) set_index(ver, SYS_GAL, opt, tobs[2], index + 2);
  if (nsys >= 4) set_index(ver, SYS_QZS, opt, tobs[3], index + 3);
  if (nsys >= 5) set_index(ver, SYS_SBS, opt, tobs[4], index + 4);
  if (nsys >= 6) set_index(ver, SYS_CMP, opt, tobs[5], index + 5);
  if (nsys >= 7) set_index(ver, SYS_IRN, opt, tobs[6], index + 6);

  /* Read record */
  int i = 0, n = 0, nsat = 0, sats[MAXOBS] = {0};
  gtime_t time = {0};
  char buff[MAXRNXLEN];
  while (fgets(buff, MAXRNXLEN, fp)) {
    /* Decode observation epoch */
    if (i == 0) {
      nsat = decode_obsepoch(fp, buff, ver, &time, flag, sats);
      if (nsat <= 0 && (*flag != 5)) {
        continue;
      }
      if (*flag == 5) {
        data[0].eventime = time;
        return 0;
      }
    } else if ((*flag <= 2 || *flag == 6) && n < MAXOBS) {
      data[n].time = time;
      data[n].sat = (uint8_t)sats[i - 1];

      /* Decode RINEX observation data */
      if (decode_obsdata(fp, buff, ver, mask, index, data + n)) n++;
    } else if (*flag == 3 || *flag == 4) { /* New site or header info follows */

      /* Decode RINEX observation data file header */
      decode_obsh(fp, buff, ver, tsys, tobs, NULL, sta);
    }
    if (++i > nsat) return n;
  }
  return -1;
}
/* Read RINEX observation data -----------------------------------------------*/
static int readrnxobs(FILE *fp, gtime_t ts, gtime_t te, long double tint, const char *opt, int rcv,
                      long double ver, int *tsys, char tobs[][MAXOBSTYPE][4], obs_t *obs,
                      sta_t *sta) {
  trace(4, "readrnxobs: rcv=%d ver=%.2Lf tsys=%d\n", rcv, ver, *tsys);

  if (!obs || rcv > MAXRCV) return 0;

  obsd_t *data = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS);
  if (!data) return 0;

  /* Read RINEX observation data body */
  uint8_t slips[MAXSAT][NFREQ + NEXOBS] = {{0}};
  long double dtime1 = 0;
  int n, n1 = 0, flag = 0, stat = 0;
  gtime_t eventime = {0}, time1 = {0};
  while ((n = readrnxobsb(fp, opt, ver, tsys, tobs, &flag, data, sta)) >= 0 && stat >= 0) {
    if (flag == 5) {
      eventime = data[0].eventime;
      n = readrnxobsb(fp, opt, ver, tsys, tobs, &flag, data, sta);
      if (fabsl(timediff(data[0].time, time1) - dtime1) >= DTTOL)
        n = readrnxobsb(fp, opt, ver, tsys, tobs, &flag, data, sta);
    }

    if (eventime.time == 0 || obs->n - n1 <= 0 || timediff(eventime, time1) >= 0) {
      for (int i = 0; i < n; i++) data[i].eventime = eventime;
    } else {
      /* Add event to previous epoch if delayed */
      for (int i = 0; i < n1; i++) obs->data[obs->n - i - 1].eventime = eventime;
      gtime_t time0 = {0};
      for (int i = 0; i < n; i++) data[i].eventime = time0;
    }
    /* Set to zero eventime for the next iteration */
    eventime.time = 0;
    eventime.sec = 0;

    for (int i = 0; i < n; i++) {
      /* UTC -> GPST */
      if (*tsys == TSYS_UTC) data[i].time = utc2gpst(data[i].time);

      /* Save cycle slip */
      saveslips(slips, data + i);
    }
    /* Screen data by time */
    if (n > 0 && !screent(data[0].time, ts, te, tint)) continue;

    for (int i = 0; i < n; i++) {
      /* Restore cycle slip */
      restslips(slips, data + i);

      data[i].rcv = (uint8_t)rcv;

      /* Save obs data */
      stat = addobsdata(obs, data + i);
      if (stat < 0) break;
    }
    n1 = n;
    dtime1 = timediff(data[0].time, time1);
    time1 = data[0].time;
  }
  trace(4, "readrnxobs: nobs=%d stat=%d\n", obs->n, stat);

  free(data);

  return stat;
}
/* Decode ephemeris ----------------------------------------------------------*/
static bool decode_eph(long double ver, int sat, gtime_t toc, const long double *data, eph_t *eph) {
  trace(4, "decode_eph: ver=%.2Lf sat=%2d\n", ver, sat);

  int sys = satsys(sat, NULL);

  if (!(sys & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_CMP | SYS_IRN))) {
    trace(4, "ephemeris error: invalid satellite sat=%2d\n", sat);
    return false;
  }
  eph_t eph0 = {0};
  *eph = eph0;

  eph->sat = sat;
  eph->toc = toc;

  eph->f0 = data[0];
  eph->f1 = data[1];
  eph->f2 = data[2];

  eph->A = SQR(data[10]);
  eph->e = data[8];
  eph->i0 = data[15];
  eph->OMG0 = data[13];
  eph->omg = data[17];
  eph->M0 = data[6];
  eph->deln = data[5];
  eph->OMGd = data[18];
  eph->idot = data[19];
  eph->crc = data[16];
  eph->crs = data[4];
  eph->cuc = data[7];
  eph->cus = data[9];
  eph->cic = data[12];
  eph->cis = data[14];

  if (sys == SYS_GPS || sys == SYS_QZS) {
    eph->iode = (int)data[3];  /* IODE */
    eph->iodc = (int)data[26]; /* IODC */
    eph->toes = data[11];      /* Toe (s) in GPS week */
    eph->week = (int)data[21]; /* GPS week */
    eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
    eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);

    eph->code = (int)data[20];     /* GPS: codes on L2 ch */
    eph->svh = (int)data[24];      /* SV health */
    eph->sva = uraindex(data[23]); /* URA index (m->index) */
    eph->flag = (int)data[22];     /* GPS: L2 P data flag */

    eph->tgd[0] = data[25]; /* TGD */
    if (sys == SYS_GPS) {
      eph->fit = data[28]; /* Fit interval (h) */
    } else {
      eph->fit = data[28] == 0.0L ? 1.0L : 2.0L; /* Fit interval (0:1h,1:>2h) */
    }
  } else if (sys == SYS_GAL) { /* GAL ver.3 */
    eph->iode = (int)data[3];  /* IODnav */
    eph->toes = data[11];      /* Toe (s) in Galileo week */
    eph->week = (int)data[21]; /* Galileo week = GPS week */
    eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
    eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);

    eph->code = (int)data[20];       /* Data sources */
                                     /* Bit 0 set: I/NAV E1-B */
                                     /* Bit 1 set: F/NAV E5a-I */
                                     /* Bit 2 set: F/NAV E5b-I */
                                     /* Bit 8 set: af0-af2 toc are for E5a.E1 */
                                     /* Bit 9 set: af0-af2 toc are for E5b.E1 */
    eph->svh = (int)data[24];        /* Sv health */
                                     /* Bit     0: E1B DVS */
                                     /* Bit   1-2: E1B HS */
                                     /* Bit     3: E5a DVS */
                                     /* Bit   4-5: E5a HS */
                                     /* Bit     6: E5b DVS */
                                     /* Bit   7-8: E5b HS */
    eph->sva = sisa_index(data[23]); /* Sisa (m->index) */

    eph->tgd[0] = data[25];                             /* BGD E5a/E1 */
    eph->tgd[1] = data[26];                             /* BGD E5b/E1 */
  } else if (sys == SYS_CMP) {                          /* BeiDou v.3.02 */
    eph->toc = bdt2gpst(eph->toc);                      /* BDT -> GPST */
    eph->iode = (int)data[3];                           /* AODE */
    eph->iodc = (int)data[28];                          /* AODC */
    eph->toes = data[11];                               /* Toe (s) in BDT week */
    eph->week = (int)data[21];                          /* BDT week */
    eph->toe = bdt2gpst(bdt2time(eph->week, data[11])); /* BDT -> GPST */
    eph->ttr = bdt2gpst(bdt2time(eph->week, data[27])); /* BDT -> GPST */
    eph->toe = adjweek(eph->toe, toc);
    eph->ttr = adjweek(eph->ttr, toc);

    eph->svh = (int)data[24];      /* SatH1 */
    eph->sva = uraindex(data[23]); /* URA index (m->index) */

    eph->tgd[0] = data[25];    /* TGD1 B1/B3 */
    eph->tgd[1] = data[26];    /* TGD2 B2/B3 */
  } else if (sys == SYS_IRN) { /* IRNSS v.3.03 */
    eph->iode = (int)data[3];  /* IODEC */
    eph->toes = data[11];      /* Toe (s) in IRNSS week */
    eph->week = (int)data[21]; /* IRNSS week */
    eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
    eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);
    eph->svh = (int)data[24];      /* SV health */
    eph->sva = uraindex(data[23]); /* URA index (m->index) */
    eph->tgd[0] = data[25];        /* TGD */
  }
  if (eph->iode < 0 || 1023 < eph->iode) {
    trace(2, "rinex nav invalid: sat=%2d iode=%d\n", sat, eph->iode);
  }
  if (eph->iodc < 0 || 1023 < eph->iodc) {
    trace(2, "rinex nav invalid: sat=%2d iodc=%d\n", sat, eph->iodc);
  }
  return true;
}
/* Decode GLONASS ephemeris --------------------------------------------------*/
static bool decode_geph(long double ver, int sat, gtime_t toc, long double *data, geph_t *geph) {
  trace(4, "decode_geph: ver=%.2Lf sat=%2d\n", ver, sat);

  if (satsys(sat, NULL) != SYS_GLO) {
    trace(4, "glonass ephemeris error: invalid satellite sat=%2d\n", sat);
    return false;
  }
  geph_t geph0 = {0};
  *geph = geph0;

  geph->sat = sat;

  /* Toc rounded by 15 min in UTC */
  int week;
  long double tow = time2gpst(toc, &week);
  toc = gpst2time(week, floorl((tow + 450.0L) / 900.0L) * 900);
  int dow = (int)floorl(tow / 86400.0L);

  /* Time of frame in UTC */
  long double tod =
      ver <= 2.99L ? data[2] : fmodl(data[2], 86400.0L); /* Tod (v.2), Tow (v.3) in UTC */
  gtime_t tof = gpst2time(week, tod + dow * 86400.0L);
  tof = adjday(tof, toc);

  geph->toe = utc2gpst(toc); /* Toc (GPST) */
  geph->tof = utc2gpst(tof); /* Tof (GPST) */

  /* IODE = Tb (7bit), Tb =index of UTC+3H within current day */
  geph->iode = (int)(fmodl(tow + 10800.0L, 86400.0L) / 900.0L + 0.5L);

  geph->taun = -data[0]; /* -taun */
  geph->gamn = data[1];  /* +gamman */

  geph->pos[0] = data[3] * 1E3L;
  geph->pos[1] = data[7] * 1E3L;
  geph->pos[2] = data[11] * 1E3L;
  geph->vel[0] = data[4] * 1E3L;
  geph->vel[1] = data[8] * 1E3L;
  geph->vel[2] = data[12] * 1E3L;
  geph->acc[0] = data[5] * 1E3L;
  geph->acc[1] = data[9] * 1E3L;
  geph->acc[2] = data[13] * 1E3L;

  geph->svh = (int)data[6];
  geph->frq = (int)data[10];
#ifdef RTK_DISABLED /*  Output dtaun instead of age */
  geph->dtaun = data[14];
#else
  geph->age = (int)data[14];
#endif
  /* Some receiver output >128 for minus frequency number */
  if (geph->frq > 128) geph->frq -= 256;

  if (geph->frq < MINFREQ_GLO || MAXFREQ_GLO < geph->frq) {
    trace(2, "rinex gnav invalid freq: sat=%2d fn=%d\n", sat, geph->frq);
  }
  return true;
}
/* Decode GEO ephemeris ------------------------------------------------------*/
static bool decode_seph(long double ver, int sat, gtime_t toc, long double *data, seph_t *seph) {
  trace(4, "decode_seph: ver=%.2Lf sat=%2d\n", ver, sat);

  if (satsys(sat, NULL) != SYS_SBS) {
    trace(4, "geo ephemeris error: invalid satellite sat=%2d\n", sat);
    return false;
  }
  seph_t seph0 = {0};
  *seph = seph0;

  seph->sat = sat;
  seph->t0 = toc;

  int week;
  time2gpst(toc, &week);
  seph->tof = adjweek(gpst2time(week, data[2]), toc);

  seph->af0 = data[0];
  seph->af1 = data[1];

  seph->pos[0] = data[3] * 1E3L;
  seph->pos[1] = data[7] * 1E3L;
  seph->pos[2] = data[11] * 1E3L;
  seph->vel[0] = data[4] * 1E3L;
  seph->vel[1] = data[8] * 1E3L;
  seph->vel[2] = data[12] * 1E3L;
  seph->acc[0] = data[5] * 1E3L;
  seph->acc[1] = data[9] * 1E3L;
  seph->acc[2] = data[13] * 1E3L;

  seph->svh = (int)data[6];
  seph->sva = uraindex(data[10]);

  return true;
}
/* Read RINEX navigation data body -------------------------------------------*/
static int readrnxnavb(FILE *fp, const char *opt, long double ver, int sys, int *type, eph_t *eph,
                       geph_t *geph, seph_t *seph) {
  trace(4, "readrnxnavb: ver=%.2Lf sys=%d\n", ver, sys);
  /* Set system mask */
  int mask = set_sysmask(opt);

  long double data[64];
  int i = 0, sat = 0, sp = 3;
  char buff[MAXRNXLEN];
  gtime_t toc;
  while (fgets(buff, MAXRNXLEN, fp)) {
    if (i == 0) {
      /* Decode satellite field */
      if (ver >= 3.0L || sys == SYS_GAL || sys == SYS_QZS) { /* Ver.3 or GAL/QZS */
        char id[8] = "";
        rtksnprintf(id, 8, "%.3s", buff);
        sat = satid2no(id);
        sp = 4;
        if (ver >= 3.0L) {
          sys = satsys(sat, NULL);
          if (!sys) {
            sys = (id[0] == 'S') ? SYS_SBS : ((id[0] == 'R') ? SYS_GLO : SYS_GPS);
          }
        }
      } else {
        int prn = (int)str2num(buff, 0, 2);

        if (sys == SYS_SBS) {
          sat = satno(SYS_SBS, prn + 100);
        } else if (sys == SYS_GLO) {
          sat = satno(SYS_GLO, prn);
        } else if (93 <= prn && prn <= 97) { /* Extension */
          sat = satno(SYS_QZS, prn + 100);
        } else
          sat = satno(SYS_GPS, prn);
      }
      /* Decode Toc field */
      if (str2time(buff + sp, 0, 19, &toc)) {
        trace(2, "rinex nav toc error: %23.23s\n", buff);
        return 0;
      }
      /* Decode data fields */
      char *p = buff + sp + 19;
      for (int j = 0; j < 3; j++, p += 19) {
        data[i++] = str2num(p, 0, 19);
      }
    } else {
      /* Decode data fields */
      char *p = buff + sp;
      for (int j = 0; j < 4; j++, p += 19) {
        data[i++] = str2num(p, 0, 19);
      }
      /* Decode ephemeris */
      if (sys == SYS_GLO && i >= 15) {
        if (!(mask & sys)) return 0;
        *type = 1;
        return decode_geph(ver, sat, toc, data, geph);
      } else if (sys == SYS_SBS && i >= 15) {
        if (!(mask & sys)) return 0;
        *type = 2;
        return decode_seph(ver, sat, toc, data, seph);
      } else if (i >= 31) {
        if (!(mask & sys)) return 0;
        *type = 0;
        return decode_eph(ver, sat, toc, data, eph);
      }
    }
  }
  return -1;
}
/* Add ephemeris to navigation data ------------------------------------------*/
static bool add_eph(nav_t *nav, const eph_t *eph) {
  int sat = eph->sat;
  if (nav->nmax[sat - 1] <= nav->n[sat - 1]) {
    nav->nmax[sat - 1] += 16;
    eph_t *nav_eph = (eph_t *)realloc(nav->eph[sat - 1], sizeof(eph_t) * nav->nmax[sat - 1]);

    if (!nav_eph) {
      trace(1, "decode_eph malloc error: n=%d\n", nav->nmax[sat - 1]);
      free(nav->eph[sat - 1]);
      nav->eph[sat - 1] = NULL;
      nav->n[sat - 1] = nav->nmax[sat - 1] = 0;
      return false;
    }
    nav->eph[sat - 1] = nav_eph;
  }
  nav->eph[sat - 1][nav->n[sat - 1]++] = *eph;
  return true;
}
static bool add_geph(nav_t *nav, const geph_t *geph) {
  int prn;
  if (satsys(geph->sat, &prn) != SYS_GLO) {
    fprintf(stderr, "** E20\n");
    return false;
  }

  if (nav->ngmax[prn - 1] <= nav->ng[prn - 1]) {
    nav->ngmax[prn - 1] += 16;
    geph_t *nav_geph = (geph_t *)realloc(nav->geph[prn - 1], sizeof(geph_t) * nav->ngmax[prn - 1]);
    if (!nav_geph) {
      trace(1, "decode_geph malloc error: n=%d\n", nav->ngmax[prn - 1]);
      free(nav->geph[prn - 1]);
      nav->geph[prn - 1] = NULL;
      nav->ng[prn - 1] = nav->ngmax[prn - 1] = 0;
      return false;
    }
    nav->geph[prn - 1] = nav_geph;
  }
  nav->geph[prn - 1][nav->ng[prn - 1]++] = *geph;
  return true;
}
static bool add_seph(nav_t *nav, const seph_t *seph) {
  int prn;
  if (satsys(seph->sat, &prn) != SYS_SBS) {
    fprintf(stderr, "** E21\n");
    return false;
  }
  int i = prn - MINPRNSBS;
  if (nav->nsmax[i] <= nav->ns[i]) {
    nav->nsmax[i] += 16;
    seph_t *nav_seph = (seph_t *)realloc(nav->seph[i], sizeof(seph_t) * nav->nsmax[i]);
    if (!nav_seph) {
      trace(1, "decode_seph malloc error: n=%d\n", nav->nsmax[i]);
      free(nav->seph[i]);
      nav->seph[i] = NULL;
      nav->ns[i] = nav->nsmax[i] = 0;
      return false;
    }
    nav->seph[i] = nav_seph;
  }
  nav->seph[i][nav->ns[i]++] = *seph;
  return true;
}
/* Read RINEX navigation data ------------------------------------------------*/
static int readrnxnav(FILE *fp, const char *opt, long double ver, int sys, nav_t *nav) {
  trace(3, "readrnxnav: ver=%.2Lf sys=%d\n", ver, sys);

  if (!nav) return 0;

  /* Read RINEX navigation data body */
  eph_t eph;
  geph_t geph;
  seph_t seph;
  int stat, type;
  while ((stat = readrnxnavb(fp, opt, ver, sys, &type, &eph, &geph, &seph)) >= 0) {
    /* Add ephemeris to navigation data */
    if (stat) {
      switch (type) {
        case 1:
          stat = add_geph(nav, &geph);
          break;
        case 2:
          stat = add_seph(nav, &seph);
          break;
        default:
          stat = add_eph(nav, &eph);
          break;
      }
      if (!stat) return 0;
    }
  }
  for (int i = 0; i < MAXSAT; i++)
    if (nav->n[i] > 0) return 1;
  for (int i = 0; i < NSATGLO; i++)
    if (nav->ng[i] > 0) return 1;
  for (int i = 0; i < NSATSBS; i++)
    if (nav->ns[i] > 0) return 1;
  return 0;
}
/* Read RINEX clock ----------------------------------------------------------*/
static int readrnxclk(FILE *fp, const char *opt, long double ver, int index, nav_t *nav) {
  trace(3, "readrnxclk: index=%d\n", index);

  if (!nav) return 0;

  /* Set system mask */
  int mask = set_sysmask(opt);
  int off = ver >= 3.04L ? 5 : 0; /* Format change for ver>=3.04 */

  char buff[MAXRNXLEN];
  while (fgets(buff, sizeof(buff), fp)) {
    gtime_t time;
    if (str2time(buff, 8 + off, 26, &time)) {
      trace(2, "rinex clk invalid epoch: %34.34s\n", buff);
      continue;
    }
    char satid[8] = "";
    rtkesubstrcpy(satid, sizeof(satid), buff, 3, 7);

    /* Only read AS (satellite clock) record */
    int sat;
    if (strncmp(buff, "AS", 2) || !(sat = satid2no(satid))) continue;

    if (!(satsys(sat, NULL) & mask)) continue;

    long double data[2];
    for (int i = 0, j = 40 + off; i < 2; i++, j += 20) data[i] = str2num(buff, j, 19);

    if (nav->nc >= nav->ncmax) {
      nav->ncmax += 1024;
      pclk_t *nav_pclk = (pclk_t *)realloc(nav->pclk, sizeof(pclk_t) * (nav->ncmax));
      if (!nav_pclk) {
        trace(1, "readrnxclk malloc error: nmax=%d\n", nav->ncmax);
        free(nav->pclk);
        nav->pclk = NULL;
        nav->nc = nav->ncmax = 0;
        return -1;
      }
      nav->pclk = nav_pclk;
    }
    if (nav->nc <= 0 || fabsl(timediff(time, nav->pclk[nav->nc - 1].time)) > 1E-9L) {
      nav->nc++;
      nav->pclk[nav->nc - 1].time = time;
      nav->pclk[nav->nc - 1].index = index;
      for (int i = 0; i < MAXSAT; i++) {
        nav->pclk[nav->nc - 1].clk[i][0] = 0.0L;
        nav->pclk[nav->nc - 1].std[i][0] = 0.0L;
      }
    }
    nav->pclk[nav->nc - 1].clk[sat - 1][0] = data[0];
    nav->pclk[nav->nc - 1].std[sat - 1][0] = data[1];
  }
  return nav->nc > 0;
}
/* Read RINEX file -----------------------------------------------------------*/
static int readrnxfp(FILE *fp, gtime_t ts, gtime_t te, long double tint, const char *opt, int flag,
                     int index, char *type, obs_t *obs, nav_t *nav, sta_t *sta) {
  trace(3, "readrnxfp: flag=%d index=%d\n", flag, index);

  /* Read RINEX file header */
  long double ver;
  int sys, tsys = TSYS_GPS;
  char tobs[NUMSYS][MAXOBSTYPE][4] = {{""}};
  if (!readrnxh(fp, &ver, type, &sys, &tsys, tobs, nav, sta, flag)) return 0;

  /* flag=0:except for clock,1:clock */
  if ((!flag && *type == 'C') || (flag && *type != 'C')) return 0;

  /* Read RINEX file body */
  switch (*type) {
    case 'O':
      return readrnxobs(fp, ts, te, tint, opt, index, ver, &tsys, tobs, obs, sta);
    case 'N':
      return readrnxnav(fp, opt, ver, sys, nav);
    case 'G':
      return readrnxnav(fp, opt, ver, SYS_GLO, nav);
    case 'H':
      return readrnxnav(fp, opt, ver, SYS_SBS, nav);
    case 'J':
      return readrnxnav(fp, opt, ver, SYS_QZS, nav); /* Extension */
    case 'L':
      return readrnxnav(fp, opt, ver, SYS_GAL, nav); /* Extension */
    case 'C':
      return readrnxclk(fp, opt, ver, index, nav);
  }
  trace(2, "unsupported rinex type ver=%.2Lf type=%c\n", ver, *type);
  return 0;
}
/* Uncompress and read RINEX file --------------------------------------------*/
static int readrnxfile(const char *file, gtime_t ts, gtime_t te, long double tint, const char *opt,
                       int flag, int index, char *type, obs_t *obs, nav_t *nav, sta_t *sta) {
  trace(3, "readrnxfile: file=%s flag=%d index=%d\n", file, flag, index);

  if (sta) init_sta(sta);

  /* Uncompress file */
  char tmpfile[FNSIZE];
  int cstat = rtk_uncompress(file, tmpfile, sizeof(tmpfile));
  if (cstat < 0) {
    trace(2, "rinex file uncompact error: %s\n", file);
    return 0;
  }
  FILE *fp = fopen(cstat ? tmpfile : file, "r");
  if (!fp) {
    trace(2, "rinex file open error: %s\n", cstat ? tmpfile : file);
    return 0;
  }
  /* Read RINEX file */
  int stat = readrnxfp(fp, ts, te, tint, opt, flag, index, type, obs, nav, sta);

  fclose(fp);

  /* Delete temporary file */
  if (cstat) remove(tmpfile);

  return stat;
}
/* Read RINEX OBS and NAV files ------------------------------------------------
 * Read RINEX OBS and NAV files
 * Args   : char *file    I      file (wild-card * expanded) ("": stdin)
 *          int   rcv     I      receiver number for obs data
 *         (gtime_t ts)   I      observation time start (ts.time==0: no limit)
 *         (gtime_t te)   I      observation time end   (te.time==0: no limit)
 *         (long double tint)  I      observation time interval (s) (0:all)
 *          char  *opt    I      RINEX options (see below,"": no option)
 *          obs_t *obs    IO     observation data   (NULL: no input)
 *          nav_t *nav    IO     navigation data    (NULL: no input)
 *          sta_t *sta    IO     station parameters (NULL: no input)
 * Return : status (1:ok,0:no data,-1:error)
 * Notes  : read data are appended to obs and nav struct
 *          before calling the function, obs and nav should be initialized.
 *          observation data and navigation data are not sorted.
 *          navigation data may be duplicated.
 *          call sortobs() or uniqnav() to sort data or delete duplicated eph.
 *
 *          RINEX options (separated by spaces) :
 *
 *            -GLss[=shift]: select GPS signal ss (ss: RINEX 3 code, "1C","2W"...)
 *            -RLss[=shift]: select GLO signal ss
 *            -ELss[=shift]: select GAL signal ss
 *            -JLss[=shift]: select QZS signal ss
 *            -CLss[=shift]: select BDS signal ss
 *            -ILss[=shift]: select IRN signal ss
 *            -SLss[=shift]: select SBS signal ss
 *
 *                 shift: carrier phase shift to be added (cycle)
 *
 *            -SYS=sys[,sys...]: select navigation systems
 *                               (sys=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN,S:SBS)
 *
 *----------------------------------------------------------------------------*/
extern int readrnxt(const char *file, int rcv, gtime_t ts, gtime_t te, long double tint,
                    const char *opt, obs_t *obs, nav_t *nav, sta_t *sta) {
  trace(3, "readrnxt: file=%s rcv=%d\n", file, rcv);

  if (!*file) {
    char type = ' ';
    return readrnxfp(stdin, ts, te, tint, opt, 0, 1, &type, obs, nav, sta);
  }
  char *files[MAXEXFILE] = {0};
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(files[i] = (char *)malloc(FNSIZE))) {
      for (i--; i >= 0; i--) free(files[i]);
      return -1;
    }
  }
  /* Expand wild-card */
  int n = expath(file, files, FNSIZE, MAXEXFILE);
  if (n <= 0) {
    for (int i = 0; i < MAXEXFILE; i++) free(files[i]);
    return 0;
  }
  /* Read RINEX files */
  char type = ' ';
  int stat = 0;
  for (int i = 0; i < n && stat >= 0; i++) {
    stat = readrnxfile(files[i], ts, te, tint, opt, 0, rcv, &type, obs, nav, sta);
  }
  /* If station name empty, set 4-char name from file head */
  if (type == 'O' && sta && !*sta->name) {
    const char *p = strrchr(file, RTKLIB_FILEPATHSEP);
    if (!p)
      rtksetstr(sta->name, sizeof(sta->name), file, 0, 4);
    else
      rtksetstr(sta->name, sizeof(sta->name), p, 1, 5);
  }
  for (int i = 0; i < MAXEXFILE; i++) free(files[i]);

  return stat;
}
extern int readrnx(const char *file, int rcv, const char *opt, obs_t *obs, nav_t *nav, sta_t *sta) {
  gtime_t t = {0};

  trace(3, "readrnx : file=%s rcv=%d\n", file, rcv);

  return readrnxt(file, rcv, t, t, 0.0L, opt, obs, nav, sta);
}
/* Compare precise clock -----------------------------------------------------*/
static int cmppclk(const void *p1, const void *p2) {
  const pclk_t *q1 = (pclk_t *)p1, *q2 = (pclk_t *)p2;
  long double tt = timediff(q1->time, q2->time);
  return tt < -1E-9L ? -1 : (tt > 1E-9L ? 1 : q1->index - q2->index);
}
/* Combine precise clock -----------------------------------------------------*/
static void combpclk(nav_t *nav) {
  trace(3, "combpclk: nc=%d\n", nav->nc);

  if (nav->nc <= 0) return;

  qsort(nav->pclk, nav->nc, sizeof(pclk_t), cmppclk);

  int i = 0;
  for (int j = 1; j < nav->nc; j++) {
    if (fabsl(timediff(nav->pclk[i].time, nav->pclk[j].time)) < 1E-9L) {
      for (int k = 0; k < MAXSAT; k++) {
        if (nav->pclk[j].clk[k][0] == 0.0L) continue;
        nav->pclk[i].clk[k][0] = nav->pclk[j].clk[k][0];
        nav->pclk[i].std[k][0] = nav->pclk[j].std[k][0];
      }
    } else if (++i < j)
      nav->pclk[i] = nav->pclk[j];
  }
  nav->nc = i + 1;

  pclk_t *nav_pclk = (pclk_t *)realloc(nav->pclk, sizeof(pclk_t) * nav->nc);
  if (!nav_pclk) {
    free(nav->pclk);
    nav->pclk = NULL;
    nav->nc = nav->ncmax = 0;
    trace(1, "combpclk malloc error nc=%d\n", nav->nc);
    return;
  }
  nav->pclk = nav_pclk;
  nav->ncmax = nav->nc;

  trace(4, "combpclk: nc=%d\n", nav->nc);
}
/* Read RINEX clock files ------------------------------------------------------
 * Read RINEX clock files
 * Args   : char *file    I      file (wild-card * expanded)
 *          nav_t *nav    IO     navigation data    (NULL: no input)
 * Return : number of precise clock
 *----------------------------------------------------------------------------*/
extern int readrnxc(const char *file, nav_t *nav) {
  trace(3, "readrnxc: file=%s\n", file);

  char *files[MAXEXFILE] = {0};
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(files[i] = (char *)malloc(FNSIZE))) {
      for (i--; i >= 0; i--) free(files[i]);
      return 0;
    }
  }
  /* Expand wild-card */
  int n = expath(file, files, FNSIZE, MAXEXFILE);

  /* Read RINEX clock files */
  int index = 0, stat = 1;
  for (int i = 0; i < n; i++) {
    gtime_t t = {0};
    char type;
    if (readrnxfile(files[i], t, t, 0.0L, "", 1, index++, &type, NULL, nav, NULL)) {
      continue;
    }
    stat = 0;
    break;
  }
  for (int i = 0; i < MAXEXFILE; i++) free(files[i]);

  if (!stat) return 0;

  /* Unique and combine ephemeris and precise clock */
  combpclk(nav);

  return nav->nc;
}
/* Initialize RINEX control ----------------------------------------------------
 * Initialize RINEX control struct and reallocate memory for observation and
 * Ephemeris buffer in RINEX control struct
 * Args   : rnxctr_t *rnx IO     RINEX control struct
 * Return : status (true:ok,false:memory allocation error)
 *----------------------------------------------------------------------------*/
extern bool init_rnxctr(rnxctr_t *rnx) {
  trace(3, "init_rnxctr:\n");

  rnx->obs.data = NULL;
  for (int i = 0; i < MAXSAT; i++) rnx->nav.eph[i] = NULL;
  for (int i = 0; i < NSATGLO; i++) rnx->nav.geph[i] = NULL;
  for (int i = 0; i < NSATSBS; i++) rnx->nav.seph[i] = NULL;

  if (!(rnx->obs.data = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS))) {
    free_rnxctr(rnx);
    return false;
  }

  eph_t eph0 = {0, -1, -1};
  for (int i = 0; i < MAXSAT; i++) {
    if (!(rnx->nav.eph[i] = (eph_t *)malloc(sizeof(eph_t) * 2))) {
      free_rnxctr(rnx);
      return false;
    }
    rnx->nav.n[i] = rnx->nav.nmax[i] = 2;
    rnx->nav.eph[i][0] = eph0;
    rnx->nav.eph[i][1] = eph0;
  }
  geph_t geph0 = {0, -1};
  for (int i = 0; i < NSATGLO; i++) {
    if (!(rnx->nav.geph[i] = (geph_t *)malloc(sizeof(geph_t) * 1))) {
      free_rnxctr(rnx);
      return false;
    }
    rnx->nav.ng[i] = 1;
    rnx->nav.geph[i][0] = geph0;
  }
  seph_t seph0 = {0};
  for (int i = 0; i < NSATSBS; i++) {
    if (!(rnx->nav.seph[i] = (seph_t *)malloc(sizeof(seph_t) * 2))) {
      free_rnxctr(rnx);
      return false;
    }
    rnx->nav.ng[i] = 2;
    rnx->nav.seph[i][0] = seph0;
    rnx->nav.seph[i][1] = seph0;
  }
  gtime_t time0 = {0};
  rnx->time = time0;
  rnx->ver = 0.0L;
  rnx->sys = rnx->tsys = 0;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < MAXOBSTYPE; j++) rnx->tobs[i][j][0] = '\0';
  rnx->obs.n = 0;
  obsd_t data0 = {{0}};
  for (int i = 0; i < MAXOBS; i++) rnx->obs.data[i] = data0;
  rnx->ephsat = rnx->ephset = 0;
  rnx->opt[0] = '\0';

  return true;
}
/* Free RINEX control ----------------------------------------------------------
 * Free observation and ephemeris buffer in RINEX control struct
 * Args   : rnxctr_t *rnx IO  RINEX control struct
 * Return : none
 *----------------------------------------------------------------------------*/
extern void free_rnxctr(rnxctr_t *rnx) {
  trace(3, "free_rnxctr:\n");

  free(rnx->obs.data);
  rnx->obs.data = NULL;
  rnx->obs.n = 0;
  for (int i = 0; i < MAXSAT; i++) {
    free(rnx->nav.eph[i]);
    rnx->nav.eph[i] = NULL;
    rnx->nav.n[i] = rnx->nav.nmax[i] = 0;
  }
  for (int i = 0; i < NSATGLO; i++) {
    free(rnx->nav.geph[i]);
    rnx->nav.geph[i] = NULL;
    rnx->nav.ng[i] = rnx->nav.ngmax[i] = 0;
  }
  for (int i = 0; i < NSATSBS; i++) {
    free(rnx->nav.seph[i]);
    rnx->nav.seph[i] = NULL;
    rnx->nav.ns[i] = rnx->nav.nsmax[i] = 0;
  }
}
/* Open RINEX data -------------------------------------------------------------
 * Fetch next RINEX message and input a message from file
 * Args   : rnxctr_t *rnx IO  RINEX control struct
 *          FILE  *fp    I    file pointer
 * Return : status (-2: end of file, 0: no message, 1: input observation data,
 *                   2: input navigation data)
 *----------------------------------------------------------------------------*/
extern int open_rnxctr(rnxctr_t *rnx, FILE *fp) {
  const char *rnxtypes = "ONGLJHC";

  trace(3, "open_rnxctr:\n");

  /* Read RINEX header from file */
  long double ver;
  char type, tobs[NUMSYS][MAXOBSTYPE][4] = {{""}};
  int sys, tsys;
  if (!readrnxh(fp, &ver, &type, &sys, &tsys, tobs, &rnx->nav, &rnx->sta, 0)) {
    trace(2, "open_rnxctr: rinex header read error\n");
    return 0;
  }
  if (!strchr(rnxtypes, type)) {
    trace(2, "open_rnxctr: not supported rinex type type=%c\n", type);
    return 0;
  }
  rnx->ver = ver;
  rnx->type = type;
  rnx->sys = sys;
  rnx->tsys = tsys;
  for (int i = 0; i < NUMSYS; i++)
    for (int j = 0; j < MAXOBSTYPE && *tobs[i][j]; j++) {
      rtkstrcpy(rnx->tobs[i][j], sizeof(rnx->tobs[0][0]), tobs[i][j]);
    }
  rnx->ephset = rnx->ephsat = 0;
  return 1;
}
/* Input RINEX control ---------------------------------------------------------
 * Fetch next RINEX message and input a message from file
 * Args   : rnxctr_t *rnx    IO  RINEX control struct
 *          FILE  *fp        I   file pointer
 * Return : status (-2: end of file, 0: no message, 1: input observation data,
 *                   2: input navigation data)
 * Notes  : if status=1, input obs data are set to rnx as follows:
 *            rnx->time      : obs data epoch time
 *            rnx->obs.n     : number of obs data
 *            rnx->obs.data[]: obs data
 *          if status=2, input nav data are set to rnx as follows:
 *            rnx->time      : ephemeris frame time
 *            rnx->ephsat    : sat-no of input ephemeris
 *            rnx->ephset    : set-no of input ephemeris (0:set1,1:set2)
 *            rnx->nav.geph[prn-1][0]     : GLOASS ephemeris (prn=slot-no)
 *            rnx->nav.seph[prn-MINPRNSBS][0]: SBAS ephemeris   (prn=PRN-no)
 *            rnx->nav.eph [sat-1][0]     : other ephemeris set1 (sat=sat-no)
 *            rnx->nav.eph [sat-1][1]     : other ephemeris set2 (sat=sat-no)
 *----------------------------------------------------------------------------*/
extern int input_rnxctr(rnxctr_t *rnx, FILE *fp) {
  trace(4, "input_rnxctr:\n");

  /* Read RINEX OBS data */
  if (rnx->type == 'O') {
    int flag;
    int n =
        readrnxobsb(fp, rnx->opt, rnx->ver, &rnx->tsys, rnx->tobs, &flag, rnx->obs.data, &rnx->sta);
    if (n <= 0) {
      rnx->obs.n = 0;
      return n < 0 ? -2 : 0;
    }
    rnx->time = rnx->obs.data[0].time;
    rnx->obs.n = n;
    return 1;
  }
  /* Read RINEX NAV data */
  int sys;
  switch (rnx->type) {
    case 'N':
      sys = SYS_NONE;
      break;
    case 'G':
      sys = SYS_GLO;
      break;
    case 'H':
      sys = SYS_SBS;
      break;
    case 'L':
      sys = SYS_GAL;
      break; /* Extension */
    case 'J':
      sys = SYS_QZS;
      break; /* Extension */
             // case 'C': ???
    default:
      return 0;
  }
  eph_t eph = {0};
  geph_t geph = {0};
  seph_t seph = {0};
  int type;
  int stat = readrnxnavb(fp, rnx->opt, rnx->ver, sys, &type, &eph, &geph, &seph);
  if (stat <= 0) {
    return stat < 0 ? -2 : 0;
  }
  if (type == 1) { /* GLONASS ephemeris */
    int prn;
    satsys(geph.sat, &prn);
    rnx->nav.geph[prn - 1][0] = geph;
    rnx->time = geph.tof;
    rnx->ephsat = geph.sat;
    rnx->ephset = 0;
  } else if (type == 2) { /* SBAS ephemeris */
    int prn;
    satsys(seph.sat, &prn);
    rnx->nav.seph[prn - MINPRNSBS][0] = seph;
    rnx->time = seph.tof;
    rnx->ephsat = seph.sat;
    rnx->ephset = 0;
  } else { /* Other ephemeris */
    int prn, sys2;
    sys2 = satsys(eph.sat, &prn);
    int set = (sys2 == SYS_GAL && (eph.code & (1 << 9))) ? 1 : 0; /* GAL 0:I/NAV,1:F/NAV */
    rnx->nav.eph[eph.sat - 1][set] = eph;
    rnx->time = eph.ttr;
    rnx->ephsat = eph.sat;
    rnx->ephset = set;
  }
  return 2;
}
/*------------------------------------------------------------------------------
 * Output RINEX functions
 *----------------------------------------------------------------------------*/

/* Output obs-types RINEX ver.2 ----------------------------------------------*/
static void outobstype_ver2(FILE *fp, const rnxopt_t *opt) {
  trace(3, "outobstype_ver2:\n");

  const char label[] = "# / TYPES OF OBSERV";

  fprintf(fp, "%6d", opt->nobs[0]);

  int i;
  for (i = 0; i < opt->nobs[0]; i++) {
    if (i > 0 && i % 9 == 0) fprintf(fp, "      ");

    fprintf(fp, "%6s", opt->tobs[0][i]);

    if (i % 9 == 8) fprintf(fp, "%-20s\n", label);
  }
  if (opt->nobs[0] == 0 || i % 9 > 0) {
    fprintf(fp, "%*s%-20s\n", (9 - i % 9) * 6, "", label);
  }
}
/* Output obs-types RINEX ver.3 ----------------------------------------------*/
static void outobstype_ver3(FILE *fp, const rnxopt_t *opt) {
  trace(3, "outobstype_ver3:\n");
  const char label[] = "SYS / # / OBS TYPES";

  for (int i = 0; navsys[i]; i++) {
    if (!(navsys[i] & opt->navsys) || !opt->nobs[i]) continue;

    fprintf(fp, "%c  %3d", syscodes[i], opt->nobs[i]);

    int j;
    for (j = 0; j < opt->nobs[i]; j++) {
      if (j > 0 && j % 13 == 0) fprintf(fp, "      ");

      char tobs[8];
      rtkstrcpy(tobs, sizeof(tobs), opt->tobs[i][j]);

      /* BDS B2x -> 1x (3.02), 2x (other) */
      if (navsys[i] == SYS_CMP) {
        if (opt->rnxver == 302 && tobs[1] == '2') tobs[1] = '1';
      }
      fprintf(fp, " %3s", tobs);

      if (j % 13 == 12) fprintf(fp, "  %-20s\n", label);
    }
    if (j % 13 > 0) {
      fprintf(fp, "%*s  %-20s\n", (13 - j % 13) * 4, "", label);
    }
  }
}
/* Output RINEX phase shift --------------------------------------------------*/
static void outrnx_phase_shift(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  static const uint8_t ref_code[][10] = {
      /* Reference signal [9] table A23 */
      {CODE_L1C, CODE_L2P, CODE_L5I, 0},                                         /* GPS */
      {CODE_L1C, CODE_L4A, CODE_L2C, CODE_L6A, CODE_L3I, 0},                     /* GLO */
      {CODE_L1B, CODE_L5I, CODE_L7I, CODE_L8I, CODE_L6B, 0},                     /* GAL */
      {CODE_L1C, CODE_L2S, CODE_L5I, CODE_L5D, CODE_L6S, 0},                     /* QZS */
      {CODE_L1C, CODE_L5I, 0},                                                   /* SBS */
      {CODE_L2I, CODE_L1D, CODE_L5D, CODE_L7I, CODE_L7D, CODE_L8D, CODE_L6I, 0}, /* BDS */
      {CODE_L5A, CODE_L9A, 0}                                                    /* IRN */
  };
  const char *label = "SYS / PHASE SHIFT";
  for (int i = 0; navsys[i]; i++) {
    if (!(navsys[i] & opt->navsys) || !opt->nobs[i]) continue;
    for (int j = 0; j < opt->nobs[i]; j++) {
      if (opt->tobs[i][j][0] != 'L') continue;
      char obs[8];
      rtkstrcpy(obs, sizeof(obs), opt->tobs[i][j]);
      int k;
      for (k = 0; ref_code[i][k]; k++) {
        if (obs2code(obs + 1) == ref_code[i][k]) break;
      }
      if (navsys[i] == SYS_CMP) { /* BDS B2x -> 1x (3.02), 2x (other) */
        if (opt->rnxver == 302 && obs[1] == '2') obs[1] = '1';
      }
      if (ref_code[i][k]) {
        fprintf(fp, "%c %3s %54s%-20s\n", syscodes[i], obs, "", label);
      } else {
        fprintf(fp, "%c %3s %8.5Lf%46s%-20s\n", syscodes[i], obs, opt->shift[i][j], "", label);
      }
    }
  }
}
/* Output RINEX GLONASS slot/freq # ------------------------------------------*/
static void outrnx_glo_fcn(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  const char *label = "GLONASS SLOT / FRQ #";
  int n = 0, prn[MAXPRNGLO], fcn[MAXPRNGLO];

  if (opt->navsys & SYS_GLO) {
    for (int i = 0; i < MAXPRNGLO; i++) {
      int sat = satno(SYS_GLO, i + 1);
      if (nav->geph[i][0].sat && nav->geph[i][0].sat != sat)
        fprintf(stderr, "** E30 %d %d %d\n", i, sat, nav->geph[i][0].sat);
      if (nav->geph[i][0].sat == sat) {
        prn[n] = i + 1;
        fcn[n++] = nav->geph[i][0].frq;
      } else if (nav->glo_fcn[i]) {
        prn[n] = i + 1;
        fcn[n++] = nav->glo_fcn[i] - 8;
      }
    }
  }
  for (int i = 0, j = 0; i < (n <= 0 ? 1 : (n - 1) / 8 + 1); i++) {
    if (i == 0)
      fprintf(fp, "%3d", n);
    else
      fprintf(fp, "   ");
    int k;
    for (k = 0; k < 8 && j < n; k++, j++) {
      fprintf(fp, " R%02d %2d", prn[j], fcn[j]);
    }
    fprintf(fp, "%*s %-20s\n", (8 - k) * 7, "", label);
  }
}
/* Output RINEX GLONASS code/phase/bias --------------------------------------*/
static void outrnx_glo_bias(FILE *fp, const rnxopt_t *opt) {
  const char *label = "GLONASS COD/PHS/BIS";

  if (opt->navsys & SYS_GLO) {
    const char *tobs[4] = {"C1C", "C1P", "C2C", "C2P"};
    fprintf(fp, " %s %8.3Lf %s %8.3Lf %s %8.3Lf %s %8.3Lf%8s%-20s\n", tobs[0], opt->glo_cp_bias[0],
            tobs[1], opt->glo_cp_bias[1], tobs[2], opt->glo_cp_bias[2], tobs[3],
            opt->glo_cp_bias[3], "", label);
  } else {
    fprintf(fp, "%*s%-20s\n", 60, "", label);
  }
}
/* Output RINEX observation data file header -----------------------------------
 * Output RINEX observation data file header
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  *nav      I   navigation data
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxobsh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxobsh:\n");

  char date[32];
  timestr_rnx(date);

  const char *sys, *tsys = "GPS";
  if (opt->navsys == SYS_GPS)
    sys = "G: GPS";
  else if (opt->navsys == SYS_GLO)
    sys = "R: GLONASS";
  else if (opt->navsys == SYS_GAL)
    sys = "E: Galileo";
  else if (opt->navsys == SYS_QZS)
    sys = "J: QZSS"; /* Ver.3.02 */
  else if (opt->navsys == SYS_CMP)
    sys = "C: BeiDou"; /* Ver.3.02 */
  else if (opt->navsys == SYS_IRN)
    sys = "I: IRNSS"; /* Ver.3.03 */
  else if (opt->navsys == SYS_SBS)
    sys = "S: SBAS Payload";
  else
    sys = "M: Mixed";

  fprintf(fp, "%9.2Lf%-11s%-20s%-20s%-20s\n", opt->rnxver / 100.0L, "", "OBSERVATION DATA", sys,
          "RINEX VERSION / TYPE");
  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  fprintf(fp, "%-60.60s%-20s\n", opt->marker, "MARKER NAME");
  fprintf(fp, "%-20.20s%-40.40s%-20s\n", opt->markerno, "", "MARKER NUMBER");

  if (opt->rnxver >= 300) {
    fprintf(fp, "%-20.20s%-40.40s%-20s\n", opt->markertype, "", "MARKER TYPE");
  }
  fprintf(fp, "%-20.20s%-40.40s%-20s\n", opt->name[0], opt->name[1], "OBSERVER / AGENCY");
  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->rec[0], opt->rec[1], opt->rec[2],
          "REC # / TYPE / VERS");
  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->ant[0], opt->ant[1], "", "ANT # / TYPE");

  long double pos[3] = {0};
  for (int i = 0; i < 3; i++)
    if (fabsl(opt->apppos[i]) < 1E8L) pos[i] = opt->apppos[i];
  long double del[3] = {0};
  for (int i = 0; i < 3; i++)
    if (fabsl(opt->antdel[i]) < 1E8L) del[i] = opt->antdel[i];
  fprintf(fp, "%14.4Lf%14.4Lf%14.4Lf%-18s%-20s\n", pos[0], pos[1], pos[2], "",
          "APPROX POSITION XYZ");
  fprintf(fp, "%14.4Lf%14.4Lf%14.4Lf%-18s%-20s\n", del[0], del[1], del[2], "",
          "ANTENNA: DELTA H/E/N");

  if (opt->rnxver <= 299) { /* Ver.2 */
    fprintf(fp, "%6d%6d%-48s%-20s\n", 1, 1, "", "WAVELENGTH FACT L1/2");
    outobstype_ver2(fp, opt);
  } else { /* Ver.3 */
    outobstype_ver3(fp, opt);
  }
  if (opt->tint > 0.0L) {
    fprintf(fp, "%10.3Lf%50s%-20s\n", opt->tint, "", "INTERVAL");
  }
  long double ep[6];
  time2epoch(opt->tstart, ep);
  fprintf(fp, "  %04.0Lf    %02.0Lf    %02.0Lf    %02.0Lf    %02.0Lf   %010.7Lf     %-12s%-20s\n",
          ep[0], ep[1], ep[2], ep[3], ep[4], ep[5], tsys, "TIME OF FIRST OBS");

  time2epoch(opt->tend, ep);
  fprintf(fp, "  %04.0Lf    %02.0Lf    %02.0Lf    %02.0Lf    %02.0Lf   %010.7Lf     %-12s%-20s\n",
          ep[0], ep[1], ep[2], ep[3], ep[4], ep[5], tsys, "TIME OF LAST OBS");

  if (opt->rnxver >= 301) {
    outrnx_phase_shift(fp, opt, nav); /* SYS / PHASE SHIFT */
  }
  if (opt->rnxver >= 302) {
    outrnx_glo_fcn(fp, opt, nav); /* GLONASS SLOT / FRQ # */
    outrnx_glo_bias(fp, opt);     /* GLONASS COD/PHS/BIS */
  }
  return fprintf(fp, "%-60.60s%-20s\n", "", "END OF HEADER") != EOF;
}
/* Output observation data field ---------------------------------------------*/
static void outrnxobsf(FILE *fp, long double obs, int lli, int std) {
  if (obs == 0.0L) {
    fprintf(fp, "              ");
  } else {
    fprintf(fp, "%14.3Lf", fmodl(obs, 1e9L));
  }
  if (lli < 0 || !(lli & (LLI_SLIP | LLI_HALFC | LLI_BOCTRK))) {
    fprintf(fp, " ");
  } else {
    fprintf(fp, "%1.1d", lli & (LLI_SLIP | LLI_HALFC | LLI_BOCTRK));
  }
  if (std <= 0)
    fprintf(fp, " ");
  else
    fprintf(fp, "%1.1x", std);
}
/* Search observation data index ---------------------------------------------*/
static int obsindex(int rnxver, int sys, const uint8_t *code, const char *tobs, const char *mask) {
  for (int i = 0; i < NFREQ + NEXOBS; i++) {
    /* Signal mask */
    if (mask[code[i] - 1] == '0') continue;

    if (rnxver <= 299) { /* Ver.2 */
      if (!strcmp(tobs, "C1") && (sys == SYS_GPS || sys == SYS_GLO || sys == SYS_QZS ||
                                  sys == SYS_SBS || sys == SYS_CMP)) {
        if (code[i] == CODE_L1C) return i;
      } else if (!strcmp(tobs, "P1")) {
        if (code[i] == CODE_L1P || code[i] == CODE_L1W || code[i] == CODE_L1Y ||
            code[i] == CODE_L1N)
          return i;
      } else if (!strcmp(tobs, "C2") && (sys == SYS_GPS || sys == SYS_QZS)) {
        if (code[i] == CODE_L2S || code[i] == CODE_L2L || code[i] == CODE_L2X) return i;
      } else if (!strcmp(tobs, "C2") && sys == SYS_GLO) {
        if (code[i] == CODE_L2C) return i;
      } else if (!strcmp(tobs, "P2")) {
        if (code[i] == CODE_L2P || code[i] == CODE_L2W || code[i] == CODE_L2Y ||
            code[i] == CODE_L2N || code[i] == CODE_L2D)
          return i;
      } else if (rnxver >= 212 && tobs[1] == 'A') { /* L1C/A */
        if (code[i] == CODE_L1C) return i;
      } else if (rnxver >= 212 && tobs[1] == 'B') { /* L1C */
        if (code[i] == CODE_L1S || code[i] == CODE_L1L || code[i] == CODE_L1X) return i;
      } else if (rnxver >= 212 && tobs[1] == 'C') { /* L2C */
        if (code[i] == CODE_L2S || code[i] == CODE_L2L || code[i] == CODE_L2X) return i;
      } else if (rnxver >= 212 && tobs[1] == 'D' && sys == SYS_GLO) { /* GLO L2C/A */
        if (code[i] == CODE_L2C) return i;
      } else if (tobs[1] == '2' && sys == SYS_CMP) { /* BDS B1 */
        if (code[i] == CODE_L2I || code[i] == CODE_L2Q || code[i] == CODE_L2X) return i;
      } else {
        const char *id = code2obs(code[i]);
        if (id[0] == tobs[1]) return i;
      }
    } else { /* Ver.3 */
      const char *id = code2obs(code[i]);
      if (!strcmp(id, tobs + 1)) return i;
    }
  }
  return -1;
}
/* Output RINEX event time ---------------------------------------------------*/
static void outrinexevent(FILE *fp, const rnxopt_t *opt, const obsd_t *obs,
                          const long double epdiff) {
  /* Reject invalid time events (> 1 minute from timestamp of epoch)  */
  if (fabsl(epdiff) > 60) {
    return;
  }

  long double epe[6];
  time2epoch(obs[0].eventime, epe);
  int n = obs->timevalid ? 0 : 1;

  if (opt->rnxver <= 299L) { /* Ver.2 */
    if (epdiff < 0) fprintf(fp, "\n");
    fprintf(fp, " %02d %2.0Lf %2.0Lf %2.0Lf %2.0Lf%11.7Lf  %d%3d", (int)epe[0] % 100, epe[1],
            epe[2], epe[3], epe[4], epe[5], 5, n);
    if (epdiff >= 0) fprintf(fp, "\n");
  } else { /* Ver.3 */
    fprintf(fp, "> %04.0Lf %2.0Lf %2.0Lf %2.0Lf %2.0Lf%11.7Lf  %d%3d\n", epe[0], epe[1], epe[2],
            epe[3], epe[4], epe[5], 5, n);
  }
  if (n) fprintf(fp, "%-60.60s%-20s\n", " Time mark is not valid", "COMMENT");
}
/* Output RINEX observation data body ------------------------------------------
 * Output RINEX observation data body
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          obsd_t *obs      I   observation data
 *          int    n         I   number of observation data
 *          int    flag      I   epoch flag (0:ok,1:power failure,>1:event flag)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxobsb(FILE *fp, const rnxopt_t *opt, const obsd_t *obs, int n, int flag) {
  trace(3, "outrnxobsb: n=%d\n", n);

  long double ep[6];
  time2epoch_n(obs[0].time, ep, 7); /* Output rounded to 7 decimals */

  int ns = 0;
  char sats[MAXOBS][4] = {""};
  int ind[MAXOBS], s[MAXOBS] = {0};
  for (int i = 0; i < n && ns < MAXOBS; i++) {
    int sys = satsys(obs[i].sat, NULL);
    if (!(sys & opt->navsys) || opt->exsats[obs[i].sat - 1]) continue;
    if (!sat2code(obs[i].sat, sats[ns])) continue;
    switch (sys) {
      case SYS_GPS:
        s[ns] = 0;
        break;
      case SYS_GLO:
        s[ns] = 1;
        break;
      case SYS_GAL:
        s[ns] = 2;
        break;
      case SYS_QZS:
        s[ns] = 3;
        break;
      case SYS_SBS:
        s[ns] = 4;
        break;
      case SYS_CMP:
        s[ns] = 5;
        break;
      case SYS_IRN:
        s[ns] = 6;
        break;
    }
    if (!opt->nobs[(opt->rnxver <= 299) ? 0 : s[ns]]) continue;
    ind[ns++] = i;
  }
  if (ns <= 0) return true;
  /* If epoch of event less than epoch of observation, then first output
  Time mark, else first output observation record */
  long double epdiff = timediff(obs[0].time, obs[0].eventime);
  if (flag == 5 && epdiff >= 0) {
    outrinexevent(fp, opt, obs, epdiff);
  }
  if (opt->rnxver <= 299) { /* Ver.2 */
    fprintf(fp, " %02d %02.0Lf %02.0Lf %02.0Lf %02.0Lf %010.7Lf  %d%3d", (int)ep[0] % 100, ep[1],
            ep[2], ep[3], ep[4], ep[5], 0, ns);
    for (int i = 0; i < ns; i++) {
      if (i > 0 && i % 12 == 0) fprintf(fp, "\n%32s", "");
      fprintf(fp, "%-3s", sats[i]);
    }
  } else { /* Ver.3 */
    fprintf(fp, "> %04.0Lf %02.0Lf %02.0Lf %02.0Lf %02.0Lf %010.7Lf  %d%3d%21s\n", ep[0], ep[1],
            ep[2], ep[3], ep[4], ep[5], 0, ns, "");
  }
  for (int i = 0; i < ns; i++) {
    int sys = satsys(obs[ind[i]].sat, NULL);

    int m;
    const char *mask;
    if (opt->rnxver <= 299) { /* Ver.2 */
      m = 0;
      mask = opt->mask[s[i]];
    } else { /* Ver.3 */
      fprintf(fp, "%-3s", sats[i]);
      m = s[i];
      mask = opt->mask[s[i]];
    }
    for (int j = 0; j < opt->nobs[m]; j++) {
      if (opt->rnxver <= 299) { /* Ver.2 */
        if (j % 5 == 0) fprintf(fp, "\n");
      }
      /* Search obs data index */
      int k = obsindex(opt->rnxver, sys, obs[ind[i]].code, opt->tobs[m][j], mask);
      if (k < 0) {
        outrnxobsf(fp, 0.0L, -1, -1);
        continue;
      }
      /* Phase shift (cyc) */
      long double dL = (obs[ind[i]].L[k] != 0.0L) ? opt->shift[m][j] : 0.0L;

      /* Output field */
      switch (opt->tobs[m][j][0]) {
        case 'C':
        case 'P':
          outrnxobsf(fp, obs[ind[i]].P[k], -1, obs[ind[i]].Pstd[k]);
          break;
        case 'L':
          outrnxobsf(fp, obs[ind[i]].L[k] + dL, obs[ind[i]].LLI[k], obs[ind[i]].Lstd[k]);
          break;
        case 'D':
          outrnxobsf(fp, obs[ind[i]].D[k], -1, -1);
          break;
        case 'S':
          outrnxobsf(fp, obs[ind[i]].SNR[k] * SNR_UNIT, -1, -1);
          break;
      }
    }

    /* Set trace level to 1 generate CSV file of raw observations   */
#ifdef TRACE
    if (gettracelevel() == 1) {
      trace(1,
            ",%16.2Lf,%3d,%13.2Lf,%13.2Lf,%9.2Lf,%2.0Lf,%1d,%1d,%13.2Lf,%13.2Lf,%9.2Lf,%2.0Lf,%1d,%"
            "1d\n",
            obs[0].time.time + obs[0].time.sec, obs[ind[i]].sat, obs[ind[i]].P[0], obs[ind[i]].L[0],
            obs[ind[i]].D[0], obs[ind[i]].SNR[0] * SNR_UNIT, obs[ind[i]].LLI[0],
            obs[ind[i]].Lstd[0], obs[ind[i]].P[1], obs[ind[i]].L[1], obs[ind[i]].D[1],
            obs[ind[i]].SNR[1] * SNR_UNIT, obs[ind[i]].LLI[1], obs[ind[i]].Lstd[1]);
    }
#endif

    if (opt->rnxver >= 300 && fprintf(fp, "\n") == EOF) return false;
  }

  if (flag == 5 && epdiff < 0) {
    outrinexevent(fp, opt, obs, epdiff);
  }
  if (opt->rnxver >= 300) return true;

  return fprintf(fp, "\n") != EOF;
}
/* Output data field in RINEX navigation data --------------------------------*/
static void outnavf_n(FILE *fp, long double value, int n) {
  long double e = (fabsl(value) < 1E-99L) ? 0.0L : floorl(log10l(fabsl(value)) + 1.0L);

  fprintf(fp, " %s.%0*.0Lf%s%+03.0Lf", value < 0.0L ? "-" : " ", n,
          fabsl(value) / powl(10.0L, e - n), NAVEXP, e);
}
static void outnavf(FILE *fp, long double value) { outnavf_n(fp, value, 12); }
/* Output iono correction for a system ---------------------------------------*/
static void out_iono_sys(FILE *fp, const char *sys, const long double *ion, int n) {
  const char *label1[] = {"ION ALPHA", "ION BETA"}, *label2 = "IONOSPHERIC CORR";

  if (norm(ion, n) <= 0.0L) return;

  for (int i = 0; i < (n + 3) / 4; i++) {
    char str[32];
    rtksnprintf(str, sizeof(str), "%s%c", sys, (!*sys || n < 4) ? ' ' : 'A' + i);
    fprintf(fp, "%-*s ", !*sys ? 1 : 4, str);
    int j;
    for (j = 0; j < 4 && i * 4 + j < n; j++) {
      fprintf(fp, " ");
      outnavf_n(fp, ion[i * 4 + j], 4);
    }
    fprintf(fp, "%*s%-20s\n", !*sys ? 10 : 7 + 12 * (4 - j), "", !*sys ? label1[i] : label2);
  }
}
/* Output iono corrections ---------------------------------------------------*/
static void out_iono(FILE *fp, int sys, const rnxopt_t *opt, const nav_t *nav) {
  if (!opt->outiono) return;

  if (sys & opt->navsys & SYS_GPS) {
    if (opt->rnxver <= 211)
      out_iono_sys(fp, "", nav->ion_gps, 8);
    else
      out_iono_sys(fp, "GPS", nav->ion_gps, 8);
  }
  if ((sys & opt->navsys & SYS_GAL) && opt->rnxver >= 212) {
    out_iono_sys(fp, "GAL", nav->ion_gal, 3);
  }
  if ((sys & opt->navsys & SYS_QZS) && opt->rnxver >= 302) {
    out_iono_sys(fp, "QZS", nav->ion_qzs, 8);
  }
  if ((sys & opt->navsys & SYS_CMP) && opt->rnxver >= 302) {
    out_iono_sys(fp, "BDS", nav->ion_cmp, 8);
  }
  if ((sys & opt->navsys & SYS_IRN) && opt->rnxver >= 303) {
    out_iono_sys(fp, "IRN", nav->ion_irn, 8);
  }
}
/* Output time system correction for a system --------------------------------*/
static void out_time_sys(FILE *fp, const char *sys, const long double *utc) {
  if (norm(utc, 3) <= 0.0L) return;

  if (*sys) {
    fprintf(fp, "%-4s ", sys);
    outnavf_n(fp, utc[0], 10);
    outnavf_n(fp, utc[1], 9);
    const char *label1 = "TIME SYSTEM CORR";
    fprintf(fp, "%7.0Lf%5.0Lf%10s%-20s\n", utc[2], utc[3], "", label1);
  } else {
    fprintf(fp, "   ");
    outnavf_n(fp, utc[0], 12);
    outnavf_n(fp, utc[1], 12);
    const char *label2 = "DELTA-UTC: A0,A1,T,W";
    fprintf(fp, "%9.0Lf%9.0Lf %-20s\n", utc[2], utc[3], label2);
  }
}
/* Output time system corrections --------------------------------------------*/
static void out_time(FILE *fp, int sys, const rnxopt_t *opt, const nav_t *nav) {
  if (!opt->outtime) return;

  if (sys & opt->navsys & SYS_GPS) {
    if (opt->rnxver <= 211)
      out_time_sys(fp, "", nav->utc_gps);
    else
      out_time_sys(fp, "GPUT", nav->utc_gps);
  }
  if ((sys & opt->navsys & SYS_GLO) && opt->rnxver >= 212) {
    /* RINEX 2.12-3.02: tau_C, 3.03- : -tau_C */
    long double utc[8] = {0};
    utc[0] = (opt->rnxver <= 302) ? nav->utc_glo[0] : -nav->utc_glo[0];
    out_time_sys(fp, "GLUT", utc);
  }
  if ((sys & opt->navsys & SYS_SBS) && opt->rnxver >= 212) {
    out_time_sys(fp, "SBUT", nav->utc_sbs);
  }
  if ((sys & opt->navsys & SYS_GAL) && opt->rnxver >= 212) {
    out_time_sys(fp, "GAUT", nav->utc_gal);
  }
  if ((sys & opt->navsys & SYS_QZS) && opt->rnxver >= 302) {
    out_time_sys(fp, "QZUT", nav->utc_qzs);
  }
  if ((sys & opt->navsys & SYS_CMP) && opt->rnxver >= 302) {
    out_time_sys(fp, "BDUT", nav->utc_cmp);
  }
  if ((sys & opt->navsys & SYS_IRN) && opt->rnxver >= 303) {
    out_time_sys(fp, "IRUT", nav->utc_irn);
  }
}
/* Output leap seconds -------------------------------------------------------*/
static void out_leaps(FILE *fp, int sys, const rnxopt_t *opt, const nav_t *nav) {
  const char *label = "LEAP SECONDS";

  if (!opt->outleaps) return;

  const long double *leaps;
  switch (sys) {
    case SYS_GAL:
      leaps = nav->utc_gal + 4;
      break;
    case SYS_QZS:
      leaps = nav->utc_qzs + 4;
      break;
    case SYS_CMP:
      leaps = nav->utc_cmp + 4;
      break;
    case SYS_IRN:
      leaps = nav->utc_irn + 4;
      break;
    default:
      leaps = nav->utc_gps + 4;
      break;
  }
  if (leaps[0] == 0.0L) return;

  if (opt->rnxver <= 300) {
    if (sys == SYS_GPS) fprintf(fp, "%6.0Lf%54s%-20s\n", leaps[0], "", label);
  } else if (norm(leaps + 1, 3) <= 0.0L) {
    fprintf(fp, "%6.0Lf%18s%3s%33s%-20s\n", leaps[0], "", (sys == SYS_CMP) ? "BDS" : "", "", label);
  } else {
    fprintf(fp, "%6.0Lf%6.0Lf%6.0Lf%6.0Lf%3s%33s%-20s\n", leaps[0], leaps[3], leaps[1], leaps[2],
            (sys == SYS_CMP) ? "BDS" : "", "", label);
  }
}
/* Output RINEX navigation data file header ------------------------------------
 * Output RINEX navigation data file header
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  nav       I   navigation data (NULL: no input)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxnavh:\n");

  char date[32];
  timestr_rnx(date);

  if (opt->rnxver <= 299) { /* Ver.2 */
    fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GPS NAV DATA", "",
            "RINEX VERSION / TYPE");
  } else { /* Ver.3 */
    const char *sys;
    if (opt->navsys == SYS_GPS)
      sys = "G: GPS";
    else if (opt->navsys == SYS_GLO)
      sys = "R: GLONASS";
    else if (opt->navsys == SYS_GAL)
      sys = "E: Galileo";
    else if (opt->navsys == SYS_QZS)
      sys = "J: QZSS"; /* v.3.02 */
    else if (opt->navsys == SYS_CMP)
      sys = "C: BeiDou"; /* v.3.02 */
    else if (opt->navsys == SYS_IRN)
      sys = "I: IRNSS"; /* v.3.03 */
    else if (opt->navsys == SYS_SBS)
      sys = "S: SBAS Payload";
    else if (opt->sep_nav)
      sys = "G: GPS";
    else
      sys = "M: Mixed";

    fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GNSS NAV DATA", sys,
            "RINEX VERSION / TYPE");
  }
  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  out_iono(fp, opt->sep_nav ? SYS_GPS : SYS_ALL, opt, nav);
  out_time(fp, opt->sep_nav ? SYS_GPS : SYS_ALL, opt, nav);
  out_leaps(fp, SYS_GPS, opt, nav);

  return fprintf(fp, "%60s%-20s\n", "", "END OF HEADER") != EOF;
}
/* Output RINEX navigation data file body --------------------------------------
 * Output RINEX navigation data file body
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          eph_t  *eph      I   ephemeris
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxnavb(FILE *fp, const rnxopt_t *opt, const eph_t *eph) {
  trace(3, "outrnxnavb: sat=%2d\n", eph->sat);

  int prn;
  int sys = satsys(eph->sat, &prn);
  if (!sys || !(sys & opt->navsys)) return false;

  long double ep[6];
  if (sys != SYS_CMP) {
    time2epoch(eph->toc, ep);
  } else {
    time2epoch(gpst2bdt(eph->toc), ep); /* GPST -> BDT */
  }
  const char *sep;
  if ((opt->rnxver >= 300 && sys == SYS_GPS) || (opt->rnxver >= 212 && sys == SYS_GAL) ||
      (opt->rnxver >= 302 && sys == SYS_QZS) || (opt->rnxver >= 302 && sys == SYS_CMP) ||
      (opt->rnxver >= 303 && sys == SYS_IRN)) {
    char code[32];
    if (!sat2code(eph->sat, code)) return false;
    fprintf(fp, "%-3s %04.0Lf %02.0Lf %02.0Lf %02.0Lf %02.0Lf %02.0Lf", code, ep[0], ep[1], ep[2],
            ep[3], ep[4], ep[5]);
    sep = "    ";
  } else if (opt->rnxver <= 299 && sys == SYS_GPS) {
    fprintf(fp, "%2d %02d %02.0Lf %02.0Lf %02.0Lf %02.0Lf %04.1Lf", prn, (int)ep[0] % 100, ep[1],
            ep[2], ep[3], ep[4], ep[5]);
    sep = "   ";
  } else {
    return false;
  }
  outnavf(fp, eph->f0);
  outnavf(fp, eph->f1);
  outnavf(fp, eph->f2);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, eph->iode); /* GPS/QZS: IODE, GAL: IODnav, BDS: AODE */
  outnavf(fp, eph->crs);
  outnavf(fp, eph->deln);
  outnavf(fp, eph->M0);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, eph->cuc);
  outnavf(fp, eph->e);
  outnavf(fp, eph->cus);
  outnavf(fp, sqrtl(eph->A));
  fprintf(fp, "\n%s", sep);

  outnavf(fp, eph->toes);
  outnavf(fp, eph->cic);
  outnavf(fp, eph->OMG0);
  outnavf(fp, eph->cis);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, eph->i0);
  outnavf(fp, eph->crc);
  outnavf(fp, eph->omg);
  outnavf(fp, eph->OMGd);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, eph->idot);
  outnavf(fp, eph->code);
  outnavf(fp, eph->week); /* GPS/QZS: GPS week, GAL: GAL week, BDS: BDT week */
  if (sys == SYS_GPS || sys == SYS_QZS) {
    outnavf(fp, eph->flag);
  } else {
    outnavf(fp, 0.0L); /* Spare */
  }
  fprintf(fp, "\n%s", sep);

  if (sys == SYS_GAL) {
    outnavf(fp, sisa_value(eph->sva));
  } else {
    outnavf(fp, uravalue(eph->sva));
  }
  outnavf(fp, eph->svh);
  outnavf(fp, eph->tgd[0]); /* GPS/QZS:TGD, GAL:BGD E5a/E1, BDS: TGD1 B1/B3 */
  if (sys == SYS_GAL || sys == SYS_CMP) {
    outnavf(fp, eph->tgd[1]); /* GAL:BGD E5b/E1, BDS: TGD2 B2/B3 */
  } else if (sys == SYS_GPS || sys == SYS_QZS) {
    outnavf(fp, eph->iodc); /* GPS/QZS:IODC */
  } else {
    outnavf(fp, 0.0L); /* Spare */
  }
  fprintf(fp, "\n%s", sep);

  long double ttr;
  int week;
  if (sys != SYS_CMP) {
    ttr = time2gpst(eph->ttr, &week);
  } else {
    ttr = time2bdt(gpst2bdt(eph->ttr), &week); /* GPST -> BDT */
  }
  outnavf(fp, ttr + (week - eph->week) * 604800.0L);

  if (sys == SYS_GPS) {
    outnavf(fp, eph->fit);
  } else if (sys == SYS_QZS) {
    outnavf(fp, eph->fit > 2.0L ? 1.0L : 0.0L);
  } else if (sys == SYS_CMP) {
    outnavf(fp, eph->iodc); /* AODC */
  } else {
    outnavf(fp, 0.0L); /* Spare */
  }
  return fprintf(fp, "\n") != EOF;
}
/* Output RINEX GNAV file header -----------------------------------------------
 * Output RINEX GNAV (GLONASS navigation data) file header
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  nav       I   navigation data (NULL: no input)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxgnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxgnavh:\n");

  char date[32];
  timestr_rnx(date);

  if (opt->rnxver <= 299) { /* Ver.2 */
    fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "GLONASS NAV DATA", "",
            "RINEX VERSION / TYPE");
  } else { /* Ver.3 */
    fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GNSS NAV DATA",
            "R: GLONASS", "RINEX VERSION / TYPE");
  }
  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  out_time(fp, SYS_GLO, opt, nav);
  out_leaps(fp, SYS_GPS, opt, nav);

  return fprintf(fp, "%60s%-20s\n", "", "END OF HEADER") != EOF;
}
/* Output RINEX GNAV file body -------------------------------------------------
 * Output RINEX GNAV (GLONASS navigation data) file body
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          geph_t  *geph    I   GLONASS ephemeris
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxgnavb(FILE *fp, const rnxopt_t *opt, const geph_t *geph) {
  trace(3, "outrnxgnavb: sat=%2d\n", geph->sat);

  int prn;
  if ((satsys(geph->sat, &prn) & opt->navsys) != SYS_GLO) return false;

  long double tof = time2gpst(gpst2utc(geph->tof), NULL); /* v.3: tow in UTC */
  if (opt->rnxver <= 299) tof = fmodl(tof, 86400.0L);     /* v.2: tod in UTC */

  gtime_t toe = gpst2utc(geph->toe); /* GPST -> UTC */
  long double ep[6];
  time2epoch(toe, ep);

  const char *sep;
  if (opt->rnxver <= 299) { /* Ver.2 */
    fprintf(fp, "%2d %02d %02.0Lf %02.0Lf %02.0Lf %02.0Lf %04.1Lf", prn, (int)ep[0] % 100, ep[1],
            ep[2], ep[3], ep[4], ep[5]);
    sep = "   ";
  } else { /* Ver.3 */
    char code[32];
    if (!sat2code(geph->sat, code)) return false;
    fprintf(fp, "%-3s %04.0Lf %02.0Lf %02.0Lf %02.0Lf %02.0Lf %02.0Lf", code, ep[0], ep[1], ep[2],
            ep[3], ep[4], ep[5]);
    sep = "    ";
  }
  outnavf(fp, -geph->taun);
  outnavf(fp, geph->gamn);
  outnavf(fp, tof);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, geph->pos[0] / 1E3L);
  outnavf(fp, geph->vel[0] / 1E3L);
  outnavf(fp, geph->acc[0] / 1E3L);
  outnavf(fp, geph->svh);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, geph->pos[1] / 1E3L);
  outnavf(fp, geph->vel[1] / 1E3L);
  outnavf(fp, geph->acc[1] / 1E3L);
  outnavf(fp, geph->frq);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, geph->pos[2] / 1E3L);
  outnavf(fp, geph->vel[2] / 1E3L);
  outnavf(fp, geph->acc[2] / 1E3L);
#ifdef RTK_DISABLED /* Input dtaun instead of age */
  outnavf(fp, geph->dtaun);
#else
  outnavf(fp, geph->age);
#endif
  return fprintf(fp, "\n") != EOF;
}
/* Output RINEX GEO navigation data file header --------------------------------
 * Output RINEX GEO navigation data file header
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  nav       I   navigation data (NULL: no input)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxhnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxhnavh:\n");

  char date[32];
  timestr_rnx(date);

  if (opt->rnxver <= 299) { /* Ver.2 */
    fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "H: GEO NAV MSG DATA",
            "", "RINEX VERSION / TYPE");
  } else { /* Ver.3 */
    fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GNSS NAV DATA",
            "S: SBAS Payload", "RINEX VERSION / TYPE");
  }
  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  out_time(fp, SYS_SBS, opt, nav);
  out_leaps(fp, SYS_GPS, opt, nav);

  return fprintf(fp, "%60s%-20s\n", "", "END OF HEADER") != EOF;
}
/* Output RINEX GEO navigation data file body ----------------------------------
 * Output RINEX GEO navigation data file body
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          seph_t  *seph    I   SBAS ephemeris
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxhnavb(FILE *fp, const rnxopt_t *opt, const seph_t *seph) {
  trace(3, "outrnxhnavb: sat=%2d\n", seph->sat);

  int prn;
  if ((satsys(seph->sat, &prn) & opt->navsys) != SYS_SBS) return false;

  long double ep[6];
  time2epoch(seph->t0, ep);

  const char *sep;
  if (opt->rnxver <= 299) { /* Ver.2 */
    fprintf(fp, "%2d %02d %2.0Lf %2.0Lf %2.0Lf %2.0Lf %4.1Lf", prn - 100, (int)ep[0] % 100, ep[1],
            ep[2], ep[3], ep[4], ep[5]);
    sep = "   ";
  } else { /* Ver.3 */
    char code[32];
    if (!sat2code(seph->sat, code)) return false;
    fprintf(fp, "%-3s %04.0Lf %2.0Lf %2.0Lf %2.0Lf %2.0Lf %2.0Lf", code, ep[0], ep[1], ep[2], ep[3],
            ep[4], ep[5]);
    sep = "    ";
  }
  outnavf(fp, seph->af0);
  outnavf(fp, seph->af1);
  outnavf(fp, time2gpst(seph->tof, NULL));
  fprintf(fp, "\n%s", sep);

  outnavf(fp, seph->pos[0] / 1E3L);
  outnavf(fp, seph->vel[0] / 1E3L);
  outnavf(fp, seph->acc[0] / 1E3L);
  outnavf(fp, seph->svh);
  fprintf(fp, "\n%s", sep);

  outnavf(fp, seph->pos[1] / 1E3L);
  outnavf(fp, seph->vel[1] / 1E3L);
  outnavf(fp, seph->acc[1] / 1E3L);
  outnavf(fp, uravalue(seph->sva));
  fprintf(fp, "\n%s", sep);

  outnavf(fp, seph->pos[2] / 1E3L);
  outnavf(fp, seph->vel[2] / 1E3L);
  outnavf(fp, seph->acc[2] / 1E3L);
  outnavf(fp, 0);

  return fprintf(fp, "\n") != EOF;
}
/* Output RINEX Galileo NAV header ---------------------------------------------
 * Output RINEX Galileo NAV file header (2.12)
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  nav       I   navigation data (NULL: no input)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxlnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxlnavh:\n");

  if (opt->rnxver < 212) return false;

  char date[32];
  timestr_rnx(date);

  fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GNSS NAV DATA",
          "E: Galileo", "RINEX VERSION / TYPE");

  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  out_iono(fp, SYS_GAL, opt, nav);
  out_time(fp, SYS_GAL, opt, nav);
  out_leaps(fp, SYS_GAL, opt, nav);

  return fprintf(fp, "%60s%-20s\n", "", "END OF HEADER") != EOF;
}
/* Output RINEX QZSS navigation data file header -------------------------------
 * Output RINEX QZSS navigation data file header
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  nav       I   navigation data (NULL: no input)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxqnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxqnavh:\n");

  if (opt->rnxver < 302) return false;

  char date[32];
  timestr_rnx(date);

  fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GNSS NAV DATA",
          "J: QZSS", "RINEX VERSION / TYPE");

  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  out_iono(fp, SYS_QZS, opt, nav);
  out_time(fp, SYS_QZS, opt, nav);
  out_leaps(fp, SYS_QZS, opt, nav);

  return fprintf(fp, "%60s%-20s\n", "", "END OF HEADER") != EOF;
}
/* Output RINEX BDS navigation data file header --------------------------------
 * Output RINEX BDS navigation data file header
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  nav       I   navigation data (NULL: no input)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxcnavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxcnavh:\n");

  if (opt->rnxver < 302) return false;

  char date[32];
  timestr_rnx(date);

  fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GNSS NAV DATA",
          "C: BeiDou", "RINEX VERSION / TYPE");

  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  out_iono(fp, SYS_CMP, opt, nav);
  out_time(fp, SYS_CMP, opt, nav);
  out_leaps(fp, SYS_CMP, opt, nav);

  return fprintf(fp, "%60s%-20s\n", "", "END OF HEADER") != EOF;
}
/* Output RINEX NavIC/IRNSS navigation data file header ------------------------
 * Output RINEX NavIC/IRNSS navigation data file header
 * Args   : FILE   *fp       I   output file pointer
 *          rnxopt_t *opt    I   RINEX options
 *          nav_t  nav       I   navigation data (NULL: no input)
 * Return : status (true:ok, false:output error)
 *----------------------------------------------------------------------------*/
extern bool outrnxinavh(FILE *fp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "outrnxinavh:\n");

  if (opt->rnxver < 303) return false;

  char date[32];
  timestr_rnx(date);

  fprintf(fp, "%9.2Lf           %-20s%-20s%-20s\n", opt->rnxver / 100.0L, "N: GNSS NAV DATA",
          "I: IRNSS", "RINEX VERSION / TYPE");

  fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", opt->prog, opt->runby, date,
          "PGM / RUN BY / DATE");

  for (int i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) continue;
    fprintf(fp, "%-60.60s%-20s\n", opt->comment[i], "COMMENT");
  }
  out_iono(fp, SYS_IRN, opt, nav);
  out_time(fp, SYS_IRN, opt, nav);
  out_leaps(fp, SYS_IRN, opt, nav);

  return fprintf(fp, "%60s%-20s\n", "", "END OF HEADER") != EOF;
}
