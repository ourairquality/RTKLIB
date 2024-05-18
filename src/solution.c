/*------------------------------------------------------------------------------
 * solution.c : solution functions
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * references :
 *     [1] National Marine Electronic Association and International Marine
 *         Electronics Association, NMEA 0183 version 4.10, August 1, 2012
 *     [2] NMEA 0183 Talker Identifier Mnemonics, March 3, 2019
 *         (https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
 *
 * version : $Revision:$ $Date:$
 * history : 2007/11/03  1.0 new
 *           2009/01/05  1.1  add function outsols(), outsolheads(),
 *                            setsolformat(), outsolexs, outsolex
 *           2009/04/02  1.2  add dummy fields in NMEA mesassage
 *                            fix bug to format lat/lon as deg-min-sec
 *           2009/04/14  1.3  add age and ratio field to solution
 *           2009/11/25  1.4  add function readsolstat()
 *           2010/02/14  1.5  fix bug on output of gpstime at week boundary
 *           2010/07/05  1.6  added api:
 *                                initsolbuf(),freesolbuf(),addsol(),getsol(),
 *                                inputsol(),outprcopts(),outprcopt()
 *                            modified api:
 *                                readsol(),readsolt(),readsolstat(),
 *                                readsolstatt(),outsolheads(),outsols(),
 *                                outsolexs(),outsolhead(),outsol(),outsolex(),
 *                                outnmea_rmc(),outnmea_gga(),outnmea_gsa(),
 *                                outnmea_gsv()
 *                            deleted api:
 *                                setsolopt(),setsolformat()
 *           2010/08/14  1.7  fix bug on initialize solution buffer (2.4.0_p2)
 *                            suppress enu-solution if base pos not available
 *                            (2.4.0_p3)
 *           2010/08/16  1.8  suppress null record if solution is not available
 *                            (2.4.0_p4)
 *           2011/01/23  1.9  fix bug on reading nmea solution data
 *                            add api freesolstatbuf()
 *           2012/02/05  1.10 fix bug on output nmea gpgsv
 *           2013/02/18  1.11 support nmea GLGSA,GAGSA,GLCSV,GACSV sentence
 *           2013/09/01  1.12 fix bug on presentation of nmea time tag
 *           2015/02/11  1.13 fix bug on checksum of $GLGSA and $GAGSA
 *                            fix bug on satellite id of $GAGSA
 *           2016/01/17  1.14 support reading NMEA GxZDA
 *                            ignore NMEA talker ID
 *           2016/07/30  1.15 suppress output if std is over opt->maxsolstd
 *           2017/06/13  1.16 support output/input of velocity solution
 *           2018/10/10  1.17 support reading solution status file
 *           2020/11/30  1.18 add NMEA talker ID GQ and GI (NMEA 0183 4.11)
 *                            add NMEA GQ/GB/GI-GSA/GSV sentences
 *                            change talker ID GP to GN for NMEA RMC/GGA
 *                            change newline to "\r\n" in SOLF_LLH,XYZ,ENU
 *                            add reading age information in NMEA GGA
 *                            use integer types in stdint.h
 *                            suppress warnings
 *-----------------------------------------------------------------------------*/
#include <ctype.h>

#include "rtklib.h"

/* constants and macros ------------------------------------------------------*/

#define SQR(x) ((x) < 0.0L ? -(x) * (x) : (x) * (x))
#define SQRTL(x) ((x) < 0.0L || (x) != (x) ? 0.0L : sqrtl(x))

/* https://gpsd.gitlab.io/gpsd/NMEA.html#_talker_ids says ...
      GA    ~    Galileo Positioning System
      GB    ~    BeiDou(China)
      GI    ~    NavIC, IRNSS(India)
      GL    ~    GLONASS, according to IEIC 61162 - 1
      GN    ~    Combination of multiple satellite systems(NMEA 1083)
      GP    ~    Global Positioning System receiver
      GQ    ~    QZSS regional GPS augmentation system(Japan)

 https://talk.newagtalk.com/forums/thread-view.asp?tid=877179&mid=7732209#M7732209
 Says "The industry pretty much standardized on using GPxxx regardless of what
       constellations are being used in the solution. To be completely NMEA compliant
       the second letter should correspond to the constellations being used. Don't
       forget Galileo and Beidou have their own unique letters too."

 And rnx2rtkp GNxxx output is not plotted by RTKPLOT, whereas GPxxx output is

 So, This NMEA Talker ID used to be "GN" but now its "GP" */
#define NMEA_TID "GP" /* NMEA talker ID for RMC and GGA sentences */
#define MAXFIELD 64   /* max number of fields in a record */
#define MAXNMEA 256   /* max length of nmea sentence */

#define KNOT2M 0.51444444444444444444L /* m/sec --> knot */

static const int nmea_sys[] = {/* NMEA systems */
                               SYS_GPS | SYS_SBS, SYS_GLO, SYS_GAL, SYS_CMP, SYS_QZS, SYS_IRN, 0};
static const char *nmea_tid[] = {/* NMEA talker IDs [2] */
                                 "GP", "GL", "GA", "GB", "GQ", "GI", ""};
static const int nmea_sid[] = {/* NMEA system IDs [1] table 21 */
                               1, 2, 3, 4, 5, 6, 0};
static const int nmea_solq[] =
    {/* NMEA GPS quality indicator [1] */
     /* 0=Fix not available or invalid */
     /* 1=GPS SPS Mode, fix valid */
     /* 2=Differential GPS, SPS Mode, fix valid */
     /* 3=GPS PPS Mode, fix valid */
     /* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
     /* 5=Float RTK. Satellite system used in RTK mode, floating integers */
     /* 6=Estimated (dead reckoning) Mode */
     /* 7=Manual Input Mode */
     /* 8=Simulation Mode */
     SOLQ_NONE,  SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP,  SOLQ_FIX,
     SOLQ_FLOAT, SOLQ_DR,     SOLQ_NONE, SOLQ_NONE, SOLQ_NONE};
/* solution option to field separator ----------------------------------------*/
static const char *opt2sep(const solopt_t *opt) {
  if (!*opt->sep)
    return " ";
  else if (!strcmp(opt->sep, "\\t"))
    return "\t";
  return opt->sep;
}
/* separate fields -----------------------------------------------------------*/
static int tonum(char *buff, const char *sep, long double *v) {
  int n = 0, len = (int)strlen(sep);
  for (char *p = buff, *q; n < MAXFIELD; p = q + len) {
    q = strstr(p, sep);
    if (q) *q = '\0';
    if (*p) v[n++] = strtold(p, NULL);
    if (!q) break;
  }
  return n;
}
/* sqrt of covariance --------------------------------------------------------*/
static long double sqvar(long double covar) { return covar < 0.0L ? -sqrtl(-covar) : sqrtl(covar); }
/* convert ddmm.mm in nmea format to deg -------------------------------------*/
static long double dmm2deg(long double dmm) {
  return floorl(dmm / 100.0L) + fmodl(dmm, 100.0L) / 60.0L;
}
/* convert time in nmea format to time ---------------------------------------*/
static void septime(long double t, long double *t1, long double *t2, long double *t3) {
  *t1 = floorl(t / 10000.0L);
  t -= *t1 * 10000.0L;
  *t2 = floorl(t / 100.0L);
  *t3 = t - *t2 * 100.0L;
}
/* solution to covariance ----------------------------------------------------*/
static void soltocov(const sol_t *sol, long double *P) {
  P[0] = sol->qr[0];        /* xx or ee */
  P[4] = sol->qr[1];        /* yy or nn */
  P[8] = sol->qr[2];        /* zz or uu */
  P[1] = P[3] = sol->qr[3]; /* xy or en */
  P[5] = P[7] = sol->qr[4]; /* yz or nu */
  P[2] = P[6] = sol->qr[5]; /* zx or ue */
}
/* covariance to solution ----------------------------------------------------*/
static void covtosol(const long double *P, sol_t *sol) {
  sol->qr[0] = P[0]; /* xx or ee */
  sol->qr[1] = P[4]; /* yy or nn */
  sol->qr[2] = P[8]; /* zz or uu */
  sol->qr[3] = P[1]; /* xy or en */
  sol->qr[4] = P[5]; /* yz or nu */
  sol->qr[5] = P[2]; /* zx or ue */
}
/* solution to velocity covariance -------------------------------------------*/
static void soltocov_vel(const sol_t *sol, long double *P) {
  P[0] = sol->qv[0];        /* xx */
  P[4] = sol->qv[1];        /* yy */
  P[8] = sol->qv[2];        /* zz */
  P[1] = P[3] = sol->qv[3]; /* xy */
  P[5] = P[7] = sol->qv[4]; /* yz */
  P[2] = P[6] = sol->qv[5]; /* zx */
}
/* velocity covariance to solution -------------------------------------------*/
static void covtosol_vel(const long double *P, sol_t *sol) {
  sol->qv[0] = P[0]; /* xx */
  sol->qv[1] = P[4]; /* yy */
  sol->qv[2] = P[8]; /* zz */
  sol->qv[3] = P[1]; /* xy */
  sol->qv[4] = P[5]; /* yz */
  sol->qv[5] = P[2]; /* zx */
}
/* decode NMEA RMC (Recommended Minimum Specific GNSS Data) sentence ---------*/
static int decode_nmearmc(char **val, int n, sol_t *sol) {
  long double tod = 0.0L, lat = 0.0L, lon = 0.0L, vel = 0.0L, dir = 0.0L, date = 0.0L, ang = 0.0L;
  char act = ' ', ns = 'N', ew = 'E', mew = 'E', mode = 'A';

  trace(4, "decode_nmearmc: n=%d\n", n);

  for (int i = 0; i < n; i++) {
    switch (i) {
      case 0:
        tod = strtold(val[i], NULL);
        break; /* time in utc (hhmmss) */
      case 1:
        act = *val[i];
        break; /* A=active,V=void */
      case 2:
        lat = strtold(val[i], NULL);
        break; /* latitude (ddmm.mmm) */
      case 3:
        ns = *val[i];
        break; /* N=north,S=south */
      case 4:
        lon = strtold(val[i], NULL);
        break; /* longitude (dddmm.mmm) */
      case 5:
        ew = *val[i];
        break; /* E=east,W=west */
      case 6:
        vel = strtold(val[i], NULL);
        break; /* speed (knots) */
      case 7:
        dir = strtold(val[i], NULL);
        break; /* track angle (deg) */
      case 8:
        date = strtold(val[i], NULL);
        break; /* date (ddmmyy) */
      case 9:
        ang = strtold(val[i], NULL);
        break; /* magnetic variation */
      case 10:
        mew = *val[i];
        break; /* E=east,W=west */
      case 11:
        mode = *val[i];
        break; /* mode indicator (>nmea 2) */
               /* A=autonomous,D=differential */
               /* E=estimated,N=not valid,S=simulator */
    }
  }
  if ((act != 'A' && act != 'V') || (ns != 'N' && ns != 'S') || (ew != 'E' && ew != 'W')) {
    trace(3, "invalid nmea rmc format\n");
    return 0;
  }
  long double pos[3] = {0};
  pos[0] = (ns == 'S' ? -1.0L : 1.0L) * dmm2deg(lat) * D2R;
  pos[1] = (ew == 'W' ? -1.0L : 1.0L) * dmm2deg(lon) * D2R;
  long double ep[6];
  septime(date, ep + 2, ep + 1, ep);
  septime(tod, ep + 3, ep + 4, ep + 5);
  ep[0] += ep[0] < 80.0L ? 2000.0L : 1900.0L;
  sol->time = utc2gpst(epoch2time(ep));
  pos2ecef(pos, sol->rr);
  sol->stat = mode == 'D' ? SOLQ_DGPS : SOLQ_SINGLE;
  sol->ns = 0;

  sol->type = 0; /* position type = xyz */

  char tstr[40];
  trace(5,
        "decode_nmearmc: %s rr=%.3Lf %.3Lf %.3Lf stat=%d ns=%d vel=%.2Lf dir=%.0Lf ang=%.0Lf "
        "mew=%c mode=%c\n",
        time2str(sol->time, tstr, 0), sol->rr[0], sol->rr[1], sol->rr[2], sol->stat, sol->ns, vel,
        dir, ang, mew, mode);

  return 2; /* update time */
}
/* decode NMEA ZDA (Time and Date) sentence ----------------------------------*/
static int decode_nmeazda(char **val, int n, sol_t *sol) {
  long double tod = 0.0L, ep[6] = {0};

  trace(4, "decode_nmeazda: n=%d\n", n);

  for (int i = 0; i < n; i++) {
    switch (i) {
      case 0:
        tod = strtold(val[i], NULL);
        break; /* time in utc (hhmmss) */
      case 1:
        ep[2] = strtold(val[i], NULL);
        break; /* day (0-31) */
      case 2:
        ep[1] = strtold(val[i], NULL);
        break; /* mon (1-12) */
      case 3:
        ep[0] = strtold(val[i], NULL);
        break; /* year */
    }
  }
  septime(tod, ep + 3, ep + 4, ep + 5);
  sol->time = utc2gpst(epoch2time(ep));
  sol->ns = 0;

  char tstr[40];
  trace(5, "decode_nmeazda: %s\n", time2str(sol->time, tstr, 0));

  return 2; /* update time */
}
/* decode NMEA GGA (Global Positioning System Fix Data) sentence -------------*/
static int decode_nmeagga(char **val, int n, sol_t *sol) {
  long double tod = 0.0L, lat = 0.0L, lon = 0.0L, hdop = 0.0L, alt = 0.0L, msl = 0.0L;
  long double age = 0.0L;
  char ns = 'N', ew = 'E', ua = ' ', um = ' ';
  int solq = 0, nrcv = 0;

  trace(4, "decode_nmeagga: n=%d\n", n);

  for (int i = 0; i < n; i++) {
    switch (i) {
      case 0:
        tod = strtold(val[i], NULL);
        break; /* UTC of position (hhmmss) */
      case 1:
        lat = strtold(val[i], NULL);
        break; /* Latitude (ddmm.mmm) */
      case 2:
        ns = *val[i];
        break; /* N=north,S=south */
      case 3:
        lon = strtold(val[i], NULL);
        break; /* Longitude (dddmm.mmm) */
      case 4:
        ew = *val[i];
        break; /* E=east,W=west */
      case 5:
        solq = atoi(val[i]);
        break; /* GPS quality indicator */
      case 6:
        nrcv = atoi(val[i]);
        break; /* # of satellites in use */
      case 7:
        hdop = strtold(val[i], NULL);
        break; /* HDOP */
      case 8:
        alt = strtold(val[i], NULL);
        break; /* Altitude MSL */
      case 9:
        ua = *val[i];
        break; /* unit (M) */
      case 10:
        msl = strtold(val[i], NULL);
        break; /* Geoid separation */
      case 11:
        um = *val[i];
        break; /* unit (M) */
      case 12:
        age = strtold(val[i], NULL);
        break; /* Age of differential */
    }
  }
  if ((ns != 'N' && ns != 'S') || (ew != 'E' && ew != 'W')) {
    trace(3, "invalid nmea gga format\n");
    return 0;
  }
  if (sol->time.time == 0) {
    trace(3, "no date info for nmea gga\n");
    return 0;
  }
  long double pos[3] = {0};
  pos[0] = (ns == 'N' ? 1.0L : -1.0L) * dmm2deg(lat) * D2R;
  pos[1] = (ew == 'E' ? 1.0L : -1.0L) * dmm2deg(lon) * D2R;
  pos[2] = alt + msl;

  long double ep[6];
  time2epoch(sol->time, ep);
  septime(tod, ep + 3, ep + 4, ep + 5);
  gtime_t time = utc2gpst(epoch2time(ep));
  long double tt = timediff(time, sol->time);
  if (tt < -43200.0L)
    sol->time = timeadd(time, 86400.0L);
  else if (tt > 43200.0L)
    sol->time = timeadd(time, -86400.0L);
  else
    sol->time = time;
  pos2ecef(pos, sol->rr);
  sol->stat = 0 <= solq && solq <= 8 ? nmea_solq[solq] : SOLQ_NONE;
  sol->ns = nrcv;
  sol->age = age;

  sol->type = 0; /* position type = xyz */

  char tstr[40];
  trace(5, "decode_nmeagga: %s rr=%.3Lf %.3Lf %.3Lf stat=%d ns=%d hdop=%.1Lf ua=%c um=%c\n",
        time2str(sol->time, tstr, 0), sol->rr[0], sol->rr[1], sol->rr[2], sol->stat, sol->ns, hdop,
        ua, um);

  return 1;
}
/* test NMEA sentence header -------------------------------------------------*/
static bool test_nmea(const char *buff) {
  if (strlen(buff) < 6 || buff[0] != '$') return false;
  return !strncmp(buff + 1, "GP", 2) || !strncmp(buff + 1, "GA", 2) || /* NMEA 4.10 [1] */
         !strncmp(buff + 1, "GL", 2) || !strncmp(buff + 1, "GN", 2) ||
         !strncmp(buff + 1, "GB", 2) || !strncmp(buff + 1, "GQ", 2) || /* NMEA 4.11 [2] */
         !strncmp(buff + 1, "GI", 2) || !strncmp(buff + 1, "BD", 2) ||
         !strncmp(buff + 1, "QZ", 2); /* extension */
}
/* test solution status message header ---------------------------------------*/
static bool test_solstat(const char *buff) {
  if (strlen(buff) < 7 || buff[0] != '$') return false;
  return !strncmp(buff + 1, "POS", 3) || !strncmp(buff + 1, "VELACC", 6) ||
         !strncmp(buff + 1, "CLK", 3) || !strncmp(buff + 1, "ION", 3) ||
         !strncmp(buff + 1, "TROP", 4) || !strncmp(buff + 1, "HWBIAS", 6) ||
         !strncmp(buff + 1, "TRPG", 4) || !strncmp(buff + 1, "AMB", 3) ||
         !strncmp(buff + 1, "SAT", 3);
}
/* decode NMEA sentence ------------------------------------------------------*/
static int decode_nmea(char *buff, sol_t *sol) {
  trace(4, "decode_nmea: buff=%s\n", buff);

  /* parse fields */
  char *val[MAXFIELD] = {0};
  int n = 0;
  for (char *p = buff, *q; *p && n < MAXFIELD; p = q + 1) {
    if ((q = strchr(p, ',')) || (q = strchr(p, '*'))) {
      val[n++] = p;
      *q = '\0';
    } else
      break;
  }
  if (n < 1) {
    return 0;
  }
  if (!strcmp(val[0] + 3, "RMC")) { /* $xxRMC */
    return decode_nmearmc(val + 1, n - 1, sol);
  } else if (!strcmp(val[0] + 3, "ZDA")) { /* $xxZDA */
    return decode_nmeazda(val + 1, n - 1, sol);
  } else if (!strcmp(val[0] + 3, "GGA")) { /* $xxGGA */
    return decode_nmeagga(val + 1, n - 1, sol);
  }
  return 0;
}
/* decode solution time ------------------------------------------------------*/
static char *decode_soltime(char *buff, const solopt_t *opt, gtime_t *time) {
  trace(4, "decode_soltime:\n");

  char s[64] = " ";
  if (!strcmp(opt->sep, "\\t"))
    rtkstrcpy(s, sizeof(s), "\t");
  else if (*opt->sep)
    rtkstrcpy(s, sizeof(s), opt->sep);
  size_t len = strlen(s);

  if (opt->posf == SOLF_STAT) {
    return buff;
  }
  long double v[MAXFIELD];
  if (opt->posf == SOLF_GSIF) {
    if (sscanf(buff, "%Lf %Lf %Lf %Lf:%Lf:%Lf", v, v + 1, v + 2, v + 3, v + 4, v + 5) < 6) {
      return NULL;
    }
    *time = timeadd(epoch2time(v), -12.0L * 3600.0L);
    char *p = strchr(buff, ':');
    if (!p || !(p = strchr(p + 1, ':'))) return NULL;
    for (p++; isdigit((int)*p) || *p == '.';) p++;
    return p + len;
  }
  /* yyyy/mm/dd hh:mm:ss or yyyy mm dd hh:mm:ss */
  if (sscanf(buff, "%Lf/%Lf/%Lf %Lf:%Lf:%Lf", v, v + 1, v + 2, v + 3, v + 4, v + 5) >= 6) {
    if (v[0] < 100.0L) {
      v[0] += v[0] < 80.0L ? 2000.0L : 1900.0L;
    }
    *time = epoch2time(v);
    if (opt->times == TIMES_UTC) {
      *time = utc2gpst(*time);
    } else if (opt->times == TIMES_JST) {
      *time = utc2gpst(timeadd(*time, -9 * 3600.0L));
    }
    char *p = strchr(buff, ':');
    if (!p || !(p = strchr(p + 1, ':'))) return NULL;
    for (p++; isdigit((int)*p) || *p == '.';) p++;
    return p + len;
  } else { /* wwww ssss */
    char *p, *q;
    int n = 0;
    for (p = buff; n < 2; p = q + len) {
      q = strstr(p, s);
      if (q) *q = '\0';
      if (sscanf(p, "%Lf", v + n) == 1) n++;
      if (!q) break;
    }
    if (n >= 2 && 0.0L <= v[0] && v[0] <= 3000.0L && 0.0L <= v[1] && v[1] < 604800.0L) {
      *time = gpst2time((int)v[0], v[1]);
      return p;
    }
  }
  return NULL;
}
/* decode x/y/z-ecef ---------------------------------------------------------*/
static int decode_solxyz(char *buff, const solopt_t *opt, sol_t *sol) {
  trace(4, "decode_solxyz:\n");

  const char *sep = opt2sep(opt);
  long double val[MAXFIELD];
  int n = tonum(buff, sep, val);
  if (n < 3) return 0;

  int i = 0;
  for (int j = 0; j < 3; j++) {
    sol->rr[j] = val[i++]; /* xyz */
  }
  if (i < n) sol->stat = (uint8_t)val[i++];
  if (i < n) sol->ns = (uint8_t)val[i++];
  if (i + 3 <= n) {
    long double P[9] = {0};
    P[0] = SQR(val[i]);
    i++; /* sdx */
    P[4] = SQR(val[i]);
    i++; /* sdy */
    P[8] = SQR(val[i]);
    i++; /* sdz */
    if (i + 3 <= n) {
      P[1] = P[3] = SQR(val[i]);
      i++; /* sdxy */
      P[5] = P[7] = SQR(val[i]);
      i++; /* sdyz */
      P[2] = P[6] = SQR(val[i]);
      i++; /* sdzx */
    }
    covtosol(P, sol);
  }
  if (i < n) sol->age = val[i++];
  if (i < n) sol->ratio = val[i++];

  if (i + 3 <= n) { /* velocity */
    for (int j = 0; j < 3; j++) {
      sol->rr[j + 3] = val[i++]; /* xyz */
    }
  }
  if (i + 3 <= n) {
    long double P[9] = {0};
    P[0] = SQR(val[i]);
    i++; /* sdx */
    P[4] = SQR(val[i]);
    i++; /* sdy */
    P[8] = SQR(val[i]);
    i++; /* sdz */
    if (i + 3 < n) {
      P[1] = P[3] = SQR(val[i]);
      i++; /* sdxy */
      P[5] = P[7] = SQR(val[i]);
      i++; /* sdyz */
      P[2] = P[6] = SQR(val[i]);
      i++; /* sdzx */
    }
    covtosol_vel(P, sol);
  }
  sol->type = 0; /* position type = xyz */

  if (MAXSOLQ < sol->stat) sol->stat = SOLQ_NONE;
  return 1;
}
/* decode lat/lon/height -----------------------------------------------------*/
static int decode_solllh(char *buff, const solopt_t *opt, sol_t *sol) {
  trace(4, "decode_solllh:\n");

  const char *sep = opt2sep(opt);
  long double val[MAXFIELD];
  int n = tonum(buff, sep, val);

  int i = 0;
  long double pos[3];
  if (!opt->degf) {
    if (n < 3) return 0;
    pos[0] = val[i++] * D2R; /* lat/lon/hgt (ddd.ddd) */
    pos[1] = val[i++] * D2R;
    pos[2] = val[i++];
  } else {
    if (n < 7) return 0;
    pos[0] = dms2deg(val) * D2R; /* lat/lon/hgt (ddd mm ss) */
    pos[1] = dms2deg(val + 3) * D2R;
    pos[2] = val[6];
    i += 7;
  }
  pos2ecef(pos, sol->rr);
  if (i < n) sol->stat = (uint8_t)val[i++];
  if (i < n) sol->ns = (uint8_t)val[i++];
  if (i + 3 <= n) {
    long double Q[9] = {0};
    Q[4] = SQR(val[i]);
    i++; /* sdn */
    Q[0] = SQR(val[i]);
    i++; /* sde */
    Q[8] = SQR(val[i]);
    i++; /* sdu */
    if (i + 3 < n) {
      Q[1] = Q[3] = SQR(val[i]);
      i++; /* sdne */
      Q[2] = Q[6] = SQR(val[i]);
      i++; /* sdeu */
      Q[5] = Q[7] = SQR(val[i]);
      i++; /* sdun */
    }
    long double P[9];
    covecef(pos, Q, P);
    covtosol(P, sol);
  }
  if (i < n) sol->age = val[i++];
  if (i < n) sol->ratio = val[i++];

  if (i + 3 <= n) { /* velocity */
    long double vel[3];
    vel[1] = val[i++]; /* vel-n */
    vel[0] = val[i++]; /* vel-e */
    vel[2] = val[i++]; /* vel-u */
    enu2ecef(pos, vel, sol->rr + 3);
  }
  if (i + 3 <= n) {
    long double Q[9] = {0};
    Q[4] = SQR(val[i]);
    i++; /* sdn */
    Q[0] = SQR(val[i]);
    i++; /* sde */
    Q[8] = SQR(val[i]);
    i++; /* sdu */
    if (i + 3 <= n) {
      Q[1] = Q[3] = SQR(val[i]);
      i++; /* sdne */
      Q[2] = Q[6] = SQR(val[i]);
      i++; /* sdeu */
      Q[5] = Q[7] = SQR(val[i]);
      i++; /* sdun */
    }
    long double P[9];
    covecef(pos, Q, P);
    covtosol_vel(P, sol);
  }
  sol->type = 0; /* position type = xyz */

  if (MAXSOLQ < sol->stat) sol->stat = SOLQ_NONE;
  return 1;
}
/* decode e/n/u-baseline -----------------------------------------------------*/
static int decode_solenu(char *buff, const solopt_t *opt, sol_t *sol) {
  trace(4, "decode_solenu:\n");

  const char *sep = opt2sep(opt);
  long double val[MAXFIELD];
  int n = tonum(buff, sep, val);
  if (n < 3) return 0;

  int i = 0;
  for (int j = 0; j < 3; j++) {
    sol->rr[j] = val[i++]; /* enu */
  }
  if (i < n) sol->stat = (uint8_t)val[i++];
  if (i < n) sol->ns = (uint8_t)val[i++];
  if (i + 3 <= n) {
    long double Q[9] = {0};
    Q[0] = SQR(val[i]);
    i++; /* sde */
    Q[4] = SQR(val[i]);
    i++; /* sdn */
    Q[8] = SQR(val[i]);
    i++; /* sdu */
    if (i + 3 <= n) {
      Q[1] = Q[3] = SQR(val[i]);
      i++; /* sden */
      Q[5] = Q[7] = SQR(val[i]);
      i++; /* sdnu */
      Q[2] = Q[6] = SQR(val[i]);
      i++; /* sdue */
    }
    covtosol(Q, sol);
  }
  if (i < n) sol->age = val[i++];
  if (i < n) sol->ratio = val[i++];

  if (i + 3 <= n) { /* velocity */
    for (int j = 0; j < 3; j++) {
      sol->rr[j + 3] = val[i++]; /* vel-enu */
    }
  }
  if (i + 3 <= n) {
    long double Q[9] = {0};
    Q[0] = val[i] * val[i];
    i++; /* sde */
    Q[4] = val[i] * val[i];
    i++; /* sdn */
    Q[8] = val[i] * val[i];
    i++; /* sdu */
    if (i + 3 <= n) {
      Q[1] = Q[3] = SQR(val[i]);
      i++; /* sden */
      Q[5] = Q[7] = SQR(val[i]);
      i++; /* sdnu */
      Q[2] = Q[6] = SQR(val[i]);
      i++; /* sdue */
    }
    covtosol_vel(Q, sol);
  }
  sol->type = 1; /* position type = enu */

  if (MAXSOLQ < sol->stat) sol->stat = SOLQ_NONE;
  return 1;
}
/* decode solution status ----------------------------------------------------*/
static int decode_solsss(const char *buff, sol_t *sol) {
  trace(4, "decode_solsss:\n");

  long double tow, pos[3], std[3] = {0};
  int week, solq;
  if (sscanf(buff, "$POS,%d,%Lf,%d,%Lf,%Lf,%Lf,%Lf,%Lf,%Lf", &week, &tow, &solq, pos, pos + 1,
             pos + 2, std, std + 1, std + 2) < 6) {
    return 0;
  }
  if (week <= 0 || norm(pos, 3) <= 0.0L || solq == SOLQ_NONE) {
    return 0;
  }
  sol->time = gpst2time(week, tow);
  for (int i = 0; i < 6; i++) {
    sol->rr[i] = i < 3 ? pos[i] : 0.0L;
    sol->qr[i] = i < 3 ? SQR(std[i]) : 0.0L;
    sol->dtr[i] = 0.0L;
  }
  sol->ns = 0;
  sol->age = sol->ratio = sol->thres = 0.0L;
  sol->type = 0; /* position type = xyz */
  sol->stat = solq;
  return 1;
}
/* decode GSI F solution -----------------------------------------------------*/
static int decode_solgsi(char *buff, const solopt_t *opt, sol_t *sol) {
  trace(4, "decode_solgsi:\n");

  long double val[MAXFIELD];
  if (tonum(buff, " ", val) < 3) return 0;

  int i = 0;
  for (int j = 0; j < 3; j++) {
    sol->rr[j] = val[i++]; /* xyz */
  }
  sol->stat = SOLQ_FIX;
  return 1;
}
/* decode solution position --------------------------------------------------*/
static int decode_solpos(char *buff, const solopt_t *opt, sol_t *sol) {
  trace(4, "decode_solpos: buff=%s\n", buff);

  sol_t sol0 = {{0}};
  *sol = sol0;

  /* decode solution time */
  char *p = decode_soltime(buff, opt, &sol->time);
  if (!p) return 0;

  /* decode solution position */
  switch (opt->posf) {
    case SOLF_XYZ:
      return decode_solxyz(p, opt, sol);
    case SOLF_LLH:
      return decode_solllh(p, opt, sol);
    case SOLF_ENU:
      return decode_solenu(p, opt, sol);
    case SOLF_GSIF:
      return decode_solgsi(p, opt, sol);
  }
  return 0;
}
/* decode reference position -------------------------------------------------*/
static void decode_refpos(char *buff, const solopt_t *opt, long double *rb) {
  trace(3, "decode_refpos: buff=%s\n", buff);

  const char *sep = opt2sep(opt);
  long double val[MAXFIELD];
  int n = tonum(buff, sep, val);
  if (n < 3) return;

  if (opt->posf == SOLF_XYZ) { /* xyz */
    for (int i = 0; i < 3; i++) rb[i] = val[i];
  } else if (opt->degf == 0) { /* lat/lon/hgt (ddd.ddd) */
    long double pos[3];
    pos[0] = val[0] * D2R;
    pos[1] = val[1] * D2R;
    pos[2] = val[2];
    pos2ecef(pos, rb);
  } else if (opt->degf == 1 && n >= 7) { /* lat/lon/hgt (ddd mm ss) */
    long double pos[3];
    pos[0] = dms2deg(val) * D2R;
    pos[1] = dms2deg(val + 3) * D2R;
    pos[2] = val[6];
    pos2ecef(pos, rb);
  }
}
/* decode solution -----------------------------------------------------------*/
static int decode_sol(char *buff, const solopt_t *opt, sol_t *sol, long double *rb) {
  trace(4, "decode_sol: buff=%s\n", buff);

  if (test_nmea(buff)) { /* decode nmea */
    return decode_nmea(buff, sol);
  } else if (test_solstat(buff)) { /* decode solution status */
    return decode_solsss(buff, sol);
  }
  if (!strncmp(buff, COMMENTH, 1)) { /* reference position */
    if (!strstr(buff, "ref pos") && !strstr(buff, "slave pos")) return 0;
    char *p = strchr(buff, ':');
    if (!p) return 0;
    decode_refpos(p + 1, opt, rb);
    return 0;
  }
  /* decode position record */
  return decode_solpos(buff, opt, sol);
}
/* decode solution options ---------------------------------------------------*/
static void decode_solopt(char *buff, solopt_t *opt) {
  trace(4, "decode_solhead: buff=%s\n", buff);

  if (strncmp(buff, COMMENTH, 1) && strncmp(buff, "+", 1)) return;

  if (strstr(buff, "GPST"))
    opt->times = TIMES_GPST;
  else if (strstr(buff, "UTC"))
    opt->times = TIMES_UTC;
  else if (strstr(buff, "JST"))
    opt->times = TIMES_JST;

  char *p = strstr(buff, "x-ecef(m)");
  if (p) {
    opt->posf = SOLF_XYZ;
    opt->degf = 0;
    rtkesubstrcpy(opt->sep, sizeof(opt->sep), p, 9, 10);
    opt->sep[1] = '\0';
  } else if ((p = strstr(buff, "latitude(d'\")"))) {
    opt->posf = SOLF_LLH;
    opt->degf = 1;
    rtkesubstrcpy(opt->sep, sizeof(opt->sep), p, 14, 15);
    opt->sep[1] = '\0';
  } else if ((p = strstr(buff, "latitude(deg)"))) {
    opt->posf = SOLF_LLH;
    opt->degf = 0;
    rtkesubstrcpy(opt->sep, sizeof(opt->sep), p, 13, 14);
    opt->sep[1] = '\0';
  } else if ((p = strstr(buff, "e-baseline(m)"))) {
    opt->posf = SOLF_ENU;
    opt->degf = 0;
    rtkesubstrcpy(opt->sep, sizeof(opt->sep), p, 13, 14);
    opt->sep[1] = '\0';
  } else if ((p = strstr(buff, "+SITE/INF"))) { /* gsi f2/f3 solution */
    opt->times = TIMES_GPST;
    opt->posf = SOLF_GSIF;
    opt->degf = 0;
    rtkstrcpy(opt->sep, sizeof(opt->sep), " ");
  }
}
/* read solution option ------------------------------------------------------*/
static void readsolopt(FILE *fp, solopt_t *opt) {
  trace(3, "readsolopt:\n");

  char buff[MAXSOLMSG + 1];
  for (int i = 0; fgets(buff, sizeof(buff), fp) && i < 100; i++) { /* only 100 lines */

    /* decode solution options */
    decode_solopt(buff, opt);
  }
}
/* input solution data from stream ---------------------------------------------
 * input solution data from stream
 * args   : uint8_t data     I stream data
 *          gtime_t ts       I  start time (ts.time==0: from start)
 *          gtime_t te       I  end time   (te.time==0: to end)
 *          long double tint      I  time interval (0: all)
 *          int    qflag     I  quality flag  (0: all)
 *          solbuf_t *solbuf IO solution buffer
 * return : status (1:solution received,0:no solution,-1:disconnect received)
 *-----------------------------------------------------------------------------*/
extern int inputsol(uint8_t data, gtime_t ts, gtime_t te, long double tint, int qflag,
                    const solopt_t *opt, solbuf_t *solbuf) {
  trace(4, "inputsol: data=0x%02x\n", data);

  if (data == '$' || (!isprint(data) && data != '\r' && data != '\n')) { /* sync header */
    solbuf->nb = 0;
  }
  if (data != '\r' && data != '\n') {
    solbuf->buff[solbuf->nb++] = data;
  }
  if (data != '\n' && solbuf->nb < MAXSOLMSG) return 0; /* sync trailer */

  solbuf->buff[solbuf->nb] = '\0';
  solbuf->nb = 0;

  /* check disconnect message */
  if (!strncmp((char *)solbuf->buff, MSG_DISCONN, strlen(MSG_DISCONN) - 2)) {
    trace(3, "disconnect received\n");
    return -1;
  }
  /* decode solution */
  sol_t sol = {{0}};
  sol.time = solbuf->time;
  int stat = decode_sol((char *)solbuf->buff, opt, &sol, solbuf->rb);
  if (stat > 0) {
    solbuf->time = sol.time; /* update current time */
    if (stat != 1) return 0;
  }
  if (stat != 1 || !screent(sol.time, ts, te, tint) || (qflag && sol.stat != qflag)) {
    return 0;
  }
  /* add solution to solution buffer */
  return addsol(solbuf, &sol);
}
/* read solution data --------------------------------------------------------*/
static bool readsoldata(FILE *fp, gtime_t ts, gtime_t te, long double tint, int qflag,
                        const solopt_t *opt, solbuf_t *solbuf) {
  trace(3, "readsoldata:\n");

  int c;
  while ((c = fgetc(fp)) != EOF) {
    /* input solution */
    inputsol((uint8_t)c, ts, te, tint, qflag, opt, solbuf);
  }
  return solbuf->n > 0;
}
/* compare solution data -----------------------------------------------------*/
static int cmpsol(const void *p1, const void *p2) {
  sol_t *q1 = (sol_t *)p1, *q2 = (sol_t *)p2;
  long double tt = timediff(q1->time, q2->time);
  return tt < -0.0L ? -1 : (tt > 0.0L ? 1 : 0);
}
/* sort solution data --------------------------------------------------------*/
static bool sort_solbuf(solbuf_t *solbuf) {
  trace(4, "sort_solbuf: n=%d\n", solbuf->n);

  if (solbuf->n <= 0) return false;

  sol_t *solbuf_data = (sol_t *)realloc(solbuf->data, sizeof(sol_t) * solbuf->n);
  if (!solbuf_data) {
    trace(1, "sort_solbuf: memory allocation error\n");
    free(solbuf->data);
    solbuf->data = NULL;
    solbuf->n = solbuf->nmax = 0;
    return false;
  }
  solbuf->data = solbuf_data;
  qsort(solbuf->data, solbuf->n, sizeof(sol_t), cmpsol);
  solbuf->nmax = solbuf->n;
  solbuf->start = 0;
  solbuf->end = solbuf->n - 1;
  return true;
}
/* read solutions data from solution files -------------------------------------
 * read solution data from solution files
 * args   : char   *files[]  I  solution files
 *          int    nfile     I  number of files
 *         (gtime_t ts)      I  start time (ts.time==0: from start)
 *         (gtime_t te)      I  end time   (te.time==0: to end)
 *         (long double tint)     I  time interval (0: all)
 *         (int    qflag)    I  quality flag  (0: all)
 *          solbuf_t *solbuf O  solution buffer
 * return : status (true:ok,false:no data or error)
 *-----------------------------------------------------------------------------*/
extern bool readsolt(const char *files[], int nfile, gtime_t ts, gtime_t te, long double tint,
                     int qflag, solbuf_t *solbuf) {
  trace(3, "readsolt: nfile=%d\n", nfile);

  initsolbuf(solbuf, 0, 0);

  solopt_t opt = solopt_default;

  for (int i = 0; i < nfile; i++) {
    FILE *fp = fopen(files[i], "rb");
    if (!fp) {
      trace(2, "readsolt: file open error %s\n", files[i]);
      continue;
    }
    /* read solution options in header */
    readsolopt(fp, &opt);
    rewind(fp);

    /* read solution data */
    if (!readsoldata(fp, ts, te, tint, qflag, &opt, solbuf)) {
      trace(2, "readsolt: no solution in %s\n", files[i]);
    }
    fclose(fp);
  }
  return sort_solbuf(solbuf);
}
extern bool readsol(const char *files[], int nfile, solbuf_t *sol) {
  trace(3, "readsol: nfile=%d\n", nfile);

  gtime_t time = {0};
  return readsolt(files, nfile, time, time, 0.0L, 0, sol);
}
/* add solution data to solution buffer ----------------------------------------
 * add solution data to solution buffer
 * args   : solbuf_t *solbuf IO solution buffer
 *          sol_t  *sol      I  solution data
 * return : status (true:ok,false:error)
 *-----------------------------------------------------------------------------*/
extern bool addsol(solbuf_t *solbuf, const sol_t *sol) {
  trace(4, "addsol:\n");

  if (solbuf->cyclic) { /* ring buffer */
    if (solbuf->nmax <= 1) return false;
    solbuf->data[solbuf->end] = *sol;
    if (++solbuf->end >= solbuf->nmax) solbuf->end = 0;
    if (solbuf->start == solbuf->end) {
      if (++solbuf->start >= solbuf->nmax) solbuf->start = 0;
    } else
      solbuf->n++;

    return true;
  }
  if (solbuf->n >= solbuf->nmax) {
    solbuf->nmax = solbuf->nmax == 0 ? 8192 : solbuf->nmax * 2;
    sol_t *solbuf_data = (sol_t *)realloc(solbuf->data, sizeof(sol_t) * solbuf->nmax);
    if (!solbuf_data) {
      trace(1, "addsol: memory allocation error\n");
      free(solbuf->data);
      solbuf->data = NULL;
      solbuf->n = solbuf->nmax = 0;
      return false;
    }
    solbuf->data = solbuf_data;
  }
  solbuf->data[solbuf->n++] = *sol;
  return true;
}
/* get solution data from solution buffer --------------------------------------
 * get solution data by index from solution buffer
 * args   : solbuf_t *solbuf I  solution buffer
 *          int    index     I  index of solution (0...)
 * return : solution data pointer (NULL: no solution, out of range)
 *-----------------------------------------------------------------------------*/
extern sol_t *getsol(solbuf_t *solbuf, int index) {
  trace(4, "getsol: index=%d\n", index);

  if (index < 0 || solbuf->n <= index) return NULL;
  index = solbuf->start + index;
  if (index >= solbuf->nmax) index -= solbuf->nmax;
  return solbuf->data + index;
}
/* initialize solution buffer --------------------------------------------------
 * initialize position solutions
 * args   : solbuf_t *solbuf I  solution buffer
 *          int    cyclic    I  solution data buffer type (0:linear,1:cyclic)
 *          int    nmax      I  initial number of solution data
 * return : status (true:ok,false:error)
 *-----------------------------------------------------------------------------*/
extern bool initsolbuf(solbuf_t *solbuf, int cyclic, int nmax) {
  trace(3, "initsolbuf: cyclic=%d nmax=%d\n", cyclic, nmax);

  solbuf->n = solbuf->nmax = solbuf->start = solbuf->end = solbuf->nb = 0;
  solbuf->cyclic = cyclic;
#if 0
  gtime_t time0 = {0};
  solbuf->time = time0;
#endif
  solbuf->data = NULL;
  for (int i = 0; i < 3; i++) {
    solbuf->rb[i] = 0.0L;
  }
  if (cyclic) {
    if (nmax <= 2) nmax = 2;
    if (!(solbuf->data = malloc(sizeof(sol_t) * nmax))) {
      trace(1, "initsolbuf: memory allocation error\n");
      return false;
    }
    solbuf->nmax = nmax;
  }
  return true;
}
/* free solution ---------------------------------------------------------------
 * free memory for solution buffer
 * args   : solbuf_t *solbuf I  solution buffer
 * return : none
 *-----------------------------------------------------------------------------*/
extern void freesolbuf(solbuf_t *solbuf) {
  trace(3, "freesolbuf: n=%d\n", solbuf->n);

  free(solbuf->data);
  solbuf->n = solbuf->nmax = solbuf->start = solbuf->end = solbuf->nb = 0;
  solbuf->data = NULL;
  for (int i = 0; i < 3; i++) {
    solbuf->rb[i] = 0.0L;
  }
}
extern void freesolstatbuf(solstatbuf_t *solstatbuf) {
  trace(3, "freesolstatbuf: n=%d\n", solstatbuf->n);

  solstatbuf->n = solstatbuf->nmax = 0;
  free(solstatbuf->data);
  solstatbuf->data = NULL;
}
/* compare solution status ---------------------------------------------------*/
static int cmpsolstat(const void *p1, const void *p2) {
  solstat_t *q1 = (solstat_t *)p1, *q2 = (solstat_t *)p2;
  long double tt = timediff(q1->time, q2->time);
  return tt < -0.0L ? -1 : (tt > 0.0L ? 1 : 0);
}
/* sort solution data --------------------------------------------------------*/
static bool sort_solstat(solstatbuf_t *statbuf) {
  trace(4, "sort_solstat: n=%d\n", statbuf->n);

  if (statbuf->n <= 0) return false;

  solstat_t *statbuf_data = realloc(statbuf->data, sizeof(solstat_t) * statbuf->n);
  if (!statbuf_data) {
    trace(1, "sort_solstat: memory allocation error\n");
    free(statbuf->data);
    statbuf->data = NULL;
    statbuf->n = statbuf->nmax = 0;
    return false;
  }
  statbuf->data = statbuf_data;
  qsort(statbuf->data, statbuf->n, sizeof(solstat_t), cmpsolstat);
  statbuf->nmax = statbuf->n;
  return true;
}
/* decode solution status ----------------------------------------------------*/
static bool decode_solstat(char *buff, solstat_t *stat) {
  trace(4, "decode_solstat: buff=%s\n", buff);

  if (strstr(buff, "$SAT") != buff) return false;

  for (char *p = buff; *p; p++)
    if (*p == ',') *p = ' ';

  char id[8] = "";
  long double tow, az, el, resp, resc, snr;
  int week, frq, vsat, fix, slip, lock, outc, slipc, rejc;
  int n = sscanf(buff, "$SAT%d%Lf%7s%d%Lf%Lf%Lf%Lf%d%Lf%d%d%d%d%d%d", &week, &tow, id, &frq, &az,
                 &el, &resp, &resc, &vsat, &snr, &fix, &slip, &lock, &outc, &slipc, &rejc);

  if (n < 15) {
    trace(2, "invalid format of solution status: %s\n", buff);
    return false;
  }
  int sat = satid2no(id);
  if (sat <= 0) {
    trace(2, "invalid satellite in solution status: %s\n", id);
    return false;
  }
  static const solstat_t stat0 = {{0}};
  *stat = stat0;
  stat->time = gpst2time(week, tow);
  stat->sat = (uint8_t)sat;
  stat->frq = (uint8_t)frq;
  stat->az = (az * D2R);
  stat->el = (el * D2R);
  stat->resp = resp;
  stat->resc = resc;
  stat->flag = (uint8_t)((vsat << 5) + (slip << 3) + fix);
  stat->snr = (uint16_t)(snr / SNR_UNIT + 0.5L);
  stat->lock = (uint16_t)lock;
  stat->outc = (uint16_t)outc;
  stat->slipc = (uint16_t)slipc;
  stat->rejc = (uint16_t)rejc;
  return true;
}
/* add solution status data --------------------------------------------------*/
static void addsolstat(solstatbuf_t *statbuf, const solstat_t *stat) {
  trace(4, "addsolstat:\n");

  if (statbuf->n >= statbuf->nmax) {
    statbuf->nmax = statbuf->nmax == 0 ? 8192 : statbuf->nmax * 2;
    solstat_t *statbuf_data =
        (solstat_t *)realloc(statbuf->data, sizeof(solstat_t) * statbuf->nmax);
    if (!statbuf_data) {
      trace(1, "addsolstat: memory allocation error\n");
      free(statbuf->data);
      statbuf->data = NULL;
      statbuf->n = statbuf->nmax = 0;
      return;
    }
    statbuf->data = statbuf_data;
  }
  statbuf->data[statbuf->n++] = *stat;
}
/* read solution status data -------------------------------------------------*/
static bool readsolstatdata(FILE *fp, gtime_t ts, gtime_t te, long double tint,
                            solstatbuf_t *statbuf) {
  trace(3, "readsolstatdata:\n");

  char buff[MAXSOLMSG + 1];
  while (fgets(buff, sizeof(buff), fp)) {
    /* decode solution status */
    solstat_t stat;
    if (!decode_solstat(buff, &stat)) continue;

    /* add solution to solution buffer */
    if (screent(stat.time, ts, te, tint)) {
      addsolstat(statbuf, &stat);
    }
  }
  return statbuf->n > 0;
}
/* read solution status --------------------------------------------------------
 * read solution status from solution status files
 * args   : char   *files[]  I  solution status files
 *          int    nfile     I  number of files
 *         (gtime_t ts)      I  start time (ts.time==0: from start)
 *         (gtime_t te)      I  end time   (te.time==0: to end)
 *         (long double tint)     I  time interval (0: all)
 *          solstatbuf_t *statbuf O  solution status buffer
 * return : status (true:ok,false:no data or error)
 *-----------------------------------------------------------------------------*/
extern bool readsolstatt(const char *files[], int nfile, gtime_t ts, gtime_t te, long double tint,
                         solstatbuf_t *statbuf) {
  trace(3, "readsolstatt: nfile=%d\n", nfile);

  statbuf->n = statbuf->nmax = 0;
  statbuf->data = NULL;

  for (int i = 0; i < nfile; i++) {
    char path[FNSIZE], *p = strrchr(files[i], '.');
    if (p && !strcmp(p, ".stat")) {
      rtksnprintf(path, sizeof(path), "%s", files[i]);
    } else {
      rtksnprintf(path, sizeof(path), "%s.stat", files[i]);
    }
    FILE *fp = fopen(path, "r");
    if (!fp) {
      trace(2, "readsolstatt: file open error %s\n", path);
      continue;
    }
    /* read solution status data */
    if (!readsolstatdata(fp, ts, te, tint, statbuf)) {
      trace(2, "readsolstatt: no solution in %s\n", path);
    }
    fclose(fp);
  }
  return sort_solstat(statbuf);
}
extern bool readsolstat(const char *files[], int nfile, solstatbuf_t *statbuf) {
  trace(3, "readsolstat: nfile=%d\n", nfile);

  gtime_t time = {0};
  return readsolstatt(files, nfile, time, time, 0.0L, statbuf);
}
/* output solution as the form of x/y/z-ecef ---------------------------------*/
static void outecef(char *buff, size_t size, const char *s, const sol_t *sol, const solopt_t *opt) {
  trace(4, "outecef:\n");

  const char *sep = opt2sep(opt);
  rtkcatprintf(buff, size,
               "%s%s%14.4Lf%s%14.4Lf%s%14.4Lf%s%3d%s%3d%s%8.4Lf%s%8.4Lf%s%8.4Lf%s"
               "%8.4Lf%s%8.4Lf%s%8.4Lf%s%6.3Lf%s%6.1Lf",
               s, sep, sol->rr[0], sep, sol->rr[1], sep, sol->rr[2], sep, sol->stat, sep, sol->ns,
               sep, SQRTL(sol->qr[0]), sep, SQRTL(sol->qr[1]), sep, SQRTL(sol->qr[2]), sep,
               sqvar(sol->qr[3]), sep, sqvar(sol->qr[4]), sep, sqvar(sol->qr[5]), sep, sol->age,
               sep, sol->ratio);

  if (opt->outvel) { /* output velocity */
    rtkcatprintf(buff, size,
                 "%s%10.5Lf%s%10.5Lf%s%10.5Lf%s%9.5Lf%s%8.5Lf%s%8.5Lf%s%8.5Lf%s"
                 "%8.5Lf%s%8.5Lf",
                 sep, sol->rr[3], sep, sol->rr[4], sep, sol->rr[5], sep, SQRTL(sol->qv[0]), sep,
                 SQRTL(sol->qv[1]), sep, SQRTL(sol->qv[2]), sep, sqvar(sol->qv[3]), sep,
                 sqvar(sol->qv[4]), sep, sqvar(sol->qv[5]));
  }
  rtkcatprintf(buff, size, "\r\n");
}
/* output solution as the form of lat/lon/height -----------------------------*/
static void outpos(char *buff, size_t size, const char *s, const sol_t *sol, const solopt_t *opt) {
  trace(4, "outpos  :\n");

  long double pos[3];
  ecef2pos(sol->rr, pos);
  long double P[9];
  soltocov(sol, P);
  long double Q[9];
  covenu(pos, P, Q);
  if (opt->height == 1) { /* geodetic height */
    pos[2] -= geoidh(pos);
  }
  const char *sep = opt2sep(opt);
  if (opt->degf) {
    long double dms1[3];
    deg2dms(pos[0] * R2D, dms1, 5);
    long double dms2[3];
    deg2dms(pos[1] * R2D, dms2, 5);
    rtkcatprintf(buff, size, "%s%s%4.0Lf%s%02.0Lf%s%08.5Lf%s%4.0Lf%s%02.0Lf%s%08.5Lf", s, sep,
                 dms1[0], sep, dms1[1], sep, dms1[2], sep, dms2[0], sep, dms2[1], sep, dms2[2]);
  } else {
    rtkcatprintf(buff, size, "%s%s%14.9Lf%s%14.9Lf", s, sep, pos[0] * R2D, sep, pos[1] * R2D);
  }
  rtkcatprintf(buff, size,
               "%s%10.4Lf%s%3d%s%3d%s%8.4Lf%s%8.4Lf%s%8.4Lf%s%8.4Lf%s%8.4Lf%s%8.4Lf"
               "%s%6.3Lf%s%6.1Lf",
               sep, pos[2], sep, sol->stat, sep, sol->ns, sep, SQRTL(Q[4]), sep, SQRTL(Q[0]), sep,
               SQRTL(Q[8]), sep, sqvar(Q[1]), sep, sqvar(Q[2]), sep, sqvar(Q[5]), sep, sol->age,
               sep, sol->ratio);

  if (opt->outvel) { /* output velocity */
    soltocov_vel(sol, P);
    long double vel[3];
    ecef2enu(pos, sol->rr + 3, vel);
    covenu(pos, P, Q);
    rtkcatprintf(buff, size,
                 "%s%10.5Lf%s%10.5Lf%s%10.5Lf%s%9.5Lf%s%8.5Lf%s%8.5Lf%s%8.5Lf%s"
                 "%8.5Lf%s%8.5Lf",
                 sep, vel[1], sep, vel[0], sep, vel[2], sep, SQRTL(Q[4]), sep, SQRTL(Q[0]), sep,
                 SQRTL(Q[8]), sep, sqvar(Q[1]), sep, sqvar(Q[2]), sep, sqvar(Q[5]));
  }
  rtkcatprintf(buff, size, "\r\n");
}
/* output solution as the form of e/n/u-baseline -----------------------------*/
static void outenu(char *buff, size_t size, const char *s, const sol_t *sol, const long double *rb,
                   const solopt_t *opt) {
  trace(4, "outenu  :\n");

  long double rr[3];
  for (int i = 0; i < 3; i++) rr[i] = sol->rr[i] - rb[i];
  long double pos[3];
  ecef2pos(rb, pos);
  long double P[9];
  soltocov(sol, P);
  long double Q[9];
  covenu(pos, P, Q);
  long double enu[3];
  ecef2enu(pos, rr, enu);
  const char *sep = opt2sep(opt);
  rtkcatprintf(buff, size,
               "%s%s%14.4Lf%s%14.4Lf%s%14.4Lf%s%3d%s%3d%s%8.4Lf%s%8.4Lf%s%8.4Lf%s"
               "%8.4Lf%s%8.4Lf%s%8.4Lf%s%6.3Lf%s%6.1Lf\r\n",
               s, sep, enu[0], sep, enu[1], sep, enu[2], sep, sol->stat, sep, sol->ns, sep,
               SQRTL(Q[0]), sep, SQRTL(Q[4]), sep, SQRTL(Q[8]), sep, sqvar(Q[1]), sep, sqvar(Q[5]),
               sep, sqvar(Q[2]), sep, sol->age, sep, sol->ratio);
}
/* output solution in the form of NMEA RMC sentence --------------------------*/
extern void outnmea_rmc(char *buff, size_t size, const sol_t *sol) {
  trace(3, "outnmea_rmc:\n");

  if (sol->stat <= SOLQ_NONE) {
    size_t s = strlen(buff);
    rtkcatprintf(buff, size, "$%sRMC,,,,,,,,,,,,,", NMEA_TID);
    char sum = 0;
    for (int i = s + 1; buff[i]; i++) sum ^= buff[i]; /* check-sum */
    rtkcatprintf(buff, size, "*%02X%c%c", sum, 0x0D, 0x0A);
    return;
  }
  gtime_t time = gpst2utc(sol->time);
  if (time.sec >= 0.995L) {
    time.time++;
    time.sec = 0.0L;
  }
  long double ep[6];
  time2epoch(time, ep);
  long double pos[3];
  ecef2pos(sol->rr, pos);
  long double enuv[3];
  ecef2enu(pos, sol->rr + 3, enuv);
  long double vel = norm(enuv, 3);
  static long double dirp = 0.0L;
  long double dir;
  if (vel >= 1.0L) {
    dir = atan2l(enuv[0], enuv[1]) * R2D;
    if (dir < 0.0L) dir += 360.0L;
    dirp = dir;
  } else {
    dir = dirp;
  }
  const char *mode = "A";
  if (sol->stat == SOLQ_DGPS || sol->stat == SOLQ_SBAS)
    mode = "D";
  else if (sol->stat == SOLQ_FLOAT || sol->stat == SOLQ_FIX)
    mode = "R";
  else if (sol->stat == SOLQ_PPP)
    mode = "P";
  long double dms1[3];
  deg2dms(fabsl(pos[0]) * R2D, dms1, 7);
  long double dms2[3];
  deg2dms(fabsl(pos[1]) * R2D, dms2, 7);
  long double amag = 0.0L;
  const char *emag = "E", *status = "V";
  size_t s = strlen(buff);
  rtkcatprintf(buff, size,
               "$%sRMC,%02.0Lf%02.0Lf%05.2Lf,A,%02.0Lf%010.7Lf,%s,%03.0Lf%010.7Lf,"
               "%s,%4.2Lf,%4.2Lf,%02.0Lf%02.0Lf%02d,%.1Lf,%s,%s,%s",
               NMEA_TID, ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0L,
               pos[0] >= 0 ? "N" : "S", dms2[0], dms2[1] + dms2[2] / 60.0L, pos[1] >= 0 ? "E" : "W",
               vel / KNOT2M, dir, ep[2], ep[1], (int)ep[0] % 100, amag, emag, mode, status);
  char sum = 0;
  for (int i = s + 1; buff[i]; i++) sum ^= buff[i]; /* check-sum */
  rtkcatprintf(buff, size, "*%02X\r\n", sum);
}
/* output solution in the form of NMEA GGA sentence --------------------------*/
extern void outnmea_gga(char *buff, size_t size, const sol_t *sol) {
  trace(3, "outnmea_gga:\n");

  if (sol->stat <= SOLQ_NONE) {
    rtkcatprintf(buff, size, "$%sGGA,,,,,,,,,,,,,,", NMEA_TID);
    char sum = 0;
    for (char *q = (char *)buff + 1; *q; q++) sum ^= *q;
    rtkcatprintf(buff, size, "*%02X%c%c", sum, 0x0D, 0x0A);
    return;
  }
  int solq;
  for (solq = 0; solq < 8; solq++)
    if (nmea_solq[solq] == sol->stat) break;
  if (solq >= 8) solq = 0;
  gtime_t time = gpst2utc(sol->time);
  if (time.sec >= 0.995L) {
    time.time++;
    time.sec = 0.0L;
  }
  long double ep[6];
  time2epoch(time, ep);
  long double pos[3];
  ecef2pos(sol->rr, pos);
  long double h = geoidh(pos);
  long double dms1[3];
  deg2dms(fabsl(pos[0]) * R2D, dms1, 7);
  long double dms2[3];
  deg2dms(fabsl(pos[1]) * R2D, dms2, 7);
  long double dop = 1.0L;
  size_t s = strlen(buff);
  rtkcatprintf(buff, size,
               "$%sGGA,%02.0Lf%02.0Lf%05.2Lf,%02.0Lf%010.7Lf,%s,%03.0Lf%010.7Lf,%s,"
               "%d,%02d,%.1Lf,%.3Lf,M,%.3Lf,M,%.3Lf,%04d",
               NMEA_TID, ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0L,
               pos[0] >= 0 ? "N" : "S", dms2[0], dms2[1] + dms2[2] / 60.0L, pos[1] >= 0 ? "E" : "W",
               solq, sol->ns, dop, pos[2] - h, h, sol->age, sol->refstationid);
  char sum = 0;
  for (int i = s + 1; buff[i]; i++) sum ^= buff[i]; /* check-sum */
  rtkcatprintf(buff, size, "*%02X\r\n", sum);
}
/* output solution in the form of NMEA GSA sentences -------------------------*/
extern void outnmea_gsa(char *buff, size_t size, const sol_t *sol, const ssat_t *ssat) {
  trace(3, "outnmea_gsa:\n");

  long double azel[MAXSAT * 2];
  int nsat = 0, mask = 0, nsys = 0, sats[MAXSAT];
  for (int i = 0; i < MAXSAT; i++) {
    if (!ssat[i].vs) continue;
    int sys = satsys(i + 1, NULL);
    if (!(sys & mask)) nsys++; /* # of systems */
    mask |= sys;
    azel[2 * nsat] = ssat[i].azel[0];
    azel[2 * nsat + 1] = ssat[i].azel[1];
    sats[nsat++] = i + 1;
  }
  long double dop[4];
  dops(nsat, azel, 0.0L, dop);

  for (int i = 0; nmea_sys[i]; i++) {
    nsat = 0;
    for (int j = 0; j < MAXSAT && nsat < 12; j++) {
      if (!(satsys(j + 1, NULL) & nmea_sys[i])) continue;
      if (ssat[j].vs) sats[nsat++] = j + 1;
    }
    if (nsat > 0) {
      size_t s = strlen(buff);
      rtkcatprintf(buff, size, "$%sGSA,A,%d", nsys > 1 ? "GN" : nmea_tid[i], sol->stat ? 3 : 1);
      for (int j = 0; j < 12; j++) {
        if (j < nsat) {
          int prn;
          int sys = satsys(sats[j], &prn);
          if (sys == SYS_SBS)
            prn -= 87; /* SBS: 33-64 */
          else if (sys == SYS_GLO)
            prn += 64; /* GLO: 65-99 */
          else if (sys == SYS_QZS)
            prn -= 192; /* QZS: 01-10 */
          rtkcatprintf(buff, size, ",%02d", prn);
        } else
          rtkcatprintf(buff, size, ",");
      }
      rtkcatprintf(buff, size, ",%3.1Lf,%3.1Lf,%3.1Lf,%d", dop[1], dop[2], dop[3], nmea_sid[i]);
      char sum = 0;
      for (int k = s + 1; buff[k]; k++) sum ^= buff[k]; /* check-sum */
      rtkcatprintf(buff, size, "*%02X\r\n", sum);
    }
  }
}
/* output solution in the form of NMEA GSV sentences -------------------------*/
extern void outnmea_gsv(char *buff, size_t size, const sol_t *sol, const ssat_t *ssat) {
  trace(3, "outnmea_gsv:\n");

  for (int i = 0; nmea_sys[i]; i++) {
    int nsat = 0, sats[MAXSAT];
    for (int j = 0; j < MAXSAT && nsat < 36; j++) {
      if (!(satsys(j + 1, NULL) & nmea_sys[i])) continue;
      if (ssat[j].azel[1] > 0.0L) sats[nsat++] = j + 1;
    }
    int nmsg = (nsat + 3) / 4;

    for (int j = 0, n = 0; j < nmsg; j++) {
      size_t s = strlen(buff);
      rtkcatprintf(buff, size, "$%sGSV,%d,%d,%02d", nmea_tid[i], nmsg, j + 1, nsat);
      for (int k = 0; k < 4; k++, n++) {
        if (n < nsat) {
          int prn;
          int sys = satsys(sats[n], &prn);
          if (sys == SYS_SBS)
            prn -= 87; /* SBS: 33-64 */
          else if (sys == SYS_GLO)
            prn += 64; /* GLO: 65-99 */
          else if (sys == SYS_QZS)
            prn -= 192; /* QZS: 01-10 */
          long double az = ssat[sats[n] - 1].azel[0] * R2D;
          if (az < 0.0L) az += 360.0L;
          long double el = ssat[sats[n] - 1].azel[1] * R2D;
          long double snr = ssat[sats[n] - 1].snr_rover[0] * SNR_UNIT;
          rtkcatprintf(buff, size, ",%02d,%02.0Lf,%03.0Lf,%02.0Lf", prn, el, az, snr);
        } else {
          rtkcatprintf(buff, size, ",,,,");
        }
      }
      rtkcatprintf(buff, size, ",0"); /* all signals */

      char sum = 0;
      for (int k = s + 1; buff[k]; k++) sum ^= buff[k]; /* check-sum */
      rtkcatprintf(buff, size, "*%02X\r\n", sum);
    }
  }
}
/* output processing options ---------------------------------------------------
 * output processing options to buffer
 * args   : char    *buff    IO  output buffer
 *          size_t   size    I   output buffer size
 *          prcopt_t *opt    I   processing options
 * return : none
 * Note   : The output is appended to the buffer which must be nul terminated.
 *-----------------------------------------------------------------------------*/
extern void outprcopts(char *buff, size_t size, const prcopt_t *opt) {
  const int sys[] = {SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS, 0};
  const char *s1[] = {"Single",
                      "DGPS",
                      "Kinematic",
                      "Static",
                      "Static-Start",
                      "Moving-Base",
                      "Fixed",
                      "PPP Kinematic",
                      "PPP Static",
                      "PPP Fixed",
                      "",
                      "",
                      ""};
  trace(3, "outprcopts:\n");

  rtkcatprintf(buff, size, "%s pos mode  : %s\r\n", COMMENTH, s1[opt->mode]);

  if (PMODE_DGPS <= opt->mode && opt->mode <= PMODE_FIXED) {
    const char *s2[] = {
        "L1", "L1+L2/E5b", "L1+L2/E5b+L5", "L1+L2/E5b+L5+L6", "L1+2+3+4+5", "L1+2+3+4+5+6", "",
        "",   ""};
    rtkcatprintf(buff, size, "%s freqs     : %s\r\n", COMMENTH, s2[opt->nf - 1]);
  }
  if (opt->mode > PMODE_SINGLE) {
    const char *s3[] = {"Forward", "Backward", "Combined-Phase Reset", "Combined-No Phase Reset",
                        "",        ""};
    rtkcatprintf(buff, size, "%s solution  : %s\r\n", COMMENTH, s3[opt->soltype]);
  }
  rtkcatprintf(buff, size, "%s elev mask : %.1Lf deg\r\n", COMMENTH, opt->elmin * R2D);
  if (opt->mode > PMODE_SINGLE) {
    rtkcatprintf(buff, size, "%s dynamics  : %s\r\n", COMMENTH, opt->dynamics ? "on" : "off");
    rtkcatprintf(buff, size, "%s tidecorr  : %s\r\n", COMMENTH, opt->tidecorr ? "on" : "off");
  }
  if (opt->mode <= PMODE_FIXED) {
    const char *s4[] = {"OFF",
                        "Broadcast",
                        "SBAS",
                        "Iono-Free LC",
                        "Estimate TEC",
                        "IONEX TEC",
                        "QZSS Broadcast",
                        "",
                        "",
                        "",
                        ""};
    rtkcatprintf(buff, size, "%s ionos opt : %s\r\n", COMMENTH, s4[opt->ionoopt]);
  }
  const char *s5[] = {"OFF", "Saastamoinen", "SBAS", "Estimate ZTD", "Estimate ZTD+Grad", "", "",
                      ""};
  rtkcatprintf(buff, size, "%s tropo opt : %s\r\n", COMMENTH, s5[opt->tropopt]);
  const char *s6[] = {
      "Broadcast", "Precise", "Broadcast+SBAS", "Broadcast+SSR APC", "Broadcast+SSR CoM", "",
      "",          ""};
  rtkcatprintf(buff, size, "%s ephemeris : %s\r\n", COMMENTH, s6[opt->sateph]);
  rtkcatprintf(buff, size, "%s navi sys  :", COMMENTH);
  for (int i = 0; sys[i]; i++) {
    if (opt->navsys & sys[i]) {
      const char *s7[] = {"GPS", "GLONASS", "Galileo", "QZSS", "BDS", "NavIC", "SBAS", "", "", ""};
      rtkcatprintf(buff, size, " %s", s7[i]);
    }
  }
  rtkcatprintf(buff, size, "\r\n");
  if (PMODE_KINEMA <= opt->mode && opt->mode <= PMODE_FIXED) {
    const char *s8[] = {"OFF", "Continuous", "Instantaneous", "Fix and Hold", "", "", ""};
    rtkcatprintf(buff, size, "%s amb res   : %s\r\n", COMMENTH, s8[opt->modear]);
    if (opt->navsys & SYS_GLO) {
      const char *s9[] = {"OFF", "ON", "AutoCal", "Fix and Hold", ""};
      rtkcatprintf(buff, size, "%s amb glo   : %s\r\n", COMMENTH, s9[opt->glomodear]);
    }
    if (opt->thresar[0] > 0.0L)
      rtkcatprintf(buff, size, "%s val thres : %.1Lf\r\n", COMMENTH, opt->thresar[0]);
  }
  if (opt->mode == PMODE_MOVEB && opt->baseline[0] > 0.0L) {
    rtkcatprintf(buff, size, "%s baseline  : %.4Lf %.4Lf m\r\n", COMMENTH, opt->baseline[0],
                 opt->baseline[1]);
  }
  for (int i = 0; i < 2; i++) {
    if (opt->mode == PMODE_SINGLE || (i >= 1 && opt->mode > PMODE_FIXED)) continue;
    rtkcatprintf(buff, size, "%s antenna%d  : %-21s (%7.4Lf %7.4Lf %7.4Lf)\r\n", COMMENTH, i + 1,
                 opt->anttype[i], opt->antdel[i][0], opt->antdel[i][1], opt->antdel[i][2]);
  }
}
/* output solution header ------------------------------------------------------
 * output solution header to buffer
 * args   : char    *buff    IO  output buffer
 *          size_t   size    I   output buffer size
 *          solopt_t *opt    I   solution options
 * return : none
 *-----------------------------------------------------------------------------*/
extern void outsolheads(char *buff, size_t size, const solopt_t *opt) {
  trace(3, "outsolheads:\n");

  buff[0] = '\0';

  if (opt->posf == SOLF_NMEA || opt->posf == SOLF_STAT || opt->posf == SOLF_GSIF) {
    return;
  }

  if (opt->outhead) {
    rtkcatprintf(buff, size, "%s (", COMMENTH);
    if (opt->posf == SOLF_XYZ) {
      rtkcatprintf(buff, size, "x/y/z-ecef=WGS84");
    } else if (opt->posf == SOLF_ENU) {
      rtkcatprintf(buff, size, "e/n/u-baseline=WGS84");
    } else {
      const char *s1[] = {"WGS84", "Tokyo"}, *s2[] = {"ellipsoidal", "geodetic"};
      rtkcatprintf(buff, size, "lat/lon/height=%s/%s", s1[opt->datum], s2[opt->height]);
    }
    const char *leg1 = "Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp";
    const char *leg2 = "ns=# of satellites";
    rtkcatprintf(buff, size, ",%s,%s)\r\n", leg1, leg2);
  }
  int timeu = opt->timeu < 0 ? 0 : (opt->timeu > 20 ? 20 : opt->timeu);
  const char *s3[] = {"GPST", "UTC ", "JST "}, *sep = opt2sep(opt);
  rtkcatprintf(buff, size, "%s  %-*s%s", COMMENTH, (opt->timef ? 16 : 8) + timeu + 1,
               s3[opt->times], sep);

  if (opt->posf == SOLF_LLH) { /* lat/lon/hgt */
    if (opt->degf) {
      rtkcatprintf(buff, size,
                   "%16s%s%16s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s"
                   "%8s%s%6s%s%6s",
                   "latitude(d'\")", sep, "longitude(d'\")", sep, "height(m)", sep, "Q", sep, "ns",
                   sep, "sdn(m)", sep, "sde(m)", sep, "sdu(m)", sep, "sdne(m)", sep, "sdeu(m)", sep,
                   "sdue(m)", sep, "age(s)", sep, "ratio");
    } else {
      rtkcatprintf(buff, size,
                   "%14s%s%14s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s"
                   "%8s%s%6s%s%6s",
                   "latitude(deg)", sep, "longitude(deg)", sep, "height(m)", sep, "Q", sep, "ns",
                   sep, "sdn(m)", sep, "sde(m)", sep, "sdu(m)", sep, "sdne(m)", sep, "sdeu(m)", sep,
                   "sdun(m)", sep, "age(s)", sep, "ratio");
    }
    if (opt->outvel) {
      rtkcatprintf(buff, size, "%s%10s%s%10s%s%10s%s%9s%s%8s%s%8s%s%8s%s%8s%s%8s", sep, "vn(m/s)",
                   sep, "ve(m/s)", sep, "vu(m/s)", sep, "sdvn", sep, "sdve", sep, "sdvu", sep,
                   "sdvne", sep, "sdveu", sep, "sdvun");
    }
  } else if (opt->posf == SOLF_XYZ) { /* x/y/z-ecef */
    rtkcatprintf(buff, size,
                 "%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s"
                 "%s%6s%s%6s",
                 "x-ecef(m)", sep, "y-ecef(m)", sep, "z-ecef(m)", sep, "Q", sep, "ns", sep,
                 "sdx(m)", sep, "sdy(m)", sep, "sdz(m)", sep, "sdxy(m)", sep, "sdyz(m)", sep,
                 "sdzx(m)", sep, "age(s)", sep, "ratio");
    if (opt->outvel) {
      rtkcatprintf(buff, size, "%s%10s%s%10s%s%10s%s%9s%s%8s%s%8s%s%8s%s%8s%s%8s", sep, "vx(m/s)",
                   sep, "vy(m/s)", sep, "vz(m/s)", sep, "sdvx", sep, "sdvy", sep, "sdvz", sep,
                   "sdvxy", sep, "sdvyz", sep, "sdvzx");
    }
  } else if (opt->posf == SOLF_ENU) { /* e/n/u-baseline */
    rtkcatprintf(buff, size,
                 "%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s"
                 "%s%6s%s%6s",
                 "e-baseline(m)", sep, "n-baseline(m)", sep, "u-baseline(m)", sep, "Q", sep, "ns",
                 sep, "sde(m)", sep, "sdn(m)", sep, "sdu(m)", sep, "sden(m)", sep, "sdnu(m)", sep,
                 "sdue(m)", sep, "age(s)", sep, "ratio");
    if (opt->outvel) {
      rtkcatprintf(buff, size, "%s%10s%s%10s%s%10s%s%9s%s%8s%s%8s%s%8s%s%8s%s%8s", sep, "ve(m/s)",
                   sep, "vn(m/s)", sep, "vu(m/s)", sep, "sdve", sep, "sdvn", sep, "sdvu", sep,
                   "sdven", sep, "sdvnu", sep, "sdvue");
    }
  }
  rtkcatprintf(buff, size, "\r\n");
}
/* std-dev of solution -------------------------------------------------------*/
static long double sol_std(const sol_t *sol) {
  /* approximate as max std-dev of 3-axis std-devs */
  if (sol->qr[0] > sol->qr[1] && sol->qr[0] > sol->qr[2]) return SQRTL(sol->qr[0]);
  if (sol->qr[1] > sol->qr[2]) return SQRTL(sol->qr[1]);
  return SQRTL(sol->qr[2]);
}
/* output solution body --------------------------------------------------------
 * output solution body to buffer
 * args   : char   *buff     IO  output buffer
 *          size_t  size     I   output buffer size
 *          sol_t  *sol      I   solution
 *          long double *rb       I   base station position {x,y,z} (ecef) (m)
 *          solopt_t *opt    I   solution options
 * return : none
 * Note   : The output is appended to the buffer which must be nul terminated.
 *-----------------------------------------------------------------------------*/
extern void outsols(char *buff, size_t size, const sol_t *sol, const long double *rb,
                    const solopt_t *opt) {
  trace(4, "outsols :\n");

  /* suppress output if std is over opt->maxsolstd */
  if (opt->maxsolstd > 0.0L && sol_std(sol) > opt->maxsolstd) {
    return;
  }
  if (opt->posf == SOLF_NMEA) {
    if (opt->nmeaintv[0] < 0.0L) return;
    gtime_t ts = {0};
    if (!screent(sol->time, ts, ts, opt->nmeaintv[0])) return;
  }
  if (sol->stat <= SOLQ_NONE || (opt->posf == SOLF_ENU && norm(rb, 3) <= 0.0L)) {
    return;
  }
  int timeu = opt->timeu < 0 ? 0 : (opt->timeu > 20 ? 20 : opt->timeu);

  gtime_t time = sol->time;
  if (opt->times >= TIMES_UTC) time = gpst2utc(time);
  if (opt->times == TIMES_JST) time = timeadd(time, 9 * 3600.0L);

  char s[40];
  const char *sep = opt2sep(opt);
  if (opt->timef)
    time2str(time, s, timeu);
  else {
    int week;
    long double gpst = time2gpst(time, &week);
    if (86400 * 7 - gpst < 0.5L / powl(10.0L, timeu)) {
      week++;
      gpst = 0.0L;
    }
    rtksnprintf(s, sizeof(s), "%4d%.16s%*.*Lf", week, sep, 6 + (timeu <= 0 ? 0 : timeu + 1), timeu,
                gpst);
  }
  switch (opt->posf) {
    case SOLF_LLH:
      outpos(buff, size, s, sol, opt);
      break;
    case SOLF_XYZ:
      outecef(buff, size, s, sol, opt);
      break;
    case SOLF_ENU:
      outenu(buff, size, s, sol, rb, opt);
      break;
    case SOLF_NMEA:
      outnmea_rmc(buff, size, sol);
      outnmea_gga(buff, size, sol);
      break;
  }
}
/* output solution extended ----------------------------------------------------
 * output solution extended information
 * args   : char   *buff    IO  output buffer
 *          size_t *size     I   output buffer size
 *          sol_t  *sol      I   solution
 *          ssat_t *ssat     I   satellite status
 *          solopt_t *opt    I   solution options
 * return : none
 * notes  : only support nmea
 * Notes  : The output is appended to the buffer which must be nul terminated.
 *-----------------------------------------------------------------------------*/
extern void outsolexs(char *buff, size_t size, const sol_t *sol, const ssat_t *ssat,
                      const solopt_t *opt) {
  trace(3, "outsolexs:\n");

  /* suppress output if std is over opt->maxsolstd */
  if (opt->maxsolstd > 0.0L && sol_std(sol) > opt->maxsolstd) {
    return;
  }
  if (opt->posf == SOLF_NMEA) {
    if (opt->nmeaintv[1] < 0.0L) return;
    gtime_t ts = {0};
    if (!screent(sol->time, ts, ts, opt->nmeaintv[1])) return;

    outnmea_gsa(buff, size, sol, ssat);
    outnmea_gsv(buff, size, sol, ssat);
  }
}
/* output processing option ----------------------------------------------------
 * output processing option to file
 * args   : FILE   *fp       I   output file pointer
 *          prcopt_t *opt    I   processing options
 * return : none
 *-----------------------------------------------------------------------------*/
extern void outprcopt(FILE *fp, const prcopt_t *opt) {
  trace(3, "outprcopt:\n");

  char buff[MAXSOLMSG + 1];
  buff[0] = '\0';
  outprcopts(buff, sizeof(buff), opt);
  fputs(buff, fp);
}
/* output solution header ------------------------------------------------------
 * output solution heade to file
 * args   : FILE   *fp       I   output file pointer
 *          solopt_t *opt    I   solution options
 * return : none
 *-----------------------------------------------------------------------------*/
extern void outsolhead(FILE *fp, const solopt_t *opt) {
  trace(3, "outsolhead:\n");

  char buff[MAXSOLMSG + 1];
  buff[0] = '\0';
  outsolheads(buff, sizeof(buff), opt);
  fputs(buff, fp);
}
/* output solution body --------------------------------------------------------
 * output solution body to file
 * args   : FILE   *fp       I   output file pointer
 *          sol_t  *sol      I   solution
 *          long double *rb       I   base station position {x,y,z} (ecef) (m)
 *          solopt_t *opt    I   solution options
 * return : none
 *-----------------------------------------------------------------------------*/
extern void outsol(FILE *fp, const sol_t *sol, const long double *rb, const solopt_t *opt) {
  trace(4, "outsol  :\n");

  char buff[MAXSOLMSG + 1];
  buff[0] = '\0';
  outsols(buff, sizeof(buff), sol, rb, opt);
  fputs(buff, fp);
}
/* output solution extended ----------------------------------------------------
 * output solution extended information to file
 * args   : FILE   *fp       I   output file pointer
 *          sol_t  *sol      I   solution
 *          ssat_t *ssat     I   satellite status
 *          solopt_t *opt    I   solution options
 * return : none
 * notes  : only support nmea
 *-----------------------------------------------------------------------------*/
extern void outsolex(FILE *fp, const sol_t *sol, const ssat_t *ssat, const solopt_t *opt) {
  trace(3, "outsolex:\n");

  char buff[MAXSOLMSG + 1];
  buff[0] = '\0';
  outsolexs(buff, sizeof(buff), sol, ssat, opt);
  fputs(buff, fp);
}
