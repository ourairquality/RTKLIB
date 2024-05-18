/*------------------------------------------------------------------------------
 * preceph.c : precise ephemeris and clock functions
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * references :
 *     [1] S.Hilla, The Extended Standard Product 3 Orbit Format (SP3-c),
 *         12 February, 2007
 *     [2] J.Ray, W.Gurtner, RINEX Extensions to Handle Clock Information,
 *         27 August, 1998
 *     [3] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
 *     [4] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
 *         Space Technology Library, 2004
 *     [5] S.Hilla, The Extended Standard Product 3 Orbit Format (SP3-d),
 *         February 21, 2016
 *
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2009/01/18 1.0  new
 *           2009/01/31 1.1  fix bug on numerical error to read sp3a ephemeris
 *           2009/05/15 1.2  support glonass,galileo,qzs
 *           2009/12/11 1.3  support wild-card expansion of file path
 *           2010/07/21 1.4  added api:
 *                               eci2ecef(),sunmoonpos(),peph2pos(),satantoff(),
 *                               readdcb()
 *                           changed api:
 *                               readsp3()
 *                           deleted api:
 *                               eph2posp()
 *           2010/09/09 1.5  fix problem when precise clock outage
 *           2011/01/23 1.6  support qzss satellite code
 *           2011/09/12 1.7  fix problem on precise clock outage
 *                           move sunmmonpos() to rtkcmn.c
 *           2011/12/01 1.8  modify api readsp3()
 *                           precede later ephemeris if ephemeris is NULL
 *                           move eci2ecef() to rtkcmn.c
 *           2013/05/08 1.9  fix bug on computing std-dev of precise clocks
 *           2013/11/20 1.10 modify option for api readsp3()
 *           2014/04/03 1.11 accept extenstion including sp3,eph,SP3,EPH
 *           2014/05/23 1.12 add function to read sp3 velocity records
 *                           change api: satantoff()
 *           2014/08/31 1.13 add member cov and vco in peph_t sturct
 *           2014/10/13 1.14 fix bug on clock error variance in peph2pos()
 *           2015/05/10 1.15 add api readfcb()
 *                           modify api readdcb()
 *           2017/04/11 1.16 fix bug on antenna offset correction in peph2pos()
 *           2020/11/30 1.17 support SP3-d [5] to accept more than 85 satellites
 *                           support NavIC/IRNSS in API peph2pos()
 *                           LC defined GPS/QZS L1-L2, GLO G1-G2, GAL E1-E5b,
 *                            BDS B1I-B2I and IRN L5-S for API satantoff()
 *                           fix bug on reading SP3 file extension
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define SQR(x) ((x) * (x))

#define NMAX 10          /* order of polynomial interpolation */
#define MAXDTE 900.0L    /* max time difference to ephem time (s) */
#define EXTERR_CLK 1E-3L /* extrapolation error for clock (m/s) */
#define EXTERR_EPH 5E-7L /* extrapolation error for ephem (m/s^2) */
#define MAX_BIAS_SYS 4   /* # of constellations supported */

/* table to translate code to code bias table index  */
static int8_t code_bias_ix[MAX_BIAS_SYS][MAXCODE];
/* initialize code bias lookup table -------------------------------------------
 *       -1 = code not supported
 *        0 = reference code (0 bias)
 *        1-3 = table index for code
 * ----------------------------------------------------------------------------*/
static void init_bias_ix(void) {
  for (int i = 0; i < MAX_BIAS_SYS; i++)
    for (int j = 0; j < MAXCODE; j++) code_bias_ix[i][j] = -1;

  /* GPS */
  code_bias_ix[0][CODE_L1W] = 0;
  code_bias_ix[0][CODE_L1C] = 1;
  code_bias_ix[0][CODE_L1L] = 2;
  code_bias_ix[0][CODE_L1X] = 3;
  code_bias_ix[0][CODE_L2W] = 0;
  code_bias_ix[0][CODE_L2L] = 1;
  code_bias_ix[0][CODE_L2S] = 2;
  code_bias_ix[0][CODE_L2X] = 3;
  /* GLONASS */
  code_bias_ix[1][CODE_L1P] = 0;
  code_bias_ix[1][CODE_L1C] = 1;
  code_bias_ix[1][CODE_L2P] = 0;
  code_bias_ix[1][CODE_L2C] = 1;
  /* Galileo */
  code_bias_ix[2][CODE_L1C] = 0;
  code_bias_ix[2][CODE_L1X] = 1;
  code_bias_ix[2][CODE_L5Q] = 0;
  code_bias_ix[2][CODE_L5I] = 1;
  code_bias_ix[2][CODE_L5X] = 2;
  /* Beidou */
  code_bias_ix[3][CODE_L2I] = 0;
  code_bias_ix[3][CODE_L6I] = 0;
}

/* satellite code to satellite system ----------------------------------------*/
static int code2sys(char code) {
  if (code == 'G' || code == ' ') return SYS_GPS;
  if (code == 'R') return SYS_GLO;
  if (code == 'E') return SYS_GAL; /* SP3-d */
  if (code == 'J') return SYS_QZS; /* SP3-d */
  if (code == 'C') return SYS_CMP; /* SP3-d */
  if (code == 'I') return SYS_IRN; /* SP3-d */
  if (code == 'L') return SYS_LEO; /* SP3-d */
  return SYS_NONE;
}
/* read SP3 header -----------------------------------------------------------*/
static int readsp3h(FILE *fp, gtime_t *time, char *type, int *sats, long double *bfact, char *tsys,
                    size_t tsize) {
  int i = 0, k = 0, ns = 0, nl = 5;

  trace(3, "readsp3h:\n");

  /* TODO: Still using b33 code due to issues with b34 */
  char buff[1024];
  while (fgets(buff, sizeof(buff), fp)) {
    if (buff[0] == '#' && (buff[1] == 'c' || buff[1] == 'd')) {
      *type = buff[2];
      if (str2time(buff, 3, 28, time)) return 0;
    } else if (buff[0] == '+' && buff[1] == ' ') {
      if (i == 2) {
        ns = (int)str2num(buff, 3, 3);
        if (ns > 85) nl = ns / 17 + (ns % 17 != 0);
      }
      for (int j = 0; j < 17 && k < ns; j++) {
        int sys = code2sys(buff[9 + 3 * j]);
        int prn = (int)str2num(buff, 10 + 3 * j, 2);
        if (k < MAXSAT) sats[k++] = satno(sys, prn);
      }
    } else if (i == 2 * nl + 2) { /* %c */
      rtkesubstrcpy(tsys, tsize, buff, 9, 12);
      tsys[3] = '\0';
    } else if (i == 2 * nl + 4) { /* %f */
      bfact[0] = str2num(buff, 3, 10);
      bfact[1] = str2num(buff, 14, 12);
    } else if (i == 2 * nl + 11) {
      break; /* at end of header */
    }
    i = i + 1; /* line counter */
  }
  return ns;
}
/* add precise ephemeris -----------------------------------------------------*/
static bool addpeph(nav_t *nav, const peph_t *peph) {
  if (nav->ne >= nav->nemax) {
    nav->nemax += 256;
    peph_t *nav_peph = (peph_t *)realloc(nav->peph, sizeof(peph_t) * nav->nemax);
    if (!nav_peph) {
      trace(1, "readsp3b malloc error n=%d\n", nav->nemax);
      free(nav->peph);
      nav->peph = NULL;
      nav->ne = nav->nemax = 0;
      return false;
    }
    nav->peph = nav_peph;
  }
  nav->peph[nav->ne++] = *peph;
  return true;
}
/* read SP3 body -------------------------------------------------------------*/
static void readsp3b(FILE *fp, char type, int *sats, int ns, const long double *bfact,
                     const char *tsys, int index, int opt, nav_t *nav) {
  int n = ns * (type == 'P' ? 1 : 2);

  trace(3, "readsp3b: type=%c ns=%d index=%d opt=%d\n", type, ns, index, opt);

  char buff[1024];
  while (fgets(buff, sizeof(buff), fp)) {
    if (!strncmp(buff, "EOF", 3)) break;

    gtime_t time;
    if (buff[0] != '*' || str2time(buff, 3, 28, &time)) {
      trace(2, "sp3 invalid epoch %31.31s\n", buff);
      continue;
    }
    if (!strcmp(tsys, "UTC")) time = utc2gpst(time); /* utc->gpst */
    peph_t peph;
    peph.time = time;
    peph.index = index;

    for (int i = 0; i < MAXSAT; i++) {
      for (int j = 0; j < 4; j++) {
        peph.pos[i][j] = 0.0L;
        peph.std[i][j] = 0.0L;
        peph.vel[i][j] = 0.0L;
        peph.vst[i][j] = 0.0L;
      }
      for (int j = 0; j < 3; j++) {
        peph.cov[i][j] = 0.0L;
        peph.vco[i][j] = 0.0L;
      }
    }
    int pred_o = 0, pred_c = 0, v = 0;
    for (int i = 0; i < n && fgets(buff, sizeof(buff), fp); i++) {
      if (strlen(buff) < 4 || (buff[0] != 'P' && buff[0] != 'V')) continue;

      int sys = buff[1] == ' ' ? SYS_GPS : code2sys(buff[1]);
      int prn = (int)str2num(buff, 2, 2);
      if (sys == SYS_SBS)
        prn += 100;
      else if (sys == SYS_QZS)
        prn += 192; /* extension to sp3-c */

      int sat = satno(sys, prn);
      if (!sat) continue;

      if (buff[0] == 'P') {
        pred_c = strlen(buff) >= 76 && buff[75] == 'P';
        pred_o = strlen(buff) >= 80 && buff[79] == 'P';
      }
      for (int j = 0; j < 4; j++) {
        /* read option for predicted value */
        if (j < 3 && (opt & 1) && pred_o) continue;
        if (j < 3 && (opt & 2) && !pred_o) continue;
        if (j == 3 && (opt & 1) && pred_c) continue;
        if (j == 3 && (opt & 2) && !pred_c) continue;

        long double val = str2num(buff, 4 + j * 14, 14);
        long double std = str2num(buff, 61 + j * 3, j < 3 ? 2 : 3);

        if (buff[0] == 'P') { /* position */
          if (val != 0.0L && fabsl(val - 999999.999999L) >= 1E-6L) {
            peph.pos[sat - 1][j] = val * (j < 3 ? 1000.0L : 1E-6L);
            v = 1; /* valid epoch */
          }
          long double base = bfact[j < 3 ? 0 : 1];
          if (base > 0.0L && std > 0.0L) {
            peph.std[sat - 1][j] = powl(base, std) * (j < 3 ? 1E-3L : 1E-12L);
          }
        } else if (v) { /* velocity */
          if (val != 0.0L && fabsl(val - 999999.999999L) >= 1E-6L) {
            peph.vel[sat - 1][j] = val * (j < 3 ? 0.1L : 1E-10L);
          }
          long double base = bfact[j < 3 ? 0 : 1];
          if (base > 0.0L && std > 0.0L) {
            peph.vst[sat - 1][j] = powl(base, std) * (j < 3 ? 1E-7L : 1E-16L);
          }
        }
      }
    }
    if (v) {
      if (!addpeph(nav, &peph)) return;
    }
  }
}
/* compare precise ephemeris -------------------------------------------------*/
static int cmppeph(const void *p1, const void *p2) {
  const peph_t *q1 = (peph_t *)p1, *q2 = (peph_t *)p2;
  long double tt = timediff(q1->time, q2->time);
  return tt < -1E-9L ? -1 : (tt > 1E-9L ? 1 : q1->index - q2->index);
}
/* combine precise ephemeris -------------------------------------------------*/
static void combpeph(nav_t *nav, int opt) {
  trace(3, "combpeph: ne=%d\n", nav->ne);

  qsort(nav->peph, nav->ne, sizeof(peph_t), cmppeph);

  if (opt & 4) return;

  int i = 0;
  for (int j = 1; j < nav->ne; j++) {
    if (fabsl(timediff(nav->peph[i].time, nav->peph[j].time)) < 1E-9L) {
      for (int k = 0; k < MAXSAT; k++) {
        if (norm(nav->peph[j].pos[k], 4) <= 0.0L) continue;
        for (int m = 0; m < 4; m++) nav->peph[i].pos[k][m] = nav->peph[j].pos[k][m];
        for (int m = 0; m < 4; m++) nav->peph[i].std[k][m] = nav->peph[j].std[k][m];
        for (int m = 0; m < 4; m++) nav->peph[i].vel[k][m] = nav->peph[j].vel[k][m];
        for (int m = 0; m < 4; m++) nav->peph[i].vst[k][m] = nav->peph[j].vst[k][m];
      }
    } else if (++i < j)
      nav->peph[i] = nav->peph[j];
  }
  nav->ne = i + 1;

  trace(4, "combpeph: ne=%d\n", nav->ne);
}
/* read sp3 precise ephemeris file ---------------------------------------------
 * read sp3 precise ephemeris/clock files and set them to navigation data
 * args   : char   *file       I   sp3-c precise ephemeris file
 *                                 (wind-card * is expanded)
 *          nav_t  *nav        IO  navigation data
 *          int    opt         I   options (1: only observed + 2: only predicted +
 *                                 4: not combined)
 * return : none
 * notes  : see ref [1]
 *          precise ephemeris is appended and combined
 *          nav->peph and nav->ne must by properly initialized before calling the
 *          function
 *          only files with extensions of .sp3, .SP3, .eph* and .EPH* are read
 *-----------------------------------------------------------------------------*/
extern void readsp3(const char *file, nav_t *nav, int opt) {
  trace(3, "readpephs: file=%s\n", file);

  char *efiles[MAXEXFILE];
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(efiles[i] = (char *)malloc(FNSIZE))) {
      for (i--; i >= 0; i--) free(efiles[i]);
      return;
    }
  }
  /* expand wild card in file path */
  int n = expath(file, efiles, FNSIZE, MAXEXFILE);

  int j;
  for (int i = j = 0; i < n; i++) {
    char *ext = strrchr(efiles[i], '.');
    if (!ext) continue;

    if (!strstr(ext, ".sp3") && !strstr(ext, ".SP3") && !strstr(ext, ".eph") &&
        !strstr(ext, ".EPH"))
      continue;

    FILE *fp = fopen(efiles[i], "r");
    if (!fp) {
      trace(2, "sp3 file open error %s\n", efiles[i]);
      continue;
    }
    /* read sp3 header */
    gtime_t time = {0};
    char type = ' ', tsys[4] = "";
    int sats[MAXSAT] = {0};
    long double bfact[2] = {0};
    int ns = readsp3h(fp, &time, &type, sats, bfact, tsys, sizeof(tsys));

    /* read sp3 body */
    readsp3b(fp, type, sats, ns, bfact, tsys, j++, opt, nav);

    fclose(fp);
  }
  for (int i = 0; i < MAXEXFILE; i++) free(efiles[i]);

  /* combine precise ephemeris */
  if (nav->ne > 0) combpeph(nav, opt);
}
/* read satellite antenna parameters -------------------------------------------
 * read satellite antenna parameters
 * args   : char   *file       I   antenna parameter file
 *          gtime_t time       I   time
 *          nav_t  *nav        IO  navigation data
 * return : status (true:ok,false:error)
 * notes  : only support antex format for the antenna parameter file
 *-----------------------------------------------------------------------------*/
extern bool readsap(const char *file, gtime_t time, nav_t *nav) {
  char tstr[40];
  trace(3, "readsap : file=%s time=%s\n", file, time2str(time, tstr, 0));

  pcvs_t pcvs = {0};
  if (!readpcv(file, &pcvs)) return false;

  for (int i = 0; i < MAXSAT; i++) {
    pcv_t *pcv = searchpcv(i + 1, "", time, &pcvs);
    pcv_t pcv0 = {0};
    nav->pcvs[i] = pcv ? *pcv : pcv0;
  }
  free(pcvs.pcv);
  return true;
}
/* read DCB parameters from DCB file -------------------------------------------
 *    - supports satellite and receiver biases
 *-----------------------------------------------------------------------------*/
static bool readdcbf(const char *file, nav_t *nav, const sta_t *sta) {
  trace(3, "readdcbf: file=%s\n", file);

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "dcb parameters file open error: %s\n", file);
    return false;
  }

  char buff[256];
  int type = 0;
  while (fgets(buff, sizeof(buff), fp)) {
    if (strstr(buff, "DIFFERENTIAL (P1-C1) CODE BIASES"))
      type = 1;
    else if (strstr(buff, "DIFFERENTIAL (P2-C2) CODE BIASES"))
      type = 2;

    char str1[32], str2[32] = "";
    if (!type || sscanf(buff, "%31s %31s", str1, str2) < 1) continue;

    long double cbias = str2num(buff, 26, 9);
    if (cbias == 0.0L) continue;

    if (sta && (!strcmp(str1, "G") || !strcmp(str1, "R"))) { /* receiver DCB */
      int i;
      for (i = 0; i < MAXRCV; i++) {
        if (!strcmp(sta[i].name, str2)) break;
      }
      if (i < MAXRCV) {
        int j = !strcmp(str1, "G") ? 0 : 1;
        nav->rbias[i][j][type - 1] = cbias * 1E-9L * CLIGHT; /* ns -> m */
      }
    } else {
      int sat = satid2no(str1);
      if (sat) {
        /* satellite dcb */
        nav->cbias[sat - 1][type - 1][0] = cbias * 1E-9L * CLIGHT; /* ns -> m */
      }
    }
  }
  fclose(fp);

  return true;
}
/* satellite system to index */
static int sys2ix(int sys) {
  const int ix[] = {0,  /* Undefined */
                    0,  /* GPS */
                    0,  /* SBS */
                    1,  /* GLO */
                    2,  /* GAL */
                    4,  /* QZS */
                    3,  /* CMP */
                    5,  /* IRN */
                    0}; /* LEO */
  return ix[sys2no(sys)];
}
/* translate satellite system and code to code bias table index ----------------
 *       -1 = code not supported
 *        0 = reference code (0 bias)
 *        1-3 = table index for code
 * ----------------------------------------------------------------------------*/
extern int code2bias_ix(int sys, int code) {
  int sys_ix = sys2ix(sys);
  if (sys_ix < MAX_BIAS_SYS)
    return code_bias_ix[sys_ix][code];
  else
    return 0;
}
/* read DCB parameters from BIA or BSX file ------------------------------------
 *    - supports satellite code biases only
 *-----------------------------------------------------------------------------*/
static bool readbiaf(const char *file, nav_t *nav) {
  trace(3, "readbiaf: file=%s\n", file);

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "dcb parameters file open error: %s\n", file);
    return false;
  }
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    char bias[6] = "", svn[6] = "", prn[6] = "", obs1[6] = "", obs2[6];
    if (sscanf(buff, "%4s %5s %4s %4s %4s", bias, svn, prn, obs1, obs2) < 5) continue;
    if (obs1[0] != 'C') continue; /* skip phase biases for now */
    long double cbias = str2num(buff, 70, 21);
    if (cbias == 0.0L) continue;
    int sat = satid2no(prn);
    int sys = satsys(sat, NULL);
    /* other code biases are L1/L2, Galileo is L1/L5 */
    int freq;
    if (obs1[1] == '1')
      freq = 0;
    else if ((sys != SYS_GAL && obs1[1] == '2') || (sys == SYS_GAL && obs1[1] == '5'))
      freq = 1;
    else
      continue;

    int code1 = obs2code(&obs1[1]);
    if (!code1) continue; /* skip if code not valid */
    int bias_ix1 = code2bias_ix(sys, code1);

    if (strcmp(bias, "OSB") == 0) {
      /* observed signal bias */
      if (bias_ix1 == 0) {                        /* this is ref code */
        for (int i = 0; i < MAX_CODE_BIASES; i++) /* adjust all other codes by ref code bias */
          nav->cbias[sat - 1][freq][i] += cbias * 1E-9L * CLIGHT; /* ns -> m */
      } else {
        nav->cbias[sat - 1][freq][bias_ix1 - 1] -= cbias * 1E-9L * CLIGHT; /* ns -> m */
      }
    } else if (strcmp(bias, "DSB") == 0) {
      /* differential signal bias */
      if (obs1[1] != obs2[1]) continue; /* skip biases between freqs for now */
      int code2 = obs2code(&obs2[1]);
      if (!code2) continue; /* skip if code not valid */
      int bias_ix2 = code2bias_ix(sys, code2);
      if (bias_ix1 == 0)                                                   /* this is ref code */
        nav->cbias[sat - 1][freq][bias_ix2 - 1] = cbias * 1E-9L * CLIGHT;  /* ns -> m */
      else if (bias_ix2 == 0)                                              /* this is ref code */
        nav->cbias[sat - 1][freq][bias_ix1 - 1] = -cbias * 1E-9L * CLIGHT; /* ns -> m */
    }
  }
  fclose(fp);

  return true;
}
/* read DCB parameters ---------------------------------------------------------
* read differential code bias (DCB) parameters
* args   : char   *file       I   DCB parameters file (wild-card * expanded)
*          nav_t  *nav        IO  navigation data
*          sta_t  *sta        I   station info data to import receiver DCB
*                                 (NULL: no use)
* return : status (true:ok,false:error)
* notes  : supports DCB, BIA, and BSX file formats
         : currently only support P1-P2, P1-C1 bias in DCB file
         : currently only supports satellite biases in BIA/BSX files
*-----------------------------------------------------------------------------*/
extern bool readdcb(const char *file, nav_t *nav, const sta_t *sta) {
  trace(3, "readdcb : file=%s\n", file);

  init_bias_ix(); /* init translation table from code to table column */

  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < MAX_CODE_BIAS_FREQS; j++)
      for (int k = 0; k < MAX_CODE_BIASES; k++) {
        nav->cbias[i][j][k] = 0.0L;
      }
  char *efiles[MAXEXFILE] = {0};
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(efiles[i] = (char *)malloc(FNSIZE))) {
      for (i--; i >= 0; i--) free(efiles[i]);
      return false;
    }
  }
  int n = expath(file, efiles, FNSIZE, MAXEXFILE);

  int dcb_ok = 0;
  for (int i = 0; i < n; i++) {
    if (strstr(efiles[i], ".BIA") || strstr(efiles[i], ".bia") || strstr(efiles[i], ".BSX") ||
        strstr(efiles[i], ".bsx"))
      dcb_ok = readbiaf(efiles[i], nav);
    else if (strstr(efiles[i], ".DCB") || strstr(efiles[i], ".dcb"))
      dcb_ok = readdcbf(efiles[i], nav, sta);
  }
  for (int i = 0; i < MAXEXFILE; i++) free(efiles[i]);

  return dcb_ok;
}
/* polynomial interpolation by Neville's algorithm ---------------------------*/
static long double interppol(const long double *x, long double *y, int n) {
  for (int j = 1; j < n; j++) {
    for (int i = 0; i < n - j; i++) {
      y[i] = (x[i + j] * y[i] - x[i] * y[i + 1]) / (x[i + j] - x[i]);
    }
  }
  return y[0];
}
/* satellite position by precise ephemeris -----------------------------------*/
static bool pephpos(gtime_t time, int sat, const nav_t *nav, long double *rs, long double *dts,
                    long double *vare, long double *varc) {
  char tstr[40];
  trace(4, "pephpos : time=%s sat=%2d\n", time2str(time, tstr, 3), sat);

  rs[0] = rs[1] = rs[2] = dts[0] = 0.0L;

  if (nav->ne < NMAX + 1 || timediff(time, nav->peph[0].time) < -MAXDTE ||
      timediff(time, nav->peph[nav->ne - 1].time) > MAXDTE) {
    trace(3, "no prec ephem %s sat=%2d\n", time2str(time, tstr, 0), sat);
    return false;
  }
  /* binary search */
  int i = 0;
  for (int j = nav->ne - 1; i < j;) {
    int k = (i + j) / 2;
    if (timediff(nav->peph[k].time, time) < 0.0L)
      i = k + 1;
    else
      j = k;
  }
  int index = i <= 0 ? 0 : i - 1;

  /* polynomial interpolation for orbit */
  i = index - (NMAX + 1) / 2;
  if (i < 0)
    i = 0;
  else if (i + NMAX >= nav->ne)
    i = nav->ne - NMAX - 1;

  long double t[NMAX + 1];
  for (int j = 0; j <= NMAX; j++) {
    t[j] = timediff(nav->peph[i + j].time, time);
    if (norm(nav->peph[i + j].pos[sat - 1], 3) <= 0.0L) {
      trace(3, "prec ephem outage %s sat=%2d\n", time2str(time, tstr, 0), sat);
      return false;
    }
  }
  long double p[3][NMAX + 1];
  for (int j = 0; j <= NMAX; j++) {
    const long double *pos = nav->peph[i + j].pos[sat - 1];
    /* correction for earth rotation ver.2.4.0 */
    long double sinll = sinl(OMGE * t[j]);
    long double cosll = cosl(OMGE * t[j]);
    p[0][j] = cosll * pos[0] - sinll * pos[1];
    p[1][j] = sinll * pos[0] + cosll * pos[1];
    p[2][j] = pos[2];
  }
  for (int i2 = 0; i2 < 3; i2++) {
    rs[i2] = interppol(t, p[i2], NMAX + 1);
  }
  long double std = 0.0L;
  if (vare) {
    long double s[3];
    for (int i2 = 0; i2 < 3; i2++) s[i2] = nav->peph[index].std[sat - 1][i2];
    std = norm(s, 3);

    /* extrapolation error for orbit */
    if (t[0] > 0.0L)
      std += EXTERR_EPH * SQR(t[0]) / 2.0L;
    else if (t[NMAX] < 0.0L)
      std += EXTERR_EPH * SQR(t[NMAX]) / 2.0L;
    *vare = SQR(std);
  }
  /* linear interpolation for clock */
  long double c[2];
  t[0] = timediff(time, nav->peph[index].time);
  t[1] = timediff(time, nav->peph[index + 1].time);
  c[0] = nav->peph[index].pos[sat - 1][3];
  c[1] = nav->peph[index + 1].pos[sat - 1][3];

  if (t[0] <= 0.0L) {
    if ((dts[0] = c[0]) != 0.0L) {
      std = nav->peph[index].std[sat - 1][3] * CLIGHT - EXTERR_CLK * t[0];
    }
  } else if (t[1] >= 0.0L) {
    if ((dts[0] = c[1]) != 0.0L) {
      std = nav->peph[index + 1].std[sat - 1][3] * CLIGHT + EXTERR_CLK * t[1];
    }
  } else if (c[0] != 0.0L && c[1] != 0.0L) {
    dts[0] = (c[1] * t[0] - c[0] * t[1]) / (t[0] - t[1]);
    i = t[0] < -t[1] ? 0 : 1;
    std = nav->peph[index + i].std[sat - 1][3] + EXTERR_CLK * fabsl(t[i]);
  } else {
    dts[0] = 0.0L;
  }
  if (varc) *varc = SQR(std);
  return true;
}
/* satellite clock by precise clock ------------------------------------------*/
static int pephclk(gtime_t time, int sat, const nav_t *nav, long double *dts, long double *varc) {
  char tstr[40];
  trace(4, "pephclk : time=%s sat=%2d\n", time2str(time, tstr, 3), sat);

  if (nav->nc < 2 || timediff(time, nav->pclk[0].time) < -MAXDTE ||
      timediff(time, nav->pclk[nav->nc - 1].time) > MAXDTE) {
    trace(3, "no prec clock %s sat=%2d\n", time2str(time, tstr, 0), sat);
    return true;
  }
  /* binary search */
  int i = 0;
  for (int j = nav->nc - 1; i < j;) {
    int k = (i + j) / 2;
    if (timediff(nav->pclk[k].time, time) < 0.0L)
      i = k + 1;
    else
      j = k;
  }
  int index = i <= 0 ? 0 : i - 1;

  /* linear interpolation for clock */
  long double t[2], c[2];
  t[0] = timediff(time, nav->pclk[index].time);
  t[1] = timediff(time, nav->pclk[index + 1].time);
  c[0] = nav->pclk[index].clk[sat - 1][0];
  c[1] = nav->pclk[index + 1].clk[sat - 1][0];

  long double std;
  if (t[0] <= 0.0L) {
    if ((dts[0] = c[0]) == 0.0L) return false;
    std = nav->pclk[index].std[sat - 1][0] * CLIGHT - EXTERR_CLK * t[0];
  } else if (t[1] >= 0.0L) {
    if ((dts[0] = c[1]) == 0.0L) return false;
    std = nav->pclk[index + 1].std[sat - 1][0] * CLIGHT + EXTERR_CLK * t[1];
  } else if (c[0] != 0.0L && c[1] != 0.0L) {
    dts[0] = (c[1] * t[0] - c[0] * t[1]) / (t[0] - t[1]);
    i = t[0] < -t[1] ? 0 : 1;
    std = nav->pclk[index + i].std[sat - 1][0] * CLIGHT + EXTERR_CLK * fabsl(t[i]);
  } else {
    trace(3, "prec clock outage %s sat=%2d\n", time2str(time, tstr, 0), sat);
    return false;
  }
  if (varc) *varc = SQR(std);
  return true;
}
/* satellite antenna phase center offset ---------------------------------------
 * compute satellite antenna phase center offset in ecef
 * args   : gtime_t time       I   time (gpst)
 *          long double *rs         I   satellite position and velocity (ecef)
 *                                 {x,y,z,vx,vy,vz} (m|m/s)
 *          int    sat         I   satellite number
 *          nav_t  *nav        I   navigation data
 *          long double *dant       O   satellite antenna phase center offset (ecef)
 *                                 {dx,dy,dz} (m) (iono-free LC value)
 * return : none
 * notes  : iono-free LC frequencies defined as follows:
 *            GPS/QZSS : L1-L2
 *            GLONASS  : G1-G2
 *            Galileo  : E1-E5b
 *            BDS      : B1I-B2I
 *            NavIC    : L5-S
 *-----------------------------------------------------------------------------*/
extern void satantoff(gtime_t time, const long double *rs, int sat, const nav_t *nav,
                      long double *dant) {
  char tstr[40];
  trace(4, "satantoff: time=%s sat=%2d\n", time2str(time, tstr, 3), sat);

  dant[0] = dant[1] = dant[2] = 0.0L;

  /* sun position in ecef */
  long double rsun[3], gmst;
  const long double erpv[5] = {0};
  sunmoonpos(gpst2utc(time), erpv, rsun, NULL, &gmst);

  /* unit vectors of satellite fixed coordinates */
  long double r[3];
  for (int i = 0; i < 3; i++) r[i] = -rs[i];
  long double ez[3];
  if (!normv3(r, ez)) return;
  for (int i = 0; i < 3; i++) r[i] = rsun[i] - rs[i];
  long double es[3];
  if (!normv3(r, es)) return;
  cross3(ez, es, r);
  long double ey[3];
  if (!normv3(r, ey)) return;
  long double ex[3];
  cross3(ey, ez, ex);

  /* iono-free LC coefficients */
  int sys = satsys(sat, NULL);
  long double freq[2];
  if (sys == SYS_GPS || sys == SYS_QZS) { /* L1-L2 */
    freq[0] = FREQL1;
    freq[1] = FREQL2;
  } else if (sys == SYS_GLO) { /* G1-G2 */
    freq[0] = sat2freq(sat, CODE_L1C, nav);
    freq[1] = sat2freq(sat, CODE_L2C, nav);
  } else if (sys == SYS_GAL) { /* E1-E5b */
    freq[0] = FREQL1;
    freq[1] = FREQE5b;
  } else if (sys == SYS_CMP) { /* B1I-B2I */
    freq[0] = FREQ1_CMP;
    freq[1] = FREQ2_CMP;
  } else if (sys == SYS_IRN) { /* L5-S */
    freq[0] = FREQL5;
    freq[1] = FREQs;
  } else
    return;

  long double C1 = SQR(freq[0]) / (SQR(freq[0]) - SQR(freq[1]));
  long double C2 = -SQR(freq[1]) / (SQR(freq[0]) - SQR(freq[1]));

  /* iono-free LC */
  const pcv_t *pcv = nav->pcvs + sat - 1;
  for (int i = 0; i < 3; i++) {
    long double dant1 = pcv->off[0][0] * ex[i] + pcv->off[0][1] * ey[i] + pcv->off[0][2] * ez[i];
    long double dant2 = pcv->off[1][0] * ex[i] + pcv->off[1][1] * ey[i] + pcv->off[1][2] * ez[i];
    dant[i] = C1 * dant1 + C2 * dant2;
  }
}
/* satellite position/clock by precise ephemeris/clock -------------------------
 * compute satellite position/clock with precise ephemeris/clock
 * args   : gtime_t time       I   time (gpst)
 *          int    sat         I   satellite number
 *          nav_t  *nav        I   navigation data
 *          int    opt         I   sat position option
 *                                 (0: center of mass, 1: antenna phase center)
 *          long double *rs         O   sat position and velocity (ecef)
 *                                 {x,y,z,vx,vy,vz} (m|m/s)
 *          long double *dts        O   sat clock {bias,drift} (s|s/s)
 *          long double *var        IO  sat position and clock error variance (m)
 *                                 (NULL: no output)
 * return : status (true:ok,false:error or data outage)
 * notes  : clock includes relativistic correction but does not contain code bias
 *          before calling the function, nav->peph, nav->ne, nav->pclk and
 *          nav->nc must be set by calling readsp3(), readrnx() or readrnxt()
 *          if precise clocks are not set, clocks in sp3 are used instead
 *-----------------------------------------------------------------------------*/
extern bool peph2pos(gtime_t time, int sat, const nav_t *nav, int opt, long double *rs,
                     long double *dts, long double *var) {
  char tstr[40];
  trace(4, "peph2pos: time=%s sat=%2d opt=%d\n", time2str(time, tstr, 3), sat, opt);

  if (sat <= 0 || MAXSAT < sat) return false;

  /* satellite position and clock bias */
  long double rss[3], dtss[1], vare = 0.0L, varc = 0.0L;
  if (!pephpos(time, sat, nav, rss, dtss, &vare, &varc) || !pephclk(time, sat, nav, dtss, &varc))
    return false;

  long double tt = 1E-3L;
  gtime_t time_tt = timeadd(time, tt);
  long double rst[3], dtst[1];
  if (!pephpos(time_tt, sat, nav, rst, dtst, NULL, NULL) || !pephclk(time_tt, sat, nav, dtst, NULL))
    return false;

  /* satellite antenna offset correction */
  long double dant[3] = {0};
  if (opt) {
    satantoff(time, rss, sat, nav, dant);
  }
  for (int i = 0; i < 3; i++) {
    rs[i] = rss[i] + dant[i];
    rs[i + 3] = (rst[i] - rss[i]) / tt;
  }
  /* relativistic effect correction */
  if (dtss[0] != 0.0L) {
    dts[0] = dtss[0] - 2.0L * dot3(rs, rs + 3) / CLIGHT / CLIGHT;
    dts[1] = (dtst[0] - dtss[0]) / tt;
  } else { /* no precise clock */
    dts[0] = dts[1] = 0.0L;
  }
  if (var) *var = vare + varc;

  return true;
}
