/*------------------------------------------------------------------------------
 * ionex.c : ionex functions
 *
 *          Copyright (C) 2011-2013 by T.TAKASU, All rights reserved.
 *
 * references:
 *     [1] S.Schear, W.Gurtner and J.Feltens, IONEX: The IONosphere Map EXchange
 *         Format Version 1, February 25, 1998
 *     [2] S.Schaer, R.Markus, B.Gerhard and A.S.Timon, Daily Global Ionosphere
 *         Maps based on GPS Carrier Phase Data Routinely produced by CODE
 *         Analysis Center, Proceeding of the IGS Analysis Center Workshop, 1996
 *
 * version : $Revision:$ $Date:$
 * history : 2011/03/29 1.0 new
 *           2013/03/05 1.1 change api readtec()
 *                          fix problem in case of lat>85deg or lat<-85deg
 *           2014/02/22 1.2 fix problem on compiled as C++
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define SQR(x) ((x) * (x))
#define VAR_NOTEC SQR(30.0L) /* variance of no tec */
#define MIN_EL 0.0L          /* min elevation angle (rad) */
#define MIN_HGT -1000.0L     /* min user height (m) */

/* get index -----------------------------------------------------------------*/
static int getindex(long double value, const long double *range) {
  if (range[2] == 0.0L) return 0;
  if (range[1] > 0.0L && (value < range[0] || range[1] < value)) return -1;
  if (range[1] < 0.0L && (value < range[1] || range[0] < value)) return -1;
  return (int)floorl((value - range[0]) / range[2] + 0.5L);
}
/* get number of items -------------------------------------------------------*/
static int nitem(const long double *range) { return getindex(range[1], range) + 1; }
/* data index (i:lat,j:lon,k:hgt) --------------------------------------------*/
static int dataindex(int i, int j, int k, const int *ndata) {
  if (i < 0 || ndata[0] <= i || j < 0 || ndata[1] <= j || k < 0 || ndata[2] <= k) return -1;
  return i + ndata[0] * (j + ndata[1] * k);
}
/* add tec data to navigation data -------------------------------------------*/
static tec_t *addtec(const long double *lats, const long double *lons, const long double *hgts,
                     long double rb, nav_t *nav) {
  trace(3, "addtec  :\n");

  int ndata[3];
  ndata[0] = nitem(lats);
  ndata[1] = nitem(lons);
  ndata[2] = nitem(hgts);
  gtime_t time0 = {0};
  if (ndata[0] > 1 && ndata[1] > 1 && ndata[2] > 0) {
    if (nav->nt >= nav->ntmax) {
      nav->ntmax += 256;
      tec_t *nav_tec = (tec_t *)realloc(nav->tec, sizeof(tec_t) * nav->ntmax);
      if (!nav_tec) {
        trace(1, "readionex malloc error ntmax=%d\n", nav->ntmax);
        free(nav->tec);
        nav->tec = NULL;
        nav->nt = nav->ntmax = 0;
        return NULL;
      }
      for (int indx = nav->ntmax - 1; indx >= nav->ntmax - 256; indx--)
        memset(&nav_tec[indx], 0, sizeof(tec_t));
      nav->tec = nav_tec;
    }
    tec_t *p = nav->tec + nav->nt;
    p->time = time0;
    p->rb = rb;
    for (int i = 0; i < 3; i++) {
      p->ndata[i] = ndata[i];
      p->lats[i] = lats[i];
      p->lons[i] = lons[i];
      p->hgts[i] = hgts[i];
    }
    int n = ndata[0] * ndata[1] * ndata[2];

    if (!(p->data = (long double *)malloc(sizeof(long double) * n)) ||
        !(p->rms = (long double *)malloc(sizeof(long double) * n))) {
      return NULL;
    }
    for (int i = 0; i < n; i++) {
      /* Thanks to 'if (ndata[0]>1 && ndata[1]>1 && ndata[2]>0)' we know analysis is wrong -
       * disable 6386 */
      p->data[i] = 0.0L;
      p->rms[i] = 0.0L;
    }
    nav->nt++;
    return p;
  } else
    return NULL;
}
/* read ionex dcb aux data ----------------------------------------------------*/
static void readionexdcb(FILE *fp, long double *dcb, long double *rms) {
  trace(3, "readionexdcb:\n");

  for (int i = 0; i < MAXSAT; i++) dcb[i] = rms[i] = 0.0L;

  char buff[1024];
  while (fgets(buff, sizeof(buff), fp)) {
    if (strlen(buff) < 60) continue;
    char *label = buff + 60;
    if (strstr(label, "PRN / BIAS / RMS") == label) {
      char id[8];
      rtkesubstrcpy(id, sizeof(id), buff, 3, 6);

      int sat = satid2no(id);
      if (!sat) {
        trace(2, "ionex invalid satellite: %s\n", id);
        continue;
      }
      dcb[sat - 1] = str2num(buff, 6, 10);
      rms[sat - 1] = str2num(buff, 16, 10);
    } else if (strstr(label, "END OF AUX DATA") == label)
      break;
  }
}
/* read ionex header ---------------------------------------------------------*/
static long double readionexh(FILE *fp, long double *lats, long double *lons, long double *hgts,
                              long double *rb, long double *nexp, long double *dcb,
                              long double *rms) {
  long double ver = 0.0L;
  char buff[1024], *label;

  trace(3, "readionexh:\n");

  while (fgets(buff, sizeof(buff), fp)) {
    if (strlen(buff) < 60) continue;
    label = buff + 60;

    if (strstr(label, "IONEX VERSION / TYPE") == label) {
      if (buff[20] == 'I') ver = str2num(buff, 0, 8);
    } else if (strstr(label, "BASE RADIUS") == label) {
      *rb = str2num(buff, 0, 8);
    } else if (strstr(label, "HGT1 / HGT2 / DHGT") == label) {
      hgts[0] = str2num(buff, 2, 6);
      hgts[1] = str2num(buff, 8, 6);
      hgts[2] = str2num(buff, 14, 6);
    } else if (strstr(label, "LAT1 / LAT2 / DLAT") == label) {
      lats[0] = str2num(buff, 2, 6);
      lats[1] = str2num(buff, 8, 6);
      lats[2] = str2num(buff, 14, 6);
    } else if (strstr(label, "LON1 / LON2 / DLON") == label) {
      lons[0] = str2num(buff, 2, 6);
      lons[1] = str2num(buff, 8, 6);
      lons[2] = str2num(buff, 14, 6);
    } else if (strstr(label, "EXPONENT") == label) {
      *nexp = str2num(buff, 0, 6);
    } else if (strstr(label, "START OF AUX DATA") == label &&
               strstr(buff, "DIFFERENTIAL CODE BIASES")) {
      readionexdcb(fp, dcb, rms);
    } else if (strstr(label, "END OF HEADER") == label) {
      return ver;
    }
  }
  return 0.0L;
}
/* read ionex body -----------------------------------------------------------*/
static bool readionexb(FILE *fp, const long double *lats, const long double *lons,
                       const long double *hgts, long double rb, long double nexp, nav_t *nav) {
  trace(3, "readionexb:\n");

  int type = 0;
  tec_t *p = NULL;
  char buff[1024];
  while (fgets(buff, sizeof(buff), fp)) {
    if (strlen(buff) < 60) continue;

    char *label = buff + 60;
    if (strstr(label, "START OF TEC MAP") == label) {
      p = addtec(lats, lons, hgts, rb, nav);
      if (p) type = 1;
    } else if (strstr(label, "END OF TEC MAP") == label) {
      type = 0;
      p = NULL;
    } else if (strstr(label, "START OF RMS MAP") == label) {
      type = 2;
      p = NULL;
    } else if (strstr(label, "END OF RMS MAP") == label) {
      type = 0;
      p = NULL;
    } else if (strstr(label, "EPOCH OF CURRENT MAP") == label) {
      gtime_t time = {0};
      if (str2time(buff, 0, 36, &time)) {
        trace(2, "ionex epoch invalid: %-36.36s\n", buff);
        continue;
      }
      if (type == 2) {
        for (int i = nav->nt - 1; i >= 0; i--) {
          if (fabsl(timediff(time, nav->tec[i].time)) >= 1.0L) continue;
          p = nav->tec + i;
          break;
        }
      } else if (p)
        p->time = time;
    } else if (strstr(label, "LAT/LON1/LON2/DLON/H") == label && p) {
      long double lat = str2num(buff, 2, 6);
      long double lon[3];
      lon[0] = str2num(buff, 8, 6);
      lon[1] = str2num(buff, 14, 6);
      lon[2] = str2num(buff, 20, 6);
      long double hgt = str2num(buff, 26, 6);

      int i = getindex(lat, p->lats);
      int k = getindex(hgt, p->hgts);
      int n = nitem(lon);

      for (int m = 0; m < n; m++) {
        if (m % 16 == 0 && !fgets(buff, sizeof(buff), fp)) break;

        int j = getindex(lon[0] + lon[2] * m, p->lons);
        int index = dataindex(i, j, k, p->ndata);
        if (index < 0) continue;

        long double x = str2num(buff, m % 16 * 5, 5);
        if (x == 9999.0L) continue;

        if (type == 1)
          p->data[index] = x * powl(10.0L, nexp);
        else
          p->rms[index] = x * powl(10.0L, nexp);
      }
    }
  }
  return true;
}
/* combine tec grid data -----------------------------------------------------*/
static void combtec(nav_t *nav) {
  trace(3, "combtec : nav->nt=%d\n", nav->nt);

  for (int i = 0; i < nav->nt - 1; i++) {
    for (int j = i + 1; j < nav->nt; j++) {
      if (timediff(nav->tec[j].time, nav->tec[i].time) < 0.0L) {
        tec_t tmp = nav->tec[i];
        nav->tec[i] = nav->tec[j];
        nav->tec[j] = tmp;
      }
    }
  }
  int n = 0;
  for (int i = 0; i < nav->nt; i++) {
    if (i > 0 && timediff(nav->tec[i].time, nav->tec[n - 1].time) == 0.0L) {
      free(nav->tec[n - 1].data);
      free(nav->tec[n - 1].rms);
      nav->tec[n - 1] = nav->tec[i];
      continue;
    }
    nav->tec[n++] = nav->tec[i];
  }
  nav->nt = n;

  trace(4, "combtec : nav->nt=%d\n", nav->nt);
}
/* read ionex tec grid file ----------------------------------------------------
 * read ionex ionospheric tec grid file
 * args   : char   *file       I   ionex tec grid file
 *                                 (wind-card * is expanded)
 *          nav_t  *nav        IO  navigation data
 *                                 nav->nt, nav->ntmax and nav->tec are modified
 *          int    opt         I   read option (1: no clear of tec data,0:clear)
 * return : none
 * notes  : see ref [1]
 *-----------------------------------------------------------------------------*/
extern void readtec(const char *file, nav_t *nav, int opt) {
  long double dcb[MAXSAT] = {0}, rms[MAXSAT] = {0};

  trace(3, "readtec : file=%s\n", file);

  /* clear of tec grid data option */
  if (!opt) {
    free(nav->tec);
    nav->tec = NULL;
    nav->nt = nav->ntmax = 0;
  }
  char *efiles[MAXEXFILE];
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(efiles[i] = (char *)malloc(FNSIZE))) {
      for (i--; i >= 0; i--) free(efiles[i]);
      return;
    }
  }
  /* expand wild card in file path */
  int n = expath(file, efiles, FNSIZE, MAXEXFILE);

  for (int i = 0; i < n; i++) {
    FILE *fp = fopen(efiles[i], "r");
    if (!fp) {
      trace(2, "ionex file open error %s\n", efiles[i]);
      continue;
    }
    /* read ionex header */
    long double lats[3] = {0}, lons[3] = {0}, hgts[3] = {0}, rb = 0.0L, nexp = -1.0L;
    if (readionexh(fp, lats, lons, hgts, &rb, &nexp, dcb, rms) <= 0.0L) {
      trace(2, "ionex file format error %s\n", efiles[i]);
      continue;
    }
    /* read ionex body */
    readionexb(fp, lats, lons, hgts, rb, nexp, nav);

    fclose(fp);
  }
  for (int i = 0; i < MAXEXFILE; i++) free(efiles[i]);

  /* combine tec grid data */
  if (nav->nt > 0) combtec(nav);

  /* P1-P2 dcb (not used)*/
  /* for (int i=0;i<MAXSAT;i++) { */
  /*    nav->cbias[i][0]=CLIGHT*dcb[i]*1E-9; */ /* ns->m */
                                                /* } */
}
/* interpolate tec grid data -------------------------------------------------*/
static bool interptec(const tec_t *tec, int k, const long double *posp, long double *value,
                      long double *rms) {
  long double dlat, dlon, a, b, d[4] = {0}, r[4] = {0};
  int i, j;

  trace(3, "interptec: k=%d posp=%.2Lf %.2Lf\n", k, posp[0] * R2D, posp[1] * R2D);
  *value = *rms = 0.0L;

  if (tec->lats[2] == 0.0L || tec->lons[2] == 0.0L) return false;

  dlat = posp[0] * R2D - tec->lats[0];
  dlon = posp[1] * R2D - tec->lons[0];
  if (tec->lons[2] > 0.0L)
    dlon -= floorl(dlon / 360) * 360.0L; /*  0<=dlon<360 */
  else
    dlon += floorl(-dlon / 360) * 360.0L; /* -360<dlon<=0 */

  a = dlat / tec->lats[2];
  b = dlon / tec->lons[2];
  i = (int)floorl(a);
  a -= i;
  j = (int)floorl(b);
  b -= j;

  /* get gridded tec data */
  for (int n = 0; n < 4; n++) {
    int index = dataindex(i + (n % 2), j + (n < 2 ? 0 : 1), k, tec->ndata);
    if (index < 0) continue;
    d[n] = tec->data[index];
    r[n] = tec->rms[index];
  }
  if (d[0] > 0.0L && d[1] > 0.0L && d[2] > 0.0L && d[3] > 0.0L) {
    /* bilinear interpolation (inside of grid) */
    *value = (1.0L - a) * (1.0L - b) * d[0] + a * (1.0L - b) * d[1] + (1.0L - a) * b * d[2] +
             a * b * d[3];
    *rms = (1.0L - a) * (1.0L - b) * r[0] + a * (1.0L - b) * r[1] + (1.0L - a) * b * r[2] +
           a * b * r[3];
  }
  /* nearest-neighbour extrapolation (outside of grid) */
  else if (a <= 0.5L && b <= 0.5L && d[0] > 0.0L) {
    *value = d[0];
    *rms = r[0];
  } else if (a > 0.5L && b <= 0.5L && d[1] > 0.0L) {
    *value = d[1];
    *rms = r[1];
  } else if (a <= 0.5L && b > 0.5L && d[2] > 0.0L) {
    *value = d[2];
    *rms = r[2];
  } else if (a > 0.5L && b > 0.5L && d[3] > 0.0L) {
    *value = d[3];
    *rms = r[3];
  } else {
    i = 0;
    for (int n = 0; n < 4; n++)
      if (d[n] > 0.0L) {
        i++;
        *value += d[n];
        *rms += r[n];
      }
    if (i == 0) return false;
    *value /= i;
    *rms /= i;
  }
  return true;
}
/* ionosphere delay by tec grid data -----------------------------------------*/
static bool iondelay(gtime_t time, const tec_t *tec, const long double *pos,
                     const long double *azel, int opt, long double *delay, long double *var) {
  const long double fact = 40.30E16L / FREQL1 / FREQL1; /* tecu->L1 iono (m) */
  long double posp[3] = {0}, vtec, rms, rp;

  char tstr[40];
  trace(3, "iondelay: time=%s pos=%.1Lf %.1Lf azel=%.1Lf %.1Lf\n", time2str(time, tstr, 0),
        pos[0] * R2D, pos[1] * R2D, azel[0] * R2D, azel[1] * R2D);

  *delay = *var = 0.0L;

  for (int i = 0; i < tec->ndata[2]; i++) { /* for a layer */

    long double hion;
    hion = tec->hgts[0] + tec->hgts[2] * i;

    /* ionospheric pierce point position */
    long double fs;
    fs = ionppp(pos, azel, tec->rb, hion, posp);

    if (opt & 2) {
      /* modified single layer mapping function (M-SLM) ref [2] */
      rp = tec->rb / (tec->rb + hion) * sinl(0.9782L * (PI / 2.0L - azel[1]));
      fs = 1.0L / sqrtl(1.0L - rp * rp);
    }
    if (opt & 1) {
      /* earth rotation correction (sun-fixed coordinate) */
      posp[1] += 2.0L * PI * timediff(time, tec->time) / 86400.0L;
    }
    /* interpolate tec grid data */
    if (!interptec(tec, i, posp, &vtec, &rms)) return false;

    *delay += fact * fs * vtec;
    *var += fact * fact * fs * fs * rms * rms;
  }
  trace(4, "iondelay: delay=%7.2Lf std=%6.2Lf\n", *delay, sqrtl(*var));

  return true;
}
/* ionosphere model by tec grid data -------------------------------------------
 * compute ionospheric delay by tec grid data
 * args   : gtime_t time     I   time (gpst)
 *          nav_t  *nav      I   navigation data
 *          long double *pos      I   receiver position {lat,lon,h} (rad,m)
 *          long double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          int    opt       I   model option
 *                                bit0: 0:earth-fixed,1:sun-fixed
 *                                bit1: 0:single-layer,1:modified single-layer
 *          long double *delay    O   ionospheric delay (L1) (m)
 *          long double *var      O   ionospheric dealy (L1) variance (m^2)
 * return : status (true:ok,false:error)
 * notes  : before calling the function, read tec grid data by calling readtec()
 *          return ok with delay=0 and var=VAR_NOTEC if el<MIN_EL or h<MIN_HGT
 *-----------------------------------------------------------------------------*/
extern bool iontec(gtime_t time, const nav_t *nav, const long double *pos, const long double *azel,
                   int opt, long double *delay, long double *var) {
  char tstr[40];
  trace(3, "iontec  : time=%s pos=%.1Lf %.1Lf azel=%.1Lf %.1Lf\n", time2str(time, tstr, 0),
        pos[0] * R2D, pos[1] * R2D, azel[0] * R2D, azel[1] * R2D);

  if (azel[1] < MIN_EL || pos[2] < MIN_HGT) {
    *delay = 0.0L;
    *var = VAR_NOTEC;
    return true;
  }
  int i;
  for (i = 0; i < nav->nt; i++) {
    if (timediff(nav->tec[i].time, time) > 0.0L) break;
  }
  if (i == 0 || i >= nav->nt) {
    trace(2, "%s: tec grid out of period\n", time2str(time, tstr, 0));
    return false;
  }
  long double tt = timediff(nav->tec[i].time, nav->tec[i - 1].time);
  if (tt == 0.0L) {
    trace(2, "tec grid time interval error\n");
    return false;
  }
  /* ionospheric delay by tec grid data */
  int stat[2];
  long double dels[2], vars[2];
  stat[0] = iondelay(time, nav->tec + i - 1, pos, azel, opt, dels, vars);
  stat[1] = iondelay(time, nav->tec + i, pos, azel, opt, dels + 1, vars + 1);

  if (!stat[0] && !stat[1]) {
    trace(2, "%s: tec grid out of area pos=%6.2Lf %7.2Lf azel=%6.1Lf %5.1Lf\n",
          time2str(time, tstr, 0), pos[0] * R2D, pos[1] * R2D, azel[0] * R2D, azel[1] * R2D);
    return false;
  }
  if (stat[0] && stat[1]) { /* linear interpolation by time */
    long double a = timediff(time, nav->tec[i - 1].time) / tt;
    *delay = dels[0] * (1.0L - a) + dels[1] * a;
    *var = vars[0] * (1.0L - a) + vars[1] * a;
  } else if (stat[0]) { /* nearest-neighbour extrapolation by time */
    *delay = dels[0];
    *var = vars[0];
  } else {
    *delay = dels[1];
    *var = vars[1];
  }
  trace(3, "iontec  : delay=%5.2Lf std=%5.2Lf\n", *delay, sqrtl(*var));
  return true;
}
