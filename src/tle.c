/*------------------------------------------------------------------------------
 * tle.c: NORAD TLE (two line element) functions
 *
 *          Copyright (C) 2012-2020 by T.TAKASU, All rights reserved.
 *
 * references:
 *     [1] F.R.Hoots and R.L.Roehrich, Spacetrack report No.3, Models for
 *         propagation of NORAD element sets, December 1980
 *     [2] D.A.Vallado, P.Crawford, R.Hujsak and T.S.Kelso, Revisiting
 *         Spacetrack Report #3, AIAA 2006-6753, 2006
 *     [3] CelesTrak (http://www.celestrak.com)
 *
 * version : $Revision:$ $Date:$
 * history : 2012/11/01 1.0  new
 *           2013/01/25 1.1  fix bug on binary search
 *           2014/08/26 1.2  fix bug on tle_pos() to get tle by satid or desig
 *           2020/11/30 1.3  fix problem on duplicated names in a satellite
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* SGP4 model propagator by STR#3 (ref [1] sec.6,11) -------------------------*/

#define DE2RA 0.174532925E-1L
#define E6A 1.E-6L
#define PIO2 1.57079633L
#define QO 120.0L
#define SO 78.0L
#define TOTHRD 0.66666667L
#define TWOPI 6.2831853L
#define X3PIO2 4.71238898L
#define XJ2 1.082616E-3L
#define XJ3 -0.253881E-5L
#define XJ4 -1.65597E-6L
#define XKE 0.743669161E-1L
#define XKMPER 6378.135L
#define XMNPDA 1440.0L
#define AE 1.0L
#define CK2 5.413080E-4L                 /* = 0.5*XJ2*AE*AE */
#define CK4 0.62098875E-6L               /* = -0.375*XJ4*AE*AE*AE*AE */
#define QOMS2T 1.8802791590152706439E-9L /* = powl((QO-SO)*AE/XKMPER,4.0) */
#define S 1.0122292801892716288L         /* = AE*(1.0+SO/XKMPER) */

static void SGP4_STR3(long double tsince, const tled_t *data, long double *rs) {
  long double xnodeo = data->OMG * DE2RA;
  long double omegao = data->omg * DE2RA;
  long double xmo = data->M * DE2RA;
  long double xincl = data->inc * DE2RA;
  long double temp = TWOPI / XMNPDA / XMNPDA;
  long double xno = data->n * temp * XMNPDA;
  long double xndt2o = data->ndot * temp;
  long double xndd6o = data->nddot * temp / XMNPDA;
  long double bstar = data->bstar / AE;
  long double eo = data->ecc;
  /*
   * recover original mean motion (xnodp) and semimajor axis (aodp)
   * from input elements
   */
  long double a1 = powl(XKE / xno, TOTHRD);
  long double cosio = cosl(xincl);
  long double theta2 = cosio * cosio;
  long double x3thm1 = 3.0L * theta2 - 1.0L;
  long double eosq = eo * eo;
  long double betao2 = 1.0L - eosq;
  long double betao = sqrtl(betao2);
  long double del1 = 1.5L * CK2 * x3thm1 / (a1 * a1 * betao * betao2);
  long double ao = a1 * (1.0L - del1 * (0.5L * TOTHRD + del1 * (1.0L + 134.0L / 81.0L * del1)));
  long double delo = 1.5L * CK2 * x3thm1 / (ao * ao * betao * betao2);
  long double xnodp = xno / (1.0L + delo);
  long double aodp = ao / (1.0L - delo);
  /*
   * initialization
   * for perigee less than 220 kilometers, the isimp flag is set and
   * the equations are truncated to linear variation in sqrt a and
   * quadratic variation in mean anomaly. also, the c3 term, the
   * delta omega term, and the delta m term are dropped.
   */
  int isimp = 0;
  if ((aodp * (1.0L - eo) / AE) < (220.0L / XKMPER + AE)) isimp = 1;

  /* for perigee below 156 km, the values of s and qoms2t are altered */
  long double s4 = S;
  long double qoms24 = QOMS2T;
  long double perige = (aodp * (1.0L - eo) - AE) * XKMPER;
  if (perige < 156.0L) {
    s4 = perige - 78.0L;
    if (perige <= 98.0L) s4 = 20.0L;
    qoms24 = powl((120.0L - s4) * AE / XKMPER, 4.0L);
    s4 = s4 / XKMPER + AE;
  }
  long double pinvsq = 1.0L / (aodp * aodp * betao2 * betao2);
  long double tsi = 1.0L / (aodp - s4);
  long double eta = aodp * eo * tsi;
  long double etasq = eta * eta;
  long double eeta = eo * eta;
  long double psisq = fabsl(1.0L - etasq);
  long double coef = qoms24 * powl(tsi, 4.0L);
  long double coef1 = coef / powl(psisq, 3.5L);
  long double c2 = coef1 * xnodp *
                   (aodp * (1.0L + 1.5L * etasq + eeta * (4.0L + etasq)) +
                    0.75L * CK2 * tsi / psisq * x3thm1 * (8.0L + 3.0L * etasq * (8.0L + etasq)));
  long double c1 = bstar * c2;
  long double sinio = sinl(xincl);
  long double a3ovk2 = -XJ3 / CK2 * powl(AE, 3.0L);
  long double c3 = coef * tsi * a3ovk2 * xnodp * AE * sinio / eo;
  long double x1mth2 = 1.0L - theta2;
  long double c4 =
      2.0L * xnodp * coef1 * aodp * betao2 *
      (eta * (2.0L + 0.5L * etasq) + eo * (0.5L + 2.0L * etasq) -
       2.0L * CK2 * tsi / (aodp * psisq) *
           (-3.0L * x3thm1 * (1.0L - 2.0L * eeta + etasq * (1.5L - 0.5L * eeta)) +
            0.75L * x1mth2 * (2.0L * etasq - eeta * (1.0L + etasq)) * cosl(2.0L * omegao)));
  long double c5 = 2.0L * coef1 * aodp * betao2 * (1.0L + 2.75L * (etasq + eeta) + eeta * etasq);
  long double theta4 = theta2 * theta2;
  long double temp1 = 3.0L * CK2 * pinvsq * xnodp;
  long double temp2 = temp1 * CK2 * pinvsq;
  long double temp3 = 1.25L * CK4 * pinvsq * pinvsq * xnodp;
  long double xmdot = xnodp + 0.5L * temp1 * betao * x3thm1 +
                      0.0625L * temp2 * betao * (13.0L - 78.0L * theta2 + 137.0L * theta4);
  long double x1m5th = 1.0L - 5.0L * theta2;
  long double omgdot = -0.5L * temp1 * x1m5th +
                       0.0625L * temp2 * (7.0L - 114.0L * theta2 + 395.0L * theta4) +
                       temp3 * (3.0L - 36.0L * theta2 + 49.0L * theta4);
  long double xhdot1 = -temp1 * cosio;
  long double xnodot =
      xhdot1 +
      (0.5L * temp2 * (4.0L - 19.0L * theta2) + 2.0L * temp3 * (3.0L - 7.0L * theta2)) * cosio;
  long double omgcof = bstar * c3 * cosl(omegao);
  long double xmcof = -TOTHRD * coef * bstar * AE / eeta;
  long double xnodcf = 3.5L * betao2 * xhdot1 * c1;
  long double t2cof = 1.5L * c1;
  long double xlcof = 0.125L * a3ovk2 * sinio * (3.0L + 5.0L * cosio) / (1.0L + cosio);
  long double aycof = 0.25L * a3ovk2 * sinio;
  long double x7thm1 = 7.0L * theta2 - 1.0L;

  long double d2, d3, d4, t3cof, t4cof, t5cof;
  if (isimp != 1) {
    long double c1sq = c1 * c1;
    d2 = 4.0L * aodp * tsi * c1sq;
    temp = d2 * tsi * c1 / 3.0L;
    d3 = (17.0L * aodp + s4) * temp;
    d4 = 0.5L * temp * aodp * tsi * (221.0L * aodp + 31.0L * s4) * c1;
    t3cof = d2 + 2.0L * c1sq;
    t4cof = 0.25L * (3.0L * d3 + c1 * (12.0L * d2 + 10.0L * c1sq));
    t5cof =
        0.2L * (3.0L * d4 + 12.0L * c1 * d3 + 6.0L * d2 * d2 + 15.0L * c1sq * (2.0L * d2 + c1sq));
  } else {
    d2 = d3 = d4 = t3cof = t4cof = t5cof = 0.0L;
  }
  /* update for secular gravity and atmospheric drag */
  long double xmdf = xmo + xmdot * tsince;
  long double omgadf = omegao + omgdot * tsince;
  long double xnoddf = xnodeo + xnodot * tsince;
  long double omega = omgadf;
  long double xmp = xmdf;
  long double tsq = tsince * tsince;
  long double xnode = xnoddf + xnodcf * tsq;
  long double tempa = 1.0L - c1 * tsince;
  long double tempe = bstar * c4 * tsince;
  long double templ = t2cof * tsq;
  if (isimp == 1) {
    long double delomg = omgcof * tsince;
    long double delmo = powl(1.0L + eta * cosl(xmo), 3.0L);
    long double delm = xmcof * (powl(1.0L + eta * cosl(xmdf), 3.0L) - delmo);
    temp = delomg + delm;
    xmp = xmdf + temp;
    omega = omgadf - temp;
    long double tcube = tsq * tsince;
    long double tfour = tsince * tcube;
    tempa = tempa - d2 * tsq - d3 * tcube - d4 * tfour;
    long double sinmo = sinl(xmo);
    tempe = tempe + bstar * c5 * (sinl(xmp) - sinmo);
    templ = templ + t3cof * tcube + tfour * (t4cof + tsince * t5cof);
  }
  long double a = aodp * powl(tempa, 2.0);
  long double e = eo - tempe;
  long double xl = xmp + omega + xnode + xnodp * templ;
  long double beta = sqrtl(1.0L - e * e);
  long double xn = XKE / powl(a, 1.5L);

  /* long period periodics */
  long double axn = e * cosl(omega);
  temp = 1.0L / (a * beta * beta);
  long double xll = temp * xlcof * axn;
  long double aynl = temp * aycof;
  long double xlt = xl + xll;
  long double ayn = e * sinl(omega) + aynl;

  /* solve keplers equation */
  long double capu = fmodl(xlt - xnode, TWOPI);
  temp2 = capu;
  long double temp4, temp5, temp6, sinepw, cosepw;
  for (int i = 0; i < 10; i++) {
    sinepw = sinl(temp2);
    cosepw = cosl(temp2);
    temp3 = axn * sinepw;
    temp4 = ayn * cosepw;
    temp5 = axn * cosepw;
    temp6 = ayn * sinepw;
    long double epw = (capu - temp4 + temp3 - temp2) / (1.0L - temp5 - temp6) + temp2;
    if (fabsl(epw - temp2) <= E6A) break;
    temp2 = epw;
  }
  /* short period preliminary quantities */
  long double ecose = temp5 + temp6;
  long double esine = temp3 - temp4;
  long double elsq = axn * axn + ayn * ayn;
  temp = 1.0L - elsq;
  long double pl = a * temp;
  long double r = a * (1.0 - ecose);
  temp1 = 1.0L / r;
  long double rdot = XKE * sqrtl(a) * esine * temp1;
  long double rfdot = XKE * sqrtl(pl) * temp1;
  temp2 = a * temp1;
  long double betal = sqrtl(temp);
  temp3 = 1.0L / (1.0L + betal);
  long double cosu = temp2 * (cosepw - axn + ayn * esine * temp3);
  long double sinu = temp2 * (sinepw - ayn - axn * esine * temp3);
  long double u = atan2l(sinu, cosu);
  long double sin2u = 2.0L * sinu * cosu;
  long double cos2u = 2.0L * cosu * cosu - 1.0;
  temp = 1.0L / pl;
  temp1 = CK2 * temp;
  temp2 = temp1 * temp;

  /* update for short periodics */
  long double rk = r * (1.0L - 1.5L * temp2 * betal * x3thm1) + 0.5L * temp1 * x1mth2 * cos2u;
  long double uk = u - 0.25L * temp2 * x7thm1 * sin2u;
  long double xnodek = xnode + 1.5L * temp2 * cosio * sin2u;
  long double xinck = xincl + 1.5L * temp2 * cosio * sinio * cos2u;
  long double rdotk = rdot - xn * temp1 * x1mth2 * sin2u;
  long double rfdotk = rfdot + xn * temp1 * (x1mth2 * cos2u + 1.5L * x3thm1);

  /* orientation vectors */
  long double sinuk = sinl(uk);
  long double cosuk = cosl(uk);
  long double sinik = sinl(xinck);
  long double cosik = cosl(xinck);
  long double sinnok = sinl(xnodek);
  long double cosnok = cosl(xnodek);
  long double xmx = -sinnok * cosik;
  long double xmy = cosnok * cosik;
  long double ux = xmx * sinuk + cosnok * cosuk;
  long double uy = xmy * sinuk + sinnok * cosuk;
  long double uz = sinik * sinuk;
  long double vx = xmx * cosuk - cosnok * sinuk;
  long double vy = xmy * cosuk - sinnok * sinuk;
  long double vz = sinik * cosuk;

  /* position and velocity */
  long double x = rk * ux;
  long double y = rk * uy;
  long double z = rk * uz;
  long double xdot = rdotk * ux + rfdotk * vx;
  long double ydot = rdotk * uy + rfdotk * vy;
  long double zdot = rdotk * uz + rfdotk * vz;

  rs[0] = x * XKMPER / AE * 1E3L; /* (m) */
  rs[1] = y * XKMPER / AE * 1E3L;
  rs[2] = z * XKMPER / AE * 1E3L;
  rs[3] = xdot * XKMPER / AE * XMNPDA / 86400.0L * 1E3L; /* (m/s) */
  rs[4] = ydot * XKMPER / AE * XMNPDA / 86400.0L * 1E3L;
  rs[5] = zdot * XKMPER / AE * XMNPDA / 86400.0L * 1E3L;
}
/* drop spaces at string tail ------------------------------------------------*/
static void chop(char *buff) {
  for (int i = strlen(buff) - 1; i >= 0; i--) {
    if (buff[i] == ' ' || buff[i] == '\r' || buff[i] == '\n')
      buff[i] = '\0';
    else
      break;
  }
}
/* test TLE line checksum ----------------------------------------------------*/
static bool checksum(const char *buff) {
  if (strlen(buff) < 69) return false;

  int cs = 0;
  for (int i = 0; i < 68; i++) {
    if ('0' <= buff[i] && buff[i] <= '9')
      cs += (int)(buff[i] - '0');
    else if (buff[i] == '-')
      cs += 1;
  }
  return (int)(buff[68] - '0') == cs % 10;
}
/* decode TLE line 1 ---------------------------------------------------------*/
static bool decode_line1(const char *buff, tled_t *data) {
  rtkesubstrcpy(data->satno, sizeof(data->satno), buff, 2, 7); /* satellite number */
  chop(data->satno);

  data->satclass = buff[7];                                     /* satellite classification */
  rtkesubstrcpy(data->desig, sizeof(data->desig), buff, 9, 17); /* international designator */
  chop(data->desig);

  long double year = str2num(buff, 18, 2);  /* epoch year */
  long double doy = str2num(buff, 20, 12);  /* epoch day of year */
  data->ndot = str2num(buff, 33, 10);       /* 1st time derivative of n */
  long double nddot = str2num(buff, 44, 6); /* 2nd time derivative of n */
  long double exp1 = str2num(buff, 50, 2);
  long double bstar = str2num(buff, 53, 6); /* Bstar drag term */
  long double exp2 = str2num(buff, 59, 2);
  data->etype = (int)str2num(buff, 62, 1); /* ephemeris type */
  data->eleno = (int)str2num(buff, 64, 4); /* ephemeris number */
  data->nddot = nddot * 1E-5L * powl(10.0, exp1);
  data->bstar = bstar * 1E-5L * powl(10.0, exp2);

  long double ep[6] = {2000, 1, 1};
  ep[0] = year + (year < 57.0L ? 2000.0L : 1900.0L);
  data->epoch = timeadd(epoch2time(ep), (doy - 1.0L) * 86400.0L);

  data->inc = data->OMG = data->ecc = data->omg = data->M = data->n = 0.0L;
  data->rev = 0;
  return true;
}
/* decode TLE line 2 ---------------------------------------------------------*/
static bool decode_line2(const char *buff, tled_t *data) {
  char satno[16];
  rtkesubstrcpy(satno, sizeof(satno), buff, 2, 7); /* satellite number */
  chop(satno);

  data->inc = str2num(buff, 8, 8);         /* inclination (deg) */
  data->OMG = str2num(buff, 17, 8);        /* RAAN (deg) */
  data->ecc = str2num(buff, 26, 7) * 1E-7; /* eccentricity */
  data->omg = str2num(buff, 34, 8);        /* argument of perigee (deg) */
  data->M = str2num(buff, 43, 8);          /* mean anomaly (deg) */
  data->n = str2num(buff, 52, 11);         /* mean motion (rev/day) */
  data->rev = (int)str2num(buff, 63, 5);   /* revolution number */

  if (strcmp(satno, data->satno)) {
    trace(2, "tle satno mismatch: %s %s\n", data->satno, satno);
    return false;
  }
  if (data->n <= 0.0L || data->ecc < 0.0L) {
    trace(2, "tle data error: %s\n", satno);
    return false;
  }
  return true;
}
/* add TLE data --------------------------------------------------------------*/
static bool add_data(tle_t *tle, const tled_t *data) {
  if (tle->n >= tle->nmax) {
    tle->nmax = tle->nmax <= 0 ? 1024 : tle->nmax * 2;

    tled_t *tle_data = (tled_t *)realloc(tle->data, sizeof(tled_t) * tle->nmax);
    if (!tle_data) {
      trace(1, "tle malloc error\n");
      free(tle->data);
      tle->data = NULL;
      tle->n = tle->nmax = 0;
      return false;
    }
    tle->data = tle_data;
  }
  tle->data[tle->n++] = *data;
  return true;
}
/* compare TLE data by satellite name ----------------------------------------*/
static int cmp_tle_data(const void *p1, const void *p2) {
  const tled_t *q1 = (const tled_t *)p1, *q2 = (const tled_t *)p2;
  return strcmp(q1->name, q2->name);
}
/* read TLE file ---------------------------------------------------------------
 * read NORAD TLE (two line element) data file (ref [2],[3])
 * args   : char   *file     I   NORAD TLE data file
 *          tle_t  *tle      O   TLE data
 * return : status (true:ok,false:error)
 * notes  : before calling the function, the TLE data should be initialized.
 *          the file should be in a two line (only TLE) or three line (satellite
 *          name + TLE) format.
 *          the characters after # in a line are treated as comments.
 *-----------------------------------------------------------------------------*/
extern bool tle_read(const char *file, tle_t *tle) {
  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "tle file open error: %s\n", file);
    return false;
  }
  tled_t data = {{0}};
  int line = 0;
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    /* delete comments */
    char *p = strchr(buff, '#');
    if (p) *p = '\0';
    chop(buff);

    if (buff[0] == '1' && checksum(buff)) {
      /* decode TLE line 1 */
      if (decode_line1(buff, &data)) line = 1;
    } else if (line == 1 && buff[0] == '2' && checksum(buff)) {
      /* decode TLE line 2 */
      if (!decode_line2(buff, &data)) continue;

      /* add TLE data */
      if (!add_data(tle, &data)) {
        fclose(fp);
        return false;
      }
      data.name[0] = '\0';
      data.alias[0] = '\0';
    } else if (buff[0]) {
      /* satellite name in three line format */
      rtkstrcpy(data.alias, sizeof(data.alias), buff);

      /* omit words in parentheses */
      p = strchr(data.alias, '(');
      if (p) *p = '\0';
      chop(data.alias);
      line = 0;
    }
  }
  fclose(fp);

  /* sort tle data by satellite name */
  if (tle->n > 0) qsort(tle->data, tle->n, sizeof(tled_t), cmp_tle_data);
  return true;
}
/* read TLE satellite name file ------------------------------------------------
 * read TLE satellite name file
 * args   : char   *file     I   TLE satellite name file
 *          tle_t  *tle      IO  TLE data
 * return : status (true:ok,false:error)
 * notes  : before calling the function, call tle_read() to read tle table
 *          the TLE satellite name file contains the following record as a text
 *          line. strings after # are treated as comments.
 *
 *          name satno [desig [# comment]]
 *
 *            name : satellite name
 *            satno: satellite catalog number
 *            desig: international designator (optional)
 *-----------------------------------------------------------------------------*/
extern bool tle_name_read(const char *file, tle_t *tle) {
  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(2, "tle satellite name file open error: %s\n", file);
    return false;
  }

  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    char *p = strchr(buff, '#');
    if (p) *p = '\0';

    char name[256], satno[256], desig[256];
    desig[0] = '\0';
    if (sscanf(buff, "%255s %255s %255s", name, satno, desig) < 2) continue;
    satno[5] = '\0';

    int i;
    for (i = 0; i < tle->n; i++) {
      if (!strcmp(tle->data[i].satno, satno) || !strcmp(tle->data[i].desig, desig)) break;
    }
    if (i >= tle->n) {
      trace(4, "no tle data: satno=%s desig=%s\n", satno, desig);
      continue;
    }
    if (!*tle->data[i].name) {
      rtkstrcpy(tle->data[i].name, sizeof(tle->data[i].name), name);
    } else {
      tled_t data = tle->data[i];
      rtkstrcpy(data.name, sizeof(data.name), name);
      if (!add_data(tle, &data)) {
        break;
      }
    }
  }
  fclose(fp);

  /* sort tle data by satellite name */
  if (tle->n > 0) qsort(tle->data, tle->n, sizeof(tled_t), cmp_tle_data);
  return true;
}
/* satellite position and velocity with TLE data -------------------------------
 * compute satellite position and velocity in ECEF with TLE data
 * args   : gtime_t time     I   time (GPST)
 *          char   *name     I   satellite name           ("": not specified)
 *          char   *satno    I   satellite catalog number ("": not specified)
 *          char   *desig    I   international designator ("": not specified)
 *          tle_t  *tle      I   TLE data
 *          erp_t  *erp      I   EOP data (NULL: not used)
 *          long double *rs       O   sat position/velocity {x,y,z,vx,vy,vz} (m,m/s)
 * return : status (true:ok,false:error)
 * notes  : the coordinates of the position and velocity are ECEF (ITRF)
 *          if erp == NULL, polar motion and ut1-utc are neglected
 *-----------------------------------------------------------------------------*/
extern bool tle_pos(gtime_t time, const char *name, const char *satno, const char *desig,
                    const tle_t *tle, const erp_t *erp, long double *rs) {
  /* binary search by satellite name or alias if name is empty */
  int stat = 1, i = 0;
  if (*name) {
    for (int j = 0, k = tle->n - 1; j <= k;) {
      i = (j + k) / 2;
      stat = strcmp(name, tle->data[i].name);
      if (!stat || ((tle->data[i].name[0] == '\0') && !(stat = strcmp(name, tle->data[i].alias))))
        break;
      if (stat < 0)
        k = i - 1;
      else
        j = i + 1;
    }
  }
  /* serial search by catalog no or international designator */
  if (stat && (*satno || *desig)) {
    for (i = 0; i < tle->n; i++) {
      if (!strcmp(tle->data[i].satno, satno) || !strcmp(tle->data[i].desig, desig)) break;
    }
    if (i < tle->n) stat = 0;
  }
  if (stat) {
    trace(4, "no tle data: name=%s satno=%s desig=%s\n", name, satno, desig);
    return false;
  }
  gtime_t tutc = gpst2utc(time);

  /* time since epoch (min) */
  long double tsince = timediff(tutc, tle->data[i].epoch) / 60.0L;

  /* SGP4 model propagator by STR#3 */
  long double rs_tle[6];
  SGP4_STR3(tsince, tle->data + i, rs_tle);

  /* erp values */
  long double erpv[5] = {0};
  if (erp) geterp(erp, time, erpv);

  /* GMST (rad) */
  long double gmst = utc2gmst(tutc, erpv[2]);

  /* TEME (true equator, mean eqinox) -> ECEF (ref [2] IID, Appendix C) */
  long double R1[9] = {0};
  R1[0] = 1.0L;
  R1[4] = R1[8] = cosl(-erpv[1]);
  R1[7] = sinl(-erpv[1]);
  R1[5] = -R1[7];
  long double R2[9] = {0};
  R2[4] = 1.0L;
  R2[0] = R2[8] = cosl(-erpv[0]);
  R2[2] = sinl(-erpv[0]);
  R2[6] = -R2[2];
  long double R3[9] = {0};
  R3[8] = 1.0;
  R3[0] = R3[4] = cosl(gmst);
  R3[3] = sinl(gmst);
  R3[1] = -R3[3];
  long double rs_pef[6];
  matmul("NN", 3, 1, 3, R3, rs_tle, rs_pef);
  matmul("NN", 3, 1, 3, R3, rs_tle + 3, rs_pef + 3);
  rs_pef[3] += OMGE * rs_pef[1];
  rs_pef[4] -= OMGE * rs_pef[0];
  long double W[9];
  matmul("NN", 3, 3, 3, R1, R2, W);
  matmul("NN", 3, 1, 3, W, rs_pef, rs);
  matmul("NN", 3, 1, 3, W, rs_pef + 3, rs + 3);
  return true;
}
