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

#define DE2RA 0.174532925E-1
#define E6A 1.E-6
#define PIO2 1.57079633
#define QO 120.0
#define SO 78.0
#define TOTHRD 0.66666667
#define TWOPI 6.2831853
#define X3PIO2 4.71238898
#define XJ2 1.082616E-3
#define XJ3 -0.253881E-5
#define XJ4 -1.65597E-6
#define XKE 0.743669161E-1
#define XKMPER 6378.135
#define XMNPDA 1440.0
#define AE 1.0
#define CK2 5.413080E-4      /* = 0.5*XJ2*AE*AE */
#define CK4 0.62098875E-6    /* = -0.375*XJ4*AE*AE*AE*AE */
#define QOMS2T 1.88027916E-9 /* = pow((QO-SO)*AE/XKMPER,4.0) */
#define S 1.01222928         /* = AE*(1.0+SO/XKMPER) */

static void SGP4_STR3(double tsince, const tled_t *data, double *rs) {
  double xnodeo = data->OMG * DE2RA;
  double omegao = data->omg * DE2RA;
  double xmo = data->M * DE2RA;
  double xincl = data->inc * DE2RA;
  double temp = TWOPI / XMNPDA / XMNPDA;
  double xno = data->n * temp * XMNPDA;
  double xndt2o = data->ndot * temp;
  double xndd6o = data->nddot * temp / XMNPDA;
  double bstar = data->bstar / AE;
  double eo = data->ecc;
  /*
   * recover original mean motion (xnodp) and semimajor axis (aodp)
   * from input elements
   */
  double a1 = pow(XKE / xno, TOTHRD);
  double cosio = cos(xincl);
  double theta2 = cosio * cosio;
  double x3thm1 = 3.0 * theta2 - 1.0;
  double eosq = eo * eo;
  double betao2 = 1.0 - eosq;
  double betao = sqrt(betao2);
  double del1 = 1.5 * CK2 * x3thm1 / (a1 * a1 * betao * betao2);
  double ao = a1 * (1.0 - del1 * (0.5 * TOTHRD + del1 * (1.0 + 134.0 / 81.0 * del1)));
  double delo = 1.5 * CK2 * x3thm1 / (ao * ao * betao * betao2);
  double xnodp = xno / (1.0 + delo);
  double aodp = ao / (1.0 - delo);
  /*
   * initialization
   * for perigee less than 220 kilometers, the isimp flag is set and
   * the equations are truncated to linear variation in sqrt a and
   * quadratic variation in mean anomaly. also, the c3 term, the
   * delta omega term, and the delta m term are dropped.
   */
  int isimp = 0;
  if ((aodp * (1.0 - eo) / AE) < (220.0 / XKMPER + AE)) isimp = 1;

  /* for perigee below 156 km, the values of s and qoms2t are altered */
  double s4 = S;
  double qoms24 = QOMS2T;
  double perige = (aodp * (1.0 - eo) - AE) * XKMPER;
  if (perige < 156.0) {
    s4 = perige - 78.0;
    if (perige <= 98.0) s4 = 20.0;
    qoms24 = pow((120.0 - s4) * AE / XKMPER, 4.0);
    s4 = s4 / XKMPER + AE;
  }
  double pinvsq = 1.0 / (aodp * aodp * betao2 * betao2);
  double tsi = 1.0 / (aodp - s4);
  double eta = aodp * eo * tsi;
  double etasq = eta * eta;
  double eeta = eo * eta;
  double psisq = fabs(1.0 - etasq);
  double coef = qoms24 * pow(tsi, 4.0);
  double coef1 = coef / pow(psisq, 3.5);
  double c2 = coef1 * xnodp *
              (aodp * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) +
               0.75 * CK2 * tsi / psisq * x3thm1 * (8.0 + 3.0 * etasq * (8.0 + etasq)));
  double c1 = bstar * c2;
  double sinio = sin(xincl);
  double a3ovk2 = -XJ3 / CK2 * pow(AE, 3.0);
  double c3 = coef * tsi * a3ovk2 * xnodp * AE * sinio / eo;
  double x1mth2 = 1.0 - theta2;
  double c4 = 2.0 * xnodp * coef1 * aodp * betao2 *
              (eta * (2.0 + 0.5 * etasq) + eo * (0.5 + 2.0 * etasq) -
               2.0 * CK2 * tsi / (aodp * psisq) *
                   (-3.0 * x3thm1 * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta)) +
                    0.75 * x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq)) * cos(2.0 * omegao)));
  double c5 = 2.0 * coef1 * aodp * betao2 * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);
  double theta4 = theta2 * theta2;
  double temp1 = 3.0 * CK2 * pinvsq * xnodp;
  double temp2 = temp1 * CK2 * pinvsq;
  double temp3 = 1.25 * CK4 * pinvsq * pinvsq * xnodp;
  double xmdot = xnodp + 0.5 * temp1 * betao * x3thm1 +
                 0.0625 * temp2 * betao * (13.0 - 78.0 * theta2 + 137.0 * theta4);
  double x1m5th = 1.0 - 5.0 * theta2;
  double omgdot = -0.5 * temp1 * x1m5th + 0.0625 * temp2 * (7.0 - 114.0 * theta2 + 395.0 * theta4) +
                  temp3 * (3.0 - 36.0 * theta2 + 49.0 * theta4);
  double xhdot1 = -temp1 * cosio;
  double xnodot =
      xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * theta2) + 2.0 * temp3 * (3.0 - 7.0 * theta2)) * cosio;
  double omgcof = bstar * c3 * cos(omegao);
  double xmcof = -TOTHRD * coef * bstar * AE / eeta;
  double xnodcf = 3.5 * betao2 * xhdot1 * c1;
  double t2cof = 1.5 * c1;
  double xlcof = 0.125 * a3ovk2 * sinio * (3.0 + 5.0 * cosio) / (1.0 + cosio);
  double aycof = 0.25 * a3ovk2 * sinio;
  double x7thm1 = 7.0 * theta2 - 1.0;

  double d2, d3, d4, t3cof, t4cof, t5cof;
  if (isimp != 1) {
    double c1sq = c1 * c1;
    d2 = 4.0 * aodp * tsi * c1sq;
    temp = d2 * tsi * c1 / 3.0;
    d3 = (17.0 * aodp + s4) * temp;
    d4 = 0.5 * temp * aodp * tsi * (221.0 * aodp + 31.0 * s4) * c1;
    t3cof = d2 + 2.0 * c1sq;
    t4cof = 0.25 * (3.0 * d3 + c1 * (12.0 * d2 + 10.0 * c1sq));
    t5cof = 0.2 * (3.0 * d4 + 12.0 * c1 * d3 + 6.0 * d2 * d2 + 15.0 * c1sq * (2.0 * d2 + c1sq));
  } else {
    d2 = d3 = d4 = t3cof = t4cof = t5cof = 0.0;
  }
  /* update for secular gravity and atmospheric drag */
  double xmdf = xmo + xmdot * tsince;
  double omgadf = omegao + omgdot * tsince;
  double xnoddf = xnodeo + xnodot * tsince;
  double omega = omgadf;
  double xmp = xmdf;
  double tsq = tsince * tsince;
  double xnode = xnoddf + xnodcf * tsq;
  double tempa = 1.0 - c1 * tsince;
  double tempe = bstar * c4 * tsince;
  double templ = t2cof * tsq;
  if (isimp == 1) {
    double delomg = omgcof * tsince;
    double delmo = pow(1.0 + eta * cos(xmo), 3.0);
    double delm = xmcof * (pow(1.0 + eta * cos(xmdf), 3.0) - delmo);
    temp = delomg + delm;
    xmp = xmdf + temp;
    omega = omgadf - temp;
    double tcube = tsq * tsince;
    double tfour = tsince * tcube;
    tempa = tempa - d2 * tsq - d3 * tcube - d4 * tfour;
    double sinmo = sin(xmo);
    tempe = tempe + bstar * c5 * (sin(xmp) - sinmo);
    templ = templ + t3cof * tcube + tfour * (t4cof + tsince * t5cof);
  }
  double a = aodp * pow(tempa, 2.0);
  double e = eo - tempe;
  double xl = xmp + omega + xnode + xnodp * templ;
  double beta = sqrt(1.0 - e * e);
  double xn = XKE / pow(a, 1.5);

  /* long period periodics */
  double axn = e * cos(omega);
  temp = 1.0 / (a * beta * beta);
  double xll = temp * xlcof * axn;
  double aynl = temp * aycof;
  double xlt = xl + xll;
  double ayn = e * sin(omega) + aynl;

  /* solve keplers equation */
  double capu = fmod(xlt - xnode, TWOPI);
  temp2 = capu;
  double temp4, temp5, temp6, sinepw, cosepw;
  for (int i = 0; i < 10; i++) {
    sinepw = sin(temp2);
    cosepw = cos(temp2);
    temp3 = axn * sinepw;
    temp4 = ayn * cosepw;
    temp5 = axn * cosepw;
    temp6 = ayn * sinepw;
    double epw = (capu - temp4 + temp3 - temp2) / (1.0 - temp5 - temp6) + temp2;
    if (fabs(epw - temp2) <= E6A) break;
    temp2 = epw;
  }
  /* short period preliminary quantities */
  double ecose = temp5 + temp6;
  double esine = temp3 - temp4;
  double elsq = axn * axn + ayn * ayn;
  temp = 1.0 - elsq;
  double pl = a * temp;
  double r = a * (1.0 - ecose);
  temp1 = 1.0 / r;
  double rdot = XKE * sqrt(a) * esine * temp1;
  double rfdot = XKE * sqrt(pl) * temp1;
  temp2 = a * temp1;
  double betal = sqrt(temp);
  temp3 = 1.0 / (1.0 + betal);
  double cosu = temp2 * (cosepw - axn + ayn * esine * temp3);
  double sinu = temp2 * (sinepw - ayn - axn * esine * temp3);
  double u = atan2(sinu, cosu);
  double sin2u = 2.0 * sinu * cosu;
  double cos2u = 2.0 * cosu * cosu - 1.0;
  temp = 1.0 / pl;
  temp1 = CK2 * temp;
  temp2 = temp1 * temp;

  /* update for short periodics */
  double rk = r * (1.0 - 1.5 * temp2 * betal * x3thm1) + 0.5 * temp1 * x1mth2 * cos2u;
  double uk = u - 0.25 * temp2 * x7thm1 * sin2u;
  double xnodek = xnode + 1.5 * temp2 * cosio * sin2u;
  double xinck = xincl + 1.5 * temp2 * cosio * sinio * cos2u;
  double rdotk = rdot - xn * temp1 * x1mth2 * sin2u;
  double rfdotk = rfdot + xn * temp1 * (x1mth2 * cos2u + 1.5 * x3thm1);

  /* orientation vectors */
  double sinuk = sin(uk);
  double cosuk = cos(uk);
  double sinik = sin(xinck);
  double cosik = cos(xinck);
  double sinnok = sin(xnodek);
  double cosnok = cos(xnodek);
  double xmx = -sinnok * cosik;
  double xmy = cosnok * cosik;
  double ux = xmx * sinuk + cosnok * cosuk;
  double uy = xmy * sinuk + sinnok * cosuk;
  double uz = sinik * sinuk;
  double vx = xmx * cosuk - cosnok * sinuk;
  double vy = xmy * cosuk - sinnok * sinuk;
  double vz = sinik * cosuk;

  /* position and velocity */
  double x = rk * ux;
  double y = rk * uy;
  double z = rk * uz;
  double xdot = rdotk * ux + rfdotk * vx;
  double ydot = rdotk * uy + rfdotk * vy;
  double zdot = rdotk * uz + rfdotk * vz;

  rs[0] = x * XKMPER / AE * 1E3; /* (m) */
  rs[1] = y * XKMPER / AE * 1E3;
  rs[2] = z * XKMPER / AE * 1E3;
  rs[3] = xdot * XKMPER / AE * XMNPDA / 86400.0 * 1E3; /* (m/s) */
  rs[4] = ydot * XKMPER / AE * XMNPDA / 86400.0 * 1E3;
  rs[5] = zdot * XKMPER / AE * XMNPDA / 86400.0 * 1E3;
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

  double year = str2num(buff, 18, 2);  /* epoch year */
  double doy = str2num(buff, 20, 12);  /* epoch day of year */
  data->ndot = str2num(buff, 33, 10);  /* 1st time derivative of n */
  double nddot = str2num(buff, 44, 6); /* 2nd time derivative of n */
  double exp1 = str2num(buff, 50, 2);
  double bstar = str2num(buff, 53, 6); /* Bstar drag term */
  double exp2 = str2num(buff, 59, 2);
  data->etype = (int)str2num(buff, 62, 1); /* ephemeris type */
  data->eleno = (int)str2num(buff, 64, 4); /* ephemeris number */
  data->nddot = nddot * 1E-5 * pow(10.0, exp1);
  data->bstar = bstar * 1E-5 * pow(10.0, exp2);

  double ep[6] = {2000, 1, 1};
  ep[0] = year + (year < 57.0 ? 2000.0 : 1900.0);
  data->epoch = timeadd(epoch2time(ep), (doy - 1.0) * 86400.0);

  data->inc = data->OMG = data->ecc = data->omg = data->M = data->n = 0.0;
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
  if (data->n <= 0.0 || data->ecc < 0.0) {
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
 *          double *rs       O   sat position/velocity {x,y,z,vx,vy,vz} (m,m/s)
 * return : status (true:ok,false:error)
 * notes  : the coordinates of the position and velocity are ECEF (ITRF)
 *          if erp == NULL, polar motion and ut1-utc are neglected
 *-----------------------------------------------------------------------------*/
extern bool tle_pos(gtime_t time, const char *name, const char *satno, const char *desig,
                    const tle_t *tle, const erp_t *erp, double *rs) {
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
  double tsince = timediff(tutc, tle->data[i].epoch) / 60.0;

  /* SGP4 model propagator by STR#3 */
  double rs_tle[6];
  SGP4_STR3(tsince, tle->data + i, rs_tle);

  /* erp values */
  double erpv[5] = {0};
  if (erp) geterp(erp, time, erpv);

  /* GMST (rad) */
  double gmst = utc2gmst(tutc, erpv[2]);

  /* TEME (true equator, mean eqinox) -> ECEF (ref [2] IID, Appendix C) */
  double R1[9] = {0};
  R1[0] = 1.0;
  R1[4] = R1[8] = cos(-erpv[1]);
  R1[7] = sin(-erpv[1]);
  R1[5] = -R1[7];
  double R2[9] = {0};
  R2[4] = 1.0;
  R2[0] = R2[8] = cos(-erpv[0]);
  R2[2] = sin(-erpv[0]);
  R2[6] = -R2[2];
  double R3[9] = {0};
  R3[8] = 1.0;
  R3[0] = R3[4] = cos(gmst);
  R3[3] = sin(gmst);
  R3[1] = -R3[3];
  double rs_pef[6];
  matmul("NN", 3, 1, 3, R3, rs_tle, rs_pef);
  matmul("NN", 3, 1, 3, R3, rs_tle + 3, rs_pef + 3);
  rs_pef[3] += OMGE * rs_pef[1];
  rs_pef[4] -= OMGE * rs_pef[0];
  double W[9];
  matmul("NN", 3, 3, 3, R1, R2, W);
  matmul("NN", 3, 1, 3, W, rs_pef, rs);
  matmul("NN", 3, 1, 3, W, rs_pef + 3, rs + 3);
  return true;
}
