/*------------------------------------------------------------------------------
 * tides.c : tidal displacement corrections
 *
 *          Copyright (C) 2015-2017 by T.TAKASU, All rights reserved.
 *
 * options : -DIERS_MODEL use IERS tide model
 *
 * references :
 *     [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
 *     [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
 *         2003, November 2003
 *     [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
 *         Space Technology Library, 2004
 *     [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
 *         May 2009
 *     [5] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
 *         Conventions (2010), 2010
 *
 * version : $Revision:$ $Date:$
 * history : 2015/05/10 1.0  separated from ppp.c
 *           2015/06/11 1.1  fix bug on computing days in tide_oload() (#128)
 *           2017/04/11 1.2  fix bug on calling geterp() in timdedisp()
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define SQR(x) ((x) * (x))

#define GME 3.986004415E+14L /* earth gravitational constant */
#define GMS 1.327124E+20L    /* sun gravitational constant */
#define GMM 4.902801E+12L    /* moon gravitational constant */

/* function prototypes -------------------------------------------------------*/
#ifdef IERS_MODEL
extern int dehanttideinel_(long double *xsta, int *year, int *mon, int *day, long double *fhr,
                           long double *xsun, long double *xmon, long double *dxtide);
#endif

/* solar/lunar tides (ref [2] 7) ---------------------------------------------*/
#ifndef IERS_MODEL
static void tide_pl(const long double *eu, const long double *rp, long double GMp,
                    const long double *pos, long double *dr) {
  const long double H3 = 0.292L, L3 = 0.015L;

  trace(4, "tide_pl : pos=%.3Lf %.3Lf\n", pos[0] * R2D, pos[1] * R2D);

  long double r = norm(rp, 3);
  if (r <= 0.0L) {
    dr[0] = dr[1] = dr[2] = 0;
    return;
  }

  long double ep[3];
  for (int i = 0; i < 3; i++) ep[i] = rp[i] / r;

  long double K2 = GMp / GME * SQR(RE_WGS84) * SQR(RE_WGS84) / (r * r * r);
  long double K3 = K2 * RE_WGS84 / r;
  long double latp = asinl(ep[2]);
  long double lonp = atan2l(ep[1], ep[0]);
  long double cosp = cosl(latp);
  long double sinll = sinl(pos[0]);
  long double cosll = cosl(pos[0]);

  /* step1 in phase (degree 2) */
  long double p = (3.0L * sinll * sinll - 1.0L) / 2.0L;
  long double H2 = 0.6078L - 0.0006L * p;
  long double L2 = 0.0847L + 0.0002L * p;
  long double a = dot3(ep, eu);
  long double dp = K2 * 3.0L * L2 * a;
  long double du = K2 * (H2 * (1.5L * a * a - 0.5L) - 3.0L * L2 * a * a);

  /* step1 in phase (degree 3) */
  dp += K3 * L3 * (7.5L * a * a - 1.5L);
  du += K3 * (H3 * (2.5L * a * a * a - 1.5L * a) - L3 * (7.5L * a * a - 1.5L) * a);

  /* step1 out-of-phase (only radial) */
  du += 3.0L / 4.0L * 0.0025L * K2 * sinl(2.0L * latp) * sinl(2.0L * pos[0]) * sinl(pos[1] - lonp);
  du += 3.0L / 4.0L * 0.0022L * K2 * cosp * cosp * cosll * cosll * sinl(2.0L * (pos[1] - lonp));

  dr[0] = dp * ep[0] + du * eu[0];
  dr[1] = dp * ep[1] + du * eu[1];
  dr[2] = dp * ep[2] + du * eu[2];

  trace(5, "tide_pl : dr=%.3Lf %.3Lf %.3Lf\n", dr[0], dr[1], dr[2]);
}
/* displacement by solid earth tide (ref [2] 7) ------------------------------*/
static void tide_solid(const long double *rsun, const long double *rmoon, const long double *pos,
                       const long double *E, long double gmst, int opt, long double *dr) {
  trace(3, "tide_solid: pos=%.3Lf %.3Lf opt=%d\n", pos[0] * R2D, pos[1] * R2D, opt);

  /* step1: time domain */
  long double eu[3];
  eu[0] = E[2];
  eu[1] = E[5];
  eu[2] = E[8];
  long double dr1[3];
  tide_pl(eu, rsun, GMS, pos, dr1);
  long double dr2[3];
  tide_pl(eu, rmoon, GMM, pos, dr2);

  /* step2: frequency domain, only K1 radial */
  long double sin2l = sinl(2.0L * pos[0]);
  long double du = -0.012L * sin2l * sinl(gmst + pos[1]);

  dr[0] = dr1[0] + dr2[0] + du * E[2];
  dr[1] = dr1[1] + dr2[1] + du * E[5];
  dr[2] = dr1[2] + dr2[2] + du * E[8];

  /* eliminate permanent deformation */
  if (opt & 8) {
    long double sinll = sinl(pos[0]);
    du = 0.1196L * (1.5L * sinll * sinll - 0.5L);
    long double dn = 0.0247L * sin2l;
    dr[0] += du * E[2] + dn * E[1];
    dr[1] += du * E[5] + dn * E[4];
    dr[2] += du * E[8] + dn * E[7];
  }
  trace(5, "tide_solid: dr=%.3Lf %.3Lf %.3Lf\n", dr[0], dr[1], dr[2]);
}
#endif /* !IERS_MODEL */

/* displacement by ocean tide loading (ref [2] 7) ----------------------------*/
static void tide_oload(gtime_t tut, const long double *odisp, long double *denu) {
  const long double args[][5] = {
      {1.40519E-4L, 2.0L, -2.0L, 0.0L, 0.00L},  /* M2 */
      {1.45444E-4L, 0.0L, 0.0L, 0.0L, 0.00L},   /* S2 */
      {1.37880E-4L, 2.0L, -3.0L, 1.0L, 0.00L},  /* N2 */
      {1.45842E-4L, 2.0L, 0.0L, 0.0L, 0.00L},   /* K2 */
      {0.72921E-4L, 1.0L, 0.0L, 0.0L, 0.25L},   /* K1 */
      {0.67598E-4L, 1.0L, -2.0L, 0.0L, -0.25L}, /* O1 */
      {0.72523E-4L, -1.0L, 0.0L, 0.0L, -0.25L}, /* P1 */
      {0.64959E-4L, 1.0L, -3.0L, 1.0L, -0.25L}, /* Q1 */
      {0.53234E-5L, 0.0L, 2.0L, 0.0L, 0.00L},   /* Mf */
      {0.26392E-5L, 0.0L, 1.0L, -1.0L, 0.00L},  /* Mm */
      {0.03982E-5L, 2.0L, 0.0L, 0.0L, 0.00L}    /* Ssa */
  };
  const long double ep1975[] = {1975, 1, 1, 0, 0, 0};

  trace(3, "tide_oload:\n");

  /* angular argument: see subroutine arg.f for reference [1] */
  long double ep[6];
  time2epoch(tut, ep);
  long double fday = ep[3] * 3600.0L + ep[4] * 60.0L + ep[5];
  ep[3] = ep[4] = ep[5] = 0.0L;
  long double days = timediff(epoch2time(ep), epoch2time(ep1975)) / 86400.0L + 1.0L;
  long double t = (27392.500528L + 1.000000035L * days) / 36525.0L;
  long double t2 = t * t;
  long double t3 = t2 * t;

  long double a[5];
  a[0] = fday;
  a[1] = (279.69668L + 36000.768930485L * t + 3.03E-4L * t2) * D2R;                  /* H0 */
  a[2] = (270.434358L + 481267.88314137L * t - 0.001133L * t2 + 1.9E-6L * t3) * D2R; /* S0 */
  a[3] = (334.329653L + 4069.0340329577L * t - 0.010325L * t2 - 1.2E-5L * t3) * D2R; /* P0 */
  a[4] = 2.0L * PI;

  /* displacements by 11 constituents */
  long double dp[3] = {0};
  for (int i = 0; i < 11; i++) {
    long double ang = 0.0L;
    for (int j = 0; j < 5; j++) ang += a[j] * args[i][j];
    for (int j = 0; j < 3; j++) dp[j] += odisp[j + i * 6] * cosl(ang - odisp[j + 3 + i * 6] * D2R);
  }
  denu[0] = -dp[1];
  denu[1] = -dp[2];
  denu[2] = dp[0];

  trace(5, "tide_oload: denu=%.3Lf %.3Lf %.3Lf\n", denu[0], denu[1], denu[2]);
}
/* iers mean pole (ref [7] eq.7.25) ------------------------------------------*/
static void iers_mean_pole(gtime_t tut, long double *xp_bar, long double *yp_bar) {
  const long double ep2000[] = {2000, 1, 1, 0, 0, 0};

  long double y = timediff(tut, epoch2time(ep2000)) / 86400.0L / 365.25L;

  if (y < 3653.0L / 365.25L) { /* until 2010.0 */
    long double y2 = y * y;
    long double y3 = y2 * y;
    *xp_bar = 55.974L + 1.8243L * y + 0.18413L * y2 + 0.007024L * y3; /* (mas) */
    *yp_bar = 346.346L + 1.7896L * y - 0.10729L * y2 - 0.000908L * y3;
  } else {                           /* after 2010.0 */
    *xp_bar = 23.513L + 7.6141L * y; /* (mas) */
    *yp_bar = 358.891L - 0.6287L * y;
  }
}
/* displacement by pole tide (ref [7] eq.7.26) --------------------------------*/
static void tide_pole(gtime_t tut, const long double *pos, const long double *erpv,
                      long double *denu) {
  trace(3, "tide_pole: pos=%.3Lf %.3Lf\n", pos[0] * R2D, pos[1] * R2D);

  /* iers mean pole (mas) */
  long double xp_bar, yp_bar;
  iers_mean_pole(tut, &xp_bar, &yp_bar);

  /* ref [7] eq.7.24 */
  long double m1 = erpv[0] / AS2R - xp_bar * 1E-3L; /* (as) */
  long double m2 = -erpv[1] / AS2R + yp_bar * 1E-3L;

  /* sinl(2*theta) = sinl(2*phi), cosl(2*theta)=-cosl(2*phi) */
  long double cosll = cosl(pos[1]);
  long double sinll = sinl(pos[1]);
  denu[0] = 9E-3L * sinl(pos[0]) * (m1 * sinll - m2 * cosll);          /* de= Slambda (m) */
  denu[1] = -9E-3L * cosl(2.0L * pos[0]) * (m1 * cosll + m2 * sinll);  /* dn=-Stheta  (m) */
  denu[2] = -33E-3L * sinl(2.0L * pos[0]) * (m1 * cosll + m2 * sinll); /* du= Sr      (m) */

  trace(5, "tide_pole : denu=%.3Lf %.3Lf %.3Lf\n", denu[0], denu[1], denu[2]);
}
/* tidal displacement ----------------------------------------------------------
 * displacements by earth tides
 * args   : gtime_t tutc     I   time in utc
 *          long double *rr       I   site position (ecef) (m)
 *          int    opt       I   options (or of the followings)
 *                                 1: solid earth tide
 *                                 2: ocean tide loading
 *                                 4: pole tide
 *                                 8: elimate permanent deformation
 *          long double *erp      I   earth rotation parameters (NULL: not used)
 *          long double *odisp    I   ocean loading parameters  (NULL: not used)
 *                                 odisp[0+i*6]: consituent i amplitude radial(m)
 *                                 odisp[1+i*6]: consituent i amplitude west  (m)
 *                                 odisp[2+i*6]: consituent i amplitude south (m)
 *                                 odisp[3+i*6]: consituent i phase radial  (deg)
 *                                 odisp[4+i*6]: consituent i phase west    (deg)
 *                                 odisp[5+i*6]: consituent i phase south   (deg)
 *                                (i=0:M2,1:S2,2:N2,3:K2,4:K1,5:O1,6:P1,7:Q1,
 *                                   8:Mf,9:Mm,10:Ssa)
 *          long double *dr       O   displacement by earth tides (ecef) (m)
 * return : none
 * notes  : see ref [1], [2] chap 7
 *          see ref [4] 5.2.1, 5.2.2, 5.2.3
 *          ver.2.4.0 does not use ocean loading and pole tide corrections
 *-----------------------------------------------------------------------------*/
extern void tidedisp(gtime_t tutc, const long double *rr, int opt, const erp_t *erp,
                     const long double *odisp, long double *dr) {
  char tstr[40];
  trace(3, "tidedisp: tutc=%s\n", time2str(tutc, tstr, 0));

  long double erpv[5] = {0};
  if (erp) {
    geterp(erp, utc2gpst(tutc), erpv);
  }
  gtime_t tut = timeadd(tutc, erpv[2]);

  dr[0] = dr[1] = dr[2] = 0.0L;

  if (norm(rr, 3) <= 0.0L) return;

  long double pos[2];
  pos[0] = asinl(rr[2] / norm(rr, 3));
  pos[1] = atan2l(rr[1], rr[0]);
  long double E[9];
  xyz2enu(pos, E);

  if (opt & 1) { /* solid earth tides */

    /* sun and moon position in ecef */
    long double rs[3], rm[3], gmst;
    sunmoonpos(tutc, erpv, rs, rm, &gmst);

    long double drt[3];
#ifdef IERS_MODEL
    long double ep[6];
    time2epoch(tutc, ep);
    int year = (int)ep[0];
    int mon = (int)ep[1];
    int day = (int)ep[2];
    long double fhr = ep[3] + ep[4] / 60.0L + ep[5] / 3600.0L;

    /* call DEHANTTIDEINEL */
    dehanttideinel_((long double *)rr, &year, &mon, &day, &fhr, rs, rm, drt);
#else
    tide_solid(rs, rm, pos, E, gmst, opt, drt);
#endif
    for (int i = 0; i < 3; i++) dr[i] += drt[i];
  }
  if ((opt & 2) && odisp) { /* ocean tide loading */
    long double denu[3];
    tide_oload(tut, odisp, denu);
    long double drt[3];
    matmul("TN", 3, 1, 3, E, denu, drt);
    for (int i = 0; i < 3; i++) dr[i] += drt[i];
  }
  if ((opt & 4) && erp) { /* pole tide */
    long double denu[3];
    tide_pole(tut, pos, erpv, denu);
    long double drt[3];
    matmul("TN", 3, 1, 3, E, denu, drt);
    for (int i = 0; i < 3; i++) dr[i] += drt[i];
  }
  trace(5, "tidedisp: dr=%.3Lf %.3Lf %.3Lf\n", dr[0], dr[1], dr[2]);
}
