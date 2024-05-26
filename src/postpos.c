/*------------------------------------------------------------------------------
 * postpos.c : post-processing positioning
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * History : 2007/05/08  1.0  new
 *           2008/06/16  1.1  support binary inputs
 *           2009/01/02  1.2  support new rtk positioning api
 *           2009/09/03  1.3  fix bug on combined mode of moving-baseline
 *           2009/12/04  1.4  fix bug on obs data buffer overflow
 *           2010/07/26  1.5  support ppp-kinematic and ppp-static
 *                            support multiple sessions
 *                            support SBAS positioning
 *                            changed api:
 *                                postpos()
 *                            deleted api:
 *                                postposopt()
 *           2010/08/16  1.6  fix bug SBAS message synchronization (2.4.0_p4)
 *           2010/12/09  1.7  support QZSS lex and SSR corrections
 *           2011/02/07  1.8  fix bug on SBAS navigation data conflict
 *           2011/03/22  1.9  add function reading g_tec file
 *           2011/08/20  1.10 fix bug on freez if solstatic=single and combined
 *           2011/09/15  1.11 add function reading stec file
 *           2012/02/01  1.12 support keyword expansion of RTCM SSR corrections
 *           2013/03/11  1.13 add function reading otl and erp data
 *           2014/06/29  1.14 fix problem on overflow of # of satellites
 *           2015/03/23  1.15 fix bug on ant type replacement by RINEX header
 *                            fix bug on combined filter for moving-base mode
 *           2015/04/29  1.16 fix bug on reading RTCM SSR corrections
 *                            add function to read satellite fcb
 *                            add function to read stec and troposphere file
 *                            add keyword replacement in dcb, erp and ionos file
 *           2015/11/13  1.17 add support of L5 antenna phase center parameters
 *                            add *.stec and *.trp file for ppp correction
 *           2015/11/26  1.18 support opt->freqopt(disable L2)
 *           2016/01/12  1.19 add carrier-phase bias correction by SSR
 *           2016/07/31  1.20 fix error message problem in rnx2rtkp
 *           2016/08/29  1.21 suppress warnings
 *           2016/10/10  1.22 fix bug on identification of file fopt->blq
 *           2017/06/13  1.23 add smoother of velocity solution
 *           2020/11/30  1.24 use API sat2freq() to get carrier frequency
 *                            fix bug on select best solution in static mode
 *                            delete function to use L2 instead of L5 PCV
 *                            writing solution file in binary mode
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define SQRT(x) ((x) <= 0.0 || (x) != (x) ? 0.0 : sqrt(x))

#define MAXPRCDAYS 100   /* Max days of continuous processing */
#define MAXINFILE 1000   /* Max number of input files */
#define MAXINVALIDTM 100 /* Max number of invalid time marks */

/* Constants/global variables ------------------------------------------------*/

static pcvs_t pcvss = {0};                      /* Satellite antenna parameters */
static pcvs_t pcvsr = {0};                      /* Receiver antenna parameters */
static obs_t obss = {0};                        /* Observation data */
static nav_t navs = {0};                        /* Navigation data */
static sbs_t sbss = {0};                        /* SBAS messages */
static sta_t stas[MAXRCV];                      /* Station information */
static int nepoch = 0;                          /* Number of observation epochs */
static int nitm = 0;                            /* Number of invalid time marks */
static int iobsu = 0;                           /* Current rover observation data index */
static int iobsr = 0;                           /* Current reference observation data index */
static int isbs = 0;                            /* Current SBAS message index */
static int iitm = 0;                            /* Current invalid time mark index */
static int reverse = 0;                         /* Analysis direction (0:forward,1:backward) */
static int aborts = 0;                          /* Abort status */
static sol_t *solf;                             /* Forward solutions */
static sol_t *solb;                             /* Backward solutions */
static double *rbf;                             /* Forward base positions */
static double *rbb;                             /* Backward base positions */
static int isolf = 0;                           /* Current forward solutions index */
static int isolb = 0;                           /* Current backward solutions index */
static char proc_rov[64] = "";                  /* Rover for current processing */
static char proc_base[64] = "";                 /* Base station for current processing */
static char rtcm_file[FNSIZE] = "";             /* RTCM data file */
static char rtcm_path[FNSIZE] = "";             /* RTCM data path */
static gtime_t invalidtm[MAXINVALIDTM] = {{0}}; /* Invalid time marks */
static rtcm_t rtcm;                             /* RTCM control struct */
static FILE *fp_rtcm = NULL;                    /* RTCM data file pointer */

/* Show message and check break ----------------------------------------------*/
static int checkbrk(const char *format, ...) {
  va_list arg;
  char buff[1024];
  if (!*format) return showmsg("");
  va_start(arg, format);
  vsnprintf(buff, sizeof(buff), format, arg);
  va_end(arg);
  if (*proc_rov && *proc_base)
    rtkcatprintf(buff, sizeof(buff), " (%s-%s)", proc_rov, proc_base);
  else if (*proc_rov)
    rtkcatprintf(buff, sizeof(buff), " (%s)", proc_rov);
  else if (*proc_base)
    rtkcatprintf(buff, sizeof(buff), " (%s)", proc_base);
  return showmsg(buff);
}
/* Output reference position -------------------------------------------------*/
static void outrpos(FILE *fp, const double *r, const solopt_t *opt) {
  const char *sep = opt->sep;

  trace(3, "outrpos :\n");

  if (opt->posf == SOLF_LLH || opt->posf == SOLF_ENU) {
    double pos[3];
    ecef2pos(r, pos);
    if (opt->degf) {
      double dms1[3], dms2[3];
      deg2dms(pos[0] * R2D, dms1, 5);
      deg2dms(pos[1] * R2D, dms2, 5);
      fprintf(fp, "%3.0f%s%02.0f%s%08.5f%s%4.0f%s%02.0f%s%08.5f%s%10.4f", dms1[0], sep, dms1[1],
              sep, dms1[2], sep, dms2[0], sep, dms2[1], sep, dms2[2], sep, pos[2]);
    } else {
      fprintf(fp, "%13.9f%s%14.9f%s%10.4f", pos[0] * R2D, sep, pos[1] * R2D, sep, pos[2]);
    }
  } else if (opt->posf == SOLF_XYZ) {
    fprintf(fp, "%14.4f%s%14.4f%s%14.4f", r[0], sep, r[1], sep, r[2]);
  }
}
/* Output header -------------------------------------------------------------*/
static void outheader(FILE *fp, const char **file, int n, const prcopt_t *popt,
                      const solopt_t *sopt) {
  trace(3, "outheader: n=%d\n", n);

  if (sopt->posf == SOLF_NMEA || sopt->posf == SOLF_STAT) {
    return;
  }
  if (sopt->outhead) {
    if (!*sopt->prog) {
      fprintf(fp, "%s program   : RTKLIB ver.%s %s\n", COMMENTH, VER_RTKLIB, PATCH_LEVEL);
    } else {
      fprintf(fp, "%s program   : %s\n", COMMENTH, sopt->prog);
    }
    for (int i = 0; i < n; i++) {
      fprintf(fp, "%s inp file  : %s\n", COMMENTH, file[i]);
    }
    int i;
    for (i = 0; i < obss.n; i++)
      if (obss.data[i].rcv == 1) break;
    int j;
    for (j = obss.n - 1; j >= 0; j--)
      if (obss.data[j].rcv == 1) break;
    if (j < i) {
      fprintf(fp, "\n%s no rover obs data\n", COMMENTH);
      return;
    }
    gtime_t ts = obss.data[i].time;
    gtime_t te = obss.data[j].time;
    int w1, w2;
    double t1 = time2gpst(ts, &w1);
    double t2 = time2gpst(te, &w2);
    if (sopt->times >= 1) {
      ts = gpst2utc(ts);
      te = gpst2utc(te);
    }
    if (sopt->times == 2) {
      ts = timeadd(ts, 9 * 3600.0);
      te = timeadd(te, 9 * 3600.0);
    }
    char s2[40], s3[40];
    time2str(ts, s2, 1);
    time2str(te, s3, 1);
    const char *s1[] = {"GPST", "UTC", "JST"};
    fprintf(fp, "%s obs start : %s %s (week%04d %8.1fs)\n", COMMENTH, s2, s1[sopt->times], w1, t1);
    fprintf(fp, "%s obs end   : %s %s (week%04d %8.1fs)\n", COMMENTH, s3, s1[sopt->times], w2, t2);
  }
  if (sopt->outopt) {
    outprcopt(fp, popt);
  }
  if (PMODE_DGPS <= popt->mode && popt->mode <= PMODE_FIXED && popt->mode != PMODE_MOVEB) {
    fprintf(fp, "%s ref pos   :", COMMENTH);
    outrpos(fp, popt->rb, sopt);
    fprintf(fp, "\n");
  }
  if (sopt->outhead || sopt->outopt) fprintf(fp, "%s\n", COMMENTH);

  outsolhead(fp, sopt);
}
/* Search next observation data index ------------------------------------------
   Note *i will be advanced outside the index range of the obs data if none
   are found. */
static int nextobsf(const obs_t *obs, int *i, int rcv) {
  for (; *i < obs->n; (*i)++)
    if (obs->data[*i].rcv == rcv) break;
  int n;
  for (n = 0; *i + n < obs->n; n++) {
    if (obs->data[*i + n].rcv != rcv) break;
    double tt = timediff(obs->data[*i + n].time, obs->data[*i].time);
    if (tt > DTTOL) break;
  }
  return n;
}
static int nextobsb(const obs_t *obs, int *i, int rcv) {
  for (; *i >= 0; (*i)--)
    if (obs->data[*i].rcv == rcv) break;
  int n;
  for (n = 0; *i - n >= 0; n++) {
    if (obs->data[*i - n].rcv != rcv) break;
    double tt = timediff(obs->data[*i - n].time, obs->data[*i].time);
    if (tt < -DTTOL) break;
  }
  return n;
}
/* Update RTCM SSR correction ------------------------------------------------*/
static void update_rtcm_ssr(gtime_t time) {
  /* Open or swap RTCM file */
  char path[FNSIZE];
  reppath(rtcm_file, path, sizeof(path), time, "", "");

  if (strcmp(path, rtcm_path)) {
    rtkstrcpy(rtcm_path, sizeof(rtcm_path), path);

    if (fp_rtcm) fclose(fp_rtcm);
    fp_rtcm = fopen(path, "rb");
    if (fp_rtcm) {
      rtcm.time = time;
      input_rtcm3f(&rtcm, fp_rtcm);
      trace(2, "rtcm file open: %s\n", path);
    }
  }
  if (!fp_rtcm) return;

  /* Read RTCM file until current time */
  while (timediff(rtcm.time, time) < 1E-3) {
    if (input_rtcm3f(&rtcm, fp_rtcm) < -1) break;

    /* Update SSR corrections */
    for (int i = 0; i < MAXSAT; i++) {
      if (!rtcm.ssr[i].update || rtcm.ssr[i].iod[0] != rtcm.ssr[i].iod[1] ||
          timediff(time, rtcm.ssr[i].t0[0]) < -1E-3)
        continue;
      navs.ssr[i] = rtcm.ssr[i];
      rtcm.ssr[i].update = 0;
    }
  }
}
/* Input obs data, navigation messages and SBAS correction -------------------*/
static int inputobs(obsd_t *obs, int solq, const prcopt_t *popt) {
  trace(3, "\ninfunc  : dir=%d iobsu=%d iobsr=%d isbs=%d\n", reverse, iobsu, iobsr, isbs);

  if (0 <= iobsu && iobsu < obss.n) {
    gtime_t time = obss.data[iobsu].time;
    settime(time);
    char tstr[40];
    if (checkbrk("processing : %s Q=%d", time2str(time, tstr, 0), solq)) {
      aborts = 1;
      showmsg("aborted");
      return -1;
    }
  }
  int n = 0;
  if (!reverse) {
    /* Input forward data */
    int nu = nextobsf(&obss, &iobsu, 1);
    if (nu <= 0) return -1;
    for (int i = 0; i < nu && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsu + i];
    if (iobsr < obss.n) {
      if (popt->intpref) {
        /* Interpolate nearest timestamps */
        int nr = nextobsf(&obss, &iobsr, 2);
        while (nr > 0) {
          if (timediff(obss.data[iobsr].time, obss.data[iobsu].time) > -DTTOL) break;
          iobsr += nr;
          nr = nextobsf(&obss, &iobsr, 2);
        }
      } else {
        /* Find the closest iobsr timestamp, before or after iobsu. */
        double dt = fabs(timediff(obss.data[iobsr].time, obss.data[iobsu].time));
        int i = iobsr, nr = nextobsf(&obss, &i, 2);
        while (nr > 0) {
          double dt_next = fabs(timediff(obss.data[i].time, obss.data[iobsu].time));
          if (dt_next > dt) break;
          dt = dt_next;
          iobsr = i;
          i += nr;
          nr = nextobsf(&obss, &i, 2);
        }
      }
      /* Recalculate nr for the determined iobsr. This does not change iobsr. */
      int nr = nextobsf(&obss, &iobsr, 2);
      for (int i = 0; i < nr && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsr + i];
    }
    iobsu += nu;

    /* Update SBAS corrections */
    while (isbs < sbss.n) {
      gtime_t time = gpst2time(sbss.msgs[isbs].week, sbss.msgs[isbs].tow);

      if (getbitu(sbss.msgs[isbs].msg, sizeof(sbss.msgs[0].msg), 8, 6) !=
          9) { /* Except for geo nav */
        sbsupdatecorr(sbss.msgs + isbs, &navs);
      }
      if (timediff(time, obs[0].time) > -1.0 - DTTOL) break;
      isbs++;
    }
    /* Update RTCM SSR corrections */
    if (*rtcm_file) {
      update_rtcm_ssr(obs[0].time);
    }
  } else {
    /* Input backward data */
    int nu = nextobsb(&obss, &iobsu, 1);
    if (nu <= 0) return -1;
    for (int i = 0; i < nu && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsu - nu + 1 + i];
    if (iobsr >= 0) {
      if (popt->intpref) {
        /* Interpolate nearest timestamps */
        int nr = nextobsb(&obss, &iobsr, 2);
        while (nr > 0) {
          if (timediff(obss.data[iobsr].time, obss.data[iobsu].time) < DTTOL) break;
          iobsr -= nr;
          nr = nextobsb(&obss, &iobsr, 2);
        }
      } else {
        /* Find the closest iobsr timestamp, before or after iobsu. */
        double dt = fabs(timediff(obss.data[iobsr].time, obss.data[iobsu].time));
        int i = iobsr, nr = nextobsb(&obss, &i, 2);
        while (nr > 0) {
          double dt_next = fabs(timediff(obss.data[i].time, obss.data[iobsu].time));
          if (dt_next > dt) break;
          dt = dt_next;
          iobsr = i;
          i -= nr;
          nr = nextobsb(&obss, &i, 2);
        }
      }
      int nr = nextobsb(&obss, &iobsr, 2);
      for (int i = 0; i < nr && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsr - nr + 1 + i];
    }
    iobsu -= nu;

    /* Update SBAS corrections */
    while (isbs >= 0) {
      gtime_t time = gpst2time(sbss.msgs[isbs].week, sbss.msgs[isbs].tow);

      if (getbitu(sbss.msgs[isbs].msg, sizeof(sbss.msgs[0].msg), 8, 6) !=
          9) { /* Except for geo nav */
        sbsupdatecorr(sbss.msgs + isbs, &navs);
      }
      if (timediff(time, obs[0].time) < 1.0 + DTTOL) break;
      isbs--;
    }
  }
  return n;
}
/* Output to file message of invalid time mark -------------------------------*/
static void outinvalidtm(FILE *fptm, const solopt_t *opt, const gtime_t tm) {
  const double secondsInAWeek = 604800;

  gtime_t time = tm;
  if (opt->times >= TIMES_UTC) time = gpst2utc(time);
  if (opt->times == TIMES_JST) time = timeadd(time, 9 * 3600.0);

  int timeu = opt->timeu < 0 ? 0 : (opt->timeu > 20 ? 20 : opt->timeu);
  char s[100];
  if (opt->timef)
    time2str(time, s, timeu);
  else {
    int week;
    double gpst = time2gpst(time, &week);
    if (secondsInAWeek - gpst < 0.5 / pow(10.0, timeu)) {
      week++;
      gpst = 0.0;
    }
    rtksnprintf(s, sizeof(s), "%4d   %*.*f", week, 6 + (timeu <= 0 ? 0 : timeu + 1), timeu, gpst);
  }
  rtkstrcat(s, sizeof(s), "   Q=0, Time mark is not valid\n");

  fwrite(s, strlen(s), 1, fptm);
}
/* Fill structure sol_t for time mark ----------------------------------------*/
static sol_t fillsoltm(const sol_t solold, const sol_t solnew, const gtime_t tm) {
  gtime_t t1 = {0}, t2 = {0};
  sol_t sol = solold;

  if (solold.stat == 0 || solnew.stat == 0) {
    sol.stat = 0;
  } else {
    sol.stat = (solold.stat > solnew.stat) ? solold.stat : solnew.stat;
  }
  sol.ns = (solold.ns < solnew.ns) ? solold.ns : solnew.ns;
  sol.ratio = (solold.ratio < solnew.ratio) ? solold.ratio : solnew.ratio;

  /* Interpolation position and speed of time mark */
  t1 = solold.time;
  t2 = solnew.time;
  sol.time = tm;

  for (int i = 0; i < 6; i++) {
    sol.rr[i] = solold.rr[i] + timediff(tm, t1) / timediff(t2, t1) * (solnew.rr[i] - solold.rr[i]);
  }

  return sol;
}

/* Carrier-phase bias correction by SSR --------------------------------------*/
static void corr_phase_bias_ssr(obsd_t *obs, int n, const nav_t *nav) {
  for (int i = 0; i < n; i++)
    for (int j = 0; j < NFREQ; j++) {
      uint8_t code = obs[i].code[j];

      double freq = sat2freq(obs[i].sat, code, nav);
      if (freq == 0.0) continue;

      /* Correct phase bias (cyc) */
      obs[i].L[j] -= nav->ssr[obs[i].sat - 1].pbias[code - 1] * freq / CLIGHT;
    }
}
/* Process positioning -------------------------------------------------------*/
static void procpos(FILE *fp, FILE *fptm, const prcopt_t *popt, const solopt_t *sopt, rtk_t *rtk,
                    enum solmode mode) {
  gtime_t time = {0};
  const int pri[] = {6, 1, 2, 3, 4, 5, 1, 6};

  trace(3, "procpos : mode=%d\n", mode); /* 0=single dir, 1=combined */

  int solstatic =
      sopt->solstatic && (popt->mode == PMODE_STATIC || popt->mode == PMODE_STATIC_START ||
                          popt->mode == PMODE_PPP_STATIC);

  rtcm_path[0] = '\0';

  int nobs, n, num = 0;
  sol_t sol = {{0}}, oldsol = {{0}}, newsol = {{0}};
  double rb[3] = {0};
  obsd_t *obs_ptr = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS * 2); /* For rover and base */
  while ((nobs = inputobs(obs_ptr, rtk->sol.stat, popt)) >= 0) {
    /* Exclude satellites */
    for (int i = n = 0; i < nobs; i++) {
      if ((satsys(obs_ptr[i].sat, NULL) & popt->navsys) && popt->exsats[obs_ptr[i].sat - 1] != 1)
        obs_ptr[n++] = obs_ptr[i];
    }
    if (n <= 0) continue;

    /* Carrier-phase bias correction */
    if (!strstr(popt->pppopt, "-ENA_FCB")) {
      corr_phase_bias_ssr(obs_ptr, n, &navs);
    }
    if (!rtkpos(rtk, obs_ptr, n, &navs)) {
      if (rtk->sol.eventime.time != 0) {
        if (mode == SOLMODE_SINGLE_DIR) {
          outinvalidtm(fptm, sopt, rtk->sol.eventime);
        } else if (!reverse && nitm < MAXINVALIDTM) {
          invalidtm[nitm++] = rtk->sol.eventime;
        }
      }
      continue;
    }

    if (mode == SOLMODE_SINGLE_DIR) { /* Forward or backward */
      if (!solstatic) {
        outsol(fp, &rtk->sol, rtk->rb, sopt);
      } else if (time.time == 0 || pri[rtk->sol.stat] <= pri[sol.stat]) {
        sol = rtk->sol;
        for (int i = 0; i < 3; i++) rb[i] = rtk->rb[i];
        if (time.time == 0 || timediff(rtk->sol.time, time) < 0.0) {
          time = rtk->sol.time;
        }
      }
      /* Check time mark */
      if (rtk->sol.eventime.time != 0) {
        newsol = fillsoltm(oldsol, rtk->sol, rtk->sol.eventime);
        num++;
        if (!solstatic && mode == SOLMODE_SINGLE_DIR) {
          outsol(fptm, &newsol, rb, sopt);
        }
      }
      oldsol = rtk->sol;
    } else if (!reverse) { /* Combined-forward */
      if (isolf >= nepoch) {
        free(obs_ptr);
        return;
      }
      solf[isolf] = rtk->sol;
      for (int i = 0; i < 3; i++) rbf[i + isolf * 3] = rtk->rb[i];
      isolf++;
    } else { /* Combined-backward */
      if (isolb >= nepoch) {
        free(obs_ptr);
        return;
      }
      solb[isolb] = rtk->sol;
      for (int i = 0; i < 3; i++) rbb[i + isolb * 3] = rtk->rb[i];
      isolb++;
    }
  }
  if (mode == SOLMODE_SINGLE_DIR && solstatic && time.time != 0.0) {
    sol.time = time;
    outsol(fp, &sol, rb, sopt);
  }

  free(obs_ptr); /* Moved from stack to heap to kill a stack overflow warning */
}
/* Validation of combined solutions ------------------------------------------*/
static bool valcomb(const sol_t *solf, const sol_t *solb, const double *rbf, const double *rbb,
                    const prcopt_t *popt) {
  trace(4, "valcomb :\n");

  /* Compare forward and backward solution */
  double dr[3], var[3];
  for (int i = 0; i < 3; i++) {
    dr[i] = solf->rr[i] - solb->rr[i];
    if (popt->mode == PMODE_MOVEB) dr[i] -= (rbf[i] - rbb[i]);
    var[i] = (double)solf->qr[i] + (double)solb->qr[i];
  }
  for (int i = 0; i < 3; i++) {
    if (dr[i] * dr[i] <= 16.0 * var[i]) continue; /* Ok if in 4-sigma */

    char tstr[40];
    time2str(solf->time, tstr, 2);
    trace(2, "degrade fix to float: %s dr=%.3f %.3f %.3f std=%.3f %.3f %.3f\n", tstr + 11, dr[0],
          dr[1], dr[2], SQRT(var[0]), SQRT(var[1]), SQRT(var[2]));
    return false;
  }
  return true;
}
/* Combine forward/backward solutions and save results -----------------------*/
static void combres(FILE *fp, FILE *fptm, const prcopt_t *popt, const solopt_t *sopt) {
  const int pri[] = {7, 1, 2, 3, 4, 5, 1, 6};

  trace(3, "combres : isolf=%d isolb=%d\n", isolf, isolb);

  int solstatic =
      sopt->solstatic && (popt->mode == PMODE_STATIC || popt->mode == PMODE_STATIC_START ||
                          popt->mode == PMODE_PPP_STATIC);

  int num = 0;
  double rbs[3] = {0}, rb[3] = {0};
  sol_t sols = {{0}}, sol = {{0}}, oldsol = {{0}}, newsol = {{0}};
  gtime_t time = {0};
  for (int i = 0, j = isolb - 1; i < isolf && j >= 0; i++, j--) {
    double tt = timediff(solf[i].time, solb[j].time);
    if (tt < -DTTOL) {
      sols = solf[i];
      for (int k = 0; k < 3; k++) rbs[k] = rbf[k + i * 3];
      j++;
    } else if (tt > DTTOL) {
      sols = solb[j];
      for (int k = 0; k < 3; k++) rbs[k] = rbb[k + j * 3];
      i--;
    } else if (pri[solf[i].stat] < pri[solb[j].stat]) {
      sols = solf[i];
      for (int k = 0; k < 3; k++) rbs[k] = rbf[k + i * 3];
    } else if (pri[solf[i].stat] > pri[solb[j].stat]) {
      sols = solb[j];
      for (int k = 0; k < 3; k++) rbs[k] = rbb[k + j * 3];
    } else {
      sols = solf[i];
      sols.time = timeadd(sols.time, -tt / 2.0);

      if ((popt->mode == PMODE_KINEMA || popt->mode == PMODE_MOVEB) && sols.stat == SOLQ_FIX) {
        /* Degrade fix to float if validation failed */
        if (!valcomb(solf + i, solb + j, rbf + i * 3, rbb + j * 3, popt)) sols.stat = SOLQ_FLOAT;
      }
      double Qf[9], Qb[9];
      for (int k = 0; k < 3; k++) {
        Qf[k + k * 3] = solf[i].qr[k];
        Qb[k + k * 3] = solb[j].qr[k];
      }
      Qf[1] = Qf[3] = solf[i].qr[3];
      Qf[5] = Qf[7] = solf[i].qr[4];
      Qf[2] = Qf[6] = solf[i].qr[5];
      Qb[1] = Qb[3] = solb[j].qr[3];
      Qb[5] = Qb[7] = solb[j].qr[4];
      Qb[2] = Qb[6] = solb[j].qr[5];

      double Qs[9];
      if (popt->mode == PMODE_MOVEB) {
        double rr_f[3], rr_b[3];
        for (int k = 0; k < 3; k++) rr_f[k] = solf[i].rr[k] - rbf[k + i * 3];
        for (int k = 0; k < 3; k++) rr_b[k] = solb[j].rr[k] - rbb[k + j * 3];
        double rr_s[3];
        if (smoother(rr_f, Qf, rr_b, Qb, 3, rr_s, Qs)) continue;
        for (int k = 0; k < 3; k++) sols.rr[k] = rbs[k] + rr_s[k];
      } else {
        if (smoother(solf[i].rr, Qf, solb[j].rr, Qb, 3, sols.rr, Qs)) continue;
      }
      sols.qr[0] = (float)Qs[0];
      sols.qr[1] = (float)Qs[4];
      sols.qr[2] = (float)Qs[8];
      sols.qr[3] = (float)Qs[1];
      sols.qr[4] = (float)Qs[5];
      sols.qr[5] = (float)Qs[2];

      /* Smoother for velocity solution */
      if (popt->dynamics) {
        for (int k = 0; k < 3; k++) {
          Qf[k + k * 3] = solf[i].qv[k];
          Qb[k + k * 3] = solb[j].qv[k];
        }
        Qf[1] = Qf[3] = solf[i].qv[3];
        Qf[5] = Qf[7] = solf[i].qv[4];
        Qf[2] = Qf[6] = solf[i].qv[5];
        Qb[1] = Qb[3] = solb[j].qv[3];
        Qb[5] = Qb[7] = solb[j].qv[4];
        Qb[2] = Qb[6] = solb[j].qv[5];
        if (smoother(solf[i].rr + 3, Qf, solb[j].rr + 3, Qb, 3, sols.rr + 3, Qs)) continue;
        sols.qv[0] = (float)Qs[0];
        sols.qv[1] = (float)Qs[4];
        sols.qv[2] = (float)Qs[8];
        sols.qv[3] = (float)Qs[1];
        sols.qv[4] = (float)Qs[5];
        sols.qv[5] = (float)Qs[2];
      }
    }
    if (!solstatic) {
      outsol(fp, &sols, rbs, sopt);
    } else if (time.time == 0 || pri[sols.stat] <= pri[sol.stat]) {
      sol = sols;
      for (int k = 0; k < 3; k++) rb[k] = rbs[k];
      if (time.time == 0 || timediff(sols.time, time) < 0.0) {
        time = sols.time;
      }
    }
    if (iitm < nitm && timediff(invalidtm[iitm], sols.time) < 0.0) {
      outinvalidtm(fptm, sopt, invalidtm[iitm]);
      iitm++;
    }
    if (sols.eventime.time != 0) {
      newsol = fillsoltm(oldsol, sols, sols.eventime);
      num++;
      if (!solstatic) {
        outsol(fptm, &newsol, rb, sopt);
      }
    }
    oldsol = sols;
  }
  if (solstatic && time.time != 0.0) {
    sol.time = time;
    outsol(fp, &sol, rb, sopt);
  }
}
/* Read prec ephemeris, SBAS data, TEC grid and open RTCM --------------------*/
static void readpreceph(const char **infile, int n, const prcopt_t *prcopt, nav_t *nav,
                        sbs_t *sbs) {
  trace(2, "readpreceph: n=%d\n", n);

  nav->ne = nav->nemax = 0;
  nav->nc = nav->ncmax = 0;
  sbs->n = sbs->nmax = 0;

  /* Read precise ephemeris files */
  for (int i = 0; i < n; i++) {
    if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
    readsp3(infile[i], nav, 0);
  }
  /* Read precise clock files */
  for (int i = 0; i < n; i++) {
    if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
    readrnxc(infile[i], nav);
  }
  /* Read SBAS message files */
  for (int i = 0; i < n; i++) {
    if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
    sbsreadmsg(infile[i], prcopt->sbassatsel, sbs);
  }

  /* Set RTCM file and initialize RTCM struct */
  rtcm_file[0] = rtcm_path[0] = '\0';
  fp_rtcm = NULL;

  for (int i = 0; i < n; i++) {
    const char *ext = strrchr(infile[i], '.');
    if (ext && (!strcmp(ext, ".rtcm3") || !strcmp(ext, ".RTCM3"))) {
      rtkstrcpy(rtcm_file, sizeof(rtcm_file), infile[i]);
      init_rtcm(&rtcm);
      break;
    }
  }
}
/* Free prec ephemeris and SBAS data -----------------------------------------*/
static void freepreceph(nav_t *nav, sbs_t *sbs) {
  trace(3, "freepreceph:\n");

  free(nav->peph);
  nav->peph = NULL;
  nav->ne = nav->nemax = 0;
  free(nav->pclk);
  nav->pclk = NULL;
  nav->nc = nav->ncmax = 0;
  free(sbs->msgs);
  sbs->msgs = NULL;
  sbs->n = sbs->nmax = 0;
  for (int i = 0; i < nav->nt; i++) {
    free(nav->tec[i].data);
    free(nav->tec[i].rms);
  }
  free(nav->tec);
  nav->tec = NULL;
  nav->nt = nav->ntmax = 0;

  if (fp_rtcm) fclose(fp_rtcm);
  free_rtcm(&rtcm);
}
/* Read obs and nav data -----------------------------------------------------*/
static bool readobsnav(gtime_t ts, gtime_t te, double ti, const char **infile, const int *index,
                       int n, const prcopt_t *prcopt, obs_t *obs, nav_t *nav, sta_t *sta) {
  char tstr[40];
  trace(3, "readobsnav: ts=%s n=%d\n", time2str(ts, tstr, 0), n);

  obs->data = NULL;
  obs->n = obs->nmax = 0;
  for (int i = 0; i < MAXSAT; i++) {
    nav->eph[i] = NULL;
    nav->n[i] = nav->nmax[i] = 0;
  }
  for (int i = 0; i < NSATGLO; i++) {
    nav->geph[i] = NULL;
    nav->ng[i] = nav->ngmax[i] = 0;
  }
  for (int i = 0; i < NSATSBS; i++) {
    nav->seph[i] = NULL;
    nav->ns[i] = nav->nsmax[i] = 0;
  }
  nepoch = 0;

  int ind = 0, nobs = 0;
  for (int i = 0, rcv = 1; i < n; i++) {
    if (checkbrk("")) return false;

    if (index[i] != ind) {
      if (obs->n > nobs) rcv++;
      ind = index[i];
      nobs = obs->n;
    }
    /* Read RINEX obs and nav file */
    if (readrnxt(infile[i], rcv, ts, te, ti, prcopt->rnxopt[rcv <= 1 ? 0 : 1], obs, nav,
                 rcv <= 2 ? sta + rcv - 1 : NULL) < 0) {
      checkbrk("error : insufficient memory");
      trace(1, "insufficient memory\n");
      return false;
    }
  }
  if (obs->n <= 0) {
    checkbrk("error : no obs data");
    trace(1, "\n");
    return false;
  }
  if (navncnt(nav) <= 0 && navngcnt(nav) <= 0 && navnscnt(nav) <= 0) {
    checkbrk("error : no nav data");
    trace(1, "\n");
    return false;
  }
  /* Sort observation data */
  nepoch = sortobs(obs);

  /* Delete duplicated ephemeris */
  uniqnav(nav);

  /* Set time span for progress display */
  if (ts.time == 0 || te.time == 0) {
    int i, j;
    for (i = 0; i < obs->n; i++)
      if (obs->data[i].rcv == 1) break;
    for (j = obs->n - 1; j >= 0; j--)
      if (obs->data[j].rcv == 1) break;
    if (i < j) {
      if (ts.time == 0) ts = obs->data[i].time;
      if (te.time == 0) te = obs->data[j].time;
      settspan(ts, te);
    }
  }
  return true;
}
/* Free obs and nav data -----------------------------------------------------*/
static void freeobsnav(obs_t *obs, nav_t *nav) {
  trace(3, "freeobsnav:\n");

  free(obs->data);
  obs->data = NULL;
  obs->n = obs->nmax = 0;
  for (int i = 0; i < MAXSAT; i++) {
    free(nav->eph[i]);
    nav->eph[i] = NULL;
    nav->n[i] = nav->nmax[i] = 0;
  }
  for (int i = 0; i < NSATGLO; i++) {
    free(nav->geph[i]);
    nav->geph[i] = NULL;
    nav->ng[i] = nav->ngmax[i] = 0;
  }
  for (int i = 0; i < NSATSBS; i++) {
    free(nav->seph[i]);
    nav->seph[i] = NULL;
    nav->ns[i] = nav->nsmax[i] = 0;
  }
}
/* Average of single position ------------------------------------------------*/
static bool avepos(double *ra, int rcv, const obs_t *obs, const nav_t *nav, const prcopt_t *opt) {
  trace(3, "avepos: rcv=%d obs.n=%d\n", rcv, obs->n);

  for (int i = 0; i < 3; i++) ra[i] = 0.0;

  int n = 0, m;
  for (int iobs = 0; (m = nextobsf(obs, &iobs, rcv)) > 0; iobs += m) {
    obsd_t data[MAXOBS];
    int j = 0;
    for (int i = 0; i < m && i < MAXOBS; i++) {
      data[j] = obs->data[iobs + i];
      if ((satsys(data[j].sat, NULL) & opt->navsys) && opt->exsats[data[j].sat - 1] != 1) j++;
    }
    gtime_t ts = {0};
    if (j <= 0 || !screent(data[0].time, ts, ts, 1.0)) continue; /* Only 1 hz */

    /* Messages are disarded. */
    char msg[128];
    msg[0] = '\0';
    sol_t sol = {{0}};
    if (!pntpos(data, j, nav, opt, &sol, NULL, NULL, msg, sizeof(msg))) continue;

    for (int i = 0; i < 3; i++) ra[i] += sol.rr[i];
    n++;
  }
  if (n <= 0) {
    trace(1, "no average of base station position\n");
    return false;
  }
  for (int i = 0; i < 3; i++) ra[i] /= n;
  return true;
}
/* Station position from file ------------------------------------------------*/
static bool getstapos(const char *file, const char *name, double *r) {
  trace(3, "getstapos: file=%s name=%s\n", file, name);

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(1, "station position file open error: %s\n", file);
    return false;
  }
  char buff[256];
  while (fgets(buff, sizeof(buff), fp)) {
    char *p = strchr(buff, '%');
    if (p) *p = '\0';

    double pos[3];
    char sname[256];
    if (sscanf(buff, "%lf %lf %lf %255s", pos, pos + 1, pos + 2, sname) < 4) continue;

    const char *q;
    for (p = sname, q = name; *p && *q; p++, q++) {
      if (toupper((int)*p) != toupper((int)*q)) break;
    }
    if (!*p) {
      pos[0] *= D2R;
      pos[1] *= D2R;
      pos2ecef(pos, r);
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  trace(1, "no station position: %s %s\n", name, file);
  return false;
}
/* Antenna phase center position ---------------------------------------------*/
static bool antpos(prcopt_t *opt, int rcvno, const obs_t *obs, const nav_t *nav, const sta_t *sta,
                   const char *posfile) {
  trace(3, "antpos  : rcvno=%d\n", rcvno);

  double *rr = rcvno == 1 ? opt->ru : opt->rb;
  enum posopt postype = rcvno == 1 ? opt->rovpos : opt->refpos;
  if (postype == POSOPT_SINGLE) { /* Average of single position */
    if (!avepos(rr, rcvno, obs, nav, opt)) {
      showmsg("error : station pos computation");
      return false;
    }
  } else if (postype == POSOPT_FILE) { /* Read from position file */
    char *name = stas[rcvno == 1 ? 0 : 1].name;
    if (!getstapos(posfile, name, rr)) {
      showmsg("error : no position of %s in %s", name, posfile);
      return false;
    }
  } else if (postype == POSOPT_RINEX) { /* Get from RINEX header */
    double dr[3] = {0};
    if (norm(stas[rcvno == 1 ? 0 : 1].pos, 3) <= 0.0) {
      showmsg("error : no position in rinex header");
      trace(1, "no position in rinex header\n");
      return false;
    }
    /* Add antenna delta unless already done in antpcv() */
    if (!strcmp(opt->anttype[rcvno], "*")) {
      if (stas[rcvno == 1 ? 0 : 1].deltype == 0) { /* ENU */
        double del[3];
        for (int i = 0; i < 3; i++) del[i] = stas[rcvno == 1 ? 0 : 1].del[i];
        del[2] += stas[rcvno == 1 ? 0 : 1].hgt;
        double pos[3];
        ecef2pos(stas[rcvno == 1 ? 0 : 1].pos, pos);
        enu2ecef(pos, del, dr);
      } else { /* XYZ */
        for (int i = 0; i < 3; i++) dr[i] = stas[rcvno == 1 ? 0 : 1].del[i];
      }
    }
    for (int i = 0; i < 3; i++) rr[i] = stas[rcvno == 1 ? 0 : 1].pos[i] + dr[i];
  }
  return true;
}
/* Open processing session ---------------------------------------------------*/
static bool openses(const prcopt_t *popt, const solopt_t *sopt, const filopt_t *fopt, nav_t *nav,
                    pcvs_t *pcvs, pcvs_t *pcvr) {
  trace(3, "openses :\n");

  /* Read satellite antenna parameters */
  if (*fopt->satantp && !(readpcv(fopt->satantp, pcvs))) {
    showmsg("error : no sat ant pcv in %s", fopt->satantp);
    trace(1, "sat antenna pcv read error: %s\n", fopt->satantp);
    return false;
  }
  /* Read receiver antenna parameters */
  if (*fopt->rcvantp && !(readpcv(fopt->rcvantp, pcvr))) {
    showmsg("error : no rec ant pcv in %s", fopt->rcvantp);
    trace(1, "rec antenna pcv read error: %s\n", fopt->rcvantp);
    return false;
  }
  /* Open geoid data */
  if (sopt->geoid > 0 && *fopt->geoid) {
    if (!opengeoid(sopt->geoid, fopt->geoid)) {
      showmsg("error : no geoid data %s", fopt->geoid);
      trace(2, "no geoid data %s\n", fopt->geoid);
    }
  }
  return true;
}
/* Close processing session --------------------------------------------------*/
static void closeses(nav_t *nav, pcvs_t *pcvs, pcvs_t *pcvr) {
  trace(3, "closeses:\n");

  /* Free antenna parameters */
  free(pcvs->pcv);
  pcvs->pcv = NULL;
  pcvs->n = pcvs->nmax = 0;
  free(pcvr->pcv);
  pcvr->pcv = NULL;
  pcvr->n = pcvr->nmax = 0;

  /* Close geoid data */
  closegeoid();

  /* Free erp data */
  free(nav->erp.data);
  nav->erp.data = NULL;
  nav->erp.n = nav->erp.nmax = 0;

  /* Close solution statistics and debug trace */
  rtkclosestat();
  traceclose();
}
/* Set antenna parameters ----------------------------------------------------*/
static void setpcv(gtime_t time, prcopt_t *popt, nav_t *nav, const pcvs_t *pcvs, const pcvs_t *pcvr,
                   const sta_t *sta) {
  pcv_t pcv0 = {0};
  int mode = PMODE_DGPS <= popt->mode && popt->mode <= PMODE_FIXED;

  /* Set satellite antenna parameters */
  for (int i = 0; i < MAXSAT; i++) {
    nav->pcvs[i] = pcv0;
    if (!(satsys(i + 1, NULL) & popt->navsys)) continue;
    const pcv_t *pcv = searchpcv(i + 1, "", time, pcvs);
    if (!pcv) {
      char id[8];
      satno2id(i + 1, id);
      trace(4, "no satellite antenna pcv: %s\n", id);
      continue;
    }
    nav->pcvs[i] = *pcv;
  }
  for (int i = 0; i < (mode ? 2 : 1); i++) {
    popt->pcvr[i] = pcv0;
    if (!strcmp(popt->anttype[i], "*")) { /* Set by station parameters */
      rtkstrcpy(popt->anttype[i], sizeof(popt->anttype[i]), sta[i].antdes);
      if (sta[i].deltype == 1) { /* XYZ */
        if (norm(sta[i].pos, 3) > 0.0) {
          double pos[3];
          ecef2pos(sta[i].pos, pos);
          double del[3];
          ecef2enu(pos, sta[i].del, del);
          for (int j = 0; j < 3; j++) popt->antdel[i][j] = del[j];
        }
      } else { /* ENU */
        for (int j = 0; j < 3; j++) popt->antdel[i][j] = stas[i].del[j];
      }
    }
    const pcv_t *pcv = searchpcv(0, popt->anttype[i], time, pcvr);
    if (!pcv) {
      trace(2, "no receiver antenna pcv: %s\n", popt->anttype[i]);
      *popt->anttype[i] = '\0';
      continue;
    }
    rtkstrcpy(popt->anttype[i], sizeof(popt->anttype[i]), pcv->type);
    popt->pcvr[i] = *pcv;
  }
}
/* Read ocean tide loading parameters ----------------------------------------*/
static void readotl(prcopt_t *popt, const char *file, const sta_t *sta) {
  int mode = PMODE_DGPS <= popt->mode && popt->mode <= PMODE_FIXED;
  for (int i = 0; i < (mode ? 2 : 1); i++) {
    readblq(file, sta[i].name, popt->odisp[i]);
  }
}
/* Write header to output file -----------------------------------------------*/
static bool outhead(const char *outfile, const char **infile, int n, const prcopt_t *popt,
                    const solopt_t *sopt) {
  trace(3, "outhead: outfile=%s n=%d\n", outfile, n);

  FILE *fp = stdout;

  if (*outfile) {
    createdir(outfile);

    fp = fopen(outfile, "wb");
    if (!fp) {
      showmsg("error : open output file %s", outfile);
      return false;
    }
  }
  /* Output header */
  outheader(fp, infile, n, popt, sopt);

  if (*outfile) fclose(fp);

  return true;
}
/* Open output file for append -----------------------------------------------*/
static FILE *openfile(const char *outfile) {
  trace(3, "openfile: outfile=%s\n", outfile);

  return !*outfile ? stdout : fopen(outfile, "ab");
}
/* Name time marks file ------------------------------------------------------*/
static void namefiletm(char *outfiletm, size_t size, const char *outfile) {
  size_t i;
  for (i = strlen(outfile); i > 0; i--) {
    if (outfile[i] == '.') {
      break;
    }
  }
  /* If no file extension, then name time marks file as name of outfile + _events.pos */
  if (i == 0) {
    i = strlen(outfile);
  }
  rtkesubstrcpy(outfiletm, size, outfile, 0, i);
  rtkstrcat(outfiletm, size, "_events.pos");
}
/* Execute processing session ------------------------------------------------*/
static bool execses(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt, const solopt_t *sopt,
                    const filopt_t *fopt, int flag, const char **infile, const int *index, int n,
                    const char *outfile) {
  trace(3, "execses : n=%d outfile=%s\n", n, outfile);

  rtk_t *rtk_ptr =
      (rtk_t *)malloc(sizeof(rtk_t)); /* Moved from stack to heap to avoid stack overflow warning */
  prcopt_t popt_ = *popt;
  solopt_t tmsopt = *sopt;

  /* Open debug trace */
  if (flag && sopt->trace > 0) {
    char tracefile[FNSIZE];
    if (*outfile) {
      rtkstrcpy(tracefile, sizeof(tracefile), outfile);
      rtkstrcat(tracefile, sizeof(tracefile), ".trace");
    } else {
      rtkstrcpy(tracefile, sizeof(tracefile), fopt->trace);
    }
    traceclose();
    traceopen(tracefile);
    tracelevel(sopt->trace);
  }
  /* Read ionosphere data file */
  const char *ext;
  if (*fopt->iono && (ext = strrchr(fopt->iono, '.'))) {
    if (strlen(ext) == 4 &&
        (ext[3] == 'i' || ext[3] == 'I' || strcmp(ext, ".INX") == 0 || strcmp(ext, ".inx") == 0)) {
      char path[FNSIZE];
      reppath(fopt->iono, path, sizeof(path), ts, "", "");
      readtec(path, &navs, 1);
    }
  }
  /* Read erp data */
  if (*fopt->eop) {
    free(navs.erp.data);
    navs.erp.data = NULL;
    navs.erp.n = navs.erp.nmax = 0;
    char path[FNSIZE];
    reppath(fopt->eop, path, sizeof(path), ts, "", "");
    if (!readerp(path, &navs.erp)) {
      showmsg("error : no erp data %s", path);
      trace(2, "no erp data %s\n", path);
    }
  }
  /* Read obs and nav data */
  if (!readobsnav(ts, te, ti, infile, index, n, &popt_, &obss, &navs, stas)) {
    /* Free obs and nav data */
    freeobsnav(&obss, &navs);
    free(rtk_ptr);
    return false;
  }

  /* Read dcb parameters from DCB, BIA, BSX files */
  for (int i = 0; i < MAX_CODE_BIASES; i++)
    for (int k = 0; k < MAX_CODE_BIAS_FREQS; k++) {
      for (int j = 0; j < MAXSAT; j++) navs.cbias[j][k][i] = -1;
      for (int j = 0; j < MAXRCV; j++) navs.rbias[j][k][i] = 0;
    }
  int dcb_ok = 0;
  for (int i = 0; i < n; i++) { /* First check infiles for .BIA or .BSX files */
    dcb_ok = readdcb(infile[i], &navs, stas);
    if (dcb_ok) break;
  }
  if (!dcb_ok && *fopt->dcb) { /* Then check if DCB file specified */
    char path[FNSIZE];
    reppath(fopt->dcb, path, sizeof(path), ts, "", "");
    dcb_ok = readdcb(path, &navs, stas);
  }
  if (!dcb_ok) {
  }
  /* Set antenna parameters */
  if (popt_.mode != PMODE_SINGLE) {
    setpcv(obss.n > 0 ? obss.data[0].time : timeget(), &popt_, &navs, &pcvss, &pcvsr, stas);
  }
  /* Read ocean tide loading parameters */
  if (popt_.mode > PMODE_SINGLE && *fopt->blq) {
    readotl(&popt_, fopt->blq, stas);
  }
  /* Rover/reference fixed position */
  if (popt_.mode == PMODE_FIXED) {
    if (!antpos(&popt_, 1, &obss, &navs, stas, fopt->stapos)) {
      freeobsnav(&obss, &navs);
      free(rtk_ptr);
      return false;
    }
    if (!antpos(&popt_, 2, &obss, &navs, stas, fopt->stapos)) {
      freeobsnav(&obss, &navs);
      free(rtk_ptr);
      return false;
    }
  } else if (PMODE_DGPS <= popt_.mode && popt_.mode <= PMODE_STATIC_START) {
    if (!antpos(&popt_, 2, &obss, &navs, stas, fopt->stapos)) {
      freeobsnav(&obss, &navs);
      free(rtk_ptr);
      return false;
    }
  }
  /* Open solution statistics */
  if (flag && sopt->sstat > 0) {
    char statfile[FNSIZE];
    rtkstrcpy(statfile, sizeof(statfile), outfile);
    rtkstrcat(statfile, sizeof(statfile), ".stat");
    rtkclosestat();
    rtkopenstat(statfile, sopt->sstat);
  }
  /* Write header to output file */
  if (flag && !outhead(outfile, infile, n, &popt_, sopt)) {
    freeobsnav(&obss, &navs);
    free(rtk_ptr);
    return false;
  }
  /* Name time events file */
  char outfiletm[FNSIZE] = {0};
  namefiletm(outfiletm, sizeof(outfiletm), outfile);
  /* Write header to file with time marks */
  outhead(outfiletm, infile, n, &popt_, &tmsopt);

  iobsu = iobsr = isbs = reverse = aborts = 0;

  if (popt_.mode == PMODE_SINGLE || popt_.soltype == SOLTYPE_FORWARD) {
    FILE *fp = openfile(outfile);
    if (fp) {
      FILE *fptm = openfile(outfiletm);
      if (fptm) {
        rtkinit(rtk_ptr, popt);
        procpos(fp, fptm, &popt_, sopt, rtk_ptr, SOLMODE_SINGLE_DIR);
        rtkfree(rtk_ptr);
        fclose(fptm);
      }
      fclose(fp);
    }
  } else if (popt_.soltype == SOLTYPE_BACKWARD) {
    FILE *fp = openfile(outfile);
    if (fp) {
      FILE *fptm = openfile(outfiletm);
      if (fptm) {
        reverse = 1;
        iobsu = iobsr = obss.n - 1;
        isbs = sbss.n - 1;
        rtkinit(rtk_ptr, popt);
        procpos(fp, fptm, &popt_, sopt, rtk_ptr, SOLMODE_SINGLE_DIR);
        rtkfree(rtk_ptr);
        fclose(fptm);
      }
      fclose(fp);
    }
  } else { /* Combined or combined with no phase reset */
    solf = (sol_t *)malloc(sizeof(sol_t) * nepoch);
    solb = (sol_t *)malloc(sizeof(sol_t) * nepoch);
    rbf = (double *)malloc(sizeof(double) * nepoch * 3);
    rbb = (double *)malloc(sizeof(double) * nepoch * 3);

    if (solf && solb) {
      isolf = isolb = 0;
      rtkinit(rtk_ptr, popt);
      procpos(NULL, NULL, &popt_, sopt, rtk_ptr, SOLMODE_COMBINED); /* Forward */
      reverse = 1;
      iobsu = iobsr = obss.n - 1;
      isbs = sbss.n - 1;
      if (popt_.soltype != SOLTYPE_COMBINED_NORESET) {
        /* Reset */
        rtkfree(rtk_ptr);
        rtkinit(rtk_ptr, popt);
      }
      procpos(NULL, NULL, &popt_, sopt, rtk_ptr, SOLMODE_COMBINED); /* Backward */
      rtkfree(rtk_ptr);

      /* Combine forward/backward solutions */
      if (!aborts) {
        FILE *fp = openfile(outfile);
        if (fp) {
          FILE *fptm = openfile(outfiletm);
          if (fptm) {
            combres(fp, fptm, &popt_, sopt);
            fclose(fptm);
          }
          fclose(fp);
        }
      }
    } else
      showmsg("error : memory allocation");
    free(solf);
    free(solb);
    free(rbf);
    free(rbb);
  }
  /* Free rtk, obs and nav data */
  free(rtk_ptr);
  freeobsnav(&obss, &navs);

  return aborts ? true : false;
}
/* Execute processing session for each rover ---------------------------------*/
static bool execses_r(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt, const solopt_t *sopt,
                      const filopt_t *fopt, int flag, const char **infile, const int *index, int n,
                      const char *outfile, const char *rov) {
  trace(3, "execses_r: n=%d outfile=%s\n", n, outfile);

  gtime_t t0 = {0};

  int i1;
  for (i1 = 0; i1 < n; i1++)
    if (strstr(infile[i1], "%r")) break;

  bool stat = false;
  if (i1 < n) { /* Include rover keywords */
    size_t rsize = strlen(rov) + 1;
    char *rov_ = (char *)malloc(rsize);
    if (!rov_) return false;
    rtkstrcpy(rov_, rsize, rov);

    char *ifile[MAXINFILE];
    for (int i = 0; i < n; i++) {
      if (!(ifile[i] = (char *)malloc(FNSIZE))) {
        free(rov_);
        for (; i >= 0; i--) free(ifile[i]);
        return false;
      }
    }

    char *q;
    for (const char *p = rov_;; p = q + 1) { /* For each rover */
      q = strchr(p, ' ');
      if (q) *q = '\0';

      if (*p) {
        rtkstrcpy(proc_rov, sizeof(proc_rov), p);
        char s[64] = "";
        if (ts.time)
          time2str(ts, s, 0);
        else
          *s = '\0';
        if (checkbrk("reading    : %s", s)) {
          stat = true;
          break;
        }
        for (int i = 0; i < n; i++) reppath(infile[i], ifile[i], FNSIZE, t0, p, "");
        char ofile[FNSIZE];
        reppath(outfile, ofile, sizeof(ofile), t0, p, "");

        /* Execute processing session */
        stat = execses(ts, te, ti, popt, sopt, fopt, flag, (const char **)ifile, index, n, ofile);
      }
      if (stat == true || !q) break;
    }
    free(rov_);
    for (int i = 0; i < n; i++) free(ifile[i]);
  } else {
    /* Execute processing session */
    stat = execses(ts, te, ti, popt, sopt, fopt, flag, infile, index, n, outfile);
  }
  return stat;
}
/* Execute processing session for each base station --------------------------*/
static bool execses_b(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt, const solopt_t *sopt,
                      const filopt_t *fopt, int flag, const char **infile, const int *index, int n,
                      const char *outfile, const char *rov, const char *base) {
  trace(3, "execses_b: n=%d outfile=%s\n", n, outfile);

  /* Read prec ephemeris and SBAS data */
  readpreceph(infile, n, popt, &navs, &sbss);

  gtime_t t0 = {0};

  int i;
  for (i = 0; i < n; i++)
    if (strstr(infile[i], "%b")) break;

  bool stat = false;
  if (i < n) { /* Include base station keywords */
    size_t bsize = strlen(base) + 1;
    char *base_ = (char *)malloc(bsize);
    if (!base_) {
      freepreceph(&navs, &sbss);
      return false;
    }
    rtkstrcpy(base_, bsize, base);

    char *ifile[MAXINFILE];
    for (int i2 = 0; i2 < n; i2++) {
      if (!(ifile[i2] = (char *)malloc(FNSIZE))) {
        free(base_);
        for (; i2 >= 0; i2--) free(ifile[i2]);
        freepreceph(&navs, &sbss);
        return false;
      }
    }
    char *q;
    for (const char *p = base_;; p = q + 1) { /* For each base station */
      q = strchr(p, ' ');
      if (q) *q = '\0';

      if (*p) {
        rtkstrcpy(proc_base, sizeof(proc_base), p);
        char s[40];
        if (ts.time)
          time2str(ts, s, 0);
        else
          *s = '\0';
        if (checkbrk("reading    : %s", s)) {
          stat = 1;
          break;
        }
        for (int i2 = 0; i2 < n; i2++) reppath(infile[i2], ifile[i2], FNSIZE, t0, "", p);
        char ofile[FNSIZE];
        reppath(outfile, ofile, sizeof(ofile), t0, "", p);

        stat = execses_r(ts, te, ti, popt, sopt, fopt, flag, (const char **)ifile, index, n,
                         (const char *)ofile, rov);
      }
      if (stat == true || !q) break;
    }
    free(base_);
    for (int i2 = 0; i2 < n; i2++) free(ifile[i2]);
  } else {
    stat = execses_r(ts, te, ti, popt, sopt, fopt, flag, infile, index, n, outfile, rov);
  }
  /* Free prec ephemeris and SBAS data */
  freepreceph(&navs, &sbss);

  return stat;
}
/* Post-processing positioning -------------------------------------------------
 * Post-processing positioning
 * Args   : gtime_t ts       I   processing start time (ts.time==0: no limit)
 *        : gtime_t te       I   processing end time   (te.time==0: no limit)
 *          double ti        I   processing interval  (s) (0:all)
 *          double tu        I   processing unit time (s) (0:all)
 *          prcopt_t *popt   I   processing options
 *          solopt_t *sopt   I   solution options
 *          filopt_t *fopt   I   file options
 *          char   **infile  I   input files (see below)
 *          int    n         I   number of input files
 *          char   *outfile  I   output file ("":stdout, see below)
 *          char   *rov      I   rover id list        (separated by " ")
 *          char   *base     I   base station id list (separated by " ")
 * Return : status (0:ok,0>:error,1:aborted)
 * Notes  : input files should contain observation data, navigation data, precise
 *          ephemeris/clock (optional), SBAS log file (optional), SSR message
 *          log file (optional) and TEC grid file (optional). only the first
 *          observation data file in the input files is recognized as the rover
 *          data.
 *
 *          the type of an input file is recognized by the file extension as ]
 *          follows:
 *              .sp3,.SP3,.eph*,.EPH*: precise ephemeris (sp3c)
 *              .sbs,.SBS,.ems,.EMS  : SBAS message log files (RTKLIB or EMS)
 *              .rtcm3,.RTCM3        : SSR message log files (RTCM3)
 *              .*i,.*I              : TEC grid files (ionex)
 *              others               : RINEX obs, nav, gnav, hnav, qnav or clock
 *
 *          inputs files can include wild-cards (*). if an file includes
 *          wild-cards, the wild-card expanded multiple files are used.
 *
 *          inputs files can include keywords. if an file includes keywords,
 *          the keywords are replaced by date, time, rover id and base station
 *          id and multiple session analyses run. refer reppath() for the
 *          keywords.
 *
 *          the output file can also include keywords. if the output file does
 *          not include keywords. the results of all multiple session analyses
 *          are output to a single output file.
 *
 *          SSR corrections are valid only for forward estimation.
 *----------------------------------------------------------------------------*/
extern int postpos(gtime_t ts, gtime_t te, double ti, double tu, const prcopt_t *popt,
                   const solopt_t *sopt, const filopt_t *fopt, const char **infile, int n,
                   const char *outfile, const char *rov, const char *base) {
  trace(3, "postpos : ti=%.0f tu=%.0f n=%d outfile=%s\n", ti, tu, n, outfile);

  int index[MAXINFILE] = {0};
  bool stat = false;
  char *ifile[MAXINFILE], ofile[FNSIZE];

  /* Open processing session */
  if (!openses(popt, sopt, fopt, &navs, &pcvss, &pcvsr)) return -1;

  if (ts.time != 0 && te.time != 0 && tu >= 0.0) {
    if (timediff(te, ts) < 0.0) {
      showmsg("error : no period");
      closeses(&navs, &pcvss, &pcvsr);
      return 0;
    }
    for (int i = 0; i < MAXINFILE; i++) {
      if (!(ifile[i] = (char *)malloc(FNSIZE))) {
        for (; i >= 0; i--) free(ifile[i]);
        closeses(&navs, &pcvss, &pcvsr);
        return -1;
      }
    }
    if (tu == 0.0 || tu > 86400.0 * MAXPRCDAYS) tu = 86400.0 * MAXPRCDAYS;
    settspan(ts, te);
    double tunit = tu < 86400.0 ? tu : 86400.0;
    int week;
    double tss = tunit * (int)floor(time2gpst(ts, &week) / tunit);
    int flag = 1;

    for (int i = 0;; i++) { /* For each periods */
      gtime_t tts = gpst2time(week, tss + i * tu);
      gtime_t tte = timeadd(tts, tu - DTTOL);
      if (timediff(tts, te) > 0.0) break;
      if (timediff(tts, ts) < 0.0) tts = ts;
      if (timediff(tte, te) > 0.0) tte = te;

      rtkstrcpy(proc_rov, sizeof(proc_rov), "");
      rtkstrcpy(proc_base, sizeof(proc_base), "");
      char tstr[40];
      if (checkbrk("reading    : %s", time2str(tts, tstr, 0))) {
        stat = true;
        break;
      }
      int k = 0, nf = 0;
      for (int j = 0; j < n; j++) {
        const char *ext = strrchr(infile[j], '.');

        if (ext && (!strcmp(ext, ".rtcm3") || !strcmp(ext, ".RTCM3"))) {
          rtkstrcpy(ifile[nf++], FNSIZE, infile[j]);
        } else {
          /* Include next day precise ephemeris or RINEX brdc nav */
          gtime_t ttte = tte;
          if (ext && (!strcmp(ext, ".sp3") || !strcmp(ext, ".SP3") || !strcmp(ext, ".eph") ||
                      !strcmp(ext, ".EPH"))) {
            ttte = timeadd(ttte, 3600.0);
          } else if (strstr(infile[j], "brdc")) {
            ttte = timeadd(ttte, 7200.0);
          }
          nf += reppaths(infile[j], ifile + nf, FNSIZE, MAXINFILE - nf, tts, ttte, "", "");
        }
        while (k < nf) index[k++] = j;

        if (nf >= MAXINFILE) {
          trace(2, "too many input files. trancated\n");
          break;
        }
      }
      if (!reppath(outfile, ofile, sizeof(ofile), tts, "", "") && i > 0) flag = 0;

      /* Execute processing session */
      stat = execses_b(tts, tte, ti, popt, sopt, fopt, flag, (const char **)ifile, index, nf,
                       (const char *)ofile, rov, base);

      if (stat == true) break;
    }
    for (int i = 0; i < n && i < MAXINFILE; i++) free(ifile[i]);
  } else if (ts.time != 0) {
    for (int i = 0; i < n && i < MAXINFILE; i++) {
      if (!(ifile[i] = (char *)malloc(FNSIZE))) {
        for (; i >= 0; i--) free(ifile[i]);
        return -1;
      }
      reppath(infile[i], ifile[i], FNSIZE, ts, "", "");
      index[i] = i;
    }
    reppath(outfile, ofile, sizeof(ofile), ts, "", "");

    /* Execute processing session */
    stat = execses_b(ts, te, ti, popt, sopt, fopt, 1, (const char **)ifile, index, n, ofile, rov,
                     base);

    for (int i = 0; i < n && i < MAXINFILE; i++) free(ifile[i]);
  } else {
    for (int i = 0; i < n; i++) index[i] = i;

    /* Execute processing session */
    stat = execses_b(ts, te, ti, popt, sopt, fopt, 1, infile, index, n, outfile, rov, base);
  }
  /* Close processing session */
  closeses(&navs, &pcvss, &pcvsr);

  return stat ? 1 : 0;
}
