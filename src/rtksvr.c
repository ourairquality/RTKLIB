/*------------------------------------------------------------------------------
 * rtksvr.c : rtk server functions
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Options : -DWIN32    use WIN32 API
 *
 * Version : $Revision:$ $Date:$
 * History : 2009/01/07  1.0  new
 *           2009/06/02  1.1  support GLONASS
 *           2010/07/25  1.2  support correction input/log stream
 *                            supoort online change of output/log streams
 *                            supoort monitor stream
 *                            added api:
 *                                rtksvropenstr(),rtksvrclosestr()
 *                            changed api:
 *                                rtksvrstart()
 *           2010/08/25  1.3  fix problem of ephemeris time inversion (2.4.0_p6)
 *           2010/09/08  1.4  fix problem of ephemeris and SSR squence upset
 *                            (2.4.0_p8)
 *           2011/01/10  1.5  change api: rtksvrstart(),rtksvrostat()
 *           2011/06/21  1.6  fix ephemeris handover problem
 *           2012/05/14  1.7  fix bugs
 *           2013/03/28  1.8  fix problem on lack of GLONASS freq number in raw
 *                            fix problem on ephemeris with inverted toe
 *                            add api rtksvrfree()
 *           2014/06/28  1.9  fix probram on ephemeris update of BeiDou
 *           2015/04/29  1.10 fix probram on SSR orbit/clock inconsistency
 *           2015/07/31  1.11 add phase bias (fcb) correction
 *           2015/12/05  1.12 support opt->pppopt=-DIS_FCB
 *           2016/07/01  1.13 support averaging single pos as base position
 *           2016/07/31  1.14 fix bug on ion/UTC parameters input
 *           2016/08/20  1.15 support api change of sendnmea()
 *           2016/09/18  1.16 fix server-crash with server-cycle > 1000
 *           2016/09/20  1.17 change api rtksvrstart()
 *           2016/10/01  1.18 change api rtksvrstart()
 *           2016/10/04  1.19 fix problem to send NMEA of single solution
 *           2016/10/09  1.20 add reset-and-single-sol mode for nmea-request
 *           2017/04/11  1.21 add rtkfree() in rtksvrfree()
 *           2020/11/30  1.22 add initializing svr->nav in rtksvrinit()
 *                            allocate double size ephemeris in rtksvrinit()
 *                            handle multiple ephemeris sets in updatesvr()
 *                            use API sat2freq() to get carrier frequency
 *                            use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

#define MIN_INT_RESET 30000 /* Mininum interval of reset command (ms) */

/* Write solution header to output stream ------------------------------------*/
static void writesolhead(stream_t *stream, const solopt_t *solopt) {
  uint8_t buff[1024];
  outsolheads((char *)buff, sizeof(buff), solopt);
  strwrite(stream, buff, strlen((char *)buff));
}
/* Save output buffer --------------------------------------------------------*/
static void saveoutbuf(rtksvr_t *svr, const uint8_t *buff, int n, int index) {
  rtksvrlock(svr);

  n = n < svr->buffsize - svr->nsb[index] ? n : svr->buffsize - svr->nsb[index];
  memcpy(svr->sbuf[index] + svr->nsb[index], buff, n);
  svr->nsb[index] += n;

  rtksvrunlock(svr);
}
/* Write solution to output stream -------------------------------------------*/
static void writesol(rtksvr_t *svr, int index) {
  tracet(4, "writesol: index=%d\n", index);

  for (int i = 0; i < 2; i++) {
    uint8_t buff[MAXSOLMSG + 1];
    buff[0] = '\0';
    if (svr->solopt[i].posf == SOLF_STAT) {
      /* Output solution status */
      rtksvrlock(svr);
      rtkoutstat(&svr->rtk, svr->solopt[i].sstat, (char *)buff, sizeof(buff));
      rtksvrunlock(svr);
    } else {
      /* Output solution */
      outsols((char *)buff, sizeof(buff), &svr->rtk.sol, svr->rtk.rb, svr->solopt + i);
    }
    int n = strlen((char *)buff);
    strwrite(svr->stream + i + 3, buff, n);

    /* Save output buffer */
    saveoutbuf(svr, buff, n, i);

    /* Output extended solution */
    buff[0] = '\0';
    outsolexs((char *)buff, sizeof(buff), &svr->rtk.sol, svr->rtk.ssat, svr->solopt + i);
    n = strlen((char *)buff);
    strwrite(svr->stream + i + 3, buff, n);

    /* Save output buffer */
    saveoutbuf(svr, buff, n, i);
  }
  /* Output solution to monitor port */
  if (svr->moni) {
    solopt_t solopt = solopt_default;
    uint8_t buff[MAXSOLMSG + 1];
    buff[0] = '\0';
    outsols((char *)buff, sizeof(buff), &svr->rtk.sol, svr->rtk.rb, &solopt);
    int n = strlen((char *)buff);
    strwrite(svr->moni, buff, n);
  }
  /* Save solution buffer */
  if (svr->nsol < MAXSOLBUF) {
    rtksvrlock(svr);
    svr->solbuf[svr->nsol++] = svr->rtk.sol;
    rtksvrunlock(svr);
  }
}
/* Update GLONASS frequency channel number in raw data struct ----------------*/
static void update_glofcn(rtksvr_t *svr) {
  for (int i = 0; i < MAXPRNGLO; i++) {
    int sat = satno(SYS_GLO, i + 1);

    int frq = -999;
    for (int j = 0; j < 3; j++) {
      if (svr->raw[j].nav.geph[i][0].sat != sat) continue;
      frq = svr->raw[j].nav.geph[i][0].frq;
    }
    if (frq < -7 || frq > 6) continue;

    for (int j = 0; j < 3; j++) {
      if (svr->raw[j].nav.geph[i][0].sat == sat) continue;
      svr->raw[j].nav.geph[i][0].sat = sat;
      svr->raw[j].nav.geph[i][0].frq = frq;
    }
  }
}
/* Update observation data ---------------------------------------------------*/
static void update_obs(rtksvr_t *svr, obs_t *obs, int index, int iobs) {
  if (iobs < MAXOBSBUF) {
    int n = 0;
    for (int i = 0; i < obs->n; i++) {
      int sat = obs->data[i].sat;
      int sys = satsys(sat, NULL);
      if (svr->rtk.opt.exsats[sat - 1] == 1 || !(sys & svr->rtk.opt.navsys)) {
        continue;
      }
      svr->obs[index][iobs].data[n] = obs->data[i];
      svr->obs[index][iobs].data[n++].rcv = index + 1;
    }
    svr->obs[index][iobs].n = n;
    sortobs(&svr->obs[index][iobs]);
  }
  svr->nmsg[index][0]++;
}
/* Update ephemeris ----------------------------------------------------------*/
static void update_eph(rtksvr_t *svr, nav_t *nav, int ephsat, int ephset, int index) {
  int prn;
  if (satsys(ephsat, &prn) != SYS_GLO) {
    if (!svr->navsel || svr->navsel == index + 1) {
      /* svr->nav.eph={current_set1,current_set2,prev_set1,prev_set2} */
      eph_t *eph1 = nav->eph[ephsat - 1] + ephset;         /* Received */
      eph_t *eph2 = svr->nav.eph[ephsat - 1] + ephset;     /* Current */
      eph_t *eph3 = svr->nav.eph[ephsat - 1] + 2 + ephset; /* Previous */
      if (eph2->ttr.time == 0 || (eph1->iode != eph3->iode && eph1->iode != eph2->iode) ||
          (timediff(eph1->toe, eph3->toe) != 0.0 && timediff(eph1->toe, eph2->toe) != 0.0) ||
          (timediff(eph1->toc, eph3->toc) != 0.0 && timediff(eph1->toc, eph2->toc) != 0.0)) {
        *eph3 = *eph2; /* Current ->previous */
        *eph2 = *eph1; /* Received->current */
      }
    }
    svr->nmsg[index][1]++;
  } else {
    if (!svr->navsel || svr->navsel == index + 1) {
      geph_t *geph1 = nav->geph[prn - 1];
      geph_t *geph2 = svr->nav.geph[prn - 1];
      geph_t *geph3 = svr->nav.geph[prn - 1] + 1;
      if (geph2->tof.time == 0 || (geph1->iode != geph3->iode && geph1->iode != geph2->iode)) {
        *geph3 = *geph2;
        *geph2 = *geph1;
        update_glofcn(svr);
      }
    }
    svr->nmsg[index][6]++;
  }
}
/* Update SBAS message -------------------------------------------------------*/
static void update_sbs(rtksvr_t *svr, sbsmsg_t *sbsmsg, int index) {
  int sbssat = svr->rtk.opt.sbassatsel;
  if (sbsmsg && (sbssat == sbsmsg->prn || sbssat == 0)) {
    sbsmsg->rcv = index + 1;
    if (svr->nsbs < MAXSBSMSG) {
      svr->sbsmsg[svr->nsbs++] = *sbsmsg;
    } else {
      int i = 0;
      for (; i < MAXSBSMSG - 1; i++) svr->sbsmsg[i] = svr->sbsmsg[i + 1];
      svr->sbsmsg[i] = *sbsmsg;
    }
    sbsupdatecorr(sbsmsg, &svr->nav);
  }
  svr->nmsg[index][3]++;
}
/* Update ion/UTC parameters -------------------------------------------------*/
static void update_ionutc(rtksvr_t *svr, const nav_t *nav, int index) {
  if (svr->navsel == 0 || svr->navsel == index + 1) {
    matcpy(svr->nav.utc_gps, nav->utc_gps, 8, 1);
    matcpy(svr->nav.utc_glo, nav->utc_glo, 8, 1);
    matcpy(svr->nav.utc_gal, nav->utc_gal, 8, 1);
    matcpy(svr->nav.utc_qzs, nav->utc_qzs, 8, 1);
    matcpy(svr->nav.utc_cmp, nav->utc_cmp, 8, 1);
    matcpy(svr->nav.utc_irn, nav->utc_irn, 9, 1);
    matcpy(svr->nav.utc_sbs, nav->utc_sbs, 4, 1);
    matcpy(svr->nav.ion_gps, nav->ion_gps, 8, 1);
    matcpy(svr->nav.ion_gal, nav->ion_gal, 4, 1);
    matcpy(svr->nav.ion_qzs, nav->ion_qzs, 8, 1);
    matcpy(svr->nav.ion_cmp, nav->ion_cmp, 8, 1);
    matcpy(svr->nav.ion_irn, nav->ion_irn, 8, 1);
  }
  svr->nmsg[index][2]++;
}
/* Update antenna position ---------------------------------------------------*/
static void update_antpos(rtksvr_t *svr, int index) {
  if (svr->rtk.opt.refpos == POSOPT_RTCM && index == 1) {
    sta_t *sta;
    if (svr->format[1] == STRFMT_RTCM2 || svr->format[1] == STRFMT_RTCM3) {
      sta = &svr->rtcm[1].sta;
    } else {
      sta = &svr->raw[1].sta;
    }
    /* Update base station position */
    for (int i = 0; i < 3; i++) {
      svr->rtk.rb[i] = sta->pos[i];
    }
    /* Antenna delta */
    double pos[3];
    ecef2pos(svr->rtk.rb, pos);
    if (sta->deltype) { /* XYZ */
      double del[3] = {0};
      del[2] = sta->hgt;
      double dr[3];
      enu2ecef(pos, del, dr);
      for (int i = 0; i < 3; i++) {
        svr->rtk.rb[i] += sta->del[i] + dr[i];
      }
    } else { /* ENU */
      double dr[3];
      enu2ecef(pos, sta->del, dr);
      for (int i = 0; i < 3; i++) {
        svr->rtk.rb[i] += dr[i];
      }
    }
  }
  svr->nmsg[index][4]++;
}
/* Update SSR corrections ----------------------------------------------------*/
static void update_ssr(rtksvr_t *svr, int index) {
  for (int i = 0; i < MAXSAT; i++) {
    if (!svr->rtcm[index].ssr[i].update) continue;

    /* Check consistency between iods of orbit and clock */
    if (svr->rtcm[index].ssr[i].iod[0] != svr->rtcm[index].ssr[i].iod[1]) {
      continue;
    }
    svr->rtcm[index].ssr[i].update = 0;

    int iode = svr->rtcm[index].ssr[i].iode;
    int prn;
    int sys = satsys(i + 1, &prn);

    /* Check corresponding ephemeris exists */
    // CCMP??
    if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS) {
      if (svr->nav.eph[i][0].iode != iode && svr->nav.eph[i][1].iode != iode) {
        continue;
      }
    } else if (sys == SYS_GLO) {
      if (svr->nav.geph[prn - 1][0].iode != iode &&
          svr->nav.geph[prn - 1 + MAXPRNGLO][0].iode != iode) {
        continue;
      }
    }
    svr->nav.ssr[i] = svr->rtcm[index].ssr[i];
  }
  svr->nmsg[index][7]++;
}
/* Update rtk server struct --------------------------------------------------*/
static void update_svr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int ephsat, int ephset,
                       sbsmsg_t *sbsmsg, int index, int iobs) {
  tracet(4, "updatesvr: ret=%d ephsat=%d ephset=%d index=%d\n", ret, ephsat, ephset, index);

  if (ret == 1) { /* Observation data */
    update_obs(svr, obs, index, iobs);
  } else if (ret == 2) { /* Ephemeris */
    update_eph(svr, nav, ephsat, ephset, index);
  } else if (ret == 3) { /* SBAS message */
    update_sbs(svr, sbsmsg, index);
  } else if (ret == 9) { /* Ion/UTC parameters */
    update_ionutc(svr, nav, index);
  } else if (ret == 5) { /* Antenna position */
    update_antpos(svr, index);
  } else if (ret == 7) { /* Dgps correction */
    svr->nmsg[index][5]++;
  } else if (ret == 10) { /* SSR message */
    update_ssr(svr, index);
  } else if (ret == -1) { /* Error */
    svr->nmsg[index][9]++;
  }
}
/* Decode receiver raw/RTCM data ---------------------------------------------*/
static int decoderaw(rtksvr_t *svr, int index) {
  tracet(4, "decoderaw: index=%d\n", index);

  rtksvrlock(svr);

  int fobs = 0;
  for (int i = 0; i < svr->nb[index]; i++) {
    /* Input RTCM/receiver raw data from stream */
    obs_t *obs;
    nav_t *nav;
    int ret, ephsat, ephset;
    sbsmsg_t *sbsmsg = NULL;
    if (svr->format[index] == STRFMT_RTCM2) {
      ret = input_rtcm2(svr->rtcm + index, svr->buff[index][i]);
      obs = &svr->rtcm[index].obs;
      nav = &svr->rtcm[index].nav;
      ephsat = svr->rtcm[index].ephsat;
      ephset = svr->rtcm[index].ephset;
    } else if (svr->format[index] == STRFMT_RTCM3) {
      ret = input_rtcm3(svr->rtcm + index, svr->buff[index][i]);
      obs = &svr->rtcm[index].obs;
      nav = &svr->rtcm[index].nav;
      ephsat = svr->rtcm[index].ephsat;
      ephset = svr->rtcm[index].ephset;
    } else {
      ret = input_raw(svr->raw + index, svr->format[index], svr->buff[index][i]);
      obs = &svr->raw[index].obs;
      nav = &svr->raw[index].nav;
      ephsat = svr->raw[index].ephsat;
      ephset = svr->raw[index].ephset;
      sbsmsg = &svr->raw[index].sbsmsg;
    }
#ifdef RTK_DISABLED /* Record for receiving tick for debug */
    if (ret == 1) {
      char tstr[40];
      trace(0, "%d %10d T=%s NS=%2d\n", index, tickget(), time2str(obs->data[0].time, tstr, 0),
            obs->n);
    }
#endif
    /* Update rtk server */
    if (ret > 0) {
      update_svr(svr, ret, obs, nav, ephsat, ephset, sbsmsg, index, fobs);
    }
    /* Observation data received */
    if (ret == 1) {
      if (fobs < MAXOBSBUF)
        fobs++;
      else
        svr->prcout++;
    }
  }
  svr->nb[index] = 0;

  rtksvrunlock(svr);

  return fobs;
}
/* Decode download file ------------------------------------------------------*/
static void decodefile(rtksvr_t *svr, int index) {
  tracet(4, "decodefile: index=%d\n", index);

  rtksvrlock(svr);

  /* Check file path completed */
  int nb = svr->nb[index];
  if (nb <= 2 || svr->buff[index][nb - 2] != '\r' || svr->buff[index][nb - 1] != '\n') {
    rtksvrunlock(svr);
    return;
  }
  char file[FNSIZE];
  strncpy(file, (char *)svr->buff[index], nb - 2);
  file[nb - 2] = '\0';
  svr->nb[index] = 0;

  rtksvrunlock(svr);

  if (svr->format[index] == STRFMT_SP3) { /* Precise ephemeris */

    /* Read sp3 precise ephemeris */
    nav_t nav = {0};
    readsp3(file, &nav, 0);
    if (nav.ne <= 0) {
      tracet(1, "sp3 file read error: %s\n", file);
      return;
    }
    /* Update precise ephemeris */
    rtksvrlock(svr);

    if (svr->nav.peph) free(svr->nav.peph);
    svr->nav.ne = svr->nav.nemax = nav.ne;
    svr->nav.peph = nav.peph;
    svr->ftime[index] = utc2gpst(timeget());
    rtkstrcpy(svr->files[index], sizeof(svr->files[0]), file);

    rtksvrunlock(svr);
  } else if (svr->format[index] == STRFMT_RNXCLK) { /* Precise clock */

    /* Read RINEX clock */
    nav_t nav = {0};
    if (readrnxc(file, &nav) <= 0) {
      tracet(1, "rinex clock file read error: %s\n", file);
      return;
    }
    /* Update precise clock */
    rtksvrlock(svr);

    if (svr->nav.pclk) free(svr->nav.pclk);
    svr->nav.nc = svr->nav.ncmax = nav.nc;
    svr->nav.pclk = nav.pclk;
    svr->ftime[index] = utc2gpst(timeget());
    rtkstrcpy(svr->files[index], sizeof(svr->files[0]), file);

    rtksvrunlock(svr);
  }
}
/* Carrier-phase bias (fcb) correction ---------------------------------------*/
static void corr_phase_bias(obsd_t *obs, int n, const nav_t *nav) {
  for (int i = 0; i < n; i++)
    for (int j = 0; j < NFREQ; j++) {
      uint8_t code = obs[i].code[j];
      double freq = sat2freq(obs[i].sat, code, nav);
      if (freq == 0.0) continue;

      /* Correct phase bias (cyc) */
      obs[i].L[j] -= nav->ssr[obs[i].sat - 1].pbias[code - 1] * freq / CLIGHT;
    }
}
/* Periodic command ----------------------------------------------------------*/
static void periodic_cmd(int cycle, const char *cmd, stream_t *stream) {
  for (const char *p = cmd, *q;; p = q + 1) {
    for (q = p;; q++)
      if (*q == '\r' || *q == '\n' || *q == '\0') break;
    int n = (int)(q - p);
    char msg[1024];
    strncpy(msg, p, n);
    msg[n] = '\0';

    int period = 0;
    char *r = strrchr(msg, '#');
    if (r) {
      sscanf(r, "# %d", &period);
      *r = '\0';
      while (*--r == ' ') *r = '\0'; /* Delete tail spaces */
    }
    if (period <= 0) period = 1000;
    if (*msg && cycle % period == 0) {
      strsendcmd(stream, msg);
    }
    if (!*q) break;
  }
}
/* Baseline length -----------------------------------------------------------*/
static double baseline_len(const rtk_t *rtk) {
  if (norm(rtk->sol.rr, 3) <= 0.0 || norm(rtk->rb, 3) <= 0.0) return 0.0;

  double dr[3];
  for (int i = 0; i < 3; i++) {
    dr[i] = rtk->sol.rr[i] - rtk->rb[i];
  }
  return norm(dr, 3) * 0.001; /* (km) */
}
/* Send NMEA request to base/nrtk input stream -------------------------------*/
static void send_nmea(rtksvr_t *svr, uint32_t *tickreset) {
  if (svr->stream[1].state != 1) return;

  sol_t sol_nmea = {{0}};
  sol_nmea.ns = 10; /* Some servers don't like when ns = 0 */

  if (svr->nmeareq == 1) { /* lat-lon-hgt mode */
    sol_nmea.stat = SOLQ_SINGLE;
    sol_nmea.time = utc2gpst(timeget());
    matcpy(sol_nmea.rr, svr->nmeapos, 3, 1);
    strsendnmea(svr->stream + 1, &sol_nmea);
  } else if (svr->nmeareq == 2) { /* Single-solution mode */
    if (norm(svr->rtk.sol.rr, 3) <= 0.0) return;
    sol_nmea.stat = SOLQ_SINGLE;
    sol_nmea.time = utc2gpst(timeget());
    matcpy(sol_nmea.rr, svr->rtk.sol.rr, 3, 1);
    strsendnmea(svr->stream + 1, &sol_nmea);
  } else if (svr->nmeareq == 3) { /* Reset-and-single-sol mode */

    /* Send reset command if baseline over threshold */
    double bl = baseline_len(&svr->rtk);
    uint32_t tick = tickget();
    if (bl >= svr->bl_reset && (int)(tick - *tickreset) > MIN_INT_RESET) {
      strsendcmd(svr->stream + 1, svr->cmd_reset);

      tracet(2, "send reset: bl=%.3f rr=%.3f %.3f %.3f rb=%.3f %.3f %.3f\n", bl, svr->rtk.sol.rr[0],
             svr->rtk.sol.rr[1], svr->rtk.sol.rr[2], svr->rtk.rb[0], svr->rtk.rb[1],
             svr->rtk.rb[2]);
      *tickreset = tick;
    }
    if (norm(svr->rtk.sol.rr, 3) <= 0.0) return;
    sol_nmea.stat = SOLQ_SINGLE;
    sol_nmea.time = utc2gpst(timeget());
    matcpy(sol_nmea.rr, svr->rtk.sol.rr, 3, 1);

    /* Set predicted position if velocity > 36km/h */
    double vel = norm(svr->rtk.sol.rr + 3, 3);
    if (vel > 10.0) {
      for (int i = 0; i < 3; i++) {
        sol_nmea.rr[i] += svr->rtk.sol.rr[i + 3] / vel * svr->bl_reset * 0.8;
      }
    }
    strsendnmea(svr->stream + 1, &sol_nmea);

    tracet(3, "send nmea: rr=%.3f %.3f %.3f\n", sol_nmea.rr[0], sol_nmea.rr[1], sol_nmea.rr[2]);
  }
}
/* Rtk server thread ---------------------------------------------------------*/
#ifdef WIN32
static DWORD WINAPI rtksvrthread(void *arg)
#else
static void *rtksvrthread(void *arg)
#endif
{
  tracet(3, "rtksvrthread:\n");

  rtksvr_t *svr = (rtksvr_t *)arg;
  svr->state = 1;

  obs_t obs;
  obsd_t data[MAXOBS * 2];
  obs.data = data;
  svr->tick = tickget();
  uint32_t ticknmea = svr->tick - 1000;
  uint32_t tick1hz = ticknmea;
  uint32_t tickreset = svr->tick - MIN_INT_RESET;
  sol_t sol = {{0}};

  for (int cycle = 0; svr->state; cycle++) {
    uint32_t tick = tickget();
    for (int i = 0; i < 3; i++) {
      uint8_t *p = svr->buff[i] + svr->nb[i];
      uint8_t *q = svr->buff[i] + svr->buffsize;

      /* Read receiver raw/RTCM data from input stream */
      int n = strread(svr->stream + i, p, q - p);
      if (n <= 0) continue;

      /* Write receiver raw/RTCM data to log stream */
      strwrite(svr->stream + i + 5, p, n);
      svr->nb[i] += n;

      /* Save peek buffer */
      rtksvrlock(svr);
      n = n < svr->buffsize - svr->npb[i] ? n : svr->buffsize - svr->npb[i];
      memcpy(svr->pbuf[i] + svr->npb[i], p, n);
      svr->npb[i] += n;
      rtksvrunlock(svr);
    }
    int fobs[3] = {0};
    for (int i = 0; i < 3; i++) {
      if (svr->format[i] == STRFMT_SP3 || svr->format[i] == STRFMT_RNXCLK) {
        /* Decode download file */
        decodefile(svr, i);
      } else {
        /* Decode receiver raw/RTCM data */
        fobs[i] = decoderaw(svr, i);
        if (i == 1 && svr->rtcm[1].staid > 0) sol.refstationid = svr->rtcm[1].staid;
      }
    }
    /* Averaging single base pos */
    if (fobs[1] > 0 && svr->rtk.opt.refpos == POSOPT_SINGLE) {
      /* Messages are discarded */
      char msg[128];
      msg[0] = '\0';
      if ((svr->rtk.opt.maxaveep <= 0 || svr->nave < svr->rtk.opt.maxaveep) &&
          pntpos(svr->obs[1][0].data, svr->obs[1][0].n, &svr->nav, &svr->rtk.opt, &sol, NULL, NULL,
                 msg, sizeof(msg))) {
        svr->nave++;
        for (int i = 0; i < 3; i++) {
          svr->rb_ave[i] += (sol.rr[i] - svr->rb_ave[i]) / svr->nave;
        }
      }
      for (int i = 0; i < 3; i++) svr->rtk.opt.rb[i] = svr->rb_ave[i];
    }
    for (int i = 0; i < fobs[0]; i++) { /* For each rover observation data */
      obs.n = 0;
      for (int j = 0; j < svr->obs[0][i].n && obs.n < MAXOBS * 2; j++) {
        obs.data[obs.n++] = svr->obs[0][i].data[j];
      }
      for (int j = 0; j < svr->obs[1][0].n && obs.n < MAXOBS * 2; j++) {
        obs.data[obs.n++] = svr->obs[1][0].data[j];
      }
      /* Carrier phase bias correction */
      if (!strstr(svr->rtk.opt.pppopt, "-DIS_FCB")) {
        corr_phase_bias(obs.data, obs.n, &svr->nav);
      }
      /* Rtk positioning */
      rtksvrlock(svr);
      rtkpos(&svr->rtk, obs.data, obs.n, &svr->nav);
      rtksvrunlock(svr);

      if (svr->rtk.sol.stat != SOLQ_NONE) {
        /* Adjust current time */
        double tt = (int)(tickget() - tick) / 1000.0 + DTTOL;
        timeset(gpst2utc(timeadd(svr->rtk.sol.time, tt)));

        /* Write solution */
        writesol(svr, i);
      }
      /* If cpu overload, increment obs outage counter and break */
      if ((int)(tickget() - tick) >= svr->cycle) {
        svr->prcout += fobs[0] - i - 1;
      }
    }
    /* Send null solution if no solution (1hz) */
    if (svr->rtk.sol.stat == SOLQ_NONE && (int)(tick - tick1hz) >= 1000) {
      writesol(svr, 0);
      tick1hz = tick;
    }
    /* Write periodic command to input stream */
    for (int i = 0; i < 3; i++) {
      periodic_cmd(cycle * svr->cycle, svr->cmds_periodic[i], svr->stream + i);
    }
    /* Send NMEA request to base/nrtk input stream */
    if (svr->nmeacycle > 0 && (int)(tick - ticknmea) >= svr->nmeacycle) {
      send_nmea(svr, &tickreset);
      ticknmea = tick;
    }
    int cputime = (int)(tickget() - tick);
    if (cputime > 0) svr->cputime = cputime;

    /* Sleep until next cycle */
    sleepms(svr->cycle - cputime);
  }
  for (int i = 0; i < MAXSTRRTK; i++) strclose(svr->stream + i);
  for (int i = 0; i < 3; i++) {
    svr->nb[i] = svr->npb[i] = 0;
    free(svr->buff[i]);
    svr->buff[i] = NULL;
    free(svr->pbuf[i]);
    svr->pbuf[i] = NULL;
    free_raw(svr->raw + i);
    free_rtcm(svr->rtcm + i);
  }
  for (int i = 0; i < 2; i++) {
    svr->nsb[i] = 0;
    free(svr->sbuf[i]);
    svr->sbuf[i] = NULL;
  }
  return 0;
}
/* Initialize rtk server -------------------------------------------------------
 * Initialize rtk server
 * Args   : rtksvr_t *svr    IO rtk server
 * Return : status (true:error,false:ok)
 *----------------------------------------------------------------------------*/
extern bool rtksvrinit(rtksvr_t *svr) {
  tracet(3, "rtksvrinit:\n");

  svr->state = svr->cycle = svr->nmeacycle = svr->nmeareq = 0;
  for (int i = 0; i < 3; i++) svr->nmeapos[i] = 0.0;
  svr->buffsize = 0;
  for (int i = 0; i < 3; i++) svr->format[i] = 0;
  for (int i = 0; i < 2; i++) svr->solopt[i] = solopt_default;
  svr->navsel = svr->nsbs = svr->nsol = 0;
  rtkinit(&svr->rtk, &prcopt_default);
  for (int i = 0; i < 3; i++) svr->nb[i] = 0;
  for (int i = 0; i < 2; i++) svr->nsb[i] = 0;
  for (int i = 0; i < 3; i++) svr->npb[i] = 0;
  for (int i = 0; i < 3; i++) svr->buff[i] = NULL;
  for (int i = 0; i < 2; i++) svr->sbuf[i] = NULL;
  for (int i = 0; i < 3; i++) svr->pbuf[i] = NULL;
  sol_t sol0 = {{0}};
  for (int i = 0; i < MAXSOLBUF; i++) svr->solbuf[i] = sol0;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 10; j++) svr->nmsg[i][j] = 0;
  gtime_t time0 = {0};
  for (int i = 0; i < 3; i++) svr->ftime[i] = time0;
  for (int i = 0; i < 3; i++) svr->files[i][0] = '\0';
  svr->moni = NULL;
  svr->tick = 0;
  svr->thread = 0;
  svr->cputime = svr->prcout = svr->nave = 0;
  for (int i = 0; i < 3; i++) svr->rb_ave[i] = 0.0;

  memset(&svr->nav, 0, sizeof(nav_t));
  eph_t eph0 = {0, -1, -1};
  for (int i = 0; i < MAXSAT; i++) {
    if (!(svr->nav.eph[i] = (eph_t *)malloc(sizeof(eph_t) * 4))) {
      tracet(1, "rtksvrinit: malloc error\n");
      return false;
    }
    svr->nav.eph[i][0] = eph0;
    svr->nav.eph[i][1] = eph0;
    svr->nav.eph[i][2] = eph0;
    svr->nav.eph[i][3] = eph0;
    svr->nav.n[i] = svr->nav.nmax[i] = 4;
  }
  geph_t geph0 = {0, -1};
  for (int i = 0; i < NSATGLO; i++) {
    if (!(svr->nav.geph[i] = (geph_t *)malloc(sizeof(geph_t) * 2))) {
      tracet(1, "rtksvrinit: malloc error\n");
      return false;
    }
    svr->nav.geph[i][0] = geph0;
    svr->nav.geph[i][1] = geph0;
    svr->nav.ng[i] = svr->nav.ngmax[i] = 2;
  }
  seph_t seph0 = {0};
  for (int i = 0; i < NSATSBS; i++) {
    if (!(svr->nav.seph[i] = (seph_t *)malloc(sizeof(seph_t) * 2))) {
      tracet(1, "rtksvrinit: malloc error\n");
      return false;
    }
    svr->nav.seph[i][0] = seph0;
    svr->nav.seph[i][1] = seph0;
    svr->nav.ns[i] = svr->nav.nsmax[i] = 2;
  }

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < MAXOBSBUF; j++) {
      if (!(svr->obs[i][j].data = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS))) {
        tracet(1, "rtksvrinit: malloc error\n");
        return false;
      }
    }
  for (int i = 0; i < 3; i++) {
    memset(svr->raw + i, 0, sizeof(raw_t));
    memset(svr->rtcm + i, 0, sizeof(rtcm_t));
  }
  for (int i = 0; i < MAXSTRRTK; i++) strinit(svr->stream + i);

  for (int i = 0; i < 3; i++) *svr->cmds_periodic[i] = '\0';
  *svr->cmd_reset = '\0';
  svr->bl_reset = 10.0;
  rtklib_initlock(&svr->lock);

  return true;
}
/* Free rtk server -------------------------------------------------------------
 * Free rtk server
 * Args   : rtksvr_t *svr    IO rtk server
 * Return : none
 *----------------------------------------------------------------------------*/
extern void rtksvrfree(rtksvr_t *svr) {
  for (int i = 0; i < MAXSAT; i++) free(svr->nav.eph[i]);
  for (int i = 0; i < NSATGLO; i++) free(svr->nav.geph[i]);
  for (int i = 0; i < NSATSBS; i++) free(svr->nav.seph[i]);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < MAXOBSBUF; j++) {
      free(svr->obs[i][j].data);
    }
  rtkfree(&svr->rtk);
}
/* Lock/unlock rtk server ------------------------------------------------------
 * Lock/unlock rtk server
 * Args   : rtksvr_t *svr    IO rtk server
 * Return : none
 *----------------------------------------------------------------------------*/
extern void rtksvrlock(rtksvr_t *svr) { rtklib_lock(&svr->lock); }
extern void rtksvrunlock(rtksvr_t *svr) { rtklib_unlock(&svr->lock); }

/* Start rtk server ------------------------------------------------------------
 * Start rtk server thread
 * Args   : rtksvr_t *svr    IO rtk server
 *          int     cycle    I  server cycle (ms)
 *          int     buffsize I  input buffer size (bytes)
 *          int     *strs    I  stream types (STR_???)
 *                              types[0]=input stream rover
 *                              types[1]=input stream base station
 *                              types[2]=input stream correction
 *                              types[3]=output stream solution 1
 *                              types[4]=output stream solution 2
 *                              types[5]=log stream rover
 *                              types[6]=log stream base station
 *                              types[7]=log stream correction
 *          char    *paths   I  input stream paths
 *          int     *format  I  input stream formats (STRFMT_???)
 *                              format[0]=input stream rover
 *                              format[1]=input stream base station
 *                              format[2]=input stream correction
 *          int     navsel   I  navigation message select
 *                              (0:rover,1:base,2:ephem,3:all)
 *          char    **cmds   I  input stream start commands
 *                              cmds[0]=input stream rover (NULL: no command)
 *                              cmds[1]=input stream base (NULL: no command)
 *                              cmds[2]=input stream corr (NULL: no command)
 *          char    **cmds_periodic I input stream periodic commands
 *                              cmds[0]=input stream rover (NULL: no command)
 *                              cmds[1]=input stream base (NULL: no command)
 *                              cmds[2]=input stream corr (NULL: no command)
 *          char    **rcvopts I receiver options
 *                              rcvopt[0]=receiver option rover
 *                              rcvopt[1]=receiver option base
 *                              rcvopt[2]=receiver option corr
 *          int     nmeacycle I NMEA request cycle (ms) (0:no request)
 *          int     nmeareq  I  NMEA request type
 *                              (0:no,1:base pos,2:single sol,3:reset and single)
 *          double *nmeapos  I  transmitted NMEA position (ECEF) (m)
 *          prcopt_t *prcopt I  rtk processing options
 *          solopt_t *solopt I  solution options
 *                              solopt[0]=solution 1 options
 *                              solopt[1]=solution 2 options
 *          stream_t *moni   I  monitor stream (NULL: not used)
 *          char   *errmsg   O  error message
 *          size_t  size     I  error message buffer size
 * Return : status (true:ok false:error)
 *----------------------------------------------------------------------------*/
extern bool rtksvrstart(rtksvr_t *svr, int cycle, int buffsize, const int *strs, const char **paths,
                        const int *formats, int navsel, const char **cmds,
                        const char **cmds_periodic, const char **rcvopts, int nmeacycle,
                        int nmeareq, const double *nmeapos, const prcopt_t *prcopt,
                        const solopt_t *solopt, stream_t *moni, char *errmsg, size_t msize) {
  gtime_t time, time0 = {0};

  tracet(3, "rtksvrstart: cycle=%d buffsize=%d navsel=%d nmeacycle=%d nmeareq=%d\n", cycle,
         buffsize, navsel, nmeacycle, nmeareq);

  if (svr->state) {
    rtksnprintf(errmsg, msize, "server already started");
    return false;
  }
  strinitcom();
  svr->cycle = cycle > 1 ? cycle : 1;
  svr->nmeacycle = nmeacycle > 1000 ? nmeacycle : 1000;
  svr->nmeareq = nmeareq;
  for (int i = 0; i < 3; i++) svr->nmeapos[i] = nmeapos[i];
  svr->buffsize = buffsize > 4096 ? buffsize : 4096;
  for (int i = 0; i < 3; i++) svr->format[i] = formats[i];
  svr->navsel = navsel;
  svr->nsbs = 0;
  svr->nsol = 0;
  svr->prcout = 0;
  rtkfree(&svr->rtk);
  rtkinit(&svr->rtk, prcopt);

  if (prcopt->initrst) { /* Init averaging pos by restart */
    svr->nave = 0;
    for (int i = 0; i < 3; i++) svr->rb_ave[i] = 0.0;
  }
  for (int i = 0; i < 3; i++) { /* Input/log streams */
    svr->nb[i] = svr->npb[i] = 0;
    if (!(svr->buff[i] = (uint8_t *)malloc(buffsize)) ||
        !(svr->pbuf[i] = (uint8_t *)malloc(buffsize))) {
      tracet(1, "rtksvrstart: malloc error\n");
      rtksnprintf(errmsg, msize, "rtk server malloc error");
      return false;
    }
    for (int j = 0; j < 10; j++) svr->nmsg[i][j] = 0;
    for (int j = 0; j < MAXOBSBUF; j++) svr->obs[i][j].n = 0;
    rtkstrcpy(svr->cmds_periodic[i], sizeof(svr->cmds_periodic[0]),
              !cmds_periodic[i] ? "" : cmds_periodic[i]);

    /* Initialize receiver raw and RTCM control */
    init_raw(svr->raw + i, formats[i]);
    init_rtcm(svr->rtcm + i);

    /* Set receiver and RTCM option */
    rtkstrcpy(svr->raw[i].opt, sizeof(svr->raw[0].opt), rcvopts[i]);
    rtkstrcpy(svr->rtcm[i].opt, sizeof(svr->rtcm[0].opt), rcvopts[i]);

    /* Connect dgps corrections */
    svr->rtcm[i].dgps = svr->nav.dgps;
  }
  for (int i = 0; i < 2; i++) { /* Output peek buffer */
    if (!(svr->sbuf[i] = (uint8_t *)malloc(buffsize))) {
      tracet(1, "rtksvrstart: malloc error\n");
      rtksnprintf(errmsg, msize, "rtk server malloc error");
      return false;
    }
  }
  /* Set solution options */
  for (int i = 0; i < 2; i++) {
    svr->solopt[i] = solopt[i];
  }
  /* Set base station position */
  if (prcopt->refpos != POSOPT_SINGLE) {
    for (int i = 0; i < 6; i++) {
      svr->rtk.rb[i] = i < 3 ? prcopt->rb[i] : 0.0;
    }
  }
  /* Update navigation data */
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < 4; j++) svr->nav.eph[i][j].ttr = time0;
  for (int i = 0; i < NSATGLO; i++)
    for (int j = 0; j < 2; j++) svr->nav.geph[i][j].tof = time0;
  for (int i = 0; i < NSATSBS; i++)
    for (int j = 0; j < 2; j++) svr->nav.seph[i][j].tof = time0;

  /* Set monitor stream */
  svr->moni = moni;

  /* Open input streams */
  for (int i = 0; i < 8; i++) {
    enum str_mode rw = i < 3 ? STR_MODE_R : STR_MODE_W;
    if (strs[i] != STR_FILE) rw |= STR_MODE_W;
    if (!stropen(svr->stream + i, strs[i], rw, paths[i])) {
      rtksnprintf(errmsg, msize, "str%d open error path=%s", i + 1, paths[i]);
      for (i--; i >= 0; i--) strclose(svr->stream + i);
      return false;
    }
    /* Set initial time for RTCM and raw */
    if (i < 3) {
      time = utc2gpst(timeget());
      svr->raw[i].time = strs[i] == STR_FILE ? strgettime(svr->stream + i) : time;
      svr->rtcm[i].time = strs[i] == STR_FILE ? strgettime(svr->stream + i) : time;
    }
  }
  /* Sync input streams */
  strsync(svr->stream, svr->stream + 1);
  strsync(svr->stream, svr->stream + 2);

  /* Write start commands to input streams */
  for (int i = 0; i < 3; i++) {
    if (!cmds[i]) continue;
    strwrite(svr->stream + i, (unsigned char *)"", 0); /* For connect */
    sleepms(100);
    strsendcmd(svr->stream + i, cmds[i]);
  }
  /* Write solution header to solution streams */
  for (int i = 3; i < 5; i++) {
    writesolhead(svr->stream + i, svr->solopt + (i - 3));
  }
  /* Create rtk server thread */
#ifdef WIN32
  if (!(svr->thread = CreateThread(NULL, 0, rtksvrthread, svr, 0, NULL))) {
#else
  if (pthread_create(&svr->thread, NULL, rtksvrthread, svr)) {
#endif
    for (int i = 0; i < MAXSTRRTK; i++) strclose(svr->stream + i);
    rtksnprintf(errmsg, msize, "thread create error\n");
    return false;
  }
  return true;
}
/* Stop rtk server -------------------------------------------------------------
 * Start rtk server thread
 * Args   : rtksvr_t *svr    IO rtk server
 *          char    **cmds   I  input stream stop commands
 *                              cmds[0]=input stream rover (NULL: no command)
 *                              cmds[1]=input stream base  (NULL: no command)
 *                              cmds[2]=input stream ephem (NULL: no command)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void rtksvrstop(rtksvr_t *svr, const char **cmds) {
  tracet(3, "rtksvrstop:\n");

  /* Write stop commands to input streams */
  rtksvrlock(svr);
  for (int i = 0; i < 3; i++) {
    if (cmds[i]) strsendcmd(svr->stream + i, cmds[i]);
  }
  rtksvrunlock(svr);

  /* Stop rtk server */
  svr->state = 0;

  /* Free rtk server thread */
#ifdef WIN32
  WaitForSingleObject(svr->thread, 10000);
  CloseHandle(svr->thread);
#else
  pthread_join(svr->thread, NULL);
#endif
}
/* Open output/log stream ------------------------------------------------------
 * Open output/log stream
 * Args   : rtksvr_t *svr    IO rtk server
 *          int     index    I  output/log stream index
 *                              (3:solution 1,4:solution 2,5:log rover,
 *                               6:log base station,7:log correction)
 *          int     str      I  output/log stream types (STR_???)
 *          char    *path    I  output/log stream path
 *          solopt_t *solopt I  solution options
 * Return : status (true:ok false:error)
 *----------------------------------------------------------------------------*/
extern bool rtksvropenstr(rtksvr_t *svr, int index, int str, const char *path,
                          const solopt_t *solopt) {
  tracet(3, "rtksvropenstr: index=%d str=%d path=%s\n", index, str, path);

  if (index < 3 || index > 7 || !svr->state) return false;

  rtksvrlock(svr);

  if (svr->stream[index].state > 0) {
    rtksvrunlock(svr);
    return false;
  }
  if (!stropen(svr->stream + index, str, STR_MODE_W, path)) {
    tracet(2, "stream open error: index=%d\n", index);
    rtksvrunlock(svr);
    return false;
  }
  if (index <= 4) {
    svr->solopt[index - 3] = *solopt;

    /* Write solution header to solution stream */
    writesolhead(svr->stream + index, svr->solopt + (index - 3));
  }
  rtksvrunlock(svr);
  return true;
}
/* Close output/log stream -----------------------------------------------------
 * Close output/log stream
 * Args   : rtksvr_t *svr    IO rtk server
 *          int     index    I  output/log stream index
 *                              (3:solution 1,4:solution 2,5:log rover,
 *                               6:log base station,7:log correction)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void rtksvrclosestr(rtksvr_t *svr, int index) {
  tracet(3, "rtksvrclosestr: index=%d\n", index);

  if (index < 3 || index > 7 || !svr->state) return;

  rtksvrlock(svr);

  strclose(svr->stream + index);

  rtksvrunlock(svr);
}
/* Get observation data status -------------------------------------------------
 * Get current observation data status
 * Args   : rtksvr_t *svr    I  rtk server
 *          int     rcv      I  receiver (0:rover,1:base,2:ephem)
 *          gtime_t *time    O  time of observation data
 *          int     *sat     O  satellite prn numbers
 *          double  *az      O  satellite azimuth angles (rad)
 *          double  *el      O  satellite elevation angles (rad)
 *          int     **snr    O  satellite snr for each freq (dBHz)
 *                              snr[i][j] = sat i freq j snr
 *          int     *vsat    O  valid satellite flag
 * Return : number of satellites
 *----------------------------------------------------------------------------*/
extern int rtksvrostat(rtksvr_t *svr, int rcv, gtime_t *time, int *sat, double *az, double *el,
                       int **snr, int *vsat) {
  tracet(4, "rtksvrostat: rcv=%d\n", rcv);

  if (!svr->state) return 0;
  rtksvrlock(svr);
  int ns = svr->obs[rcv][0].n;
  if (ns > 0) {
    *time = svr->obs[rcv][0].data[0].time;
  }
  for (int i = 0; i < ns; i++) {
    sat[i] = svr->obs[rcv][0].data[i].sat;
    az[i] = svr->rtk.ssat[sat[i] - 1].azel[0];
    el[i] = svr->rtk.ssat[sat[i] - 1].azel[1];
    for (int j = 0; j < NFREQ; j++) {
      snr[i][j] = (int)(svr->obs[rcv][0].data[i].SNR[j] * SNR_UNIT + 0.5);
    }
    if (svr->rtk.sol.stat == SOLQ_NONE || svr->rtk.sol.stat == SOLQ_SINGLE) {
      vsat[i] = svr->rtk.ssat[sat[i] - 1].vs;
    } else {
      vsat[i] = svr->rtk.ssat[sat[i] - 1].vsat[0];
    }
  }
  rtksvrunlock(svr);
  return ns;
}
/* Get stream status -----------------------------------------------------------
 * Get current stream status
 * Args   : rtksvr_t *svr    I  rtk server
 *          int     *sstat   O  status of streams
 *          char    *msg     O  status messages
 *          size_t  size     I  status messages buffer size
 * Return : none
 * Note   : Messages are appended to msg which must be nul terminated.
 *----------------------------------------------------------------------------*/
extern void rtksvrsstat(rtksvr_t *svr, int *sstat, char *msg, size_t msize) {
  tracet(4, "rtksvrsstat:\n");

  rtksvrlock(svr);
  for (int i = 0; i < MAXSTRRTK; i++) {
    char s[MAXSTRMSG];
    s[0] = '\0';
    sstat[i] = strstat(svr->stream + i, s, sizeof(s));
    if (*s) rtkcatprintf(msg, msize, "(%d) %s ", i + 1, s);
  }
  rtksvrunlock(svr);
}
/* Mark current position -------------------------------------------------------
 * Open output/log stream
 * Args   : rtksvr_t *svr    IO rtk server
 *          char    *name    I  marker name
 *          char    *comment I  comment string
 * Return : status (true:ok false:error)
 *----------------------------------------------------------------------------*/
extern bool rtksvrmark(rtksvr_t *svr, const char *name, const char *comment) {
  tracet(4, "rtksvrmark:name=%s comment=%s\n", name, comment);

  if (!svr->state) return false;

  rtksvrlock(svr);

  char tstr[40];
  time2str(svr->rtk.sol.time, tstr, 3);
  int week;
  double tow = time2gpst(svr->rtk.sol.time, &week);
  double pos[3];
  ecef2pos(svr->rtk.sol.rr, pos);

  for (int i = 0; i < 2; i++) {
    char buff[MAXSOLMSG + 1];
    if (svr->solopt[i].posf == SOLF_STAT) {
      rtksnprintf(buff, sizeof(buff), "$MARK,%d,%.3f,%d,%.4f,%.4f,%.4f,%s,%s\r\n", week, tow,
                  svr->rtk.sol.stat, svr->rtk.sol.rr[0], svr->rtk.sol.rr[1], svr->rtk.sol.rr[2],
                  name, comment);
    } else if (svr->solopt[i].posf == SOLF_NMEA) {
      rtksnprintf(buff, sizeof(buff), "$GPTXT,01,01,02,MARK:%s,%s,%.9f,%.9f,%.4f,%d,%s", name, tstr,
                  pos[0] * R2D, pos[1] * R2D, pos[2], svr->rtk.sol.stat, comment);
      /* Check-sum */
      int sum = 0;
      for (char *q = (char *)buff + 1; *q; q++) sum ^= *q;
      rtkcatprintf(buff, sizeof(buff), "*%02X\r\n", sum);
    } else {
      rtksnprintf(buff, sizeof(buff), "%s MARK: %s,%s,%.9f,%.9f,%.4f,%d,%s\r\n", COMMENTH, name,
                  tstr, pos[0] * R2D, pos[1] * R2D, pos[2], svr->rtk.sol.stat, comment);
    }
    size_t len = strlen(buff);
    strwrite(svr->stream + i + 3, (uint8_t *)buff, (int)len);
    saveoutbuf(svr, (uint8_t *)buff, (int)len, i);
  }
  if (svr->moni) {
    char buff[MAXSOLMSG + 1];
    rtksnprintf(buff, sizeof(buff), "%s MARK: %s,%s,%.9f,%.9f,%.4f,%d,%s\r\n", COMMENTH, name, tstr,
                pos[0] * R2D, pos[1] * R2D, pos[2], svr->rtk.sol.stat, comment);
    strwrite(svr->moni, (uint8_t *)buff, (int)strlen(buff));
  }
  rtksvrunlock(svr);
  return true;
}
