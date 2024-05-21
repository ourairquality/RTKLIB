/*------------------------------------------------------------------------------
 * streamsvr.c : stream server functions
 *
 *          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
 *
 * Options : -DWIN32    use WIN32 API
 *
 * Version : $Revision:$ $Date:$
 * History : 2010/07/18 1.0  moved from stream.c
 *           2011/01/18 1.1  change api strsvrstart()
 *           2012/12/04 1.2  add stream conversion function
 *           2012/12/25 1.3  fix bug on cyclic navigation data output
 *                           suppress warnings
 *           2013/05/08 1.4  fix bug on 1 s offset for JAVAD -> RTCM conversion
 *           2014/10/16 1.5  support input from stdout
 *           2015/12/05 1.6  support RTCM 3 mt 63 BeiDou ephemeris
 *           2016/07/23 1.7  change api strsvrstart(),strsvrstop()
 *                           support command for output streams
 *           2016/08/20 1.8  support api change of sendnmea()
 *           2016/09/03 1.9  support NTRIP caster function
 *           2016/09/06 1.10 add api strsvrsetsrctbl()
 *           2016/09/17 1.11 add relay back function of output stream
 *                           fix bug on RTCM cyclic output of BeiDou ephemeris
 *           2016/10/01 1.12 change api startstrserver()
 *           2017/04/11 1.13 fix bug on search of next satellite in nextsat()
 *           2018/11/05 1.14 update message type of BeiDou ephemeirs
 *                           support multiple MSM messages if nsat x nsig > 64
 *           2020/11/30 1.15 support RTCM MT1131-1137,1041 (NavIC/IRNSS)
 *                           add log paths in API strsvrstart()
 *                           add log status in API strsvrstat()
 *                           support multiple ephemeris sets (e.g. I/NAV-F/NAV)
 *                           delete API strsvrsetsrctbl()
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include "rtklib.h"

/* Test observation data message ---------------------------------------------*/
static bool is_obsmsg(int msg) {
  return (1001 <= msg && msg <= 1004) || (1009 <= msg && msg <= 1012) ||
         (1071 <= msg && msg <= 1077) || (1081 <= msg && msg <= 1087) ||
         (1091 <= msg && msg <= 1097) || (1101 <= msg && msg <= 1107) ||
         (1111 <= msg && msg <= 1117) || (1121 <= msg && msg <= 1127) ||
         (1131 <= msg && msg <= 1137);
}
/* Test navigation data message ----------------------------------------------*/
static bool is_navmsg(int msg) {
  return msg == 1019 || msg == 1020 || msg == 1044 || msg == 1045 || msg == 1046 || msg == 1042 ||
         msg == 63 || msg == 1041;
}
/* Test station info message -------------------------------------------------*/
static int is_stamsg(int msg) {
  return msg == 1005 || msg == 1006 || msg == 1007 || msg == 1008 || msg == 1033 || msg == 1230;
}
/* Test time interval --------------------------------------------------------*/
static bool is_tint(gtime_t time, double tint) {
  if (tint <= 0.0) return true;
  return fmod(time2gpst(time, NULL) + DTTOL, tint) <= 2.0 * DTTOL;
}
/* New stream converter --------------------------------------------------------
 * Generate new stream converter
 * Args   : int    itype     I   input stream type  (STRFMT_???)
 *          int    otype     I   output stream type (STRFMT_???)
 *          char   *msgs     I   output message type and interval (, separated)
 *          int    staid     I   station id
 *          int    stasel    I   station info selection (0:remote,1:local)
 *          char   *opt      I   RTCM or receiver raw options
 * Return : stream generator (NULL:error)
 *----------------------------------------------------------------------------*/
extern strconv_t *strconvnew(int itype, int otype, const char *msgs, int staid, int stasel,
                             const char *opt) {
  strconv_t *conv = (strconv_t *)malloc(sizeof(strconv_t));
  if (!conv) return NULL;

  conv->nmsg = 0;
  char buff[1024];
  rtkstrcpy(buff, sizeof(buff), msgs);
  char *q;
  for (char *p = strtok_r(buff, ",", &q); p; p = strtok_r(NULL, ",", &q)) {
    double tint = 0.0;
    int msg;
    if (sscanf(p, "%d(%lf)", &msg, &tint) < 1) continue;
    conv->msgs[conv->nmsg] = msg;
    conv->tint[conv->nmsg] = tint;
    conv->tick[conv->nmsg] = tickget();
    conv->ephsat[conv->nmsg++] = 0;
    if (conv->nmsg >= 32) break;
  }
  if (conv->nmsg <= 0) {
    free(conv);
    return NULL;
  }
  conv->itype = itype;
  conv->otype = otype;
  conv->stasel = stasel;
  if (!init_rtcm(&conv->rtcm) || !init_rtcm(&conv->out)) {
    free(conv);
    return NULL;
  }
  if (!init_raw(&conv->raw, itype)) {
    free_rtcm(&conv->rtcm);
    free_rtcm(&conv->out);
    free(conv);
    return NULL;
  }
  if (stasel) conv->out.staid = staid;
  rtksnprintf(conv->rtcm.opt, sizeof(conv->rtcm.opt), "-EPHALL %s", opt);
  rtksnprintf(conv->raw.opt, sizeof(conv->raw.opt), "-EPHALL %s", opt);
  return conv;
}
/* Free stream converter -------------------------------------------------------
 * Free stream converter
 * Args   : strconv_t *conv  IO  stream converter
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strconvfree(strconv_t *conv) {
  if (!conv) return;
  free_rtcm(&conv->rtcm);
  free_rtcm(&conv->out);
  free_raw(&conv->raw);
  free(conv);
}
/* Copy received data from receiver raw to RTCM ------------------------------*/
static void raw2rtcm(rtcm_t *out, const raw_t *raw, int ret) {
  out->time = raw->time;

  if (ret == 1) {
    for (int i = 0; i < raw->obs.n; i++) {
      out->time = raw->obs.data[i].time;
      out->obs.data[i] = raw->obs.data[i];

      int prn, sys = satsys(raw->obs.data[i].sat, &prn);
      if (sys == SYS_GLO && raw->nav.glo_fcn[prn - 1]) {
        out->nav.glo_fcn[prn - 1] = raw->nav.glo_fcn[prn - 1];
      }
    }
    out->obs.n = raw->obs.n;
  } else if (ret == 2) {
    int sat = raw->ephsat, set = raw->ephset;
    int prn, sys = satsys(sat, &prn);
    if (sys == SYS_GLO) {
      out->nav.geph[prn - 1][0] = raw->nav.geph[prn - 1][0];
      out->ephsat = sat;
      out->ephset = set;
    } else if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP ||
               sys == SYS_IRN) {
      out->nav.eph[sat - 1][set] = raw->nav.eph[sat - 1][set];
      out->ephsat = sat;
      out->ephset = set;
    }
  } else if (ret == 5) {
    out->sta = raw->sta;
  } else if (ret == 9) {
    matcpy(out->nav.utc_gps, raw->nav.utc_gps, 8, 1);
    matcpy(out->nav.utc_glo, raw->nav.utc_glo, 8, 1);
    matcpy(out->nav.utc_gal, raw->nav.utc_gal, 8, 1);
    matcpy(out->nav.utc_qzs, raw->nav.utc_qzs, 8, 1);
    matcpy(out->nav.utc_cmp, raw->nav.utc_cmp, 8, 1);
    matcpy(out->nav.utc_irn, raw->nav.utc_irn, 9, 1);
    matcpy(out->nav.utc_sbs, raw->nav.utc_sbs, 4, 1);
    matcpy(out->nav.ion_gps, raw->nav.ion_gps, 8, 1);
    matcpy(out->nav.ion_gal, raw->nav.ion_gal, 4, 1);
    matcpy(out->nav.ion_qzs, raw->nav.ion_qzs, 8, 1);
    matcpy(out->nav.ion_cmp, raw->nav.ion_cmp, 8, 1);
    matcpy(out->nav.ion_irn, raw->nav.ion_irn, 8, 1);
  }
}
/* Copy received data from receiver RTCM to RTCM -----------------------------*/
static void rtcm2rtcm(rtcm_t *out, const rtcm_t *rtcm, int ret, int stasel) {
  out->time = rtcm->time;

  if (!stasel) out->staid = rtcm->staid;

  if (ret == 1) {
    for (int i = 0; i < rtcm->obs.n; i++) {
      out->obs.data[i] = rtcm->obs.data[i];

      int prn, sys = satsys(rtcm->obs.data[i].sat, &prn);
      if (sys == SYS_GLO && rtcm->nav.glo_fcn[prn - 1]) {
        out->nav.glo_fcn[prn - 1] = rtcm->nav.glo_fcn[prn - 1];
      }
    }
    out->obs.n = rtcm->obs.n;
  } else if (ret == 2) {
    int sat = rtcm->ephsat, set = rtcm->ephset;
    int prn, sys = satsys(sat, &prn);
    if (sys == SYS_GLO) {
      out->nav.geph[prn - 1][0] = rtcm->nav.geph[prn - 1][0];
      out->ephsat = sat;
      out->ephset = set;
    } else if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP ||
               sys == SYS_IRN) {
      out->nav.eph[sat - 1][set] = rtcm->nav.eph[sat - 1][set];
      out->ephsat = sat;
      out->ephset = set;
    }
  } else if (ret == 5) {
    if (!stasel) out->sta = rtcm->sta;
  }
}
/* Write RTCM3 MSM to stream -------------------------------------------------*/
static void write_rtcm3_msm(stream_t *str, rtcm_t *out, int msg, int sync) {
  int sys;
  if (1071 <= msg && msg <= 1077)
    sys = SYS_GPS;
  else if (1081 <= msg && msg <= 1087)
    sys = SYS_GLO;
  else if (1091 <= msg && msg <= 1097)
    sys = SYS_GAL;
  else if (1101 <= msg && msg <= 1107)
    sys = SYS_SBS;
  else if (1111 <= msg && msg <= 1117)
    sys = SYS_QZS;
  else if (1121 <= msg && msg <= 1127)
    sys = SYS_CMP;
  else if (1131 <= msg && msg <= 1137)
    sys = SYS_IRN;
  else
    return;

  /* Count number of satellites and signals */
  obsd_t *data = out->obs.data;
  int nobs = out->obs.n, nsat = 0, nsig = 0, mask[MAXCODE] = {0};
  for (int i = 0; i < nobs && i < MAXOBS; i++) {
    if (satsys(data[i].sat, NULL) != sys) continue;
    nsat++;
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      int code = data[i].code[j];
      if (!code || mask[code - 1]) continue;
      mask[code - 1] = 1;
      nsig++;
    }
  }
  if (nsig > 64) return;

  /* Pack data to multiple messages if nsat x nsig > 64 */
  int ns, nmsg;
  if (nsig > 0) {
    ns = 64 / nsig;             /* Max number of sats in a message */
    nmsg = (nsat - 1) / ns + 1; /* Number of messages */
  } else {
    ns = 0;
    nmsg = 1;
  }
  obsd_t buff[MAXOBS];
  out->obs.data = buff;

  for (int i = 0, j = 0; i < nmsg; i++) {
    int n;
    for (n = 0; n < ns && j < nobs && j < MAXOBS; j++) {
      if (satsys(data[j].sat, NULL) != sys) continue;
      out->obs.data[n++] = data[j];
    }
    out->obs.n = n;

    if (gen_rtcm3(out, msg, 0, i < nmsg - 1 ? 1 : sync)) {
      strwrite(str, out->buff, out->nbyte);
    }
  }
  out->obs.data = data;
  out->obs.n = nobs;
}
/* Write obs data messages ---------------------------------------------------*/
static void write_obs(gtime_t time, stream_t *str, strconv_t *conv) {
  int j = 0;

  for (int i = 0; i < conv->nmsg; i++) {
    if (!is_obsmsg(conv->msgs[i]) || !is_tint(time, conv->tint[i])) continue;

    j = i; /* Index of last message */
  }
  for (int i = 0; i < conv->nmsg; i++) {
    if (!is_obsmsg(conv->msgs[i]) || !is_tint(time, conv->tint[i])) continue;

    /* Generate messages */
    if (conv->otype == STRFMT_RTCM2) {
      if (!gen_rtcm2(&conv->out, conv->msgs[i], i != j)) continue;

      /* Write messages to stream */
      strwrite(str, conv->out.buff, conv->out.nbyte);
    } else if (conv->otype == STRFMT_RTCM3) {
      if (conv->msgs[i] <= 1012) {
        if (!gen_rtcm3(&conv->out, conv->msgs[i], 0, i != j)) continue;
        strwrite(str, conv->out.buff, conv->out.nbyte);
      } else { /* Write RTCM3 MSM to stream */
        write_rtcm3_msm(str, &conv->out, conv->msgs[i], i != j);
      }
    }
  }
}
/* Write nav data messages ---------------------------------------------------*/
static void write_nav(gtime_t time, stream_t *str, strconv_t *conv) {
  for (int i = 0; i < conv->nmsg; i++) {
    if (!is_navmsg(conv->msgs[i]) || conv->tint[i] > 0.0) continue;

    /* Generate messages */
    if (conv->otype == STRFMT_RTCM2) {
      if (!gen_rtcm2(&conv->out, conv->msgs[i], 0)) continue;
    } else if (conv->otype == STRFMT_RTCM3) {
      if (!gen_rtcm3(&conv->out, conv->msgs[i], 0, 0)) continue;
    } else
      continue;

    /* Write messages to stream */
    strwrite(str, conv->out.buff, conv->out.nbyte);
  }
}
/* Next ephemeris satellite --------------------------------------------------*/
static int nextsat(const nav_t *nav, int sat, int msg) {
  int sys, set, p1, p2;
  switch (msg) {
    case 1019:
      sys = SYS_GPS;
      set = 0;
      p1 = MINPRNGPS;
      p2 = MAXPRNGPS;
      break;
    case 1020:
      sys = SYS_GLO;
      set = 0;
      p1 = MINPRNGLO;
      p2 = MAXPRNGLO;
      break;
    case 1044:
      sys = SYS_QZS;
      set = 0;
      p1 = MINPRNQZS;
      p2 = MAXPRNQZS;
      break;
    case 1045:
      sys = SYS_GAL;
      set = 1;
      p1 = MINPRNGAL;
      p2 = MAXPRNGAL;
      break;
    case 1046:
      sys = SYS_GAL;
      set = 0;
      p1 = MINPRNGAL;
      p2 = MAXPRNGAL;
      break;
    case 63:
    case 1042:
      sys = SYS_CMP;
      set = 0;
      p1 = MINPRNCMP;
      p2 = MAXPRNCMP;
      break;
    case 1041:
      sys = SYS_IRN;
      set = 0;
      p1 = MINPRNIRN;
      p2 = MAXPRNIRN;
      break;
    default:
      return 0;
  }
  int p0;
  if (satsys(sat, &p0) != sys) return satno(sys, p1);

  /* Search next valid ephemeris */
  for (int p = p0 >= p2 ? p1 : p0 + 1; p != p0; p = p >= p2 ? p1 : p + 1) {
    if (sys == SYS_GLO) {
      sat = satno(sys, p);
      if (nav->geph[p - 1][0].sat == sat) return sat;
    } else {
      sat = satno(sys, p);
      if (nav->eph[sat - 1][set].sat == sat) return sat;
    }
  }
  return 0;
}
/* Write cyclic nav data messages --------------------------------------------*/
static void write_nav_cycle(stream_t *str, strconv_t *conv) {
  uint32_t tick = tickget();

  for (int i = 0; i < conv->nmsg; i++) {
    if (!is_navmsg(conv->msgs[i]) || conv->tint[i] <= 0.0) continue;

    /* Output cycle */
    int tint = (int)(conv->tint[i] * 1000.0);
    if ((int)(tick - conv->tick[i]) < tint) continue;
    conv->tick[i] = tick;

    /* Next satellite */
    int sat = nextsat(&conv->out.nav, conv->ephsat[i], conv->msgs[i]);
    if (!sat) {
      continue;
    }
    conv->out.ephsat = conv->ephsat[i] = sat;

    /* Generate messages */
    if (conv->otype == STRFMT_RTCM2) {
      if (!gen_rtcm2(&conv->out, conv->msgs[i], 0)) continue;
    } else if (conv->otype == STRFMT_RTCM3) {
      if (!gen_rtcm3(&conv->out, conv->msgs[i], 0, 0)) continue;
    } else
      continue;

    /* Write messages to stream */
    strwrite(str, conv->out.buff, conv->out.nbyte);
  }
}
/* Write cyclic station info messages ----------------------------------------*/
static void write_sta_cycle(stream_t *str, strconv_t *conv) {
  uint32_t tick = tickget();

  for (int i = 0; i < conv->nmsg; i++) {
    if (!is_stamsg(conv->msgs[i])) continue;

    /* Output cycle */
    int tint = conv->tint[i] == 0.0 ? 30000 : (int)(conv->tint[i] * 1000.0);
    if ((int)(tick - conv->tick[i]) < tint) continue;
    conv->tick[i] = tick;

    /* Generate messages */
    if (conv->otype == STRFMT_RTCM2) {
      if (!gen_rtcm2(&conv->out, conv->msgs[i], 0)) continue;
    } else if (conv->otype == STRFMT_RTCM3) {
      if (!gen_rtcm3(&conv->out, conv->msgs[i], 0, 0)) continue;
    } else
      continue;

    /* Write messages to stream */
    strwrite(str, conv->out.buff, conv->out.nbyte);
  }
}
/* Convert stearm ------------------------------------------------------------*/
static void strconv(stream_t *str, strconv_t *conv, const uint8_t *buff, int n) {
  for (int i = 0; i < n; i++) {
    int ret;
    /* Input RTCM 2 messages */
    if (conv->itype == STRFMT_RTCM2) {
      ret = input_rtcm2(&conv->rtcm, buff[i]);
      rtcm2rtcm(&conv->out, &conv->rtcm, ret, conv->stasel);
    }
    /* Input RTCM 3 messages */
    else if (conv->itype == STRFMT_RTCM3) {
      ret = input_rtcm3(&conv->rtcm, buff[i]);
      rtcm2rtcm(&conv->out, &conv->rtcm, ret, conv->stasel);
    }
    /* Input receiver raw messages */
    else {
      ret = input_raw(&conv->raw, conv->itype, buff[i]);
      raw2rtcm(&conv->out, &conv->raw, ret);
    }
    /* Write obs and nav data messages to stream */
    switch (ret) {
      case 1:
        write_obs(conv->out.time, str, conv);
        break;
      case 2:
        write_nav(conv->out.time, str, conv);
        break;
    }
  }
  /* Write cyclic nav data and station info messages to stream */
  write_nav_cycle(str, conv);
  write_sta_cycle(str, conv);
}
/* Periodic command ----------------------------------------------------------*/
static void periodic_cmd(int cycle, const char *cmd, stream_t *stream) {
  const char *q;
  for (const char *p = cmd;; p = q + 1) {
    for (q = p;; q++)
      if (*q == '\r' || *q == '\n' || *q == '\0') break;
    int n = (int)(q - p);
    char msg[1024];
    rtkesubstrcpy(msg, sizeof(msg), p, 0, n);

    int period = 0;
    char *r = strrchr(msg, '#');
    if (r) {
      sscanf(r, "# %d", &period);
      size_t end = r - msg;
      msg[end] = '\0';
      /* Delete tail spaces */
      while (end > 0) {
        if (msg[end - 1] != ' ') break;
        msg[--end] = '\0';
      }
    }
    if (period <= 0) period = 1000;
    if (*msg && cycle % period == 0) {
      strsendcmd(stream, msg);
    }
    if (!*q) break;
  }
}
/* Stearm server thread ------------------------------------------------------*/
#ifdef WIN32
static DWORD WINAPI strsvrthread(void *arg)
#else
static void *strsvrthread(void *arg)
#endif
{
  strsvr_t *svr = (strsvr_t *)arg;
  sol_t sol_nmea = {{0}};

  tracet(3, "strsvrthread:\n");

  svr->tick = tickget();
  uint32_t tick_nmea = svr->tick - 1000;

  for (int cyc = 0; svr->state; cyc++) {
    uint32_t tick = tickget();

    /* Read data from input stream */
    int n;
    while ((n = strread(svr->stream, svr->buff, svr->buffsize)) > 0 && svr->state) {
      /* Write data to output streams */
      for (int i = 1; i < svr->nstr; i++) {
        if (svr->conv[i - 1]) {
          strconv(svr->stream + i, svr->conv[i - 1], svr->buff, n);
        } else {
          strwrite(svr->stream + i, svr->buff, n);
        }
      }
      /* Write data to log stream */
      strwrite(svr->strlog, svr->buff, n);

      rtklib_lock(&svr->lock);
      for (int i = 0; i < n && svr->npb < svr->buffsize; i++) {
        svr->pbuf[svr->npb++] = svr->buff[i];
      }
      rtklib_unlock(&svr->lock);
    }
    for (int i = 1; i < svr->nstr; i++) {
      /* Read message from output stream if connected */
      uint8_t buff[1024];
      while (strstat(svr->stream + i, NULL, 0) >= 2 &&
             (n = strread(svr->stream + i, buff, sizeof(buff))) > 0) {
        /* Relay back message from output stream to input stream */
        if (i == svr->relayback) {
          strwrite(svr->stream, buff, n);
        }
        /* Write data to log stream */
        strwrite(svr->strlog + i, buff, n);
      }
    }
    /* Write periodic command to input stream */
    for (int i = 0; i < svr->nstr; i++) {
      periodic_cmd(cyc * svr->cycle, svr->cmds_periodic[i], svr->stream + i);
    }
    /* Write NMEA messages to input stream */
    if (svr->nmeacycle > 0 && (int)(tick - tick_nmea) >= svr->nmeacycle) {
      sol_nmea.stat = SOLQ_SINGLE;
      sol_nmea.ns = 10; /* Some servers don't like when ns = 0 */
      sol_nmea.time = utc2gpst(timeget());
      matcpy(sol_nmea.rr, svr->nmeapos, 3, 1);
      strsendnmea(svr->stream, &sol_nmea);
      tick_nmea = tick;
    }
    sleepms(svr->cycle - (int)(tickget() - tick));
  }
  for (int i = 0; i < svr->nstr; i++) strclose(svr->stream + i);
  for (int i = 0; i < svr->nstr; i++) strclose(svr->strlog + i);
  svr->npb = 0;
  free(svr->buff);
  svr->buff = NULL;
  free(svr->pbuf);
  svr->pbuf = NULL;

  return 0;
}
/* Initialize stream server ----------------------------------------------------
 * Initialize stream server
 * Args   : strsvr_t *svr    IO  stream sever struct
 *          int    nout      I   number of output streams
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsvrinit(strsvr_t *svr, int nout) {
  tracet(3, "strsvrinit: nout=%d\n", nout);

  svr->state = 0;
  svr->cycle = 0;
  svr->buffsize = 0;
  svr->nmeacycle = 0;
  svr->relayback = 0;
  svr->npb = 0;
  for (int i = 0; i < 16; i++) *svr->cmds_periodic[i] = '\0';
  for (int i = 0; i < 3; i++) svr->nmeapos[i] = 0.0;
  svr->buff = svr->pbuf = NULL;
  svr->tick = 0;
  for (int i = 0; i < nout + 1 && i < 16; i++) strinit(svr->stream + i);
  {
    int i = 0;
    for (i = 0; i < nout + 1 && i < 16; i++) strinit(svr->strlog + i);
    svr->nstr = i;
  }
  for (int i = 0; i < 16; i++) svr->conv[i] = NULL;
  svr->thread = 0;
  rtklib_initlock(&svr->lock);
}
/* Start stream server ---------------------------------------------------------
 * Start stream server
 * Args   : strsvr_t *svr    IO  stream sever struct
 *          int    *opts     I   stream options
 *              opts[0]= inactive timeout (ms)
 *              opts[1]= interval to reconnect (ms)
 *              opts[2]= averaging time of data rate (ms)
 *              opts[3]= receive/send buffer size (bytes);
 *              opts[4]= server cycle (ms)
 *              opts[5]= NMEA request cycle (ms) (0:no)
 *              opts[6]= file swap margin (s)
 *              opts[7]= relay back of output stream (0:no)
 *          int    *strs     I   stream types (STR_???)
 *              strs[0]= input stream
 *              strs[1]= output stream 1
 *              strs[2]= output stream 2
 *              strs[3]= output stream 3
 *              ...
 *          char   **paths   I   stream paths
 *              paths[0]= input stream
 *              paths[1]= output stream 1
 *              paths[2]= output stream 2
 *              paths[3]= output stream 3
 *              ...
 *          char   **logs    I   log paths
 *              logs[0]= input log path
 *              logs[1]= output stream 1 return log path
 *              logs[2]= output stream 2 retrun log path
 *              logs[3]= output stream 2 retrun log path
 *              ...
 *          strconv_t **conv I   stream converter
 *              conv[0]= output stream 1 converter
 *              conv[1]= output stream 2 converter
 *              conv[2]= output stream 3 converter
 *              ...
 *          char   **cmds    I   start/stop commands (NULL: no cmd)
 *              cmds[0]= input stream command
 *              cmds[1]= output stream 1 command
 *              cmds[2]= output stream 2 command
 *              cmds[3]= output stream 3 command
 *              ...
 *          char   **cmds_periodic I periodic commands (NULL: no cmd)
 *              cmds[0]= input stream command
 *              cmds[1]= output stream 1 command
 *              cmds[2]= output stream 2 command
 *              cmds[3]= output stream 3 command
 *              ...
 *          double *nmeapos  I   NMEA request position (ECEF) (m) (NULL: no)
 * Return : status (true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool strsvrstart(strsvr_t *svr, const int *opts, const int *strs, const char **paths,
                        const char **logs, strconv_t **conv, const char **cmds,
                        const char **cmds_periodic, const double *nmeapos) {
  tracet(3, "strsvrstart:\n");

  if (svr->state) return false;

  strinitcom();

  int stropt[8] = {0};
  for (int i = 0; i < 4; i++) stropt[i] = opts[i];
  stropt[4] = opts[6];
  strsetopt(stropt);
  svr->cycle = opts[4];
  svr->buffsize = opts[3] < 4096 ? 4096 : opts[3];                 /* >=4096byte */
  svr->nmeacycle = 0 < opts[5] && opts[5] < 1000 ? 1000 : opts[5]; /* >=1s */
  svr->relayback = opts[7];
  for (int i = 0; i < 3; i++) svr->nmeapos[i] = nmeapos ? nmeapos[i] : 0.0;
  for (int i = 0; i < svr->nstr; i++) {
    rtkstrcpy(svr->cmds_periodic[i], sizeof(svr->cmds_periodic[0]),
              !cmds_periodic[i] ? "" : cmds_periodic[i]);
  }
  for (int i = 0; i < svr->nstr - 1; i++) svr->conv[i] = conv[i];

  if (!(svr->buff = (uint8_t *)malloc(svr->buffsize)) ||
      !(svr->pbuf = (uint8_t *)malloc(svr->buffsize))) {
    free(svr->buff);
    free(svr->pbuf);
    return false;
  }
  /* Open streams */
  for (int i = 0; i < svr->nstr; i++) {
    char file1[MAXSTRPATH];
    rtkstrcpy(file1, sizeof(file1), paths[0]);
    char *p = strstr(file1, "::");
    if (p) *p = '\0';
    char file2[MAXSTRPATH];
    rtkstrcpy(file2, sizeof(file2), paths[i]);
    p = strstr(file2, "::");
    if (p) *p = '\0';
    if (i > 0 && *file1 && !strcmp(file1, file2)) {
      rtksnprintf(svr->stream[i].msg, sizeof(svr->stream[0].msg), "output path error: %-512.512s",
                  file2);
      for (i--; i >= 0; i--) strclose(svr->stream + i);
      return false;
    }
    int rw;
    if (strs[i] == STR_FILE) {
      rw = i == 0 ? STR_MODE_R : STR_MODE_W;
    } else {
      rw = STR_MODE_RW;
    }
    if (stropen(svr->stream + i, strs[i], rw, paths[i])) continue;
    for (i--; i >= 0; i--) strclose(svr->stream + i);
    return false;
  }
  /* Open log streams */
  for (int i = 0; i < svr->nstr; i++) {
    if (strs[i] == STR_NONE || strs[i] == STR_FILE || !*logs[i]) continue;
    stropen(svr->strlog + i, STR_FILE, STR_MODE_W, logs[i]);
  }
  /* Write start commands to input/output streams */
  for (int i = 0; i < svr->nstr; i++) {
    if (!cmds[i]) continue;
    strwrite(svr->stream + i, (uint8_t *)"", 0); /* For connect */
    sleepms(100);
    strsendcmd(svr->stream + i, cmds[i]);
  }
  svr->state = 1;

  /* Create stream server thread */
#ifdef WIN32
  if (!(svr->thread = CreateThread(NULL, 0, strsvrthread, svr, 0, NULL))) {
#else
  if (pthread_create(&svr->thread, NULL, strsvrthread, svr)) {
#endif
    for (int i = 0; i < svr->nstr; i++) strclose(svr->stream + i);
    svr->state = 0;
    return false;
  }
  return true;
}
/* Stop stream server ----------------------------------------------------------
 * Stop stream server
 * Args   : strsvr_t *svr    IO  stream server struct
 *          char  **cmds     I   stop commands (NULL: no cmd)
 *              cmds[0]= input stream command
 *              cmds[1]= output stream 1 command
 *              cmds[2]= output stream 2 command
 *              cmds[3]= output stream 3 command
 *              ...
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsvrstop(strsvr_t *svr, const char **cmds) {
  tracet(3, "strsvrstop:\n");

  for (int i = 0; i < svr->nstr; i++) {
    if (cmds[i]) strsendcmd(svr->stream + i, cmds[i]);
  }
  svr->state = 0;

#ifdef WIN32
  WaitForSingleObject(svr->thread, 10000);
  CloseHandle(svr->thread);
#else
  pthread_join(svr->thread, NULL);
#endif
}
/* Get stream server status ----------------------------------------------------
 * Get status of stream server
 * Args   : strsvr_t *svr    IO  stream sever struct
 *          int    *stat     O   stream status
 *          int    *log_stat O   log status
 *          int    *byte     O   bytes received/sent
 *          int    *bps      O   bitrate received/sent
 *          char   *msg      O   messages
 *          size_t size      I   messages buffer size
 * Return : none
 * Note   : Messages are appended to msg which must be nul terminated.
 *----------------------------------------------------------------------------*/
extern void strsvrstat(strsvr_t *svr, int *stat, int *log_stat, int *byte, int *bps, char *msg,
                       size_t msize) {
  tracet(4, "strsvrstat:\n");

  for (int i = 0; i < svr->nstr; i++) {
    if (i == 0) {
      strsum(svr->stream, byte, bps, NULL, NULL);
    } else {
      int bps_in;
      strsum(svr->stream + i, NULL, &bps_in, byte + i, bps + i);
    }
    char s[MAXSTRMSG] = "";
    stat[i] = strstat(svr->stream + i, s, sizeof(s));
    if (*s) rtkcatprintf(msg, msize, "(%d) %s ", i, s);
    log_stat[i] = strstat(svr->strlog + i, NULL, 0);
  }
}
/* Peek input/output stream ----------------------------------------------------
 * Peek input/output stream of stream server
 * Args   : strsvr_t *svr    IO  stream sever struct
 *          uint8_t *buff    O   stream buff
 *          int    nmax      I   buffer size (bytes)
 * Return : stream size (bytes)
 *----------------------------------------------------------------------------*/
extern int strsvrpeek(strsvr_t *svr, uint8_t *buff, int nmax) {
  if (!svr->state) return 0;

  rtklib_lock(&svr->lock);
  int n = svr->npb < nmax ? svr->npb : nmax;
  if (n > 0) {
    memcpy(buff, svr->pbuf, n);
  }
  if (n < svr->npb) {
    memmove(svr->pbuf, svr->pbuf + n, svr->npb - n);
  }
  svr->npb -= n;
  rtklib_unlock(&svr->lock);
  return n;
}
