/*------------------------------------------------------------------------------
 * convrnx.c : RINEX translator for RTCM and receiver raw data log
 *
 *          Copyright (C) 2009-2020 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision: 1.2 $ $Date: 2008/07/17 21:48:06 $
 * History : 2009/04/10 1.0  new
 *           2009/06/02 1.1  support GLONASS
 *           2009/12/18 1.2  add check return of init_rtcm()/init_raw()
 *           2010/07/15 1.3  support wildcard expansion of input file
 *                           support RINEX 3.00
 *                           support RINEX as input format
 *                           support output of geo navigation message
 *                           support RTCM antenna and receiver info
 *                           changed api:
 *                               convrnx()
 *           2011/05/27 1.4  support GW10, JAVAD, LEX receiver
 *                           support lex message conversion
 *                           change api convrnx()
 *           2012/10/18 1.5  support multiple codes in a frequency
 *           2012/10/29 1.6  fix bug on scanning obs types
 *                           support output of compass navigation data
 *                           add supported obs types for RINEX input
 *           2013/03/11 1.7  support binex and RINEX 3.02
 *                           add approx position in RINEX obs header if blank
 *           2014/05/24 1.8  support BeiDou B1
 *           2014/08/26 1.9  support input format rt17
 *           2015/05/24 1.10 fix bug on setting antenna delta in rtcm2opt()
 *           2016/07/04 1.11 support IRNSS
 *           2016/10/10 1.12 support event output by staid change in RTCM
 *                           support separated navigation files for ver.3
 *           2017/06/06 1.13 fix bug on array overflow in set_obstype() and
 *                           scan_obstype()
 *           2018/10/10 1.14 add trace of half-cycle ambiguity status
 *                           fix bug on missing navigation data
 *           2020/11/30 1.15 force scanning receiver log for obs-types (2-pass)
 *                           delete scanobs in RINEX options (rnxopt_t)
 *                           add phase shift option (phshift) in rnxopt_t
 *                           sort obs-types by freq-index and code priority
 *                           add test obs-types supported by RINEX versions
 *                           support receiver/antenna info in raw data
 *                           fix bug on writing BDS/IRN nav header in closefile()
 *                           fix bug on screening time in screent_ttol()
 *                           fix bug on screening QZS L1S messages as SBAS
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

#define NOUTFILE 9        /* Number of output files */
#define NSATSYS 7         /* Number of satellite systems */
#define TSTARTMARGIN 60.0 /* Time margin for file name replacement */

#define EVENT_STARTMOVE 2 /* RINEX event start moving antenna */
#define EVENT_NEWSITE 3   /* RINEX event new site occupation */
#define EVENT_HEADER 4    /* RINEX event header info follows */
#define EVENT_EXTERNAL 5  /* RINEX event external event */

/* Type definitions ----------------------------------------------------------*/

typedef struct stas_tag { /* Station list type */
  int staid;              /* Station IS */
  gtime_t ts, te;         /* First and last observation time */
  sta_t sta;              /* Station parameters */
  struct stas_tag *next;  /* Next list */
} stas_t;

typedef struct halfc_tag { /* Half-cycle ambiguity list type */
  gtime_t ts, te;          /* First and last observation time */
  uint8_t stat;            /* Half-cycle ambiguity status */
  struct halfc_tag *next;  /* Next list */
} halfc_t;

typedef struct {                          /* Stream file type */
  int format;                             /* Stream format (STRFMT_???) */
  int staid;                              /* Station ID */
  int ephsat, ephset;                     /* Satellite and set of input ephemeris */
  gtime_t time;                           /* Current time */
  gtime_t tstart;                         /* Start time */
  obs_t *obs;                             /* Pointer to input observation data */
  nav_t *nav;                             /* Pointer to input navigation data */
  sta_t *sta;                             /* Pointer to input station parameters */
  rtcm_t rtcm;                            /* Input RTCM data */
  raw_t raw;                              /* Input receiver raw data */
  rnxctr_t rnx;                           /* Input RINEX control data */
  stas_t *stas;                           /* Station list */
  uint8_t slips[MAXSAT][NFREQ + NEXOBS];  /* Cycle slip flag cache */
  halfc_t *halfc[MAXSAT][NFREQ + NEXOBS]; /* Half-cycle ambiguity list */
  FILE *fp;                               /* Output file pointer */
} strfile_t;

/* Global variables ----------------------------------------------------------*/
static const int navsys[] = {/* System codes */
                             SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_SBS, SYS_CMP, SYS_IRN, 0};
static const char vercode[][MAXCODE] = {
    /* Supported obs-type by RINEX version */
    /* 0........1.........2.........3.........4.........5.........6........          */
    /* 11111111111112222222222555777666666688822663331155599991555677788444     CODE */
    /* CPWYMNSLEABXZCDSLXPWYMNIQXIQXABCXZSLIQXIQIQIQXIQABCABCXDDPZEDPZDPABX          */
    "00000000...0.0000000000000..........................................", /* GPS */
    "00...........0....0..........44.4..........222...................444", /* GLO */
    "0........0000..........0000000000...000.............................", /* GAL */
    "2.....22...22..222.....222......2422....................4444........", /* QZS */
    "0......................000..........................................", /* SBS */
    ".4...4...4.4.....1.......41114..1.....41111............444..44444...", /* BDS */
    ".........................3......................3333333............."  /* IRN */
};
/* Convert RINEX obs-type ver.3 -> ver.2 -------------------------------------*/
static void convcode(int rnxver, int sys, char *type, size_t size) {
  if (rnxver >= 212 && (sys == SYS_GPS || sys == SYS_QZS || sys == SYS_SBS) &&
      !strcmp(type + 1, "1C")) { /* L1C/A */
    rtkstrcpy(type + 1, size - 1, "A");
  } else if (rnxver >= 212 && (sys == SYS_GPS || sys == SYS_QZS) &&
             (!strcmp(type + 1, "1S") || !strcmp(type + 1, "1L") ||
              !strcmp(type + 1, "1X"))) { /* L1C */
    rtkstrcpy(type + 1, size - 1, "B");
  } else if (rnxver >= 212 && (sys == SYS_GPS || sys == SYS_QZS) &&
             (!strcmp(type + 1, "2S") || !strcmp(type + 1, "2L") ||
              !strcmp(type + 1, "2X"))) { /* L2C */
    rtkstrcpy(type + 1, size - 1, "C");
  } else if (rnxver >= 212 && sys == SYS_GLO && !strcmp(type + 1, "1C")) { /* L1C/A */
    rtkstrcpy(type + 1, size - 1, "A");
  } else if (rnxver >= 212 && sys == SYS_GLO && !strcmp(type + 1, "2C")) { /* L2C/A */
    rtkstrcpy(type + 1, size - 1, "D");
  } else if (sys == SYS_CMP && (!strcmp(type + 1, "2I") || !strcmp(type + 1, "2Q") ||
                                !strcmp(type + 1, "2X"))) { /* B1_2 */
    rtkstrcpy(type + 1, size - 1, "2");
  } else if (!strcmp(type, "C1P") || !strcmp(type, "C1W") || !strcmp(type, "C1Y") ||
             !strcmp(type, "C1N")) { /* L1P,P(Y) */
    rtkstrcpy(type, size - 1, "P1");
  } else if (!strcmp(type, "C2P") || !strcmp(type, "C2W") || !strcmp(type, "C2Y") ||
             !strcmp(type, "C2N") || !strcmp(type, "C2D")) { /* L2P,P(Y) */
    rtkstrcpy(type, size - 1, "P2");
  } else {
    type[2] = '\0';
  }
}
/* Generate stream file ------------------------------------------------------*/
static strfile_t *gen_strfile(int format, const char *opt) {
  gtime_t time0 = {0};

  trace(3, "init_strfile:\n");

  strfile_t *str = (strfile_t *)calloc(sizeof(strfile_t), 1);
  if (!str) return NULL;

  str->format = format;
  str->staid = -1;
  str->ephsat = str->ephset = 0;
  str->time = str->tstart = time0;

  if (format == STRFMT_RTCM2 || format == STRFMT_RTCM3) {
    if (!init_rtcm(&str->rtcm)) {
      free(str);
      showmsg("init rtcm error");
      return NULL;
    }
    str->rtcm.time = time0;
    str->obs = &str->rtcm.obs;
    str->nav = &str->rtcm.nav;
    str->sta = &str->rtcm.sta;
    rtkstrcpy(str->rtcm.opt, sizeof(str->rtcm.opt), opt);
  } else if (format <= MAXRCVFMT) {
    if (!init_raw(&str->raw, format)) {
      free(str);
      showmsg("init raw error");
      return NULL;
    }
    str->raw.time = time0;
    str->obs = &str->raw.obs;
    str->nav = &str->raw.nav;
    str->sta = &str->raw.sta;
    rtkstrcpy(str->raw.opt, sizeof(str->raw.opt), opt);
  } else if (format == STRFMT_RINEX) {
    if (!init_rnxctr(&str->rnx)) {
      free(str);
      showmsg("init rnx error");
      return NULL;
    }
    str->rnx.time = time0;
    str->obs = &str->rnx.obs;
    str->nav = &str->rnx.nav;
    str->sta = &str->rnx.sta;
    rtkstrcpy(str->rnx.opt, sizeof(str->raw.opt), opt);
  } else {
    free(str);
    return NULL;
  }

  str->stas = NULL;
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      str->slips[i][j] = 0;
      str->halfc[i][j] = NULL;
    }
  str->fp = NULL;
  return str;
}
/* Free stream file ----------------------------------------------------------*/
static void free_strfile(strfile_t *str) {
  trace(3, "free_strfile:\n");

  if (str->format == STRFMT_RTCM2 || str->format == STRFMT_RTCM3) {
    free_rtcm(&str->rtcm);
  } else if (str->format <= MAXRCVFMT) {
    free_raw(&str->raw);
  } else if (str->format == STRFMT_RINEX) {
    free_rnxctr(&str->rnx);
  }
  for (stas_t *sp = str->stas, *sp_next; sp; sp = sp_next) {
    sp_next = sp->next;
    free(sp);
  }
  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      for (halfc_t *hp = str->halfc[i][j], *hp_next; hp; hp = hp_next) {
        hp_next = hp->next;
        free(hp);
      }
    }
  free(str);
}
/* Input stream file ---------------------------------------------------------*/
static int input_strfile(strfile_t *str) {
  trace(4, "input_strfile:\n");

  int type = 0;
  if (str->format == STRFMT_RTCM2) {
    type = input_rtcm2f(&str->rtcm, str->fp);
    if (type >= 1) {
      str->time = str->rtcm.time;
      str->ephsat = str->rtcm.ephsat;
      str->ephset = str->rtcm.ephset;
      str->staid = str->rtcm.staid;
    }
  } else if (str->format == STRFMT_RTCM3) {
    type = input_rtcm3f(&str->rtcm, str->fp);
    if (type >= 1) {
      str->time = str->rtcm.time;
      str->ephsat = str->rtcm.ephsat;
      str->ephset = str->rtcm.ephset;
      str->staid = str->rtcm.staid;
    }
  } else if (str->format <= MAXRCVFMT) {
    type = input_rawf(&str->raw, str->format, str->fp);
    if (type >= 1) {
      str->time = str->raw.time;
      str->ephsat = str->raw.ephsat;
      str->ephset = str->raw.ephset;
      str->staid = 0;
    }
  } else if (str->format == STRFMT_RINEX) {
    type = input_rnxctr(&str->rnx, str->fp);
    if (type >= 1) {
      str->time = str->rnx.time;
      str->ephsat = str->rnx.ephsat;
      str->ephset = str->rnx.ephset;
      str->staid = 0;
    }
  }
  if (!str->tstart.time && str->time.time) {
    str->tstart = str->time;
  }
  char tstr[40];
  trace(4, "input_strfile: time=%s type=%d\n", time2str(str->time, tstr, 3), type);
  return type;
}
/* Open stream file ----------------------------------------------------------*/
static bool open_strfile(strfile_t *str, const char *file) {
  trace(3, "open_strfile: file=%s\n", file);

  if (str->format == STRFMT_RTCM2 || str->format == STRFMT_RTCM3) {
    if (!(str->fp = fopen(file, "rb"))) {
      showmsg("rtcm open error: %s", file);
      return false;
    }
    str->rtcm.time = str->time;
  } else if (str->format <= MAXRCVFMT) {
    if (!(str->fp = fopen(file, "rb"))) {
      showmsg("log open error: %s", file);
      return false;
    }
    str->raw.time = str->time;
  } else if (str->format == STRFMT_RINEX) {
    if (!(str->fp = fopen(file, "r"))) {
      showmsg("rinex open error: %s", file);
      return false;
    }
    /* Open RINEX control */
    if (!open_rnxctr(&str->rnx, str->fp)) {
      showmsg("no rinex file: %s", file);
      fclose(str->fp);
      return false;
    }
    str->rnx.time = str->time;
  }
  return true;
}
/* Close stream file ---------------------------------------------------------*/
static void close_strfile(strfile_t *str) {
  trace(3, "close_strfile:\n");

  if (str->format == STRFMT_RTCM2 || str->format == STRFMT_RTCM3) {
    if (str->fp) fclose(str->fp);
  } else if (str->format <= MAXRCVFMT) {
    if (str->fp) fclose(str->fp);
  } else if (str->format == STRFMT_RINEX) {
    if (str->fp) fclose(str->fp);
  }
}
/* Set format and files in RINEX options comments ----------------------------*/
static void setopt_file(int format, char **paths, int n, const int *mask, rnxopt_t *opt) {
  int i;
  for (i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) break;
  }
  if (i < MAXCOMMENT) {
    rtksnprintf(opt->comment[i++], sizeof(opt->comment[0]), "format: %.55s", formatstrs[format]);
  }
  for (int j = 0; j < n && i < MAXCOMMENT; j++) {
    if (!mask[j]) continue;
    rtksnprintf(opt->comment[i++], sizeof(opt->comment[0]), "log: %.58s", paths[j]);
  }
  if (*opt->rcvopt) {
    rtksnprintf(opt->comment[i++], sizeof(opt->comment[0]), "options: %.54s", opt->rcvopt);
  }
}
/* Unset RINEX options comments ----------------------------------------------*/
static void unsetopt_file(rnxopt_t *opt) {
  int brk = 0;

  for (int i = MAXCOMMENT - 1; i >= 0 && !brk; i--) {
    if (!*opt->comment[i]) continue;
    if (!strncmp(opt->comment[i], "format: ", 8)) brk = 1;
    *opt->comment[i] = '\0';
  }
}
/* Sort obs-types ------------------------------------------------------------*/
static void sort_obstype(uint8_t *codes, uint8_t *types, int n, int sys) {
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++) {
      int idx1 = code2idx(navsys[sys], codes[i]);
      int idx2 = code2idx(navsys[sys], codes[j]);
      int pri1 = getcodepri(navsys[sys], codes[i], "");
      int pri2 = getcodepri(navsys[sys], codes[j], "");
      if (idx1 < idx2 || (idx1 == idx2 && pri1 >= pri2)) continue;
      uint8_t tmp = codes[i];
      codes[i] = codes[j];
      codes[j] = tmp;
      tmp = types[i];
      types[i] = types[j];
      types[j] = tmp;
    }
}
/* Set obs-types in RINEX options --------------------------------------------*/
static void setopt_obstype(const uint8_t *codes, const uint8_t *types, int sys, rnxopt_t *opt) {
  trace(3, "setopt_obstype: sys=%d\n", sys);

  opt->nobs[sys] = 0;

  if (!(navsys[sys] & opt->navsys)) return;

  for (int i = 0; codes[i]; i++) {
    const char *id = code2obs(codes[i]);
    if (!id) continue;
    int idx = code2idx(navsys[sys], codes[i]);
    if (idx < 0) continue;
    if (!(opt->freqtype & (1 << idx)) || opt->mask[sys][codes[i] - 1] == '0') {
      continue;
    }
    if (opt->rnxver >= 300) {
      char ver = vercode[sys][codes[i] - 1];
      if (ver < '0' || ver > '0' + opt->rnxver - 300) {
        trace(2, "unsupported obs type: rnxver=%.2f sys=%d code=%s\n", opt->rnxver / 100.0, sys,
              code2obs(codes[i]));
        continue;
      }
    }
    for (int j = 0; j < 4; j++) {
      if (!(opt->obstype & (1 << j))) continue;
      if (types && !(types[i] & (1 << j))) continue;

      /* Obs-types in ver.3 */
      char type[16];
      const char type_str[] = "CLDS";
      rtksnprintf(type, sizeof(type), "%c%s", type_str[j], id);
      if (type[0] == 'C' && type[2] == 'N') continue; /* Codeless */

      if (opt->rnxver <= 299) { /* Ver.2 */

        /* Ver.3 -> ver.2 */
        convcode(opt->rnxver, navsys[sys], type, sizeof(type));

        /* Check duplicated obs-type */
        int k;
        for (k = 0; k < opt->nobs[0]; k++) {
          if (!strcmp(opt->tobs[0][k], type)) break;
        }
        if (k >= opt->nobs[0] && opt->nobs[0] < MAXOBSTYPE) {
          rtkstrcpy(opt->tobs[0][opt->nobs[0]++], sizeof(opt->tobs[0][0]), type);
        }
      } else if (opt->nobs[sys] < MAXOBSTYPE) { /* Ver.3 */
        rtkstrcpy(opt->tobs[sys][opt->nobs[sys]++], sizeof(opt->tobs[0][0]), type);
      }
    }
  }
}
/* Set phase shift in RINEX options (RINEX 3.04 A23) -------------------------*/
static void setopt_phshift(rnxopt_t *opt) {
  for (int i = 0; i < NSATSYS; i++)
    for (int j = 0; j < opt->nobs[i]; j++) {
      if (opt->tobs[i][j][0] != 'L') continue;
      uint8_t code = obs2code(opt->tobs[i][j] + 1);

      if (navsys[i] == SYS_GPS) {
        if (code == CODE_L1S || code == CODE_L1L || code == CODE_L1X || code == CODE_L1P ||
            code == CODE_L1W || code == CODE_L1N) {
          opt->shift[i][j] = 0.25; /* +1/4 cyc */
        } else if (code == CODE_L2C || code == CODE_L2S || code == CODE_L2L || code == CODE_L2X ||
                   code == CODE_L5Q) {
          opt->shift[i][j] = -0.25; /* -1/4 cyc */
        }
      } else if (navsys[i] == SYS_GLO) {
        if (code == CODE_L1P || code == CODE_L2P || code == CODE_L3Q) {
          opt->shift[i][j] = 0.25; /* +1/4 cyc */
        }
      } else if (navsys[i] == SYS_GAL) {
        if (code == CODE_L1C) {
          opt->shift[i][j] = 0.5; /* +1/2 cyc */
        } else if (code == CODE_L5Q || code == CODE_L7Q || code == CODE_L8Q) {
          opt->shift[i][j] = -0.25; /* -1/4 cyc */
        } else if (code == CODE_L6C) {
          opt->shift[i][j] = -0.5; /* -1/2 cyc */
        }
      } else if (navsys[i] == SYS_QZS) {
        if (code == CODE_L1S || code == CODE_L1L || code == CODE_L1X) {
          opt->shift[i][j] = 0.25; /* +1/4 cyc */
        } else if (code == CODE_L5Q || code == CODE_L5P) {
          opt->shift[i][j] = -0.25; /* -1/4 cyc */
        }
      } else if (navsys[i] == SYS_CMP) {
        if (code == CODE_L2P || code == CODE_L7Q || code == CODE_L6Q) {
          opt->shift[i][j] = -0.25; /* -1/4 cyc */
        } else if (code == CODE_L1P || code == CODE_L5P || code == CODE_L7P) {
          opt->shift[i][j] = 0.25; /* +1/4 cyc */
        }
      }
    }
}
/* Set station ID list to RINEX options comments -----------------------------*/
static void setopt_sta_list(const strfile_t *str, rnxopt_t *opt) {
  const stas_t *p;
  int n = 0;
  for (p = str->stas; p; p = p->next) {
    n++;
  }
  if (n <= 1) return;

  int i;
  for (i = 0; i < MAXCOMMENT; i++) {
    if (!*opt->comment[i]) break;
  }
  if (i >= MAXCOMMENT) return;
  rtksnprintf(opt->comment[i++], sizeof(opt->comment[0]), "%5s  %22s  %22s", "STAID",
              "TIME OF FIRST OBS", "TIME OF LAST OBS");

  for (p = str->stas, n--; p && n >= 0; p = p->next, n--) {
    if (i + n >= MAXCOMMENT) continue;
    char s1[40];
    time2str(p->ts, s1, 2);
    char s2[40];
    time2str(p->te, s2, 2);
    rtksnprintf(opt->comment[i + n], sizeof(opt->comment[0]), " %04d  %s  %s", p->staid, s1, s2);
  }
}
/* Set station info in RINEX options -----------------------------------------*/
static void setopt_sta(const strfile_t *str, rnxopt_t *opt) {
  trace(3, "setopt_sta:\n");

  const stas_t *p;
  /* Search first station in station list */
  for (p = str->stas; p; p = p->next) {
    if (!p->next) break;
    if (opt->ts.time && timediff(p->next->te, opt->ts) < 0.0) break;
  }
  const sta_t *sta;
  if (p && p->sta.name[0] != '\0') {
    sta = &p->sta;
    setopt_sta_list(str, opt);
  } else {
    sta = str->sta;
  }
  /* Marker name and number */
  if (!*opt->marker && !*opt->markerno) {
    rtkstrcpy(opt->marker, sizeof(opt->marker), sta->name);
    rtkstrcpy(opt->markerno, sizeof(opt->markerno), sta->marker);
  }
  /* Receiver and antenna info */
  if (!*opt->rec[0] && !*opt->rec[1] && !*opt->rec[2]) {
    rtkstrcpy(opt->rec[0], sizeof(opt->rec[0]), sta->recsno);
    rtkstrcpy(opt->rec[1], sizeof(opt->rec[1]), sta->rectype);
    rtkstrcpy(opt->rec[2], sizeof(opt->rec[2]), sta->recver);
  }
  if (!*opt->ant[0] && !*opt->ant[1] && !*opt->ant[2]) {
    rtkstrcpy(opt->ant[0], sizeof(opt->ant[0]), sta->antsno);
    rtkstrcpy(opt->ant[1], sizeof(opt->ant[1]), sta->antdes);
    if (sta->antsetup) {
      rtksnprintf(opt->ant[2], sizeof(opt->ant[2]), "%d", sta->antsetup);
    } else
      *opt->ant[2] = '\0';
  }
  /* Antenna approx position */
  if (!opt->autopos && norm(sta->pos, 3) > 0.0) {
    matcpy(opt->apppos, sta->pos, 3, 1);
  }
  /* Antenna delta */
  if (norm(opt->antdel, 3) > 0.0) {
    ;
  } else if (norm(sta->del, 3) > 0.0) {
    if (!sta->deltype) {                  /* ENU */
      opt->antdel[0] = sta->del[2];       /* h */
      opt->antdel[1] = sta->del[0];       /* e */
      opt->antdel[2] = sta->del[1];       /* n */
    } else if (norm(sta->pos, 3) > 0.0) { /* XYZ */
      double pos[3];
      ecef2pos(sta->pos, pos);
      double enu[3];
      ecef2enu(pos, sta->del, enu);
      opt->antdel[0] = enu[2]; /* h */
      opt->antdel[1] = enu[0]; /* e */
      opt->antdel[2] = enu[1]; /* n */
    } else {
      trace(2,
            "failed to update RINEX option antenna delta from xyz due to no station "
            "position\n");
    }
  } else {
    opt->antdel[0] = sta->hgt;
    opt->antdel[1] = 0.0;
    opt->antdel[2] = 0.0;
  }
}
/* Update station list -------------------------------------------------------*/
static void update_stas(strfile_t *str) {
  if (!str->stas || str->stas->staid != str->staid) { /* Station ID changed */
    stas_t *p = (stas_t *)calloc(sizeof(stas_t), 1);
    if (!p) return;
    p->staid = str->staid;
    p->ts = p->te = str->time;
    p->next = str->stas;
    str->stas = p;
  } else {
    str->stas->te = str->time;
  }
}
/* Update station info in station list ---------------------------------------*/
static void update_stainf(strfile_t *str) {
  if (str->stas && str->stas->staid == str->staid) {
    str->stas->sta = *str->sta;
  }
}
/* Dump station list ---------------------------------------------------------*/
static void dump_stas(const strfile_t *str) {
#if 1 /* For debug */
  trace(2, "# STATION LIST\n");
  trace(2, "# %17s %19s %5s %6s %16s %16s %12s %13s %9s %2s %6s %6s %6s\n", "TIME", "STAID",
        "MARKER", "ANTENNA", "RECEIVER", "LATITUDE", "LONGITUDE", "HIGHT", "DT", "DEL1", "DEL2",
        "DEL3");

  stas_t *p;
  for (p = str->stas; p; p = p->next) {
    char s1[40];
    time2str(p->ts, s1, 0);
    char s2[40];
    time2str(p->te, s2, 0);
    double pos[3];
    ecef2pos(p->sta.pos, pos);
    trace(2,
          "%s %s  %04d %-6.6s %-16.16s %-16.16s %12.8f %13.8f %9.3f %2d "
          "%6.3f %6.3f %6.3f\n",
          s1, s2, p->staid, p->sta.name, p->sta.antdes, p->sta.rectype, pos[0] * R2D, pos[1] * R2D,
          pos[2], p->sta.deltype, p->sta.del[0], p->sta.del[1], p->sta.del[2]);
  }
#endif
}
/* Add half-cycle ambiguity list ---------------------------------------------*/
static bool add_halfc(strfile_t *str, int sat, int idx, gtime_t time) {
  halfc_t *p = (halfc_t *)calloc(sizeof(halfc_t), 1);
  if (!p) return false;
  p->ts = p->te = time;
  p->stat = 0;
  p->next = str->halfc[sat - 1][idx];
  str->halfc[sat - 1][idx] = p;
  return true;
}
/* Update half-cycle ambiguity -----------------------------------------------*/
static void update_halfc(strfile_t *str, const obsd_t *obs) {
  int sat = obs->sat;

  for (int i = 0; i < NFREQ + NEXOBS; i++) {
    if (obs->L[i] == 0.0) continue;

    /* If no list, start list */
    if (!str->halfc[sat - 1][i]) {
      if (!add_halfc(str, sat, i, obs->time)) continue;
    }
    /* Reset list if true cycle slip */
    if ((obs->LLI[i] & LLI_SLIP) && !(obs->LLI[i] & (LLI_HALFA | LLI_HALFS))) {
      str->halfc[sat - 1][i]->stat = 0;
    }
    if (obs->LLI[i] & LLI_HALFC) { /* Halfcyc unresolved */
      /* If new list, set unresolved start epoch */
      if (str->halfc[sat - 1][i]->stat == 0) {
        str->halfc[sat - 1][i]->ts = obs->time;
      }
      /* Update unresolved end epoch and set status to active */
      str->halfc[sat - 1][i]->te = obs->time;
      str->halfc[sat - 1][i]->stat = 1;
    } /* Else if resolved, update status */
    else if (str->halfc[sat - 1][i]->stat == 1) {
      if (obs->LLI[i] & LLI_HALFA) {
        str->halfc[sat - 1][i]->stat = 2; /* Resolved with add */
      } else if (obs->LLI[i] & LLI_HALFS) {
        str->halfc[sat - 1][i]->stat = 3; /* Resolved with subtract */
      } else {
        str->halfc[sat - 1][i]->stat = 4; /* Resolved with no adjust */
      }
      /* Create new list entry */
      if (!add_halfc(str, sat, i, obs->time)) continue;
    }
  }
}
/* Dump half-cycle ambiguity list --------------------------------------------*/
static void dump_halfc(const strfile_t *str) {
#ifdef RTK_DISABLED /* For debug */
  trace(2, "# HALF-CYCLE AMBIGUITY CORRECTIONS\n");
  trace(2, "# %20s %22s %4s %3s %3s\n", "START", "END", "SAT", "FRQ", "COR");

  for (int i = 0; i < MAXSAT; i++)
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      for (halfc_t *p = str->halfc[i][j]; p; p = p->next) {
        if (p->stat <= 1) continue;
        char s0[8], s1[40], s2[40];
        satno2id(i + 1, s0);
        time2str(p->ts, s1, 2);
        time2str(p->te, s2, 2);
        const char *stats[] = {"ADD", "SUB", "NON"};
        trace(2, "%s %s %4s %3d %3s\n", s1, s2, s0, j + 1, stats[p->stat - 2]);
      }
    }
#endif
}
/* Resolve half-cycle ambiguity ----------------------------------------------*/
static void resolve_halfc(const strfile_t *str, obsd_t *data, int n) {
  for (int i = 0; i < n; i++)
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      int sat = data[i].sat;
      for (halfc_t *p = str->halfc[sat - 1][j]; p; p = p->next) {
        if (p->stat <= 1) continue; /* Unresolved half cycle */
        if (timediff(data[i].time, p->ts) < -DTTOL || timediff(data[i].time, p->te) > DTTOL)
          continue;

        if (p->stat == 2) { /* Add half cycle */
          data[i].L[j] += 0.5;
        } else if (p->stat == 3) { /* Subtract half cycle  */
          data[i].L[j] -= 0.5;
        }
        data[i].LLI[j] &= ~LLI_HALFC;
      }
      data[i].LLI[j] &= ~(LLI_HALFA | LLI_HALFS);
    }
}
/* Scan input files ----------------------------------------------------------*/
static bool scan_file(char **files, int nf, rnxopt_t *opt, strfile_t *str, int *mask) {
  trace(3, "scan_file: nf=%d\n", nf);

  uint8_t codes[NSATSYS][33] = {{0}};
  uint8_t types[NSATSYS][33] = {{0}};
  int c = 0, abort = 0, n[NSATSYS] = {0};

  for (int m = 0; m < nf && !abort; m++) {
    if (!open_strfile(str, files[m])) {
      continue;
    }
    int type;
    while ((type = input_strfile(str)) >= -1) {
      if (opt->ts.time && timediff(str->time, opt->ts) < -opt->ttol) continue;
      if (opt->te.time && timediff(str->time, opt->te) > -opt->ttol) break;
      mask[m] = 1; /* Update file mask */

      if (type == 1) { /* Observation data */
        for (int i = 0; i < str->obs->n; i++) {
          int sys = satsys(str->obs->data[i].sat, NULL);
          if (!(sys & opt->navsys)) continue;
          int l;
          for (l = 0; navsys[l]; l++)
            if (navsys[l] == sys) break;
          if (!navsys[l]) continue;

          /* Update obs-types */
          for (int j = 0; j < NFREQ + NEXOBS; j++) {
            if (!str->obs->data[i].code[j]) continue;

            int k;
            for (k = 0; k < n[l]; k++) {
              if (codes[l][k] == str->obs->data[i].code[j]) break;
            }
            if (k >= n[l] && n[l] < 32) {
              codes[l][n[l]++] = str->obs->data[i].code[j];
            }
            if (k < n[l]) {
              if (str->obs->data[i].P[j] != 0.0) types[l][k] |= 1;
              if (str->obs->data[i].L[j] != 0.0) types[l][k] |= 2;
              if (str->obs->data[i].D[j] != 0.0) types[l][k] |= 4;
              if (str->obs->data[i].SNR[j] != 0) types[l][k] |= 8;
            }
          }
          /* Update half-cycle ambiguity list */
          if (opt->halfcyc) {
            update_halfc(str, str->obs->data + i);
          }
        }
        /* Update station list */
        update_stas(str);
      } else if (type == 5) { /* Station info */
        /* Update station info */
        update_stainf(str);
      }
      if (++c % 11) continue;

      char tstr[40], msg[128];
      rtksnprintf(msg, sizeof(msg), "scanning: %s %s%s%s%s%s%s%s", time2str(str->time, tstr, 0),
                  n[0] ? "G" : "", n[1] ? "R" : "", n[2] ? "E" : "", n[3] ? "J" : "",
                  n[4] ? "S" : "", n[5] ? "C" : "", n[6] ? "I" : "");
      abort = showmsg(msg);
      if (abort) break;
    }
    close_strfile(str);
  }
  showmsg("");

  if (abort) {
    trace(2, "aborted in scan\n");
    return false;
  }
  for (int i = 0; i < NSATSYS; i++)
    for (int j = 0; j < n[i]; j++) {
      trace(2, "scan_file: sys=%d code=%s type=%d\n", i, code2obs(codes[i][j]), types[i][j]);
    }
  /* Sort and set obs-types in RINEX options */
  for (int i = 0; i < NSATSYS; i++) {
    sort_obstype(codes[i], types[i], n[i], i);
    setopt_obstype(codes[i], types[i], i, opt);

    for (int j = 0; j < n[i]; j++) {
      trace(3, "scan_file: sys=%d code=%s\n", i, code2obs(codes[i][j]));
    }
  }
  /* Set station info in RINEX options */
  setopt_sta(str, opt);

  /* Set phase shifts in RINEX options */
  if (opt->phshift) {
    setopt_phshift(opt);
  }
  /* Set GLONASS FCN and clear ephemeris */
  for (int i = 0; i < MAXSAT; i++) {
    for (int j = 0; j < str->nav->n[i]; j++) {
      eph_t eph0 = {0, -1, -1};
      str->nav->eph[i][j] = eph0;
    }
  }
  for (int k = 0; k < NSATGLO; k++) {
    for (int i = 0; i < str->nav->ng[k]; i++) {
      int prn;
      if (satsys(str->nav->geph[k][i].sat, &prn) != SYS_GLO) continue;
      str->nav->glo_fcn[prn - 1] = str->nav->geph[k][i].frq + 8;
      geph_t geph0 = {0, -1};
      str->nav->geph[k][i] = geph0;
    }
  }
  for (int k = 0; k < NSATSBS; k++) {
    for (int i = 0; i < str->nav->ns[k]; i++) {
      seph_t seph0 = {0};
      str->nav->seph[k][i] = seph0;
    }
  }
  dump_stas(str);
  dump_halfc(str);
  return true;
}
/* Write RINEX header --------------------------------------------------------*/
static void write_header(FILE **ofp, int idx, const rnxopt_t *opt, const nav_t *nav) {
  switch (idx) {
    case 0:
      outrnxobsh(ofp[0], opt, nav);
      break;
    case 1:
      outrnxnavh(ofp[1], opt, nav);
      break;
    case 2:
      outrnxgnavh(ofp[2], opt, nav);
      break;
    case 3:
      outrnxhnavh(ofp[3], opt, nav);
      break;
    case 4:
      outrnxqnavh(ofp[4], opt, nav);
      break;
    case 5:
      outrnxlnavh(ofp[5], opt, nav);
      break;
    case 6:
      outrnxcnavh(ofp[6], opt, nav);
      break;
    case 7:
      outrnxinavh(ofp[7], opt, nav);
      break;
  }
}
/* Open output files ---------------------------------------------------------*/
static bool openfile(FILE **ofp, char *files[], const char *file, const rnxopt_t *opt,
                     const nav_t *nav) {
  trace(3, "openfile:\n");

  for (int i = 0; i < NOUTFILE; i++) {
    if (!*files[i]) continue;

    char path[FNSIZE];
    rtkstrcpy(path, sizeof(path), files[i]);

    /* Check overwrite input file and modify output file */
    if (!strcmp(path, file)) rtkstrcat(path, sizeof(path), "_");

    /* Create directory if not exist */
    createdir(path);

    if (!(ofp[i] = fopen(path, "w"))) {
      showmsg("file open error: %s", path);
      for (i--; i >= 0; i--)
        if (ofp[i]) fclose(ofp[i]);
      return false;
    }
    /* Write RINEX header */
    write_header(ofp, i, opt, nav);
  }
  return true;
}
/* Close output files --------------------------------------------------------*/
static void closefile(FILE **ofp, const rnxopt_t *opt, const nav_t *nav) {
  trace(3, "closefile:\n");

  for (int i = 0; i < NOUTFILE; i++) {
    if (!ofp[i]) continue;

    /* Rewrite RINEX header */
    rewind(ofp[i]);
    write_header(ofp, i, opt, nav);

    fclose(ofp[i]);
  }
}
/* Output RINEX event --------------------------------------------------------*/
static void outrnxevent(FILE *fp, const rnxopt_t *opt, gtime_t time, int event, const stas_t *stas,
                        int staid) {
  trace(3, "outrnxevent: event=%d\n", event);

  if (event == EVENT_STARTMOVE) {
    fprintf(fp, "%*s%d%3d\n", (opt->rnxver >= 300) ? 31 : 28, "", event, 2);
    fprintf(fp, "%-60s%-20s\n", "EVENT: START MOVING ANTENNA", "COMMENT");
    fprintf(fp, "%-60s%-20s\n", opt->marker, "MARKER NAME");
  } else if (event == EVENT_NEWSITE) {
    const stas_t *p = NULL;
    for (const stas_t *q = stas; q; q = q->next) {
      if (q->staid == staid && timediff(time, q->te) <= 0.0) p = q;
    }
    fprintf(fp, "%*s%d%3d\n", (opt->rnxver >= 300) ? 31 : 28, "", event, 6);
    fprintf(fp, "%-60s%-20s\n", "EVENT: NEW SITE OCCUPATION", "COMMENT");
    if (!p) {
      fprintf(fp, "%04d%56s%-20s\n", staid, "", "MARKER NAME");
      return;
    }
    fprintf(fp, "%-60s%-20s\n", p->sta.name, "MARKER NAME");
    fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", p->sta.recsno, p->sta.rectype, p->sta.recver,
            "REC # / TYPE / VERS");
    fprintf(fp, "%-20.20s%-20.20s%-20.20s%-20s\n", p->sta.antsno, p->sta.antdes, "",
            "ANT # / TYPE");
    fprintf(fp, "%14.4f%14.4f%14.4f%-18s%-20s\n", p->sta.pos[0], p->sta.pos[1], p->sta.pos[2], "",
            "APPROX POSITION XYZ");

    /* Antenna delta */
    double del[3];
    if (norm(p->sta.del, 3) > 0.0) {
      if (!p->sta.deltype) {                  /* ENU */
        del[0] = p->sta.del[2];               /* h */
        del[1] = p->sta.del[0];               /* e */
        del[2] = p->sta.del[1];               /* n */
      } else if (norm(p->sta.pos, 3) > 0.0) { /* XYZ */
        double pos[3];
        ecef2pos(p->sta.pos, pos);
        double enu[3];
        ecef2enu(pos, p->sta.del, enu);
        del[0] = enu[2]; /* h */
        del[1] = enu[0]; /* e */
        del[2] = enu[1]; /* n */
      } else {
        trace(2,
              "failed to output RINEX option antenna delta from xyz due to no station "
              "position\n");
        del[0] = del[1] = del[2] = 0.0;
      }
    } else {
      del[0] = p->sta.hgt;
      del[1] = del[2] = 0.0;
    }
    fprintf(fp, "%14.4f%14.4f%14.4f%-18s%-20s\n", del[0], del[1], del[2], "",
            "ANTENNA: DELTA H/E/N");
  } else if (event == EVENT_EXTERNAL) {
    double ep[6];
    time2epoch(time, ep);
    fprintf(fp, "%s %02d %02.0f %02.0f %02.0f %02.0f %010.7f  %d%3d\n",
            (opt->rnxver >= 300) ? ">" : "", (opt->rnxver >= 300) ? (int)ep[0] : (int)ep[0] % 100,
            ep[1], ep[2], ep[3], ep[4], ep[5], event, 1);
    fprintf(fp, "%-60s%-20s\n", "EXTERNAL EVENT", "COMMENT");
  }
}
/* Save cycle slips ----------------------------------------------------------*/
static void save_slips(strfile_t *str, obsd_t *data, int n) {
  for (int i = 0; i < n; i++)
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      if (data[i].LLI[j] & LLI_SLIP) str->slips[data[i].sat - 1][j] = 1;
    }
}
/* Restore cycle slips -------------------------------------------------------*/
static void rest_slips(strfile_t *str, obsd_t *data, int n) {
  for (int i = 0; i < n; i++)
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      if (data[i].L[j] != 0.0 && str->slips[data[i].sat - 1][j]) {
        data[i].LLI[j] |= LLI_SLIP;
        str->slips[data[i].sat - 1][j] = 0;
      }
    }
}
/* Screen time with time tolerance -------------------------------------------*/
static int screent_ttol(gtime_t time, gtime_t ts, gtime_t te, double tint, double ttol) {
  if (ttol <= 0.0) ttol = DTTOL;

  return (tint <= 0.0 || fmod(time2gpst(time, NULL) + ttol, tint) <= ttol * 2.0) &&
         (ts.time == 0 || timediff(time, ts) >= -ttol) &&
         (te.time == 0 || timediff(time, te) < ttol);
}
/* Convert observation data --------------------------------------------------*/
static void convobs(FILE **ofp, rnxopt_t *opt, strfile_t *str, int *n, gtime_t *tend, int *staid) {
  trace(3, "convobs :\n");

  if (!ofp[0] || str->obs->n <= 0) return;

  gtime_t time = str->obs->data[0].time;

  /* Avoid duplicated data by multiple files handover */
  if (tend->time && timediff(time, *tend) < opt->ttol) return;
  *tend = time;

  /* Save cycle slips */
  save_slips(str, str->obs->data, str->obs->n);

  if (!screent_ttol(time, opt->ts, opt->te, opt->tint, opt->ttol)) return;

  /* Restore cycle slips */
  rest_slips(str, str->obs->data, str->obs->n);

  if (str->staid != *staid) { /* Station ID changed */

    if (*staid >= 0) { /* Output RINEX event */
      outrnxevent(ofp[0], opt, str->time, EVENT_NEWSITE, str->stas, str->staid);
    }
    *staid = str->staid;

    /* Set cycle slips */
    for (int i = 0; i < str->obs->n; i++)
      for (int j = 0; j < NFREQ + NEXOBS; j++) {
        if (str->obs->data[i].L[j] != 0.0) {
          str->obs->data[i].LLI[j] |= LLI_SLIP;
        }
      }
  }
  /* Resolve half-cycle ambiguity */
  if (opt->halfcyc) {
    resolve_halfc(str, str->obs->data, str->obs->n);
  }
  /* Output RINEX observation data */
  outrnxobsb(ofp[0], opt, str->obs->data, str->obs->n, str->obs->flag);
  /* n[NOUTFILE+1] - count of events converted to RINEX */
  if (str->obs->flag == 5) n[NOUTFILE + 1]++;
  /* Set to zero flag for the next iteration (initialization) */
  str->obs->flag = 0;

  if (opt->tstart.time == 0) opt->tstart = time;
  opt->tend = time;

  n[0]++;
}
/* Convert navigation data ---------------------------------------------------*/
static void convnav(FILE **ofp, const rnxopt_t *opt, const strfile_t *str, int *n) {
  int sep_nav = (opt->rnxver <= 299 || opt->sep_nav);

  trace(3, "convnav :\n");

  int sat = str->ephsat;
  int set = str->ephset;
  int prn;
  int sys = satsys(sat, &prn);
  if (!(sys & opt->navsys) || opt->exsats[sat - 1]) return;

  double dtoe;
  switch (sys) {
    case SYS_GLO:
      dtoe = MAXDTOE_GLO;
      break;
    case SYS_GAL:
      dtoe = MAXDTOE_GAL;
      break;
    case SYS_QZS:
      dtoe = MAXDTOE_QZS;
      break;
    case SYS_CMP:
      dtoe = MAXDTOE_CMP;
      break;
    case SYS_IRN:
      dtoe = MAXDTOE_IRN;
      break;
    case SYS_SBS:
      dtoe = MAXDTOE_SBS;
      break;
    default:
      dtoe = MAXDTOE;
      break;
  }
  gtime_t ts = opt->ts;
  if (ts.time != 0) ts = timeadd(ts, -dtoe);
  if (!screent(str->time, ts, opt->te, 0.0)) return;

  if (sys == SYS_GPS) {
    if (ofp[1]) {
      outrnxnavb(ofp[1], opt, str->nav->eph[sat - 1] + set);
      n[1]++;
    }
  } else if (sys == SYS_GLO) {
    if (ofp[1] && !sep_nav) {
      outrnxgnavb(ofp[1], opt, str->nav->geph[prn - 1]);
      n[1]++;
    } else if (ofp[2] && sep_nav) {
      outrnxgnavb(ofp[2], opt, str->nav->geph[prn - 1]);
      n[2]++;
    }
  } else if (sys == SYS_SBS) {
    if (ofp[1] && !sep_nav) {
      outrnxhnavb(ofp[1], opt, str->nav->seph[prn - MINPRNSBS]);
      n[1]++;
    } else if (ofp[3] && sep_nav) {
      outrnxhnavb(ofp[3], opt, str->nav->seph[prn - MINPRNSBS]);
      n[3]++;
    }
  } else if (sys == SYS_QZS) {
    if (ofp[1] && !sep_nav) {
      outrnxnavb(ofp[1], opt, str->nav->eph[sat - 1] + set);
      n[1]++;
    } else if (ofp[4] && sep_nav) {
      outrnxnavb(ofp[4], opt, str->nav->eph[sat - 1] + set);
      n[4]++;
    }
  } else if (sys == SYS_GAL) {
    if (ofp[1] && !sep_nav) {
      outrnxnavb(ofp[1], opt, str->nav->eph[sat - 1] + set);
      n[1]++;
    } else if (ofp[5] && sep_nav) {
      outrnxnavb(ofp[5], opt, str->nav->eph[sat - 1] + set);
      n[5]++;
    }
  } else if (sys == SYS_CMP) {
    if (ofp[1] && !sep_nav) {
      outrnxnavb(ofp[1], opt, str->nav->eph[sat - 1] + set);
      n[1]++;
    } else if (ofp[6] && sep_nav) {
      outrnxnavb(ofp[6], opt, str->nav->eph[sat - 1] + set);
      n[6]++;
    }
  } else if (sys == SYS_IRN) {
    if (ofp[1] && !sep_nav) {
      outrnxnavb(ofp[1], opt, str->nav->eph[sat - 1] + set);
      n[1]++;
    } else if (ofp[7] && sep_nav) {
      outrnxnavb(ofp[7], opt, str->nav->eph[sat - 1] + set);
      n[7]++;
    }
  }
}
/* Convert SBAS message ------------------------------------------------------*/
static void convsbs(FILE **ofp, const rnxopt_t *opt, strfile_t *str, int *n, gtime_t *tend) {
  int sep_nav = opt->rnxver <= 299 || opt->sep_nav;

  trace(3, "convsbs :\n");

  gtime_t time = gpst2time(str->raw.sbsmsg.week, str->raw.sbsmsg.tow);

  if (!screent(time, opt->ts, opt->te, 0.0)) return;

  /* Avoid duplicated data by multiple files handover */
  if (tend->time && timediff(time, *tend) < opt->ttol) return;
  *tend = time;

  int prn = str->raw.sbsmsg.prn;
  int sys;
  if (MINPRNSBS <= prn && prn <= MAXPRNSBS) {
    sys = SYS_SBS;
  } else if (MINPRNQZS_S <= prn && prn <= MAXPRNQZS_S) {
    sys = SYS_QZS;
    prn += 10;
  } else {
    trace(2, "sbas message satellite error: prn=%d\n", prn);
    return;
  }
  int sat = satno(sys, prn);
  if (!sat || opt->exsats[sat - 1] == 1) return;

  /* Output SBAS message log */
  if (ofp[NOUTFILE - 1]) {
    sbsoutmsg(ofp[NOUTFILE - 1], &str->raw.sbsmsg);
    n[NOUTFILE - 1]++;
  }
  /* Output SBAS ephemeris */
  if ((opt->navsys & SYS_SBS) && sbsupdatecorr(&str->raw.sbsmsg, str->nav) == 9) {
    if (ofp[1] && !sep_nav) {
      outrnxhnavb(ofp[1], opt, str->nav->seph[prn - MINPRNSBS]);
      n[1]++;
    } else if (ofp[3] && sep_nav) {
      outrnxhnavb(ofp[3], opt, str->nav->seph[prn - MINPRNSBS]);
      n[3]++;
    }
  }
}
/* Set approx position in RINEX options --------------------------------------*/
static void setopt_apppos(const strfile_t *str, rnxopt_t *opt) {
  prcopt_t prcopt = prcopt_default;
  prcopt.navsys = opt->navsys;
  sol_t sol = {{0}};

  /* Point positioning with last obs data */
  char msg[128];
  msg[0] = '\0';
  if (!pntpos(str->obs->data, str->obs->n, str->nav, &prcopt, &sol, NULL, NULL, msg, sizeof(msg))) {
    trace(2, "point position error (%s)\n", msg);
    return;
  }
  matcpy(opt->apppos, sol.rr, 3, 1);
}
/* Show conversion status ----------------------------------------------------*/
static int showstat(int sess, gtime_t ts, gtime_t te, const int *n) {
  const char type[] = "ONGHQLCISET";
  char msg[1024] = "";
  if (sess > 0) {
    rtksnprintf(msg, sizeof(msg), "(%d) ", sess);
  }
  if (ts.time != 0) {
    char s[40];
    time2str(ts, s, 0);
    rtkcatprintf(msg, sizeof(msg), "%s", s);
  }
  if (te.time != 0 && timediff(te, ts) > 0.9) {
    char s[40];
    time2str(te, s, 0);
    rtkcatprintf(msg, sizeof(msg), "-%s", s + 5);
  }
  rtkcatprintf(msg, sizeof(msg), ": ");

  /* +2 to NOUTFILE for counters of errors and events */
  for (int i = 0; i < NOUTFILE + 2; i++) {
    if (n[i] == 0) continue;
    rtkcatprintf(msg, sizeof(msg), "%c=%d%s", type[i], n[i], i < NOUTFILE + 1 ? " " : "");
  }
  return showmsg(msg);
}
/* RINEX converter for single-session ----------------------------------------*/
static int convrnx_s(int sess, int format, rnxopt_t *opt, const char *file, char **ofile) {
  trace(3,
        "convrnx_s: sess=%d format=%d file=%s ofile=%s %s %s %s %s %s %s "
        "%s %s\n",
        sess, format, file, ofile[0], ofile[1], ofile[2], ofile[3], ofile[4], ofile[5], ofile[6],
        ofile[7], ofile[8]);

  /* Replace keywords in input file */
  char path[FNSIZE];
  const char *staname = *opt->staid ? opt->staid : "0000";
  if (reppath(file, path, sizeof(path), opt->ts, staname, "") < 0) {
    showmsg("no time for input file: %s", file);
    return 0;
  }
  /* Expand wild-cards in input file */
  char *epath[MAXEXFILE] = {0};
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(epath[i] = (char *)malloc(FNSIZE))) {
      for (int i2 = 0; i2 < MAXEXFILE; i2++) free(epath[i2]);
      return 0;
    }
  }

  int nf = expath(path, epath, FNSIZE, MAXEXFILE);
  if (nf <= 0) {
    showmsg("no input file: %s", path);
    return 0;
  }
  strfile_t *str = gen_strfile(format, opt->rcvopt);
  if (!str) {
    for (int i = 0; i < MAXEXFILE; i++) free(epath[i]);
    return 0;
  }
  if (format == STRFMT_RTCM2 || format == STRFMT_RTCM3 || format == STRFMT_RT17) {
    str->time = opt->trtcm;
  } else if (opt->ts.time) {
    str->time = timeadd(opt->ts, -1.0);
  }
  /* Set GLONASS FCN in RINEX options */
  for (int i = 0; i < MAXPRNGLO; i++) {
    str->nav->glo_fcn[i] = opt->glofcn[i]; /* FCN+8 */
  }
  /* Scan input files */
  int mask[MAXEXFILE] = {0};
  if (!scan_file(epath, nf, opt, str, mask)) {
    for (int i = 0; i < MAXEXFILE; i++) free(epath[i]);
    free_strfile(str);
    return 0;
  }
  /* Set format and file in RINEX options comments */
  setopt_file(format, epath, nf, mask, opt);

  /* Replace keywords in output file */
  char *paths[NOUTFILE], s[NOUTFILE][FNSIZE];
  for (int i = 0; i < NOUTFILE; i++) {
    paths[i] = s[i];
    if (reppath(ofile[i], paths[i], sizeof(s[0]), opt->ts.time ? opt->ts : str->tstart, staname,
                "") < 0) {
      showmsg("no time for output path: %s", ofile[i]);
      for (int i2 = 0; i2 < MAXEXFILE; i2++) free(epath[i2]);
      free_strfile(str);
      return 0;
    }
  }
  /* Open output files */
  FILE *ofp[NOUTFILE] = {NULL};
  if (!openfile(ofp, paths, path, opt, str->nav)) {
    for (int i = 0; i < MAXEXFILE; i++) free(epath[i]);
    free_strfile(str);
    return 0;
  }
  str->time = str->tstart;

  int n[NOUTFILE + 2] = {0}, abort = 0;
  gtime_t tend[3] = {{0}};
  int staid = -1;
  for (int i = 0; i < nf && !abort; i++) {
    if (!mask[i]) continue;

    /* Open stream file */
    if (!open_strfile(str, epath[i])) continue;

    /* Input message */
    int type;
    for (int j = 0; (type = input_strfile(str)) >= -1; j++) {
      if (!(j % 11) && (abort = showstat(sess, str->time, str->time, n))) break;
      if (opt->te.time && timediff(str->time, opt->te) > -opt->ttol) break;

      /* Convert message */
      switch (type) {
        case 1:
          convobs(ofp, opt, str, n, tend, &staid);
          break;
        case 2:
          convnav(ofp, opt, str, n);
          break;
        case 3:
          convsbs(ofp, opt, str, n, tend + 1);
          break;
        case -1:
          n[NOUTFILE]++;
          break; /* Error */
      }
      /* Set approx position in RINEX option */
      if (type == 1 && !opt->autopos && norm(opt->apppos, 3) <= 0.0) {
        setopt_apppos(str, opt);
      }
    }
    /* Close stream file */
    close_strfile(str);
  }
  /* Close output files */
  closefile(ofp, opt, str->nav);

  /* Remove empty output files */
  for (int i = 0; i < NOUTFILE; i++) {
    if (ofp[i] && n[i] <= 0) remove(ofile[i]);
  }
  showstat(sess, opt->tstart, opt->tend, n);

  /* Unset RINEX options comments */
  unsetopt_file(opt);

  free_strfile(str);
  for (int i = 0; i < MAXEXFILE; i++) free(epath[i]);

  return abort ? -1 : 1;
}
/* RINEX converter -------------------------------------------------------------
 * Convert receiver log file to RINEX obs/nav, SBAS log files
 * Args   : int    format I      receiver raw format (STRFMT_???)
 *          rnxopt_t *opt IO     RINEX options (see below)
 *          char   *file  I      RTCM, receiver raw or RINEX file
 *                               (wild-cards (*) are expanded)
 *          char   **ofile IO    output files
 *                               ofile[0] RINEX OBS file   ("": no output)
 *                               ofile[1] RINEX NAV file   ("": no output)
 *                               ofile[2] RINEX GNAV file  ("": no output)
 *                               ofile[3] RINEX HNAV file  ("": no output)
 *                               ofile[4] RINEX QNAV file  ("": no output)
 *                               ofile[5] RINEX LNAV file  ("": no output)
 *                               ofile[6] RINEX CNAV file  ("": no output)
 *                               ofile[7] RINEX INAV file  ("": no output)
 *                               ofile[8] SBAS log file    ("": no output)
 * Return : status (1:ok,0:error,-1:abort)
 * Notes  : the following members of opt are replaced by information in last
 *          converted RINEX: opt->tstart, opt->tend, opt->obstype, opt->nobs
 *          keywords in ofile[] are replaced by first observation date/time and
 *          station ID (%r)
 *          the order of wild-card expanded files must be in-order by time
 *----------------------------------------------------------------------------*/
extern int convrnx(int format, rnxopt_t *opt, const char *file, char **ofile) {
  gtime_t t0 = {0};
  rnxopt_t opt_ = *opt;

  trace(3, "convrnx: format=%d file=%s ofile=%s %s %s %s %s %s %s %s %s\n", format, file, ofile[0],
        ofile[1], ofile[2], ofile[3], ofile[4], ofile[5], ofile[6], ofile[7], ofile[8]);

  showmsg("");

  /* Disable systems according to RINEX version */
  int sys_GRS = SYS_GPS | SYS_GLO | SYS_SBS;
  if (opt->rnxver <= 210)
    opt_.navsys &= sys_GRS;
  else if (opt->rnxver <= 211)
    opt_.navsys &= sys_GRS | SYS_GAL;
  else if (opt->rnxver <= 212)
    opt_.navsys &= sys_GRS | SYS_GAL | SYS_CMP;
  else if (opt->rnxver <= 300)
    opt_.navsys &= sys_GRS | SYS_GAL;
  else if (opt->rnxver <= 301)
    opt_.navsys &= sys_GRS | SYS_GAL | SYS_CMP;
  else if (opt->rnxver <= 302)
    opt_.navsys &= sys_GRS | SYS_GAL | SYS_CMP | SYS_QZS;

  /* Disable frequency according to RINEX version */
  if (opt->rnxver <= 210) opt_.freqtype &= 0x3;

  int stat = 1;
  if (opt->ts.time == 0 || opt->te.time == 0 || opt->tunit <= 0.0) {
    /* Single session */
    opt_.tstart = opt_.tend = t0;
    stat = convrnx_s(0, format, &opt_, file, ofile);
  } else if (timediff(opt->ts, opt->te) < 0.0) {
    /* Multiple session */
    double tu = opt->tunit < 86400.0 ? opt->tunit : 86400.0;
    int week;
    double ts = tu * (int)floor(time2gpst(opt->ts, &week) / tu);

    for (int i = 0;; i++) { /* For each session */
      opt_.ts = gpst2time(week, ts + i * tu);
      opt_.te = timeadd(opt_.ts, tu);
      if (opt->trtcm.time) {
        opt_.trtcm = timeadd(opt->trtcm, timediff(opt_.ts, opt->ts));
      }
      if (timediff(opt_.ts, opt->te) > -opt->ttol) break;

      if (timediff(opt_.ts, opt->ts) < 0.0) opt_.ts = opt->ts;
      if (timediff(opt_.te, opt->te) > 0.0) opt_.te = opt->te;
      opt_.tstart = opt_.tend = t0;
      if ((stat = convrnx_s(i + 1, format, &opt_, file, ofile)) < 0) break;
    }
  } else {
    showmsg("no period");
    return 0;
  }
  /* Output start and end time */
  opt->tstart = opt_.tstart;
  opt->tend = opt_.tend;

  return stat;
}
