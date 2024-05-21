/*------------------------------------------------------------------------------
 * options.c : options functions
 *
 *          Copyright (C) 2010-2020 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision:$ $Date:$
 * History : 2010/07/20  1.1  moved from postpos.c
 *                            added api:
 *                                searchopt(),str2opt(),opt2str(),opt2buf(),
 *                                loadopts(),saveopts(),resetsysopts(),
 *                                getsysopts(),setsysopts()
 *           2010/09/11  1.2  add options
 *                                pos2-elmaskhold,pos1->snrmaskena
 *                                pos1-snrmask1,2,3
 *           2013/03/11  1.3  add pos1-posopt1,2,3,4,5,pos2-syncsol
 *                                misc-rnxopt1,2,pos1-snrmask_r,_b,_L1,_L2,_L5
 *           2014/10/21  1.4  add pos2-bdsarmode
 *           2015/02/20  1.4  add ppp-fixed as pos1-posmode option
 *           2015/05/10  1.5  add pos2-arthres1,2,3,4
 *           2015/05/31  1.6  add pos2-armaxiter, pos1-posopt6
 *                            add selection precise for pos1-pospot3
 *           2015/11/26  1.7  modify pos1-frequency 4:l1+l2+l5+l6 -> l1+l5
 *           2015/12/05  1.8  add misc-pppopt
 *           2016/06/10  1.9  add ant2-maxaveep,ant2-initrst
 *           2016/07/31  1.10 add out-outsingle,out-maxsolstd
 *           2017/06/14  1.11 add out-outvel
 *           2020/11/30  1.12 change options pos1-frequency, pos1-ionoopt,
 *                             pos1-tropopt, pos1-sateph, pos1-navsys,
 *                             pos2-gloarmode,
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include "rtklib.h"

/* System options buffer -----------------------------------------------------*/
static prcopt_t prcopt_;
static solopt_t solopt_;
static filopt_t filopt_;
static int antpostype_[2];
static long double elmask_, elmaskar_, elmaskhold_;
static long double antpos_[2][3];
static char exsats_[1024];
static char snrmask_[NFREQ][1024];

/* System options table ------------------------------------------------------*/
#define SWTOPT "0:off,1:on"
#define MODOPT                                                                                 \
  "0:single,1:dgps,2:kinematic,3:static,4:static-start,5:movingbase,6:fixed,7:ppp-kine,8:ppp-" \
  "static,9:ppp-fixed"
#define FRQOPT "1:l1,2:l1+l2,3:l1+l2+l5,4:l1+l2+l5+l6"
#define TYPOPT "0:forward,1:backward,2:combined,3:combined-nophasereset"
#define IONOPT "0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec,5:ionex-tec,6:qzs-brdc"
#define TRPOPT "0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad"
#define EPHOPT "0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom"
#define NAVOPT "1:gps+2:sbas+4:glo+8:gal+16:qzs+32:bds+64:navic"
#define GAROPT "0:off,1:on,2:autocal,3:fix-and-hold"
#define WEIGHTOPT "0:elevation,1:snr"
#define SOLOPT "0:llh,1:xyz,2:enu,3:nmea"
#define TSYOPT "0:gpst,1:utc,2:jst"
#define TFTOPT "0:tow,1:hms"
#define DFTOPT "0:deg,1:dms"
#define HGTOPT "0:ellipsoidal,1:geodetic"
#define GEOOPT "0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000"
#define STAOPT "0:all,1:single"
#define STSOPT "0:off,1:state,2:residual"
#define ARMOPT "0:off,1:continuous,2:instantaneous,3:fix-and-hold"
#define POSOPT "0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw"
#define TIDEOPT "0:off,1:on,2:otl"
#define PHWOPT "0:off,1:on,2:precise"

EXPORT opt_t sysopts[] = {
    {"pos1-posmode", 3, (void *)&prcopt_.mode, 0, MODOPT},
    {"pos1-frequency", 3, (void *)&prcopt_.nf, 0, FRQOPT},
    {"pos1-soltype", 3, (void *)&prcopt_.soltype, 0, TYPOPT},
    {"pos1-elmask", 1, (void *)&elmask_, 0, "deg"},
    {"pos1-snrmask_r", 3, (void *)&prcopt_.snrmask.ena[0], 0, SWTOPT},
    {"pos1-snrmask_b", 3, (void *)&prcopt_.snrmask.ena[1], 0, SWTOPT},
    {"pos1-snrmask_L1", 2, (void *)snrmask_[0], sizeof(snrmask_[0]), ""},
    {"pos1-snrmask_L2", 2, (void *)snrmask_[1], sizeof(snrmask_[1]), ""},
    {"pos1-snrmask_L5", 2, (void *)snrmask_[2], sizeof(snrmask_[2]), ""},
    {"pos1-dynamics", 3, (void *)&prcopt_.dynamics, 0, SWTOPT},
    {"pos1-tidecorr", 3, (void *)&prcopt_.tidecorr, 0, TIDEOPT},
    {"pos1-ionoopt", 3, (void *)&prcopt_.ionoopt, 0, IONOPT},
    {"pos1-tropopt", 3, (void *)&prcopt_.tropopt, 0, TRPOPT},
    {"pos1-sateph", 3, (void *)&prcopt_.sateph, 0, EPHOPT},
    {"pos1-posopt1", 3, (void *)&prcopt_.posopt[0], 0, SWTOPT},
    {"pos1-posopt2", 3, (void *)&prcopt_.posopt[1], 0, SWTOPT},
    {"pos1-posopt3", 3, (void *)&prcopt_.posopt[2], 0, PHWOPT},
    {"pos1-posopt4", 3, (void *)&prcopt_.posopt[3], 0, SWTOPT},
    {"pos1-posopt5", 3, (void *)&prcopt_.posopt[4], 0, SWTOPT},
    {"pos1-posopt6", 3, (void *)&prcopt_.posopt[5], 0, SWTOPT},
    {"pos1-exclsats", 2, (void *)exsats_, sizeof(exsats_), "prn ..."},
    {"pos1-navsys", 0, (void *)&prcopt_.navsys, 0, NAVOPT},

    {"pos2-armode", 3, (void *)&prcopt_.modear, 0, ARMOPT},
    {"pos2-gloarmode", 3, (void *)&prcopt_.glomodear, 0, GAROPT},
    {"pos2-bdsarmode", 3, (void *)&prcopt_.bdsmodear, 0, SWTOPT},
    {"pos2-arfilter", 3, (void *)&prcopt_.arfilter, 0, SWTOPT},
    {"pos2-arthres", 1, (void *)&prcopt_.thresar[0], 0, ""},
    {"pos2-arthresmin", 1, (void *)&prcopt_.thresar[5], 0, ""},
    {"pos2-arthresmax", 1, (void *)&prcopt_.thresar[6], 0, ""},
    {"pos2-arthres1", 1, (void *)&prcopt_.thresar[1], 0, ""},
    {"pos2-arthres2", 1, (void *)&prcopt_.thresar[2], 0, ""},
    {"pos2-arthres3", 1, (void *)&prcopt_.thresar[3], 0, ""},
    {"pos2-arthres4", 1, (void *)&prcopt_.thresar[4], 0, ""},
    {"pos2-varholdamb", 1, (void *)&prcopt_.varholdamb, 0, "cyc^2"},
    {"pos2-gainholdamb", 1, (void *)&prcopt_.gainholdamb, 0, ""},
    {"pos2-arlockcnt", 0, (void *)&prcopt_.minlock, 0, ""},
    {"pos2-minfixsats", 0, (void *)&prcopt_.minfixsats, 0, ""},
    {"pos2-minholdsats", 0, (void *)&prcopt_.minholdsats, 0, ""},
    {"pos2-mindropsats", 0, (void *)&prcopt_.mindropsats, 0, ""},
    {"pos2-arelmask", 1, (void *)&elmaskar_, 0, "deg"},
    {"pos2-arminfix", 0, (void *)&prcopt_.minfix, 0, ""},
    {"pos2-armaxiter", 0, (void *)&prcopt_.armaxiter, 0, ""},
    {"pos2-elmaskhold", 1, (void *)&elmaskhold_, 0, "deg"},
    {"pos2-aroutcnt", 0, (void *)&prcopt_.maxout, 0, ""},
    {"pos2-maxage", 1, (void *)&prcopt_.maxtdiff, 0, "s"},
    {"pos2-syncsol", 3, (void *)&prcopt_.syncsol, 0, SWTOPT},
    {"pos2-slipthres", 1, (void *)&prcopt_.thresslip, 0, "m"},
    {"pos2-dopthres", 1, (void *)&prcopt_.thresdop, 0, "m"},
    {"pos2-rejionno", 1, (void *)&prcopt_.maxinno[0], 0, "m"},
    {"pos2-rejcode", 1, (void *)&prcopt_.maxinno[1], 0, "m"},
    {"pos2-niter", 0, (void *)&prcopt_.niter, 0, ""},
    {"pos2-baselen", 1, (void *)&prcopt_.baseline[0], 0, "m"},
    {"pos2-basesig", 1, (void *)&prcopt_.baseline[1], 0, "m"},

    {"out-solformat", 3, (void *)&solopt_.posf, 0, SOLOPT},
    {"out-outhead", 3, (void *)&solopt_.outhead, 0, SWTOPT},
    {"out-outopt", 3, (void *)&solopt_.outopt, 0, SWTOPT},
    {"out-outvel", 3, (void *)&solopt_.outvel, 0, SWTOPT},
    {"out-timesys", 3, (void *)&solopt_.times, 0, TSYOPT},
    {"out-timeform", 3, (void *)&solopt_.timef, 0, TFTOPT},
    {"out-timendec", 0, (void *)&solopt_.timeu, 0, ""},
    {"out-degform", 3, (void *)&solopt_.degf, 0, DFTOPT},
    {"out-fieldsep", 2, (void *)&solopt_.sep, sizeof(solopt_.sep), ""},
    {"out-outsingle", 3, (void *)&prcopt_.outsingle, 0, SWTOPT},
    {"out-maxsolstd", 1, (void *)&solopt_.maxsolstd, 0, "m"},
    {"out-height", 3, (void *)&solopt_.height, 0, HGTOPT},
    {"out-geoid", 3, (void *)&solopt_.geoid, 0, GEOOPT},
    {"out-solstatic", 3, (void *)&solopt_.solstatic, 0, STAOPT},
    {"out-nmeaintv1", 1, (void *)&solopt_.nmeaintv[0], 0, "s"},
    {"out-nmeaintv2", 1, (void *)&solopt_.nmeaintv[1], 0, "s"},
    {"out-outstat", 3, (void *)&solopt_.sstat, 0, STSOPT},
    {"stats-eratio1", 1, (void *)&prcopt_.eratio[0], 0, ""},
    {"stats-eratio2", 1, (void *)&prcopt_.eratio[1], 0, ""},
    {"stats-eratio5", 1, (void *)&prcopt_.eratio[2], 0, ""},
    {"stats-errphase", 1, (void *)&prcopt_.err[1], 0, "m"},
    {"stats-errphaseel", 1, (void *)&prcopt_.err[2], 0, "m"},
    {"stats-errphasebl", 1, (void *)&prcopt_.err[3], 0, "m/10km"},
    {"stats-errdoppler", 1, (void *)&prcopt_.err[4], 0, "Hz"},
    {"stats-snrmax", 1, (void *)&prcopt_.err[5], 0, "dB.Hz"},
    {"stats-errsnr", 1, (void *)&prcopt_.err[6], 0, "m"},
    {"stats-errrcv", 1, (void *)&prcopt_.err[7], 0, " "},
    {"stats-stdbias", 1, (void *)&prcopt_.std[0], 0, "m"},
    {"stats-stdiono", 1, (void *)&prcopt_.std[1], 0, "m"},
    {"stats-stdtrop", 1, (void *)&prcopt_.std[2], 0, "m"},
    {"stats-prnaccelh", 1, (void *)&prcopt_.prn[3], 0, "m/s^2"},
    {"stats-prnaccelv", 1, (void *)&prcopt_.prn[4], 0, "m/s^2"},
    {"stats-prnbias", 1, (void *)&prcopt_.prn[0], 0, "m"},
    {"stats-prniono", 1, (void *)&prcopt_.prn[1], 0, "m"},
    {"stats-prntrop", 1, (void *)&prcopt_.prn[2], 0, "m"},
    {"stats-prnpos", 1, (void *)&prcopt_.prn[5], 0, "m"},
    {"stats-clkstab", 1, (void *)&prcopt_.sclkstab, 0, "s/s"},

    {"ant1-postype", 3, (void *)&antpostype_[0], 0, POSOPT},
    {"ant1-pos1", 1, (void *)&antpos_[0][0], 0, "deg|m"},
    {"ant1-pos2", 1, (void *)&antpos_[0][1], 0, "deg|m"},
    {"ant1-pos3", 1, (void *)&antpos_[0][2], 0, "m|m"},
    {"ant1-anttype", 2, (void *)prcopt_.anttype[0], sizeof(prcopt_.anttype[0]), ""},
    {"ant1-antdele", 1, (void *)&prcopt_.antdel[0][0], 0, "m"},
    {"ant1-antdeln", 1, (void *)&prcopt_.antdel[0][1], 0, "m"},
    {"ant1-antdelu", 1, (void *)&prcopt_.antdel[0][2], 0, "m"},

    {"ant2-postype", 3, (void *)&antpostype_[1], 0, POSOPT},
    {"ant2-pos1", 1, (void *)&antpos_[1][0], 0, "deg|m"},
    {"ant2-pos2", 1, (void *)&antpos_[1][1], 0, "deg|m"},
    {"ant2-pos3", 1, (void *)&antpos_[1][2], 0, "m|m"},
    {"ant2-anttype", 2, (void *)prcopt_.anttype[1], sizeof(prcopt_.anttype[1]), ""},
    {"ant2-antdele", 1, (void *)&prcopt_.antdel[1][0], 0, "m"},
    {"ant2-antdeln", 1, (void *)&prcopt_.antdel[1][1], 0, "m"},
    {"ant2-antdelu", 1, (void *)&prcopt_.antdel[1][2], 0, "m"},
    {"ant2-maxaveep", 0, (void *)&prcopt_.maxaveep, 0, ""},
    {"ant2-initrst", 3, (void *)&prcopt_.initrst, 0, SWTOPT},

    {"misc-timeinterp", 3, (void *)&prcopt_.intpref, 0, SWTOPT},
    {"misc-sbasatsel", 0, (void *)&prcopt_.sbassatsel, 0, "0:all"},
    {"misc-rnxopt1", 2, (void *)prcopt_.rnxopt[0], sizeof(prcopt_.rnxopt[0]), ""},
    {"misc-rnxopt2", 2, (void *)prcopt_.rnxopt[1], sizeof(prcopt_.rnxopt[1]), ""},
    {"misc-pppopt", 2, (void *)prcopt_.pppopt, sizeof(prcopt_.pppopt), ""},

    {"file-satantfile", 2, (void *)&filopt_.satantp, sizeof(filopt_.satantp), ""},
    {"file-rcvantfile", 2, (void *)&filopt_.rcvantp, sizeof(filopt_.rcvantp), ""},
    {"file-staposfile", 2, (void *)&filopt_.stapos, sizeof(filopt_.stapos), ""},
    {"file-geoidfile", 2, (void *)&filopt_.geoid, sizeof(filopt_.geoid), ""},
    {"file-ionofile", 2, (void *)&filopt_.iono, sizeof(filopt_.iono), ""},
    {"file-dcbfile", 2, (void *)&filopt_.dcb, sizeof(filopt_.dcb), ""},
    {"file-eopfile", 2, (void *)&filopt_.eop, sizeof(filopt_.eop), ""},
    {"file-blqfile", 2, (void *)&filopt_.blq, sizeof(filopt_.blq), ""},
    {"file-tempdir", 2, (void *)&filopt_.tempdir, sizeof(filopt_.tempdir), ""},
    {"file-geexefile", 2, (void *)&filopt_.geexe, sizeof(filopt_.geexe), ""},
    {"file-solstatfile", 2, (void *)&filopt_.solstat, sizeof(filopt_.solstat), ""},
    {"file-tracefile", 2, (void *)&filopt_.trace, sizeof(filopt_.trace), ""},

    {"", 0, NULL, 0, ""} /* Terminator */
};
/* Discard space characters at tail ------------------------------------------*/
static void chop(char *str) {
  char *p = strchr(str, '#');
  if (p) *p = '\0'; /* Comment */
  for (p = str + strlen(str) - 1; p >= str && !isgraph((int)*p); p--) *p = '\0';
}
/* Enum to string ------------------------------------------------------------*/
static void enum2str(char *s, size_t size, const char *comment, int val) {
  char str[32];
  rtksnprintf(str, sizeof(str), "%d:", val);

  const char *p = strstr(comment, str);
  if (!p) {
    rtkcatprintf(s, size, "%d", val);
    return;
  }
  int n = strlen(str);
  const char *q = strchr(p + n, ',');
  if (!q && !(q = strchr(p + n, ')'))) {
    rtksubstrcat(s, size, p, n);
    return;
  }
  size_t end = q - p;
  rtkesubstrcat(s, size, p, n, end);
}
/* String to enum --------------------------------------------------------------
 * Note if str is empty then the first comment digit is returned.
 */
static bool str2enum(const char *str, const char *comment, int *val) {
  for (const char *p = comment;; p++) {
    p = strstr(p, str);
    if (!p) break;
    size_t i = p - comment;
    if (i < 1) continue;
    if (comment[--i] != ':') continue;
    /* Search for preceding digits */
    size_t j = i;
    while (j > 0) {
      char c = comment[j - 1];
      if (c < '0' || c > '9') break;
      j--;
    }
    if (j == i) continue; /* No digits found */
    return sscanf(comment + j, "%d", val) == 1;
  }
  char s[32];
  rtksnprintf(s, sizeof(s), "%.30s:", str);
  const char *p = strstr(comment, s);
  if (p) { /* Number */
    return sscanf(p, "%d", val) == 1;
  }
  return false;
}
/* Search option ---------------------------------------------------------------
 * Search option record
 * Args   : char   *name     I  option name
 *          opt_t  *opts     I  options table
 *                              (terminated with table[i].name="")
 * Return : option record (NULL: not found)
 *----------------------------------------------------------------------------*/
extern opt_t *searchopt(const char *name, const opt_t *opts) {
  trace(3, "searchopt: name=%s\n", name);

  for (int i = 0; *opts[i].name; i++) {
    if (strstr(opts[i].name, name)) return (opt_t *)(opts + i);
  }
  return NULL;
}
/* String to option value ------------------------------------------------------
 * Convert string to option value
 * Args   : opt_t  *opt      O  option
 *          char   *str      I  option value string
 * Return : status (true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool str2opt(opt_t *opt, const char *str) {
  switch (opt->format) {
    case 0:
      *(int *)opt->var = atoi(str);
      break;
    case 1:
      *(long double *)opt->var = strtold(str, NULL);
      break;
    case 2:
      if (strlen(str) + 1 > opt->vsize) return false;
      rtkstrcpy((char *)opt->var, opt->vsize, str);
      break;
    case 3:
      return str2enum(str, opt->comment, (int *)opt->var);
    default:
      return false;
  }
  return true;
}
/* Option value to string ------------------------------------------------------
 * Convert option value to string
 * Args   : opt_t  *opt      I  option
 *          char   *str      O  option value string
 *          size_t size      I  option value string size
 * Return : none
 * Note   : The output is appended to the buffer which must be nul terminated.
 *----------------------------------------------------------------------------*/
extern void opt2str(const opt_t *opt, char *str, size_t size) {
  trace(3, "opt2str : name=%s\n", opt->name);

  switch (opt->format) {
    case 0:
      rtkcatprintf(str, size, "%d", *(int *)opt->var);
      break;
    case 1:
      rtkcatprintf(str, size, "%.15Lg", *(long double *)opt->var);
      break;
    case 2:
      rtkcatprintf(str, size, "%s", (char *)opt->var);
      break;
    case 3:
      enum2str(str, size, opt->comment, *(int *)opt->var);
      break;
  }
}
/* Option to string ------------------------------------------------------------
 * Convert option to string (keyword=value # comment)
 * Args   : opt_t  *opt      I  option
 *          char   *buff     O  option string
 *          size_t size      I  option string buffer size
 * Return : none
 * Note   : The output is appended to the buffer which must be nul terminated.
 *----------------------------------------------------------------------------*/
extern void opt2buf(const opt_t *opt, char *buff, size_t size) {
  trace(3, "opt2buf : name=%s\n", opt->name);

  rtkcatprintf(buff, size, "%-18s =", opt->name);
  opt2str(opt, buff, size);
  if (*opt->comment) {
    int n = 30 - (int)strlen(buff);
    if (n > 0) {
      rtkcatprintf(buff, size, "%*s", n, "");
    }
    rtkcatprintf(buff, size, " # (%s)", opt->comment);
  }
}
/* Load options ----------------------------------------------------------------
 * Load options from file
 * Args   : char   *file     I  options file
 *          opt_t  *opts     IO options table
 *                              (terminated with table[i].name="")
 * Return : status (true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool loadopts(const char *file, opt_t *opts) {
  trace(3, "loadopts: file=%s\n", file);

  FILE *fp = fopen(file, "r");
  if (!fp) {
    trace(1, "loadopts: options file open error (%s)\n", file);
    return false;
  }
  int n = 0;
  char buff[2048];
  while (fgets(buff, sizeof(buff), fp)) {
    n++;
    chop(buff);

    if (buff[0] == '\0') continue;

    char *p = strstr(buff, "=");
    if (!p) {
      fprintf(stderr, "invalid option %s (%s:%d)\n", buff, file, n);
      continue;
    }
    *p++ = '\0';
    chop(buff);
    opt_t *opt = searchopt(buff, opts);
    if (!opt) continue;

    if (!str2opt(opt, p)) {
      fprintf(stderr, "invalid option value %s (%s:%d)\n", buff, file, n);
      continue;
    }
  }
  fclose(fp);

  return true;
}
/* Save options to file --------------------------------------------------------
 * Save options to file
 * Args   : char   *file     I  options file
 *          char   *mode     I  write mode ("w":overwrite,"a":append);
 *          char   *comment  I  header comment (NULL: no comment)
 *          opt_t  *opts     I  options table
 *                              (terminated with table[i].name="")
 * Return : status (true:ok,false:error)
 *----------------------------------------------------------------------------*/
extern bool saveopts(const char *file, const char *mode, const char *comment, const opt_t *opts) {
  trace(3, "saveopts: file=%s mode=%s\n", file, mode);

  FILE *fp = fopen(file, mode);
  if (!fp) {
    trace(1, "saveopts: options file open error (%s)\n", file);
    return false;
  }
  if (comment) fprintf(fp, "# %s\n\n", comment);

  for (int i = 0; *opts[i].name; i++) {
    char buff[2048];
    buff[0] = '\0';
    opt2buf(opts + i, buff, sizeof(buff));
    fprintf(fp, "%s\n", buff);
  }
  fclose(fp);
  return true;
}
/* System options buffer to options ------------------------------------------*/
static void buff2sysopts(void) {
  prcopt_.elmin = elmask_ * D2R;
  prcopt_.elmaskar = elmaskar_ * D2R;
  prcopt_.elmaskhold = elmaskhold_ * D2R;

  for (int i = 0; i < 2; i++) {
    int *ps = i == 0 ? &prcopt_.rovpos : &prcopt_.refpos;
    long double *rr = i == 0 ? prcopt_.ru : prcopt_.rb;

    if (antpostype_[i] == 0) { /* lat/lon/hgt */
      *ps = 0;
      long double pos[3];
      pos[0] = antpos_[i][0] * D2R;
      pos[1] = antpos_[i][1] * D2R;
      pos[2] = antpos_[i][2];
      pos2ecef(pos, rr);
    } else if (antpostype_[i] == 1) { /* xyz-ecef */
      *ps = 0;
      rr[0] = antpos_[i][0];
      rr[1] = antpos_[i][1];
      rr[2] = antpos_[i][2];
    } else
      *ps = antpostype_[i] - 1;
  }
  /* Excluded satellites */
  for (int i = 0; i < MAXSAT; i++) prcopt_.exsats[i] = 0;
  if (exsats_[0] != '\0') {
    char buff[1024];
    rtkstrcpy(buff, sizeof(buff), exsats_);
    char *q;
    for (char *p = strtok_r(buff, " ", &q); p; p = strtok_r(NULL, " ", &q)) {
      const char *id;
      if (*p == '+')
        id = p + 1;
      else
        id = p;
      int sat = satid2no(id);
      if (!sat) continue;
      prcopt_.exsats[sat - 1] = *p == '+' ? 2 : 1;
    }
  }
  /* Snrmask */
  for (int i = 0; i < NFREQ; i++) {
    for (int j = 0; j < 9; j++) prcopt_.snrmask.mask[i][j] = 0.0L;
    char buff[1024];
    rtkstrcpy(buff, sizeof(buff), snrmask_[i]);
    char *q;
    const char *p = strtok_r(buff, ",", &q);
    for (int j = 0; p && j < 9; p = strtok_r(NULL, ",", &q)) {
      prcopt_.snrmask.mask[i][j++] = strtold(p, NULL);
    }
  }
  /* Guard number of frequencies */
  if (prcopt_.nf > NFREQ) {
    fprintf(stderr, "Number of frequencies %d limited to %d, rebuild with NFREQ=%d\n", prcopt_.nf,
            NFREQ, prcopt_.nf);
    prcopt_.nf = NFREQ;
  }
  /* Number of frequency (4:L1+L5) TODO ????*/
  /*if (prcopt_.nf==4) {
      prcopt_.nf=3;
      prcopt_.freqopt=1;
  }*/
}
/* Options to system options buffer ------------------------------------------*/
static void sysopts2buff(void) {
  elmask_ = prcopt_.elmin * R2D;
  elmaskar_ = prcopt_.elmaskar * R2D;
  elmaskhold_ = prcopt_.elmaskhold * R2D;

  for (int i = 0; i < 2; i++) {
    const int *ps = i == 0 ? &prcopt_.rovpos : &prcopt_.refpos;
    const long double *rr = i == 0 ? prcopt_.ru : prcopt_.rb;

    if (*ps == 0) {
      antpostype_[i] = 0;
      long double pos[3];
      ecef2pos(rr, pos);
      antpos_[i][0] = pos[0] * R2D;
      antpos_[i][1] = pos[1] * R2D;
      antpos_[i][2] = pos[2];
    } else
      antpostype_[i] = *ps + 1;
  }
  /* Excluded satellites */
  exsats_[0] = '\0';
  for (int sat = 1; sat <= MAXSAT && strlen(exsats_) < sizeof(exsats_) - 32; sat++) {
    if (prcopt_.exsats[sat - 1]) {
      char id[8];
      satno2id(sat, id);
      rtkcatprintf(exsats_, sizeof(exsats_), "%s%s%s", strlen(exsats_) == 0 ? "" : " ",
                   prcopt_.exsats[sat - 1] == 2 ? "+" : "", id);
    }
  }
  /* Snrmask */
  for (int i = 0; i < NFREQ; i++) {
    snrmask_[i][0] = '\0';
    for (int j = 0; j < 9; j++) {
      rtkcatprintf(snrmask_[i], sizeof(snrmask_[i]), "%s%.0Lf", j > 0 ? "," : "",
                   prcopt_.snrmask.mask[i][j]);
    }
  }
  /* Number of frequency (4:L1+L5) TODO ???? */
  /*if (prcopt_.nf==3&&prcopt_.freqopt==1) {
      prcopt_.nf=4;
      prcopt_.freqopt=0;
  }*/
}
/* Reset system options to default ---------------------------------------------
 * Reset system options to default
 * Args   : none
 * Return : none
 *----------------------------------------------------------------------------*/
extern void resetsysopts(void) {
  trace(3, "resetsysopts:\n");

  prcopt_ = prcopt_default;
  solopt_ = solopt_default;
  filopt_.satantp[0] = '\0';
  filopt_.rcvantp[0] = '\0';
  filopt_.stapos[0] = '\0';
  filopt_.geoid[0] = '\0';
  filopt_.dcb[0] = '\0';
  filopt_.blq[0] = '\0';
  filopt_.solstat[0] = '\0';
  filopt_.trace[0] = '\0';
  for (int i = 0; i < 2; i++) antpostype_[i] = 0;
  elmask_ = 15.0L;
  elmaskar_ = 0.0L;
  elmaskhold_ = 0.0L;
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 3; j++) {
      antpos_[i][j] = 0.0L;
    }
  exsats_[0] = '\0';
}
/* Get system options ----------------------------------------------------------
 * Get system options
 * Args   : prcopt_t *popt   IO processing options (NULL: no output)
 *          solopt_t *sopt   IO solution options   (NULL: no output)
 *          folopt_t *fopt   IO file options       (NULL: no output)
 * Return : none
 * Notes  : to load system options, use loadopts() before calling the function
 *----------------------------------------------------------------------------*/
extern void getsysopts(prcopt_t *popt, solopt_t *sopt, filopt_t *fopt) {
  trace(3, "getsysopts:\n");

  buff2sysopts();
  if (popt) *popt = prcopt_;
  if (sopt) *sopt = solopt_;
  if (fopt) *fopt = filopt_;
}
/* Set system options ----------------------------------------------------------
 * Set system options
 * Args   : prcopt_t *prcopt I  processing options (NULL: default)
 *          solopt_t *solopt I  solution options   (NULL: default)
 *          filopt_t *filopt I  file options       (NULL: default)
 * Return : none
 * Notes  : to save system options, use saveopts() after calling the function
 *----------------------------------------------------------------------------*/
extern void setsysopts(const prcopt_t *prcopt, const solopt_t *solopt, const filopt_t *filopt) {
  trace(3, "setsysopts:\n");

  resetsysopts();
  if (prcopt) prcopt_ = *prcopt;
  if (solopt) solopt_ = *solopt;
  if (filopt) filopt_ = *filopt;
  sysopts2buff();
}
