/*------------------------------------------------------------------------------
 * convbin.c : convert receiver binary log file to RINEX obs/nav, SBAS messages
 *
 *          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
 *
 * Options : -DWIN32 use windows file path separator
 *
 * Version : $Revision: 1.1 $ $Date: 2008/07/17 22:13:04 $
 * History : 2008/06/22 1.0 new
 *           2009/06/17 1.1 support GLONASS
 *           2009/12/19 1.2 fix bug on disable of GLONASS
 *                          fix bug on improper header for RTCM2 and RTCM3
 *           2010/07/18 1.3 add option -v, -t, -h, -x
 *           2011/01/15 1.4 add option -ro, -hc, -hm, -hn, -ht, -ho, -hr, -ha,
 *                            -hp, -hd, -y, -c, -q
 *                          support GW10 and JAVAD receiver, Galileo, QZSS
 *                          support RINEX file name convention
 *           2012/10/22 1.5 add option -scan, -oi, -ot, -ol
 *                          change default RINEX version to 2.11
 *                          fix bug on default output directory (/ -> .)
 *                          support Galileo nav (LNAV) output
 *                          support compass
 *           2012/11/19 1.6 fix bug on setting code mask in RINEX options
 *           2013/02/18 1.7 support binex
 *           2013/05/19 1.8 support auto format for file path with wild-card
 *           2014/02/08 1.9 add option -span -trace -mask
 *           2014/08/26 1.10 add Trimble RT17 support
 *           2014/12/26 1.11 add option -nomask
 *           2016/01/23 1.12 enable septentrio
 *           2016/05/25 1.13 fix bug on initializing output file paths in
 *                           convbin()
 *           2016/06/09 1.14 fix bug on output file with -v 3.02
 *           2016/07/01 1.15 support log format CMR/CMR+
 *           2016/07/31 1.16 add option -halfc
 *           2017/05/26 1.17 add input format tersus
 *           2017/06/06 1.18 fix bug on output BeiDou and IRNSS nav files
 *                           add option -tt
 *           2018/10/10 1.19 default options are changed.
 *                             scan input file: off - on
 *                             number of freq: 2 -> 3
 *                           add option -noscan
 *           2020/11/30 1.20 include NavIC in default systems
 *                           force option -scan
 *                           delete option -noscan
 *                           suppress warnings
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "rtklib.h"

#define PRGNAME "CONVBIN"
#define TRACEFILE "convbin.trace"
#define NOUTFILE 9 /* Number of output files */

/* Help text -----------------------------------------------------------------*/
static const char *help[] = {
    "",
    " Synopsis",
    "",
    " convbin [option ...] file",
    "",
    " Description",
    "",
    " Convert RTCM, receiver raw data log and RINEX file to RINEX and SBAS/LEX",
    " message file. SBAS message file complies with RTKLIB SBAS/LEX message",
    " format. It supports the following messages or files.",
    "",
    " RTCM 2                : Type 1, 3, 9, 14, 16, 17, 18, 19, 22",
    " RTCM 3                : Type 1002, 1004, 1005, 1006, 1010, 1012, 1019, 1020",
    "                         Type 1071-1127 (MSM except for compact msg)",
    " NovAtel OEMV/4,OEMStar: RANGECMPB, RANGEB, RAWEPHEMB, IONUTCB, RAWWASSFRAMEB",
    " u-blox LEA-4T/5T/6T/8/9  : RXM-RAW, RXM-RAWX, RXM-SFRB",
    " Swift Piksi Multi     : ",
    " Hemisphere            : BIN76, BIN80, BIN94, BIN95, BIN96",
    " SkyTraq S1315F        : msg0xDD, msg0xE0, msg0xDC",
    " GW10                  : msg0x08, msg0x03, msg0x27, msg0x20",
    " Javad                 : [R*],[r*],[*R],[*r],[P*],[p*],[*P],[*p],[D*],[*d],",
    "                         [E*],[*E],[F*],[TC],[GE],[NE],[EN],[QE],[UO],[IO],",
    "                         [WD]",
    " NVS                   : BINR",
    " BINEX                 : big-endian, regular CRC, forward record (0xE2)",
    "                         0x01-01,0x01-02,0x01-03,0x01-04,0x01-06,0x7f-05",
    " Trimble               : RT17",
    " Septentrio            : SBF",
    " RINEX                 : OBS, NAV, GNAV, HNAV, LNAV, QNAV",
    "",
    " Options [default]",
    "",
    "     file         input receiver binary log file",
    "     -ts y/m/d h:m:s  start time [all]",
    "     -te y/m/d h:m:s  end time [all]",
    "     -tr y/m/d h:m:s  approximated time for RTCM",
    "     -ti tint     observation data interval (s) [all]",
    "     -tt ttol     observation data epoch tolerance (s) [0.005]",
    "     -span span   time span (h) [all]",
    "     -r format    log format type",
    "                  rtcm2= RTCM 2",
    "                  rtcm3= RTCM 3",
    "                  nov  = NovAtel OEM/4/V/6/7,OEMStar",
    "                  ubx  = ublox LEA-4T/5T/6T/7T/M8T/F9",
    "                  sbp  = Swift Navigation SBP",
    "                  hemis= Hemisphere Eclipse/Crescent",
    "                  stq  = SkyTraq S1315F",
    "                  javad= Javad GREIS",
    "                  nvs  = NVS NV08C BINR",
    "                  binex= BINEX",
    "                  rt17 = Trimble RT17",
    "                  sbf  = Septentrio SBF",
    "                  rinex= RINEX",
    "     -ro opt      receiver options",
    "     -f freq      number of frequencies [5]",
    "     -hc comment  rinex header: comment line",
    "     -hm marker   rinex header: marker name",
    "     -hn markno   rinex header: marker number",
    "     -ht marktype rinex header: marker type",
    "     -ho observ   rinex header: observer name and agency separated by /",
    "     -hr rec      rinex header: receiver number, type and version separated by /",
    "     -ha ant      rinex header: antenna number and type separated by /",
    "     -hp pos      rinex header: approx position x/y/z separated by /",
    "     -hd delta    rinex header: antenna delta h/e/n separated by /",
    "     -v ver       rinex version [3.04]",
    "     -od          include doppler frequency in rinex obs [on]",
    "     -os          include snr in rinex obs [on]",
    "     -oi          include iono correction in rinex nav header [off]",
    "     -ot          include time correction in rinex nav header [off]",
    "     -ol          include leap seconds in rinex nav header [off]",
    "     -halfc       half-cycle ambiguity correction [off]",
    "     -mask   [sig[,...]] signal mask(s) (sig={G|R|E|J|S|C|I}L{1C|1P|1W|...})",
    "     -nomask [sig[,...]] signal no mask (same as above)",
    "     -x sat       exclude satellite",
    "     -y sys       exclude systems (G:GPS,R:GLO,E:GAL,J:QZS,S:SBS,C:BDS,I:IRN)",
    "     -d dir       output directory [same as input file]",
    "     -c staid     use RINEX file name convention with staid [off]",
    "     -o ofile     output RINEX OBS file",
    "     -n nfile     output RINEX NAV file",
    "     -g gfile     output RINEX GNAV file",
    "     -h hfile     output RINEX HNAV file",
    "     -q qfile     output RINEX QNAV file",
    "     -l lfile     output RINEX LNAV file",
    "     -b cfile     output RINEX CNAV file",
    "     -i ifile     output RINEX INAV file",
    "     -s sfile     output SBAS message file",
    "     -trace level output trace level [off]",
    "",
    " If any output file specified, default output files (<file>.obs,",
    " <file>.nav, <file>.gnav, <file>.hnav, <file>.qnav, <file>.lnav,",
    " <file>.cnav, <file>.inav and <file>.sbs) are used. To obtain week number info",
    " for RTCM file, use -tr option to specify the approximated log start time.",
    " Without -tr option, the program obtains the week number from the time-tag file"
    " (if it exists) or the last modified time of the log file instead.",
    "",
    " If receiver type is not specified, type is recognized by the input",
    " file extension as follows.",
    "     *.rtcm2       RTCM 2",
    "     *.rtcm3       RTCM 3",
    "     *.gps         NovAtel OEM4/V/6/7,OEMStar",
    "     *.ubx         u-blox LEA-4T/5T/6T/7T/M8T/F9",
    "     *.sbp         Swift Navigation SBP",
    "     *.bin         Hemisphere Eclipse/Crescent",
    "     *.stq         SkyTraq S1315F",
    "     *.jps         Javad GREIS",
    "     *.bnx,*binex  BINEX",
    "     *.rt17        Trimble RT17",
    "     *.sbf         Septentrio SBF",
    "     *.obs,*.*o    RINEX OBS",
    "     *.rnx         RINEX OBS"
    "     *.nav,*.*n    RINEX NAV",
};
/* Print help ----------------------------------------------------------------*/
static void printhelp(void) {
  for (unsigned i = 0; i < sizeof(help) / sizeof(*help); i++) fprintf(stderr, "%s\n", help[i]);
  exit(0);
}
/* Show message --------------------------------------------------------------*/
extern int showmsg(const char *format, ...) {
  va_list arg;
  va_start(arg, format);
  vfprintf(stderr, format, arg);
  va_end(arg);
  fprintf(stderr, *format ? "\r" : "\n");
  return 0;
}
/* Convert main --------------------------------------------------------------*/
static bool convbin(int format, rnxopt_t *opt, const char *ifile, char **file, char *dir) {
  /* Replace wild-card (*) in input file by 0 */
  char ifile_[1024];
  rtkstrcpy(ifile_, sizeof(ifile_), ifile);
  for (char *p = ifile_; *p; p++)
    if (*p == '*') *p = '0';

  int def = !file[0] && !file[1] && !file[2] && !file[3] && !file[4] && !file[5] && !file[6] &&
            !file[7] && !file[8];

  /* Stack storage for the output file strings */
  char ofile_[NOUTFILE][FNSIZE], *ofile[NOUTFILE];
  for (int i = 0; i < NOUTFILE; i++) {
    ofile[i] = ofile_[i];
    *ofile[i] = '\0';
  }

  size_t ofile_size = sizeof(ofile_[0]);
  if (file[0])
    rtkstrcpy(ofile[0], ofile_size, file[0]);
  else if (*opt->staid) {
    rtkstrcpy(ofile[0], ofile_size, "%r%n0.%yO");
  } else if (def) {
    rtkstrcpy(ofile[0], ofile_size, ifile_);
    char *p = strrchr(ofile[0], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[0]), ".obs");
    else
      rtkstrcat(ofile[0], ofile_size, ".obs");
  }
  if (file[1])
    rtkstrcpy(ofile[1], ofile_size, file[1]);
  else if (*opt->staid) {
    rtkstrcpy(ofile[1], ofile_size, "%r%n0.%y");
    char *extnav = (opt->rnxver <= 299 || opt->navsys == SYS_GPS) ? "N" : "P";
    rtkstrcat(ofile[1], ofile_size, extnav);
  } else if (def) {
    rtkstrcpy(ofile[1], ofile_size, ifile_);
    char *p = strrchr(ofile[1], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[1]), ".nav");
    else
      rtkstrcat(ofile[1], ofile_size, ".nav");
  }
  if (file[2])
    rtkstrcpy(ofile[2], ofile_size, file[2]);
  else if (opt->rnxver <= 299 && *opt->staid) {
    rtkstrcpy(ofile[2], ofile_size, "%r%n0.%yG");
  } else if (opt->rnxver <= 299 && def) {
    rtkstrcpy(ofile[2], ofile_size, ifile_);
    char *p = strrchr(ofile[2], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[2]), ".gnav");
    else
      rtkstrcat(ofile[2], ofile_size, ".gnav");
  }
  if (file[3])
    rtkstrcpy(ofile[3], ofile_size, file[3]);
  else if (opt->rnxver <= 299 && *opt->staid) {
    rtkstrcpy(ofile[3], ofile_size, "%r%n0.%yH");
  } else if (opt->rnxver <= 299 && def) {
    rtkstrcpy(ofile[3], ofile_size, ifile_);
    char *p = strrchr(ofile[3], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[3]), ".hnav");
    else
      rtkstrcat(ofile[3], ofile_size, ".hnav");
  }
  if (file[4])
    rtkstrcpy(ofile[4], ofile_size, file[4]);
  else if (opt->rnxver <= 299 && *opt->staid) {
    rtkstrcpy(ofile[4], ofile_size, "%r%n0.%yQ");
  } else if (opt->rnxver <= 299 && def) {
    rtkstrcpy(ofile[4], ofile_size, ifile_);
    char *p = strrchr(ofile[4], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[4]), ".qnav");
    else
      rtkstrcat(ofile[4], ofile_size, ".qnav");
  }
  if (file[5])
    rtkstrcpy(ofile[5], ofile_size, file[5]);
  else if (opt->rnxver <= 299 && *opt->staid) {
    rtkstrcpy(ofile[5], ofile_size, "%r%n0.%yL");
  } else if (opt->rnxver <= 299 && def) {
    rtkstrcpy(ofile[5], ofile_size, ifile_);
    char *p = strrchr(ofile[5], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[5]), ".lnav");
    else
      rtkstrcat(ofile[5], ofile_size, ".lnav");
  }
  if (file[6])
    rtkstrcpy(ofile[6], ofile_size, file[6]);
  else if (opt->rnxver <= 299 && *opt->staid) {
    rtkstrcpy(ofile[6], ofile_size, "%r%n0.%yC");
  } else if (opt->rnxver <= 299 && def) {
    rtkstrcpy(ofile[6], ofile_size, ifile_);
    char *p = strrchr(ofile[6], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[6]), ".cnav");
    else
      rtkstrcat(ofile[6], ofile_size, ".cnav");
  }
  if (file[7])
    rtkstrcpy(ofile[7], ofile_size, file[7]);
  else if (opt->rnxver <= 299 && *opt->staid) {
    rtkstrcpy(ofile[7], ofile_size, "%r%n0.%yI");
  } else if (opt->rnxver <= 299 && def) {
    rtkstrcpy(ofile[7], ofile_size, ifile_);
    char *p = strrchr(ofile[7], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[7]), ".inav");
    else
      rtkstrcat(ofile[7], ofile_size, ".inav");
  }
  char *extlog = "sbs";
  if (file[8])
    rtkstrcpy(ofile[8], ofile_size, file[8]);
  else if (*opt->staid) {
    rtkstrcpy(ofile[8], ofile_size, "%r%n0_%y.");
    rtkstrcat(ofile[8], ofile_size, extlog);
  } else if (def) {
    rtkstrcpy(ofile[8], ofile_size, ifile_);
    char *p = strrchr(ofile[8], '.');
    if (p)
      rtkstrcpy(p, ofile_size - (p - ofile[8]), ".");
    else
      rtkstrcat(ofile[8], ofile_size, ".");
    rtkstrcat(ofile[8], ofile_size, extlog);
  }
  for (int i = 0; i < NOUTFILE; i++) {
    if (!*dir || !*ofile[i]) continue;
    char work[FNSIZE];
    char *p = strrchr(ofile[i], RTKLIB_FILEPATHSEP);
    if (p)
      rtkstrcpy(work, sizeof(work), p + 1);
    else
      rtkstrcpy(work, sizeof(work), ofile[i]);
    rtksnprintf(ofile[i], sizeof(ofile_[0]), "%s%c%s", dir, RTKLIB_FILEPATHSEP, work);
  }
  fprintf(stderr, "input file  : %s (%s)\n", ifile, formatstrs[format]);

  if (*ofile[0]) fprintf(stderr, "->rinex obs : %s\n", ofile[0]);
  if (*ofile[1]) fprintf(stderr, "->rinex nav : %s\n", ofile[1]);
  if (*ofile[2]) fprintf(stderr, "->rinex gnav: %s\n", ofile[2]);
  if (*ofile[3]) fprintf(stderr, "->rinex hnav: %s\n", ofile[3]);
  if (*ofile[4]) fprintf(stderr, "->rinex qnav: %s\n", ofile[4]);
  if (*ofile[5]) fprintf(stderr, "->rinex lnav: %s\n", ofile[5]);
  if (*ofile[6]) fprintf(stderr, "->rinex cnav: %s\n", ofile[6]);
  if (*ofile[7]) fprintf(stderr, "->rinex inav: %s\n", ofile[7]);
  if (*ofile[8]) fprintf(stderr, "->sbas log  : %s\n", ofile[8]);

  if (!convrnx(format, opt, ifile, ofile)) {
    fprintf(stderr, "\n");
    return false;
  }
  fprintf(stderr, "\n");
  return true;
}
/* Set signal mask -----------------------------------------------------------*/
static void setmask(const char *argv, rnxopt_t *opt, int mask) {
  char buff[1024];
  rtkstrcpy(buff, sizeof(buff), argv);
  char *r;
  for (char *p = strtok_r(buff, ",", &r); p; p = strtok_r(NULL, ",", &r)) {
    if (strlen(p) < 4 || p[1] != 'L') continue;
    int i;
    if (p[0] == 'G')
      i = 0;
    else if (p[0] == 'R')
      i = 1;
    else if (p[0] == 'E')
      i = 2;
    else if (p[0] == 'J')
      i = 3;
    else if (p[0] == 'S')
      i = 4;
    else if (p[0] == 'C')
      i = 5;
    else if (p[0] == 'I')
      i = 6;
    else
      continue;
    int code = obs2code(p + 2);
    if (code) {
      opt->mask[i][code - 1] = mask ? '1' : '0';
    }
  }
}
/* Get start time of input file ----------------------------------------------*/
static bool get_filetime(const char *file, gtime_t *time) {
  char path[FNSIZE], *paths[1];
  paths[0] = path;

  if (!expath(file, paths, sizeof(path), 1)) return false;

  /* Get start time of time-tag file */
  char path_tag[FNSIZE];
  rtksnprintf(path_tag, sizeof(path_tag), "%.1019s.tag", path);
  FILE *fp = fopen(path_tag, "rb");
  if (fp) {
    uint8_t buff[64];
    uint32_t time_time;
    if (fread(buff, 64, 1, fp) == 1 && !strncmp((char *)buff, "TIMETAG", 7) &&
        fread(&time_time, 4, 1, fp) == 1) {
      time->time = time_time;
      time->sec = 0.0L;
      fclose(fp);
      return true;
    }
    fclose(fp);
  }
  /* Get modified time of input file */
  struct stat st;
  struct tm *tm;
  if (!stat(path, &st) && (tm = gmtime(&st.st_mtime))) {
    long double ep[6];
    ep[0] = tm->tm_year + 1900;
    ep[1] = tm->tm_mon + 1;
    ep[2] = tm->tm_mday;
    ep[3] = tm->tm_hour;
    ep[4] = tm->tm_min;
    ep[5] = tm->tm_sec;
    *time = utc2gpst(epoch2time(ep));
    return true;
  }
  return false;
}
/* Parse command line options ------------------------------------------------*/
static int cmdopts(int argc, char **argv, rnxopt_t *opt, char **ifile, char **ofile, char **dir,
                   int *trace) {
  opt->rnxver = 304;
  opt->obstype = OBSTYPE_PR | OBSTYPE_CP;
  opt->navsys = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_SBS | SYS_CMP | SYS_IRN;
  opt->ttol = 0.005L;

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 64; j++) opt->mask[i][j] = '1';

  long double span = 0.0L;
  char *fmt = "";
  int nf = 5, nc = 2, format = -1;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-ts") && i + 2 < argc) {
      long double eps[] = {1980, 1, 1, 0, 0, 0};
      sscanf(argv[++i], "%Lf/%Lf/%Lf", eps, eps + 1, eps + 2);
      sscanf(argv[++i], "%Lf:%Lf:%Lf", eps + 3, eps + 4, eps + 5);
      opt->ts = epoch2time(eps);
    } else if (!strcmp(argv[i], "-te") && i + 2 < argc) {
      long double epe[] = {2037, 12, 31, 0, 0, 0};
      sscanf(argv[++i], "%Lf/%Lf/%Lf", epe, epe + 1, epe + 2);
      sscanf(argv[++i], "%Lf:%Lf:%Lf", epe + 3, epe + 4, epe + 5);
      opt->te = epoch2time(epe);
    } else if (!strcmp(argv[i], "-tr") && i + 2 < argc) {
      long double epr[] = {2010, 1, 1, 0, 0, 0};
      sscanf(argv[++i], "%Lf/%Lf/%Lf", epr, epr + 1, epr + 2);
      sscanf(argv[++i], "%Lf:%Lf:%Lf", epr + 3, epr + 4, epr + 5);
      opt->trtcm = epoch2time(epr);
    } else if (!strcmp(argv[i], "-ti") && i + 1 < argc) {
      opt->tint = strtold(argv[++i], NULL);
    } else if (!strcmp(argv[i], "-tt") && i + 1 < argc) {
      opt->ttol = strtold(argv[++i], NULL);
    } else if (!strcmp(argv[i], "-span") && i + 1 < argc) {
      span = strtold(argv[++i], NULL);
    } else if (!strcmp(argv[i], "-r") && i + 1 < argc) {
      fmt = argv[++i];
    } else if (!strcmp(argv[i], "-ro") && i + 1 < argc) {
      rtkstrcpy(opt->rcvopt, sizeof(opt->rcvopt), argv[++i]);
    } else if (!strcmp(argv[i], "-f") && i + 1 < argc) {
      nf = atoi(argv[++i]);
    } else if (!strcmp(argv[i], "-hc") && i + 1 < argc) {
      if (nc < MAXCOMMENT) rtkstrcpy(opt->comment[nc++], sizeof(opt->comment[0]), argv[++i]);
    } else if (!strcmp(argv[i], "-hm") && i + 1 < argc) {
      rtkstrcpy(opt->marker, sizeof(opt->marker), argv[++i]);
    } else if (!strcmp(argv[i], "-hn") && i + 1 < argc) {
      rtkstrcpy(opt->markerno, sizeof(opt->markerno), argv[++i]);
    } else if (!strcmp(argv[i], "-ht") && i + 1 < argc) {
      rtkstrcpy(opt->markertype, sizeof(opt->markertype), argv[++i]);
    } else if (!strcmp(argv[i], "-ho") && i + 1 < argc) {
      char buff[256];
      rtkstrcpy(buff, sizeof(buff), argv[++i]);
      char *r, *p = strtok_r(buff, "/", &r);
      for (int j = 0; j < 2 && p; j++, p = strtok_r(NULL, "/", &r)) {
        rtkstrcpy(opt->name[j], sizeof(opt->name[0]), p);
      }
    } else if (!strcmp(argv[i], "-hr") && i + 1 < argc) {
      char buff[256];
      rtkstrcpy(buff, sizeof(buff), argv[++i]);
      char *r, *p = strtok_r(buff, "/", &r);
      for (int j = 0; j < 3 && p; j++, p = strtok_r(NULL, "/", &r)) {
        rtkstrcpy(opt->rec[j], sizeof(opt->rec[0]), p);
      }
    } else if (!strcmp(argv[i], "-ha") && i + 1 < argc) {
      char buff[256];
      rtkstrcpy(buff, sizeof(buff), argv[++i]);
      char *r, *p = strtok_r(buff, "/", &r);
      for (int j = 0; j < 3 && p; j++, p = strtok_r(NULL, "/", &r)) {
        rtkstrcpy(opt->ant[j], sizeof(opt->ant[0]), p);
      }
    } else if (!strcmp(argv[i], "-hp") && i + 1 < argc) {
      char buff[256];
      rtkstrcpy(buff, sizeof(buff), argv[++i]);
      char *r, *p = strtok_r(buff, "/", &r);
      for (int j = 0; j < 3 && p; j++, p = strtok_r(NULL, "/", &r)) {
        opt->apppos[j] = strtold(p, NULL);
      }
    } else if (!strcmp(argv[i], "-hd") && i + 1 < argc) {
      char buff[256];
      rtkstrcpy(buff, sizeof(buff), argv[++i]);
      char *r, *p = strtok_r(buff, "/", &r);
      for (int j = 0; j < 3 && p; j++, p = strtok_r(NULL, "/", &r)) {
        opt->antdel[j] = strtold(p, NULL);
      }
    } else if (!strcmp(argv[i], "-v") && i + 1 < argc) {
      opt->rnxver = (int)(strtold(argv[++i], NULL) * 100.0L);
    } else if (!strcmp(argv[i], "-od")) {
      opt->obstype |= OBSTYPE_DOP;
    } else if (!strcmp(argv[i], "-os")) {
      opt->obstype |= OBSTYPE_SNR;
    } else if (!strcmp(argv[i], "-oi")) {
      opt->outiono = 1;
    } else if (!strcmp(argv[i], "-ot")) {
      opt->outtime = 1;
    } else if (!strcmp(argv[i], "-ol")) {
      opt->outleaps = 1;
    } else if (!strcmp(argv[i], "-scan")) {
      /* Obsolete */;
    } else if (!strcmp(argv[i], "-halfc")) {
      opt->halfcyc = 1;
    } else if (!strcmp(argv[i], "-mask") && i + 1 < argc) {
      for (int j = 0; j < 6; j++)
        for (int k = 0; k < 64; k++) opt->mask[j][k] = '0';
      setmask(argv[++i], opt, 1);
    } else if (!strcmp(argv[i], "-nomask") && i + 1 < argc) {
      setmask(argv[++i], opt, 0);
    } else if (!strcmp(argv[i], "-x") && i + 1 < argc) {
      int sat = satid2no(argv[++i]);
      if (sat) opt->exsats[sat - 1] = 1;
    } else if (!strcmp(argv[i], "-y") && i + 1 < argc) {
      char *sys = argv[++i];
      if (!strcmp(sys, "G"))
        opt->navsys &= ~SYS_GPS;
      else if (!strcmp(sys, "R"))
        opt->navsys &= ~SYS_GLO;
      else if (!strcmp(sys, "E"))
        opt->navsys &= ~SYS_GAL;
      else if (!strcmp(sys, "J"))
        opt->navsys &= ~SYS_QZS;
      else if (!strcmp(sys, "S"))
        opt->navsys &= ~SYS_SBS;
      else if (!strcmp(sys, "C"))
        opt->navsys &= ~SYS_CMP;
      else if (!strcmp(sys, "I"))
        opt->navsys &= ~SYS_IRN;
    } else if (!strcmp(argv[i], "-d") && i + 1 < argc) {
      *dir = argv[++i];
    } else if (!strcmp(argv[i], "-c") && i + 1 < argc) {
      rtkstrcpy(opt->staid, sizeof(opt->staid), argv[++i]);
    } else if (!strcmp(argv[i], "-o") && i + 1 < argc)
      ofile[0] = argv[++i];
    else if (!strcmp(argv[i], "-n") && i + 1 < argc)
      ofile[1] = argv[++i];
    else if (!strcmp(argv[i], "-g") && i + 1 < argc)
      ofile[2] = argv[++i];
    else if (!strcmp(argv[i], "-h") && i + 1 < argc)
      ofile[3] = argv[++i];
    else if (!strcmp(argv[i], "-q") && i + 1 < argc)
      ofile[4] = argv[++i];
    else if (!strcmp(argv[i], "-l") && i + 1 < argc)
      ofile[5] = argv[++i];
    else if (!strcmp(argv[i], "-b") && i + 1 < argc)
      ofile[6] = argv[++i];
    else if (!strcmp(argv[i], "-i") && i + 1 < argc)
      ofile[7] = argv[++i];
    else if (!strcmp(argv[i], "-s") && i + 1 < argc)
      ofile[8] = argv[++i];
    else if (!strcmp(argv[i], "-trace") && i + 1 < argc) {
      *trace = atoi(argv[++i]);
    } else if (!strncmp(argv[i], "-", 1))
      printhelp();

    else
      *ifile = argv[i];
  }
  if (span > 0.0L && opt->ts.time) {
    opt->te = timeadd(opt->ts, span * 3600.0L - 1e-3L);
  }
  if (nf >= 1) opt->freqtype |= FREQTYPE_L1;
  if (nf >= 2) opt->freqtype |= FREQTYPE_L2;
  if (nf >= 3) opt->freqtype |= FREQTYPE_L3;
  if (nf >= 4) opt->freqtype |= FREQTYPE_L4;
  if (nf >= 5) opt->freqtype |= FREQTYPE_L5;

  if (!opt->trtcm.time) {
    get_filetime(*ifile, &opt->trtcm);
  }
  if (*fmt) {
    if (!strcmp(fmt, "rtcm2"))
      format = STRFMT_RTCM2;
    else if (!strcmp(fmt, "rtcm3"))
      format = STRFMT_RTCM3;
    else if (!strcmp(fmt, "nov"))
      format = STRFMT_OEM4;
    else if (!strcmp(fmt, "cnav"))
      format = STRFMT_CNAV;
    else if (!strcmp(fmt, "ubx"))
      format = STRFMT_UBX;
    else if (!strcmp(fmt, "sbp"))
      format = STRFMT_SBP;
    else if (!strcmp(fmt, "hemis"))
      format = STRFMT_CRES;
    else if (!strcmp(fmt, "stq"))
      format = STRFMT_STQ;
    else if (!strcmp(fmt, "javad"))
      format = STRFMT_JAVAD;
    else if (!strcmp(fmt, "nvs"))
      format = STRFMT_NVS;
    else if (!strcmp(fmt, "binex"))
      format = STRFMT_BINEX;
    else if (!strcmp(fmt, "rt17"))
      format = STRFMT_RT17;
    else if (!strcmp(fmt, "sbf"))
      format = STRFMT_SEPT;
    else if (!strcmp(fmt, "tersus"))
      format = STRFMT_TERSUS;
    else if (!strcmp(fmt, "rinex"))
      format = STRFMT_RINEX;
  } else {
    char *paths[1], path[FNSIZE];
    paths[0] = path;
    char *p;
    if (!expath(*ifile, paths, sizeof(path), 1) || !(p = strrchr(path, '.'))) return -1;
    if (!strcmp(p, ".rtcm2"))
      format = STRFMT_RTCM2;
    else if (!strcmp(p, ".rtcm3"))
      format = STRFMT_RTCM3;
    else if (!strcmp(p, ".gps"))
      format = STRFMT_OEM4;
    else if (!strcmp(p, ".ubx"))
      format = STRFMT_UBX;
    else if (!strcmp(p, ".sbp"))
      format = STRFMT_SBP;
    else if (!strcmp(p, ".bin"))
      format = STRFMT_CRES;
    else if (!strcmp(p, ".stq"))
      format = STRFMT_STQ;
    else if (!strcmp(p, ".jps"))
      format = STRFMT_JAVAD;
    else if (!strcmp(p, ".bnx"))
      format = STRFMT_BINEX;
    else if (!strcmp(p, ".binex"))
      format = STRFMT_BINEX;
    else if (!strcmp(p, ".rt17"))
      format = STRFMT_RT17;
    else if (!strcmp(p, ".sbf"))
      format = STRFMT_SEPT;
    else if (!strcmp(p, ".trs"))
      format = STRFMT_TERSUS;
    else if (!strcmp(p, ".obs"))
      format = STRFMT_RINEX;
    else if (!strcmp(p + 3, "o"))
      format = STRFMT_RINEX;
    else if (!strcmp(p + 3, "O"))
      format = STRFMT_RINEX;
    else if (!strcmp(p, ".rnx"))
      format = STRFMT_RINEX;
    else if (!strcmp(p, ".nav"))
      format = STRFMT_RINEX;
    else if (!strcmp(p + 3, "n"))
      format = STRFMT_RINEX;
    else if (!strcmp(p + 3, "N"))
      format = STRFMT_RINEX;
  }
  return format;
}
/* Main ----------------------------------------------------------------------*/
int main(int argc, char **argv) {
  /* Parse command line options */
  rnxopt_t opt = {{0}};
  char *ifile = "", *ofile[NOUTFILE] = {0}, *dir = "";
  int trace = 0;
  int format = cmdopts(argc, argv, &opt, &ifile, ofile, &dir, &trace);

  if (!*ifile) {
    fprintf(stderr, "no input file\n");
    return EXIT_FAILURE;
  }
  if (format < 0) {
    fprintf(stderr, "input format can not be recognized\n");
    return EXIT_FAILURE;
  }
  rtksnprintf(opt.prog, sizeof(opt.prog), "%s %s %s", PRGNAME, VER_RTKLIB, PATCH_LEVEL);
  rtksnprintf(opt.comment[0], sizeof(opt.comment[0]), "log: %-55.55s", ifile);
  rtksnprintf(opt.comment[1], sizeof(opt.comment[0]), "format: %s", formatstrs[format]);
  if (*opt.rcvopt) {
    rtkstrcat(opt.comment[1], sizeof(opt.comment[1]), ", option: ");
    rtkstrcat(opt.comment[1], sizeof(opt.comment[1]), opt.rcvopt);
  }
  if (trace > 0) {
    traceopen(TRACEFILE);
    tracelevel(trace);
  }
  bool stat = convbin(format, &opt, ifile, ofile, dir);

  traceclose();

  return stat ? 0 : EXIT_FAILURE;
}
