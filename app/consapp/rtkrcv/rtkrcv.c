/*------------------------------------------------------------------------------
 * rtkrcv.c : rtk-gps/gnss receiver console ap
 *
 *          Copyright (C) 2009-2015 by T.TAKASU, All rights reserved.
 *
 * notes   :
 *     current version does not support win32 without pthread library
 *
 * version : $Revision:$ $Date:$
 * history : 2009/12/13 1.0  new
 *           2010/07/18 1.1  add option -m
 *           2010/08/12 1.2  fix bug on ftp/http
 *           2011/01/22 1.3  add option misc-proxyaddr,misc-fswapmargin
 *           2011/08/19 1.4  fix bug on size of arg solopt arg for rtksvrstart()
 *           2012/11/03 1.5  fix bug on setting output format
 *           2013/06/30 1.6  add "nvs" option for inpstr*-format
 *           2014/02/10 1.7  fix bug on printing obs data
 *                           add print of status, glonass nav data
 *                           ignore SIGHUP
 *           2014/04/27 1.8  add "binex" option for inpstr*-format
 *           2014/08/10 1.9  fix cpu overload with abnormal telnet shutdown
 *           2014/08/26 1.10 support input format "rt17"
 *                           change file paths of solution status and debug trace
 *           2015/01/10 1.11 add line editing and command history
 *                           separate codes for virtual console to vt.c
 *           2015/05/22 1.12 fix bug on sp3 id in inpstr*-format options
 *           2015/07/31 1.13 accept 4:stat for outstr1-format or outstr2-format
 *                           add reading satellite dcb
 *           2015/12/14 1.14 add option -sta for station name (#339)
 *           2015/12/25 1.15 fix bug on -sta option (#339)
 *           2015/01/26 1.16 support septentrio
 *           2016/07/01 1.17 support CMR/CMR+
 *           2016/08/20 1.18 add output of patch level with version
 *           2016/09/05 1.19 support ntrip caster for output stream
 *           2016/09/19 1.20 support multiple remote console connections
 *                           add option -w
 *           2017/09/01 1.21 add command ssr
 *-----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "rtklib.h"
#include "vt.h"

#define PRGNAME "rtkrcv"                    /* program name */
#define CMDPROMPT "rtkrcv> "                /* command prompt */
#define MAXCON 32                           /* max number of consoles */
#define MAXARG 10                           /* max number of args in a command */
#define MAXCMD 256                          /* max length of a command */
#define MAXSTR 1024                         /* max length of a stream */
#define OPTSDIR "."                         /* default config directory */
#define OPTSFILE "rtkrcv.conf"              /* default config file */
#define NAVIFILE "rtkrcv.nav"               /* navigation save file */
#define STATFILE "rtkrcv_%Y%m%d%h%M.stat"   /* solution status file */
#define TRACEFILE "rtkrcv_%Y%m%d%h%M.trace" /* debug trace file */
#define INTKEEPALIVE 1000                   /* keep alive interval (ms) */

#define ESC_CLEAR "\033[H\033[2J" /* ansi/vt100 escape: erase screen */
#define ESC_RESET "\033[0m"       /* ansi/vt100: reset attribute */
#define ESC_BOLD "\033[1m"        /* ansi/vt100: bold */

#define SQRT(x) ((x) <= 0.0L || (x) != (x) ? 0.0L : sqrtl(x))

/* type definitions ----------------------------------------------------------*/

typedef struct {    /* console type */
  int state;        /* state (0:stop,1:run) */
  vt_t *vt;         /* virtual terminal */
  pthread_t thread; /* console thread */
} con_t;

/* function prototypes -------------------------------------------------------*/
extern FILE *popen(const char *, const char *);
extern int pclose(FILE *);

/* global variables ----------------------------------------------------------*/
static rtksvr_t svr;  /* rtk server struct */
static stream_t moni; /* monitor stream */

static int intflg = 0; /* interrupt flag (2:shutdown) */

static char passwd[MAXSTR] = "admin"; /* login password */
static int timetype = 0;              /* time format (0:gpst,1:utc,2:jst,3:tow) */
static int soltype = 0;               /* sol format (0:dms,1:deg,2:xyz,3:enu,4:pyl) */
static int solflag = 2;               /* sol flag (1:std+2:age/ratio/ns) */
static int strtype[] = {              /* stream types */
                        STR_SERIAL, STR_NONE, STR_NONE, STR_NONE,
                        STR_NONE,   STR_NONE, STR_NONE, STR_NONE};
static char strpath[8][MAXSTR] = {"", "", "", "", "", "", "", ""}; /* stream paths */
static int strfmt[] = {                                            /* stream formats */
                       STRFMT_UBX, STRFMT_RTCM3, STRFMT_SP3, SOLF_LLH, SOLF_NMEA};
static int svrcycle = 10;                 /* server cycle (ms) */
static int timeout = 10000;               /* timeout time (ms) */
static int reconnect = 10000;             /* reconnect interval (ms) */
static int nmeacycle = 5000;              /* nmea request cycle (ms) */
static int buffsize = 32768;              /* input buffer size (bytes) */
static int navmsgsel = 0;                 /* navigation message select */
static char proxyaddr[256] = "";          /* http/ntrip proxy */
static int nmeareq = 0;                   /* nmea request type (0:off,1:lat/lon,2:single) */
static long double nmeapos[] = {0, 0, 0}; /* nmea position (lat/lon/height) (deg,m) */
static char rcvcmds[3][MAXSTR] = {""};    /* receiver commands files */
static char startcmd[MAXSTR] = "";        /* start command */
static char stopcmd[MAXSTR] = "";         /* stop command */
static int modflgr[256] = {0};            /* modified flags of receiver options */
static int modflgs[256] = {0};            /* modified flags of system options */
static int moniport = 0;                  /* monitor port */
static int keepalive = 0;                 /* keep alive flag */
static int start = 0;                     /* auto start */
static int fswapmargin = 30;              /* file swap margin (s) */
static char sta_name[256] = "";           /* station name */

static prcopt_t prcopt;            /* processing options */
static solopt_t solopt[2] = {{0}}; /* solution options */
static filopt_t filopt = {""};     /* file options */

/* help text -----------------------------------------------------------------*/
static const char *usage[] = {
    "usage: rtkrcv [-s][-p port][-d dev][-o file][-w pwd][-r level][-t level][-sta sta]",
    "options",
    "  -s         start RTK server on program startup",
    "  -nc        start RTK server on program startup with no console",
    "  -p port    port number for telnet console",
    "  -m port    port number for monitor stream",
    "  -d dev     terminal device for console",
    "  -o file    processing options file",
    "  -w pwd     login password for remote console (\"\": no password)",
    "  -r level   output solution status file (0:off,1:states,2:residuals)",
    "  -t level   debug trace level (0:off,1-5:on)",
    "  -sta sta   station name for receiver dcb"};
static const char *helptxt[] = {"start                 : start rtk server",
                                "stop                  : stop rtk server",
                                "restart               : restart rtk sever",
                                "solution [cycle]      : show solution",
                                "status [cycle]        : show rtk status",
                                "satellite [-n] [cycle]: show satellite status",
                                "observ [-n] [cycle]   : show observation data",
                                "navidata [cycle]      : show navigation data",
                                "stream [cycle]        : show stream status",
                                "ssr [cycle]           : show ssr corrections",
                                "error                 : show error/warning messages",
                                "option [opt]          : show option(s)",
                                "set opt [val]         : set option",
                                "load [file]           : load options from file",
                                "save [file]           : save options to file",
                                "log [file|off]        : start/stop log to file",
                                "help|? [path]         : print help",
                                "exit|ctr-D            : logout console (only for telnet)",
                                "shutdown              : shutdown rtk server",
                                "!command [arg...]     : execute command in shell",
                                ""};
static const char *pathopts[] =
    {/* path options help */
     "stream path formats",
     "serial   : port[:bit_rate[:byte[:parity(n|o|e)[:stopb[:fctr(off|on)[#port]]]]]]]",
     "file     : path[::T[::+offset][::xspeed]]",
     "tcpsvr   : :port",
     "tcpcli   : addr:port",
     "ntripsvr : [passwd@]addr:port/mntpnt[:str]",
     "ntripcli : user:passwd@addr:port/mntpnt",
     "ntripcas : user:passwd@:[port]/mpoint[:srctbl]",
     "ftp      : user:passwd@addr/path[::T=poff,tint,off,rint]",
     "http     : addr/path[::T=poff,tint,off,rint]",
     ""};
/* receiver options table ----------------------------------------------------*/
#define TIMOPT "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT "0:off,1:std+2:age/ratio/ns"
#define ISTOPT "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripcli,7:ftp,8:http"
#define OSTOPT "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,5:ntripsvr,9:ntripcas"
#define FMTOPT                                                                                \
  "0:rtcm2,1:rtcm3,2:oem4,4:ubx,5:swift,6:hemis,7:skytraq,8:javad,9:nvs,10:binex,11:rt17,12:" \
  "sbf,14,15:sp3"
#define NMEOPT "0:off,1:latlon,2:single"
#define SOLOPT "0:llh,1:xyz,2:enu,3:nmea,4:stat"
#define MSGOPT "0:all,1:rover,2:base,3:corr"

static opt_t rcvopts[] = {{"console-passwd", 2, (void *)passwd, sizeof(passwd), ""},
                          {"console-timetype", 3, (void *)&timetype, 0, TIMOPT},
                          {"console-soltype", 3, (void *)&soltype, 0, CONOPT},
                          {"console-solflag", 0, (void *)&solflag, 0, FLGOPT},

                          {"inpstr1-type", 3, (void *)&strtype[0], 0, ISTOPT},
                          {"inpstr2-type", 3, (void *)&strtype[1], 0, ISTOPT},
                          {"inpstr3-type", 3, (void *)&strtype[2], 0, ISTOPT},
                          {"inpstr1-path", 2, (void *)strpath[0], sizeof(strpath[0]), ""},
                          {"inpstr2-path", 2, (void *)strpath[1], sizeof(strpath[1]), ""},
                          {"inpstr3-path", 2, (void *)strpath[2], sizeof(strpath[2]), ""},
                          {"inpstr1-format", 3, (void *)&strfmt[0], 0, FMTOPT},
                          {"inpstr2-format", 3, (void *)&strfmt[1], 0, FMTOPT},
                          {"inpstr3-format", 3, (void *)&strfmt[2], 0, FMTOPT},
                          {"inpstr2-nmeareq", 3, (void *)&nmeareq, 0, NMEOPT},
                          {"inpstr2-nmealat", 1, (void *)&nmeapos[0], 0, "deg"},
                          {"inpstr2-nmealon", 1, (void *)&nmeapos[1], 0, "deg"},
                          {"inpstr2-nmeahgt", 1, (void *)&nmeapos[2], 0, "m"},
                          {"outstr1-type", 3, (void *)&strtype[3], 0, OSTOPT},
                          {"outstr2-type", 3, (void *)&strtype[4], 0, OSTOPT},
                          {"outstr1-path", 2, (void *)strpath[3], sizeof(strpath[3]), ""},
                          {"outstr2-path", 2, (void *)strpath[4], sizeof(strpath[4]), ""},
                          {"outstr1-format", 3, (void *)&strfmt[3], 0, SOLOPT},
                          {"outstr2-format", 3, (void *)&strfmt[4], 0, SOLOPT},
                          {"logstr1-type", 3, (void *)&strtype[5], 0, OSTOPT},
                          {"logstr2-type", 3, (void *)&strtype[6], 0, OSTOPT},
                          {"logstr3-type", 3, (void *)&strtype[7], 0, OSTOPT},
                          {"logstr1-path", 2, (void *)strpath[5], sizeof(strpath[5]), ""},
                          {"logstr2-path", 2, (void *)strpath[6], sizeof(strpath[6]), ""},
                          {"logstr3-path", 2, (void *)strpath[7], sizeof(strpath[7]), ""},

                          {"misc-svrcycle", 0, (void *)&svrcycle, 0, "ms"},
                          {"misc-timeout", 0, (void *)&timeout, 0, "ms"},
                          {"misc-reconnect", 0, (void *)&reconnect, 0, "ms"},
                          {"misc-nmeacycle", 0, (void *)&nmeacycle, 0, "ms"},
                          {"misc-buffsize", 0, (void *)&buffsize, 0, "bytes"},
                          {"misc-navmsgsel", 3, (void *)&navmsgsel, 0, MSGOPT},
                          {"misc-proxyaddr", 2, (void *)proxyaddr, sizeof(proxyaddr), ""},
                          {"misc-fswapmargin", 0, (void *)&fswapmargin, 0, "s"},

                          {"misc-startcmd", 2, (void *)startcmd, sizeof(startcmd), ""},
                          {"misc-stopcmd", 2, (void *)stopcmd, sizeof(stopcmd), ""},

                          {"file-cmdfile1", 2, (void *)rcvcmds[0], sizeof(rcvcmds[0]), ""},
                          {"file-cmdfile2", 2, (void *)rcvcmds[1], sizeof(rcvcmds[1]), ""},
                          {"file-cmdfile3", 2, (void *)rcvcmds[2], sizeof(rcvcmds[2]), ""},

                          {"", 0, NULL, 0, ""}};
/* print usage ---------------------------------------------------------------*/
static void printusage(void) {
  for (unsigned i = 0; i < sizeof(usage) / sizeof(*usage); i++) {
    fprintf(stderr, "%s\n", usage[i]);
  }
  exit(0);
}
/* external stop signal ------------------------------------------------------*/
static void sigshut(int sig) {
  trace(3, "sigshut: sig=%d\n", sig);

  intflg = 1;
}
/* discard space characters at tail ------------------------------------------*/
static void chop(char *str) {
  char *p;
  for (p = str + strlen(str) - 1; p >= str && !isgraph((int)*p); p--) *p = '\0';
}
/* thread to send keep alive for monitor port --------------------------------*/
static void *sendkeepalive(void *arg) {
  trace(3, "sendkeepalive: start\n");

  while (keepalive) {
    strwrite(&moni, (uint8_t *)"\r", 1);
    sleepms(INTKEEPALIVE);
  }
  trace(3, "sendkeepalive: stop\n");
  return NULL;
}
/* open monitor port ---------------------------------------------------------*/
static bool openmoni(int port) {
  trace(3, "openmomi: port=%d\n", port);

  char path[64];
  rtksnprintf(path, sizeof(path), ":%d", port);
  if (!stropen(&moni, STR_TCPSVR, STR_MODE_RW, path)) return false;
  strsettimeout(&moni, timeout, reconnect);
  keepalive = 1;
  pthread_t thread;
  pthread_create(&thread, NULL, sendkeepalive, NULL);
  return true;
}
/* close monitor port --------------------------------------------------------*/
static void closemoni(void) {
  trace(3, "closemoni:\n");
  keepalive = 0;

  /* send disconnect message */
  strwrite(&moni, (uint8_t *)MSG_DISCONN, strlen(MSG_DISCONN));

  /* wait fin from clients */
  sleepms(1000);

  strclose(&moni);
}
/* confirm overwrite ---------------------------------------------------------*/
static bool confwrite(vt_t *vt, const char *file) {
  char buff[MAXSTR];
  rtkstrcpy(buff, sizeof(buff), file);
  char *p = strstr(buff, "::");
  if (p) *p = '\0'; /* omit options in path */
  if (!vt->state) return true;
  FILE *fp = fopen(buff, "r");
  if (!fp) return true; /* no existing file */
  fclose(fp);
  vt_printf(vt, "overwrite %-16s ? (y/n): ", buff);
  if (!vt_gets(vt, buff, sizeof(buff)) || vt->brk) return false;
  return toupper((int)buff[0]) == 'Y';
}
/* login ---------------------------------------------------------------------*/
static bool login(vt_t *vt) {
  trace(3, "login: passwd=%s type=%d\n", passwd, vt->type);

  if (!*passwd || !vt->type) return true;

  while (!(intflg & 2)) {
    if (!vt_printf(vt, "password: ", PRGNAME)) return false;
    vt->blind = 1;
    char buff[256];
    if (!vt_gets(vt, buff, sizeof(buff)) || vt->brk) {
      vt->blind = 0;
      return false;
    }
    vt->blind = 0;
    if (!strcmp(buff, passwd)) break;
    vt_printf(vt, "\ninvalid password\n");
  }
  return true;
}
/* read receiver commands ----------------------------------------------------*/
static bool readcmd(const char *file, char *cmd, size_t size, int type) {
  trace(3, "readcmd: file=%s\n", file);

  cmd[0] = '\0';

  FILE *fp = fopen(file, "r");
  if (!fp) return false;

  char buff[MAXSTR];
  int i = 0;
  while (fgets(buff, sizeof(buff), fp)) {
    if (*buff == '@')
      i++;
    else if (i == type && strlen(cmd) + strlen(buff) + 1 < size) {
      rtkcatprintf(cmd, size, "%s", buff);
    }
  }
  fclose(fp);
  return true;
}
/* read antenna file ---------------------------------------------------------*/
static void readant(vt_t *vt, prcopt_t *opt, nav_t *nav) {
  trace(3, "readant:\n");

  const pcv_t pcv0 = {0};
  opt->pcvr[0] = opt->pcvr[1] = pcv0;
  if (!*filopt.rcvantp) return;

  gtime_t time = timeget();
  pcvs_t pcvr = {0};
  if (readpcv(filopt.rcvantp, &pcvr)) {
    for (int i = 0; i < 2; i++) {
      if (!*opt->anttype[i]) continue;
      pcv_t *pcv = searchpcv(0, opt->anttype[i], time, &pcvr);
      if (!pcv) {
        vt_printf(vt, "no antenna %s in %s", opt->anttype[i], filopt.rcvantp);
        continue;
      }
      opt->pcvr[i] = *pcv;
    }
  } else
    vt_printf(vt, "antenna file open error %s", filopt.rcvantp);

  pcvs_t pcvs = {0};
  if (readpcv(filopt.satantp, &pcvs)) {
    for (int i = 0; i < MAXSAT; i++) {
      pcv_t *pcv = searchpcv(i + 1, "", time, &pcvs);
      if (!pcv) continue;
      nav->pcvs[i] = *pcv;
    }
  } else
    vt_printf(vt, "antenna file open error %s", filopt.satantp);

  free(pcvr.pcv);
  free(pcvs.pcv);
}
/* start rtk server ----------------------------------------------------------*/
static bool startsvr(vt_t *vt) {
  trace(3, "startsvr:\n");

  /* read start commands from command files */
  char s1[3][MAXRCVCMD] = {"", "", ""}, *cmds[] = {NULL, NULL, NULL};
  char s2[3][MAXRCVCMD] = {"", "", ""}, *cmds_periodic[] = {NULL, NULL, NULL};
  for (int i = 0; i < 3; i++) {
    if (!*rcvcmds[i]) continue;
    if (!readcmd(rcvcmds[i], s1[i], sizeof(s1[0]), 0)) {
      vt_printf(vt, "no command file: %s\n", rcvcmds[i]);
    } else
      cmds[i] = s1[i];
    if (!readcmd(rcvcmds[i], s2[i], sizeof(s2[0]), 2)) {
      vt_printf(vt, "no command file: %s\n", rcvcmds[i]);
    } else
      cmds_periodic[i] = s2[i];
  }
  /* confirm overwrite */
  if (vt != NULL) {
    for (int i = 3; i < 8; i++) {
      if (strtype[i] == STR_FILE && !confwrite(vt, strpath[i])) return false;
    }
  }
  if (prcopt.refpos == 4) { /* rtcm */
    for (int i = 0; i < 3; i++) prcopt.rb[i] = 0.0L;
  }
  long double pos[3];
  pos[0] = nmeapos[0] * D2R;
  pos[1] = nmeapos[1] * D2R;
  pos[2] = nmeapos[2];
  long double npos[3];
  pos2ecef(pos, npos);

  /* read antenna file */
  readant(vt, &prcopt, &svr.nav);

  /* read dcb file */
  if (*filopt.dcb) {
    static sta_t sta[MAXRCV] = {{""}};
    rtkstrcpy(sta[0].name, sizeof(sta[0].name), sta_name);
    readdcb(filopt.dcb, &svr.nav, sta);
  }
  /* open geoid data file */
  if (solopt[0].geoid > 0 && !opengeoid(solopt[0].geoid, filopt.geoid)) {
    trace(2, "geoid data open error: %s\n", filopt.geoid);
    vt_printf(vt, "geoid data open error: %s\n", filopt.geoid);
  }
  for (int i = 0; *rcvopts[i].name; i++) modflgr[i] = 0;
  for (int i = 0; *sysopts[i].name; i++) modflgs[i] = 0;

  /* set stream options */
  int stropt[8] = {0};
  stropt[0] = timeout;
  stropt[1] = reconnect;
  stropt[2] = 1000;
  stropt[3] = buffsize;
  stropt[4] = fswapmargin;
  strsetopt(stropt);

  if (strfmt[2] == 8) strfmt[2] = STRFMT_SP3;

  /* set ftp/http directory and proxy */
  strsetdir(filopt.tempdir);
  strsetproxy(proxyaddr);

  /* execute start command */
  int ret;
  if (*startcmd && (ret = system(startcmd))) {
    trace(2, "command exec error: %s (%d)\n", startcmd, ret);
    vt_printf(vt, "command exec error: %s (%d)\n", startcmd, ret);
  }
  solopt[0].posf = strfmt[3];
  solopt[1].posf = strfmt[4];

  /* start rtk server */
  char *paths[] = {strpath[0], strpath[1], strpath[2], strpath[3],
                   strpath[4], strpath[5], strpath[6], strpath[7]};
  char *ropts[] = {"", "", ""};
  char errmsg[2048] = "";
  if (!rtksvrstart(&svr, svrcycle, buffsize, strtype, (const char **)paths, strfmt, navmsgsel,
                   (const char **)cmds, (const char **)cmds_periodic, (const char **)ropts,
                   nmeacycle, nmeareq, npos, &prcopt, solopt, &moni, errmsg, sizeof(errmsg))) {
    trace(2, "rtk server start error (%s)\n", errmsg);
    vt_printf(vt, "rtk server start error (%s)\n", errmsg);
    return false;
  }
  return true;
}
/* stop rtk server -----------------------------------------------------------*/
static void stopsvr(vt_t *vt) {
  trace(3, "stopsvr:\n");

  if (!svr.state) return;

  /* read stop commands from command files */
  char s[3][MAXRCVCMD] = {"", "", ""}, *cmds[] = {NULL, NULL, NULL};
  for (int i = 0; i < 3; i++) {
    if (!*rcvcmds[i]) continue;
    if (!readcmd(rcvcmds[i], s[i], sizeof(s[0]), 1)) {
      vt_printf(vt, "no command file: %s\n", rcvcmds[i]);
    } else
      cmds[i] = s[i];
  }
  /* stop rtk server */
  rtksvrstop(&svr, (const char **)cmds);

  /* execute stop command */
  int ret;
  if (*stopcmd && (ret = system(stopcmd))) {
    trace(2, "command exec error: %s (%d)\n", stopcmd, ret);
    vt_printf(vt, "command exec error: %s (%d)\n", stopcmd, ret);
  }
  if (solopt[0].geoid > 0) closegeoid();

  vt_printf(vt, "stop rtk server\n");
}
/* print time ----------------------------------------------------------------*/
static void prtime(vt_t *vt, gtime_t time) {
  char tstr[40] = "";
  if (timetype == 1) {
    time2str(gpst2utc(time), tstr, 2);
  } else if (timetype == 2) {
    time2str(timeadd(gpst2utc(time), 9 * 3600.0L), tstr, 2);
  } else if (timetype == 3) {
    int week;
    long double tow = time2gpst(time, &week);
    rtksnprintf(tstr, sizeof(tstr), "  %04d %9.2Lf", week, tow);
  } else
    time2str(time, tstr, 1);
  vt_printf(vt, "%s ", tstr);
}
/* print solution ------------------------------------------------------------*/
static void prsolution(vt_t *vt, const sol_t *sol, const long double *rb) {
  trace(4, "prsolution:\n");

  if (sol->time.time == 0 || !sol->stat) return;
  prtime(vt, sol->time);
  const char *solstr[] = {"------", "FIX", "FLOAT", "SBAS", "DGPS", "SINGLE", "PPP", ""};
  vt_printf(vt, "(%-6s)", solstr[sol->stat]);

  long double bl[3] = {0};
  if (norm(sol->rr, 3) > 0.0L && norm(rb, 3) > 0.0L) {
    for (int i = 0; i < 3; i++) bl[i] = sol->rr[i] - rb[i];
  }
  long double len = norm(bl, 3);
  long double Qr[9];
  Qr[0] = sol->qr[0];
  Qr[4] = sol->qr[1];
  Qr[8] = sol->qr[2];
  Qr[1] = Qr[3] = sol->qr[3];
  Qr[5] = Qr[7] = sol->qr[4];
  Qr[2] = Qr[6] = sol->qr[5];

  if (soltype == 0) {
    long double pos[3] = {0};
    long double Qe[9] = {0};
    long double dms1[3] = {0}, dms2[3] = {0};
    if (norm(sol->rr, 3) > 0.0L) {
      ecef2pos(sol->rr, pos);
      covenu(pos, Qr, Qe);
      deg2dms(pos[0] * R2D, dms1, 4);
      deg2dms(pos[1] * R2D, dms2, 4);
      if (solopt[0].height == 1) pos[2] -= geoidh(pos); /* geodetic */
    }
    vt_printf(vt, " %s:%2.0Lf %02.0Lf %07.4Lf", pos[0] < 0 ? "S" : "N", fabsl(dms1[0]), dms1[1],
              dms1[2]);
    vt_printf(vt, " %s:%3.0Lf %02.0Lf %07.4Lf", pos[1] < 0 ? "W" : "E", fabsl(dms2[0]), dms2[1],
              dms2[2]);
    vt_printf(vt, " H:%8.3Lf", pos[2]);
    if (solflag & 1) {
      vt_printf(vt, " (N:%6.3Lf E:%6.3Lf U:%6.3Lf)", SQRT(Qe[4]), SQRT(Qe[0]), SQRT(Qe[8]));
    }
  } else if (soltype == 1) {
    long double pos[3] = {0};
    long double Qe[9] = {0};
    if (norm(sol->rr, 3) > 0.0L) {
      ecef2pos(sol->rr, pos);
      covenu(pos, Qr, Qe);
      if (solopt[0].height == 1) pos[2] -= geoidh(pos); /* geodetic */
    }
    vt_printf(vt, " %s:%11.8Lf", pos[0] < 0.0L ? "S" : "N", fabsl(pos[0]) * R2D);
    vt_printf(vt, " %s:%12.8Lf", pos[1] < 0.0L ? "W" : "E", fabsl(pos[1]) * R2D);
    vt_printf(vt, " H:%8.3Lf", pos[2]);
    if (solflag & 1) {
      vt_printf(vt, " (E:%6.3Lf N:%6.3Lf U:%6.3Lfm)", SQRT(Qe[0]), SQRT(Qe[4]), SQRT(Qe[8]));
    }
  } else if (soltype == 2) {
    vt_printf(vt, " X:%12.3Lf", sol->rr[0]);
    vt_printf(vt, " Y:%12.3Lf", sol->rr[1]);
    vt_printf(vt, " Z:%12.3Lf", sol->rr[2]);
    if (solflag & 1) {
      vt_printf(vt, " (X:%6.3Lf Y:%6.3Lf Z:%6.3Lf)", SQRT(Qr[0]), SQRT(Qr[4]), SQRT(Qr[8]));
    }
  } else if (soltype == 3) {
    long double enu[3] = {0};
    long double Qe[9] = {0};
    if (len > 0.0L) {
      long double pos[3] = {0};
      ecef2pos(rb, pos);
      ecef2enu(pos, bl, enu);
      covenu(pos, Qr, Qe);
    }
    vt_printf(vt, " E:%12.3Lf", enu[0]);
    vt_printf(vt, " N:%12.3Lf", enu[1]);
    vt_printf(vt, " U:%12.3Lf", enu[2]);
    if (solflag & 1) {
      vt_printf(vt, " (E:%6.3Lf N:%6.3Lf U:%6.3Lf)", SQRT(Qe[0]), SQRT(Qe[4]), SQRT(Qe[8]));
    }
  } else if (soltype == 4) {
    long double pitch = 0.0L, yaw = 0.0L;
    long double Qe[9] = {0};
    if (len > 0.0L) {
      long double pos[3] = {0};
      ecef2pos(rb, pos);
      long double enu[3] = {0};
      ecef2enu(pos, bl, enu);
      covenu(pos, Qr, Qe);
      pitch = asinl(enu[2] / len);
      yaw = atan2l(enu[0], enu[1]);
      if (yaw < 0.0L) yaw += 2.0L * PI;
    }
    vt_printf(vt, " P:%12.3Lf", pitch * R2D);
    vt_printf(vt, " Y:%12.3Lf", yaw * R2D);
    vt_printf(vt, " L:%12.3Lf", len);
    if (solflag & 1) {
      vt_printf(vt, " (E:%6.3Lf N:%6.3Lf U:%6.3Lf)", SQRT(Qe[0]), SQRT(Qe[4]), SQRT(Qe[8]));
    }
  }
  if (solflag & 2) {
    vt_printf(vt, " A:%4.1Lf R:%5.1Lf N:%2d", sol->age, sol->ratio, sol->ns);
  }
  vt_printf(vt, "\n");
}
/* print status --------------------------------------------------------------*/
static void prstatus(vt_t *vt) {
  const char *svrstate[] = {"stop", "run"}, *type[] = {"rover", "base", "corr"};
  const char *sol[] = {"-", "fix", "float", "SBAS", "DGPS", "single", "PPP", ""};
  const char *mode[] = {"single",      "DGPS",  "kinematic",  "static",    "static-start",
                        "moving-base", "fixed", "PPP-kinema", "PPP-static"};
  const char *freq[] = {"-", "L1", "L1+L2", "L1+L2+E5b", "L1+L2+E5b+L5", "", ""};

  trace(4, "prstatus:\n");

  rtksvrlock(&svr);
  rtk_t rtk = svr.rtk;
  pthread_t thread = svr.thread;
  int cycle = svr.cycle;
  int state = svr.state;
  int rtkstat = svr.rtk.sol.stat;
  int nsat0 = svr.obs[0][0].n;
  int nsat1 = svr.obs[1][0].n;
  int rcvcount = svr.raw[0].obs.rcvcount;
  int tmcount = svr.raw[0].obs.tmcount;
  int cputime = svr.cputime;
  int prcout = svr.prcout;
  int nave = svr.nave;
  int nb[3] = {0};
  for (int i = 0; i < 3; i++) nb[i] = svr.nb[i];
  int nmsg[3][10] = {{0}};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 10; j++) {
      nmsg[i][j] = svr.nmsg[i][j];
    }
  long double rt[3] = {0};
  if (svr.state) {
    long double runtime = (tickget() - svr.tick) / 1000.0L;
    rt[0] = floorl(runtime / 3600.0L);
    runtime -= rt[0] * 3600.0L;
    rt[1] = floorl(runtime / 60.0L);
    rt[2] = runtime - rt[1] * 60.0L;
  }
  rtcm_t rtcm[3];
  for (int i = 0; i < 3; i++) rtcm[i] = svr.rtcm[i];
  int timevalid; /* TODO unused? */
  gtime_t eventime = {0};
  if (svr.raw[0].obs.data != NULL) {
    timevalid = svr.raw[0].obs.data[0].timevalid;
    eventime = svr.raw[0].obs.data[0].eventime;
  }
  char tmstr[40];
  time2str(eventime, tmstr, 9);
  rtksvrunlock(&svr);

  int n = 0;
  long double azel[MAXSAT * 2];
  for (int i = 0; i < MAXSAT; i++) {
    if (rtk.opt.mode == PMODE_SINGLE && !rtk.ssat[i].vs) continue;
    if (rtk.opt.mode != PMODE_SINGLE && !rtk.ssat[i].vsat[0]) continue;
    azel[n * 2] = rtk.ssat[i].azel[0];
    azel[1 + n * 2] = rtk.ssat[i].azel[1];
    n++;
  }
  long double dop[4] = {0};
  dops(n, azel, 0.0L, dop);

  vt_printf(vt, "\n%s%-28s: %s%s\n", ESC_BOLD, "Parameter", "Value", ESC_RESET);
  vt_printf(vt, "%-28s: %s %s\n", "rtklib version", VER_RTKLIB, PATCH_LEVEL);
  vt_printf(vt, "%-28s: %d\n", "rtk server thread", thread);
  vt_printf(vt, "%-28s: %s\n", "rtk server state", svrstate[state]);
  vt_printf(vt, "%-28s: %d\n", "processing cycle (ms)", cycle);
  vt_printf(vt, "%-28s: %s\n", "positioning mode", mode[rtk.opt.mode]);
  vt_printf(vt, "%-28s: %s\n", "frequencies", freq[rtk.opt.nf]);
  vt_printf(vt, "%-28s: %02.0Lf:%02.0Lf:%04.1Lf\n", "accumulated time to run", rt[0], rt[1], rt[2]);
  vt_printf(vt, "%-28s: %d\n", "cpu time for a cycle (ms)", cputime);
  vt_printf(vt, "%-28s: %d\n", "missing obs data count", prcout);
  vt_printf(vt, "%-28s: %d,%d\n", "bytes in input buffer", nb[0], nb[1]);
  char s[1024];
  for (int i = 0; i < 3; i++) {
    rtksnprintf(s, sizeof(s), "# of input data %s", type[i]);
    vt_printf(vt,
              "%-28s: obs(%d),nav(%d),gnav(%d),ion(%d),sbs(%d),pos(%d),dgps(%d),ssr(%d),err(%d)\n",
              s, nmsg[i][0], nmsg[i][1], nmsg[i][6], nmsg[i][2], nmsg[i][3], nmsg[i][4], nmsg[i][5],
              nmsg[i][7], nmsg[i][9]);
  }
  for (int i = 0; i < 3; i++) {
    s[0] = '\0';
    for (int j = 1; j < 100; j++) {
      if (rtcm[i].nmsg2[j] == 0) continue;
      rtkcatprintf(s, sizeof(s), "%s%d(%d)", strlen(s) > 0 ? "," : "", j, rtcm[i].nmsg2[j]);
    }
    if (rtcm[i].nmsg2[0] > 0) {
      rtkcatprintf(s, sizeof(s), "%sother2(%d)", strlen(s) > 0 ? "," : "", rtcm[i].nmsg2[0]);
    }
    for (int j = 1; j < 300; j++) {
      if (rtcm[i].nmsg3[j] == 0) continue;
      rtkcatprintf(s, sizeof(s), "%s%d(%d)", strlen(s) > 0 ? "," : "", j + 1000, rtcm[i].nmsg3[j]);
    }
    if (rtcm[i].nmsg3[0] > 0) {
      rtkcatprintf(s, sizeof(s), "%sother3(%d)", strlen(s) > 0 ? "," : "", rtcm[i].nmsg3[0]);
    }
    vt_printf(vt, "%-15s %-9s: %s\n", "# of rtcm messages", type[i], s);
  }
  vt_printf(vt, "%-28s: %s\n", "solution status", sol[rtkstat]);
  char tstr[40];
  time2str(rtk.sol.time, tstr, 9);
  vt_printf(vt, "%-28s: %s\n", "time of receiver clock rover", rtk.sol.time.time ? tstr : "-");
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf,%.3Lf\n", "time sys offset (ns)", rtk.sol.dtr[1] * 1e9L,
            rtk.sol.dtr[2] * 1e9L, rtk.sol.dtr[3] * 1e9L, rtk.sol.dtr[4] * 1e9L);
  vt_printf(vt, "%-28s: %.3Lf\n", "solution interval (s)", rtk.tt);
  vt_printf(vt, "%-28s: %.3Lf\n", "age of differential (s)", rtk.sol.age);
  vt_printf(vt, "%-28s: %.3Lf\n", "ratio for ar validation", rtk.sol.ratio);
  vt_printf(vt, "%-28s: %d\n", "# of satellites rover", nsat0);
  vt_printf(vt, "%-28s: %d\n", "# of satellites base", nsat1);
  vt_printf(vt, "%-28s: %d\n", "# of valid satellites", rtk.sol.ns);
  vt_printf(vt, "%-28s: %.1Lf,%.1Lf,%.1Lf,%.1Lf\n", "GDOP/PDOP/HDOP/VDOP", dop[0], dop[1], dop[2],
            dop[3]);
  vt_printf(vt, "%-28s: %d\n", "# of real estimated states", rtk.na);
  vt_printf(vt, "%-28s: %d\n", "# of all estimated states", rtk.nx);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "pos xyz single (m) rover", rtk.sol.rr[0], rtk.sol.rr[1],
            rtk.sol.rr[2]);
  long double pos[3];
  if (norm(rtk.sol.rr, 3) > 0.0L)
    ecef2pos(rtk.sol.rr, pos);
  else
    pos[0] = pos[1] = pos[2] = 0.0L;
  vt_printf(vt, "%-28s: %.8Lf,%.8Lf,%.3Lf\n", "pos llh single (deg,m) rover", pos[0] * R2D,
            pos[1] * R2D, pos[2]);
  long double vel[3];
  ecef2enu(pos, rtk.sol.rr + 3, vel);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "vel enu (m/s) rover", vel[0], vel[1], vel[2]);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "pos xyz float (m) rover", rtk.x ? rtk.x[0] : 0,
            rtk.x ? rtk.x[1] : 0, rtk.x ? rtk.x[2] : 0);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "pos xyz float std (m) rover",
            rtk.P ? SQRT(rtk.P[0]) : 0, rtk.P ? SQRT(rtk.P[1 + 1 * rtk.nx]) : 0,
            rtk.P ? SQRT(rtk.P[2 + 2 * rtk.nx]) : 0);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "pos xyz fixed (m) rover", rtk.xa ? rtk.xa[0] : 0,
            rtk.xa ? rtk.xa[1] : 0, rtk.xa ? rtk.xa[2] : 0);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "pos xyz fixed std (m) rover",
            rtk.Pa ? SQRT(rtk.Pa[0]) : 0, rtk.Pa ? SQRT(rtk.Pa[1 + 1 * rtk.na]) : 0,
            rtk.Pa ? SQRT(rtk.Pa[2 + 2 * rtk.na]) : 0);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "pos xyz (m) base", rtk.rb[0], rtk.rb[1], rtk.rb[2]);
  if (norm(rtk.rb, 3) > 0.0L)
    ecef2pos(rtk.rb, pos);
  else
    pos[0] = pos[1] = pos[2] = 0.0L;
  vt_printf(vt, "%-28s: %.8Lf,%.8Lf,%.3Lf\n", "pos llh (deg,m) base", pos[0] * R2D, pos[1] * R2D,
            pos[2]);
  vt_printf(vt, "%-28s: %d\n", "# of average single pos base", nave);
  vt_printf(vt, "%-28s: %s\n", "ant type rover", rtk.opt.pcvr[0].type);
  long double *del = rtk.opt.antdel[0];
  vt_printf(vt, "%-28s: %.3Lf %.3Lf %.3Lf\n", "ant delta rover", del[0], del[1], del[2]);
  vt_printf(vt, "%-28s: %s\n", "ant type base", rtk.opt.pcvr[1].type);
  del = rtk.opt.antdel[1];
  vt_printf(vt, "%-28s: %.3Lf %.3Lf %.3Lf\n", "ant delta base", del[0], del[1], del[2]);
  ecef2enu(pos, rtk.rb + 3, vel);
  vt_printf(vt, "%-28s: %.3Lf,%.3Lf,%.3Lf\n", "vel enu (m/s) base", vel[0], vel[1], vel[2]);
  long double bl1 = 0.0L;
  if (rtk.opt.mode > 0 && rtk.x && norm(rtk.x, 3) > 0.0L) {
    long double rr[3];
    for (int i = 0; i < 3; i++) rr[i] = rtk.x[i] - rtk.rb[i];
    bl1 = norm(rr, 3);
  }
  long double bl2 = 0.0L;
  if (rtk.opt.mode > 0 && rtk.xa && norm(rtk.xa, 3) > 0.0L) {
    long double rr[3];
    for (int i = 0; i < 3; i++) rr[i] = rtk.xa[i] - rtk.rb[i];
    bl2 = norm(rr, 3);
  }
  vt_printf(vt, "%-28s: %.3Lf\n", "baseline length float (m)", bl1);
  vt_printf(vt, "%-28s: %.3Lf\n", "baseline length fixed (m)", bl2);
  vt_printf(vt, "%-28s: %s\n", "last time mark", tmcount ? tmstr : "-");
  vt_printf(vt, "%-28s: %d\n", "receiver time mark count", rcvcount);
  vt_printf(vt, "%-28s: %d\n", "rtklib time mark count", tmcount);
}
/* print satellite -----------------------------------------------------------*/
static void prsatellite(vt_t *vt, int nf) {
  trace(4, "prsatellite:\n");

  rtksvrlock(&svr);
  rtk_t rtk = svr.rtk;
  rtksvrunlock(&svr);
  if (nf <= 0 || nf > NFREQ) nf = NFREQ;
  vt_printf(vt, "\n%s%3s %2s %5s %4s", ESC_BOLD, "SAT", "C1", "Az", "El");
  int frq[] = {1, 2, 5, 7, 8, 6};
  for (int j = 0; j < nf; j++) vt_printf(vt, " L%d", frq[j]);
  for (int j = 0; j < nf; j++) vt_printf(vt, "  Fix%d", frq[j]);
  for (int j = 0; j < nf; j++) vt_printf(vt, "  P%dRes", frq[j]);
  for (int j = 0; j < nf; j++) vt_printf(vt, "   L%dRes", frq[j]);
  for (int j = 0; j < nf; j++) vt_printf(vt, "  Sl%d", frq[j]);
  for (int j = 0; j < nf; j++) vt_printf(vt, "  Lock%d", frq[j]);
  for (int j = 0; j < nf; j++) vt_printf(vt, " Rj%d", frq[j]);
  vt_printf(vt, "%s\n", ESC_RESET);

  for (int i = 0; i < MAXSAT; i++) {
    if (rtk.ssat[i].azel[1] <= 0.0L) continue;
    char id[8];
    satno2id(i + 1, id);
    vt_printf(vt, "%3s %2s", id, rtk.ssat[i].vs ? "OK" : "-");
    long double az = rtk.ssat[i].azel[0] * R2D;
    if (az < 0.0L) az += 360.0L;
    long double el = rtk.ssat[i].azel[1] * R2D;
    vt_printf(vt, " %5.1Lf %4.1Lf", az, el);
    for (int j = 0; j < nf; j++) vt_printf(vt, " %2s", rtk.ssat[i].vsat[j] ? "OK" : "-");
    for (int j = 0; j < nf; j++) {
      int fix = rtk.ssat[i].fix[j];
      vt_printf(vt, " %5s", fix == 1 ? "FLOAT" : (fix == 2 ? "FIX" : (fix == 3 ? "HOLD" : "-")));
    }
    for (int j = 0; j < nf; j++) vt_printf(vt, "%7.3Lf", rtk.ssat[i].resp[j]);
    for (int j = 0; j < nf; j++) vt_printf(vt, "%8.4Lf", rtk.ssat[i].resc[j]);
    for (int j = 0; j < nf; j++) vt_printf(vt, " %4d", rtk.ssat[i].slipc[j]);
    for (int j = 0; j < nf; j++) vt_printf(vt, " %6d", rtk.ssat[i].lock[j]);
    for (int j = 0; j < nf; j++) vt_printf(vt, " %3d", rtk.ssat[i].rejc[j]);
    vt_printf(vt, "\n");
  }
}
/* print observation data ----------------------------------------------------*/
static void probserv(vt_t *vt, int nf) {
  trace(4, "probserv:\n");

  int n = 0;
  obsd_t obs[MAXOBS * 2];
  rtksvrlock(&svr);
  for (int i = 0; i < svr.obs[0][0].n && n < MAXOBS * 2; i++) {
    obs[n++] = svr.obs[0][0].data[i];
  }
  for (int i = 0; i < svr.obs[1][0].n && n < MAXOBS * 2; i++) {
    obs[n++] = svr.obs[1][0].data[i];
  }
  rtksvrunlock(&svr);

  if (nf <= 0 || nf > NFREQ) nf = NFREQ;
  vt_printf(vt, "\n%s%-22s %3s %s", ESC_BOLD, "      TIME(GPST)", "SAT", "R");
  int frq[] = {1, 2, 5, 7, 8, 6, 9};
  for (int i = 0; i < nf; i++) vt_printf(vt, "        P%d(m)", frq[i]);
  for (int i = 0; i < nf; i++) vt_printf(vt, "       L%d(cyc)", frq[i]);
  for (int i = 0; i < nf; i++) vt_printf(vt, "  D%d(Hz)", frq[i]);
  for (int i = 0; i < nf; i++) vt_printf(vt, " S%d", frq[i]);
  vt_printf(vt, " LLI%s\n", ESC_RESET);
  for (int i = 0; i < n; i++) {
    char tstr[40];
    time2str(obs[i].time, tstr, 2);
    char id[8];
    satno2id(obs[i].sat, id);
    vt_printf(vt, "%s %3s %d", tstr, id, obs[i].rcv);
    for (int j = 0; j < nf; j++) vt_printf(vt, "%13.3Lf", obs[i].P[j]);
    for (int j = 0; j < nf; j++) vt_printf(vt, "%14.3Lf", obs[i].L[j]);
    for (int j = 0; j < nf; j++) vt_printf(vt, "%8.1Lf", obs[i].D[j]);
    for (int j = 0; j < nf; j++) vt_printf(vt, "%3.0Lf", obs[i].SNR[j] * SNR_UNIT);
    for (int j = 0; j < nf; j++) vt_printf(vt, "%2d", obs[i].LLI[j]);
    vt_printf(vt, "\n");
  }
}
/* print navigation data -----------------------------------------------------*/
static void prnavidata(vt_t *vt) {
  trace(4, "prnavidata:\n");

  rtksvrlock(&svr);
  gtime_t time = svr.rtk.sol.time;
  eph_t eph[MAXSAT];
  for (int i = 0; i < MAXSAT; i++) eph[i] = svr.nav.eph[i][0];
  geph_t geph[MAXPRNGLO];
  for (int i = 0; i < MAXPRNGLO; i++) geph[i] = svr.nav.geph[i][0];
  long double ion[8];
  for (int i = 0; i < 8; i++) ion[i] = svr.nav.ion_gps[i];
  long double utc[8];
  for (int i = 0; i < 8; i++) utc[i] = svr.nav.utc_gps[i];
  rtksvrunlock(&svr);

  vt_printf(vt, "\n%s%3s %3s %3s %3s %3s %3s %3s %19s %19s %19s %3s %3s%s\n", ESC_BOLD, "SAT", "S",
            "IOD", "IOC", "FRQ", "A/A", "SVH", "Toe", "Toc", "Ttr/Tof", "L2C", "L2P", ESC_RESET);
  for (int i = 0; i < MAXSAT; i++) {
    int prn;
    if (!(satsys(i + 1, &prn) & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_CMP)) || eph[i].sat != i + 1)
      continue;
    int valid = eph[i].toe.time != 0 && !eph[i].svh && fabsl(timediff(time, eph[i].toe)) <= MAXDTOE;
    char id[8];
    satno2id(i + 1, id);
    char s1[64];
    if (eph[i].toe.time != 0)
      time2str(eph[i].toe, s1, 0);
    else
      rtkstrcpy(s1, sizeof(s1), "-");
    char s2[64];
    if (eph[i].toc.time != 0)
      time2str(eph[i].toc, s2, 0);
    else
      rtkstrcpy(s2, sizeof(s2), "-");
    char s3[64];
    if (eph[i].ttr.time != 0)
      time2str(eph[i].ttr, s3, 0);
    else
      rtkstrcpy(s3, sizeof(s3), "-");
    vt_printf(vt, "%3s %3s %3d %3d %3d %3d %03X %19s %19s %19s %3d %3d\n", id, valid ? "OK" : "-",
              eph[i].iode, eph[i].iodc, 0, eph[i].sva, eph[i].svh, s1, s2, s3, eph[i].code,
              eph[i].flag);
  }
  for (int i = 0; i < MAXSAT; i++) {
    int prn;
    if (!(satsys(i + 1, &prn) & SYS_GLO) || geph[prn - 1].sat != i + 1) continue;
    int valid = geph[prn - 1].toe.time != 0 && !geph[prn - 1].svh &&
                fabsl(timediff(time, geph[prn - 1].toe)) <= MAXDTOE_GLO;
    char id[8];
    satno2id(i + 1, id);
    char s1[64];
    if (geph[prn - 1].toe.time != 0)
      time2str(geph[prn - 1].toe, s1, 0);
    else
      rtkstrcpy(s1, sizeof(s1), "-");
    char s2[64];
    if (geph[prn - 1].tof.time != 0)
      time2str(geph[prn - 1].tof, s2, 0);
    else
      rtkstrcpy(s2, sizeof(s2), "-");
    vt_printf(vt, "%3s %3s %3d %3d %3d %3d  %02X %19s %19s %19s %3d %3d\n", id, valid ? "OK" : "-",
              geph[prn - 1].iode, 0, geph[prn - 1].frq, geph[prn - 1].age, geph[prn].svh, s1, "-",
              s2, 0, 0);
  }
  vt_printf(vt, "ION: %9.2LE %9.2LE %9.2LE %9.2LE %9.2LE %9.2LE %9.2LE %9.2LE\n", ion[0], ion[1], ion[2],
            ion[3], ion[4], ion[5], ion[6], ion[7]);
  vt_printf(vt, "UTC: %9.2LE %9.2LE %9.2LE %9.2LE  LEAPS: %.0Lf\n", utc[0], utc[1], utc[2], utc[3],
            utc[4]);
}
/* print error/warning messages ----------------------------------------------*/
static void prerror(vt_t *vt) {
  trace(4, "prerror:\n");

  rtksvrlock(&svr);
  int n = svr.rtk.neb;
  if (n > 0) {
    svr.rtk.errbuf[n] = '\0';
    vt_puts(vt, svr.rtk.errbuf);
    svr.rtk.neb = 0;
  }
  rtksvrunlock(&svr);
}
/* print stream --------------------------------------------------------------*/
static void prstream(vt_t *vt) {
  trace(4, "prstream:\n");

  rtksvrlock(&svr);
  stream_t stream[9];
  for (int i = 0; i < 8; i++) stream[i] = svr.stream[i];
  int format[9] = {0};
  for (int i = 0; i < 3; i++) format[i] = svr.format[i];
  for (int i = 3; i < 5; i++) format[i] = svr.solopt[i - 3].posf;
  stream[8] = moni;
  format[8] = SOLF_LLH;
  rtksvrunlock(&svr);

  vt_printf(vt, "\n%s%-12s %-8s %-5s %s %10s %7s %10s %7s %-24s %s%s\n", ESC_BOLD, "Stream", "Type",
            "Fmt", "S", "In-byte", "In-bps", "Out-byte", "Out-bps", "Path", "Message", ESC_RESET);
  for (int i = 0; i < 9; i++) {
    const char *ch[] = {"input rover", "input base", "input corr", "output sol1", "output sol2",
                        "log rover",   "log base",   "log corr",   "monitor"};
    const char *type[] = {"-",      "serial", "file", "tcpsvr", "tcpcli",  "udp",
                          "ntrips", "ntripc", "ftp",  "http",   "ntripcas"};
    const char *fmt[] = {"rtcm2", "rtcm3",   "oem4",  "",    "ubx",   "swift",
                         "hemis", "skytreq", "javad", "nvs", "binex", "rt17",
                         "sbf",   "",        "",      "sp3", ""};
    const char *sol[] = {"llh", "xyz", "enu", "nmea", "stat", "-"};
    vt_printf(vt, "%-12s %-8s %-5s %s %10d %7d %10d %7d %-24.24s %s\n", ch[i], type[stream[i].type],
              i < 3 ? fmt[format[i]] : (i < 5 || i == 8 ? sol[format[i]] : "-"),
              stream[i].state < 0 ? "E" : (stream[i].state ? "C" : "-"), stream[i].inb,
              stream[i].inr, stream[i].outb, stream[i].outr, stream[i].path, stream[i].msg);
  }
}
/* print ssr correction ------------------------------------------------------*/
static void prssr(vt_t *vt) {
  rtksvrlock(&svr);
  gtime_t time = svr.rtk.sol.time;
  ssr_t ssr[MAXSAT];
  for (int i = 0; i < MAXSAT; i++) {
    ssr[i] = svr.nav.ssr[i];
  }
  rtksvrunlock(&svr);

  static char buff[128 * MAXSAT];
  rtksnprintf(buff, sizeof(buff),
              "\n%s%3s %3s %3s %3s %3s %19s %6s %6s %6s %6s %6s %6s %8s "
              "%6s %6s %6s%s\n",
              ESC_BOLD, "SAT", "S", "UDI", "IOD", "URA", "T0", "D0-A", "D0-C", "D0-R", "D1-A",
              "D1-C", "D1-R", "C0", "C1", "C2", "C-HR", ESC_RESET);
  for (int i = 0; i < MAXSAT; i++) {
    if (!ssr[i].t0[0].time) continue;
    char id[8];
    satno2id(i + 1, id);
    int valid = fabsl(timediff(time, ssr[i].t0[0])) <= 1800.0L;
    char tstr[40];
    time2str(ssr[i].t0[0], tstr, 0);
    rtkcatprintf(buff, sizeof(buff),
                 "%3s %3s %3.0Lf %3d %3d %19s %6.3Lf %6.3Lf %6.3Lf %6.3Lf %6.3Lf "
                 "%6.3Lf %8.3Lf %6.3Lf %6.4Lf %6.3Lf\n",
                 id, valid ? "OK" : "-", ssr[i].udi[0], ssr[i].iode, ssr[i].ura, tstr,
                 ssr[i].deph[0], ssr[i].deph[1], ssr[i].deph[2], ssr[i].ddeph[0] * 1E3L,
                 ssr[i].ddeph[1] * 1E3L, ssr[i].ddeph[2] * 1E3L, ssr[i].dclk[0], ssr[i].dclk[1] * 1E3L,
                 ssr[i].dclk[2] * 1E3L, ssr[i].hrclk);
  }
  vt_puts(vt, buff);
}
/* start command -------------------------------------------------------------*/
static void cmd_start(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_start:\n");

  if (!startsvr(vt)) return;
  vt_printf(vt, "rtk server start\n");
}
/* stop command --------------------------------------------------------------*/
static void cmd_stop(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_stop:\n");

  stopsvr(vt);
  vt_printf(vt, "rtk server stop\n");
}
/* restart command -----------------------------------------------------------*/
static void cmd_restart(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_restart:\n");

  stopsvr(vt);
  if (!startsvr(vt)) return;
  vt_printf(vt, "rtk server restart\n");
}
/* solution command ----------------------------------------------------------*/
static void cmd_solution(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_solution:\n");

  int cycle = 0;
  if (narg > 1) cycle = (int)(strtold(args[1], NULL) * 1000.0L);

  if (cycle > 0) svr.nsol = 0;

  while (!vt_chkbrk(vt)) {
    rtksvrlock(&svr);
    for (int i = 0; i < svr.nsol; i++) prsolution(vt, &svr.solbuf[i], svr.rtk.rb);
    svr.nsol = 0;
    rtksvrunlock(&svr);
    if (cycle > 0)
      sleepms(cycle);
    else
      return;
  }
}
/* status command ------------------------------------------------------------*/
static void cmd_status(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_status:\n");

  int cycle = 0;
  if (narg > 1) cycle = (int)(strtold(args[1], NULL) * 1000.0L);

  while (!vt_chkbrk(vt)) {
    if (cycle > 0) vt_printf(vt, ESC_CLEAR);
    prstatus(vt);
    if (cycle > 0)
      sleepms(cycle);
    else
      return;
  }
  vt_printf(vt, "\n");
}
/* satellite command ---------------------------------------------------------*/
static void cmd_satellite(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_satellite:\n");

  int nf = 2, cycle = 0;
  for (int i = 1; i < narg; i++) {
    if (sscanf(args[i], "-%d", &nf) < 1) cycle = (int)(strtold(args[i], NULL) * 1000.0L);
  }
  while (!vt_chkbrk(vt)) {
    if (cycle > 0) vt_printf(vt, ESC_CLEAR);
    prsatellite(vt, nf);
    if (cycle > 0)
      sleepms(cycle);
    else
      return;
  }
  vt_printf(vt, "\n");
}
/* observ command ------------------------------------------------------------*/
static void cmd_observ(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_observ:\n");

  int nf = 2, cycle = 0;
  for (int i = 1; i < narg; i++) {
    if (sscanf(args[i], "-%d", &nf) < 1) cycle = (int)(strtold(args[i], NULL) * 1000.0L);
  }
  while (!vt_chkbrk(vt)) {
    if (cycle > 0) vt_printf(vt, ESC_CLEAR);
    probserv(vt, nf);
    if (cycle > 0)
      sleepms(cycle);
    else
      return;
  }
  vt_printf(vt, "\n");
}
/* navidata command ----------------------------------------------------------*/
static void cmd_navidata(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_navidata:\n");

  int cycle = 0;
  if (narg > 1) cycle = (int)(strtold(args[1], NULL) * 1000.0L);

  while (!vt_chkbrk(vt)) {
    if (cycle > 0) vt_printf(vt, ESC_CLEAR);
    prnavidata(vt);
    if (cycle > 0)
      sleepms(cycle);
    else
      return;
  }
  vt_printf(vt, "\n");
}
/* error command -------------------------------------------------------------*/
static void cmd_error(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_error:\n");

  rtksvrlock(&svr);
  svr.rtk.neb = 0;
  rtksvrunlock(&svr);

  while (!vt_chkbrk(vt)) {
    prerror(vt);
    sleepms(100);
  }
  vt_printf(vt, "\n");
}
/* stream command ------------------------------------------------------------*/
static void cmd_stream(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_stream:\n");

  int cycle = 0;
  if (narg > 1) cycle = (int)(strtold(args[1], NULL) * 1000.0L);

  while (!vt_chkbrk(vt)) {
    if (cycle > 0) vt_printf(vt, ESC_CLEAR);
    prstream(vt);
    if (cycle > 0)
      sleepms(cycle);
    else
      return;
  }
  vt_printf(vt, "\n");
}
/* ssr command ---------------------------------------------------------------*/
static void cmd_ssr(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_ssr:\n");

  int cycle = 0;
  if (narg > 1) cycle = (int)(strtold(args[1], NULL) * 1000.0L);

  while (!vt_chkbrk(vt)) {
    if (cycle > 0) vt_printf(vt, ESC_CLEAR);
    prssr(vt);
    if (cycle > 0)
      sleepms(cycle);
    else
      return;
  }
  vt_printf(vt, "\n");
}
/* option command ------------------------------------------------------------*/
static void cmd_option(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_option:\n");

  for (int i = 0; *rcvopts[i].name; i++) {
    if (narg >= 2 && !strstr(rcvopts[i].name, args[1])) continue;
    char buff[MAXSTR];
    buff[0] = '\0';
    rtkcatprintf(buff, sizeof(buff), "%-18s =", rcvopts[i].name);
    opt2str(rcvopts + i, buff, sizeof(buff));
    if (*rcvopts[i].comment) {
      int n = 30 - strlen(buff);
      if (n > 0) {
        rtkcatprintf(buff, sizeof(buff), "%*s", n, "");
      }
      rtkcatprintf(buff, sizeof(buff), " # (%s)", rcvopts[i].comment);
    }
    vt_printf(vt, "%s%s\n", modflgr[i] ? "*" : " ", buff);
  }
  for (int i = 0; *sysopts[i].name; i++) {
    if (narg >= 2 && !strstr(sysopts[i].name, args[1])) continue;
    char buff[MAXSTR];
    buff[0] = '\0';
    rtkcatprintf(buff, sizeof(buff), "%-18s =", sysopts[i].name);
    opt2str(sysopts + i, buff, sizeof(buff));
    if (*sysopts[i].comment) {
      int n = 30 - strlen(buff);
      if (n > 0) {
        rtkcatprintf(buff, sizeof(buff), "%*s", n, "");
      }
      rtkcatprintf(buff, sizeof(buff), " # (%s)", sysopts[i].comment);
    }
    vt_printf(vt, "%s%s\n", modflgs[i] ? "*" : " ", buff);
  }
}
/* set command ---------------------------------------------------------------*/
static void cmd_set(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_set:\n");

  if (narg < 2) {
    vt_printf(vt, "specify option type\n");
    return;
  }
  int *modf;
  opt_t *opt = searchopt(args[1], rcvopts);
  if (opt) {
    modf = modflgr + (int)(opt - rcvopts);
  } else if ((opt = searchopt(args[1], sysopts))) {
    modf = modflgs + (int)(opt - sysopts);
  } else {
    vt_printf(vt, "no option type: %s\n", args[1]);
    return;
  }
  char buff[MAXSTR];
  if (narg < 3) {
    vt_printf(vt, "%s", opt->name);
    if (*opt->comment) vt_printf(vt, " (%s)", opt->comment);
    vt_printf(vt, ": ");
    if (!vt_gets(vt, buff, sizeof(buff)) || vt->brk) return;
  } else
    rtkstrcpy(buff, sizeof(buff), args[2]);

  chop(buff);
  if (!str2opt(opt, buff)) {
    vt_printf(vt, "invalid option value: %s %s\n", opt->name, buff);
    return;
  }
  getsysopts(&prcopt, solopt, &filopt);
  solopt[1] = solopt[0];

  vt_printf(vt, "option %s changed.", opt->name);
  if (strncmp(opt->name, "console", 7)) {
    *modf = 1;
    vt_printf(vt, " restart to enable it");
  }
  vt_printf(vt, "\n");
}
/* load command --------------------------------------------------------------*/
static void cmd_load(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_load:\n");

  char file[MAXSTR] = "";
  if (narg >= 2) {
    rtkstrcpy(file, sizeof(file), args[1]);
  } else {
    rtksnprintf(file, sizeof(file), "%s/%s", OPTSDIR, OPTSFILE);
  }
  resetsysopts();
  if (!loadopts(file, sysopts)) {
    vt_printf(vt, "no options file: %s\n", file);
    return;
  }
  getsysopts(&prcopt, solopt, &filopt);
  solopt[1] = solopt[0];

  if (!loadopts(file, rcvopts)) {
    vt_printf(vt, "no options file: %s\n", file);
    return;
  }
  vt_printf(vt, "options loaded from %s. restart to enable them\n", file);
}
/* save command --------------------------------------------------------------*/
static void cmd_save(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_save:\n");

  char file[MAXSTR] = "";
  if (narg >= 2) {
    rtkstrcpy(file, sizeof(file), args[1]);
  } else {
    rtksnprintf(file, sizeof(file), "%s/%s", OPTSDIR, OPTSFILE);
  }
  if (!confwrite(vt, file)) return;
  char s[40];
  time2str(utc2gpst(timeget()), s, 0);
  char comment[256];
  rtksnprintf(comment, sizeof(comment), "%s options (%s, v.%s %s)", PRGNAME, s, VER_RTKLIB,
              PATCH_LEVEL);
  setsysopts(&prcopt, solopt, &filopt);
  if (!saveopts(file, "w", comment, rcvopts) || !saveopts(file, "a", NULL, sysopts)) {
    vt_printf(vt, "options save error: %s\n", file);
    return;
  }
  vt_printf(vt, "options saved to %s\n", file);
}
/* log command ---------------------------------------------------------------*/
static void cmd_log(char **args, int narg, vt_t *vt) {
  trace(3, "cmd_log:\n");

  if (narg < 2) {
    vt_printf(vt, "specify log file\n");
    return;
  }
  if (!strcmp(args[1], "off")) {
    vt_closelog(vt);
    vt_printf(vt, "log off\n");
    return;
  }
  if (!confwrite(vt, args[1])) return;

  if (!vt_openlog(vt, args[1])) {
    vt_printf(vt, "log open error: %s\n", args[1]);
    return;
  }
  vt_printf(vt, "log on: %s\n", args[1]);
}
/* help command --------------------------------------------------------------*/
static void cmd_help(char **args, int narg, vt_t *vt) {
  char str[] = "path";
  if (narg < 2) {
    vt_printf(vt, "%s (ver.%s %s)\n", PRGNAME, VER_RTKLIB, PATCH_LEVEL);
    for (int i = 0; *helptxt[i]; i++) vt_printf(vt, "%s\n", helptxt[i]);
  } else if (strstr(str, args[1]) == str) {
    for (int i = 0; *pathopts[i]; i++) vt_printf(vt, "%s\n", pathopts[i]);
  } else {
    vt_printf(vt, "unknown help: %s\n", args[1]);
  }
}
/* exec command --------------------------------------------------------------*/
static int cmd_exec(const char *cmd, vt_t *vt) {
  FILE *fp = popen(cmd, "r");
  if (!fp) {
    vt_printf(vt, "command exec error\n");
    return -1;
  }
  while (!vt_chkbrk(vt)) {
    char buff[MAXSTR];
    if (!fgets(buff, sizeof(buff), fp)) break;
    vt_printf(vt, buff);
  }
  int ret = pclose(fp);
  if (ret) {
    vt_printf(vt, "command exec error (%d)\n", ret);
  }
  return ret;
}
/* console thread ------------------------------------------------------------*/
static void *con_thread(void *arg) {
  trace(3, "console_thread:\n");

  const char *cmds[] = {"start",  "stop",     "restart", "solution", "status", "satellite",
                        "observ", "navidata", "stream",  "ssr",      "error",  "option",
                        "set",    "load",     "save",    "log",      "help",   "?",
                        "exit",   "shutdown", ""};
  con_t *con = (con_t *)arg;

  vt_printf(con->vt, "\n%s** %s ver.%s %s console (h:help) **%s\n", ESC_BOLD, PRGNAME, VER_RTKLIB,
            PATCH_LEVEL, ESC_RESET);

  if (!login(con->vt)) {
    vt_close(con->vt);
    con->state = 0;
    return NULL;
  }

  /* auto start if option set */
  if (start & 1) { /* start with console */
    cmd_start(NULL, 0, con->vt);
    start = 0;
  }

  while (con->state) {
    /* output prompt */
    if (!vt_puts(con->vt, CMDPROMPT)) break;

    /* input command */
    char buff[MAXCMD];
    if (!vt_gets(con->vt, buff, sizeof(buff))) break;

    if (buff[0] == '!') { /* shell escape */
      cmd_exec(buff + 1, con->vt);
      continue;
    }
    /* parse command */
    char *args[MAXARG];
    int narg = 0;
    char *r;
    for (char *p = strtok_r(buff, " \t\n", &r); p && narg < MAXARG;
         p = strtok_r(NULL, " \t\n", &r)) {
      args[narg++] = p;
    }
    if (narg == 0) continue;

    int j = -1;
    for (int i = 0; *cmds[i]; i++) {
      if (strstr(cmds[i], args[0]) == cmds[i]) j = i;
    }
    switch (j) {
      case 0:
        cmd_start(args, narg, con->vt);
        break;
      case 1:
        cmd_stop(args, narg, con->vt);
        break;
      case 2:
        cmd_restart(args, narg, con->vt);
        break;
      case 3:
        cmd_solution(args, narg, con->vt);
        break;
      case 4:
        cmd_status(args, narg, con->vt);
        break;
      case 5:
        cmd_satellite(args, narg, con->vt);
        break;
      case 6:
        cmd_observ(args, narg, con->vt);
        break;
      case 7:
        cmd_navidata(args, narg, con->vt);
        break;
      case 8:
        cmd_stream(args, narg, con->vt);
        break;
      case 9:
        cmd_ssr(args, narg, con->vt);
        break;
      case 10:
        cmd_error(args, narg, con->vt);
        break;
      case 11:
        cmd_option(args, narg, con->vt);
        break;
      case 12:
        cmd_set(args, narg, con->vt);
        break;
      case 13:
        cmd_load(args, narg, con->vt);
        break;
      case 14:
        cmd_save(args, narg, con->vt);
        break;
      case 15:
        cmd_log(args, narg, con->vt);
        break;
      case 16:
        cmd_help(args, narg, con->vt);
        break;
      case 17:
        cmd_help(args, narg, con->vt);
        break;
      case 18: /* exit */
        if (con->vt->type) con->state = 0;
        break;
      case 19: /* shutdown */
        if (!strcmp(args[0], "shutdown")) {
          vt_printf(con->vt, "rtk server shutdown ...\n");
          sleepms(1000);
          intflg = 1;
          con->state = 0;
        }
        break;
      default:
        vt_printf(con->vt, "unknown command: %s.\n", args[0]);
        break;
    }
  }
  vt_close(con->vt);
  return NULL;
}
/* open console --------------------------------------------------------------*/
static con_t *con_open(int sock, const char *dev) {
  trace(3, "con_open: sock=%d dev=%s\n", sock, dev);

  con_t *con = (con_t *)malloc(sizeof(con_t));
  if (!con) return NULL;

  if (!(con->vt = vt_open(sock, dev))) {
    free(con);
    return NULL;
  }
  /* start console thread */
  con->state = 1;
  if (pthread_create(&con->thread, NULL, con_thread, con)) {
    free(con);
    return NULL;
  }
  return con;
}
/* close console -------------------------------------------------------------*/
static void con_close(con_t *con) {
  trace(3, "con_close:\n");

  if (!con) return;
  con->state = 0;
  pthread_join(con->thread, NULL);
  free(con);
}
/* open socket for remote console --------------------------------------------*/
static int open_sock(int port) {
  trace(3, "open_sock: port=%d\n", port);

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    fprintf(stderr, "socket error (%d)\n", errno);
    return 0;
  }
  int on = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on));
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    fprintf(stderr, "bind error (%d)\n", errno);
    close(sock);
    return -1;
  }
  listen(sock, 5);
  return sock;
}
/* accept remote console connection ------------------------------------------*/
static void accept_sock(int ssock, con_t **con) {
  if (ssock <= 0) return;

  trace(4, "accept_sock: ssock=%d\n", ssock);

  for (int i = 1; i < MAXCON; i++) {
    if (!con[i] || con[i]->state) continue;
    con_close(con[i]);
    con[i] = NULL;
  }
  fd_set rs;
  FD_ZERO(&rs);
  FD_SET(ssock, &rs);
  struct timeval tv = {0};
  if (select(ssock + 1, &rs, NULL, NULL, &tv) <= 0) return;
  struct sockaddr_in addr;
  socklen_t len = sizeof(addr);
  int sock = accept(ssock, (struct sockaddr *)&addr, &len);
  if (sock <= 0) return;
  for (int i = 1; i < MAXCON; i++) {
    if (con[i]) continue;

    con[i] = con_open(sock, "");

    trace(3, "remote console connected: addr=%s\n", inet_ntoa(addr.sin_addr));
    return;
  }
  close(sock);
  trace(2, "remote console connection refused. addr=%s\n", inet_ntoa(addr.sin_addr));
}
/* rtkrcv main -----------------------------------------------------------------
 * synopsis
 *     rtkrcv [-s][-p port][-d dev][-o file][-r level][-t level][-sta sta]
 *
 * description
 *     A command line version of the real-time positioning AP by rtklib. To start
 *     or stop RTK server, to configure options or to print solution/status,
 *     login a console and input commands. As default, /dev/tty is used for the
 *     console. Use -p option for network login with telnet protocol. To show
 *     the available commands, type ? or help on the console. With -p option,
 *     multiple telnet console logins are allowed. The initial processing options
 *     are loaded from default file rtkrcv.conf. To change the file, use -o
 *     option. To configure the processing options, edit the options file or use
 *     set, load or save command on the console. To shutdown the program, use
 *     shutdown command on the console or send USR2 signal to the process.
 *
 * option
 *     -s         start RTK server on program startup
 *     -p port    port number for telnet console
 *     -m port    port number for monitor stream
 *     -d dev     terminal device for console
 *     -o file    processing options file
 *     -w pwd     login password for remote console ("": no password)
 *     -r level   output solution status file (0:off,1:states,2:residuals)
 *     -t level   debug trace level (0:off,1-5:on)
 *     -sta sta   station name for receiver dcb
 *
 * command
 *     start
 *       Start RTK server. No need the command if the program runs with -s
 *       option.
 *
 *     stop
 *       Stop RTK server.
 *
 *     restart
 *       Restart RTK server. If the processing options are set, execute the
 *       command to enable the changes.
 *
 *     solution [cycle]
 *       Show solutions. Without option, only one solution is shown. With
 *       option, the solution is displayed at intervals of cycle (s). To stop
 *       cyclic display, send break (ctr-C).
 *
 *     status [cycle]
 *       Show RTK status. Use option cycle for cyclic display.
 *
 *     satellite [-n] [cycle]
 *       Show satellite status. Use option cycle for cyclic display. Option -n
 *       specify number of frequencies.
 *
 *     observ [-n] [cycle]
 *       Show observation data. Use option cycle for cyclic display. Option -n
 *       specify number of frequencies.
 *
 *     navidata [cycle]
 *       Show navigation data. Use option cycle for cyclic display.
 *
 *     stream [cycle]
 *       Show stream status. Use option cycle for cyclic display.
 *
 *     error
 *       Show error/warning messages. To stop messages, send break (ctr-C).
 *
 *     option [opt]
 *       Show the values of processing options. Without option, all options are
 *       displayed. With option, only pattern-matched options are displayed.
 *
 *     set opt [val]
 *       Set the value of a processing option to val. With out option val,
 *       prompt message is shown to input the value. The change of the
 *       processing option is not enabled before RTK server is restarted.
 *
 *     load [file]
 *       Load processing options from file. Without option, default file
 *       rtkrcv.conf is used. To enable the changes, restart RTK server.
 *
 *     save [file]
 *       Save current processing options to file. Without option, default file
 *       rtkrcv.conf is used.
 *
 *     log [file|off]
 *       Record console log to file. To stop recording the log, use option off.
 *
 *     help|? [path]
 *       Show the command list. With option path, the stream path options are
 *       shown.
 *
 *     exit
 *       Exit and logout console. The status of RTK server is not affected by
 *       the command.
 *
 *     shutdown
 *       Shutdown RTK server and exit the program.
 *
 *     !command [arg...]
 *       Execute command by the operating system shell. Do not use the
 *       interactive command.
 *
 * notes
 *     Short form of a command is allowed. In case of the short form, the
 *     command is distinguished according to header characters.
 *
 *     The -r argument only affects the status file. The status output streams
 *     take their level from the out-outstat option.
 *
 *-----------------------------------------------------------------------------*/
int main(int argc, char **argv) {
  int port = 0, outstat = 0, trace = 0;
  const char *dev = "";
  char file[MAXSTR] = "";
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-s"))
      start |= 1; /* console */
    else if (!strcmp(argv[i], "-nc"))
      start |= 2; /* no console */
    else if (!strcmp(argv[i], "-p") && i + 1 < argc)
      port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-m") && i + 1 < argc)
      moniport = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-d") && i + 1 < argc)
      dev = argv[++i];
    else if (!strcmp(argv[i], "-o") && i + 1 < argc)
      rtkstrcpy(file, sizeof(file), argv[++i]);
    else if (!strcmp(argv[i], "-w") && i + 1 < argc)
      rtkstrcpy(passwd, sizeof(passwd), argv[++i]);
    else if (!strcmp(argv[i], "-r") && i + 1 < argc)
      outstat = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-t") && i + 1 < argc)
      trace = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-sta") && i + 1 < argc)
      rtkstrcpy(sta_name, sizeof(sta_name), argv[++i]);
    else
      printusage();
  }
  if (trace > 0) {
    traceopen(TRACEFILE);
    tracelevel(trace);
  }
  /* initialize rtk server and monitor port */
  rtksvrinit(&svr);
  strinit(&moni);

  /* load options file */
  if (!*file) rtksnprintf(file, sizeof(file), "%s/%s", OPTSDIR, OPTSFILE);

  resetsysopts();
  if (!loadopts(file, rcvopts) || !loadopts(file, sysopts)) {
    fprintf(stderr, "no options file: %s. defaults used\n", file);
  }
  getsysopts(&prcopt, solopt, &filopt);
  /* Copy the system options for the second output solution stream */
  solopt[1] = solopt[0];

  /* read navigation data */
  if (!readnav(NAVIFILE, &svr.nav)) {
    fprintf(stderr, "no navigation data: %s\n", NAVIFILE);
  }
  if (outstat > 0) {
    rtkopenstat(STATFILE, outstat);
  }
  /* open monitor port */
  if (moniport > 0 && !openmoni(moniport)) {
    fprintf(stderr, "monitor port open error: %d\n", moniport);
  }
  con_t *con[MAXCON] = {0};
  int sock = 0;
  if (port) {
    /* open socket for remote console */
    sock = open_sock(port);
    if (sock <= 0) {
      fprintf(stderr, "console open error port=%d\n", port);
      if (moniport > 0) closemoni();
      if (outstat > 0) rtkclosestat();
      traceclose();
      return EXIT_FAILURE;
    }
  } else if (start & 2) { /* start without console */
    startsvr(NULL);
  } else {
    /* open device for local console */
    if (!(con[0] = con_open(0, dev))) {
      fprintf(stderr, "console open error dev=%s\n", dev);
      if (moniport > 0) closemoni();
      if (outstat > 0) rtkclosestat();
      traceclose();
      return EXIT_FAILURE;
    }
  }
  signal(SIGINT, sigshut);  /* keyboard interrupt */
  signal(SIGTERM, sigshut); /* external shutdown signal */
  signal(SIGUSR2, sigshut);
  signal(SIGHUP, SIG_IGN);
  signal(SIGPIPE, SIG_IGN);

  while (!intflg) {
    /* accept remote console connection */
    accept_sock(sock, con);
    sleepms(100);
  }
  /* stop rtk server */
  stopsvr(NULL);

  /* close consoles */
  for (int i = 0; i < MAXCON; i++) {
    con_close(con[i]);
  }
  if (moniport > 0) closemoni();
  if (outstat > 0) rtkclosestat();

  /* save navigation data */
  if (!savenav(NAVIFILE, &svr.nav)) {
    fprintf(stderr, "navigation data save error: %s\n", NAVIFILE);
  }
  traceclose();
  return 0;
}
