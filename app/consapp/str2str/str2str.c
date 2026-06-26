//------------------------------------------------------------------------------
// str2str.c : console version of stream server
//
//          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
//
// Version : $Revision: 1.1 $ $Date: 2008/07/17 21:54:53 $
// History : 2009/06/17  1.0  new
//           2011/05/29  1.1  add -f, -l and -x option
//           2011/11/29  1.2  fix bug on recognize ntrips:// (rtklib_2.4.1_p4)
//           2012/12/25  1.3  add format conversion functions
//                            add -msg, -opt and -sta options
//                            modify -p option
//           2013/01/25  1.4  fix bug on showing message
//           2014/02/21  1.5  ignore SIG_HUP
//           2014/08/10  1.5  fix bug on showing message
//           2014/08/26  1.6  support input format GW10, binex and rt17
//           2014/10/14  1.7  use stdin or stdout if option -in or -out omitted
//           2014/11/08  1.8  add option -a, -i and -o
//           2015/03/23  1.9  fix bug on parsing of command line options
//           2016/01/23  1.10 enable septentrio
//           2016/01/26  1.11 fix bug on station position by -p option (#126)
//                            add option -px
//           2016/07/01  1.12 support CMR/CMR+
//           2016/07/23  1.13 add option -c1 -c2 -c3 -c4
//           2016/09/03  1.14 support NTRIP caster
//                            add option -ft,-fl
//           2016/09/06  1.15 add reload source table by USR2 signal
//           2016/09/17  1.16 add option -b
//           2017/05/26  1.17 add input format tersus
//           2020/11/30  1.18 support api change strsvrstart(),strsvrstat()
//----------------------------------------------------------------------------
#define _POSIX_C_SOURCE 200112L
#include <fcntl.h>
#include <signal.h>
#ifndef WIN32
#include <unistd.h>
#endif
#include "rtklib.h"

#define PRGNAME "str2str"                     // Program name.
#define TRACEFILE "str2str_%Y%m%d%h%M.trace"  // Debug trace file.
#define LOGFILE "str2str_%Y%m%d%h%M.log"      // Deamon log file.

/* global variables ----------------------------------------------------------*/
static volatile int intrflg = 0;  // Interrupt flag.

// Help text -----------------------------------------------------------------
static const char *help[] = {
    "",
    " usage: str2str [-in stream] [[-msg ...] -out stream [-out stream...]] [options]",
    "",
    " Input data from a stream and divide and output them to multiple streams.",
    " The input stream can be serial, tcp client, tcp server, ntrip client, or",
    " a file. The output stream can be serial, tcp client, tcp server, ntrip",
    " server, or a file. str2str is a resident type application. To stop it, type",
    " ctr-c in the console if run in the foreground or send signal SIGINT for a",
    " background process. If both of the input stream and the output stream have a",
    " #format then the format of the input messages are converted to the output",
    " format. To specify the output messages use the -msg option before the",
    " respective -out options. To specify the log file use the -log option before",
    " a respective -in or -out option, and the streams should have separate",
    " log files. If the options -in or -out are omitted then stdin for input or",
    " stdout for output are used. If the stream for the option -in or -out is null,",
    " the stdin or stdout are used. Command options are as follows.",
    "",
    " -in  stream[#format] input  stream path and format",
    " -out stream[#format] output stream path and format",
    "",
    "  stream path",
    "    serial       : serial://port[:brate[:bsize[:parity[:stopb[:fctr]]]]][#tcp_port]",
    "    tcp server   : tcpsvr://:port",
    "    tcp client   : tcpcli://addr[:port]",
    "    ntrip client : ntripcli://[user[:passwd]@]addr[:port][/mntpnt]",
    "    ntrip source : ntripsrc://[user[:passwd]@]addr[:port]/mntpnt[:str] (only out)",
    "    ntrip caster : ntripcas://[user:passwd@][:port]/mntpnt[:srctbl]",
    "    udp server   : udpsvr://addr:port (only in)",
    "    udp client   : udpcli://addr:port (only out)",
    "    file         : [file://]path[::T][::+start][::xseppd][::S=swap]",
    "",
    "  format",
    "    rtcm2        : RTCM 2 (only in)",
    "    rtcm3        : RTCM 3",
    "    nov          : NovAtel OEMV/4/6,OEMStar (only in)",
    "    ubx          : ublox LEA-4T/5T/6T/8/9 (only in)",
    "    swiftnav     : SwiftNav Piksi Multi",
    "    hemis        : Hemisphere Eclipse/Crescent (only in)",
    "    stq          : SkyTraq S1315F (only in)",
    "    javad        : Javad (only in)",
    "    nvs          : NVS BINR (only in)",
    "    binex        : BINEX (only in)",
    "    rt17         : Trimble RT17 (only in)",
    "    sbf          : Septentrio SBF (only in)",
    "    unicore      : Unicore (only in)",
    "",
    " -msg \"type[(tint)][,type[(tint)]...]\"",
    "                   rtcm message types and output intervals (s)",
    " -log file         input log file or output return log file",
    " -sta sta          station id",
    " -opt opt          receiver dependent options",
    " -s  msec          timeout time (ms) [10000]",
    " -r  msec          reconnect interval (ms) [10000]",
    " -n  msec          nmea request cycle (m) [0]",
    " -f  sec           file swap margin (s) [30]",
    " -c  file          input commands file [no]",
    " -c1 file          output 1 commands file [no]",
    " -c2 file          output 2 commands file [no]",
    " -c3 file          output 3 commands file [no]",
    " -c4 file          output 4 commands file [no]",
    " -p  lat lon hgt   station position (latitude/longitude/height) (deg,m)",
    " -px x y z         station position (x/y/z-ecef) (m)",
    " -a  antinfo       antenna info (separated by ,)",
    " -i  rcvinfo       receiver info (separated by ,)",
    " -o  e n u         antenna offset (e,n,u) (m)",
    " -l  local_dir     ftp/http local directory []",
    " -x  proxy_addr    http/ntrip proxy address [no]",
    " -b  str_no        relay back messages from output str to input str [no]",
    " -t  level         trace level [0]",
    " -fl file          log file [str2str.trace]",
    " --tlssvrcert file TLS server certificate file",
    " --tlssvrkey file  TLS server certificate private key file",
    " --tlssvrcafile file TLS server CA file",
    " --tlssvrcadir dir TLS server CA directory",
    " --tlssvrverify n  TLS server peer certificate verify: 0 no, 1 yes [yes]",
    " --tlsclircert file TLS client certificate file",
    " --tlsclicafile file TLS client CA file",
    " --tlsclicadir dir TLS client CA directory",
    " --tlsclikey file  TLS client certificate private key file",
    " --tlscliverify n  TLS client peer certificate verify: 0 no, 1 yes [no]",
#ifndef WIN32
    " --daemon          detach from the console",
#endif
    " --version         print version",
    " -h                print help",
    "",
    "  command file cheat sheet:",
    "    - # begins a comment, empty lines are ignored",
    "    - @ separates sections: first startup commands, then stop commands, then periodic "
    "commands",
    "    - binary commands begin with !",
    "      - WAIT sleep for milliseconds (max 3 seconds)",
    "      - BRATE set the bitrate, defaults to 115200",
    "      - UBX send a UBX command (space separated; messages like CFG-MSG, followed by integer "
    "decimal (not hex) arguments)",
    "      - STQ send Skytraq commands",
    "      - NVS send Nvs commands",
    "      - HEX send hex messages",
    "    - other string lines (like NMEA commands) are send as is",
};
// Print help ----------------------------------------------------------------
static void printhelp(void) {
  for (unsigned i = 0; i < sizeof(help) / sizeof(*help); i++) fprintf(stderr, "%s\n", help[i]);
  exit(0);
}
// Signal handler ------------------------------------------------------------
static void sigfunc(int sig) {
  (void)sig;
  intrflg = 1;
}
// Decode format -------------------------------------------------------------
static void decodefmt(const char *path, int *fmt) {

  *fmt = -1;

  char *p = strrchr(path, '#');
  if (p != NULL) {
    if (!strcmp(p, "#rtcm2"))
      *fmt = STRFMT_RTCM2;
    else if (!strcmp(p, "#rtcm3"))
      *fmt = STRFMT_RTCM3;
    else if (!strcmp(p, "#nov"))
      *fmt = STRFMT_OEM4;
#ifdef RTK_DISABLED
    else if (!strcmp(p, "#cnav"))
      *fmt = STRFMT_CNAV;
#endif
    else if (!strcmp(p, "#ubx"))
      *fmt = STRFMT_UBX;
    else if (!strcmp(p, "#swift"))
      *fmt = STRFMT_SBP;
    else if (!strcmp(p, "#hemis"))
      *fmt = STRFMT_CRES;
    else if (!strcmp(p, "#stq"))
      *fmt = STRFMT_STQ;
    else if (!strcmp(p, "#javad"))
      *fmt = STRFMT_JAVAD;
    else if (!strcmp(p, "#nvs"))
      *fmt = STRFMT_NVS;
    else if (!strcmp(p, "#binex"))
      *fmt = STRFMT_BINEX;
    else if (!strcmp(p, "#rt17"))
      *fmt = STRFMT_RT17;
    else if (!strcmp(p, "#sbf"))
      *fmt = STRFMT_SEPT;
    else if (!strcmp(p, "#unicore"))
      *fmt = STRFMT_UNICORE;
    else
      return;
    *p = '\0';
  }
}
// Decode stream path --------------------------------------------------------
static unsigned decodepath(const char *path, unsigned *type, char *strpath, size_t size, int *fmt) {
  char buff[MAXSTRPATH];
  rsstrcpy(buff, sizeof(buff), path);

  // Decode format.
  decodefmt(buff, fmt);

  strpath[0] = '\0';

  // Decode type.
  char *p = strstr(buff, "://");
  if (p == NULL) {
    rsstrcpy(strpath, size, buff);
    *type = STR_FILE;
    return 1;
  }
  if (!strncmp(path, "serial", 6))
    *type = STR_SERIAL;
  else if (!strncmp(path, "tcpsvr", 6))
    *type = STR_TCPSVR;
  else if (!strncmp(path, "tcpcli", 6))
    *type = STR_TCPCLI;
  else if (!strncmp(path, "ntripcas", 8))
    *type = STR_NTRIPCAS;
  else if (!strncmp(path, "ntripsrc", 8))
    *type = STR_NTRIPSRC;
  else if (!strncmp(path, "ntripcli", 8))
    *type = STR_NTRIPCLI;
  else if (!strncmp(path, "file", 4))
    *type = STR_FILE;
  else if (!strncmp(path, "udpsvr", 6))
    *type = STR_UDPSVR;
  else if (!strncmp(path, "udpcli", 6))
    *type = STR_UDPCLI;
  else {
    fprintf(stderr, "stream path error: %s\n", buff);
    return 0;
  }
  rssubstrcpy(strpath, size, p, 3);
  return 1;
}
// Read receiver commands ----------------------------------------------------
static void readcmd(const char *file, char *cmd, size_t size, int type) {
  cmd[0] = '\0';

  FILE *fp = fopen(file, "r");
  if (!fp) return;

  int i = 0;
  int line_start = 0;
  size_t end = 0;
  char buff[81];
  while (fgets(buff, sizeof(buff), fp)) {
    size_t avail = strlen(buff);
    int line_end = avail > 0 && (buff[avail - 1] == '\n' || buff[avail - 1] == '\r');
    if (line_start && *buff == '@') {
      i++;
      // Flush to the end of this line.
      while (!line_end && fgets(buff, sizeof(buff), fp)) {
        avail = strlen(buff);
        line_end = avail > 0 && (buff[avail - 1] == '\n' || buff[avail - 1] == '\r');
      }
      line_start = line_end;
      continue;
    }
    line_start = line_end;
    if (i != type) continue;
    size_t req = end + avail + 1;
    if (req > size) {
      // Fill as much as possible and exit.
      memcpy(cmd + end, buff, size - end);
      cmd[size - 1] = '\0';
      fprintf(stderr, "Stream command size overflow in file %s\n", file);
      break;
    }
    memcpy(cmd + end, buff, avail);
    end += avail;
    cmd[end] = '\0';
  }
  fclose(fp);
}

#ifndef WIN32
static void daemonise(void) {
  // In case we were not started in the background, fork and let the parent
  // exit.  Guarantees that the child is not a process group leader.
  int childpid = fork();
  if (childpid < 0) {
    perror("fork\n");
    _exit(1);
  } else if (childpid > 0) {
    // Parent.
    _exit(0);
  }

  // Make ourselves the leader of a new process group with no controlling
  // terminal.
  if (setsid() < 0) {
    perror("setsid\n");
    _exit(1);
  }

  for (int fd = 0; fd < 10; fd++) close(fd);

  open("/dev/null", O_RDWR);
  gtime_t time = utc2gpst(timeget());
  char path[1024];
  reppath(LOGFILE, path, time, "", "");
  open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  dup(1);
}
#endif

/* str2str -------------------------------------------------------------------*/
int main(int argc, char **argv) {
  static char s1[MAXSTRSVR][MAXSTRPATH] = {{0}};
  static char cmd_strs[MAXSTRSVR][MAXRCVCMD];
  static char cmd_periodic_strs[MAXSTRSVR][MAXRCVCMD];
  char *cmds[MAXSTRSVR], *cmds_periodic[MAXSTRSVR];
  char *cmdfile[MAXSTRSVR];
  char *paths[MAXSTRSVR];
  unsigned types[MAXSTRSVR];
  const char *logs[MAXSTRSVR];
  const char *msg = "1004,1019";  // Current messages.
  const char *msgs[MAXSTRSVR];       // Messages per output stream.
  int stat[MAXSTRSVR], log_stat[MAXSTRSVR], fmts[MAXSTRSVR];

  for (unsigned i = 0; i < MAXSTRSVR; i++) {
    paths[i] = s1[i];
    paths[i][0] = '\0';
    logs[i] = "";
    cmds[i] = cmd_strs[i];
    cmds[i][0] = '\0';
    cmds_periodic[i] = cmd_periodic_strs[i];
    cmds_periodic[i][0] = '\0';
    cmdfile[i] = "";
    types[i] = STR_FILE;
    msgs[i] = msg;
    stat[i] = log_stat[i] = fmts[i] = 0;
  }

  double stapos[3] = {0}, stadel[3] = {0};
  const char *local = "", *proxy = "", *opt = "";
  const char *antinfo = "", *rcvinfo = "";
  char *ant[] = {"", "", ""}, *rcv[] = {"", "", ""}, *logfile = "";
  int dispint = 5000, trlevel = 0, sta = 0, daemon = 0;
  int opts[] = {10000, 10000, 2000, 32768, 10, 0, 30, 0};
  const char *log = "";           // Log for the next input or output stream.
  const char *tlssvrcertfile = "";  // TLS server certificate file.
  const char *tlssvrkeyfile = "";   // TLS server certificate private key file.
  const char *tlssvrcafile = "";    // TLS server CA file.
  const char *tlssvrcadir = "";     // TLS server CA directory.
  unsigned tlssvrverify = 0;        // TLS server peer verify.
  const char *tlsclicertfile = "";  // TLS client certificate file.
  const char *tlsclikeyfile = "";   // TLS client certificate private key file.
  const char *tlsclicafile = "";    // TLS client CA file.
  const char *tlsclicadir = "";     // TLS client CA directory.
  unsigned tlscliverify = 1;        // TLS client peer verify.
  unsigned n = 0;                   // The number of output streams.

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-in") && i + 1 < argc) {
      if (!decodepath(argv[++i], types, paths[0], sizeof(s1[0]), fmts)) return EXIT_FAILURE;
      // Use the previous specified log file.
      logs[0] = log;
      log = "";
    } else if (!strcmp(argv[i], "-msg") && i + 1 < argc)
      msg = argv[++i];
    else if (!strcmp(argv[i], "-out") && i + 1 < argc && n < MAXSTRSVR - 1) {
      if (!decodepath(argv[++i], types + n + 1, paths[n + 1], sizeof(s1[0]), fmts + n + 1)) return EXIT_FAILURE;
      // Capture the current messages for this output stream.
      msgs[n + 1] = msg;
      // Use the previous specified log file.
      logs[n + 1] = log;
      log = "";
      n++;
    } else if (!strcmp(argv[i], "-log") && i + 1 < argc) {
      if (log != NULL && *log) fprintf(stderr, "Warning log '%s' ignored.\n", log);
      log = argv[++i];
    } else if (!strcmp(argv[i], "-p") && i + 3 < argc) {
      double pos[3];
      pos[0] = atof(argv[++i]) * D2R;
      pos[1] = atof(argv[++i]) * D2R;
      pos[2] = atof(argv[++i]);
      pos2ecef(pos, stapos);
    } else if (!strcmp(argv[i], "-px") && i + 3 < argc) {
      stapos[0] = atof(argv[++i]);
      stapos[1] = atof(argv[++i]);
      stapos[2] = atof(argv[++i]);
    } else if (!strcmp(argv[i], "-o") && i + 3 < argc) {
      stadel[0] = atof(argv[++i]);
      stadel[1] = atof(argv[++i]);
      stadel[2] = atof(argv[++i]);
    } else if (!strcmp(argv[i], "-opt") && i + 1 < argc)
      opt = argv[++i];
    else if (!strcmp(argv[i], "-sta") && i + 1 < argc)
      sta = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-d") && i + 1 < argc)
      dispint = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-s") && i + 1 < argc)
      opts[0] = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-r") && i + 1 < argc)
      opts[1] = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-n") && i + 1 < argc)
      opts[5] = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-f") && i + 1 < argc)
      opts[6] = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-c") && i + 1 < argc)
      cmdfile[0] = argv[++i];
    else if (!strcmp(argv[i], "-c1") && i + 1 < argc)
      cmdfile[1] = argv[++i];
    else if (!strcmp(argv[i], "-c2") && i + 1 < argc)
      cmdfile[2] = argv[++i];
    else if (!strcmp(argv[i], "-c3") && i + 1 < argc)
      cmdfile[3] = argv[++i];
    else if (!strcmp(argv[i], "-c4") && i + 1 < argc)
      cmdfile[4] = argv[++i];
    else if (!strcmp(argv[i], "-a") && i + 1 < argc)
      antinfo = argv[++i];
    else if (!strcmp(argv[i], "-i") && i + 1 < argc)
      rcvinfo = argv[++i];
    else if (!strcmp(argv[i], "-l") && i + 1 < argc)
      local = argv[++i];
    else if (!strcmp(argv[i], "-x") && i + 1 < argc)
      proxy = argv[++i];
    else if (!strcmp(argv[i], "-b") && i + 1 < argc)
      opts[7] = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-fl") && i + 1 < argc)
      logfile = argv[++i];
    else if (!strcmp(argv[i], "-t") && i + 1 < argc)
      trlevel = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tlssvrcert") && i + 1 < argc)
      tlssvrcertfile = argv[++i];
    else if (!strcmp(argv[i], "--tlssvrkey") && i + 1 < argc)
      tlssvrkeyfile = argv[++i];
    else if (!strcmp(argv[i], "--tlssvrcafile") && i + 1 < argc)
      tlssvrcafile = argv[++i];
    else if (!strcmp(argv[i], "--tlssvrcadir") && i + 1 < argc)
      tlssvrcadir = argv[++i];
    else if (!strcmp(argv[i], "--tlssvrverify") && i + 1 < argc)
      tlssvrverify = strtoul(argv[++i], NULL, 10);
    else if (!strcmp(argv[i], "--tlsclicert") && i + 1 < argc)
      tlsclicertfile = argv[++i];
    else if (!strcmp(argv[i], "--tlsclicafile") && i + 1 < argc)
      tlsclicafile = argv[++i];
    else if (!strcmp(argv[i], "--tlsclicadir") && i + 1 < argc)
      tlsclicadir = argv[++i];
    else if (!strcmp(argv[i], "--tlsclikey") && i + 1 < argc)
      tlsclikeyfile = argv[++i];
    else if (!strcmp(argv[i], "--tlscliverify") && i + 1 < argc)
      tlscliverify = strtoul(argv[++i], NULL, 10);
    else if (!strcmp(argv[i], "--daemon"))
      daemon = 1;
    else if (!strcmp(argv[i], "--deamon"))
      daemon = 1;
    else if (!strcmp(argv[i], "--version")) {
      fprintf(stderr, "str2str RTKLIB %s %s\n", VER_RTKLIB, PATCH_LEVEL);
      exit(0);
    } else if (*argv[i] == '-') {
      if (n >= MAXSTRSVR - 1) {
        fprintf(stderr, "Error too many streams, the limit is %u.\n", MAXSTRSVR);
        return EXIT_FAILURE;
      }
      printhelp();
    }
  }
  if (n == 0) {
    n = 1;  // stdout
    logs[1] = log;
    log = "";
  }
  if (log != NULL && *log) fprintf(stderr, "Warning log '%s' ignored.\n", log);

  strconv_t *conv[MAXSTRSVR] = {NULL};
  char buff[256];
  for (unsigned i = 0; i < n; i++) {
    if (fmts[i + 1] <= 0) continue;
    if (fmts[i + 1] != STRFMT_RTCM3) {
      fprintf(stderr, "unsupported output format\n");
      return EXIT_FAILURE;
    }
    if (fmts[0] < 0) {
      fprintf(stderr, "specify input format\n");
      return EXIT_FAILURE;
    }
    conv[i] = strconvnew(fmts[0], fmts[i + 1], msgs[i + 1], sta, sta != 0, opt);
    if (conv[i] == NULL) {
      fprintf(stderr, "stream conversion error\n");
      return EXIT_FAILURE;
    }
    rsstrcpy(buff, sizeof(buff), antinfo);
    char *r, *p = strtok_r(buff, ",", &r);
    for (unsigned j = 0; p && j < 3; p = strtok_r(NULL, ",", &r)) ant[j++] = p;
    rsstrcpy(conv[i]->out.sta.antdes, sizeof(conv[0]->out.sta.antdes), ant[0]);
    rsstrcpy(conv[i]->out.sta.antsno, sizeof(conv[0]->out.sta.antsno), ant[1]);
    conv[i]->out.sta.antsetup = atoi(ant[2]);
    rsstrcpy(buff, sizeof(buff), rcvinfo);
    p = strtok_r(buff, ",", &r);
    for (unsigned j = 0; p && j < 3; p = strtok_r(NULL, ",", &r)) rcv[j++] = p;
    rsstrcpy(conv[i]->out.sta.rectype, sizeof(conv[i]->out.sta.rectype), rcv[0]);
    rsstrcpy(conv[i]->out.sta.recver, sizeof(conv[i]->out.sta.recver), rcv[1]);
    rsstrcpy(conv[i]->out.sta.recsno, sizeof(conv[i]->out.sta.recsno), rcv[2]);
    matcpy(conv[i]->out.sta.pos, stapos, 3, 1);
    matcpy(conv[i]->out.sta.del, stadel, 3, 1);
  }
#ifndef WIN32
  if (daemon) daemonise();
  signal(SIGHUP, SIG_IGN);
  signal(SIGPIPE, SIG_IGN);
#endif
  signal(SIGTERM, sigfunc);
  signal(SIGINT, sigfunc);

  // Stream server
  strsvr_t *strsvr = (strsvr_t *)malloc(sizeof(strsvr_t));
  if (!strsvr) return EXIT_FAILURE;

  strsvrinit(strsvr, n);

  if (trlevel > 0) {
    traceopen(*logfile ? logfile : TRACEFILE);
    tracelevel(trlevel);
  }
  fprintf(stderr, "stream server start\n");

  strsetdir(local);
  strsetproxy(proxy);

  for (unsigned i = 0; i < MAXSTRSVR; i++) {
    if (*cmdfile[i]) readcmd(cmdfile[i], cmds[i], sizeof(cmd_strs[0]), 0);
    if (*cmdfile[i]) readcmd(cmdfile[i], cmds_periodic[i], sizeof(cmd_periodic_strs[0]), 2);
  }
  // Initialize the TLS certificates.
  strinittls(tlssvrcertfile, tlssvrkeyfile, tlssvrcafile, tlssvrcadir, tlssvrverify, tlsclicertfile,
             tlsclikeyfile, tlsclicafile, tlsclicadir, tlscliverify);
  // Start stream server.
  if (!strsvrstart(strsvr, opts, types, (const char **)paths, (const char **)logs, conv,
                   (const char **)cmds, (const char **)cmds_periodic, stapos)) {
    free(strsvr);
    fprintf(stderr, "stream server start error\n");
    return EXIT_FAILURE;
  }
  const char ss[] = {'E', '-', 'W', 'C', 'C'};
  for (intrflg = 0; !intrflg;) {
    // Get stream server status.
    size_t byte[MAXSTRSVR] = {0};
    unsigned bps[MAXSTRSVR] = {0};
    char strmsg[MAXSTRMSG] = "";
    strsvrstat(strsvr, stat, log_stat, byte, bps, strmsg, sizeof(strmsg));

    // Show stream server status.
    buff[0] = '\0';
    for (unsigned i = 0; i < MAXSTRSVR; i++) rscatprintf(buff, sizeof(buff), "%c", ss[stat[i] + 1]);

    char tstr[40];
    time2str(utc2gpst(timeget()), tstr, 0);
    fprintf(stderr, "%s [%s] %10zu B %7u bps %s\n", tstr, buff, byte[0], bps[0], strmsg);

    sleepms(dispint);
  }
  for (unsigned i = 0; i < MAXSTRSVR; i++) {
    if (*cmdfile[i]) readcmd(cmdfile[i], cmds[i], sizeof(cmd_strs[0]), 1);
  }
  // Stop stream server.
  strsvrstop(strsvr, (const char **)cmds);

  for (unsigned i = 0; i < n; i++) strconvfree(conv[i]);
  if (trlevel > 0) traceclose();

  free(strsvr);
  fprintf(stderr, "stream server stop\n");
  return 0;
}
