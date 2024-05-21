/*------------------------------------------------------------------------------
 * stream.c : stream input/output functions
 *
 *          Copyright (C) 2008-2020 by T.TAKASU, All rights reserved.
 *
 * Options : -DWIN32    use WIN32 API
 *           -DSVR_REUSEADDR reuse TCP server address
 *
 * References :
 *     [1] RTCM Recommendaed Standards for Networked Transport for RTCM via
 *         Internet Protocol (NTRIP), Version 1.0, Semptember 30, 2004
 *     [2] H.Niksic and others, GNU Wget 1.12, The non-iteractive download
 *         utility, 4 September 2009
 *     [3] RTCM Recommendaed Standards for Networked Transport for RTCM via
 *         Internet Protocol (NTRIP), Version 2.0, June 28, 2011
 *
 * Version : $Revision:$ $Date:$
 * History : 2009/01/16 1.0  new
 *           2009/04/02 1.1  support NMEA request in NTRIP request
 *                           support time-tag of file as stream
 *           2009/09/04 1.2  ported to Linux environment
 *                           add fflush() to save file stream
 *           2009/10/10 1.3  support multiple connection for TCP server
 *                           add keyword replacement in file path
 *                           add function strsendnmea(), strsendcmd()
 *           2010/07/18 1.4  support FTP/HTTP stream types
 *                           add keywords replacement of %ha,%hb,%hc in path
 *                           add api: strsetdir(),strsettimeout()
 *           2010/08/31 1.5  reconnect after error of NTRIP client
 *                           fix bug on no file swap at week start (2.4.0_p6)
 *           2011/05/29 1.6  add fast stream replay mode
 *                           add time margin to swap file
 *                           change api strsetopt()
 *                           introduce non_block send for send socket
 *                           add api: strsetproxy()
 *           2011/12/21 1.7  fix bug decode tcppath (rtklib_2.4.1_p5)
 *           2012/06/09 1.8  fix problem if user or password contains /
 *                           (rtklib_2.4.1_p7)
 *           2012/12/25 1.9  compile option SVR_REUSEADDR added
 *           2013/03/10 1.10 fix problem with NTRIP mountpoint containing "/"
 *           2013/04/15 1.11 fix bug on swapping files if swapmargin=0
 *           2013/05/28 1.12 fix bug on playback of file with 64 bit size_t
 *           2014/05/23 1.13 retry to connect after gethostbyname() error
 *                           fix bug on malloc size in openftp()
 *           2014/06/21 1.14 add general hex message rcv command by !HEX ...
 *           2014/10/16 1.15 support stdin/stdout for input/output from/to file
 *           2014/11/08 1.16 fix getconfig error (87) with bluetooth device
 *           2015/01/12 1.15 add rcv command to change bitrate by !BRATE
 *           2016/01/16 1.16 add constant CRTSCTS for non-CRTSCTS-defined env.
 *                           fix serial status for non-windows systems
 *           2016/06/09 1.17 fix bug on !BRATE rcv command always failed
 *                           fix program on struct alignment in time tag header
 *           2016/06/21 1.18 reverse time-tag handler of file to previous
 *           2016/07/23 1.19 add output of received stream to TCP port for serial
 *           2016/08/20 1.20 modify api strsendnmea()
 *           2016/08/29 1.21 fix bug on starting serial thread for windows
 *           2016/09/03 1.22 add NTRIP caster functions
 *                           add api strstatx(),strsetsrctbl()
 *                           add api strsetsel(),strgetsel()
 *           2016/09/06 1.23 fix bug on NTRIP caster socket and request handling
 *           2016/09/27 1.24 support UDP server and client
 *           2016/10/10 1.25 support ::P={4|8} option in path for STR_FILE
 *           2018/11/05 1.26 fix bug on default playback speed (= 0)
 *                           fix bug on file playback as slave mode
 *                           fix bug on timeset() in GPST instead of UTC
 *                           update trace levels and buffer sizes
 *           2019/05/10 1.27 fix bug on dropping message on TCP stream (#144)
 *           2019/08/19 1.28 support 460800 and 921600 bps for serial
 *           2020/11/30 1.29 delete API strsetsrctbl(), strsetsel(), strgetsel()
 *                           fix bug on numerical error in computing output rate
 *                           no support stream type STR_NTRIPC_S in API stropen()
 *                           no support rcv. command LEXR in API strsendcmd()
 *                           change stream type STR_NTRIPC_C to STR_NTRIPCAS
 *                           accept HTTP/1.1 as protocol for NTRIP caster
 *                           suppress warning for buffer overflow by sprintf()
 *                           use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include <ctype.h>

#include "rtklib.h"
#ifndef WIN32
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#define __USE_MISC
#ifndef CRTSCTS
#define CRTSCTS 020000000000
#endif
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <termios.h>
#endif

/* Constants -----------------------------------------------------------------*/

#define TINTACT 200              /* Period for stream active (ms) */
#define SERIBUFFSIZE 4096        /* Serial buffer size (bytes) */
#define TIMETAGH_LEN 64          /* Time tag file header length */
#define MAXCLI 32                /* Max client connection for TCP svr */
#define MAXSTATMSG 32            /* Max length of status message */
#define DEFAULT_MEMBUF_SIZE 4096 /* Default memory buffer size (bytes) */

#define NTRIP_AGENT "RTKLIB/" VER_RTKLIB "_" PATCH_LEVEL
#define NTRIP_CLI_PORT 2101                       /* Default ntrip-client connection port */
#define NTRIP_SVR_PORT 80                         /* Default ntrip-server connection port */
#define NTRIP_MAXRSP 32768                        /* Max size of NTRIP response */
#define NTRIP_MAXSTR 256                          /* Max length of mountpoint string */
#define NTRIP_RSP_OK_CLI "ICY 200 OK\r\n"         /* NTRIP response: client */
#define NTRIP_RSP_OK_SVR "OK\r\n"                 /* NTRIP response: server */
#define NTRIP_RSP_SRCTBL "SOURCETABLE 200 OK\r\n" /* NTRIP response: source table */
#define NTRIP_RSP_TBLEND "ENDSOURCETABLE"
#define NTRIP_RSP_HTTP "HTTP/"  /* NTRIP response: HTTP */
#define NTRIP_RSP_ERROR "ERROR" /* NTRIP response: error */
#define NTRIP_RSP_UNAUTH "HTTP/1.0 401 Unauthorized\r\n"
#define NTRIP_RSP_ERR_PWD "ERROR - Bad Pasword\r\n"
#define NTRIP_RSP_ERR_MNTP "ERROR - Bad Mountpoint\r\n"

#define FTP_CMD "wget" /* FTP/HTTP command */
#define FTP_TIMEOUT 30 /* FTP/HTTP timeout (s) */

#define MIN(x, y) ((x) < (y) ? (x) : (y))

/* Macros --------------------------------------------------------------------*/

#ifdef WIN32
#define dev_t HANDLE
#define socket_t SOCKET
typedef int socklen_t;
#else
#define dev_t int
#define socket_t int
#define closesocket close
#endif

/* Type definition -----------------------------------------------------------*/

typedef struct {             /* File control type */
  FILE *fp;                  /* File pointer */
  FILE *fp_tag;              /* File pointer of tag file */
  FILE *fp_tmp;              /* Temporary file pointer for swap */
  FILE *fp_tag_tmp;          /* Temporary file pointer of tag file for swap */
  char path[MAXSTRPATH];     /* File path */
  char openpath[MAXSTRPATH]; /* Open file path */
  int mode;                  /* File mode */
  int timetag;               /* Time tag flag (0:off,1:on) */
  int repmode;               /* Replay mode (0:master,1:slave) */
  int offset;                /* Time offset (ms) for slave */
  int size_fpos;             /* File position size (bytes) */
  gtime_t time;              /* Start time */
  gtime_t wtime;             /* Write time */
  uint32_t tick;             /* Start tick */
  uint32_t tick_f;           /* Start tick in file */
  long fpos_n;               /* Next file position */
  uint32_t tick_n;           /* Next tick */
  long double start;         /* Start offset (s) */
  long double speed;         /* Replay speed (time factor) */
  long double swapintv;      /* Swap interval (hr) (0: no swap) */
  rtklib_lock_t lock;        /* Lock flag */
} file_t;

typedef struct {           /* TCP control type */
  int state;               /* State (0:close,1:wait,2:connect) */
  char saddr[256];         /* Address string */
  int port;                /* Port */
  struct sockaddr_in addr; /* Address resolved */
  socket_t sock;           /* Socket descriptor */
  int tcon;                /* Reconnect time (ms) (-1:never,0:now) */
  uint32_t tact;           /* Data active tick */
  uint32_t tdis;           /* Disconnect tick */
} tcp_t;

typedef struct tcpsvr_tag { /* TCP server type */
  tcp_t svr;                /* TCP server control */
  tcp_t cli[MAXCLI];        /* TCP client controls */
} tcpsvr_t;

typedef struct { /* TCP cilent type */
  tcp_t svr;     /* TCP server control */
  int toinact;   /* Inactive timeout (ms) (0:no timeout) */
  int tirecon;   /* Reconnect interval (ms) (0:no reconnect) */
} tcpcli_t;

typedef struct { /* Serial control type */
  dev_t dev;     /* Serial device */
  int error;     /* Error state */
#ifdef WIN32
  int state, wp, rp;  /* State,write/read pointer */
  int buffsize;       /* Write buffer size (bytes) */
  HANDLE thread;      /* Write thread */
  rtklib_lock_t lock; /* Lock flag */
  uint8_t *buff;      /* Write buffer */
#endif
  tcpsvr_t *tcpsvr; /* TCP server for received stream */
} serial_t;

typedef struct {              /* NTRIP control type */
  int state;                  /* State (0:close,1:wait,2:connect) */
  int type;                   /* Type (0:server,1:client) */
  int nb;                     /* Response buffer size */
  char url[MAXSTRPATH];       /* Url for proxy */
  char mntpnt[256];           /* Mountpoint */
  char user[256];             /* User */
  char passwd[256];           /* Password */
  char str[NTRIP_MAXSTR];     /* Mountpoint string for server */
  uint8_t buff[NTRIP_MAXRSP]; /* Response buffer */
  tcpcli_t *tcp;              /* TCP client */
} ntrip_t;

typedef struct {              /* NTRIP client/server connection type */
  int state;                  /* State (0:close,1:connect) */
  char mntpnt[256];           /* Mountpoint */
  char str[NTRIP_MAXSTR];     /* Mountpoint string for server */
  int nb;                     /* Request buffer size */
  uint8_t buff[NTRIP_MAXRSP]; /* Request buffer */
} ntripc_con_t;

typedef struct {             /* NTRIP caster control type */
  int state;                 /* State (0:close,1:wait,2:connect) */
  int type;                  /* Type (0:server,1:client) */
  char mntpnt[256];          /* Mountpoint */
  char user[256];            /* User */
  char passwd[256];          /* Password */
  char srctbl[NTRIP_MAXSTR]; /* Source table */
  tcpsvr_t *tcp;             /* TCP server */
  ntripc_con_t con[MAXCLI];  /* NTRIP client/server connections */
} ntripc_t;

typedef struct {           /* UDP type */
  int state;               /* State (0:close,1:open) */
  int type;                /* Type (0:server,1:client) */
  int port;                /* Port */
  char saddr[256];         /* Address (server:filter,client:server) */
  struct sockaddr_in addr; /* Address resolved */
  socket_t sock;           /* Socket descriptor */
} udp_t;

typedef struct {          /* FTP download control type */
  int state;              /* State (0:close,1:download,2:complete,3:error) */
  int proto;              /* Protocol (0:ftp,1:http) */
  int error;              /* Error code (0:no error,1-10:wget error, */
                          /*            11:no temp dir,12:uncompact error) */
  char addr[FNSIZE];      /* Download address */
  char file[FNSIZE];      /* Download file path */
  char user[256];         /* User for FTP */
  char passwd[256];       /* Password for FTP */
  char local[FNSIZE];     /* Local file path */
  int topts[4];           /* Time options {poff,tint,toff,tretry} (s) */
  gtime_t tnext;          /* Next retry time (GPST) */
  rtklib_thread_t thread; /* Download thread */
} ftp_t;

typedef struct {      /* Memory buffer type */
  int state, wp, rp;  /* State,write/read pointer */
  int bufsize;        /* Buffer size (bytes) */
  rtklib_lock_t lock; /* Lock flag */
  uint8_t *buf;       /* Write buffer */
} membuf_t;

/* Proto types for static functions ------------------------------------------*/

static tcpsvr_t *opentcpsvr(const char *path, char *msg, size_t msize);
static void closetcpsvr(tcpsvr_t *tcpsvr);
static int writetcpsvr(tcpsvr_t *tcpsvr, uint8_t *buff, int n, char *msg, size_t msize);

/* Global options ------------------------------------------------------------*/

static int toinact = 10000;        /* Inactive timeout (ms) */
static int ticonnect = 10000;      /* Interval to re-connect (ms) */
static int tirate = 1000;          /* Averaging time for data rate (ms) */
static int buffsize = 32768;       /* Receive/send buffer size (bytes) */
static char localdir[FNSIZE] = ""; /* Local directory for FTP/HTTP */
static char proxyaddr[256] = "";   /* HTTP/NTRIP/FTP proxy address */
static uint32_t tick_master = 0;   /* Time tick master for replay */
static int fswapmargin = 30;       /* File swap margin (s) */

/* Read/write serial buffer --------------------------------------------------*/
#ifdef WIN32
static int readseribuff(serial_t *serial, uint8_t *buff, int nmax) {
  tracet(5, "readseribuff: dev=%d\n", serial->dev);

  rtklib_lock(&serial->lock);
  int ns;
  for (ns = 0; serial->rp != serial->wp && ns < nmax; ns++) {
    buff[ns] = serial->buff[serial->rp];
    if (++serial->rp >= serial->buffsize) serial->rp = 0;
  }
  rtklib_unlock(&serial->lock);
  tracet(5, "readseribuff: ns=%d rp=%d wp=%d\n", ns, serial->rp, serial->wp);
  return ns;
}
static int writeseribuff(serial_t *serial, uint8_t *buff, int n) {
  tracet(5, "writeseribuff: dev=%d n=%d\n", serial->dev, n);

  rtklib_lock(&serial->lock);
  int ns;
  for (ns = 0; ns < n; ns++) {
    int wp = serial->wp;
    serial->buff[wp] = buff[ns];
    if (++wp >= serial->buffsize) wp = 0;
    if (wp != serial->rp)
      serial->wp = wp;
    else {
      tracet(2, "serial buffer overflow: size=%d\n", serial->buffsize);
      break;
    }
  }
  rtklib_unlock(&serial->lock);
  tracet(5, "writeseribuff: ns=%d rp=%d wp=%d\n", ns, serial->rp, serial->wp);
  return ns;
}
#endif /* WIN32 */

/* Write serial thread -------------------------------------------------------*/
#ifdef WIN32
static DWORD WINAPI serialthread(void *arg) {
  tracet(3, "serialthread:\n");

  serial_t *serial = (serial_t *)arg;

  for (;;) {
    uint32_t tick = tickget();
    int n;
    uint8_t buff[128];
    while ((n = readseribuff(serial, buff, sizeof(buff))) > 0) {
      DWORD ns;
      if (!WriteFile(serial->dev, buff, n, &ns, NULL)) serial->error = 1;
    }
    if (!serial->state) break;
    sleepms(10 - (int)(tickget() - tick)); /* cycle=10ms */
  }
  free(serial->buff);
  return 0;
}
#endif /* WIN32 */

/* Open serial ---------------------------------------------------------------*/
static serial_t *openserial(const char *path, int mode, char *msg, size_t msize) {
  const int br[] = {300,   600,   1200,   2400,   4800,   9600,  19200,
                    38400, 57600, 115200, 230400, 460800, 921600};
#ifdef WIN32
  DWORD error, rw = 0, siz = sizeof(COMMCONFIG);
  COMMCONFIG cc = {0};
  COMMTIMEOUTS co = {MAXDWORD, 0, 0, 0, 0}; /* Non-block-read */
  char dcb[64] = "";
#else
#ifdef __APPLE__
  /* MacOS doesn't support higher baudrates (>230400B) */
  const speed_t bs[] = {B300,   B600,   B1200,  B2400,   B4800,  B9600,
                        B19200, B38400, B57600, B115200, B230400};
#else  /* Regular Linux with higher baudrates */
  const speed_t bs[] = {B300,   B600,   B1200,   B2400,   B4800,   B9600,  B19200,
                        B38400, B57600, B115200, B230400, B460800, B921600};
#endif /* Ifdef __APPLE__ */
  struct termios ios = {0};
  int rw = 0;
#endif
  tracet(3, "openserial: path=%s mode=%d\n", path, mode);

  serial_t *serial = (serial_t *)calloc(1, sizeof(serial_t));
  if (!serial) return NULL;

  char port[128];
  char parity = 'N';
  int brate = 115200, bsize = 8, stopb = 1;
  char fctr[64] = "";
  const char *p = strchr(path, ':');
  if (p) {
    size_t end = p - path;
    rtkesubstrcpy(port, sizeof(port), path, 0, end);
    sscanf(p, ":%d:%d:%c:%d:%63s", &brate, &bsize, &parity, &stopb, fctr);
  } else
    rtkstrcpy(port, sizeof(port), path);

  int tcp_port = 0;
  p = strchr(path, '#');
  if (p) sscanf(p, "#%d", &tcp_port);
  int i;
  for (i = 0; i < 13; i++)
    if (br[i] == brate) break;
  if (i >= 13) {
    rtkcatprintf(msg, msize, "bitrate error (%d)", brate);
    tracet(1, "openserial: %s path=%s\n", msg, path);
    free(serial);
    return NULL;
  }
  parity = (char)toupper((int)parity);

  char dev[128];
#ifdef WIN32
  rtksnprintf(dev, sizeof(dev), "\\\\.\\%s", port);
  if (mode & STR_MODE_R) rw |= GENERIC_READ;
  if (mode & STR_MODE_W) rw |= GENERIC_WRITE;

  serial->dev = CreateFile(dev, rw, 0, 0, OPEN_EXISTING, 0, NULL);
  if (serial->dev == INVALID_HANDLE_VALUE) {
    rtkcatprintf(msg, msize, "%s open error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s path=%s\n", msg, path);
    free(serial);
    return NULL;
  }
  if (!GetCommConfig(serial->dev, &cc, &siz)) {
    rtkcatprintf(msg, msize, "%s getconfig error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s\n", msg);
    CloseHandle(serial->dev);
    free(serial);
    return NULL;
  }
  rtksnprintf(dcb, sizeof(dcb), "baud=%d parity=%c data=%d stop=%d", brate, parity, bsize, stopb);
  if (!BuildCommDCB(dcb, &cc.dcb)) {
    rtkcatprintf(msg, msize, "%s buiddcb error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s\n", msg);
    CloseHandle(serial->dev);
    free(serial);
    return NULL;
  }
  if (!strcmp(fctr, "rts")) {
    cc.dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
  }
  SetCommConfig(serial->dev, &cc, siz); /* Ignore error to support novatel */
  SetCommTimeouts(serial->dev, &co);
  ClearCommError(serial->dev, &error, NULL);
  PurgeComm(serial->dev, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

  /* Create write thread */
  rtklib_initlock(&serial->lock);
  serial->state = serial->wp = serial->rp = serial->error = 0;
  serial->buffsize = buffsize;
  if (!(serial->buff = (uint8_t *)malloc(buffsize))) {
    CloseHandle(serial->dev);
    free(serial);
    return NULL;
  }
  serial->state = 1;
  if (!(serial->thread = CreateThread(NULL, 0, serialthread, serial, 0, NULL))) {
    rtkcatprintf(msg, msize, "%s serial thread error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s\n", msg);
    CloseHandle(serial->dev);
    serial->state = 0;
    free(serial);
    return NULL;
  }
  rtkcatprintf(msg, msize, "%s", port);
#else
  rtksnprintf(dev, sizeof(dev), "/dev/%.*s", (int)sizeof(port) - 6, port);

  if ((mode & STR_MODE_R) && (mode & STR_MODE_W))
    rw = O_RDWR;
  else if (mode & STR_MODE_R)
    rw = O_RDONLY;
  else if (mode & STR_MODE_W)
    rw = O_WRONLY;

  if ((serial->dev = open(dev, rw | O_NOCTTY | O_NONBLOCK)) < 0) {
    rtkcatprintf(msg, msize, "%s open error (%d)", dev, errno);
    tracet(1, "openserial: %s dev=%s\n", msg, dev);
    free(serial);
    return NULL;
  }
  tcgetattr(serial->dev, &ios);
  ios.c_iflag = 0;
  ios.c_oflag = 0;
  ios.c_lflag = 0;    /* Non-canonical */
  ios.c_cc[VMIN] = 0; /* Non-block-mode */
  ios.c_cc[VTIME] = 0;
  cfsetospeed(&ios, bs[i]);
  cfsetispeed(&ios, bs[i]);
  ios.c_cflag |= bsize == 7 ? CS7 : CS8;
  ios.c_cflag |= parity == 'O' ? (PARENB | PARODD) : (parity == 'E' ? PARENB : 0);
  ios.c_cflag |= stopb == 2 ? CSTOPB : 0;
  ios.c_cflag |= !strcmp(fctr, "rts") ? CRTSCTS : 0;
  tcsetattr(serial->dev, TCSANOW, &ios);
  tcflush(serial->dev, TCIOFLUSH);
  rtkcatprintf(msg, msize, "%s", dev);
#endif
  serial->tcpsvr = NULL;

  /* Open TCP sever to output received stream */
  if (tcp_port > 0) {
    char path_tcp[32];
    rtksnprintf(path_tcp, sizeof(path_tcp), ":%d", tcp_port);
    char msg_tcp[128];
    serial->tcpsvr = opentcpsvr(path_tcp, msg_tcp, sizeof(msg_tcp));
  }
  tracet(3, "openserial: dev=%d\n", serial->dev);
  return serial;
}
/* Close serial --------------------------------------------------------------*/
static void closeserial(serial_t *serial) {
  tracet(3, "closeserial: dev=%d\n", serial->dev);

  if (!serial) return;
#ifdef WIN32
  serial->state = 0;
  WaitForSingleObject(serial->thread, 10000);
  CloseHandle(serial->dev);
  CloseHandle(serial->thread);
#else
  close(serial->dev);
#endif
  if (serial->tcpsvr) {
    closetcpsvr(serial->tcpsvr);
  }
  free(serial);
}
/* Read serial ---------------------------------------------------------------*/
static int readserial(serial_t *serial, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readserial: dev=%d n=%d\n", serial->dev, n);
  if (!serial) return 0;
#ifdef WIN32
  DWORD nr;
  if (!ReadFile(serial->dev, buff, n, &nr, NULL)) return 0;
#else
  int nr = read(serial->dev, buff, n);
  if (nr < 0) return 0;
#endif
  tracet(5, "readserial: exit dev=%d nr=%d\n", serial->dev, nr);

  /* Write received stream to TCP server port */
  if (serial->tcpsvr && nr > 0) {
    /* TODO handle no-blocking write ? */
    char msg_tcp[128];
    writetcpsvr(serial->tcpsvr, buff, (int)nr, msg_tcp, sizeof(msg_tcp));
  }
  return nr;
}
/* Write serial --------------------------------------------------------------*/
static int writeserial(serial_t *serial, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(3, "writeserial: dev=%d n=%d\n", serial->dev, n);

  if (!serial) return 0;
#ifdef WIN32
  int ns = writeseribuff(serial, buff, n) if (ns < n) serial->error = 1;
#else
  int ns = write(serial->dev, buff, n);
  if (ns < 0) {
    if (errno == EAGAIN) {
      /* TODO ?? */
    }
    ns = 0;
    serial->error = 1;
  }
#endif
  tracet(5, "writeserial: exit dev=%d ns=%d\n", serial->dev, ns);
  return ns;
}
/* Get state serial ----------------------------------------------------------*/
static int stateserial(serial_t *serial) { return !serial ? 0 : (serial->error ? -1 : 2); }
/* Get extended state serial -------------------------------------------------*/
static int statexserial(serial_t *serial, char *msg, size_t msize) {
  int state = !serial ? 0 : (serial->error ? -1 : 2);

  rtkcatprintf(msg, msize, "serial:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  if (!state) return 0;
  rtkcatprintf(msg, msize, "  dev     = %d\n", (int)serial->dev);
  rtkcatprintf(msg, msize, "  error   = %d\n", serial->error);
#ifdef WIN32
  rtkcatprintf(msg, msize, "  buffsize= %d\n", serial->buffsize);
  rtkcatprintf(msg, msize, "  wp      = %d\n", serial->wp);
  rtkcatprintf(msg, msize, "  rp      = %d\n", serial->rp);
#endif
  return state;
}
/* Open file -----------------------------------------------------------------*/
static bool openfile_(file_t *file, gtime_t time, char *msg, size_t msize) {
  char tstr[40];
  tracet(3, "openfile_: path=%s time=%s\n", file->path, time2str(time, tstr, 0));

  file->time = utc2gpst(timeget());
  file->tick = file->tick_f = tickget();
  file->fpos_n = 0;
  file->tick_n = 0;

  /* Use stdin or stdout if file path is null */
  if (!*file->path) {
    file->fp = file->mode & STR_MODE_R ? stdin : stdout;
    return true;
  }
  /* Replace keywords */
  reppath(file->path, file->openpath, sizeof(file->openpath), time, "", "");

  /* Create directory */
  if ((file->mode & STR_MODE_W) && !(file->mode & STR_MODE_R)) {
    createdir(file->openpath);
  }
  char *rw;
  if (file->mode & STR_MODE_R)
    rw = "rb";
  else
    rw = "wb";

  if (!(file->fp = fopen(file->openpath, rw))) {
    rtkcatprintf(msg, msize, "file open error: %s", file->openpath);
    tracet(1, "openfile: %s\n", msg);
    return false;
  }
  tracet(4, "openfile_: open file %s (%s)\n", file->openpath, rw);

  char tagpath[MAXSTRPATH + 4] = "";
  rtksnprintf(tagpath, sizeof(tagpath), "%s.tag", file->openpath);

  if (file->timetag) { /* Output/sync time-tag */

    if (!(file->fp_tag = fopen(tagpath, rw))) {
      rtkcatprintf(msg, msize, "tag open error: %s", tagpath);
      tracet(1, "openfile: %s\n", msg);
      fclose(file->fp);
      return false;
    }
    tracet(4, "openfile_: open tag file %s (%s)\n", tagpath, rw);

    if (file->mode & STR_MODE_R) {
      char tagh[TIMETAGH_LEN + 1] = "";
      long double time_sec;
      uint32_t time_time;
      if (fread(&tagh, TIMETAGH_LEN, 1, file->fp_tag) == 1 &&
          fread(&time_time, sizeof(time_time), 1, file->fp_tag) == 1 &&
          fread(&time_sec, sizeof(time_sec), 1, file->fp_tag) == 1) {
        memcpy(&file->tick_f, tagh + TIMETAGH_LEN - 4, sizeof(file->tick_f));
        file->time.time = (time_t)time_time;
        file->time.sec = time_sec;
        file->wtime = file->time;
      } else {
        file->tick_f = 0;
      }
      /* Adust time to read playback file */
      timeset(gpst2utc(file->time));
    } else {
      char tagh[TIMETAGH_LEN + 1] = "";
      long double time_sec;
      uint32_t time_time;
      rtksnprintf(tagh, sizeof(tagh), "TIMETAG RTKLIB %s", VER_RTKLIB);
      memcpy(tagh + TIMETAGH_LEN - 4, &file->tick_f, sizeof(file->tick_f));
      time_time = (uint32_t)file->time.time;
      time_sec = file->time.sec;
      fwrite(&tagh, 1, TIMETAGH_LEN, file->fp_tag);
      fwrite(&time_time, 1, sizeof(time_time), file->fp_tag);
      fwrite(&time_sec, 1, sizeof(time_sec), file->fp_tag);
      /* Time tag file structure   */
      /*   HEADER(60)+TICK(4)+TIME(4+8)+ */
      /*   TICK0(4)+FPOS0(4/8)+    */
      /*   TICK1(4)+FPOS1(4/8)+... */
    }
  } else if (file->mode & STR_MODE_W) { /* Remove time-tag */
    FILE *fp = fopen(tagpath, "rb");
    if (fp) {
      fclose(fp);
      remove(tagpath);
    }
  }
  return true;
}
/* Close file ----------------------------------------------------------------*/
static void closefile_(file_t *file) {
  tracet(3, "closefile_: path=%s\n", file->path);

  if (file->fp) fclose(file->fp);
  if (file->fp_tag) fclose(file->fp_tag);
  if (file->fp_tmp) fclose(file->fp_tmp);
  if (file->fp_tag_tmp) fclose(file->fp_tag_tmp);
  file->fp = file->fp_tag = file->fp_tmp = file->fp_tag_tmp = NULL;

  /* Reset time offset */
  timereset();
}
/* Open file (path=filepath[::T[::+<off>][::x<speed>]][::S=swapintv][::P={4|8}] */
static file_t *openfile(const char *path, int mode, char *msg, size_t msize) {
  tracet(3, "openfile: path=%s mode=%d\n", path, mode);

  if (!(mode & (STR_MODE_R | STR_MODE_W))) return NULL;

  /* File options */
  long double speed = 1.0L, start = 0.0L, swapintv = 0.0L;
  int timetag = 0, size_fpos = 4;                                  /* Default 4B */
  for (int pi = 0; (pi = strstri(path, pi, "::")) >= 0; pi += 2) { /* File options */
    if (path[pi + 2] == 'T')
      timetag = 1;
    else if (path[pi + 2] == '+')
      sscanf(path + pi + 2, "+%Lf", &start);
    else if (path[pi + 2] == 'x')
      sscanf(path + pi + 2, "x%Lf", &speed);
    else if (path[pi + 2] == 'S')
      sscanf(path + pi + 2, "S=%Lf", &swapintv);
    else if (path[pi + 2] == 'P')
      sscanf(path + pi + 2, "P=%d", &size_fpos);
  }
  if (start <= 0.0L) start = 0.0L;
  if (swapintv <= 0.0L) swapintv = 0.0L;

  file_t *file = (file_t *)malloc(sizeof(file_t));
  if (!file) return NULL;

  file->fp = file->fp_tag = file->fp_tmp = file->fp_tag_tmp = NULL;
  rtkstrcpy(file->path, sizeof(file->path), path);
  int pi = strstri(file->path, 0, "::");
  if (pi >= 0) file->path[pi] = '\0';
  file->openpath[0] = '\0';
  file->mode = mode;
  file->timetag = timetag;
  file->repmode = 0;
  file->offset = 0;
  file->size_fpos = size_fpos;
  gtime_t time0 = {0};
  file->time = file->wtime = time0;
  file->tick = file->tick_f = file->tick_n = file->fpos_n = 0;
  file->start = start;
  file->speed = speed;
  file->swapintv = swapintv;
  rtklib_initlock(&file->lock);

  gtime_t time = utc2gpst(timeget());

  /* Open new file */
  if (!openfile_(file, time, msg, msize)) {
    free(file);
    return NULL;
  }
  return file;
}
/* Close file ----------------------------------------------------------------*/
static void closefile(file_t *file) {
  tracet(3, "closefile: fp=%d\n", file->fp);

  if (!file) return;
  closefile_(file);
  free(file);
}
/* Open new swap file --------------------------------------------------------*/
static void swapfile(file_t *file, gtime_t time, char *msg, size_t msize) {
  char tstr[40];
  tracet(3, "swapfile: fp=%d time=%s\n", file->fp, time2str(time, tstr, 0));

  /* Return if old swap file open */
  if (file->fp_tmp || file->fp_tag_tmp) return;

  /* Check path of new swap file */
  char openpath[MAXSTRPATH];
  reppath(file->path, openpath, sizeof(openpath), time, "", "");

  if (!strcmp(openpath, file->openpath)) {
    tracet(2, "swapfile: no need to swap %s\n", openpath);
    return;
  }
  /* Save file pointer to temporary pointer */
  file->fp_tmp = file->fp;
  file->fp_tag_tmp = file->fp_tag;

  /* Open new swap file */
  openfile_(file, time, msg, msize);
}
/* Close old swap file -------------------------------------------------------*/
static void swapclose(file_t *file) {
  tracet(3, "swapclose: fp_tmp=%d\n", file->fp_tmp);

  if (file->fp_tmp) fclose(file->fp_tmp);
  if (file->fp_tag_tmp) fclose(file->fp_tag_tmp);
  file->fp_tmp = file->fp_tag_tmp = NULL;
}
/* Get state file ------------------------------------------------------------*/
static int statefile(const file_t *file) { return file ? 2 : 0; }
/* Get extended state file ---------------------------------------------------*/
static int statexfile(const file_t *file, char *msg, size_t msize) {
  char tstr1[40], tstr2[40];
  int state = file ? 2 : 0;

  rtkcatprintf(msg, msize, "file:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  if (!state) return 0;
  time2str(file->time, tstr1, 3);
  time2str(file->wtime, tstr2, 3);
  rtkcatprintf(msg, msize, "  path    = %s\n", file->path);
  rtkcatprintf(msg, msize, "  openpath= %s\n", file->openpath);
  rtkcatprintf(msg, msize, "  mode    = %d\n", file->mode);
  rtkcatprintf(msg, msize, "  timetag = %d\n", file->timetag);
  rtkcatprintf(msg, msize, "  repmode = %d\n", file->repmode);
  rtkcatprintf(msg, msize, "  offsete = %d\n", file->offset);
  rtkcatprintf(msg, msize, "  time    = %s\n", tstr1);
  rtkcatprintf(msg, msize, "  wtime   = %s\n", tstr2);
  rtkcatprintf(msg, msize, "  tick    = %u\n", file->tick);
  rtkcatprintf(msg, msize, "  tick_f  = %u\n", file->tick_f);
  rtkcatprintf(msg, msize, "  start   = %.3Lf\n", file->start);
  rtkcatprintf(msg, msize, "  speed   = %.3Lf\n", file->speed);
  rtkcatprintf(msg, msize, "  swapintv= %.3Lf\n", file->swapintv);
  return state;
}
/* Read file -----------------------------------------------------------------*/
static int readfile(file_t *file, uint8_t *buff, int nmax, char *msg, size_t msize) {
  tracet(4, "readfile: fp=%d nmax=%d\n", file->fp, nmax);

  if (!file) return 0;

  if (file->fp == stdin) {
#ifndef WIN32
    /* Input from stdin */
    fd_set rs;
    FD_ZERO(&rs);
    FD_SET(0, &rs);
    struct timeval tv = {0};
    if (!select(1, &rs, NULL, NULL, &tv)) return 0;
    int nr = (int)read(0, buff, nmax);
    if (nr < 0) return 0;
    return nr;
#else
    return 0;
#endif
  }
  if (file->fp_tag) {
    uint32_t t, tick;
    /* Target tick */
    if (file->repmode) { /* Slave */
      t = (uint32_t)(tick_master + file->offset);
    } else { /* Master */
      t = (uint32_t)((tickget() - file->tick) * file->speed + file->start * 1000.0L);
      tick_master = t;
    }
    /* Seek time-tag file to get next tick and file position */
    while ((int)(file->tick_n - t) <= 0) {
      uint32_t fpos_4B;
      uint64_t fpos_8B;
      if (fread(&file->tick_n, sizeof(tick), 1, file->fp_tag) < 1 ||
          fread((file->size_fpos == 4) ? (void *)&fpos_4B : (void *)&fpos_8B, file->size_fpos, 1,
                file->fp_tag) < 1) {
        file->tick_n = (uint32_t)(-1);
        long pos = ftell(file->fp);
        fseek(file->fp, 0L, SEEK_END);
        file->fpos_n = ftell(file->fp);
        fseek(file->fp, pos, SEEK_SET);
        break;
      }
      file->fpos_n = (long)((file->size_fpos == 4) ? fpos_4B : fpos_8B);
    }
    if (file->tick_n == (uint32_t)(-1)) {
      rtksnprintf(msg, msize, "end");
    } else {
      rtksnprintf(msg, msize, "T%+.1Lfs", (int)t * 0.001L);
      file->wtime = timeadd(file->time, (int)t * 0.001L);
      timeset(timeadd(gpst2utc(file->time), (int)file->tick_n * 0.001L));
    }
    long n = file->fpos_n - ftell(file->fp);
    if (n < nmax) {
      nmax = n;
    }
  }
  int nr = 0;
  if (nmax > 0) {
    nr = (int)fread(buff, 1, nmax, file->fp);
  }
  if (feof(file->fp)) {
    rtksnprintf(msg, msize, "end");
  }
  tracet(5, "readfile: fp=%d nr=%d\n", file->fp, nr);
  return nr;
}
/* Write file ----------------------------------------------------------------*/
static int writefile(file_t *file, const uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "writefile: fp=%d n=%d\n", file->fp, n);

  if (!file) return 0;

  gtime_t wtime = utc2gpst(timeget()); /* Write time in GPST */

  /* Swap writing file */
  if (file->swapintv > 0.0L && file->wtime.time != 0) {
    long double intv = file->swapintv * 3600.0L;
    int week1;
    long double tow1 = time2gpst(file->wtime, &week1);
    int week2;
    long double tow2 = time2gpst(wtime, &week2);
    tow2 += 604800.0L * (week2 - week1);

    /* Open new swap file */
    if (floorl((tow1 + fswapmargin) / intv) < floorl((tow2 + fswapmargin) / intv)) {
      swapfile(file, timeadd(wtime, fswapmargin), msg, msize);
    }
    /* Close old swap file */
    if (floorl((tow1 - fswapmargin) / intv) < floorl((tow2 - fswapmargin) / intv)) {
      swapclose(file);
    }
  }
  if (!file->fp) return 0;

  int ns = (int)fwrite(buff, 1, n, file->fp);
  long fpos = ftell(file->fp);
  fflush(file->fp);
  file->wtime = wtime;

  long fpos_tmp = 0;
  if (file->fp_tmp) {
    fwrite(buff, 1, n, file->fp_tmp);
    fpos_tmp = ftell(file->fp_tmp);
    fflush(file->fp_tmp);
  }
  uint32_t tick = tickget();
  if (file->fp_tag) {
    tick -= file->tick;
    fwrite(&tick, 1, sizeof(tick), file->fp_tag);
    uint32_t fpos_4B;
    uint64_t fpos_8B;
    if (file->size_fpos == 4) {
      fpos_4B = (uint32_t)fpos;
      fwrite(&fpos_4B, 1, sizeof(fpos_4B), file->fp_tag);
    } else {
      fpos_8B = (uint64_t)fpos;
      fwrite(&fpos_8B, 1, sizeof(fpos_8B), file->fp_tag);
    }
    fflush(file->fp_tag);

    if (file->fp_tag_tmp) {
      fwrite(&tick, 1, sizeof(tick), file->fp_tag_tmp);
      if (file->size_fpos == 4) {
        fpos_4B = (uint32_t)fpos_tmp;
        fwrite(&fpos_4B, 1, sizeof(fpos_4B), file->fp_tag_tmp);
      } else {
        fpos_8B = (uint64_t)fpos_tmp;
        fwrite(&fpos_8B, 1, sizeof(fpos_8B), file->fp_tag_tmp);
      }
      fflush(file->fp_tag_tmp);
    }
  }
  tracet(5, "writefile: fp=%d ns=%d tick=%5d fpos=%d\n", file->fp, ns, tick, fpos);

  return ns;
}
/* Sync files by time-tag ----------------------------------------------------*/
static void syncfile(file_t *file1, file_t *file2) {
  if (!file1->fp_tag || !file2->fp_tag) return;
  file1->repmode = 0;
  file2->repmode = 1;
  file2->offset = (int)(file1->tick_f - file2->tick_f);
}
/* Decode TCP/NTRIP path (path=[user[:passwd]@]addr[:port][/mntpnt[:str]]) ---*/
static void decodetcppath(const char *path, char addr[256], char port[256], char user[256],
                          char passwd[256], char mntpnt[256], char str[NTRIP_MAXSTR]) {
  tracet(4, "decodetcpepath: path=%s\n", path);

  if (port) *port = '\0';
  if (user) *user = '\0';
  if (passwd) *passwd = '\0';
  if (mntpnt) *mntpnt = '\0';
  if (str) *str = '\0';

  char buff[MAXSTRPATH];
  rtkstrcpy(buff, sizeof(buff), path);

  int pi = strrchri(buff, 0, '@');
  if (pi < 0) pi = 0;

  pi = strchri(buff, pi, '/');
  if (pi >= 0) {
    int qi = strchri(buff, pi + 1, ':');
    if (qi >= 0) {
      buff[qi] = '\0';
      if (str) rtksnprintf(str, NTRIP_MAXSTR, "%.*s", NTRIP_MAXSTR - 1, buff + qi + 1);
    }
    buff[pi] = '\0';
    if (mntpnt) rtksnprintf(mntpnt, 256, "%.255s", buff + pi + 1);
  }
  pi = strrchri(buff, 0, '@');
  if (pi >= 0) {
    buff[pi++] = '\0';
    int qi = strchri(buff, 0, ':');
    if (qi >= 0) {
      buff[qi] = '\0';
      if (passwd) rtksnprintf(passwd, 256, "%.255s", buff + qi + 1);
    }
    if (user) rtksnprintf(user, 256, "%.255s", buff);
  } else
    pi = 0;

  int qi = strchri(buff + pi, 0, ':');
  if (qi >= 0) {
    buff[pi + qi] = '\0';
    if (port) rtksnprintf(port, 256, "%.255s", buff + pi + qi + 1);
  }
  if (addr) rtksnprintf(addr, 256, "%.255s", buff + pi);
}
/* Get socket error ----------------------------------------------------------*/
#ifdef WIN32
static int errsock(void) { return WSAGetLastError(); }
#else
static int errsock(void) { return errno; }
#endif

/* Set socket option ---------------------------------------------------------*/
static bool setsock(socket_t sock, char *msg, size_t msize) {
  tracet(3, "setsock: sock=%d\n", sock);

#ifdef WIN32
  int tv = 0;
#else
  struct timeval tv = {0};
#endif
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)) == -1 ||
      setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv)) == -1) {
    rtkcatprintf(msg, msize, "sockopt error: notimeo");
    tracet(1, "setsock: setsockopt error 1 sock=%d err=%d\n", sock, errsock());
    closesocket(sock);
    return false;
  }
  int bs = buffsize;
  if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (const char *)&bs, sizeof(bs)) == -1 ||
      setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (const char *)&bs, sizeof(bs)) == -1) {
    tracet(1, "setsock: setsockopt error 2 sock=%d err=%d bs=%d\n", sock, errsock(), bs);
    rtkcatprintf(msg, msize, "sockopt error: bufsiz");
  }
  int mode = 1;
  if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&mode, sizeof(mode)) == -1) {
    tracet(1, "setsock: setsockopt error 3 sock=%d err=%d\n", sock, errsock());
    rtkcatprintf(msg, msize, "sockopt error: nodelay");
  }
  return true;
}
/* Non-block accept ----------------------------------------------------------*/
static socket_t accept_nb(socket_t sock, struct sockaddr *addr, socklen_t *len) {
  fd_set rs;
  FD_ZERO(&rs);
  FD_SET(sock, &rs);
  struct timeval tv = {0};
  int ret = select(sock + 1, &rs, NULL, NULL, &tv);
  if (ret <= 0) return (socket_t)ret;
  return accept(sock, addr, len);
}
/* Non-block connect ---------------------------------------------------------*/
static int connect_nb(socket_t sock, struct sockaddr *addr, socklen_t len) {
#ifdef WIN32
  u_long mode = 1;
  ioctlsocket(sock, FIONBIO, &mode);
  if (connect(sock, addr, len) == -1) {
    int err = errsock();
    if (err == WSAEWOULDBLOCK || err == WSAEINPROGRESS || err == WSAEALREADY || err == WSAEINVAL)
      return 0;
    if (err != WSAEISCONN) return -1;
  }
#else
  int flag = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flag | O_NONBLOCK);
  if (connect(sock, addr, len) == -1) {
    int err = errsock();
    if (err != EISCONN && err != EINPROGRESS && err != EALREADY) return -1;
    fd_set rs, ws;
    FD_ZERO(&rs);
    FD_SET(sock, &rs);
    ws = rs;
    struct timeval tv = {0};
    if (select(sock + 1, &rs, &ws, NULL, &tv) == 0) return 0;
  }
#endif
  return 1;
}
/* Non-block receive ---------------------------------------------------------*/
static int recv_nb(socket_t sock, uint8_t *buff, int n) {
  fd_set rs;
  FD_ZERO(&rs);
  FD_SET(sock, &rs);
  struct timeval tv = {0};
  int ret = select(sock + 1, &rs, NULL, NULL, &tv);
  if (ret <= 0) return ret;
  int nr = recv(sock, (char *)buff, n, 0);
  return nr <= 0 ? -1 : nr;
}
/* Non-block send ------------------------------------------------------------*/
static int send_nb(socket_t sock, uint8_t *buff, int n) {
  fd_set ws;
  FD_ZERO(&ws);
  FD_SET(sock, &ws);
  struct timeval tv = {0};
  int ret = select(sock + 1, NULL, &ws, NULL, &tv);
  if (ret <= 0) return ret;
  int ns = send(sock, (char *)buff, n, 0);
  return ns < n ? -1 : ns;
}
/* Generate TCP socket -------------------------------------------------------*/
static bool gentcp(tcp_t *tcp, int type, char *msg, size_t msize) {
  tracet(3, "gentcp: type=%d\n", type);

  /* Generate socket */
  if ((tcp->sock = socket(AF_INET, SOCK_STREAM, 0)) == (socket_t)-1) {
    rtkcatprintf(msg, msize, "socket error (%d)", errsock());
    tracet(1, "gentcp: socket error err=%d\n", errsock());
    tcp->state = -1;
    return false;
  }
  if (!setsock(tcp->sock, msg, msize)) {
    tcp->state = -1;
    return false;
  }
  memset(&tcp->addr, 0, sizeof(tcp->addr));
  tcp->addr.sin_family = AF_INET;
  tcp->addr.sin_port = htons(tcp->port);

  if (type == 0) { /* Server socket */

#ifdef SVR_REUSEADDR
    /* Multiple-use of server socket */
    int opt = 1;
    setsockopt(tcp->sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));
#endif
    if (bind(tcp->sock, (struct sockaddr *)&tcp->addr, sizeof(tcp->addr)) == -1) {
      rtkcatprintf(msg, msize, "bind error (%d) : %d", errsock(), tcp->port);
      tracet(1, "gentcp: bind error port=%d err=%d\n", tcp->port, errsock());
      closesocket(tcp->sock);
      tcp->state = -1;
      return false;
    }
    listen(tcp->sock, 5);
  } else { /* Client socket */
    struct hostent *hp = gethostbyname(tcp->saddr);
    if (!hp) {
      rtkcatprintf(msg, msize, "address error (%s)", tcp->saddr);
      tracet(1, "gentcp: gethostbyname error addr=%s err=%d\n", tcp->saddr, errsock());
      closesocket(tcp->sock);
      tcp->state = 0;
      tcp->tcon = ticonnect;
      tcp->tdis = tickget();
      return false;
    }
    memcpy(&tcp->addr.sin_addr, hp->h_addr, hp->h_length);
  }
  tcp->state = 1;
  tcp->tact = tickget();
  tracet(5, "gentcp: exit sock=%d\n", tcp->sock);
  return true;
}
/* Disconnect TCP ------------------------------------------------------------*/
static void discontcp(tcp_t *tcp, int tcon) {
  tracet(3, "discontcp: sock=%d tcon=%d\n", tcp->sock, tcon);

  closesocket(tcp->sock);
  tcp->state = 0;
  tcp->tcon = tcon;
  tcp->tdis = tickget();
}
/* Open TCP server -----------------------------------------------------------*/
static tcpsvr_t *opentcpsvr(const char *path, char *msg, size_t msize) {
  tracet(3, "opentcpsvr: path=%s\n", path);

  tcpsvr_t *tcpsvr = (tcpsvr_t *)malloc(sizeof(tcpsvr_t));
  if (!tcpsvr) return NULL;
  tcpsvr_t tcpsvr0 = {{0}};
  *tcpsvr = tcpsvr0;
  char port[256] = "";
  decodetcppath(path, tcpsvr->svr.saddr, port, NULL, NULL, NULL, NULL);
  if (sscanf(port, "%d", &tcpsvr->svr.port) < 1) {
    rtkcatprintf(msg, msize, "port error: %s", port);
    tracet(1, "opentcpsvr: port error port=%s\n", port);
    free(tcpsvr);
    return NULL;
  }
  if (!gentcp(&tcpsvr->svr, 0, msg, msize)) {
    free(tcpsvr);
    return NULL;
  }
  tcpsvr->svr.tcon = 0;
  return tcpsvr;
}
/* Close TCP server ----------------------------------------------------------*/
static void closetcpsvr(tcpsvr_t *tcpsvr) {
  tracet(3, "closetcpsvr:\n");

  for (int i = 0; i < MAXCLI; i++) {
    if (tcpsvr->cli[i].state) closesocket(tcpsvr->cli[i].sock);
  }
  closesocket(tcpsvr->svr.sock);
  free(tcpsvr);
}
/* Update TCP server ---------------------------------------------------------*/
static void updatetcpsvr(tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  tracet(4, "updatetcpsvr: state=%d\n", tcpsvr->svr.state);

  if (tcpsvr->svr.state == 0) return;

  char saddr[256] = "";
  int n = 0;
  for (int i = 0; i < MAXCLI; i++) {
    if (!tcpsvr->cli[i].state) continue;
    rtkstrcpy(saddr, sizeof(saddr), tcpsvr->cli[i].saddr);
    n++;
  }
  if (n == 0) {
    tcpsvr->svr.state = 1;
    rtksnprintf(msg, msize, "waiting...");
    return;
  }
  tcpsvr->svr.state = 2;
  if (n == 1)
    rtksnprintf(msg, msize, "%s", saddr);
  else
    rtksnprintf(msg, msize, "%d clients", n);
}
/* Accept client connection --------------------------------------------------*/
static bool accsock(tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  tracet(4, "accsock: sock=%d\n", tcpsvr->svr.sock);

  int i;
  for (i = 0; i < MAXCLI; i++) {
    if (tcpsvr->cli[i].state == 0) break;
  }
  if (i >= MAXCLI) {
    tracet(2, "accsock: too many clients sock=%d\n", tcpsvr->svr.sock);
    return false;
  }
  struct sockaddr_in addr;
  socklen_t len = sizeof(addr);
  socket_t sock = accept_nb(tcpsvr->svr.sock, (struct sockaddr *)&addr, &len);
  if (sock == (socket_t)-1) {
    int err = errsock();
    rtksnprintf(msg, msize, "accept error (%d)", err);
    tracet(1, "accsock: accept error sock=%d err=%d\n", tcpsvr->svr.sock, err);
    closesocket(tcpsvr->svr.sock);
    tcpsvr->svr.state = 0;
    return false;
  }
  if (sock == 0) return false;
  if (!setsock(sock, msg, msize)) return false;

  tcpsvr->cli[i].sock = sock;
  memcpy(&tcpsvr->cli[i].addr, &addr, sizeof(addr));
  rtkstrcpy(tcpsvr->cli[i].saddr, sizeof(tcpsvr->cli[0].saddr), inet_ntoa(addr.sin_addr));
  rtksnprintf(msg, msize, "%s", tcpsvr->cli[i].saddr);
  tracet(3, "accsock: connected sock=%d addr=%s i=%d\n", tcpsvr->cli[i].sock, tcpsvr->cli[i].saddr,
         i);
  tcpsvr->cli[i].state = 2;
  tcpsvr->cli[i].tact = tickget();
  return true;
}
/* Wait socket accept --------------------------------------------------------*/
static bool waittcpsvr(tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  tracet(4, "waittcpsvr: sock=%d state=%d\n", tcpsvr->svr.sock, tcpsvr->svr.state);

  if (tcpsvr->svr.state <= 0) return false;

  while (accsock(tcpsvr, msg, msize))
    ;

  updatetcpsvr(tcpsvr, msg, msize);
  return tcpsvr->svr.state == 2;
}
/* Read TCP server -----------------------------------------------------------*/
static int readtcpsvr(tcpsvr_t *tcpsvr, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readtcpsvr: state=%d\n", tcpsvr->svr.state);

  if (!waittcpsvr(tcpsvr, msg, msize)) return 0;

  for (int i = 0; i < MAXCLI; i++) {
    if (tcpsvr->cli[i].state != 2) continue;

    int nr = recv_nb(tcpsvr->cli[i].sock, buff, n);
    if (nr == -1) {
      int err = errsock();
      if (err) {
        tracet(2, "readtcpsvr: recv error sock=%d err=%d\n", tcpsvr->cli[i].sock, err);
      }
      discontcp(&tcpsvr->cli[i], ticonnect);
      updatetcpsvr(tcpsvr, msg, msize);
    }
    if (nr > 0) {
      tcpsvr->cli[i].tact = tickget();
      return nr;
    }
  }
  return 0;
}
/* Write TCP server ----------------------------------------------------------*/
static int writetcpsvr(tcpsvr_t *tcpsvr, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "writetcpsvr: state=%d n=%d\n", tcpsvr->svr.state, n);

  if (!waittcpsvr(tcpsvr, msg, msize)) return 0;

  int nmax = 0;
  for (int i = 0; i < MAXCLI; i++) {
    if (tcpsvr->cli[i].state != 2) continue;

    int ns = send_nb(tcpsvr->cli[i].sock, buff, n);
    if (ns == -1) {
      int err = errsock();
      if (err) {
        tracet(2, "writetcpsvr: send error i=%d sock=%d err=%d\n", i, tcpsvr->cli[i].sock, err);
      }
      discontcp(&tcpsvr->cli[i], ticonnect);
      updatetcpsvr(tcpsvr, msg, msize);
    } else {
      if (ns > nmax) nmax = ns;
      if (ns > 0) tcpsvr->cli[i].tact = tickget();
    }
  }
  return nmax;
}
/* Get state TCP server ------------------------------------------------------*/
static int statetcpsvr(tcpsvr_t *tcpsvr) { return tcpsvr ? tcpsvr->svr.state : 0; }
/* Print extended state TCP --------------------------------------------------*/
static void statextcp(const tcp_t *tcp, char *msg, size_t msize) {
  rtkcatprintf(msg, msize, "    state = %d\n", tcp->state);
  rtkcatprintf(msg, msize, "    saddr = %s\n", tcp->saddr);
  rtkcatprintf(msg, msize, "    port  = %d\n", tcp->port);
  rtkcatprintf(msg, msize, "    sock  = %d\n", (int)tcp->sock);
#ifdef RTK_DISABLED /* For debug */
  rtkcatprintf(msg, msize, "    tcon  = %d\n", tcp->tcon);
  rtkcatprintf(msg, msize, "    tact  = %u\n", tcp->tact);
  rtkcatprintf(msg, msize, "    tdis  = %u\n", tcp->tdis);
#endif
}
/* Get extended state TCP server ---------------------------------------------*/
static int statextcpsvr(const tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  int state = tcpsvr ? tcpsvr->svr.state : 0;

  rtkcatprintf(msg, msize, "tcpsvr:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  if (!state) return 0;
  rtkcatprintf(msg, msize, "  svr:\n");
  statextcp(&tcpsvr->svr, msg, msize);
  for (int i = 0; i < MAXCLI; i++) {
    if (!tcpsvr->cli[i].state) continue;
    rtkcatprintf(msg, msize, "  cli#%d:\n", i);
    statextcp(tcpsvr->cli + i, msg, msize);
  }
  return state;
}
/* Connect server ------------------------------------------------------------*/
static bool consock(tcpcli_t *tcpcli, char *msg, size_t msize) {
  tracet(4, "consock: sock=%d\n", tcpcli->svr.sock);

  /* Wait re-connect */
  if (tcpcli->svr.tcon < 0 ||
      (tcpcli->svr.tcon > 0 && (int)(tickget() - tcpcli->svr.tdis) < tcpcli->svr.tcon)) {
    return false;
  }
  /* Non-block connect */
  int stat =
      connect_nb(tcpcli->svr.sock, (struct sockaddr *)&tcpcli->svr.addr, sizeof(tcpcli->svr.addr));
  if (stat == -1) {
    int err = errsock();
    rtksnprintf(msg, msize, "connect error (%d)", err);
    tracet(2, "consock: connect error sock=%d err=%d\n", tcpcli->svr.sock, err);
    closesocket(tcpcli->svr.sock);
    tcpcli->svr.state = 0;
    return false;
  }
  if (!stat) { /* Not connect */
    rtksnprintf(msg, msize, "connecting...");
    return false;
  }
  rtksnprintf(msg, msize, "%s", tcpcli->svr.saddr);
  tracet(3, "consock: connected sock=%d addr=%s\n", tcpcli->svr.sock, tcpcli->svr.saddr);
  tcpcli->svr.state = 2;
  tcpcli->svr.tact = tickget();
  return true;
}
/* Open TCP client -----------------------------------------------------------*/
static tcpcli_t *opentcpcli(const char *path, char *msg, size_t msize) {
  tracet(3, "opentcpcli: path=%s\n", path);

  tcpcli_t *tcpcli = (tcpcli_t *)malloc(sizeof(tcpcli_t));
  if (!tcpcli) return NULL;
  tcpcli_t tcpcli0 = {{0}};
  *tcpcli = tcpcli0;
  char port[256] = "";
  decodetcppath(path, tcpcli->svr.saddr, port, NULL, NULL, NULL, NULL);
  if (sscanf(port, "%d", &tcpcli->svr.port) < 1) {
    rtkcatprintf(msg, msize, "port error: %s", port);
    tracet(2, "opentcp: port error port=%s\n", port);
    free(tcpcli);
    return NULL;
  }
  tcpcli->svr.tcon = 0;
  tcpcli->toinact = toinact;
  tcpcli->tirecon = ticonnect;
  return tcpcli;
}
/* Close TCP client ----------------------------------------------------------*/
static void closetcpcli(tcpcli_t *tcpcli) {
  tracet(3, "closetcpcli: sock=%d\n", tcpcli->svr.sock);

  closesocket(tcpcli->svr.sock);
  free(tcpcli);
}
/* Wait socket connect -------------------------------------------------------*/
static bool waittcpcli(tcpcli_t *tcpcli, char *msg, size_t msize) {
  tracet(4, "waittcpcli: sock=%d state=%d\n", tcpcli->svr.sock, tcpcli->svr.state);

  if (tcpcli->svr.state < 0) return false;

  if (tcpcli->svr.state == 0) { /* Close */
    if (!gentcp(&tcpcli->svr, 1, msg, msize)) return false;
  }
  if (tcpcli->svr.state == 1) { /* Wait */
    if (!consock(tcpcli, msg, msize)) return false;
  }
  if (tcpcli->svr.state == 2) { /* Connect */
    if (tcpcli->toinact > 0 && (int)(tickget() - tcpcli->svr.tact) > tcpcli->toinact) {
      rtksnprintf(msg, msize, "timeout");
      tracet(2, "waittcpcli: inactive timeout sock=%d\n", tcpcli->svr.sock);
      discontcp(&tcpcli->svr, tcpcli->tirecon);
      return false;
    }
  }
  return true;
}
/* Read TCP client -----------------------------------------------------------*/
static int readtcpcli(tcpcli_t *tcpcli, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readtcpcli: sock=%d\n", tcpcli->svr.sock);

  if (!waittcpcli(tcpcli, msg, msize)) return 0;

  int nr = recv_nb(tcpcli->svr.sock, buff, n);
  if (nr == -1) {
    int err = errsock();
    if (err) {
      tracet(2, "readtcpcli: recv error sock=%d err=%d\n", tcpcli->svr.sock, err);
      rtksnprintf(msg, msize, "recv error (%d)", err);
    } else {
      rtksnprintf(msg, msize, "disconnected");
    }
    discontcp(&tcpcli->svr, tcpcli->tirecon);
    return 0;
  }
  if (nr > 0) tcpcli->svr.tact = tickget();
  tracet(5, "readtcpcli: exit sock=%d nr=%d\n", tcpcli->svr.sock, nr);
  return nr;
}
/* Write TCP client ----------------------------------------------------------*/
static int writetcpcli(tcpcli_t *tcpcli, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(3, "writetcpcli: sock=%d state=%d n=%d\n", tcpcli->svr.sock, tcpcli->svr.state, n);

  if (!waittcpcli(tcpcli, msg, msize)) return 0;

  int ns = send_nb(tcpcli->svr.sock, buff, n);
  if (ns == -1) {
    int err = errsock();
    if (err) {
      tracet(2, "writetcp: send error sock=%d err=%d\n", tcpcli->svr.sock, err);
      rtksnprintf(msg, msize, "send error (%d)", err);
    }
    discontcp(&tcpcli->svr, tcpcli->tirecon);
    return 0;
  }
  if (ns > 0) tcpcli->svr.tact = tickget();
  tracet(5, "writetcpcli: exit sock=%d ns=%d\n", tcpcli->svr.sock, ns);
  return ns;
}
/* Get state TCP client ------------------------------------------------------*/
static int statetcpcli(tcpcli_t *tcpcli) { return tcpcli ? tcpcli->svr.state : 0; }
/* Get extended state TCP client ---------------------------------------------*/
static int statextcpcli(tcpcli_t *tcpcli, char *msg, size_t msize) {
  return tcpcli ? tcpcli->svr.state : 0;
}
/* Base64 encoder ------------------------------------------------------------*/
static void encbase64(char *str, size_t size, const uint8_t *byte, int n) {
  const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  tracet(4, "encbase64: n=%d\n", n);

  /* TODO respect size */
  int j = 0;
  for (int i = 0; i / 8 < n;) {
    int b = 0;
    for (int k = 0; k < 6; k++, i++) {
      b <<= 1;
      if (i / 8 < n) b |= (byte[i / 8] >> (7 - i % 8)) & 0x1;
    }
    RTKBOUNDSCHECK(str, size, j);
    str[j++] = table[b];
  }
  while (j & 0x3) {
    RTKBOUNDSCHECK(str, size, j);
    str[j++] = '=';
  }
  RTKBOUNDSCHECK(str, size, j);
  str[j] = '\0';
  tracet(5, "encbase64: str=%s\n", str);
}
/* Send NTRIP server request -------------------------------------------------*/
static bool reqntrip_s(ntrip_t *ntrip, char *msg, size_t msize) {
  tracet(3, "reqntrip_s: state=%d\n", ntrip->state);

  /* Reset the msg buffer */
  msg[0] = '\0';

  char buff[FNSIZE + NTRIP_MAXSTR];
  buff[0] = '\0';
  rtkcatprintf(buff, sizeof(buff), "SOURCE %s %s\r\n", ntrip->passwd, ntrip->mntpnt);
  rtkcatprintf(buff, sizeof(buff), "Source-Agent: NTRIP %s\r\n", NTRIP_AGENT);
  rtkcatprintf(buff, sizeof(buff), "STR: %s\r\n", ntrip->str);
  rtkcatprintf(buff, sizeof(buff), "\r\n");

  size_t len = strlen(buff);
  if (writetcpcli(ntrip->tcp, (uint8_t *)buff, len, msg, msize) != (int)len) return false;

  tracet(3, "reqntrip_s: send request state=%d ns=%d\n", ntrip->state, len);
  tracet(5, "reqntrip_s: n=%d buff=\n%s\n", len, buff);
  ntrip->state = 1;
  return true;
}
/* Send NTRIP client request -------------------------------------------------*/
static bool reqntrip_c(ntrip_t *ntrip, char *msg, size_t msize) {
  tracet(3, "reqntrip_c: state=%d\n", ntrip->state);

  /* Reset the msg buffer */
  msg[0] = '\0';

  char buff[MAXSTRPATH + FNSIZE];
  buff[0] = '\0';
  rtkcatprintf(buff, sizeof(buff), "GET %s/%s HTTP/1.0\r\n", ntrip->url, ntrip->mntpnt);
  rtkcatprintf(buff, sizeof(buff), "User-Agent: NTRIP %s\r\n", NTRIP_AGENT);

  if (!*ntrip->user) {
    rtkcatprintf(buff, sizeof(buff), "Accept: */*\r\n");
    rtkcatprintf(buff, sizeof(buff), "Connection: close\r\n");
  } else {
    char user[514];
    rtksnprintf(user, sizeof(user), "%s:%s", ntrip->user, ntrip->passwd);
    rtkcatprintf(buff, sizeof(buff), "Authorization: Basic ");
    size_t len = strlen(buff);
    encbase64(buff + len, sizeof(buff) - len, (uint8_t *)user, strlen(user));
    rtkcatprintf(buff, sizeof(buff), "\r\n");
  }
  rtkcatprintf(buff, sizeof(buff), "\r\n");

  size_t len = strlen(buff);
  if (writetcpcli(ntrip->tcp, (uint8_t *)buff, len, msg, msize) != (int)len) return false;

  tracet(3, "reqntrip_c: send request state=%d ns=%d\n", ntrip->state, len);
  tracet(5, "reqntrip_c: n=%d buff=\n%s\n", len, buff);
  ntrip->state = 1;
  return true;
}
/* Test NTRIP server response ------------------------------------------------*/
static bool rspntrip_s(ntrip_t *ntrip, char *msg, size_t msize) {
  tracet(3, "rspntrip_s: state=%d nb=%d\n", ntrip->state, ntrip->nb);
  ntrip->buff[ntrip->nb] = '0';
  tracet(5, "rspntrip_s: n=%d buff=\n%s\n", ntrip->nb, ntrip->buff);

  int pi = strstri((char *)ntrip->buff, 0, NTRIP_RSP_OK_SVR);
  if (pi >= 0) { /* Ok */
    pi += strlen(NTRIP_RSP_OK_SVR);
    ntrip->nb -= pi;
    for (int i = 0; i < ntrip->nb; i++) ((char *)ntrip->buff)[i] = ((char *)ntrip->buff)[pi + i];
    ntrip->state = 2;
    rtksnprintf(msg, msize, "%s/%s", ntrip->tcp->svr.saddr, ntrip->mntpnt);
    tracet(3, "rspntrip_s: response ok nb=%d\n", ntrip->nb);
    return true;
  }
  if (strstr((char *)ntrip->buff, NTRIP_RSP_ERROR)) { /* Error */
    int nb = ntrip->nb < MAXSTATMSG ? ntrip->nb : MAXSTATMSG;
    rtksnprintf(msg, msize, "%.*s", nb, (char *)ntrip->buff);
    int pi = strchri(msg, 0, '\r');
    if (pi >= 0) msg[pi] = '\0';
    tracet(3, "rspntrip_s: %s nb=%d\n", msg, ntrip->nb);
    ntrip->nb = 0;
    ntrip->buff[0] = '\0';
    ntrip->state = 0;
    discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
  } else if (ntrip->nb >= NTRIP_MAXRSP) { /* Buffer overflow */
    rtksnprintf(msg, msize, "response overflow");
    tracet(3, "rspntrip_s: response overflow nb=%d\n", ntrip->nb);
    ntrip->nb = 0;
    ntrip->buff[0] = '\0';
    ntrip->state = 0;
    discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
  }
  tracet(5, "rspntrip_s: exit state=%d nb=%d\n", ntrip->state, ntrip->nb);
  return false;
}
/* Test NTRIP client response ------------------------------------------------*/
static bool rspntrip_c(ntrip_t *ntrip, char *msg, size_t msize) {
  tracet(3, "rspntrip_c: state=%d nb=%d\n", ntrip->state, ntrip->nb);
  ntrip->buff[ntrip->nb] = '0';
  tracet(5, "rspntrip_c: n=%d buff=\n%s\n", ntrip->nb, ntrip->buff);

  int pi = strstri((char *)ntrip->buff, 0, NTRIP_RSP_OK_CLI);
  if (pi >= 0) { /* Ok */
    pi += strlen(NTRIP_RSP_OK_CLI);
    ntrip->nb -= pi;
    for (int i = 0; i < ntrip->nb; i++) ((char *)ntrip->buff)[i] = ((char *)ntrip->buff)[pi + i];
    ntrip->state = 2;
    rtksnprintf(msg, msize, "%s/%s", ntrip->tcp->svr.saddr, ntrip->mntpnt);
    tracet(3, "rspntrip_c: response ok nb=%d\n", ntrip->nb);
    ntrip->tcp->tirecon = ticonnect;
    return true;
  }
  if (strstr((char *)ntrip->buff, NTRIP_RSP_SRCTBL)) { /* Source table */
    if (!*ntrip->mntpnt) {                             /* Source table request */
      ntrip->state = 2;
      rtksnprintf(msg, msize, "source table received");
      tracet(3, "rspntrip_c: receive source table nb=%d\n", ntrip->nb);
      return true;
    }
    rtksnprintf(msg, msize, "no mountp. reconnect...");
    tracet(2, "rspntrip_c: no mount point nb=%d\n", ntrip->nb);
    ntrip->nb = 0;
    ntrip->buff[0] = '\0';
    ntrip->state = 0;
    /* Increase subsequent disconnect time to avoid too many reconnect requests */
    if (ntrip->tcp->tirecon > 300000) ntrip->tcp->tirecon = ntrip->tcp->tirecon * 5 / 4;

    discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
  } else if ((pi = strstri((char *)ntrip->buff, 0, NTRIP_RSP_HTTP)) >= 0) { /* HTTP response */
    int qi = strchri((char *)ntrip->buff, pi, '\r');
    if (qi >= 0)
      ntrip->buff[qi] = '\0';
    else
      ntrip->buff[128] = '\0';
    rtkstrcpy(msg, msize, (char *)ntrip->buff + pi);
    tracet(3, "rspntrip_s: %s nb=%d\n", msg, ntrip->nb);
    ntrip->nb = 0;
    ntrip->buff[0] = '\0';
    ntrip->state = 0;
    discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
  } else if (ntrip->nb >= NTRIP_MAXRSP) { /* Buffer overflow */
    rtksnprintf(msg, msize, "response overflow");
    tracet(2, "rspntrip_s: response overflow nb=%d\n", ntrip->nb);
    ntrip->nb = 0;
    ntrip->buff[0] = '\0';
    ntrip->state = 0;
    discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
  }
  tracet(5, "rspntrip_c: exit state=%d nb=%d\n", ntrip->state, ntrip->nb);
  return false;
}
/* Wait NTRIP request/response -----------------------------------------------*/
static bool waitntrip(ntrip_t *ntrip, char *msg, size_t msize) {
  tracet(4, "waitntrip: state=%d nb=%d\n", ntrip->state, ntrip->nb);

  if (ntrip->state < 0) return false; /* Error */

  if (ntrip->tcp->svr.state < 2) ntrip->state = 0; /* TCP disconnected */

  if (ntrip->state == 0) { /* Send request */
    if (!(ntrip->type == 0 ? reqntrip_s(ntrip, msg, msize) : reqntrip_c(ntrip, msg, msize))) {
      return false;
    }
    tracet(3, "waitntrip: state=%d nb=%d\n", ntrip->state, ntrip->nb);
  }
  if (ntrip->state == 1) { /* Read response */
    char *p = (char *)ntrip->buff + ntrip->nb;
    int n = readtcpcli(ntrip->tcp, (uint8_t *)p, NTRIP_MAXRSP - ntrip->nb - 1, msg, msize);
    if (n == 0) {
      tracet(5, "waitntrip: readtcp n=%d\n", n);
      return false;
    }
    ntrip->nb += n;
    ntrip->buff[ntrip->nb] = '\0';

    /* Wait response */
    return ntrip->type == 0 ? rspntrip_s(ntrip, msg, msize) : rspntrip_c(ntrip, msg, msize);
  }
  return true;
}
/* Open NTRIP ----------------------------------------------------------------*/
static ntrip_t *openntrip(const char *path, int type, char *msg, size_t msize) {
  tracet(3, "openntrip: path=%s type=%d\n", path, type);

  ntrip_t *ntrip = (ntrip_t *)malloc(sizeof(ntrip_t));
  if (!ntrip) return NULL;

  ntrip->state = 0;
  ntrip->type = type; /* 0:server,1:client */
  ntrip->nb = 0;
  ntrip->url[0] = '\0';
  ntrip->mntpnt[0] = ntrip->user[0] = ntrip->passwd[0] = ntrip->str[0] = '\0';
  for (int i = 0; i < NTRIP_MAXRSP; i++) ntrip->buff[i] = 0;

  /* Decode TCP/NTRIP path */
  char addr[256] = "", port[256] = "";
  decodetcppath(path, addr, port, ntrip->user, ntrip->passwd, ntrip->mntpnt, ntrip->str);

  /* Use default port if no port specified */
  if (!*port) {
    rtksnprintf(port, sizeof(port), "%d", type ? NTRIP_CLI_PORT : NTRIP_SVR_PORT);
  }
  char tpath[MAXSTRPATH];
  rtksnprintf(tpath, sizeof(tpath), "%s:%s", addr, port);

  /* NTRIP access via proxy server */
  if (*proxyaddr) {
    rtksnprintf(ntrip->url, sizeof(ntrip->url), "http://%.*s", MAXSTRPATH - 8, tpath);
    rtksnprintf(tpath, sizeof(tpath), "%.*s", MAXSTRPATH - 1, proxyaddr);
  }
  /* Open TCP client stream */
  if (!(ntrip->tcp = opentcpcli(tpath, msg, msize))) {
    tracet(2, "openntrip: opentcp error\n");
    free(ntrip);
    return NULL;
  }
  return ntrip;
}
/* Close NTRIP ---------------------------------------------------------------*/
static void closentrip(ntrip_t *ntrip) {
  tracet(3, "closentrip: state=%d\n", ntrip->state);

  closetcpcli(ntrip->tcp);
  free(ntrip);
}
/* Read NTRIP ----------------------------------------------------------------*/
static int readntrip(ntrip_t *ntrip, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readntrip:\n");

  if (!waitntrip(ntrip, msg, msize)) return 0;

  if (ntrip->nb > 0) { /* Read response buffer first */
    int nb = ntrip->nb <= n ? ntrip->nb : n;
    memcpy(buff, ntrip->buff + ntrip->nb - nb, nb);
    ntrip->nb = 0;
    return nb;
  }
  return readtcpcli(ntrip->tcp, buff, n, msg, msize);
}
/* Write NTRIP ---------------------------------------------------------------*/
static int writentrip(ntrip_t *ntrip, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(3, "writentrip: n=%d\n", n);

  if (!waitntrip(ntrip, msg, msize)) return 0;

  return writetcpcli(ntrip->tcp, buff, n, msg, msize);
}
/* Get state NTRIP -----------------------------------------------------------*/
static int statentrip(ntrip_t *ntrip) {
  return !ntrip ? 0 : (ntrip->state == 0 ? ntrip->tcp->svr.state : ntrip->state);
}
/* Get extended state NTRIP --------------------------------------------------*/
static int statexntrip(ntrip_t *ntrip, char *msg, size_t msize) {
  int state = !ntrip ? 0 : (ntrip->state == 0 ? ntrip->tcp->svr.state : ntrip->state);

  rtkcatprintf(msg, msize, "ntrip:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  if (!state) return 0;
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  rtkcatprintf(msg, msize, "  type    = %d\n", ntrip->type);
  rtkcatprintf(msg, msize, "  nb      = %d\n", ntrip->nb);
  rtkcatprintf(msg, msize, "  url     = %s\n", ntrip->url);
  rtkcatprintf(msg, msize, "  mntpnt  = %s\n", ntrip->mntpnt);
  rtkcatprintf(msg, msize, "  user    = %s\n", ntrip->user);
  rtkcatprintf(msg, msize, "  passwd  = %s\n", ntrip->passwd);
  rtkcatprintf(msg, msize, "  str     = %s\n", ntrip->str);
  rtkcatprintf(msg, msize, "  svr:\n");
  statextcp(&ntrip->tcp->svr, msg, msize);
  return state;
}
/* Open ntrip-caster ---------------------------------------------------------*/
static ntripc_t *openntripc(const char *path, char *msg, size_t msize) {
  tracet(3, "openntripc: path=%s\n", path);

  ntripc_t *ntripc = (ntripc_t *)malloc(sizeof(ntripc_t));
  if (!ntripc) return NULL;

  ntripc->state = 0;
  ntripc->mntpnt[0] = ntripc->user[0] = ntripc->passwd[0] = ntripc->srctbl[0] = '\0';
  for (int i = 0; i < MAXCLI; i++) {
    ntripc->con[i].state = 0;
    ntripc->con[i].nb = 0;
    memset(ntripc->con[i].buff, 0, NTRIP_MAXRSP);
  }
  /* Decode TCP/NTRIP path */
  char port[256] = "";
  decodetcppath(path, NULL, port, ntripc->user, ntripc->passwd, ntripc->mntpnt, ntripc->srctbl);

  if (!*ntripc->mntpnt) {
    tracet(2, "openntripc: no mountpoint path=%s\n", path);
    free(ntripc);
    return NULL;
  }
  /* Use default port if no port specified */
  if (!*port) {
    rtksnprintf(port, sizeof(port), "%d", NTRIP_CLI_PORT);
  }
  char tpath[MAXSTRPATH];
  rtksnprintf(tpath, sizeof(tpath), ":%s", port);

  /* Open TCP server stream */
  if (!(ntripc->tcp = opentcpsvr(tpath, msg, msize))) {
    tracet(2, "openntripc: opentcpsvr error port=%d\n", port);
    free(ntripc);
    return NULL;
  }
  return ntripc;
}
/* Close ntrip-caster --------------------------------------------------------*/
static void closentripc(ntripc_t *ntripc) {
  tracet(3, "closentripc: state=%d\n", ntripc->state);

  closetcpsvr(ntripc->tcp);
  free(ntripc);
}
/* Disconnect ntrip-caster connection ----------------------------------------*/
static void discon_ntripc(ntripc_t *ntripc, int i) {
  tracet(3, "discon_ntripc: i=%d\n", i);

  discontcp(&ntripc->tcp->cli[i], ticonnect);
  ntripc->con[i].nb = 0;
  ntripc->con[i].buff[0] = '\0';
  ntripc->con[i].state = 0;
}
/* Send NTRIP source table ---------------------------------------------------*/
static void send_srctbl(const ntripc_t *ntripc, socket_t sock) {
  char srctbl[512 + NTRIP_MAXSTR], buff[256];

  rtksnprintf(srctbl, sizeof(srctbl), "STR;%s;%s\r\n%s\r\n", ntripc->mntpnt, ntripc->srctbl,
              NTRIP_RSP_TBLEND);
  buff[0] = '\0';
  rtkcatprintf(buff, sizeof(buff), "%s", NTRIP_RSP_SRCTBL);
  rtkcatprintf(buff, sizeof(buff), "Server: %s %s %s\r\n", "RTKLIB", VER_RTKLIB, PATCH_LEVEL);
  char tstr[40];
  rtkcatprintf(buff, sizeof(buff), "Date: %s UTC\r\n", time2str(timeget(), tstr, 0));
  rtkcatprintf(buff, sizeof(buff), "Connection: close\r\n");
  rtkcatprintf(buff, sizeof(buff), "Content-Type: text/plain\r\n");
  rtkcatprintf(buff, sizeof(buff), "Content-Length: %d\r\n\r\n", (int)strlen(srctbl));
  send_nb(sock, (uint8_t *)buff, (int)strlen(buff));
  send_nb(sock, (uint8_t *)srctbl, (int)strlen(srctbl));
}
/* Test NTRIP client request -------------------------------------------------*/
static void rsp_ntripc(ntripc_t *ntripc, int i) {
  tracet(3, "rspntripc_c i=%d\n", i);
  ntripc_con_t *con = ntripc->con + i;
  con->buff[con->nb] = '\0';
  tracet(5, "rspntripc_c: n=%d,buff=\n%s\n", con->nb, con->buff);

  if (con->nb >= NTRIP_MAXRSP - 1) { /* Buffer overflow */
    tracet(2, "rsp_ntripc_c: request buffer overflow\n");
    discon_ntripc(ntripc, i);
    return;
  }
  /* Test GET and User-Agent */
  char *p = strstr((char *)con->buff, "GET"), *q;
  if (!p || !(q = strstr(p, "\r\n")) || !(q = strstr(q, "User-Agent:")) || !strstr(q, "\r\n")) {
    tracet(2, "rsp_ntripc_c: NTRIP request error\n");
    discon_ntripc(ntripc, i);
    return;
  }
  /* Test protocol */
  char url[256] = "", proto[256] = "";
  if (sscanf(p, "GET %255s %255s", url, proto) < 2 ||
      (strcmp(proto, "HTTP/1.0") && strcmp(proto, "HTTP/1.1"))) {
    tracet(2, "rsp_ntripc_c: NTRIP request error proto=%s\n", proto);
    discon_ntripc(ntripc, i);
    return;
  }
  char mntpnt[256] = "";
  int ri = strchri(url, 0, '/');
  if (ri >= 0) rtksubstrcpy(mntpnt, sizeof(mntpnt), url, ri + 1);

  /* Test mountpoint */
  if (!*mntpnt || strcmp(mntpnt, ntripc->mntpnt)) {
    tracet(2, "rsp_ntripc_c: no mountpoint %s\n", mntpnt);

    /* Send source table */
    send_srctbl(ntripc, ntripc->tcp->cli[i].sock);
    discon_ntripc(ntripc, i);
    return;
  }
  /* Test authentication */
  if (*ntripc->passwd) {
    char user[513];
    rtksnprintf(user, sizeof(user), "%s:%s", ntripc->user, ntripc->passwd);
    char user_pwd[256];
    rtksnprintf(user_pwd, sizeof(user_pwd), "Authorization: Basic ");
    size_t plen = strlen(user_pwd);
    encbase64(user_pwd + plen, sizeof(user_pwd) - plen, (uint8_t *)user, strlen(user));
    char *pa = strstr((char *)con->buff, "Authorization:");
    if (!pa || strncmp(pa, user_pwd, strlen(user_pwd))) {
      tracet(2, "rsp_ntripc_c: authroziation error\n");
      const char *rsp1 = NTRIP_RSP_UNAUTH;
      send_nb(ntripc->tcp->cli[i].sock, (uint8_t *)rsp1, strlen(rsp1));
      discon_ntripc(ntripc, i);
      return;
    }
  }
  /* Send OK response */
  const char *rsp2 = NTRIP_RSP_OK_CLI;
  send_nb(ntripc->tcp->cli[i].sock, (uint8_t *)rsp2, strlen(rsp2));

  con->state = 1;
  rtkstrcpy(con->mntpnt, sizeof(con->mntpnt), mntpnt);
}
/* Handle NTRIP client connect request ---------------------------------------*/
static void wait_ntripc(ntripc_t *ntripc, char *msg, size_t msize) {
  tracet(4, "wait_ntripc\n");

  ntripc->state = ntripc->tcp->svr.state;

  if (!waittcpsvr(ntripc->tcp, msg, msize)) return;

  for (int i = 0; i < MAXCLI; i++) {
    if (ntripc->tcp->cli[i].state != 2 || ntripc->con[i].state) continue;

    /* Receive NTRIP client request */
    uint8_t *buff = ntripc->con[i].buff + ntripc->con[i].nb;
    int nmax = NTRIP_MAXRSP - ntripc->con[i].nb - 1;

    int n = recv_nb(ntripc->tcp->cli[i].sock, buff, nmax);
    if (n == -1) {
      int err = errsock();
      if (err) {
        tracet(2, "wait_ntripc: recv error sock=%d err=%d\n", ntripc->tcp->cli[i].sock, err);
      }
      discon_ntripc(ntripc, i);
      continue;
    }
    if (n <= 0) continue;

    /* Test NTRIP client request */
    ntripc->con[i].nb += n;
    rsp_ntripc(ntripc, i);
  }
}
/* Read ntrip-caster ---------------------------------------------------------*/
static int readntripc(ntripc_t *ntripc, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readntripc:\n");

  wait_ntripc(ntripc, msg, msize);

  for (int i = 0; i < MAXCLI; i++) {
    if (!ntripc->con[i].state) continue;

    int nr = recv_nb(ntripc->tcp->cli[i].sock, buff, n);

    if (nr < 0) {
      int err = errsock();
      if (err) {
        tracet(2, "readntripc: recv error i=%d sock=%d err=%d\n", i, ntripc->tcp->cli[i].sock, err);
      }
      discon_ntripc(ntripc, i);
    } else if (nr > 0) {
      ntripc->tcp->cli[i].tact = tickget();
      return nr;
    }
  }
  return 0;
}
/* Write ntrip-caster --------------------------------------------------------*/
static int writentripc(ntripc_t *ntripc, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "writentripc: n=%d\n", n);

  wait_ntripc(ntripc, msg, msize);

  int ns = 0;
  for (int i = 0; i < MAXCLI; i++) {
    if (!ntripc->con[i].state) continue;

    ns = send_nb(ntripc->tcp->cli[i].sock, buff, n);

    if (ns < n) {
      int err = errsock();
      if (err) {
        tracet(2, "writentripc: send error i=%d sock=%d err=%d\n", i, ntripc->tcp->cli[i].sock,
               err);
      }
      discon_ntripc(ntripc, i);
    } else {
      ntripc->tcp->cli[i].tact = tickget();
    }
  }
  return ns;
}
/* Get state ntrip-caster ----------------------------------------------------*/
static int statentripc(ntripc_t *ntripc) { return !ntripc ? 0 : ntripc->state; }
/* Get extended state ntrip-caster -------------------------------------------*/
static int statexntripc(ntripc_t *ntripc, char *msg, size_t msize) {
  int state = !ntripc ? 0 : ntripc->state;

  rtkcatprintf(msg, msize, "ntripc:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", ntripc->state);
  if (!state) return 0;
  rtkcatprintf(msg, msize, "  type    = %d\n", ntripc->type);
  rtkcatprintf(msg, msize, "  mntpnt  = %s\n", ntripc->mntpnt);
  rtkcatprintf(msg, msize, "  user    = %s\n", ntripc->user);
  rtkcatprintf(msg, msize, "  passwd  = %s\n", ntripc->passwd);
  rtkcatprintf(msg, msize, "  srctbl  = %s\n", ntripc->srctbl);
  rtkcatprintf(msg, msize, "  svr:\n");
  statextcp(&ntripc->tcp->svr, msg, msize);
  for (int i = 0; i < MAXCLI; i++) {
    if (!ntripc->tcp->cli[i].state) continue;
    rtkcatprintf(msg, msize, "  cli#%d:\n", i);
    statextcp(ntripc->tcp->cli + i, msg, msize);
    rtkcatprintf(msg, msize, "    mntpnt= %s\n", ntripc->con[i].mntpnt);
    rtkcatprintf(msg, msize, "    nb    = %d\n", ntripc->con[i].nb);
  }
  return state;
}
/* Generate UDP socket -------------------------------------------------------*/
static udp_t *genudp(int type, int port, const char *saddr, char *msg, size_t msize) {
  tracet(3, "genudp: type=%d\n", type);

  udp_t *udp = (udp_t *)malloc(sizeof(udp_t));
  if (!udp) return NULL;
  udp->state = 2;
  udp->type = type;
  udp->port = port;
  rtkstrcpy(udp->saddr, sizeof(udp->saddr), saddr);

  if ((udp->sock = socket(AF_INET, SOCK_DGRAM, 0)) == (socket_t)-1) {
    free(udp);
    rtkcatprintf(msg, msize, "socket error (%d)", errsock());
    return NULL;
  }
  int bs = buffsize;
  if (setsockopt(udp->sock, SOL_SOCKET, SO_RCVBUF, (const char *)&bs, sizeof(bs)) == -1 ||
      setsockopt(udp->sock, SOL_SOCKET, SO_SNDBUF, (const char *)&bs, sizeof(bs)) == -1) {
    tracet(2, "genudp: setsockopt error sock=%d err=%d bs=%d\n", udp->sock, errsock(), bs);
    rtkcatprintf(msg, msize, "sockopt error: bufsiz");
  }
  memset(&udp->addr, 0, sizeof(udp->addr));
  udp->addr.sin_family = AF_INET;
  udp->addr.sin_port = htons(port);

  if (!udp->type) { /* UDP server */
    udp->addr.sin_addr.s_addr = htonl(INADDR_ANY);
#ifdef SVR_REUSEADDR
    int opt = 1;
    setsockopt(udp->sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));
#endif
    if (bind(udp->sock, (struct sockaddr *)&udp->addr, sizeof(udp->addr)) == -1) {
      tracet(2, "genudp: bind error sock=%d port=%d err=%d\n", udp->sock, port, errsock());
      rtkcatprintf(msg, msize, "bind error (%d): %d", errsock(), port);
      closesocket(udp->sock);
      free(udp);
      return NULL;
    }
  } else { /* UDP client */
    int opt = 1;
    if (!strcmp(saddr, "255.255.255.255") &&
        setsockopt(udp->sock, SOL_SOCKET, SO_BROADCAST, (const char *)&opt, sizeof(opt)) == -1) {
      tracet(2, "genudp: setsockopt error sock=%d err=%d\n", udp->sock, errsock());
      rtkcatprintf(msg, msize, "sockopt error: broadcast");
    }
    struct hostent *hp = gethostbyname(saddr);
    if (!hp) {
      rtkcatprintf(msg, msize, "address error (%s)", saddr);
      closesocket(udp->sock);
      free(udp);
      return NULL;
    }
    memcpy(&udp->addr.sin_addr, hp->h_addr, hp->h_length);
  }
  return udp;
}
/* Open UDP server -----------------------------------------------------------*/
static udp_t *openudpsvr(const char *path, char *msg, size_t msize) {
  tracet(3, "openudpsvr: path=%s\n", path);

  char sport[256] = "";
  decodetcppath(path, NULL, sport, NULL, NULL, NULL, NULL);

  int port;
  if (sscanf(sport, "%d", &port) < 1) {
    rtkcatprintf(msg, msize, "port error: %s", sport);
    tracet(2, "openudpsvr: port error port=%s\n", port);
    return NULL;
  }
  return genudp(0, port, "", msg, msize);
}
/* Close UDP server ----------------------------------------------------------*/
static void closeudpsvr(udp_t *udpsvr) {
  tracet(3, "closeudpsvr: sock=%d\n", udpsvr->sock);

  closesocket(udpsvr->sock);
  free(udpsvr);
}
/* Read UDP server -----------------------------------------------------------*/
static int readudpsvr(udp_t *udpsvr, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readudpsvr: sock=%d n=%d\n", udpsvr->sock, n);

  fd_set rs;
  FD_ZERO(&rs);
  FD_SET(udpsvr->sock, &rs);
  struct timeval tv = {0};
  int ret = select(udpsvr->sock + 1, &rs, NULL, NULL, &tv);
  if (ret <= 0) return ret;
  int nr = recvfrom(udpsvr->sock, (char *)buff, n, 0, NULL, NULL);
  return nr <= 0 ? -1 : nr;
}
/* Get state UDP server ------------------------------------------------------*/
static int stateudpsvr(udp_t *udpsvr) { return udpsvr ? udpsvr->state : 0; }
/* Get extended state UDP server ---------------------------------------------*/
static int statexudpsvr(udp_t *udpsvr, char *msg, size_t msize) {
  int state = udpsvr ? udpsvr->state : 0;

  rtkcatprintf(msg, msize, "udpsvr:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  if (!state) return 0;
  rtkcatprintf(msg, msize, "  type    = %d\n", udpsvr->type);
  rtkcatprintf(msg, msize, "  sock    = %d\n", (int)udpsvr->sock);
  rtkcatprintf(msg, msize, "  port    = %d\n", udpsvr->port);
  return state;
}
/* Open UDP client -----------------------------------------------------------*/
static udp_t *openudpcli(const char *path, char *msg, size_t msize) {
  tracet(3, "openudpsvr: path=%s\n", path);

  char sport[256] = "", saddr[256] = "";
  decodetcppath(path, saddr, sport, NULL, NULL, NULL, NULL);

  int port;
  if (sscanf(sport, "%d", &port) < 1) {
    rtkcatprintf(msg, msize, "port error: %s", sport);
    tracet(2, "openudpcli: port error port=%s\n", sport);
    return NULL;
  }
  return genudp(1, port, saddr, msg, msize);
}
/* Close UDP client ----------------------------------------------------------*/
static void closeudpcli(udp_t *udpcli) {
  tracet(3, "closeudpcli: sock=%d\n", udpcli->sock);

  closesocket(udpcli->sock);
  free(udpcli);
}
/* Write UDP client ----------------------------------------------------------*/
static int writeudpcli(udp_t *udpcli, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "writeudpcli: sock=%d n=%d\n", udpcli->sock, n);

  return (int)sendto(udpcli->sock, (char *)buff, n, 0, (struct sockaddr *)&udpcli->addr,
                     sizeof(udpcli->addr));
}
/* Get state UDP client ------------------------------------------------------*/
static int stateudpcli(udp_t *udpcli) { return udpcli ? udpcli->state : 0; }
/* Get extended state UDP client ---------------------------------------------*/
static int statexudpcli(udp_t *udpcli, char *msg, size_t msize) {
  int state = udpcli ? udpcli->state : 0;

  rtkcatprintf(msg, msize, "udpsvr:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  if (!state) return 0;
  rtkcatprintf(msg, msize, "  type    = %d\n", udpcli->type);
  rtkcatprintf(msg, msize, "  sock    = %d\n", (int)udpcli->sock);
  rtkcatprintf(msg, msize, "  addr    = %s\n", udpcli->saddr);
  rtkcatprintf(msg, msize, "  port    = %d\n", udpcli->port);
  return state;
}
/* Decode FTP path -----------------------------------------------------------*/
static void decodeftppath(const char *path, char *addr, size_t asize, char *file, size_t fsize,
                          char *user, size_t usize, char *passwd, size_t psize, int *topts) {
  tracet(4, "decodeftpath: path=%s\n", path);

  if (user) *user = '\0';
  if (passwd) *passwd = '\0';
  if (topts) {
    topts[0] = 0;    /* Time offset in path (s) */
    topts[1] = 3600; /* Download interval (s) */
    topts[2] = 0;    /* Download time offset (s) */
    topts[3] = 0;    /* Retry interval (s) (0: no retry) */
  }
  char buff[MAXSTRPATH];
  rtkstrcpy(buff, sizeof(buff), path);

  char *p = strchr(buff, '/');
  if (p) {
    char *q = strstr(p + 1, "::");
    if (q) {
      *q = '\0';
      if (topts) sscanf(q + 2, "T=%d,%d,%d,%d", topts, topts + 1, topts + 2, topts + 3);
    }
    rtkstrcpy(file, fsize, p + 1);
    *p = '\0';
  } else
    file[0] = '\0';

  p = strrchr(buff, '@');
  if (p) {
    *p++ = '\0';
    char *q = strchr(buff, ':');
    if (q) {
      *q = '\0';
      if (passwd) rtkstrcpy(passwd, psize, q + 1);
    }
    if (user) rtkstrcpy(user, usize, buff);
  } else
    p = buff;

  rtkstrcpy(addr, asize, p);
}
/* Next download time --------------------------------------------------------*/
static gtime_t nextdltime(const int *topts, int stat) {
  tracet(3, "nextdltime: topts=%d %d %d %d stat=%d\n", topts[0], topts[1], topts[2], topts[3],
         stat);

  /* Current time (GPST) */
  gtime_t time = utc2gpst(timeget());
  int week;
  long double tow = time2gpst(time, &week);

  /* Next retry time */
  if (stat == 0 && topts[3] > 0) {
    tow = (floorl((tow - topts[2]) / topts[3]) + 1.0L) * topts[3] + topts[2];
    return gpst2time(week, tow);
  }
  /* Next interval time */
  int tint = topts[1] <= 0 ? 3600 : topts[1];
  tow = (floorl((tow - topts[2]) / tint) + 1.0L) * tint + topts[2];
  time = gpst2time(week, tow);

  return time;
}
/* FTP thread ----------------------------------------------------------------*/
#ifdef WIN32
static DWORD WINAPI ftpthread(void *arg)
#else
static void *ftpthread(void *arg)
#endif
{
  ftp_t *ftp = (ftp_t *)arg;

  tracet(3, "ftpthread:\n");

  if (!*localdir) {
    tracet(2, "no local directory\n");
    ftp->error = 11;
    ftp->state = 3;
    return 0;
  }
  /* Replace keyword in file path and local path */
  gtime_t time = timeadd(utc2gpst(timeget()), ftp->topts[0]);
  char remote[FNSIZE];
  reppath(ftp->file, remote, sizeof(remote), time, "", "");

  char *pr = strrchr(remote, '/');
  if (pr)
    pr++;
  else
    pr = remote;
  char local[FNSIZE];
  rtksnprintf(local, sizeof(local), "%.768s%c%.254s", localdir, RTKLIB_FILEPATHSEP, pr);
  char errfile[FNSIZE];
  rtksnprintf(errfile, sizeof(errfile), "%.1019s.err", local);

  /* If local file exist, skip download */
  char tmpfile[FNSIZE];
  rtkstrcpy(tmpfile, sizeof(tmpfile), local);
  char *pe = strrchr(tmpfile, '.');
  if (pe && (!strcmp(pe, ".z") || !strcmp(pe, ".gz") || !strcmp(pe, ".zip") || !strcmp(pe, ".Z") ||
             !strcmp(pe, ".GZ") || !strcmp(pe, ".ZIP"))) {
    *pe = '\0';
  }
  FILE *fp = fopen(tmpfile, "rb");
  if (fp) {
    fclose(fp);
    rtksnprintf(ftp->local, sizeof(ftp->local), "%.1023s", tmpfile);
    tracet(3, "ftpthread: file exists %s\n", ftp->local);
    ftp->state = 2;
    return 0;
  }
  /* Proxy settings for wget (ref [2]) */
  char env[1024] = "";
  const char *proxyopt = "";
  if (*proxyaddr) {
    const char *proto = ftp->proto ? "http" : "ftp";
    rtksnprintf(env, sizeof(env), "set %.4s_proxy=http://%.998s & ", proto, proxyaddr);
    proxyopt = "--proxy=on ";
  }
  /* Download command (ref [2]) */
  char cmd[5120], opt[1024];
  if (ftp->proto == 0) { /* FTP */
    rtksnprintf(opt, sizeof(opt),
                "--ftp-user=%.32s --ftp-password=%.32s --glob=off "
                "--passive-ftp %.32s -t 1 -T %d -O \"%.768s\"",
                ftp->user, ftp->passwd, proxyopt, FTP_TIMEOUT, local);
    rtksnprintf(cmd, sizeof(cmd), "%s%s %s \"ftp://%s/%s\" 2> \"%.768s\"\n", env, FTP_CMD, opt,
                ftp->addr, remote, errfile);
  } else { /* HTTP */
    rtksnprintf(opt, sizeof(opt), "%.32s -t 1 -T %d -O \"%.768s\"", proxyopt, FTP_TIMEOUT, local);
    rtksnprintf(cmd, sizeof(cmd), "%s%s %s \"http://%s/%s\" 2> \"%.768s\"\n", env, FTP_CMD, opt,
                ftp->addr, remote, errfile);
  }
  /* Execute download command */
  int ret = execcmd(cmd);
  if (ret) {
    remove(local);
    tracet(2, "execcmd error: cmd=%s ret=%d\n", cmd, ret);
    ftp->error = ret;
    ftp->state = 3;
    return 0;
  }
  remove(errfile);

  /* Uncompress downloaded file */
  char *p = strrchr(local, '.');
  if (p && (!strcmp(p, ".z") || !strcmp(p, ".gz") || !strcmp(p, ".zip") || !strcmp(p, ".Z") ||
            !strcmp(p, ".GZ") || !strcmp(p, ".ZIP"))) {
    if (rtk_uncompress(local, tmpfile, sizeof(tmpfile))) {
      remove(local);
      rtkstrcpy(local, sizeof(local), tmpfile);
    } else {
      tracet(2, "file uncompact error: %s\n", local);
      ftp->error = 12;
      ftp->state = 3;
      return 0;
    }
  }
  rtkstrcpy(ftp->local, sizeof(ftp->local), local);
  ftp->state = 2; /* FTP completed */

  tracet(3, "ftpthread: complete cmd=%s\n", cmd);
  return 0;
}
/* Open FTP ------------------------------------------------------------------*/
static ftp_t *openftp(const char *path, int type, char *msg, size_t msize) {
  tracet(3, "openftp: path=%s type=%d\n", path, type);

  ftp_t *ftp = (ftp_t *)malloc(sizeof(ftp_t));
  if (!ftp) return NULL;

  ftp->state = 0;
  ftp->proto = type;
  ftp->error = 0;
  ftp->thread = 0;
  ftp->local[0] = '\0';

  /* Decode FTP path */
  decodeftppath(path, ftp->addr, sizeof(ftp->addr), ftp->file, sizeof(ftp->file), ftp->user,
                sizeof(ftp->user), ftp->passwd, sizeof(ftp->passwd), ftp->topts);

  /* Set first download time */
  ftp->tnext = timeadd(timeget(), 10.0L);

  return ftp;
}
/* Close FTP -----------------------------------------------------------------*/
static void closeftp(ftp_t *ftp) {
  tracet(3, "closeftp: state=%d\n", ftp->state);

  if (ftp->state != 1) free(ftp);
}
/* Read FTP ------------------------------------------------------------------*/
static int readftp(ftp_t *ftp, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readftp: n=%d\n", n);

  gtime_t time = utc2gpst(timeget());

  if (timediff(time, ftp->tnext) < 0.0L) { /* Until download time? */
    return 0;
  }
  if (ftp->state <= 0) { /* FTP/HTTP not executed? */
    ftp->state = 1;
    rtksnprintf(msg, msize, "%s://%s", ftp->proto ? "http" : "ftp", ftp->addr);

#ifdef WIN32
    if (!(ftp->thread = CreateThread(NULL, 0, ftpthread, ftp, 0, NULL))) {
#else
    if (pthread_create(&ftp->thread, NULL, ftpthread, ftp)) {
#endif
      tracet(2, "readftp: ftp thread create error\n");
      ftp->state = 3;
      rtkstrcpy(msg, msize, "ftp thread error");
      return 0;
    }
  }
  if (ftp->state <= 1) return 0; /* FTP/HTTP on going? */

  if (ftp->state == 3) { /* FTP error */
    rtksnprintf(msg, msize, "%s error (%d)", ftp->proto ? "http" : "ftp", ftp->error);

    /* Set next retry time */
    ftp->tnext = nextdltime(ftp->topts, 0);
    ftp->state = 0;
    return 0;
  }
  /* Return local file path if FTP completed */
  rtkstrcpy((char *)buff, n, ftp->local);
  rtkstrcat((char *)buff, n, "\r\n");

  /* Set next download time */
  ftp->tnext = nextdltime(ftp->topts, 1);
  ftp->state = 0;

  rtkstrcpy(msg, msize, "");

  return (int)strlen((char *)buff);
}
/* Get state FTP -------------------------------------------------------------*/
static int stateftp(const ftp_t *ftp) {
  return !ftp ? 0 : (ftp->state == 0 ? 2 : (ftp->state <= 2 ? 3 : -1));
}
/* Get extended state FTP ----------------------------------------------------*/
static int statexftp(const ftp_t *ftp, char *msg, size_t msize) {
  return !ftp ? 0 : (ftp->state == 0 ? 2 : (ftp->state <= 2 ? 3 : -1));
}
/* Open memory buffer --------------------------------------------------------*/
static membuf_t *openmembuf(const char *path, char *msg, size_t msize) {
  tracet(3, "openmembuf: path=%s\n", path);

  int bufsize = DEFAULT_MEMBUF_SIZE;
  sscanf(path, "%d", &bufsize);

  membuf_t *membuf = (membuf_t *)malloc(sizeof(membuf_t));
  if (!membuf) return NULL;
  membuf->state = 1;
  membuf->rp = 0;
  membuf->wp = 0;
  if (!(membuf->buf = (uint8_t *)malloc(bufsize))) {
    free(membuf);
    return NULL;
  }
  membuf->bufsize = bufsize;
  rtklib_initlock(&membuf->lock);

  rtkcatprintf(msg, msize, "membuf sizebuf=%d", bufsize);

  return membuf;
}
/* Close memory buffer -------------------------------------------------------*/
static void closemembuf(membuf_t *membuf) {
  tracet(3, "closemembufp\n");

  free(membuf->buf);
  free(membuf);
}
/* Read memory buffer --------------------------------------------------------*/
static int readmembuf(membuf_t *membuf, uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(4, "readmembuf: n=%d\n", n);

  if (!membuf) return 0;

  rtklib_lock(&membuf->lock);

  int i, nr = 0;
  for (i = membuf->rp; i != membuf->wp && nr < n; i++) {
    if (i >= membuf->bufsize) i = 0;
    buff[nr++] = membuf->buf[i];
  }
  membuf->rp = i;
  rtklib_unlock(&membuf->lock);
  return nr;
}
/* Write memory buffer -------------------------------------------------------*/
static int writemembuf(membuf_t *membuf, const uint8_t *buff, int n, char *msg, size_t msize) {
  tracet(3, "writemembuf: n=%d\n", n);

  if (!membuf) return 0;

  rtklib_lock(&membuf->lock);

  int i;
  for (i = 0; i < n; i++) {
    membuf->buf[membuf->wp++] = buff[i];
    if (membuf->wp >= membuf->bufsize) membuf->wp = 0;
    if (membuf->wp == membuf->rp) {
      rtkstrcpy(msg, msize, "mem-buffer overflow");
      membuf->state = -1;
      rtklib_unlock(&membuf->lock);
      return i + 1;
    }
  }
  rtklib_unlock(&membuf->lock);
  return i;
}
/* Get state memory buffer ---------------------------------------------------*/
static int statemembuf(membuf_t *membuf) { return !membuf ? 0 : membuf->state; }
/* Get extended state memory buffer ------------------------------------------*/
static int statexmembuf(membuf_t *membuf, char *msg, size_t msize) {
  int state = !membuf ? 0 : membuf->state;

  rtkcatprintf(msg, msize, "membuf:\n");
  rtkcatprintf(msg, msize, "  state   = %d\n", state);
  if (!state) return 0;
  rtkcatprintf(msg, msize, "  buffsize= %d\n", membuf->bufsize);
  rtkcatprintf(msg, msize, "  wp      = %d\n", membuf->wp);
  rtkcatprintf(msg, msize, "  rp      = %d\n", membuf->rp);
  return state;
}
/* Initialize stream environment -----------------------------------------------
 * Initialize stream environment
 * Args   : none
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strinitcom(void) {
  tracet(3, "strinitcom:\n");

#ifdef WIN32
  WSADATA data;
  WSAStartup(MAKEWORD(2, 0), &data);
#endif
}
/* Initialize stream -----------------------------------------------------------
 * Initialize stream struct
 * Args   : stream_t *stream IO  stream
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strinit(stream_t *stream) {
  tracet(3, "strinit:\n");

  stream->type = 0;
  stream->mode = 0;
  stream->state = 0;
  stream->inb = stream->inr = stream->outb = stream->outr = 0;
  stream->tick_i = stream->tick_o = stream->tact = stream->inbt = stream->outbt = 0;
  rtklib_initlock(&stream->lock);
  stream->port = NULL;
  stream->path[0] = '\0';
  stream->msg[0] = '\0';
}
/* Open stream -----------------------------------------------------------------
 *
 * Open stream to read or write data from or to virtual devices.
 *
 * Args   : stream_t *stream IO  stream
 *          int type         I   stream type
 *                                 STR_SERIAL   = serial device
 *                                 STR_FILE     = file (record and playback)
 *                                 STR_TCPSVR   = TCP server
 *                                 STR_TCPCLI   = TCP client
 *                                 STR_NTRIPSVR = NTRIP server
 *                                 STR_NTRIPCLI = NTRIP client
 *                                 STR_NTRIPCAS = NTRIP caster client
 *                                 STR_UDPSVR   = UDP server (read only)
 *                                 STR_UDPCLI   = UDP client (write only)
 *                                 STR_MEMBUF   = memory buffer (FIFO)
 *                                 STR_FTP      = download by FTP (raed only)
 *                                 STR_HTTP     = download by HTTP (raed only)
 *          int mode         I   stream mode (STR_MODE_???)
 *                                 STR_MODE_R   = read only
 *                                 STR_MODE_W   = write only
 *                                 STR_MODE_RW  = read and write
 *          char *path       I   stream path (see below)
 *
 * Return : status (true:ok,false:error)
 *
 * Notes  : see reference [1] for NTRIP
 *          STR_FTP/HTTP needs "wget" in command search paths
 *
 * Stream path ([] options):
 *
 *   STR_SERIAL   port[:brate[:bsize[:parity[:stopb[:fctr[#port]]]]]]
 *                    port  = COM??  (windows)
 *                            tty??? (linuex, omit /dev/)
 *                    brate = bit rate     (bps)
 *                    bsize = bit size     (7|8)
 *                    parity= parity       (n|o|e)
 *                    stopb = stop bits    (1|2)
 *                    fctr  = flow control (off|rts)
 *                    port  = TCP server port to output received stream
 *
 *   STR_FILE     path[::T][::+start][::xseppd][::S=swap][::P={4|8}]
 *                    path  = file path
 *                            (can include keywords defined by )
 *                    ::T   = enable time tag
 *                    start = replay start offset (s)
 *                    speed = replay speed factor
 *                    swap  = output swap interval (hr) (0: no swap)
 *                    ::P={4|8} = file pointer size (4:32bit,8:64bit)
 *
 *   STR_TCPSVR   :port
 *                    port  = TCP server port to accept
 *
 *   STR_TCPCLI   addr:port
 *                    addr  = TCP server address to connect
 *                    port  = TCP server port to connect
 *
 *   STR_NTRIPSVR [:passwd@]addr[:port]/mponit[:string]
 *                    addr  = NTRIP caster address to connect
 *                    port  = NTRIP caster server port to connect
 *                    passwd= NTRIP caster server password to connect
 *                    mpoint= NTRIP mountpoint
 *                    string= NTRIP server string
 *
 *   STR_NTRIPCLI [user[:passwd]@]addr[:port]/mpoint
 *                    addr  = NTRIP caster address to connect
 *                    port  = NTRIP caster client port to connect
 *                    user  = NTRIP caster client user to connect
 *                    passwd= NTRIP caster client password to connect
 *                    mpoint= NTRIP mountpoint
 *
 *   STR_NTRIPCAS [user[:passwd]@][:port]/mpoint[:srctbl]
 *                    port  = NTRIP caster client port to accept connection
 *                    user  = NTRIP caster client user to accept connection
 *                    passwd= NTRIP caster client password to accept connection
 *                    mpoint= NTRIP mountpoint
 *                    srctbl= NTRIP source table entry (STR) (ref [3] 6.3)
 *                      (ID;format;format-details;carrier;nav-system;network;
 *                       country;latitude;longitude;nmea;solution;generator;
 *                       compr-encrp;autentication;fee;bitrate;...;misc)
 *
 *   STR_UDPSVR   :port
 *                    port  = UDP server port to receive
 *
 *   STR_UDPCLI   addr:port
 *                    addr  = UDP server or broadcast address to send
 *                    port  = UDP server or broadcast port to send
 *
 *   STR_MEMBUF   [size]
 *                    size  = FIFO size (bytes) ("":4096)
 *
 *   STR_FTP      [user[:passwd]@]addr/path[::T=poff[,tint[,toff,tret]]]]
 *                    user  = FTP server user
 *                    passwd= FTP server password
 *                    addr  = FTP server address
 *                    path  = FTP server file path
 *                    poff  = time offset for path extension (s)
 *                    tint  = download interval (s)
 *                    toff  = download time offset (s)
 *                    tret  = download retry interval (s) (0:no retry)
 *
 *   STR_HTTP     addr/path[::T=poff[,tint[,toff,tret]]]]
 *                    addr  = HTTP server address
 *                    path  = HTTP server file path
 *                    poff  = time offset for path extension (s)
 *                    tint  = download interval (s)
 *                    toff  = download time offset (s)
 *                    tret  = download retry interval (s) (0:no retry)
 *
 *----------------------------------------------------------------------------*/
extern bool stropen(stream_t *stream, int type, int mode, const char *path) {
  tracet(3, "stropen: type=%d mode=%d path=%s\n", type, mode, path);

  stream->type = type;
  stream->mode = mode;
  rtkstrcpy(stream->path, sizeof(stream->path), path);
  stream->inb = stream->inr = stream->outb = stream->outr = 0;
  stream->tick_i = stream->tick_o = tickget();
  stream->inbt = stream->outbt = 0;
  stream->msg[0] = '\0';
  stream->port = NULL;
  char *msg = stream->msg;
  size_t msize = sizeof(stream->msg);
  switch (type) {
    case STR_SERIAL:
      stream->port = openserial(path, mode, msg, msize);
      break;
    case STR_FILE:
      stream->port = openfile(path, mode, msg, msize);
      break;
    case STR_TCPSVR:
      stream->port = opentcpsvr(path, msg, msize);
      break;
    case STR_TCPCLI:
      stream->port = opentcpcli(path, msg, msize);
      break;
    case STR_NTRIPSVR:
      stream->port = openntrip(path, 0, msg, msize);
      break;
    case STR_NTRIPCLI:
      stream->port = openntrip(path, 1, msg, msize);
      break;
    case STR_NTRIPCAS:
      stream->port = openntripc(path, msg, msize);
      break;
    case STR_UDPSVR:
      stream->port = openudpsvr(path, msg, msize);
      break;
    case STR_UDPCLI:
      stream->port = openudpcli(path, msg, msize);
      break;
    case STR_MEMBUF:
      stream->port = openmembuf(path, msg, msize);
      break;
    case STR_FTP:
      stream->port = openftp(path, 0, msg, msize);
      break;
    case STR_HTTP:
      stream->port = openftp(path, 1, msg, msize);
      break;
    default:
      stream->state = 0;
      return true;
  }
  stream->state = !stream->port ? -1 : 1;
  return stream->port != NULL;
}
/* Close stream ----------------------------------------------------------------
 * Close stream
 * Args   : stream_t *stream IO  stream
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strclose(stream_t *stream) {
  tracet(3, "strclose: type=%d mode=%d\n", stream->type, stream->mode);

  strlock(stream);

  if (stream->port) {
    switch (stream->type) {
      case STR_SERIAL:
        closeserial((serial_t *)stream->port);
        break;
      case STR_FILE:
        closefile((file_t *)stream->port);
        break;
      case STR_TCPSVR:
        closetcpsvr((tcpsvr_t *)stream->port);
        break;
      case STR_TCPCLI:
        closetcpcli((tcpcli_t *)stream->port);
        break;
      case STR_NTRIPSVR:
        closentrip((ntrip_t *)stream->port);
        break;
      case STR_NTRIPCLI:
        closentrip((ntrip_t *)stream->port);
        break;
      case STR_NTRIPCAS:
        closentripc((ntripc_t *)stream->port);
        break;
      case STR_UDPSVR:
        closeudpsvr((udp_t *)stream->port);
        break;
      case STR_UDPCLI:
        closeudpcli((udp_t *)stream->port);
        break;
      case STR_MEMBUF:
        closemembuf((membuf_t *)stream->port);
        break;
      case STR_FTP:
        closeftp((ftp_t *)stream->port);
        break;
      case STR_HTTP:
        closeftp((ftp_t *)stream->port);
        break;
    }
  } else {
    trace(3, "no port to close stream: type=%d\n", stream->type);
  }
  stream->type = 0;
  stream->mode = 0;
  stream->state = 0;
  stream->inr = stream->outr = 0;
  stream->path[0] = '\0';
  stream->msg[0] = '\0';
  stream->port = NULL;

  strunlock(stream);
}
/* Sync streams ----------------------------------------------------------------
 * Sync time for streams
 * Args   : stream_t *stream1 IO stream 1
 *          stream_t *stream2 IO stream 2
 * Return : none
 * Notes  : for replay files with time tags
 *----------------------------------------------------------------------------*/
extern void strsync(stream_t *stream1, stream_t *stream2) {
  if (stream1->type != STR_FILE || stream2->type != STR_FILE) return;
  file_t *file1 = (file_t *)stream1->port;
  file_t *file2 = (file_t *)stream2->port;
  if (file1 && file2) syncfile(file1, file2);
}
/* Lock/unlock stream ----------------------------------------------------------
 * Lock/unlock stream
 * Args   : stream_t *stream I  stream
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strlock(stream_t *stream) { rtklib_lock(&stream->lock); }
extern void strunlock(stream_t *stream) { rtklib_unlock(&stream->lock); }

/* Read stream -----------------------------------------------------------------
 * Read data from stream (unblocked)
 * Args   : stream_t *stream I  stream
 *          unsinged char *buff O data buffer
 *          int    n         I  maximum data length
 * Return : read data length
 * Notes  : if no data, return immediately with no data
 *----------------------------------------------------------------------------*/
extern int strread(stream_t *stream, uint8_t *buff, int n) {
  tracet(4, "strread: n=%d\n", n);

  if (!(stream->mode & STR_MODE_R) || !stream->port) return 0;

  strlock(stream);

  char *msg = stream->msg;

  /* The stream msg buffer is not cleared on each read, and the read
   * functions overwrite the msg buffer only when noting exceptional
   * events, they do not append to the msg buffer. */
  size_t msize = sizeof(stream->msg);
  uint32_t tick = tickget();

  int nr = 0;
  switch (stream->type) {
    case STR_SERIAL:
      nr = readserial((serial_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_FILE:
      nr = readfile((file_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_TCPSVR:
      nr = readtcpsvr((tcpsvr_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_TCPCLI:
      nr = readtcpcli((tcpcli_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_NTRIPSVR:
    case STR_NTRIPCLI:
      nr = readntrip((ntrip_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_NTRIPCAS:
      nr = readntripc((ntripc_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_UDPSVR:
      nr = readudpsvr((udp_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_MEMBUF:
      nr = readmembuf((membuf_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_FTP:
      nr = readftp((ftp_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_HTTP:
      nr = readftp((ftp_t *)stream->port, buff, n, msg, msize);
      break;
    default:
      strunlock(stream);
      return 0;
  }
  if (nr > 0) {
    stream->inb += nr;
    stream->tact = tick;
  }
  int tt = (int)(tick - stream->tick_i);
  if (tt >= tirate) {
    stream->inr = (uint32_t)((long double)((stream->inb - stream->inbt) * 8) / (tt * 0.001L));
    stream->tick_i = tick;
    stream->inbt = stream->inb;
  }
  strunlock(stream);
  return nr;
}
/* Write stream ----------------------------------------------------------------
 * Write data to stream (unblocked)
 * Args   : stream_t *stream I   stream
 *          unsinged char *buff I data buffer
 *          int    n         I   data length
 * Return : status (0:error,0<bytes written)
 * Notes  : write data to buffer and return immediately
 *----------------------------------------------------------------------------*/
extern int strwrite(stream_t *stream, uint8_t *buff, int n) {
  tracet(4, "strwrite: n=%d\n", n);

  if (!(stream->mode & STR_MODE_W) || !stream->port) return 0;

  strlock(stream);

  uint32_t tick = tickget();
  /* The stream msg buffer is not cleared on each write, and the write
   * functions overwrite the msg buffer only when noting exceptional
   * events, they do not append to the msg buffer. */
  char *msg = stream->msg;
  size_t msize = sizeof(stream->msg);

  int ns;
  switch (stream->type) {
    case STR_SERIAL:
      ns = writeserial((serial_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_FILE:
      ns = writefile((file_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_TCPSVR:
      ns = writetcpsvr((tcpsvr_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_TCPCLI:
      ns = writetcpcli((tcpcli_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_NTRIPSVR:
    case STR_NTRIPCLI:
      ns = writentrip((ntrip_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_NTRIPCAS:
      ns = writentripc((ntripc_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_UDPCLI:
      ns = writeudpcli((udp_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_MEMBUF:
      ns = writemembuf((membuf_t *)stream->port, buff, n, msg, msize);
      break;
    case STR_FTP:
    case STR_HTTP:
    default:
      strunlock(stream);
      return 0;
  }
  if (ns > 0) {
    stream->outb += ns;
    stream->tact = tick;
  }
  int tt = (int)(tick - stream->tick_o);
  if (tt > tirate) {
    stream->outr = (uint32_t)((long double)((stream->outb - stream->outbt) * 8) / (tt * 0.001L));
    stream->tick_o = tick;
    stream->outbt = stream->outb;
  }
  strunlock(stream);
  return ns;
}
/* Get stream status -----------------------------------------------------------
 * Get stream status
 * Args   : stream_t *stream I   stream
 *          char   *msg      IO  status message (NULL: no output)
 * Return : status (-1:error,0:close,1:wait,2:connect,3:active)
 * Note   : Messages are appended to msg which must be nul terminated.
 *----------------------------------------------------------------------------*/
extern int strstat(stream_t *stream, char *msg, size_t msize) {
  tracet(4, "strstat:\n");

  strlock(stream);
  if (msg) {
    rtkstrcat(msg, msize, stream->msg);
  }
  if (!stream->port) {
    strunlock(stream);
    return stream->state;
  }
  int state = 0;
  switch (stream->type) {
    case STR_SERIAL:
      state = stateserial((serial_t *)stream->port);
      break;
    case STR_FILE:
      state = statefile((file_t *)stream->port);
      break;
    case STR_TCPSVR:
      state = statetcpsvr((tcpsvr_t *)stream->port);
      break;
    case STR_TCPCLI:
      state = statetcpcli((tcpcli_t *)stream->port);
      break;
    case STR_NTRIPSVR:
    case STR_NTRIPCLI:
      state = statentrip((ntrip_t *)stream->port);
      break;
    case STR_NTRIPCAS:
      state = statentripc((ntripc_t *)stream->port);
      break;
    case STR_UDPSVR:
      state = stateudpsvr((udp_t *)stream->port);
      break;
    case STR_UDPCLI:
      state = stateudpcli((udp_t *)stream->port);
      break;
    case STR_MEMBUF:
      state = statemembuf((membuf_t *)stream->port);
      break;
    case STR_FTP:
      state = stateftp((ftp_t *)stream->port);
      break;
    case STR_HTTP:
      state = stateftp((ftp_t *)stream->port);
      break;
    default:
      strunlock(stream);
      return 0;
  }
  if (state == 2 && (int)(tickget() - stream->tact) <= TINTACT) state = 3;
  strunlock(stream);
  return state;
}
/* Get extended stream status --------------------------------------------------
 * Get extended stream status
 * Args   : stream_t *stream I   stream
 *          char   *msg      IO  extended status message
 * Return : status (-1:error,0:close,1:wait,2:connect,3:active)
 * Note   : The output is appended to msg which must be nul terminated.
 *----------------------------------------------------------------------------*/
extern int strstatx(stream_t *stream, char *msg, size_t msize) {
  tracet(4, "strstatx:\n");

  strlock(stream);

  if (!stream->port) {
    strunlock(stream);
    return stream->state;
  }

  int state = 0;
  switch (stream->type) {
    case STR_SERIAL:
      state = statexserial((serial_t *)stream->port, msg, msize);
      break;
    case STR_FILE:
      state = statexfile((file_t *)stream->port, msg, msize);
      break;
    case STR_TCPSVR:
      state = statextcpsvr((tcpsvr_t *)stream->port, msg, msize);
      break;
    case STR_TCPCLI:
      state = statextcpcli((tcpcli_t *)stream->port, msg, msize);
      break;
    case STR_NTRIPSVR:
    case STR_NTRIPCLI:
      state = statexntrip((ntrip_t *)stream->port, msg, msize);
      break;
    case STR_NTRIPCAS:
      state = statexntripc((ntripc_t *)stream->port, msg, msize);
      break;
    case STR_UDPSVR:
      state = statexudpsvr((udp_t *)stream->port, msg, msize);
      break;
    case STR_UDPCLI:
      state = statexudpcli((udp_t *)stream->port, msg, msize);
      break;
    case STR_MEMBUF:
      state = statexmembuf((membuf_t *)stream->port, msg, msize);
      break;
    case STR_FTP:
      state = statexftp((ftp_t *)stream->port, msg, msize);
      break;
    case STR_HTTP:
      state = statexftp((ftp_t *)stream->port, msg, msize);
      break;
    default:
      strunlock(stream);
      return 0;
  }
  if (state == 2 && (int)(tickget() - stream->tact) <= TINTACT) state = 3;
  strunlock(stream);
  return state;
}
/* Get stream statistics summary -----------------------------------------------
 * Get stream statistics summary
 * Args   : stream_t *stream I   stream
 *          int    *inb      IO   bytes of input  (NULL: no output)
 *          int    *inr      IO   bps of input    (NULL: no output)
 *          int    *outb     IO   bytes of output (NULL: no output)
 *          int    *outr     IO   bps of output   (NULL: no output)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsum(stream_t *stream, int *inb, int *inr, int *outb, int *outr) {
  tracet(4, "strsum:\n");

  strlock(stream);
  if (inb) *inb = stream->inb;
  if (inr) *inr = stream->inr;
  if (outb) *outb = stream->outb;
  if (outr) *outr = stream->outr;
  strunlock(stream);
}
/* Set global stream options ---------------------------------------------------
 * Set global stream options
 * Args   : int    *opt      I   options
 *              opt[0]= inactive timeout (ms) (0: no timeout)
 *              opt[1]= interval to reconnect (ms)
 *              opt[2]= averaging time of data rate (ms)
 *              opt[3]= receive/send buffer size (bytes);
 *              opt[4]= file swap margin (s)
 *              opt[5]= reserved
 *              opt[6]= reserved
 *              opt[7]= reserved
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsetopt(const int *opt) {
  tracet(3, "strsetopt: opt=%d %d %d %d %d %d %d %d\n", opt[0], opt[1], opt[2], opt[3], opt[4],
         opt[5], opt[6], opt[7]);

  toinact = 0 < opt[0] && opt[0] < 1000 ? 1000 : opt[0]; /* >=1s */
  ticonnect = opt[1] < 1000 ? 1000 : opt[1];             /* >=1s */
  tirate = opt[2] < 100 ? 100 : opt[2];                  /* >=0.1s */
  buffsize = opt[3] < 4096 ? 4096 : opt[3];              /* >=4096byte */
  fswapmargin = opt[4] < 0 ? 0 : opt[4];
}
/* Set timeout time ------------------------------------------------------------
 * Set timeout time
 * Args   : stream_t *stream I   stream (STR_TCPCLI,STR_NTRIPCLI,STR_NTRIPSVR)
 *          int     toinact  I   inactive timeout (ms) (0: no timeout)
 *          int     tirecon  I   reconnect interval (ms) (0: no reconnect)
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsettimeout(stream_t *stream, int toinact, int tirecon) {
  tracet(3, "strsettimeout: toinact=%d tirecon=%d\n", toinact, tirecon);

  tcpcli_t *tcpcli;
  if (stream->type == STR_TCPCLI) {
    tcpcli = (tcpcli_t *)stream->port;
  } else if (stream->type == STR_NTRIPCLI || stream->type == STR_NTRIPSVR) {
    tcpcli = ((ntrip_t *)stream->port)->tcp;
  } else
    return;

  tcpcli->toinact = toinact;
  tcpcli->tirecon = tirecon;
}
/* Set local directory ---------------------------------------------------------
 * Set local directory path for FTP/HTTP download
 * Args   : char   *dir      I   directory for download files
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsetdir(const char *dir) {
  tracet(3, "strsetdir: dir=%s\n", dir);

  rtkstrcpy(localdir, sizeof(localdir), dir);
}
/* Set HTTP/NTRIP proxy address ------------------------------------------------
 * Set HTTP/NTRIP proxy address
 * Args   : char   *addr     I   HTTP/NTRIP proxy address <address>:<port>
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsetproxy(const char *addr) {
  tracet(3, "strsetproxy: addr=%s\n", addr);

  rtkstrcpy(proxyaddr, sizeof(proxyaddr), addr);
}
/* Get stream time -------------------------------------------------------------
 * Get stream time
 * Args   : stream_t *stream I   stream
 * Return : current time or replay time for playback file
 *----------------------------------------------------------------------------*/
extern gtime_t strgettime(stream_t *stream) {
  file_t *file;
  if (stream->type == STR_FILE && (stream->mode & STR_MODE_R) && (file = (file_t *)stream->port)) {
    return timeadd(file->time, file->start); /* Replay start time */
  }
  return utc2gpst(timeget());
}
/* Send NMEA request -----------------------------------------------------------
 * Send NMEA gpgga message to stream
 * Args   : stream_t *stream I   stream
 *          sol_t *sol       I   solution
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsendnmea(stream_t *stream, const sol_t *sol) {
  tracet(3, "strsendnmea: rr=%.3Lf %.3Lf %.3Lf\n", sol->rr[0], sol->rr[1], sol->rr[2]);

  uint8_t buff[1024];
  outnmea_gga((char *)buff, sizeof(buff), sol);
  strwrite(stream, buff, strlen((char *)buff));
}
/* Generate general hex message ----------------------------------------------*/
static int gen_hex(const char *msg, uint8_t *buff, size_t size) {
  trace(4, "gen_hex: msg=%s\n", msg);

  char mbuff[1024];
  rtkstrcpy(mbuff, sizeof(mbuff), msg);
  char *args[256], *r;
  int narg = 0;
  for (char *p = strtok_r(mbuff, " ", &r); p && narg < 256; p = strtok_r(NULL, " ", &r)) {
    args[narg++] = p;
  }
  size_t len = 0;
  for (int i = 0; i < narg; i++) {
    uint32_t byte;
    if (sscanf(args[i], "%x", &byte)) {
      RTKBOUNDSCHECK(buff, size, len);
      buff[len++] = (uint8_t)byte;
    }
  }
  return (int)len;
}
/* Set bitrate ---------------------------------------------------------------*/
static int set_brate(stream_t *str, int brate) {
  int type = str->type;
  if (type != STR_SERIAL) return 0;

  char path[FNSIZE];
  rtkstrcpy(path, sizeof(path), str->path);

  char *p = strchr(path, ':');
  if (!p) {
    rtkcatprintf(path, sizeof(path), ":%d", brate);
  } else {
    char buff[FNSIZE] = "";
    char *q = strchr(p + 1, ':');
    if (q) rtkstrcpy(buff, sizeof(buff), q);
    size_t i = p - path;
    rtksnprintf(path + i, sizeof(path) - i, ":%d%s", brate, buff);
  }
  int mode = str->mode;
  strclose(str);
  return stropen(str, type, mode, path);
}
/* Send receiver command -------------------------------------------------------
 * Send receiver commands to stream
 * Args   : stream_t *stream I   stream
 *          char   *cmd      I   receiver command strings
 * Return : none
 *----------------------------------------------------------------------------*/
extern void strsendcmd(stream_t *str, const char *cmd) {
  tracet(3, "strsendcmd: cmd=%s\n", cmd);

  const char *p = cmd;

  for (;;) {
    const char *q;
    for (q = p;; q++)
      if (*q == '\r' || *q == '\n' || *q == '\0') break;
    int n = (int)(q - p);
    char msg[1024];
    rtkesubstrcpy(msg, sizeof(msg), p, 0, n);

    if (!*msg || *msg == '#') { /* Null or comment */
      ;
    } else if (*msg == '!') { /* Binary escape */

      if (!strncmp(msg + 1, "WAIT", 4)) { /* Wait */
        int ms;
        if (sscanf(msg + 5, "%d", &ms) < 1) ms = 100;
        if (ms > 3000) ms = 3000; /* Max 3 s */
        sleepms(ms);
      } else if (!strncmp(msg + 1, "BRATE", 5)) { /* Set bitrate */
        int brate;
        if (sscanf(msg + 6, "%d", &brate) < 1) brate = 115200;
        set_brate(str, brate);
        sleepms(500);
      } else if (!strncmp(msg + 1, "UBX", 3)) { /* Ublox */
        uint8_t buff[1024];
        int m = gen_ubx(msg + 4, buff, sizeof(buff));
        if (m > 0) strwrite(str, buff, m);
      } else if (!strncmp(msg + 1, "STQ", 3)) { /* Skytraq */
        uint8_t buff[1024];
        int m = gen_stq(msg + 4, buff, sizeof(buff));
        if (m > 0) strwrite(str, buff, m);
      } else if (!strncmp(msg + 1, "NVS", 3)) { /* Nvs */
        uint8_t buff[1024];
        int m = gen_nvs(msg + 4, buff, sizeof(buff));
        if (m > 0) strwrite(str, buff, m);
      } else if (!strncmp(msg + 1, "HEX", 3)) { /* General hex message */
        uint8_t buff[1024];
        int m = gen_hex(msg + 4, buff, sizeof(buff));
        if (m > 0) strwrite(str, buff, m);
      }
    } else {
      const char cmdend[] = "\r\n";
      rtkstrcat(msg, sizeof(msg), cmdend);
      strwrite(str, (uint8_t *)msg, n + 2);
    }
    if (*q == '\0')
      break;
    else
      p = q + 1;
  }
}
