//------------------------------------------------------------------------------
// stream.c : stream input/output functions
//
//          Copyright (C) 2008-2020 by T.TAKASU, All rights reserved.
//
// Options : -DWIN32    use WIN32 API
//           -DSVR_REUSEADDR reuse TCP server address
//
// References :
//     [1] RTCM Recommendaed Standards for Networked Transport for RTCM via
//         Internet Protocol (NTRIP), Version 1.0, Semptember 30, 2004
//     [2] H.Niksic and others, GNU Wget 1.12, The non-iteractive download
//         utility, 4 September 2009
//     [3] RTCM Recommendaed Standards for Networked Transport for RTCM via
//         Internet Protocol (NTRIP), Version 2.0, June 28, 2011
//
// Version : $Revision:$ $Date:$
// History : 2009/01/16 1.0  new
//           2009/04/02 1.1  support NMEA request in NTRIP request
//                           support time-tag of file as stream
//           2009/09/04 1.2  ported to Linux environment
//                           add fflush() to save file stream
//           2009/10/10 1.3  support multiple connection for TCP server
//                           add keyword replacement in file path
//                           add function strsendnmea(), strsendcmd()
//           2010/07/18 1.4  support FTP/HTTP stream types
//                           add keywords replacement of %ha,%hb,%hc in path
//                           add api: strsetdir(),strsettimeout()
//           2010/08/31 1.5  reconnect after error of NTRIP client
//                           fix bug on no file swap at week start (2.4.0_p6)
//           2011/05/29 1.6  add fast stream replay mode
//                           add time margin to swap file
//                           change api strsetopt()
//                           introduce non_block send for send socket
//                           add api: strsetproxy()
//           2011/12/21 1.7  fix bug decode tcppath (rtklib_2.4.1_p5)
//           2012/06/09 1.8  fix problem if user or password contains /
//                           (rtklib_2.4.1_p7)
//           2012/12/25 1.9  compile option SVR_REUSEADDR added
//           2013/03/10 1.10 fix problem with NTRIP mountpoint containing "/"
//           2013/04/15 1.11 fix bug on swapping files if swapmargin=0
//           2013/05/28 1.12 fix bug on playback of file with 64 bit size_t
//           2014/05/23 1.13 retry to connect after gethostbyname() error
//                           fix bug on malloc size in openftp()
//           2014/06/21 1.14 add general hex message rcv command by !HEX ...
//           2014/10/16 1.15 support stdin/stdout for input/output from/to file
//           2014/11/08 1.16 fix getconfig error (87) with bluetooth device
//           2015/01/12 1.15 add rcv command to change bitrate by !BRATE
//           2016/01/16 1.16 add constant CRTSCTS for non-CRTSCTS-defined env.
//                           fix serial status for non-windows systems
//           2016/06/09 1.17 fix bug on !BRATE rcv command always failed
//                           fix program on struct alignment in time tag header
//           2016/06/21 1.18 reverse time-tag handler of file to previous
//           2016/07/23 1.19 add output of received stream to TCP port for serial
//           2016/08/20 1.20 modify api strsendnmea()
//           2016/08/29 1.21 fix bug on starting serial thread for windows
//           2016/09/03 1.22 add NTRIP caster functions
//                           add api strstatx(),strsetsrctbl()
//                           add api strsetsel(),strgetsel()
//           2016/09/06 1.23 fix bug on NTRIP caster socket and request handling
//           2016/09/27 1.24 support UDP server and client
//           2016/10/10 1.25 support ::P={4|8} option in path for STR_FILE
//           2018/11/05 1.26 fix bug on default playback speed (= 0)
//                           fix bug on file playback as slave mode
//                           fix bug on timeset() in GPST instead of UTC
//                           update trace levels and buffer sizes
//           2019/05/10 1.27 fix bug on dropping message on TCP stream (#144)
//           2019/08/19 1.28 support 460800 and 921600 bps for serial
//           2020/11/30 1.29 delete API strsetsrctbl(), strsetsel(), strgetsel()
//                           fix bug on numerical error in computing output rate
//                           no support stream type STR_NTRIPC_S in API stropen()
//                           no support rcv. command LEXR in API strsendcmd()
//                           change stream type STR_NTRIPC_C to STR_NTRIPCAS
//                           accept HTTP/1.1 as protocol for NTRIP caster
//                           suppress warning for buffer overflow by sprintf()
//                           use integer types in stdint.h
//------------------------------------------------------------------------------
#define _POSIX_C_SOURCE 200112L

#ifndef WIN32
// For 64 bit file offsets on Linux and MacOS.
#define _FILE_OFFSET_BITS 64
#endif

#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <ctype.h>
#include <inttypes.h>

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

#ifdef RTKLIB_OPENSSL
#include <openssl/err.h>
#include <openssl/ssl.h>
#endif

// Constants -------------------------------------------------------------------

#define TINTACT 200               // Period for stream active (ms)
#define SERIBUFFSIZE 4096         // Serial buffer size (bytes)
#define TIMETAGH_LEN 64           // Time tag file header length
#define MAXCLI 32                 // Max client connection for TCP svr
#define DEFAULT_MEMBUF_SIZE 4096  // Default memory buffer size (bytes)

#define NTRIP_AGENT "RTKLIB/" VER_RTKLIB "_" PATCH_LEVEL
#define NTRIP_CLI_PORT 2101  // Default NTRIP-client connection port
// Some sites use port 2102 for https but many uses this port for http.
#define NTRIP_CLI_TLS_PORT 443  // Default NTRIP-client TLS connection port
#define NTRIP_SVR_PORT 80       // Default NTRIP-server connection port
#define NTRIP_SVR_TLS_PORT 443  // Default NTRIP-server TLS connection port
#define NTRIP_MAXSTR 256        // Max length of mountpoint string

#define FTP_CMD "wget"  // FTP/HTTP command
#define FTP_TIMEOUT 30  // FTP/HTTP timeout (s)

#define MIN(x, y) ((x) < (y) ? (x) : (y))

// Macros --------------------------------------------------------------------

#ifdef WIN32
#define dev_t HANDLE
#define socket_t SOCKET
typedef int socklen_t;
#define PRIDEV "p"
#define PRISOCK "llu"
#define FNOINHERIT "N"
#else
#define dev_t int
#define socket_t int
#define closesocket close
#define PRIDEV "d"
#define PRISOCK "d"
#define FNOINHERIT "e"
#endif

// Type definition -----------------------------------------------------------

typedef struct {              // File control type
  FILE *fp;                   // File pointer
  FILE *fp_tag;               // File pointer of tag file
  FILE *fp_tmp;               // Temporary file pointer for swap
  FILE *fp_tag_tmp;           // Temporary file pointer of tag file for swap
  char path[MAXSTRPATH];      // File path
  char openpath[MAXSTRPATH];  // Open file path
  unsigned mode;              // File mode
  unsigned timetag;           // Time tag flag (0:off,1:on)
  unsigned repmode;           // Replay mode (0:master,1:slave)
  int offset;                 // Time offset (ms) for slave
  unsigned size_fpos;         // File position size (bytes)
  gtime_t time;               // Start time
  gtime_t wtime;              // Write time
  uint32_t tick;              // Start tick
  uint32_t tick_f;            // Start tick in file
  int64_t fpos_n;  // Next file position
  uint32_t tick_n;     // Next tick
  double start;        // Start offset (s)
  double speed;        // Replay speed (time factor)
  double swapintv;     // Swap interval (hr) (0: no swap)
  rtklib_lock_t lock;  // Lock flag
} file_t;

typedef struct {           // TCP control type
  unsigned state;          // State (0:closed,1:waiting,2:connected)
  char saddr[256];         // Address string
  unsigned port;           // Port
  struct addrinfo *addrs;  // Addresses, from getaddrinfo, to be freed or reused.
  struct addrinfo *addrr;  // Remaining addresses. The head is the current.
  socket_t sock;           // Socket descriptor
  unsigned want;           // Socket wait reason: 0 none, 1 read, 2 write.
  int tcon;                // Reconnect time (ms) (-1:never,0:now)
  uint32_t tact;           // Data active tick
  uint32_t tdis;           // Disconnect tick
  unsigned retry;          // Quick connection retry flag.
  unsigned tlsp;           // TLS stream: 1: yes, 0: no.
  unsigned tlsverify;      // TLS Verify peer: 1: yes, 0: no.
  unsigned tlsstate;       // TLS sub-states: 0: Uninitialized, 1: Connect, 2: Connected.
#ifdef RTKLIB_OPENSSL
  SSL_CTX *ctx;
  SSL *ssl;
#endif
} tcp_t;

typedef struct tcpsvr_tag {  // TCP server type
  tcp_t svr;                 // TCP server control
  tcp_t cli[MAXCLI];         // TCP client controls
  size_t obufsize;           // Size of the output buffers.
  uint8_t *obuf[MAXCLI];     // Output buffers.
  size_t nout[MAXCLI];       // Output available in the buffer.
} tcpsvr_t;

typedef struct {     // TCP client type
  tcp_t svr;         // TCP server control
  unsigned toinact;  // Inactive timeout (ms) (0:no timeout)
  int tirecon;       // Reconnect interval (ms) (-1:no reconnect)
} tcpcli_t;

typedef struct {  // Serial control type
  dev_t dev;      // Serial device
  int error;      // Error state
#ifndef WIN32
  unsigned want;  // Wait reason: 0 none, 1 read, 2 write.
#endif
#ifdef WIN32
  int state;           // State
  size_t wp, rp;       // Write/read pointer
  size_t buffsize;     // Write buffer size (bytes)
  HANDLE thread;       // Write thread
  rtklib_lock_t lock;  // Lock flag
  uint8_t *buff;       // Write buffer
#endif
  tcpsvr_t *tcpsvr;  // TCP server for received stream
} serial_t;

typedef struct {           // NTRIP control type
  unsigned ver;            // NTRIP version: 1 or 2.
  unsigned type;           // Type (0:server,1:client)
  unsigned state;          // State (0:close,1:wait,2:connect)
  char url[MAXSTRPATH];    // URL for proxy
  char mntpnt[256];        // Mountpoint
  char user[256];          // User
  char passwd[256];        // Password
  char str[NTRIP_MAXSTR];  // Mountpoint string for server
  size_t ibufsize;         // Size of the input buffer.
  uint8_t *ibuf;           // Input buffer. e.g. for responses.
  size_t nin;              // Input available in the buffer.
  unsigned chunkin;        // Chunk transfer encoding of input data.
  ssize_t chunkrem;   // Remaining content to read from a chunk. (-1: header, 0: tail, 1+ content)
  size_t obufsize;    // Size of the output buffer.
  uint8_t *obuf;      // Output buffer. e.g. for requests.
  size_t nout;        // Output available in the buffer.
  unsigned chunkout;  // Chunk transfer encoding of output data.
  tcpcli_t *tcp;      // TCP client
} ntrip_t;

typedef struct {  // NTRIP client/server connection type
  unsigned ver;   // NTRIP client version: 1 or 2.
  // This state manages the HTTP layer, after the tcp_t layer is connected.
  unsigned state;     // State (0:close,1:established)
  unsigned substate;  // Request and response state.
  char mntpnt[256];   // Mountpoint
  char str[NTRIP_MAXSTR];  // Mountpoint string for server
  size_t ibufsize;         // Size of the input buffer.
  uint8_t *ibuf;           // Input buffer. e.g. for requests.
  size_t nin;              // Input available in the buffer.
  unsigned chunkin;        // Chunk transfer encoding of input data.
  ssize_t chunkrem;   // Remaining content to read from a chunk. (-1: header, 0: tail, 1+ content)
  size_t obufsize;    // Size of the output buffer.
  uint8_t *obuf;      // Output buffer. e.g. for responses.
  size_t nout;        // Output available in the buffer.
  unsigned chunkout;  // Check transfer encoding on output.
} ntripc_con_t;

typedef struct {              // NTRIP caster control type
  unsigned ver;               // NTRIP version: 0:either, 1: only 1, 2: only 2.
  unsigned type;              // Type: 0:either, 1:clients only, 2:sources only */
  unsigned state;             // State: 0:close, 1:wait, 2:connect
  char mntpnt[256];           // Mountpoint
  char user[256];             // User
  char passwd[256];           // Password
  char srctbl[NTRIP_MAXSTR];  // Source table
  tcpsvr_t *tcp;              // TCP server
  ntripc_con_t con[MAXCLI];   // NTRIP client/server connections
} ntripc_t;

typedef struct {                 // UDP type
  unsigned state;                // State (0:close,1:open)
  unsigned type;                 // Type (0:server,1:client)
  unsigned port;                 // Port
  char saddr[256];               // Address (server:filter,client:server)
  struct sockaddr_storage addr;  // Address resolved
  socklen_t addrlen;
  socket_t sock;  // Socket descriptor
  unsigned want;  // Socket wait reason: 0 none, 1 read, 2 write.
} udp_t;

typedef struct {           // FTP download control type
  unsigned state;          // State (0:close,1:download,2:complete,3:error)
  unsigned proto;          // Protocol (0:ftp,1:http)
  int error;               // Error code (0:no error,1-10:wget error,
                           //            11:no temp dir,12:uncompact error)
  char addr[MAXSTRPATH];   // Download address
  char file[MAXSTRPATH];   // Download file path
  char user[256];          // User for FTP
  char passwd[256];        // Password for FTP
  char local[MAXSTRPATH];  // Local file path
  int topts[4];            // Time options {poff,tint,toff,tretry} (s)
  gtime_t tnext;           // Next retry time (GPST)
  rtklib_thread_t thread;  // Download thread
} ftp_t;

typedef struct {       // Memory buffer type
  int state;           // State
  size_t wp, rp;       // Write/read pointer
  size_t bufsize;      // Buffer size (bytes)
  rtklib_lock_t lock;  // Lock flag
  uint8_t *buf;        // Write buffer
} membuf_t;

typedef struct {
  fd_set rs, ws;       // Read and write wait.
#ifndef WIN32
  int nfds;            // Number of file descriptors used.
#endif
} wantset_t;

void *strwantalloc(void) {
  wantset_t *wantset = (wantset_t *)malloc(sizeof(wantset_t));
  return (void *)wantset;
}
void strwantfree(void *wantset) {
  free(wantset);
}
void strwantinit(void *wantset) {
  FD_ZERO(&((wantset_t *)wantset)->rs);
  FD_ZERO(&((wantset_t *)wantset)->ws);
#ifndef WIN32
  ((wantset_t *)wantset)->nfds = 0;
#endif
}
int strwantwait(void *wantset, unsigned ms) {
  struct timeval tv = {0};
#ifdef WIN32
  tv.tv_sec = (long)(ms / 1000);
  tv.tv_usec = (long)((ms % 1000) * 1000);
  int ret = select(0, &((wantset_t *)wantset)->rs, &((wantset_t *)wantset)->ws, NULL, &tv);
#else
  tv.tv_sec = (time_t)(ms / 1000);
  tv.tv_usec = (suseconds_t)((ms % 1000) * 1000);
  int ret = select(((wantset_t *)wantset)->nfds, &((wantset_t *)wantset)->rs, &((wantset_t *)wantset)->ws, NULL, &tv);
#endif
  trace(5, "strwantwait ret=%d\n", ret);
  return ret;
}

// Proto types for static functions ------------------------------------------

static tcpsvr_t *opentcpsvr(const char *path, char *msg, size_t msize);
static void closetcpsvr(tcpsvr_t *tcpsvr);
static size_t writetcpsvr(tcpsvr_t *tcpsvr, const uint8_t *buff, size_t size, size_t n, char *msg,
                          size_t msize);

// Global options ------------------------------------------------------------

static unsigned toinact = 10000;              // Inactive timeout (ms)
static int ticonnect = 10000;                 // Interval to re-connect (ms), -1 never.
static unsigned tirate = 1000;                // Averaging time for data rate (ms)
static size_t buffsize = 32768;               // Receive/send buffer size (bytes)
static char localdir[MAXSTRPATH] = "";        // Local directory for FTP/HTTP
static char proxyaddr[256] = "";              // HTTP/NTRIP/FTP proxy address
static uint32_t tick_master = 0;              // Time tick master for replay
static unsigned fswapmargin = 30;             // File swap margin (s)
static unsigned ntripcliver = 1;              // Default NTRIP client version.
static char tlssvrcertfile[MAXSTRPATH] = "";  // TLS server certificate file.
static char tlssvrkeyfile[MAXSTRPATH] = "";   // TLS server certificate private key file.
static char tlssvrcafile[MAXSTRPATH] = "";    // TLS server CA file.
static char tlssvrcadir[MAXSTRPATH] = "";     // TLS server CA directory.
static char tlssvrpasswd[256] = "";           // Password for encrypted server files.
static unsigned tlssvrverify = 1;             // Enable TLS server peer verification.
static char tlsclicertfile[MAXSTRPATH] = "";  // TLS client certificate file.
static char tlsclikeyfile[MAXSTRPATH] = "";   // TLS client certificate private key file.
static char tlsclicafile[MAXSTRPATH] = "";    // TLS client CA file.
static char tlsclicadir[MAXSTRPATH] = "";     // TLS client CA directory.
static char tlsclipasswd[256] = "";           // Password for encrypted client files.
static unsigned tlscliverify = 1;             // Enable TLS client peer verification.

// Read/write serial buffer --------------------------------------------------
#ifdef WIN32
static size_t readseribuff(serial_t *serial, uint8_t *buff, size_t size, size_t nmax) {
  tracet(5, "readseribuff: dev=%" PRIDEV "\n", serial->dev);

  if (nmax == 0) return 0;
  RBOUNDSCHECK(buff, size, nmax - 1);

  rtklib_lock(&serial->lock);
  size_t ns;
  for (ns = 0; serial->rp != serial->wp && ns < nmax; ns++) {
    buff[ns] = serial->buff[serial->rp];
    if (++serial->rp >= serial->buffsize) serial->rp = 0;
  }
  rtklib_unlock(&serial->lock);
  tracet(5, "readseribuff: ns=%zu rp=%zu wp=%zu\n", ns, serial->rp, serial->wp);
  return ns;
}
static size_t writeseribuff(serial_t *serial, const uint8_t *buff, size_t size, size_t n) {
  tracet(5, "writeseribuff: dev=%" PRIDEV " n=%zu\n", serial->dev, n);

  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);

  rtklib_lock(&serial->lock);
  size_t ns;
  for (ns = 0; ns < n; ns++) {
    size_t wp = serial->wp;
    serial->buff[wp] = buff[ns];
    if (++wp >= serial->buffsize) wp = 0;
    if (wp != serial->rp)
      serial->wp = wp;
    else {
      tracet(2, "serial buffer overflow: size=%zu\n", serial->buffsize);
      break;
    }
  }
  rtklib_unlock(&serial->lock);
  tracet(5, "writeseribuff: ns=%zu rp=%zu wp=%zu\n", ns, serial->rp, serial->wp);
  return ns;
}
#endif  // WIN32

// Write serial thread -------------------------------------------------------
#ifdef WIN32
static DWORD WINAPI serialthread(void *arg) {
  tracet(3, "serialthread:\n");

  serial_t *serial = (serial_t *)arg;

  for (;;) {
    uint32_t tick = tickget();
    size_t n;
    uint8_t buff[128];
    while ((n = readseribuff(serial, buff, sizeof(buff), sizeof(buff))) > 0) {
      DWORD ns;
      if (!WriteFile(serial->dev, buff, (DWORD)n, &ns, NULL)) serial->error = 1;
    }
    if (serial->state == 0) break;
    sleepms(10 - (int)(tickget() - tick));  // cycle=10ms
  }
  free(serial->buff);
  return 0;
}
#endif  // WIN32

// Open serial ---------------------------------------------------------------
static serial_t *openserial(const char *path, unsigned mode, char *msg, size_t msize) {
#ifdef WIN32
  const unsigned br[] = {300,   600,   1200,   2400,   4800,   9600,  19200,
                         38400, 57600, 115200, 230400, 460800, 921600};
#else
#ifdef __APPLE__
  // MacOS doesn't support higher baud rates (>230400B)
  const unsigned br[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
  const speed_t bs[] = {B300,   B600,   B1200,  B2400,   B4800,  B9600,
                        B19200, B38400, B57600, B115200, B230400};
#else   // Regular Linux with higher baud rates
  const unsigned br[] = {300,   600,   1200,   2400,   4800,   9600,  19200,
                         38400, 57600, 115200, 230400, 460800, 921600};
  const speed_t bs[] = {B300,   B600,   B1200,   B2400,   B4800,   B9600,  B19200,
                        B38400, B57600, B115200, B230400, B460800, B921600};
#endif  // Ifdef __APPLE__
#endif
  tracet(3, "openserial: path=%s mode=%u\n", path, mode);

  serial_t *serial = (serial_t *)calloc(1, sizeof(serial_t));
  if (serial == NULL) return NULL;

  char port[128], parity = 'N', fctr[64] = "";
  unsigned brate = 115200, bsize = 8, stopb = 1;
  ssize_t end = rsstrchr(path, 0, ':');
  if (end >= 0) {
    rsesubstrcpy(port, sizeof(port), path, 0, end);
    sscanf(path + end, ":%u:%u:%c:%u:%63s", &brate, &bsize, &parity, &stopb, fctr);
  } else {
    rsstrcpy(port, sizeof(port), path);
  }

  unsigned tcp_port = 0;
  ssize_t pi = rsstrchr(path, 0, '#');
  if (pi >= 0) sscanf(path + pi, "#%u", &tcp_port);
  size_t i;
  size_t nbr = sizeof(br) / sizeof(int);
  for (i = 0; i < nbr; i++)
    if (br[i] == brate) break;
  if (i >= nbr) {
    rscatprintf(msg, msize, "bitrate error (%d)", brate);
    tracet(1, "openserial: %s path=%s\n", msg, path);
    free(serial);
    return NULL;
  }
  parity = (char)toupper((int)parity);

  char dev[128];
#ifdef WIN32
  rssnprintf(dev, sizeof(dev), "\\\\.\\%s", port);
  DWORD rw = 0;
  if (mode & STR_MODE_R) rw |= GENERIC_READ;
  if (mode & STR_MODE_W) rw |= GENERIC_WRITE;

  serial->dev = CreateFile(dev, rw, 0, 0, OPEN_EXISTING, 0, NULL);
  if (serial->dev == INVALID_HANDLE_VALUE) {
    rscatprintf(msg, msize, "%s open error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s path=%s\n", msg, path);
    free(serial);
    return NULL;
  }
  DWORD siz = sizeof(COMMCONFIG);
  COMMCONFIG cc = {0};
  if (!GetCommConfig(serial->dev, &cc, &siz)) {
    rscatprintf(msg, msize, "%s getconfig error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s\n", msg);
    CloseHandle(serial->dev);
    free(serial);
    return NULL;
  }
  char dcb[64] = "";
  rssnprintf(dcb, sizeof(dcb), "baud=%d parity=%c data=%d stop=%d", brate, parity, bsize, stopb);
  if (!BuildCommDCB(dcb, &cc.dcb)) {
    rscatprintf(msg, msize, "%s builddcb error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s\n", msg);
    CloseHandle(serial->dev);
    free(serial);
    return NULL;
  }
  if (strcmp(fctr, "rts") == 0) cc.dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
  SetCommConfig(serial->dev, &cc, siz);      // Ignore error to support Novatel.
  COMMTIMEOUTS co = {MAXDWORD, 0, 0, 0, 0};  // Non-blocking-read
  SetCommTimeouts(serial->dev, &co);
  DWORD error;
  ClearCommError(serial->dev, &error, NULL);
  PurgeComm(serial->dev, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

  // Create write thread.
  rtklib_initlock(&serial->lock);
  serial->state = serial->error = 0;
  serial->wp = serial->rp = 0;
  serial->buffsize = buffsize;
  serial->buff = (uint8_t *)malloc(buffsize);
  if (serial->buff == NULL) {
    CloseHandle(serial->dev);
    free(serial);
    return NULL;
  }
  serial->state = 1;
  serial->thread = CreateThread(NULL, 0, serialthread, serial, 0, NULL);
  if (serial->thread == NULL) {
    rscatprintf(msg, msize, "%s serial thread error (%d)", port, (int)GetLastError());
    tracet(1, "openserial: %s\n", msg);
    CloseHandle(serial->dev);
    serial->state = 0;
    free(serial);
    return NULL;
  }
  rscatprintf(msg, msize, "%s", port);
#else
  rssnprintf(dev, sizeof(dev), "/dev/%.*s", (int)sizeof(port) - 6, port);

  unsigned rw = 0;
  if ((mode & STR_MODE_R) && (mode & STR_MODE_W))
    rw = O_RDWR;
  else if (mode & STR_MODE_R)
    rw = O_RDONLY;
  else if (mode & STR_MODE_W)
    rw = O_WRONLY;

  serial->dev = open(dev, rw | O_NOCTTY | O_NONBLOCK);
  if (serial->dev < 0) {
    rscatprintf(msg, msize, "%s open error (%d)", dev, errno);
    tracet(1, "openserial: %s dev=%s\n", msg, dev);
    free(serial);
    return NULL;
  }
  struct termios ios = {0};
  tcgetattr(serial->dev, &ios);
  ios.c_iflag = 0;
  ios.c_oflag = 0;
  ios.c_lflag = 0;     // Non-canonical.
  ios.c_cc[VMIN] = 0;  // Non-block-mode.
  ios.c_cc[VTIME] = 0;
  cfsetospeed(&ios, bs[i]);
  cfsetispeed(&ios, bs[i]);
  ios.c_cflag |= bsize == 7 ? CS7 : CS8;
  ios.c_cflag |= parity == 'O' ? (PARENB | PARODD) : (parity == 'E' ? PARENB : 0);
  ios.c_cflag |= stopb == 2 ? CSTOPB : 0;
  ios.c_cflag |= strcmp(fctr, "rts") == 0 ? CRTSCTS : 0;
  tcsetattr(serial->dev, TCSANOW, &ios);
  tcflush(serial->dev, TCIOFLUSH);
  rscatprintf(msg, msize, "%s", dev);
#endif
  serial->tcpsvr = NULL;

  // Open TCP server to output received stream.
  if (tcp_port > 0) {
    char path_tcp[32];
    rssnprintf(path_tcp, sizeof(path_tcp), ":%u", tcp_port);
    char msg_tcp[128] = "";
    serial->tcpsvr = opentcpsvr(path_tcp, msg_tcp, sizeof(msg_tcp));
  }
  tracet(3, "openserial: dev=%" PRIDEV "\n", serial->dev);
  return serial;
}
// Close serial --------------------------------------------------------------
static void closeserial(serial_t *serial) {
  tracet(3, "closeserial: dev=%" PRIDEV "\n", serial->dev);

  if (serial == NULL) return;
#ifdef WIN32
  serial->state = 0;
  WaitForSingleObject(serial->thread, 10000);
  CloseHandle(serial->dev);
  CloseHandle(serial->thread);
#else
  close(serial->dev);
#endif
  if (serial->tcpsvr) closetcpsvr(serial->tcpsvr);
  free(serial);
}
// Read serial ---------------------------------------------------------------
static size_t readserial(serial_t *serial, uint8_t *buff, size_t size, size_t n, char *msg,
                         size_t msize) {
  (void)msg;
  (void)msize;
  tracet(4, "readserial: dev=%" PRIDEV " n=%zu\n", serial->dev, n);
  if (serial == NULL) return 0;
  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);
#ifdef WIN32
  DWORD nrdw;
  if (!ReadFile(serial->dev, buff, (DWORD)n, &nrdw, NULL)) return 0;
  ssize_t nr = nrdw;
#else
  serial->want = 0;
  ssize_t nr = read(serial->dev, buff, n);
  if (nr < 0) {
    int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) serial->want = 1;  // Read wait.
    return 0;
  }
#endif
  tracet(5, "readserial: exit dev=%" PRIDEV " nr=%zd\n", serial->dev, nr);

  // Write received stream to TCP server port.
  if (serial->tcpsvr && nr > 0) {
    // Expect the write buffer to have room, otherwise connections are closed.
    char msg_tcp[128];
    writetcpsvr(serial->tcpsvr, buff, size, (size_t)nr, msg_tcp, sizeof(msg_tcp));
  }
  return (size_t)nr;
}
// Write serial --------------------------------------------------------------
static size_t writeserial(serial_t *serial, const uint8_t *buff, size_t size, size_t n, char *msg,
                          size_t msize) {
  (void)msg;
  (void)msize;
  tracet(4, "writeserial: dev=%" PRIDEV " n=%zu\n", serial->dev, n);

  if (serial == NULL) return 0;

  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);

#ifdef WIN32
  ssize_t ns = writeseribuff(serial, buff, size, n);
  if (ns < 0 || ns < (ssize_t)n) {
    serial->error = 1;
    ns = 0;
  }
#else
  serial->want = 0;
  ssize_t ns = write(serial->dev, buff, n);
  if (ns < 0) {
    int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
      serial->want = 2;  // Write wait.
      // This code is dependent on the serial stream system buffer being
      // larger enough, and this really needs an output buffer to hold the
      // data, and for that buffer to continue writing when possible.
      tracet(2, "writeserial: wait loss, dev=%" PRIDEV " n=%zu\n", serial->dev, n);
      return 0;
    }
    serial->error = 1;
    return 0;
  }
#endif
  tracet(5, "writeserial: exit dev=%" PRIDEV " ns=%zd\n", serial->dev, ns);
  return (size_t)ns;
}
static void wantserial(serial_t *serial, unsigned op, wantset_t *wantset) {
  tracet(4, "wantserial: op=%u\n", op);

  if (serial == NULL) return;

  if (serial->error != 0) return;

  // For Windows as seperate thread handles reads and write, and upper layers
  // are expected to poll for data.
#ifndef WIN32
  fd_set *rs = &wantset->rs, *ws = &wantset->ws;
  if (serial->want != 0) {
    if (serial->want & 1) FD_SET(serial->dev, rs);
    if (serial->want & 2) FD_SET(serial->dev, ws);
    int n = 1 + (int)serial->dev;
    if (n > wantset->nfds) wantset->nfds = n;
  }
  if (op != 0) {
    if (op & 1) FD_SET(serial->dev, rs);
    if (op & 2) FD_SET(serial->dev, ws);
    int n = 1 + (int)serial->dev;
    if (n > wantset->nfds) wantset->nfds = n;
  }
#endif
}
// Get state serial ----------------------------------------------------------
static int stateserial(const serial_t *serial) {
  return serial == NULL ? 0 : (serial->error ? -1 : 2);
}
// Get extended state serial -------------------------------------------------
static int statexserial(serial_t *serial, char *msg, size_t msize) {
  int state = serial == NULL ? 0 : (serial->error ? -1 : 2);
  rscatprintf(msg, msize, "serial:\n");
  rscatprintf(msg, msize, "  state   = %d\n", state);
  if (state == 0) return 0;
  rscatprintf(msg, msize, "  dev     = %" PRIDEV "\n", serial->dev);
  rscatprintf(msg, msize, "  error   = %d\n", serial->error);
#ifdef WIN32
  rscatprintf(msg, msize, "  buffsize= %zu\n", serial->buffsize);
  rscatprintf(msg, msize, "  wp      = %zu\n", serial->wp);
  rscatprintf(msg, msize, "  rp      = %zu\n", serial->rp);
#endif
  return state;
}
// Open file -----------------------------------------------------------------
static int openfile_(file_t *file, gtime_t time, char *msg, size_t msize) {
  char tstr[40];
  tracet(3, "openfile_: path=%s time=%s\n", file->path, time2str(time, tstr, 0));

  file->time = utc2gpst(timeget());
  file->tick = file->tick_f = tickget();
  file->fpos_n = 0;
  file->tick_n = 0;

  // Use stdin or stdout if file path is null.
  if (file->path[0] == '\0') {
    file->fp = file->mode & STR_MODE_R ? stdin : stdout;
    return 1;
  }
  // Replace keywords.
  reppath(file->path, file->openpath, time, "", "");

  // Create directory.
  if ((file->mode & STR_MODE_W) && !(file->mode & STR_MODE_R)) createdir(file->openpath);
  const char *rw;
  if (file->mode & STR_MODE_R)
    rw = "rb" FNOINHERIT;
  else
    rw = "wb" FNOINHERIT;

  file->fp = fopen(file->openpath, rw);
  if (file->fp == NULL) {
    rssnprintf(msg, msize, "file open error: %s", file->openpath);
    tracet(1, "openfile: %s\n", msg);
    return 0;
  }
  tracet(4, "openfile_: open file %s (%s)\n", file->openpath, rw);

  char tagpath[MAXSTRPATH + 4] = "";
  rssnprintf(tagpath, sizeof(tagpath), "%s.tag", file->openpath);

  if (file->timetag) {  // Output/sync time-tag.

    file->fp_tag = fopen(tagpath, rw);
    if (file->fp_tag == NULL) {
      rssnprintf(msg, msize, "tag open error: %s", tagpath);
      tracet(1, "openfile: %s\n", msg);
      fclose(file->fp);
      return 0;
    }
    tracet(4, "openfile_: open tag file %s (%s)\n", tagpath, rw);

    if (file->mode & STR_MODE_R) {
      char tagh[TIMETAGH_LEN + 1] = "";
      double time_sec;
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
      // Adjust time to read playback file
      timeset(gpst2utc(file->time));
    } else {
      char tagh[TIMETAGH_LEN + 1] = "";
      double time_sec;
      uint32_t time_time;
      rssnprintf(tagh, sizeof(tagh), "TIMETAG RTKLIB %s", VER_RTKLIB);
      memcpy(tagh + TIMETAGH_LEN - 4, &file->tick_f, sizeof(file->tick_f));
      time_time = (uint32_t)file->time.time;
      time_sec = file->time.sec;
      fwrite(&tagh, 1, TIMETAGH_LEN, file->fp_tag);
      fwrite(&time_time, 1, sizeof(time_time), file->fp_tag);
      fwrite(&time_sec, 1, sizeof(time_sec), file->fp_tag);
      // Time tag file structure.
      //   HEADER(60)+TICK(4)+TIME(4+8)+
      //   TICK0(4)+FPOS0(4/8)+
      //   TICK1(4)+FPOS1(4/8)+...
    }
  } else if (file->mode & STR_MODE_W) {  // Remove time-tag.
    FILE *fp = fopen(tagpath, "rb");
    if (fp) {
      fclose(fp);
      remove(tagpath);
    }
  }
  return 1;
}
// Close file ----------------------------------------------------------------
static void closefile_(file_t *file) {
  tracet(3, "closefile_: path=%s\n", file->path);

  if (file->fp) fclose(file->fp);
  if (file->fp_tag) fclose(file->fp_tag);
  if (file->fp_tmp) fclose(file->fp_tmp);
  if (file->fp_tag_tmp) fclose(file->fp_tag_tmp);
  file->fp = file->fp_tag = file->fp_tmp = file->fp_tag_tmp = NULL;

  // Reset time offset.
  timereset();
}
// Open file (path=filepath[::T[::+<off>][::x<speed>]][::S=swapintv][::P={4|8}]
static file_t *openfile(const char *path, unsigned mode, char *msg, size_t msize) {
  tracet(3, "openfile: path=%s mode=%u\n", path, mode);

  if ((mode & (STR_MODE_R | STR_MODE_W)) == 0) return NULL;

  // File options
  double speed = 1.0, start = 0.0, swapintv = 0.0;
  unsigned timetag = 0;
  unsigned size_fpos = 4;  // Default 4B.
  for (ssize_t pi = 0; (pi = rsstrstr(path, pi, "::")) >= 0; pi += 2) {
    if (path[pi + 2] == 'T')
      timetag = 1;
    else if (path[pi + 2] == '+')
      sscanf(path + pi + 2, "+%lf", &start);
    else if (path[pi + 2] == 'x')
      sscanf(path + pi + 2, "x%lf", &speed);
    else if (path[pi + 2] == 'S')
      sscanf(path + pi + 2, "S=%lf", &swapintv);
    else if (path[pi + 2] == 'P')
      sscanf(path + pi + 2, "P=%u", &size_fpos);
  }
  if (start <= 0.0) start = 0.0;
  if (swapintv <= 0.0) swapintv = 0.0;

  file_t *file = (file_t *)malloc(sizeof(file_t));
  if (file == NULL) return NULL;
  memset(file, 0, sizeof(file_t));

  file->fp = file->fp_tag = file->fp_tmp = file->fp_tag_tmp = NULL;
  rsstrcpy(file->path, sizeof(file->path), path);
  ssize_t pi = rsstrstr(file->path, 0, "::");
  if (pi >= 0) file->path[pi] = '\0';
  file->openpath[0] = '\0';
  file->mode = mode;
  file->timetag = timetag;
  file->repmode = 0;
  file->offset = 0;
  file->size_fpos = size_fpos;
  gtime_t time0 = {0};
  file->time = file->wtime = time0;
  file->tick = file->tick_f = file->tick_n = 0;
  file->fpos_n = 0;
  file->start = start;
  file->speed = speed;
  file->swapintv = swapintv;
  rtklib_initlock(&file->lock);

  gtime_t time = utc2gpst(timeget());

  // Open new file.
  if (!openfile_(file, time, msg, msize)) {
    free(file);
    return NULL;
  }
  return file;
}
// Close file ----------------------------------------------------------------
static void closefile(file_t *file) {
  if (file == NULL) return;
  tracet(3, "closefile: fp=%p\n", (void *)file->fp);
  closefile_(file);
  free(file);
}
// Open new swap file --------------------------------------------------------
static void swapfile(file_t *file, gtime_t time, char *msg, size_t msize) {
  char tstr[40];
  tracet(3, "swapfile: fp=%p time=%s\n", (void *)file->fp, time2str(time, tstr, 0));

  // Return if old swap file open.
  if (file->fp_tmp || file->fp_tag_tmp) return;

  // Check path of new swap file.
  char openpath[MAXSTRPATH];
  reppath(file->path, openpath, time, "", "");

  if (strcmp(openpath, file->openpath) == 0) {
    tracet(2, "swapfile: no need to swap %s\n", openpath);
    return;
  }
  // Save file pointer to temporary pointer.
  file->fp_tmp = file->fp;
  file->fp_tag_tmp = file->fp_tag;

  // Open new swap file.
  openfile_(file, time, msg, msize);
}
// Close old swap file -------------------------------------------------------
static void swapclose(file_t *file) {
  tracet(3, "swapclose: fp_tmp=%p\n", (void *)file->fp_tmp);

  if (file->fp_tmp) fclose(file->fp_tmp);
  if (file->fp_tag_tmp) fclose(file->fp_tag_tmp);
  file->fp_tmp = file->fp_tag_tmp = NULL;
}
// Get state file ------------------------------------------------------------
static int statefile(const file_t *file) { return file ? 2 : 0; }
// Get extended state file ---------------------------------------------------
static int statexfile(const file_t *file, char *msg, size_t msize) {
  unsigned state = file ? 2 : 0;
  rscatprintf(msg, msize, "file:\n");
  rscatprintf(msg, msize, "  state   = %u\n", state);
  if (state == 0) return 0;
  char tstr1[40], tstr2[40];
  time2str(file->time, tstr1, 3);
  time2str(file->wtime, tstr2, 3);
  rscatprintf(msg, msize, "  path    = %s\n", file->path);
  rscatprintf(msg, msize, "  openpath= %s\n", file->openpath);
  rscatprintf(msg, msize, "  mode    = %u\n", file->mode);
  rscatprintf(msg, msize, "  timetag = %u\n", file->timetag);
  rscatprintf(msg, msize, "  repmode = %u\n", file->repmode);
  rscatprintf(msg, msize, "  offset  = %d\n", file->offset);
  rscatprintf(msg, msize, "  time    = %s\n", tstr1);
  rscatprintf(msg, msize, "  wtime   = %s\n", tstr2);
  rscatprintf(msg, msize, "  tick    = %u\n", file->tick);
  rscatprintf(msg, msize, "  tick_f  = %u\n", file->tick_f);
  rscatprintf(msg, msize, "  start   = %.3f\n", file->start);
  rscatprintf(msg, msize, "  speed   = %.3f\n", file->speed);
  rscatprintf(msg, msize, "  swapintv= %.3f\n", file->swapintv);
  return state;
}
static int64_t xftell(FILE *stream)
{
#ifdef WIN32
  return _ftelli64(stream);
#else
  return ftello(stream);
#endif
}
static int xfseek(FILE *stream, int64_t offset, int whence)
{
#ifdef WIN32
  return _fseeki64(stream, offset, whence);
#else
  return fseeko(stream, offset, whence);
#endif
}
// Read file -----------------------------------------------------------------
static ssize_t readfile(file_t *file, uint8_t *buff, size_t size, size_t nmax, char *msg,
                        size_t msize) {
  if (file == NULL) return 0;
  tracet(4, "readfile: fp=%p nmax=%zu\n", (void *)file->fp, nmax);

  if (file->fp == stdin) {
#ifndef WIN32
    // Input from stdin.
    if (nmax == 0) return 0;
    RBOUNDSCHECK(buff, size, nmax - 1);
    fd_set rs;
    // Input from stdin
    FD_ZERO(&rs);
    FD_SET(0, &rs);
    struct timeval tv = {0};
    if (select(1, &rs, NULL, NULL, &tv) == 0) return 0;
    ssize_t nr = read(0, buff, nmax);
    if (nr < 0) return 0;
    return (size_t)nr;
#else
    return 0;
#endif
  }
  if (file->fp_tag) {
    uint32_t t, tick;
    // Target tick.
    if (file->repmode) {  // Slave.
      t = (uint32_t)(tick_master + file->offset);
    } else {  // Master.
      t = (uint32_t)((tickget() - file->tick) * file->speed + file->start * 1000.0);
      tick_master = t;
    }
    // Seek time-tag file to get next tick and file position.
    while ((int)(file->tick_n - t) <= 0) {
      uint32_t fpos_4B;
      uint64_t fpos_8B;
      if (fread(&file->tick_n, sizeof(tick), 1, file->fp_tag) < 1 ||
          fread((file->size_fpos == 4) ? (void *)&fpos_4B : (void *)&fpos_8B, file->size_fpos, 1,
                file->fp_tag) < 1) {
        file->tick_n = (uint32_t)(-1);
        int64_t pos = xftell(file->fp);
        xfseek(file->fp, 0, SEEK_END);
        file->fpos_n = xftell(file->fp);
        xfseek(file->fp, pos, SEEK_SET);
        break;
      }
      file->fpos_n = (file->size_fpos == 4) ? (int64_t)fpos_4B : (int64_t)fpos_8B;
    }
    if (file->tick_n == (uint32_t)(-1)) {
      rsstrcpy(msg, msize, "end");
    } else {
      rssnprintf(msg, msize, "T%+.1fs", (int)t * 0.001);
      file->wtime = timeadd(file->time, (int)t * 0.001);
      timeset(timeadd(gpst2utc(file->time), (int)file->tick_n * 0.001));
    }
    int64_t n = file->fpos_n - xftell(file->fp);
    if (n < 0)
      nmax = 0;  // Should be unreachable.
    else if ((size_t)n < nmax)
      nmax = (size_t)n;
  }
  size_t nr = 0;
  if (nmax > 0) {
    RBOUNDSCHECK(buff, size, nmax - 1);
    nr = fread(buff, 1, nmax, file->fp);
  }
  if (feof(file->fp)) rsstrcpy(msg, msize, "end");
  tracet(5, "readfile: fp=%p nr=%zu\n", (void *)file->fp, nr);
  return nr;
}
// Write file ----------------------------------------------------------------
static ssize_t writefile(file_t *file, const uint8_t *buff, size_t size, size_t n, char *msg,
                         size_t msize) {
  if (file == NULL) return 0;
  tracet(4, "writefile: fp=%p n=%zu\n", (void *)file->fp, n);

  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);

  gtime_t wtime = utc2gpst(timeget());  // Write time in GPST.

  // Swap writing file
  if (file->swapintv > 0.0 && file->wtime.time != 0) {
    double intv = file->swapintv * 3600.0;
    int week1;
    double tow1 = time2gpst(file->wtime, &week1);
    int week2;
    double tow2 = time2gpst(wtime, &week2);
    tow2 += 604800.0 * (week2 - week1);

    // Open new swap file.
    if (floor((tow1 + fswapmargin) / intv) < floor((tow2 + fswapmargin) / intv))
      swapfile(file, timeadd(wtime, fswapmargin), msg, msize);
    // Close old swap file
    if (floor((tow1 - fswapmargin) / intv) < floor((tow2 - fswapmargin) / intv)) swapclose(file);
  }
  if (file->fp == NULL) return 0;

  size_t ns = fwrite(buff, 1, n, file->fp);
  int64_t fpos = xftell(file->fp);
  int64_t fpos_tmp = 0;
  fflush(file->fp);
  file->wtime = wtime;

  if (file->fp_tmp) {
    fwrite(buff, 1, n, file->fp_tmp);
    fpos_tmp = xftell(file->fp_tmp);
    fflush(file->fp_tmp);
  }
  uint32_t tick = tickget();
  if (file->fp_tag) {
    tick -= file->tick;
    fwrite(&tick, 1, sizeof(tick), file->fp_tag);
    if (file->size_fpos == 4) {
      uint32_t fpos_4B = (uint32_t)fpos;
      fwrite(&fpos_4B, 1, sizeof(fpos_4B), file->fp_tag);
    } else {
      uint64_t fpos_8B = (uint64_t)fpos;
      fwrite(&fpos_8B, 1, sizeof(fpos_8B), file->fp_tag);
    }
    fflush(file->fp_tag);

    if (file->fp_tag_tmp) {
      fwrite(&tick, 1, sizeof(tick), file->fp_tag_tmp);
      if (file->size_fpos == 4) {
        uint32_t fpos_4B = (uint32_t)fpos_tmp;
        fwrite(&fpos_4B, 1, sizeof(fpos_4B), file->fp_tag_tmp);
      } else {
        uint64_t fpos_8B = (uint64_t)fpos_tmp;
        fwrite(&fpos_8B, 1, sizeof(fpos_8B), file->fp_tag_tmp);
      }
      fflush(file->fp_tag_tmp);
    }
  }
  tracet(5, "writefile: fp=%p ns=%zu tick=%5" PRIu32 " fpos=%jd\n", (void *)file->fp, ns, tick,
         (intmax_t)fpos);

  return ns;
}
// Sync files by time-tag ----------------------------------------------------
static void syncfile(file_t *file1, file_t *file2) {
  if (file1->fp_tag == NULL || file2->fp_tag == NULL) return;
  file1->repmode = 0;
  file2->repmode = 1;
  file2->offset = (int)(file1->tick_f - file2->tick_f);
}

// Escaping and unescaping support for pathname components -------------------
static char to_hex(char code) {
  const char hex[] = "0123456789ABCDEF";
  return hex[code & 15];
}
static int from_hex(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  return -1;
}
// URL escape a string into the destination.
void strurlescape(const char *src, size_t start, size_t n, char *dst, size_t dsize) {
  size_t j = 0, end = n >= SIZE_MAX - start ? SIZE_MAX : start + n;
  for (size_t i = start; i < end && src[i] != '\0'; i++) {
    char ch = src[i];
    if (isalnum((unsigned char)ch) || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
      RBOUNDSCHECK(dst, dsize, j);
      dst[j++] = ch;
      continue;
    }
    RBOUNDSCHECK(dst, dsize, j + 2);
    dst[j++] = '%';
    dst[j++] = to_hex(ch >> 4);
    dst[j++] = to_hex(ch & 15);
  }
  RBOUNDSCHECK(dst, dsize, j);
  dst[j] = '\0';
}
// URL unescape a string into the destination.
void strurlunescape(const char *src, size_t start, size_t n, char *dst, size_t dsize) {
  size_t i = start, j = 0, end = n >= SIZE_MAX - start ? SIZE_MAX : start + n;
  while (i < end && src[i] != '\0') {
    if (src[i] == '%') {
      if (src[i + 1] != '\0' && src[i + 2] != '\0') {
        int h1 = from_hex(src[i + 1]);
        int h2 = from_hex(src[i + 2]);
        if (h1 >= 0 && h2 >= 0) {
          RBOUNDSCHECK(dst, dsize, j);
          dst[j++] = (char)((h1 << 4) | h2);
          i += 3;
          continue;
        }
      }
    }
    dst[j++] = src[i++];
  }
  RBOUNDSCHECK(dst, dsize, j);
  dst[j] = '\0';
}

// Decode TCP/NTRIP path (path=[user[:passwd]@]addr[:port][/mntpnt[:str]])
// The path components are url escaped and returned unescaped.
// Note this ignores other suffix options.
static void decodetcppath(const char *path, char addr[256], char port[256], char user[256],
                          char passwd[256], char mntpnt[256], char str[NTRIP_MAXSTR]) {
  tracet(4, "decodetcpepath: path=%s\n", path);

  if (port) port[0] = '\0';
  if (user) user[0] = '\0';
  if (passwd) passwd[0] = '\0';
  if (mntpnt) mntpnt[0] = '\0';
  if (str) str[0] = '\0';

  char buff[MAXSTRPATH];
  rsstrcpy(buff, sizeof(buff), path);

  // Strip any trailing options.
  ssize_t pi = rsstrstr(buff, 0, "::");
  if (pi >= 0) buff[pi] = '\0';

  pi = rsstrrchr(buff, 0, '@');
  if (pi < 0) pi = 0;

  pi = rsstrchr(buff, pi, '/');
  if (pi >= 0) {
    ssize_t qi = rsstrchr(buff, pi + 1, ':');
    if (qi >= 0) {
      buff[qi] = '\0';
      if (str) strurlunescape(buff, qi + 1, SIZE_MAX, str, NTRIP_MAXSTR);
    }
    buff[pi] = '\0';
    if (mntpnt) strurlunescape(buff, pi + 1, SIZE_MAX, mntpnt, 256);
  }

  pi = rsstrrchr(buff, 0, '@');
  if (pi >= 0) {
    buff[pi++] = '\0';
    ssize_t qi = rsstrchr(buff, 0, ':');
    if (qi >= 0) {
      buff[qi] = '\0';
      if (passwd) strurlunescape(buff, qi + 1, SIZE_MAX, passwd, 256);
    }
    if (user) strurlunescape(buff, 0, SIZE_MAX, user, 256);
  } else {
    pi = 0;
  }

  ssize_t qi = rsstrchr(buff + pi, 0, ':');
  if (qi >= 0) {
    buff[pi + qi] = '\0';
    if (port) strurlunescape(buff, pi + qi + 1, SIZE_MAX, port, 256);
  }
  if (addr) strurlunescape(buff, pi, SIZE_MAX, addr, 256);
}
// Get socket error ----------------------------------------------------------
#ifdef WIN32
static int errsock(void) { return WSAGetLastError(); }
#else
static int errsock(void) { return errno; }
#endif

// Set socket option ---------------------------------------------------------
// Note if it returns 0 then it has closed the socket.
static int setsock(socket_t sock, char *msg, size_t msize) {
  tracet(3, "setsock: sock=%" PRISOCK "\n", sock);

#ifdef WIN32
  DWORD tv = 0;
#else
  struct timeval tv = {0};
#endif
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)) == -1 ||
      setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv)) == -1) {
    rssnprintf(msg, msize, "sockopt error: notimeo");
    tracet(1, "setsock: setsockopt error 1 sock=%" PRISOCK " err=%d\n", sock, errsock());
    closesocket(sock);
    return 0;
  }
  int bs = (int)buffsize;
  if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (const char *)&bs, sizeof(bs)) == -1 ||
      setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (const char *)&bs, sizeof(bs)) == -1) {
    tracet(1, "setsock: setsockopt error 2 sock=%" PRISOCK " err=%d bs=%d\n", sock, errsock(), bs);
    rscatprintf(msg, msize, "sockopt error: bufsiz");
  }
  int mode = 1;
  if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&mode, sizeof(mode)) == -1) {
    tracet(1, "setsock: setsockopt error 3 sock=%" PRISOCK " err=%d\n", sock, errsock());
    rscatprintf(msg, msize, "sockopt error: nodelay");
  }

  // Do not want apps to exec other processes that inherit these streams.
#ifdef WIN32
  SetHandleInformation((HANDLE)sock, HANDLE_FLAG_INHERIT, 0);
#else
  int flags = fcntl(sock, F_GETFD);
  if (flags != -1) fcntl(sock, F_SETFD, flags | FD_CLOEXEC);
#endif

  return 1;
}
// Non-blocking connect ---------------------------------------------------------
// Return: (-1: error, 0: not connected, 1: connected)
//
// Note the caller depends on -1 being returned if a connection attempt fails,
// so that it can back off before retrying. The caller should keep retrying if 0
// is returned.
static int connect_nb(socket_t sock, struct sockaddr *addr, socklen_t len, int *err) {
#ifdef WIN32
  u_long mode = 1;
  ioctlsocket(sock, FIONBIO, &mode);
  if (connect(sock, addr, len) == -1) {
    *err = errsock();
    if (*err == WSAEWOULDBLOCK || *err == WSAEINPROGRESS || *err == WSAEALREADY ||
        *err == WSAEINVAL)
      return 0;
    if (*err != WSAEISCONN) return -1;
  }
#else
  int flag = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flag | O_NONBLOCK);
  if (connect(sock, addr, len) == -1) {
    *err = errsock();
    if (*err == EINPROGRESS || *err == EALREADY) {
      *err = 0;
      return 0;
    }
    if (*err != EISCONN) return -1;
  }
#endif
  *err = 0;
  return 1;
}

#ifdef RTKLIB_OPENSSL
// Report and flush errors.
static void ssl_report_errors(void) {
  unsigned long err_code;
  while ((err_code = ERR_get_error()) != 0) {
    char err_buf[256];
    ERR_error_string_n(err_code, err_buf, sizeof(err_buf));
    tracet(3, "OpenSSL error: %s\n", err_buf);
  }
}
#endif

// Non-blocking receive ------------------------------------------------------
static ssize_t recv_nb(tcp_t *tcp, uint8_t *buff, size_t size, size_t start, size_t n, int *err) {
  if (n > 0) RBOUNDSCHECK(buff, size - start, n - 1);
  *err = 0;
  tcp->want = 0;  // Select() test below.
  if (tcp->tlsp) {
#ifdef RTKLIB_OPENSSL
    int sock = SSL_get_fd(tcp->ssl);
    if (sock < 0) return -1;
    if (!SSL_has_pending(tcp->ssl)) {
      int want = SSL_want(tcp->ssl);
      switch (want) {
        case SSL_NOTHING:
        case SSL_READING: {
          fd_set rs;
          FD_ZERO(&rs);
          FD_SET(sock, &rs);
          struct timeval tv = {0};
#ifdef WIN32
          int ret = select(0, &rs, NULL, NULL, &tv);
#else
          int ret = select(sock + 1, &rs, NULL, NULL, &tv);
#endif
          if (ret == 0) {
            tcp->want = 1;  // Read wait.
            return 0;
          }
          if (ret < 0) {
            *err = errsock();
            return ret;
          }
          break;
        }
        case SSL_WRITING: {
          fd_set ws;
          FD_ZERO(&ws);
          FD_SET(sock, &ws);
          struct timeval tv = {0};
#ifdef WIN32
          int ret = select(0, NULL, &ws, NULL, &tv);
#else
          int ret = select(sock + 1, NULL, &ws, NULL, &tv);
#endif
          if (ret == 0) {
            tcp->want = 2;  // Write wait.
            return 0;
          }
          if (ret < 0) {
            *err = errsock();
            return ret;
          }
          break;
        }
        default:
          tracet(2, "recv_nb tls: sock=%" PRISOCK " unexpected want=%d\n", tcp->sock, want);
          return -1;
      }
    }
    size_t nr;
    int res = SSL_read_ex(tcp->ssl, buff + start, n, &nr);
    if (res != 1) {
      int sslerr = SSL_get_error(tcp->ssl, res);
      if (sslerr == SSL_ERROR_WANT_READ) {
        tcp->want = 1;  // Read wait.
        tracet(5, "recv_nb tls: sock=%" PRISOCK " read want=%d\n", tcp->sock, sslerr);
        return 0;  // To be retried.
      }
      if (sslerr == SSL_ERROR_WANT_WRITE) {
        tcp->want = 2;  // Write wait.
        tracet(5, "recv_nb tls: sock=%" PRISOCK " read want=%d\n", tcp->sock, sslerr);
        return 0;  // To be retried.
      }
      if (sslerr == SSL_ERROR_ZERO_RETURN) {  // Graceful EOF.
        tracet(3, "recv_nb tls: eof n=%zu\n", n);
        return -1;
      }
      if (sslerr == SSL_ERROR_SYSCALL) *err = errsock();
      tracet(3, "recv_nb tls: sock=%" PRISOCK " unexpected tlserr=%d err=%d\n", tcp->sock, sslerr,
             *err);
      ssl_report_errors();
      return -1;
    }
    return nr;
#else
    tracet(1, "recv_nb: TLS not supported sock=%" PRISOCK "\n", tcp->sock);
    return -1;
#endif
  }

  socket_t sock = tcp->sock;
  fd_set rs;
  FD_ZERO(&rs);
  FD_SET(sock, &rs);
  struct timeval tv = {0};
#ifdef WIN32
  int ret = select(0, &rs, NULL, NULL, &tv);
#else
  int ret = select(sock + 1, &rs, NULL, NULL, &tv);
#endif
  if (ret < 0) {
    *err = errsock();
    return ret;
  }
  if (ret == 0) {
    tcp->want = 1;  // Read wait.
    *err = 0;
    return ret;
  }
#ifdef WIN32
  ssize_t nr = recv(sock, (char *)buff + start, (int)n, 0);
#else
  ssize_t nr = recv(sock, (char *)buff + start, n, 0);
#endif
  if (nr == 0) {
    *err = 0;
    // Orderly EOF.
    tracet(3, "recv_nb: sock=%" PRISOCK " eof \n", sock);
    return -1;
  }
  if (nr < 0) {
    *err = errsock();
#ifdef WIN32
    if (*err == WSAEWOULDBLOCK)
#else
    if (*err == EAGAIN || *err == EWOULDBLOCK || *err == EINTR)
#endif
    {
      *err = 0;
      tcp->want = 1;  // Write read.
      return 0;       // To be retried.
    }
    return -1;
  }
  *err = 0;
  return nr;
}

// Non-blocking send ------------------------------------------------------------
static ssize_t send_nb(tcp_t *tcp, const uint8_t *buff, size_t size, size_t start, size_t n,
                       int *err) {
  tracet(5, "send_nb: sock=%" PRISOCK " size=%zu start=%zu n=%zu\n", tcp->sock, size, start, n);
  if (n > 0) RBOUNDSCHECK(buff, size - start, n - 1);
  *err = 0;
  tcp->want = 0;  // Select() test below.
  if (tcp->tlsp) {
#ifdef RTKLIB_OPENSSL
    int sock = SSL_get_fd(tcp->ssl);
    int want = SSL_want(tcp->ssl);
    if (want != SSL_NOTHING)
      tracet(5, "send_nb tls: sock=%" PRISOCK " n=%zu want=%d\n", tcp->sock, n, want);
    switch (want) {
      case SSL_NOTHING:
        break;
      case SSL_WRITING: {
        fd_set ws;
        FD_ZERO(&ws);
        FD_SET(sock, &ws);
        struct timeval tv = {0};
#ifdef WIN32
        int ret = select(0, NULL, &ws, NULL, &tv);
#else
        int ret = select(sock + 1, NULL, &ws, NULL, &tv);
#endif
        if (ret == 0) {
          tcp->want = 2;  // Write wait.
          return 0;
        }
        if (ret < 0) {
          *err = errsock();
          return ret;
        }
        break;
      }
      case SSL_READING: {
        fd_set rs;
        FD_ZERO(&rs);
        FD_SET(sock, &rs);
        struct timeval tv = {0};
#ifdef WIN32
        int ret = select(0, &rs, NULL, NULL, &tv);
#else
        int ret = select(sock + 1, &rs, NULL, NULL, &tv);
#endif
        if (ret == 0) {
          tcp->want = 1;  // Read wait.
          return 0;
        }
        if (ret < 0) {
          *err = errsock();
          return ret;
        }
        break;
      }
      default:
        tracet(3, "send_nb tls: sock=%" PRISOCK " tls unexpected want=%d\n", tcp->sock, want);
        return -1;
    }
    size_t ns;
    int res = SSL_write_ex(tcp->ssl, buff + start, n, &ns);
    tracet(5, "send_nb tls: sock=%" PRISOCK " want=%d n=%zu res=%d ns=%zu err=%d\n", tcp->sock,
           want, n, res, ns, *err);
    if (res != 1) {
      int sslerr = SSL_get_error(tcp->ssl, res);
      if (sslerr == SSL_ERROR_WANT_READ) {
        tcp->want = 1;  // Read wait.
        tracet(5, "send_nb tls: sock=%" PRISOCK " write want read\n", tcp->sock);
        return 0;  // To be retried.
      }
      if (sslerr == SSL_ERROR_WANT_WRITE) {
        tcp->want = 2;  // Write wait.
        tracet(5, "send_nb tls: sock=%" PRISOCK " write want write\n", tcp->sock);
        return 0;  // To be retried.
      }
      if (sslerr == SSL_ERROR_ZERO_RETURN) {  // Graceful EOF.
        tracet(3, "send_nb tls: sock=%" PRISOCK " eof\n", tcp->sock);
        return -1;
      }
      if (sslerr == SSL_ERROR_SYSCALL) *err = errsock();
      tracet(3, "send_nb tls: sock=%" PRISOCK " unexpected sslerr=%d err=%d\n", tcp->sock, sslerr,
             *err);
      ssl_report_errors();
      return -1;
    }
    return ns;
#else
    tracet(1, "send_nb: sock=%" PRISOCK " TLS not supported\n", tcp->sock);
    return -1;
#endif
  }

  socket_t sock = tcp->sock;
  fd_set ws;
  FD_ZERO(&ws);
  FD_SET(sock, &ws);
  struct timeval tv = {0};
#ifdef WIN32
  int ret = select(0, NULL, &ws, NULL, &tv);
#else
  int ret = select(sock + 1, NULL, &ws, NULL, &tv);
#endif
  if (ret < 0) {
    *err = errsock();
    return ret;
  }
  if (ret == 0) {
    tcp->want = 2;  // Write wait.
    *err = 0;
    return ret;
  }
#ifdef WIN32
  ssize_t ns = send(sock, (char *)buff + start, (int)n, 0);
#else
  ssize_t ns = send(sock, (char *)buff + start, n, 0);
#endif
  if (ns < 0) {
    *err = errsock();
#ifdef WIN32
    if (*err == WSAEWOULDBLOCK)
#else
    if (*err == EAGAIN || *err == EWOULDBLOCK || *err == EINTR)
#endif
    {
      *err = 0;
      tcp->want = 2;  // Write wait.
      return 0;       // To be retried.
    }
    return -1;
  }
  *err = 0;
  return ns;
}

// Generate server TCP socket ------------------------------------------------
static unsigned gentcpsvr(tcp_t *tcp, char *msg, size_t msize) {
  tracet(3, "gentcpsvr: addr='%s' port=%u\n", tcp->saddr, tcp->port);

  char portstr[6];
  rssnprintf(portstr, sizeof(portstr), "%u", tcp->port);

  struct addrinfo hints, *res = NULL;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET6;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_NUMERICSERV | AI_ADDRCONFIG | AI_PASSIVE | AI_V4MAPPED;
  const char *saddr = tcp->saddr;
  int status = getaddrinfo(saddr[0] == '\0' ? NULL : saddr, portstr, &hints, &res);
  if (status != 0) {
    rssnprintf(msg, msize, "address error (%s)", tcp->saddr);
    tracet(1, "gentcpsvr: getaddrinfo error addr=%s %s\n", tcp->saddr, gai_strerror(status));
    tcp->state = -1;
    return 0;
  }

  // Try each address using the first to successfully bind.
  struct addrinfo *next = res;
  for (; next != NULL; next = next->ai_next) {
    // Expect inet6 but double check and trace the address used.
    if (next->ai_family == AF_INET) {
      struct sockaddr_in *ipv4 = (struct sockaddr_in *)next->ai_addr;
      char ipstr[INET_ADDRSTRLEN];
      inet_ntop(next->ai_family, &ipv4->sin_addr, ipstr, sizeof ipstr);
      tracet(3, "gentcpsvr: ipv4 addr='%s' port='%u'\n", ipstr, ntohs(ipv4->sin_port));
    } else if (next->ai_family == AF_INET6) {
      struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)next->ai_addr;
      char ipstr[INET6_ADDRSTRLEN];
      inet_ntop(next->ai_family, &ipv6->sin6_addr, ipstr, sizeof ipstr);
      tracet(3, "gentcpsvr: ipv6 addr='%s' port=%u\n", ipstr, ntohs(ipv6->sin6_port));
    } else {
      rssnprintf(msg, msize, "getaddrinfo error");
      tracet(1, "gentcpsvr: getaddinfo unexpected addr family=%d\n", next->ai_family);
      continue;
    }

    // Generate socket.
    tcp->sock = socket(next->ai_family, next->ai_socktype, next->ai_protocol);
    if (tcp->sock == (socket_t)-1) {
      int err = errsock();
      rssnprintf(msg, msize, "socket error (%d)", err);
      tracet(1, "gentcpsvr: socket error err=%d\n", err);
      continue;
    }

    if (next->ai_family == AF_INET6) {
      struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)next->ai_addr;
      int v6only = 1;
      // Accept dual-stack for the wildcard address. Ensure v6only is not set.
      if (IN6_IS_ADDR_UNSPECIFIED(&ipv6->sin6_addr)) v6only = 0;
      tracet(4, "gentcpsvr: sockopt sock=%" PRISOCK " v6only=%d\n", tcp->sock, v6only);
      if (setsockopt(tcp->sock, IPPROTO_IPV6, IPV6_V6ONLY, (const char *)&v6only, sizeof(v6only)) <
          0) {
        int err = errsock();
        rssnprintf(msg, msize, "sockopt error: v6only");
        tracet(1, "gentcpsvr: sockopt error v6only sock=%" PRISOCK " err=%d\n", tcp->sock, err);
        closesocket(tcp->sock);
        continue;
      }
    }
    
    if (!setsock(tcp->sock, msg, msize)) {
      // setsock() closes the socket on error.
      continue;
    }

#ifdef SVR_REUSEADDR
    // Multiple-use of server socket.
    int reuseaddr = 1;
    setsockopt(tcp->sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuseaddr, sizeof(reuseaddr));
#endif

#ifdef WIN32
    if (bind(tcp->sock, next->ai_addr, (int)next->ai_addrlen) == -1) {
#else
    if (bind(tcp->sock, next->ai_addr, next->ai_addrlen) == -1) {
#endif
      int err = errsock();
      rssnprintf(msg, msize, "bind error (%d) : %d", err, tcp->port);
      tracet(1, "gentcp: bind error port=%u err=%d\n", tcp->port, err);
      closesocket(tcp->sock);
      continue;
    }
    if (listen(tcp->sock, 5) == -1) {
      int err = errsock();
      rssnprintf(msg, msize, "listen error (%d) : %d", err, tcp->port);
      tracet(1, "gentcp: bind error port=%u err=%d\n", tcp->port, err);
      closesocket(tcp->sock);
      continue;
    }

    // Don't need to keep the address.
    tcp->addrs = tcp->addrr = NULL;
    freeaddrinfo(res);
    tcp->state = 1;
    tcp->tact = tickget();
    tracet(5, "gentcp: exit sock=%" PRISOCK "\n", tcp->sock);
    return 1;
  }

  tracet(5, "gentcp: all addresses failed\n");
  freeaddrinfo(res);
  tcp->state = -1;
  return 0;
}

// Generate client TCP socket ------------------------------------------------
static unsigned gentcpcli(tcpcli_t *tcpcli, char *msg, size_t msize) {
  tcp_t *tcp = &tcpcli->svr;
  tracet(3, "gentcpcli: addr='%s' port=%u\n", tcp->saddr, tcp->port);

  // The connect failure paths are responsible for popping the address list if
  // a connect attempt fails. When the list is empty a fresh query is
  // attempted.

  if (tcp->retry > 0) {
    // Quick retry of the connection without respecting tcon. For quickly
    // retrying a connection with the next address.
    tcp->retry--;
  } else if (tcp->tcon < 0 ||
             (tcp->tcon > 0 && (uint32_t)(tickget() - tcp->tdis) < (uint32_t)tcp->tcon)) {
    // Wait reconnect, if there no remaining addresses to try.
    tracet(4, "gentcpcli: waiting to reconnect tcon=%d tdis=%u", tcp->tcon, tcp->tdis);
    return 0;
  }

  if (tcp->addrr == NULL) {
    char portstr[6];
    rssnprintf(portstr, sizeof(portstr), "%u", tcp->port);
    struct addrinfo hints, *new;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_NUMERICSERV | AI_ADDRCONFIG;
    int status = getaddrinfo(tcp->saddr, portstr, &hints, &new);
    if (status == 0) {
      // Success. Update the address list, freeing the prior list.
      if (tcp->addrs != NULL) {
        freeaddrinfo(tcp->addrs);
        tcp->addrs = NULL;
      }
      tcp->addrs = new;
    } else {
      rssnprintf(msg, msize, "address error (%s)", tcp->saddr);
      tracet(1, "gentcpcli: getaddrinfo error addr=%s %s\n", tcp->saddr, gai_strerror(status));
      if (tcp->addrs == NULL) {
        // No fallback prior address list.
        tcp->state = 0;
        tcp->want = 0;
        tcp->tcon = tcpcli->tirecon;
        tcp->tdis = tickget();
        return 0;
      }
      // Fall through to retry with the current address list.
    }
    tcp->addrr = tcp->addrs;
  }

  while (tcp->addrr != NULL) {
    if (tcp->addrr->ai_family == AF_INET) {
      struct sockaddr_in *ipv4 = (struct sockaddr_in *)tcp->addrr->ai_addr;
      char ipstr[INET_ADDRSTRLEN];
      inet_ntop(tcp->addrr->ai_family, &ipv4->sin_addr, ipstr, sizeof ipstr);
      tracet(5, "gentcpcli: ipv4 addr='%s' port='%u'\n", ipstr, ntohs(ipv4->sin_port));
    } else if (tcp->addrr->ai_family == AF_INET6) {
      struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)tcp->addrr->ai_addr;
      char ipstr[INET6_ADDRSTRLEN];
      inet_ntop(tcp->addrr->ai_family, &ipv6->sin6_addr, ipstr, sizeof ipstr);
      tracet(5, "gentcpcli: ipv6 addr='%s' port=%u\n", ipstr, ntohs(ipv6->sin6_port));
    } else {
      tracet(1, "gentcpcli: unexpected addr family=%d\n", tcp->addrr->ai_family);
      continue;
    }
    tcp->sock = socket(tcp->addrr->ai_family, tcp->addrr->ai_socktype, tcp->addrr->ai_protocol);
    if (tcp->sock != (socket_t)-1) break;
    int err = errsock();
    rscatprintf(msg, msize, "socket error (%d)", err);
    tracet(1, "gentcpcli: socket error err=%d\n", err);
    tcp->addrr = tcp->addrr->ai_next;
  }
  if (tcp->addrr == NULL) {
    freeaddrinfo(tcp->addrs);
    tcp->addrs = tcp->addrr = NULL;
    tcp->state = 0;
    tcp->want = 0;
    return 0;
  }

  tracet(5, "gentcpcli: created sock=%" PRISOCK "\n", tcp->sock);
  if (!setsock(tcp->sock, msg, msize)) {
    tcp->state = 0;
    tcp->want = 0;
    tcp->addrr = tcp->addrr->ai_next;        // Pop the address list.
    if (tcp->addrr != NULL) tcp->retry = 1;  // Quick retry.
    return 0;
  }
  tcp->state = 1;
  tcp->want = 0;
  tcp->tact = tickget();
  tracet(5, "gentcpcli: exit sock=%" PRISOCK "\n", tcp->sock);
  return 1;
}

// Disconnect TCP ------------------------------------------------------------
static void discontcp(tcp_t *tcp, int tcon) {
  tracet(3, "discontcp: sock=%" PRISOCK " tcon=%d\n", tcp->sock, tcon);
  if (tcp->tlsp) {
#ifdef RTKLIB_OPENSSL
    if (tcp->ssl != NULL) {
      // If established then attempt a shutdown - does not wait.
      if (tcp->state == 2) SSL_shutdown(tcp->ssl);
      SSL_free(tcp->ssl);
      tcp->ssl = NULL;
    }
#endif
  }
  closesocket(tcp->sock);
  tcp->state = 0;
  tcp->want = 0;
  tcp->tlsstate = 0;  // Uninitialized.
  tcp->tcon = tcon;
  tcp->tdis = tickget();
}

// If not the first attempt then back off the reconnect time. The tcon is
// generally reset when a connection is successfully established, and the tcon
// is not generally backed off from a connected state.
static int backofftcon(const tcp_t *tcp, int tcon) {
  if (tcon > 0 && tcp->tcon > 0) {
    if (tcp->tcon < 300000) {
      int tconb = tcp->tcon * 5 / 4;
      tracet(3, "backofftcp: sock=%" PRISOCK " backing off tcon=%d\n", tcp->sock, tconb);
      return tconb;
    }
  }
  return tcon;
}

// OpenSSL callback for an encrypted file.
static int encpasswd(char *buf, int size, int rwflag, void *userdata) {
  tracet(3, "encpasswd: buf=%p size=%d flag=%d userdata=%p\n", buf, size, rwflag, userdata);
  if (userdata == NULL) return 0;
  const char *passwd = (char *)userdata;
  int len = (int)strlen(passwd);
  if (len > size) len = size;
  memcpy(buf, passwd, len);
  return len;
}

// Open TCP server -----------------------------------------------------------
//
// Note the server can allow unencrypted connections when TLS is enabled with
// the 'A' option by peeking at the first input byte, but this is not the
// default. This only works when the client reliably sends a message when
// opening the stream and when that message can not be confused with the TLS
// hand-shake byte '0x16'. It is primarily used along with NTRIP casters see
// below, but might be workable with a server receiving a plain text stream.
static tcpsvr_t *opentcpsvr(const char *path, char *msg, size_t msize) {
  tracet(3, "opentcpsvr: path=%s\n", path);

  // Path options.
  // tlsp: 0: no TLS, 1: only TLS, 2: allow TLS or unencrypted.
  unsigned tlsp = 0, verify = tlssvrverify;
  for (const char *p = path; (p = strstr(p, "::")); p += 2) {
    if (p[2] == 'S')
      tlsp = 1;
    else if (p[2] == 'A')
      tlsp = 2;
    else if (p[2] == 'V')
      sscanf(p + 2, "V=%u", &verify);
  }
  tracet(5, "opentcpsvr: tls=%u verify=%u\n", tlsp, verify);

  tcpsvr_t *tcpsvr = malloc(sizeof(tcpsvr_t));
  if (tcpsvr == NULL) return NULL;
  memset(tcpsvr, 0, sizeof(tcpsvr_t));
  tcpsvr->obufsize = buffsize;
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    tcpsvr->obuf[ci] = malloc(tcpsvr->obufsize);
    if (tcpsvr->obuf[ci] == NULL) {
      for (unsigned j = 0; j < MAXCLI; j++) free(tcpsvr->obuf[j]);
      free(tcpsvr);
      rssnprintf(msg, msize, "alloc failed");
      return NULL;
    }
  }

  char port[256] = "";
  decodetcppath(path, tcpsvr->svr.saddr, port, NULL, NULL, NULL, NULL);
  if (sscanf(port, "%u", &tcpsvr->svr.port) < 1) {
    rssnprintf(msg, msize, "port error: %s", port);
    tracet(1, "opentcpsvr: port error port=%s\n", port);
    free(tcpsvr);
    return NULL;
  }

  if (tlsp == 0 && tcpsvr->svr.port == 443) {
    tracet(3, "opentcpsvr: enforcing TLS for port=%u\n", tcpsvr->svr.port);
    tlsp = 1;
  }
  tcpsvr->svr.tlsp = tlsp;
  tcpsvr->svr.tlsverify = tlsp > 0 ? verify : 0;
  if (!gentcpsvr(&tcpsvr->svr, msg, msize)) {
    closetcpsvr(tcpsvr);
    return NULL;
  }
  tcpsvr->svr.tcon = 0;

  if (tcpsvr->svr.tlsp > 0) {
#ifdef RTKLIB_OPENSSL
    tcpsvr->svr.ctx = SSL_CTX_new(TLS_server_method());
    if (tcpsvr->svr.ctx == NULL) {
      ssl_report_errors();
      closetcpsvr(tcpsvr);
      rssnprintf(msg, msize, "TLS init error");
      tracet(1, "opentcpsvr tls: TLS initialization error\n");
      return NULL;
    }
    SSL_CTX_set_options(tcpsvr->svr.ctx, SSL_OP_NO_SSLv2);
    SSL_CTX_set_options(tcpsvr->svr.ctx, SSL_OP_NO_SSLv3);
    SSL_CTX_set_options(tcpsvr->svr.ctx, SSL_OP_IGNORE_UNEXPECTED_EOF);

    long current_mode = SSL_CTX_get_mode(tcpsvr->svr.ctx);
    current_mode |= SSL_MODE_ENABLE_PARTIAL_WRITE;
    current_mode |= SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER;
    current_mode |= SSL_MODE_AUTO_RETRY;
    SSL_CTX_set_mode(tcpsvr->svr.ctx, current_mode);

    // Server session caching.
    SSL_CTX_set_session_cache_mode(tcpsvr->svr.ctx, SSL_SESS_CACHE_SERVER);

    // Set the password to use for encrypted files - to at least avoid
    // the default callback and to fail more gracefully.
    SSL_CTX_set_default_passwd_cb(tcpsvr->svr.ctx, encpasswd);
    SSL_CTX_set_default_passwd_cb_userdata(tcpsvr->svr.ctx, (void *)tlssvrpasswd);

    // A server defaults to verify 'none'.
    if (tcpsvr->svr.tlsverify) {
      // Assume a client certificate is required if it is being requested
      // and otherwise verified, but that could be a separate option. Note
      // that the content is not otherwise verified - no fields are being
      // currently being checked.
      SSL_CTX_set_verify(tcpsvr->svr.ctx, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, NULL);
      if (tlssvrcafile[0] != '\0') {
        if (SSL_CTX_load_verify_file(tcpsvr->svr.ctx, tlssvrcafile) != 1) {
          tracet(1, "opentcpsvr tls: load verify file failed for '%s'\n", tlssvrcafile);
          ssl_report_errors();
        }
      }
      if (tlssvrcadir[0] != '\0') {
        if (SSL_CTX_load_verify_dir(tcpsvr->svr.ctx, tlssvrcadir) != 1) {
          tracet(1, "opentcpsvr tls: load verify dir failed for '%s'\n", tlssvrcadir);
          ssl_report_errors();
        }
      }
      if (SSL_CTX_set_default_verify_paths(tcpsvr->svr.ctx) != 1) {
        tracet(2, "opentcpsvr tls: set default verify paths failed\n");
        ssl_report_errors();
      }
#ifdef WIN32
      if (SSL_CTX_load_verify_store(tcpsvr->svr.ctx, "org.openssl.winstore://") != 1) {
        tracet(2, "opentcpsvr tls: load verify store failed\n");
        ssl_report_errors();
      }
#endif
    } else {
      tracet(5, "opentcpsvr tls: verify none\n");
      SSL_CTX_set_verify(tcpsvr->svr.ctx, SSL_VERIFY_NONE, NULL);
    }

    // A server certificate is required, so fail here with an error if
    // not supplied or the use fails.
    tracet(4, "opentcpsvr: TLS cert='%s' key='%s'\n", tlssvrcertfile, tlssvrkeyfile);
    if (tlssvrcertfile[0] == '\0') {
      closetcpsvr(tcpsvr);
      rssnprintf(msg, msize, "TLS svr cert file required");
      tracet(1, "opentcpsvr tls: server certificate file required\n");
      return NULL;
    }
    if (SSL_CTX_use_certificate_file(tcpsvr->svr.ctx, tlssvrcertfile, SSL_FILETYPE_PEM) <= 0) {
      ssl_report_errors();
      closetcpsvr(tcpsvr);
      rssnprintf(msg, msize, "TLS cert use failed");
      tracet(1, "opentcpsvr tls: certificate use failed for '%s'\n", tlssvrcertfile);
      return NULL;
    }
    if (tlssvrkeyfile[0] == '\0') {
      closetcpsvr(tcpsvr);
      rssnprintf(msg, msize, "TLS svr cert key file required");
      tracet(1, "opentcpsvr tls: server certificate key file required\n");
      return NULL;
    }
    if (SSL_CTX_use_PrivateKey_file(tcpsvr->svr.ctx, tlssvrkeyfile, SSL_FILETYPE_PEM) <= 0) {
      ssl_report_errors();
      closetcpsvr(tcpsvr);
      rssnprintf(msg, msize, "TLS key use failed");
      tracet(1, "opentcpsvr tls: private use failed for '%s'\n", tlssvrkeyfile);
      return NULL;
    }
#else
    closetcpsvr(tcpsvr);
    tracet(1, "opentcpsvr tls: not supported\n");
    rssnprintf(msg, msize, "TLS not supported");
    return NULL;
#endif
  }

  tcpsvr->svr.want = 1;  // Wait to accept new connections.
  return tcpsvr;
}
// Close TCP server ----------------------------------------------------------
static void closetcpsvr(tcpsvr_t *tcpsvr) {
  tracet(3, "closetcpsvr:\n");

  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (tcpsvr->cli[ci].state) {
      if (tcpsvr->cli[ci].tlsp > 0) {
#ifdef RTKLIB_OPENSSL
        if (tcpsvr->cli[ci].ssl != NULL) {
          // If established then attempt a shutdown - does not wait.
          if (tcpsvr->cli[ci].state == 2) SSL_shutdown(tcpsvr->cli[ci].ssl);
          SSL_free(tcpsvr->cli[ci].ssl);
          tcpsvr->cli[ci].ssl = NULL;
        }
#endif
      }
      closesocket(tcpsvr->cli[ci].sock);
    }
    free(tcpsvr->obuf[ci]);
  }
  if (tcpsvr->svr.tlsp) {
#ifdef RTKLIB_OPENSSL
    if (tcpsvr->svr.ctx != NULL) SSL_CTX_free(tcpsvr->svr.ctx);
    tcpsvr->svr.ctx = NULL;
#endif
  }
  closesocket(tcpsvr->svr.sock);
  free(tcpsvr);
}
// Update TCP server ---------------------------------------------------------
static void updatetcpsvr(tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  tracet(4, "updatetcpsvr: state=%u\n", tcpsvr->svr.state);

  if (tcpsvr->svr.state == 0) return;

  char saddr[256] = "";
  unsigned n = 0;
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (tcpsvr->cli[ci].state == 0) continue;
    rsstrcpy(saddr, sizeof(saddr), tcpsvr->cli[ci].saddr);
    n++;
  }
  if (n == 0) {
    tcpsvr->svr.state = 1;
    rsstrcpy(msg, msize, "waiting...");
    return;
  }
  tcpsvr->svr.state = 2;
  if (n == 1)
    rssnprintf(msg, msize, "%s", saddr);
  else
    rssnprintf(msg, msize, "%d clients", n);
}
// Accept client connection --------------------------------------------------
// Note this resets the 'want' states.
static unsigned accsock(tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  tracet(4, "accsock: sock=%" PRISOCK " tlsp=%u\n", tcpsvr->svr.sock, tcpsvr->svr.tlsp);

  unsigned ci;
  for (ci = 0; ci < MAXCLI; ci++) {
    if (tcpsvr->cli[ci].state == 0) break;
  }
  if (ci >= MAXCLI) {
    tracet(2, "accsock: too many clients sock=%" PRISOCK "\n", tcpsvr->svr.sock);
    return 0;
  }

  // Note there are some paths that fall through without intending to setting
  // this flag, error paths to be retried at the next polling interval.
  tcpsvr->svr.want = 0;

  // Non-blocking accept.
  fd_set rs;
  FD_ZERO(&rs);
  FD_SET(tcpsvr->svr.sock, &rs);
  struct timeval tv = {0};
#ifdef WIN32
  int ret = select(0, &rs, NULL, NULL, &tv);
#else
  int ret = select(tcpsvr->svr.sock + 1, &rs, NULL, NULL, &tv);
#endif
  if (ret == 0) {
    tcpsvr->svr.want = 1;  // Read wait.
    return 0;
  }
  if (ret < 0) {
    int err = errsock();
    rssnprintf(msg, msize, "accept error (%d)", err);
    tracet(2, "accsock: accept error sock=%" PRISOCK " err=%d\n", tcpsvr->svr.sock, err);
    closesocket(tcpsvr->svr.sock);
    tcpsvr->svr.state = 0;
    return 0;
  }

  struct sockaddr_in6 addr;
  socklen_t len = sizeof(addr);
  socket_t sock = accept(tcpsvr->svr.sock, (struct sockaddr *)&addr, &len);
  if (sock == (socket_t)-1) {
    int err = errsock();
    rssnprintf(msg, msize, "accept error (%d)", err);
    if (err == EBADF || err == EFAULT || err == ENOTSOCK || err == EOPNOTSUPP || err == EPROTO) {
      // Error with the server socket - close the server.
      tracet(2, "accsock: accept error sock=%" PRISOCK " err=%d\n", tcpsvr->svr.sock, err);
      closesocket(tcpsvr->svr.sock);
      tcpsvr->svr.state = 0;
      return 0;
    }
    // Assumed to be an error accepting this client connection and not a fault
    // in the server socket, so it does not close the server. Linux can pass
    // these through to accept().
    tracet(3, "accsock: accept error sock=%" PRISOCK " err=%d\n", tcpsvr->svr.sock, err);
    return 0;
  }

  // setsock() will close the socket on an error.
  if (!setsock(sock, msg, msize)) return 0;

  tcpsvr->cli[ci].sock = sock;
  char saddr[INET6_ADDRSTRLEN];
  inet_ntop(AF_INET6, &addr.sin6_addr, saddr, sizeof(saddr));
  rsstrcpy(tcpsvr->cli[ci].saddr, sizeof(tcpsvr->cli[0].saddr), saddr);
  rssnprintf(msg, msize, "%s", tcpsvr->cli[ci].saddr);
  tracet(3, "accsock: connected sock=%" PRISOCK " addr=%s ci=%u\n", tcpsvr->cli[ci].sock,
         tcpsvr->cli[ci].saddr, ci);

  if (tcpsvr->svr.tlsp == 2) {
    // Probe for TLS handshake byte, allowing TLS and non-TLS
    // connections. This can be used when the client is expected to send a
    // request that is distinct from the TLS handshake, such as for NTRIP
    // servers. But it might not work for a raw TCP/IP server where the
    // client may not send initial data or where it could be confused with
    // the TLS handshake byte.
    tcpsvr->cli[ci].state = 1;  // Wait.
    tcpsvr->cli[ci].want = 1;   // Read wait.
    tcpsvr->cli[ci].tlsp = 0;
    tcpsvr->cli[ci].tlsstate = 1;  // TLS probe.
  } else if (tcpsvr->svr.tlsp == 1) {
    // Require a TLS connection.
    tcpsvr->cli[ci].state = 1;  // Wait.
    tcpsvr->cli[ci].want = 1;   // Read wait.
    tcpsvr->cli[ci].tlsp = 1;
    tcpsvr->cli[ci].tlsstate = 2;  // TLS new.
  } else {
    // Raw TCP/IP connection.
    tracet(3, "accsock: connected\n");
    tcpsvr->cli[ci].state = 2;  // Connected
    tcpsvr->cli[ci].tlsp = 0;
    tcpsvr->cli[ci].tlsstate = 0;
    tcpsvr->nout[ci] = 0;
  }
  tcpsvr->cli[ci].tact = tickget();
  return 1;
}
// Wait socket accept --------------------------------------------------------
// Note this resets the 'want' states.
static unsigned waittcpsvr(tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  tracet(4, "waittcpsvr: sock=%" PRISOCK " state=%u\n", tcpsvr->svr.sock, tcpsvr->svr.state);

  while (accsock(tcpsvr, msg, msize));

  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    tcpsvr->cli[ci].want = 0;
    if (tcpsvr->cli[ci].state == 0) continue;
    if (tcpsvr->cli[ci].state == 1) {
      if (tcpsvr->cli[ci].tlsstate == 1) {
        // TLS Probe.
        tracet(5, "waittcpsvr tls: probe\n");
        socket_t sock = tcpsvr->cli[ci].sock;
        fd_set rs;
        FD_ZERO(&rs);
        FD_SET(sock, &rs);
        struct timeval tv = {0};
#ifdef WIN32
        int ret = select(0, &rs, NULL, NULL, &tv);
#else
        int ret = select(sock + 1, &rs, NULL, NULL, &tv);
#endif
        if (ret < 0) {
          closesocket(tcpsvr->cli[ci].sock);
          tcpsvr->cli[ci].state = 0;
          tcpsvr->cli[ci].tlsp = 0;
          tcpsvr->cli[ci].tlsstate = 0;
          tracet(3, "waittcpsvr tls: probe failed socket=%" PRISOCK "\n", tcpsvr->cli[ci].sock);
          rssnprintf(msg, msize, "TLS probe failed");
          continue;
        }
        if (ret == 0) {
          tcpsvr->cli[ci].want = 1;  // Want read.
          continue;
        }
        char buf[1];
        ssize_t nr = recv(tcpsvr->cli[ci].sock, buf, 1, MSG_PEEK);
        if (nr <= 0) {
          closesocket(tcpsvr->cli[ci].sock);
          tcpsvr->cli[ci].state = 0;
          tcpsvr->cli[ci].tlsp = 0;
          tcpsvr->cli[ci].tlsstate = 0;
          tracet(3, "waittcpsvr tls: probe failed socket=%" PRISOCK "\n", tcpsvr->cli[ci].sock);
          rssnprintf(msg, msize, "TLS probe failed");
          continue;
        }
        tracet(5, "waittcpsvr tls: probe byte=%02x\n", buf[0]);
        if (buf[0] != 0x16) {
          // Assume a raw TCP/IP connection.
          tcpsvr->cli[ci].state = 2;  // Connected
          tcpsvr->cli[ci].tlsp = 0;
          tcpsvr->cli[ci].tlsstate = 0;
          tcpsvr->nout[ci] = 0;
          continue;
        }
        // Appears to be a TLS handshake.
#ifndef RTKLIB_OPENSSL
        // If RTKLIB is not build with TLS support and a TLS connection then
        // close the connection here.
        closesocket(tcpsvr->cli[ci].sock);
        tcpsvr->cli[ci].state = 0;
        tcpsvr->cli[ci].tlsp = 0;
        tcpsvr->cli[ci].tlsstate = 0;
        tracet(1, "waittcpsvr tls: connection not supported socket=%" PRISOCK "\n",
               tcpsvr->cli[ci].sock);
        rssnprintf(msg, msize, "TLS not supported");
        continue;
#else
        tcpsvr->cli[ci].tlsstate = 2;  // New.
        tcpsvr->cli[ci].tlsp = 1;
        tcpsvr->cli[ci].tact = tickget();
#endif
      }
      if (tcpsvr->cli[ci].tlsstate == 2) {
        // A new TLS connection.
#ifdef RTKLIB_OPENSSL
        tracet(5, "waittcpsvr tls: new verify=%u\n", tcpsvr->svr.tlsverify);
        tcpsvr->cli[ci].tlsverify = tcpsvr->svr.tlsverify;
        tcpsvr->cli[ci].ssl = SSL_new(tcpsvr->svr.ctx);
        if (tcpsvr->cli[ci].ssl == NULL) {
          closesocket(tcpsvr->cli[ci].sock);
          tcpsvr->cli[ci].state = 0;
          tcpsvr->cli[ci].tlsp = 0;
          tcpsvr->cli[ci].tlsstate = 0;
          tracet(2, "waittcpsvr tls: new failed socket=%" PRISOCK "\n", tcpsvr->cli[ci].sock);
          rssnprintf(msg, msize, "ssl new failed");
          continue;
        }
        if (SSL_set_fd(tcpsvr->cli[ci].ssl, (int)tcpsvr->cli[ci].sock) != 1) {
          ssl_report_errors();
          SSL_free(tcpsvr->cli[ci].ssl);
          tcpsvr->cli[ci].ssl = NULL;
          closesocket(tcpsvr->cli[ci].sock);
          tcpsvr->cli[ci].state = 0;
          tcpsvr->cli[ci].tlsp = 0;
          tcpsvr->cli[ci].tlsstate = 0;
          tracet(2, "waittcpsvr tls: set fd fail socket=%" PRISOCK "\n", tcpsvr->cli[ci].sock);
          rssnprintf(msg, msize, "ssl set fd failed");
          continue;
        }
        // Set to non-blocking.
        if (!BIO_socket_nbio((int)tcpsvr->cli[ci].sock, 1))
          tracet(2, "waittcpsvr tls: error setting socket to non-blocking\n");
        tcpsvr->cli[ci].tlsstate = 3;  // Accept.
#else
        closesocket(tcpsvr->cli[ci].sock);
        tcpsvr->cli[ci].state = 0;
        tcpsvr->cli[ci].tlsp = 0;
        tcpsvr->cli[ci].tlsstate = 0;
        tracet(1, "waittcpsvr tls: not supported\n");
        rssnprintf(msg, msize, "TLS not supported");
        continue;
#endif
      }
      if (tcpsvr->cli[ci].tlsstate == 3) {
        // TLS accept.
#ifdef RTKLIB_OPENSSL
        int ret = SSL_accept(tcpsvr->cli[ci].ssl);
        tracet(5, "waittcpsvr tls: accept ret=%d\n", ret);
        if (ret == 1) {
          tcpsvr->cli[ci].state = 2;     // Connected
          tcpsvr->cli[ci].tlsstate = 4;  // Accepted.
          tcpsvr->nout[ci] = 0;
          continue;
        }
        int sslerr = SSL_get_error(tcpsvr->cli[ci].ssl, ret);
        if (sslerr == SSL_ERROR_WANT_READ) {
          tracet(5, "waittcpsvr tls: accept want read\n");
          tcpsvr->cli[ci].want = 1;  // Read wait.
          continue;                  // To be retried.
        }
        if (sslerr == SSL_ERROR_WANT_WRITE) {
          tracet(5, "waittcpsvr tls: accept want write\n");
          tcpsvr->cli[ci].want = 2;  // Write wait.
          continue;                  // To be retried.
        }
        if (sslerr == SSL_ERROR_ZERO_RETURN) {  // Graceful EOF.
          SSL_free(tcpsvr->cli[ci].ssl);
          tcpsvr->cli[ci].ssl = NULL;
          tcpsvr->cli[ci].state = 0;
          tcpsvr->cli[ci].tlsstate = 0;
          tcpsvr->cli[ci].tlsp = 0;
          closesocket(tcpsvr->cli[ci].sock);
          tracet(3, "waittcpsvr tls: accept eof\n");
          rssnprintf(msg, msize, "ssl eof");
          continue;
        }
        int err = 0;
        if (sslerr == SSL_ERROR_SYSCALL) err = errsock();
        SSL_free(tcpsvr->cli[ci].ssl);
        tcpsvr->cli[ci].ssl = NULL;
        tcpsvr->cli[ci].state = 0;
        tcpsvr->cli[ci].tlsstate = 0;
        tcpsvr->cli[ci].tlsp = 0;
        closesocket(tcpsvr->cli[ci].sock);
        ssl_report_errors();
        tracet(3, "waittcpsvr tls: accept unexpected sslerr=%d err=%d\n", sslerr, err);
        rssnprintf(msg, msize, "TLS error");
        continue;
#endif
      }
    }
    if (tcpsvr->cli[ci].state == 2) {
      // Connected. Continue sending buffered output.
      if (tcpsvr->nout[ci] > 0) {
        int err;
        ssize_t ns = send_nb(&tcpsvr->cli[ci], tcpsvr->obuf[ci], tcpsvr->obufsize, 0,
                             tcpsvr->nout[ci], &err);
        if (ns < 0) {
          if (err) {
            tracet(3, "waittcpsvr: send error ci=%u sock=%" PRISOCK " err=%d\n", ci,
                   tcpsvr->cli[ci].sock, err);
          }
          discontcp(&tcpsvr->cli[ci], ticonnect);
          updatetcpsvr(tcpsvr, msg, msize);
          continue;
        }
        if ((size_t)ns <= tcpsvr->nout[ci]) {
          size_t rem = tcpsvr->nout[ci] - (size_t)ns;
          memmove(tcpsvr->obuf[ci], tcpsvr->obuf[ci] + ns, rem);
          tcpsvr->nout[ci] = rem;
        }
        if (ns > 0) tcpsvr->cli[ci].tact = tickget();
      }
    }
  }

  updatetcpsvr(tcpsvr, msg, msize);
  return tcpsvr->svr.state == 2;
}
// Read TCP server -----------------------------------------------------------
static size_t readtcpsvr(tcpsvr_t *tcpsvr, uint8_t *buff, size_t size, size_t n, char *msg,
                         size_t msize) {
  tracet(4, "readtcpsvr: state=%u\n", tcpsvr->svr.state);

  if (!waittcpsvr(tcpsvr, msg, msize)) return 0;

  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (tcpsvr->cli[ci].state != 2) continue;
    int err;
    ssize_t nr = recv_nb(&tcpsvr->cli[ci], buff, size, 0, n, &err);
    if (nr < 0) {
      if (err) {
        tracet(2, "readtcpsvr: recv error sock=%" PRISOCK " err=%d\n", tcpsvr->cli[ci].sock, err);
      }
      discontcp(&tcpsvr->cli[ci], ticonnect);
      updatetcpsvr(tcpsvr, msg, msize);
      continue;
    }
    if (nr > 0) {
      tcpsvr->cli[ci].tact = tickget();
      return (size_t)nr;
    }
  }
  return 0;
}
// Write TCP server ----------------------------------------------------------
//
// Returns the number of bytes written to each client and 0 if there are no
// clients. The design is intended to allow the caller to write without
// checking the return value and with the expectation that client connections
// can handle the rate or will be closed. This layer adds some extra buffering
// to handle transients, in addition to the TLS library and system level
// buffering. It is expected that writes are well below the buffer size and a
// large write filling the buffer will cause client connections to be closed.
static size_t writetcpsvr(tcpsvr_t *tcpsvr, const uint8_t *buff, size_t size, size_t n, char *msg,
                          size_t msize) {
  tracet(4, "writetcpsvr: state=%u n=%zu\n", tcpsvr->svr.state, n);

  // This will also continue flushing the output buffers.
  if (!waittcpsvr(tcpsvr, msg, msize)) return 0;

  // Can early out in this case.
  if (n == 0) return 0;

  RBOUNDSCHECK(buff, size, n - 1);

  size_t ns = 0;
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (tcpsvr->cli[ci].state != 2) continue;

    // Append the content onto the output buffer.
    if (tcpsvr->nout[ci] + n > tcpsvr->obufsize) {
      tracet(2, "writetcpsvr: output buffer overflow\n");
      // Close the connection.
      discontcp(&tcpsvr->cli[ci], ticonnect);
      updatetcpsvr(tcpsvr, msg, msize);
      continue;
    }
    memcpy(tcpsvr->obuf[ci] + tcpsvr->nout[ci], buff, n);
    tcpsvr->nout[ci] += n;
    ns = n;

#if 1
    // Try sending the output buffer content now.
    if (tcpsvr->nout[ci] > 0) {
      int err;
      ssize_t nsn =
          send_nb(&tcpsvr->cli[ci], tcpsvr->obuf[ci], tcpsvr->obufsize, 0, tcpsvr->nout[ci], &err);
      tracet(5, "writetcpsvr: send_nb %zu %zd\n", tcpsvr->nout[ci], nsn);
      if (nsn < 0) {
        if (err) {
          tracet(2, "writetcpsvr: send error ci=%u sock=%" PRISOCK " err=%d\n", ci,
                 tcpsvr->cli[ci].sock, err);
        }
        discontcp(&tcpsvr->cli[ci], ticonnect);
        updatetcpsvr(tcpsvr, msg, msize);
        continue;
      }
      if ((size_t)nsn <= tcpsvr->nout[ci]) {
        size_t rem = tcpsvr->nout[ci] - nsn;
        memmove(tcpsvr->obuf[ci], tcpsvr->obuf[ci] + nsn, rem);
        tcpsvr->nout[ci] = rem;
      }
      if (nsn > 0) tcpsvr->cli[ci].tact = tickget();
    }
#endif
  }
  return ns;
}
static int wanttcpsvr(tcpsvr_t *tcpsvr, unsigned op, wantset_t *wantset) {
  tracet(4, "wanttcpsvr: op=%u\n", op);

  if (tcpsvr->svr.state == 0) return 0;

  fd_set *rs = &wantset->rs, *ws = &wantset->ws;

  // The new connection socket for accept.
  if (tcpsvr->svr.want & 1) {
    FD_SET(tcpsvr->svr.sock, rs);
    tcpsvr->svr.want = 0;
#ifndef WIN32
    int n = 1 + (int)tcpsvr->svr.sock;
    if (n > wantset->nfds) wantset->nfds = n;
#endif
  }

  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (tcpsvr->cli[ci].state < 1) continue;
    if (tcpsvr->cli[ci].want != 0) {
      if (tcpsvr->cli[ci].want & 1) FD_SET(tcpsvr->cli[ci].sock, rs);
      if (tcpsvr->cli[ci].want & 2) FD_SET(tcpsvr->cli[ci].sock, ws);
      tcpsvr->cli[ci].want = 0;
#ifndef WIN32
      int n = 1 + (int)tcpsvr->cli[ci].sock;
      if (n > wantset->nfds) wantset->nfds = n;
#endif
      continue;
    }
    if (op & 1) {
      FD_SET(tcpsvr->cli[ci].sock, rs);
#ifndef WIN32
      int n = 1 + (int)tcpsvr->cli[ci].sock;
      if (n > wantset->nfds) wantset->nfds = n;
#endif
      continue;
    }
  }
  return 0;
}
// Get state TCP server ------------------------------------------------------
static int statetcpsvr(const tcpsvr_t *tcpsvr) { return tcpsvr ? tcpsvr->svr.state : 0; }
// Print extended state TCP --------------------------------------------------
static void statextcp(const tcp_t *tcp, char *msg, size_t msize) {
  rscatprintf(msg, msize, "    state = %u\n", tcp->state);
  rscatprintf(msg, msize, "    saddr = %s\n", tcp->saddr);
  rscatprintf(msg, msize, "    port  = %u\n", tcp->port);
  rscatprintf(msg, msize, "    sock  = %" PRISOCK "\n", tcp->sock);
#ifdef RTK_DISABLED  // For debug
  rscatprintf(msg, msize, "    tcon  = %d\n", tcp->tcon);
  rscatprintf(msg, msize, "    tact  = %u\n", tcp->tact);
  rscatprintf(msg, msize, "    tdis  = %u\n", tcp->tdis);
#endif
}
// Get extended state TCP server ---------------------------------------------
static int statextcpsvr(const tcpsvr_t *tcpsvr, char *msg, size_t msize) {
  unsigned state = tcpsvr ? tcpsvr->svr.state : 0;
  rscatprintf(msg, msize, "tcpsvr:\n");
  rscatprintf(msg, msize, "  state   = %u\n", state);
  if (state == 0) return 0;
  rscatprintf(msg, msize, "  svr:\n");
  statextcp(&tcpsvr->svr, msg, msize);
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (tcpsvr->cli[ci].state == 0) continue;
    rscatprintf(msg, msize, "  cli#%d:\n", ci);
    statextcp(tcpsvr->cli + ci, msg, msize);
  }
  return state;
}
// Connect to server ---------------------------------------------------------
static unsigned consock(tcpcli_t *tcpcli, char *msg, size_t msize) {
  tracet(4, "consock: sock=%" PRISOCK "\n", tcpcli->svr.sock);

  tcpcli->svr.want = 0;

  if (tcpcli->svr.addrr == NULL) {
    // No address, should not happen, gentcpcli() should set the address,
    // return to the closed state.
    tracet(1, "gentcpcli: unexpected address\n");
    tcpcli->svr.state = 0;
    tcpcli->svr.tdis = tickget();
    return 0;
  }

  // Check the current address, and report it in a trace.
  if (tcpcli->svr.addrr->ai_family == AF_INET) {
    struct sockaddr_in *ipv4 = (struct sockaddr_in *)tcpcli->svr.addrr->ai_addr;
    char ipstr[INET_ADDRSTRLEN];
    inet_ntop(tcpcli->svr.addrr->ai_family, &ipv4->sin_addr, ipstr, sizeof ipstr);
    tracet(5, "consock: ipv4 addr='%s' port='%u'\n", ipstr, ntohs(ipv4->sin_port));
  } else if (tcpcli->svr.addrr->ai_family == AF_INET6) {
    struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)tcpcli->svr.addrr->ai_addr;
    char ipstr[INET6_ADDRSTRLEN];
    inet_ntop(tcpcli->svr.addrr->ai_family, &ipv6->sin6_addr, ipstr, sizeof ipstr);
    tracet(5, "consock: ipv6 addr='%s' port=%u\n", ipstr, ntohs(ipv6->sin6_port));
  } else {
    // Should not happen, gentcpcli() should filter the family, but pop the
    // address and return to the closed state.
    tracet(1, "consock: unexpected addr family=%d\n", tcpcli->svr.addrr->ai_family);
    tcpcli->svr.addrr = tcpcli->svr.addrr->ai_next;        // Pop the address list.
    if (tcpcli->svr.addrr != NULL) tcpcli->svr.retry = 1;  // Quick retry of next address.
    tcpcli->svr.tdis = tickget();
    tcpcli->svr.state = 0;
    return 0;
  }

  int err, stat = connect_nb(tcpcli->svr.sock, tcpcli->svr.addrr->ai_addr,
                             (socklen_t)tcpcli->svr.addrr->ai_addrlen, &err);
  if (stat == -1) {
    rssnprintf(msg, msize, "connect error (%d)", err);
    tracet(1, "consock: connect error sock=%" PRISOCK " err=%d\n", tcpcli->svr.sock, err);
    closesocket(tcpcli->svr.sock);
    tcpcli->svr.state = 0;
    tcpcli->svr.addrr = tcpcli->svr.addrr->ai_next;  // Pop the address list.
    // If there are no more addresses to try then back off before
    // retrying, otherwise a quick retry of the next address.
    if (tcpcli->svr.addrr == NULL)
      tcpcli->svr.tcon = backofftcon(&tcpcli->svr, tcpcli->tirecon);
    else
      tcpcli->svr.retry = 1;  // Quick retry.
    tcpcli->svr.tdis = tickget();
    return 0;
  }
  if (stat == 0) {  // Not connected.
    tcpcli->svr.want = 1;
    rsstrcpy(msg, msize, "connecting...");
    return 0;
  }

  if (tcpcli->svr.tlsp) {
#ifdef RTKLIB_OPENSSL
    tcpcli->svr.tlsstate = 0;  // Uninitialized.
    tcpcli->svr.ssl = SSL_new(tcpcli->svr.ctx);
    if (tcpcli->svr.ssl == NULL) {
      tracet(1, "consock tls: ssl new fail, close socket: %" PRISOCK "\n", tcpcli->svr.sock);
      discontcp(&tcpcli->svr, backofftcon(&tcpcli->svr, tcpcli->tirecon));
      return 0;
    }

    if (SSL_set_fd(tcpcli->svr.ssl, (int)tcpcli->svr.sock) != 1) {
      tracet(3, "consock tls: SSL_set_fd fail socket=%" PRISOCK "\n", tcpcli->svr.sock);
      ssl_report_errors();
      discontcp(&tcpcli->svr, backofftcon(&tcpcli->svr, tcpcli->tirecon));
      return 0;
    }

    // Set to non-blocking.
    if (!BIO_socket_nbio((int)tcpcli->svr.sock, 1))
      tracet(2, "consock tls: error setting socket to non-blocking\n");

    // Don't set the SNI for an IP address, or would use SSL_set1_ipaddr()
    // from OpenSSL 4.0.
    struct sockaddr_in sa4;
    struct sockaddr_in6 sa6;
    if (inet_pton(AF_INET, tcpcli->svr.saddr, &(sa4.sin_addr)) != 1 &&
        inet_pton(AF_INET6, tcpcli->svr.saddr, &(sa6.sin6_addr)) != 1) {
      tracet(5, "consock tls: set_tlsext for host '%s'\n", tcpcli->svr.saddr);
      if (SSL_set_tlsext_host_name(tcpcli->svr.ssl, tcpcli->svr.saddr) != 1)
        tracet(3, "consock tls :set_tlsext error for host '%s'\n", tcpcli->svr.saddr);
    }

    // Assume the host name is to be verified if otherwise verifying
    // the certificate, but this could be a separate option.
    if (tcpcli->svr.tlsverify) {
#if OPENSSL_VERSION_PREREQ(4, 0)
      if (SSL_set1_dnsname(tcpcli->svr.ssl, tcpcli->svr.saddr) != 1)
#else
      if (SSL_set1_host(tcpcli->svr.ssl, tcpcli->svr.saddr) != 1)
#endif
        tracet(3, "consock tls: set1_host error for host '%s'\n", tcpcli->svr.saddr);
    }

    tcpcli->svr.tlsstate = 1;  // Establish.
#else
    tracet(1, "consock tls: not supported\n");
    rsstrcpy(msg, msize, "TLS not supported");
    discontcp(&tcpcli->svr, backofftcon(&tcpcli->svr, tcpcli->tirecon));
    return 0;
#endif
  }

  rsstrcpy(msg, msize, tcpcli->svr.saddr);
  tracet(3, "consock: connected sock=%" PRISOCK " addr=%s\n", tcpcli->svr.sock, tcpcli->svr.saddr);
  tcpcli->svr.state = 2;
  tcpcli->svr.tcon = tcpcli->tirecon;  // Reset.
  tcpcli->svr.tact = tickget();
  return 1;
}

// Open TCP client -----------------------------------------------------------
// Note fallback from TLS to unencrypted is not implemented for clients.
static tcpcli_t *opentcpcli(const char *path, char *msg, size_t msize) {
  tracet(3, "opentcpcli: path=%s\n", path);

  // Path options.
  // tlsp: 0: non-TLS, 1: TLS.
  unsigned tlsp = 0, verify = tlscliverify;
  for (const char *p = path; (p = strstr(p, "::")); p += 2) {
    if (p[2] == 'S')
      tlsp = 1;
    else if (p[2] == 'V')
      sscanf(p + 2, "V=%u", &verify);
  }
  if (verify > 1) verify = 1;

  tcpcli_t *tcpcli = (tcpcli_t *)malloc(sizeof(tcpcli_t));
  if (tcpcli == NULL) return NULL;
  memset(tcpcli, 0, sizeof(tcpcli_t));

  char port[256] = "";
  decodetcppath(path, tcpcli->svr.saddr, port, NULL, NULL, NULL, NULL);
  if (sscanf(port, "%u", &tcpcli->svr.port) < 1) {
    rscatprintf(msg, msize, "port error: %s", port);
    tracet(1, "opentcp: port error port=%s\n", port);
    free(tcpcli);
    return NULL;
  }

  if (tlsp == 0 && tcpcli->svr.port == 443) {
    tracet(3, "opentcpcli: enforcing TLS for port=%u\n", tcpcli->svr.port);
    tlsp = 1;
  }
  tcpcli->svr.tlsp = tlsp;
  tcpcli->svr.tlsverify = tlsp ? verify : 0;
  tcpcli->svr.tcon = 0;
  tcpcli->toinact = toinact;
  tcpcli->tirecon = ticonnect;
  tcpcli->svr.tact = tcpcli->svr.tdis = tickget();
  tcpcli->svr.retry = 1;  // Quick retry.

  if (tcpcli->svr.tlsp) {
#ifdef RTKLIB_OPENSSL
    tcpcli->svr.ctx = SSL_CTX_new(TLS_client_method());
    if (tcpcli->svr.ctx == NULL) {
      ssl_report_errors();
      free(tcpcli);
      tracet(1, "opentcpcli: TLS initialization error\n");
      rscatprintf(msg, msize, "TLS init error");
      return NULL;
    }
    SSL_CTX_set_options(tcpcli->svr.ctx, SSL_OP_NO_SSLv2);
    SSL_CTX_set_options(tcpcli->svr.ctx, SSL_OP_NO_SSLv3);
    SSL_CTX_set_options(tcpcli->svr.ctx, SSL_OP_IGNORE_UNEXPECTED_EOF);

    long current_mode = SSL_CTX_get_mode(tcpcli->svr.ctx);
    current_mode |= SSL_MODE_ENABLE_PARTIAL_WRITE;
    current_mode |= SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER;
    current_mode |= SSL_MODE_AUTO_RETRY;
    SSL_CTX_set_mode(tcpcli->svr.ctx, current_mode);

    // Set the password to use for encrypted files - to at least avoid
    // the default callback and to fail more gracefully.
    SSL_CTX_set_default_passwd_cb(tcpcli->svr.ctx, encpasswd);
    SSL_CTX_set_default_passwd_cb_userdata(tcpcli->svr.ctx, (void *)tlsclipasswd);

    if (tcpcli->svr.tlsverify) {
      SSL_CTX_set_verify(tcpcli->svr.ctx, SSL_VERIFY_PEER, NULL);
      if (tlsclicafile[0] != '\0') {
        if (SSL_CTX_load_verify_file(tcpcli->svr.ctx, tlsclicafile) != 1) {
          tracet(1, "opentcpcli: SSL load verify file failed for '%s'\n", tlsclicafile);
          ssl_report_errors();
        }
      }
      if (tlsclicadir[0] != '\0') {
        if (SSL_CTX_load_verify_dir(tcpcli->svr.ctx, tlsclicadir) != 1) {
          tracet(1, "opentcpcli: SSL load verify dir failed for '%s'\n", tlsclicadir);
          ssl_report_errors();
        }
      }
      if (SSL_CTX_set_default_verify_paths(tcpcli->svr.ctx) != 1) {
        tracet(1, "opentcpcli: SSL set default verify paths failed\n");
        ssl_report_errors();
      }
#ifdef WIN32
      if (SSL_CTX_load_verify_store(tcpcli->svr.ctx, "org.openssl.winstore://") != 1) {
        tracet(1, "opentcpcli: TLS load verify store failed\n");
        ssl_report_errors();
      }
#endif
    } else {
      tracet(3, "opentcpcli: SSL verify none\n");
      SSL_CTX_set_verify(tcpcli->svr.ctx, SSL_VERIFY_NONE, NULL);
    }

    tracet(5, "opentcpcli: TLS cert='%s' key='%s'\n", tlsclicertfile, tlsclikeyfile);
    if (tlsclicertfile[0] != '\0' &&
        SSL_CTX_use_certificate_file(tcpcli->svr.ctx, tlsclicertfile, SSL_FILETYPE_PEM) <= 0) {
      ssl_report_errors();
      free(tcpcli);
      tracet(1, "opentcpcli tls: certificate use failed for '%s'\n", tlssvrcertfile);
      rscatprintf(msg, msize, "TLS cert use failed");
      return NULL;
    }
    if (tlsclikeyfile[0] != '\0' &&
        SSL_CTX_use_PrivateKey_file(tcpcli->svr.ctx, tlsclikeyfile, SSL_FILETYPE_PEM) <= 0) {
      ssl_report_errors();
      free(tcpcli);
      tracet(1, "opentcpcli tls: private use failed for '%s'\n", tlssvrkeyfile);
      rscatprintf(msg, msize, "TLS key use failed");
      return NULL;
    }
#else
    free(tcpcli);
    tracet(1, "opentcpcli: TLS not supported\n");
    rscatprintf(msg, msize, "TLS not supported");
    return NULL;
#endif
  }

  return tcpcli;
}

// Close TCP client ----------------------------------------------------------
static void closetcpcli(tcpcli_t *tcpcli) {
  tracet(3, "closetcpcli: sock=%" PRISOCK "\n", tcpcli->svr.sock);

  if (tcpcli->svr.tlsp) {
#ifdef RTKLIB_OPENSSL
    if (tcpcli->svr.ssl != NULL) {
      if (tcpcli->svr.state == 2 && SSL_shutdown(tcpcli->svr.ssl) != 1) {
        // Block here a little before giving up, to give some chance of a
        // graceful close. This usually occurs at the end of a session where
        // some delay is typically okay.
        for (unsigned i = 0; i < 10; i++) {
          char buff[1024];
          size_t nr;
          int res = SSL_read_ex(tcpcli->svr.ssl, buff, 1024, &nr);
          tracet(4, "closetcpcli tls: shutdown read res=%d nr=%zu\n", res, nr);
          if (res != 1) {
            int sslerr = SSL_get_error(tcpcli->svr.ssl, res);
            if (sslerr == SSL_ERROR_WANT_READ || sslerr == SSL_ERROR_WANT_WRITE) {
              tracet(5, "closetcpcli tls: shutdown read tls: read want=%d\n", sslerr);
              sleepms(50);
              continue;  // Retry.
            }
            if (sslerr == SSL_ERROR_ZERO_RETURN) {  // Graceful EOF.
              tracet(5, "closetcpcli tls: shutdown read tls: eof\n");
              break;
            }
            if (sslerr == SSL_ERROR_SYSCALL) {
              int err = errsock();
              tracet(4, "closetcpcli tls: shutdown unexpected sslerr=%d err=%d\n", sslerr, err);
            }
            ssl_report_errors();
            break;
          }
        }
      }
      SSL_free(tcpcli->svr.ssl);
      tcpcli->svr.ssl = NULL;
    }
    if (tcpcli->svr.ctx != NULL) {
      SSL_CTX_free(tcpcli->svr.ctx);
      tcpcli->svr.ctx = NULL;
    }
#else
    tracet(1, "closetcpcli tls: not supported\n");
    return;
#endif
  }

  closesocket(tcpcli->svr.sock);
  freeaddrinfo(tcpcli->svr.addrs);
  free(tcpcli);
}

// Wait socket connect -------------------------------------------------------
static unsigned waittcpcli(tcpcli_t *tcpcli, char *msg, size_t msize) {
  tracet(4, "waittcpcli: sock=%" PRISOCK " state=%u\n", tcpcli->svr.sock, tcpcli->svr.state);

  tcpcli->svr.want = 0;

  if (tcpcli->svr.state == 0) {  // Closed.
    if (!gentcpcli(tcpcli, msg, msize)) return 0;
  }
  if (tcpcli->svr.state == 1) {  // Wait.
    if (!consock(tcpcli, msg, msize)) return 0;
  }

  if (tcpcli->svr.tlsp) {
#ifdef RTKLIB_OPENSSL
    if (tcpcli->svr.state == 2) {
      if (tcpcli->svr.tlsstate == 1) {  // Connect.
        int ret = SSL_connect(tcpcli->svr.ssl);
        if (ret == 1) {
          tcpcli->svr.tlsstate = 2;            // Connected.
          tcpcli->svr.tcon = tcpcli->tirecon;  // Reset.
          tracet(3, "waittcpcli tls: connected socket=%" PRISOCK "\n", tcpcli->svr.sock);
          return 1;
        }
        int sslerr = SSL_get_error(tcpcli->svr.ssl, ret);
        if (sslerr == SSL_ERROR_WANT_READ) {
          tracet(5, "waittcpcli tls: connect want read\n");
          tcpcli->svr.want = 1;  // Read wait.
          return 0;              // To be retried.
        }
        if (sslerr == SSL_ERROR_WANT_WRITE) {
          tracet(5, "waittcpcli tls: connect want write\n");
          tcpcli->svr.want = 2;  // Write wait.
          return 0;              // To be retried.
        }
        if (sslerr == SSL_ERROR_ZERO_RETURN) {  // Graceful EOF.
          closesocket(tcpcli->svr.sock);
          SSL_free(tcpcli->svr.ssl);
          tcpcli->svr.ssl = NULL;
          tcpcli->svr.state = 0;
          tcpcli->svr.tlsstate = 0;  // Uninitialized.
          tcpcli->svr.tcon = backofftcon(&tcpcli->svr, tcpcli->tirecon);
          tcpcli->svr.tdis = tickget();
          tracet(4, "waittcpcli tls: eof\n");
          rsstrcpy(msg, msize, "TLS eof");
          return 0;
        }
        int err = 0;
        if (sslerr == SSL_ERROR_SYSCALL) err = errsock();
        closesocket(tcpcli->svr.sock);
        SSL_free(tcpcli->svr.ssl);
        tcpcli->svr.ssl = NULL;
        tcpcli->svr.state = 0;
        tcpcli->svr.tlsstate = 0;
        tcpcli->svr.tcon = backofftcon(&tcpcli->svr, tcpcli->tirecon);
        tcpcli->svr.tdis = tickget();
        tracet(2, "waittcpcli tls: unexpected sslerr=%d err=%d\n", sslerr, err);
        ssl_report_errors();
        rsstrcpy(msg, msize, "TLS error");
        return 0;
      }
      if (tcpcli->svr.tlsstate == 2) {  // Connected.
        if (tcpcli->toinact > 0 && (uint32_t)(tickget() - tcpcli->svr.tact) > tcpcli->toinact) {
          rsstrcpy(msg, msize, "timeout");
          tracet(2, "waittcpcli: inactive timeout sock=%" PRISOCK "\n", tcpcli->svr.sock);
          discontcp(&tcpcli->svr, tcpcli->tirecon);
          return 0;
        }
      }
    }
    return 1;
#else
    tracet(1, "waittcpcli tls: not supported\n");
    rsstrcpy(msg, msize, "TLS not supported");
    discontcp(&tcpcli->svr, tcpcli->tirecon);
    return 0;
#endif
  }

  if (tcpcli->svr.state == 2) {  // Connected.
    if (tcpcli->toinact > 0) {
      uint32_t ti = (uint32_t)(tickget() - tcpcli->svr.tact);
      if (ti > 0 && ti > tcpcli->toinact) {
        rsstrcpy(msg, msize, "timeout");
        tracet(2, "waittcpcli: inactive timeout sock=%" PRISOCK "\n", tcpcli->svr.sock);
        discontcp(&tcpcli->svr, tcpcli->tirecon);
        return 0;
      }
    }
  }
  return 1;
}

// Read TCP client -----------------------------------------------------------
static size_t readtcpcli(tcpcli_t *tcpcli, uint8_t *buff, size_t size, size_t start, size_t n,
                         char *msg, size_t msize) {
  tracet(4, "readtcpcli: sock=%" PRISOCK " start=%zu n=%zu\n", tcpcli->svr.sock, start, n);

  if (!waittcpcli(tcpcli, msg, msize)) return 0;

  int err;
  ssize_t nr = recv_nb(&tcpcli->svr, buff, size, start, n, &err);
  if (nr < 0) {
    if (err) {
      tracet(2, "readtcpcli: recv error sock=%" PRISOCK " err=%d\n", tcpcli->svr.sock, err);
      rssnprintf(msg, msize, "recv error (%d)", err);
    } else {
      rsstrcpy(msg, msize, "disconnected");
    }
    discontcp(&tcpcli->svr, tcpcli->tirecon);
    return 0;
  }
  tracet(5, "readtcpcli: exit sock=%" PRISOCK " nr=%zd\n", tcpcli->svr.sock, nr);
  if (nr > 0) {
    tcpcli->svr.tact = tickget();
    return (size_t)nr;
  }
  return 0;
}
// Write TCP client ----------------------------------------------------------
static size_t writetcpcli(tcpcli_t *tcpcli, const uint8_t *buff, size_t size, size_t n, char *msg,
                          size_t msize) {
  tracet(3, "writetcpcli: sock=%" PRISOCK " state=%u n=%zu\n", tcpcli->svr.sock, tcpcli->svr.state,
         n);

  if (!waittcpcli(tcpcli, msg, msize)) return 0;

  int err;
  ssize_t ns = send_nb(&tcpcli->svr, buff, size, 0, n, &err);
  if (ns < 0) {
    if (err) {
      tracet(2, "writetcp: send error sock=%" PRISOCK " err=%d\n", tcpcli->svr.sock, err);
      rssnprintf(msg, msize, "send error (%d)", err);
    }
    discontcp(&tcpcli->svr, tcpcli->tirecon);
    return 0;
  }
  tracet(5, "writetcpcli: exit sock=%" PRISOCK " ns=%zd\n", tcpcli->svr.sock, ns);
  if (ns > 0) {
    tcpcli->svr.tact = tickget();
    return (size_t)ns;
  }
  return 0;
}
static void wanttcpcli(tcpcli_t *tcpcli, unsigned op, wantset_t *wantset) {
  tracet(4, "wanttcpcli: op=%u\n", op);

  if (tcpcli->svr.state == 0) return;

  fd_set *rs = &wantset->rs, *ws = &wantset->ws;

  if (tcpcli->svr.want != 0) {
    if (tcpcli->svr.want & 1) FD_SET(tcpcli->svr.sock, rs);
    if (tcpcli->svr.want & 2) FD_SET(tcpcli->svr.sock, ws);
    tcpcli->svr.want = 0;
#ifndef WIN32
    int n = 1 + (int)tcpcli->svr.sock;
    if (n > wantset->nfds) wantset->nfds = n;
#endif
  }
  if (op & 1) {
    FD_SET(tcpcli->svr.sock, rs);
#ifndef WIN32
    int n = 1 + (int)tcpcli->svr.sock;
    if (n > wantset->nfds) wantset->nfds = n;
#endif
  }
}
// Get state TCP client ------------------------------------------------------
static int statetcpcli(const tcpcli_t *tcpcli) { return tcpcli ? tcpcli->svr.state : 0; }
// Get extended state TCP client ---------------------------------------------
static int statextcpcli(tcpcli_t *tcpcli, char *msg, size_t msize) {
  (void)msg;
  (void)msize;
  return tcpcli ? tcpcli->svr.state : 0;
}
// Base64 encoder ------------------------------------------------------------
static void encbase64(char *str, size_t size, const uint8_t *byte, size_t n) {
  const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  tracet(4, "encbase64: n=%zu\n", n);

  size_t j = 0;
  for (size_t i = 0; i / 8 < n;) {
    unsigned b = 0;
    for (unsigned k = 0; k < 6; k++, i++) {
      b <<= 1;
      if (i / 8 < n) b |= (byte[i / 8] >> (7 - i % 8)) & 0x1;
    }
    RBOUNDSCHECK(str, size, j);
    str[j++] = table[b];
  }
  while (j & 0x3) {
    RBOUNDSCHECK(str, size, j);
    str[j++] = '=';
  }
  RBOUNDSCHECK(str, size, j);
  str[j] = '\0';
  tracet(5, "encbase64: str=%s\n", str);
}

// Peek for the first HTTP request or response status line, terminated by
// CR-LF, and copy it into the 'line' string. Permit a missing carriage
// return. The headerstart output is placed just after this line.  Return (-1:
// error, overflow, 0: fail, continue reading, 1: success)
static int find_http_first_line(const char *buf, size_t nin, char *line, size_t lsize,
                                size_t *headerstart) {
  *headerstart = 0;
  size_t i;
  for (i = 0; i < nin && i < lsize - 1; i++) {
    if (buf[i] == '\r' && i + 1 < nin && buf[i + 1] == '\n') {
      line[i] = '\0';
      *headerstart = i + 2;
      break;
    }
    if (buf[i] == '\n') {
      line[i] = '\0';
      *headerstart = i + 1;
      break;
    }
    line[i] = buf[i];
  }
  if (i >= lsize - 1) {
    tracet(2, "find_http_first_line: request or status line overflow nin=%zu\n", nin);
    return -1;
  }
  return *headerstart > 0 ? 1 : 0;
}

// Search for the given header in a HTTP response, and return the value or
// NULL if not found.
static char *get_http_header(const char *buf, size_t len, const char *header) {
  size_t vidx = 0, vlen = 0, i = 0;
  while (1) {
    size_t start = i, sep = i;
    // Search for the end of the next line, also noting the first colon.
    for (; i < len; i++) {
      if (buf[i] == '\r' || buf[i] == '\n') break;
      if (buf[i] == ':' && sep == start) sep = i;
    }
    if (i >= len) break;
    if (sep > start) {
      size_t hlen = sep - start;
      // Test the header, case-insensitive.
      if (hlen == strlen(header)) {
        size_t j;
        for (j = 0; j < hlen; j++) {
          if (tolower(header[j]) != tolower(buf[start + j])) break;
        }
        if (j >= hlen) {
          // Strip leading space.
          size_t vs = sep + 1;
          while (vs < i && buf[vs] == ' ') vs++;
          vlen = i - vs;
          vidx = vs;
          break;
        }
      }
    }
    // Skip the line suffix.
    if (buf[i] == '\r' && i + 1 < len && buf[i + 1] == '\n')
      i += 2;
    else if (buf[i] == '\r')
      i++;
    else if (buf[i] == '\n')
      i++;
  }
  if (vidx == 0) return NULL;
  char *value = malloc(vlen + 1);
  if (value == NULL) return NULL;
  memcpy(value, buf + vidx, vlen);
  value[vlen] = '\0';
  tracet(5, "get_http_header: '%s' found '%s'\n", header, value);
  return value;
}

// Peek for end of the HTTP headers in the input buffer terminated by an empty
// header. The headerend output is placed after the line terminators of the
// last header and the contenstart output place at the state of the
// content. Be tolerant of missing carriage returns. Return true is found.
static unsigned find_http_headers(const char *buf, size_t nin, size_t headerstart,
                                  size_t *headerend, size_t *contentstart) {
  *headerend = headerstart;
  *contentstart = headerstart;
  if (headerstart < nin && buf[headerstart] == '\n') {
    *contentstart = headerstart + 1;
  } else if (headerstart + 1 < nin && buf[headerstart] == '\r' && buf[headerstart + 1] == '\n') {
    *contentstart = headerstart + 2;
  } else {
    const char *phe = strstr(buf + headerstart, "\r\n\r\n");
    if (phe) {
      *headerend = phe - buf + 2;
      *contentstart = *headerend + 2;
    } else {
      phe = strstr(buf + headerstart, "\n\r\n");
      if (phe) {
        *headerend = phe - buf + 1;
        *contentstart = *headerend + 2;
      } else {
        phe = strstr(buf + headerstart, "\r\n\n");
        if (phe) {
          *headerend = phe - buf + 2;
          *contentstart = *headerend + 1;
        } else {
          phe = strstr(buf + headerstart, "\n\n");
          if (phe) {
            *headerend = phe - buf + 1;
            *contentstart = *headerend + 1;
          }
        }
      }
    }
  }
  return *contentstart > headerstart;
}

// Set the default NTRIP version used in client path requests.
// This is effective when the path options do not declare a version.
// Note the caster is always NTRIP v2 capable.
void strsetntripver(unsigned ver) {
  if (ver < 1) ver = 1;
  if (ver > 2) ver = 2;
  ntripcliver = ver;
}

// Send a NTRIP client source request ------------------------------------------
//
// Note chunked transfer encoding is indicated for the output stream when
// using version 2 and it is assumed the server is capable of accepting
// this. A version 1 server would not be expected to accept a version 2 POST
// method request and if the server should reply with a version 1 response
// then output chunked transfer is disabled.
//
// Note the POST method need not send cache control headers and the version 1
// SOURCE method is specialized and not relevant to caching.
//
static unsigned reqntrip_s(ntrip_t *ntrip) {
  tracet(3, "reqntrip_s: ver=%u state=%u\n", ntrip->ver, ntrip->state);

  unsigned ver = ntrip->ver;
  ntrip->chunkin = ntrip->chunkout = 0;

  // The output buffer is assumed to be large enough for the request.
  char *obuf = (char *)ntrip->obuf;
  size_t size = ntrip->obufsize;
  obuf[0] = '\0';

  if (ver == 2) {
    rscatprintf(obuf, size, "POST %s/%s HTTP/1.1\r\n", ntrip->url, ntrip->mntpnt);
    rscatprintf(obuf, size, "Host: %s:%u\r\n", ntrip->tcp->svr.saddr, ntrip->tcp->svr.port);
    rscatprintf(obuf, size, "Ntrip-Version: Ntrip/2.0\r\n");
    if (ntrip->str[0] != '\0') rscatprintf(obuf, size, "Ntrip-STR: %s\r\n", ntrip->str);
    rscatprintf(obuf, size, "User-Agent: NTRIP %s\r\n", NTRIP_AGENT);
    rscatprintf(obuf, size, "Connection: close\r\n");
    rscatprintf(obuf, size, "Transfer-Encoding: chunked\r\n");
    char user[514];
    rssnprintf(user, sizeof(user), "%s:%s", ntrip->user, ntrip->passwd);
    rscatprintf(obuf, size, "Authorization: Basic ");
    size_t len = strlen(obuf);
    encbase64(obuf + len, size - len, (uint8_t *)user, strlen(user));
    rscatprintf(obuf, size, "\r\n");
    rscatprintf(obuf, size, "\r\n");
    ntrip->nout = strlen(obuf);
    ntrip->chunkin = 0;
    ntrip->chunkout = 1;
    ntrip->state = 1;
    tracet(3, "reqntrip_s: send request state=%u ns=%zu\n", ntrip->state, ntrip->nout);
    return 1;
  }

  // NTRIP version 1.
  rscatprintf(obuf, size, "SOURCE %s %s\r\n", ntrip->passwd, ntrip->mntpnt);
  rscatprintf(obuf, size, "Source-Agent: NTRIP %s\r\n", NTRIP_AGENT);
  if (ntrip->str[0] != '\0') rscatprintf(obuf, size, "STR: %s\r\n", ntrip->str);
  rscatprintf(obuf, size, "\r\n");
  ntrip->nout = strlen(obuf);
  tracet(3, "reqntrip_s: send request state=%u ns=%zu\n", ntrip->state, ntrip->nout);
  ntrip->chunkin = ntrip->chunkout = 0;
  ntrip->state = 1;
  return 1;
}

// Send a NTRIP client get request ---------------------------------------------
static unsigned reqntrip_c(ntrip_t *ntrip) {
  tracet(3, "reqntrip_c: ver=%u state=%u\n", ntrip->ver, ntrip->state);

  unsigned ver = ntrip->ver;
  // Note the NTRIP client request does not use chunked transfer encoding
  // for the output stream, even for version 2, but it will accept chunked
  // transfer encoded data from the server as determined from the respponse.
  ntrip->chunkin = ntrip->chunkout = 0;

  // The output buffer is assumed to be large enough for the request.
  char *obuf = (char *)ntrip->obuf;
  size_t obufsize = ntrip->obufsize;
  obuf[0] = '\0';

  rscatprintf(obuf, obufsize, "GET %s/%s HTTP/1.%d\r\n", ntrip->url, ntrip->mntpnt,
              ver == 2 ? 1 : 0);
  rscatprintf(obuf, obufsize, "Host: %s:%u\r\n", ntrip->tcp->svr.saddr, ntrip->tcp->svr.port);
  if (ver == 2) rscatprintf(obuf, obufsize, "Ntrip-Version: Ntrip/2.0\r\n");
  rscatprintf(obuf, obufsize, "User-Agent: NTRIP %s\r\n", NTRIP_AGENT);

  if (!*ntrip->user) {
    rscatprintf(obuf, obufsize, "Accept: */*\r\n");
    rscatprintf(obuf, obufsize, "Connection: close\r\n");
  } else {
    char user[514];
    rssnprintf(user, sizeof(user), "%s:%s", ntrip->user, ntrip->passwd);
    rscatprintf(obuf, obufsize, "Authorization: Basic ");
    size_t len = strlen(obuf);
    encbase64(obuf + len, obufsize - len, (uint8_t *)user, strlen(user));
    rscatprintf(obuf, obufsize, "\r\n");
  }
  rscatprintf(obuf, obufsize, "\r\n");
  ntrip->nout = strlen(obuf);

  tracet(3, "reqntrip_c: send request state=%u ns=%zu\n", ntrip->state, ntrip->nout);
  ntrip->state = 1;
  return 1;
}

// Test NTRIP client response from a caster ------------------------------------
//
// Note the clients version sets the version indicated in the request, and
// NTRIP v1 responses are accepted even if the client is set to v2.
//
static unsigned rspntrip(ntrip_t *ntrip, char *msg, size_t msize) {
  unsigned ver = ntrip->ver;
  tracet(3, "rspntrip: ver=%u state=%u nin=%zu\n", ver, ntrip->state, ntrip->nin);

  // Peek for a NTRIP v1 error response, that might not be a terminated line.
  char *ibuf = (char *)ntrip->ibuf;
  if (ntrip->nin >= 5 && strncmp(ibuf, "ERROR", 5) == 0) {
    // NTRIP v1 response error. Copy over and report the error.
    char reason[32];
    size_t i = 0;
    while (i < 32 - 1 && i < ntrip->nin && ibuf[i] != '\r' && ibuf[i] != '\n') {
      reason[i] = ibuf[i];
      i++;
    }
    reason[i] = '\0';
    rsstrcpy(msg, msize, reason);
    ntrip->nin = ntrip->nout = 0;
    ntrip->ibuf[0] = '\0';
    ntrip->state = 0;  // Closed.
    discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
    return 0;
  }

  // Peek for a terminated response status line.
  char statusline[1024];
  size_t headerstart = 0;
  int rline = find_http_first_line(ibuf, ntrip->nin, statusline, sizeof(statusline), &headerstart);
  if (rline < 0) {
    rsstrcpy(msg, msize, "response message overflow");
    tracet(2, "rspntrip: response message overflow nin=%zu\n", ntrip->nin);
    ntrip->nin = ntrip->nout = 0;
    ibuf[0] = '\0';
    ntrip->state = 0;  // Closed.
    discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
    return 0;
  }
  if (rline == 0) return 0;  // Continue reading.
  tracet(2, "rspntrip status='%s'\n", statusline);

  ntrip->chunkin = 0;
  ntrip->chunkrem = -1;

  // The ICY NTRIP version 1 response has no HTTP headers, not even an empty
  // terminator, so handle this as a special case first.
  if (strcmp(statusline, "ICY 200 OK") == 0) {
    if (ntrip->ver > 1) {
      // Sent a ver 2 request and got back a ver 1 response. Assume the server
      // is ver 1 and turn off the chunk transfer encoding as would have been
      // declared in the request header.
      tracet(1, "rspntrip: warning ver 1 response to ver 2 request\n");
      ntrip->chunkout = 0;
    }
    // Discard the response line.
    ntrip->nin -= headerstart;
    memmove(ibuf, ibuf + headerstart, ntrip->nin);
    ntrip->state = 2;                            // Established.
    ntrip->tcp->svr.tcon = ntrip->tcp->tirecon;  // Reset.
    rssnprintf(msg, msize, "%s/%s", ntrip->tcp->svr.saddr, ntrip->mntpnt);
    tracet(3, "rspntrip: response ok nin=%zu\n", ntrip->nin);
    return 1;
  }

  // Test for HTTP errors, before reading all the headers, in case the headers
  // are not properly terminated.
  if ((strncmp(statusline, "HTTP/1.0 ", 9) == 0 && strncmp(statusline + 9, "200", 3) != 0) ||
      (strncmp(statusline, "HTTP/1.1 ", 9) == 0 && strncmp(statusline + 9, "200", 3) != 0)) {
    // HTTP error or unexpected response. Copy over and report the error.
    char reason[32] = "";
    size_t i = 0;
    while (i < 32 - 1 && statusline[9 + i] != '\0') {
      reason[i] = statusline[9 + i];
      i++;
    }
    reason[i] = '\0';
    rsstrcpy(msg, msize, reason);
    ntrip->nin = ntrip->nout = 0;
    ntrip->ibuf[0] = '\0';
    ntrip->state = 0;  // Closed.
    discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
    return 0;
  }

  // Peek for the end of the HTTP headers, and if not found then continue reading.
  size_t headerend = headerstart, contentstart = headerstart;
  if (!find_http_headers(ibuf, ntrip->nin, headerstart, &headerend, &contentstart)) {
    if (ntrip->nin >= 8192 || ntrip->nin >= ntrip->ibufsize) {
      // Practical limit or buffer full.
      rscatprintf(msg, msize, "headers overflow");
      tracet(2, "rspntrip: response headers overflow nin=%zu\n", ntrip->nin);
      ntrip->nin = ntrip->nout = 0;
      ntrip->ibuf[0] = '\0';
      ntrip->state = 0;  // Closed.
      discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
      return 0;
    }
    return 0;  // Continue reading.
  }

  const char *headers = ibuf + headerstart;
  size_t headersize = headerend - headerstart;

  if (ntrip->type == 1 && strcmp(statusline, "SOURCETABLE 200 OK") == 0) {
    // NTRIP version 1 source table GET request.
    if (!*ntrip->mntpnt) {  // Source table request
      // Discard all input buffer content before the content start.
      ntrip->nin -= contentstart;
      memmove(ibuf, ibuf + contentstart, ntrip->nin);
      ntrip->state = 2;                            // Established.
      ntrip->tcp->svr.tcon = ntrip->tcp->tirecon;  // Reset.
      rsstrcpy(msg, msize, "source table received");
      tracet(3, "rspntrip: receive source table nin=%zu\n", ntrip->nin);
      ntrip->chunkin = ntrip->chunkout = 0;
      ntrip->chunkrem = -1;
      return 1;
    }
    // NTRIP v1 sends the source table if the mount point is not found.
    rsstrcpy(msg, msize, "no mountp. reconnect...");
    tracet(3, "rspntrip: no mount point nin=%zu\n", ntrip->nin);
    ntrip->nin = ntrip->nout = 0;
    discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
    return 0;
  }

  ntrip->chunkin = 0;
  ntrip->chunkrem = -1;
  char *tfrenc = get_http_header(headers, headersize, "Transfer-Encoding");
  if (tfrenc != NULL) {
    if (strncmp(tfrenc, "chunked", 7) == 0) ntrip->chunkin = 1;
  }
  free(tfrenc);

  if (strcmp(statusline, "HTTP/1.1 200 OK") == 0 || strcmp(statusline, "HTTP/1.0 200 OK") == 0) {
    if (ntrip->type == 0) {
      // NTRIP v2 response for a POST.
      // Discard all input buffer content before the content start.
      ntrip->nin -= contentstart;
      memmove(ibuf, ibuf + contentstart, ntrip->nin);
      ntrip->state = 2;                            // Established.
      ntrip->tcp->svr.tcon = ntrip->tcp->tirecon;  // Reset.
      rssnprintf(msg, msize, "%s/%s", ntrip->tcp->svr.saddr, ntrip->mntpnt);
      tracet(3, "rspntrip: response ok nin=%zu\n", ntrip->nin);
      return 1;
    }

    // NTRIP version 2 response.
    unsigned contenc = 0;
    char *cnttype = get_http_header(headers, headersize, "Content-Type");
    if (cnttype != NULL) {
      if (strncmp(cnttype, "text/plain", 10) == 0)
        contenc = 1;
      else if (strncmp(cnttype, "gnss/sourcetable", 16) == 0)
        contenc = 2;
      else if (strncmp(cnttype, "gnss/data", 16) == 0)
        contenc = 3;
    }
    free(cnttype);

    if (ntrip->mntpnt[0] == '\0' || contenc == 1 || contenc == 2) {
      // Expecting a source table response, or got a source table response.
      // Discard all input buffer content before the content start.
      ntrip->nin -= contentstart;
      memmove(ibuf, ibuf + contentstart, ntrip->nin);
      ntrip->state = 2;                            // Established.
      ntrip->tcp->svr.tcon = ntrip->tcp->tirecon;  // Reset.
      rsstrcpy(msg, msize, "source table received");
      tracet(3, "rspntrip: receive source table nin=%zu\n", ntrip->nin);
      return 1;
    }

    if (contenc == 3) {  // GNSS/data
      // Discard all input buffer content before the content start.
      ntrip->nin -= contentstart;
      memmove(ibuf, ibuf + contentstart, ntrip->nin);
      ntrip->state = 2;                            // Established.
      ntrip->tcp->svr.tcon = ntrip->tcp->tirecon;  // Reset.
      rssnprintf(msg, msize, "%s/%s", ntrip->tcp->svr.saddr, ntrip->mntpnt);
      tracet(3, "rspntrip: response ok nin=%zu\n", ntrip->nin);
      return 1;
    }

    rsstrcpy(msg, msize, "no mountp. reconnect...");
    tracet(2, "rspntrip: no mount point nin=%zu\n", ntrip->nin);
    ntrip->nin = ntrip->nout = 0;
    ibuf[0] = '\0';
    ntrip->state = 0;  // Closed.
    discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
  }
  tracet(5, "rspntrip: exit state=%u nin=%zu\n", ntrip->state, ntrip->nin);
  return 0;
}

// Wait NTRIP request/response -----------------------------------------------
// Client to a server or caster.
static unsigned waitntrip(ntrip_t *ntrip, char *msg, size_t msize) {
  tracet(4, "waitntrip: ver=%u state=%u nin=%zu  nout=%zu\n", ntrip->ver, ntrip->state, ntrip->nin,
         ntrip->nout);

  if (!waittcpcli(ntrip->tcp, msg, msize)) {
    ntrip->state = 0;  // Not yet connected.
    return 0;
  }

  // TCP/IP connected.
  if (ntrip->state == 0) {
    ntrip->nout = 0;
    // Reset the msg buffer.
    msg[0] = '\0';
    // Prepare request.
    unsigned succ = ntrip->type == 0 ? reqntrip_s(ntrip) : reqntrip_c(ntrip);
    if (!succ) return 0;
    ntrip->state = 1;  // Wait to be sent.
  }

  if (ntrip->state == 1) {
    // Waiting to send the request and get a response.
    if (ntrip->nout > 0) {
      // Sending request.
      int err;
      ssize_t ns =
          send_nb(&ntrip->tcp->svr, (uint8_t *)ntrip->obuf, ntrip->obufsize, 0, ntrip->nout, &err);
      if (ns < 0) {
        if (err) tracet(2, "waitntrip: response write error err=%d\n", err);
        ntrip->nin = ntrip->nout = 0;
        ntrip->state = 0;  // Closed.
        discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
        return 0;
      }
      if ((size_t)ns <= ntrip->nout) {
        size_t rem = ntrip->nout - ns;
        memmove(ntrip->obuf, ntrip->obuf + ns, rem);
        ntrip->nout = rem;
      }
    }
    // Waiting for response.
    size_t nmax = ntrip->ibufsize - ntrip->nin - 1;
    int err;
    ssize_t n =
        recv_nb(&ntrip->tcp->svr, (uint8_t *)ntrip->ibuf, ntrip->ibufsize, ntrip->nin, nmax, &err);
    if (n < 0) {
      if (err) {
        tracet(2, "waitntrip: recv error sock=%" PRISOCK " err=%d\n", ntrip->tcp->svr.sock, err);
      }
      ntrip->nin = ntrip->nout = 0;
      ntrip->state = 0;  // Closed.
      discontcp(&ntrip->tcp->svr, backofftcon(&ntrip->tcp->svr, ntrip->tcp->tirecon));
      return 0;
    }
    if (n <= 0) return 0;
    ntrip->nin += n;
    // Wait response
    return rspntrip(ntrip, msg, msize);
  }

  if (ntrip->state == 2) {
    // Continue flushing the output buffer.
    if (ntrip->nout > 0) {
      int err;
      ssize_t nsn = send_nb(&ntrip->tcp->svr, ntrip->obuf, ntrip->obufsize, 0, ntrip->nout, &err);
      if (nsn < 0) {
        if (err) {
          tracet(2, "waitntrip: send error sock=%" PRISOCK " err=%d\n", ntrip->tcp->svr.sock, err);
        }
        ntrip->nin = ntrip->nout = 0;
        ntrip->state = 0;  // Closed.
        discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
        return 0;
      }
      if ((size_t)nsn <= ntrip->nout) {
        size_t rem = ntrip->nout - nsn;
        memmove(ntrip->obuf, ntrip->obuf + nsn, rem);
        ntrip->nout = rem;
      }
      if (nsn > 0) ntrip->tcp->svr.tact = tickget();
    }
  }
  return 1;
}
// Open NTRIP client or source -------------------------------------------------
// type: 0: source, 1: client
static ntrip_t *openntrip(const char *path, unsigned type, char *msg, size_t msize) {
  tracet(3, "openntrip: path=%s\n", path);
  // Path options.
  // tlsp: 0: non-TLS, 1: TLS.
  unsigned ver = 0, tlsp = 0, verify = tlscliverify;
  for (const char *p = path; (p = strstr(p, "::")); p += 2) {
    if (p[2] == 'N')
      sscanf(p + 2, "N=%u", &ver);
    else if (p[2] == 'S')
      tlsp = 1;
    else if (p[2] == 'V')
      sscanf(p + 2, "V=%u", &verify);
  }
  if (ver > 2) ver = 2;
  if (verify > 1) verify = 1;

  ntrip_t *ntrip = (ntrip_t *)malloc(sizeof(ntrip_t));
  if (ntrip == NULL) return NULL;
  memset(ntrip, 0, sizeof(ntrip_t));
  ntrip->ibufsize = ntrip->obufsize = buffsize;
  ntrip->ibuf = (uint8_t *)malloc(ntrip->ibufsize);
  ntrip->obuf = (uint8_t *)malloc(ntrip->obufsize);
  if (ntrip->ibuf == NULL || ntrip->obuf == NULL) {
    free(ntrip->ibuf);
    free(ntrip->obuf);
    free(ntrip);
    return NULL;
  }

  ntrip->state = 0;    // Connect.
  ntrip->type = type;  // 0:server,1:client
  ntrip->nin = ntrip->nout = 0;
  ntrip->url[0] = '\0';
  ntrip->mntpnt[0] = ntrip->user[0] = ntrip->passwd[0] = ntrip->str[0] = '\0';

  // Decode TCP/NTRIP path
  char addr[256] = "", port[256] = "";
  decodetcppath(path, addr, port, ntrip->user, ntrip->passwd, ntrip->mntpnt, ntrip->str);

  // Use default port if no port specified
  if (port[0] == '\0' || strcmp(port, "0") == 0) {
    if (tlsp)
      rssnprintf(port, sizeof(port), "%d", type ? NTRIP_CLI_TLS_PORT : NTRIP_SVR_TLS_PORT);
    else
      rssnprintf(port, sizeof(port), "%d", type ? NTRIP_CLI_PORT : NTRIP_SVR_PORT);
  }
  if (tlsp == 0 && atoi(port) == 443) {
    tracet(3, "openntrip: enforcing TLS for port=%s\n", port);
    tlsp = 1;
  }
  // Default to version 2 if TLS is enabled, otherwise use the default.
  if (ver == 0) ver = tlsp ? 2 : ntripcliver;
  ntrip->ver = ver;

  char addresc[256 * 3], portesc[256 * 3];
  strurlescape(addr, 0, SIZE_MAX, addresc, sizeof(addresc));
  strurlescape(port, 0, SIZE_MAX, portesc, sizeof(portesc));
  char tpath[MAXSTRPATH];
  rssnprintf(tpath, sizeof(tpath), "%s:%s%s::V=%u", addresc, portesc, tlsp ? "::S" : "", verify);

  // NTRIP access via proxy server.
  if (*proxyaddr) {
    rssnprintf(ntrip->url, sizeof(ntrip->url), "http%s://%s", tlsp ? "s" : "", tpath);
    rssnprintf(tpath, sizeof(tpath), "%s", proxyaddr);
  }
  // Open TCP client stream.
  ntrip->tcp = opentcpcli(tpath, msg, msize);
  if (ntrip->tcp == NULL) {
    tracet(2, "openntrip: opentcp error\n");
    free(ntrip->ibuf);
    free(ntrip->obuf);
    free(ntrip);
    return NULL;
  }
  return ntrip;
}

// Close NTRIP ---------------------------------------------------------------
static void closentrip(ntrip_t *ntrip) {
  tracet(3, "closentrip: state=%u\n", ntrip->state);
  closetcpcli(ntrip->tcp);
  free(ntrip->ibuf);
  free(ntrip->obuf);
  free(ntrip);
}

// Decode a chunked transfer encoded input buffer.
//
// buff - the output buffer
//
// n - the maximum number of bytes to be copied to the output buffer.
//
// ibuf - the input buffer, the data is decoded from start and after being
// consumed the buffer content is moved down.
//
// nin - the number of bytes available in the input buffer. Updated as bytes
// are consumed.
//
// rem - the number of bytes remaining to be copied out from the current
// chunk, a working state updated as content is copied out. The value of -1
// indicates that the header with the chunk size is next to be decoded, and
// this is the initial value. The value of 0 indicates that the trailing CR-LF
// is yet to have been decoded.
//
// Returns (-1: on EOF or error, 0: more input needed: 1+: bytes copied out)
//
static ssize_t decode_chunk(uint8_t *buff, size_t n, uint8_t *ibuf, size_t *nin, ssize_t *rem) {
  size_t nr = 0;
  tracet(5, "decode_chunk: n=%zu nin=%zu rem=%zd\n", n, *nin, *rem);
  while (n > 0 && *nin > 0) {
    if (*rem < 0) {
      // Look for the header terminator CR-LF.
      size_t content = 0;
      for (size_t i = 0; i + 2 < *nin; i++) {
        if (ibuf[i] == '\r' && ibuf[i + 1] == '\n') {
          content = i + 2;
          break;
        }
        // Permit a lone LF terminator.
        if (ibuf[i] == '\n') {
          content = i + 1;
          break;
        }
      }
      if (content == 0) {
        // Do not have the full header in the input buffer. Return with any
        // data that has been copied out to continue filling the input buffer.
        break;
      }
      // Decode the chuck size.
      char *p;
      size_t size = (size_t)strtoul((char *)ibuf, &p, 16);
      if (p == (char *)ibuf) {
        tracet(2, "decode_chunk: error parsing chunk size nr=%zu\n", nr);
        // If some data has been copied out then firstly return that.
        if (nr > 0) break;
        return -1;
      }
      tracet(5, "decode_chunk: size=%zu\n", size);
      if (size == 0) {  // EOF.
        tracet(2, "decode_chunk: end-of-file nr=%zu\n", nr);
        // If some data has been copied then firstly return that.
        if (nr > 0) break;
        return -1;
      }
      *rem = size;
      *nin -= content;
      memmove(ibuf, ibuf + content, *nin);
    }
    if (*rem > 0) {
      // Available in this chunk.
      size_t avail = (size_t)*rem > *nin ? *nin : (size_t)*rem;
      // Amount to copy out.
      size_t cp = n > avail ? avail : n;
      memcpy(buff + nr, ibuf, cp);
      *nin -= cp;
      *rem -= cp;
      n -= cp;
      nr += cp;
      memmove(ibuf, ibuf + cp, *nin);
    }
    if (*rem == 0) {
      // Expect the trailing CR-LF
      if (*nin >= 2 && ibuf[0] == '\r' && ibuf[1] == '\n') {
        *nin -= 2;
        *rem = -1;  // The next header.
        memmove(ibuf, ibuf + 2, *nin);
        continue;
      }
      // Permit a lone LF terminator.
      if (*nin >= 1 && ibuf[0] == '\n') {
        *nin -= 1;
        *rem = -1;  // The next header.
        memmove(ibuf, ibuf + 1, *nin);
        continue;
      }
      break;
    }
  }
  return nr;
}

// Read NTRIP ----------------------------------------------------------------
static size_t readntrip(ntrip_t *ntrip, uint8_t *buff, size_t size, size_t n, char *msg,
                        size_t msize) {
  tracet(4, "readntrip:\n");

  if (!waitntrip(ntrip, msg, msize)) return 0;

  // Read into the input buffer.
  tracet(5, "readntrip size=%zu n=%zu nin=%zu\n", size, n, ntrip->nin);
  if (ntrip->nin < ntrip->ibufsize)
    ntrip->nin += readtcpcli(ntrip->tcp, ntrip->ibuf, ntrip->ibufsize, ntrip->nin,
                             ntrip->ibufsize - ntrip->nin, msg, msize);

  if (ntrip->nin == 0) return 0;

  if (ntrip->chunkin) {
    ssize_t nr = decode_chunk(buff, n, ntrip->ibuf, &ntrip->nin, &ntrip->chunkrem);
    if (nr < 0) {
      rscatprintf(msg, msize, "end");
      return 0;
    }
    return nr;
  }

  // Read from the input buffer.
  size_t nin = ntrip->nin;
  if (nin <= n) {
    // Empty the buffer.
    RBOUNDSCHECK(ntrip->ibuf, ntrip->ibufsize, nin - 1);
    RBOUNDSCHECK(buff, size, nin - 1);
    memcpy(buff, ntrip->ibuf, nin);
    ntrip->nin = 0;
    return nin;
  }
  // Partial use of the input buffer
  RBOUNDSCHECK(ntrip->ibuf, ntrip->ibufsize, n - 1);
  RBOUNDSCHECK(buff, size, n - 1);
  memcpy(buff, ntrip->ibuf, n);
  RBOUNDSCHECK(ntrip->ibuf, ntrip->ibufsize, nin - 1);
  memmove(ntrip->ibuf, ntrip->ibuf + n, nin - n);
  ntrip->nin = nin - n;
  return n;
}

// Write NTRIP ---------------------------------------------------------------
//
// Returns the number of bytes written and 0 if there is no connection. The
// design is similar to writetcpsvr(), intended for streaming, but the output
// buffer is also used for chunked transfer encoding.
static size_t writentrip(ntrip_t *ntrip, const uint8_t *buff, size_t size, size_t n, char *msg,
                         size_t msize) {
  tracet(3, "writentrip: n=%zu\n", n);

  // This will also continue flushing the output buffers.
  if (!waitntrip(ntrip, msg, msize)) return 0;

  // Can early out in this case, and don't write chunks of size 0,
  if (n == 0) return 0;

  RBOUNDSCHECK(buff, size, n - 1);

  size_t chunkextra = 0;
  if (ntrip->chunkout) chunkextra = 10;

  // Append the content onto the output buffer.
  if (ntrip->nout + n + chunkextra > ntrip->obufsize) {
    tracet(2, "writentrip: output buffer overflow\n");
    discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
    ntrip->state = 0;
    return 0;
  }
  if (ntrip->chunkout) {
    char chunksize[10] = "";
    rssnprintf(chunksize, sizeof(chunksize), "%x\r\n", n);
    size_t headerlen = strlen(chunksize);
    memcpy(ntrip->obuf + ntrip->nout, chunksize, headerlen);
    memcpy(ntrip->obuf + ntrip->nout + headerlen, buff, n);
    const char chunksuffix[] = "\r\n";
    memcpy(ntrip->obuf + ntrip->nout + headerlen + n, chunksuffix, 2);
    ntrip->nout += headerlen + n + 2;
  } else {
    memcpy(ntrip->obuf + ntrip->nout, buff, n);
    ntrip->nout += n;
  }

#if 1
  // Try sending it now.
  if (ntrip->nout > 0) {
    int err;
    ssize_t nsn = send_nb(&ntrip->tcp->svr, ntrip->obuf, ntrip->obufsize, 0, ntrip->nout, &err);
    if (nsn < 0) {
      if (err) {
        tracet(2, "writentrip: send error sock=%" PRISOCK " err=%d\n", ntrip->tcp->svr.sock, err);
      }
      discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
      ntrip->state = 0;
      return 0;
    }
    if ((size_t)nsn <= ntrip->nout) {
      size_t rem = ntrip->nout - nsn;
      memmove(ntrip->obuf, ntrip->obuf + nsn, rem);
      ntrip->nout = rem;
    }
    if (nsn > 0) ntrip->tcp->svr.tact = tickget();
  }
#endif

  return n;
}
static void wantntrip(ntrip_t *ntrip, unsigned op, wantset_t *wantset) {
  wanttcpcli(ntrip->tcp, op, wantset);
}
// Get state NTRIP -----------------------------------------------------------
static int statentrip(const ntrip_t *ntrip) {
  tracet(5, "statentrip: %u %u\n", ntrip->state, ntrip->tcp->svr.state);
  return !ntrip ? 0 : (ntrip->state == 0 ? ntrip->tcp->svr.state : ntrip->state);
}
// Get extended state NTRIP --------------------------------------------------
static int statexntrip(ntrip_t *ntrip, char *msg, size_t msize) {
  unsigned state = ntrip == NULL ? 0 : (ntrip->state == 0 ? ntrip->tcp->svr.state : ntrip->state);
  rscatprintf(msg, msize, "ntrip:\n");
  rscatprintf(msg, msize, "  state   = %u\n", state);
  if (state == 0) return 0;
  rscatprintf(msg, msize, "  type    = %u\n", ntrip->type);
  rscatprintf(msg, msize, "  nin     = %zu\n", ntrip->nin);
  rscatprintf(msg, msize, "  nout    = %zu\n", ntrip->nout);
  rscatprintf(msg, msize, "  url     = %s\n", ntrip->url);
  rscatprintf(msg, msize, "  mntpnt  = %s\n", ntrip->mntpnt);
  rscatprintf(msg, msize, "  user    = %s\n", ntrip->user);
  rscatprintf(msg, msize, "  passwd  = %s\n", ntrip->passwd);
  rscatprintf(msg, msize, "  str     = %s\n", ntrip->str);
  rscatprintf(msg, msize, "  svr:\n");
  statextcp(&ntrip->tcp->svr, msg, msize);
  return state;
}
// Open NTRIP caster ---------------------------------------------------------
static ntripc_t *openntripcas(const char *path, char *msg, size_t msize) {
  tracet(3, "openntripcas: path=%s\n", path);
  // Path options.
  // tlsp: 0: no TLS, 1: require TLS, 2: enable TLS but allow unencrypted.
  // The NTRIP version defaults to 0 accepting either version 1 or 2 requests.
  unsigned ver = 0, type = 0, tlsp = 0, verify = tlssvrverify;
  for (const char *p = path; (p = strstr(p, "::")); p += 2) {
    if (p[2] == 'S')
      tlsp = 1;
    else if (p[2] == 'N')
      sscanf(p + 2, "N=%u", &ver);
    else if (p[2] == 'T')
      sscanf(p + 2, "T=%u", &type);
    else if (p[2] == 'A')
      tlsp = 2;
    else if (p[2] == 'V')
      sscanf(p + 2, "V=%u", &verify);
  }
  if (ver > 2) ver = 2;
  if (type > 2) verify = 2;
  if (verify > 1) verify = 1;

  ntripc_t *ntripc = (ntripc_t *)malloc(sizeof(ntripc_t));
  if (ntripc == NULL) return NULL;
  memset(ntripc, 0, sizeof(ntripc_t));
  ntripc->ver = ver;
  ntripc->type = type;
  ntripc->state = 0;
  ntripc->mntpnt[0] = ntripc->user[0] = ntripc->passwd[0] = ntripc->srctbl[0] = '\0';
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    ntripc->con[ci].state = ntripc->con[ci].substate = 0;
    ntripc->con[ci].ibufsize = ntripc->con[ci].obufsize = buffsize;
    ntripc->con[ci].ibuf = ntripc->con[ci].obuf = NULL;
    ntripc->con[ci].nin = ntripc->con[ci].nout = 0;
  }
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    ntripc->con[ci].ibuf = (uint8_t *)malloc(ntripc->con[ci].ibufsize);
    ntripc->con[ci].obuf = (uint8_t *)malloc(ntripc->con[ci].obufsize);
    if (ntripc->con[ci].ibuf == NULL || ntripc->con[ci].obuf == NULL) goto fail;
  }

  // Decode TCP/NTRIP path
  char addr[256] = "", port[256] = "";
  decodetcppath(path, addr, port, ntripc->user, ntripc->passwd, ntripc->mntpnt, ntripc->srctbl);

  if (ntripc->mntpnt[0] == '\0') {
    tracet(2, "openntripcas: no mountpoint path=%s\n", path);
    goto fail;
  }
  // Use default port if no port specified
  if (port[0] == '\0' || strcmp(port, "0") == 0)
    rssnprintf(port, sizeof(port), "%u", tlsp != 0 ? NTRIP_CLI_TLS_PORT : NTRIP_CLI_PORT);
  char addresc[256 * 3], portesc[256 * 3];
  strurlescape(addr, 0, SIZE_MAX, addresc, sizeof(addresc));
  strurlescape(port, 0, SIZE_MAX, portesc, sizeof(portesc));
  char tpath[MAXSTRPATH];
  rssnprintf(tpath, sizeof(tpath), "%s:%s%s::V=%u", addresc, portesc,
             tlsp == 1   ? "::S"
             : tlsp == 2 ? "::A"
                         : "",
             verify);

  // Open TCP server stream
  ntripc->tcp = opentcpsvr(tpath, msg, msize);
  if (ntripc->tcp == NULL) {
    tracet(2, "openntripcas: opentcpsvr error port=%s\n", port);
  fail:
    for (unsigned ci = 0; ci < MAXCLI; ci++) {
      free(ntripc->con[ci].ibuf);
      free(ntripc->con[ci].obuf);
    }
    free(ntripc);
    return NULL;
  }
  return ntripc;
}
// Close NTRIP caster --------------------------------------------------------
static void closentripcas(ntripc_t *ntripc) {
  tracet(3, "closentripcas: state=%u\n", ntripc->state);
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    free(ntripc->con[ci].ibuf);
    free(ntripc->con[ci].obuf);
  }
  closetcpsvr(ntripc->tcp);
  free(ntripc);
}
// Disconnect ntrip-caster connection ----------------------------------------
static void discon_ntripcas(ntripc_t *ntripc, unsigned ci) {
  tracet(3, "discon_ntripcas: i=%u\n", ci);
  discontcp(&ntripc->tcp->cli[ci], ticonnect);
  ntripc->con[ci].nin = ntripc->con[ci].nout = 0;
  ntripc->con[ci].ibuf[0] = '\0';
  ntripc->con[ci].state = ntripc->con[ci].substate = 0;
}

// NTRIP caster - send response.
//
// Queue a http response. The connection is closed when done. The response
// line is required and should not be terminated in CR-LF - the prefix and
// line terminator are added here. The extra headers are optional and these
// header lines should be terminated by CR-LF, and if a Content-Length header
// is required then it should be included in these. The content is optional,
// need not be nul terminated. All the inputs are copied and can be freed by
// the caller. The response line and headers and content are assumed to fit
// in the output buffer, otherwise the content is truncated. Currently only
// small content lengths need be sent and the implementation is kept simple.
// This is intended for regular http responses, and will not suit NTRIP v1
// error responses.
static void ntripcas_send(ntripc_t *ntripc, unsigned ci, const char *response, const char *headers,
                          const uint8_t *content, size_t clen) {
  ntripc_con_t *con = ntripc->con + ci;
  RBOUNDSCHECK(con->obuf, con->obufsize, 10 + strlen(response) + 2);  // Response line.
  char *obuf = (char *)con->obuf;
  size_t obufsize = con->obufsize;
  obuf[0] = '\0';
  // Response line.
  if (ntripc->ver != 1)
    rsstrcat(obuf, obufsize, "HTTP/1.1 ");
  else
    rsstrcat(obuf, obufsize, "HTTP/1.0 ");
  rsstrcat(obuf, obufsize, response);
  rsstrcat(obuf, obufsize, "\r\n");
  rscatprintf(obuf, obufsize, "Server: RTKLIB %s %s\r\n", VER_RTKLIB, PATCH_LEVEL);
  char tstr[40];
  rscatprintf(obuf, obufsize, "Date: %s UTC\r\n", time2str(timeget(), tstr, 0));
  rscatprintf(obuf, obufsize, "Connection: close\r\n");
  // The caller is expected to terminate each line of these extra headers with CR-LF.
  if (headers) rsstrcat(obuf, obufsize, headers);
  if (content == NULL) {
    // No content to include. Terminate the headers.
    rsstrcat(obuf, obufsize, "\r\n");
    con->nout = strlen(obuf);
  } else {
    // Headers for the content.
    // Default content-type.
    if (headers == NULL) rscatprintf(obuf, obufsize, "Content-Type: text/plain\r\n");
    rscatprintf(obuf, obufsize, "Content-Length: %zu\r\n", clen);
    // Terminate the headers.
    rsstrcat(obuf, obufsize, "\r\n");
    // Fill the output buffer with as must of the content as possible.
    size_t hlen = strlen(obuf), j = 0;
    while (j < clen && hlen + j < obufsize) {
      obuf[hlen + j] = content[j];
      j++;
    }
    con->nout = hlen + ci;  // Output buffer usage.
    size_t rem = obufsize - ci;
    if (rem > 0) {
      tracet(2, "ntripcas_send: truncating content rem=%zu\n", rem);
    }
  }
  con->chunkout = 0;
  con->state = 0;
  con->substate = 2;  // Send and close.
}

// Send NTRIP source table ---------------------------------------------------
static void send_srctbl(ntripc_t *ntripc, unsigned ci) {
  unsigned ver = ntripc->con[ci].ver;

  char srctbl[512 + NTRIP_MAXSTR];
  rssnprintf(srctbl, sizeof(srctbl), "STR;%s;%s\r\n%s\r\n", ntripc->mntpnt, ntripc->srctbl,
             "ENDSOURCETABLE");
  // Assume this fits into the output buffer.
  if (ver == 2) {
    ntripcas_send(ntripc, ci, "200 OK",
                  // clang-format off
                  "Content-Type: gnss/sourcetable\r\n"
                  "Cache-Control: no-store,no-cache,max-age=0\r\n",
                  // clang-format on
                  (uint8_t *)srctbl, strlen(srctbl));
    return;
  }

  // Ntrip version 1.
  char *obuf = (char *)ntripc->con[ci].obuf;
  ssize_t obufsize = ntripc->con[ci].obufsize;
  obuf[0] = '\0';
  rscatprintf(obuf, obufsize, "SOURCETABLE 200 OK\r\n");
  rscatprintf(obuf, obufsize, "Content-Type: text/plain\r\n");
  rscatprintf(obuf, obufsize, "Server: %s %s %s\r\n", "RTKLIB", VER_RTKLIB, PATCH_LEVEL);
  char tstr[40];
  rscatprintf(obuf, obufsize, "Date: %s UTC\r\n", time2str(timeget(), tstr, 0));
  rscatprintf(obuf, obufsize, "Content-Length: %zu\r\n", strlen(srctbl));
  rscatprintf(obuf, obufsize, "Connection: close\r\n\r\n");
  rscatprintf(obuf, obufsize, "%s", srctbl);
  ntripc->con[ci].nout = strlen(obuf);
  ntripc_con_t *con = ntripc->con + ci;
  con->chunkout = 0;
  con->state = 0;
  con->substate = 2;  // Send and close.
}

// Respond to a NTRIP version 1 'source' request.
//
// The request and response are not regular http, but the request does have
// headers in the regular format that need to be skipped.
//
// Note: some clients look for "ERROR - Bad Password" and "400 Bad Request",
// not even an "ERROR" prefix.
static void rsp_sourcev1(ntripc_t *ntripc, unsigned ci, const char *requestline,
                         size_t contentstart) {
  tracet(4, "rsp_sourcev1: '%s'\n", requestline);
  ntripc_con_t *con = ntripc->con + ci;
  if (ntripc->type == 1 || ntripc->ver == 2) {
    // Either a 'GET only' caster, or NTRIP v2 only caster.
    tracet(2, "rsp_ntripcas: NTRIP v1 source request rejected\n");
    const char *rsp1 = "ERROR - Not Supported\r\n";
    size_t rlen = strlen(rsp1);
    memcpy(con->obuf, rsp1, rlen);
    con->nout = rlen;
    con->chunkout = 0;
    con->state = 0;
    con->substate = 2;  // Send and close.
    return;
  }
  if (con->ver == 2) {
    tracet(2, "rsp_ntripcas: SOURCE request with NTRIP v2 request\n");
    con->ver = 1;  // Fallback to v1.
  }
  if (con->chunkin == 1) {
    tracet(2, "rsp_ntripcas: SOURCE request with chunked transfer encoding\n");
    // Too inconsistent, give up.
    const char *rsp1 = "ERROR - Bad Request\r\n";
    size_t rlen = strlen(rsp1);
    memcpy(con->obuf, rsp1, rlen);
    con->nout = rlen;
    con->chunkout = 0;
    con->state = 0;
    con->substate = 2;  // Send and close.
    return;
  }
  char passwd[256];
  size_t reqlinelen = strlen(requestline);
  if (reqlinelen < 8 || requestline[6] != ' ') {
    tracet(2, "rsp_ntripcas: NTRIP v1 source request error\n");
    const char *rsp1 = "ERROR - Bad Password\r\n";
    size_t rlen = strlen(rsp1);
    memcpy(con->obuf, rsp1, rlen);
    con->nout = rlen;
    con->chunkout = 0;
    con->state = 0;
    con->substate = 2;  // Send and close.
    return;
  }
  // Copy a password terminated by a space.
  size_t passwdlen = 0;
  for (passwdlen = 0; passwdlen + 1 < sizeof(passwd); passwdlen++) {
    size_t k = 7 + passwdlen;
    if (k >= reqlinelen || requestline[k] == ' ' || requestline[k] == '\r' ||
        requestline[k] == '\n')
      break;
    passwd[passwdlen] = requestline[k];
  }
  if (passwdlen + 1 >= sizeof(passwd) || 7 + passwdlen >= reqlinelen ||
      requestline[7 + passwdlen] != ' ') {
    tracet(2, "rsp_ntripcas: NTRIP v1 source request error\n");
    const char *rsp1 = "ERROR - Bad Password\r\n";
    size_t rlen = strlen(rsp1);
    memcpy(con->obuf, rsp1, rlen);
    con->nout = rlen;
    con->chunkout = 0;
    con->state = 0;
    con->substate = 2;  // Send and close.
    return;
  }
  passwd[passwdlen] = '\0';
  // Copy a mount point terminated by white-space.
  char mntpnt[256] = "";
  size_t mntpntlen = 0;
  for (mntpntlen = 0; mntpntlen < sizeof(mntpnt) - 1; mntpntlen++) {
    size_t k = 7 + passwdlen + 1 + mntpntlen;
    if (k >= reqlinelen || requestline[k] == ' ' || requestline[k] == '\r' ||
        requestline[k] == '\n')
      break;
    mntpnt[mntpntlen] = requestline[k];
  }
  if (mntpntlen >= sizeof(mntpnt) - 1) {
    tracet(2, "rsp_ntripcas: NTRIP v1 source mntpnt too long\n");
    const char *rsp1 = "ERROR - Mount Point Invalid\r\n";
    size_t rlen = strlen(rsp1);
    memcpy(con->obuf, rsp1, rlen);
    con->nout = rlen;
    con->chunkout = 0;
    con->state = 0;
    con->substate = 2;  // Send and close.
    return;
  }
  mntpnt[mntpntlen] = '\0';
  tracet(5, "passwd='%s' mntpnt='%s'\n", passwd, mntpnt);
  if (ntripc->mntpnt[0] != '\0' && strcmp(mntpnt, ntripc->mntpnt) != 0) {
    tracet(2, "rsp_ntripcas: NTRIP v1 source mntpnt error, expected '%s\n", ntripc->mntpnt);
    const char *rsp1 = "ERROR - Mount Point Invalid\r\n";
    size_t rlen = strlen(rsp1);
    memcpy(con->obuf, rsp1, rlen);
    con->nout = rlen;
    con->chunkout = 0;
    con->state = 0;
    con->substate = 2;  // Send and close.
    return;
  }
  // This path has no username, just the password.
  if (ntripc->passwd[0] != '\0' && strcmp(passwd, ntripc->passwd) != 0) {
    tracet(2, "rsp_ntripcas: NTRIP v1 source authorization error\n");
    const char *rsp1 = "ERROR - Bad Password\r\n";
    size_t rlen = strlen(rsp1);
    memcpy(con->obuf, rsp1, rlen);
    con->nout = rlen;
    con->chunkout = 0;
    con->state = 0;
    con->substate = 2;  // Send and close.
    return;
  }
  // Send NTRIP caster v1 OK response.
  const char *rsp2 = "ICY 200 OK\r\n";
  size_t rlen = strlen(rsp2);
  memcpy(con->obuf, rsp2, rlen);
  con->nout = rlen;
  con->chunkout = 0;
  rsstrcpy(con->mntpnt, sizeof(con->mntpnt), mntpnt);
  // Discard all input buffer content before the content start.
  con->nin -= contentstart;
  char *ibuf = (char *)con->ibuf;
  memmove(ibuf, ibuf + contentstart, con->nin);
  con->state = 0;
  con->substate = 1;  // Send and connected.
}

// Test NTRIP client request -------------------------------------------------
static void rsp_ntripcas(ntripc_t *ntripc, unsigned ci) {
  ntripc_con_t *con = ntripc->con + ci;
  tracet(3, "rsp_ntripcas: ci=%u n=%zu\n", ci, con->nin);
  char *ibuf = (char *)con->ibuf;

  // Peek for the request line.
  char requestline[1024];
  size_t headerstart = 0;
  int rline = find_http_first_line(ibuf, con->nin, requestline, sizeof(requestline), &headerstart);
  if (rline < 0) {
    tracet(2, "rsp_ntripcas: http request line overflow nin=%zu\n", con->nin);
    ntripcas_send(ntripc, ci, "414 URI Too Long", NULL, NULL, 0);
    return;
  }
  if (rline == 0) return;  // Continue reading.
  tracet(5, "rsp_ntripcas request='%s'\n", requestline);

  char method[33];
  size_t methodlen;
  for (methodlen = 0; methodlen + 1 < sizeof(method); methodlen++) {
    if (requestline[methodlen] == '\0' || requestline[methodlen] == ' ' ||
        requestline[methodlen] == '\r' || requestline[methodlen] == '\n')
      break;
    method[methodlen] = requestline[methodlen];
  }
  if (methodlen == 0 || methodlen + 1 >= sizeof(method)) {
    tracet(2, "rsp_ntripcas: NTRIP request method error\n");
    ntripcas_send(ntripc, ci, "400 Bad Request", NULL, NULL, 0);
    return;
  }
  method[methodlen] = '\0';
  tracet(5, "rsp_ntripcas: method='%s'\n", method);

  // Peek for HTTP headers terminated by an empty header, and if not found
  // then return to continue reading.
  size_t headerend = headerstart, contentstart = headerstart;
  if (!find_http_headers(ibuf, con->nin, headerstart, &headerend, &contentstart)) {
    if (con->nin >= 8192 || con->nin >= con->ibufsize) {  // Buffer full.
      // Practical limit or buffer full.
      ntripcas_send(ntripc, ci, "431 Request Header Fields Too Large", NULL, NULL, 0);
      return;
    }
    return;  // Continue reading.
  }

  const char *headers = ibuf + headerstart;
  size_t headersize = headerend - headerstart;

  con->ver = 1;  // Default to NTRIP v1 client.
  con->chunkin = 0;
  con->chunkrem = -1;

  if (strcmp(method, "SOURCE") == 0) {
    // This path is split out because it is far from a regular http request and response.
    rsp_sourcev1(ntripc, ci, requestline, contentstart);
    return;
  }

  unsigned ntrip = 1;  // Default to a NTRIP client.
  char *useragent = get_http_header(headers, headersize, "User-Agent");
  if (useragent != NULL) {
    if (strstr(useragent, "NTRIP") == NULL && strstr(useragent, "ntrip") == NULL) ntrip = 0;
  }
  free(useragent);
  // For non-NTRIP clients default to NTRIP v2 unless the caster is configured
  // for only NTRIP v1.
  if (ntrip == 0 && ntripc->ver != 1) con->ver = 2;

  char *ntripver = get_http_header(headers, headersize, "Ntrip-Version");
  if (ntripver != NULL) {
    if (strncmp(ntripver, "Ntrip/2.0", 9) == 0) con->ver = 2;
  }
  free(ntripver);

  // If a POST request is received then assume NTRIP v2 even if the version
  // header is missing.
  if (con->ver != 2 && strcmp(method, "POST") == 0) {
    tracet(4, "rsp_ntripcas: POST request without ntrip v2 header, assuming v2 request\n");
    con->ver = 2;
  }

  char *tfrenc = get_http_header(headers, headersize, "Transfer-Encoding");
  if (tfrenc != NULL) {
    if (strncmp(tfrenc, "chunked", 7) == 0) con->chunkin = 1;
  }
  free(tfrenc);

  // If the request uses chunked transfer encoding then assume NTRIP v2 even
  // if the version header is missing.
  if (con->ver != 2 && con->chunkin) {
    tracet(4,
           "rsp_ntripcas: chunked transfer request without ntrip v2 header, assuming v2 request\n");
    con->ver = 2;
  }

  // Enforce Ntrip version restrictions.
  if ((ntripc->ver == 1 && con->ver != 1) || (ntripc->ver == 2 && con->ver != 2)) {
    tracet(2, "rsp_ntripcas: NTRIP request version %u rejected, expected ver=%u\n", con->ver,
           ntripc->ver);
    ntripcas_send(ntripc, ci, "505 HTTP Version Not Supported", NULL, NULL, 0);
    return;
  }

  // Expecting a regular HTTP request, so extract the URL and protocol
  // from the request line.
  char url[256] = "", proto[256] = "";
  if (sscanf(requestline + methodlen, " %255s %255s", url, proto) < 2 ||
      (strcmp(proto, "HTTP/1.0") && strcmp(proto, "HTTP/1.1"))) {
    tracet(2, "rsp_ntripcas: NTRIP request error proto=%s\n", proto);
    ntripcas_send(ntripc, ci, "505 HTTP Version Not Supported", NULL, NULL, 0);
    return;
  }
  char mntpnt[256] = "";
  ssize_t ri = rsstrchr(url, 0, '/');
  if (ri >= 0) rssubstrcpy(mntpnt, sizeof(mntpnt), url, ri + 1);

  // Enforce the type restrictions.
  if ((ntripc->type == 1 && strcmp(method, "GET") != 0) ||
      (ntripc->type == 2 && strcmp(method, "POST") != 0) ||
      (ntripc->ver == 1 && strcmp(method, "POST") == 0) ||
      (strcmp(method, "GET") != 0 && strcmp(method, "POST") != 0)) {
    tracet(2, "rsp_ntripcas: NTRIP unexpected method '%s'\n", method);
    ntripcas_send(ntripc, ci, "405 Method Not Allowed", NULL, NULL, 0);
    return;
  }

  // Test mountpoint.
  if (mntpnt[0] == '\0') {
    tracet(2, "rsp_ntripcas: no mountpoint %s\n", mntpnt);
    if (strcmp(method, "GET") == 0) {
      // Queue the sending of the source table.
      send_srctbl(ntripc, ci);
    } else {
      // POST, or other method, with no mount point.
      ntripcas_send(ntripc, ci, "404 Not Found", NULL, NULL, 0);
    }
    return;
  }

  if (strcmp(mntpnt, ntripc->mntpnt) != 0) {
    // Unexpected mount point. NTRIP v1 GET response sends the source table
    // while NTRIP v2 responds with not-found.
    tracet(2, "rsp_ntripcas: unexpected mountpoint '%s'\n", mntpnt);
    if (con->ver == 1 && strcmp(method, "GET") == 0) {
      // Queue the sending of the source table.
      send_srctbl(ntripc, ci);
    } else {
      // NTRIP v2 GET, or POST, or other method.
      ntripcas_send(ntripc, ci, "404 Not Found", NULL, NULL, 0);
    }
    return;
  }

  // Test authentication.
  if (ntripc->passwd[0] != '\0') {
    char user[513];
    rssnprintf(user, sizeof(user), "%s:%s", ntripc->user, ntripc->passwd);
    char user_pwd[712];
    rsstrcpy(user_pwd, sizeof(user_pwd), "Basic ");
    size_t plen = strlen(user_pwd);
    encbase64(user_pwd + plen, sizeof(user_pwd) - plen, (uint8_t *)user, strlen(user));
    // Get the header value.
    char *authorization = get_http_header(headers, headersize, "Authorization");
    if (authorization == NULL || strcmp(user_pwd, authorization) != 0) {
      tracet(2, "rsp_ntripc_c: authorization error\n");
      if (con->ver == 2) {
        char wwwauth[512 + NTRIP_MAXSTR];
        rssnprintf(wwwauth, sizeof(wwwauth), "WWW-Authenticate: Basic realm=\"/%s\"\r\n",
                   ntripc->mntpnt);
        ntripcas_send(ntripc, ci, "401 Unauthorized", wwwauth, NULL, 0);
      } else {
        ntripcas_send(ntripc, ci, "401 Unauthorized", NULL, NULL, 0);
      }
      free(authorization);
      return;
    }
    free(authorization);
  }

  if (con->ver == 2) {
    // Send NTRIP caster v2 GNSS data response.
    char *obuf = (char *)con->obuf;
    size_t obufsize = con->obufsize;
    obuf[0] = '\0';
    rscatprintf(obuf, obufsize, "HTTP/1.1 200 OK\r\n");
    rscatprintf(obuf, obufsize, "Server: %s %s %s\r\n", "RTKLIB", VER_RTKLIB, PATCH_LEVEL);
    char tstr[40];
    rscatprintf(obuf, obufsize, "Date: %s UTC\r\n", time2str(timeget(), tstr, 0));
    if (strcmp(method, "GET") == 0) {
      // Cache control for a successful GET request. Not need for a POST method.
      rscatprintf(obuf, obufsize, "Cache-Control: no-store,no-cache,max-age=0\r\n");
      // Not for the back channel of a POST request?
      rscatprintf(obuf, obufsize, "Content-Type: gnss/data\r\n");
    }
    rscatprintf(obuf, obufsize, "Connection: close\r\n");
    // Can send data chunked transfer encoded to a version 2 client.
    con->chunkout = 1;
    rscatprintf(obuf, obufsize, "Transfer-Encoding: chunked\r\nn");
    // Terminate headers.
    rscatprintf(obuf, obufsize, "\r\n");
    con->nout = strlen(obuf);
    rsstrcpy(con->mntpnt, sizeof(con->mntpnt), mntpnt);
  } else {
    // Send NTRIP caster v1 OK response.
    const char *rsp2 = "ICY 200 OK\r\n";
    size_t rlen = strlen(rsp2);
    memcpy(con->obuf, rsp2, rlen);
    con->nout = rlen;
    con->chunkin = con->chunkout = 0;
    con->chunkrem = -1;
    rsstrcpy(con->mntpnt, sizeof(con->mntpnt), mntpnt);
  }
  // Discard all input buffer content before the content start.
  con->nin -= contentstart;
  memmove(ibuf, ibuf + contentstart, con->nin);
  con->state = 0;
  con->substate = 1;  // Send and connected.
}
// Handle NTRIP client connect request ---------------------------------------
static void wait_ntripcas(ntripc_t *ntripc, char *msg, size_t msize) {
  tracet(4, "wait_ntripcas\n");
  ntripc->state = ntripc->tcp->svr.state;

  // This accepts new connections and brings them to the connected state from
  // where they are handled below.
  // The output buffer is not used in the connected state.
  if (!waittcpsvr(ntripc->tcp, msg, msize)) return;

  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (ntripc->tcp->cli[ci].state != 2) continue;
    if (ntripc->con[ci].state == 0 && ntripc->con[ci].substate == 0) {
      // Filling the input buffer and testing it.
      if (ntripc->con[ci].nin >= ntripc->con[ci].ibufsize - 1) {
        // Input buffer overflow.
        tracet(2, "wait_ntripcas: input buffer overflow sock=%" PRISOCK "\n",
               ntripc->tcp->cli[ci].sock);
        discon_ntripcas(ntripc, ci);
        continue;
      }
      // Receive NTRIP client request.
      size_t nmax = ntripc->con[ci].ibufsize - ntripc->con[ci].nin - 1;
      int err;
      ssize_t n = recv_nb(&ntripc->tcp->cli[ci], ntripc->con[ci].ibuf, ntripc->con[ci].ibufsize,
                          ntripc->con[ci].nin, nmax, &err);
      if (n < 0) {
        if (err) {
          tracet(2, "wait_ntripcas: recv error sock=%" PRISOCK " err=%d\n",
                 ntripc->tcp->cli[ci].sock, err);
        }
        discon_ntripcas(ntripc, ci);
        continue;
      }
      if (n <= 0) continue;
      // Test NTRIP client request. The state may progress and be handled
      // immediately below.
      ntripc->con[ci].nin += n;
      rsp_ntripcas(ntripc, ci);
    }
    if (ntripc->con[ci].state == 0 &&
        (ntripc->con[ci].substate == 1 || ntripc->con[ci].substate == 2)) {
      // Sending a response.
      if (ntripc->con[ci].nout > 0) {
        int err;
        ssize_t ns = send_nb(&ntripc->tcp->cli[ci], (uint8_t *)ntripc->con[ci].obuf,
                             ntripc->con[ci].obufsize, 0, ntripc->con[ci].nout, &err);

        if (ns < 0) {
          if (err) tracet(2, "wait_ntripcas: response write error err=%d\n", err);
          discon_ntripcas(ntripc, ci);
          continue;
        }
        if ((size_t)ns <= ntripc->con[ci].nout) {
          size_t rem = ntripc->con[ci].nout - ns;
          memmove(ntripc->con[ci].obuf, ntripc->con[ci].obuf + ns, rem);
          ntripc->con[ci].nout = rem;
        }
      }
      if (ntripc->con[ci].nout == 0) {
        if (ntripc->con[ci].substate == 1) {
          // Connected.
          ntripc->con[ci].state = 1;
          ntripc->con[ci].substate = 0;
          continue;
        }
        if (ntripc->con[ci].substate == 2) {
          // Flush and closing.
          ntripc->con[ci].substate = 3;
        }
      }
    }
    if (ntripc->con[ci].state == 0 && ntripc->con[ci].substate == 3) {
      // Flush and closing.
#ifdef RTKLIB_OPENSSL
      tcp_t *tcp = &ntripc->tcp->cli[ci];
      if (tcp->tlsp && tcp->ssl != NULL) {
        tcp->want = 0;
        if (SSL_shutdown(tcp->ssl) != 1) {
          char buff[1024];
          size_t nr;
          int res = SSL_read_ex(tcp->ssl, buff, 1024, &nr);
          if (res != 1) {
            int sslerr = SSL_get_error(tcp->ssl, res);
            if (sslerr == SSL_ERROR_WANT_READ) {
              tracet(5, "shutdown read tls: read want read\n");
              tcp->want = 1;  // Read wait.
              continue;       // Retry.
            }
            if (sslerr == SSL_ERROR_WANT_WRITE) {
              tracet(5, "shutdown read tls: read want write\n");
              tcp->want = 2;  // Write wait.
              continue;       // Retry.
            }
            if (sslerr == SSL_ERROR_ZERO_RETURN) {  // Graceful EOF.
              tracet(4, "shutdown read tls: eof\n");
            }
            if (sslerr == SSL_ERROR_SYSCALL) {
              int err = errsock();
              tracet(3, "shutdown tls unexpected sslerr=%d err=%d\n", sslerr, err);
            }
            ssl_report_errors();
            SSL_free(tcp->ssl);
            tcp->ssl = NULL;
            tcp->tlsstate = 0;  // Uninitialized.
          }
        }
      }
#endif
      discon_ntripcas(ntripc, ci);
      continue;
    }
    if (ntripc->con[ci].state == 1) {
      // Established. Continue sending buffered output.
      if (ntripc->con[ci].nout > 0) {
        int err;
        ssize_t ns = send_nb(&ntripc->tcp->cli[ci], (uint8_t *)ntripc->con[ci].obuf,
                             ntripc->con[ci].obufsize, 0, ntripc->con[ci].nout, &err);
        if (ns < 0) {
          if (err) tracet(2, "wait_ntripcas: write error err=%d\n", err);
          discon_ntripcas(ntripc, ci);
          continue;
        }
        if ((size_t)ns <= ntripc->con[ci].nout) {
          size_t rem = ntripc->con[ci].nout - ns;
          memmove(ntripc->con[ci].obuf, ntripc->con[ci].obuf + ns, rem);
          ntripc->con[ci].nout = rem;
        }
        if (ns > 0) ntripc->tcp->cli[ci].tact = tickget();
      }
    }
  }
}
// Read NTRIP caster ---------------------------------------------------------
static size_t readntripcas(ntripc_t *ntripc, uint8_t *buff, size_t size, size_t n, char *msg,
                           size_t msize) {
  tracet(4, "readntripcas:\n");

  wait_ntripcas(ntripc, msg, msize);

  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    ntripc_con_t *con = ntripc->con + ci;
    if (con->state == 0) continue;
    // Read into the input buffer.
    if (con->nin < con->ibufsize) {
      int err;
      ssize_t nr = recv_nb(&ntripc->tcp->cli[ci], con->ibuf, con->ibufsize, 0,
                           con->ibufsize - con->nin, &err);
      if (nr < 0) {
        if (err) {
          tracet(2, "readntripcas: recv error ci=%u sock=%" PRISOCK " err=%d\n", ci,
                 ntripc->tcp->cli[ci].sock, err);
        }
        discon_ntripcas(ntripc, ci);
        continue;
      }
      if (nr > 0) {
        con->nin += nr;
        ntripc->tcp->cli[ci].tact = tickget();
      }
    }
  }

  // Can early out in this case.
  if (n == 0) return 0;

  RBOUNDSCHECK(buff, size, n - 1);

  // Return from the first input buffer with data.
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    ntripc_con_t *con = ntripc->con + ci;
    if (con->state == 0) continue;
    if (con->nin == 0) continue;
    if (con->chunkin) {
      ssize_t nr = decode_chunk(buff, n, con->ibuf, &con->nin, &con->chunkrem);
      if (nr < 0) {
        // EOF on this stream.
        discon_ntripcas(ntripc, ci);
        continue;
      }
      if (nr > 0) return nr;
    }
    // Read from the input buffer.
    size_t nin = con->nin;
    if (nin <= n) {
      // Empty the buffer.
      memcpy(buff, con->ibuf, nin);
      con->nin = 0;
      return nin;
    }
    // Partial use of the input buffer.
    memcpy(buff, con->ibuf, n);
    memmove(con->ibuf, con->ibuf + n, nin - n);
    con->nin = nin - n;
    return n;
  }
  return 0;
}
// Write NTRIP caster --------------------------------------------------------
//
// Returns the number of bytes written to each client and 0 if there are no
// clients. See writetcpsvr, and the output buffer is also used for chunked
// transfer encoding.
static size_t writentripcas(ntripc_t *ntripc, const uint8_t *buff, size_t size, size_t n, char *msg,
                            size_t msize) {
  tracet(4, "writentripcas: n=%zu\n", n);

  // This will also continue flushing the output buffers.
  wait_ntripcas(ntripc, msg, msize);

  // Can early out in this case, and don't write chunks of size 0,
  if (n == 0) return 0;

  RBOUNDSCHECK(buff, size, n - 1);

  size_t ns = 0;
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (ntripc->con[ci].state == 0) continue;

    size_t chunkextra = 0;
    if (ntripc->con[ci].chunkout) chunkextra = 10;

    // Append the content onto the output buffer.
    if (ntripc->con[ci].nout + n + chunkextra > ntripc->con[ci].obufsize) {
      tracet(2, "writentripcas: output buffer overflow\n");
      // Close the connection.
      discon_ntripcas(ntripc, ci);
      continue;
    }
    if (ntripc->con[ci].chunkout) {
      char chunksize[10] = "";
      rssnprintf(chunksize, sizeof(chunksize), "%x\r\n", n);
      size_t headerlen = strlen(chunksize);
      memcpy(ntripc->con[ci].obuf + ntripc->con[ci].nout, chunksize, headerlen);
      memcpy(ntripc->con[ci].obuf + ntripc->con[ci].nout + headerlen, buff, n);
      const char chunksuffix[] = "\r\n";
      memcpy(ntripc->con[ci].obuf + ntripc->con[ci].nout + headerlen + n, chunksuffix, 2);
      ntripc->con[ci].nout += headerlen + n + 2;
    } else {
      memcpy(ntripc->con[ci].obuf + ntripc->con[ci].nout, buff, n);
      ntripc->con[ci].nout += n;
    }
    ns = n;

#if 1
    // Try sending it now.
    if (ntripc->con[ci].nout > 0) {
      int err;
      ssize_t nsn = send_nb(&ntripc->tcp->cli[ci], (uint8_t *)ntripc->con[ci].obuf,
                            ntripc->con[ci].obufsize, 0, ntripc->con[ci].nout, &err);

      if (nsn < 0) {
        if (err) {
          tracet(2, "writentripcas: send error ci=%u sock=%" PRISOCK " err=%d\n", ci,
                 ntripc->tcp->cli[ci].sock, err);
        }
        discon_ntripcas(ntripc, ci);
        continue;
      }
      if ((size_t)nsn <= ntripc->con[ci].nout) {
        size_t rem = ntripc->con[ci].nout - nsn;
        memmove(ntripc->con[ci].obuf, ntripc->con[ci].obuf + nsn, rem);
        ntripc->con[ci].nout = rem;
      }
      if (nsn > 0) ntripc->tcp->cli[ci].tact = tickget();
    }
#endif
  }
  return ns;
}

// For a wait on a write operation this depends on a prior call to strwrite
// setting the 'want' write flag. If this were to wait on a write to all
// clients then it may not wait at all.
static void wantntripc(ntripc_t *ntripc, unsigned op, wantset_t *wantset) {
  tracet(4, "wantntripc: op=%u\n", op);
  wanttcpsvr(ntripc->tcp, op, wantset);
}
// Get state NTRIP caster ----------------------------------------------------
static int statentripcas(const ntripc_t *ntripc) { return !ntripc ? 0 : ntripc->state; }
// Get extended state NTRIP caster -------------------------------------------
static int statexntripcas(ntripc_t *ntripc, char *msg, size_t msize) {
  unsigned state = ntripc == NULL ? 0 : ntripc->state;
  rscatprintf(msg, msize, "ntripcas:\n");
  rscatprintf(msg, msize, "  state   = %u\n", state);
  if (state == 0) return 0;
  rscatprintf(msg, msize, "  type    = %d\n", ntripc->type);
  rscatprintf(msg, msize, "  mntpnt  = %s\n", ntripc->mntpnt);
  rscatprintf(msg, msize, "  user    = %s\n", ntripc->user);
  rscatprintf(msg, msize, "  passwd  = %s\n", ntripc->passwd);
  rscatprintf(msg, msize, "  srctbl  = %s\n", ntripc->srctbl);
  rscatprintf(msg, msize, "  svr:\n");
  statextcp(&ntripc->tcp->svr, msg, msize);
  for (unsigned ci = 0; ci < MAXCLI; ci++) {
    if (ntripc->tcp->cli[ci].state == 0) continue;
    rscatprintf(msg, msize, "  cli#%d:\n", ci);
    statextcp(ntripc->tcp->cli + ci, msg, msize);
    rscatprintf(msg, msize, "    mntpnt= %s\n", ntripc->con[ci].mntpnt);
    rscatprintf(msg, msize, "    nint  = %zu\n", ntripc->con[ci].nin);
    rscatprintf(msg, msize, "    nout  = %zu\n", ntripc->con[ci].nout);
  }
  return state;
}

// Generate UDP socket -------------------------------------------------------
static udp_t *genudp(unsigned type, unsigned port, const char *saddr, char *msg, size_t msize) {
  tracet(3, "genudp: type=%u port=%u saddr='%s'\n", type, port, saddr);

  udp_t *udp = (udp_t *)malloc(sizeof(udp_t));
  if (udp == NULL) return NULL;
  memset(udp, 0, sizeof(udp_t));
  udp->state = 2;
  udp->type = type;
  udp->port = port;
  rsstrcpy(udp->saddr, sizeof(udp->saddr), saddr);

  char portstr[6];
  rssnprintf(portstr, sizeof(portstr), "%u", udp->port);

  struct addrinfo hints, *res = NULL;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV | AI_ADDRCONFIG;
  if (type == 0) hints.ai_flags |= AI_PASSIVE;
  int status = getaddrinfo(saddr[0] == '\0' ? NULL : saddr, portstr, &hints, &res);
  if (status != 0) {
    rssnprintf(msg, msize, "address error (%s)", saddr);
    tracet(1, "genupd: getaddrinfo error addr=%s %s\n", saddr, gai_strerror(status));
    free(udp);
    return NULL;
  }

  // Try each address using the first to successfully bind.
  struct addrinfo *next = res;

  for (; next != NULL; next = next->ai_next) {
    // Check and print the address used.
    if (next->ai_family == AF_INET) {
      struct sockaddr_in *ipv4 = (struct sockaddr_in *)next->ai_addr;
      char ipstr[INET_ADDRSTRLEN];
      inet_ntop(next->ai_family, &ipv4->sin_addr, ipstr, sizeof ipstr);
      tracet(3, "genudp: ipv4 addr='%s' port='%u'\n", ipstr, ntohs(ipv4->sin_port));
    } else if (next->ai_family == AF_INET6) {
      struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)next->ai_addr;
      char ipstr[INET6_ADDRSTRLEN];
      inet_ntop(next->ai_family, &ipv6->sin6_addr, ipstr, sizeof ipstr);
      tracet(3, "genudp: ipv6 addr='%s' port=%u\n", ipstr, ntohs(ipv6->sin6_port));
    } else {
      rssnprintf(msg, msize, "getaddrinfo error");
      tracet(1, "genudp: getaddinfo unexpected addr family=%d\n", next->ai_family);
      continue;
    }
  }
  next = res;  // reset

  for (; next != NULL; next = next->ai_next) {
    // Check and print the address used.
    if (next->ai_family == AF_INET) {
      struct sockaddr_in *ipv4 = (struct sockaddr_in *)next->ai_addr;
      char ipstr[INET_ADDRSTRLEN];
      inet_ntop(next->ai_family, &ipv4->sin_addr, ipstr, sizeof ipstr);
      tracet(3, "genudp: ipv4 addr='%s' port='%u'\n", ipstr, ntohs(ipv4->sin_port));
    } else if (next->ai_family == AF_INET6) {
      struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)next->ai_addr;
      char ipstr[INET6_ADDRSTRLEN];
      inet_ntop(next->ai_family, &ipv6->sin6_addr, ipstr, sizeof ipstr);
      tracet(3, "genudp: ipv6 addr='%s' port=%u\n", ipstr, ntohs(ipv6->sin6_port));
    } else {
      rssnprintf(msg, msize, "getaddrinfo error");
      tracet(1, "genudp: getaddinfo unexpected addr family=%d\n", next->ai_family);
      continue;
    }

    // Generate socket.
    udp->sock = socket(next->ai_family, next->ai_socktype, next->ai_protocol);
    if (udp->sock == (socket_t)-1) {
      int err = errsock();
      rssnprintf(msg, msize, "socket error (%d)", err);
      tracet(1, "genudp: socket error err=%d\n", err);
      continue;
    }

    int bs = (int)buffsize;
    if (setsockopt(udp->sock, SOL_SOCKET, SO_RCVBUF, (const char *)&bs, sizeof(bs)) == -1 ||
        setsockopt(udp->sock, SOL_SOCKET, SO_SNDBUF, (const char *)&bs, sizeof(bs)) == -1) {
      tracet(2, "genudp: setsockopt error sock=%" PRISOCK " err=%d bs=%d\n", udp->sock, errsock(),
             bs);
      rscatprintf(msg, msize, "sockopt error: bufsiz");
    }
    memcpy(&udp->addr, next->ai_addr, next->ai_addrlen);
    udp->addrlen = (socklen_t)next->ai_addrlen;

    if (udp->type == 0) {  // UDP server.
#ifdef SVR_REUSEADDR
      int opt = 1;
      setsockopt(udp->sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));
#endif
      if (bind(udp->sock, (struct sockaddr *)&udp->addr, udp->addrlen) == -1) {
        tracet(2, "genudp: bind error sock=%" PRISOCK " port=%u err=%d\n", udp->sock, port,
               errsock());
        rscatprintf(msg, msize, "bind error (%d): %d", errsock(), port);
        closesocket(udp->sock);
        continue;
      }
    } else {  // UDP client.
      if (next->ai_family == AF_INET && strcmp(saddr, "255.255.255.255") == 0) {
        int opt = 1;
        if (setsockopt(udp->sock, SOL_SOCKET, SO_BROADCAST, (const char *)&opt, sizeof(opt)) ==
            -1) {
          tracet(2, "genudp: setsockopt error sock=%" PRISOCK " err=%d\n", udp->sock, errsock());
          rscatprintf(msg, msize, "sockopt error: broadcast");
        }
      }
    }
    freeaddrinfo(res);
    return udp;
  }

  freeaddrinfo(res);
  free(udp);
  return NULL;
}
// Open UDP server -----------------------------------------------------------
static udp_t *openudpsvr(const char *path, char *msg, size_t msize) {
  tracet(3, "openudpsvr: path=%s\n", path);

  char sport[256] = "", saddr[256] = "";
  decodetcppath(path, saddr, sport, NULL, NULL, NULL, NULL);

  unsigned port;
  if (sscanf(sport, "%u", &port) < 1) {
    rscatprintf(msg, msize, "port error: %s", sport);
    tracet(1, "openudpsvr: port error port=%s\n", sport);
    return NULL;
  }
  return genudp(0, port, saddr, msg, msize);
}
// Close UDP server ----------------------------------------------------------
static void closeudpsvr(udp_t *udpsvr) {
  tracet(3, "closeudpsvr: sock=%" PRISOCK "\n", udpsvr->sock);

  closesocket(udpsvr->sock);
  free(udpsvr);
}
// Read UDP server -----------------------------------------------------------
static size_t readudpsvr(udp_t *udpsvr, uint8_t *buff, size_t size, size_t n, char *msg,
                         size_t msize) {
  (void)msg;
  (void)msize;
  tracet(4, "readudpsvr: sock=%" PRISOCK " n=%zu\n", udpsvr->sock, n);
  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);
  udpsvr->want = 0;
  fd_set rs;
  FD_ZERO(&rs);
  FD_SET(udpsvr->sock, &rs);
  struct timeval tv = {0};
#ifdef WIN32
  int ret = select(0, &rs, NULL, NULL, &tv);
#else
  int ret = select(udpsvr->sock + 1, &rs, NULL, NULL, &tv);
#endif
  if (ret == 0) {
    udpsvr->want = 1;  // Read wait.
    return 0;
  }
  if (ret < 0) return 0;
#ifdef WIN32
  ssize_t nr = recvfrom(udpsvr->sock, (char *)buff, (int)n, 0, NULL, NULL);
#else
  ssize_t nr = recvfrom(udpsvr->sock, (char *)buff, n, 0, NULL, NULL);
#endif
  if (nr >= 0) return (size_t)nr;
  int err = errsock();
#ifdef WIN32
  if (err == WSAEWOULDBLOCK)
#else
  if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
#endif
    udpsvr->want = 1;  // Read wait.
  return 0;
}
static void wantudpsvr(udp_t *udpsvr, unsigned op, wantset_t *wantset) {
  tracet(4, "wantudpsvr: op=%u\n", op);
  if (udpsvr->state == 0) return;
  fd_set *rs = &wantset->rs;
  if (udpsvr->want & 1) {
    FD_SET(udpsvr->sock, rs);
    udpsvr->want = 0;
#ifndef WIN32
    int n = 1 + (int)udpsvr->sock;
    if (n > wantset->nfds) wantset->nfds = n;
#endif
  }
  if (op & 1) {
    FD_SET(udpsvr->sock, rs);
#ifndef WIN32
    int n = 1 + (int)udpsvr->sock;
    if (n > wantset->nfds) wantset->nfds = n;
#endif
  }
}
// Get state UDP server ------------------------------------------------------
static int stateudpsvr(udp_t *udpsvr) { return udpsvr ? udpsvr->state : 0; }
// Get extended state UDP server ---------------------------------------------
static int statexudpsvr(udp_t *udpsvr, char *msg, size_t msize) {
  int state = udpsvr ? udpsvr->state : 0;

  rscatprintf(msg, msize, "udpsvr:\n");
  rscatprintf(msg, msize, "  state   = %d\n", state);
  if (state == 0) return 0;
  rscatprintf(msg, msize, "  type    = %d\n", udpsvr->type);
  rscatprintf(msg, msize, "  sock    = %d\n", (int)udpsvr->sock);
  rscatprintf(msg, msize, "  want    = %u\n", udpsvr->want);
  rscatprintf(msg, msize, "  port    = %d\n", udpsvr->port);
  return state;
}
// Open UDP client -----------------------------------------------------------
static udp_t *openudpcli(const char *path, char *msg, size_t msize) {
  tracet(3, "openudpsvr: path=%s\n", path);

  char sport[256] = "", saddr[256] = "";
  decodetcppath(path, saddr, sport, NULL, NULL, NULL, NULL);

  unsigned port;
  if (sscanf(sport, "%u", &port) < 1) {
    rscatprintf(msg, msize, "port error: %s", sport);
    tracet(1, "openudpcli: port error port=%s\n", sport);
    return NULL;
  }
  return genudp(1, port, saddr, msg, msize);
}
// Close UDP client ----------------------------------------------------------
static void closeudpcli(udp_t *udpcli) {
  tracet(3, "closeudpcli: sock=%" PRISOCK "\n", udpcli->sock);

  closesocket(udpcli->sock);
  free(udpcli);
}
// Write UDP client -----------------------------------------------------------
static size_t writeudpcli(udp_t *udpcli, const uint8_t *buff, size_t size, size_t n, char *msg,
                          size_t msize) {
  (void)msg;
  (void)msize;
  tracet(4, "writeudpcli: sock=%" PRISOCK " n=%zu\n", udpcli->sock, n);

  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);

  udpcli->want = 0;

#ifdef WIN32
  ssize_t ns = sendto(udpcli->sock, (char *)buff, (int)n, 0, (struct sockaddr *)&udpcli->addr,
                      udpcli->addrlen);
#else
  ssize_t ns =
      sendto(udpcli->sock, (char *)buff, n, 0, (struct sockaddr *)&udpcli->addr, udpcli->addrlen);
#endif
  if (ns >= 0) return (size_t)ns;

  int err = errsock();
#ifdef WIN32
  if (err == WSAEWOULDBLOCK)
#else
  if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
#endif
    udpcli->want = 2;  // Write wait.
  return 0;
}
// This only waits if a prior call set the 'want' flag to write.
static void wantudpcli(udp_t *udpcli, unsigned op, wantset_t *wantset) {
  tracet(4, "wantudpcli: op=%u\n", op);
  if (udpcli->state == 0) return;
  if (udpcli->want & 2) {
    fd_set *ws = &wantset->ws;
    FD_SET(udpcli->sock, ws);
    udpcli->want = 0;
#ifndef WIN32
    int n = 1 + (int)udpcli->sock;
    if (n > wantset->nfds) wantset->nfds = n;
#endif
  }
}
// Get state UDP client ------------------------------------------------------
static int stateudpcli(udp_t *udpcli) { return udpcli ? udpcli->state : 0; }
// Get extended state UDP client ---------------------------------------------
static int statexudpcli(udp_t *udpcli, char *msg, size_t msize) {
  int state = udpcli ? udpcli->state : 0;

  rscatprintf(msg, msize, "udpsvr:\n");
  rscatprintf(msg, msize, "  state   = %d\n", state);
  if (state == 0) return 0;
  rscatprintf(msg, msize, "  type    = %d\n", udpcli->type);
  rscatprintf(msg, msize, "  sock    = %d\n", (int)udpcli->sock);
  rscatprintf(msg, msize, "  want    = %u\n", udpcli->want);
  rscatprintf(msg, msize, "  addr    = %s\n", udpcli->saddr);
  rscatprintf(msg, msize, "  port    = %d\n", udpcli->port);
  return state;
}
// Decode FTP path -----------------------------------------------------------
static void decodeftppath(const char *path, char *addr, size_t asize, char *file, size_t fsize,
                          char *user, size_t usize, char *passwd, size_t psize, int *topts) {
  tracet(4, "decodeftpath: path=%s\n", path);

  if (user) user[0] = '\0';
  if (passwd) passwd[0] = '\0';
  if (topts) {
    topts[0] = 0;     // Time offset in path (s).
    topts[1] = 3600;  // Download interval (s).
    topts[2] = 0;     // Download time offset (s).
    topts[3] = 0;     // Retry interval (s) (0: no retry).
  }
  char buff[MAXSTRPATH];
  rsstrcpy(buff, sizeof(buff), path);

  ssize_t pi = rsstrchr(buff, 0, '/');
  if (pi >= 0) {
    ssize_t qi = rsstrstr(buff, pi + 1, "::");
    if (qi >= 0) {
      buff[qi] = '\0';
      if (topts) sscanf(buff + qi + 2, "T=%d,%d,%d,%d", topts, topts + 1, topts + 2, topts + 3);
    }
    rssubstrcpy(file, fsize, buff, pi + 1);
    buff[pi] = '\0';
  } else {
    file[0] = '\0';
  }

  pi = rsstrrchr(buff, 0, '@');
  if (pi >= 0) {
    buff[pi++] = '\0';
    ssize_t qi = rsstrchr(buff, 0, ':');
    if (qi >= 0) {
      buff[qi] = '\0';
      if (passwd) rsstrcpy(passwd, psize, buff + qi + 1);
    }
    if (user) rsstrcpy(user, usize, buff);
  } else {
    pi = 0;
  }

  rsstrcpy(addr, asize, buff + pi);
}
// Next download time --------------------------------------------------------
static gtime_t nextdltime(const int *topts, unsigned stat) {
  tracet(3, "nextdltime: topts=%d %d %d %d stat=%u\n", topts[0], topts[1], topts[2], topts[3],
         stat);

  // Current time (GPST).
  gtime_t time = utc2gpst(timeget());
  int week;
  double tow = time2gpst(time, &week);

  // Next retry time.
  if (stat == 0 && topts[3] > 0) {
    tow = (floor((tow - topts[2]) / topts[3]) + 1.0) * topts[3] + topts[2];
    return gpst2time(week, tow);
  }
  // Next interval time.
  int tint = topts[1] <= 0 ? 3600 : topts[1];
  tow = (floor((tow - topts[2]) / tint) + 1.0) * tint + topts[2];
  time = gpst2time(week, tow);

  return time;
}
// FTP thread ----------------------------------------------------------------
#ifdef WIN32
static DWORD WINAPI ftpthread(void *arg)
#else
static void *ftpthread(void *arg)
#endif
{
  tracet(3, "ftpthread:\n");

  ftp_t *ftp = (ftp_t *)arg;

  if (localdir[0] == '\0') {
    tracet(1, "no local directory\n");
    ftp->error = 11;
    ftp->state = 3;
    return 0;
  }
  // Replace keyword in file path and local path.
  gtime_t time = timeadd(utc2gpst(timeget()), ftp->topts[0]);
  char remote[MAXSTRPATH];
  reppath(ftp->file, remote, time, "", "");

  char *pr = strrchr(remote, '/');
  if (pr)
    pr++;
  else
    pr = remote;
  char local[MAXSTRPATH];
  rssnprintf(local, sizeof(local), "%.768s%c%.254s", localdir, RTKLIB_FILEPATHSEP, pr);
  char errfile[MAXSTRPATH];
  rssnprintf(errfile, sizeof(errfile), "%.1019s.err", local);

  // If local file exist, skip download.
  char tmpfile[MAXSTRPATH];
  rsstrcpy(tmpfile, sizeof(tmpfile), local);
  char *pe = strrchr(tmpfile, '.');
  if (pe && (strcmp(pe, ".z") == 0 || strcmp(pe, ".gz") == 0 || strcmp(pe, ".zip") == 0 ||
             strcmp(pe, ".Z") == 0 || strcmp(pe, ".GZ") == 0 || strcmp(pe, ".ZIP") == 0)) {
    *pe = '\0';
  }
  FILE *fp = fopen(tmpfile, "rb");
  if (fp) {
    fclose(fp);
    rssnprintf(ftp->local, sizeof(ftp->local), "%.1023s", tmpfile);
    tracet(3, "ftpthread: file exists %s\n", ftp->local);
    ftp->state = 2;
    return 0;
  }
  // Proxy settings for wget (ref [2]).
  char env[1024] = "";
  const char *proxyopt = "";
  if (proxyaddr[0]) {
    const char *proto = ftp->proto ? "http" : "ftp";
    rssnprintf(env, sizeof(env), "set %.4s_proxy=http://%.998s & ", proto, proxyaddr);
    proxyopt = "--proxy=on ";
  }
  // Download command (ref [2]).
  char cmd[5120], opt[1024];
  if (ftp->proto == 0) {  // FTP.
    rssnprintf(opt, sizeof(opt),
               "--ftp-user=%.32s --ftp-password=%.32s --glob=off "
               "--passive-ftp %.32s -t 1 -T %d -O \"%.768s\"",
               ftp->user, ftp->passwd, proxyopt, FTP_TIMEOUT, local);
    rssnprintf(cmd, sizeof(cmd), "%s%s %s \"ftp://%s/%s\" 2> \"%.768s\"\n", env, FTP_CMD, opt,
               ftp->addr, remote, errfile);
  } else {  // HTTP.
    rssnprintf(opt, sizeof(opt), "%.32s -t 1 -T %d -O \"%.768s\"", proxyopt, FTP_TIMEOUT, local);
    rssnprintf(cmd, sizeof(cmd), "%s%s %s \"http://%s/%s\" 2> \"%.768s\"\n", env, FTP_CMD, opt,
               ftp->addr, remote, errfile);
  }
  // Execute download command.
  int ret = execcmd(cmd);
  if (ret) {
    remove(local);
    tracet(1, "execcmd error: cmd=%s ret=%d\n", cmd, ret);
    ftp->error = ret;
    ftp->state = 3;
    return 0;
  }
  remove(errfile);

  // Uncompress downloaded file.
  const char *p = strrchr(local, '.');
  if (p && (strcmp(p, ".z") == 0 || strcmp(p, ".gz") == 0 || strcmp(p, ".zip") == 0 ||
            strcmp(p, ".Z") == 0 || strcmp(p, ".GZ") == 0 || strcmp(p, ".ZIP") == 0)) {
    if (rtk_uncompress(local, tmpfile)) {
      remove(local);
      rsstrcpy(local, sizeof(local), tmpfile);
    } else {
      tracet(1, "file uncompact error: %s\n", local);
      ftp->error = 12;
      ftp->state = 3;
      return 0;
    }
  }
  rsstrcpy(ftp->local, sizeof(ftp->local), local);
  ftp->state = 2;  // FTP completed.

  tracet(3, "ftpthread: complete cmd=%s\n", cmd);
  return 0;
}
// Open FTP ------------------------------------------------------------------
static ftp_t *openftp(const char *path, unsigned type, char *msg, size_t msize) {
  (void)msg;
  (void)msize;
  tracet(3, "openftp: path=%s type=%u\n", path, type);

  ftp_t *ftp = (ftp_t *)malloc(sizeof(ftp_t));
  if (ftp == NULL) return NULL;
  memset(ftp, 0, sizeof(ftp_t));

  ftp->state = 0;
  ftp->proto = type;
  ftp->error = 0;
  ftp->thread = 0;
  ftp->local[0] = '\0';

  // Decode FTP path.
  decodeftppath(path, ftp->addr, sizeof(ftp->addr), ftp->file, sizeof(ftp->file), ftp->user,
                sizeof(ftp->user), ftp->passwd, sizeof(ftp->passwd), ftp->topts);

  // Set first download time.
  ftp->tnext = timeadd(timeget(), 10.0);

  return ftp;
}
// Close FTP -----------------------------------------------------------------
static void closeftp(ftp_t *ftp) {
  tracet(3, "closeftp: state=%u\n", ftp->state);

  if (ftp->state != 1) free(ftp);
}
// Read FTP ------------------------------------------------------------------
static size_t readftp(ftp_t *ftp, uint8_t *buff, size_t size, size_t n, char *msg, size_t msize) {
  tracet(4, "readftp: n=%zu\n", n);

  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);

  gtime_t time = utc2gpst(timeget());
  if (timediff(time, ftp->tnext) < 0.0) {  // Until download time?
    return 0;
  }
  if (ftp->state == 0) {  // FTP/HTTP not executed?
    ftp->state = 1;
    rssnprintf(msg, msize, "%s://%s", ftp->proto ? "http" : "ftp", ftp->addr);

#ifdef WIN32
    ftp->thread = CreateThread(NULL, 0, ftpthread, ftp, 0, NULL);
    if (ftp->thread == NULL) {
#else
    if (pthread_create(&ftp->thread, NULL, ftpthread, ftp)) {
#endif
      tracet(1, "readftp: ftp thread create error\n");
      ftp->state = 3;
      rsstrcpy(msg, msize, "ftp thread error");
      return 0;
    }
  }
  if (ftp->state == 1) return 0;  // FTP/HTTP on going?

  if (ftp->state == 3) {  // FTP error.
    rssnprintf(msg, msize, "%s error (%d)", ftp->proto ? "http" : "ftp", ftp->error);

    // Set next retry time.
    ftp->tnext = nextdltime(ftp->topts, 0);
    ftp->state = 0;
    return 0;
  }
  // Return local file path if FTP completed.
  rsstrcpy((char *)buff, n, ftp->local);
  rsstrcat((char *)buff, n, "\r\n");

  // Set next download time.
  ftp->tnext = nextdltime(ftp->topts, 1);
  ftp->state = 0;

  rsstrcpy(msg, msize, "");

  return strlen((char *)buff);
}
// Get state FTP -------------------------------------------------------------
static int stateftp(const ftp_t *ftp) {
  return !ftp ? 0 : (ftp->state == 0 ? 2 : (ftp->state <= 2 ? 3 : -1));
}
// Get extended state FTP ----------------------------------------------------
static int statexftp(const ftp_t *ftp, char *msg, size_t msize) {
  (void)msg;
  (void)msize;
  return !ftp ? 0 : (ftp->state == 0 ? 2 : (ftp->state <= 2 ? 3 : -1));
}

// Open memory buffer --------------------------------------------------------
static membuf_t *openmembuf(const char *path, char *msg, size_t msize) {
  tracet(3, "openmembuf: path=%s\n", path);

  size_t bufsize = DEFAULT_MEMBUF_SIZE;
  sscanf(path, "%zu", &bufsize);

  membuf_t *membuf = (membuf_t *)malloc(sizeof(membuf_t));
  if (membuf == NULL) return NULL;
  memset(membuf, 0, sizeof(membuf_t));
  membuf->state = 1;
  membuf->rp = 0;
  membuf->wp = 0;
  membuf->buf = (uint8_t *)malloc(bufsize);
  if (membuf->buf == NULL) {
    free(membuf);
    return NULL;
  }
  membuf->bufsize = bufsize;
  rtklib_initlock(&membuf->lock);

  rscatprintf(msg, msize, "membuf sizebuf=%zu", bufsize);

  return membuf;
}
// Close memory buffer -------------------------------------------------------
static void closemembuf(membuf_t *membuf) {
  tracet(3, "closemembufp\n");

  free(membuf->buf);
  free(membuf);
}
// Read memory buffer --------------------------------------------------------
static size_t readmembuf(membuf_t *membuf, uint8_t *buff, size_t size, size_t n, char *msg,
                         size_t msize) {
  (void)msg;
  (void)msize;
  tracet(4, "readmembuf: n=%zu\n", n);

  if (membuf == NULL) return 0;

  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);

  rtklib_lock(&membuf->lock);

  size_t i = membuf->rp, nr = 0;
  while (i != membuf->wp && nr < n) {
    buff[nr++] = membuf->buf[i];
    if (++i >= membuf->bufsize) i = 0;
  }
  membuf->rp = i;
  rtklib_unlock(&membuf->lock);
  return nr;
}
// Write memory buffer -------------------------------------------------------
static size_t writemembuf(membuf_t *membuf, const uint8_t *buff, size_t size, size_t n, char *msg,
                          size_t msize) {
  tracet(3, "writemembuf: n=%zu\n", n);

  if (membuf == NULL) return 0;

  if (n == 0) return 0;
  RBOUNDSCHECK(buff, size, n - 1);

  rtklib_lock(&membuf->lock);

  size_t i;
  for (i = 0; i < n; i++) {
    membuf->buf[membuf->wp++] = buff[i];
    if (membuf->wp >= membuf->bufsize) membuf->wp = 0;
    if (membuf->wp == membuf->rp) {
      rsstrcpy(msg, msize, "mem-buffer overflow");
      membuf->state = -1;
      rtklib_unlock(&membuf->lock);
      return i + 1;
    }
  }
  rtklib_unlock(&membuf->lock);
  return i;
}
// Get state memory buffer ---------------------------------------------------
static int statemembuf(membuf_t *membuf) { return !membuf ? 0 : membuf->state; }
// Get extended state memory buffer ------------------------------------------
static int statexmembuf(membuf_t *membuf, char *msg, size_t msize) {
  int state = membuf == NULL ? 0 : membuf->state;

  rscatprintf(msg, msize, "membuf:\n");
  rscatprintf(msg, msize, "  state   = %d\n", state);
  if (state == 0) return 0;
  rscatprintf(msg, msize, "  buffsize= %zu\n", membuf->bufsize);
  rscatprintf(msg, msize, "  wp      = %zu\n", membuf->wp);
  rscatprintf(msg, msize, "  rp      = %zu\n", membuf->rp);
  return state;
}

// Initialize stream environment -----------------------------------------------
// Initialize stream environment
// Args   : none
// Return : none
//-----------------------------------------------------------------------------
void strinitcom(void) {
  tracet(3, "strinitcom:\n");

#ifdef WIN32
  WSADATA data;
  WSAStartup(MAKEWORD(2, 0), &data);
#endif
}
// Initialize stream TLS environment -------------------------------------------
// Args   : char *svrcertfile     TLS server certificate file.
//        : char *svrkeyfile      TLS server certificate private key file.
//        : char *svrcafile       TLS server CA file.
//        : char *svrcadir        TLS server CA directory.
//        : unsigned svrverify    TLS server enable peer verification.
//        : char *clicertfile     TLS client certificate file.
//        : char *clikeyfile      TLS client certificate private key file.
//        : char *clicafile       TLS client CA file.
//        : char *clicadir        TLS client CA directory.
//        : unsigned cliverify    TLS client enable peer verification.
// Return : none
void strinittls(const char *svrcertfile, const char *svrkeyfile, const char *svrcafile,
                const char *svrcadir, unsigned svrverify, const char *clicertfile,
                const char *clikeyfile, const char *clicafile, const char *clicadir,
                unsigned cliverify) {
  tracet(3,
         "strinittls: svrcertfile='%s' svrkeyfile='%s' svrcafile='%s' svrcadir='%s' svrverify=%u\n",
         svrcertfile, svrkeyfile, svrcafile, svrcadir, svrverify);
  tracet(3,
         "strinittls: clicertfile='%s' clikeyfile='%s' clicafile='%s' clicadir='%s' cliverify=%u\n",
         clicertfile, clikeyfile, clicafile, clicadir, cliverify);
  rsstrcpy(tlssvrcertfile, sizeof(tlssvrcertfile), svrcertfile ? svrcertfile : "");
  rsstrcpy(tlssvrkeyfile, sizeof(tlssvrkeyfile), svrkeyfile ? svrkeyfile : "");
  rsstrcpy(tlssvrcafile, sizeof(tlssvrcafile), svrcafile ? svrcafile : "");
  rsstrcpy(tlssvrcadir, sizeof(tlssvrcadir), svrcadir ? svrcadir : "");
  tlssvrverify = svrverify;
  rsstrcpy(tlsclicertfile, sizeof(tlsclicertfile), clicertfile ? clicertfile : "");
  rsstrcpy(tlsclikeyfile, sizeof(tlsclikeyfile), clikeyfile ? clikeyfile : "");
  rsstrcpy(tlsclicafile, sizeof(tlsclicafile), clicafile ? clicafile : "");
  rsstrcpy(tlsclicadir, sizeof(tlsclicadir), clicadir ? clicadir : "");
  tlscliverify = cliverify;
}
// Initialize stream -----------------------------------------------------------
// Initialize stream struct
// Args   : stream_t *stream IO  stream
// Return : none
//-----------------------------------------------------------------------------
void strinit(stream_t *stream) {
  tracet(3, "strinit:\n");

  stream->type = 0;
  stream->mode = 0;
  stream->state = 0;
  stream->inb = stream->outb = 0;
  stream->inr = stream->outr = 0;
  stream->tick_i = stream->tick_o = stream->tact = 0;
  stream->inbt = stream->outbt = 0;
  rtklib_initlock(&stream->lock);
  stream->port = NULL;
  stream->path[0] = '\0';
  stream->msg[0] = '\0';
}
// Open stream -----------------------------------------------------------------
//
// Open stream to read or write data from or to virtual devices.
//
// Args   : stream_t *stream IO  stream
//          unsigned type    I   stream type
//                                 STR_SERIAL   = serial device
//                                 STR_FILE     = file (record and playback)
//                                 STR_TCPSVR   = TCP server
//                                 STR_TCPCLI   = TCP client
//                                 STR_NTRIPSRC = NTRIP source
//                                 STR_NTRIPCLI = NTRIP client
//                                 STR_NTRIPCAS = NTRIP caster client
//                                 STR_UDPSVR   = UDP server (read only)
//                                 STR_UDPCLI   = UDP client (write only)
//                                 STR_MEMBUF   = memory buffer (FIFO)
//                                 STR_FTP      = download by FTP (read only)
//                                 STR_HTTP     = download by HTTP (read only)
//          unsigned mode    I   stream mode (STR_MODE_???)
//                                 STR_MODE_R   = read only
//                                 STR_MODE_W   = write only
//                                 STR_MODE_RW  = read and write
//          char *path       I   stream path (see below)
//
// Return : status (0:error,1:ok)
//
// Notes  : see reference [1] for NTRIP
//          STR_FTP/HTTP needs "wget" in command search paths
//
// Stream path ([] options):
//
//   STR_SERIAL   port[:brate[:bsize[:parity[:stopb[:fctr[#port]]]]]]
//                    port  = COM??  (windows)
//                            tty??? (linuex, omit /dev/)
//                    brate = bit rate     (bps)
//                    bsize = bit size     (7|8)
//                    parity= parity       (n|o|e)
//                    stopb = stop bits    (1|2)
//                    fctr  = flow control (off|rts)
//                    port  = TCP server port to output received stream
//
//   STR_FILE     path[::T][::+start][::xseppd][::S=swap][::P={4|8}]
//                    path  = file path
//                            (can include keywords defined by )
//                    ::T   = enable time tag
//                    start = replay start offset (s)
//                    speed = replay speed factor
//                    swap  = output swap interval (hr) (0: no swap)
//                    ::P={4|8} = file pointer size (4:32bit,8:64bit)
//
//   STR_TCPSVR   :port[::S|::A][::V={0|1}]
//                    port  = TCP server port to accept
//                    ::S   = Require TLS encryption.
//                    ::A   = Allow unencrytped and TLS encryption connections.
//                    ::V={0|1} = TLS client peer verification.
//
//   STR_TCPCLI   addr:port[::S][::V={0|1}]
//                    addr  = TCP server address to connect
//                    port  = TCP server port to connect
//                    ::S   = Require TLS encryption.
//                    ::V={0|1} = TLS client peer verification.
//
//   STR_NTRIPSRC [user[:passwd]@]addr[:port]/mpoint[:string][::S][::N={1|2}][::V={0|1}]
//                    addr  = NTRIP caster address to connect
//                    port  = NTRIP caster source port to connect
//                    passwd= NTRIP caster source password to connect
//                    mpoint= NTRIP mountpoint
//                    string= NTRIP server string
//                    ::N={1|2} = NTRIP version 1 or 2 (default 1).
//                    ::S   = Require TLS encryption.
//                    ::V={0|1} = TLS client peer verification.
//
//   STR_NTRIPCLI [user[:passwd]@]addr[:port]/mpoint[::S][::N={1|2}][::V={0|1}]
//                    addr  = NTRIP caster address to connect
//                    port  = NTRIP caster client port to connect
//                    user  = NTRIP caster client user to connect
//                    passwd= NTRIP caster client password to connect
//                    mpoint= NTRIP mountpoint
//                    ::N={1|2} = NTRIP version 1 or 2 (default 1).
//                    ::S   = Require TLS encryption.
//                    ::V={0|1} = TLS client peer verification.
//
//   STR_NTRIPCAS [user[:passwd]@][:port]/mpoint[:srctbl][::S|::A][::V={0|1}]
//                    port  = NTRIP caster client port to accept connection
//                    user  = NTRIP caster client user to accept connection
//                    passwd= NTRIP caster client password to accept connection
//                    mpoint= NTRIP mountpoint
//                    srctbl= NTRIP source table entry (STR) (ref [3] 6.3)
//                      (ID;format;format-details;carrier;nav-system;network;
//                       country;latitude;longitude;nmea;solution;generator;
//                       compr-encrp;autentication;fee;bitrate;...;misc)
//                    ::N={0|1|2} = NTRIP version 1 or 2, or 0 both (default 0).
//                    ::T={0|1|2} = Type: 0:either,1:clients only,2:sources only.
//                    ::S   = Require TLS encryption.
//                    ::A   = Allow unencrytped and TLS encryption connections.
//                    ::V={0|1} = TLS server peer verification.
//
//   STR_UDPSVR   addr:port
//                    addr  = UDP server or broadcast address to receive
//                    port  = UDP server port to receive
//
//   STR_UDPCLI   addr:port
//                    addr  = UDP server or broadcast address to send
//                    port  = UDP server or broadcast port to send
//
//   STR_MEMBUF   [size]
//                    size  = FIFO size (bytes) ("":4096)
//
//   STR_FTP      [user[:passwd]@]addr/path[::T=poff[,tint[,toff,tret]]]]
//                    user  = FTP server user
//                    passwd= FTP server password
//                    addr  = FTP server address
//                    path  = FTP server file path
//                    poff  = time offset for path extension (s)
//                    tint  = download interval (s)
//                    toff  = download time offset (s)
//                    tret  = download retry interval (s) (0:no retry)
//
//   STR_HTTP     addr/path[::T=poff[,tint[,toff,tret]]]]
//                    addr  = HTTP server address
//                    path  = HTTP server file path
//                    poff  = time offset for path extension (s)
//                    tint  = download interval (s)
//                    toff  = download time offset (s)
//                    tret  = download retry interval (s) (0:no retry)
//
//-----------------------------------------------------------------------------
int stropen(stream_t *stream, unsigned type, unsigned mode, const char *path) {
  tracet(3, "stropen: type=%u mode=%u path=%s\n", type, mode, path);

  stream->type = type;
  stream->mode = mode;
  rsstrcpy(stream->path, sizeof(stream->path), path);
  stream->inb = stream->outb = 0;
  stream->inr = stream->outr = 0;
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
    case STR_NTRIPSRC:
      stream->port = openntrip(path, 0, msg, msize);
      break;
    case STR_NTRIPCLI:
      stream->port = openntrip(path, 1, msg, msize);
      break;
    case STR_NTRIPCAS:
      stream->port = openntripcas(path, msg, msize);
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
      return 1;
  }
  stream->state = !stream->port ? -1 : 1;
  return stream->port != NULL;
}
// Close stream ----------------------------------------------------------------
// Close stream
// Args   : stream_t *stream IO  stream
// Return : none
//-----------------------------------------------------------------------------
void strclose(stream_t *stream) {
  tracet(3, "strclose: type=%u mode=%u\n", stream->type, stream->mode);

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
      case STR_NTRIPSRC:
      case STR_NTRIPCLI:
        closentrip((ntrip_t *)stream->port);
        break;
      case STR_NTRIPCAS:
        closentripcas((ntripc_t *)stream->port);
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
    tracet(3, "no port to close stream: type=%u\n", stream->type);
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
// Sync streams ----------------------------------------------------------------
// Sync time for streams
// Args   : stream_t *stream1 IO stream 1
//          stream_t *stream2 IO stream 2
// Return : none
// Notes  : for replay files with time tags.
//-----------------------------------------------------------------------------
void strsync(stream_t *stream1, stream_t *stream2) {
  if (stream1->type != STR_FILE || stream2->type != STR_FILE) return;
  file_t *file1 = (file_t *)stream1->port;
  file_t *file2 = (file_t *)stream2->port;
  if (file1 && file2) syncfile(file1, file2);
}
// Lock/unlock stream ----------------------------------------------------------
// Lock/unlock stream
// Args   : stream_t *stream I  stream
// Return : none
//-----------------------------------------------------------------------------
void strlock(stream_t *stream) { rtklib_lock(&stream->lock); }
void strunlock(stream_t *stream) { rtklib_unlock(&stream->lock); }

// Read stream -----------------------------------------------------------------
// Read data from stream (unblocked)
// Args   : stream_t      *stream I  stream
//          unsigned char *buff   O  data buffer
//          size_t         size   I  buff size, for bounds checking
//          size_t         start  I  start index in buff
//          size_t         n      I  maximum data length
// Return : read data length
// Notes  : if no data, return immediately with no data.
//-----------------------------------------------------------------------------
size_t strread(stream_t *stream, uint8_t *buff, size_t size, size_t start, size_t n) {
  tracet(4, "strread: n=%zu\n", n);

  if ((stream->mode & STR_MODE_R) == 0 || stream->port == NULL) return 0;

  if (n > 0) RBOUNDSCHECK(buff, size, start + n - 1);

  strlock(stream);

  // The stream msg buffer is not cleared on each read, and the read
  // functions overwrite the msg buffer only when noting exceptional
  // events, they do not append to the msg buffer.
  char *msg = stream->msg;
  size_t msize = sizeof(stream->msg);
  uint32_t tick = tickget();
  size_t nr = 0;
  switch (stream->type) {
    case STR_SERIAL:
      nr = readserial((serial_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_FILE:
      nr = readfile((file_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_TCPSVR:
      nr = readtcpsvr((tcpsvr_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_TCPCLI:
      nr = readtcpcli((tcpcli_t *)stream->port, buff, size, start, n, msg, msize);
      break;
    case STR_NTRIPSRC:
    case STR_NTRIPCLI:
      nr = readntrip((ntrip_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_NTRIPCAS:
      nr = readntripcas((ntripc_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_UDPSVR:
      nr = readudpsvr((udp_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_MEMBUF:
      nr = readmembuf((membuf_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_FTP:
      nr = readftp((ftp_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_HTTP:
      nr = readftp((ftp_t *)stream->port, buff + start, size - start, n, msg, msize);
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
  if (tt > (int)tirate) {
    stream->inr = (uint32_t)((double)((stream->inb - stream->inbt) * 8) / (tt * 0.001));
    stream->tick_i = tick;
    stream->inbt = stream->inb;
  }
  strunlock(stream);
  return nr;
}
// Write stream ----------------------------------------------------------------
// Write data to stream (unblocked)
// Args   : stream_t      *stream  I  stream
//          unsigned char *buff    I  data buffer
//          size_t         size    I  buff size, for bounds checking
//          size_t         start   I  start index in buff
//          size_t         n       I  data length
// Return : status (0:error, >0 bytes written)
// Notes  : Write data to buffer and return immediately.
//          Call with n=0 to continue writing buffered data.
//-----------------------------------------------------------------------------
size_t strwrite(stream_t *stream, const uint8_t *buff, size_t size, size_t start, size_t n) {
  tracet(4, "strwrite: n=%zu\n", n);

  if ((stream->mode & STR_MODE_W) == 0 || stream->port == NULL) return 0;

  if (n > 0) RBOUNDSCHECK(buff, size, start + n - 1);

  strlock(stream);

  // The stream msg buffer is not cleared on each write, and the write
  // functions overwrite the msg buffer only when noting exceptional
  // events, they do not append to the msg buffer.
  char *msg = stream->msg;
  size_t msize = sizeof(stream->msg);
  uint32_t tick = tickget();
  size_t ns;
  switch (stream->type) {
    case STR_SERIAL:
      ns = writeserial((serial_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_FILE:
      ns = writefile((file_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_TCPSVR:
      ns = writetcpsvr((tcpsvr_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_TCPCLI:
      ns = writetcpcli((tcpcli_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_NTRIPSRC:
    case STR_NTRIPCLI:
      ns = writentrip((ntrip_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_NTRIPCAS:
      ns = writentripcas((ntripc_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_UDPCLI:
      ns = writeudpcli((udp_t *)stream->port, buff + start, size - start, n, msg, msize);
      break;
    case STR_MEMBUF:
      ns = writemembuf((membuf_t *)stream->port, buff + start, size - start, n, msg, msize);
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
  if (tt > (int)tirate) {
    stream->outr = (uint32_t)((double)((stream->outb - stream->outbt) * 8) / (tt * 0.001));
    stream->tick_o = tick;
    stream->outbt = stream->outb;
  }
  strunlock(stream);
  return ns;
}

// Want socket stream reasons--------------------------------------------------
// Socket stream wait setup.
// Args   : stream_t      *stream  I  stream
//          unsigned       op      I  0: none, 1: read.
//          void *         wantset IO set of sockets to wait on.
// Return : 
//
// Set up the file descriptor read and write sets to wait on socket operations
// flagged as wanting to read or write. These flags are set by prior stream
// operations, a read or write, and from internal stream operations. e.g
// accepting connections; and the TLS protocol layers.
//
// Stream functions are typically non-block and reset these flag when polling
// on an read of write and a call to a stream read or write operation is
// needed before this call to reset the flags. The flags are cleared as they
// are added to the sets, so that a failure to service them with a read or
// write does not cause them to keep triggering a wait.
//
// If the 'op' flag is set to 1 then the sets will be set to also wait for new
// input, where a following call to stread read can make new progress. The
// caller must call strream read to clear that input otherwise it will keep
// being added to the set and trigging a wait.
//
// Stream write operations are expected to always succeed so there is no
// waiting for write progress. For an output stream with multiple clients is
// would not be useful.
// -----------------------------------------------------------------------------
void strwant(stream_t *stream, unsigned op, void *wantset) {
  tracet(4, "strwant: op=%u\n", op);
  if (stream->port == NULL) return;

  strlock(stream);
  switch (stream->type) {
    case STR_SERIAL:
      wantserial((serial_t *)stream->port, op, (wantset_t *)wantset);
      break;
    case STR_TCPSVR:
      wanttcpsvr((tcpsvr_t *)stream->port, op, (wantset_t *)wantset);
      break;
    case STR_TCPCLI:
      wanttcpcli((tcpcli_t *)stream->port, op, (wantset_t *)wantset);
      break;
    case STR_NTRIPSRC:
    case STR_NTRIPCLI:
      wantntrip((ntrip_t *)stream->port, op, (wantset_t *)wantset);
      break;
    case STR_NTRIPCAS:
      wantntripc((ntripc_t *)stream->port, op, (wantset_t *)wantset);
      break;
    case STR_UDPCLI:
      wantudpcli((udp_t *)stream->port, op, (wantset_t *)wantset);
      break;
    case STR_UDPSVR:
      wantudpsvr((udp_t *)stream->port, op, (wantset_t *)wantset);
      break;
    case STR_MEMBUF:
    case STR_FILE:
    case STR_FTP:
    case STR_HTTP:
    default:
      strunlock(stream);
      return;
  }
  strunlock(stream);
}

// Get stream status -----------------------------------------------------------
// Get stream status
// Args   : stream_t *stream I   stream
//          char   *msg      IO  status message (NULL: no output)
// Return : status (-1:error,0:close,1:wait,2:connect,3:active)
// Note   : Messages are appended to msg which must be nul terminated.
//-----------------------------------------------------------------------------
int strstat(stream_t *stream, char *msg, size_t msize) {
  tracet(4, "strstat:\n");

  strlock(stream);
  if (msg) rsstrcpy(msg, msize, stream->msg);
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
    case STR_NTRIPSRC:
    case STR_NTRIPCLI:
      state = statentrip((ntrip_t *)stream->port);
      break;
    case STR_NTRIPCAS:
      state = statentripcas((ntripc_t *)stream->port);
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
// Get extended stream status --------------------------------------------------
// Get extended stream status
// Args   : stream_t *stream I   stream
//          char   *msg      IO  extended status message
// Return : status (-1:error,0:close,1:wait,2:connect,3:active)
// Note   : The output is appended to msg which must be nul terminated.
//-----------------------------------------------------------------------------
int strstatx(stream_t *stream, char *msg, size_t msize) {
  tracet(4, "strstatx:\n");

  strlock(stream);

  if (stream->port == NULL) {
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
    case STR_NTRIPSRC:
    case STR_NTRIPCLI:
      state = statexntrip((ntrip_t *)stream->port, msg, msize);
      break;
    case STR_NTRIPCAS:
      state = statexntripcas((ntripc_t *)stream->port, msg, msize);
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
      msg[0] = '\0';
      strunlock(stream);
      return 0;
  }
  if (state == 2 && (int)(tickget() - stream->tact) <= TINTACT) state = 3;
  strunlock(stream);
  return state;
}
// Get stream statistics summary -----------------------------------------------
// Get stream statistics summary
// Args   : stream_t *stream I   stream
//          size_t   *inb      IO   bytes of input  (NULL: no output)
//          unsigned *inr      IO   bps of input    (NULL: no output)
//          size_t   *outb     IO   bytes of output (NULL: no output)
//          unsigned *outr     IO   bps of output   (NULL: no output)
// Return : none
//-----------------------------------------------------------------------------
void strsum(stream_t *stream, size_t *inb, unsigned *inr, size_t *outb, unsigned *outr) {
  tracet(4, "strsum:\n");

  strlock(stream);
  if (inb) *inb = stream->inb;
  if (inr) *inr = stream->inr;
  if (outb) *outb = stream->outb;
  if (outr) *outr = stream->outr;
  strunlock(stream);
}
// Set global stream options ---------------------------------------------------
// Set global stream options
// Args   : int    *opt      I   options
//              opt[0]= inactive timeout (ms) (0: no timeout)
//              opt[1]= interval to reconnect (ms)
//              opt[2]= averaging time of data rate (ms)
//              opt[3]= receive/send buffer size (bytes);
//              opt[4]= file swap margin (s)
//              opt[5]= reserved
//              opt[6]= reserved
//              opt[7]= reserved
// Return : none
//-----------------------------------------------------------------------------
void strsetopt(const int *opt) {
  tracet(3, "strsetopt: opt=%d %d %d %d %d %d %d %d\n", opt[0], opt[1], opt[2], opt[3], opt[4],
         opt[5], opt[6], opt[7]);

  toinact = 0 < opt[0] && opt[0] < 1000 ? 1000 : opt[0] < 0 ? 0 : (unsigned)opt[0];  // >=1s
  ticonnect = opt[1] < 1000 ? 1000 : (unsigned)opt[1];  // >=1s
  tirate = opt[2] < 100 ? 100 : opt[2];                 // >=0.1s
  buffsize = opt[3] < 4096 ? 4096 : opt[3];             // >=4096byte
  fswapmargin = opt[4] < 0 ? 0 : opt[4];
}
// Set timeout time ------------------------------------------------------------
// Set timeout time
// Args   : stream_t *stream I   stream (STR_TCPCLI,STR_NTRIPCLI,STR_NTRIPSRC)
//          unsigned toinact I   inactive timeout (ms) (0: no timeout)
//          int     tirecon  I   reconnect interval (ms) (-1: no reconnect)
// Return : none
//-----------------------------------------------------------------------------
void strsettimeout(stream_t *stream, unsigned toinact, int tirecon) {
  tracet(3, "strsettimeout: toinact=%u tirecon=%d\n", toinact, tirecon);

  tcpcli_t *tcpcli;
  if (stream->type == STR_TCPCLI) {
    tcpcli = (tcpcli_t *)stream->port;
  } else if (stream->type == STR_NTRIPCLI || stream->type == STR_NTRIPSRC) {
    tcpcli = ((ntrip_t *)stream->port)->tcp;
  } else
    return;

  tcpcli->toinact = toinact;
  tcpcli->tirecon = tirecon;
}
// Set local directory ---------------------------------------------------------
// Set local directory path for FTP/HTTP download
// Args   : char   *dir      I   directory for download files
// Return : none
//-----------------------------------------------------------------------------
void strsetdir(const char *dir) {
  tracet(3, "strsetdir: dir=%s\n", dir);
  rsstrcpy(localdir, sizeof(localdir), dir);
}
// Set HTTP/NTRIP proxy address ------------------------------------------------
// Set HTTP/NTRIP proxy address
// Args   : char   *addr     I   HTTP/NTRIP proxy address <address>:<port>
// Return : none
//-----------------------------------------------------------------------------
void strsetproxy(const char *addr) {
  tracet(3, "strsetproxy: addr=%s\n", addr);
  rsstrcpy(proxyaddr, sizeof(proxyaddr), addr);
}
// Get stream time -------------------------------------------------------------
// Get stream time
// Args   : stream_t *stream I   stream
// Return : current time or replay time for playback file
//-----------------------------------------------------------------------------
gtime_t strgettime(stream_t *stream) {
  if (stream->type == STR_FILE && (stream->mode & STR_MODE_R)) {
    const file_t *file = (file_t *)stream->port;
    if (file) return timeadd(file->time, file->start);  // Replay start time
  }
  return utc2gpst(timeget());
}
// Send NMEA request -----------------------------------------------------------
// Send NMEA gpgga message to stream
// Args   : stream_t *stream I   stream
//          sol_t *sol       I   solution
// Return : none
//-----------------------------------------------------------------------------
void strsendnmea(stream_t *stream, const sol_t *sol) {
  tracet(3, "strsendnmea: rr=%.3f %.3f %.3f\n", sol->rr[0], sol->rr[1], sol->rr[2]);

  uint8_t buff[1024];
  outnmea_gga(buff, sol);
  strwrite(stream, buff, sizeof(buff), 0, strlen((char *)buff));
}
// Generate general hex message ----------------------------------------------
static size_t gen_hex(const char *msg, uint8_t *buff, size_t size) {
  tracet(4, "gen_hex: msg=%s\n", msg);

  char mbuff[1024];
  rsstrcpy(mbuff, sizeof(mbuff), msg);
  char *args[256], *r;
  unsigned narg = 0;
  for (char *p = strtok_r(mbuff, " ", &r); p && narg < 256; p = strtok_r(NULL, " ", &r)) {
    args[narg++] = p;
  }
  size_t len = 0;
  for (unsigned i = 0; i < narg; i++) {
    uint32_t byte;
    if (sscanf(args[i], "%x", &byte)) {
      RBOUNDSCHECK(buff, size, len);
      buff[len++] = (uint8_t)byte;
    }
  }
  return len;
}
// Set bitrate ---------------------------------------------------------------
static int set_brate(stream_t *str, unsigned brate) {
  unsigned type = str->type;
  if (type != STR_SERIAL) return 0;

  char path[MAXSTRPATH];
  rsstrcpy(path, sizeof(path), str->path);

  ssize_t pi = rsstrchr(path, 0, ':');
  if (pi < 0) {
    rscatprintf(path, sizeof(path), ":%u", brate);
  } else {
    char buff[MAXSTRPATH] = "";
    ssize_t qi = rsstrchr(path, pi + 1, ':');
    if (qi >= 0) rssubstrcpy(buff, sizeof(buff), path, qi);
    rssnprintf(path + pi, sizeof(path) - pi, ":%u%s", brate, buff);
  }
  unsigned mode = str->mode;
  strclose(str);
  return stropen(str, type, mode, path);
}
// Send receiver command -------------------------------------------------------
// Send receiver commands to stream
// Args   : stream_t *stream I   stream
//          char   *cmd      I   receiver command strings
// Return : none
//-----------------------------------------------------------------------------
void strsendcmd(stream_t *stream, const char *cmd) {
  tracet(3, "strsendcmd: cmd=%s\n", cmd);

  const char *p = cmd;

  for (;;) {
    const char *q;
    for (q = p;; q++)
      if (*q == '\r' || *q == '\n' || *q == '\0') break;
    int n = (int)(q - p);
    char msg[1024];
    rsesubstrcpy(msg, sizeof(msg), p, 0, n);

    if (msg[0] == '\0' || *msg == '#') {  // Null or comment
      ;
    } else if (*msg == '!') {  // Binary escape

      if (strncmp(msg + 1, "WAIT", 4) == 0) {  // Wait
        unsigned ms;
        if (sscanf(msg + 5, "%u", &ms) < 1) ms = 100;
        if (ms > 3000) ms = 3000;  // Max 3 s
        sleepms(ms);
      } else if (strncmp(msg + 1, "BRATE", 5) == 0) {  // Set bitrate
        unsigned brate;
        if (sscanf(msg + 6, "%u", &brate) < 1) brate = 115200;
        set_brate(stream, brate);
        sleepms(500);
      } else if (strncmp(msg + 1, "UBX", 3) == 0) {  // Ublox
        uint8_t buff[1024];
        ssize_t m = gen_ubx(msg + 4, buff);
        if (m > 0) strwrite(stream, buff, sizeof(buff), 0, m);
      } else if (strncmp(msg + 1, "STQ", 3) == 0) {  // Skytraq
        uint8_t buff[1024];
        ssize_t m = gen_stq(msg + 4, buff);
        if (m > 0) strwrite(stream, buff, sizeof(buff), 0, m);
      } else if (strncmp(msg + 1, "NVS", 3) == 0) {  // Nvs
        uint8_t buff[1024];
        ssize_t m = gen_nvs(msg + 4, buff);
        if (m > 0) strwrite(stream, buff, sizeof(buff), 0, m);
      } else if (strncmp(msg + 1, "HEX", 3) == 0) {  // General hex message
        uint8_t buff[1024];
        size_t m = gen_hex(msg + 4, buff, sizeof(buff));
        if (m > 0) strwrite(stream, buff, sizeof(buff), 0, m);
      }
    } else {
      const char cmdend[] = "\r\n";
      rsstrcat(msg, sizeof(msg), cmdend);
      strwrite(stream, (uint8_t *)msg, sizeof(msg), 0, n + 2);
    }
    if (*q == '\0')
      break;
    else
      p = q + 1;
  }
}
