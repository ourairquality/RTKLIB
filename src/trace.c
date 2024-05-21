/* Debug trace functions -----------------------------------------------------*/
#ifdef TRACE
#include "rtklib.h"

static FILE *fp_trace = NULL;    /* File pointer of trace */
static char file_trace[FNSIZE];  /* Trace file */
static int level_trace = 0;      /* Level of trace */
static uint32_t tick_trace = 0;  /* Tick time at traceopen (ms) */
static gtime_t time_trace = {0}; /* Time at traceopen */
static rtklib_lock_t lock_trace; /* Lock for trace */

static void traceswap(void) {
  gtime_t time = utc2gpst(timeget());

  rtklib_lock(&lock_trace);

  if ((int)(time2gpst(time, NULL) / INT_SWAP_TRAC) ==
      (int)(time2gpst(time_trace, NULL) / INT_SWAP_TRAC)) {
    rtklib_unlock(&lock_trace);
    return;
  }
  time_trace = time;

  char path[FNSIZE];
  if (!reppath(file_trace, path, sizeof(path), time, "", "")) {
    rtklib_unlock(&lock_trace);
    return;
  }
  if (fp_trace) fclose(fp_trace);

  fp_trace = fopen(path, "w");
  if (!fp_trace) {
    fp_trace = stderr;
  }
  rtklib_unlock(&lock_trace);
}
extern void traceopen(const char *file) {
  gtime_t time = utc2gpst(timeget());

  char path[FNSIZE];
  reppath(file, path, sizeof(path), time, "", "");
  if (!*path || !(fp_trace = fopen(path, "w"))) fp_trace = stderr;
  rtkstrcpy(file_trace, sizeof(file_trace), file);
  tick_trace = tickget();
  time_trace = time;
  rtklib_initlock(&lock_trace);
}
extern void traceclose(void) {
  if (fp_trace && fp_trace != stderr) fclose(fp_trace);
  fp_trace = NULL;
  file_trace[0] = '\0';
}
extern void tracelevel(int level) { level_trace = level; }
extern int gettracelevel(void) { return level_trace; }
extern void trace_impl(int level, const char *format, ...) {
  va_list ap;

  /* Print error message to stderr */
  if (level <= 1) {
    va_start(ap, format);
    vfprintf(stderr, format, ap);
    va_end(ap);
  }
  if (!fp_trace || level > level_trace) return;
  traceswap();
  fprintf(fp_trace, "%d ", level);
  va_start(ap, format);
  vfprintf(fp_trace, format, ap);
  va_end(ap);
  fflush(fp_trace);
}
extern void tracet_impl(int level, const char *format, ...) {
  va_list ap;

  if (!fp_trace || level > level_trace) return;
  traceswap();
  fprintf(fp_trace, "%d %9.3Lf: ", level, (tickget() - tick_trace) / 1000.0L);
  va_start(ap, format);
  vfprintf(fp_trace, format, ap);
  va_end(ap);
  fflush(fp_trace);
}
extern void tracemat_impl(int level, const long double *A, int n, int m, int p, int q) {
  if (!fp_trace || level > level_trace) return;
  matfprint(A, n, m, p, q, fp_trace);
  fflush(fp_trace);
}
extern void traceobs_impl(int level, const obsd_t *obs, int n) {
  if (!fp_trace || level > level_trace) return;
  for (int i = 0; i < n; i++) {
    char str[40];
    time2str(obs[i].time, str, 3);
    char id[8];
    satno2id(obs[i].sat, id);
    fprintf(fp_trace,
            " (%2d) %s %-3s rcv%d %13.3Lf %13.3Lf %13.3Lf %13.3Lf %d %d %d %d "
            "%x %x %3.2Lf %3.2Lf\n",
            i + 1, str, id, obs[i].rcv, obs[i].L[0], obs[i].L[1], obs[i].P[0], obs[i].P[1],
            obs[i].LLI[0], obs[i].LLI[1], obs[i].code[0], obs[i].code[1], obs[i].Lstd[0],
            obs[i].Pstd[0], obs[i].SNR[0] * SNR_UNIT, obs[i].SNR[1] * SNR_UNIT);
  }
  fflush(fp_trace);
}
extern void tracenav_impl(int level, const nav_t *nav) {
  if (!fp_trace || level > level_trace) return;
  for (int i = 0; i < MAXSAT; i++) {
    for (int j = 0; j < nav->n[i]; j++) {
      char s1[40];
      time2str(nav->eph[i][j].toe, s1, 0);
      char s2[40];
      time2str(nav->eph[i][j].ttr, s2, 0);
      char id[8];
      satno2id(nav->eph[i][j].sat, id);
      fprintf(fp_trace, "(%3d) %-3s : %s %s %3d %3d %02x\n", i + 1, id, s1, s2, nav->eph[i][j].iode,
              nav->eph[i][j].iodc, nav->eph[i][j].svh);
    }
  }
  fprintf(fp_trace, "(ion) %9.4Le %9.4Le %9.4Le %9.4Le\n", nav->ion_gps[0], nav->ion_gps[1],
          nav->ion_gps[2], nav->ion_gps[3]);
  fprintf(fp_trace, "(ion) %9.4Le %9.4Le %9.4Le %9.4Le\n", nav->ion_gps[4], nav->ion_gps[5],
          nav->ion_gps[6], nav->ion_gps[7]);
  fprintf(fp_trace, "(ion) %9.4Le %9.4Le %9.4Le %9.4Le\n", nav->ion_gal[0], nav->ion_gal[1],
          nav->ion_gal[2], nav->ion_gal[3]);
}
extern void tracegnav_impl(int level, const nav_t *nav) {
  if (!fp_trace || level > level_trace) return;
  for (int i = 0; i < NSATGLO; i++) {
    for (int j = 0; j < nav->ng[i]; j++) {
      char s1[40];
      time2str(nav->geph[i][j].toe, s1, 0);
      char s2[40];
      time2str(nav->geph[i][j].tof, s2, 0);
      char id[8];
      satno2id(nav->geph[i][j].sat, id);
      fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d %8.3Lf\n", i + 1, id, s1, s2,
              nav->geph[i][j].frq, nav->geph[i][j].svh, nav->geph[i][j].taun * 1E6L);
    }
  }
}
extern void tracehnav_impl(int level, const nav_t *nav) {
  if (!fp_trace || level > level_trace) return;
  for (int i = 0; i < NSATSBS; i++) {
    for (int j = 0; j < nav->ns[i]; j++) {
      char s1[40];
      time2str(nav->seph[i][j].t0, s1, 0);
      char s2[40];
      time2str(nav->seph[i][j].tof, s2, 0);
      char id[8];
      satno2id(nav->seph[i][j].sat, id);
      fprintf(fp_trace, "(%3d) %-3s : %s %s %2d %2d\n", i + 1, id, s1, s2, nav->seph[i][j].svh,
              nav->seph[i][j].sva);
    }
  }
}
extern void tracepeph_impl(int level, const nav_t *nav) {
  if (!fp_trace || level > level_trace) return;

  for (int i = 0; i < nav->ne; i++) {
    char s[40];
    time2str(nav->peph[i].time, s, 0);
    for (int j = 0; j < MAXSAT; j++) {
      char id[8];
      satno2id(j + 1, id);
      fprintf(fp_trace,
              "%-3s %d %-3s %13.3Lf %13.3Lf %13.3Lf %13.3Lf %6.3Lf %6.3Lf "
              "%6.3Lf %6.3Lf\n",
              s, nav->peph[i].index, id, nav->peph[i].pos[j][0], nav->peph[i].pos[j][1],
              nav->peph[i].pos[j][2], nav->peph[i].pos[j][3] * 1E9L, nav->peph[i].std[j][0],
              nav->peph[i].std[j][1], nav->peph[i].std[j][2], nav->peph[i].std[j][3] * 1E9L);
    }
  }
}
extern void tracepclk_impl(int level, const nav_t *nav) {
  if (!fp_trace || level > level_trace) return;

  for (int i = 0; i < nav->nc; i++) {
    char s[40];
    time2str(nav->pclk[i].time, s, 0);
    for (int j = 0; j < MAXSAT; j++) {
      char id[8];
      satno2id(j + 1, id);
      fprintf(fp_trace, "%-3s %d %-3s %13.3Lf %6.3Lf\n", s, nav->pclk[i].index, id,
              nav->pclk[i].clk[j][0] * 1E9L, nav->pclk[i].std[j][0] * 1E9L);
    }
  }
}
extern void traceb_impl(int level, const uint8_t *p, size_t size, int n) {
  if (!fp_trace || level > level_trace || n <= 0) return;
  RTKBOUNDSCHECK(p, size, n - 1);
  for (int i = 0; i < n; i++) fprintf(fp_trace, "%02X%s", p[i], i % 8 == 7 ? " " : "");
  fprintf(fp_trace, "\n");
}

#endif /* TRACE */
