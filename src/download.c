/*------------------------------------------------------------------------------
 * download.c : GNSS data downloader
 *
 *          Copyright (C) 2012-2020 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision:$ $Date:$
 * History : 2012/12/28  1.0  new
 *           2013/06/02  1.1  replace S_IREAD by S_IRUSR
 *           2020/11/30  1.2  support protocol https:// and ftps://
 *                            support compressed RINEX (CRX) files
 *                            support wild-card (*) in URL for FTP and FTPS
 *                            use "=" to separate file name from URL
 *                            fix bug on double-free of download paths
 *                            limit max number of download paths
 *                            use integer types in stdint.h
 *----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199506
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "rtklib.h"

#define FTP_CMD "wget"         /* FTP/HTTP command */
#define FTP_TIMEOUT 60         /* FTP/HTTP timeout (s) */
#define FTP_LISTING ".listing" /* FTP listing file */
#define FTP_NOFILE 2048        /* FTP error no file */
#define HTTP_NOFILE 1          /* HTTP error no file */
#define FTP_RETRY 3            /* FTP number of retry */
#define MAX_PATHS 131072       /* Max number of download paths */

/* Type definitions ----------------------------------------------------------*/

typedef struct { /* Download path type */
  char *remot;   /* Remote path */
  char *local;   /* Local path */
} path_t;

typedef struct { /* Download paths type */
  path_t *path;  /* Download paths */
  int n, nmax;   /* Number and max number of paths */
} paths_t;

/* Execute command with test timeout -----------------------------------------*/
extern int execcmd_to(const char *cmd) {
#ifdef WIN32
  PROCESS_INFORMATION info;
  STARTUPINFO si = {0};
  DWORD stat;
  char cmds[4096];

  si.cb = sizeof(si);
  rtksnprintf(cmds, sizeof(cmds), "cmd /c %s", cmd);
  if (!CreateProcess(NULL, (LPTSTR)cmds, NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si,
                     &info))
    return -1;

  while (WaitForSingleObject(info.hProcess, 10) == WAIT_TIMEOUT) {
    showmsg("");
  }
  if (!GetExitCodeProcess(info.hProcess, &stat)) stat = -1;
  CloseHandle(info.hProcess);
  CloseHandle(info.hThread);
  return (int)stat;
#else
  return system(cmd);
#endif
}
/* Generate path by replacing keywords ---------------------------------------*/
static void genpath(const char *file, const char *name, gtime_t time, int seqno, char *path,
                    size_t size) {
  char l_name[FNSIZE] = "", u_name[FNSIZE] = "";
  rtkstrcpy(l_name, sizeof(l_name), name);
  rtkstrcpy(u_name, sizeof(u_name), name);
  for (int i = 0; l_name[i]; i++) l_name[i] = (char)tolower(l_name[i]);
  for (int i = 0; u_name[i]; i++) u_name[i] = (char)tolower(u_name[i]);

  char buff[FNSIZE];
  size_t i = 0;
  for (int j = 0; file[j]; i++, j++) {
    RTKBOUNDSCHECK(buff, sizeof(buff), i);
    buff[i] = file[j];
    if (file[j] != '%') continue;
    char ch = file[j + 1];
    if (ch == 's' || ch == 'r') {
      rtkstrcpy(buff + i, sizeof(buff) - i, l_name);
      i = strlen(buff) - 1;
      j++;
    } else if (ch == 'S' || ch == 'R') {
      rtkstrcpy(buff + i, sizeof(buff) - i, u_name);
      i = strlen(buff) - 1;
      j++;
    } else if (ch == 'N') {
      rtksnprintf(buff + i, sizeof(buff) - i, "%d", seqno);
      i = strlen(buff) - 1;
      j++;
    } else if (ch == '{') {
      size_t vstart = j + 2;
      const char *r = strchr(file + vstart, '}');
      if (!r) continue;
      char var[FNSIZE] = "";
      size_t vend = r - file;
      rtkesubstrcpy(var, sizeof(var), file, vstart, vend);
      const char *env = getenv(var);
      if (env) {
        rtkstrcpy(buff + i, sizeof(buff) - i, env);
        i = strlen(buff) - 1;
      }
      j = r - file;
    }
  }
  buff[i] = '\0';
  reppath(buff, path, size, time, "", "");
}
/* Parse field strings separated by spaces -----------------------------------*/
static char *parse_str(char *buff, char *str, int nmax) {
  char *p, *q, sep[] = " \r\n";

  for (p = buff; *p == ' '; p++)
    ;

  if (*p == '"') sep[0] = *p++; /* Enclosed within quotation marks */

  for (q = str; *p && !strchr(sep, *p); p++) {
    if (q < str + nmax - 1) *q++ = *p;
  }
  *q = '\0';
  return *p ? p + 1 : p;
}
/* Compare str1 and str2 with wildcards (*) ----------------------------------*/
static bool cmp_str(const char *str1, const char *str2) {
  char s1[1026], s2[1026];

  rtksnprintf(s1, sizeof(s1), "^%s$", str1);
  rtksnprintf(s2, sizeof(s2), "^%s$", str2);

  char *p = s1, *r;
  for (char *q = strtok_r(s2, "*", &r); q; q = strtok_r(NULL, "*", &r)) {
    p = strstr(p, q);
    if (p)
      p += strlen(q);
    else
      break;
  }
  return p != NULL;
}
/* Remote to local file path -------------------------------------------------*/
static void remot2local(const char *remot, const char *dir, char *local, size_t size) {
  char *p = strrchr(remot, '=');
  if (p)
    p++;
  else if ((p = strrchr(remot, '/')))
    p++;
  else
    p = (char *)remot;

  rtksnprintf(local, size, "%s%c%s", dir, RTKLIB_FILEPATHSEP, p);
}
/* Test file existence -------------------------------------------------------*/
static bool exist_file(const char *local) {
#ifdef WIN32
  DWORD stat = GetFileAttributes(local);
  return stat != 0xFFFFFFFF;
#else
  struct stat buff;
  if (stat(local, &buff)) return 0;
  return (buff.st_mode & S_IRUSR) != 0;
#endif
}
/* Test local file existence -------------------------------------------------*/
static int test_file(const char *local) {
  if (strchr(local, '*')) { /* Test wild-card (*) in path */
    return 0;
  }
  int comp = 0;
  char buff[FNSIZE];
  rtkstrcpy(buff, sizeof(buff), local);
  char *p = strrchr(buff, '.');
  if (p && (!strcmp(p, ".z") || !strcmp(p, ".gz") || !strcmp(p, ".zip") || !strcmp(p, ".Z") ||
            !strcmp(p, ".GZ") || !strcmp(p, ".ZIP"))) {
    *p = '\0';
    if (exist_file(buff)) return 1;
    comp = 1;
  }
  p = strrchr(buff, '.');
  if (p && strlen(p) == 4 && (*(p + 3) == 'd' || *(p + 3) == 'D')) {
    *(p + 3) = *(p + 3) == 'd' ? 'o' : 'O';
    if (exist_file(buff)) return 1;
    comp = 1;
  }
  p = strrchr(buff, '.');
  if (p && (!strcmp(p, ".crx") || !strcmp(p, ".CRX"))) {
    if (!strcmp(p, ".crx"))
      rtkstrcpy(p, 5, ".cro");
    else
      rtkstrcpy(p, 5, ".CRO");
    if (exist_file(buff)) return 1;
    comp = 1;
  }
  if (!exist_file(buff)) return 0;
  return comp ? 2 : 1;
}
/* Free download paths -------------------------------------------------------*/
static void free_path(paths_t *paths) {
  if (!paths) return;
  for (int i = 0; i < paths->n; i++) {
    free(paths->path[i].remot);
    free(paths->path[i].local);
  }
  free(paths->path);
}
/* Add download paths --------------------------------------------------------*/
static bool add_path(paths_t *paths, const char *remot, const char *dir) {
  path_t *paths_path;

  if (paths->n >= paths->nmax) {
    int nmax = paths->nmax <= 0 ? FNSIZE : paths->nmax * 2;
    if (nmax > MAX_PATHS) {
      return false;
    }
    paths_path = (path_t *)realloc(paths->path, sizeof(path_t) * nmax);
    if (!paths_path) {
      return false;
    }
    paths->path = paths_path;
    paths->nmax = nmax;
  }
  char local[FNSIZE];
  remot2local(remot, dir, local, sizeof(local));

  paths->path[paths->n].remot = paths->path[paths->n].local = NULL;

  char *path_remot = (char *)malloc(strlen(remot) + 1);
  if (!path_remot) return 0;
  char *path_local = (char *)malloc(strlen(local) + 1);
  if (!path_local) {
    free(path_remot);
    return false;
  }
  /* Note the use of strcpy here is safe as the destination has been just allocated to fit */
  strcpy(path_remot, remot);
  strcpy(path_local, local);
  paths->path[paths->n].remot = path_remot;
  paths->path[paths->n].local = path_local;
  paths->n++;
  return true;
}
/* Generate download path ----------------------------------------------------*/
static bool gen_path(gtime_t time, gtime_t time_p, int seqnos, int seqnoe, const url_t *url,
                     const char *sta, const char *dir, paths_t *paths) {
  if (!*dir) dir = url->dir;
  if (!*dir) dir = ".";

  if (strstr(url->path, "%N")) {
    for (int i = seqnos; i <= seqnoe; i++) {
      char remot[FNSIZE], dir_t[FNSIZE];
      genpath(url->path, sta, time, i, remot, sizeof(remot));
      genpath(dir, sta, time, i, dir_t, sizeof(dir_t));
      if (time_p.time) {
        char remot_p[FNSIZE];
        genpath(url->path, sta, time_p, i, remot_p, sizeof(remot_p));
        if (!strcmp(remot_p, remot)) continue;
      }
      if (!add_path(paths, remot, dir_t)) return false;
    }
  } else {
    char remot[FNSIZE], dir_t[FNSIZE];
    genpath(url->path, sta, time, 0, remot, sizeof(remot));
    genpath(dir, sta, time, 0, dir_t, sizeof(dir_t));
    if (time_p.time) {
      char remot_p[FNSIZE];
      genpath(url->path, sta, time_p, 0, remot_p, sizeof(remot_p));
      if (!strcmp(remot_p, remot)) return false;
    }
    if (!add_path(paths, remot, dir_t)) return false;
  }
  return true;
}
/* Generate download paths ---------------------------------------------------*/
static bool gen_paths(gtime_t time, gtime_t time_p, int seqnos, int seqnoe, const url_t *url,
                      const char **stas, int nsta, const char *dir, paths_t *paths) {
  if (strstr(url->path, "%s") || strstr(url->path, "%S")) {
    for (int i = 0; i < nsta; i++) {
      if (!gen_path(time, time_p, seqnos, seqnoe, url, stas[i], dir, paths)) {
        return false;
      }
    }
  } else {
    if (!gen_path(time, time_p, seqnos, seqnoe, url, "", dir, paths)) {
      return false;
    }
  }
  return true;
}
/* Compact download paths ----------------------------------------------------*/
static void compact_paths(paths_t *paths) {
  for (int i = 0; i < paths->n; i++) {
    for (int j = i + 1; j < paths->n; j++) {
      if (strcmp(paths->path[i].remot, paths->path[j].remot)) continue;
      free(paths->path[j].remot);
      free(paths->path[j].local);
      for (int k = j; k < paths->n - 1; k++) paths->path[k] = paths->path[k + 1];
      paths->n--;
      j--;
    }
  }
}
/* Generate local directory recursively --------------------------------------*/
static bool mkdir_r(const char *dir) {
#ifdef WIN32
  if (!*dir || !strcmp(dir + 1, ":\\")) return 1;

  char pdir[FNSIZE];
  rtkstrcpy(pdir, sizeof(pdir), dir);
  char *p = strrchr(pdir, RTKLIB_FILEPATHSEP);
  if (p) {
    *p = '\0';
    WIN32_FIND_DATA data;
    HANDLE h = FindFirstFile(pdir, &data);
    if (h == INVALID_HANDLE_VALUE) {
      if (!mkdir_r(pdir)) return false;
    } else
      FindClose(h);
  }
  if (CreateDirectory(dir, NULL) || GetLastError() == ERROR_ALREADY_EXISTS) return true;

  trace(2, "directory generation error: dir=%s\n", dir);
  return false;
#else
  if (!*dir) return true;

  char pdir[FNSIZE];
  rtkstrcpy(pdir, sizeof(pdir), dir);
  char *p = strrchr(pdir, RTKLIB_FILEPATHSEP);
  if (p) {
    *p = '\0';
    FILE *fp = fopen(pdir, "r");
    if (!fp) {
      if (!mkdir_r(pdir)) return false;
    } else
      fclose(fp);
  }
  if (!mkdir(dir, 0777) || errno == EEXIST) return true;

  trace(2, "directory generation error: dir=%s\n", dir);
  return false;
#endif
}
/* Get remote file list for FTP or FTPS --------------------------------------*/
static bool get_list(const path_t *path, const char *usr, const char *pwd, const char *proxy) {
  const char *opt2 = "";
#ifndef WIN32
  opt2 = " -o /dev/null";
#endif
  remove(FTP_LISTING);

  char remot[FNSIZE];
  rtkstrcpy(remot, sizeof(remot), path->remot);

  char *p = strrchr(remot, '/');
  if (p) {
    size_t start = (p - remot) + 1;
    rtkstrcpy(remot + start, sizeof(remot) - start, "__REQUEST_LIST__");
  } else
    return false;

  char env[1024] = "";
  const char *opt = "";
  if (*proxy) {
    rtksnprintf(env, sizeof(env), "set ftp_proxy=http://%s & ", proxy);
    opt = "--proxy=on ";
  }
  char cmd[4096];
  rtksnprintf(cmd, sizeof(cmd),
              "%s%s %s --ftp-user=%s --ftp-password=%s --glob=off "
              "--passive-ftp --no-remove-listing -N %s-t 1 -T %d%s\n",
              env, FTP_CMD, remot, usr, pwd, opt, FTP_TIMEOUT, opt2);
  execcmd_to(cmd);

  FILE *fp = fopen(FTP_LISTING, "r");
  if (!fp) return false;
  fclose(fp);
  return true;
}
/* Replace wild-card (*) in the paths ----------------------------------------*/
static bool rep_paths(path_t *path, const char *file) {
  char buff1[FNSIZE], buff2[FNSIZE];
  rtkstrcpy(buff1, sizeof(buff1), path->remot);
  rtkstrcpy(buff2, sizeof(buff2), path->local);
  size_t i1 = 0;
  char *p = strrchr(buff1, '/');
  if (p) i1 = (p - buff1) + 1;
  size_t i2 = 0;
  char *q = strrchr(buff2, RTKLIB_FILEPATHSEP);
  if (q) i2 = (q - buff2) + 1;
  rtkstrcpy(buff1 + i1, sizeof(buff1) - i1, file);
  rtkstrcpy(buff2 + i2, sizeof(buff2) - i2, file);

  char *remot = (char *)malloc(strlen(buff1) + 1);
  if (!remot) return false;
  char *local = (char *)malloc(strlen(buff2) + 1);
  if (!local) {
    free(remot);
    return false;
  }
  /* Note the use of strcpy is safe here as the destination buffer was just allocated to fit */
  strcpy(remot, buff1);
  strcpy(local, buff2);
  free(path->remot);
  free(path->local);
  path->remot = remot;
  path->local = local;
  return true;
}
/* Test file in remote file list ---------------------------------------------*/
static bool test_list(path_t *path) {
  FILE *fp = fopen(FTP_LISTING, "r");
  if (!fp) return true;

  char *p = strrchr(path->remot, '/');
  if (!p) {
    fclose(fp);
    return true;
  }
  const char *file = p + 1;

  /* Search file in remote file list */
  char buff[FNSIZE];
  while (fgets(buff, sizeof(buff), fp)) {
    /* Remove symbolic link */
    p = strstr(buff, "->");
    if (p) *p = '\0';

    for (int i = strlen(buff) - 1; i >= 0; i--) {
      if (strchr(" \r\n", buff[i]))
        buff[i] = '\0';
      else
        break;
    }
    /* File as last field */
    const char *list;
    p = strrchr(buff, ' ');
    if (p)
      list = p + 1;
    else
      list = buff;

    if (!strcmp(file, list)) {
      fclose(fp);
      return true;
    }
    /* Compare with wild-card (*) */
    if (cmp_str(list, file)) {
      /* Replace wild-card (*) in the paths */
      if (!rep_paths(path, list)) {
        fclose(fp);
        return false;
      }
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  return false;
}
/* Execute download ----------------------------------------------------------*/
static bool exec_down(path_t *path, char *remot_p, size_t rsize, const char *usr, const char *pwd,
                      const char *proxy, int opts, int *n, FILE *fp) {
  char dir[FNSIZE];
  rtkstrcpy(dir, sizeof(dir), path->local);
  char *p = strrchr(dir, RTKLIB_FILEPATHSEP);
  if (p) *p = '\0';

  int proto;
  if (!strncmp(path->remot, "ftp://", 6))
    proto = 0;
  else if (!strncmp(path->remot, "ftps://", 7))
    proto = 2;
  else if (!strncmp(path->remot, "http://", 7))
    proto = 1;
  else if (!strncmp(path->remot, "https://", 8))
    proto = 1;
  else {
    trace(2, "exec_down: invalid path %s\n", path->remot);
    showmsg("STAT=X");
    if (fp) fprintf(fp, "%s ERROR (INVALID PATH)\n", path->remot);
    n[1]++;
    return false;
  }
  /* Test local file existence */
  if (!(opts & DLOPT_FORCE) && test_file(path->local)) {
    showmsg("STAT=.");
    if (fp) fprintf(fp, "%s in %s\n", path->remot, dir);
    n[2]++;
    return false;
  }
  showmsg("STAT=_");

  /* Get remote file list for FTP or FTPS */
  if ((proto == 0 || proto == 2) && (p = strrchr(path->remot, '/')) &&
      strncmp(path->remot, remot_p, p - path->remot)) {
    if (get_list(path, usr, pwd, proxy)) {
      rtkstrcpy(remot_p, rsize, path->remot);
    }
  }
  /* Test file in listing for FTP or FTPS or extend wild-card in file path */
  if ((proto == 0 || proto == 2) && !test_list(path)) {
    showmsg("STAT=x");
    if (fp) fprintf(fp, "%s NO_FILE\n", path->remot);
    n[1]++;
    return false;
  }
  /* Generate local directory recursively */
  if (!mkdir_r(dir)) {
    showmsg("STAT=X");
    if (fp) fprintf(fp, "%s -> %s ERROR (LOCAL DIR)\n", path->remot, dir);
    n[3]++;
    return false;
  }
  /* Re-test local file existence for file with wild-card */
  if (!(opts & DLOPT_FORCE) && test_file(path->local)) {
    showmsg("STAT=.");
    if (fp) fprintf(fp, "%s in %s\n", path->remot, dir);
    n[2]++;
    return false;
  }
  /* Proxy option */
  char opt[1024] = "";
  char env[1024] = "";
  if (*proxy) {
    rtksnprintf(env, sizeof(env), "set %s_proxy=http://%s & ",
                proto == 0 || proto == 2 ? "ftp" : "http", proxy);
    rtksnprintf(opt, sizeof(opt), " --proxy=on ");
  }
  /* Download command */
#ifdef WIN32
  const char *opt2 = "";
#else
  const char *opt2 = " 2> /dev/null";
#endif
  char errfile[FNSIZE];
  rtksnprintf(errfile, sizeof(errfile), "%s.err", path->local);
  char cmd[4096];
  if (proto == 0 || proto == 2) {
    rtksnprintf(cmd, sizeof(cmd),
                "%s%s %s --ftp-user=%s --ftp-password=%s --glob=off "
                "--passive-ftp %s-t %d -T %d -O \"%s\" -o \"%s\"%s\n",
                env, FTP_CMD, path->remot, usr, pwd, opt, FTP_RETRY, FTP_TIMEOUT, path->local,
                errfile, opt2);
  } else {
    if (*pwd) {
      rtkcatprintf(opt, sizeof(opt), " --http-user=%s --http-password=%s ", usr, pwd);
    }
    rtksnprintf(cmd, sizeof(cmd), "%s%s %s %s-t %d -T %d -O \"%s\" -o \"%s\"%s\n", env, FTP_CMD,
                path->remot, opt, FTP_RETRY, FTP_TIMEOUT, path->local, errfile, opt2);
  }
  if (fp) fprintf(fp, "%s -> %s", path->remot, dir);

  /* Execute download command */
  int ret = execcmd_to(cmd);
  if (ret) {
    if ((proto == 0 && ret == FTP_NOFILE) || (proto == 1 && ret == HTTP_NOFILE)) {
      showmsg("STAT=x");
      if (fp) fprintf(fp, " NO_FILE\n");
      n[1]++;
    } else {
      trace(2, "exec_down: error proto=%d %d\n", proto, ret);
      showmsg("STAT=X");
      if (fp) fprintf(fp, " ERROR (%d)\n", ret);
      n[3]++;
    }
    remove(path->local);
    if (!(opts & DLOPT_HOLDERR)) {
      remove(errfile);
    }
    return ret == 2;
  }
  remove(errfile);

  /* Uncompress download file */
  if (!(opts & DLOPT_KEEPCMP) && (p = strrchr(path->local, '.')) &&
      (!strcmp(p, ".z") || !strcmp(p, ".gz") || !strcmp(p, ".zip") || !strcmp(p, ".Z") ||
       !strcmp(p, ".GZ") || !strcmp(p, ".ZIP"))) {
    char tmpfile[FNSIZE];
    if (rtk_uncompress(path->local, tmpfile, sizeof(tmpfile))) {
      remove(path->local);
    } else {
      trace(2, "exec_down: uncompress error\n");
      showmsg("STAT=C");
      if (fp) fprintf(fp, " ERROR (UNCOMP)\n");
      n[3]++;
      return false;
    }
  }
  showmsg("STAT=o");
  if (fp) fprintf(fp, " OK\n");
  n[0]++;
  return false;
}
/* Test local file -----------------------------------------------------------*/
static bool test_local(gtime_t ts, gtime_t te, long double ti, const char *path, const char *sta,
                       const char *dir, int *nc, int *nt, FILE *fp) {
  bool abort = false;

  for (gtime_t time = ts; timediff(time, te) <= 1E-3; time = timeadd(time, ti)) {
    char remot[FNSIZE];
    genpath(path, sta, time, 0, remot, sizeof(remot));
    char dir_t[FNSIZE];
    genpath(dir, sta, time, 0, dir_t, sizeof(dir_t));
    char local[FNSIZE];
    remot2local(remot, dir_t, local, sizeof(local));

    char str[2 * FNSIZE];
    rtksnprintf(str, sizeof(str), "%s->%s", path, local);
    if (showmsg(str)) {
      abort = true;
      break;
    }

    int stat = test_file(local);

    fprintf(fp, " %s", stat == 0 ? "-" : (stat == 1 ? "o" : "z"));

    showmsg("STAT=%s", stat == 0 ? "x" : (stat == 1 ? "o" : "z"));

    (*nt)++;
    if (stat) (*nc)++;
  }
  fprintf(fp, "\n");
  return abort;
}
/* Test local files ----------------------------------------------------------*/
static bool test_locals(gtime_t ts, gtime_t te, long double ti, const url_t *url, const char **stas,
                        int nsta, const char *dir, int *nc, int *nt, FILE *fp) {
  if (strstr(url->path, "%s") || strstr(url->path, "%S")) {
    fprintf(fp, "%s\n", url->type);
    for (int i = 0; i < nsta; i++) {
      fprintf(fp, "%-12s:", stas[i]);
      if (test_local(ts, te, ti, url->path, stas[i], *dir ? dir : url->dir, nc + i, nt + i, fp)) {
        return true;
      }
    }
  } else {
    fprintf(fp, "%-12s:", url->type);
    if (test_local(ts, te, ti, url->path, "", *dir ? dir : url->dir, nc, nt, fp)) {
      return true;
    }
  }
  return false;
}
/* Print total count of local files ------------------------------------------*/
static int print_total(const url_t *url, const char **stas, int nsta, const int *nc, const int *nt,
                       FILE *fp) {
  if (strstr(url->path, "%s") || strstr(url->path, "%S")) {
    fprintf(fp, "%s\n", url->type);
    for (int i = 0; i < nsta; i++) {
      fprintf(fp, "%-12s: %5d/%5d\n", stas[i], nc[i], nt[i]);
    }
    return nsta;
  }
  fprintf(fp, "%-12s: %5d/%5d\n", url->type, nc[0], nt[0]);
  return 1;
}
/* Read URL list file ----------------------------------------------------------
 * Read URL list file for GNSS data
 * Args   : char   *file     I   URL list file
 *          char   **types   I   selected types ("*":wildcard)
 *          int    ntype     I   number of selected types
 *          urls_t *urls     O   URL list
 *          int    nmax      I   max number of URL list
 * Return : number of URL addresses (0:error)
 * Notes  :
 *    (1) URL list file contains records containing the following fields
 *        separated by spaces. if a field contains spaces, enclose it within "".
 *
 *        data_type  url_address       default_local_directory
 *
 *    (2) strings after # in a line are treated as comments
 *    (3) url_address should be:
 *
 *        ftp://host_address/file_path or
 *        ftps://host_address/file_path or
 *        http://host_address/file_path or
 *        https://host_address/file_path
 *
 *    (4) the field url_address or default_local_directory can include the
 *        follwing keywords replaced by date, time, station names and environment
 *        variables.
 *
 *        %Y -> yyyy    : year (4 digits) (2000-2099)
 *        %y -> yy      : year (2 digits) (00-99)
 *        %m -> mm      : month           (01-12)
 *        %d -> dd      : day of month    (01-31)
 *        %h -> hh      : hours           (00-23)
 *        %H -> a       : hour code       (a-x)
 *        %M -> mm      : minutes         (00-59)
 *        %n -> ddd     : day of year     (001-366)
 *        %W -> wwww    : GPS week        (0001-9999)
 *        %D -> d       : day of GPS week (0-6)
 *        %N -> nnn     : general number
 *        %s -> ssss    : station name    (lower-case)
 *        %S -> SSSS    : station name    (upper-case)
 *        %r -> rrrr    : station name
 *        %{env} -> env : environment variable
 *----------------------------------------------------------------------------*/
extern int dl_readurls(const char *file, const char **types, int ntype, url_t *urls, int nmax) {
  FILE *fp = fopen(file, "r");
  if (!fp) {
    fprintf(stderr, "options file read error %s\n", file);
    return 0;
  }
  int n = 0;
  for (int i = 0; i < ntype; i++) {
    rewind(fp);
    char buff[2048];
    while (fgets(buff, sizeof(buff), fp) && n < nmax) {
      char *p = strchr(buff, '#');
      if (p) *p = '\0';
      char type[32];
      p = parse_str(buff, type, sizeof(type));
      char path[FNSIZE];
      p = parse_str(p, path, sizeof(path));
      char dir[FNSIZE];
      parse_str(p, dir, sizeof(dir));
      if (!*type || !*path) continue;
      if (!cmp_str(type, types[i])) continue;
      rtkstrcpy(urls[n].type, sizeof(urls[0].type), type);
      rtkstrcpy(urls[n].path, sizeof(urls[0].path), path);
      rtkstrcpy(urls[n++].dir, sizeof(urls[0].dir), dir);
    }
  }
  fclose(fp);

  if (n <= 0) {
    fprintf(stderr, "no url in options file %s\n", file);
    return 0;
  }
  return n;
}
/* Read station list file ------------------------------------------------------
 * Read station list file
 * Args   : char   *file     I   station list file
 *          char   **stas    O   stations
 *          size_t size      I   stations buffer size
 *          int    nmax      I   max number of stations
 * Return : number of stations (0:error)
 * Notes  :
 *    (1) station list file contains station names separated by spaces.
 *    (2) strings after # in a line are treated as comments
 *----------------------------------------------------------------------------*/
extern int dl_readstas(const char *file, char **stas, size_t size, int nmax) {
  FILE *fp = fopen(file, "r");
  if (!fp) {
    fprintf(stderr, "station list file read error %s\n", file);
    return 0;
  }
  int n = 0;
  char buff[4096];
  while (fgets(buff, sizeof(buff), fp) && n < nmax) {
    char *p = strchr(buff, '#');
    if (p) *p = '\0';
    char *r;
    for (p = strtok_r(buff, " \r\n", &r); p && n < nmax; p = strtok_r(NULL, " \r\n", &r)) {
      rtkstrcpy(stas[n++], size, p);
    }
  }
  fclose(fp);

  if (n <= 0) {
    fprintf(stderr, "no station in station file %s\n", file);
    return 0;
  }
  return n;
}
/* Execute download ------------------------------------------------------------
 * Execute download
 * Args   : gtime_t ts,te    I   time start and end
 *          long double tint      I   time interval (s)
 *          int    seqnos    I   sequence number start
 *          int    seqnoe    I   sequence number end
 *          url_t  *urls     I   URL list
 *          int    nurl      I   number of URL list
 *          char   **stas    I   station list
 *          int    nsta      I   number of station list
 *          char   *dir      I   local directory
 *          char   *remote_p I   previous remote file path
 *          char   *usr      I   login user for FTP or FTPS
 *          char   *pwd      I   login password for FTP or FTPS
 *          char   *proxy    I   proxy server address
 *          int    opts      I   download options (or of the followings)
 *                                 DLOPT_FORCE = force download existing file
 *                                 DLOPT_KEEPCMP=keep compressed file
 *                                 DLOPT_HOLDERR=hold on error file
 *                                 DLOPT_HOLDLST=hold on listing file
 *          char   *msg      O   output messages
 *          FILE   *fp       IO  log file pointer (NULL: no output log)
 * Return : status (1:ok,0:error,-1:aborted)
 * Notes  : The URL list should be read by using dl_readurl()
 *          In the FTP or FTPS cases, the file name in a URL can contain wild-
 *          cards (*). The directory in a URL can not contain any wild-cards.
 *          If the file name contains wild-cards, dl_exec() gets a file-list in
 *          the remote directory and downloads the firstly matched file in the
 *          remote file-list. The secondary matched or the following files are
 *          not downloaded.
 *----------------------------------------------------------------------------*/
extern int dl_exec(gtime_t ts, gtime_t te, long double ti, int seqnos, int seqnoe,
                   const url_t *urls, int nurl, const char **stas, int nsta, const char *dir,
                   const char *usr, const char *pwd, const char *proxy, int opts, char *msg,
                   size_t msize, FILE *fp) {
  paths_t paths = {0};
  gtime_t ts_p = {0};
  int n[4] = {0};
  uint32_t tick = tickget();

  showmsg("STAT=_");

  /* Generate download paths  */
  while (timediff(ts, te) < 1E-3) {
    for (int i = 0; i < nurl; i++) {
      if (!gen_paths(ts, ts_p, seqnos, seqnoe, urls + i, stas, nsta, dir, &paths)) {
        free_path(&paths);
        rtksnprintf(msg, msize, "too many download files");
        return 0;
      }
    }
    ts_p = ts;
    ts = timeadd(ts, ti);
  }
  /* Compact download paths */
  compact_paths(&paths);

  if (paths.n <= 0) {
    rtksnprintf(msg, msize, "no download data");
    return 0;
  }
  for (int i = 0; i < paths.n; i++) {
    char str[2048];
    rtksnprintf(str, sizeof(str), "%s->%s (%d/%d)", paths.path[i].remot, paths.path[i].local, i + 1,
                paths.n);
    if (showmsg(str)) break;

    /* Execute download */
    char remot_p[FNSIZE] = "";
    if (exec_down(paths.path + i, remot_p, sizeof(remot_p), usr, pwd, proxy, opts, n, fp)) {
      break;
    }
  }
  if (!(opts & DLOPT_HOLDLST)) {
    remove(FTP_LISTING);
  }
  rtksnprintf(msg, msize, "OK=%d No_File=%d Skip=%d Error=%d (Time=%.1Lf s)", n[0], n[1], n[2],
              n[3], (tickget() - tick) * 0.001L);

  free_path(&paths);

  return 1;
}
/* Execute local file test -----------------------------------------------------
 * Execute local file test
 * Args   : gtime_t ts,te    I   time start and end
 *          long double tint      I   time interval (s)
 *          url_t  *urls     I   remote URL addresses
 *          int    nurl      I   number of remote URL addresses
 *          char   **stas    I   stations
 *          int    nsta      I   number of stations
 *          char   *dir      I   local directory
 *          int    ncol      I   number of column
 *          int    datefmt   I   date format (0:year-dow,1:year-dd/mm,2:week)
 *          FILE   *fp       IO  log test result file pointer
 * Return : status (1:ok,0:error,-1:aborted)
 *----------------------------------------------------------------------------*/
extern void dl_test(gtime_t ts, gtime_t te, long double ti, const url_t *urls, int nurl,
                    const char **stas, int nsta, const char *dir, int ncol, int datefmt, FILE *fp) {
  if (ncol < 1)
    ncol = 1;
  else if (ncol > 200)
    ncol = 200;

  char tstr[40];
  fprintf(fp, "** LOCAL DATA AVAILABILITY (%s, %s) **\n\n", time2str(timeget(), tstr, 0),
          *dir ? dir : "*");

  int n = 0;
  for (int i = 0; i < nurl; i++) {
    n += strstr(urls[i].path, "%s") || strstr(urls[i].path, "%S") ? nsta : 1;
  }
  int *nc = imat(n, 1);
  int *nt = imat(n, 1);
  for (int i = 0; i < n; i++) nc[i] = nt[i] = 0;

  int abort = 0;
  for (; timediff(ts, te) < 1E-3 && !abort; ts = timeadd(ts, ti * ncol)) {
    char year[32], date[32], date_p[32];
    genpath(datefmt == 0 ? "   %Y-" : "%Y/%m/", "", ts, 0, year, sizeof(year));
    if (datefmt <= 1)
      fprintf(fp, "%s %s", datefmt == 0 ? "DOY " : "DATE", year);
    else
      fprintf(fp, "WEEK          ");
    *date_p = '\0';
    int flag = 0;

    int m = datefmt == 2 ? 1 : 2;

    for (int i = 0; i < (ncol + m - 1) / m; i++) {
      gtime_t time = timeadd(ts, ti * i * m);
      if (timediff(time, te) >= 1E-3) break;

      if (datefmt <= 1) {
        genpath(datefmt == 0 ? "%n" : "%d", "", time, 0, date, sizeof(date));
        fprintf(fp, "%-4s", strcmp(date, date_p) ? date : "");
      } else {
        int week;
        if (fabsl(time2gpst(time, &week)) < 1.0L) {
          fprintf(fp, "%04d", week);
          flag = 1;
        } else {
          fprintf(fp, "%s", flag ? "" : "  ");
          flag = 0;
        }
      }
      rtkstrcpy(date_p, sizeof(date_p), date);
    }
    fprintf(fp, "\n");

    int j = 0;
    for (int i = 0; i < nurl && !abort; i++) {
      gtime_t time = timeadd(ts, ti * ncol - 1.0L);
      if (timediff(time, te) >= 0.0L) time = te;

      /* Test local files */
      abort = test_locals(ts, time, ti, urls + i, stas, nsta, dir, nc + j, nt + j, fp);

      j += strstr(urls[i].path, "%s") || strstr(urls[i].path, "%S") ? nsta : 1;
    }
    fprintf(fp, "\n");
  }
  fprintf(fp, "# COUNT     : FILES/TOTAL\n");

  int j = 0;
  for (int i = 0; i < nurl; i++) {
    j += print_total(urls + i, stas, nsta, nc + j, nt + j, fp);
  }
  free(nc);
  free(nt);
}
