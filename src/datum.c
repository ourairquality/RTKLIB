/*------------------------------------------------------------------------------
 * datum.c : datum transformation
 *
 *          Copyright (C) 2007 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * History : 2007/02/08 1.0 new
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

#define MAXPRM 400000 /* Max number of parameter records */

typedef struct { /* Datum trans parameter type */
  int code;      /* Mesh code */
  float db, dl;  /* Difference of latitude/longitude (sec) */
} tprm_t;

static tprm_t *prm = NULL; /* Datum trans parameter table */
static int n = 0;          /* Datum trans parameter table size */

/* Compare datum trans parameters --------------------------------------------*/
static int cmpprm(const void *p1, const void *p2) {
  const tprm_t *q1 = (tprm_t *)p1, *q2 = (tprm_t *)p2;
  return q1->code - q2->code;
}
/* Search datum trans parameter ----------------------------------------------*/
static int searchprm(double lat, double lon) {
  lon -= 6000.0;
  int n1 = (int)(lat / 40.0);
  lat -= n1 * 40.0;
  int m1 = (int)(lon / 60.0);
  lon -= m1 * 60.0;
  int n2 = (int)(lat / 5.0);
  lat -= n2 * 5.0;
  int m2 = (int)(lon / 7.5);
  lon -= m2 * 7.5;
  int code =
      n1 * 1000000 + m1 * 10000 + n2 * 1000 + m2 * 100 + (int)(lat / 0.5) * 10 + (int)(lon / 0.75);

  for (int i = 0, j = n - 1; i < j;) { /* Binary search */
    int k = (i + j) / 2;
    if (prm[k].code == code) return k;
    if (prm[k].code < code)
      i = k + 1;
    else
      j = k;
  }
  return -1;
}
/* Tokyo datum to jgd2000 lat/lon corrections --------------------------------*/
static int dlatdlon(const double *post, double *dpos) {
  if (n == 0) return -1;

  double lat = post[0] * R2D * 60.0, lon = post[1] * R2D * 60.0; /* Arcmin */
  double dlat = 0.5, dlon = 0.75, db[2][2], dl[2][2];
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++) {
      int k = searchprm(lat + i * dlat, lon + j * dlon);
      if (k < 0) return -1;
      db[i][j] = prm[k].db;
      dl[i][j] = prm[k].dl;
    }
  double a = lat / dlat - (int)(lat / dlat), c = 1.0 - a;
  double b = lon / dlon - (int)(lon / dlon), d = 1.0 - b;
  dpos[0] =
      (db[0][0] * c * d + db[1][0] * a * d + db[0][1] * c * b + db[1][1] * a * b) * D2R / 3600.0;
  dpos[1] =
      (dl[0][0] * c * d + dl[1][0] * a * d + dl[0][1] * c * b + dl[1][1] * a * b) * D2R / 3600.0;
  return 0;
}
/* Load datum transformation parameter -----------------------------------------
 * Load datum transformation parameter
 * Args   : char  *file      I   datum trans parameter file path
 * Return : status (0:ok,0>:error)
 * Notes  : parameters file shall comply with GSI TKY2JGD.par
 *----------------------------------------------------------------------------*/
extern int loaddatump(const char *file) {
  if (n > 0) return 0; /* Already loaded */

  FILE *fp = fopen(file, "r");
  if (!fp) {
    fprintf(stderr, "%s : datum prm file open error : %s\n", __FILE__, file);
    return -1;
  }
  prm = (tprm_t *)malloc(sizeof(tprm_t) * MAXPRM);
  if (!prm) {
    fclose(fp);
    fprintf(stderr, "%s : memory allocation error\n", __FILE__);
    return -1;
  }
  char buff[256];
  while (fgets(buff, sizeof(buff), fp) && n < MAXPRM) {
    if (sscanf(buff, "%d %f %f", &prm[n].code, &prm[n].db, &prm[n].dl) >= 3) n++;
  }
  fclose(fp);
  qsort(prm, n, sizeof(tprm_t), cmpprm); /* Sort parameter table */
  return 0;
}
/* Tokyo datum to JGD2000 datum ------------------------------------------------
 * Transform position in Tokyo datum to JGD2000 datum
 * Args   : double *pos      I   position in Tokyo datum   {lat,lon,h} (rad,m)
 *                           O   position in JGD2000 datum {lat,lon,h} (rad,m)
 * Return : status (0:ok,0>:error,out of range)
 * Notes  : before calling, call loaddatump() to set parameter table
 *----------------------------------------------------------------------------*/
extern int tokyo2jgd(double *pos) {
  double post[2];
  post[0] = pos[0];
  post[1] = pos[1];
  double dpos[2];
  if (dlatdlon(post, dpos)) return -1;
  pos[0] = post[0] + dpos[0];
  pos[1] = post[1] + dpos[1];
  return 0;
}
/* JGD2000 datum to Tokyo datum ------------------------------------------------
 * Transform position in JGD2000 datum to Tokyo datum
 * Args   : double *pos      I   position in JGD2000 datum {lat,lon,h} (rad,m)
 *                           O   position in Tokyo datum   {lat,lon,h} (rad,m)
 * Return : status (0:ok,0>:error,out of range)
 * Notes  : before calling, call loaddatump() to set parameter table
 *----------------------------------------------------------------------------*/
extern int jgd2tokyo(double *pos) {
  double posj[2];
  posj[0] = pos[0];
  posj[1] = pos[1];
  for (int i = 0; i < 2; i++) {
    double dpos[2];
    if (dlatdlon(pos, dpos)) return -1;
    pos[0] = posj[0] - dpos[0];
    pos[1] = posj[1] - dpos[1];
  }
  return 0;
}
