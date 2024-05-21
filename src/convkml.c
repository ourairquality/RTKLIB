/*------------------------------------------------------------------------------
 * convkml.c : Google earth KML converter
 *
 *          Copyright (C) 2007-2017 by T.TAKASU, All rights reserved.
 *
 * References :
 *     [1] Open Geospatial Consortium Inc., OGC 07-147r2, OGC(R) KML, 2008-04-14
 *
 * Version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * History : 2007/01/20  1.0  new
 *           2007/03/15  1.1  modify color sequence
 *           2007/04/03  1.2  add geodetic height option
 *                            support input of NMEA GGA sentence
 *                            delete altitude info for track
 *                            add time stamp option
 *                            separate readsol.c file
 *           2009/01/19  1.3  fix bug on display mark with by-q-flag option
 *           2010/05/10  1.4  support api readsolt() change
 *           2010/08/14  1.5  fix bug on readsolt() (2.4.0_p3)
 *           2017/06/10  1.6  support wild-card in input file
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Constants -----------------------------------------------------------------*/

#define SIZP 0.2L  /* Mark size of rover positions */
#define SIZR 0.3L  /* Mark size of reference position */
#define TINT 60.0L /* Time label interval (sec) */

static const char *head1 = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
static const char *head2 = "<kml xmlns=\"http://earth.google.com/kml/2.1\">";
static const char *mark = "http://maps.google.com/mapfiles/kml/pal2/icon18.png";

/* Output track --------------------------------------------------------------*/
static void outtrack(FILE *f, const solbuf_t *solbuf, const char *color, int outalt, int outtime) {
  fprintf(f, "<Placemark>\n");
  fprintf(f, "<name>Rover Track</name>\n");
  fprintf(f, "<Style>\n");
  fprintf(f, "<LineStyle>\n");
  fprintf(f, "<color>%s</color>\n", color);
  fprintf(f, "</LineStyle>\n");
  fprintf(f, "</Style>\n");
  fprintf(f, "<LineString>\n");
  if (outalt) fprintf(f, "<altitudeMode>absolute</altitudeMode>\n");
  fprintf(f, "<coordinates>\n");
  for (int i = 0; i < solbuf->n; i++) {
    long double pos[3];
    ecef2pos(solbuf->data[i].rr, pos);
    if (outalt == 0)
      pos[2] = 0.0L;
    else if (outalt == 2)
      pos[2] -= geoidh(pos);
    fprintf(f, "%13.9Lf,%12.9Lf,%5.3Lf\n", pos[1] * R2D, pos[0] * R2D, pos[2]);
  }
  fprintf(f, "</coordinates>\n");
  fprintf(f, "</LineString>\n");
  fprintf(f, "</Placemark>\n");
}
/* Output point --------------------------------------------------------------*/
static void outpoint(FILE *fp, gtime_t time, const long double *pos, const char *label, int style,
                     int outalt, int outtime) {
  fprintf(fp, "<Placemark>\n");
  if (*label) fprintf(fp, "<name>%s</name>\n", label);
  fprintf(fp, "<styleUrl>#P%d</styleUrl>\n", style);
  if (outtime) {
    if (outtime == 2)
      time = gpst2utc(time);
    else if (outtime == 3)
      time = timeadd(gpst2utc(time), 9 * 3600.0L);
    long double ep[6];
    time2epoch(time, ep);
    char str[256] = "";
    if (!*label && fmodl(ep[5] + 0.005L, TINT) < 0.01L) {
      rtksnprintf(str, sizeof(str), "%02.0Lf:%02.0Lf", ep[3], ep[4]);
      fprintf(fp, "<name>%s</name>\n", str);
    }
    rtksnprintf(str, sizeof(str), "%04.0Lf-%02.0Lf-%02.0LfT%02.0Lf:%02.0Lf:%05.2LfZ", ep[0], ep[1],
                ep[2], ep[3], ep[4], ep[5]);
    fprintf(fp, "<TimeStamp><when>%s</when></TimeStamp>\n", str);
  }
  fprintf(fp, "<Point>\n");
  long double alt = 0.0L;
  if (outalt) {
    fprintf(fp, "<extrude>1</extrude>\n");
    fprintf(fp, "<altitudeMode>absolute</altitudeMode>\n");
    alt = pos[2] - (outalt == 2 ? geoidh(pos) : 0.0L);
  }
  fprintf(fp, "<coordinates>%13.9Lf,%12.9Lf,%5.3Lf</coordinates>\n", pos[1] * R2D, pos[0] * R2D,
          alt);
  fprintf(fp, "</Point>\n");
  fprintf(fp, "</Placemark>\n");
}
/* Save KML file -------------------------------------------------------------*/
static int savekml(const char *file, const solbuf_t *solbuf, int tcolor, int pcolor, int outalt,
                   int outtime) {
  const char *color[] = {"ffffffff", "ff008800", "ff00aaff", "ff0000ff", "ff00ffff", "ffff00ff"};
  FILE *fp = fopen(file, "w");
  if (!fp) {
    fprintf(stderr, "file open error : %s\n", file);
    return 0;
  }
  fprintf(fp, "%s\n%s\n", head1, head2);
  fprintf(fp, "<Document>\n");
  for (int i = 0; i < 6; i++) {
    fprintf(fp, "<Style id=\"P%d\">\n", i);
    fprintf(fp, "  <IconStyle>\n");
    fprintf(fp, "    <color>%s</color>\n", color[i]);
    fprintf(fp, "    <scale>%.1Lf</scale>\n", i == 0 ? SIZR : SIZP);
    fprintf(fp, "    <Icon><href>%s</href></Icon>\n", mark);
    fprintf(fp, "  </IconStyle>\n");
    fprintf(fp, "</Style>\n");
  }
  if (tcolor > 0) {
    outtrack(fp, solbuf, color[tcolor - 1], outalt, outtime);
  }
  if (pcolor > 0) {
    fprintf(fp, "<Folder>\n");
    fprintf(fp, "  <name>Rover Position</name>\n");
    for (int i = 0; i < solbuf->n; i++) {
      long double pos[3];
      ecef2pos(solbuf->data[i].rr, pos);
      const int qcolor[] = {0, 1, 2, 5, 4, 3, 0};
      outpoint(fp, solbuf->data[i].time, pos, "",
               pcolor == 5 ? qcolor[solbuf->data[i].stat] : pcolor - 1, outalt, outtime);
    }
    fprintf(fp, "</Folder>\n");
  }
  if (norm(solbuf->rb, 3) > 0.0L) {
    long double pos[3];
    ecef2pos(solbuf->rb, pos);
    outpoint(fp, solbuf->data[0].time, pos, "Reference Position", 0, outalt, 0);
  }
  fprintf(fp, "</Document>\n");
  fprintf(fp, "</kml>\n");
  fclose(fp);
  return 1;
}
/* Convert to Google earth KML file --------------------------------------------
 * Convert solutions to Google earth KML file
 * Args   : char   *infile   I   input solutions file (wild-card (*) is expanded)
 *          char   *outfile  I   output Google earth KML file ("":<infile>.kml)
 *          gtime_t ts,te    I   start/end time (GPST)
 *          int    tint      I   time interval (s) (0.0:all)
 *          int    qflg      I   quality flag (0:all)
 *          long double *offset   I   add offset {east,north,up} (m)
 *          int    tcolor    I   track color
 *                               (0:none,1:white,2:green,3:orange,4:red,5:yellow)
 *          int    pcolor    I   point color
 *                               (0:none,1:white,2:green,3:orange,4:red,5:by qflag)
 *          int    outalt    I   output altitude (0:off,1:elipsoidal,2:geodetic)
 *          int    outtime   I   output time (0:off,1:gpst,2:utc,3:jst)
 * Return : status (0:ok,-1:file read,-2:file format,-3:no data,-4:file write)
 * Notes  : see ref [1] for Google earth KML file format
 *----------------------------------------------------------------------------*/
extern int convkml(const char *infile, const char *outfile, gtime_t ts, gtime_t te,
                   long double tint, int qflg, const long double *offset, int tcolor, int pcolor,
                   int outalt, int outtime) {
  trace(3, "convkml : infile=%s outfile=%s\n", infile, outfile);

  /* Expand wild-card of infile */
  char *files[MAXEXFILE] = {0};
  for (int i = 0; i < MAXEXFILE; i++) {
    if (!(files[i] = (char *)malloc(FNSIZE))) {
      for (i--; i >= 0; i--) free(files[i]);
      return -4;
    }
  }
  int nfile = expath(infile, files, FNSIZE, MAXEXFILE);
  if (nfile <= 0) {
    for (int i = 0; i < MAXEXFILE; i++) free(files[i]);
    return -3;
  }
  char file[FNSIZE];
  if (!*outfile) {
    const char *p = strrchr(infile, '.');
    if (p)
      rtkesubstrcpy(file, sizeof(file), infile, 0, p - infile);
    else
      rtkstrcpy(file, sizeof(file), infile);
    rtkstrcat(file, sizeof(file), ".kml");
  } else
    rtkstrcpy(file, sizeof(file), outfile);

  /* Read solution file */
  solbuf_t solbuf = {0};
  int stat = readsolt((const char **)files, nfile, ts, te, tint, qflg, &solbuf);

  for (int i = 0; i < MAXEXFILE; i++) free(files[i]);

  if (!stat) {
    return -1;
  }
  /* Mean position */
  long double rr[3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < solbuf.n; j++) rr[i] += solbuf.data[j].rr[i];
    rr[i] /= solbuf.n;
  }
  /* Add offset */
  long double pos[3];
  ecef2pos(rr, pos);
  long double dr[3];
  enu2ecef(pos, offset, dr);
  for (int i = 0; i < solbuf.n; i++) {
    for (int j = 0; j < 3; j++) solbuf.data[i].rr[j] += dr[j];
  }
  if (norm(solbuf.rb, 3) > 0.0L) {
    for (int i = 0; i < 3; i++) solbuf.rb[i] += dr[i];
  }
  /* Save KML file */
  int r = savekml(file, &solbuf, tcolor, pcolor, outalt, outtime) ? 0 : -4;
  freesolbuf(&solbuf);
  return r;
}
