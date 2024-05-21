/*------------------------------------------------------------------------------
 * vt.h : header file for virtual console
 *
 *          Copyright (C) 2015 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision:$ $Date:$
 * History : 2015/01/11 1.0  separated from rtkrcv.c
 *----------------------------------------------------------------------------*/
#ifndef VT_H
#define VT_H
#include <termios.h>

#include "rtklib.h"

#define MAXBUFF 4096 /* Size of line buffer */
#define MAXHIST 256  /* Size of history buffer */

/* Type definitions ----------------------------------------------------------*/
typedef struct vt_tag { /* Virtual console type */
  int state;            /* state(0:close,1:open) */
  int type;             /* Type (0:dev,1:telnet) */
  int in, out;          /* Input/output file descriptor */
  int n, nesc;          /* Number of line buffer/escape */
  int cur;              /* Cursor position */
  int cur_h;            /* Current history */
  int brk;              /* Break status */
  int blind;            /* Blind inpu mode */
  struct termios tio;   /* Original terminal attribute */
  char buff[MAXBUFF];   /* Line buffer */
  char esc[8];          /* Escape buffer */
  char *hist[MAXHIST];  /* History buffer */
  FILE *logfp;          /* Log file pointer */
} vt_t;

/* Function prototypes -------------------------------------------------------*/
extern vt_t *vt_open(int sock, const char *dev);
extern void vt_close(vt_t *vt);
extern int vt_getc(vt_t *vt, char *c);
extern int vt_gets(vt_t *vt, char *buff, int n);
extern int vt_putc(vt_t *vt, char c);
extern int vt_puts(vt_t *vt, const char *buff);
extern int vt_printf(vt_t *vt, const char *format, ...);
extern int vt_chkbrk(vt_t *vt);
extern int vt_openlog(vt_t *vt, const char *file);
extern void vt_closelog(vt_t *vt);

#endif /* VT_H */
