/*------------------------------------------------------------------------------
 * ppp_ar.c : ppp ambiguity resolution
 *
 * Reference :
 *    [1] H.Okumura, C-gengo niyoru saishin algorithm jiten (in Japanese),
 *        Software Technology, 1991
 *
 *          Copyright (C) 2012-2015 by T.TAKASU, All rights reserved.
 *
 * Version : $Revision:$ $Date:$
 * History : 2013/03/11  1.0  new
 *           2016/05/10  1.1  delete codes
 *----------------------------------------------------------------------------*/
#include "rtklib.h"

/* Ambiguity resolution in ppp -----------------------------------------------*/
extern int ppp_ar(rtk_t *rtk, const obsd_t *obs, int n, int *exc, const nav_t *nav,
                  const long double *azel, long double *x, long double *P) {
  return 0;
}
