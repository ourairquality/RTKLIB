/*------------------------------------------------------------------------------
 * lambda.c : integer ambiguity resolution
 *
 *          Copyright (C) 2007-2008 by T.TAKASU, All rights reserved.
 *
 * reference :
 *     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
 *         a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
 *         1995
 *     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
 *         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
 *
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2007/01/13 1.0 new
 *           2015/05/31 1.1 add api lambda_reduction(), lambda_search()
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* constants/macros ----------------------------------------------------------*/

#define LOOPMAX 10000 /* maximum count of search loop */

#define SGN(x) ((x) <= 0.0L ? -1.0L : 1.0L)
#define ROUND(x) (floorl((x) + 0.5L))
#define SWAP(x, y)    \
  do {                \
    long double tmp_; \
    tmp_ = x;         \
    x = y;            \
    y = tmp_;         \
  } while (0)

/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static int LD(int n, const long double *Q, long double *L, long double *D) {
  int info = 0;
  long double *A = mat(n, n);

  memcpy(A, Q, sizeof(long double) * n * n);
  for (int i = n - 1; i >= 0; i--) {
    if ((D[i] = A[i + i * n]) <= 0.0L) {
      info = -1;
      break;
    }
    long double a = sqrtl(D[i]);
    for (int j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;
    for (int j = 0; j <= i - 1; j++)
      for (int k = 0; k <= j; k++) A[j + k * n] -= L[i + k * n] * L[i + j * n];
    for (int j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];
  }
  free(A);
  if (info) fprintf(stderr, "%s : LD factorization error\n", __FILE__);
  return info;
}
/* integer gauss transformation ----------------------------------------------*/
static void gauss(int n, long double *L, long double *Z, int i, int j) {
  int mu = (int)ROUND(L[i + j * n]);

  if (mu != 0) {
    for (int k = i; k < n; k++) L[k + n * j] -= (long double)mu * L[k + i * n];
    for (int k = 0; k < n; k++) Z[k + n * j] -= (long double)mu * Z[k + i * n];
  }
}
/* permutations --------------------------------------------------------------*/
static void perm(int n, long double *L, long double *D, int j, long double del, long double *Z) {
  long double eta = D[j] / del;
  long double lam = D[j + 1] * L[j + 1 + j * n] / del;
  D[j] = eta * D[j + 1];
  D[j + 1] = del;
  for (int k = 0; k <= j - 1; k++) {
    long double a0 = L[j + k * n];
    long double a1 = L[j + 1 + k * n];
    L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
    L[j + 1 + k * n] = eta * a0 + lam * a1;
  }
  L[j + 1 + j * n] = lam;
  for (int k = j + 2; k < n; k++) SWAP(L[k + j * n], L[k + (j + 1) * n]);
  for (int k = 0; k < n; k++) SWAP(Z[k + j * n], Z[k + (j + 1) * n]);
}
/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
static void reduction(int n, long double *L, long double *D, long double *Z) {
  int j = n - 2, k = n - 2;
  while (j >= 0) {
    if (j <= k)
      for (int i = j + 1; i < n; i++) gauss(n, L, Z, i, j);
    long double del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
    if (del + 1E-6L < D[j + 1]) { /* compared considering numerical error */
      perm(n, L, D, j, del, Z);
      k = j;
      j = n - 2;
    } else
      j--;
  }
}
/* modified lambda (mlambda) search (ref. [2]) -------------------------------
* args   : n      I  number of float parameters
*          m      I  number of fixed solution
           L,D    I  transformed covariance matrix
           zs     I  transformed double-diff phase biases
           zn     O  fixed solutions
           s      O  sum of residuals for fixed solutions                    */
static int search(int n, int m, const long double *L, const long double *D, const long double *zs,
                  long double *zn, long double *s) {
  long double maxdist = 1E99L;
  long double *S = zeros(n, n), *dist = mat(n, 1), *zb = mat(n, 1), *z = mat(n, 1),
              *step = mat(n, 1);

  int k = n - 1;
  dist[k] = 0.0L;
  zb[k] = zs[k];
  z[k] = ROUND(zb[k]);
  long double y = zb[k] - z[k];
  step[k] = SGN(y); /* step towards closest integer */
  int nn = 0, imax = 0, c;
  for (c = 0; c < LOOPMAX; c++) {
    long double newdist;
    newdist = dist[k] + y * y / D[k]; /* newdist=sum(((z(j)-zb(j))^2/d(j))) */
    if (newdist < maxdist) {
      /* Case 1: move down */
      if (k != 0) {
        dist[--k] = newdist;
        for (int i = 0; i <= k; i++)
          S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
        zb[k] = zs[k] + S[k + k * n];
        z[k] = ROUND(zb[k]); /* next valid integer */
        y = zb[k] - z[k];
        step[k] = SGN(y);
      }
      /* Case 2: store the found candidate and try next valid integer */
      else {
        if (nn < m) { /* store the first m initial points */
          if (nn == 0 || newdist > s[imax]) imax = nn;
          for (int i = 0; i < n; i++) zn[i + nn * n] = z[i];
          s[nn++] = newdist;
        } else {
          if (newdist < s[imax]) {
            for (int i = 0; i < n; i++) zn[i + imax * n] = z[i];
            s[imax] = newdist;
            for (int i = imax = 0; i < m; i++)
              if (s[imax] < s[i]) imax = i;
          }
          maxdist = s[imax];
        }
        z[0] += step[0]; /* next valid integer */
        y = zb[0] - z[0];
        step[0] = -step[0] - SGN(step[0]);
      }
    }
    /* Case 3: exit or move up */
    else {
      if (k == n - 1)
        break;
      else {
        k++;             /* move up */
        z[k] += step[k]; /* next valid integer */
        y = zb[k] - z[k];
        step[k] = -step[k] - SGN(step[k]);
      }
    }
  }
  for (int i = 0; i < m - 1; i++) { /* sort by s */
    for (int j = i + 1; j < m; j++) {
      if (s[i] < s[j]) continue;
      SWAP(s[i], s[j]);
      for (int k2 = 0; k2 < n; k2++) SWAP(zn[k2 + i * n], zn[k2 + j * n]);
    }
  }
  free(S);
  free(dist);
  free(zb);
  free(z);
  free(step);

  if (c >= LOOPMAX) {
    fprintf(stderr, "%s : search loop count overflow\n", __FILE__);
    return -2;
  }
  return 0;
}
/* lambda/mlambda integer least-square estimation ------------------------------
 * integer least-square estimation. reduction is performed by lambda (ref.[1]),
 * and search by mlambda (ref.[2]).
 * args   : int    n      I  number of float parameters
 *          int    m      I  number of fixed solutions
 *          long double *a     I  float parameters (n x 1) (double-diff phase biases)
 *          long double *Q     I  covariance matrix of float parameters (n x n)
 *          long double *F     O  fixed solutions (n x m)
 *          long double *s     O  sum of squared residulas of fixed solutions (1 x m)
 * return : status (0:ok,other:error)
 * notes  : matrix stored by column-major order (fortran convension)
 *-----------------------------------------------------------------------------*/
extern int lambda(int n, int m, const long double *a, const long double *Q, long double *F,
                  long double *s) {
  if (n <= 0 || m <= 0) return -1;

  long double *L = zeros(n, n), *D = mat(n, 1), *Z = eye(n), *z = mat(n, 1), *E = mat(n, m);

  /* LD (lower diagonal) factorization (Q=L'*diag(D)*L) */
  int info = LD(n, Q, L, D);
  if (!info) {
    /* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) */
    reduction(n, L, D, Z);
    matmul("TN", n, 1, n, Z, a, z); /* z=Z'*a */

    /* mlambda search
        z = transformed long double-diff phase biases
        L,D = transformed covariance matrix */
    info = search(n, m, L, D, z, E, s);
    if (!info) {                        /* returns 0 if no error */
      info = solve("T", Z, E, n, m, F); /* F=Z'\E */
    }
  }
  free(L);
  free(D);
  free(Z);
  free(z);
  free(E);
  return info;
}
/* lambda reduction ------------------------------------------------------------
 * reduction by lambda (ref [1]) for integer least square
 * args   : int    n      I  number of float parameters
 *          long double *Q     I  covariance matrix of float parameters (n x n)
 *          long double *Z     O  lambda reduction matrix (n x n)
 * return : status (0:ok,other:error)
 *-----------------------------------------------------------------------------*/
extern int lambda_reduction(int n, const long double *Q, long double *Z) {
  if (n <= 0) return -1;

  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      Z[i + j * n] = i == j ? 1.0L : 0.0L;
    }
  /* LD factorization */
  long double *L = zeros(n, n), *D = mat(n, 1);
  int info = LD(n, Q, L, D);
  if (info) {
    free(L);
    free(D);
    return info;
  }
  /* lambda reduction */
  reduction(n, L, D, Z);

  free(L);
  free(D);
  return 0;
}
/* mlambda search --------------------------------------------------------------
 * search by  mlambda (ref [2]) for integer least square
 * args   : int    n      I  number of float parameters
 *          int    m      I  number of fixed solutions
 *          long double *a     I  float parameters (n x 1)
 *          long double *Q     I  covariance matrix of float parameters (n x n)
 *          long double *F     O  fixed solutions (n x m)
 *          long double *s     O  sum of squared residulas of fixed solutions (1 x m)
 * return : status (0:ok,other:error)
 *-----------------------------------------------------------------------------*/
extern int lambda_search(int n, int m, const long double *a, const long double *Q, long double *F,
                         long double *s) {
  if (n <= 0 || m <= 0) return -1;

  /* LD factorization */
  long double *L = zeros(n, n), *D = mat(n, 1);
  int info = LD(n, Q, L, D);
  if (info) {
    free(L);
    free(D);
    return info;
  }
  /* mlambda search */
  info = search(n, m, L, D, a, F, s);

  free(L);
  free(D);
  return info;
}
