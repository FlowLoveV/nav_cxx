#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "algorithm/ambiguity_fixer.hpp"

using navp::f64;
using navp::i32;

#define LOOPMAX 5000 /* maximum count of search loop */
#define SGN(x) ((x) <= 0.0 ? -1.0 : 1.0)
#define ROUND(x) (floor((x) + 0.5))
#define SWAP(x, y) \
  do {             \
    f64 tmp_;      \
    tmp_ = x;      \
    x = y;         \
    y = tmp_;      \
  } while (0)

f64 *mat(i32 n, i32 m);

i32 matinv(f64 *A, i32 n);

/* new matrix -------------------------------------------------------------*/
extern f64 *mat(i32 n, i32 m) {
  f64 *p;

  if (n <= 0 || m <= 0) return NULL;
  if (!(p = (f64 *)malloc(sizeof(f64) * n * m))) {
    printf("matrix memory allocation error: n=%d,m=%d\n", n, m);
  }
  return p;
}

/* new integer matrix ------------------------------------------------------*/
i32 *imat(i32 n, i32 m) {
  i32 *p;

  if (n <= 0 || m <= 0) return NULL;
  if (!(p = (i32 *)malloc(sizeof(i32) * n * m))) {
    printf("integer matrix memory allocation error: n=%d,m=%d\n", n, m);
  }
  return p;
}

/* zero matrix -------------------------------------------------------------*/
f64 *zeros(i32 n, i32 m) {
  f64 *p;

  if (n <= 0 || m <= 0) return NULL;
  if (!(p = (f64 *)calloc(sizeof(f64), n * m))) {
    printf("matrix memory allocation error: n=%d,m=%d\n", n, m);
  }

  return p;
}

/* identity matrix --------------------------------------------------------*/
f64 *lambda_eye(i32 n) {
  f64 *p;
  i32 i;

  if ((p = zeros(n, n)))
    for (i = 0; i < n; i++) p[i + i * n] = 1.0;
  return p;
}

/* multiply matrix -----------------------------------------------------------*/
void matmul(const char *tr, i32 n, i32 k, i32 m, f64 alpha, const f64 *A, const f64 *B, f64 beta, f64 *C) {
  f64 d;
  i32 i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

  for (i = 0; i < n; i++)
    for (j = 0; j < k; j++) {
      d = 0.0;
      switch (f) {
        case 1:
          for (x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m];
          break;
        case 2:
          for (x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k];
          break;
        case 3:
          for (x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m];
          break;
        case 4:
          for (x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k];
          break;
      }
      if (beta == 0.0)
        C[i + j * n] = alpha * d;
      else
        C[i + j * n] = alpha * d + beta * C[i + j * n];
    }
}

/* copy matrix --------------------------------------------------------------*/
void matcpy(f64 *A, const f64 *B, i32 n, i32 m) { memcpy(A, B, sizeof(f64) * n * m); }

/* solve linear equation -----------------------------------------------------*/
i32 solve(const char *tr, const f64 *A, const f64 *Y, i32 n, i32 m, f64 *X) {
  f64 *B = mat(n, n);
  i32 info;

  matcpy(B, A, n, n);
  if (!(info = matinv(B, n))) matmul(tr[0] == 'N' ? "NN" : "TN", n, m, n, 1.0, B, Y, 0.0, X);  // X=B'*Y=E/Z'
  free(B);
  return info;
}

/* LU decomposition ----------------------------------------------------------*/
static i32 ludcmp(f64 *A, i32 n, i32 *indx, f64 *d) {
  f64 big, s, tmp, *vv = mat(n, 1);
  i32 i, imax = 0, j, k;

  *d = 1.0;
  for (i = 0; i < n; i++) {
    big = 0.0;
    for (j = 0; j < n; j++)
      if ((tmp = fabs(A[i + j * n])) > big) big = tmp;
    if (big > 0.0)
      vv[i] = 1.0 / big;
    else {
      free(vv);
      return -1;
    }
  }
  for (j = 0; j < n; j++) {
    for (i = 0; i < j; i++) {
      s = A[i + j * n];
      for (k = 0; k < i; k++) s -= A[i + k * n] * A[k + j * n];
      A[i + j * n] = s;
    }
    big = 0.0;
    for (i = j; i < n; i++) {
      s = A[i + j * n];
      for (k = 0; k < j; k++) s -= A[i + k * n] * A[k + j * n];
      A[i + j * n] = s;
      if ((tmp = vv[i] * fabs(s)) >= big) {
        big = tmp;
        imax = i;
      }
    }
    if (j != imax) {
      for (k = 0; k < n; k++) {
        tmp = A[imax + k * n];
        A[imax + k * n] = A[j + k * n];
        A[j + k * n] = tmp;
      }
      *d = -(*d);
      vv[imax] = vv[j];
    }
    indx[j] = imax;
    if (A[j + j * n] == 0.0) {
      free(vv);
      return -1;
    }
    if (j != n - 1) {
      tmp = 1.0 / A[j + j * n];
      for (i = j + 1; i < n; i++) A[i + j * n] *= tmp;
    }
  }
  free(vv);
  return 0;
}
/* LU back-substitution ------------------------------------------------------*/
static void lubksb(const f64 *A, i32 n, const i32 *indx, f64 *b) {
  f64 s;
  i32 i, ii = -1, ip, j;

  for (i = 0; i < n; i++) {
    ip = indx[i];
    s = b[ip];
    b[ip] = b[i];
    if (ii >= 0)
      for (j = ii; j < i; j++) s -= A[i + j * n] * b[j];
    else if (s)
      ii = i;
    b[i] = s;
  }
  for (i = n - 1; i >= 0; i--) {
    s = b[i];
    for (j = i + 1; j < n; j++) s -= A[i + j * n] * b[j];
    b[i] = s / A[i + i * n];
  }
}

/* inverse of matrix ---------------------------------------------------------*/
i32 matinv(f64 *A, i32 n) {
  f64 d, *B;
  i32 i, j, *indx;

  indx = imat(n, 1);
  B = mat(n, n);
  matcpy(B, A, n, n);
  if (ludcmp(B, n, indx, &d)) {
    free(indx);
    free(B);
    return -1;
  }
  for (j = 0; j < n; j++) {
    for (i = 0; i < n; i++) A[i + j * n] = 0.0;
    A[j + j * n] = 1.0;
    lubksb(B, n, indx, A + j * n);
  }
  free(indx);
  free(B);
  return 0;
}

/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static i32 LD(i32 n, const f64 *Q, f64 *L, f64 *D)  // ´Ë´¦·Ö½âËã·¨Í¬ FMFAC5£¨²Î¼û"Ä£ºý¶È¾­µä°æ.pdf"£©
{
  i32 i, j, k, info = 0;
  f64 a, *A = mat(n, n);  // mat():¸ø¾ØÕó·ÖÅäÄÚ´æ

  memcpy(A, Q, sizeof(f64) * n * n);  // ½«QÕó¸´ÖÆµ½AÖÐ
  for (i = n - 1; i >= 0;
       i--) {  // FMFAC5ÖÐÊÇ´Ói=n¿ªÊ¼µÄ£¬ÎªºÎÓÐÇø±ð£¿ÓÉÓÚ¾ØÕóÔªËØÊÇ´Ó0µ½n*n-1£¬¶øFMFAC5ÖÐÊÇ1µ½n*n£¬¹Ê´Ë´¦ÐèÒª¶à¼õ1
    if ((D[i] = A[i + i * n]) <= 0.0) {
      info = -1;
      break;
    }  // D(i,i)=Q(i,i),×¢Òâ´Ë´¦DÊÇÒ»¸öÁÐÏòÁ¿¶ø·Ç¾ØÕó
    a = sqrt(D[i]);                                            // a=sqrt(Q(i,i))
    for (j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;  // L(i,1:i)=Q(i,1:i)/sqrt(Q(i,i))
    for (j = 0; j <= i - 1; j++)
      for (k = 0; k <= j; k++)
        A[j + k * n] -= L[i + k * n] * L[i + j * n];  // Q(j,1:j)=Q(j,1:j)-L(i,1:j)L(i,j) Í¬ÑùµÄÓÉÓÚ¾ØÕóÔ­Òò£¬j´Ó0¿ªÊ¼
    for (j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];  // L(i,1:i)=L(i,1:i)/L(i,i)
  }
  free(A);
  if (info) printf("LD factorization error\n");
  return info;
}

/* LDLT factorization  ( Q=L*diag(D)*L' ) ------2017.5.9----------------------------------------- */
static i32 LDLT(i32 n, const f64 *Q, f64 *L, f64 *D) {
  i32 i, j, k, info = 0;
  f64 *A = mat(n, n);

  memcpy(A, Q, sizeof(f64) * n * n);
  for (i = 0; i < n; i++) L[i + i * n] = 1;
  for (j = 0; j < n - 1; j++) {
    if ((D[j] = A[j + j * n]) <= 0.0) {
      info = -1;
      break;
    }
    for (i = 1; i < n; i++) L[i + j * n] = A[i + j * n] / D[j];
    for (k = j + 1; k < n; k++)
      for (i = k; i < n; i++) A[i + k * n] = A[i + k * n] - A[k + j * n] * L[i + j * n];
  }
  D[n - 1] = A[n * n - 1];
  free(A);
  return info;
}

/* integer gauss transformation ----------------------------------------------*/
static void gauss(i32 n, f64 *L, f64 *Z, i32 i, i32 j)  // ÀàËÆZTRANËã·¨£¬µ«iºÍjµ÷»»ÁË,j±íÊ¾ÁÐ
{
  i32 k, mu;

  if ((mu = (i32)ROUND(L[i + j * n])) != 0) {                        // ËÄÉáÎåÈë
    for (k = i; k < n; k++) L[k + n * j] -= (f64)mu * L[k + i * n];  // ¸ßË¹ÏûÔª
    for (k = 0; k < n; k++) Z[k + n * j] -= (f64)mu * Z[k + i * n];  // Z¾ØÕó×öÏàÍ¬µÄ±ä»¯
  }
}

/* permutations½»»» --------------------------------------------------------------*/
static void perm(i32 n, f64 *L, f64 *D, i32 j, f64 del, f64 *Z)  // Ìõ¼þ·½²îÅÅÐò
{                                                                // ÓëSRC1Ëã·¨ÖÐµÄÊµÏÖÏàÍ¬
  i32 k;
  f64 eta, lam, a0, a1;

  eta = D[j] / del;
  lam = D[j + 1] * L[j + 1 + j * n] / del;  // lamda3=...
  D[j] = eta * D[j + 1];
  D[j + 1] = del;
  for (k = 0; k <= j - 1; k++) {  // ÓÉÓÚ¾ØÕóÔ­Òò£¬³õÖµ±äÎª0
    a0 = L[j + k * n];
    a1 = L[j + 1 + k * n];  // lamda1=...  lamda2=...
    L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
    L[j + 1 + k * n] = eta * a0 + lam * a1;
  }
  L[j + 1 + j * n] = lam;
  for (k = j + 2; k < n; k++) SWAP(L[k + j * n], L[k + (j + 1) * n]);
  for (k = 0; k < n; k++) SWAP(Z[k + j * n], Z[k + (j + 1) * n]);
}

/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
static void reduction(i32 n, f64 *L, f64 *D,
                      f64 *Z)  // ÀàËÆSRC1Ëã·¨£¬°Ñ¸ßË¹ÏûÔªºÍÌõ¼þ·½²îÅÅÐò·ÅÔÚÒ»Æð½øÐÐ£¨Ä£ºý¶È¾­µä°æ.pdf£©
{  // ÎÒÈÏÎª¸ÃËã·¨ÓëSRC1Ö»ÊÇ¿´ÆðÀ´²»Í¬£¬µ«ÔËÐÐ¹ý³ÌÊÇÏàÍ¬µÄ
  i32 i, j, k;
  f64 del;

  j = n - 2;
  k = n - 2;  // ´Ë´¦kÖ¸i1,ÓÉÓÚ¾ØÕóÔ­Òò±äÎªn-2   jÖ¸i,Í¬ÑùµÄÓÉÓÚ¾ØÕóÔ­Òò±äÎªn-2
  while (j >= 0) {  // while i>1
    if (j <= k)
      for (i = j + 1; i < n; i++) gauss(n, L, Z, i, j);           // ZTRANËã·¨
    del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];  // ¼ÆËãdelta
    if (del + 1E-6 < D[j + 1]) {                                  /* compared considering numerical error */
      perm(n, L, D, j, del, Z);
      k = j;
      j = n - 2;  // ¼´i1=i£¬i¸³³õÖµÒÔ±ã½øÐÐÏÂÒ»´ÎÑ­»·¡££¨Ïà±ÈÓÚSRC1Ëã·¨ÉÙÁËsw²ÎÊý²¢ÇÒÉÙÁËswÄÇÒ»²ãÑ­»·£©
    } else
      j--;
  }
}

/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
static i32 search(i32 n, i32 m, const f64 *L, const f64 *D, const f64 *zs, f64 *zn,
                  f64 *s) {  // n:float parameters(nb=16)   m: fixed solutions(=2)   zs:¼´z=Z'*a
  i32 i, j, k, c, nn = 0, imax = 0;
  f64 newdist, maxdist = 1E99, y;
  f64 *S = zeros(n, n), *dist = mat(n, 1), *zb = mat(n, 1), *z = mat(n, 1), *step = mat(n, 1);
  // ²Î¿¼LDLT_MSearch.m
  //*zbÎªC_z£¬¼´Ìõ¼þ¹ÀÖµ   *zÎª¶Ô*zsÈ¡Õûºó½á¹û    yÎªzb[k]-z[k]    *znÖÐ´æ´¢ËÑË÷µ½µÄºòÑ¡×é *s´æ´¢ºòÑ¡×é¶ÔÓ¦µÄ¿Õ¼ä´óÐ¡

  k = n - 1;
  dist[k] = 0.0;
  zb[k] = zs[k];  // µÚÒ»¸öÌõ¼þ¹ÀÖµ³õÊ¼»¯Îª¸¡µã½â
  z[k] = ROUND(zb[k]);
  y = zb[k] - z[k];
  step[k] = SGN(y);                // SGN(x): ((x)<=0.0?-1.0:1.0)   ROUND(x): (floor((x)+0.5))
  for (c = 0; c < LOOPMAX; c++) {  // LOOPMAX: maximum count of search loop  =10000
    newdist = dist[k] + y * y / D[k];
    if (newdist < maxdist) {
      if (k != 0) {  // ¼ÌÐøÏòÏÂÒ»²ãËÑ
        dist[--k] = newdist;
        for (i = 0; i <= k; i++) S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
        zb[k] = zs[k] + S[k + k * n];
        z[k] = ROUND(zb[k]);
        y = zb[k] - z[k];
        step[k] = SGN(y);
      } else {  // ´æ´¢ºòÑ¡×é
        if (nn < m) {
          if (nn == 0 || newdist > s[imax]) imax = nn;
          for (i = 0; i < n; i++) zn[i + nn * n] = z[i];
          s[nn++] = newdist;
        } else {
          if (newdist < s[imax]) {
            for (i = 0; i < n; i++) zn[i + imax * n] = z[i];
            s[imax] = newdist;
            for (i = imax = 0; i < m; i++)
              if (s[imax] < s[i]) imax = i;
          }
          maxdist = s[imax];
        }
        z[0] += step[0];
        y = zb[0] - z[0];
        step[0] = -step[0] - SGN(step[0]);
      }
    } else {  // ·µ»ØÉÏÒ»²ã
      if (k == n - 1)
        break;
      else {
        k++;
        z[k] += step[k];
        y = zb[k] - z[k];
        step[k] = -step[k] - SGN(step[k]);
      }
    }
  }
  for (i = 0; i < m - 1; i++) { /* sort by s */  // ¶ÔºòÑ¡×éÅÅÐò£¬°´ÕÕ¶ÔÓ¦¿Õ¼äÓÉÐ¡µ½´óµÄË³Ðò
    for (j = i + 1; j < m; j++) {
      if (s[i] < s[j]) continue;
      SWAP(s[i], s[j]);
      for (k = 0; k < n; k++) SWAP(zn[k + i * n], zn[k + j * n]);
    }
  }
  free(S);
  free(dist);
  free(zb);
  free(z);
  free(step);

  if (c >= LOOPMAX) {
    printf("search loop count overflow\n");
    return -1;
  }
  return 0;
}

/* lambda/mlambda integer least-square estimation ------------------------------
 * integer least-square estimation. reduction is performed by lambda (ref.[1]),
 * and search by mlambda (ref.[2]).
 * args   : i32    n      I  number of float parameters ,n=nb=32
 *          i32    m      I  number of fixed solutions ,m=2
 *          f64 *a     I  float parameters (n x 1)
 *          f64 *Q     I  covariance matrix of float parameters (n x n)
 *¼´Qb£¨²»ÊÇ»ùÏßµÄ£¬¶øÊÇrtklibÀï¶Ô¹Ì¶¨½âµÄÃüÃû£© f64 *F     O  fixed solutions (n x m)  ¼´b f64 *s     O  sum of
 *squared residuals of fixed solutions (1 x m) return : status (0:ok,other:error) notes  : matrix stored by column-major
 *order (fortran convension)
 *-----------------------------------------------------------------------------*/
i32 lambda(i32 n, i32 m, const f64 *a, const f64 *Q, f64 *F, f64 *s) {
  i32 info;
  f64 *L, *D, *Z, *z, *E;

  if (n <= 0 || m <= 0) return -1;
  L = zeros(n, n);
  D = mat(n, 1);
  Z = lambda_eye(n);
  z = mat(n, 1), E = mat(n, m);

  /* LD factorization */
  if (!(info = LD(n, Q, L, D))) {
    /* lambda reduction */
    reduction(n, L, D, Z);  // ¸ßË¹ÏûÔª&Ìõ¼þ·½²îÅÅÐò£¬µÃµ½ÐÂµÄLDL·Ö½âºÍÕûÊý±ä»»¾ØÕóZ
    matmul("TN", n, 1, n, 1.0, Z, a, 0.0, z); /* z=Z'*a */

    /* mlambda search */
    if (!(info = search(n, m, L, D, z, E, s))) {  // ´Ë´¦zÎªÕûÊý±ä»»ºóµÄ¸¡µã½âzs    EÎªºòÑ¡×ézn sÎªºòÑ¡×é¶ÔÓ¦µÄ¿Õ¼ä´óÐ¡

      info = solve("T", Z, E, n, m, F); /* F=E/Z' */  // ¶ÔzÄæ±ä»»µÃµ½a
    }
  }
  free(L);
  free(D);
  free(Z);
  free(z);
  free(E);
  return info;  // info=0
}

namespace navp::algorithm {

AmbiguityFixer::AmbiguityFixer(utils::NavVector3f64 float_baseline, f64 *float_ambiguity,
                               const utils::NavMatrixDf64 &float_qxx)
    : float_baseline_(std::move(float_baseline)),
      float_ambiguity_(float_ambiguity, float_qxx.rows() - 3),
      float_qxx_(std::addressof(float_qxx)) {
  if (!_valid()) state_ = State::InitializeFail;
}

bool AmbiguityFixer::_valid() const noexcept {
  return float_qxx_->rows() == float_qxx_->cols() && float_qxx_->rows() == 3 + float_ambiguity_.rows();
}

utils::NavMatrixDf64 AmbiguityFixer::_construct_k1() const noexcept {
  std::size_t n = float_ambiguity_.rows();
  utils::NavMatrixDf64 k1(n, n + 3);
  k1.setZero();
  k1.block(0, 3, n, n).setIdentity();
  return k1;
}

utils::NavMatrixDf64 AmbiguityFixer::_construct_k2() const noexcept {
  std::size_t n = float_ambiguity_.rows();
  utils::NavMatrixDf64 k2(3, 3 + n);
  k2.setZero();
  k2.block(0, 0, 3, 3).setIdentity();
  return k2;
}

bool AmbiguityFixer::fix() noexcept {
  if (state_ == State::InitializeFail) return false;
  std::size_t n = float_ambiguity_.rows();
  auto k1 = _construct_k1(), k2 = _construct_k2();
  float_qbb_ = k2 * (*float_qxx_) * k2.transpose();
  utils::NavMatrixDf64 float_qaa_ = k1 * (*float_qxx_) * k1.transpose();
  utils::NavVectorDf64 alternative_ar(2 * n);
  f64 alternative_ar_residual[2];
  i32 solved = lambda(n, 2, float_ambiguity_.data(), float_qaa_.data(), alternative_ar.data(), alternative_ar_residual);
  if (solved != 0) {
    state_ = State::FixFail;
    return false;
  }
  state_ = State::FixSuccess;
  ratio_ = static_cast<f32>(alternative_ar_residual[1] / alternative_ar_residual[0]);
  // evaluate fixed baseline
  fixed_ambiguity_ = alternative_ar.segment(0, n);
  utils::NavMatrixDf64 float_qba_ = k2 * (*float_qxx_) * k1.transpose();
  auto inverse_float_qaa_ = float_qaa_.inverse();
  fixed_baseline_ = float_baseline_ - float_qba_ * inverse_float_qaa_ * (float_ambiguity_ - fixed_ambiguity_);
  fixed_qbb_ = float_qbb_ - float_qba_ * inverse_float_qaa_ * float_qba_.transpose();
  return true;
}

}  // namespace navp::algorithm