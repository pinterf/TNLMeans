/*
**                 TNLMeans v1.0.3 for Avisynth 2.5.x
**
**   TNLMeans is an implementation of the NL-means denoising algorithm.
**   Aside from the original method, TNLMeans also supports extension
**   into 3D, a faster, block based approach, and a multiscale version.
**
**   Copyright (C) 2006-2007 Kevin Stone
**
**   This program is free software; you can redistribute it and/or modify
**   it under the terms of the GNU General Public License as published by
**   the Free Software Foundation; either version 2 of the License, or
**   (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**   but WITHOUT ANY WARRANTY; without even the implied warranty of
**   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**   GNU General Public License for more details.
**
**   You should have received a copy of the GNU General Public License
**   along with this program; if not, write to the Free Software
**   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "TNLMeans.h"
#include <algorithm>
#include <cstring>
#include "common.h"

static int planes_y[4] = { PLANAR_Y, PLANAR_U, PLANAR_V, PLANAR_A };
static int planes_r[4] = { PLANAR_G, PLANAR_B, PLANAR_R, PLANAR_A };

TNLMeans::TNLMeans(PClip _child, int _Ax, int _Ay, int _Az, int _Sx, int _Sy, int _Bx,
  int _By, bool _ms, double _a, double _h, bool _sse, PClip _hclip, IScriptEnvironment* env) :
  GenericVideoFilter(_child), Ax(_Ax), Ay(_Ay), Az(_Az), Sx(_Sx), Sy(_Sy), Bx(_Bx),
  By(_By), ms(_ms), a(_a), h(_h), sse(_sse), hclip(_hclip)
{
  has_at_least_v8 = true;
  try { env->CheckVersion(8); }
  catch (const AvisynthError&) { has_at_least_v8 = false; }

  planes = (vi.IsPlanarRGB() || vi.IsPlanarRGBA()) ? planes_r : planes_y;
  pixelsize = vi.ComponentSize();
  bits_per_pixel = vi.BitsPerComponent();
  planecount = std::min(vi.NumComponents(), 3); // no alpha

  // YUY2 support by no-the-fly YV16 conversion
  if (!vi.IsPlanar() || vi.BitsPerComponent() > 16)
    env->ThrowError("TNLMeans:  only YUY2, 8-16 bit Y, planar YUV or planar RGB inputs are supported!");
  if (h <= 0.0)
    env->ThrowError("TNLMeans:  h must be greater than 0!");
  if (a <= 0.0)
    env->ThrowError("TNLMeans:  a must be greater than 0!");
  if (Ax < 0)
    env->ThrowError("TNLMeans:  Ax must be greater than or equal to 0!");
  if (Ay < 0)
    env->ThrowError("TNLMeans:  Ay must be greater than or equal to 0!");
  if (Az < 0)
    env->ThrowError("TNLMeans:  Az must be greater than or equal to 0!");
  if (Bx < 0)
    env->ThrowError("TNLMeans:  Bx must be greater than or equal to 0!");
  if (By < 0)
    env->ThrowError("TNLMeans:  By must be greater than or equal to 0!");
  if (Sx < 0)
    env->ThrowError("TNLMeans:  Sx must be greater than or equal to 0!");
  if (Sy < 0)
    env->ThrowError("TNLMeans:  Sy must be greater than or equal to 0!");
  if (Sx < Bx)
    env->ThrowError("TNLMeans:  Sx must be greater than or equal to Bx!");
  if (Sy < By)
    env->ThrowError("TNLMeans:  Sy must be greater than or equal to By!");

  // achieve the same weights bit-depth independently
  if(vi.IsRGB()) // full scale
    h = h * ((1 << bits_per_pixel) - 1) / 255.0;
  else // bit shift
    h = h * (1 << (bits_per_pixel - 8));
  h2in = -1.0 / (h * h);
  hin = -1.0 / h;
  Sxd = Sx * 2 + 1;
  Syd = Sy * 2 + 1;
  Sxa = Sxd * Syd;
  Bxd = Bx * 2 + 1;
  Byd = By * 2 + 1;
  Bxa = Bxd * Byd;
  Axd = Ax * 2 + 1;
  Ayd = Ay * 2 + 1;
  Axa = Axd * Ayd;
  Azdm1 = Az * 2;
  a2 = a * a;
  child->SetCacheHints(CACHE_NOTHING, 0);
  if (ms)
  {
    VideoInfo vi_h = hclip->GetVideoInfo();
    if (Az)
    {
      fcfs = std::make_unique<nlCache>(Az * 2 + 1, false, vi);
      fchs = std::make_unique<nlCache>(Az * 2 + 1, false, vi_h);
    }
    else
    {
      nlfs = std::make_unique<nlFrame>(false, 1, vi);
      nlhs = std::make_unique<nlFrame>(false, 1, vi_h);
    }
    const int Sxh = (Sx + 1) >> 1;
    const int Syh = (Sy + 1) >> 1;
    const int Sxdh = Sxh * 2 + 1;
    const int Sydh = Syh * 2 + 1;
    const int Bxh = (Bx + 1) >> 1;
    const int Byh = (By + 1) >> 1;
    gwh.resize(Sxdh * Sydh);
    int w = 0, m, n;
    for (int j = -Syh; j <= Syh; ++j)
    {
      if (j < 0) m = std::min(j + Byh, 0);
      else m = std::max(j - Byh, 0);
      for (int k = -Sxh; k <= Sxh; ++k)
      {
        if (k < 0) n = std::min(k + Bxh, 0);
        else n = std::max(k - Bxh, 0);
        gwh[w++] = exp(-((m * m + n * n) / (2 * a2)));
      }
    }
  }
  else
  {
    if (Az) {
      fc = std::make_unique<nlCache>(Az * 2 + 1, (Bx > 0 || By > 0), vi);
    }
    if (Bx || By)
    {
      sumsb.resize(Bxa);
      weightsb.resize(Bxa);
    }
    else if (Az == 0)
    {
      ds.sums.resize(vi.width * vi.height);
      ds.weights.resize(vi.width * vi.height);
      ds.wmaxs.resize(vi.width * vi.height);
    }
  }
  gw.resize(Sxd * Syd);
  int w = 0, m, n;
  for (int j = -Sy; j <= Sy; ++j)
  {
    if (j < 0) m = std::min(j + By, 0);
    else m = std::max(j - By, 0);
    for (int k = -Sx; k <= Sx; ++k)
    {
      if (k < 0) n = std::min(k + Bx, 0);
      else n = std::max(k - Bx, 0);
      gw[w++] = exp(-((m * m + n * n) / (2 * a2)));
    }
  }
}

TNLMeans::~TNLMeans()
{
}

PVideoFrame __stdcall TNLMeans::GetFrame(int n, IScriptEnvironment* env)
{
  if (sse)
  {
    if (Az)
    {
      if (ms) {
        if (bits_per_pixel == 8)
          return GetFrameT_MS<uint8_t>(n, env);
        else
          return GetFrameT_MS<uint16_t>(n, env);
      }
      if (Bx || By) {
        if (bits_per_pixel == 8)
          return GetFrameWZB<false, uint8_t>(n, env);
        else
          return GetFrameWZB<false, uint16_t>(n, env);
      }
      if (bits_per_pixel == 8)
        return GetFrameWZ<false, uint8_t>(n, env);
      else
        return GetFrameWZ<false, uint16_t>(n, env);
    }
    if (ms) {
      if (bits_per_pixel == 8)
        return GetFrameNT_MS<uint8_t>(n, env);
      else
        return GetFrameNT_MS<uint16_t>(n, env);
    }
    if (Bx || By) {
      if (bits_per_pixel == 8)
        return GetFrameWOZB<false, uint8_t>(n, env);
      else
        return GetFrameWOZB<false, uint16_t>(n, env);
    }
    if (bits_per_pixel == 8)
      return GetFrameWOZ<false, uint8_t>(n, env);
    else
      return GetFrameWOZ<false, uint16_t>(n, env);
  }
  if (Az)
  {
    if (ms) {
      if (bits_per_pixel == 8)
        return GetFrameT_MS<uint8_t>(n, env);
      else
        return GetFrameT_MS<uint16_t>(n, env);
    }
    if (Bx || By) {
      if (bits_per_pixel == 8)
        return GetFrameWZB<true, uint8_t>(n, env);
      else
        return GetFrameWZB<true, uint16_t>(n, env);
    }
    if (bits_per_pixel == 8)
      return GetFrameWZ<true, uint8_t>(n, env);
    else
      return GetFrameWZ<true, uint16_t>(n, env);
  }
  if (ms) {
    if (bits_per_pixel == 8)
      return GetFrameNT_MS<uint8_t>(n, env);
    else
      return GetFrameNT_MS<uint16_t>(n, env);
  }
  if (Bx || By) {
    if (bits_per_pixel == 8)
      return GetFrameWOZB<true, uint8_t>(n, env);
    else
      return GetFrameWOZB<true, uint16_t>(n, env);
  }
  if (bits_per_pixel == 8)
    return GetFrameWOZ<true, uint8_t>(n, env);
  else
    return GetFrameWOZ<true, uint16_t>(n, env);
}

template<bool SAD, typename pixel_t>
PVideoFrame __stdcall TNLMeans::GetFrameWZ(int n, IScriptEnvironment* env)
{
  fc->resetCacheStart(n - Az, n + Az);
  for (int i = n - Az; i <= n + Az; ++i)
  {
    nlFrame* nl = fc->frames[fc->getCachePos(i - n + Az)];
    if (nl->fnum != i)
    {
      PVideoFrame src = child->GetFrame(mapn(i), env);
      nl->pf = src;
      nl->setFNum(i);
      fc->clearDS(nl);
    }
  }

  const int MAX_PIXEL_VALUE = sizeof(pixel_t) == 1 ? 255 : (1 << bits_per_pixel) - 1;
  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  std::vector<const uint8_t*> pfplut(fc->size);
  std::vector<int> pfplut_pitch(fc->size);
  std::vector<const SDATA*> dslut(fc->size);
  std::vector<int*> dsalut(fc->size);
  for (int i = 0; i < fc->size; ++i)
    dsalut[i] = fc->frames[fc->getCachePos(i)]->dsa.data();
  int* ddsa = dsalut[Az];

  PVideoFrame srcPF = fc->frames[fc->getCachePos(Az)]->pf;
  PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &srcPF) : env->NewVideoFrame(vi); // frame property support

  const int startz = Az - std::min(n, Az);
  const int stopz = Az + std::min(vi.num_frames - n - 1, Az);
  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(srcPF->GetReadPtr(plane));
    const pixel_t* pf2p = srcp;
    const int src_pitch = dst->GetPitch(plane) / sizeof(pixel_t);
    pixel_t* dstp = reinterpret_cast<pixel_t *>(dst->GetWritePtr(plane));
    const int dst_pitch = dst->GetPitch(plane) / sizeof(pixel_t);
    const int height = dst->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = dst->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    for (int i = 0; i < fc->size; ++i) {
      const int pos = fc->getCachePos(i);
      pfplut[i] = fc->frames[pos]->pf->GetReadPtr(plane);
      pfplut_pitch[i] = fc->frames[pos]->pf->GetPitch(plane) / sizeof(pixel_t);
      dslut[i] = &fc->frames[pos]->ds[b];
    }
    const SDATA* dds = dslut[Az];
    for (int y = 0; y < height; ++y)
    {
      const int startyt = std::max(y - Ay, 0);
      const int stopy = std::min(y + Ay, heightm1);
      const int doffy = y * width;
      for (int x = 0; x < width; ++x)
      {
        const int startxt = std::max(x - Ax, 0);
        const int stopx = std::min(x + Ax, widthm1);
        const int doff = doffy + x;
        double* dsum = const_cast<double *>(&dds->sums[doff]);
        double* dweight = const_cast<double*>(&dds->weights[doff]);
        double* dwmax = const_cast<double*>(&dds->wmaxs[doff]);
        for (int z = startz; z <= stopz; ++z)
        {
          if (ddsa[z] == 1) continue;
          else ddsa[z] = 2;
          const int starty = (z == Az) ? y : startyt;
          const SDATA* cds = dslut[z];
          int* cdsa = dsalut[z];
          const pixel_t* pf1p = reinterpret_cast<const pixel_t *>(pfplut[z]);
          const int pf1p_pitch = pfplut_pitch[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int startx = (u == y && z == Az) ? x + 1 : startxt;
            const int yT = -std::min(std::min(Sy, u), y);
            const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
            const pixel_t* s1_saved = pf1p + (u + yT) * pf1p_pitch;
            const pixel_t* s2_saved = pf2p + (y + yT) * src_pitch + x;
            const double* gw_saved = gw.data() + (yT + Sy) * Sxd + Sx;
            const int pf1pl = u * pf1p_pitch;
            const int coffy = u * width;
            for (int v = startx; v <= stopx; ++v)
            {
              const int coff = coffy + v;
              double* csum = const_cast<double*>(&cds->sums[coff]);
              double* cweight = const_cast<double*>(&cds->weights[coff]);
              double* cwmax = const_cast<double*>(&cds->wmaxs[coff]);
              const int xL = -std::min(std::min(Sx, v), x);
              const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
              const pixel_t* s1 = s1_saved + v;
              const pixel_t* s2 = s2_saved;
              const double* gwT = gw_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  if constexpr(SAD)
                    diff += abs(s1[k] - s2[k]) * gwT[k];
                  else
                    diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                  gweights += gwT[k];
                }
                s1 += pf1p_pitch;
                s2 += src_pitch;
                gwT += Sxd;
              }
              double weight;
              if constexpr(SAD)
                weight = exp((diff / gweights) * hin);
              else
                weight = exp((diff / gweights) * h2in);
              *dweight += weight;
              *dsum += pf1p[pf1pl + v] * weight;
              if (weight > *dwmax) *dwmax = weight;
              if (cdsa[Azdm1 - z] != 1)
              {
                *cweight += weight;
                *csum += srcp[x] * weight;
                if (weight > *cwmax) *cwmax = weight;
              }
            }
          }
        }
        const double wmax = *dwmax <= DBL_EPSILON ? 1.0 : *dwmax;
        *dsum += srcp[x] * wmax;
        *dweight += wmax;
        dstp[x] = std::max(std::min(int(((*dsum) / (*dweight)) + 0.5), MAX_PIXEL_VALUE), 0);
      }
      dstp += dst_pitch;
      srcp += src_pitch;
    }
  }
  int j = fc->size - 1;
  for (int i = 0; i < fc->size; ++i, --j)
  {
    int* cdsa = fc->frames[fc->getCachePos(i)]->dsa.data();
    if (ddsa[i] == 2) ddsa[i] = cdsa[j] = 1;
  }
  return dst;
}


template<bool SAD, typename pixel_t>
PVideoFrame __stdcall TNLMeans::GetFrameWZB(int n, IScriptEnvironment* env)
{
  fc->resetCacheStart(n - Az, n + Az);
  for (int i = n - Az; i <= n + Az; ++i)
  {
    nlFrame* nl = fc->frames[fc->getCachePos(i - n + Az)];
    if (nl->fnum != i)
    {
      PVideoFrame src = child->GetFrame(mapn(i), env);
      nl->pf = src;
      nl->setFNum(i);
    }
  }

  const int MAX_PIXEL_VALUE = sizeof(pixel_t) == 1 ? 255 : (1 << bits_per_pixel) - 1;
  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  std::vector<const uint8_t*> pfplut(fc->size);
  std::vector<int> pfplut_pitch(fc->size);

  PVideoFrame srcPF = fc->frames[fc->getCachePos(Az)]->pf;
  PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &srcPF) : env->NewVideoFrame(vi); // frame property support

  const int startz = Az - std::min(n, Az);
  const int stopz = Az + std::min(vi.num_frames - n - 1, Az);
  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t *>(srcPF->GetReadPtr(plane));
    const pixel_t* pf2p = srcp;
    const int src_pitch = srcPF->GetPitch(plane) / sizeof(pixel_t);
    pixel_t* dstp = reinterpret_cast<pixel_t*>(dst->GetWritePtr(plane));
    const int dst_pitch = dst->GetPitch(plane) / sizeof(pixel_t);
    const int height = dst->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = dst->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    double* sumsb_saved = sumsb.data() + Bx;
    double* weightsb_saved = weightsb.data() + Bx;
    for (int i = 0; i < fc->size; ++i) {
      const int pos = fc->getCachePos(i);
      pfplut[i] = fc->frames[pos]->pf->GetReadPtr(plane);
      pfplut_pitch[i] = fc->frames[pos]->pf->GetPitch(plane) / sizeof(pixel_t);
    }
    for (int y = By; y < height + By; y += Byd)
    {
      const int starty = std::max(y - Ay, By);
      const int stopy = std::min(y + Ay, heightm1 - std::min(By, heightm1 - y));
      const int yTr = std::min(Byd, height - y + By);
      for (int x = Bx; x < width + Bx; x += Bxd)
      {
        std::fill(sumsb.begin(), sumsb.end(), 0.0);
        std::fill(weightsb.begin(), weightsb.end(), 0.0);
        double wmax = 0.0;
        const int startx = std::max(x - Ax, Bx);
        const int stopx = std::min(x + Ax, widthm1 - std::min(Bx, widthm1 - x));
        const int xTr = std::min(Bxd, width - x + Bx);
        for (int z = startz; z <= stopz; ++z)
        {
          const pixel_t* pf1p = reinterpret_cast<const pixel_t *>(pfplut[z]);
          const int pf1p_pitch = pfplut_pitch[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int yT = -std::min(std::min(Sy, u), y);
            const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
            const int yBb = std::min(std::min(By, heightm1 - u), heightm1 - y);
            const pixel_t* s1_saved = pf1p + (u + yT) * pf1p_pitch;
            const pixel_t* s2_saved = pf2p + (y + yT) * src_pitch + x;
            const pixel_t* sbp_saved = pf1p + (u - By) * pf1p_pitch;
            const double* gw_saved = gw.data() + (yT + Sy) * Sxd + Sx;
            const int pf1pl = u * pf1p_pitch;
            for (int v = startx; v <= stopx; ++v)
            {
              if (z == Az && u == y && v == x) continue;
              const int xL = -std::min(std::min(Sx, v), x);
              const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
              const pixel_t* s1 = s1_saved + v;
              const pixel_t* s2 = s2_saved;
              const double* gwT = gw_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  if constexpr(SAD)
                    diff += abs(s1[k] - s2[k]) * gwT[k];
                  else
                    diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                  gweights += gwT[k];
                }
                s1 += pf1p_pitch;
                s2 += src_pitch;
                gwT += Sxd;
              }
              double weight;
              if constexpr(SAD)
                weight = exp((diff / gweights) * hin);
              else
                weight = exp((diff / gweights) * h2in);
              const int xRb = std::min(std::min(Bx, widthm1 - v), widthm1 - x);
              const pixel_t* sbp = sbp_saved + v;
              double* sumsbT = sumsb_saved;
              double* weightsbT = weightsb_saved;
              for (int j = -By; j <= yBb; ++j)
              {
                for (int k = -Bx; k <= xRb; ++k)
                {
                  sumsbT[k] += sbp[k] * weight;
                  weightsbT[k] += weight;
                }
                sbp += pf1p_pitch;
                sumsbT += Bxd;
                weightsbT += Bxd;
              }
              if (weight > wmax) wmax = weight;
            }
          }
        }
        const pixel_t* srcpT = srcp + x - Bx;
        pixel_t* dstpT = dstp + x - Bx;
        double* sumsbTr = sumsb.data();
        double* weightsbTr = weightsb.data();
        if (wmax <= DBL_EPSILON) wmax = 1.0;
        for (int j = 0; j < yTr; ++j)
        {
          for (int k = 0; k < xTr; ++k)
          {
            sumsbTr[k] += srcpT[k] * wmax;
            weightsbTr[k] += wmax;
            dstpT[k] = std::max(std::min(int((sumsbTr[k] / weightsbTr[k]) + 0.5), MAX_PIXEL_VALUE), 0);
          }
          srcpT += src_pitch;
          dstpT += dst_pitch;
          sumsbTr += Bxd;
          weightsbTr += Bxd;
        }
      }
      dstp += dst_pitch * Byd;
      srcp += src_pitch * Byd;
    }
  }
  return dst;
}

template<bool SAD, typename pixel_t>
PVideoFrame __stdcall TNLMeans::GetFrameWOZ(int n, IScriptEnvironment* env)
{
  PVideoFrame src = child->GetFrame(mapn(n), env);
  PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &src) : env->NewVideoFrame(vi); // frame property support

  const int MAX_PIXEL_VALUE = sizeof(pixel_t) == 1 ? 255 : (1 << bits_per_pixel) - 1;
  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(src->GetReadPtr(plane));
    const pixel_t* pfp = srcp;
    const int src_pitch = src->GetPitch(plane) / sizeof(pixel_t);
    pixel_t* dstp = reinterpret_cast<pixel_t*>(dst->GetWritePtr(plane));
    const int dst_pitch = dst->GetPitch(plane) / sizeof(pixel_t);
    const int height = dst->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = dst->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    std::fill(ds.sums.begin(), ds.sums.end(), 0.0);
    std::fill(ds.weights.begin(), ds.weights.end(), 0.0);
    std::fill(ds.wmaxs.begin(), ds.wmaxs.end(), 0.0);
    for (int y = 0; y < height; ++y)
    {
      const int stopy = std::min(y + Ay, heightm1);
      const int doffy = y * width;
      for (int x = 0; x < width; ++x)
      {
        const int startxt = std::max(x - Ax, 0);
        const int stopx = std::min(x + Ax, widthm1);
        const int doff = doffy + x;
        double* dsum = &ds.sums[doff];
        double* dweight = &ds.weights[doff];
        double* dwmax = &ds.wmaxs[doff];
        for (int u = y; u <= stopy; ++u)
        {
          const int startx = u == y ? x + 1 : startxt;
          const int yT = -std::min(std::min(Sy, u), y);
          const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
          const pixel_t* s1_saved = pfp + (u + yT) * src_pitch;
          const pixel_t* s2_saved = pfp + (y + yT) * src_pitch + x;
          const double* gw_saved = gw.data() + (yT + Sy) * Sxd + Sx;
          const int pfpl = u * src_pitch;
          const int coffy = u * width;
          for (int v = startx; v <= stopx; ++v)
          {
            const int coff = coffy + v;
            double* csum = &ds.sums[coff];
            double* cweight = &ds.weights[coff];
            double* cwmax = &ds.wmaxs[coff];
            const int xL = -std::min(std::min(Sx, v), x);
            const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
            const pixel_t* s1 = s1_saved + v;
            const pixel_t* s2 = s2_saved;
            const double* gwT = gw_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                if constexpr(SAD)
                  diff += abs(s1[k] - s2[k]) * gwT[k];
                else
                  diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                gweights += gwT[k];
              }
              s1 += src_pitch;
              s2 += src_pitch;
              gwT += Sxd;
            }
            double weight;
            if constexpr(SAD)
              weight = exp((diff / gweights) * hin);
            else
              weight = exp((diff / gweights) * h2in);
            *cweight += weight;
            *dweight += weight;
            *csum += srcp[x] * weight;
            *dsum += pfp[pfpl + v] * weight;
            if (weight > *cwmax) *cwmax = weight;
            if (weight > *dwmax) *dwmax = weight;
          }
        }
        const double wmax = *dwmax <= DBL_EPSILON ? 1.0 : *dwmax;
        *dsum += srcp[x] * wmax;
        *dweight += wmax;
        dstp[x] = std::max(std::min(int(((*dsum) / (*dweight)) + 0.5), MAX_PIXEL_VALUE), 0);
      }
      dstp += dst_pitch;
      srcp += src_pitch;
    }
  }
  return dst;
}

template<bool SAD, typename pixel_t>
PVideoFrame __stdcall TNLMeans::GetFrameWOZB(int n, IScriptEnvironment* env)
{
  PVideoFrame src = child->GetFrame(mapn(n), env);
  PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &src) : env->NewVideoFrame(vi); // frame property support

  const int MAX_PIXEL_VALUE = sizeof(pixel_t) == 1 ? 255 : (1 << bits_per_pixel) - 1;
  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(src->GetReadPtr(plane));
    const pixel_t* pfp = srcp;
    const int src_pitch = dst->GetPitch(plane) / sizeof(pixel_t);
    pixel_t* dstp = reinterpret_cast<pixel_t*>(dst->GetWritePtr(plane));
    const int dst_pitch = dst->GetPitch(plane) / sizeof(pixel_t);
    const int height = dst->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = dst->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    double* sumsb_saved = sumsb.data() + Bx;
    double* weightsb_saved = weightsb.data() + Bx;
    for (int y = By; y < height + By; y += Byd)
    {
      const int starty = std::max(y - Ay, By);
      const int stopy = std::min(y + Ay, heightm1 - std::min(By, heightm1 - y));
      const int yTr = std::min(Byd, height - y + By);
      for (int x = Bx; x < width + Bx; x += Bxd)
      {
        std::fill(sumsb.begin(), sumsb.end(), 0.0);
        std::fill(weightsb.begin(), weightsb.end(), 0.0);
        double wmax = 0.0;
        const int startx = std::max(x - Ax, Bx);
        const int stopx = std::min(x + Ax, widthm1 - std::min(Bx, widthm1 - x));
        const int xTr = std::min(Bxd, width - x + Bx);
        for (int u = starty; u <= stopy; ++u)
        {
          const int yT = -std::min(std::min(Sy, u), y);
          const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
          const int yBb = std::min(std::min(By, heightm1 - u), heightm1 - y);
          const pixel_t* s1_saved = pfp + (u + yT) * src_pitch;
          const pixel_t* s2_saved = pfp + (y + yT) * src_pitch + x;
          const pixel_t* sbp_saved = pfp + (u - By) * src_pitch;
          const double* gw_saved = gw.data() + (yT + Sy) * Sxd + Sx;
          for (int v = startx; v <= stopx; ++v)
          {
            if (u == y && v == x) continue;
            const int xL = -std::min(std::min(Sx, v), x);
            const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
            const pixel_t* s1 = s1_saved + v;
            const pixel_t* s2 = s2_saved;
            const double* gwT = gw_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                if constexpr(SAD)
                  diff += abs(s1[k] - s2[k]) * gwT[k];
                else
                  diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                gweights += gwT[k];
              }
              s1 += src_pitch;
              s2 += src_pitch;
              gwT += Sxd;
            }
            double weight;
            if constexpr(SAD)
              weight = exp((diff / gweights) * hin);
            else
              weight = exp((diff / gweights) * h2in);
            const int xRb = std::min(std::min(Bx, widthm1 - v), widthm1 - x);
            const pixel_t* sbp = sbp_saved + v;
            double* sumsbT = sumsb_saved;
            double* weightsbT = weightsb_saved;
            for (int j = -By; j <= yBb; ++j)
            {
              for (int k = -Bx; k <= xRb; ++k)
              {
                sumsbT[k] += sbp[k] * weight;
                weightsbT[k] += weight;
              }
              sumsbT += Bxd;
              weightsbT += Bxd;
              sbp += src_pitch;
            }
            if (weight > wmax) wmax = weight;
          }
        }
        const pixel_t* srcpT = srcp + x - Bx;
        pixel_t* dstpT = dstp + x - Bx;
        double* sumsbTr = sumsb.data();
        double* weightsbTr = weightsb.data();
        if (wmax <= DBL_EPSILON) wmax = 1.0;
        for (int j = 0; j < yTr; ++j)
        {
          for (int k = 0; k < xTr; ++k)
          {
            sumsbTr[k] += srcpT[k] * wmax;
            weightsbTr[k] += wmax;
            dstpT[k] = std::max(std::min(int((sumsbTr[k] / weightsbTr[k]) + 0.5), MAX_PIXEL_VALUE), 0);
          }
          srcpT += src_pitch;
          dstpT += dst_pitch;
          sumsbTr += Bxd;
          weightsbTr += Bxd;
        }
      }
      dstp += dst_pitch * Byd;
      srcp += src_pitch * Byd;
    }
  }
  return dst;
}

int TNLMeans::mapn(int n)
{
  if (n < 0) return 0;
  if (n >= vi.num_frames) return vi.num_frames - 1;
  return n;
}

nlFrame::nlFrame()
{
  fnum = -20;
  pf = nullptr;
}

nlFrame::nlFrame(bool _useblocks, int _size, VideoInfo& vi)
{
  fnum = -20;
  pf = nullptr;
  if (!_useblocks)
  {
    const int planecount = std::min(vi.NumComponents(), 3);
    const int* planes = (vi.IsPlanarRGB() || vi.IsPlanarRGBA()) ? planes_r : planes_y;
    ds.resize(planecount);
    for (int i = 0; i < planecount; ++i)
    {
      const int plane = planes[i];
      const int pixels = (vi.width >> vi.GetPlaneWidthSubsampling(plane)) * (vi.height >> vi.GetPlaneHeightSubsampling(plane));
      ds[i].sums.resize(pixels);
      ds[i].weights.resize(pixels);
      ds[i].wmaxs.resize(pixels);
    }
    dsa.resize(_size, 0); // init with zero
  }
}

nlFrame::~nlFrame()
{
  pf = nullptr;
}

void nlFrame::setFNum(int i)
{
  fnum = i;
}

nlCache::nlCache()
{
  start_pos = size = -20;
}

nlCache::nlCache(int _size, bool _useblocks, VideoInfo& vi)
{
  start_pos = size = -20;
  if (_size > 0)
  {
    start_pos = 0;
    size = _size;
    frames.resize(size);
    for (int i = 0; i < size; ++i)
      frames[i] = new nlFrame(_useblocks, _size, vi);
  }
}

nlCache::~nlCache()
{
  for (int i = 0; i < frames.size(); ++i)
  {
    if (frames[i]) delete frames[i];
  }
}

void nlCache::resetCacheStart(int first, int last)
{
  for (int j = first; j <= last; ++j)
  {
    for (int i = 0; i < size; ++i)
    {
      if (frames[i]->fnum == j)
      {
        start_pos = i - j + first;
        if (start_pos < 0) start_pos += size;
        else if (start_pos >= size) start_pos -= size;
        return;
      }
    }
  }
}

void nlCache::clearDS(nlFrame* nl)
{
  for (int i = 0; i < nl->ds.size(); ++i)
  {
    std::fill(nl->ds[i].sums.begin(), nl->ds[i].sums.end(), 0.0);
    std::fill(nl->ds[i].weights.begin(), nl->ds[i].weights.end(), 0.0);
    std::fill(nl->ds[i].wmaxs.begin(), nl->ds[i].wmaxs.end(), 0.0);
  }
  for (int i = 0; i < size; ++i) nl->dsa[i] = 0;
}

int nlCache::getCachePos(int n)
{
  return (start_pos + n) % size;
}

AVSValue __cdecl Create_TNLMeans(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  PClip clip = args[0].AsClip();

  const bool isYUY2 = clip->GetVideoInfo().IsYUY2();

  // YUY2 support kept by converting pre-post on-the fly
  if (isYUY2) {
    AVSValue new_args[1] = { clip };
    clip = env->Invoke("ConvertToYV16", AVSValue(new_args, 1)).AsClip();
  }

  PClip hclip;
  if (args[8].IsBool() && args[8].AsBool()) // ms
  {
    const int rm = args[9].IsInt() ? args[9].AsInt() : 4;
    if (rm < 0 || rm > 5)
      env->ThrowError("TNLMeans:  rm must be set to 0, 1, 2, 3, 4, or 5!");
    if (!args[0].IsClip())
      env->ThrowError("TNLMeans:  first argument must be a clip!");

    const int w = clip->GetVideoInfo().width;
    const int h = clip->GetVideoInfo().height;

    if (w % 2 != 0 || h % 2 != 0)
      env->ThrowError("TNLMeans:  clip width and height must be even when 'ms' is true!");

    AVSValue argsv[4] = { clip, w / 2, h / 2 };
    try {
      hclip = env->Invoke(rm == 0 ? "PointResize" :
        rm == 1 ? "BilinearResize" :
        rm == 2 ? "BicubicResize" :
        rm == 3 ? "LanczosResize" :
        rm == 4 ? "Spline16Resize" :
        "Spline36Resize", AVSValue(argsv, 4)).AsClip();
    }
    catch (IScriptEnvironment::NotFound)
    {
      env->ThrowError("TNLMeans:  unable to invoke resizer!");
    }
    catch (AvisynthError e)
    {
      env->ThrowError("TNLMeans:  error invoking resizer (%s)!", e.msg);
    }
  }
  const bool usse = args[12].IsBool() ? args[12].AsBool() : true;
  auto Result = new TNLMeans(clip, args[1].AsInt(4), args[2].AsInt(4), args[3].AsInt(0),
    args[4].AsInt(2), args[5].AsInt(2), args[6].AsInt(1), args[7].AsInt(1), args[8].AsBool(false),
    args[10].AsFloat(1.0), args[11].AsFloat(usse ? 1.8f : 0.5f), usse, hclip, env);

  if (isYUY2) {
    AVSValue new_args2[1] = { Result };
    return env->Invoke("ConvertToYUY2", AVSValue(new_args2, 1)).AsClip();
  }
  return Result;
}

// Declare and initialise server pointers static storage.
const AVS_Linkage* AVS_linkage = 0;

// DLL entry point called from LoadPlugin() to setup a user plugin.
extern "C" __declspec(dllexport) const char* __stdcall
AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors) {

  AVS_linkage = vectors;
  env->AddFunction("TNLMeans", "c[Ax]i[Ay]i[Az]i[Sx]i[Sy]i[Bx]i[By]i[ms]b[rm]i[a]f[h]f[sse]b",
    Create_TNLMeans, 0);
  return 0;
}
