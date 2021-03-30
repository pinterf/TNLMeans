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


TNLMeans::TNLMeans(PClip _child, int _Ax, int _Ay, int _Az, int _Sx, int _Sy, int _Bx,
  int _By, bool _ms, double _a, double _h, bool _sse, PClip _hclip, IScriptEnvironment* env) :
  GenericVideoFilter(_child), Ax(_Ax), Ay(_Ay), Az(_Az), Sx(_Sx), Sy(_Sy), Bx(_Bx),
  By(_By), ms(_ms), a(_a), h(_h), sse(_sse), hclip(_hclip)
{
  int cpuFlags = env->GetCPUFlags();
  fc = fcfs = fchs = nullptr;
  dstPF = srcPFr = nullptr;
  gw = gwh = nullptr;
  weightsb = sumsb = nullptr;
  nlfs = nlhs = nullptr;
  ds = nullptr;
  if (vi.IsRGB() || vi.IsY() || vi.BitsPerComponent() > 8)
    env->ThrowError("TNLMeans:  only yuy2 and 8 bit planar YUV input are supported!");
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
  dstPF = new PlanarFrame(vi, cpuFlags);
  if (ms)
  {
    VideoInfo vi_h = hclip->GetVideoInfo();
    if (Az)
    {
      fcfs = new nlCache(Az * 2 + 1, false, vi, cpuFlags);
      fchs = new nlCache(Az * 2 + 1, false, vi_h, cpuFlags);
    }
    else
    {
      nlfs = new nlFrame(false, 1, vi, cpuFlags);
      nlhs = new nlFrame(false, 1, vi_h, cpuFlags);
    }
    const int Sxh = (Sx + 1) >> 1;
    const int Syh = (Sy + 1) >> 1;
    const int Sxdh = Sxh * 2 + 1;
    const int Sydh = Syh * 2 + 1;
    const int Bxh = (Bx + 1) >> 1;
    const int Byh = (By + 1) >> 1;
    gwh = (double*)_aligned_malloc(Sxdh * Sydh * sizeof(double), 16);
    if (!gwh) env->ThrowError("TNLMeans:  malloc failure (gwh)!");
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
    if (Az) fc = new nlCache(Az * 2 + 1, (Bx > 0 || By > 0), vi, cpuFlags);
    else srcPFr = new PlanarFrame(vi, cpuFlags);
    if (Bx || By)
    {
      sumsb = (double*)_aligned_malloc(Bxa * sizeof(double), 16);
      if (!sumsb) env->ThrowError("TNLMeans:  malloc failure (sumsb)!");
      weightsb = (double*)_aligned_malloc(Bxa * sizeof(double), 16);
      if (!weightsb) env->ThrowError("TNLMeans:  malloc failure (weightsb)!");
    }
    else if (Az == 0)
    {
      ds = new SDATA();
      ds->sums = (double*)_aligned_malloc(vi.width * vi.height * sizeof(double), 16);
      ds->weights = (double*)_aligned_malloc(vi.width * vi.height * sizeof(double), 16);
      ds->wmaxs = (double*)_aligned_malloc(vi.width * vi.height * sizeof(double), 16);
      if (!ds || !ds->sums || !ds->weights || !ds->wmaxs)
        env->ThrowError("TNLMeans:  malloc failure (ds)!");
    }
  }
  gw = (double*)_aligned_malloc(Sxd * Syd * sizeof(double), 16);
  if (!gw) env->ThrowError("TNLMeans:  malloc failure (gw)!");
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
  if (fc) delete fc;
  if (fcfs) delete fcfs;
  if (fchs) delete fchs;
  if (dstPF) delete dstPF;
  if (srcPFr) delete srcPFr;
  if (gw) _aligned_free(gw);
  if (gwh) _aligned_free(gwh);
  if (sumsb) _aligned_free(sumsb);
  if (weightsb) _aligned_free(weightsb);
  if (nlfs) delete nlfs;
  if (nlhs) delete nlhs;
  if (ds)
  {
    _aligned_free(ds->sums);
    _aligned_free(ds->weights);
    _aligned_free(ds->wmaxs);
    delete ds;
  }
}

PVideoFrame __stdcall TNLMeans::GetFrame(int n, IScriptEnvironment* env)
{
  if (sse)
  {
    if (Az)
    {
      if (ms) return GetFrameT_MS(n, env);
      if (Bx || By) return GetFrameWZB<false>(n, env);
      return GetFrameWZ<false>(n, env);
    }
    if (ms) return GetFrameNT_MS(n, env);
    if (Bx || By) return GetFrameWOZB<false>(n, env);
    return GetFrameWOZ<false>(n, env);
  }
  if (Az)
  {
    if (ms) return GetFrameT_MS(n, env);
    if (Bx || By) return GetFrameWZB<true>(n, env);
    return GetFrameWZ<true>(n, env);
  }
  if (ms) return GetFrameNT_MS(n, env);
  if (Bx || By) return GetFrameWOZB<true>(n, env);
  return GetFrameWOZ<true>(n, env);
}

template<bool SAD>
PVideoFrame __stdcall TNLMeans::GetFrameWZ(int n, IScriptEnvironment* env)
{
  fc->resetCacheStart(n - Az, n + Az);
  for (int i = n - Az; i <= n + Az; ++i)
  {
    nlFrame* nl = fc->frames[fc->getCachePos(i - n + Az)];
    if (nl->fnum != i)
    {
      PVideoFrame src = child->GetFrame(mapn(i), env);
      nl->pf->copyFrom(src, vi);
      nl->setFNum(i);
      fc->clearDS(nl);
    }
  }
  const uint8_t** pfplut =
    (const uint8_t**)_aligned_malloc(fc->size * sizeof(const uint8_t*), 16);
  if (!pfplut) env->ThrowError("TNLMeans:  malloc failure (pfplut)!");
  const SDATA** dslut =
    (const SDATA**)_aligned_malloc(fc->size * sizeof(SDATA*), 16);
  if (!dslut) env->ThrowError("TNLMeans:  malloc failure (dslut)!");
  int** dsalut =
    (int**)_aligned_malloc(fc->size * sizeof(int*), 16);
  if (!dsalut) env->ThrowError("TNLMeans:  malloc failure (dsalut)!");
  for (int i = 0; i < fc->size; ++i)
    dsalut[i] = fc->frames[fc->getCachePos(i)]->dsa;
  int* ddsa = dsalut[Az];
  PlanarFrame* srcPF = fc->frames[fc->getCachePos(Az)]->pf;
  const int startz = Az - std::min(n, Az);
  const int stopz = Az + std::min(vi.num_frames - n - 1, Az);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pf2p = srcPF->GetPtr(b);
    uint8_t* dstp = dstPF->GetPtr(b);
    const int pitch = dstPF->GetPitch(b);
    const int height = dstPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = dstPF->GetWidth(b);
    const int widthm1 = width - 1;
    for (int i = 0; i < fc->size; ++i)
    {
      const int pos = fc->getCachePos(i);
      pfplut[i] = fc->frames[pos]->pf->GetPtr(b);
      dslut[i] = fc->frames[pos]->ds[b];
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
        double* dsum = &dds->sums[doff];
        double* dweight = &dds->weights[doff];
        double* dwmax = &dds->wmaxs[doff];
        for (int z = startz; z <= stopz; ++z)
        {
          if (ddsa[z] == 1) continue;
          else ddsa[z] = 2;
          const int starty = (z == Az) ? y : startyt;
          const SDATA* cds = dslut[z];
          int* cdsa = dsalut[z];
          const uint8_t* pf1p = pfplut[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int startx = (u == y && z == Az) ? x + 1 : startxt;
            const int yT = -std::min(std::min(Sy, u), y);
            const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
            const uint8_t* s1_saved = pf1p + (u + yT) * pitch;
            const uint8_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const double* gw_saved = gw + (yT + Sy) * Sxd + Sx;
            const int pf1pl = u * pitch;
            const int coffy = u * width;
            for (int v = startx; v <= stopx; ++v)
            {
              const int coff = coffy + v;
              double* csum = &cds->sums[coff];
              double* cweight = &cds->weights[coff];
              double* cwmax = &cds->wmaxs[coff];
              const int xL = -std::min(std::min(Sx, v), x);
              const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
              const uint8_t* s1 = s1_saved + v;
              const uint8_t* s2 = s2_saved;
              const double* gwT = gw_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  if constexpr(SAD)
                    diff += abs(s1[k] - s2[k]) * gwT[k];
                  else
                    diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                  gweights += gwT[k];
                }
                s1 += pitch;
                s2 += pitch;
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
        dstp[x] = std::max(std::min(int(((*dsum) / (*dweight)) + 0.5), 255), 0);
      }
      dstp += pitch;
      srcp += pitch;
    }
  }
  int j = fc->size - 1;
  for (int i = 0; i < fc->size; ++i, --j)
  {
    int* cdsa = fc->frames[fc->getCachePos(i)]->dsa;
    if (ddsa[i] == 2) ddsa[i] = cdsa[j] = 1;
  }
  _aligned_free(dsalut);
  _aligned_free(dslut);
  _aligned_free(pfplut);
  PVideoFrame dst = env->NewVideoFrame(vi);
  dstPF->copyTo(dst, vi);
  return dst;
}


template<bool SAD>
PVideoFrame __stdcall TNLMeans::GetFrameWZB(int n, IScriptEnvironment* env)
{
  fc->resetCacheStart(n - Az, n + Az);
  for (int i = n - Az; i <= n + Az; ++i)
  {
    nlFrame* nl = fc->frames[fc->getCachePos(i - n + Az)];
    if (nl->fnum != i)
    {
      PVideoFrame src = child->GetFrame(mapn(i), env);
      nl->pf->copyFrom(src, vi);
      nl->setFNum(i);
    }
  }
  const uint8_t** pfplut =
    (const uint8_t**)_aligned_malloc(fc->size * sizeof(const uint8_t*), 16);
  if (!pfplut) env->ThrowError("TNLMeans:  malloc failure (pfplut)!");
  PlanarFrame* srcPF = fc->frames[fc->getCachePos(Az)]->pf;
  const int startz = Az - std::min(n, Az);
  const int stopz = Az + std::min(vi.num_frames - n - 1, Az);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pf2p = srcPF->GetPtr(b);
    uint8_t* dstp = dstPF->GetPtr(b);
    const int pitch = dstPF->GetPitch(b);
    const int height = dstPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = dstPF->GetWidth(b);
    const int widthm1 = width - 1;
    double* sumsb_saved = sumsb + Bx;
    double* weightsb_saved = weightsb + Bx;
    for (int i = 0; i < fc->size; ++i)
      pfplut[i] = fc->frames[fc->getCachePos(i)]->pf->GetPtr(b);
    for (int y = By; y < height + By; y += Byd)
    {
      const int starty = std::max(y - Ay, By);
      const int stopy = std::min(y + Ay, heightm1 - std::min(By, heightm1 - y));
      const int yTr = std::min(Byd, height - y + By);
      for (int x = Bx; x < width + Bx; x += Bxd)
      {
        memset(sumsb, 0, Bxa * sizeof(double));
        memset(weightsb, 0, Bxa * sizeof(double));
        double wmax = 0.0;
        const int startx = std::max(x - Ax, Bx);
        const int stopx = std::min(x + Ax, widthm1 - std::min(Bx, widthm1 - x));
        const int xTr = std::min(Bxd, width - x + Bx);
        for (int z = startz; z <= stopz; ++z)
        {
          const uint8_t* pf1p = pfplut[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int yT = -std::min(std::min(Sy, u), y);
            const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
            const int yBb = std::min(std::min(By, heightm1 - u), heightm1 - y);
            const uint8_t* s1_saved = pf1p + (u + yT) * pitch;
            const uint8_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const uint8_t* sbp_saved = pf1p + (u - By) * pitch;
            const double* gw_saved = gw + (yT + Sy) * Sxd + Sx;
            const int pf1pl = u * pitch;
            for (int v = startx; v <= stopx; ++v)
            {
              if (z == Az && u == y && v == x) continue;
              const int xL = -std::min(std::min(Sx, v), x);
              const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
              const uint8_t* s1 = s1_saved + v;
              const uint8_t* s2 = s2_saved;
              const double* gwT = gw_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  if constexpr(SAD)
                    diff += abs(s1[k] - s2[k]) * gwT[k];
                  else
                    diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                  gweights += gwT[k];
                }
                s1 += pitch;
                s2 += pitch;
                gwT += Sxd;
              }
              double weight;
              if constexpr(SAD)
                weight = exp((diff / gweights) * hin);
              else
                weight = exp((diff / gweights) * h2in);
              const int xRb = std::min(std::min(Bx, widthm1 - v), widthm1 - x);
              const uint8_t* sbp = sbp_saved + v;
              double* sumsbT = sumsb_saved;
              double* weightsbT = weightsb_saved;
              for (int j = -By; j <= yBb; ++j)
              {
                for (int k = -Bx; k <= xRb; ++k)
                {
                  sumsbT[k] += sbp[k] * weight;
                  weightsbT[k] += weight;
                }
                sbp += pitch;
                sumsbT += Bxd;
                weightsbT += Bxd;
              }
              if (weight > wmax) wmax = weight;
            }
          }
        }
        const uint8_t* srcpT = srcp + x - Bx;
        uint8_t* dstpT = dstp + x - Bx;
        double* sumsbTr = sumsb;
        double* weightsbTr = weightsb;
        if (wmax <= DBL_EPSILON) wmax = 1.0;
        for (int j = 0; j < yTr; ++j)
        {
          for (int k = 0; k < xTr; ++k)
          {
            sumsbTr[k] += srcpT[k] * wmax;
            weightsbTr[k] += wmax;
            dstpT[k] = std::max(std::min(int((sumsbTr[k] / weightsbTr[k]) + 0.5), 255), 0);
          }
          srcpT += pitch;
          dstpT += pitch;
          sumsbTr += Bxd;
          weightsbTr += Bxd;
        }
      }
      dstp += pitch * Byd;
      srcp += pitch * Byd;
    }
  }
  _aligned_free(pfplut);
  PVideoFrame dst = env->NewVideoFrame(vi);
  dstPF->copyTo(dst, vi);
  return dst;
}

template<bool SAD>
PVideoFrame __stdcall TNLMeans::GetFrameWOZ(int n, IScriptEnvironment* env)
{
  PVideoFrame src = child->GetFrame(mapn(n), env);
  srcPFr->copyFrom(src, vi);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPFr->GetPtr(b);
    const uint8_t* pfp = srcPFr->GetPtr(b);
    uint8_t* dstp = dstPF->GetPtr(b);
    const int pitch = dstPF->GetPitch(b);
    const int height = dstPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = dstPF->GetWidth(b);
    const int widthm1 = width - 1;
    memset(ds->sums, 0, height * width * sizeof(double));
    memset(ds->weights, 0, height * width * sizeof(double));
    memset(ds->wmaxs, 0, height * width * sizeof(double));
    for (int y = 0; y < height; ++y)
    {
      const int stopy = std::min(y + Ay, heightm1);
      const int doffy = y * width;
      for (int x = 0; x < width; ++x)
      {
        const int startxt = std::max(x - Ax, 0);
        const int stopx = std::min(x + Ax, widthm1);
        const int doff = doffy + x;
        double* dsum = &ds->sums[doff];
        double* dweight = &ds->weights[doff];
        double* dwmax = &ds->wmaxs[doff];
        for (int u = y; u <= stopy; ++u)
        {
          const int startx = u == y ? x + 1 : startxt;
          const int yT = -std::min(std::min(Sy, u), y);
          const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
          const uint8_t* s1_saved = pfp + (u + yT) * pitch;
          const uint8_t* s2_saved = pfp + (y + yT) * pitch + x;
          const double* gw_saved = gw + (yT + Sy) * Sxd + Sx;
          const int pfpl = u * pitch;
          const int coffy = u * width;
          for (int v = startx; v <= stopx; ++v)
          {
            const int coff = coffy + v;
            double* csum = &ds->sums[coff];
            double* cweight = &ds->weights[coff];
            double* cwmax = &ds->wmaxs[coff];
            const int xL = -std::min(std::min(Sx, v), x);
            const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
            const uint8_t* s1 = s1_saved + v;
            const uint8_t* s2 = s2_saved;
            const double* gwT = gw_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                if constexpr(SAD)
                  diff += abs(s1[k] - s2[k]) * gwT[k];
                else
                  diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                gweights += gwT[k];
              }
              s1 += pitch;
              s2 += pitch;
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
        dstp[x] = std::max(std::min(int(((*dsum) / (*dweight)) + 0.5), 255), 0);
      }
      dstp += pitch;
      srcp += pitch;
    }
  }
  PVideoFrame dst = env->NewVideoFrame(vi);
  dstPF->copyTo(dst, vi);
  return dst;
}

template<bool SAD>
PVideoFrame __stdcall TNLMeans::GetFrameWOZB(int n, IScriptEnvironment* env)
{
  PVideoFrame src = child->GetFrame(mapn(n), env);
  srcPFr->copyFrom(src, vi);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPFr->GetPtr(b);
    const uint8_t* pfp = srcPFr->GetPtr(b);
    uint8_t* dstp = dstPF->GetPtr(b);
    const int pitch = dstPF->GetPitch(b);
    const int height = dstPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = dstPF->GetWidth(b);
    const int widthm1 = width - 1;
    double* sumsb_saved = sumsb + Bx;
    double* weightsb_saved = weightsb + Bx;
    for (int y = By; y < height + By; y += Byd)
    {
      const int starty = std::max(y - Ay, By);
      const int stopy = std::min(y + Ay, heightm1 - std::min(By, heightm1 - y));
      const int yTr = std::min(Byd, height - y + By);
      for (int x = Bx; x < width + Bx; x += Bxd)
      {
        memset(sumsb, 0, Bxa * sizeof(double));
        memset(weightsb, 0, Bxa * sizeof(double));
        double wmax = 0.0;
        const int startx = std::max(x - Ax, Bx);
        const int stopx = std::min(x + Ax, widthm1 - std::min(Bx, widthm1 - x));
        const int xTr = std::min(Bxd, width - x + Bx);
        for (int u = starty; u <= stopy; ++u)
        {
          const int yT = -std::min(std::min(Sy, u), y);
          const int yB = std::min(std::min(Sy, heightm1 - u), heightm1 - y);
          const int yBb = std::min(std::min(By, heightm1 - u), heightm1 - y);
          const uint8_t* s1_saved = pfp + (u + yT) * pitch;
          const uint8_t* s2_saved = pfp + (y + yT) * pitch + x;
          const uint8_t* sbp_saved = pfp + (u - By) * pitch;
          const double* gw_saved = gw + (yT + Sy) * Sxd + Sx;
          for (int v = startx; v <= stopx; ++v)
          {
            if (u == y && v == x) continue;
            const int xL = -std::min(std::min(Sx, v), x);
            const int xR = std::min(std::min(Sx, widthm1 - v), widthm1 - x);
            const uint8_t* s1 = s1_saved + v;
            const uint8_t* s2 = s2_saved;
            const double* gwT = gw_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                if constexpr(SAD)
                  diff += abs(s1[k] - s2[k]) * gwT[k];
                else
                  diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwT[k];
                gweights += gwT[k];
              }
              s1 += pitch;
              s2 += pitch;
              gwT += Sxd;
            }
            double weight;
            if constexpr(SAD)
              weight = exp((diff / gweights) * hin);
            else
              weight = exp((diff / gweights) * h2in);
            const int xRb = std::min(std::min(Bx, widthm1 - v), widthm1 - x);
            const uint8_t* sbp = sbp_saved + v;
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
              sbp += pitch;
            }
            if (weight > wmax) wmax = weight;
          }
        }
        const uint8_t* srcpT = srcp + x - Bx;
        uint8_t* dstpT = dstp + x - Bx;
        double* sumsbTr = sumsb;
        double* weightsbTr = weightsb;
        if (wmax <= DBL_EPSILON) wmax = 1.0;
        for (int j = 0; j < yTr; ++j)
        {
          for (int k = 0; k < xTr; ++k)
          {
            sumsbTr[k] += srcpT[k] * wmax;
            weightsbTr[k] += wmax;
            dstpT[k] = std::max(std::min(int((sumsbTr[k] / weightsbTr[k]) + 0.5), 255), 0);
          }
          srcpT += pitch;
          dstpT += pitch;
          sumsbTr += Bxd;
          weightsbTr += Bxd;
        }
      }
      dstp += pitch * Byd;
      srcp += pitch * Byd;
    }
  }
  PVideoFrame dst = env->NewVideoFrame(vi);
  dstPF->copyTo(dst, vi);
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
  ds = nullptr;
  dsa = nullptr;
}

nlFrame::nlFrame(bool _useblocks, int _size, VideoInfo& vi, int cpuFlags)
{
  fnum = -20;
  pf = new PlanarFrame(vi, cpuFlags);
  ds = nullptr;
  dsa = nullptr;
  if (!_useblocks)
  {
    ds = (SDATA**)malloc(3 * sizeof(SDATA*));
    for (int i = 0; i < 3; ++i)
    {
      ds[i] = new SDATA();
      ds[i]->sums = (double*)_aligned_malloc(pf->GetHeight(i) * pf->GetWidth(i) * sizeof(double), 16);
      ds[i]->weights = (double*)_aligned_malloc(pf->GetHeight(i) * pf->GetWidth(i) * sizeof(double), 16);
      ds[i]->wmaxs = (double*)_aligned_malloc(pf->GetHeight(i) * pf->GetWidth(i) * sizeof(double), 16);
    }
    dsa = (int*)malloc(_size * sizeof(int));
    for (int i = 0; i < _size; ++i) dsa[i] = 0;
  }
}

nlFrame::~nlFrame()
{
  if (pf) delete pf;
  if (ds)
  {
    for (int i = 0; i < 3; ++i)
    {
      _aligned_free(ds[i]->sums);
      _aligned_free(ds[i]->weights);
      _aligned_free(ds[i]->wmaxs);
      delete ds[i];
    }
    free(ds);
  }
  if (dsa) free(dsa);
}

void nlFrame::setFNum(int i)
{
  fnum = i;
}

nlCache::nlCache()
{
  start_pos = size = -20;
}

nlCache::nlCache(int _size, bool _useblocks, VideoInfo& vi, int cpuFlags)
{
  start_pos = size = -20;
  if (_size > 0)
  {
    start_pos = 0;
    size = _size;
    frames.resize(size);
    for (int i = 0; i < size; ++i)
      frames[i] = new nlFrame(_useblocks, _size, vi, cpuFlags);
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
  for (int i = 0; i < 3; ++i)
  {
    memset(nl->ds[i]->sums, 0, nl->pf->GetHeight(i) * nl->pf->GetWidth(i) * sizeof(double));
    memset(nl->ds[i]->weights, 0, nl->pf->GetHeight(i) * nl->pf->GetWidth(i) * sizeof(double));
    memset(nl->ds[i]->wmaxs, 0, nl->pf->GetHeight(i) * nl->pf->GetWidth(i) * sizeof(double));
  }
  for (int i = 0; i < size; ++i) nl->dsa[i] = 0;
}

int nlCache::getCachePos(int n)
{
  return (start_pos + n) % size;
}

AVSValue __cdecl Create_TNLMeans(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  PClip hclip;
  if (args[8].IsBool() && args[8].AsBool())
  {
    const int rm = args[9].IsInt() ? args[9].AsInt() : 4;
    if (rm < 0 || rm > 5)
      env->ThrowError("TNLMeans:  rm must be set to 0, 1, 2, 3, 4, or 5!");
    if (!args[0].IsClip())
      env->ThrowError("TNLMeans:  first argument must be a clip!");
    AVSValue argsv[4] = { args[0].AsClip(), args[0].AsClip()->GetVideoInfo().width / 2,
      args[0].AsClip()->GetVideoInfo().height / 2 };
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
  return new TNLMeans(args[0].AsClip(), args[1].AsInt(4), args[2].AsInt(4), args[3].AsInt(0),
    args[4].AsInt(2), args[5].AsInt(2), args[6].AsInt(1), args[7].AsInt(1), args[8].AsBool(false),
    args[10].AsFloat(1.0), args[11].AsFloat(usse ? 1.8f : 0.5f), usse, hclip, env);
}

/* New 2.6 requirement!!! */
// Declare and initialise server pointers static storage.
const AVS_Linkage* AVS_linkage = 0;

/* New 2.6 requirement!!! */
// DLL entry point called from LoadPlugin() to setup a user plugin.
extern "C" __declspec(dllexport) const char* __stdcall
AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors) {

  /* New 2.6 requirment!!! */
  // Save the server pointers.
  AVS_linkage = vectors;
  env->AddFunction("TNLMeans", "c[Ax]i[Ay]i[Az]i[Sx]i[Sy]i[Bx]i[By]i[ms]b[rm]i[a]f[h]f[sse]b",
    Create_TNLMeans, 0);
  return 0;
}
