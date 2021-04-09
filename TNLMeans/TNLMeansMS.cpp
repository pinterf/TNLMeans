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

template<typename pixel_t>
PVideoFrame TNLMeans::GetFrameNT_MS(int n, IScriptEnvironment* env)
{
  PVideoFrame src = child->GetFrame(n, env);
  PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &src) : env->NewVideoFrame(vi); // frame property support

  nlfs->pf = src;
  PVideoFrame src_h = hclip->GetFrame(n, env);
  VideoInfo vi_h = hclip->GetVideoInfo();
  nlhs->pf = src_h; // h_clip size is half in each direction

  if (Bx || By)
  {
    if (sse)
    {
      MSWOZB<false, pixel_t>(nlfs.get(), 2, 2, Sx, Sy, Bx, By, gw.data());
      MSWOZB<false, pixel_t>(nlhs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh.data());
    }
    else
    {
      MSWOZB<true, pixel_t>(nlfs.get(), 2, 2, Sx, Sy, Bx, By, gw.data());
      MSWOZB<true, pixel_t>(nlhs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh.data());
    }
  }
  else
  {
    if (sse)
    {
      MSWOZ<false, pixel_t>(nlfs.get(), 2, 2, Sx, Sy, gw.data());
      MSWOZ<false, pixel_t>(nlhs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh.data());
    }
    else
    {
      MSWOZ<true, pixel_t>(nlfs.get(), 2, 2, Sx, Sy, gw.data());
      MSWOZ<true, pixel_t>(nlhs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh.data());
    }
  }
  combineMSWeights<pixel_t>(&dst, nlfs.get(), nlhs.get());
  return dst;
}

template<typename pixel_t>
PVideoFrame TNLMeans::GetFrameT_MS(int n, IScriptEnvironment* env)
{
  PVideoFrame dst = env->NewVideoFrame(vi);

  if (Bx || By)
  {
    if (sse)
    {
      MSWZB<false, pixel_t>(fcfs.get(), 2, 2, Az, Sx, Sy, Bx, By, gw.data(), n, false, env);
      MSWZB<false, pixel_t>(fchs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh.data(), n, true, env);
    }
    else
    {
      MSWZB<true, pixel_t>(fcfs.get(), 2, 2, Az, Sx, Sy, Bx, By, gw.data(), n, false, env);
      MSWZB<true, pixel_t>(fchs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh.data(), n, true, env);
    }
  }
  else
  {
    if (sse)
    {
      MSWZ<false, pixel_t>(fcfs.get(), 2, 2, Az, Sx, Sy, gw.data(), n, false, env);
      MSWZ<false, pixel_t>(fchs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh.data(), n, true, env);
    }
    else
    {
      MSWZ<true, pixel_t>(fcfs.get(), 2, 2, Az, Sx, Sy, gw.data(), n, false, env);
      MSWZ<true, pixel_t>(fchs.get(), (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh.data(), n, true, env);
    }
  }

  nlFrame* fs = fcfs.get()->frames[fcfs.get()->getCachePos(Az)];
  if(has_at_least_v8)
    env->copyFrameProps(fs->pf, dst);
  combineMSWeights<pixel_t>(&dst, fs, fchs.get()->frames[fchs.get()->getCachePos(Az)]);

  return dst;
}

// instantiate them
template PVideoFrame TNLMeans::GetFrameNT_MS<uint8_t>(int n, IScriptEnvironment* env);
template PVideoFrame TNLMeans::GetFrameNT_MS<uint16_t>(int n, IScriptEnvironment* env);
template PVideoFrame TNLMeans::GetFrameT_MS<uint8_t>(int n, IScriptEnvironment* env);
template PVideoFrame TNLMeans::GetFrameT_MS<uint16_t>(int n, IScriptEnvironment* env);

template<typename pixel_t>
void TNLMeans::combineMSWeights(PVideoFrame* dst, nlFrame* fs, nlFrame* hs)
{
  // PVideoFrame pointer because GetWritePtr is OK only for refcount=1
  const int SIX = sizeof(pixel_t) == 1 ? 6 : 6 << (bits_per_pixel - 8);
  const int MAX_PIXEL_VALUE = sizeof(pixel_t) == 1 ? 255 : (1 << bits_per_pixel) - 1;

  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    pixel_t* dstp = reinterpret_cast<pixel_t*>((*dst)->GetWritePtr(plane));
    const int dst_pitch = (*dst)->GetPitch(plane) / sizeof(pixel_t);
    pixel_t* dstpn = dstp + dst_pitch;
    const int width = (*dst)->GetRowSize(plane) / pixelsize;
    const int height = (*dst)->GetHeight(plane);
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(fs->pf->GetReadPtr(plane));
    const int src_pitch = fs->pf->GetPitch(plane) / sizeof(pixel_t);
    const pixel_t* srcpn = srcp + src_pitch;
    const pixel_t* srcph = reinterpret_cast<const pixel_t*>(hs->pf->GetReadPtr(plane));
    const int srch_pitch = hs->pf->GetPitch(plane) / sizeof(pixel_t);
    const double* hsw = hs->ds[b].weights.data();
    const double* hss = hs->ds[b].sums.data();
    const double* hswm = hs->ds[b].wmaxs.data();
    double* fsw = fs->ds[b].weights.data();
    double* fss = fs->ds[b].sums.data();
    double* fswm = fs->ds[b].wmaxs.data();
    double* fswn = fsw + width;
    double* fssn = fss + width;
    double* fswmn = fswm + width;

    for (int y = 0; y < height; y += 2)
    {
      for (int x = 0; x < width; x += 2)
      {
        const double hswt = hsw[x >> 1];
        const double hsst = hss[x >> 1];
        const double hswmt = hswm[x >> 1];
        const int ph = srcph[x >> 1];
        if (abs(srcp[x] - ph) < SIX && abs(srcp[x + 1] - ph) < SIX &&
          abs(srcpn[x] - ph) < SIX && abs(srcpn[x + 1] - ph) < SIX)
        {
          fsw[x] += hswt;
          fsw[x + 1] += hswt;
          fswn[x] += hswt;
          fswn[x + 1] += hswt;
          fss[x] += hsst;
          fss[x + 1] += hsst;
          fssn[x] += hsst;
          fssn[x + 1] += hsst;
          if (hswmt > fswm[x]) fswm[x] = hswmt;
          if (hswmt > fswm[x + 1]) fswm[x + 1] = hswmt;
          if (hswmt > fswmn[x]) fswmn[x] = hswmt;
          if (hswmt > fswmn[x + 1]) fswmn[x + 1] = hswmt;
        }
        if (fswm[x] <= DBL_EPSILON) fswm[x] = 1.0;
        fsw[x] += fswm[x];
        fss[x] += srcp[x] * fswm[x];
        dstp[x] = std::max(std::min(int((fss[x] / fsw[x]) + 0.5), MAX_PIXEL_VALUE), 0);
        if (fswm[x + 1] <= DBL_EPSILON) fswm[x + 1] = 1.0;
        fsw[x + 1] += fswm[x + 1];
        fss[x + 1] += srcp[x + 1] * fswm[x + 1];
        dstp[x + 1] = std::max(std::min(int((fss[x + 1] / fsw[x + 1]) + 0.5), MAX_PIXEL_VALUE), 0);
        if (fswmn[x] <= DBL_EPSILON) fswmn[x] = 1.0;
        fswn[x] += fswmn[x];
        fssn[x] += srcpn[x] * fswmn[x];
        dstpn[x] = std::max(std::min(int((fssn[x] / fswn[x]) + 0.5), MAX_PIXEL_VALUE), 0);
        if (fswmn[x + 1] <= DBL_EPSILON) fswmn[x + 1] = 1.0;
        fswn[x + 1] += fswmn[x + 1];
        fssn[x + 1] += srcpn[x + 1] * fswmn[x + 1];
        dstpn[x + 1] = std::max(std::min(int((fssn[x + 1] / fswn[x + 1]) + 0.5), MAX_PIXEL_VALUE), 0);
      }
      dstp += dst_pitch << 1;
      dstpn += dst_pitch << 1;
      srcp += src_pitch << 1;
      srcpn += src_pitch << 1;
      srcph += srch_pitch;
      hsw += width >> 1;
      hss += width >> 1;
      hswm += width >> 1;
      fsw += width << 1;
      fss += width << 1;
      fswm += width << 1;
      fswn += width << 1;
      fssn += width << 1;
      fswmn += width << 1;
    }
  }
}

template<bool SAD, typename pixel_t>
void TNLMeans::MSWOZ(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
  const int Syi, const double* gwi)
{
  PVideoFrame srcPF = nl->pf;
  const int Sxdi = Sxi * 2 + 1;

  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(srcPF->GetReadPtr(plane));
    const pixel_t* pfp = srcp;
    const int pitch = srcPF->GetPitch(plane) / sizeof(pixel_t);
    const int height = srcPF->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = srcPF->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    SDATA* dds = &nl->ds[b];
    std::fill(dds->sums.begin(), dds->sums.end(), 0.0);
    std::fill(dds->weights.begin(), dds->weights.end(), 0.0);
    std::fill(dds->wmaxs.begin(), dds->wmaxs.end(), 0.0);
    for (int y = 0; y < height; ++y)
    {
      const int stopy = std::min(y + Ayi, heightm1);
      const int doffy = y * width;
      for (int x = 0; x < width; ++x)
      {
        const int startxt = std::max(x - Axi, 0);
        const int stopx = std::min(x + Axi, widthm1);
        const int doff = doffy + x;
        double* dsum = &dds->sums[doff];
        double* dweight = &dds->weights[doff];
        double* dwmax = &dds->wmaxs[doff];
        for (int u = y; u <= stopy; ++u)
        {
          const int startx = u == y ? x + 1 : startxt;
          const int yT = -std::min(std::min(Syi, u), y);
          const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
          const pixel_t* s1_saved = pfp + (u + yT) * pitch;
          const pixel_t* s2_saved = pfp + (y + yT) * pitch + x;
          const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
          const int pfpl = u * pitch;
          const int coffy = u * width;
          for (int v = startx; v <= stopx; ++v)
          {
            const int coff = coffy + v;
            double* csum = &dds->sums[coff];
            double* cweight = &dds->weights[coff];
            double* cwmax = &dds->wmaxs[coff];
            const int xL = -std::min(std::min(Sxi, v), x);
            const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
            const pixel_t* s1 = s1_saved + v;
            const pixel_t* s2 = s2_saved;
            const double* gwiT = gwi_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                if constexpr(SAD)
                  diff += abs(s1[k] - s2[k]) * gwiT[k];
                else
                  diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                gweights += gwiT[k];
              }
              s1 += pitch;
              s2 += pitch;
              gwiT += Sxdi;
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
      }
      srcp += pitch;
    }
  }
}

template<bool SAD, typename pixel_t>
void TNLMeans::MSWOZB(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
  const int Syi, const int Bxi, const int Byi, const double* gwi)
{
  PVideoFrame srcPF = nl->pf;
  const int Bydi = Byi * 2 + 1;
  const int Bxdi = Bxi * 2 + 1;
  const int Sxdi = Sxi * 2 + 1;

  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(srcPF->GetReadPtr(plane));
    const pixel_t* pfp = srcp;
    const int pitch = srcPF->GetPitch(plane) / sizeof(pixel_t);
    const int height = srcPF->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = srcPF->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    SDATA *dds = &nl->ds[b];
    std::fill(dds->sums.begin(), dds->sums.end(), 0.0);
    std::fill(dds->weights.begin(), dds->weights.end(), 0.0);
    std::fill(dds->wmaxs.begin(), dds->wmaxs.end(), 0.0);
    for (int y = Byi; y < height + Byi; y += Bydi)
    {
      const int starty = std::max(y - Ayi, Byi);
      const int stopy = std::min(y + Ayi, heightm1 - std::min(Byi, heightm1 - y));
      const int doffy = (y - Byi) * width;
      for (int x = Bxi; x < width + Bxi; x += Bxdi)
      {
        const int startx = std::max(x - Axi, Bxi);
        const int stopx = std::min(x + Axi, widthm1 - std::min(Bxi, widthm1 - x));
        const int doff = doffy + x;
        double* dsum_saved = dds->sums.data() + doff;
        double* dweight_saved = dds->weights.data() + doff;
        double* dwmax_saved = dds->wmaxs.data() + doff;
        for (int u = starty; u <= stopy; ++u)
        {
          const int yT = -std::min(std::min(Syi, u), y);
          const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
          const int yBb = std::min(std::min(Byi, heightm1 - u), heightm1 - y);
          const pixel_t* s1_saved = pfp + (u + yT) * pitch;
          const pixel_t* s2_saved = pfp + (y + yT) * pitch + x;
          const pixel_t* sbp_saved = pfp + (u - Byi) * pitch;
          const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
          for (int v = startx; v <= stopx; ++v)
          {
            if (u == y && v == x) continue;
            const int xL = -std::min(std::min(Sxi, v), x);
            const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
            const pixel_t* s1 = s1_saved + v;
            const pixel_t* s2 = s2_saved;
            const double* gwiT = gwi_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                if constexpr(SAD)
                  diff += abs(s1[k] - s2[k]) * gwiT[k];
                else
                  diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                gweights += gwiT[k];
              }
              s1 += pitch;
              s2 += pitch;
              gwiT += Sxdi;
            }
            double weight;
            if constexpr(SAD)
              weight = exp((diff / gweights) * hin);
            else
              weight = exp((diff / gweights) * h2in);
            const int xRb = std::min(std::min(Bxi, widthm1 - v), widthm1 - x);
            const pixel_t* sbp = sbp_saved + v;
            double* dsum = dsum_saved;
            double* dweight = dweight_saved;
            double* dwmax = dwmax_saved;
            for (int j = -Byi; j <= yBb; ++j)
            {
              for (int k = -Bxi; k <= xRb; ++k)
              {
                dsum[k] += sbp[k] * weight;
                dweight[k] += weight;
                if (weight > dwmax[k]) dwmax[k] = weight;
              }
              sbp += pitch;
              dsum += width;
              dweight += width;
              dwmax += width;
            }
          }
        }
      }
      srcp += pitch * Bydi;
    }
  }
}

template<bool SAD, typename pixel_t>
void TNLMeans::MSWZ(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
  const int Syi, const double* gwi, int n, bool hc, IScriptEnvironment* env)
{
  const int Sxdi = Sxi * 2 + 1;
  const int Azdm1i = Azi * 2;

  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  fci->resetCacheStart(n - Azi, n + Azi);
  for (int i = n - Azi; i <= n + Azi; ++i)
  {
    nlFrame* nl = fci->frames[fci->getCachePos(i - n + Azi)];
    if (nl->fnum != i)
    {
      if (hc) {
        PVideoFrame src_h = hclip->GetFrame(mapn(i), env);
        VideoInfo vi_h = hclip->GetVideoInfo();
        nl->pf = src_h;
      }
      else {
        PVideoFrame src = child->GetFrame(mapn(i), env);
        nl->pf = src;
      }
      nl->setFNum(i);
      fci->clearDS(nl);
    }
  }
  std::vector<const uint8_t*> pfplut(fci->size);
  std::vector<int> pfplut_pitch(fci->size);
  std::vector<const SDATA*> dslut(fci->size);
  std::vector<int*> dsalut(fci->size);
  for (int i = 0; i < fci->size; ++i)
    dsalut[i] = fci->frames[fci->getCachePos(i)]->dsa.data();
  int* ddsa = dsalut[Azi];

  PVideoFrame srcPF = fci->frames[fci->getCachePos(Azi)]->pf;

  const int startz = Azi - std::min(n, Azi);
  const int stopz = Azi + std::min(vi.num_frames - n - 1, Azi);
  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(srcPF->GetReadPtr(plane));
    const pixel_t* pf2p = srcp;
    const int pitch = srcPF->GetPitch(plane) / sizeof(pixel_t);
    const int height = srcPF->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = srcPF->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    for (int i = 0; i < fci->size; ++i)
    {
      const int pos = fci->getCachePos(i);
      pfplut[i] = fci->frames[pos]->pf->GetReadPtr(plane);
      pfplut_pitch[i] = fci->frames[pos]->pf->GetPitch(plane) / sizeof(pixel_t);
      dslut[i] = &fci->frames[pos]->ds[b];
    }
    const SDATA* dds = dslut[Azi];
    for (int y = 0; y < height; ++y)
    {
      const int startyt = std::max(y - Ayi, 0);
      const int stopy = std::min(y + Ayi, heightm1);
      const int doffy = y * width;
      for (int x = 0; x < width; ++x)
      {
        const int startxt = std::max(x - Axi, 0);
        const int stopx = std::min(x + Axi, widthm1);
        const int doff = doffy + x;
        double* dsum = const_cast<double*>(&dds->sums[doff]);
        double* dweight = const_cast<double*>(&dds->weights[doff]);
        double* dwmax = const_cast<double*>(&dds->wmaxs[doff]);
        for (int z = startz; z <= stopz; ++z)
        {
          if (ddsa[z] == 1) continue;
          else ddsa[z] = 2;
          const int starty = (z == Azi) ? y : startyt;
          const SDATA* cds = dslut[z];
          int* cdsa = dsalut[z];
          const pixel_t* pf1p = reinterpret_cast<const pixel_t*>(pfplut[z]);
          const int pf1p_pitch = pfplut_pitch[z]; // other frame, pitch can be different
          for (int u = starty; u <= stopy; ++u)
          {
            const int startx = (u == y && z == Azi) ? x + 1 : startxt;
            const int yT = -std::min(std::min(Syi, u), y);
            const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
            const pixel_t* s1_saved = pf1p + (u + yT) * pf1p_pitch;
            const pixel_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
            const int pf1pl = u * pf1p_pitch;
            const int coffy = u * width;
            for (int v = startx; v <= stopx; ++v)
            {
              const int coff = coffy + v;
              double* csum = const_cast<double*>(&cds->sums[coff]);
              double* cweight = const_cast<double*>(&cds->weights[coff]);
              double* cwmax = const_cast<double*>(&cds->wmaxs[coff]);
              const int xL = -std::min(std::min(Sxi, v), x);
              const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
              const pixel_t* s1 = s1_saved + v;
              const pixel_t* s2 = s2_saved;
              const double* gwiT = gwi_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  if constexpr(SAD)
                    diff += abs(s1[k] - s2[k]) * gwiT[k];
                  else
                    diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                  gweights += gwiT[k];
                }
                s1 += pf1p_pitch;
                s2 += pitch;
                gwiT += Sxdi;
              }
              double weight;
              if constexpr(SAD)
                weight = exp((diff / gweights) * hin);
              else
                weight = exp((diff / gweights) * h2in);
              *dweight += weight;
              *dsum += pf1p[pf1pl + v] * weight;
              if (weight > *dwmax) *dwmax = weight;
              if (cdsa[Azdm1i - z] != 1)
              {
                *cweight += weight;
                *csum += srcp[x] * weight;
                if (weight > *cwmax) *cwmax = weight;
              }
            }
          }
        }
      }
      srcp += pitch;
    }
  }
  int j = fci->size - 1;
  for (int i = 0; i < fci->size; ++i, --j)
  {
    int* cdsa = fci->frames[fci->getCachePos(i)]->dsa.data();
    if (ddsa[i] == 2) ddsa[i] = cdsa[j] = 1;
  }
}

template<bool SAD, typename pixel_t>
void TNLMeans::MSWZB(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
  const int Syi, const int Bxi, const int Byi, const double* gwi, int n,
  bool hc, IScriptEnvironment* env)
{
  const int Bydi = Byi * 2 + 1;
  const int Bxdi = Bxi * 2 + 1;
  const int Sxdi = Sxi * 2 + 1;

  // 16 bits SSD requires int64 intermediate
  typedef typename std::conditional<sizeof(pixel_t) == 1 && !SAD, int, int64_t> ::type safeint_t;

  fci->resetCacheStart(n - Azi, n + Azi);
  for (int i = n - Azi; i <= n + Azi; ++i)
  {
    nlFrame* nl = fci->frames[fci->getCachePos(i - n + Azi)];
    if (nl->fnum != i)
    {
      if (hc) {
        PVideoFrame src_h = hclip->GetFrame(mapn(i), env);
        VideoInfo vi_h = hclip->GetVideoInfo();
        nl->pf = src_h;
      }
      else {
        PVideoFrame src = child->GetFrame(mapn(i), env);
        nl->pf = src;
      }
      nl->setFNum(i);
    }
  }

  std::vector<const uint8_t*> pfplut(fci->size);
  std::vector<int> pfplut_pitch(fci->size);

  PVideoFrame srcPF = fci->frames[fci->getCachePos(Azi)]->pf;

  const int startz = Azi - std::min(n, Azi);
  const int stopz = Azi + std::min(vi.num_frames - n - 1, Azi);
  for (int b = 0; b < planecount; ++b)
  {
    const int plane = planes[b];
    const pixel_t* srcp = reinterpret_cast<const pixel_t*>(srcPF->GetReadPtr(plane));
    const pixel_t* pf2p = srcp;
    const int pitch = srcPF->GetPitch(plane) / sizeof(pixel_t);
    const int height = srcPF->GetHeight(plane);
    const int heightm1 = height - 1;
    const int width = srcPF->GetRowSize(plane) / pixelsize;
    const int widthm1 = width - 1;
    for (int i = 0; i < fci->size; ++i) {
      pfplut[i] = fci->frames[fci->getCachePos(i)]->pf->GetReadPtr(plane);
      pfplut_pitch[i] = fci->frames[fci->getCachePos(i)]->pf->GetPitch(plane) / sizeof(pixel_t);
    }
    SDATA* dds = &fci->frames[fci->getCachePos(Azi)]->ds[b];
    std::fill(dds->sums.begin(), dds->sums.end(), 0.0);
    std::fill(dds->weights.begin(), dds->weights.end(), 0.0);
    std::fill(dds->wmaxs.begin(), dds->wmaxs.end(), 0.0);
    for (int y = Byi; y < height + Byi; y += Bydi)
    {
      const int starty = std::max(y - Ayi, Byi);
      const int stopy = std::min(y + Ayi, heightm1 - std::min(Byi, heightm1 - y));
      const int doffy = (y - Byi) * width;
      for (int x = Bxi; x < width + Bxi; x += Bxdi)
      {
        const int startx = std::max(x - Axi, Bxi);
        const int stopx = std::min(x + Axi, widthm1 - std::min(Bxi, widthm1 - x));
        const int doff = doffy + x;
        double* dsum_saved = dds->sums.data() + doff;
        double* dweight_saved = dds->weights.data() + doff;
        double* dwmax_saved = dds->wmaxs.data() + doff;
        for (int z = startz; z <= stopz; ++z)
        {
          const pixel_t* pf1p = reinterpret_cast<const pixel_t*>(pfplut[z]);
          const int pf1p_pitch = pfplut_pitch[z]; // other frame, pitch can be different
          for (int u = starty; u <= stopy; ++u)
          {
            const int yT = -std::min(std::min(Syi, u), y);
            const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
            const int yBb = std::min(std::min(Byi, heightm1 - u), heightm1 - y);
            const pixel_t* s1_saved = pf1p + (u + yT) * pf1p_pitch;
            const pixel_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const pixel_t* sbp_saved = pf1p + (u - Byi) * pf1p_pitch;
            const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
            const int pf1pl = u * pf1p_pitch;
            for (int v = startx; v <= stopx; ++v)
            {
              if (z == Azi && u == y && v == x) continue;
              const int xL = -std::min(std::min(Sxi, v), x);
              const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
              const pixel_t* s1 = s1_saved + v;
              const pixel_t* s2 = s2_saved;
              const double* gwiT = gwi_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  if constexpr(SAD)
                    diff += abs(s1[k] - s2[k]) * gwiT[k];
                  else
                    diff += (safeint_t)(s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                  gweights += gwiT[k];
                }
                s1 += pf1p_pitch;
                s2 += pitch;
                gwiT += Sxdi;
              }
              double weight;
              if constexpr(SAD)
                weight = exp((diff / gweights) * hin);
              else
                weight = exp((diff / gweights) * h2in);
              const int xRb = std::min(std::min(Bxi, widthm1 - v), widthm1 - x);
              const pixel_t* sbp = sbp_saved + v;
              double* dsum = dsum_saved;
              double* dweight = dweight_saved;
              double* dwmax = dwmax_saved;
              for (int j = -Byi; j <= yBb; ++j)
              {
                for (int k = -Bxi; k <= xRb; ++k)
                {
                  dsum[k] += sbp[k] * weight;
                  dweight[k] += weight;
                  if (weight > dwmax[k]) dwmax[k] = weight;
                }
                sbp += pf1p_pitch;
                dsum += width;
                dweight += width;
                dwmax += width;
              }
            }
          }
        }
      }
      srcp += pitch * Bydi;
    }
  }
}
