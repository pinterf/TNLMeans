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

PVideoFrame __stdcall TNLMeans::GetFrameNT_MS(int n, IScriptEnvironment* env)
{
  PVideoFrame src = child->GetFrame(n, env);
  nlfs->pf->copyFrom(src, vi);
  PVideoFrame src_h = hclip->GetFrame(n, env);
  VideoInfo vi_h = hclip->GetVideoInfo();
  nlhs->pf->copyFrom(src_h, vi_h);
  if (Bx || By)
  {
    if (sse)
    {
      MSWOZB(nlfs, 2, 2, Sx, Sy, Bx, By, gw);
      MSWOZB(nlhs, (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh);
    }
    else
    {
      MSWOZB_SAD(nlfs, 2, 2, Sx, Sy, Bx, By, gw);
      MSWOZB_SAD(nlhs, (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh);
    }
  }
  else
  {
    if (sse)
    {
      MSWOZ(nlfs, 2, 2, Sx, Sy, gw);
      MSWOZ(nlhs, (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh);
    }
    else
    {
      MSWOZ_SAD(nlfs, 2, 2, Sx, Sy, gw);
      MSWOZ_SAD(nlhs, (Ax + 1) >> 1, (Ay + 1) >> 1, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh);
    }
  }
  combineMSWeights(dstPF, nlfs, nlhs);
  PVideoFrame dst = env->NewVideoFrame(vi);
  dstPF->copyTo(dst, vi);
  return dst;
}

PVideoFrame __stdcall TNLMeans::GetFrameT_MS(int n, IScriptEnvironment* env)
{
  if (Bx || By)
  {
    if (sse)
    {
      MSWZB(fcfs, 2, 2, Az, Sx, Sy, Bx, By, gw, n, false, env);
      MSWZB(fchs, (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh, n, true, env);
    }
    else
    {
      MSWZB_SAD(fcfs, 2, 2, Az, Sx, Sy, Bx, By, gw, n, false, env);
      MSWZB_SAD(fchs, (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, (Bx + 1) >> 1, (By + 1) >> 1, gwh, n, true, env);
    }
  }
  else
  {
    if (sse)
    {
      MSWZ(fcfs, 2, 2, Az, Sx, Sy, gw, n, false, env);
      MSWZ(fchs, (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh, n, true, env);
    }
    else
    {
      MSWZ_SAD(fcfs, 2, 2, Az, Sx, Sy, gw, n, false, env);
      MSWZ_SAD(fchs, (Ax + 1) >> 1, (Ay + 1) >> 1, Az, (Sx + 1) >> 1, (Sy + 1) >> 1, gwh, n, true, env);
    }
  }
  combineMSWeights(dstPF, fcfs->frames[fcfs->getCachePos(Az)], fchs->frames[fchs->getCachePos(Az)]);
  PVideoFrame dst = env->NewVideoFrame(vi);
  dstPF->copyTo(dst, vi);
  return dst;
}

void TNLMeans::combineMSWeights(PlanarFrame* dst, nlFrame* fs, nlFrame* hs)
{
  for (int b = 0; b < 3; ++b)
  {
    uint8_t* dstp = dst->GetPtr(b);
    const int dst_pitch = dst->GetPitch(b);
    uint8_t* dstpn = dstp + dst_pitch;
    const int width = dst->GetWidth(b);
    const int height = dst->GetHeight(b);
    const uint8_t* srcp = fs->pf->GetPtr(b);
    const int src_pitch = fs->pf->GetPitch(b);
    const uint8_t* srcpn = srcp + src_pitch;
    const uint8_t* srcph = hs->pf->GetPtr(b);
    const int srch_pitch = hs->pf->GetPitch(b);
    const double* hsw = hs->ds[b]->weights;
    const double* hss = hs->ds[b]->sums;
    const double* hswm = hs->ds[b]->wmaxs;
    double* fsw = fs->ds[b]->weights;
    double* fss = fs->ds[b]->sums;
    double* fswm = fs->ds[b]->wmaxs;
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
        if (abs(srcp[x] - ph) < 6 && abs(srcp[x + 1] - ph) < 6 &&
          abs(srcpn[x] - ph) < 6 && abs(srcpn[x + 1] - ph) < 6)
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
        dstp[x] = std::max(std::min(int((fss[x] / fsw[x]) + 0.5), 255), 0);
        if (fswm[x + 1] <= DBL_EPSILON) fswm[x + 1] = 1.0;
        fsw[x + 1] += fswm[x + 1];
        fss[x + 1] += srcp[x + 1] * fswm[x + 1];
        dstp[x + 1] = std::max(std::min(int((fss[x + 1] / fsw[x + 1]) + 0.5), 255), 0);
        if (fswmn[x] <= DBL_EPSILON) fswmn[x] = 1.0;
        fswn[x] += fswmn[x];
        fssn[x] += srcpn[x] * fswmn[x];
        dstpn[x] = std::max(std::min(int((fssn[x] / fswn[x]) + 0.5), 255), 0);
        if (fswmn[x + 1] <= DBL_EPSILON) fswmn[x + 1] = 1.0;
        fswn[x + 1] += fswmn[x + 1];
        fssn[x + 1] += srcpn[x + 1] * fswmn[x + 1];
        dstpn[x + 1] = std::max(std::min(int((fssn[x + 1] / fswn[x + 1]) + 0.5), 255), 0);
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

void TNLMeans::MSWOZ(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
  const int Syi, const double* gwi)
{
  PlanarFrame* srcPF = nl->pf;
  const int Sxdi = Sxi * 2 + 1;
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pfp = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    SDATA* dds = nl->ds[b];
    memset(dds->sums, 0, height * width * sizeof(double));
    memset(dds->weights, 0, height * width * sizeof(double));
    memset(dds->wmaxs, 0, height * width * sizeof(double));
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
          const uint8_t* s1_saved = pfp + (u + yT) * pitch;
          const uint8_t* s2_saved = pfp + (y + yT) * pitch + x;
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
            const uint8_t* s1 = s1_saved + v;
            const uint8_t* s2 = s2_saved;
            const double* gwiT = gwi_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                gweights += gwiT[k];
              }
              s1 += pitch;
              s2 += pitch;
              gwiT += Sxdi;
            }
            const double weight = exp((diff / gweights) * h2in);
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

void TNLMeans::MSWOZ_SAD(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
  const int Syi, const double* gwi)
{
  PlanarFrame* srcPF = nl->pf;
  const int Sxdi = Sxi * 2 + 1;
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pfp = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    SDATA* dds = nl->ds[b];
    memset(dds->sums, 0, height * width * sizeof(double));
    memset(dds->weights, 0, height * width * sizeof(double));
    memset(dds->wmaxs, 0, height * width * sizeof(double));
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
          const uint8_t* s1_saved = pfp + (u + yT) * pitch;
          const uint8_t* s2_saved = pfp + (y + yT) * pitch + x;
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
            const uint8_t* s1 = s1_saved + v;
            const uint8_t* s2 = s2_saved;
            const double* gwiT = gwi_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                diff += abs(s1[k] - s2[k]) * gwiT[k];
                gweights += gwiT[k];
              }
              s1 += pitch;
              s2 += pitch;
              gwiT += Sxdi;
            }
            const double weight = exp((diff / gweights) * hin);
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

void TNLMeans::MSWOZB(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
  const int Syi, const int Bxi, const int Byi, const double* gwi)
{
  PlanarFrame* srcPF = nl->pf;
  const int Bydi = Byi * 2 + 1;
  const int Bxdi = Bxi * 2 + 1;
  const int Sxdi = Sxi * 2 + 1;
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pfp = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    SDATA* dds = nl->ds[b];
    memset(dds->sums, 0, height * width * sizeof(double));
    memset(dds->weights, 0, height * width * sizeof(double));
    memset(dds->wmaxs, 0, height * width * sizeof(double));
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
        double* dsum_saved = dds->sums + doff;
        double* dweight_saved = dds->weights + doff;
        double* dwmax_saved = dds->wmaxs + doff;
        for (int u = starty; u <= stopy; ++u)
        {
          const int yT = -std::min(std::min(Syi, u), y);
          const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
          const int yBb = std::min(std::min(Byi, heightm1 - u), heightm1 - y);
          const uint8_t* s1_saved = pfp + (u + yT) * pitch;
          const uint8_t* s2_saved = pfp + (y + yT) * pitch + x;
          const uint8_t* sbp_saved = pfp + (u - Byi) * pitch;
          const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
          for (int v = startx; v <= stopx; ++v)
          {
            if (u == y && v == x) continue;
            const int xL = -std::min(std::min(Sxi, v), x);
            const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
            const uint8_t* s1 = s1_saved + v;
            const uint8_t* s2 = s2_saved;
            const double* gwiT = gwi_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                gweights += gwiT[k];
              }
              s1 += pitch;
              s2 += pitch;
              gwiT += Sxdi;
            }
            const double weight = exp((diff / gweights) * h2in);
            const int xRb = std::min(std::min(Bxi, widthm1 - v), widthm1 - x);
            const uint8_t* sbp = sbp_saved + v;
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

void TNLMeans::MSWOZB_SAD(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
  const int Syi, const int Bxi, const int Byi, const double* gwi)
{
  PlanarFrame* srcPF = nl->pf;
  const int Bydi = Byi * 2 + 1;
  const int Bxdi = Bxi * 2 + 1;
  const int Sxdi = Sxi * 2 + 1;
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pfp = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    SDATA* dds = nl->ds[b];
    memset(dds->sums, 0, height * width * sizeof(double));
    memset(dds->weights, 0, height * width * sizeof(double));
    memset(dds->wmaxs, 0, height * width * sizeof(double));
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
        double* dsum_saved = dds->sums + doff;
        double* dweight_saved = dds->weights + doff;
        double* dwmax_saved = dds->wmaxs + doff;
        for (int u = starty; u <= stopy; ++u)
        {
          const int yT = -std::min(std::min(Syi, u), y);
          const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
          const int yBb = std::min(std::min(Byi, heightm1 - u), heightm1 - y);
          const uint8_t* s1_saved = pfp + (u + yT) * pitch;
          const uint8_t* s2_saved = pfp + (y + yT) * pitch + x;
          const uint8_t* sbp_saved = pfp + (u - Byi) * pitch;
          const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
          for (int v = startx; v <= stopx; ++v)
          {
            if (u == y && v == x) continue;
            const int xL = -std::min(std::min(Sxi, v), x);
            const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
            const uint8_t* s1 = s1_saved + v;
            const uint8_t* s2 = s2_saved;
            const double* gwiT = gwi_saved;
            double diff = 0.0, gweights = 0.0;
            for (int j = yT; j <= yB; ++j)
            {
              for (int k = xL; k <= xR; ++k)
              {
                diff += abs(s1[k] - s2[k]) * gwiT[k];
                gweights += gwiT[k];
              }
              s1 += pitch;
              s2 += pitch;
              gwiT += Sxdi;
            }
            const double weight = exp((diff / gweights) * hin);
            const int xRb = std::min(std::min(Bxi, widthm1 - v), widthm1 - x);
            const uint8_t* sbp = sbp_saved + v;
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

void TNLMeans::MSWZ(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
  const int Syi, const double* gwi, int n, bool hc, IScriptEnvironment* env)
{
  const int Sxdi = Sxi * 2 + 1;
  const int Azdm1i = Azi * 2;
  fci->resetCacheStart(n - Azi, n + Azi);
  for (int i = n - Azi; i <= n + Azi; ++i)
  {
    nlFrame* nl = fci->frames[fci->getCachePos(i - n + Azi)];
    if (nl->fnum != i)
    {
      if (hc) {
        PVideoFrame src_h = hclip->GetFrame(mapn(i), env);
        VideoInfo vi_h = hclip->GetVideoInfo();
        nl->pf->copyFrom(src_h, vi_h);
      }
      else {
        PVideoFrame src = child->GetFrame(mapn(i), env);
        nl->pf->copyFrom(src, vi);
      }
      nl->setFNum(i);
      fci->clearDS(nl);
    }
  }
  const uint8_t** pfplut =
    (const uint8_t**)_aligned_malloc(fci->size * sizeof(const uint8_t*), 16);
  if (!pfplut) env->ThrowError("TNLMeans:  malloc failure (pfplut)!");
  const SDATA** dslut =
    (const SDATA**)_aligned_malloc(fci->size * sizeof(SDATA*), 16);
  if (!dslut) env->ThrowError("TNLMeans:  malloc failure (dslut)!");
  int** dsalut =
    (int**)_aligned_malloc(fci->size * sizeof(int*), 16);
  if (!dsalut) env->ThrowError("TNLMeans:  malloc failure (dsalut)!");
  for (int i = 0; i < fci->size; ++i)
    dsalut[i] = fci->frames[fci->getCachePos(i)]->dsa;
  int* ddsa = dsalut[Azi];
  PlanarFrame* srcPF = fci->frames[fci->getCachePos(Azi)]->pf;
  const int startz = Azi - std::min(n, Azi);
  const int stopz = Azi + std::min(vi.num_frames - n - 1, Azi);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pf2p = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    for (int i = 0; i < fci->size; ++i)
    {
      const int pos = fci->getCachePos(i);
      pfplut[i] = fci->frames[pos]->pf->GetPtr(b);
      dslut[i] = fci->frames[pos]->ds[b];
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
        double* dsum = &dds->sums[doff];
        double* dweight = &dds->weights[doff];
        double* dwmax = &dds->wmaxs[doff];
        for (int z = startz; z <= stopz; ++z)
        {
          if (ddsa[z] == 1) continue;
          else ddsa[z] = 2;
          const int starty = (z == Azi) ? y : startyt;
          const SDATA* cds = dslut[z];
          int* cdsa = dsalut[z];
          const uint8_t* pf1p = pfplut[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int startx = (u == y && z == Azi) ? x + 1 : startxt;
            const int yT = -std::min(std::min(Syi, u), y);
            const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
            const uint8_t* s1_saved = pf1p + (u + yT) * pitch;
            const uint8_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
            const int pf1pl = u * pitch;
            const int coffy = u * width;
            for (int v = startx; v <= stopx; ++v)
            {
              const int coff = coffy + v;
              double* csum = &cds->sums[coff];
              double* cweight = &cds->weights[coff];
              double* cwmax = &cds->wmaxs[coff];
              const int xL = -std::min(std::min(Sxi, v), x);
              const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
              const uint8_t* s1 = s1_saved + v;
              const uint8_t* s2 = s2_saved;
              const double* gwiT = gwi_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                  gweights += gwiT[k];
                }
                s1 += pitch;
                s2 += pitch;
                gwiT += Sxdi;
              }
              const double weight = exp((diff / gweights) * h2in);
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
    int* cdsa = fci->frames[fci->getCachePos(i)]->dsa;
    if (ddsa[i] == 2) ddsa[i] = cdsa[j] = 1;
  }
  _aligned_free(dsalut);
  _aligned_free(dslut);
  _aligned_free(pfplut);
}

void TNLMeans::MSWZ_SAD(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
  const int Syi, const double* gwi, int n, bool hc, IScriptEnvironment* env)
{
  const int Sxdi = Sxi * 2 + 1;
  const int Azdm1i = Azi * 2;
  fci->resetCacheStart(n - Azi, n + Azi);
  for (int i = n - Azi; i <= n + Azi; ++i)
  {
    nlFrame* nl = fci->frames[fci->getCachePos(i - n + Azi)];
    if (nl->fnum != i)
    {
      if (hc) {
        PVideoFrame src_h = hclip->GetFrame(mapn(i), env);
        VideoInfo vi_h = hclip->GetVideoInfo();
        nl->pf->copyFrom(src_h, vi_h);
      }
      else {
        PVideoFrame src = child->GetFrame(mapn(i), env);
        nl->pf->copyFrom(src, vi);
      }
      nl->setFNum(i);
      fci->clearDS(nl);
    }
  }
  const uint8_t** pfplut =
    (const uint8_t**)_aligned_malloc(fci->size * sizeof(const uint8_t*), 16);
  if (!pfplut) env->ThrowError("TNLMeans:  malloc failure (pfplut)!");
  const SDATA** dslut =
    (const SDATA**)_aligned_malloc(fci->size * sizeof(SDATA*), 16);
  if (!dslut) env->ThrowError("TNLMeans:  malloc failure (dslut)!");
  int** dsalut =
    (int**)_aligned_malloc(fci->size * sizeof(int*), 16);
  if (!dsalut) env->ThrowError("TNLMeans:  malloc failure (dsalut)!");
  for (int i = 0; i < fci->size; ++i)
    dsalut[i] = fci->frames[fci->getCachePos(i)]->dsa;
  int* ddsa = dsalut[Azi];
  PlanarFrame* srcPF = fci->frames[fci->getCachePos(Azi)]->pf;
  const int startz = Azi - std::min(n, Azi);
  const int stopz = Azi + std::min(vi.num_frames - n - 1, Azi);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pf2p = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    for (int i = 0; i < fci->size; ++i)
    {
      const int pos = fci->getCachePos(i);
      pfplut[i] = fci->frames[pos]->pf->GetPtr(b);
      dslut[i] = fci->frames[pos]->ds[b];
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
        double* dsum = &dds->sums[doff];
        double* dweight = &dds->weights[doff];
        double* dwmax = &dds->wmaxs[doff];
        for (int z = startz; z <= stopz; ++z)
        {
          if (ddsa[z] == 1) continue;
          else ddsa[z] = 2;
          const int starty = (z == Azi) ? y : startyt;
          const SDATA* cds = dslut[z];
          int* cdsa = dsalut[z];
          const uint8_t* pf1p = pfplut[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int startx = (u == y && z == Azi) ? x + 1 : startxt;
            const int yT = -std::min(std::min(Syi, u), y);
            const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
            const uint8_t* s1_saved = pf1p + (u + yT) * pitch;
            const uint8_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
            const int pf1pl = u * pitch;
            const int coffy = u * width;
            for (int v = startx; v <= stopx; ++v)
            {
              const int coff = coffy + v;
              double* csum = &cds->sums[coff];
              double* cweight = &cds->weights[coff];
              double* cwmax = &cds->wmaxs[coff];
              const int xL = -std::min(std::min(Sxi, v), x);
              const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
              const uint8_t* s1 = s1_saved + v;
              const uint8_t* s2 = s2_saved;
              const double* gwiT = gwi_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  diff += abs(s1[k] - s2[k]) * gwiT[k];
                  gweights += gwiT[k];
                }
                s1 += pitch;
                s2 += pitch;
                gwiT += Sxdi;
              }
              const double weight = exp((diff / gweights) * hin);
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
    int* cdsa = fci->frames[fci->getCachePos(i)]->dsa;
    if (ddsa[i] == 2) ddsa[i] = cdsa[j] = 1;
  }
  _aligned_free(dsalut);
  _aligned_free(dslut);
  _aligned_free(pfplut);
}

void TNLMeans::MSWZB(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
  const int Syi, const int Bxi, const int Byi, const double* gwi, int n,
  bool hc, IScriptEnvironment* env)
{
  const int Bydi = Byi * 2 + 1;
  const int Bxdi = Bxi * 2 + 1;
  const int Sxdi = Sxi * 2 + 1;
  fci->resetCacheStart(n - Azi, n + Azi);
  for (int i = n - Azi; i <= n + Azi; ++i)
  {
    nlFrame* nl = fci->frames[fci->getCachePos(i - n + Azi)];
    if (nl->fnum != i)
    {
      if (hc) {
        PVideoFrame src_h = hclip->GetFrame(mapn(i), env);
        VideoInfo vi_h = hclip->GetVideoInfo();
        nl->pf->copyFrom(src_h, vi_h);
      }
      else {
        PVideoFrame src = child->GetFrame(mapn(i), env);
        nl->pf->copyFrom(src, vi);
      }
      nl->setFNum(i);
    }
  }
  const uint8_t** pfplut =
    (const uint8_t**)_aligned_malloc(fci->size * sizeof(const uint8_t*), 16);
  if (!pfplut) env->ThrowError("TNLMeans:  malloc failure (pfplut)!");
  PlanarFrame* srcPF = fci->frames[fci->getCachePos(Azi)]->pf;
  const int startz = Azi - std::min(n, Azi);
  const int stopz = Azi + std::min(vi.num_frames - n - 1, Azi);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pf2p = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    for (int i = 0; i < fci->size; ++i)
      pfplut[i] = fci->frames[fci->getCachePos(i)]->pf->GetPtr(b);
    SDATA* dds = fci->frames[fci->getCachePos(Azi)]->ds[b];
    memset(dds->sums, 0, height * width * sizeof(double));
    memset(dds->weights, 0, height * width * sizeof(double));
    memset(dds->wmaxs, 0, height * width * sizeof(double));
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
        double* dsum_saved = dds->sums + doff;
        double* dweight_saved = dds->weights + doff;
        double* dwmax_saved = dds->wmaxs + doff;
        for (int z = startz; z <= stopz; ++z)
        {
          const uint8_t* pf1p = pfplut[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int yT = -std::min(std::min(Syi, u), y);
            const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
            const int yBb = std::min(std::min(Byi, heightm1 - u), heightm1 - y);
            const uint8_t* s1_saved = pf1p + (u + yT) * pitch;
            const uint8_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const uint8_t* sbp_saved = pf1p + (u - Byi) * pitch;
            const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
            const int pf1pl = u * pitch;
            for (int v = startx; v <= stopx; ++v)
            {
              if (z == Azi && u == y && v == x) continue;
              const int xL = -std::min(std::min(Sxi, v), x);
              const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
              const uint8_t* s1 = s1_saved + v;
              const uint8_t* s2 = s2_saved;
              const double* gwiT = gwi_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  diff += (s1[k] - s2[k]) * (s1[k] - s2[k]) * gwiT[k];
                  gweights += gwiT[k];
                }
                s1 += pitch;
                s2 += pitch;
                gwiT += Sxdi;
              }
              const double weight = exp((diff / gweights) * h2in);
              const int xRb = std::min(std::min(Bxi, widthm1 - v), widthm1 - x);
              const uint8_t* sbp = sbp_saved + v;
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
      }
      srcp += pitch * Bydi;
    }
  }
  _aligned_free(pfplut);
}

void TNLMeans::MSWZB_SAD(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
  const int Syi, const int Bxi, const int Byi, const double* gwi, int n,
  bool hc, IScriptEnvironment* env)
{
  const int Bydi = Byi * 2 + 1;
  const int Bxdi = Bxi * 2 + 1;
  const int Sxdi = Sxi * 2 + 1;
  fci->resetCacheStart(n - Azi, n + Azi);
  for (int i = n - Azi; i <= n + Azi; ++i)
  {
    nlFrame* nl = fci->frames[fci->getCachePos(i - n + Azi)];
    if (nl->fnum != i)
    {
      if (hc) {
        PVideoFrame src_h = hclip->GetFrame(mapn(i), env);
        VideoInfo vi_h = hclip->GetVideoInfo();
        nl->pf->copyFrom(src_h, vi_h);
      }
      else {
        PVideoFrame src = child->GetFrame(mapn(i), env);
        nl->pf->copyFrom(src, vi);
      }
      nl->setFNum(i);
    }
  }
  const uint8_t** pfplut =
    (const uint8_t**)_aligned_malloc(fci->size * sizeof(const uint8_t*), 16);
  if (!pfplut) env->ThrowError("TNLMeans:  malloc failure (pfplut)!");
  PlanarFrame* srcPF = fci->frames[fci->getCachePos(Azi)]->pf;
  const int startz = Azi - std::min(n, Azi);
  const int stopz = Azi + std::min(vi.num_frames - n - 1, Azi);
  for (int b = 0; b < 3; ++b)
  {
    const uint8_t* srcp = srcPF->GetPtr(b);
    const uint8_t* pf2p = srcPF->GetPtr(b);
    const int pitch = srcPF->GetPitch(b);
    const int height = srcPF->GetHeight(b);
    const int heightm1 = height - 1;
    const int width = srcPF->GetWidth(b);
    const int widthm1 = width - 1;
    for (int i = 0; i < fci->size; ++i)
      pfplut[i] = fci->frames[fci->getCachePos(i)]->pf->GetPtr(b);
    SDATA* dds = fci->frames[fci->getCachePos(Azi)]->ds[b];
    memset(dds->sums, 0, height * width * sizeof(double));
    memset(dds->weights, 0, height * width * sizeof(double));
    memset(dds->wmaxs, 0, height * width * sizeof(double));
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
        double* dsum_saved = dds->sums + doff;
        double* dweight_saved = dds->weights + doff;
        double* dwmax_saved = dds->wmaxs + doff;
        for (int z = startz; z <= stopz; ++z)
        {
          const uint8_t* pf1p = pfplut[z];
          for (int u = starty; u <= stopy; ++u)
          {
            const int yT = -std::min(std::min(Syi, u), y);
            const int yB = std::min(std::min(Syi, heightm1 - u), heightm1 - y);
            const int yBb = std::min(std::min(Byi, heightm1 - u), heightm1 - y);
            const uint8_t* s1_saved = pf1p + (u + yT) * pitch;
            const uint8_t* s2_saved = pf2p + (y + yT) * pitch + x;
            const uint8_t* sbp_saved = pf1p + (u - Byi) * pitch;
            const double* gwi_saved = gwi + (yT + Syi) * Sxdi + Sxi;
            const int pf1pl = u * pitch;
            for (int v = startx; v <= stopx; ++v)
            {
              if (z == Azi && u == y && v == x) continue;
              const int xL = -std::min(std::min(Sxi, v), x);
              const int xR = std::min(std::min(Sxi, widthm1 - v), widthm1 - x);
              const uint8_t* s1 = s1_saved + v;
              const uint8_t* s2 = s2_saved;
              const double* gwiT = gwi_saved;
              double diff = 0.0, gweights = 0.0;
              for (int j = yT; j <= yB; ++j)
              {
                for (int k = xL; k <= xR; ++k)
                {
                  diff += abs(s1[k] - s2[k]) * gwiT[k];
                  gweights += gwiT[k];
                }
                s1 += pitch;
                s2 += pitch;
                gwiT += Sxdi;
              }
              const double weight = exp((diff / gweights) * hin);
              const int xRb = std::min(std::min(Bxi, widthm1 - v), widthm1 - x);
              const uint8_t* sbp = sbp_saved + v;
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
      }
      srcp += pitch * Bydi;
    }
  }
  _aligned_free(pfplut);
}
