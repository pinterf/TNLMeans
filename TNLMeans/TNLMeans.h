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

#include <math.h>
#include <malloc.h>
#include <float.h>
#include "avisynth.h"
#include "PlanarFrame.h"
#include <vector>

struct SDATA
{
  double* weights;
  double* sums;
  double* wmaxs;
};

class nlFrame
{
public:
  int fnum;
  PlanarFrame* pf;
  SDATA** ds;
  int* dsa;
  nlFrame();
  nlFrame(bool _useblocks, int _size, VideoInfo& vi, int cpuFlags);
  ~nlFrame();
  void setFNum(int i);
};

class nlCache
{
public:
  std::vector<nlFrame*> frames;
  int start_pos, size;
  nlCache();
  nlCache(int _size, bool _useblocks, VideoInfo& vi, int cpuFlags);
  ~nlCache();
  void resetCacheStart(int first, int last);
  int getCachePos(int n);
  void clearDS(nlFrame* nl);
};

class TNLMeans : public GenericVideoFilter
{
private:
  int Ax, Ay, Az;
  int Sx, Sy;
  int Bx, By;
  int Sxd, Syd, Sxa;
  int Bxd, Byd, Bxa;
  int Axd, Ayd, Axa, Azdm1;
  double a, a2, h, hin, h2in, * gw;
  double* sumsb, * weightsb, * gwh;
  bool ms, sse;
  nlCache* fc, * fcfs, * fchs;
  SDATA* ds;
  PlanarFrame* dstPF, * srcPFr;
  nlFrame* nlfs, * nlhs;
  PClip hclip;
  int mapn(int n);

  template<bool SAD>
  PVideoFrame __stdcall GetFrameWOZ(int n, IScriptEnvironment* env);

  template<bool SAD>
  PVideoFrame __stdcall GetFrameWZ(int n, IScriptEnvironment* env);

  template<bool SAD>
  PVideoFrame __stdcall GetFrameWZB(int n, IScriptEnvironment* env);

  template<bool SAD>
  PVideoFrame __stdcall GetFrameWOZB(int n, IScriptEnvironment* env);

  PVideoFrame __stdcall GetFrameNT_MS(int n, IScriptEnvironment* env);
  PVideoFrame __stdcall GetFrameT_MS(int n, IScriptEnvironment* env);

  template<bool SAD>
  void MSWOZ(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
    const int Syi, const double* gwi);

  template<bool SAD>
  void MSWOZB(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
    const int Syi, const int Bxi, const int Byi, const double* gwi);

  template<bool SAD>
  void MSWZ(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
    const int Syi, const double* gwi, int n, bool hc, IScriptEnvironment* env);

  template<bool SAD>
  void MSWZB(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
    const int Syi, const int Bxi, const int Byi, const double* gwi, int n, bool hc,
    IScriptEnvironment* env);

  void combineMSWeights(PlanarFrame* dst, nlFrame* fs, nlFrame* hs);

public:
  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
  TNLMeans(PClip _child, int _Ax, int _Ay, int _Az, int _Sx, int _Sy,
    int _Bx, int _By, bool _ms, double _a, double _h, bool _sse, PClip _hclip,
    IScriptEnvironment* env);
  ~TNLMeans();
};
