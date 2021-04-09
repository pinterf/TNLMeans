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
#include <vector>
#include <memory>

struct SDATA
{
  std::vector<double> weights;
  std::vector<double> sums;
  std::vector<double> wmaxs;
};

class nlFrame
{
public:
  int fnum;
  PVideoFrame pf;
  std::vector<SDATA> ds;
  std::vector<int> dsa;
  nlFrame();
  nlFrame(bool _useblocks, int _size, VideoInfo& vi);
  ~nlFrame();
  void setFNum(int i);
};

class nlCache
{
public:
  std::vector<nlFrame*> frames;
  int start_pos, size;
  nlCache();
  nlCache(int _size, bool _useblocks, VideoInfo& vi);
  ~nlCache();
  void resetCacheStart(int first, int last);
  int getCachePos(int n);
  void clearDS(nlFrame* nl);
};

class TNLMeans : public GenericVideoFilter
{
private:
  bool has_at_least_v8;

  int Ax, Ay, Az; // x, y, and z (temporal) axis radii of the search window
  int Sx, Sy; // x-axis and y-axis radii of the support (similarity neighborhood) window
  int Bx, By; // x-axis and y-axis radii of the base window. 0: single pixel
  int Sxd, Syd, Sxa;
  int Bxd, Byd, Bxa;
  int Axd, Ayd, Axa, Azdm1;
  double a; // standard deviation of the gaussian used for weighting the difference calculation used for computing neighborhood similarity.
  double a2;
  double h; // Controls the strength of the filtering (blurring)
  double hin, h2in;

  std::vector<double> gw;
  std::vector<double> sumsb;
  std::vector<double> weightsb;
  std::vector<double> gwh;

  bool ms; // multiscale
  bool sse; // sum of squared errors (differences) mode

  std::unique_ptr<nlCache> fc;
  std::unique_ptr<nlCache> fcfs;
  std::unique_ptr<nlCache> fchs;
  SDATA ds;

  int pixelsize;
  int bits_per_pixel;
  int planecount;
  int* planes;

  std::unique_ptr<nlFrame> nlfs;
  std::unique_ptr<nlFrame> nlhs;

  PClip hclip;
  int mapn(int n);

  template<bool SAD, typename pixel_t>
  PVideoFrame __stdcall GetFrameWOZ(int n, IScriptEnvironment* env);

  template<bool SAD, typename pixel_t>
  PVideoFrame __stdcall GetFrameWZ(int n, IScriptEnvironment* env);

  template<bool SAD, typename pixel_t>
  PVideoFrame __stdcall GetFrameWZB(int n, IScriptEnvironment* env);

  template<bool SAD, typename pixel_t>
  PVideoFrame __stdcall GetFrameWOZB(int n, IScriptEnvironment* env);

  template<typename pixel_t>
  PVideoFrame GetFrameNT_MS(int n, IScriptEnvironment* env);
  template<typename pixel_t>
  PVideoFrame GetFrameT_MS(int n, IScriptEnvironment* env);

  template<bool SAD, typename pixel_t>
  void MSWOZ(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
    const int Syi, const double* gwi);

  template<bool SAD, typename pixel_t>
  void MSWOZB(nlFrame* nl, const int Axi, const int Ayi, const int Sxi,
    const int Syi, const int Bxi, const int Byi, const double* gwi);

  template<bool SAD, typename pixel_t>
  void MSWZ(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
    const int Syi, const double* gwi, int n, bool hc, IScriptEnvironment* env);

  template<bool SAD, typename pixel_t>
  void MSWZB(nlCache* fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
    const int Syi, const int Bxi, const int Byi, const double* gwi, int n, bool hc,
    IScriptEnvironment* env);

  template<typename pixel_t>
  void combineMSWeights(PVideoFrame* dst, nlFrame* fs, nlFrame* hs);

public:
  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override;
  TNLMeans(PClip _child, int _Ax, int _Ay, int _Az, int _Sx, int _Sy,
    int _Bx, int _By, bool _ms, double _a, double _h, bool _sse, PClip _hclip,
    IScriptEnvironment* env);
  ~TNLMeans();

  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE : 0;
  }

};
