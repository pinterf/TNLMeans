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

#include <windows.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <malloc.h>
#include <float.h>
#include "avisynth.h"
#include "PlanarFrame.h"

struct SDATA
{
	double *weights;
	double *sums;
	double *wmaxs;
};

class nlFrame
{
public:
	int fnum;
	PlanarFrame *pf;
	SDATA **ds;
	int *dsa;
	nlFrame::nlFrame();
	nlFrame::nlFrame(bool _useblocks, int _size, VideoInfo &vi);
	nlFrame::~nlFrame();
	void nlFrame::setFNum(int i);
};

class nlCache
{
public:
	nlFrame **frames;
	int start_pos, size;
	nlCache::nlCache();
	nlCache::nlCache(int _size, bool _useblocks, VideoInfo &vi);
	nlCache::~nlCache();
	void nlCache::resetCacheStart(int first, int last);
	int nlCache::getCachePos(int n);
	void nlCache::clearDS(nlFrame *nl);
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
	double a, a2, h, hin, h2in, *gw;
	double *sumsb, *weightsb, *gwh;
	bool ms, sse;
	nlCache *fc, *fcfs, *fchs;
	SDATA *ds;
	PlanarFrame *dstPF, *srcPFr;
	nlFrame *nlfs, *nlhs;
	PClip hclip;
	int TNLMeans::mapn(int n);
	PVideoFrame __stdcall TNLMeans::GetFrameWZ(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameWZB(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameWOZ(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameWOZB(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameWZ_SAD(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameWZB_SAD(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameWOZ_SAD(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameWOZB_SAD(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameNT_MS(int n, IScriptEnvironment *env);
	PVideoFrame __stdcall TNLMeans::GetFrameT_MS(int n, IScriptEnvironment *env);
	void TNLMeans::MSWOZ(nlFrame *nl, const int Axi, const int Ayi, const int Sxi,
		const int Syi, const double *gwi);
	void TNLMeans::MSWOZB(nlFrame *nl, const int Axi, const int Ayi, const int Sxi,
		const int Syi, const int Bxi, const int Byi, const double *gwi);
	void TNLMeans::MSWZ(nlCache *fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
		const int Syi, const double *gwi, int n, bool hc, IScriptEnvironment *env);
	void TNLMeans::MSWZB(nlCache *fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
		const int Syi, const int Bxi, const int Byi, const double *gwi, int n, bool hc, 
		IScriptEnvironment *env);
	void TNLMeans::MSWOZ_SAD(nlFrame *nl, const int Axi, const int Ayi, const int Sxi,
		const int Syi, const double *gwi);
	void TNLMeans::MSWOZB_SAD(nlFrame *nl, const int Axi, const int Ayi, const int Sxi,
		const int Syi, const int Bxi, const int Byi, const double *gwi);
	void TNLMeans::MSWZ_SAD(nlCache *fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
		const int Syi, const double *gwi, int n, bool hc, IScriptEnvironment *env);
	void TNLMeans::MSWZB_SAD(nlCache *fci, const int Axi, const int Ayi, const int Azi, const int Sxi,
		const int Syi, const int Bxi, const int Byi, const double *gwi, int n, bool hc, 
		IScriptEnvironment *env);
	void TNLMeans::combineMSWeights(PlanarFrame *dst, nlFrame *fs, nlFrame *hs);

public:
	PVideoFrame __stdcall TNLMeans::GetFrame(int n, IScriptEnvironment *env);
	TNLMeans::TNLMeans(PClip _child, int _Ax, int _Ay, int _Az, int _Sx, int _Sy,
		int _Bx, int _By, bool _ms, double _a, double _h, bool _sse, PClip _hclip, 
		IScriptEnvironment *env);
	TNLMeans::~TNLMeans();
};