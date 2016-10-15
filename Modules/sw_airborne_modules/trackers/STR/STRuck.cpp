/*
* Struck: Structured Output Tracking with Kernels
*
* Code to accompany the paper:
*   Struck: Structured Output Tracking with Kernels
*   Sam Hare, Amir Saffari, Philip H. S. Torr
*   International Conference on Computer Vision (ICCV), 2011
*
* Copyright (C) 2011 Sam Hare, Oxford Brookes University, Oxford, UK
*
* This file is part of Struck.
*
* Struck is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Struck is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Struck.  If not, see <http://www.gnu.org/licenses/>.
*
*/

#include "Tracker.h"
#include "Config.h"

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "modules/computer_vision/lib/vision/image.h"

extern "C" {
	int STRuck(struct image_t* img);
}

using namespace std;
using namespace cv;

static const int kLiveBoxWidth = 80;
static const int kLiveBoxHeight = 80;

//INITIAL BOUNDING BOX
int Xa = 50;
int Ya = 20;
int Wa = 20;
int Ha = 40;

int initKey = 0;

void rectangle(Mat& rMat, const FloatRect& rRect, const Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, Point(r.XMin(), r.YMin()), Point(r.XMax(), r.YMax()), rColour);
}

int STRuck(struct image_t* img)
{
	static int isFirstTime = 1;
	static int isNotInit = 1;
	static Config conf;
	static bool useCamera;
	static FloatRect initBB;

	Mat image(img->h,img->w,CV_8UC3);

	uchar *bgrbuf = image.data;
	uint8_t *imgbuf = (uint8_t *) img->buf;

	for (uint16_t y = 0; y < img->h; y++) {
		for (uint16_t x = 0; x < img->w; x += 2) {
			int aux = 1.164*(imgbuf[1]-16)+2.018*(imgbuf[0]-128);
			aux = (aux > 255) ? 255 : aux;
			aux = (aux < 0) ? 0 : aux;
			bgrbuf[0] = aux;
			aux = 1.164*(imgbuf[1]-16)-0.813*(imgbuf[2]-128)-0.391*(imgbuf[0]-128);
			aux = (aux > 255) ? 255 : aux;
			aux = (aux < 0) ? 0 : aux;
			bgrbuf[1] = aux;
			bgrbuf[2] = 1.164*(imgbuf[1]-16)+1.596*(imgbuf[2]-128);
			aux = (aux > 255) ? 255 : aux;
			aux = (aux < 0) ? 0 : aux;
			bgrbuf[2] = aux;
			bgrbuf[3] = 1.164*(imgbuf[3]-16)+2.018*(imgbuf[0]-128);
			aux = (aux > 255) ? 255 : aux;
			aux = (aux < 0) ? 0 : aux;
			bgrbuf[3] = aux;
			bgrbuf[4] = 1.164*(imgbuf[3]-16)-0.813*(imgbuf[2]-128)-0.391*(imgbuf[0]-128);
			aux = (aux > 255) ? 255 : aux;
			aux = (aux < 0) ? 0 : aux;
			bgrbuf[4] = aux;
			bgrbuf[5] = 1.164*(imgbuf[3]-16)+1.596*(imgbuf[2]-128);
			aux = (aux > 255) ? 255 : aux;
			aux = (aux < 0) ? 0 : aux;
			bgrbuf[5] = aux;

			imgbuf += 4;
			bgrbuf += 6;
		}
	}

	if (isFirstTime)
	{
		conf.quietMode = true;
		conf.debugMode = false;

		conf.sequenceBasePath = "";
		conf.sequenceName = "";
		conf.resultsPath = "";

		// conf.frameWidth = 320;
		// conf.frameHeight = 240;

		conf.frameWidth = img->w;
		conf.frameHeight = img->h;

		conf.seed = 0;
		conf.searchRadius = 30;
		conf.svmC = 100.0;
		conf.svmBudgetSize = 100;

		conf.features.clear();
		Config::FeatureKernelPair fkp;
		fkp.feature = Config::kFeatureTypeHistogram;
		fkp.kernel = Config::kKernelTypeIntersection;
		conf.features.push_back(fkp);

		cout << conf << endl;

		if (conf.features.size() == 0)
		{
			cout << "error: no features specified in config" << endl;
			return EXIT_FAILURE;
		}

		// if no sequence specified then use the camera
		useCamera = (conf.sequenceName == "");

		if (useCamera)
		{
			//INITIAL BOUNDING BOX
			// initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
			initBB = IntRect(Xa,Ya,Wa,Ha);
			cout << "press 'i' to initialise tracker" << endl;
		}

		srand(conf.seed);
		isFirstTime = 0;
	}

	static Tracker tracker(conf);

	//TRACKER MAIN LOOP
	if (initKey > 30)
	{
		if (isNotInit)
		{
			tracker.Initialise(image,initBB);
			isNotInit = 0;
		}
		else
		{
			tracker.Track(image);

			Xa = tracker.GetBB().XMin();
			Ya = tracker.GetBB().YMin();
			Wa = tracker.GetBB().Width();
			Ha = tracker.GetBB().Height();
		}
	}
	else
	{
		initKey++;
	}
		return FALSE;
	}
