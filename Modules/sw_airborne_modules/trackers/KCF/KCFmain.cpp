#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <dirent.h>
#include "modules/computer_vision/lib/vision/image.h"
#include "kcftracker.hpp"
#include <stdio.h>

using namespace std;
using namespace cv;

//INITIAL BOUNDING BOX
int Xa;
int Ya;
int Wa;
int Ha;

//ORIGINAL
// int Xa = 50;
// int Ya = 20;
// int Wa = 20;
// int Ha = 40;

int initKey = 0;

extern "C" {
	int KCF(struct image_t* img);
}

int KCF(struct image_t* img)
{
	static int isFirstTime = 1;

	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool LAB = false;

	static KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	static Rect result;

	//CONVERT IMAGE FROM YUV422 TO BGR
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

	//TRACKER MAIN LOOP
	if (initKey)
	{
		printf("%d\n",initKey);
		if (isFirstTime)
		{
			tracker.init(Rect(Xa,Ya,Wa,Ha),image);
			isFirstTime = 0;
		}
		else
		{
			result = tracker.update(image);

			Xa = result.x;
			Ya = result.y;
			Wa = result.width;
			Ha = result.height;
		}
	}
	return 1;
}
