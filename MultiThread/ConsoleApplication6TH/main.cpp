#include <stdio.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include<iostream>
#include "MySinglePointTo3D.h"  //添加单点三维重建函数
#include"ChangeToBinary.h"
#include "math.h"
#include<time.h>
#include<cmath>
#include<Windows.h>

#define WIDTH 640 // 圖像寬度
#define HEIGHT 480 // 高度
#define THRESHOLD_MAX_VALUE 255 //二值化最大取值

using namespace std;
using namespace cv;






double cax2;
double cay2;
double cax1;
double cay1;
double TemP2;
double TemP1;
double AL[2];
double AR[2];
int Stime = clock();




void Matrix_multiplication(double R[3][3], BALL_3D_PARA *Camera, BALL_3D_PARA *quanju)
{
	quanju->x = Camera->x*R[0][0] + Camera->y*R[0][1] + Camera->z*R[0][2];
	quanju->y = Camera->x*R[1][0] + Camera->y*R[1][1] + Camera->z*R[1][2];
	quanju->z = Camera->x*R[2][0] + Camera->y*R[2][1] + Camera->z*R[2][2];
}


#define Deta_X 0
#define Deta_Y 0
#define Deta_Z 0
void Matrix_multiplication_T(double R[3][3], BALL_3D_PARA *Camera, BALL_3D_PARA *quanju)
{
	quanju->x = Camera->x*R[0][0] + Camera->y*R[0][1] + Camera->z*R[0][2] + Deta_X;
	quanju->y = Camera->x*R[1][0] + Camera->y*R[1][1] + Camera->z*R[1][2] + Deta_Y;
	quanju->z = Camera->x*R[2][0] + Camera->y*R[2][1] + Camera->z*R[2][2] + Deta_Z;
}



void GetDistance(BALL_3D_PARA *A, BALL_3D_PARA *B)
{

	double temp = sqrt(
		(A->x - B->x)*(A->x - B->x) +
		(A->y - B->y)*(A->y - B->y) +
		(A->z - B->z)*(A->z - B->z)
		);
}


//500相机参6
//左摄像机内参
//形式：
//	fx	0	Cx	0
//	0	fy	Cy	0
//	0	0	1	0
double gLeftIntr[3][4] =
{
	648.354171433213, 0, 362.712495843168, 0,
	0, 645.928168515814, 226.741397232047, 0,
	0, 0, 1, 0
};

//左摄像机外参
//形式：[R T]
//R[3][3]为matlab R2014a中的
//T[3][1]为matlab R2014a中的
//	a	b	c	Tx
//	d	e	f	Ty
//	g	h	i	Tz
//	0	0	0	1
//要转换为Opencv的定义
//直接使用Matlab中的RT矩阵
double gLeftExtr[4][4] =
{
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1
};

//左摄像机畸变
double DistortionLeft[5] =		//畸变系数	k1,k2,p1,p2,k3
{ -0.415926547893560, 0.257381157057591, -0.00205223383922628, 5.10215779541327e-05, -0.102010405903885 };




//右摄像机内参
//形式：
//	fx	0	Cx	0
//	0	fy	Cy	0
//	0	0	1	0
double gRightIntr[3][4] =
{
	645.667592039445, 0, 326.562206869635, 0,
	0, 642.973319598377, 212.836540867544, 0,
	0, 0, 1, 0
};

//右摄像机外参
//形式：[R T]
//R[3][3]为matlab R2014a中的
//T[3][1]为matlab R2014a中的
//	a	b	c	Tx
//	d	e	f	Ty
//	g	h	i	Tz
//	0	0	0	1
double gRightExtr[4][4] =
{
	0.999636233513811, -0.0269657216483252, -0.000500502054275144, 94.2730361945096,
	0.0269645545397994, 0.999633951587760, -0.00220808322105509, 0.312816574905889,
	0.000559861403808164, 0.00219378417944077, 0.999997436929807, 0.0657973362464453,
	0.0, 0.0, 0.0, 1.0
};
//右摄像机畸变
double DistortionRight[5] =
{ -0.399600055815944, 0.162620196903043, -0.00298672861475292, 0.000424588875733097, 0.0621465858113033 };


CvCapture* capture2 = cvCreateCameraCapture(1);
CvCapture* capture1 = cvCreateCameraCapture(0);



int key;


// 圖像生成
IplImage *backgroundImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1); // 背景画像用IplImage
IplImage *grayImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);  // gray画像用IplImage
IplImage *differenceImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1); // 差分画像用IplImage

IplImage *backgroundImage1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1); // 背景画像用IplImage
IplImage *grayImage1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);  // gray画像用IplImage
IplImage *differenceImage1 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1); // 差分画像用IplImage

//图像队列
#define NUMOFLIST 30
IplImage *Limage[NUMOFLIST];
IplImage *Rimage[NUMOFLIST];

bool Lbool[NUMOFLIST];
bool Rbool[NUMOFLIST];

int StartL = 0;
int StartR = 0;

int EndL = 0;
int EndR = 0;


int H = 0;
int T = 0;


bool LOK = false;
bool ROK = false;

int nL = 0;
int nR = 0;

static char windowNameCapture2[] = "Capture2";
static char windowNameDifference2[] = "Difference2";

static char windowNameCapture1[] = "Capture1";
static char windowNameDifference1[] = "Difference1";

DWORD WINAPI CollectImageL(LPVOID LP)
{
while (true)
{
	cout << clock() << endl;
		//if (!Lbool[StartL])
		{
			Limage[StartL] = cvQueryFrame(capture2);
			cvShowImage(windowNameCapture2, Limage[StartL]);
			Lbool[StartL] = true;
			StartL++;
			if (StartL >= NUMOFLIST)
			{
				StartL = 0;
			}
		}
		
		cvWaitKey(20);

	}
	
}

DWORD WINAPI CollectImageR(LPVOID LP)
{
	while (true)
	{
		cout << clock() << endl;
		//if (!Rbool[StartR])
		{
			Rimage[StartR] = cvQueryFrame(capture1);
			cvShowImage(windowNameCapture1, Rimage[StartR]);
			Rbool[StartL] = true;
			StartR++;
			if (StartR >= NUMOFLIST)
			{
				StartR = 0;
			}
		}
		
		cvWaitKey(20);
	}

	
}




DWORD WINAPI ImageHandleL(LPVOID LP)
{

	static IplImage *tempImage2 = cvCreateImage(cvSize(640, 480), 8, 1);
	while (true)
	{
		cout << clock() << endl;
		while (EndL<StartL)
		{
			cvCvtColor(Limage[EndL], tempImage2, CV_BGR2GRAY);

			cvAbsDiff(tempImage2, backgroundImage2, tempImage2);

			//  二值化處理
			cvThreshold(tempImage2, tempImage2,25, THRESHOLD_MAX_VALUE, CV_THRESH_BINARY_INV);

			IplImage *img2 = tempImage2;

			cvDilate(img2, img2, NULL, 3);
			cvErode(img2, img2, NULL, 3);
			//cvDilate(img2, img2, NULL, 10);

			cvShowImage(windowNameDifference2, img2);

			int x;
			int y;;
			double x1 = 0;
			double y1 = 0;
			nL = 0;

			for (int i = 0; i < img2->height; i++)
			{
				char *P = img2->imageData + i*img2->width;
				for (int j = 0; j < img2->width; j++)
				{
					if (P[j] == 0)
					{
						y = i;
						x = j;
						x1 += x;
						y1 += y;
						nL++;

					}
				}
			}

			AR[0] = x1 / nL;
			AR[1] = y1 / nL;
			cout << AL[0] << endl;
			cout << clock() << endl;

			Lbool[EndL] = false;
			EndL++;
			if (EndL == NUMOFLIST-1)EndL = 0;

			LOK = true;



			
		}

		cvWaitKey(1);
		
	}

}



DWORD WINAPI ImageHandleR(LPVOID LP)
{

	static IplImage *tempImage1 = cvCreateImage(cvSize(640, 480), 8, 1);
	while (true)
	{
		cout << clock() << endl;
		while (EndR<StartR)
		{
			cvCvtColor(Rimage[EndR], tempImage1, CV_BGR2GRAY);

			cvAbsDiff(tempImage1, backgroundImage1, tempImage1);

			//  二值化處理
			cvThreshold(tempImage1, tempImage1, 25, THRESHOLD_MAX_VALUE, CV_THRESH_BINARY_INV);

			IplImage *img1 = tempImage1;

			cvDilate(img1, img1, NULL, 3);
			cvErode(img1, img1, NULL, 3);
			//cvDilate(img2, img2, NULL, 10);

			cvShowImage(windowNameDifference1, img1);

			int x0;
			int y0;
			double x10 = 0;
			double y10 = 0;
			nR = 0;
			for (int i = 0; i < img1->height; i++)
			{
				char *P = img1->imageData + i*img1->width;
				for (int j = 0; j < img1->width; j++)
				{
					if (P[j] == 0)
					{
						y0 = i;
						x0 = j;
						x10 += x0;
						y10 += y0;
						nR++;

					}
				}
			}

			AL[0] = x10 / nR;
			AL[1] = y10 / nR;
		//	cout <<"右相机x像素"<< AR[0] << endl;
			cout << clock() << endl;
			Rbool[EndR] = false;
			EndR++;
			if (EndR == NUMOFLIST-1)EndR = 0;

			ROK = true;


		}
		cvWaitKey(1);
	}

}






int main(int argc, char **argv)
{
	for (int i = 0; i < NUMOFLIST; i++)
	{
		Limage[i] = cvCreateImage(cvSize(640, 480), 8, 3);
		Rimage[i] = cvCreateImage(cvSize(640, 480), 8, 3);

		Lbool[i] = 0;
		Rbool[i] = 0;

	}





	//命名窗口


	// 攝像頭初始化
	if ((capture2 = cvCreateCameraCapture(1)) == NULL || (capture1 = cvCreateCameraCapture(0)) == NULL) {
		// 無攝像頭時
		printf("NO CAMERA!\n");
		return -1;
	}
	//// 生成窗口b
	cvNamedWindow(windowNameCapture2, CV_WINDOW_AUTOSIZE);
	cvNamedWindow(windowNameDifference2, CV_WINDOW_AUTOSIZE);
	cvNamedWindow(windowNameCapture1, CV_WINDOW_AUTOSIZE);
	cvNamedWindow(windowNameDifference1, CV_WINDOW_AUTOSIZE);

	// 獲得初始圖像
	IplImage *frameImage2 = cvQueryFrame(capture2);
	IplImage *frameImage1 = cvQueryFrame(capture1);


	

	// 將初始圖像轉化為黑白圖像，并儲存為背景圖像
	cvCvtColor(frameImage2, backgroundImage2, CV_BGR2GRAY);
	cvCvtColor(frameImage1, backgroundImage1, CV_BGR2GRAY);


	BALL_3D_PARA A_position;


	//启动线程
	CreateThread(NULL, 0, CollectImageL, NULL, NULL, NULL);
	CreateThread(NULL, 0, CollectImageR, NULL, NULL, NULL);

	CreateThread(NULL, 0, ImageHandleL, NULL, NULL, NULL);
	CreateThread(NULL, 0, ImageHandleR, NULL, NULL, NULL);

	while (1)
	{
		if (LOK&&ROK)
		{
			if (nL > 500 && nR > 500)
			{
				//cout << "Time" << clock() - Stime << endl;
				processCam(
					DistortionLeft,
					gLeftIntr[0],
					gLeftExtr[0],
					DistortionRight,
					gRightIntr[0],
					gRightExtr[0],
					AL[0],
					AL[1],
					AR[0],
					AR[1],
					&A_position
					);
			}

			LOK = false;
			ROK = false;


		}
		
		key=cvWaitKey(1);


		if (key == 'b')
		{
			// 'b'取即時畫像為背景圖像
			frameImage2 = cvQueryFrame(capture2);
			cvCvtColor(frameImage2, backgroundImage2, CV_BGR2GRAY);
			frameImage1 = cvQueryFrame(capture1);
			cvCvtColor(frameImage1, backgroundImage1, CV_BGR2GRAY);


		}

		// 顯示畫像
		
		

	}



	cvReleaseCapture(&capture2);
	cvReleaseCapture(&capture1);

	// 釋放memory
	cvReleaseImage(&backgroundImage2);
	cvReleaseImage(&grayImage2);
	cvReleaseImage(&differenceImage2);
	cvReleaseImage(&backgroundImage1);
	cvReleaseImage(&grayImage1);
	cvReleaseImage(&differenceImage1);

	// 窗口銷毀
	cvDestroyWindow(windowNameCapture2);
	cvDestroyWindow(windowNameDifference2);
	cvDestroyWindow(windowNameCapture1);
	cvDestroyWindow(windowNameDifference1);
	return 0;
}