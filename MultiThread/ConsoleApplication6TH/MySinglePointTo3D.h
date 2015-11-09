#include<iostream>
#include<fstream>
using namespace std;

struct BALL_3D_PARA
{
	double x;
	double y;
	double z;
};

/*输出位置信息*/
void txt_write_xyz(FILE *fp, double x, double y, double z){
	fprintf(fp, "%lf ", x);
	fprintf(fp, "%lf ", y);
	fprintf(fp, "%lf\n ", z);

}

bool MatrixInv(double *A, double *Ainv, int n)
{//矩阵求逆
	//A:输入矩阵
	//Ainv:输出逆矩阵
	//n,矩阵维数
	int i, j, k;
	double d = 1.0, temp = 0, lamda = 0;

	double *a = (double*)malloc(sizeof(double) * 2 * n*n);
	/*初始化处理*/
	for (i = 0; i<n; i++)
		for (j = 0; j<n; j++)
		{
			a[i * 2 * n + j] = A[i*n + j];
			a[i * 2 * n + n + j] = (i == j ? 1.0 : 0.0);
		}
	/*根据初等变换原理,采用列消去法*/
	for (i = 0; i<n; i++)
	{
		double aii = a[i * 2 * n + i];
		if (aii + d == d)
		{
			k = i + 1;
			while (k<n&&a[k * 2 * n + i] + d == d) k++;
			if (k == n)
			{
				free(a);
				return false;
			}
			for (j = 0; j<2 * n; j++)/*换行*/
			{
				temp = a[i * 2 * n + j];
				a[i * 2 * n + j] = a[k * 2 * n + j];
				a[k * 2 * n + j] = temp;
			}
		}
		aii = a[i * 2 * n + i];
		for (j = 0; j<2 * n; j++) a[i * 2 * n + j] /= aii;/*主行归一化*/
		for (k = 0; k<n; k++)/*列消去*/
		{
			if (k == i || a[k * 2 * n + i] + d == d) continue;
			lamda = a[k * 2 * n + i] / a[i * 2 * n + i];
			for (j = 0; j<2 * n; j++) a[k * 2 * n + j] = a[k * 2 * n + j] - lamda*a[i * 2 * n + j];
		}
	}

	for (i = 0; i<n; i++)/*传出逆矩阵*/
		for (j = 0; j<n; j++)
			Ainv[i*n + j] = a[i * 2 * n + n + j];
	free(a);
	return true;
}

bool MatrixMulti(double *A, double *B, double *C, int am, int an, int bm, int bn)
{/*矩阵相乘,A,B为输入阵,C为积的矩阵,m,n为行和列*/
	int i, j, l, u;

	if (an != bm)
	{
		return false;
	}
	for (i = 0; i<am; i++)
		for (j = 0; j<bn; j++)
		{
			u = i*bn + j;
			C[u] = 0.0;
			for (l = 0; l<an; l++)
				C[u] = C[u] + A[i*an + l] * B[l*bn + j];

			if (C[u]<0.00000001)
			{
				//	C[u]=0;
			}
		}
	return true;
}

bool MatrixTrans(double *matrixOrigin, double *matrixTrans, int row, int col)
{//矩阵转置
	int i, j;
	for (i = 0; i<col; i++)
		for (j = 0; j<row; j++)
			matrixTrans[i*row + j] = matrixOrigin[j*col + i];
	return true;
}



//畸变校正函数
double DeDistortion(
	double *DistortionOfCamera,
	double *CameraIntr,
	double u_distort,
	double v_distort,
	double &u,
	double &v)
{
	//Learning Opencv P442
	//参数说明 
	//DistortionOfCamera	相机畸变参数		1X5
	//CameraIntr			相机内参			3X4


	//double u0 = gRightIntr[0][2];     
	//double v0 = gRightIntr[1][2]; 
	//double kx = gRightIntr[0][0]; 
	//double ky = gRightIntr[1][1];
	double u0 = *(CameraIntr + 2);	// gRightIntr[0][2];	(图像坐标原点)
	double v0 = *(CameraIntr + 4 + 2);	// grightintr[1][2];	(图像坐标原点)
	double kx = *(CameraIntr);		// grightintr[0][0];	fx
	double ky = *(CameraIntr + 4 + 1);	// grightintr[1][1];	fy


	//    畸变因子
	double k1 = DistortionOfCamera[0];
	double k2 = DistortionOfCamera[1];
	double p1 = DistortionOfCamera[2];
	double p2 = DistortionOfCamera[3];
	double k3 = DistortionOfCamera[4];

	// 归一化坐标	
	double x_dist, y_dist;
	x_dist = (u_distort - u0) / kx;
	y_dist = (v_distort - v0) / ky;

	// 开始畸变补偿
	int k = 10;  // 迭代次数，实测值为3次就差不多了，可以多试凑几次

	double r2;   // x*x + y*y
	double x, y;	// 迭代变量
	double k_radial; // 径向
	double k_tang_x, k_tang_y;   // 切向

	double delta_;  // 残差
	double x0, y0;  // 每轮迭代开始时x,y的值，用于


	x = x_dist;  // 迭代初值
	y = y_dist;
	// 开始迭代
	for (int i = 0; i<k; i++)  // 迭代3次后,[x,y]的相对变化量为0.0003949
	{
		x0 = x; y0 = y;

		r2 = x*x + y*y;
		k_radial = 1 + k1 * r2 + k2 * r2*r2 + k3 * r2*r2*r2;
		k_tang_x = 2 * p1*x*y + p2*(r2 + 2 * x*x);
		k_tang_y = 2 * p2*x*y + p1*(r2 + 2 * y*y);

		x = (x_dist - k_tang_x) / k_radial;
		y = (y_dist - k_tang_y) / k_radial;

		// 如果不使用残差决定是否结束迭代，则可以不运行下面if中的代码
		// 以提高运算速度
		if (0)
		{
			double delta = sqrt((x - x0)*(x - x0) + (y - y0)*(y - y0));
			double normal = sqrt((x0)*(x0)+(y0)*(y0));
			delta_ = delta / normal;
			if (delta_ < 0.0001)
				break;
		}

	}
	delta_ = 0;
	// 输出
	u = x * kx + u0;
	v = y * ky + v0;
	return delta_;
}




bool Compute3D(
	double *RightIntr,
	double *LeftIntr,
	double *RightExtr,
	double *LeftExtr,
	double x1,
	double y1,
	double x2,
	double y2,
	double &x,
	double &y,
	double &z)
{//计算三维坐标
	//	printf("x1:%d,y1:%d,x2:%d,y2:%d\n",x1,y1,x2,y2);



	double gLeftMat[3][4];
	double gRightMat[3][4];

	double gTempMatA[4][3];
	double gTempMatAT[3][4];

	double gTempMatB[4];
	double gTempMatC[3];

	double gTempMatD[3][3];
	double gTempMatE[3][3];
	double gTempMatF[3][4];

	MatrixMulti(RightIntr, RightExtr, gRightMat[0], 3, 4, 4, 4);
	MatrixMulti(LeftIntr, LeftExtr, gLeftMat[0], 3, 4, 4, 4);

	gTempMatA[0][0] = x1*gLeftMat[2][0] - gLeftMat[0][0];
	gTempMatA[0][1] = x1*gLeftMat[2][1] - gLeftMat[0][1];
	gTempMatA[0][2] = x1*gLeftMat[2][2] - gLeftMat[0][2];
	gTempMatA[1][0] = y1*gLeftMat[2][0] - gLeftMat[1][0];
	gTempMatA[1][1] = y1*gLeftMat[2][1] - gLeftMat[1][1];
	gTempMatA[1][2] = y1*gLeftMat[2][2] - gLeftMat[1][2];
	gTempMatA[2][0] = x2*gRightMat[2][0] - gRightMat[0][0];
	gTempMatA[2][1] = x2*gRightMat[2][1] - gRightMat[0][1];
	gTempMatA[2][2] = x2*gRightMat[2][2] - gRightMat[0][2];
	gTempMatA[3][0] = y2*gRightMat[2][0] - gRightMat[1][0];
	gTempMatA[3][1] = y2*gRightMat[2][1] - gRightMat[1][1];
	gTempMatA[3][2] = y2*gRightMat[2][2] - gRightMat[1][2];

	gTempMatB[0] = gLeftMat[0][3] - x1*gLeftMat[2][3];
	gTempMatB[1] = gLeftMat[1][3] - y1*gLeftMat[2][3];
	gTempMatB[2] = gRightMat[0][3] - x2*gRightMat[2][3];
	gTempMatB[3] = gRightMat[1][3] - y2*gRightMat[2][3];

	//计算D = A^T*A
	MatrixTrans(gTempMatA[0], gTempMatAT[0], 4, 3);
	MatrixMulti(gTempMatAT[0], gTempMatA[0], gTempMatD[0], 3, 4, 4, 3);

	//计算E = D^-1
	if (!MatrixInv(gTempMatD[0], gTempMatE[0], 3))
	{
		printf("矩阵求逆失败，计算出错!\n");
		return false;
	}
	//计算F = E*A^T
	MatrixMulti(gTempMatE[0], gTempMatAT[0], gTempMatF[0], 3, 3, 3, 4);
	//计算结果C = F*B
	MatrixMulti(gTempMatF[0], gTempMatB, gTempMatC, 3, 4, 4, 1);


	//单位就是标定棋盘的边长  一般用mm
	//单位转换 mm—〉m
	//输出单位 m
	//x = gTempMatC[0]/1000.0;
	//y = gTempMatC[1]/1000.0;
	//z = gTempMatC[2]/1000.0;


	//输出单位单位 mm
	x = gTempMatC[0];
	y = gTempMatC[1];
	z = gTempMatC[2];


	//桌面中心坐标系
	//	x = gTempMatC[0] - 1226;
	//	y = gTempMatC[1] - 300;
	//	z = gTempMatC[2] +15;
	if (fabs(x) > 4 || fabs(y) > 2 || fabs(z) > 2)
	{
		//printf("Compute3D Failed! ");
		//printf("x1:%d,y1:%d,x2:%d,y2:%d\n");
		return false;
	}

	return true;
}




bool processCam(
	double *DistortionOfLeftCamera,
	double *LeftCameraIntr,
	double *LeftCameraExtr,
	double *DistortionOfRightCamera,
	double *RightCameraIntr,
	double *RightCameraExtr,
	double Left_x,
	double Left_y,
	double Right_x,
	double Right_y,
	BALL_3D_PARA *ball
	)
{
	//畸变矫正
	double x10 = Left_x;
	double y10 = Left_y;
	double x20 = Right_x;
	double y20 = Right_y;

	DeDistortion(DistortionOfLeftCamera, LeftCameraIntr, x10, y10, Left_x, Left_y);
	DeDistortion(DistortionOfRightCamera, RightCameraIntr, x20, y20, Right_x, Right_y);

	//cout << "畸变校正后的点" << endl;
	//cout<< Left_x << "	" << Left_y << endl;
	//cout << Right_x << "	" << Right_y << endl;

	BALL_3D_PARA Temp;
	//计算三维坐标
	//这个三维坐标的世界坐标系的原点是  你选定的那一组图片的坐标系（左右相机外参设定的 RT）
	Compute3D(RightCameraIntr, LeftCameraIntr, RightCameraExtr, LeftCameraExtr, Left_x, Left_y, Right_x, Right_y, Temp.x, Temp.y, Temp.z);
	//Compute3D(RightCameraIntr, LeftCameraIntr, RightCameraExtr, LeftCameraExtr, Left_x, Left_y, Right_x, Right_y, Temp.x, Temp.y, Temp.z);


	//相机倾斜的坐标转换
	ball->x = Temp.x;
	ball->y = Temp.y;
	ball->z = Temp.z;


	//ball->y = (445.0 / 482)*Temp.y + 0.3842321411*Temp.z;
	//ball->z = -0.3842321411*Temp.y + (445.0 / 482)*Temp.z;

	

	/*double sita_PI = (sita / 180)*3.141592653;
	ball->y = cos(-sita_PI)*Temp.y + sin(-sita_PI)*Temp.z;
	ball->z = -sin(-sita_PI)*Temp.y + cos(-sita_PI)*Temp.z;*/

	static ofstream fout("E:\\毕业设计\\输出坐标\\1.txt");
	cout << "x=" << Temp.x << '\t' << "y=" << Temp.y << '\t' << "z=" << Temp.z << endl;
	fout << Temp.x << '\t' << Temp.y << '\t' << Temp.z << endl;

	return true;
}