/*此版本加入PID算法*/
#include "mex.h"
#include "math.h"

#define CAR_LENGTH 5
#define CAR_WITH   2.5
#define PID_Kp  0.2;
#define PID_Ki  0.012;
#define PID_Kd 0.2


int ncols;

struct _PID//定义PID算法所需要的参数结构体
{
	double PID_ACTUAL_VALUE;
	double PID_SET_VALUE;
	double Kp;
	double Ki;
	double Kd;
	double ERR;
	double LAST_ERR;
	double UP_ERR;
	double INCREASE_VALUE;
}PID;

void PID_Init()//PID结构参数的初始化
{
	PID.PID_ACTUAL_VALUE = 0;
	PID.PID_SET_VALUE = 0;
	PID.ERR = 0;
	PID.LAST_ERR = 0;
	PID.UP_ERR = 0;
	PID.INCREASE_VALUE = 0;
	PID.Kp = PID_Kp;
	PID.Ki = PID_Ki;
	PID.Kd = PID_Kd;
}

double PID_Algorithm(double ACTUAL_VALUE, double SET_VALUE)//PID算法，增量型PID算法
{
	PID.PID_ACTUAL_VALUE = ACTUAL_VALUE;//PID.PID_ACTUAL_VALUE 为GPS测量的实际位置
	PID.PID_SET_VALUE = SET_VALUE;
	PID.ERR = PID.PID_SET_VALUE - PID.PID_ACTUAL_VALUE;
	PID.INCREASE_VALUE = PID.Kp*(PID.ERR - PID.LAST_ERR) + PID.Ki*PID.ERR + PID.Kd*(PID.ERR - 2 * PID.LAST_ERR + PID.UP_ERR);
	PID.PID_ACTUAL_VALUE = ACTUAL_VALUE + PID.INCREASE_VALUE;
	PID.LAST_ERR = PID.ERR;
	PID.UP_ERR = PID.LAST_ERR;
	return PID.PID_ACTUAL_VALUE;
}

void cal(double x[], double y[], double Turn_AngleL[], double Turn_AngleR[], double Turn_AngleL_REAL[], double Turn_AngleR_REAL[], double Turn_AngleL_PID[], double Turn_AngleR_PID[])
{
	int i, j;
	Turn_AngleL_REAL[0] = 0;
	Turn_AngleR_REAL[0] = 0;
	Turn_AngleL[0] = 0;
	Turn_AngleR[0] = 0;
	for (i = 0; i < ncols-1; i++)
	{
		Turn_AngleL_REAL[i + 1] = atan((x[i + 1] - x[i]) / (y[i + 1] - y[i]));    //实际左轮转角
		Turn_AngleL[i+1] = Turn_AngleL_REAL[i + 1] - Turn_AngleL_REAL[i];           //左轮应该转的角度

//		for (j = 0; j > 0; j--)//每次得到下一个目标点的转角值，采用PID算法
//		{
			Turn_AngleL_PID[i] = PID_Algorithm(Turn_AngleL[i], Turn_AngleL[i + 1]);
//		}
		
		Turn_AngleR_REAL[i + 1] = atan(CAR_LENGTH / (2 * CAR_WITH + CAR_LENGTH / tan(Turn_AngleL_REAL[i+1])));//实际右轮转角
		Turn_AngleR[i+1] = Turn_AngleR_REAL[i + 1] - Turn_AngleR_REAL[i];           //右轮应该转的角度
		
//		for (j = 0; j > 0; j--)
//		{
			Turn_AngleR_PID[i] = PID_Algorithm(Turn_AngleR[i], Turn_AngleR[i + 1]);
//		}
	}
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *x, *y, *Turn_AngleL, *Turn_AngleR, *Turn_AngleL_REAL, *Turn_AngleR_REAL, *Turn_AngleL_PID, *Turn_AngleR_PID;
	ncols = mxGetM(prhs[0]);

	plhs[0] = mxCreateDoubleMatrix(1, ncols, mxREAL);
	plhs[1] = mxCreateDoubleMatrix(1, ncols, mxREAL);
	plhs[2] = mxCreateDoubleMatrix(1, ncols, mxREAL);
	plhs[3] = mxCreateDoubleMatrix(1, ncols, mxREAL);
	plhs[4] = mxCreateDoubleMatrix(1, ncols-1, mxREAL);
	plhs[5] = mxCreateDoubleMatrix(1, ncols-1, mxREAL);
	
	x = mxGetPr(prhs[0]);
	y = mxGetPr(prhs[1]);

	Turn_AngleL = mxGetPr(plhs[0]);
	Turn_AngleR = mxGetPr(plhs[1]);
	Turn_AngleL_REAL = mxGetPr(plhs[2]);
	Turn_AngleR_REAL = mxGetPr(plhs[3]);
	Turn_AngleL_PID = mxGetPr(plhs[4]);
	Turn_AngleR_PID = mxGetPr(plhs[5]);

	PID_Init();

	cal(x, y, Turn_AngleL, Turn_AngleR, Turn_AngleL_REAL, Turn_AngleR_REAL, Turn_AngleL_PID, Turn_AngleR_PID);
}
