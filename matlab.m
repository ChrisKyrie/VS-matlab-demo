function [Turn_AngleL, Turn_AngleR, Turn_AngleL_REAL, Turn_AngleR_REAL,Turn_AngleL_PID, Turn_AngleR_PID ] = fcn(x, y)
coder.extrinsic('Project1');

InputAdd1= zeros(1, 1517);
InputAdd2 = zeros(1,1517);
InputAdd1 = x;
InputAdd2 = y;
OutPutAdd1 = zeros(1, 1516);
OutPutAdd2 = zeros(1, 1516);
OutPutAdd3 = zeros(1, 1516);
OutPutAdd4 = zeros(1, 1516);
OutPutAdd5 = zeros(1, 1515);
OutPutAdd6 = zeros(1, 1515);

[OutPutAdd1, OutPutAdd2, OutPutAdd3, OutPutAdd4, OutPutAdd5, OutPutAdd6] = Project1(InputAdd1, InputAdd2);
Turn_AngleL = OutPutAdd1;
Turn_AngleR = OutPutAdd2;
Turn_AngleL_REAL = OutPutAdd3;
Turn_AngleR_REAL = OutPutAdd4;
Turn_AngleL_PID = OutPutAdd5;
Turn_AngleR_PID = OutPutAdd6;
pause(10);
