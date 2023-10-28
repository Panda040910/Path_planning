/**
  ******************************************************************************
  * @file    local_path_planning.c
  * @brief   This file provides code for the local_path_planning of Rc2024 ,
  *          the R1/R2 want to go somewhere, using this path will equit a fastest speed 
	* @author  PanDezhi  
	* @Date    2023.10.12 
  ******************************************************************************
  * @attention
	* int main()
	*{
	*  for (double t = 0.0; t <= 1.0; t += 0.1) 
	* {
	* Point target = evaluateBSpline(&spline, t);
	* Point current ;
	* double error_x = target.x - current.x;
	* double error_y = target.y - current.y;
	*   // 基于误差计算速度和舵角
	*   // 速度控制算法 
	*   // 舵角控制算法 
	*
  * }
  ******************************************************************************
  *  
  ******************************************************************************  
**/
#include "local_path_planning.h"
#include "main.h"
#include <stdlib.h>          // for qsort
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

Point controlPoints[] = {{0.0, 0.0}, {1.0, 1.0}, {2.0, -1.0}, {3.0, 1.0}};//坐标点
BSpline spline = {controlPoints, 4};                                      //(控制点，控制点个数)

// 计算n阶贝塞尔曲线上的点
Point evaluateBezier(const Point* controlPoints, double t) {
    Point* points = (Point*)malloc(spline.numControlPoints * sizeof(Point));

    for (int i = 0; i < spline.numControlPoints; i++) {
        points[i] = controlPoints[i];
    }

    for (int k = 1; k < spline.numControlPoints; k++) {
        for (int i = 0; i < spline.numControlPoints - k; i++) {
            points[i].x = (1 - t) * points[i].x + t * points[i + 1].x;
            points[i].y = (1 - t) * points[i].y + t * points[i + 1].y;
        }
    }

    Point result = points[0];
    free(points);
    return result;
}



// 计算B样条曲线上的点
Point evaluateBSpline(const BSpline* spline, double t) {
    Point result = {0.0, 0.0};

    for (int i = 0; i < spline->numControlPoints; i++) {
        double basis = pow(1 - t, spline->numControlPoints - i - 1) * pow(t, i);
        result.x += basis * spline->controlPoints[i].x;
        result.y += basis * spline->controlPoints[i].y;
    }

    return result;
}