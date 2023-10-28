#ifndef LOCAL_PATH_PLANNING_H
#define LOCAL_PATH_PLANNING_H
#include <stdbool.h>
#include "main.h"

// 结构体表示点的坐标
typedef struct {
    double x;
    double y;
} Point;

// 结构体表示B样条曲线
typedef struct {
    Point* controlPoints;
    int numControlPoints;
} BSpline;

Point evaluateBSpline(const BSpline* spline, double t);
Point evaluateBezier(const Point* controlPoints, double t); 

#endif