#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
using namespace std;

const double EARTH_RAD_EQ = 6378.137 * 1000; //unit: m
const double SCALE = cos(31.00*M_PI/180.0);
const double OFFSET_X = 41633;
const double OFFSET_Y = 3150;
const double ORIGIN_X = 11545750.7201;
const double ORIGIN_Y = 3113873.77736;

class pointToGeo
{
public:
    double x;
    double y;
    double lon;
    double lat;

public:
	pointToGeo();
	~pointToGeo(){}

    void gps2meter();
    void meter2gps(double x_,double y_);
};