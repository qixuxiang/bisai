
#include "pointToGeo.h"

pointToGeo::pointToGeo()
{
    lat=0.0;
    lon=0.0;
    x=0.0;
    y=0.0;
}

void pointToGeo::gps2meter()
{
    x = SCALE * EARTH_RAD_EQ * lon * M_PI / 180.0 - OFFSET_X - ORIGIN_X;
    y = SCALE * EARTH_RAD_EQ * log(tan((90.0 + lat) * (M_PI / 360.0))) - OFFSET_Y - ORIGIN_Y;
}

void pointToGeo::meter2gps(double x_,double y_)
{
	lat = 360.0 / M_PI * atan(exp( (y_+OFFSET_Y+ORIGIN_Y) /(SCALE * EARTH_RAD_EQ)))-90.0;
	lon = 180.0 * (x_ + OFFSET_X + ORIGIN_X) / (SCALE * EARTH_RAD_EQ * M_PI);
}