#include "math.h"

double latlon_toDistance(double lat_to, double lon_to, double lat_from, double lon_from)
{

	double dlat = (lat_to - lat_from)*DEG_2rad;
	double dlon = (lon_to - lon_from)*DEG_2rad;
	
	double a = pow(sin(dlat/2.0), 2.0) + cos(lat_from*DEG_2rad)*cos(lat_to*DEG_2rad)*pow(sin(dlon/2.0), 2.0);
	double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));

	return (Rad_earth*c);
}

void getPointAhead(double lat_from,double lon_from,double distMtrs,double azimuth, double *coord_deg)
{
    double radiusFraction = distMtrs / Rad_earth;
    double bearing = azimuth*DEG_2rad;
    double lat1 = lat_from*DEG_2rad;
    double lon1 = lon_from*DEG_2rad;
		
    double lat2_part1 = sin(lat1) * cos(radiusFraction);
    double lat2_part2 = cos(lat1) * sin(radiusFraction) * cos(bearing);
		
    double lat2 = asin(lat2_part1 + lat2_part2);
	
    double lon2_part1 = sin(bearing) * sin(radiusFraction) * cos(lat1);
    double lon2_part2 = cos(radiusFraction) - (sin(lat1) * sin(lat2));
		
    double lon2 = lon1 + atan2(lon2_part1, lon2_part2);
          lon2 = mod_f((lon2 + 3.0 * Pi), (2.0 * Pi)) - Pi;

    coord_deg[0] = lat2*RAD_2deg;
	  coord_deg[1] = lon2*RAD_2deg;
}

void mtrsToGeopoint(double latAsMtrs, double lonAsMtrs, double *coord)
{
double coord_temp[2];
    getPointAhead(0.0, 0.0, lonAsMtrs, 90.0,coord_temp);
    getPointAhead(coord_temp[0], coord_temp[1], latAsMtrs, 0.0,coord);
	
}

float Find_bearing(double lat_this, double lon_this, double lat_home, double lon_home)
{	
  double dlon = DEG_2rad*(lon_this-lon_home);
  lat_home = DEG_2rad*lat_home;
  lat_this = DEG_2rad*lat_this;

  double y = sin(dlon) * cos(lat_this);
  double x = cos(lat_this) * sin(lat_home) - sin(lat_this) * cos(lat_home) * cos(dlon);

	float brg = atan2(y,x) * RAD_2deg;
  if(brg<0.0f) brg += 360.0f;
	
  return brg;
}
