#ifndef _nav_h
#define _nav_h

double latlon_toDistance(double lat_to, double lon_to, double lat_from, double lon_from);
float Find_bearing(double lat_this, double lon_this, double lat_home, double lon_home);
void getPointAhead(double lat_from,double lon_from,double distMtrs,double azimuth, double *coord_deg);
void mtrsToGeopoint(double latAsMtrs, double lonAsMtrs, double *coord);
#endif

