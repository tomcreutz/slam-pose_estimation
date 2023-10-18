#ifndef _POSE_ESTIMATION_PROJECTED_COORDINATE_SYSTEM_HPP
#define _POSE_ESTIMATION_PROJECTED_COORDINATE_SYSTEM_HPP

#include <Eigen/Geometry>

class OGRCoordinateTransformation;

namespace pose_estimation
{

/**
     * Helper class to project from GPS coordinates to a local
     * plane in NWU coordinates and vice versa.
     */
class GeographicProjection
{
public:
  GeographicProjection(double latitude, double longitude, double x = 0., double y = 0.);
  virtual ~GeographicProjection();

  // makes the class uncopyable
  GeographicProjection(const GeographicProjection &) = delete;
  GeographicProjection & operator=(const GeographicProjection &) = delete;

  bool worldToNav(double latitude, double longitude, double & x, double & y);
  bool navToWorld(double x, double y, double & latitude, double & longitude);

protected:
  OGRCoordinateTransformation * world2nav;
  OGRCoordinateTransformation * nav2world;
  Eigen::Vector2d offset;
};

}

#endif
