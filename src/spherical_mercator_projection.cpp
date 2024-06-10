#include "sam_geo_includes.h"
#include "spherical_mercator_projection.h"


namespace sam::geo {

static constexpr double maxLat = 1.484421727442949242798945159198;
inline static double sec(double x) { return 1.0 / cos(x); }

inline static double LatToY(double lat_rad) {
  lat_rad = std::min(maxLat, std::max(-maxLat, lat_rad));
  return (1 - (log(tan(lat_rad) + sec(lat_rad)) / glm::pi<float>())) / 2;
}

inline static double LatToMercLat(double lat_rad) {
  return (LatToY(lat_rad) - 0.5) * glm::pi<float>();
}

inline static double YToLat(double mercy) {
  return atan(sinh(glm::pi<float>() * (1 - 2 * mercy)));
}

inline static double MercLatToLat(double mercy) {
  float y = (mercy / glm::pi<float>()) + 0.5;
  return YToLat(y);
}

template <typename T>
T SphericalMercatorProjection<T>::ToEquidistantLat(const T mercatorLat) const {
  return (T)MercLatToLat(mercatorLat);
}

template <typename T>
glm::vec<3, T>
SphericalMercatorProjection<T>::ToWorld(const glm::vec<3, T> &geoPt) const {
  float lat = MercLatToLat(geoPt.y);
  float radius = m_earthRadius * (1 + geoPt.z);
  float sinLat = sinf(lat);
  float cosLatRadius = cosf(lat) * radius * (1 + geoPt.z);
  float sinLon = sinf(geoPt.x);
  float cosLon = cosf(geoPt.x);

  glm::vec<3, T> wsPoint(cosLatRadius * cosLon, cosLatRadius * sinLon,
                            -sinLat * radius * (1 + geoPt.z));

#if 0
  // {(rad cos(x))/Sqrt[sinh^2(my)+1],
   // (rad sin(x))/Sqrt[sinh^2(my)+1],
  // -((rad sinh(my))/Sqrt[sinh^2(my)+1])}
  T invSinh = static_cast<T>(1.0) / glm::sqrt(glm::sqr(glm::sinh(geoPt.y)) + 1);

  glm::vec<3, T> wsPoint2(cosLon * invSinh * radius, sinLon * invSinh * radius, -(radius * glm::sinh(geoPt.y)) * invSinh);
#endif
  return wsPoint;
}

template <typename T>
glm::qua<T>
SphericalMercatorProjection<T>::RotateWorld(const glm::vec<3, T> &geoPt) const {
  float lat = MercLatToLat(geoPt.y);
  return glm::qua<T>(
             -glm::half_pi<T>(), glm::vec<3, T>(0, 1, 0)) *
         glm::qua<T>(
             glm::half_pi<T>(), glm::vec<3, T>(0, 0, 1)) *
         glm::qua<T>(
             -geoPt.x, glm::vec<3, T>(0, 1, 0)) *
         glm::qua<T>(lat, glm::vec<3, T>(1, 0, 0));
}

template <typename T>
glm::vec<3, T> SphericalMercatorProjection<T>::GetFrameTangent(
    const glm::vec<3, T> &gsPoint) const {
  return glm::vec<3, T>(-sinf(gsPoint.x), cosf(gsPoint.x), 0.0f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
bool SphericalMercatorProjection<T>::IsGeopointFacingCamera(
    const glm::vec3 &gsPoint,
    const glm::vec<3, T> &cameraEye) const {
  // Make sure the tile point is not over the horizon.
  // NOTE: Lower the point to sea level to better handle mountainous terrain.
  // MRK.
  // NOTE: A little bit of magic with triangles here.  MRK.
  glm::vec<3, T> wsPoint = ToWorld(gsPoint);
  return (dot(wsPoint, (cameraEye - wsPoint))) > -0.0f;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
glm::vec<3, T> SphericalMercatorProjection<T>::NormalAtGeo(
    const glm::vec<3, T>&gsPoint) const {
  glm::vec<3, T> wsPoint =
      ToWorld(glm::vec<3, T>(glm::vec<2, T>(gsPoint), 0));  
  return normalize(wsPoint);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
glm::vec<2, T>
SphericalMercatorProjection<T>::ToGeo2(const glm::vec<3, T> &wsPt) const {
  T radius = length(glm::vec<3, T>(wsPt));
  T latitude = asin(-wsPt.z / radius);
  T longitude = atan2(wsPt.y, wsPt.x);
  return glm::vec<2, T>(longitude, LatToMercLat(latitude));
}

template <typename T>
glm::vec<3, T>
SphericalMercatorProjection<T>::ToGeo3(const glm::vec<3, T>& wsPt) const {
    T radius = length(glm::vec<3, T>(wsPt));
    T latitude = asin(-wsPt[2] / radius);
    T longitude = atan2(wsPt[1], wsPt[0]);
    return glm::vec<3, T>(longitude, LatToMercLat(latitude), (radius / m_earthRadius) - 1);
}

template class SphericalMercatorProjection<double>;
template class SphericalMercatorProjection<float>;
} // namespace sam::geo