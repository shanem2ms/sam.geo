#include "sam_geo_includes.h"
#include "spherical_projection.h"
#include "glmext/utils.h"

namespace sam::geo {


template <typename T>
glm::vec<3, T>
spherical_projection<T>::ToWorld(const glm::vec<3, T> &geoPt) {
  T radius = m_earthRadius * (1 + geoPt[2]);
  T sinLat = sin(geoPt[1]);
  T cosLatRadius = cos(geoPt[1]) * radius;
  T sinLon = sin(geoPt[0]);
  T cosLon = cos(geoPt[0]);

  glm::vec3 wsPoint(cosLatRadius * cosLon,
                                    cosLatRadius * sinLon, -sinLat * radius);

  return wsPoint;
}



template <typename T>
glm::qua<T>
spherical_projection<T>::RotateWorld(const glm::vec<3, T> &geoPt) {

    /*
    glm::vec3 wsUp = glm::normalize(-ToWorld(geoPt));
    
    glm::vec3 wsEast = glm::normalize(glm::cross(wsUp, glm::vec3(0, 0, 1) - wsUp));
    glm::vec3 wsNorth = glm::normalize(glm::cross(wsUp, wsEast));

    glm::mat3x3 rotMat(wsEast, wsNorth, wsUp);
    glm::quat rquat = glm::quat(rotMat);
    glm::mat3x3 omat = glm::makeRot(rquat);
    glm::vec3 wsLookVec = rquat * glm::vec3(0, 0, 1);

    return rquat;*/
    return glm::quat_from_axisangle<T>(
        -glm::half_pi<T>(), glm::vec<3, T>(0, 1, 0)) *
        glm::quat_from_axisangle<T>(
        glm::half_pi<T>(), glm::vec<3, T>(0, 0, 1)) *
        glm::quat_from_axisangle<T>(
        -geoPt[0], glm::vec<3, T>(0, 1, 0)) *
        glm::quat_from_axisangle<T>(
             geoPt[1], glm::vec<3, T>(1, 0, 0));
}

template <typename T>
glm::vec<3, T>
spherical_projection<T>::GetFrameTangent(const glm::vec<3, T> &geoPt) {
  return glm::vec<3, T>(-sinf(geoPt[0]), cosf(geoPt[0]), 0.0f);
}

template <typename T>
T
spherical_projection<T>::GetAltitude(const glm::vec<3, T> &wsPt) {
  return (length(glm::vec<3, T>(wsPt)) -
          (T)m_earthRadius) /
         (T)m_earthRadius;
}

template <typename T>
glm::vec<2, T>
spherical_projection<T>::ToGeo2(const glm::vec<3, T> &wsPt) {
  T radius = length(wsPt);
  T latitude = asin(-wsPt[2] / radius);
  T longitude = atan2(wsPt[1], wsPt[0]);
  return glm::vec2(longitude, latitude);
}


template <typename T>
glm::vec<3, T>
spherical_projection<T>::ToGeo3(const glm::vec<3, T>& wsPt) {
    T radius = length(glm::vec<3, T>(wsPt));
    T latitude = asin(-wsPt[2] / radius);
    T longitude = atan2(wsPt[1], wsPt[0]);
    return glm::vec<3, T>(longitude, latitude, (radius / m_earthRadius) - 1);
}

template<typename T> glm::vec<3, T> 
spherical_projection<T>::NormalAtGeo(const glm::vec<3, T>& geoPt)
{
    T sinLat = sin(geoPt[1]);
    T cosLatRadius = cos(geoPt[1]);
    T sinLon = sin(geoPt[0]);
    T cosLon = cos(geoPt[0]);

    glm::vec<3, T> wsPoint(cosLatRadius * cosLon,
        cosLatRadius * sinLon, -sinLat);

    return wsPoint;
}

template class spherical_projection<double>;
template class spherical_projection<float>;
} // namespace sam::geo