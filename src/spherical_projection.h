#pragma once

namespace sam::geo
{
    template<typename T> class spherical_projection
    {
        T m_earthRadius;
    public:
        spherical_projection(T earthRadius = (T)1.0) :
            m_earthRadius(earthRadius) {}

        glm::vec<3, T> ToWorld(const glm::vec<3, T>& geoPt);
        glm::vec<3, T> ToWorld2(const glm::vec<2, T>& geoPt)
        { return ToWorld(glm::vec3(geoPt, 0)); }

        glm::qua<T> RotateWorld(const glm::vec<3, T>& geoPt);
        glm::qua<T> RotateWorld2(const glm::vec<2, T>& geoPt) 
        { return RotateWorld(glm::vec3(geoPt, 0)); }
        glm::vec<3, T> GetFrameTangent(const glm::vec<3, T>& geoPt);
        T GetAltitude(const glm::vec<3, T>& geoPt);

        glm::vec<3, T> NormalAtGeo(const glm::vec<3, T>& gsPoint);
        glm::vec<2, T> ToGeo2(const glm::vec<3, T>& wsPt);
        glm::vec<3, T> ToGeo3(const glm::vec<3, T>& wsPt);

        T Radius() const { return m_earthRadius; }
    };

    // Earth radii.  Distances are in meters.  The mean radius is 6367444.5.
    const double g_sEarthEquatorialRadius = 6378137.0;
    const float g_sEarthEquatorialRadiusF = (float)g_sEarthEquatorialRadius;
    const double g_sEarthPolarRadius = 6356752.0;
    const float g_sEarthPolarRadiusF = (float)g_sEarthPolarRadius;
    const double g_sEarthMeanRadius = (g_sEarthEquatorialRadius + g_sEarthPolarRadius) / 2.0;
    const float g_sEarthMeanRadiusF = (float)g_sEarthMeanRadius;
}
