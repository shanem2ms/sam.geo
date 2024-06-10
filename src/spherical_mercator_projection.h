#pragma once
namespace sam::geo
{
    template<typename T> class SphericalMercatorProjection
    {      
        T m_earthRadius;
    public:     
        SphericalMercatorProjection(T earthRadius = (T)1.0) :
            m_earthRadius(earthRadius){}

        glm::vec<3, T> ToWorld(const glm::vec<3, T>& geoPt) const;
        glm::vec<3, T> ToWorld2(const glm::vec<2, T>& geoPt) const
        { return ToWorld(glm::vec3(geoPt, 0)); }

        T ToEquidistantLat(const T mercatorLat) const;
        glm::qua<T> RotateWorld(const glm::vec<3, T>& geoPt) const;
        glm::vec<3, T> GetFrameTangent(const glm::vec<3, T>& geoPt) const;

        bool IsGeopointFacingCamera(
            const glm::vec3& gsPoint, const glm::vec<3, T>& cameraEye) const;

        glm::vec<3, T> NormalAtGeo(const glm::vec<3, T>& gsPoint) const;
        glm::vec<2, T> ToGeo2(const glm::vec<3, T>& wsPt) const;
        glm::vec<3, T> ToGeo3(const glm::vec<3, T>& wsPt) const;

        T Radius() const { return m_earthRadius; }
    };
}
