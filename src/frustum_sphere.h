#pragma once
#include "spherical_mercator_projection.h"
#include "glmext/range.h"
#include "glmext/Circle3.h"
#include "glmext/Frustum.h"

namespace sam::geo
{

    template <typename T> struct frustum_arc
    {
        std::vector<glm::vec<3, T>> pts;
        int sideIdx;
    };

    typedef frustum_arc<float> frustum_arcf;

    template <typename T> struct geo_frustum
    {
        glm::vec<3, T> nearLeft;
        glm::vec<3, T> nearRight;
        glm::vec<3, T> farLeft;
        glm::vec<3, T> farRight;
    };

    typedef geo_frustum<float> geo_frustumf;

    template <typename T> class frustum_sphere
    {
        static int calculateIntersectionAngles(const glm::circle3_t<T>& circle, const glm::planef& P, T& a0, T& a1);
        // Function to calculate the intersection circle of a sphere and a plane
        static bool intersectSpherePlane(const glm::vec<3, T>& center, T radius, const glm::planef& plane, glm::circle3_t<T>& circle);
        // Function to discretize the circle into points in 3D space on the original plane
        static std::vector<glm::rangef> GetIntersectionArcs(const glm::circle3_t<T>& circle, const std::vector<glm::planef>& planes);

    public:
        static std::vector<frustum_arc<T>> get_intersection_points(const glm::frustumf& frustum, const glm::vec<3, T>& viewPos, T radius, T occlusionSphereRadius, bool& horizonInView,
            geo_frustum<T>& geo_frustum);
        static bool get_occlusion_circle(const glm::vec<3, T>& viewPos, T radiusA, T radiusB, glm::planef& occlusion_plane, T& occlusion_radius);
        static bool get_horizon_plane(const glm::vec<3, T>& viewPos, T radius, glm::planef& horizon_plane, T& earthCutRadius);
        static glm::mat4x4 get_surface_matrix(const geo_frustum<T>& geo_frustum, const sam::geo::SphericalMercatorProjection<T>& proj);
    };

}