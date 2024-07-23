#include "sam_geo_includes.h"
#include "frustum_sphere.h"
#include "sam_geo_includes.h"
#include "glmext/Intersection.h"

namespace sam::geo
{
    // Function to calculate the intersection circle of a sphere and a plane
    template <typename T> bool frustum_sphere<T>::intersectSpherePlane(const glm::vec<3, T>& center, T radius, const glm::plane_t<T>& plane, glm::circle3_t<T>& circle)
    {
        T distance = glm::dot(plane.normal, center) - plane.d;
        if (std::abs(distance) > radius) return false; // No intersection

        circle.radius = std::sqrt(radius * radius - distance * distance);
        circle.center = center - plane.normal * distance;
        circle.normal = plane.normal;
        return true;
    }


    template <typename T> int intersectLineCircle(const glm::vec<2, T>& p1, const glm::vec<2, T>& p2, const glm::vec<2, T>& center, T radius,
        glm::vec<2, T>& i0, glm::vec<2, T>& i1)
    {

        // Line equation: (x - x1) / (x2 - x1) = (y - y1) / (y2 - y1)
        // Rearrange line equation: ax + by + c = 0
        double a = p2.y - p1.y;
        double b = p1.x - p2.x;
        double c = p2.x * p1.y - p1.x * p2.y;

        // Circle equation: (x - cx)^2 + (y - cy)^2 = r^2
        // Substitute line equation into circle equation to get a quadratic equation:
        // A * x^2 + B * x + C = 0
        double A = a * a + b * b;
        double B = 2 * (a * c + a * b * center.y - b * b * center.x);
        double C = c * c + 2 * b * c * center.y + b * b * (center.x * center.x + center.y * center.y - radius * radius);

        // Discriminant of the quadratic equation
        double D = B * B - 4 * A * C;

        if (D >= 0) {
            // One or two solutions
            double sqrtD = std::sqrt(D);
            double t1 = (-B + sqrtD) / (2 * A);
            double t2 = (-B - sqrtD) / (2 * A);

            i0 = { t1, (-a * t1 - c) / b };

            if (D > 0) {
                i1 = { t2, (-a * t2 - c) / b };
                return 2;
            }
            return 1;
        }

        return 0;
    }


    // Function to calculate the intersection angles
    template <typename T> int frustum_sphere<T>::calculateIntersectionAngles(
        const glm::circle3_t<T>& c,       // U dir of the circle's plane
        const glm::plane_t<T>& P,
        T& a0,                 // Output angle 0
        T& a1                  // Output angle 1
    ) {

        glm::plane_t<T> circle_plane(c.normal, c.center);

        glm::vec<3, T> linedir;
        glm::vec<3, T> linept;
        if (!glm::intersect(P, circle_plane, linept, linedir))
            return -1;

        glm::vec<3, T> plane_u, plane_v;
        glm::generate_uv(circle_plane, plane_u, plane_v);

        glm::vec<3, T> testw = glm::cross(plane_u, plane_v);

        glm::vec<2, T> linept2 = glm::project(plane_u, plane_v, linept);
        glm::vec<2, T> linedir2 = glm::project(plane_u, plane_v, linedir);
        glm::vec<2, T> circlept2 = glm::project(plane_u, plane_v, c.center);

        glm::vec<2, T> i0, i1;
        if (intersectLineCircle(linept2, linept2 + linedir2, circlept2, c.radius, i0, i1) < 2)
        {
            return (glm::dot(c.center, P.normal) - P.d) < 0 ? 0 : 2;
        }

        glm::vec<2, T> ri0 = (i0 - circlept2);
        a0 = atan2(ri0.y, ri0.x);
        if (a0 < 0)
            a0 += glm::two_pi<T>();
        glm::vec<2, T> ri1 = (i1 - circlept2);
        a1 = atan2(ri1.y, ri1.x);
        if (a1 < 0)
            a1 += glm::two_pi<T>();

        T a1t = a1;
        // Check which part of arc is on inside
        while (a1t < a0)
            a1t += glm::two_pi<T>();
        T midpt = (a0 + a1t) * 0.5;
        if (glm::dot(c.PtFromAngle(midpt), P.normal) - P.d < 0)
        {
            std::swap(a0, a1);
        }
        while (a1 < a0)
            a1 += glm::two_pi<T>();

        return 1;
    }

    // Function to normalize an angle to the range [0, 360)
    template<typename T> T normalizeAngle(T angle) {
        while (angle < 0) angle += glm::two_pi<T>();
        while (angle >= glm::two_pi<T>()) angle -= glm::two_pi<T>();
        return angle;
    }
    // Function to find the intersection arcs
    template<typename T> std::vector<glm::range_t<T>> findIntersectionArcs(glm::range_t<T> A, glm::range_t<T> B)
    {
        std::vector<glm::range_t<T>> intersections;

        if (A.min > B.min)
            std::swap(A, B);

        while (A.min < B.max)
        {
            glm::range_t<T> r = A.intersect(B);
            if (!r.empty())
            {
                T delta = r.max - r.min;
                r.min = normalizeAngle(r.min);
                r.max = r.min + delta;
                intersections.push_back(r);
            }
            A.offset(glm::two_pi<T>());
        }
        return intersections;
    }


    template <typename T> bool frustum_sphere<T>::get_horizon_plane(const glm::vec<3, T>& viewPos, T radius, glm::plane_t<T>& horizon_plane, T& earthCutRadius)
    {
        // Calculate 3d circle of horizon line on ideal (radius = 1) spherical Earth surface.
        const glm::vec<3, T> wsEyeNormal = -glm::normalize(glm::vec<3, T>(viewPos));
        const T wsEyeDistFromEarthCenter = length(glm::vec<3, T>(viewPos));

        // Horizon has no sense when eye position is under the surface.
        if (wsEyeDistFromEarthCenter < radius)
        {
            return false;
        }

        const T distToHorizonSquared = glm::sqr(wsEyeDistFromEarthCenter) - radius;
        const T earthCutDistFromEye = distToHorizonSquared / wsEyeDistFromEarthCenter;
        earthCutRadius = glm::sqrt(distToHorizonSquared - glm::sqr(earthCutDistFromEye));
        horizon_plane = glm::plane_t<T>(-wsEyeNormal, viewPos + wsEyeNormal * earthCutDistFromEye);
        return true;
    }

    // Given two spheres oriented at 0,0,0 with radiusA and radiusB, calculates the plane of occlusion of sphere B by
    // sphere A.
    template <typename T> bool frustum_sphere<T>::get_occlusion_circle(const glm::vec<3, T>& viewPos, T radiusA, T radiusB, glm::plane_t<T>& occlusion_plane, T& occlusion_radius)
    {
        glm::plane_t<T> horizon_plane;
        T horizon_radius;
        get_horizon_plane(viewPos, radiusA, horizon_plane, horizon_radius);

        glm::vec<3, T> uDir = glm::generate_u(horizon_plane);
        glm::vec<3, T> center = horizon_plane.normal * horizon_plane.d;
        glm::vec<3, T> horizon_pt = center + uDir * horizon_radius;
        glm::vec<3, T> viewPt = viewPos;
        glm::vec<3, T> viewToHorizon_dir = glm::normalize(horizon_pt - viewPt);

        glm::sphere_t<T> sphereB(glm::vec<3, T>(0, 0, 0), radiusB);
        std::vector<glm::vec<3, T>> projPts;
        projPts.push_back(horizon_pt);
        T t[2];
        int ipts = glm::intersect(sphereB, glm::ray_t<T>(viewPt, viewToHorizon_dir), t[0], t[1]);
        if (ipts > 0)
        {
            //projPts.push_back();
            glm::vec<3, T> ptOnSphereB = viewPt + viewToHorizon_dir * t[0];
            occlusion_plane = glm::plane_t<T>(horizon_plane.normal, ptOnSphereB);
            occlusion_radius = glm::sqrt(sphereB.mRadius * sphereB.mRadius - occlusion_plane.d * occlusion_plane.d);
            return true;
        }
        return false;
    }

    template <typename T> std::vector<glm::range_t<T>> frustum_sphere<T>::GetIntersectionArcs(const glm::circle3_t<T>& circle, const std::vector<glm::plane_t<T>>& planes)
    {
        std::vector<glm::range_t<T>> arcs;
        arcs.push_back(glm::range_t<T>(0, glm::two_pi<T>()));
        for (int i = 0; i < planes.size(); ++i)
        {
            const glm::plane_t<T>& clipPlane = planes[i];
            T a, b;
            int result = frustum_sphere<T>::calculateIntersectionAngles(circle, clipPlane, a, b);

            if (result == 0)
            {
                arcs = std::vector<glm::range_t<T>>();
                break;
            }
            else if (result == 2)
                continue;
            glm::range_t<T> rga(a, b);
            std::vector<glm::range_t<T>> newarcs;
            for (auto& arc : arcs)
            {
                std::vector<glm::range_t<T>> arcs2 = findIntersectionArcs(arc, rga);
                newarcs.insert(newarcs.end(), arcs2.begin(), arcs2.end());
            }
            std::swap(newarcs, arcs);
        }
        return arcs;
    }

    template<typename T> glm::vec<2, T> to_spherical(const glm::vec<3, T>& pt, T radius)
    {
        T latitude = asin(-pt[2] / radius);
        T longitude = atan2(pt[1], pt[0]);
        return glm::vec<2, T>(longitude, latitude);
    }

    template <typename T> std::vector<frustum_arc<T>> frustum_sphere<T>::get_intersection_points(const glm::frustum_t<T>& frustum, const glm::vec<3, T>& viewPos,
        T radius, T occlusionSphereRadius, bool& horizonInView, geo_frustum<T>& geo_frustum)
    {
        std::vector<glm::plane_t<T>> planes(&frustum.mPlanes[0], &frustum.mPlanes[6]);

        horizonInView = false;
        // Check if horizon is in view
        if (occlusionSphereRadius > 0)
        {
            glm::plane_t<T> occluder_horizon_plane;
            T occluder_horizon_radius;
            get_horizon_plane(viewPos, occlusionSphereRadius, occluder_horizon_plane, occluder_horizon_radius);

            glm::vec<3, T> horizoncenter = occluder_horizon_plane.normal * occluder_horizon_plane.d;
            glm::circle3_t<T> circle(horizoncenter, occluder_horizon_plane.normal, occluder_horizon_radius);
            std::vector<glm::range_t<T>> arcs = GetIntersectionArcs(circle, planes);
            if (arcs.size() > 0)
            {
                horizonInView = true;
                glm::plane_t<T> occlusion_plane;
                T occlusio_radius;
                get_occlusion_circle(viewPos, occlusionSphereRadius, radius, occlusion_plane,
                    occlusio_radius);
                planes.push_back(occlusion_plane);
            }
        }

        glm::plane_t<T> horizon_plane;
        T horizon_radius;
        if (get_horizon_plane(viewPos, radius, horizon_plane, horizon_radius))
            planes.push_back(horizon_plane);

        std::vector<frustum_arc<T>> circlePointsList;
        for (int planeIdx = 0; planeIdx < planes.size(); ++planeIdx)
        {
            glm::circle3_t<T> circle;
            if (intersectSpherePlane(glm::vec<3, T>(0, 0, 0), radius, planes[planeIdx], circle))
            {
                std::vector<glm::plane_t<T>> clipplanes = planes;
                clipplanes.erase(clipplanes.begin() + planeIdx);

                std::vector<glm::range_t<T>> arcs = GetIntersectionArcs(circle, clipplanes);
                if (planeIdx == 0 && arcs.size() > 0)
                {
                    geo_frustum.nearLeft = circle.PtFromAngle(arcs[0].max);
                    geo_frustum.farLeft = circle.PtFromAngle(arcs[0].min);
                }
                else if (planeIdx == 1 && arcs.size() > 0)
                {
                    geo_frustum.nearRight = circle.PtFromAngle(arcs[0].min);
                    geo_frustum.farRight = circle.PtFromAngle(arcs[0].max);
                }

                for (auto& arc : arcs)
                {
                    std::vector<glm::vec<3, T>> arcpts = circle.discretize(100, arc.min, arc.max);
                    circlePointsList.push_back({ arcpts, planeIdx });
                }
            }

        }
        return circlePointsList;
    }
    template <typename T> glm::mat4x4 frustum_sphere<T>::get_surface_matrix(const geo_frustum<T>& geo_frustum, const sam::geo::SphericalMercatorProjection<T>& proj)
    {
        glm::vec<2, T> A = proj.ToGeo2(geo_frustum.nearLeft);
        glm::vec<2, T> B = proj.ToGeo2(geo_frustum.nearRight);
        glm::vec<2, T> C = proj.ToGeo2(geo_frustum.farLeft);
        glm::vec<2, T> D = proj.ToGeo2(geo_frustum.farRight);

        glm::vec<2, T> nearCenter = (A + B) * (T)0.5;
        glm::mat4x4 offsetMatrix = glm::translate(glm::vec<3, T>(-nearCenter, 0));

        glm::vec4 A1 = offsetMatrix * glm::vec4(A, 0.5, 1);
        glm::vec4 B1 = offsetMatrix * glm::vec4(B, 0.5, 1);
        glm::vec4 C1 = offsetMatrix * glm::vec4(C, 0.5, 1);
        glm::vec4 D1 = offsetMatrix * glm::vec4(D, 0.5, 1);

        glm::vec<2, T> nearLine = B - A;
        nearLine = glm::normalize(nearLine);
        glm::vec<2, T> rotdir = glm::vec<2, T>(nearLine.y, -nearLine.x);
        glm::vec<2, T> rtan(rotdir.y, -rotdir.x);
        glm::mat4x4 rotMat = glm::identity<glm::mat4x4>();
        rotMat[0][0] = rtan.x;
        rotMat[0][1] = rotdir.x;
        rotMat[1][0] = rtan.y;
        rotMat[1][1] = rotdir.y;

        T xscaleNearInv = 2.0 / glm::dot(rtan, (B - A));
        T xscaleFar = glm::dot(rtan, (D - C)) * xscaleNearInv;
        T yscale = glm::dot(rotdir, (C - A)) * xscaleNearInv;

        glm::mat4x4 sclMat = glm::scale(glm::vec<3, T>(xscaleNearInv, xscaleNearInv, 1));
        T wscale = (xscaleFar * 0.5 - 1) / yscale;
        T wyscale = (xscaleFar * 0.5 + 1) / yscale;
        T wzscale = (xscaleFar * 0.5 - 1) / yscale;

        glm::mat4x4 wMat = glm::identity<glm::mat4x4>();
        wMat[1][1] = wyscale;
        wMat[1][2] = wzscale;
        wMat[1][3] = wscale;
        wMat[2][2] = 0;
        wMat[3][1] = -1;
        wMat[3][2] = 0.5;

        return wMat * sclMat * rotMat * offsetMatrix;
    }


    template class frustum_sphere<float>;
}