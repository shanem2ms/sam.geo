////////////////////////////////////////////////////////////////////////////////////////////////////
//  COPYRIGHT (C) 2024
////////////////////////////////////////////////////////////////////////////////////////////////////
/// \author Shane Morrison
/// Bounds that can have an orientation
///
/// min -  lower left corner in worldspace of the bounding box
/// vecs - a mat3x3 which contains 3 rotated axis vectors, which define the extents of the bounding
///     box in the rotated x,y, and z directions.
#pragma once


struct rotated_bounds
{
    // Function to compute the minimum distance between a point and a rotated 3D bounding box
    double minDistanceToBoundingBox(const glm::dvec3& point, const glm::dmat3x3& vecs, const glm::dvec3& min, const glm::dvec3& sizes) {
        // Translate the point to the bounding box's local coordinate system
        glm::dvec3 localPoint = point - min;

        // Project the point onto the bounding box's axes
        double p1 = glm::dot(localPoint, vecs[0]);
        double p2 = glm::dot(localPoint, vecs[1]);
        double p3 = glm::dot(localPoint, vecs[2]);

        // Clamp the coordinates to the bounding box's extents
        double c1 = std::min(std::max(p1, 0.0), sizes.x);
        double c2 = std::min(std::max(p2, 0.0), sizes.y);
        double c3 = std::min(std::max(p3, 0.0), sizes.z);

        // Convert the clamped coordinates back to world coordinates
        glm::dvec3 clampedPoint = min + c1 * vecs[0] + c2 * vecs[1] + c3 * vecs[2];

        // Compute the distance between the original point and the clamped point
        double distance = glm::distance(point, clampedPoint);

        return distance;
    }

    // Function to check if a point is inside the rotated 3D bounding box
    bool isPointInsideBoundingBox(const glm::dvec3& point, const glm::dmat3x3& vecs, const glm::dvec3& min, const glm::dvec3& sizes) {
        // Translate the point to the bounding box's local coordinate system
        glm::dvec3 localPoint = point - min;

        // Project the point onto the bounding box's axes
        double p1 = glm::dot(localPoint, vecs[0]);
        double p2 = glm::dot(localPoint, vecs[1]);
        double p3 = glm::dot(localPoint, vecs[2]);

        // Check if the projected coordinates are within the bounding box's extents
        bool insideX = (0.0 <= p1) && (p1 <= sizes.x);
        bool insideY = (0.0 <= p2) && (p2 <= sizes.y);
        bool insideZ = (0.0 <= p3) && (p3 <= sizes.z);

        // The point is inside the bounding box if it's within the extents on all three axes
        return insideX && insideY && insideZ;
    }

public:
    rotated_bounds() : empty(true) {
    }

    rotated_bounds(const glm::dmat3x3& r,
        const glm::dvec3& m) :
        empty(false),
        vecs(r),
        min(m)
    {
    }

    rotated_bounds(rotated_bounds b[4])
    {
        for (int i = 0; i < 4; ++i)
        {
            if (b[i].empty)
                continue;
        }
    }

    glm::dvec3 to_bounds_pt(const glm::dvec3& wsPt)
    {
        return (wsPt - min) * glm::inverse(vecs);
    }
    

    void get_rotation_scale(glm::dvec3 &size, glm::dmat3x3 &nrot)
    {
        glm::dvec3 sizes(glm::length(vecs[0]),
            glm::length(vecs[1]),
            glm::length(vecs[2]));

        glm::dvec3 r0 = vecs[0] / sizes.x;
        glm::dvec3 r1 = vecs[1] / sizes.y;
        glm::dvec3 r2 = vecs[2] / sizes.z;
        size = sizes;
        nrot = glm::dmat3x3(r0, r1, r2);
    }

    double distance_to_pt(const glm::dvec3& point)
    {
        glm::dvec3 size;
        glm::dmat3x3 nrot;
        get_rotation_scale(size, nrot);
        return minDistanceToBoundingBox(point, nrot, min, size);
    }

    bool pt_is_inside(const glm::dvec3& point)
    {
        glm::dvec3 size;
        glm::dmat3x3 nrot;
        get_rotation_scale(size, nrot);
        return isPointInsideBoundingBox(point, nrot, min, size);
    }

    glm::dvec3 to_local_pt(const glm::dvec3& wsPt)
    {
        glm::dvec3 plocal = wsPt - min;
        glm::dvec3 pbounds(
            glm::dot(plocal, vecs[0]) / glm::length(vecs[0]),
            glm::dot(plocal, vecs[1]) / glm::length(vecs[1]),
            glm::dot(plocal, vecs[2]) / glm::length(vecs[2]));
        return pbounds;
    }
   
    glm::dmat3x3 vecs; 
    glm::dvec3 min;
    bool empty;
};
