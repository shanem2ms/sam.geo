#pragma once
#include "tileloc.h"
#include "glmext/Plane.h"
#include "glm/vec3.hpp"
#include "tile_selection.h"
#include "glmext/Frustum.h"
#include "spherical_mercator_projection.h"
#include <set>

namespace sam::geo
{
    class Tile3dSelection
    {

        void GetTilesInView(SelectedTileList& tiles, const geo::tile_loc& tileLoc, const glm::frustumd& frustum, const glm::dvec3& viewPos,
            float minAltitude, float maxAltitude, int MaxLOD);
        int IsTileInView(const geo::tile_loc& tileLoc,
            float minAltitude, float maxAltitude, const glm::frustumd& frustum, const glm::dvec3& viewPos);
    public:

        int IsTileInView(const geo::tile_loc& tileLoc,
            float minAltitude, float maxAltitude, const glm::dmat4x4& viewProj, const glm::dvec3& viewPos);
        void GetTileList(SelectedTileList& tiles, const glm::dvec3& viewPos, const glm::dmat4x4& invViewProj,
            float minAltitude, float maxAltitude, int MaxLOD);
        void SetProjection(const SphericalMercatorProjection<double>& projection)
        {
            m_sphericalMercatorProjection = projection;
        }
    private:
        SphericalMercatorProjection<double> m_sphericalMercatorProjection;
    };
}
