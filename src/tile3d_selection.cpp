#include "sam_geo_includes.h"
#include "tile3d_selection.h"
#include "spherical_projection.h"
#include "tileloc.h"
#include "rotated_bounds.h"

namespace sam::geo
{
    rotated_bounds RotatedBoundsFromTileLoc(const geo::tile_loc& tileLoc,
        double bottomAlt, double size,
        const SphericalMercatorProjection<double>& proj)
    {
        glm::aabox2f gsBounds = tileLoc.GetBounds<float>();
        glm::vec2 mid = gsBounds.center();
        glm::vec2 corners[4] =
        { gsBounds.corner<0>(),
            gsBounds.corner<1>(),
            gsBounds.corner<2>(),
            gsBounds.corner<3>() };

        glm::vec3 wsMid = proj.ToWorld(glm::vec3(mid, bottomAlt));

        glm::vec3 ft = proj.GetFrameTangent(glm::vec3(mid, 0));
        glm::vec3 nrm = proj.NormalAtGeo(glm::vec3(mid, 0));
        glm::normalize(ft);
        glm::dvec3 btan = glm::cross(ft, nrm);

        glm::mat3x3 invRotMat(btan, ft, nrm);
        glm::mat3x3 rotMat = glm::transpose(invRotMat);

        glm::mat3x3 sclMat;
        glm::aaboxf wsBounds;

        float topAlt = bottomAlt + size;
        for (auto& p : corners)
        {
            glm::vec3 wsPt0 = proj.ToWorld(glm::vec3(p, bottomAlt));
            glm::vec3 bndsPt0 = wsPt0 * invRotMat;
            wsBounds += bndsPt0;

            glm::vec3 wsPt1 = proj.ToWorld(glm::vec3(p, topAlt));
            glm::vec3 bndsPt1 = wsPt1 * invRotMat;
            wsBounds += bndsPt1;
        }

        wsBounds += wsMid * invRotMat;
        glm::vec3 wsBndsMin = wsBounds.mMin * rotMat;

        glm::vec3 wsExtents = wsBounds.mMax - wsBounds.mMin;
        sclMat = glm::mat3x3(
            glm::vec3(wsExtents.x, 0, 0),
            glm::vec3(0, wsExtents.y, 0),
            glm::vec3(0, 0, wsExtents.z));
        glm::mat3x3 is = invRotMat * sclMat;
        glm::mat3x3 invis = sclMat * glm::transpose(invRotMat);
        glm::mat3x3 bndsVectors = glm::transpose(invis);
        return rotated_bounds(bndsVectors, wsBndsMin);
    }

    bool PtInside(const rotated_bounds& bnds, glm::dvec3 pt)
    {
        glm::dvec3 dv(pt - bnds.min);
        for (int i = 0; i < 3; ++i)
        {
            float dx = glm::dot(dv, bnds.vecs[i]);
            if (dx < 0 || dx > glm::dot(bnds.vecs[i], bnds.vecs[i]))
                return false;
        }
        return true;
    }


    void GetCornerPts(const rotated_bounds& bnds, glm::dvec3 cpts[8])
    {
        for (int i = 0; i < 8; ++i)
        {
            glm::dvec3 distx = (i & 1) ? bnds.vecs[0] : glm::dvec3(0, 0, 0);
            glm::dvec3 disty = ((i >> 1) & 1) ? bnds.vecs[1] : glm::dvec3(0, 0, 0);
            glm::dvec3 distz = ((i >> 2) & 1) ? bnds.vecs[2] : glm::dvec3(0, 0, 0);
            cpts[i] = bnds.min + distx + disty + distz;
        }
    }

    int Tile3dSelection::IsTileInView(const geo::tile_loc& tileLoc, float minAltitude, float maxAltitude,
        const glm::dmat4x4& viewProj, const glm::dvec3& viewPos)
    {
        glm::frustumd frustum(viewProj);
        return IsTileInView(tileLoc, minAltitude, maxAltitude, frustum, viewPos);
    }

    int Tile3dSelection::IsTileInView(const geo::tile_loc& tileLoc, float minAltitude, float maxAltitude,
        const glm::frustumd& frustum, const glm::dvec3& viewPos)
    {
        rotated_bounds rb = RotatedBoundsFromTileLoc(tileLoc, minAltitude, maxAltitude - minAltitude,
            m_sphericalMercatorProjection);
        if (rb.pt_is_inside(viewPos))
            return 2;

        glm::dvec3 cpts[8];
        GetCornerPts(rb, cpts);

        for (int i = 0; i < 4; ++i)
        {
            const glm::planed& plane = frustum.mPlanes[i];
            int sidect = 0;
            for (int j = 0; j < 8; ++j)
            {
                if (plane.distanceTo(cpts[j]) < 0)
                    sidect++;
            }
            if (sidect == 8)
                return 0;
        }

        for (int i = 0; i < 8; ++i)
        {
            glm::rayd ray(viewPos, glm::normalize(cpts[i] - viewPos));
            glm::sphered sphere(glm::dvec3(0, 0, 0), m_sphericalMercatorProjection.Radius());
            int hits = -1;
            double t0, t1;
            ray.intersect(sphere, hits, t0, t1);
            if (hits == 0)
                return 1;

            double lensq = glm::dot(cpts[i] - viewPos, cpts[i] - viewPos);
            if (lensq < t0 * t0)
                return 1;
        }
        return 0;
    }

    void Tile3dSelection::GetTilesInView(SelectedTileList& outTiles, const geo::tile_loc& tileLoc,
        const glm::frustumd& frustum, const glm::dvec3& viewPos, float minAltitude, float maxAltitude, int MaxLOD)
    {
        int tileInView = IsTileInView(tileLoc, minAltitude, maxAltitude, frustum, viewPos);
        if (tileInView == 0)
            return;
        uint32_t targetLevel = MaxLOD;
        if (tileLoc.GetLevel() >= targetLevel)
        {
            outTiles.Insert(SelectedTile(tileLoc));
            return;
        }

        if (tileInView == 1)
        {
            rotated_bounds rb = RotatedBoundsFromTileLoc(tileLoc, minAltitude, maxAltitude - minAltitude,
                m_sphericalMercatorProjection);
            double dist = rb.distance_to_pt(viewPos);
            double lod = log2(m_sphericalMercatorProjection.Radius()) - log2(dist) + 2;

            targetLevel = std::max(std::min(targetLevel, (uint32_t)lod), 0U);
            if (tileLoc.GetLevel() >= targetLevel)
            {
                outTiles.Insert(SelectedTile(tileLoc));
                return;
            }
        }

        geo::tile_loc children[4];
        tileLoc.GetChildren(children);
        for (int i = 0; i < 4; ++i)
        {
            GetTilesInView(outTiles, children[i], frustum, viewPos, minAltitude, maxAltitude, MaxLOD);
        }
    }

    void Tile3dSelection::GetTileList(SelectedTileList& outTiles, const glm::dvec3& viewPos,
        const glm::dmat4x4& viewProj, float minAltitude, float maxAltitude, int maxLOD)
    {
        glm::frustumd frustum(viewProj);

        std::vector<geo::tile_loc> level1Tiles;
        tile_loc::GetAllTilesAtLevel(2, level1Tiles);
        for (const auto& tileLoc : level1Tiles)
        {
            GetTilesInView(outTiles, tileLoc, frustum, viewPos, minAltitude, maxAltitude, maxLOD);
        }
    }

} // namespace sam::earth
