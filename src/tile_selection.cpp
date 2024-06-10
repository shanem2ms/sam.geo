#include "sam_geo_includes.h"
#include "tile_selection.h"
#include "spherical_mercator_projection.h"
#include "tileloc.h"
#ifdef USE_XXHASH
#include "xxhash.h"
#else
#include <extern/komihash.h>
#endif
#include <map>

namespace sam::geo
{
    bool TileSelection::WorldPointInView(glm::dvec3& worldPt, const glm::dmat4x4& viewProj)
    {
        glm::dvec4 screenPt = viewProj * glm::dvec4(worldPt, 1);
        if (screenPt[0] < -screenPt[3] || screenPt[0] > screenPt[3] || screenPt[1] < -screenPt[3] ||
            screenPt[1] > screenPt[3])
            return false;
        return true;
    }

    bool TileSelection::IsTileInView(const tile_loc& tileLoc, const glm::dmat4x4& viewProj,
        const glm::dvec3& lookVec,
        const glm::planed greatCirclePlane, double greatCircleRadius, float minAltitude, float maxAltitude)
    {
        glm::aabox2d bounds = tileLoc.GetGsBounds<double>();
        glm::dvec2 epts[3] = { bounds.getMin(),
                            bounds.getMax(),
                            bounds.center() };

        glm::dvec3 wsPts[9];
        glm::dvec3 wsNrm[9];
        for (int i = 0; i < 9; ++i)
        {
            glm::dvec2 pt(epts[i / 3][0], epts[i % 3][1]);
            wsPts[i] = m_sphericalMercatorProjection.ToWorld(glm::dvec3(pt, minAltitude));
            wsNrm[i] = m_sphericalMercatorProjection.NormalAtGeo(glm::dvec3(pt, minAltitude));
        }

        glm::dvec3 o = greatCirclePlane.getOffset() * greatCirclePlane.getNormal();
        bool isInsideGreatCircle = false;
        for (int i = 0; i < 9; ++i)
        {
            double dist = dot(wsPts[i] - o, greatCirclePlane.getNormal());
            glm::dvec3 planePt = wsPts[i] - dist * greatCirclePlane.getNormal();
            if (glm::dot(glm::dvec3(planePt - o), glm::dvec3(planePt - o)) < glm::sqr(greatCircleRadius))
            {
                isInsideGreatCircle = true;
                break;
            }
        }

        if (!isInsideGreatCircle)
            return false;

        for (int i = 0; i < 9; ++i)
        {
            if (WorldPointInView(wsPts[i], viewProj) && dot(wsNrm[i], lookVec) < 0)
                return true;
        }
        return false;
    }

    bool TileSelection::IsSelectedTileInView(const SelectedTile& tile, const glm::dmat4x4& viewProj,
        const glm::dvec3& lookVec, const glm::planed greatCirclePlane,
        double greatCircleRadius, float minAltitude, float maxAltitude)
    {
        tile_loc childLocs[4];
        tile.m_tileLoc.GetChildren(childLocs);
        for (int idx = 0; idx < 4; ++idx)
        {
            if (!tile.m_childIsInSelection[idx] &&
                IsTileInView(childLocs[idx], viewProj, lookVec,
                    greatCirclePlane, greatCircleRadius, minAltitude, maxAltitude))
            {
                return true;
            }
        }

        return false;
    }



    ////////////////////////////////////////////////////////////////////////////////////////////////////

    bool TileSelection::GetEarthHorizonCircle(const glm::dvec3& wsEye, glm::planed& circlePlane, double& radius) const
    {
        // Calculate 3d circle of horizon line on ideal (radius = 1) spherical Earth surface.
        const glm::dvec3 wsEyeNormal = -glm::normalize(glm::dvec3(wsEye));
        const float wsEyeDistFromEarthCenter = length(glm::dvec3(wsEye));

        // Horizon has no sense when eye position is under the surface.
        if (wsEyeDistFromEarthCenter < 1.0)
        {
            return false;
        }

        const double distToHorizonSquared = glm::sqr(wsEyeDistFromEarthCenter) - 1.0;
        const double earthCutDistFromEye = distToHorizonSquared / wsEyeDistFromEarthCenter;
        const double earthCutRadius = glm::sqrt(distToHorizonSquared - glm::sqr(earthCutDistFromEye));

        circlePlane = glm::planed(wsEyeNormal, wsEye + wsEyeNormal * earthCutDistFromEye);
        radius = earthCutRadius;
        return true;
    }

    bool TileSelection::TileDistCheck(const tile_loc& tileLoc, const glm::dvec3& viewPos)
    {
        glm::aabox2d bounds = tileLoc.GetGsBounds<double>();
        glm::dvec3 mid = m_sphericalMercatorProjection.ToWorld(glm::dvec3(bounds.center(), 0));

        glm::dvec3 p1 = m_sphericalMercatorProjection.ToWorld(glm::dvec3(bounds.corner<0>(), 0));
        glm::dvec3 p2 = m_sphericalMercatorProjection.ToWorld(glm::dvec3(bounds.corner<2>(), 0));

        double tileLen = length(glm::dvec3(p2 - p1));
        double tileDist = length(glm::dvec3(viewPos - mid));
        return (tileDist < tileLen);
    }

    float TileSelection::TileAngle(const tile_loc& tileLoc, const glm::dvec3& lookVec)
    {
        glm::aabox2d bounds = tileLoc.GetGsBounds<double>();
        glm::dvec3 mid = m_sphericalMercatorProjection.NormalAtGeo(glm::dvec3(bounds.center(), 0));
        return dot(mid, lookVec);
    }

    bool TileSelection::IsTileFacingViewer(const tile_loc& tileLoc, const glm::dvec3& lookVec)
    {
        glm::aabox2d bounds = tileLoc.GetGsBounds<double>();
        glm::dvec3 wsNrm[4] = { m_sphericalMercatorProjection.NormalAtGeo(glm::dvec3(bounds.corner<0>(), 0)),
                          m_sphericalMercatorProjection.NormalAtGeo(glm::dvec3(bounds.corner<1>(), 0)),
                          m_sphericalMercatorProjection.NormalAtGeo(glm::dvec3(bounds.corner<2>(), 0)),
                          m_sphericalMercatorProjection.NormalAtGeo(glm::dvec3(bounds.corner<3>(), 0)) };
        for (int i = 0; i < 4; ++i)
        {
            if (dot(wsNrm[i], lookVec) < 0)
                return true;
        }
        return false;
    }

    double TileSelection::TargetLodAtWorldPt(glm::dvec3& worldPt, const glm::dmat4x4& viewProj, const glm::dmat4x4& invViewProj)
    {
        glm::dvec4 screenPt;
        screenPt = viewProj * glm::dvec4(worldPt, 1);
        glm::dvec4 screenPt2 = screenPt;
        screenPt2[0] += 0.1f * screenPt[3];
        screenPt2[1] += 0.1f * screenPt[3];
        glm::dvec4 hitPt2;
        hitPt2 = invViewProj * screenPt2;
        hitPt2 /= hitPt2[3];
        float worldDist = (abs(worldPt[0] - hitPt2[0]) + abs(worldPt[1] - hitPt2[1])) * 0.5f;
        return log2(m_sphericalMercatorProjection.Radius()) - log2(worldDist) - 1;
    }

    namespace
    {
        inline double TriangleArea(const glm::dvec2 tri[3])
        {
            double x1 = tri[0][0];
            double y1 = tri[0][1];
            double x2 = tri[1][0];
            double y2 = tri[1][1];
            double x3 = tri[2][0];
            double y3 = tri[2][1];

            return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) * 0.5);
        }
    }
    float TileSelection::GetTileScreenArea(const geo::tile_loc& tileLoc, const glm::dmat4x4& viewProj, float minAltitude, float maxAltitude)
    {
        glm::aabox2d bounds = tileLoc.GetBounds<double>();
        glm::dvec3 wsCorners[4] = { m_sphericalMercatorProjection.ToWorld(glm::dvec3(bounds.corner<0>(), minAltitude)),
                              m_sphericalMercatorProjection.ToWorld(glm::dvec3(bounds.corner<1>(), minAltitude)),
                              m_sphericalMercatorProjection.ToWorld(glm::dvec3(bounds.corner<2>(), minAltitude)),
                              m_sphericalMercatorProjection.ToWorld(glm::dvec3(bounds.corner<3>(), minAltitude)) };
        glm::dvec4 screenPts[4];
        for (int i = 0; i < 4; ++i)
        {
            screenPts[i] = viewProj * glm::dvec4(wsCorners[i], 1);
            screenPts[i] /= screenPts[i][3];
        }

        glm::dvec2 tri0[3] = { glm::dvec3(screenPts[0]), glm::dvec3(screenPts[1]), glm::dvec3(screenPts[2]) };
        glm::dvec2 tri1[3] = { glm::dvec3(screenPts[2]), glm::dvec3(screenPts[1]), glm::dvec3(screenPts[3]) };

        return TriangleArea(tri0) + TriangleArea(tri1);
    }

    bool TileSelection::CollectAllNeighbors(std::set<geo::tile_loc>& tiles,
        const geo::tile_loc& loc,
        std::set<uint32_t>& visited, const glm::dvec3& viewPos,
        const glm::dmat4x4& viewProj, const glm::dvec3& lookVec,
        const glm::planed greatCirclePlane,
        double greatCircleRadius, bool forceAdaquateRes,
        float minAltitude, float maxAltitude)
    {
        bool needsLowerResTiles = false;
        tile_loc siblings[8];
        loc.GetSiblings(siblings);
        for (int i = 0; i < 8; ++i)
        {
            if (forceAdaquateRes || visited.find(siblings[i].GetIndex()) == visited.end())
            {
                visited.insert(siblings[i].GetIndex());
                if (IsTileInView(siblings[i], viewProj, lookVec,
                    greatCirclePlane, greatCircleRadius, minAltitude, maxAltitude))
                {
                    double area = GetTileScreenArea(siblings[i], viewProj, minAltitude, maxAltitude);
                    if (forceAdaquateRes || area > 0.5)
                    {
                        tiles.insert(siblings[i]);
                        needsLowerResTiles |= CollectAllNeighbors(tiles, siblings[i], visited, viewPos, viewProj, lookVec,
                            greatCirclePlane, greatCircleRadius, false, minAltitude, maxAltitude);
                    }
                    else
                    {
                        needsLowerResTiles = true;
                    }
                }
            }
        }

        return needsLowerResTiles;
    }

    bool TileSelection::GetGeoPtAtView(const glm::dvec3& viewPos,
        const glm::dmat4x4& viewProj, float testAltitude, glm::dvec2& outPt)
    {
        glm::dmat4x4 invViewProj = glm::inverse(viewProj);
        glm::dvec4 wsRay0, wsRay1;
        wsRay0 = invViewProj * glm::dvec4(0, 0, 0, 1);
        wsRay1 = invViewProj * glm::dvec4(0, 0, 1, 1);
        wsRay0 /= wsRay0[3];
        wsRay1 /= wsRay1[3];

        glm::dvec3 lookVec = glm::dvec3(wsRay1 - wsRay0);
        normalize(lookVec);
        glm::rayd ray(glm::dvec3(glm::dvec3(viewPos)), lookVec);
        glm::sphered sphere(glm::dvec3(0, 0, 0), m_sphericalMercatorProjection.Radius() + testAltitude);
        double t0, t1;
        int numHits;
        if (ray.intersect(sphere, numHits, t0, t1))
        {
            glm::dvec3 hitPt = ray.getOrigin() + ray.getDir() * t0;
            outPt = m_sphericalMercatorProjection.ToGeo3(hitPt);
            return true;
        }

        return false;
    }

    void TileSelection::GetTileList(SelectedTileList& outTiles, const glm::dvec3& viewPos,
        const glm::dmat4x4& viewProj, float minAltitude, float maxAltitude, int maxLOD)
    {
        // Fix this
        glm::dmat4x4 invViewProj = glm::inverse(viewProj);
        glm::dvec4 wsRay0, wsRay1;
        wsRay0 = invViewProj * glm::dvec4(0, 0, 0, 1);
        wsRay1 = invViewProj * glm::dvec4(0, 0, 1, 1);
        wsRay0 /= wsRay0[3];
        wsRay1 /= wsRay1[3];


        glm::planed greatCirclePlane;
        double greatCircleRadius = 0;
        GetEarthHorizonCircle(viewPos, greatCirclePlane, greatCircleRadius);
        glm::dvec3 lookVec = glm::dvec3(wsRay1 - wsRay0);
        lookVec = normalize(lookVec);
        glm::rayd ray(glm::dvec3(glm::dvec3(viewPos)), lookVec);
        glm::sphered sphere(glm::dvec3(0, 0, 0), m_sphericalMercatorProjection.Radius() + minAltitude);
        double t0, t1;
        int numHits;
        double len = length(glm::dvec3(viewPos));
        if (ray.intersect(sphere, numHits, t0, t1))
        {
            glm::dvec3 hitPt = ray.getOrigin() + ray.getDir() * t0;
            glm::dvec2 geoPt = m_sphericalMercatorProjection.ToGeo2(hitPt);
            float targetLevel = TargetLodAtWorldPt(hitPt, viewProj, invViewProj);
            int startLevel = maxLOD > 0 ? std::min((int)targetLevel, maxLOD) : (int)targetLevel;
            geo::tile_loc tileLoc = geo::tile_loc::GetTileAtGeopoint(geoPt, startLevel);
            std::set<uint32_t> visited;
            std::set<geo::tile_loc> tiles;

            tiles.insert(tileLoc);
            visited.insert(tileLoc.GetIndex());
            std::set<tile_loc> parentTiles;
            while (CollectAllNeighbors(tiles, tileLoc, visited, viewPos, viewProj, lookVec, greatCirclePlane,
                greatCircleRadius, true, minAltitude, maxAltitude))
            {
                tileLoc = tileLoc.GetParent();
                std::set<tile_loc> nextParentTiles;
                for (auto& ptl : tiles)
                {
                    nextParentTiles.insert(
                        ptl.GetParentAtLevel(tileLoc.GetLevel()));
                }
                for (auto& ptl : nextParentTiles)
                {
                    tiles.insert(ptl);
                    visited.insert(ptl.GetIndex());
                }
                std::swap(parentTiles, nextParentTiles);
            };


            std::map<geo::tile_loc, SelectedTile> selectedTiles;
            for (auto& tile : tiles)
            {
                selectedTiles.insert(std::make_pair(tile, SelectedTile(tile)));
                auto itParent = selectedTiles.find(tile.GetParent());
                if (itParent != selectedTiles.end())
                {
                    geo::tile_loc relative = tile.Global2Local(itParent->first);
                    int childOffset = relative.m_y * 2 + relative.m_x;
                    itParent->second.m_childIsInSelection[childOffset] = true;
                }
            }

            for (auto itSelectedTile = selectedTiles.begin(); itSelectedTile != selectedTiles.end();)
            {
                const SelectedTile& st = itSelectedTile->second;
                int nChildrenInSelection = 0;
                for (int i = 0; i < 4; ++i)
                {
                    nChildrenInSelection += st.m_childIsInSelection[i] ? 1 : 0;
                }
                if (nChildrenInSelection == 4)
                    itSelectedTile = selectedTiles.erase(itSelectedTile);
                else if (nChildrenInSelection > 0 &&
                    !IsSelectedTileInView(itSelectedTile->second, viewProj, lookVec, greatCirclePlane,
                        greatCircleRadius, minAltitude, maxAltitude))
                    itSelectedTile = selectedTiles.erase(itSelectedTile);
                else
                    ++itSelectedTile;
            }
            for (auto& kv : selectedTiles)
            {
                outTiles.Insert(kv.second);
            }
        }
    }

    void SelectedTileList::Insert(const SelectedTile& tile)
    {
        m_hash = 0;
        m_tiles.insert(tile);
    }

    size_t SelectedTileList::Size()
    {
        return m_tiles.size();
    }

    bool SelectedTileList::Contains(const geo::tile_loc& loc) const
    {
        return m_tiles.find(loc) != m_tiles.end();
    }

    int64_t SelectedTileList::HashVal() const
    {
        if (m_hash == 0)
        {
            std::vector<tile_loc> locs;
            for (auto& tile : m_tiles)
            {
                locs.push_back(tile.m_tileLoc);
            }

            m_hash = TileListHash(locs.begin(), locs.end());
        }

        return m_hash;
    }

    SelectedTileList SelectedTileList::GetTileListWithMaxLod(int lod) const
    {
        std::set<geo::tile_loc> lodTiles;
        for (auto& stile : m_tiles)
        {
            lodTiles.insert(
                stile.m_tileLoc.GetParentAtLevel(lod));
        }

        SelectedTileList outList;
        for (auto& outTile : lodTiles)
        {
            outList.Insert(SelectedTile(outTile));
        }

        return outList;
    }

#ifdef USE_XXHASH
    template<class IT> int64_t TileListHash(IT begin, IT end)
    {
        XXH64_state_t* const state = XXH64_createState();
        XXH64_hash_t const seed = 0;   /* or any other value */
        XXH64_reset(state, seed);

        for (IT it = begin; it != end; ++it)
        {
            uint64_t uid = it->GetUniqueId();
            XXH64_update(state, &uid, sizeof(uid));

        }
        int64_t hash = XXH64_digest(state);
        XXH64_freeState(state);
        return hash;
    }
#else
    template<class IT> int64_t TileListHash(IT begin, IT end)
    {
        komihash_stream_t stream;
        komihash_stream_init(&stream, 0x0123456789ABCDEF);
        for (IT it = begin; it != end; ++it)
        {
            uint64_t uid = it->GetUniqueId();
            komihash_stream_update(&stream, &uid, sizeof(uid));
        }
        return komihash_stream_final(&stream);
    }
#endif
} // namespace sam::earth
