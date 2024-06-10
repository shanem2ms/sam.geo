#pragma once
#include "tileloc.h"
#include "spherical_mercator_projection.h"
#include <set>

namespace sam::geo
{
    template<class IT> int64_t TileListHash(IT begin, IT end);

    class SelectedTile
    {
    public:
        SelectedTile(const geo::tile_loc& tileLoc) :
            m_tileLoc(tileLoc)
        {
            memset(&m_childIsInSelection, 0, sizeof(m_childIsInSelection));
        }
        bool operator<(const SelectedTile& rhs) const
        {
            return m_tileLoc < rhs.m_tileLoc;
        }

        geo::tile_loc m_tileLoc;
        bool m_childIsInSelection[4];
    };

    class SelectedTileList
    {
        std::set<SelectedTile> m_tiles;
        mutable int64_t m_hash;
    public:

        typedef std::set<SelectedTile>::iterator iterator;
        typedef std::set<SelectedTile>::const_iterator const_iterator;
        void Insert(const SelectedTile& tile);
        size_t Size();
        int64_t HashVal() const;
        bool Contains(const geo::tile_loc& loc) const;
        iterator begin() { return m_tiles.begin(); }
        iterator end() { return m_tiles.end(); }
        const_iterator begin() const { return m_tiles.begin(); }
        const_iterator end() const { return m_tiles.end(); }
        SelectedTileList GetTileListWithMaxLod(int lod) const;
    };


    class TileSelection
    {
        bool CollectAllNeighbors(std::set<geo::tile_loc>& tiles, const geo::tile_loc& loc, std::set<uint32_t>& visited,
            const glm::dvec3& viewPos, const glm::dmat4x4& viewProj,
            const glm::dvec3& lookVec,
            const glm::planed greatCirclePlane, double greatCircleRadius,
            bool forceAdaquateRes, float minAltitude, float maxAltitude);

        float GetTileScreenArea(const geo::tile_loc& tileLoc, const glm::dmat4x4& viewProj, float minAltitude, float maxAltitude);
        double TargetLodAtWorldPt(glm::dvec3& worldPt, const glm::dmat4x4& viewProj, const glm::dmat4x4& invViewProj);
        bool IsTileFacingViewer(const geo::tile_loc& tileLoc, const glm::dvec3& lookVec);
        float TileAngle(const geo::tile_loc& tileLoc, const glm::dvec3& lookVec);
        bool TileDistCheck(const geo::tile_loc& tileLoc, const glm::dvec3& viewPos);
        bool IsTileInView(const geo::tile_loc& tileLoc, const glm::dmat4x4& viewProj,
            const glm::dvec3& lookVec, const glm::planed greatCirclePlane,
            double greatCircleRadius, float minAltitude, float maxAltitude);
        bool IsSelectedTileInView(const SelectedTile& tile, const glm::dmat4x4& viewProj,
            const glm::dvec3& lookVec,
            const glm::planed greatCirclePlane, double greatCircleRadius,
            float minAltitude, float maxAltitude);
        bool WorldPointInView(glm::dvec3& worldPt, const glm::dmat4x4& viewProj);
        bool GetEarthHorizonCircle(const glm::dvec3& wsEye, glm::planed& circlePlane, double& radius) const;
    public:
        void GetTileList(SelectedTileList& tiles, const glm::dvec3& viewPos, const glm::dmat4x4& invViewProj,
            float minAltitude, float maxAltitude, int maxLOD);
        bool GetGeoPtAtView(const glm::dvec3& viewPos,
            const glm::dmat4x4& viewProj, float testAltitude, glm::dvec2& outPt);

        void SetProjection(const SphericalMercatorProjection<double>& projection)
        {
            m_sphericalMercatorProjection = projection;
        }
    private:
        SphericalMercatorProjection<double> m_sphericalMercatorProjection;
    };
}
