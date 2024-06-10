#include "sam_geo_includes.h"
#include "tileloc.h"

namespace sam::geo
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // tile_loc Implementation.
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    bool tile_loc::operator<(const tile_loc &rhs) const
    {
        if (m_level < rhs.m_level)
            return true;
        else if (m_level > rhs.m_level)
            return false;

        if (m_x < rhs.m_x)
            return true;
        else if (m_x > rhs.m_x)
            return false;

        if (m_y < rhs.m_y)
            return true;
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    bool tile_loc::operator>(const tile_loc &rhs) const
    {
        if (m_level > rhs.m_level)
            return true;
        else if (m_level < rhs.m_level)
            return false;

        if (m_x > rhs.m_x)
            return true;
        else if (m_x < rhs.m_x)
            return false;

        if (m_y > rhs.m_y)
            return true;
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename T> glm::aabox2_t<T> tile_loc::GetGsBounds() const
    {
        glm::vec<2, T> tileAngularSpan(glm::two_pi<double>() / (double)(1ull << (uint64_t)m_level),
                              glm::pi<float>() / (double)(1ull << (uint64_t)m_level));

        glm::vec<2, T> minGsPt(
            float(m_x * tileAngularSpan[0] - glm::pi<float>()),
            float(m_y * tileAngularSpan[1] - glm::half_pi<T>()));
        glm::vec<2, T> maxGsPt(
            float((m_x + 1) * tileAngularSpan[0] - glm::pi<float>()),
            float((m_y + 1) * tileAngularSpan[1] - glm::half_pi<T>()));

        return glm::aabox2_t<T>(minGsPt, maxGsPt);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    tile_loc tile_loc::GetParentAtLevel(uint32_t level) const
    {
        if (level > m_level)
        {
            return *this;
        }
        if (level < 0)
        {
            level = 0;
        }

        uint32_t levelDelta = m_level - level;
        return tile_loc(level, m_x >> levelDelta, m_y >> levelDelta);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    tile_loc tile_loc::GetNorthSibling() const
    {
        unsigned int x = m_x;
        int y = m_y + 1;

        if (y >= (1 << m_level))
        {
            x = m_x;
            y = m_y;
        }

        return tile_loc(m_level, x, (unsigned int)(y));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    tile_loc tile_loc::GetSouthSibling() const
    {
        unsigned int x = m_x;
        int y = m_y - 1;

        if (y < 0)
        {
            x = m_x;
            y = 0;
        }

        return tile_loc(m_level, x, (unsigned int)(y));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    glm::vec2 tile_loc::GetCenter() const
    {        
        glm::dvec2 tileAngularSpan(glm::two_pi<double>() / (double)(1ull << (uint64_t)m_level),
                              glm::pi<float>() / (double)(1ull << (uint64_t)m_level));

        glm::vec2 gsCenter(
            (float)m_x + 0.5f * tileAngularSpan[0] - glm::pi<float>(),
            (float)m_y + 0.5f * tileAngularSpan[1] - glm::half_pi<float>());

        return gsCenter;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    glm::vec4 tile_loc::GetTranslationAndScale(const tile_loc &childTile) const
    {
        // Deal with equality situation (very common case) swiftly.
        if (*this == childTile)
        {
            return glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
        }

        // First determine if the tile is actually a child.
        int deltaLevel = (int)childTile.m_level - (int)m_level;
        if (deltaLevel < 0 ||
            childTile.m_x >> deltaLevel != m_x || childTile.m_y >> deltaLevel != m_y)
        {
            return glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
        }

        float scale = 1.0f / float(1 << deltaLevel);

        glm::vec4 translateAndScale(
            float(childTile.m_x - (m_x << deltaLevel)) * scale,
            float(childTile.m_y - (m_y << deltaLevel)) * scale,
            scale, scale);

        return translateAndScale;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    tile_loc tile_loc::Global2Local(
        const tile_loc &globalLoc) const
    {
        // Check if global location LOD is too low.
        int levelDelta = m_level - globalLoc.m_level;
        if (levelDelta < 0)
        {
            return tile_loc(sInvalidLOD, (unsigned int)0, (unsigned int)0);
        }

        // Check if the location in the LOD is outside the root tile(s).
        unsigned int globalRootX = m_x >> levelDelta;
        unsigned int globalRootY = m_y >> levelDelta;

        if (globalRootX != globalLoc.m_x || globalRootY != globalLoc.m_y)
        {
            return tile_loc(sInvalidLOD, (unsigned int)0, (unsigned int)0);
        }

        unsigned int startXAtCurrentLevel = globalLoc.m_x << levelDelta;
        unsigned int startYAtCurrentLevel = globalLoc.m_y << levelDelta;

        return tile_loc(
            levelDelta,
            m_x - startXAtCurrentLevel,
            m_y - startYAtCurrentLevel);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    tile_loc tile_loc::Local2Global(
        const tile_loc &localLoc) const
    {
        // Compute the global LOD level.
        int globalLevel = m_level + localLoc.m_level;

        // Find the base offsets at the current global LOD.
        unsigned int globalBaseX = localLoc.m_x << m_level;
        unsigned int globalBaseY = localLoc.m_y << m_level;
        unsigned int globalOffsetX = m_x;
        unsigned int globalOffsetY = m_y;

        return tile_loc(
            globalLevel,
            globalBaseX + globalOffsetX,
            globalBaseY + globalOffsetY);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename T> tile_loc tile_loc::GetTileAtGeopoint(const glm::vec<2, T>& gsPoint, uint32_t globalLOD)
    {
        unsigned int dimension = 1ull << (uint64_t)globalLOD;
        glm::vec<2, T> tileAngularSpan(glm::two_pi<double>() / dimension, glm::pi<float>() / dimension);

        unsigned int x = (unsigned int)((gsPoint[0] + glm::pi<float>()) / tileAngularSpan[0]);
        unsigned int y = (unsigned int)((gsPoint[1] + glm::half_pi<T>()) / tileAngularSpan[1]);

        x = std::min(x, dimension * 2 - 1);
        y = std::min(y, dimension - 1);

        return tile_loc(globalLOD, x, y);
    }

    inline void GetAllTilesAtLevelRecursive(uint32_t lod, tile_loc cur, std::vector<tile_loc> &outTiles)
    {
        tile_loc children[4];
        cur.GetChildren(children);
        if (lod == cur.GetLevel() + 1)
        {
            outTiles.insert(outTiles.end(), children, children + 4);
        }
        else
        {
            for (int i = 0; i < 4; ++i)
            {
                GetAllTilesAtLevelRecursive(lod, children[i], outTiles);
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void tile_loc::GetAllTilesAtLevel(uint32_t lod, std::vector<tile_loc> &outTiles)
    {
        tile_loc top(0, 0, 0);
        GetAllTilesAtLevelRecursive(lod, top, outTiles);
    }

    template glm::aabox2d tile_loc::GetGsBounds() const;
    template glm::aabox2f tile_loc::GetGsBounds() const;
    template tile_loc tile_loc::GetTileAtGeopoint(const glm::vec<2, double>& gsPoint, uint32_t globalLOD);
    template tile_loc tile_loc::GetTileAtGeopoint(const glm::vec2& gsPoint, uint32_t globalLOD);
}