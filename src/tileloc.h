#pragma once

namespace sam::geo
{

// Forward declarations.
    
    /// Class encapsulating the level of detail and the xy location
    /// of the tile within the grid at that level of detail.
    class tile_loc
    {
    public:
        static const uint32_t sInvalidLOD = 0xffffffff;

        /// Default Constructor.  Produces first tile of lowest LOD.
        tile_loc() :
            m_level(sInvalidLOD),
            m_x(0),
            m_y(0)
        { }

        /// Construction by parameters.
        /// \param level LOD level of the tile.
        /// \param x The coordinate of the tile along the width (longitude)
        /// \param y The coordinate of the tile along the height (latitude)
        tile_loc(uint32_t level, uint32_t x, uint32_t y) :
            m_level(level),
            m_x(x),
            m_y(y)
        { }

        /// Destructor
        ~tile_loc()
        { }

        /// Get/set level.
        void SetLevel(uint32_t level)
        {
            m_level = level;
        }

        uint32_t GetLevel() const
        {
            return m_level;
        }

        /// Get/set x.
        void SetX(uint32_t x)
        {
            m_x = x;
        }

        uint32_t GetX() const
        {
            return m_x;
        }

        /// Get/set y.
        void SetY(uint32_t y)
        {
            m_y = y;
        }

        uint32_t GetY() const
        {
            return m_y;
        }

        uint32_t operator [](int idx) const
        { return *(&m_x + idx); }
        /// Equality comparison.
        bool operator==(const tile_loc & rhs) const;

        /// Inequality comparison.
        bool operator!=(const tile_loc & rhs) const
        {
            return !(*this == rhs);
        }

        /// Less Than comparison.
        bool operator<(const tile_loc & rhs) const;

        /// Greater Than comparison.
        bool operator>(const tile_loc & rhs) const;

        /// Less Than or Equal comparison.
        bool operator<=(const tile_loc & rhs) const
        {
            return !(*this > rhs);
        }

        /// Greater Than or Equal comparison.
        bool operator>=(const tile_loc & rhs) const
        {
            return !(*this < rhs);
        }

        /// Returns an index of the tile in the grid of the LOD.  
        /// Effectively this equals y times the width of the grid plus x.
        uint32_t GetIndex() const
        {
            return m_y * (2 << m_level) + m_x;
        }

        /// Returns the bounding box of the tile in longitude, latitude (radians).
        template<typename T> glm::aabox2_t<T> GetGsBounds() const;

        template<typename T> glm::aabox2_t<T> GetBounds() const
        {
            return GetGsBounds<T>();
        }

        /// Returns the root hemisphere where the location resides.
        int GetRootHemisphere() const
        {
            return (m_x / (1 << m_level)) ? 1 : 0;
        }

        /// Returns the location of a parent tile.  If the tile is at level 0,
        /// returns itself.
        tile_loc GetParent() const
        {
            if (m_level == 0) return *this;
            return tile_loc(m_level - 1, m_x >> 1, m_y >> 1);
        }

        /// Returns the location of a parent tile at the given level.
        /// \exception eInvalidArgument if level > GetLevel().
        tile_loc GetParentAtLevel(uint32_t level) const;

        /// Returns whether this is a parent tile of the tile provided.  We
        /// are not our own parent.
        bool IsParentOf(const tile_loc & tileLocation) const
        {
            return (m_level < tileLocation.m_level&&
                m_x == tileLocation.m_x >> (tileLocation.m_level - m_level) &&
                m_y == tileLocation.m_y >> (tileLocation.m_level - m_level));
        }


        static void GetAllTilesAtLevel(uint32_t lod, std::vector<tile_loc> &outTiles);

        /// Returns the location of the child at the quadrant identified.
        tile_loc GetNorthwestChild() const;
        tile_loc GetNortheastChild() const;
        tile_loc GetSouthwestChild() const;
        tile_loc GetSoutheastChild() const;

        /// Stores children in the order: NW, NE, SW, SE.
        /// TileLocationItr can be a raw pointer, a std iterator, or anything
        /// else that supports incrementing (operator++) and dereferencing
        /// (operator*).
        /// Requires: this must be able to dereference tileLocationItr,
        /// increment it three times, and dereference it after each increment.
        static const size_t ChildCount = 4;
        template <typename TileLocationItr>
        void GetChildren(TileLocationItr tileLocationItr) const;

        /// Returns the sibling/neighbor in the direction identified.  The method
        /// wraps through the poles.
        tile_loc GetNorthSibling() const;
        tile_loc GetSouthSibling() const;
        tile_loc GetEastSibling() const;
        tile_loc GetWestSibling() const;

        /// Stores siblings in the order: E, NE, N, NW, W, SW, S, SE
        /// TileLocationItr can be a raw pointer, a std iterator, or anything
        /// else that supports incrementing (operator++) and dereferencing
        /// (operator*).
        /// Requires: the storage referenced by tileLocationItr must provide
        /// space for 8 siblings.
        static const size_t SiblingCount = 8;
        template <typename TileLocationItr>
        void GetSiblings(TileLocationItr tileLocationItr) const;

        /// Like GetSiblings but returns only those that are unique.
        /// Naturally, this is more expensive than GetSiblings.
        /// The storage referenced by tileLocationItr must still provide
        /// space for SiblingCount siblings.
        /// NOTE: This provides a different result from GetSiblings when
        /// the count of tile rows or columns is less than 3. For our
        /// current (late 2008) tiling of the earth, that is only
        /// LODS 0 and 1. KAH
        /// \return the number of siblings provided.
        template <typename TileLocationItr>
        size_t GetUniqueSiblings(TileLocationItr tileLocationItr) const;

        /// Returns tile which spans a geographic point at specified global LOD.
        template<typename T> static tile_loc GetTileAtGeopoint(const glm::vec<2, T> & gsPoint, uint32_t globalLOD);

        /// Determines whether a tile is southernmost, northernmost, etc.
        bool IsSouthernmost() const
        {
            return bool(m_y == 0);
        }

        bool IsNorthernmost() const
        {
            return bool(m_y == (uint32_t)(1 << m_level) - 1);
        }

        bool IsWesternmost() const
        {
            return bool(m_x == 0);
        }

        bool IsEasternmost() const
        {
            return bool(m_x == (uint32_t)((1 << m_level) * 2 - 1));
        }

        /// Returns the Cartesian center geographic point of the tile.
        glm::vec2 GetCenter() const;

        /// Determines whether a geographic point is in the tile.
        template <typename T> bool Contains(const glm::vec<2, T> & gsPoint) const
        {
            return GetGsBounds<T>().contains(gsPoint);
        }

        /// Returns the translation and scale to be applied to a tile with this
        /// location to match the provided location, if this is a parent of
        /// childTile. Otherwise, returns a vector of all zeroes.
        glm::vec4 GetTranslationAndScale(const tile_loc & childTile) const;

        /// Returns a location relative to the input global location, if this
        /// location is in the tree described by the inputs. Otherwise returns
        /// a location whose m_level is sInvalidLOD.
        tile_loc Global2Local(const tile_loc &globalLoc) const;

        /// Sets the tile location to be invalid.
        void SetInvalid()
        {
            m_level = sInvalidLOD;
        }

        /// Generate a global location from a local location.
        tile_loc Local2Global(const tile_loc &globalLoc) const;

        /// Returns whether the tile location is a valid tile location.
        bool IsValid() const
        {
            return m_level != sInvalidLOD;
        }

        uint64_t GetUniqueId() const
        {
            return ((uint64_t)(m_level)) << 58 |
                ((uint64_t)(m_y)) << 29 |
                ((uint64_t)(m_x));
        }

    public:
        /// LOD level of the tile.  Global LODs start at 0 (western hemisphere,
        /// eastern hemisphere) and increase as the detail gets greater.
        uint32_t m_level;
        
        /// The coordinate of the tile along the width (longitude).  0 starts at
        /// -180 degrees with increasing x to the east.
        uint32_t m_x;

        /// The coordinate of the tile along the height (latitude).  0 starts at
        /// -90 degrees (south pole) with increasing y to the north.
        uint32_t m_y;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Inlines
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline bool tile_loc::operator==(const tile_loc& rhs) const
    {
        return (m_level == rhs.m_level &&
            m_x == rhs.m_x &&
            m_y == rhs.m_y);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline tile_loc tile_loc::GetNorthwestChild() const
    {
        return tile_loc(m_level + 1, m_x * 2, m_y * 2 + 1);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline tile_loc tile_loc::GetNortheastChild() const
    {
        return tile_loc(m_level + 1, m_x * 2 + 1, m_y * 2 + 1);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline tile_loc tile_loc::GetSouthwestChild() const
    {
        return tile_loc(m_level + 1, m_x * 2, m_y * 2);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline tile_loc tile_loc::GetSoutheastChild() const
    {
        return tile_loc(m_level + 1, m_x * 2 + 1, m_y * 2);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline tile_loc tile_loc::GetEastSibling() const
    {
        return tile_loc(m_level, (m_x + 1) % (1 << m_level), m_y);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline tile_loc tile_loc::GetWestSibling() const
    {
        return tile_loc(m_level, (m_x + (1 << m_level) - 1) % (1 << m_level), m_y);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    template <typename TileLocationItr>
    inline void tile_loc::GetChildren(TileLocationItr tileLocationItr) const
    {
        *tileLocationItr = tile_loc(m_level + 1, m_x * 2, m_y * 2);
        ++tileLocationItr;
        *tileLocationItr = tile_loc(m_level + 1, m_x * 2 + 1, m_y * 2);
        ++tileLocationItr;
        *tileLocationItr = tile_loc(m_level + 1, m_x * 2, m_y * 2 + 1);
        ++tileLocationItr;
        *tileLocationItr = tile_loc(m_level + 1, m_x * 2 + 1, m_y * 2 + 1);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    /// Inlined not because it should be on its merits, but to avoid the
    /// template instantiation hassle. KAH
    template <typename TileLocationItr>
    inline void tile_loc::GetSiblings(TileLocationItr tileLocationItr) const
    {
        tile_loc tileLocation;

        tileLocation = GetEastSibling();                    // East
        *tileLocationItr = tileLocation;
        ++tileLocationItr;

        *tileLocationItr = tileLocation.GetNorthSibling();  // Northeast
        ++tileLocationItr;

        tileLocation = GetNorthSibling();                   // North
        *tileLocationItr = tileLocation;
        ++tileLocationItr;

        *tileLocationItr = tileLocation.GetWestSibling();   // Northwest
        ++tileLocationItr;

        tileLocation = GetWestSibling();                    // West
        *tileLocationItr = tileLocation;
        ++tileLocationItr;

        *tileLocationItr = tileLocation.GetSouthSibling();  // Southwest
        ++tileLocationItr;

        tileLocation = GetSouthSibling();                   // South
        *tileLocationItr = tileLocation;
        ++tileLocationItr;

        *tileLocationItr = tileLocation.GetEastSibling();   // Southeast
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    template <typename TileLocationItr>
    inline bool WxwIsUniqueSibling(
        const tile_loc& sibling,
        TileLocationItr beginItr,
        TileLocationItr endItr)
    {
        return std::find(beginItr, endItr, sibling) == endItr;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    /// Inlined not because it should be on its merits, but to avoid the
    /// template instantiation hassle. KAH
    template <typename TileLocationItr>
    inline size_t tile_loc::GetUniqueSiblings(TileLocationItr tileLocationItr) const
    {
        tile_loc tileLocation;
        TileLocationItr tileLocationBeginItr = tileLocationItr;

        // East
        tileLocation = GetEastSibling();
        *tileLocationItr = tileLocation;
        ++tileLocationItr;
        size_t siblingCount = 1;

        // Northeast
        tileLocation = tileLocation.GetNorthSibling();
        if (tileLocation != *tileLocationBeginItr)  // a cheaper test since this is sibling #2!
        {
            *tileLocationItr = tileLocation;
            ++siblingCount;
            ++tileLocationItr;
        }

        // North
        tileLocation = GetNorthSibling();
        if (WxwIsUniqueSibling(tileLocation, tileLocationBeginItr, tileLocationItr))
        {
            *tileLocationItr = tileLocation;
            ++siblingCount;
            ++tileLocationItr;
        }

        // Northwest
        tileLocation = tileLocation.GetWestSibling();
        if (WxwIsUniqueSibling(tileLocation, tileLocationBeginItr, tileLocationItr))
        {
            *tileLocationItr = tileLocation;
            ++siblingCount;
            ++tileLocationItr;
        }

        // West
        tileLocation = GetWestSibling();
        if (WxwIsUniqueSibling(tileLocation, tileLocationBeginItr, tileLocationItr))
        {
            *tileLocationItr = tileLocation;
            ++siblingCount;
            ++tileLocationItr;
        }

        // Southwest
        tileLocation = tileLocation.GetSouthSibling();
        if (WxwIsUniqueSibling(tileLocation, tileLocationBeginItr, tileLocationItr))
        {
            *tileLocationItr = tileLocation;
            ++siblingCount;
            ++tileLocationItr;
        }

        // South
        tileLocation = GetSouthSibling();
        if (WxwIsUniqueSibling(tileLocation, tileLocationBeginItr, tileLocationItr))
        {
            *tileLocationItr = tileLocation;
            ++siblingCount;
            ++tileLocationItr;
        }

        // Southeast
        tileLocation = tileLocation.GetEastSibling();
        if (WxwIsUniqueSibling(tileLocation, tileLocationBeginItr, tileLocationItr))
        {
            *tileLocationItr = tileLocation;
            ++siblingCount;
        }

        return siblingCount;
    }

}
