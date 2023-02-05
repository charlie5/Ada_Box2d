with
     b2_Collision,
     b2_circle_Shape,
     b2_edge_Shape,
     b2_polygon_Shape,
     b2_Math;


package b2_collide_Edge
is
   procedure dummy;

   use b2_Collision,
       b2_edge_Shape,
       b2_circle_Shape,
       b2_polygon_Shape,
       b2_Math;


   --  Compute the collision manifold between an edge and a circle.
   --
   --  void b2CollideEdgeAndCircle (b2Manifold* manifold,
   --                               const b2EdgeShape*   polygonA, const b2Transform& xfA,
   --                               const b2CircleShape* circleB,  const b2Transform& xfB);

   procedure b2CollideEdgeAndCircle (manifold :    out b2Manifold;
                                     edgeA    : in     b2EdgeShape;     xfA : in b2Transform;
                                     circleB  : in     b2CircleShape;   xfB : in b2Transform);



--  // This structure is used to keep track of the best separating axis.
--  struct b2EPAxis
--  {
--    enum Type
--    {
--       e_unknown,
--       e_edgeA,
--       e_edgeB
--    };
--
--    b2Vec2 normal;
--    Type type;
--    int32 index;
--    float separation;
--  };



--  // This holds polygon B expressed in frame A.
--  struct b2TempPolygon
--  {
--    b2Vec2 vertices[b2_maxPolygonVertices];
--    b2Vec2 normals[b2_maxPolygonVertices];
--    int32 count;
--  };
--



--  // Reference face used for clipping
--  struct b2ReferenceFace
--  {
--    int32 i1, i2;
--    b2Vec2 v1, v2;
--    b2Vec2 normal;
--
--    b2Vec2 sideNormal1;
--    float sideOffset1;
--
--    b2Vec2 sideNormal2;
--    float sideOffset2;
--  };
--



--  static b2EPAxis b2ComputeEdgeSeparation(const b2TempPolygon& polygonB, const b2Vec2& v1, const b2Vec2& normal1)
--  {
--    b2EPAxis axis;
--    axis.type = b2EPAxis::e_edgeA;
--    axis.index = -1;
--    axis.separation = -FLT_MAX;
--    axis.normal.SetZero();
--
--    b2Vec2 axes[2] = { normal1, -normal1 };
--
--    // Find axis with least overlap (min-max problem)
--    for (int32 j = 0; j < 2; ++j)
--    {
--       float sj = FLT_MAX;
--
--       // Find deepest polygon vertex along axis j
--       for (int32 i = 0; i < polygonB.count; ++i)
--       {
--          float si = b2Dot(axes[j], polygonB.vertices[i] - v1);
--          if (si < sj)
--          {
--             sj = si;
--          }
--       }
--
--       if (sj > axis.separation)
--       {
--          axis.index = j;
--          axis.separation = sj;
--          axis.normal = axes[j];
--       }
--    }
--
--    return axis;
--  }



--
--  static b2EPAxis b2ComputePolygonSeparation(const b2TempPolygon& polygonB, const b2Vec2& v1, const b2Vec2& v2)
--  {
--    b2EPAxis axis;
--    axis.type = b2EPAxis::e_unknown;
--    axis.index = -1;
--    axis.separation = -FLT_MAX;
--    axis.normal.SetZero();
--
--    for (int32 i = 0; i < polygonB.count; ++i)
--    {
--       b2Vec2 n = -polygonB.normals[i];
--
--       float s1 = b2Dot(n, polygonB.vertices[i] - v1);
--       float s2 = b2Dot(n, polygonB.vertices[i] - v2);
--       float s = b2Min(s1, s2);
--
--       if (s > axis.separation)
--       {
--          axis.type = b2EPAxis::e_edgeB;
--          axis.index = i;
--          axis.separation = s;
--          axis.normal = n;
--       }
--    }
--
--    return axis;
--  }
--


   --  Compute the collision manifold between an edge and a polygon.
   --
   --  void b2CollideEdgeAndPolygon (b2Manifold* manifold,
   --                               const b2EdgeShape*    edgeA,   const b2Transform& xfA,
   --                               const b2PolygonShape* circleB, const b2Transform& xfB);

   procedure b2CollideEdgeAndPolygon (manifold :    out b2Manifold;
                                      edgeA    : in     b2edgeShape;      xfA : in b2Transform;
                                      polygonB : in     b2polygonShape;   xfB : in b2Transform);


end b2_collide_Edge;
