with
     box2d.b2_Collision,
     box2d.b2_circle_Shape,
     box2d.b2_edge_Shape,
     box2d.b2_polygon_Shape,
     box2d.b2_Math,
     --  box2d.b2_Settings,

     Interfaces;


package box2d.b2_collide_Edge
is
   use b2_Collision,
       b2_edge_Shape,
       b2_circle_Shape,
       b2_polygon_Shape,
       b2_Math,
       --  b2_Settings,
       Interfaces;



   --  Compute the collision manifold between an edge and a circle.
   --
   --  void b2CollideEdgeAndCircle (b2Manifold* manifold,
   --                               const b2EdgeShape*   polygonA, const b2Transform& xfA,
   --                               const b2CircleShape* circleB,  const b2Transform& xfB);

   procedure b2CollideEdgeAndCircle (manifold :    out b2Manifold;
                                     edgeA    : in     b2EdgeShape;     xfA : in b2Transform;
                                     circleB  : in     b2CircleShape;   xfB : in b2Transform);





   --  // This structure is used to keep track of the best separating axis.
   --
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

   type b2EPAxis_Kind is (e_unknown,
                          e_edgeA,
                          e_edgeB);

   type b2EPAxis is
      record
           Normal     : b2Vec2;
           Kind       : b2EPAxis_Kind;
           Index      : Integer;
           Separation : Real;
      end record;





   --  // This holds polygon B expressed in frame A.
   --
   --  struct b2TempPolygon
   --  {
   --    b2Vec2 vertices[b2_maxPolygonVertices];
   --    b2Vec2 normals[b2_maxPolygonVertices];
   --    int32 count;
   --  };
   --

   type tempPolygonVec2s is array (Unsigned_8 range 0 .. b2_maxPolygonVertices - 1) of b2Vec2;

   type b2TempPolygon is
      record
         vertices : tempPolygonVec2s;
         normals  : tempPolygonVec2s;
         count    : Unsigned_8;
      end record;




   --  // Reference face used for clipping
   --
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

   type b2ReferenceFace is
      record
         i1, i2 : Natural;
         v1, v2 : b2Vec2;
         normal : b2Vec2;

         sideNormal1 : b2Vec2;
         sideOffset1 : Real;

         sideNormal2 : b2Vec2;
         sideOffset2 : Real;
      end record;



   --  static b2EPAxis b2ComputeEdgeSeparation (const b2TempPolygon& polygonB, const b2Vec2& v1, const b2Vec2& normal1)
   --

   function b2computeEdgeSeparation (polygonB : in b2TempPolygon;
                                     v1       : in b2Vec2;
                                     normal1  : in b2Vec2) return b2EPAxis;





   --  static b2EPAxis b2ComputePolygonSeparation(const b2TempPolygon& polygonB, const b2Vec2& v1, const b2Vec2& v2)
   --

   function b2computePolygonSeparation (polygonB : in b2TempPolygon;
                                        v1, v2   : in b2Vec2) return b2EPAxis;




   --  Compute the collision manifold between an edge and a polygon.
   --
   --  void b2CollideEdgeAndPolygon (b2Manifold* manifold,
   --                               const b2EdgeShape*    edgeA,   const b2Transform& xfA,
   --                               const b2PolygonShape* circleB, const b2Transform& xfB);
   --

   procedure b2CollideEdgeAndPolygon (manifold :    out b2Manifold;
                                      edgeA    : in     b2edgeShape;      xfA : in b2Transform;
                                      polygonB : in     b2polygonShape;   xfB : in b2Transform);


end box2d.b2_collide_Edge;
