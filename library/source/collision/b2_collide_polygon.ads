with
     b2_Collision,
     b2_polygon_Shape,
     b2_Math,
     b2_Settings;


package b2_collide_Polygon
is
   use b2_Collision,
       b2_polygon_Shape,
       b2_Math,
       b2_Settings;


   --  Find the max separation between poly1 and poly2 using edge normals from poly1.
   --
   --  static float b2FindMaxSeparation (int32* edgeIndex,
   --                                    const b2PolygonShape* poly1, const b2Transform& xf1,
   --                                    const b2PolygonShape* poly2, const b2Transform& xf2)

   function b2FindMaxSeparation (edgeIndex : out Natural;   poly1 : in b2PolygonShape;   xf1 : in b2Transform;
                                                            poly2 : in b2PolygonShape;   xf2 : in b2Transform) return Real;



   --  static void b2FindIncidentEdge (b2ClipVertex c[2],
   --                                  const b2PolygonShape* poly1, const b2Transform& xf1, int32 edge1,
   --                                  const b2PolygonShape* poly2, const b2Transform& xf2)

   procedure b2FindIncidentEdge (c : out b2ClipVertex_Pair;   poly1 : in b2PolygonShape;   xf1 : in b2Transform;
                                                              Edge1 : in Natural;
                                                              poly2 : in b2PolygonShape;   xf2 : in b2Transform);

   --  Compute the collision manifold between two polygons.
   --
   --  void b2CollidePolygons (b2Manifold* manifold,
   --                          const b2PolygonShape* polygonA, const b2Transform& xfA,
   --                          const b2PolygonShape* polygonB, const b2Transform& xfB);

   procedure b2CollidePolygons (manifold : out b2Manifold;   polygonA : in b2PolygonShape;   xfA : in b2Transform;
                                                             polygonB : in b2PolygonShape;   xfB : in b2Transform);

end b2_collide_Polygon;
