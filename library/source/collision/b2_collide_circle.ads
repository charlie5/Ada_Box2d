with
     b2_Collision,
     b2_circle_Shape,
     b2_polygon_Shape,
     b2_Math;


package b2_collide_Circle
is
   use b2_Collision,
       b2_circle_Shape,
       b2_polygon_Shape,
       b2_Math;


   --  Compute the collision manifold between two circles.
   --
   --  void b2CollideCircles (b2Manifold* manifold,
   --                         const b2CircleShape* circleA, const b2Transform& xfA,
   --                         const b2CircleShape* circleB, const b2Transform& xfB);

   procedure b2CollideCircles (manifold :    out b2Manifold;
                               circleA  : in     b2CircleShape;   xfA : in b2Transform;
                               circleB  : in     b2CircleShape;   xfB : in b2Transform);


   --  Compute the collision manifold between a polygon and a circle.
   --
   --  void b2CollidePolygonAndCircle (b2Manifold* manifold,
   --                                  const b2PolygonShape* polygonA, const b2Transform& xfA,
   --                                  const b2CircleShape*  circleB,  const b2Transform& xfB);

   procedure b2CollidePolygonAndCircle (manifold :    out b2Manifold;
                                        polygonA : in     b2PolygonShape;   xfA : in b2Transform;
                                        circleB  : in     b2CircleShape;    xfB : in b2Transform);


end b2_collide_Circle;
