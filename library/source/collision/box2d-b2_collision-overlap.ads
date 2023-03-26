with
     box2d.b2_Math,
     box2d.b2_Types,
     box2d.b2_Common,
     box2d.b2_Shape,

     interfaces.C;


package box2d.b2_Collision.overlap
is
   use b2_Shape;
   --  use b2_Math,
   --      b2_Types,
   --      b2_Common;

   --  Determine if two generic shapes overlap.
   --
   --  bool b2TestOverlap (const b2Shape* shapeA, int32 indexA,
   --                      const b2Shape* shapeB, int32 indexB,
   --                      const b2Transform& xfA, const b2Transform& xfB);

   function b2TestOverlap (shapeA : access b2Shape;       indexA : in Natural;
                           shapeB : access b2Shape;       indexB : in Natural;
                           xfA    : in     b2Transform;   xfB    : in b2Transform) return Boolean;


end box2d.b2_Collision.overlap;
