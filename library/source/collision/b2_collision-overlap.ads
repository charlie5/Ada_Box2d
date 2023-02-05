with
     b2_Math,
     b2_Types,
     b2_Common,
     b2_Shape,

     interfaces.C;


package b2_Collision.overlap
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

   function b2TestOverlap (shapeA : in b2Shape;   indexA : in int32;   xfA : in b2Transform;
                           shapeB : in b2Shape;   indexB : in int32;   xfB : in b2Transform) return Boolean;


end b2_Collision.overlap;