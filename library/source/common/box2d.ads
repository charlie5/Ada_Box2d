with
     box2d_Settings;


package Box2D with pure
is
   subtype Real is box2d_Settings.Real;


   ------------
   -- User Data
   --

   --  type void_ptr is access all uint8;
   --

   subtype void_ptr is box2d_Settings.void_ptr;


   -- You can define this to inject whatever data you want in b2Body.
   --

   subtype b2BodyUserData is box2d_Settings.b2BodyUserData;


   -- You can define this to inject whatever data you want in b2Fixture.
   --

   subtype b2FixtureUserData is box2d_Settings.b2FixtureUserData;


   -- You can define this to inject whatever data you want in b2Joint.
   --

   subtype b2JointUserData is box2d_Settings.b2JointUserData;


   -- The maximum number of vertices on a convex polygon. You cannot increase
   -- this too much because b2BlockAllocator has a maximum object size.
   --
   b2_maxPolygonVertices      : constant := box2d_Settings.b2_maxPolygonVertices;


   default_max_chain_Vertices : constant := box2d_Settings.default_max_chain_Vertices;


   -- You can use this to change the length scale used by your game.
   -- For example for inches you could use '39.4'.
   --
   b2_lengthUnitsPerMeter     : constant := box2d_Settings.b2_lengthUnitsPerMeter;



private

   DEBUG : Boolean renames box2d_Settings.DEBUG;


end Box2D;
