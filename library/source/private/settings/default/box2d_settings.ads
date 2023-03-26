with
     System;


package box2d_Settings with pure
--
-- Settings that can be overriden for your application.
--
-- Set the "Box2d_Settings_Mode" scenario variable to 'custom', in your build, if you want to override settings.
--
-- Then use "box2d/library/source/private/settings/custom/box2d_settings.ads" to define your custom settings.
--
is
   DEBUG : constant Boolean := True;



   subtype Real is Float;



   --------------------
   -- Tunable Constants
   --

   -- You can use this to change the length scale used by your game.
   -- For example for inches you could use '39.4'.
   --
   b2_lengthUnitsPerMeter     : constant := 1.0;


   -- The maximum number of vertices on a convex polygon. You cannot increase
   -- this too much because b2BlockAllocator has a maximum object size.
   --
   b2_maxPolygonVertices      : constant := 8;



   default_max_chain_Vertices : constant := 16;




   ------------
   -- User Data
   --

   --  type void_ptr is access all uint8;
   --

   subtype void_ptr is system.Address;


   -- You can define this to inject whatever data you want in b2Body.
   --
   -- struct b2BodyUserData
   -- {
   --   b2BodyUserData()
   --   {
   --      pointer = 0;
   --   }
   --
   --   For legacy compatibility
   --   uintptr_t pointer;
   -- };
   --

   type b2BodyUserData is
      record
         Pointer : void_ptr;
      end record;



   -- You can define this to inject whatever data you want in b2Fixture.
   --
   -- struct b2FixtureUserData
   -- {
   --   b2FixtureUserData()
   --   {
   --      pointer = 0;
   --   }
   --
   --   For legacy compatibility
   --   uintptr_t pointer;
   -- };
   --

   type b2FixtureUserData is
      record
         Pointer : void_ptr;
      end record;



   -- You can define this to inject whatever data you want in b2Joint.
   --
   -- struct b2JointUserData
   -- {
   --   b2JointUserData()
   --   {
   --      pointer = 0;
   --   }
   --
   --   For legacy compatibility
   --   uintptr_t pointer;
   -- };

   type b2JointUserData is
      record
         Pointer : void_ptr;
      end record;


end box2d_Settings;
