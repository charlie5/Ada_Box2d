with
     b2_Types,
     System;


package b2_Settings
--
-- Settings that can be overriden for your application.
--
is
   use b2_Types;

   DEBUG : constant Boolean := True;



   -- Define this macro in your build if you want to override settings.
   --
   -- #ifdef B2_USER_SETTINGS
   --
   --    This is a user file that includes custom definitions of the macros, structs, and functions defined below.
   --
   --    #include "b2_user_settings"
   --
   --
   -- #else
   --
   --    #include <stdarg.h>
   --    #include <stdint.h>

   subtype Real is Float;


   -- Tunable Constants

   -- You can use this to change the length scale used by your game.
   -- For example for inches you could use '39.4'.
   --
   b2_lengthUnitsPerMeter : constant := 1.0;

   -- The maximum number of vertices on a convex polygon. You cannot increase
   -- this too much because b2BlockAllocator has a maximum object size.
   --
   b2_maxPolygonVertices  : constant := 8;


   default_max_chain_Vertices : constant := 16;




   -- User Data
   --

   --  type void_ptr is access all uint8;
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




   --------------------
   -- Memory Allocation
   --

   -- Default allocation functions.
   --

   -- void* b2Alloc_Default (int32   size);

   --  function b2alloc_default (Size : in int32) return void_ptr;



   -- void   b2Free_Default (void*   mem);

   --  procedure b2free_default (Mem : in out void_ptr);



   -- Implement this function to use your own memory allocator.
   --
   -- inline void* b2Alloc (int32 size)
   -- {
   --   return b2Alloc_Default(size);
   -- }

   --  function b2alloc (Size : in int32) return void_ptr
   --    with Inline;


   -- If you implement b2Alloc, you should also implement this function.
   --
   -- inline void b2Free (void* mem);

   --  procedure b2free (Mem : in out void_ptr)
   --    with Inline;




   --
   --
   -- #endif // B2_USER_SETTINGS



   -- #include "b2_common"

end b2_Settings;
