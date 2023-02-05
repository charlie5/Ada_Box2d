with
     b2_Types,
     System;


package b2_Settings
--
-- Settings that can be overriden for your application.
--
is
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



   -- User Data
   --

   -- You can define this to inject whatever data you want in b2Body.
   --
   -- struct b2BodyUserData
   -- {
   --   b2BodyUserData()
   --   {
   --      pointer = 0;
   --   }
   --
   --   /// For legacy compatibility
   --   uintptr_t pointer;
   -- };

   type b2BodyUserData is
      record
         Pointer : system.Address := system.null_Address;
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
   --   /// For legacy compatibility
   --   uintptr_t pointer;
   -- };

   type b2FixtureUserData is
      record
         Pointer : system.Address := system.null_Address;
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
   --   /// For legacy compatibility
   --   uintptr_t pointer;
   -- };

   type b2JointUserData is
      record
         Pointer : system.Address := system.null_Address;
      end record;



   -- Memory Allocation
   --
   -- Default allocation functions.
   --
   -- void* b2Alloc_Default (int32   size);
   -- void   b2Free_Default (void*   mem);



   -- Implement this function to use your own memory allocator.
   --
   -- inline void* b2Alloc (int32 size)
   -- {
   --   return b2Alloc_Default(size);
   -- }
   --
   --
   -- If you implement b2Alloc, you should also implement this function.
   --
   -- inline void b2Free (void* mem)
   -- {
   --   b2Free_Default (mem);
   -- }



   -- Default logging function.
   --
   -- void b2Log_Default (const char*   string,
   --                     va_list       args);

   procedure b2Log_Default (Message : in String);


   -- Implement this to use your own logging.
   --
   -- inline void b2Log (const char*   string, ...)
   -- {
   --   va_list   args;
   --
   --   va_start (args, string);
   --   b2Log_Default (string, args);
   --   va_end (args);
   -- }

   procedure b2Log (Message : in String);


   --
   --
   -- #endif // B2_USER_SETTINGS



   -- #include "b2_common"

end b2_Settings;
