with
     b2_Settings,
     b2_Types,
     ada.Numerics;


package b2_Common
is
   use b2_Settings;


   b2DEBUG : Boolean := True;


   --  #define B2_NOT_USED(x) ((void)(x))
   --  #define b2Assert(A) assert(A)


   --  #define b2_maxFloat    FLT_MAX
   --
   b2_maxFloat : constant := Real'Last;


   --  #define b2_epsilon     FLT_EPSILON
   --
   b2_Epsilon  : constant := Real'model_Epsilon;


   --  #define b2_pi       3.14159265359f
   --
   b2_Pi       : constant := ada.Numerics.Pi;




   --  Global tuning constants based on meters-kilograms-seconds (MKS) units.
   --


   --  Collision
   --

   --  The maximum number of contact points between two convex shapes. Do
   --  not change this value.
   --
   b2_maxManifoldPoints : constant := 2;


   --  This is used to fatten AABBs in the dynamic tree. This allows proxies
   --  to move by a small amount without triggering a tree adjustment.
   --  This is in meters.
   --
   b2_aabbExtension  : constant := 0.1 * b2_lengthUnitsPerMeter;


   --  This is used to fatten AABBs in the dynamic tree. This is used to predict
   --  the future position based on the current displacement.
   --  This is a dimensionless multiplier.
   --
   b2_aabbMultiplier : constant := 4.0;


   --  A small length used as a collision and constraint tolerance. Usually it is
   --  chosen to be numerically significant, but visually insignificant. In meters.
   --
   b2_linearSlop     : constant := 0.005 * b2_lengthUnitsPerMeter;


   --  A small angle used as a collision and constraint tolerance. Usually it is
   --  chosen to be numerically significant, but visually insignificant.
   --
   b2_angularSlop    : constant := 2.0 / 180.0 * b2_Pi;


   --  The radius of the polygon/edge shape skin. This should not be modified. Making
   --  this smaller means polygons will have an insufficient buffer for continuous collision.
   --  Making it larger may create artifacts for vertex collision.
   --
   b2_polygonRadius  : constant := 2.0 * b2_linearSlop;


   --  Maximum number of sub-steps per contact in continuous physics simulation.
   --
   b2_maxSubSteps    : constant := 8;



   --  Dynamics
   --

   --  Maximum number of contacts to be handled to solve a TOI impact.
   --
   b2_maxTOIContacts        : constant := 32;


   --  The maximum linear position correction used when solving constraints. This helps to
   --  prevent overshoot. Meters.
   --
   b2_maxLinearCorrection   : constant := 0.2 * b2_lengthUnitsPerMeter;


   --  The maximum angular position correction used when solving constraints. This helps to
   --  prevent overshoot.
   --
   b2_maxAngularCorrection  : constant := 8.0 / 180.0 * b2_Pi;


   --  The maximum linear translation of a body per step. This limit is very large and is used
   --  to prevent numerical problems. You shouldn't need to adjust this. Meters.
   --
   b2_maxTranslation        : constant := 2.0 * b2_lengthUnitsPerMeter;
   b2_maxTranslationSquared : constant := b2_maxTranslation * b2_maxTranslation;


   --  The maximum angular velocity of a body. This limit is very large and is used
   --  to prevent numerical problems. You shouldn't need to adjust this.
   --
   b2_maxRotation           : constant := 0.5 * b2_Pi;
   b2_maxRotationSquared    : constant := b2_maxRotation * b2_maxRotation;


   --  This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
   --  that overlap is removed in one time step. However using values close to 1 often lead
   --  to overshoot.
   --
   b2_Baumgarte    : constant := 0.2;
   b2_toiBaumgarte : constant := 0.75;



   --  Sleep
   --

   --  The time that a body must be still before it will go to sleep.
   --
   b2_timeToSleep : constant := 0.5;


   --  A body cannot sleep if its linear velocity is above this tolerance.
   --
   b2_linearSleepTolerance : constant := 0.01 * b2_lengthUnitsPerMeter;


   --  A body cannot sleep if its angular velocity is above this tolerance.
   --
   b2_angularSleepTolerance : constant := 2.0 / 180.0 * b2_Pi;




   --------
   -- Dumps
   --

   --  Dump to a file. Only one dump file allowed at a time.
   --

   --  void b2OpenDump (const char* fileName);
   --
   procedure b2OpenDump (fileName : in String);


   --  void b2Dump     (const char* string, ...);
   --
   procedure b2Dump (Message : in String);


   --  void b2CloseDump();
   --
   procedure b2CloseDump;




   -----------------------------
   --  Version numbering scheme.
   --

   --  See http://en.wikipedia.org/wiki/Software_versioning.
   --
   --  struct b2Version
   --  {
   --    int32 major;      ///< significant changes
   --    int32 minor;      ///< incremental changes
   --    int32 revision;   ///< bug fixes
   --  };

   use b2_Types;

   type b2Version is
      record
         Major,                -- Significant changes.
         Minor,                -- Incremental changes.
         Revision : int32;     -- Bug fixes.
      end record;

   --  Current version.
   --

   --  extern b2Version b2_version;
   --
   b2_Version : constant b2Version := (Major    => 2,
                                       Minor    => 4,
                                       Revision => 0);


end b2_Common;
