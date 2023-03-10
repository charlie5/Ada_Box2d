with
     b2_Math,
     b2_Settings;


package b2_time_Step
is
   use b2_Math,
       b2_Settings;



   --  Profiling data. Times are in milliseconds.
   --
   --  struct b2Profile
   --  {
   --    float step;
   --    float collide;
   --    float solve;
   --    float solveInit;
   --    float solveVelocity;
   --    float solvePosition;
   --    float broadphase;
   --    float solveTOI;
   --  };
   --

   type b2Profile is
      record
         step,
         collide,
         solve,
         solveInit,
         solveVelocity,
         solvePosition,
         broadphase,
         solveTOI     : Real;
      end record;




   --  This is an internal structure.
   --
   --  struct b2TimeStep
   --  {
   --    float dt;         // time step
   --    float inv_dt;     // inverse time step (0 if dt == 0).
   --    float dtRatio; // dt * inv_dt0
   --    int32 velocityIterations;
   --    int32 positionIterations;
   --    bool warmStarting;
   --  };
   --

   type b2TimeStep is
      record
         dt                 : Real;     -- Time step.
         inv_dt             : Real;     -- Inverse time step (0 if dt == 0).
         dtRatio            : Real;     -- dt * inv_dt0.

         velocityIterations,
         positionIterations : Natural;

         warmStarting       : Boolean;
      end record;




   --  This is an internal structure.
   --
   --  struct b2Position
   --  {
   --    b2Vec2 c;
   --    float a;
   --  };
   --

   type b2Position is
      record
         c : b2Vec2;
         a : Real;
      end record;




   --  This is an internal structure.
   --
   --  struct b2Velocity
   --  {
   --    b2Vec2 v;
   --    float w;
   --  };
   --

   type b2Velocity is
      record
         v : b2Vec2;
         w : Real;
      end record;




   --  Solver Data
   --
   --  struct b2SolverData
   --  {
   --    b2TimeStep step;
   --    b2Position* positions;
   --    b2Velocity* velocities;
   --  };
   --

   type b2SolverData is
      record
           step       :        b2TimeStep;
           positions  : access b2Position;
           velocities : access b2Velocity;
      end record;


end b2_time_Step;
