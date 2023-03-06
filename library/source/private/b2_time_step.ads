with
     b2_Math;


package b2_time_Step
is

   --
--  Profiling data. Times are in milliseconds.
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

   type b2Profile is tagged null record;




--  This is an internal structure.
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
--  This is an internal structure.
--  struct b2Position
--  {
--    b2Vec2 c;
--    float a;
--  };
--
--  This is an internal structure.
--  struct b2Velocity
--  {
--    b2Vec2 v;
--    float w;
--  };
--
--  Solver Data
--  struct b2SolverData
--  {
--    b2TimeStep step;
--    b2Position* positions;
--    b2Velocity* velocities;
--  };
--
end b2_time_Step;
