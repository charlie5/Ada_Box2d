with
     b2_Collision,
     b2_Math,
     b2_time_Step;


package b2_contact_Solver
is
   procedure dummy;


   --
--  class b2Contact;
--  class b2Body;
--  class b2StackAllocator;
--  struct b2ContactPositionConstraint;
--
--  struct b2VelocityConstraintPoint
--  {
--    b2Vec2 rA;
--    b2Vec2 rB;
--    float normalImpulse;
--    float tangentImpulse;
--    float normalMass;
--    float tangentMass;
--    float velocityBias;
--  };
--
--  struct b2ContactVelocityConstraint
--  {
--    b2VelocityConstraintPoint points[b2_maxManifoldPoints];
--    b2Vec2 normal;
--    b2Mat22 normalMass;
--    b2Mat22 K;
--    int32 indexA;
--    int32 indexB;
--    float invMassA, invMassB;
--    float invIA, invIB;
--    float friction;
--    float restitution;
--    float threshold;
--    float tangentSpeed;
--    int32 pointCount;
--    int32 contactIndex;
--  };
--
--  struct b2ContactSolverDef
--  {
--    b2TimeStep step;
--    b2Contact** contacts;
--    int32 count;
--    b2Position* positions;
--    b2Velocity* velocities;
--    b2StackAllocator* allocator;
--  };
--
--  class b2ContactSolver
--  {
--  public:
--    b2ContactSolver(b2ContactSolverDef* def);
--    ~b2ContactSolver();
--
--    void InitializeVelocityConstraints();
--
--    void WarmStart();
--    void SolveVelocityConstraints();
--    void StoreImpulses();
--
--    bool SolvePositionConstraints();
--    bool SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB);
--
--    b2TimeStep m_step;
--    b2Position* m_positions;
--    b2Velocity* m_velocities;
--    b2StackAllocator* m_allocator;
--    b2ContactPositionConstraint* m_positionConstraints;
--    b2ContactVelocityConstraint* m_velocityConstraints;
--    b2Contact** m_contacts;
--    int m_count;
--  };
--
end b2_contact_Solver;
