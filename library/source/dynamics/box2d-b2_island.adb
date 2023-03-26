with
     box2d.b2_Distance,
     box2d.b2_Fixture,
     box2d.b2_Timer,
     box2d.b2_World,
     box2d.b2_Common,
     --  box2d.b2_Settings,

     ada.unchecked_Deallocation,
     Interfaces;


package body box2d.b2_Island
is
   --  /*
   --  Position Correction Notes
   --  =========================
   --  I tried the several algorithms for position correction of the 2D revolute joint.
   --  I looked at these systems:
   --  - simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
   --  - suspension bridge with 30 1m long planks of length 1m.
   --  - multi-link chain with 30 1m long links.
   --
   --  Here are the algorithms:
   --
   --  Baumgarte - A fraction of the position error is added to the velocity error. There is no
   --  separate position solver.
   --
   --  Pseudo Velocities - After the velocity solver and position integration,
   --  the position error, Jacobian, and effective mass are recomputed. Then
   --  the velocity constraints are solved with pseudo velocities and a fraction
   --  of the position error is added to the pseudo velocity error. The pseudo
   --  velocities are initialized to zero and there is no warm-starting. After
   --  the position solver, the pseudo velocities are added to the positions.
   --  This is also called the First Order World method or the Position LCP method.
   --
   --  Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
   --  position error is re-computed for each constraint and the positions are updated
   --  after the constraint is solved. The radius vectors (aka Jacobians) are
   --  re-computed too (otherwise the algorithm has horrible instability). The pseudo
   --  velocity states are not needed because they are effectively zero at the beginning
   --  of each iteration. Since we have the current position error, we allow the
   --  iterations to terminate early if the error becomes smaller than b2_linearSlop.
   --
   --  Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
   --  each time a constraint is solved.
   --
   --  Here are the results:
   --  Baumgarte - this is the cheapest algorithm but it has some stability problems,
   --  especially with the bridge. The chain links separate easily close to the root
   --  and they jitter as they struggle to pull together. This is one of the most common
   --  methods in the field. The big drawback is that the position correction artificially
   --  affects the momentum, thus leading to instabilities and false bounce. I used a
   --  bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
   --  factor makes joints and contacts more spongy.
   --
   --  Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
   --  stable. However, joints still separate with large angular velocities. Drag the
   --  simple pendulum in a circle quickly and the joint will separate. The chain separates
   --  easily and does not recover. I used a bias factor of 0.2. A larger value lead to
   --  the bridge collapsing when a heavy cube drops on it.
   --
   --  Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
   --  Velocities, but in other ways it is worse. The bridge and chain are much more
   --  stable, but the simple pendulum goes unstable at high angular velocities.
   --
   --  Full NGS - stable in all tests. The joints display good stiffness. The bridge
   --  still sags, but this is better than infinite forces.
   --
   --  Recommendations
   --  Pseudo Velocities are not really worthwhile because the bridge and chain cannot
   --  recover from joint separation. In other cases the benefit over Baumgarte is small.
   --
   --  Modified NGS is not a robust method for the revolute joint due to the violent
   --  instability seen in the simple pendulum. Perhaps it is viable with other constraint
   --  types, especially scalar constraints where the effective mass is a scalar.
   --
   --  This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
   --  and is very fast. I don't think we can escape Baumgarte, especially in highly
   --  demanding cases where high constraint fidelity is not needed.
   --
   --  Full NGS is robust and easy on the eyes. I recommend this as an option for
   --  higher fidelity simulation and certainly for suspension bridges and long chains.
   --  Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
   --  joint separation can be problematic. The number of NGS iterations can be reduced
   --  for better performance without harming robustness much.
   --
   --  Each joint in a can be handled differently in the position solver. So I recommend
   --  a system where the user can select the algorithm on a per joint basis. I would
   --  probably default to the slower Full NGS and let the user select the faster
   --  Baumgarte method in performance critical scenarios.
   --  */
   --



   --  /*
   --  Cache Performance
   --
   --  The Box2D solvers are dominated by cache misses. Data structures are designed
   --  to increase the number of cache hits. Much of misses are due to random access
   --  to body data. The constraint structures are iterated over linearly, which leads
   --  to few cache misses.
   --
   --  The bodies are not accessed during iteration. Instead read only data, such as
   --  the mass values are stored with the constraints. The mutable data are the constraint
   --  impulses and the bodies velocities/positions. The impulses are held inside the
   --  constraint structures. The body velocities/positions are held in compact, temporary
   --  arrays to increase the number of cache hits. Linear and angular velocity are
   --  stored in a single array since multiple arrays lead to multiple misses.
   --  */
   --



   --  /*
   --  2D Rotation
   --
   --  R = [cos(theta) -sin(theta)]
   --      [sin(theta) cos(theta) ]
   --
   --  thetaDot = omega
   --
   --  Let q1 = cos(theta), q2 = sin(theta).
   --  R = [q1 -q2]
   --      [q2  q1]
   --
   --  q1Dot = -thetaDot * q2
   --  q2Dot = thetaDot * q1
   --
   --  q1_new = q1_old - dt * w * q2
   --  q2_new = q2_old + dt * w * q1
   --  then normalize.
   --
   --  This might be faster than computing sin+cos.
   --  However, we can compute sin+cos of the same angle fast.
   --  */
   --




   --  b2Island::b2Island(
   --    int32 bodyCapacity,
   --    int32 contactCapacity,
   --    int32 jointCapacity,
   --    b2StackAllocator* allocator,
   --    b2ContactListener* listener)
   --  {
   --    m_bodyCapacity = bodyCapacity;
   --    m_contactCapacity = contactCapacity;
   --    m_jointCapacity    = jointCapacity;
   --    m_bodyCount = 0;
   --    m_contactCount = 0;
   --    m_jointCount = 0;
   --
   --    m_allocator = allocator;
   --    m_listener = listener;
   --
   --    m_bodies = (b2Body**)m_allocator->Allocate(bodyCapacity * sizeof(b2Body*));
   --    m_contacts = (b2Contact**)m_allocator->Allocate(contactCapacity    * sizeof(b2Contact*));
   --    m_joints = (b2Joint**)m_allocator->Allocate(jointCapacity * sizeof(b2Joint*));
   --
   --    m_velocities = (b2Velocity*)m_allocator->Allocate(m_bodyCapacity * sizeof(b2Velocity));
   --    m_positions = (b2Position*)m_allocator->Allocate(m_bodyCapacity * sizeof(b2Position));
   --  }
   --

   function to_b2Island (bodyCapacity    : Natural;
                         contactCapacity : Natural;
                         jointCapacity   : Natural;
                         listener        : access b2ContactListener'Class) return b2Island
   is
      Self : b2Island;
   begin
      Self.m_bodyCapacity     := bodyCapacity;
      Self.m_contactCapacity  := contactCapacity;
      Self.m_jointCapacity    := jointCapacity;
      Self.m_bodyCount        := 0;
      Self.m_contactCount     := 0;
      Self.m_jointCount       := 0;

      Self.m_listener   := listener;

      Self.m_bodies     := new b2Bodies   (0 .. bodyCapacity    - 1);
      Self.m_contacts   := new b2Contacts (0 .. contactCapacity - 1);
      Self.m_joints     := new b2Joints   (0 .. jointCapacity   - 1);

      Self.m_velocities := new b2Velocities (0 .. Self.m_bodyCapacity - 1);
      Self.m_positions  := new b2Positions  (0 .. Self.m_bodyCapacity - 1);

      return Self;
   end to_b2Island;




   --  b2Island::~b2Island()
   --  {
   --    // Warning: the order should reverse the constructor order.
   --    m_allocator->Free(m_positions);
   --    m_allocator->Free(m_velocities);
   --    m_allocator->Free(m_joints);
   --    m_allocator->Free(m_contacts);
   --    m_allocator->Free(m_bodies);
   --  }
   --

   procedure destruct (Self : in out b2Island)
   is
      procedure free is new ada.unchecked_Deallocation (b2Positions,  b2Positions_ptr);
      procedure free is new ada.unchecked_Deallocation (b2Velocities, b2Velocities_ptr);
      procedure free is new ada.unchecked_Deallocation (b2Joints,     b2Joints_ptr);
      procedure free is new ada.unchecked_Deallocation (b2Contacts,   b2Contacts_ptr);
      procedure free is new ada.unchecked_Deallocation (b2Bodies,     b2Bodies_ptr);
   begin
        -- Warning: the order should reverse the constructor order.
        free (Self.m_positions);
        free (Self.m_velocities);
        free (Self.m_joints);
        free (Self.m_contacts);
        free (Self.m_bodies);
   end destruct;




   --    void Clear()
   --    {
   --       m_bodyCount = 0;
   --       m_contactCount = 0;
   --       m_jointCount = 0;
   --    }
   --

   procedure clear (Self : in out b2Island)
   is
   begin
      Self.m_bodyCount    := 0;
      Self.m_contactCount := 0;
      Self.m_jointCount   := 0;
   end clear;




   --    void Add(b2Body* body)
   --    {
   --       b2Assert(m_bodyCount < m_bodyCapacity);
   --       body->m_islandIndex = m_bodyCount;
   --       m_bodies[m_bodyCount] = body;
   --       ++m_bodyCount;
   --    }
   --

   procedure add (Self : in out b2Island;   the_Body : b2Body_ptr)
   is
      pragma assert (Self.m_bodyCount < Self.m_bodyCapacity);
   begin
      the_Body.m_islandIndex_is (Self.m_bodyCount);

      Self.m_bodies (Self.m_bodyCount) := the_Body;
      Self.m_bodyCount                 := Self.m_bodyCount + 1;
   end add;




   --    void Add(b2Contact* contact)
   --    {
   --       b2Assert(m_contactCount < m_contactCapacity);
   --       m_contacts[m_contactCount++] = contact;
   --    }
   --

   procedure add (Self : in out b2Island;   Contact : access b2Contact)
   is
      pragma assert (Self.m_contactCount < Self.m_contactCapacity);
   begin
      Self.m_contacts (Self.m_contactCount) := contact.all'unchecked_Access;
      Self.m_contactCount                   := Self.m_contactCount + 1;
   end add;




   --    void Add(b2Joint* joint)
   --    {
   --       b2Assert(m_jointCount < m_jointCapacity);
   --       m_joints[m_jointCount++] = joint;
   --    }
   --

   procedure add (Self : in out b2Island;   Joint : in b2Joint_ptr)
   is
      pragma assert (Self.m_jointCount < Self.m_jointCapacity);
   begin
      Self.m_joints (Self.m_jointCount) := joint;
      Self.m_jointCount                 := Self.m_jointCount + 1;
   end add;





   --  void b2Island::Solve(b2Profile* profile, const b2TimeStep& step, const b2Vec2& gravity, bool allowSleep)
   --  {
   --    b2Timer timer;
   --
   --    float h = step.dt;
   --
   --    // Integrate velocities and apply damping. Initialize the body state.
   --    for (int32 i = 0; i < m_bodyCount; ++i)
   --    {
   --       b2Body* b = m_bodies[i];
   --
   --       b2Vec2 c = b->m_sweep.c;
   --       float a = b->m_sweep.a;
   --       b2Vec2 v = b->m_linearVelocity;
   --       float w = b->m_angularVelocity;
   --
   --       // Store positions for continuous collision.
   --       b->m_sweep.c0 = b->m_sweep.c;
   --       b->m_sweep.a0 = b->m_sweep.a;
   --
   --       if (b->m_type == b2_dynamicBody)
   --       {
   --          // Integrate velocities.
   --          v += h * b->m_invMass * (b->m_gravityScale * b->m_mass * gravity + b->m_force);
   --          w += h * b->m_invI * b->m_torque;
   --
   --          // Apply damping.
   --          // ODE: dv/dt + c * v = 0
   --          // Solution: v(t) = v0 * exp(-c * t)
   --          // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
   --          // v2 = exp(-c * dt) * v1
   --          // Pade approximation:
   --          // v2 = v1 * 1 / (1 + c * dt)
   --          v *= 1.0f / (1.0f + h * b->m_linearDamping);
   --          w *= 1.0f / (1.0f + h * b->m_angularDamping);
   --       }
   --
   --       m_positions[i].c = c;
   --       m_positions[i].a = a;
   --       m_velocities[i].v = v;
   --       m_velocities[i].w = w;
   --    }
   --
   --    timer.Reset();
   --
   --    // Solver data
   --    b2SolverData solverData;
   --    solverData.step = step;
   --    solverData.positions = m_positions;
   --    solverData.velocities = m_velocities;
   --
   --    // Initialize velocity constraints.
   --    b2ContactSolverDef contactSolverDef;
   --    contactSolverDef.step = step;
   --    contactSolverDef.contacts = m_contacts;
   --    contactSolverDef.count = m_contactCount;
   --    contactSolverDef.positions = m_positions;
   --    contactSolverDef.velocities = m_velocities;
   --    contactSolverDef.allocator = m_allocator;
   --
   --    b2ContactSolver contactSolver(&contactSolverDef);
   --    contactSolver.InitializeVelocityConstraints();
   --
   --    if (step.warmStarting)
   --    {
   --       contactSolver.WarmStart();
   --    }
   --
   --    for (int32 i = 0; i < m_jointCount; ++i)
   --    {
   --       m_joints[i]->InitVelocityConstraints(solverData);
   --    }
   --
   --    profile->solveInit = timer.GetMilliseconds();
   --
   --    // Solve velocity constraints
   --    timer.Reset();
   --    for (int32 i = 0; i < step.velocityIterations; ++i)
   --    {
   --       for (int32 j = 0; j < m_jointCount; ++j)
   --       {
   --          m_joints[j]->SolveVelocityConstraints(solverData);
   --       }
   --
   --       contactSolver.SolveVelocityConstraints();
   --    }
   --
   --    // Store impulses for warm starting
   --    contactSolver.StoreImpulses();
   --    profile->solveVelocity = timer.GetMilliseconds();
   --
   --    // Integrate positions
   --    for (int32 i = 0; i < m_bodyCount; ++i)
   --    {
   --       b2Vec2 c = m_positions[i].c;
   --       float a = m_positions[i].a;
   --       b2Vec2 v = m_velocities[i].v;
   --       float w = m_velocities[i].w;
   --
   --       // Check for large velocities
   --       b2Vec2 translation = h * v;
   --       if (b2Dot(translation, translation) > b2_maxTranslationSquared)
   --       {
   --          float ratio = b2_maxTranslation / translation.Length();
   --          v *= ratio;
   --       }
   --
   --       float rotation = h * w;
   --       if (rotation * rotation > b2_maxRotationSquared)
   --       {
   --          float ratio = b2_maxRotation / b2Abs(rotation);
   --          w *= ratio;
   --       }
   --
   --       // Integrate
   --       c += h * v;
   --       a += h * w;
   --
   --       m_positions[i].c = c;
   --       m_positions[i].a = a;
   --       m_velocities[i].v = v;
   --       m_velocities[i].w = w;
   --    }
   --
   --    // Solve position constraints
   --    timer.Reset();
   --    bool positionSolved = false;
   --    for (int32 i = 0; i < step.positionIterations; ++i)
   --    {
   --       bool contactsOkay = contactSolver.SolvePositionConstraints();
   --
   --       bool jointsOkay = true;
   --       for (int32 j = 0; j < m_jointCount; ++j)
   --       {
   --          bool jointOkay = m_joints[j]->SolvePositionConstraints(solverData);
   --          jointsOkay = jointsOkay && jointOkay;
   --       }
   --
   --       if (contactsOkay && jointsOkay)
   --       {
   --          // Exit early if the position errors are small.
   --          positionSolved = true;
   --          break;
   --       }
   --    }
   --
   --    // Copy state buffers back to the bodies
   --    for (int32 i = 0; i < m_bodyCount; ++i)
   --    {
   --       b2Body* body = m_bodies[i];
   --       body->m_sweep.c = m_positions[i].c;
   --       body->m_sweep.a = m_positions[i].a;
   --       body->m_linearVelocity = m_velocities[i].v;
   --       body->m_angularVelocity = m_velocities[i].w;
   --       body->SynchronizeTransform();
   --    }
   --
   --    profile->solvePosition = timer.GetMilliseconds();
   --
   --    Report(contactSolver.m_velocityConstraints);
   --
   --    if (allowSleep)
   --    {
   --       float minSleepTime = b2_maxFloat;
   --
   --       const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
   --       const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;
   --
   --       for (int32 i = 0; i < m_bodyCount; ++i)
   --       {
   --          b2Body* b = m_bodies[i];
   --          if (b->GetType() == b2_staticBody)
   --          {
   --             continue;
   --          }
   --
   --          if ((b->m_flags & b2Body::e_autoSleepFlag) == 0 ||
   --             b->m_angularVelocity * b->m_angularVelocity > angTolSqr ||
   --             b2Dot(b->m_linearVelocity, b->m_linearVelocity) > linTolSqr)
   --          {
   --             b->m_sleepTime = 0.0f;
   --             minSleepTime = 0.0f;
   --          }
   --          else
   --          {
   --             b->m_sleepTime += h;
   --             minSleepTime = b2Min(minSleepTime, b->m_sleepTime);
   --          }
   --       }
   --
   --       if (minSleepTime >= b2_timeToSleep && positionSolved)
   --       {
   --          for (int32 i = 0; i < m_bodyCount; ++i)
   --          {
   --             b2Body* b = m_bodies[i];
   --             b->SetAwake(false);
   --          }
   --       }
   --    }
   --  }
   --

   procedure solve (Self : in out b2Island;   profile    : access b2Profile;
                                              step       : in     b2TimeStep;
                                              gravity    : in     b2Vec2;
                                              allowSleep : in     Boolean)
   is
      use b2_Timer;
          --  b2_Settings;

      timer :          b2Timer;
      h     : constant Real   := step.dt;

   begin
      -- Integrate velocities and apply damping. Initialize the body state.
      --
      for i in 0 .. Self.m_bodyCount - 1
      loop
         declare
            b : constant access b2Body := Self.m_bodies (i);

            c : constant b2Vec2 := b.m_sweep.c;
            a : constant Real   := b.m_sweep.a;
            v :          b2Vec2 := b.getLinearVelocity;
            w :          Real   := b.getAngularVelocity;
         begin
            -- Store positions for continuous collision.
            b.m_sweep.c0 := b.m_sweep.c;
            b.m_sweep.a0 := b.m_sweep.a;

            if b.getType = b2_dynamicBody
            then
               -- Integrate velocities.
               v := v + h * b.m_invMass * (b.getGravityScale * b.getMass * gravity + b.m_force);
               w := w + h * b.m_invI * b.m_torque;

               -- Apply damping.
               -- ODE: dv/dt + c * v = 0
               -- Solution: v(t) = v0 * exp(-c * t)
               -- Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
               -- v2 = exp(-c * dt) * v1
               -- Pade approximation:
               -- v2 = v1 * 1 / (1 + c * dt)
               --
               v := v * (1.0 / (1.0 + h * b.getLinearDamping));
               w := w * (1.0 / (1.0 + h * b.getAngularDamping));
            end if;

            Self.m_positions  (i).c := c;
            Self.m_positions  (i).a := a;
            Self.m_velocities (i).v := v;
            Self.m_velocities (i).w := w;
         end;
      end loop;

      timer.reset;

      declare
         solverData       :         b2SolverData;
         contactSolverDef : aliased b2ContactSolverDef;
      begin
         -- Solver data
         solverData.step       := step;
         solverData.positions  := Self.m_positions;
         solverData.velocities := Self.m_velocities;

         -- Initialize velocity constraints.
         contactSolverDef.step       := step;
         contactSolverDef.contacts   := Self.m_contacts;
         contactSolverDef.count      := Self.m_contactCount;
         contactSolverDef.positions  := Self.m_positions;
         contactSolverDef.velocities := Self.m_velocities;

         declare
            contactSolver : b2ContactSolver := to_b2ContactSolver (contactSolverDef'Access);
         begin
            contactSolver.initializeVelocityConstraints;

            if step.warmStarting
            then
               contactSolver.warmStart;
            end if;

            for i in 0 .. Self.m_jointCount - 1
            loop
               Self.m_joints (i).initVelocityConstraints (solverData);
            end loop;

            profile.solveInit := timer.getMilliseconds;

            -- Solve velocity constraints
            timer.reset;

            for i in 0 .. step.velocityIterations - 1
            loop
               for j in 0 .. Self.m_jointCount - 1
               loop
                  Self.m_joints (j).solveVelocityConstraints (solverData);
               end loop;

               contactSolver.solveVelocityConstraints;
            end loop;

            -- Store impulses for warm starting.
            contactSolver.storeImpulses;
            profile.solveVelocity := timer.getMilliseconds;

            -- Integrate positions.
            for i in 0 .. Self.m_bodyCount - 1
            loop
               declare
                  use b2_Common;

                  c : b2Vec2  := Self.m_positions  (i).c;
                  a : Real    := Self.m_positions  (i).a;
                  v : b2Vec2  := Self.m_velocities (i).v;
                  w : Real    := Self.m_velocities (i).w;

                  translation : constant b2Vec2 := h * v;
                  ratio       : Real;
                  rotation    : Real;
               begin
                  -- Check for large velocities.
                  if b2Dot (translation, translation) > b2_maxTranslationSquared
                  then
                     ratio := b2_maxTranslation / Length (translation);
                     v     := v * ratio;
                  end if;

                  rotation := h * w;

                  if rotation * rotation > b2_maxRotationSquared
                  then
                     ratio := b2_maxRotation / abs rotation;
                     w     := w * ratio;
                  end if;

                  -- Integrate.
                  c := c + h * v;
                  a := a + h * w;

                  Self.m_positions  (i).c := c;
                  Self.m_positions  (i).a := a;
                  Self.m_velocities (i).v := v;
                  Self.m_velocities (i).w := w;
               end;
            end loop;


            -- Solve position constraints.
            --
            timer.reset;

            declare
               positionSolved : Boolean := False;
            begin
               for i in 0 .. step.positionIterations - 1
               loop
                  declare
                     contactsOkay : constant Boolean := contactSolver.solvePositionConstraints;
                     jointsOkay   : Boolean := True;
                  begin
                     for j in 0 .. Self.m_jointCount - 1
                     loop
                        declare
                           jointOkay : constant Boolean := Self.m_joints (j).solvePositionConstraints (solverData);
                        begin
                           jointsOkay := jointsOkay and jointOkay;
                        end;
                     end loop;

                     if contactsOkay and jointsOkay
                     then
                        -- Exit early if the position errors are small.
                        positionSolved := True;
                        exit;
                     end if;
                  end;
               end loop;

               -- Copy state buffers back to the bodies.
               --
               for i in 0 .. Self.m_bodyCount - 1
               loop
                  declare
                     the_Body : constant access b2Body := Self.m_bodies (i);
                  begin
                     the_Body.m_sweep.c := Self.m_positions (i).c;
                     the_Body.m_sweep.a := Self.m_positions (i).a;

                     the_Body. setLinearVelocity (Self.m_velocities (i).v);
                     the_Body.setAngularVelocity (Self.m_velocities (i).w);

                     the_Body.synchronizeTransform;
                  end;
               end loop;

               profile.solvePosition := timer.getMilliseconds;

               Self.report (contactSolver.m_velocityConstraints.all);

               if allowSleep
               then
                  declare
                     use b2_Common;

                     minSleepTime : Real := b2_maxFloat;

                     linTolSqr    : constant Real := b2_linearSleepTolerance  * b2_linearSleepTolerance;
                     angTolSqr    : constant Real := b2_angularSleepTolerance * b2_angularSleepTolerance;
                  begin
                     for i in 0 .. Self.m_bodyCount - 1
                     loop
                        declare
                           use Interfaces;

                           b : constant access b2Body := Self.m_bodies (i);
                        begin
                           if b.getType = b2_staticBody
                           then
                              goto Continue;
                           end if;

                           if  (b.m_flags and b2_Body.e_autoSleepFlag)           = 0
                             or b.getAngularVelocity * b.getAngularVelocity      > angTolSqr
                             or b2Dot (b.getLinearVelocity, b.getLinearVelocity) > linTolSqr
                           then
                              b.m_sleepTime_is (0.0);
                              minSleepTime := 0.0;

                           else
                              b.m_sleepTime_is (b.m_sleepTime + h);
                              minSleepTime := Real'min (minSleepTime, b.m_sleepTime);
                           end if;
                        end;

                        <<Continue>>
                     end loop;

                     if    minSleepTime >= b2_timeToSleep
                       and positionSolved
                     then
                        for i in 0 .. Self.m_bodyCount - 1
                        loop
                           declare
                              b : constant access b2Body := Self.m_bodies (i);
                           begin
                              b.setAwake (False);
                           end;
                        end loop;
                     end if;

                  end;
               end if;
            end;
         end;
      end;
   end solve;






   --  void b2Island::SolveTOI(const b2TimeStep& subStep, int32 toiIndexA, int32 toiIndexB)
   --  {
   --    b2Assert(toiIndexA < m_bodyCount);
   --    b2Assert(toiIndexB < m_bodyCount);
   --
   --    // Initialize the body state.
   --    for (int32 i = 0; i < m_bodyCount; ++i)
   --    {
   --       b2Body* b = m_bodies[i];
   --       m_positions[i].c = b->m_sweep.c;
   --       m_positions[i].a = b->m_sweep.a;
   --       m_velocities[i].v = b->m_linearVelocity;
   --       m_velocities[i].w = b->m_angularVelocity;
   --    }
   --
   --    b2ContactSolverDef contactSolverDef;
   --    contactSolverDef.contacts = m_contacts;
   --    contactSolverDef.count = m_contactCount;
   --    contactSolverDef.allocator = m_allocator;
   --    contactSolverDef.step = subStep;
   --    contactSolverDef.positions = m_positions;
   --    contactSolverDef.velocities = m_velocities;
   --    b2ContactSolver contactSolver(&contactSolverDef);
   --
   --    // Solve position constraints.
   --    for (int32 i = 0; i < subStep.positionIterations; ++i)
   --    {
   --       bool contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
   --       if (contactsOkay)
   --       {
   --          break;
   --       }
   --    }
   --
   --  #if 0
   --    // Is the new position really safe?
   --    for (int32 i = 0; i < m_contactCount; ++i)
   --    {
   --       b2Contact* c = m_contacts[i];
   --       b2Fixture* fA = c->GetFixtureA();
   --       b2Fixture* fB = c->GetFixtureB();
   --
   --       b2Body* bA = fA->GetBody();
   --       b2Body* bB = fB->GetBody();
   --
   --       int32 indexA = c->GetChildIndexA();
   --       int32 indexB = c->GetChildIndexB();
   --
   --       b2DistanceInput input;
   --       input.proxyA.Set(fA->GetShape(), indexA);
   --       input.proxyB.Set(fB->GetShape(), indexB);
   --       input.transformA = bA->GetTransform();
   --       input.transformB = bB->GetTransform();
   --       input.useRadii = false;
   --
   --       b2DistanceOutput output;
   --       b2SimplexCache cache;
   --       cache.count = 0;
   --       b2Distance(&output, &cache, &input);
   --
   --       if (output.distance == 0 || cache.count == 3)
   --       {
   --          cache.count += 0;
   --       }
   --    }
   --  #endif
   --
   --    // Leap of faith to new safe state.
   --    m_bodies[toiIndexA]->m_sweep.c0 = m_positions[toiIndexA].c;
   --    m_bodies[toiIndexA]->m_sweep.a0 = m_positions[toiIndexA].a;
   --    m_bodies[toiIndexB]->m_sweep.c0 = m_positions[toiIndexB].c;
   --    m_bodies[toiIndexB]->m_sweep.a0 = m_positions[toiIndexB].a;
   --
   --    // No warm starting is needed for TOI events because warm
   --    // starting impulses were applied in the discrete solver.
   --    contactSolver.InitializeVelocityConstraints();
   --
   --    // Solve velocity constraints.
   --    for (int32 i = 0; i < subStep.velocityIterations; ++i)
   --    {
   --       contactSolver.SolveVelocityConstraints();
   --    }
   --
   --    // Don't store the TOI contact forces for warm starting
   --    // because they can be quite large.
   --
   --    float h = subStep.dt;
   --
   --    // Integrate positions
   --    for (int32 i = 0; i < m_bodyCount; ++i)
   --    {
   --       b2Vec2 c = m_positions[i].c;
   --       float a = m_positions[i].a;
   --       b2Vec2 v = m_velocities[i].v;
   --       float w = m_velocities[i].w;
   --
   --       // Check for large velocities
   --       b2Vec2 translation = h * v;
   --       if (b2Dot(translation, translation) > b2_maxTranslationSquared)
   --       {
   --          float ratio = b2_maxTranslation / translation.Length();
   --          v *= ratio;
   --       }
   --
   --       float rotation = h * w;
   --       if (rotation * rotation > b2_maxRotationSquared)
   --       {
   --          float ratio = b2_maxRotation / b2Abs(rotation);
   --          w *= ratio;
   --       }
   --
   --       // Integrate
   --       c += h * v;
   --       a += h * w;
   --
   --       m_positions[i].c = c;
   --       m_positions[i].a = a;
   --       m_velocities[i].v = v;
   --       m_velocities[i].w = w;
   --
   --       // Sync bodies
   --       b2Body* body = m_bodies[i];
   --       body->m_sweep.c = c;
   --       body->m_sweep.a = a;
   --       body->m_linearVelocity = v;
   --       body->m_angularVelocity = w;
   --       body->SynchronizeTransform();
   --    }
   --
   --    Report(contactSolver.m_velocityConstraints);
   --  }
   --

   procedure SolveTOI (Self : in out b2Island;   subStep   : in b2TimeStep;
                                                 toiIndexA : in Natural;
                                                 toiIndexB : in Natural)
   is
      pragma assert (toiIndexA < Self.m_bodyCount);
      pragma assert (toiIndexB < Self.m_bodyCount);

      contactSolver : b2ContactSolver;
   begin
      -- Initialize the body state.
      --
      for i in 0 .. Self.m_bodyCount - 1
      loop
         declare
            b : constant access b2Body := Self.m_bodies (i);
         begin
            Self.m_positions  (i).c := b.m_sweep.c;
            Self.m_positions  (i).a := b.m_sweep.a;
            Self.m_velocities (i).v := b.getLinearVelocity;
            Self.m_velocities (i).w := b.getAngularVelocity;
         end;
      end loop;


      declare
         contactSolverDef : aliased b2ContactSolverDef;
      begin
         contactSolverDef.contacts   := Self.m_contacts;
         contactSolverDef.count      := Self.m_contactCount;
         contactSolverDef.step       := subStep;
         contactSolverDef.positions  := Self.m_positions;
         contactSolverDef.velocities := Self.m_velocities;

         contactSolver := to_b2ContactSolver (contactSolverDef'Access);

         -- Solve position constraints.
         --
         for i in 0 .. subStep.positionIterations - 1
         loop
            declare
               contactsOkay : constant Boolean := contactSolver.SolveTOIPositionConstraints (toiIndexA, toiIndexB);
            begin
               if contactsOkay
               then
                  exit;
               end if;
            end;
         end loop;
      end;


      if False
      then
         -- Is the new position really safe?
         --
         for i in 0 .. Self.m_contactCount - 1
         loop
            declare
               use b2_Fixture,
                   b2_Distance;

               c  : constant access b2Contact := Self.m_contacts (i);
               fA : constant access b2Fixture := c.getFixtureA;
               fB : constant access b2Fixture := c.getFixtureB;

               bA : constant access b2Body    := fA.getBody;
               bB : constant access b2Body    := fB.getBody;

               indexA : constant Natural := c.getChildIndexA;
               indexB : constant Natural := c.getChildIndexB;

               input  : b2DistanceInput;
            begin
               input.proxyA.set (fA.getShape, indexA);
               input.proxyB.set (fB.getShape, indexB);

               input.transformA := bA.getTransform;
               input.transformB := bB.getTransform;
               input.useRadii   := False;

               declare
                  output : b2DistanceOutput;
                  cache  : b2SimplexCache;
               begin
                  cache.count := 0;
                  b2Distance (output, cache, input);

                  if   output.distance = 0.0
                    or cache.count     = 3
                  then
                     cache.count := cache.count + 0;
                  end if;
               end;
            end;
         end loop;
      end if;


      -- Leap of faith to new safe state.
      --
      Self.m_bodies (toiIndexA).m_sweep.c0 := Self.m_positions (toiIndexA).c;
      Self.m_bodies (toiIndexA).m_sweep.a0 := Self.m_positions (toiIndexA).a;
      Self.m_bodies (toiIndexB).m_sweep.c0 := Self.m_positions (toiIndexB).c;
      Self.m_bodies (toiIndexB).m_sweep.a0 := Self.m_positions (toiIndexB).a;

      -- No warm starting is needed for TOI events because warm
      -- starting impulses were applied in the discrete solver.
      --
      contactSolver.initializeVelocityConstraints;


      -- Solve velocity constraints.
      --
      for i in 0 .. subStep.velocityIterations - 1
      loop
         contactSolver.SolveVelocityConstraints;
      end loop;


      -- Don't store the TOI contact forces for warm starting
      -- because they can be quite large.
      --
      declare
         --  use b2_Settings;

         h : constant Real := subStep.dt;
      begin
         -- Integrate positions.
         --
         for i in 0 .. Self.m_bodyCount - 1
         loop
            declare
               use b2_Common;

               c : b2Vec2 := Self.m_positions  (i).c;
               a : Real   := Self.m_positions  (i).a;
               v : b2Vec2 := Self.m_velocities (i).v;
               w : Real   := Self.m_velocities (i).w;

               translation : constant b2Vec2 := h * v;
               rotation    :          Real;
            begin
               -- Check for large velocities.
               --
               if b2Dot(translation, translation) > b2_maxTranslationSquared
               then
                  declare
                     ratio : constant Real := b2_maxTranslation / Length (translation);
                  begin
                     v := v * ratio;
                  end;
               end if;

               rotation := h * w;

               if rotation * rotation > b2_maxRotationSquared
               then
                  declare
                     ratio : constant Real := b2_maxRotation / abs rotation;
                  begin
                     w := w * ratio;
                  end;
               end if;

               -- Integrate.
               --
               c := c + h * v;
               a := a + h * w;

               Self.m_positions  (i).c := c;
               Self.m_positions  (i).a := a;
               Self.m_velocities (i).v := v;
               Self.m_velocities (i).w := w;

               -- Sync bodies.
               --
               declare
                  the_Body : constant access b2Body := Self.m_bodies (i);
               begin
                  the_Body.m_sweep.c := c;
                  the_Body.m_sweep.a := a;

                  the_Body. setLinearVelocity (v);
                  the_Body.setAngularVelocity (w);

                  the_Body.synchronizeTransform;
               end;
            end;
         end loop;
      end;

      Self.report (contactSolver.m_velocityConstraints.all);
   end SolveTOI;





   --  void b2Island::Report(const b2ContactVelocityConstraint* constraints)
   --  {
   --    if (m_listener == nullptr)
   --    {
   --       return;
   --    }
   --
   --    for (int32 i = 0; i < m_contactCount; ++i)
   --    {
   --       b2Contact* c = m_contacts[i];
   --
   --       const b2ContactVelocityConstraint* vc = constraints + i;
   --
   --       b2ContactImpulse impulse;
   --       impulse.count = vc->pointCount;
   --       for (int32 j = 0; j < vc->pointCount; ++j)
   --       {
   --          impulse.normalImpulses[j] = vc->points[j].normalImpulse;
   --          impulse.tangentImpulses[j] = vc->points[j].tangentImpulse;
   --       }
   --
   --       m_listener->PostSolve(c, &impulse);
   --    }
   --  }

   procedure report (Self : in out b2Island;   constraints : in b2ContactVelocityConstraints)
   is
   begin
      if Self.m_listener = null
      then
         return;
      end if;

      for i in 0 .. Self.m_contactCount - 1
      loop
         declare
            c       : constant access b2Contact := Self.m_contacts (i);
            vc      :                 b2ContactVelocityConstraint renames constraints (i);
            impulse :                 b2ContactImpulse;
         begin
            impulse.count := vc.pointCount;

            for j in 0 .. vc.pointCount - 1
            loop
               impulse.normalImpulses  (j) := vc.points (j). normalImpulse;
               impulse.tangentImpulses (j) := vc.points (j).tangentImpulse;
            end loop;

            Self.m_listener.postSolve (c.all, impulse);
         end;
      end loop;
   end Report;



end box2d.b2_Island;
