with
     b2_Contact,
     b2_World,
     b2_broad_Phase,
     ada.unchecked_Deallocation;


package body b2_Body
is
   use b2_broad_Phase,
       b2_Contact,
       Interfaces;



   ------------
   -- b2BodyDef
   --

   function to_b2BodyDef return b2BodyDef
   is
      Self : b2BodyDef;
   begin
      Self.position        := (0.0, 0.0);
      Self.angle           := 0.0;
      Self.linearVelocity  := (0.0, 0.0);
      Self.angularVelocity := 0.0;
      Self.linearDamping   := 0.0;
      Self.angularDamping  := 0.0;
      Self.allowSleep      := True;
      Self.awake           := True;
      Self.fixedRotation   := False;
      Self.bullet          := False;
      Self.Kind            := b2_staticBody;
      Self.enabled         := True;
      Self.gravityScale    := 1.0;

      return Self;
   end to_b2BodyDef;





   ---------
   -- b2Body
   --


   --  b2Fixture* b2Body::CreateFixture(const b2FixtureDef* def)
   --  {
   --    b2Assert(m_world->IsLocked() == false);
   --    if (m_world->IsLocked() == true)
   --    {
   --       return nullptr;
   --    }
   --
   --    b2BlockAllocator* allocator = &m_world->m_blockAllocator;
   --
   --    void* memory = allocator->Allocate(sizeof(b2Fixture));
   --    b2Fixture* fixture = new (memory) b2Fixture;
   --    fixture->Create(allocator, this, def);
   --
   --    if (m_flags & e_enabledFlag)
   --    {
   --       b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
   --       fixture->CreateProxies(broadPhase, m_xf);
   --    }
   --
   --    fixture->m_next = m_fixtureList;
   --    m_fixtureList = fixture;
   --    ++m_fixtureCount;
   --
   --    fixture->m_body = this;
   --
   --    // Adjust mass properties if needed.
   --    if (fixture->m_density > 0.0f)
   --    {
   --       ResetMassData();
   --    }
   --
   --    // Let the world know we have a new fixture. This will cause new contacts
   --    // to be created at the beginning of the next time step.
   --    m_world->m_newContacts = true;
   --
   --    return fixture;
   --  }
   --

   function createFixture (Self : in out b2Body;   def : in b2_Fixture.b2FixtureDef) return b2_Fixture.b2Fixture_ptr
   is
      pragma assert (Self.m_world.isLocked = False);
   begin
     if Self.m_world.IsLocked = True
     then
        return null;
     end if;

      declare
         fixture : constant b2_Fixture.b2Fixture_ptr := new b2_Fixture.b2Fixture' (to_b2Fixture);
      begin
         fixture.create (Self'Access, def);

         if (Self.m_flags and e_enabledFlag) /= 0
         then
            declare
               broadPhase : constant access b2BroadPhase := Self.m_world.m_contactManager.m_broadPhase'Access;
            begin
               fixture.createProxies (broadPhase, Self.m_xf);
            end;
         end if;

         fixture.m_next_is (Self.m_fixtureList);
         Self.m_fixtureList  := fixture;
         Self.m_fixtureCount := Self.m_fixtureCount + 1;
         fixture.m_body_is (Self'Access);

         -- Adjust mass properties if needed.
         --
         if fixture.m_density > 0.0
         then
            Self.ResetMassData;
         end if;

         -- Let the world know we have a new fixture. This will cause new contacts
         -- to be created at the beginning of the next time step.
         --
         Self.m_world.m_newContacts_is (True);

         return fixture;
      end;
   end createFixture;





   --    Creates a fixture from a shape and attach it to this body.
   --    This is a convenience function. Use b2FixtureDef if you need to set parameters
   --    like friction, restitution, user data, or filtering.
   --    If the density is non-zero, this function automatically updates the mass of the body.
   --
   --    @param shape the shape to be cloned.
   --    @param density the shape density (set to zero for static bodies).
   --    @warning This function is locked during callbacks.
   --
   --    b2Fixture* CreateFixture(const b2Shape* shape, float density);
   --
   --  b2Fixture* b2Body::CreateFixture(const b2Shape* shape, float density)
   --  {
   --    b2FixtureDef def;
   --    def.shape = shape;
   --    def.density = density;
   --
   --    return CreateFixture(&def);
   --  }
   --

   function createFixture (Self : in out b2Body;   Shape   : access constant b2Shape'Class;
                                                   Density : in              Real) return b2_Fixture.b2Fixture_ptr
   is
      def : b2FixtureDef;
   begin
      def.shape   := shape;
      def.density := density;

     return Self.createFixture (def);
   end createFixture;





   --    Destroy a fixture. This removes the fixture from the broad-phase and
   --    destroys all contacts associated with this fixture. This will
   --    automatically adjust the mass of the body if the body is dynamic and the
   --    fixture has positive density.
   --    All fixtures attached to a body are implicitly destroyed when the body is destroyed.
   --
   --    @param fixture the fixture to be removed.
   --    @warning This function is locked during callbacks.
   --
   --    void DestroyFixture(b2Fixture* fixture);
   --
   --  void b2Body::DestroyFixture(b2Fixture* fixture)
   --  {
   --    if (fixture == NULL)
   --    {
   --       return;
   --    }
   --
   --    b2Assert(m_world->IsLocked() == false);
   --    if (m_world->IsLocked() == true)
   --    {
   --       return;
   --    }
   --
   --    b2Assert(fixture->m_body == this);
   --
   --    // Remove the fixture from this body's singly linked list.
   --    b2Assert(m_fixtureCount > 0);
   --    b2Fixture** node = &m_fixtureList;
   --    bool found = false;
   --    while (*node != nullptr)
   --    {
   --       if (*node == fixture)
   --       {
   --          *node = fixture->m_next;
   --          found = true;
   --          break;
   --       }
   --
   --       node = &(*node)->m_next;
   --    }
   --
   --    // You tried to remove a shape that is not attached to this body.
   --    b2Assert(found);
   --
   --    // Destroy any contacts associated with the fixture.
   --    b2ContactEdge* edge = m_contactList;
   --    while (edge)
   --    {
   --       b2Contact* c = edge->contact;
   --       edge = edge->next;
   --
   --       b2Fixture* fixtureA = c->GetFixtureA();
   --       b2Fixture* fixtureB = c->GetFixtureB();
   --
   --       if (fixture == fixtureA || fixture == fixtureB)
   --       {
   --          // This destroys the contact and removes it from
   --          // this body's contact list.
   --          m_world->m_contactManager.Destroy(c);
   --       }
   --    }
   --
   --    b2BlockAllocator* allocator = &m_world->m_blockAllocator;
   --
   --    if (m_flags & e_enabledFlag)
   --    {
   --       b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
   --       fixture->DestroyProxies(broadPhase);
   --    }
   --
   --    fixture->m_body = nullptr;
   --    fixture->m_next = nullptr;
   --    fixture->Destroy(allocator);
   --    fixture->~b2Fixture();
   --    allocator->Free(fixture, sizeof(b2Fixture));
   --
   --    --m_fixtureCount;
   --
   --    // Reset the mass data.
   --    ResetMassData();
   --  }
   --

   procedure destroyFixture (Self : access b2Body;   fixture : in out b2_Fixture.b2Fixture_ptr)
   is
   begin
      if fixture = null
      then
         return;
      end if;

      pragma assert (Self.m_world.isLocked = false);

      if Self.m_world.isLocked = True
      then
         return;
      end if;

      pragma assert (fixture.m_body = Self);

      -- Remove the fixture from this body's singly linked list.
      --
      pragma assert (Self.m_fixtureCount > 0);

      declare
         type b2Fixture_ptr_ptr is access all b2Fixture_ptr;
         --  node : access b2Fixture** node := &Self.m_fixtureList;
         --  node : access b2Fixture_ptr := Self.m_fixtureList'Access;                -- TODO: Check this !
         node      : b2Fixture_ptr_ptr := Self.m_fixtureList'Access;                  -- TODO: Check this !
         found     : Boolean           := False;
         node_next : aliased b2Fixture_ptr;
      begin
         while node.all /= null
         loop
            if node.all = fixture
            then
               node.all := fixture.m_next;
               found    := True;
               exit;
            end if;

            node_next := node.all.m_Next;
            node      := node_next'Access;

            pragma assert (node = node.all.m_next'unrestricted_Access);
            --  node := node.all.m_next'unrestricted_Access;
         end loop;

         -- You tried to remove a shape that is not attached to this body.
         --
         pragma assert (Found);
      end;

      -- Destroy any contacts associated with the fixture.
      --
      declare
         edge : access b2ContactEdge := Self.m_contactList;
         c    : access b2Contact;

         fixtureA,
         fixtureB : b2Fixture_ptr;
      begin
         while edge /= null
         loop
            c    := edge.contact;
            edge := edge.next;

            fixtureA := c.GetFixtureA;
            fixtureB := c.GetFixtureB;

            if    fixture = fixtureA
               or fixture = fixtureB
            then
               -- This destroys the contact and removes it from this body's contact list.
               Self.m_world.m_contactManager.destroy (c);
            end if;
         end loop;
      end;

      if (Self.m_flags and e_enabledFlag) /= 0
      then
         declare
            broadPhase : constant access b2BroadPhase := Self.m_world.m_contactManager.m_broadPhase'Access;
         begin
            fixture.destroyProxies (broadPhase);
         end;
      end if;

      fixture.m_Body_is (null);
      fixture.m_Next_is (null);
      fixture.Destroy;
      -- fixture.destruct;      -- There is no destructor for b2Fixture !
      free (fixture);

      Self.m_fixtureCount := Self.m_fixtureCount - 1;

      Self.resetMassData;     -- Reset the mass data.
   end destroyFixture;





   --    Set the position of the body's origin and rotation.
   --    Manipulating a body's transform may cause non-physical behavior.
   --    Note: contacts are updated on the next call to b2World::Step.
   --
   --    @param position the world position of the body's local origin.
   --    @param angle the world rotation in radians.
   --
   --    void SetTransform(const b2Vec2& position, float angle);
   --
   --  void b2Body::SetTransform(const b2Vec2& position, float angle)
   --  {
   --    b2Assert(m_world->IsLocked() == false);
   --    if (m_world->IsLocked() == true)
   --    {
   --       return;
   --    }
   --
   --    m_xf.q.Set(angle);
   --    m_xf.p = position;
   --
   --    m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
   --    m_sweep.a = angle;
   --
   --    m_sweep.c0 = m_sweep.c;
   --    m_sweep.a0 = angle;
   --
   --    b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
   --    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --    {
   --       f->Synchronize(broadPhase, m_xf, m_xf);
   --    }
   --
   --    // Check for new contacts the next step
   --    m_world->m_newContacts = true;
   --  }
   --

   procedure setTransform (Self : in out b2Body;   position : in b2Vec2;
                                                   Angle    : in Real)
   is
      pragma assert (Self.m_world.isLocked = False);
   begin
      if Self.m_world.isLocked = True
      then
         return;
      end if;

      set (Self.m_xf.q, angle);
      Self.m_xf.p := position;

      Self.m_sweep.c  := b2Mul (Self.m_xf, Self.m_sweep.localCenter);
      Self.m_sweep.a  := angle;

      Self.m_sweep.c0 := Self.m_sweep.c;
      Self.m_sweep.a0 := angle;

      declare
         broadPhase : constant access b2BroadPhase := Self.m_world.m_contactManager.m_broadPhase'Access;
         f          :          access b2Fixture    := Self.m_fixtureList;
      begin
     --  for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
         while f /= null
         loop
            f.synchronize (broadPhase, Self.m_xf, Self.m_xf);
            f := f.m_next;
         end loop;
      end;

     -- Check for new contacts the next step.
     Self.m_world.m_newContacts_is (True);
   end setTransform;




   --    Get the body transform for the body's origin.
   --
   --    @return the world transform of the body's origin.
   --
   --    const b2Transform& GetTransform() const;
   --
   --  inline const b2Transform& b2Body::GetTransform() const
   --  {
   --    return m_xf;
   --  }
   --

   function getTransform (Self : in b2Body) return b2Transform
   is
   begin
      return Self.m_xf;
   end getTransform;




   --    Get the world body origin position.
   --
   --    @return the world position of the body's origin.
   --
   --    const b2Vec2& GetPosition() const;
   --
   --  inline const b2Vec2& b2Body::GetPosition() const
   --  {
   --    return m_xf.p;
   --  }
   --

   function getPosition (Self : in b2Body) return b2Vec2
   is
   begin
      return Self.m_xf.p;
   end getPosition;




   --    Get the angle in radians.
   --
   --    @return the current world rotation angle in radians.
   --
   --    float GetAngle() const;
   --
   --  inline float b2Body::GetAngle() const
   --  {
   --    return m_sweep.a;
   --  }
   --

   function getAngle (Self : in b2Body) return Real
   is
   begin
      return Self.m_sweep.a;
   end getAngle;



   --    Get the world position of the center of mass.
   --
   --    const b2Vec2& GetWorldCenter() const;
   --
   --  inline const b2Vec2& b2Body::GetWorldCenter() const
   --  {
   --    return m_sweep.c;
   --  }
   --

   function getWorldCenter (Self : in b2Body) return b2Vec2
   is
   begin
      return Self.m_sweep.c;
   end getWorldCenter;





   --    Get the local position of the center of mass.
   --
   --    const b2Vec2& GetLocalCenter() const;
   --
   --  inline const b2Vec2& b2Body::GetLocalCenter() const
   --  {
   --    return m_sweep.localCenter;
   --  }
   --

   function getLocalCenter (Self : in b2Body) return b2Vec2
   is
   begin
      return Self.m_sweep.localCenter;
   end getLocalCenter;




   --    Set the linear velocity of the center of mass.
   --
   --    @param v the new linear velocity of the center of mass.
   --
   --    void SetLinearVelocity(const b2Vec2& v);
   --
   --  inline void b2Body::SetLinearVelocity(const b2Vec2& v)
   --  {
   --    if (m_type == b2_staticBody)
   --    {
   --       return;
   --    }
   --
   --    if (b2Dot(v,v) > 0.0f)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    m_linearVelocity = v;
   --  }
   --

   procedure setLinearVelocity (Self : in out b2Body;   v : in b2Vec2)
   is
   begin
      if Self.m_type = b2_staticBody
      then
         return;
      end if;

      if b2Dot (v,v) > 0.0
      then
         Self.setAwake (True);
      end if;

      Self.m_linearVelocity := v;
   end setLinearVelocity;




   --    Get the linear velocity of the center of mass.
   --
   --    @return the linear velocity of the center of mass.
   --
   --    const b2Vec2& GetLinearVelocity() const;
   --
   --  inline const b2Vec2& b2Body::GetLinearVelocity() const
   --  {
   --    return m_linearVelocity;
   --  }
   --

   function getLinearVelocity (Self : in b2Body) return b2Vec2
   is
   begin
      return Self.m_linearVelocity;
   end getLinearVelocity;





   --    Set the angular velocity.
   --
   --    @param omega the new angular velocity in radians/second.
   --
   --    void SetAngularVelocity(float omega);
   --
   --  inline void b2Body::SetAngularVelocity(float w)
   --  {
   --    if (m_type == b2_staticBody)
   --    {
   --       return;
   --    }
   --
   --    if (w * w > 0.0f)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    m_angularVelocity = w;
   --  }
   --

   procedure setAngularVelocity (Self : in out b2Body;   omega : in Real)
   is
   begin
      if Self.m_type = b2_staticBody
      then
         return;
      end if;

      if omega * omega > 0.0
      then
         Self.setAwake (True);
      end if;

      Self.m_angularVelocity := omega;
   end setAngularVelocity;





   --    Get the angular velocity.
   --
   --    @return the angular velocity in radians/second.
   --
   --    float GetAngularVelocity() const;
   --
   --  inline float b2Body::GetAngularVelocity() const
   --  {
   --    return m_angularVelocity;
   --  }
   --

   function getAngularVelocity (Self : in b2Body) return Real
   is
   begin
      return Self.m_angularVelocity;
   end getAngularVelocity;




   --    Apply a force at a world point. If the force is not
   --    applied at the center of mass, it will generate a torque and
   --    affect the angular velocity. This wakes up the body.
   --
   --    @param force the world force vector, usually in Newtons (N).
   --    @param point the world position of the point of application.
   --    @param wake also wake up the body
   --
   --    void ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake);
   --
   --  inline void b2Body::ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake)
   --  {
   --    if (m_type != b2_dynamicBody)
   --    {
   --       return;
   --    }
   --
   --    if (wake && (m_flags & e_awakeFlag) == 0)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    // Don't accumulate a force if the body is sleeping.
   --    if (m_flags & e_awakeFlag)
   --    {
   --       m_force += force;
   --       m_torque += b2Cross(point - m_sweep.c, force);
   --    }
   --  }
   --

   procedure applyForce (Self : in out b2Body;   force : in b2Vec2;
                                                 Point : in b2Vec2;
                                                 Wake  : in Boolean)
   is
   begin
      if Self.m_type /= b2_dynamicBody
      then
         return;
      end if;

      if wake and (Self.m_flags and e_awakeFlag) = 0
      then
         Self.setAwake (True);
      end if;

      -- Don't accumulate a force if the body is sleeping.
      --
      if (Self.m_flags and e_awakeFlag) /= 0
      then
         Self.m_force  := Self.m_force  + force;
         Self.m_torque := Self.m_torque + b2Cross (point - Self.m_sweep.c,
                                                   force);
      end if;
   end applyForce;




   --    Apply a force to the center of mass. This wakes up the body.
   --
   --    @param force the world force vector, usually in Newtons (N).
   --    @param wake also wake up the body
   --
   --    void ApplyForceToCenter(const b2Vec2& force, bool wake);
   --
   --  inline void b2Body::ApplyForceToCenter(const b2Vec2& force, bool wake)
   --  {
   --    if (m_type != b2_dynamicBody)
   --    {
   --       return;
   --    }
   --
   --    if (wake && (m_flags & e_awakeFlag) == 0)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    // Don't accumulate a force if the body is sleeping
   --    if (m_flags & e_awakeFlag)
   --    {
   --       m_force += force;
   --    }
   --  }
   --

   procedure applyForceToCenter (Self : in out b2Body;   force : in b2Vec2;
                                                         Wake  : in Boolean)
   is
   begin
      if Self.m_type /= b2_dynamicBody
      then
         return;
      end if;

      if wake and (Self.m_flags and e_awakeFlag) = 0
      then
         Self.setAwake (True);
      end if;

      -- Don't accumulate a force if the body is sleeping.
      --
      if (Self.m_flags and e_awakeFlag) /= 0
      then
         Self.m_force := Self.m_force + force;
      end if;
   end applyForceToCenter;





   --    Apply a torque. This affects the angular velocity
   --    without affecting the linear velocity of the center of mass.
   --
   --    @param torque about the z-axis (out of the screen), usually in N-m.
   --    @param wake also wake up the body
   --
   --    void ApplyTorque(float torque, bool wake);
   --
   --  inline void b2Body::ApplyTorque(float torque, bool wake)
   --  {
   --    if (m_type != b2_dynamicBody)
   --    {
   --       return;
   --    }
   --
   --    if (wake && (m_flags & e_awakeFlag) == 0)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    // Don't accumulate a force if the body is sleeping
   --    if (m_flags & e_awakeFlag)
   --    {
   --       m_torque += torque;
   --    }
   --  }
   --

   procedure ApplyTorque (Self : in out b2Body;   Torque : in Real;
                          Wake   : in Boolean)
   is
   begin
      if Self.m_type /= b2_dynamicBody
      then
         return;
      end if;

      if     wake
        and (Self.m_flags and e_awakeFlag) = 0
      then
         Self.setAwake (True);
      end if;

      -- Don't accumulate a force if the body is sleeping.
      --
      if (Self.m_flags and e_awakeFlag) /= 0
      then
         Self.m_torque := Self.m_torque + torque;
      end if;
   end ApplyTorque;




   --    Apply an impulse at a point. This immediately modifies the velocity.
   --    It also modifies the angular velocity if the point of application
   --    is not at the center of mass. This wakes up the body.
   --
   --    @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
   --    @param point the world position of the point of application.
   --    @param wake also wake up the body
   --
   --    void ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake);
   --
   --  inline void b2Body::ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake)
   --  {
   --    if (m_type != b2_dynamicBody)
   --    {
   --       return;
   --    }
   --
   --    if (wake && (m_flags & e_awakeFlag) == 0)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    // Don't accumulate velocity if the body is sleeping
   --    if (m_flags & e_awakeFlag)
   --    {
   --       m_linearVelocity += m_invMass * impulse;
   --       m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
   --    }
   --  }
   --

   procedure applyLinearImpulse (Self : in out b2Body;   impulse : in b2Vec2;
                                 Point   : in b2Vec2;
                                 Wake    : in Boolean)
   is
   begin
      if Self.m_type /= b2_dynamicBody
      then
         return;
      end if;

      if     wake
        and (Self.m_flags and e_awakeFlag) = 0
      then
         Self.setAwake (True);
      end if;

      -- Don't accumulate velocity if the body is sleeping.
      --
      if (Self.m_flags and e_awakeFlag) /= 0
      then
         Self.m_linearVelocity  := Self.m_linearVelocity   +  Self.m_invMass * impulse;
         Self.m_angularVelocity := Self.m_angularVelocity  +  Self.m_invI    * b2Cross (point - Self.m_sweep.c,
                                                                                        impulse);
      end if;
   end applyLinearImpulse;




   --    Apply an impulse to the center of mass. This immediately modifies the velocity.
   --
   --    @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
   --    @param wake also wake up the body
   --
   --    void ApplyLinearImpulseToCenter(const b2Vec2& impulse, bool wake);
   --
   --  inline void b2Body::ApplyLinearImpulseToCenter(const b2Vec2& impulse, bool wake)
   --  {
   --    if (m_type != b2_dynamicBody)
   --    {
   --       return;
   --    }
   --
   --    if (wake && (m_flags & e_awakeFlag) == 0)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    // Don't accumulate velocity if the body is sleeping
   --    if (m_flags & e_awakeFlag)
   --    {
   --       m_linearVelocity += m_invMass * impulse;
   --    }
   --  }
   --

   procedure ApplyLinearImpulseToCenter (Self : in out b2Body;   impulse : in b2Vec2;
                                                                 Wake    : in Boolean)
   is
   begin
      if Self.m_type /= b2_dynamicBody
      then
         return;
      end if;

      if     wake
        and (Self.m_flags and e_awakeFlag) = 0
      then
         Self.setAwake (True);
      end if;

      -- Don't accumulate velocity if the body is sleeping.
      --
      if (Self.m_flags and e_awakeFlag) /= 0
      then
         Self.m_linearVelocity := Self.m_linearVelocity  +  Self.m_invMass * impulse;
      end if;
   end ApplyLinearImpulseToCenter;




   --    Apply an angular impulse.
   --
   --    @param impulse the angular impulse in units of kg*m*m/s
   --    @param wake also wake up the body
   --
   --    void ApplyAngularImpulse(float impulse, bool wake);
   --
   --  inline void b2Body::ApplyAngularImpulse(float impulse, bool wake)
   --  {
   --    if (m_type != b2_dynamicBody)
   --    {
   --       return;
   --    }
   --
   --    if (wake && (m_flags & e_awakeFlag) == 0)
   --    {
   --       SetAwake(true);
   --    }
   --
   --    // Don't accumulate velocity if the body is sleeping
   --    if (m_flags & e_awakeFlag)
   --    {
   --       m_angularVelocity += m_invI * impulse;
   --    }
   --  }
   --

   procedure applyAngularImpulse (Self : in out b2Body;   impulse : in Real;
                                  Wake    : in Boolean)
   is
   begin
      if Self.m_type /= b2_dynamicBody
      then
         return;
      end if;

      if      wake
        and  (Self.m_flags and e_awakeFlag) = 0
      then
         Self.setAwake (True);
      end if;

      -- Don't accumulate velocity if the body is sleeping.
      --
      if (Self.m_flags and e_awakeFlag) /= 0
      then
         Self.m_angularVelocity := Self.m_angularVelocity  +  Self.m_invI * impulse;
      end if;
   end applyAngularImpulse;




   --    Get the total mass of the body.
   --
   --    @return the mass, usually in kilograms (kg).
   --
   --    float GetMass() const;
   --
   --  inline float b2Body::GetMass() const
   --  {
   --    return m_mass;
   --  }
   --

   function getMass (Self : in b2Body) return Real
   is
   begin
      return Self.m_mass;
   end getMass;





   --    Get the rotational inertia of the body about the local origin.
   --
   --    @return the rotational inertia, usually in kg-m^2.
   --
   --    float GetInertia() const;
   --
   --  inline float b2Body::GetInertia() const
   --  {
   --    return m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
   --  }
   --

   function getInertia (Self : in b2Body) return Real
   is
   begin
      return Self.m_I + Self.m_mass * b2Dot (Self.m_sweep.localCenter,
                                             Self.m_sweep.localCenter);
   end getInertia;




   --    Get the mass data of the body.
   --
   --    @return a struct containing the mass, inertia and center of the body.
   --
   --    void GetMassData(b2MassData* data) const;
   --
   --  inline void b2Body::GetMassData(b2MassData* data) const
   --  {
   --    data->mass = m_mass;
   --    data->I = m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
   --    data->center = m_sweep.localCenter;
   --  }
   --

   procedure getMassData (Self : in b2Body;   data : out b2massData)
   is
   begin
      data.mass   := Self.m_mass;
      data.I      := Self.m_I + Self.m_mass * b2Dot (Self.m_sweep.localCenter,
                                                     Self.m_sweep.localCenter);
      data.center := Self.m_sweep.localCenter;
   end getMassData;




   --    Set the mass properties to override the mass properties of the fixtures.
   --    Note that this changes the center of mass position.
   --    Note that creating or destroying fixtures can also alter the mass.
   --    This function has no effect if the body isn't dynamic.
   --
   --    @param data the mass properties.
   --
   --    void SetMassData(const b2MassData* data);
   --
   --  void b2Body::SetMassData(const b2MassData* massData)
   --  {
   --    b2Assert(m_world->IsLocked() == false);
   --    if (m_world->IsLocked() == true)
   --    {
   --       return;
   --    }
   --
   --    if (m_type != b2_dynamicBody)
   --    {
   --       return;
   --    }
   --
   --    m_invMass = 0.0f;
   --    m_I = 0.0f;
   --    m_invI = 0.0f;
   --
   --    m_mass = massData->mass;
   --    if (m_mass <= 0.0f)
   --    {
   --       m_mass = 1.0f;
   --    }
   --
   --    m_invMass = 1.0f / m_mass;
   --
   --    if (massData->I > 0.0f && (m_flags & b2Body::e_fixedRotationFlag) == 0)
   --    {
   --       m_I = massData->I - m_mass * b2Dot(massData->center, massData->center);
   --       b2Assert(m_I > 0.0f);
   --       m_invI = 1.0f / m_I;
   --    }
   --
   --    // Move center of mass.
   --    b2Vec2 oldCenter = m_sweep.c;
   --    m_sweep.localCenter =  massData->center;
   --    m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
   --
   --    // Update center of mass velocity.
   --    m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
   --  }
   --

   procedure setMassData (Self : in out b2Body;   data : in b2massData)
   is
      pragma assert (Self.m_world.isLocked = False);
   begin
      if Self.m_world.isLocked = True
      then
         return;
      end if;

      if Self.m_type /= b2_dynamicBody
      then
         return;
      end if;

      Self.m_invMass := 0.0;
      Self.m_I       := 0.0;
      Self.m_invI    := 0.0;

      Self.m_mass    := Data.mass;

     if Self.m_mass <= 0.0
     then
        Self.m_mass := 1.0;
     end if;

     Self.m_invMass := 1.0 / Self.m_mass;

      if     Data.I > 0.0
        and (Self.m_flags and b2_Body.e_fixedRotationFlag) = 0
      then
         Self.m_I    := Data.I - Self.m_mass * b2Dot (Data.center, Data.center);
         pragma assert (Self.m_I > 0.0);
         Self.m_invI := 1.0 / Self.m_I;
      end if;

      -- Move center of mass.
      --
      declare
         oldCenter : constant b2Vec2 := Self.m_sweep.c;
      begin
         Self.m_sweep.localCenter := Data.center;
         Self.m_sweep.c           := b2Mul (Self.m_xf, Self.m_sweep.localCenter);
         Self.m_sweep.c0          := Self.m_sweep.c;

         -- Update center of mass velocity.
         --
         Self.m_linearVelocity := Self.m_linearVelocity + b2Cross (Self.m_angularVelocity,
                                                                   Self.m_sweep.c - oldCenter);
      end;
   end setMassData;





   --    This resets the mass properties to the sum of the mass properties of the fixtures.
   --    This normally does not need to be called unless you called SetMassData to override
   --    the mass and you later want to reset the mass.
   --
   --    void ResetMassData();
   --
   --  void b2Body::ResetMassData()
   --  {
   --    // Compute mass data from shapes. Each shape has its own density.
   --    m_mass = 0.0f;
   --    m_invMass = 0.0f;
   --    m_I = 0.0f;
   --    m_invI = 0.0f;
   --    m_sweep.localCenter.SetZero();
   --
   --    // Static and kinematic bodies have zero mass.
   --    if (m_type == b2_staticBody || m_type == b2_kinematicBody)
   --    {
   --       m_sweep.c0 = m_xf.p;
   --       m_sweep.c = m_xf.p;
   --       m_sweep.a0 = m_sweep.a;
   --       return;
   --    }
   --
   --    b2Assert(m_type == b2_dynamicBody);
   --
   --    // Accumulate mass over all fixtures.
   --    b2Vec2 localCenter = b2Vec2_zero;
   --    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --    {
   --       if (f->m_density == 0.0f)
   --       {
   --          continue;
   --       }
   --
   --       b2MassData massData;
   --       f->GetMassData(&massData);
   --       m_mass += massData.mass;
   --       localCenter += massData.mass * massData.center;
   --       m_I += massData.I;
   --    }
   --
   --    // Compute center of mass.
   --    if (m_mass > 0.0f)
   --    {
   --       m_invMass = 1.0f / m_mass;
   --       localCenter *= m_invMass;
   --    }
   --
   --    if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
   --    {
   --       // Center the inertia about the center of mass.
   --       m_I -= m_mass * b2Dot(localCenter, localCenter);
   --       b2Assert(m_I > 0.0f);
   --       m_invI = 1.0f / m_I;
   --
   --    }
   --    else
   --    {
   --       m_I = 0.0f;
   --       m_invI = 0.0f;
   --    }
   --
   --    // Move center of mass.
   --    b2Vec2 oldCenter = m_sweep.c;
   --    m_sweep.localCenter = localCenter;
   --    m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
   --
   --    // Update center of mass velocity.
   --    m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
   --  }
   --

   procedure resetMassData (Self : in out b2Body)
   is
   begin
      -- Compute mass data from shapes. Each shape has its own density.
      --
      Self.m_mass    := 0.0;
      Self.m_invMass := 0.0;
      Self.m_I       := 0.0;
      Self.m_invI    := 0.0;
      setZero (Self.m_sweep.localCenter);

      -- Static and kinematic bodies have zero mass.
      --
      if   Self.m_type = b2_staticBody
        or Self.m_type = b2_kinematicBody
      then
         Self.m_sweep.c0 := Self.m_xf.p;
         Self.m_sweep.c  := Self.m_xf.p;
         Self.m_sweep.a0 := Self.m_sweep.a;

         return;
      end if;

        pragma assert (Self.m_type = b2_dynamicBody);

      -- Accumulate mass over all fixtures.
      --
      declare
         localCenter :        b2Vec2    := b2Vec2_zero;
         f           : access b2Fixture := Self.m_fixtureList;
      begin
         while f /= null
         loop
            if f.m_density = 0.0
            then
               goto Continue;
            end if;

            declare
               massData : b2MassData;
            begin
               f.getMassData (massData);

               Self.m_mass := Self.m_mass + massData.mass;
               localCenter := localCenter + massData.mass * massData.center;
               Self.m_I    := Self.m_I + massData.I;
            end;

            <<Continue>>
            f := f.m_next;
         end loop;

         -- Compute center of mass.
         --
         if Self.m_mass > 0.0
         then
            Self.m_invMass := 1.0 / Self.m_mass;
            localCenter    := localCenter * Self.m_invMass;
         end if;

         if     Self.m_I > 0.0
           and (Self.m_flags and e_fixedRotationFlag) = 0
         then
            -- Center the inertia about the center of mass.
            --
            Self.m_I    := Self.m_I - Self.m_mass * b2Dot (localCenter,
                                                           localCenter);
            pragma assert (Self.m_I > 0.0);
            Self.m_invI := 1.0 / Self.m_I;

         else
            Self.m_I    := 0.0;
            Self.m_invI := 0.0;
         end if;

         -- Move center of mass.
         --
         declare
            oldCenter : constant b2Vec2 := Self.m_sweep.c;
         begin
            Self.m_sweep.localCenter := localCenter;
            Self.m_sweep.c           := b2Mul (Self.m_xf, Self.m_sweep.localCenter);
            Self.m_sweep.c0          := Self.m_sweep.c;

            -- Update center of mass velocity.
            --
            Self.m_linearVelocity := Self.m_linearVelocity + b2Cross (Self.m_angularVelocity,
                                                                      Self.m_sweep.c - oldCenter);
         end;
      end;
   end resetMassData;





   --    Get the world coordinates of a point given the local coordinates.
   --
   --    @param localPoint a point on the body measured relative the the body's origin.
   --    @return the same point expressed in world coordinates.
   --
   --    b2Vec2 GetWorldPoint(const b2Vec2& localPoint) const;
   --
   --  inline b2Vec2 b2Body::GetWorldPoint(const b2Vec2& localPoint) const
   --  {
   --    return b2Mul(m_xf, localPoint);
   --  }
   --

   function getWorldPoint (Self : in b2Body;   localPoint : in b2Vec2) return b2Vec2
   is
   begin
      return b2Mul (Self.m_xf, localPoint);
   end getWorldPoint;




   --    Get the world coordinates of a vector given the local coordinates.
   --
   --    @param localVector a vector fixed in the body.
   --    @return the same vector expressed in world coordinates.
   --
   --    b2Vec2 GetWorldVector(const b2Vec2& localVector) const;
   --
   --  inline b2Vec2 b2Body::GetWorldVector(const b2Vec2& localVector) const
   --  {
   --    return b2Mul(m_xf.q, localVector);
   --  }
   --

   function getWorldVector (Self : in b2Body;   localVector : in b2Vec2) return b2Vec2
   is
   begin
      return b2Mul (Self.m_xf.q, localVector);
   end getWorldVector;





   --    Gets a local point relative to the body's origin given a world point.
   --
   --    @param worldPoint a point in world coordinates.
   --    @return the corresponding local point relative to the body's origin.
   --
   --    b2Vec2 GetLocalPoint(const b2Vec2& worldPoint) const;
   --
   --  inline b2Vec2 b2Body::GetLocalPoint(const b2Vec2& worldPoint) const
   --  {
   --    return b2MulT(m_xf, worldPoint);
   --  }
   --

   function getLocalPoint (Self : in b2Body;   worldPoint : in b2Vec2) return b2Vec2
   is
   begin
      return b2MulT (Self.m_xf, worldPoint);
   end getLocalPoint;




   --    Gets a local vector given a world vector.
   --
   --    @param worldVector a vector in world coordinates.
   --    @return the corresponding local vector.
   --
   --    b2Vec2 GetLocalVector(const b2Vec2& worldVector) const;
   --
   --  inline b2Vec2 b2Body::GetLocalVector(const b2Vec2& worldVector) const
   --  {
   --    return b2MulT(m_xf.q, worldVector);
   --  }
   --

   function getLocalVector (Self : in b2Body;   worldVector : in b2Vec2) return b2Vec2
   is
   begin
      return b2MulT (Self.m_xf.q, worldVector);
   end getLocalVector;





   --    Get the world linear velocity of a world point attached to this body.
   --
   --    @param worldPoint a point in world coordinates.
   --    @return the world velocity of a point.
   --
   --    b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const;
   --
   --  inline b2Vec2 b2Body::GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const
   --  {
   --    return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
   --  }
   --

   function getLinearVelocityFromWorldPoint (Self : in b2Body;   worldPoint : in b2Vec2) return b2Vec2
   is
   begin
      return Self.m_linearVelocity + b2Cross (Self.m_angularVelocity,
                                              worldPoint - Self.m_sweep.c);
   end getLinearVelocityFromWorldPoint;





   --    Get the world velocity of a local point.
   --
   --    @param localPoint a point in local coordinates.
   --    @return the world velocity of a point.
   --
   --    b2Vec2 GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const;
   --
   --  inline b2Vec2 b2Body::GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const
   --  {
   --    return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
   --  }
   --

   function getLinearVelocityFromLocalPoint (Self : in b2Body;   localPoint : in b2Vec2) return b2Vec2
   is
   begin
      return Self.getLinearVelocityFromWorldPoint (Self.getWorldPoint (localPoint));
   end getLinearVelocityFromLocalPoint;




   --    Get the linear damping of the body.
   --
   --    float GetLinearDamping() const;
   --
   --  inline float b2Body::GetLinearDamping() const
   --  {
   --    return m_linearDamping;
   --  }
   --

   function getLinearDamping (Self : in b2Body) return Real
   is
   begin
      return Self.m_linearDamping;
   end getLinearDamping;




   --    Set the linear damping of the body.
   --
   --    void SetLinearDamping(float linearDamping);
   --
   --  inline void b2Body::SetLinearDamping(float linearDamping)
   --  {
   --    m_linearDamping = linearDamping;
   --  }
   --

   procedure setLinearDamping (Self : in out b2Body;   linearDamping : in Real)
   is
   begin
      Self.m_linearDamping := linearDamping;
   end setLinearDamping;




   --    Get the angular damping of the body.
   --
   --    float GetAngularDamping() const;
   --
   --  inline float b2Body::GetAngularDamping() const
   --  {
   --    return m_angularDamping;
   --  }
   --

   function getAngularDamping (Self : in b2Body) return Real
   is
   begin
      return Self.m_angularDamping;
   end getAngularDamping;





   --    Set the angular damping of the body.
   --
   --    void SetAngularDamping(float angularDamping);
   --
   --  inline void b2Body::SetAngularDamping(float angularDamping)
   --  {
   --    m_angularDamping = angularDamping;
   --  }
   --

   procedure setAngularDamping (Self : in out b2Body;   angularDamping : in Real)
   is
   begin
      Self.m_angularDamping := angularDamping;
   end setAngularDamping;




   --    Get the gravity scale of the body.
   --
   --    float GetGravityScale() const;
   --
   --  inline float b2Body::GetGravityScale() const
   --  {
   --    return m_gravityScale;
   --  }
   --

   function getGravityScale (Self : in b2Body) return Real
   is
   begin
      return Self.m_gravityScale;
   end getGravityScale;




   --    Set the gravity scale of the body.
   --
   --    void SetGravityScale(float scale);
   --
   --  inline void b2Body::SetGravityScale(float scale)
   --  {
   --    m_gravityScale = scale;
   --  }
   --

   procedure setGravityScale (Self : in out b2Body;   Scale : in Real)
   is
   begin
      Self.m_gravityScale := scale;
   end setGravityScale;




   --    Set the type of this body. This may alter the mass and velocity.
   --
   --    void SetType(b2BodyType type);
   --
   --  void b2Body::SetType(b2BodyType type)
   --  {
   --    b2Assert(m_world->IsLocked() == false);
   --    if (m_world->IsLocked() == true)
   --    {
   --       return;
   --    }
   --
   --    if (m_type == type)
   --    {
   --       return;
   --    }
   --
   --    m_type = type;
   --
   --    ResetMassData();
   --
   --    if (m_type == b2_staticBody)
   --    {
   --       m_linearVelocity.SetZero();
   --       m_angularVelocity = 0.0f;
   --       m_sweep.a0 = m_sweep.a;
   --       m_sweep.c0 = m_sweep.c;
   --       m_flags &= ~e_awakeFlag;
   --       SynchronizeFixtures();
   --    }
   --
   --    SetAwake(true);
   --
   --    m_force.SetZero();
   --    m_torque = 0.0f;
   --
   --    // Delete the attached contacts.
   --    b2ContactEdge* ce = m_contactList;
   --    while (ce)
   --    {
   --       b2ContactEdge* ce0 = ce;
   --       ce = ce->next;
   --       m_world->m_contactManager.Destroy(ce0->contact);
   --    }
   --    m_contactList = nullptr;
   --
   --    // Touch the proxies so that new contacts will be created (when appropriate)
   --    b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
   --    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --    {
   --       int32 proxyCount = f->m_proxyCount;
   --       for (int32 i = 0; i < proxyCount; ++i)
   --       {
   --          broadPhase->TouchProxy(f->m_proxies[i].proxyId);
   --       }
   --    }
   --  }
   --

   procedure setType (Self : in out b2Body;   Kind : in b2BodyType)
   is
      pragma assert (Self.m_world.isLocked = False);
   begin
      if Self.m_world.isLocked = True
      then
         return;
      end if;

      if Self.m_type = Kind
      then
         return;
      end if;

      Self.m_type := Kind;

      Self.resetMassData;

      if Self.m_type = b2_staticBody
      then
         setZero (Self.m_linearVelocity);

         Self.m_angularVelocity := 0.0;
         Self.m_sweep.a0        := Self.m_sweep.a;
         Self.m_sweep.c0        := Self.m_sweep.c;
         Self.m_flags           := Self.m_flags and not e_awakeFlag;

         Self.synchronizeFixtures;
      end if;

     Self.setAwake (True);

     setZero (Self.m_force);
     Self.m_torque := 0.0;

      -- Delete the attached contacts.
      --
      declare
         ce : access b2ContactEdge := Self.m_contactList;
      begin
         while ce /= null
         loop
            declare
               ce0 : constant access b2ContactEdge := ce;
            begin
               ce := ce.next;
               Self.m_world.m_contactManager.destroy (ce0.contact);
            end;
         end loop;
      end;

      Self.m_contactList := null;

      -- Touch the proxies so that new contacts will be created (when appropriate).
      --
      declare
         broadPhase : constant access b2BroadPhase := Self.m_world.m_contactManager.m_broadPhase'Access;
         f          :          access b2Fixture    := Self.m_fixtureList;
         proxyCount :                 Natural;
      begin
         while f /= null
         loop
            proxyCount := f.m_proxyCount;

            for i in 0 .. proxyCount - 1
            loop
               broadPhase.touchProxy (f.m_proxies (i).proxyId);
            end loop;

            f := f.m_next;
         end loop;
      end;
   end setType;




   --    Get the type of this body.
   --
   --  inline b2BodyType b2Body::GetType() const
   --  {
   --    return m_type;
   --  }
   --

   function getType (Self : in b2Body) return b2BodyType
   is
   begin
      return Self.m_type;
   end getType;





   --    Should this body be treated like a bullet for continuous collision detection?
   --
   --    void SetBullet(bool flag);
   --
   --  inline void b2Body::SetBullet(bool flag)
   --  {
   --    if (flag)
   --    {
   --       m_flags |= e_bulletFlag;
   --    }
   --    else
   --    {
   --       m_flags &= ~e_bulletFlag;
   --    }
   --  }
   --

   procedure setBullet (Self : in out b2Body;   Flag : in Boolean)
   is
   begin
      if flag then Self.m_flags := Self.m_flags or      e_bulletFlag;
              else Self.m_flags := Self.m_flags and not e_bulletFlag;
      end if;
   end setBullet;





   --    Is this body treated like a bullet for continuous collision detection?
   --
   --    bool IsBullet() const;
   --
   --  inline bool b2Body::IsBullet() const
   --  {
   --    return (m_flags & e_bulletFlag) == e_bulletFlag;
   --  }
   --

   function IsBullet (Self : in b2Body) return Boolean
   is
   begin
      return (Self.m_flags and e_bulletFlag) = e_bulletFlag;
   end IsBullet;





   --    You can disable sleeping on this body. If you disable sleeping, the
   --    body will be woken.
   --
   --    void SetSleepingAllowed(bool flag);
   --
   --  inline void b2Body::SetSleepingAllowed(bool flag)
   --  {
   --    if (flag)
   --    {
   --       m_flags |= e_autoSleepFlag;
   --    }
   --    else
   --    {
   --       m_flags &= ~e_autoSleepFlag;
   --       SetAwake(true);
   --    }
   --  }
   --

   procedure setSleepingAllowed (Self : in out b2Body;   Flag : in Boolean)
   is
   begin
      if flag
      then
         Self.m_flags := Self.m_flags or e_autoSleepFlag;
      else
         Self.m_flags := Self.m_flags and not e_autoSleepFlag;
         Self.setAwake (True);
      end if;
   end setSleepingAllowed;





   --    Is this body allowed to sleep
   --
   --    bool IsSleepingAllowed() const;
   --
   --  inline bool b2Body::IsSleepingAllowed() const
   --  {
   --    return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
   --  }
   --

   function isSleepingAllowed (Self : in b2Body) return Boolean
   is
   begin
      return (Self.m_flags and e_autoSleepFlag) = e_autoSleepFlag;
   end isSleepingAllowed;





   --    Set the sleep state of the body. A sleeping body has very
   --    low CPU cost.
   --
   --    @param flag set to true to wake the body, false to put it to sleep.
   --
   --    void SetAwake(bool flag);
   --
   --  inline void b2Body::SetAwake(bool flag)
   --  {
   --    if (m_type == b2_staticBody)
   --    {
   --       return;
   --    }
   --
   --    if (flag)
   --    {
   --       m_flags |= e_awakeFlag;
   --       m_sleepTime = 0.0f;
   --    }
   --    else
   --    {
   --       m_flags &= ~e_awakeFlag;
   --       m_sleepTime = 0.0f;
   --       m_linearVelocity.SetZero();
   --       m_angularVelocity = 0.0f;
   --       m_force.SetZero();
   --       m_torque = 0.0f;
   --    }
   --  }
   --

   procedure setAwake (Self : in out b2Body;   Flag : in Boolean)
   is
   begin
      if Self.m_type = b2_staticBody
      then
         return;
      end if;

      if flag
      then
         Self.m_flags     := Self.m_flags or e_awakeFlag;
         Self.m_sleepTime := 0.0;

      else
         Self.m_flags           := Self.m_flags and not e_awakeFlag;
         Self.m_sleepTime       := 0.0;
         Self.m_angularVelocity := 0.0;
         Self.m_torque          := 0.0;

         setZero (Self.m_linearVelocity);
         setZero (Self.m_force);
      end if;
   end setAwake;





   --    Get the sleeping state of this body.
   --
   --    @return true if the body is awake.
   --
   --    bool IsAwake() const;
   --
   --  inline bool b2Body::IsAwake() const
   --  {
   --    return (m_flags & e_awakeFlag) == e_awakeFlag;
   --  }
   --

   function isAwake (Self : in b2Body) return Boolean
   is
   begin
      return (Self.m_flags and e_awakeFlag) = e_awakeFlag;
   end isAwake;





   --    Allow a body to be disabled. A disabled body is not simulated and cannot
   --    be collided with or woken up.
   --    If you pass a flag of true, all fixtures will be added to the broad-phase.
   --    If you pass a flag of false, all fixtures will be removed from the
   --    broad-phase and all contacts will be destroyed.
   --    Fixtures and joints are otherwise unaffected. You may continue
   --    to create/destroy fixtures and joints on disabled bodies.
   --    Fixtures on a disabled body are implicitly disabled and will
   --    not participate in collisions, ray-casts, or queries.
   --    Joints connected to a disabled body are implicitly disabled.
   --    An diabled body is still owned by a b2World object and remains
   --    in the body list.
   --
   --    void SetEnabled(bool flag);
   --
   --  void b2Body::SetEnabled(bool flag)
   --  {
   --    b2Assert(m_world->IsLocked() == false);
   --
   --    if (flag == IsEnabled())
   --    {
   --       return;
   --    }
   --
   --    if (flag)
   --    {
   --       m_flags |= e_enabledFlag;
   --
   --       // Create all proxies.
   --       b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
   --       for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --       {
   --          f->CreateProxies(broadPhase, m_xf);
   --       }
   --
   --       // Contacts are created at the beginning of the next
   --       m_world->m_newContacts = true;
   --    }
   --    else
   --    {
   --       m_flags &= ~e_enabledFlag;
   --
   --       // Destroy all proxies.
   --       b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
   --       for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --       {
   --          f->DestroyProxies(broadPhase);
   --       }
   --
   --       // Destroy the attached contacts.
   --       b2ContactEdge* ce = m_contactList;
   --       while (ce)
   --       {
   --          b2ContactEdge* ce0 = ce;
   --          ce = ce->next;
   --          m_world->m_contactManager.Destroy(ce0->contact);
   --       }
   --       m_contactList = nullptr;
   --    }
   --  }
   --

   procedure setEnabled (Self : in out b2Body;   Flag : in Boolean)
   is
     pragma assert (Self.m_world.isLocked = False);
   begin
      if flag = Self.isEnabled
      then
         return;
      end if;

      if flag
      then
         Self.m_flags := Self.m_flags or e_enabledFlag;

         -- Create all proxies.
         --
         declare
            broadPhase : constant access b2BroadPhase := Self.m_world.m_contactManager.m_broadPhase'Access;
            f          :          access b2Fixture    := Self.m_fixtureList;
         begin
            while f /= null
            loop
               f.createProxies (broadPhase, Self.m_xf);
               f := f.m_next;
            end loop;
         end;

         -- Contacts are created at the beginning of the next.
         --
         Self.m_world.m_newContacts_is (True);

      else
         Self.m_flags := Self.m_flags and not e_enabledFlag;

         -- Destroy all proxies.
         --
         declare
            broadPhase : constant access b2BroadPhase := Self.m_world.m_contactManager.m_broadPhase'Access;
            f          :          access b2Fixture    := Self.m_fixtureList;
         begin
            while f /= null
            loop
               f.destroyProxies (broadPhase);
               f := f.m_next;
            end loop;
         end;

         -- Destroy the attached contacts.
         --
         declare
            ce  : access b2ContactEdge := Self.m_contactList;
            ce0 : access b2ContactEdge;
         begin
            while ce /= null
            loop
               ce0 := ce;
               ce  := ce.next;
               Self.m_world.m_contactManager.destroy (ce0.contact);
            end loop;

            Self.m_contactList := null;
         end;
      end if;
   end setEnabled;





   --    Get the active state of the body.
   --
   --    bool IsEnabled() const;
   --
   --  inline bool b2Body::IsEnabled() const
   --  {
   --    return (m_flags & e_enabledFlag) == e_enabledFlag;
   --  }
   --

   function isEnabled (Self : in b2Body) return Boolean
   is
   begin
      return (Self.m_flags and e_enabledFlag) = e_enabledFlag;
   end isEnabled;





   --    Set this body to have fixed rotation. This causes the mass
   --    to be reset.
   --
   --    void SetFixedRotation(bool flag);
   --
   --  void b2Body::SetFixedRotation(bool flag)
   --  {
   --    bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
   --    if (status == flag)
   --    {
   --       return;
   --    }
   --
   --    if (flag)
   --    {
   --       m_flags |= e_fixedRotationFlag;
   --    }
   --    else
   --    {
   --       m_flags &= ~e_fixedRotationFlag;
   --    }
   --
   --    m_angularVelocity = 0.0f;
   --
   --    ResetMassData();
   --  }
   --

   procedure setFixedRotation (Self : in out b2Body;   Flag : in Boolean)
   is
   begin
      null;
   end setFixedRotation;





   --    Does this body have fixed rotation?
   --
   --    bool IsFixedRotation() const;
   --
   --  inline bool b2Body::IsFixedRotation() const
   --  {
   --    return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
   --  }
   --

   function isFixedRotation (Self : in b2Body) return Boolean
   is
   begin
      return (Self.m_flags and e_fixedRotationFlag) = e_fixedRotationFlag;
   end isFixedRotation;




   --    Get the list of all fixtures attached to this body.
   --
   --    b2Fixture* GetFixtureList();
   --
   --  inline b2Fixture* b2Body::GetFixtureList()
   --  {
   --    return m_fixtureList;
   --  }
   --

   function getFixtureList (Self : in out b2Body) return access b2_Fixture.b2Fixture
   is
   begin
      return Self.m_fixtureList;
   end getFixtureList;



   --    const b2Fixture* GetFixtureList() const;
   --
   --  inline const b2Fixture* b2Body::GetFixtureList() const
   --  {
   --    return m_fixtureList;
   --  }
   --

   --  function getFixtureList (Self : in b2Body) return access constant b2_Fixture.b2Fixture
   --  is
   --  begin
   --     return Self.m_fixtureList;
   --  end getFixtureList;





   --    Get the list of all joints attached to this body.
   --
   --    b2JointEdge* GetJointList();
   --
   --  inline b2JointEdge* b2Body::GetJointList()
   --  {
   --    return m_jointList;
   --  }
   --

   function getJointList (Self : in out b2Body) return access b2_Joint.b2JointEdge
   is
   begin
      return Self.m_jointList;
   end getJointList;




   --    const b2JointEdge* GetJointList() const;
   --
   --  inline const b2JointEdge* b2Body::GetJointList() const
   --  {
   --    return m_jointList;
   --  }
   --

   --  function getJointList (Self : in b2Body) return access constant b2_Joint.b2JointEdge
   --  is
   --  begin
   --     return Self.m_jointList;
   --  end getJointList;






   --    Get the list of all contacts attached to this body.
   --
   --    @warning this list changes during the time step and you may
   --    miss some collisions if you don't use b2ContactListener.
   --
   --    b2ContactEdge* GetContactList();
   --
   --  inline b2ContactEdge* b2Body::GetContactList()
   --  {
   --    return m_contactList;
   --  }
   --

   function getContactList (Self : in out b2Body) return access b2_Contact.b2ContactEdge
   is
   begin
      return Self.m_contactList;
   end getContactList;



   --    const b2ContactEdge* GetContactList() const;
   --
   --  inline const b2ContactEdge* b2Body::GetContactList() const
   --  {
   --    return m_contactList;
   --  }
   --

   --  function getContactList (Self : in b2Body) return b2_Contact.b2ContactEdge
   --  is
   --  begin
   --     return Self.m_contactList;
   --  end getContactList;




   --    Get the next body in the world's body list.
   --
   --    b2Body* GetNext();
   --
   --  inline b2Body* b2Body::GetNext()
   --  {
   --    return m_next;
   --  }
   --

   function getNext (Self : in out b2Body) return access b2Body
   is
   begin
      return Self.m_next;
   end getNext;



   --    const b2Body* GetNext() const;
   --
   --  inline const b2Body* b2Body::GetNext() const
   --  {
   --    return m_next;
   --  }
   --

   --  function getNext (Self : in b2Body) return access constant b2Body
   --  is
   --  begin
   --     return Self.m_next;
   --  end getNext;




   --    Get the user data pointer that was provided in the body definition.
   --
   --    b2BodyUserData& GetUserData();
   --
   --  inline b2BodyUserData& b2Body::GetUserData()
   --  {
   --    return m_userData;
   --  }
   --

   function getUserData (Self : in b2Body) return b2BodyUserData
   is
   begin
      return Self.m_userData;
   end getUserData;




   --    Set the user data. Use this to store your application specific data.
   --
   --    void SetUserData(void* data);
   --

   procedure setUserData (Self : in out b2Body;   data : in b2BodyUserData)
   is
   begin
      Self.m_userData := Data;
   end setUserData;



   --    Get the parent world of this body.
   --
   --    b2World* GetWorld();
   --
   --  inline const b2World* b2Body::GetWorld() const
   --  {
   --    return m_world;
   --  }
   --

   function getWorld (Self : in b2Body) return b2_World.b2World
   is
   begin
      return Self.m_world.all;
   end getWorld;



   --    const b2World* GetWorld() const;
   --
   --  inline b2World* b2Body::GetWorld()
   --  {
   --    return m_world;
   --  }
   --

   function getWorld (Self : in out b2Body) return access b2_World.b2World
   is
   begin
      return Self.m_world;
   end getWorld;






   --    Dump this body to a file
   --
   --    void Dump();
   --
   --  void b2Body::Dump()
   --  {
   --    int32 bodyIndex = m_islandIndex;
   --
   --    // %.9g is sufficient to save and load the same value using text
   --    // FLT_DECIMAL_DIG == 9
   --
   --    b2Dump("{\n");
   --    b2Dump("  b2BodyDef bd;\n");
   --    b2Dump("  bd.type = b2BodyType(%d);\n", m_type);
   --    b2Dump("  bd.position.Set(%.9g, %.9g);\n", m_xf.p.x, m_xf.p.y);
   --    b2Dump("  bd.angle = %.9g;\n", m_sweep.a);
   --    b2Dump("  bd.linearVelocity.Set(%.9g, %.9g);\n", m_linearVelocity.x, m_linearVelocity.y);
   --    b2Dump("  bd.angularVelocity = %.9g;\n", m_angularVelocity);
   --    b2Dump("  bd.linearDamping = %.9g;\n", m_linearDamping);
   --    b2Dump("  bd.angularDamping = %.9g;\n", m_angularDamping);
   --    b2Dump("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
   --    b2Dump("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
   --    b2Dump("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
   --    b2Dump("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
   --    b2Dump("  bd.enabled = bool(%d);\n", m_flags & e_enabledFlag);
   --    b2Dump("  bd.gravityScale = %.9g;\n", m_gravityScale);
   --    b2Dump("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
   --    b2Dump("\n");
   --    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --    {
   --       b2Dump("  {\n");
   --       f->Dump(bodyIndex);
   --       b2Dump("  }\n");
   --    }
   --    b2Dump("}\n");
   --  }
   --

   procedure dump (Self : in out b2Body)
   is
   begin
      raise program_Error with "TODO";
   end dump;





   --    b2Body(const b2BodyDef* bd, b2World* world);
   --
   --  b2Body::b2Body(const b2BodyDef* bd, b2World* world)
   --  {
   --    b2Assert(bd->position.IsValid());
   --    b2Assert(bd->linearVelocity.IsValid());
   --    b2Assert(b2IsValid(bd->angle));
   --    b2Assert(b2IsValid(bd->angularVelocity));
   --    b2Assert(b2IsValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
   --    b2Assert(b2IsValid(bd->linearDamping) && bd->linearDamping >= 0.0f);
   --
   --    m_flags = 0;
   --
   --    if (bd->bullet)
   --    {
   --       m_flags |= e_bulletFlag;
   --    }
   --    if (bd->fixedRotation)
   --    {
   --       m_flags |= e_fixedRotationFlag;
   --    }
   --    if (bd->allowSleep)
   --    {
   --       m_flags |= e_autoSleepFlag;
   --    }
   --    if (bd->awake && bd->type != b2_staticBody)
   --    {
   --       m_flags |= e_awakeFlag;
   --    }
   --    if (bd->enabled)
   --    {
   --       m_flags |= e_enabledFlag;
   --    }
   --
   --    m_world = world;
   --
   --    m_xf.p = bd->position;
   --    m_xf.q.Set(bd->angle);
   --
   --    m_sweep.localCenter.SetZero();
   --    m_sweep.c0 = m_xf.p;
   --    m_sweep.c = m_xf.p;
   --    m_sweep.a0 = bd->angle;
   --    m_sweep.a = bd->angle;
   --    m_sweep.alpha0 = 0.0f;
   --
   --    m_jointList = nullptr;
   --    m_contactList = nullptr;
   --    m_prev = nullptr;
   --    m_next = nullptr;
   --
   --    m_linearVelocity = bd->linearVelocity;
   --    m_angularVelocity = bd->angularVelocity;
   --
   --    m_linearDamping = bd->linearDamping;
   --    m_angularDamping = bd->angularDamping;
   --    m_gravityScale = bd->gravityScale;
   --
   --    m_force.SetZero();
   --    m_torque = 0.0f;
   --
   --    m_sleepTime = 0.0f;
   --
   --    m_type = bd->type;
   --
   --    m_mass = 0.0f;
   --    m_invMass = 0.0f;
   --
   --    m_I = 0.0f;
   --    m_invI = 0.0f;
   --
   --    m_userData = bd->userData;
   --
   --    m_fixtureList = nullptr;
   --    m_fixtureCount = 0;
   --  }
   --

   --  function to_b2Body (bd : in b2BodyDef;   world : in b2World_ptr) return b2Body
   --

   function to_b2Body (bd : in b2BodyDef;   world : access b2_World.b2World) return b2Body
   is
      pragma assert (isValid (bd.position));
      pragma assert (ISvALID (bd.linearVelocity));

      pragma assert (b2IsValid (bd.angle));
      pragma assert (b2IsValid (bd.angularVelocity));
      pragma assert (b2IsValid (bd.angularDamping) and bd.angularDamping >= 0.0);
      pragma assert (b2IsValid (bd.linearDamping)  and bd.linearDamping  >= 0.0);

      Self : b2Body;

   begin
      Self.m_flags := 0;

      if bd.bullet
      then
         Self.m_flags := Self.m_flags or e_bulletFlag;
      end if;

      if bd.fixedRotation
      then
         Self.m_flags := Self.m_flags or e_fixedRotationFlag;
      end if;

      if bd.allowSleep
      then
         Self.m_flags := Self.m_flags or e_autoSleepFlag;
      end if;

      if    bd.awake
        and bd.Kind /= b2_staticBody
      then
         Self.m_flags := Self.m_flags or e_awakeFlag;
      end if;

      if bd.enabled
      then
         Self.m_flags := Self.m_flags or e_enabledFlag;
      end if;

      Self.m_world := world;

      Self.m_xf.p := bd.position;
      set (Self.m_xf.q, bd.angle);

      setZero (Self.m_sweep.localCenter);
      Self.m_sweep.c0     := Self.m_xf.p;
      Self.m_sweep.c      := Self.m_xf.p;
      Self.m_sweep.a0     := bd.angle;
      Self.m_sweep.a      := bd.angle;
      Self.m_sweep.alpha0 := 0.0;

      Self.m_jointList   := null;
      Self.m_contactList := null;
      Self.m_prev        := null;
      Self.m_next        := null;

      Self.m_linearVelocity  := bd.linearVelocity;
      Self.m_angularVelocity := bd.angularVelocity;

      Self.m_linearDamping  := bd.linearDamping;
      Self.m_angularDamping := bd.angularDamping;
      Self.m_gravityScale   := bd.gravityScale;

      setZero (Self.m_force);
      Self.m_torque    := 0.0;
      Self.m_sleepTime := 0.0;

      Self.m_type      := bd.Kind;

      Self.m_mass      := 0.0;
      Self.m_invMass   := 0.0;

      Self.m_I    := 0.0;
      Self.m_invI := 0.0;

      Self.m_userData := bd.userData;

      Self.m_fixtureList  := null;
      Self.m_fixtureCount := 0;

      return Self;
   end to_b2Body;





   --  b2Body::~b2Body()
   --  {
   --    // shapes and joints are destroyed in b2World::Destroy
   --  }
   --

   procedure destruct (Self : in out b2Body)
   is
   begin
      null;     -- Shapes and joints are destroyed in b2World::Destroy.
   end destruct;



   procedure free (Self : in out b2Body_ptr)
   is
      procedure deallocate is new ada.unchecked_Deallocation (b2Body, b2Body_ptr);
   begin
      deallocate (Self);
   end free;




   --    void SynchronizeFixtures();
   --
   --  void b2Body::SynchronizeFixtures()
   --  {
   --    b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
   --
   --    if (m_flags & b2Body::e_awakeFlag)
   --    {
   --       b2Transform xf1;
   --       xf1.q.Set(m_sweep.a0);
   --       xf1.p = m_sweep.c0 - b2Mul(xf1.q, m_sweep.localCenter);
   --
   --       for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --       {
   --          f->Synchronize(broadPhase, xf1, m_xf);
   --       }
   --    }
   --    else
   --    {
   --       for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
   --       {
   --          f->Synchronize(broadPhase, m_xf, m_xf);
   --       }
   --    }
   --  }
   --

   procedure synchronizeFixtures (Self : in out b2Body)
   is
      broadPhase : constant access b2BroadPhase := Self.m_world.m_contactManager.m_broadPhase'Access;

      xf1 :        b2Transform;
      f   : access b2Fixture  := Self.m_fixtureList;

   begin
      if (Self.m_flags and b2_Body.e_awakeFlag) /= 0
      then
         set (xf1.q, Self.m_sweep.a0);

         xf1.p := Self.m_sweep.c0 - b2Mul (xf1.q,
                                           Self.m_sweep.localCenter);
         while f /= null
         loop
            f.synchronize (broadPhase, xf1, Self.m_xf);

            f := f.m_Next;
         end loop;

      else
         while f /= null
         loop
            f.synchronize (broadPhase, Self.m_xf, Self.m_xf);

            f := f.m_Next;
         end loop;
      end if;
   end synchronizeFixtures;




   --    void SynchronizeTransform();
   --
   --  inline void b2Body::SynchronizeTransform()
   --  {
   --    m_xf.q.Set(m_sweep.a);
   --    m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
   --  }
   --

   procedure synchronizeTransform (Self : in out b2Body)
   is
   begin
      set (Self.m_xf.q,
           Self.m_sweep.a);

      Self.m_xf.p := Self.m_sweep.c - b2Mul (Self.m_xf.q,
                                             Self.m_sweep.localCenter);
   end synchronizeTransform;




   --    This is used to prevent connected bodies from colliding.
   --    It may lie, depending on the collideConnected flag.

   --    bool ShouldCollide(const b2Body* other) const;
   --
   --  bool b2Body::ShouldCollide(const b2Body* other) const
   --  {
   --    // At least one body should be dynamic.
   --    if (m_type != b2_dynamicBody && other->m_type != b2_dynamicBody)
   --    {
   --       return false;
   --    }
   --
   --    // Does a joint prevent collision?
   --    for (b2JointEdge* jn = m_jointList; jn; jn = jn->next)
   --    {
   --       if (jn->other == other)
   --       {
   --          if (jn->joint->m_collideConnected == false)
   --          {
   --             return false;
   --          }
   --       }
   --    }
   --
   --    return true;
   --  }
   --

   function shouldCollide (Self : in b2Body;   other : access b2Body) return Boolean
   is
   begin
      -- At least one body should be dynamic.
      --
      if     Self.m_type /= b2_dynamicBody
        and Other.m_type /= b2_dynamicBody
      then
         return False;
      end if;

      -- Does a joint prevent collision?
      --
      declare
         jn : access b2_Joint.b2JointEdge := Self.m_jointList;
      begin
         while jn /= null
         loop
            if jn.other = other
            then
               if jn.joint.getCollideConnected = False
               then
                  return False;
               end if;
            end if;

            jn := jn.next;
         end loop;
      end;

      return True;
   end shouldCollide;




   --    void Advance(float t);
   --
   --  };
   --
   --  inline void b2Body::Advance(float alpha)
   --  {
   --    // Advance to the new safe time. This doesn't sync the broad-phase.
   --    m_sweep.Advance(alpha);
   --    m_sweep.c = m_sweep.c0;
   --    m_sweep.a = m_sweep.a0;
   --    m_xf.q.Set(m_sweep.a);
   --    m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
   --  }
   --

   procedure advance (Self : in out b2Body;   t : in Real)
   is
   begin
      -- Advance to the new safe time. This doesn't sync the broad-phase.
      --
      Self.m_sweep.advance (t);
      Self.m_sweep.c := Self.m_sweep.c0;
      Self.m_sweep.a := Self.m_sweep.a0;
      set (Self.m_xf.q, Self.m_sweep.a);
      Self.m_xf.p    := Self.m_sweep.c - b2Mul (Self.m_xf.q,
                                                Self.m_sweep.localCenter);
   end advance;





   ----------------------------
   --- Protected C++ functions.
   --


   function m_contactList (Self : in b2Body) return access b2_Contact.b2ContactEdge
   is
   begin
      return Self.m_contactList;
   end m_contactList;


   procedure m_contactList_is (Self : in out b2Body;   Now : access b2_Contact.b2ContactEdge)
   is
   begin
      if Now = null
      then
         Self.m_contactList := null;
      else
         Self.m_contactList :=  Now.all'unchecked_Access;
      end if;
      --  Self.m_contactList := Now.all'unchecked_Access;
   end m_contactList_is;



   function m_Force (Self : in b2Body) return b2Vec2
   is
   begin
      return Self.m_Force;
   end m_Force;


   procedure m_Force_is  (Self : in out b2Body;   Now : in b2Vec2)
   is
   begin
      Self.m_force := Now;
   end m_Force_is;



   function m_Torque (Self : in b2Body) return Real
   is
   begin
      return Self.m_Torque;
   end m_Torque;


   procedure m_Torque_is (Self : in out b2Body;   Now : in Real)
   is
   begin
      Self.m_torque := Now;
   end m_Torque_is;



   procedure m_prev_is (Self : in out b2Body;   Now : in b2Body_ptr)
   is
   begin
      Self.m_prev := Now;
   end m_prev_is;



   procedure m_next_is (Self : in out b2Body;   Now : in b2Body_ptr)
   is
   begin
      Self.m_next := Now;
   end m_next_is;



   function m_prev (Self : in b2Body) return b2Body_ptr
   is
   begin
      return Self.m_prev;
   end m_prev;



   function m_next (Self : in b2Body) return b2Body_ptr
   is
   begin
      return Self.m_next;
   end m_next;





   procedure m_jointList_is (Self : in out b2Body;   Now : access b2JointEdge)
   is
   begin
      Self.m_jointList := Now;
   end m_jointList_is;



   procedure m_fixtureList_is (Self : in out b2Body;   Now : in b2Fixture_ptr)
   is
   begin
      Self.m_fixtureList := Now;
   end m_fixtureList_is;



   function m_sleepTime (Self : in b2Body) return Real
   is
   begin
      return Self.m_sleepTime;
   end m_sleepTime;


   procedure m_sleepTime_is (Self : in out b2Body;   Now : in Real)
   is
   begin
      Self.m_sleepTime := Now;
   end m_sleepTime_is;



   procedure decrement_m_fixtureCount (Self : in out b2Body)
   is
   begin
      Self.m_fixtureCount := Self.m_fixtureCount - 1;
   end decrement_m_fixtureCount;



   procedure zero_m_fixtureCount (Self : in out b2Body)
   is
   begin
      Self.m_fixtureCount := 0;
   end zero_m_fixtureCount;




   function m_islandIndex (Self : in b2Body) return Natural
   is
   begin
      return Self.m_islandIndex;
   end m_islandIndex;



   procedure m_islandIndex_is (Self : in out b2Body;   Now : in Natural)
   is
   begin
      Self.m_islandIndex := Now;
   end m_islandIndex_is;




   function m_invMass (Self : in b2Body) return Real
   is
   begin
      return Self.m_invMass;
   end m_invMass;




   function m_invI (Self : in b2Body) return Real
   is
   begin
      return Self.m_invI;
   end m_invI;




   function m_xf (Self : access b2Body) return access b2Transform
   is
   begin
      return Self.m_xf'Access;
   end m_xf;




   function m_sweep (Self : access b2Body) return access b2Sweep
   is
   begin
      return Self.m_sweep'Access;
   end m_sweep;



   function m_Flags (Self : in b2Body) return flag_Set
   is
   begin
      return Self.m_Flags;
   end m_Flags;


   procedure m_Flags_is (Self : in out b2Body;   Now : flag_Set)
   is
   begin
      Self.m_Flags := Now;
   end m_Flags_is;


end b2_Body;
