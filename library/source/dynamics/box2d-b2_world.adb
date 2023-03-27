with
     box2d.b2_contact_Solver,
     box2d.b2_Island,
     box2d.b2_broad_Phase,
     box2d.b2_chain_Shape,
     box2d.b2_circle_Shape,
     box2d.b2_edge_Shape,
     box2d.b2_polygon_Shape,
--  box2d.b2_pulley_Joint,
     box2d.b2_Time_of_impact,
     box2d.b2_Timer,
     box2d.b2_World,
     box2d.b2_Common,

     Interfaces;


package body box2d.b2_World
is
   use b2_broad_Phase;

   --  b2World::b2World(const b2Vec2& gravity)
   --  {
   --    m_destructionListener = nullptr;
   --    m_debugDraw = nullptr;
   --
   --    m_bodyList = nullptr;
   --    m_jointList = nullptr;
   --
   --    m_bodyCount = 0;
   --    m_jointCount = 0;
   --
   --    m_warmStarting = true;
   --    m_continuousPhysics = true;
   --    m_subStepping = false;
   --
   --    m_stepComplete = true;
   --
   --    m_allowSleep = true;
   --    m_gravity = gravity;
   --
   --    m_newContacts = false;
   --    m_locked = false;
   --    m_clearForces = true;
   --
   --    m_inv_dt0 = 0.0f;
   --
   --    m_contactManager.m_allocator = &m_blockAllocator;
   --
   --    memset(&m_profile, 0, sizeof(b2Profile));
   --  }
   --

   function to_b2World (gravity : in b2Vec2) return b2World
   is
      Self : b2World;
   begin
      Self.m_bodyCount  := 0;
      Self.m_jointCount := 0;

      Self.m_warmStarting      := True;
      Self.m_continuousPhysics := True;
      Self.m_subStepping       := False;

      Self.m_stepComplete := True;

      Self.m_allowSleep := True;
      Self.m_gravity    := Gravity;

      Self.m_newContacts := False;
      Self.m_locked      := False;
      Self.m_clearForces := True;

      Self.m_inv_dt0 := 0.0;

      Self.m_profile := (others => 0.0);

      Self.m_contactManager := to_b2ContactManager;

      return Self;
   end to_b2World;




   --  b2World::~b2World()
   --  {
   --    // Some shapes allocate using b2Alloc.
   --    b2Body* b = m_bodyList;
   --    while (b)
   --    {
   --       b2Body* bNext = b->m_next;
   --
   --       b2Fixture* f = b->m_fixtureList;
   --       while (f)
   --       {
   --          b2Fixture* fNext = f->m_next;
   --          f->m_proxyCount = 0;
   --          f->Destroy(&m_blockAllocator);
   --          f = fNext;
   --       }
   --
   --       b = bNext;
   --    }
   --  }
   --

   procedure destruct (Self : in out b2World)
   is
      -- Some shapes allocate using b2Alloc.
      b     : access b2Body := Self.m_bodyList;
      bNext : access b2Body;
      f     : access b2Fixture;
      fNext : access b2Fixture;

   begin
      while b /= null
      loop
         bNext := b.getNext;
         f     := b.getFixtureList;

         while f /= null
         loop
           fNext := f.m_next;
           f.m_proxyCount_is (0);
           f.destroy;
           f     := fNext;
         end loop;

         b := bNext;
      end loop;


      Self.m_contactManager.destruct;
   end destruct;




   --  inline b2Body* b2World::GetBodyList()
   --  {
   --    return m_bodyList;
   --  }
   --

   --  inline const b2Body* b2World::GetBodyList() const
   --  {
   --    return m_bodyList;
   --  }
   --

   function GetBodyList (Self : in b2World) return b2Body_ptr
   is
   begin
      return Self.m_bodyList;
   end GetBodyList;






   --  inline b2Joint* b2World::GetJointList()
   --  {
   --    return m_jointList;
   --  }
   --

   --  inline const b2Joint* b2World::GetJointList() const
   --  {
   --    return m_jointList;
   --  }
   --

   function getJointList (Self : in b2World) return b2Joint_ptr
   is
   begin
      return Self.m_jointList;
   end getJointList;




   --  inline b2Contact* b2World::GetContactList()
   --  {
   --    return m_contactManager.m_contactList;
   --  }
   --

   --  inline const b2Contact* b2World::GetContactList() const
   --  {
   --    return m_contactManager.m_contactList;
   --  }
   --

   function GetContactList (Self : in b2World) return b2Contact_ptr
   is
   begin
      return Self.m_contactManager.m_contactList;
   end GetContactList;




   --  inline int32 b2World::GetBodyCount() const
   --  {
   --    return m_bodyCount;
   --  }
   --

   function GetBodyCount (Self : in b2World) return Natural
   is
   begin
      return Self.m_bodyCount;
   end GetBodyCount;




   --  inline int32 b2World::GetJointCount() const
   --  {
   --    return m_jointCount;
   --  }
   --

   function GetJointCount (Self : in b2World) return Natural
   is
   begin
      return Self.m_jointCount;
   end GetJointCount;





   --  inline int32 b2World::GetContactCount() const
   --  {
   --    return m_contactManager.m_contactCount;
   --  }
   --

   function GetContactCount (Self : in b2World) return Natural
   is
   begin
      return Self.m_contactManager.m_contactCount;
   end GetContactCount;




   --  inline void b2World::SetGravity(const b2Vec2& gravity)
   --  {
   --    m_gravity = gravity;
   --  }
   --

   procedure SetGravity (Self : in out b2World;   gravity : in b2Vec2)
   is
   begin
      Self.m_gravity := gravity;
   end SetGravity;




   --  inline b2Vec2 b2World::GetGravity() const
   --  {
   --    return m_gravity;
   --  }
   --

   function GetGravity (Self : in b2World) return b2Vec2
   is
   begin
      return Self.m_gravity;
   end GetGravity;




   --  inline void b2World::SetAutoClearForces(bool flag)
   --  {
   --    m_clearForces = flag;
   --  }
   --

   procedure SetAutoClearForces (Self : in out b2World;   flag : in Boolean)
   is
   begin
      Self.m_clearForces := flag;
   end SetAutoClearForces;




   --  Get the flag that controls automatic clearing of forces after each time step.
   --  inline bool b2World::GetAutoClearForces() const
   --  {
   --    return m_clearForces;
   --  }
   --

   function GetAutoClearForces (Self : in b2World) return Boolean
   is
   begin
      return Self.m_clearForces;
   end GetAutoClearForces;




   --  inline const b2ContactManager& b2World::GetContactManager() const
   --  {
   --    return m_contactManager;
   --  }
   --

   function GetContactManager (Self : in b2World) return b2ContactManager
   is
   begin
      return Self.m_contactManager;
   end GetContactManager;




   --  inline const b2Profile& b2World::GetProfile() const
   --  {
   --    return m_profile;
   --  }
   --

   function GetProfile (Self : in b2World) return b2Profile
   is
   begin
      return Self.m_profile;
   end GetProfile;




   --  inline bool b2World::IsLocked() const
   --  {
   --    return m_locked;
   --  }
   --

   function isLocked (Self : in b2World) return Boolean
   is
   begin
      return Self.m_locked;
   end isLocked;




   --  void b2World::SetDestructionListener(b2DestructionListener* listener)
   --  {
   --    m_destructionListener = listener;
   --  }
   --

   procedure setDestructionListener (Self : in out b2World;   listener : access b2DestructionListener)
   is
   begin
      Self.m_destructionListener := listener;
   end setDestructionListener;




   --  void b2World::SetContactFilter(b2ContactFilter* filter)
   --  {
   --    m_contactManager.m_contactFilter = filter;
   --  }
   --

   procedure SetContactFilter (Self : in out b2World;   filter : access b2ContactFilter)

   is
   begin
      Self.m_contactManager.m_contactFilter := filter;
   end SetContactFilter;





   --  void b2World::SetContactListener(b2ContactListener* listener)
   --  {
   --    m_contactManager.m_contactListener = listener;
   --  }
   --

   procedure setContactListener (Self : in out b2World;   listener : access b2ContactListener'Class)
   is
   begin
      Self.m_contactManager.m_contactListener := listener;
   end SetContactListener;




   --  void b2World::SetDebugDraw(b2Draw* debugDraw)
   --  {
   --    m_debugDraw = debugDraw;
   --  }
   --

   procedure SetDebugDraw (Self : in out b2World;   debugDraw : access b2Draw)
   is
   begin
      Self.m_debugDraw := debugDraw;
   end SetDebugDraw;






   --  b2Body* b2World::CreateBody(const b2BodyDef* def)
   --  {
   --    b2Assert(IsLocked() == false);
   --    if (IsLocked())
   --    {
   --       return nullptr;
   --    }
   --
   --    void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
   --    b2Body* b = new (mem) b2Body(def, this);
   --
   --    // Add to world doubly linked list.
   --    b->m_prev = nullptr;
   --    b->m_next = m_bodyList;
   --    if (m_bodyList)
   --    {
   --       m_bodyList->m_prev = b;
   --    }
   --    m_bodyList = b;
   --    ++m_bodyCount;
   --
   --    return b;
   --  }
   --

   function createBody (Self : in out b2World;   def : in b2BodyDef) return b2Body_ptr
   is
      pragma assert (Self.isLocked = False);
   begin
      if Self.isLocked
      then
         return null;
      end if;

      declare
         b : constant b2Body_ptr := new b2Body' (to_b2Body (bd    => def,
                                                            world => Self'unchecked_Access));
      begin
         -- Add to world doubly linked list.
         --
         b.m_prev_is (null);
         b.m_next_is (Self.m_bodyList);

         if Self.m_bodyList /= null
         then
            Self.m_bodyList.m_prev_is (b);
         end if;

         Self.m_bodyList  := b;
         Self.m_bodyCount := Self.m_bodyCount + 1;

         return b;
      end;
   end createBody;




   --  void b2World::DestroyBody(b2Body* b)
   --  {
   --    b2Assert(m_bodyCount > 0);
   --    b2Assert(IsLocked() == false);
   --    if (IsLocked())
   --    {
   --       return;
   --    }
   --
   --    // Delete the attached joints.
   --    b2JointEdge* je = b->m_jointList;
   --    while (je)
   --    {
   --       b2JointEdge* je0 = je;
   --       je = je->next;
   --
   --       if (m_destructionListener)
   --       {
   --          m_destructionListener->SayGoodbye(je0->joint);
   --       }
   --
   --       DestroyJoint(je0->joint);
   --
   --       b->m_jointList = je;
   --    }
   --    b->m_jointList = nullptr;
   --
   --    // Delete the attached contacts.
   --    b2ContactEdge* ce = b->m_contactList;
   --    while (ce)
   --    {
   --       b2ContactEdge* ce0 = ce;
   --       ce = ce->next;
   --       m_contactManager.Destroy(ce0->contact);
   --    }
   --    b->m_contactList = nullptr;
   --
   --    // Delete the attached fixtures. This destroys broad-phase proxies.
   --    b2Fixture* f = b->m_fixtureList;
   --    while (f)
   --    {
   --       b2Fixture* f0 = f;
   --       f = f->m_next;
   --
   --       if (m_destructionListener)
   --       {
   --          m_destructionListener->SayGoodbye(f0);
   --       }
   --
   --       f0->DestroyProxies(&m_contactManager.m_broadPhase);
   --       f0->Destroy(&m_blockAllocator);
   --       f0->~b2Fixture();
   --       m_blockAllocator.Free(f0, sizeof(b2Fixture));
   --
   --       b->m_fixtureList = f;
   --       b->m_fixtureCount -= 1;
   --    }
   --    b->m_fixtureList = nullptr;
   --    b->m_fixtureCount = 0;
   --
   --    // Remove world body list.
   --    if (b->m_prev)
   --    {
   --       b->m_prev->m_next = b->m_next;
   --    }
   --
   --    if (b->m_next)
   --    {
   --       b->m_next->m_prev = b->m_prev;
   --    }
   --
   --    if (b == m_bodyList)
   --    {
   --       m_bodyList = b->m_next;
   --    }
   --
   --    --m_bodyCount;
   --    b->~b2Body();
   --    m_blockAllocator.Free(b, sizeof(b2Body));
   --  }
   --

   procedure destroyBody (Self : in out b2World;   the_Body : in out b2Body_ptr)
   is
      pragma assert (Self.m_bodyCount > 0);
      pragma assert (Self.isLocked = False);

      b : b2Body_ptr renames the_Body;

   begin
     if Self.isLocked
     then
        return;
     end if;

      declare
         je  : access b2JointEdge := b.getJointList;
         je0 : access b2JointEdge;
      begin
      -- Delete the attached joints.
      --
         while je /= null
         loop
            je0 := je;
            je  := je.next;

            if Self.m_destructionListener /= null
            then
               Self.m_destructionListener.sayGoodbye (je0.joint);
            end if;

            Self.destroyJoint (je0.joint);

            b.m_jointList_is (je);
         end loop;
      end;

      b.m_jointList_is (null);

      -- Delete the attached contacts.
      --
      declare
         ce  : access b2ContactEdge := b.m_contactList;
         ce0 : access b2ContactEdge;
      begin
         while ce /= null
         loop
            ce0 := ce;
            ce  := ce.next;

            Self.m_contactManager.destroy (ce0.contact);
         end loop;
      end;

      b.m_contactList_is (null);

      -- Delete the attached fixtures. This destroys broad-phase proxies.
      --
      declare
         f  : b2Fixture_ptr := b.getFixtureList;
         f0 : b2Fixture_ptr;
      begin
         while f /= null
         loop
            f0 := f;
            f  := f.m_next;

            if Self.m_destructionListener /= null
            then
               Self.m_destructionListener.sayGoodbye (f0);
            end if;

            f0.destroyProxies (Self.m_contactManager.m_broadPhase'Access);
            f0.destroy;
            f0.destruct;
            free (f0);

            b.m_fixtureList_is (f);
            b.decrement_m_fixtureCount;
         end loop;
      end;

      b.m_fixtureList_is (null);
      b.zero_m_fixtureCount;

      -- Remove world body list.
      --
      if b.m_prev /= null
      then
         b.m_prev.m_next_is (b.m_next);
      end if;

      if b.m_next /= null
      then
         b.m_next.m_prev_is (b.m_prev);
      end if;

      if b = Self.m_bodyList
      then
         Self.m_bodyList := b.m_next;
      end if;

      Self.m_bodyCount := Self.m_bodyCount - 1;

      b.destruct;
      free (b);
   end destroyBody;




   --  b2Joint* b2World::CreateJoint(const b2JointDef* def)
   --  {
   --    b2Assert(IsLocked() == false);
   --    if (IsLocked())
   --    {
   --       return nullptr;
   --    }
   --
   --    b2Joint* j = b2Joint::Create(def, &m_blockAllocator);
   --
   --    // Connect to the world list.
   --    j->m_prev = nullptr;
   --    j->m_next = m_jointList;
   --    if (m_jointList)
   --    {
   --       m_jointList->m_prev = j;
   --    }
   --    m_jointList = j;
   --    ++m_jointCount;
   --
   --    // Connect to the bodies' doubly linked lists.
   --    j->m_edgeA.joint = j;
   --    j->m_edgeA.other = j->m_bodyB;
   --    j->m_edgeA.prev = nullptr;
   --    j->m_edgeA.next = j->m_bodyA->m_jointList;
   --    if (j->m_bodyA->m_jointList) j->m_bodyA->m_jointList->prev = &j->m_edgeA;
   --    j->m_bodyA->m_jointList = &j->m_edgeA;
   --
   --    j->m_edgeB.joint = j;
   --    j->m_edgeB.other = j->m_bodyA;
   --    j->m_edgeB.prev = nullptr;
   --    j->m_edgeB.next = j->m_bodyB->m_jointList;
   --    if (j->m_bodyB->m_jointList) j->m_bodyB->m_jointList->prev = &j->m_edgeB;
   --    j->m_bodyB->m_jointList = &j->m_edgeB;
   --
   --    b2Body* bodyA = def->bodyA;
   --    b2Body* bodyB = def->bodyB;
   --
   --    // If the joint prevents collisions, then flag any contacts for filtering.
   --    if (def->collideConnected == false)
   --    {
   --       b2ContactEdge* edge = bodyB->GetContactList();
   --       while (edge)
   --       {
   --          if (edge->other == bodyA)
   --          {
   --             // Flag the contact for filtering at the next time step (where either
   --             // body is awake).
   --             edge->contact->FlagForFiltering();
   --          }
   --
   --          edge = edge->next;
   --       }
   --    }
   --
   --    // Note: creating a joint doesn't wake the bodies.
   --
   --    return j;
   --  }
   --

   function createJoint (Self : in out b2World;   def : in b2JointDef) return b2Joint_ptr
   is
      pragma assert (Self.isLocked = False);
   begin
      if Self.isLocked
      then
         return null;
      end if;

      declare
         j : constant b2Joint_ptr := b2_Joint.create (def);
      begin
         -- Connect to the world list.
         --
         j.m_prev_is (null);
         j.m_next_is (Self.m_jointList);

         if Self.m_jointList /= null
         then
            Self.m_jointList.m_prev_is (j);
         end if;

         Self.m_jointList  := j;
         Self.m_jointCount := Self.m_jointCount + 1;

         -- Connect to the bodies' doubly linked lists.
         --
         j.m_edgeA.joint := j;
         j.m_edgeA.other := j.getBodyB;
         j.m_edgeA.prev  := null;
         j.m_edgeA.next  := j.getBodyA.getJointList;

         if j.getBodyA.getJointList /= null
         then
            j.getBodyA.getJointList.prev := j.m_edgeA;
         end if;

         j.getBodyA.m_jointList_is (j.m_edgeA);

         j.m_edgeB.joint := j;
         j.m_edgeB.other := j.getBodyA;
         j.m_edgeB.prev  := null;
         j.m_edgeB.next  := j.getBodyB.getJointList;

         if j.getBodyB.getJointList /= null
         then
            j.getBodyB.getJointList.prev := j.m_edgeB;
         end if;

        j.getBodyB.m_jointList_is (j.m_edgeB);

         declare
            bodyA : constant b2Body_ptr := def.bodyA;
            bodyB : constant b2Body_ptr := def.bodyB;

            edge  : access b2ContactEdge;
         begin
            -- If the joint prevents collisions, then flag any contacts for filtering.
            --
            if def.collideConnected = False
            then
               edge := bodyB.getContactList;

               while edge /= null
               loop
                  if edge.other = bodyA
                  then
                     -- Flag the contact for filtering at the next time step (where either body is awake).
                     --
                     edge.contact.flagForFiltering;
                  end if;

                  edge := edge.next;
               end loop;
            end if;
         end;

        -- Note: creating a joint doesn't wake the bodies.

         return j;
      end;
   end createJoint;




   --  void b2World::DestroyJoint(b2Joint* j)
   --  {
   --    b2Assert(IsLocked() == false);
   --    if (IsLocked())
   --    {
   --       return;
   --    }
   --
   --    bool collideConnected = j->m_collideConnected;
   --
   --    // Remove from the doubly linked list.
   --    if (j->m_prev)
   --    {
   --       j->m_prev->m_next = j->m_next;
   --    }
   --
   --    if (j->m_next)
   --    {
   --       j->m_next->m_prev = j->m_prev;
   --    }
   --
   --    if (j == m_jointList)
   --    {
   --       m_jointList = j->m_next;
   --    }
   --
   --    // Disconnect from island graph.
   --    b2Body* bodyA = j->m_bodyA;
   --    b2Body* bodyB = j->m_bodyB;
   --
   --    // Wake up connected bodies.
   --    bodyA->SetAwake(true);
   --    bodyB->SetAwake(true);
   --
   --    // Remove from body 1.
   --    if (j->m_edgeA.prev)
   --    {
   --       j->m_edgeA.prev->next = j->m_edgeA.next;
   --    }
   --
   --    if (j->m_edgeA.next)
   --    {
   --       j->m_edgeA.next->prev = j->m_edgeA.prev;
   --    }
   --
   --    if (&j->m_edgeA == bodyA->m_jointList)
   --    {
   --       bodyA->m_jointList = j->m_edgeA.next;
   --    }
   --
   --    j->m_edgeA.prev = nullptr;
   --    j->m_edgeA.next = nullptr;
   --
   --    // Remove from body 2
   --    if (j->m_edgeB.prev)
   --    {
   --       j->m_edgeB.prev->next = j->m_edgeB.next;
   --    }
   --
   --    if (j->m_edgeB.next)
   --    {
   --       j->m_edgeB.next->prev = j->m_edgeB.prev;
   --    }
   --
   --    if (&j->m_edgeB == bodyB->m_jointList)
   --    {
   --       bodyB->m_jointList = j->m_edgeB.next;
   --    }
   --
   --    j->m_edgeB.prev = nullptr;
   --    j->m_edgeB.next = nullptr;
   --
   --    b2Joint::Destroy(j, &m_blockAllocator);
   --
   --    b2Assert(m_jointCount > 0);
   --    --m_jointCount;
   --
   --    // If the joint prevents collisions, then flag any contacts for filtering.
   --    if (collideConnected == false)
   --    {
   --       b2ContactEdge* edge = bodyB->GetContactList();
   --       while (edge)
   --       {
   --          if (edge->other == bodyA)
   --          {
   --             // Flag the contact for filtering at the next time step (where either
   --             // body is awake).
   --             edge->contact->FlagForFiltering();
   --          }
   --
   --          edge = edge->next;
   --       }
   --    }
   --  }
   --

   procedure destroyJoint (Self : in out b2World;   joint : in b2Joint_ptr)
   is
      pragma assert (Self.isLocked = False);

      j                : b2Joint_ptr renames joint;
      collideConnected : Boolean;

   begin
      if Self.isLocked
      then
         return;
      end if;

      collideConnected := j.getCollideConnected;

      -- Remove from the doubly linked list.
      --
      if j.m_prev /= null
      then
         j.m_prev.m_next_is (j.m_next);
      end if;

      if j.m_next /= null
      then
         j.m_next.m_prev_is (j.m_prev);
      end if;

      if j = Self.m_jointList
      then
         Self.m_jointList := j.m_next;
      end if;

      -- Disconnect from island graph.
      --
      declare
         bodyA : constant access b2Body := j.getBodyA;
         bodyB : constant access b2Body := j.getBodyB;
      begin
         -- Wake up connected bodies.
         --
         bodyA.setAwake (True);
         bodyB.setAwake (True);

         -- Remove from body 1.
         --
         if j.m_edgeA.prev /= null
         then
            j.m_edgeA.prev.next := j.m_edgeA.next;
         end if;

         if j.m_edgeA.next /= null
         then
            j.m_edgeA.next.prev := j.m_edgeA.prev;
         end if;

         if j.m_edgeA = bodyA.getJointList
         then
            bodyA.m_jointList_is (j.m_edgeA.next);
         end if;

         j.m_edgeA.prev := null;
         j.m_edgeA.next := null;

         -- Remove from body 2
         --
         if j.m_edgeB.prev /= null
         then
            j.m_edgeB.prev.next := j.m_edgeB.next;
         end if;

         if j.m_edgeB.next /= null
         then
            j.m_edgeB.next.prev := j.m_edgeB.prev;
         end if;

         if j.m_edgeB = bodyB.getJointList
         then
            bodyB.m_jointList_is (j.m_edgeB.next);
         end if;

         j.m_edgeB.prev := null;
         j.m_edgeB.next := null;

         b2_Joint.destroy (j);

         pragma assert       (Self.m_jointCount > 0);
         Self.m_jointCount := Self.m_jointCount - 1;

         -- If the joint prevents collisions, then flag any contacts for filtering.
         --
         if collideConnected = False
         then
            declare
               edge : access b2ContactEdge := bodyB.getContactList;
            begin
               while edge /= null
               loop
                  if edge.other = bodyA
                  then
                     -- Flag the contact for filtering at the next time step (where either body is awake).
                     edge.contact.flagForFiltering;
                  end if;

                  edge := edge.next;
               end loop;
            end;
         end if;
      end;
   end destroyJoint;




   --  void b2World::SetAllowSleeping(bool flag)
   --  {
   --    if (flag == m_allowSleep)
   --    {
   --       return;
   --    }
   --
   --    m_allowSleep = flag;
   --    if (m_allowSleep == false)
   --    {
   --       for (b2Body* b = m_bodyList; b; b = b->m_next)
   --       {
   --          b->SetAwake(true);
   --       }
   --    }
   --  }
   --

   procedure setAllowSleeping (Self : in out b2World;   flag : in Boolean)
   is
   begin
      if flag = Self.m_allowSleep
      then
         return;
      end if;

      Self.m_allowSleep := flag;

      if Self.m_allowSleep = False
      then
         declare
            b : access b2Body := Self.m_bodyList;
         begin
        --  for (b2Body* b = m_bodyList; b; b = b->m_next)
            while b /= null
            loop
               b.setAwake (True);
               b := b.m_next;
            end loop;
         end;
      end if;
   end setAllowSleeping;




   --    bool GetAllowSleeping() const { return m_allowSleep; }
   --

   function GetAllowSleeping (Self : in b2World) return Boolean
   is
   begin
      return Self.m_allowSleep;
   end GetAllowSleeping;




   --    Enable/disable warm starting. For testing.
   --

   --    void SetWarmStarting(bool flag) { m_warmStarting = flag; }
   --

   procedure SetWarmStarting (Self : in out b2World;   flag : in Boolean)
   is
   begin
      Self.m_warmStarting := flag;
   end SetWarmStarting;



   --    bool GetWarmStarting() const { return m_warmStarting; }
   --

   function GetWarmStarting (Self : in b2World) return Boolean
   is
   begin
      return Self.m_warmStarting;
   end GetWarmStarting;




   --    Enable/disable continuous physics. For testing.
   --

   --    void SetContinuousPhysics(bool flag) { m_continuousPhysics = flag; }
   --

   procedure SetContinuousPhysics (Self : in out b2World;   flag : in Boolean)
   is
   begin
      Self.m_continuousPhysics := flag;
   end SetContinuousPhysics;



   --    bool GetContinuousPhysics() const { return m_continuousPhysics; }
   --

   function GetContinuousPhysics (Self : in b2World) return Boolean
   is
   begin
      return Self.m_continuousPhysics;
   end GetContinuousPhysics;



   --    Enable/disable single stepped continuous physics. For testing.
   --

   --    void SetSubStepping(bool flag) { m_subStepping = flag; }
   --

   procedure SetSubStepping (Self : in out b2World;   flag : in Boolean)
   is
   begin
      Self.m_subStepping := flag;
   end SetSubStepping;



   --    bool GetSubStepping() const { return m_subStepping; }
   --

   function GetSubStepping (Self : in b2World) return Boolean
   is
   begin
      return Self.m_subStepping;
   end GetSubStepping;









   --  // Find islands, integrate and solve constraints, solve position constraints
   --
   --  void b2World::Solve(const b2TimeStep& step)
   --  {
   --    m_profile.solveInit = 0.0f;
   --    m_profile.solveVelocity = 0.0f;
   --    m_profile.solvePosition = 0.0f;
   --
   --    // Size the island for the worst case.
   --    b2Island island(m_bodyCount,
   --                m_contactManager.m_contactCount,
   --                m_jointCount,
   --                &m_stackAllocator,
   --                m_contactManager.m_contactListener);
   --
   --    // Clear all the island flags.
   --    for (b2Body* b = m_bodyList; b; b = b->m_next)
   --    {
   --       b->m_flags &= ~b2Body::e_islandFlag;
   --    }
   --    for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
   --    {
   --       c->m_flags &= ~b2Contact::e_islandFlag;
   --    }
   --    for (b2Joint* j = m_jointList; j; j = j->m_next)
   --    {
   --       j->m_islandFlag = false;
   --    }
   --
   --    // Build and simulate all awake islands.
   --    int32 stackSize = m_bodyCount;
   --    b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
   --    for (b2Body* seed = m_bodyList; seed; seed = seed->m_next)
   --    {
   --       if (seed->m_flags & b2Body::e_islandFlag)
   --       {
   --          continue;
   --       }
   --
   --       if (seed->IsAwake() == false || seed->IsEnabled() == false)
   --       {
   --          continue;
   --       }
   --
   --       // The seed can be dynamic or kinematic.
   --       if (seed->GetType() == b2_staticBody)
   --       {
   --          continue;
   --       }
   --
   --       // Reset island and stack.
   --       island.Clear();
   --       int32 stackCount = 0;
   --       stack[stackCount++] = seed;
   --       seed->m_flags |= b2Body::e_islandFlag;
   --
   --       // Perform a depth first search (DFS) on the constraint graph.
   --       while (stackCount > 0)
   --       {
   --          // Grab the next body off the stack and add it to the island.
   --          b2Body* b = stack[--stackCount];
   --          b2Assert(b->IsEnabled() == true);
   --          island.Add(b);
   --
   --          // To keep islands as small as possible, we don't
   --          // propagate islands across static bodies.
   --          if (b->GetType() == b2_staticBody)
   --          {
   --             continue;
   --          }
   --
   --          // Make sure the body is awake (without resetting sleep timer).
   --          b->m_flags |= b2Body::e_awakeFlag;
   --
   --          // Search all contacts connected to this body.
   --          for (b2ContactEdge* ce = b->m_contactList; ce; ce = ce->next)
   --          {
   --             b2Contact* contact = ce->contact;
   --
   --             // Has this contact already been added to an island?
   --             if (contact->m_flags & b2Contact::e_islandFlag)
   --             {
   --                continue;
   --             }
   --
   --             // Is this contact solid and touching?
   --             if (contact->IsEnabled() == false ||
   --                contact->IsTouching() == false)
   --             {
   --                continue;
   --             }
   --
   --             // Skip sensors.
   --             bool sensorA = contact->m_fixtureA->m_isSensor;
   --             bool sensorB = contact->m_fixtureB->m_isSensor;
   --             if (sensorA || sensorB)
   --             {
   --                continue;
   --             }
   --
   --             island.Add(contact);
   --             contact->m_flags |= b2Contact::e_islandFlag;
   --
   --             b2Body* other = ce->other;
   --
   --             // Was the other body already added to this island?
   --             if (other->m_flags & b2Body::e_islandFlag)
   --             {
   --                continue;
   --             }
   --
   --             b2Assert(stackCount < stackSize);
   --             stack[stackCount++] = other;
   --             other->m_flags |= b2Body::e_islandFlag;
   --          }
   --
   --          // Search all joints connect to this body.
   --          for (b2JointEdge* je = b->m_jointList; je; je = je->next)
   --          {
   --             if (je->joint->m_islandFlag == true)
   --             {
   --                continue;
   --             }
   --
   --             b2Body* other = je->other;
   --
   --             // Don't simulate joints connected to diabled bodies.
   --             if (other->IsEnabled() == false)
   --             {
   --                continue;
   --             }
   --
   --             island.Add(je->joint);
   --             je->joint->m_islandFlag = true;
   --
   --             if (other->m_flags & b2Body::e_islandFlag)
   --             {
   --                continue;
   --             }
   --
   --             b2Assert(stackCount < stackSize);
   --             stack[stackCount++] = other;
   --             other->m_flags |= b2Body::e_islandFlag;
   --          }
   --       }
   --
   --       b2Profile profile;
   --       island.Solve(&profile, step, m_gravity, m_allowSleep);
   --       m_profile.solveInit += profile.solveInit;
   --       m_profile.solveVelocity += profile.solveVelocity;
   --       m_profile.solvePosition += profile.solvePosition;
   --
   --       // Post solve cleanup.
   --       for (int32 i = 0; i < island.m_bodyCount; ++i)
   --       {
   --          // Allow static bodies to participate in other islands.
   --          b2Body* b = island.m_bodies[i];
   --          if (b->GetType() == b2_staticBody)
   --          {
   --             b->m_flags &= ~b2Body::e_islandFlag;
   --          }
   --       }
   --    }
   --
   --    m_stackAllocator.Free(stack);
   --
   --    {
   --       b2Timer timer;
   --       // Synchronize fixtures, check for out of range bodies.
   --       for (b2Body* b = m_bodyList; b; b = b->GetNext())
   --       {
   --          // If a body was not in an island then it did not move.
   --          if ((b->m_flags & b2Body::e_islandFlag) == 0)
   --          {
   --             continue;
   --          }
   --
   --          if (b->GetType() == b2_staticBody)
   --          {
   --             continue;
   --          }
   --
   --          // Update fixtures (for broad-phase).
   --          b->SynchronizeFixtures();
   --       }
   --
   --       // Look for new contacts.
   --       m_contactManager.FindNewContacts();
   --       m_profile.broadphase = timer.GetMilliseconds();
   --    }
   --  }
   --

   procedure Solve (Self : in out b2World;   step : in b2TimeStep)
   is
      use b2_Island;
   begin
      Self.m_profile.solveInit     := 0.0;
      Self.m_profile.solveVelocity := 0.0;
      Self.m_profile.solvePosition := 0.0;

      declare
         use Interfaces;

         -- Size the island for the worst case.
         --
         island : b2Island := to_b2Island (Self.m_bodyCount,
                                           Self.m_contactManager.m_contactCount,
                                           Self.m_jointCount,
                                           Self.m_contactManager.m_contactListener);
         b :        b2Body_ptr;
         c : access b2Contact;
         j : access b2Joint;

      begin
         -- Clear all the island flags.
         --
         b := Self.m_bodyList;

         while b /= null
         loop
            b.m_Flags_is (b.m_flags and not b2_Body.e_islandFlag);
            b         := b.m_next;
         end loop;


         c := Self.m_contactManager.m_contactList;

         while c /= null
         loop
            c.m_flags_is (c.m_flags and not b2_Contact.e_islandFlag);
            c         := c.m_next;
         end loop;


         j := Self.m_jointList;

         while j /= null
         loop
            j.m_islandFlag_is (False);
            j              := j.m_next;
         end loop;


         -- Build and simulate all awake islands.
         --
         declare

            type b2Body_Stack is array (Natural range <>) of b2Body_ptr;

            stackCount :          Natural;
            stackSize  : constant Natural := Self.m_bodyCount;
            stack      :          b2Body_Stack (0 .. stackSize - 1);

            seed       :          b2Body_ptr := Self.m_bodyList;
         begin
            while seed /= null
            loop
               if (seed.m_flags and b2_Body.e_islandFlag) /= 0
               then
                  goto Continue_1;
               end if;

               if   seed.isAwake   = False
                 or seed.isEnabled = False
               then
                  goto Continue_1;
               end if;

               -- The seed can be dynamic or kinematic.
               --
               if seed.getType = b2_staticBody
               then
                  goto Continue_1;
               end if;

               -- Reset island and stack.
               --
               island.clear;
               stackCount         := 0;
               stack (stackCount) := seed;
               stackCount         := stackCount + 1;

               seed.m_Flags_is (seed.m_flags or b2_Body.e_islandFlag);

               -- Perform a depth first search (DFS) on the constraint graph.
               --
               while stackCount > 0
               loop
                  -- Grab the next body off the stack and add it to the island.
                  --
                  stackCount := stackCount - 1;
                  b          := stack (stackCount);
                  pragma assert (b.isEnabled = True);

                  island.add (b);

                  -- To keep islands as small as possible, we don't
                  -- propagate islands across static bodies.
                  --
                  if b.getType = b2_staticBody
                  then
                     goto Continue_2;
                  end if;

                  -- Make sure the body is awake (without resetting sleep timer).
                  --
                  b.m_Flags_is ( b.m_flags or b2_Body.e_awakeFlag);

                  -- Search all contacts connected to this body.
                  --
                  declare
                     ce      : access b2ContactEdge := b.m_contactList;
                     contact : access b2Contact;
                     other   :        b2Body_ptr;
                  begin
                     while ce /= null
                     loop
                        contact := ce.contact;

                        -- Has this contact already been added to an island?
                        --
                        if (contact.m_flags and b2_Contact.e_islandFlag) /= 0
                        then
                           goto Continue_3;
                        end if;

                        -- Is this contact solid and touching?
                        --
                        if   contact.isEnabled  = False
                          or contact.isTouching = False
                        then
                           goto Continue_3;
                        end if;

                        -- Skip sensors.
                        --
                        declare
                           sensorA : constant Boolean := contact.getFixtureA.isSensor;
                           sensorB : constant Boolean := contact.getfixtureB.isSensor;
                        begin
                           if sensorA or sensorB
                           then
                              goto Continue_3;
                           end if;
                        end;

                        island.add (contact);
                        contact.m_Flags_is (contact.m_flags or b2_Contact.e_islandFlag);

                        other := ce.other;

                        -- Was the other body already added to this island?
                        --
                        if (other.m_flags and b2_Body.e_islandFlag) /= 0
                        then
                           goto Continue_3;
                        end if;

                        pragma assert (stackCount < stackSize);
                        stack (stackCount) := other;
                        stackCount         := stackCount + 1;

                        other.m_Flags_is (other.m_flags or b2_Body.e_islandFlag);

                        <<Continue_3>>
                        ce := ce.next;
                     end loop;
                  end;


                  -- Search all joints connect to this body.
                  --
                  declare
                     je    : access b2JointEdge := b.getJointList;
                     other :        b2Body_ptr;
                  begin
                     while je /= null
                     loop
                        if je.joint.m_islandFlag = True
                        then
                           goto Continue_4;
                        end if;

                        other := je.other;

                        -- Don't simulate joints connected to diabled bodies.
                        --
                        if other.isEnabled = False
                        then
                           goto Continue_4;
                        end if;

                        island.add (je.joint);
                        je.joint.m_islandFlag_is (True);

                        if (other.m_flags and b2_Body.e_islandFlag) /= 0
                        then
                           goto Continue_4;
                        end if;

                        pragma assert (stackCount < stackSize);
                        stack (stackCount) := other;
                        stackCount         := stackCount + 1;

                        other.m_Flags_is (other.m_flags or b2_Body.e_islandFlag);

                        <<Continue_4>>
                        je := je.next;
                     end loop;
                  end;

                  <<Continue_2>>
               end loop;

               declare
                  profile : aliased b2Profile;
               begin
                  island.solve (profile'Access, step, Self.m_gravity, Self.m_allowSleep);

                  Self.m_profile.solveInit     := Self.m_profile.solveInit     + profile.solveInit;
                  Self.m_profile.solveVelocity := Self.m_profile.solveVelocity + profile.solveVelocity;
                  Self.m_profile.solvePosition := Self.m_profile.solvePosition + profile.solvePosition;

                  -- Post solve cleanup.
                  --
                  for i in 0 .. island.m_bodyCount - 1
                  loop
                     declare
                        b : constant access b2Body := island.m_bodies (i);
                     begin
                        -- Allow static bodies to participate in other islands.
                        --
                        if b.getType = b2_staticBody
                        then
                           b.m_Flags_is (b.m_flags and not b2_Body.e_islandFlag);
                        end if;
                     end;
                  end loop;
               end;

               <<Continue_1>>
               seed := seed.m_next;
            end loop;

            --  free (stack);
         end;

         Island.destruct;
      end;


      declare
         use b2_Timer;
         use type b2_Body.Flag;

         timer : constant b2Timer := to_b2Timer;
         b     : access b2Body  := Self.m_bodyList;
      begin
         -- Synchronize fixtures, check for out of range bodies.
         --
         while b /= null
         loop
            -- If a body was not in an island then it did not move.
            if (b.m_flags and b2_Body.e_islandFlag) = 0
            then
               goto Continue_5;
            end if;

            if b.getType = b2_staticBody
            then
               goto Continue_5;
            end if;

            -- Update fixtures (for broad-phase).
            --
            b.synchronizeFixtures;

            <<Continue_5>>
            b := b.getNext;
         end loop;

         -- Look for new contacts.
         --
         Self.m_contactManager.findNewContacts;
         Self.m_profile.broadphase := timer.getMilliseconds;
      end;
   end Solve;





   --  // Find TOI contacts and solve them.
   --  void b2World::SolveTOI(const b2TimeStep& step)
   --  {
   --    b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener);
   --
   --    if (m_stepComplete)
   --    {
   --       for (b2Body* b = m_bodyList; b; b = b->m_next)
   --       {
   --          b->m_flags &= ~b2Body::e_islandFlag;
   --          b->m_sweep.alpha0 = 0.0f;
   --       }
   --
   --       for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
   --       {
   --          // Invalidate TOI
   --          c->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
   --          c->m_toiCount = 0;
   --          c->m_toi = 1.0f;
   --       }
   --    }
   --
   --    // Find TOI events and solve them.
   --    for (;;)
   --    {
   --       // Find the first TOI.
   --       b2Contact* minContact = nullptr;
   --       float minAlpha = 1.0f;
   --
   --       for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
   --       {
   --          // Is this contact disabled?
   --          if (c->IsEnabled() == false)
   --          {
   --             continue;
   --          }
   --
   --          // Prevent excessive sub-stepping.
   --          if (c->m_toiCount > b2_maxSubSteps)
   --          {
   --             continue;
   --          }
   --
   --          float alpha = 1.0f;
   --          if (c->m_flags & b2Contact::e_toiFlag)
   --          {
   --             // This contact has a valid cached TOI.
   --             alpha = c->m_toi;
   --          }
   --          else
   --          {
   --             b2Fixture* fA = c->GetFixtureA();
   --             b2Fixture* fB = c->GetFixtureB();
   --
   --             // Is there a sensor?
   --             if (fA->IsSensor() || fB->IsSensor())
   --             {
   --                continue;
   --             }
   --
   --             b2Body* bA = fA->GetBody();
   --             b2Body* bB = fB->GetBody();
   --
   --             b2BodyType typeA = bA->m_type;
   --             b2BodyType typeB = bB->m_type;
   --             b2Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);
   --
   --             bool activeA = bA->IsAwake() && typeA != b2_staticBody;
   --             bool activeB = bB->IsAwake() && typeB != b2_staticBody;
   --
   --             // Is at least one body active (awake and dynamic or kinematic)?
   --             if (activeA == false && activeB == false)
   --             {
   --                continue;
   --             }
   --
   --             bool collideA = bA->IsBullet() || typeA != b2_dynamicBody;
   --             bool collideB = bB->IsBullet() || typeB != b2_dynamicBody;
   --
   --             // Are these two non-bullet dynamic bodies?
   --             if (collideA == false && collideB == false)
   --             {
   --                continue;
   --             }
   --
   --             // Compute the TOI for this contact.
   --             // Put the sweeps onto the same time interval.
   --             float alpha0 = bA->m_sweep.alpha0;
   --
   --             if (bA->m_sweep.alpha0 < bB->m_sweep.alpha0)
   --             {
   --                alpha0 = bB->m_sweep.alpha0;
   --                bA->m_sweep.Advance(alpha0);
   --             }
   --             else if (bB->m_sweep.alpha0 < bA->m_sweep.alpha0)
   --             {
   --                alpha0 = bA->m_sweep.alpha0;
   --                bB->m_sweep.Advance(alpha0);
   --             }
   --
   --             b2Assert(alpha0 < 1.0f);
   --
   --             int32 indexA = c->GetChildIndexA();
   --             int32 indexB = c->GetChildIndexB();
   --
   --             // Compute the time of impact in interval [0, minTOI]
   --             b2TOIInput input;
   --             input.proxyA.Set(fA->GetShape(), indexA);
   --             input.proxyB.Set(fB->GetShape(), indexB);
   --             input.sweepA = bA->m_sweep;
   --             input.sweepB = bB->m_sweep;
   --             input.tMax = 1.0f;
   --
   --             b2TOIOutput output;
   --             b2TimeOfImpact(&output, &input);
   --
   --             // Beta is the fraction of the remaining portion of the .
   --             float beta = output.t;
   --             if (output.state == b2TOIOutput::e_touching)
   --             {
   --                alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
   --             }
   --             else
   --             {
   --                alpha = 1.0f;
   --             }
   --
   --             c->m_toi = alpha;
   --             c->m_flags |= b2Contact::e_toiFlag;
   --          }
   --
   --          if (alpha < minAlpha)
   --          {
   --             // This is the minimum TOI found so far.
   --             minContact = c;
   --             minAlpha = alpha;
   --          }
   --       }
   --
   --       if (minContact == nullptr || 1.0f - 10.0f * b2_epsilon < minAlpha)
   --       {
   --          // No more TOI events. Done!
   --          m_stepComplete = true;
   --          break;
   --       }
   --
   --       // Advance the bodies to the TOI.
   --       b2Fixture* fA = minContact->GetFixtureA();
   --       b2Fixture* fB = minContact->GetFixtureB();
   --       b2Body* bA = fA->GetBody();
   --       b2Body* bB = fB->GetBody();
   --
   --       b2Sweep backup1 = bA->m_sweep;
   --       b2Sweep backup2 = bB->m_sweep;
   --
   --       bA->Advance(minAlpha);
   --       bB->Advance(minAlpha);
   --
   --       // The TOI contact likely has some new contact points.
   --       minContact->Update(m_contactManager.m_contactListener);
   --       minContact->m_flags &= ~b2Contact::e_toiFlag;
   --       ++minContact->m_toiCount;
   --
   --       // Is the contact solid?
   --       if (minContact->IsEnabled() == false || minContact->IsTouching() == false)
   --       {
   --          // Restore the sweeps.
   --          minContact->SetEnabled(false);
   --          bA->m_sweep = backup1;
   --          bB->m_sweep = backup2;
   --          bA->SynchronizeTransform();
   --          bB->SynchronizeTransform();
   --          continue;
   --       }
   --
   --       bA->SetAwake(true);
   --       bB->SetAwake(true);
   --
   --       // Build the island
   --       island.Clear();
   --       island.Add(bA);
   --       island.Add(bB);
   --       island.Add(minContact);
   --
   --       bA->m_flags |= b2Body::e_islandFlag;
   --       bB->m_flags |= b2Body::e_islandFlag;
   --       minContact->m_flags |= b2Contact::e_islandFlag;
   --
   --       // Get contacts on bodyA and bodyB.
   --       b2Body* bodies[2] = {bA, bB};
   --       for (int32 i = 0; i < 2; ++i)
   --       {
   --          b2Body* body = bodies[i];
   --          if (body->m_type == b2_dynamicBody)
   --          {
   --             for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
   --             {
   --                if (island.m_bodyCount == island.m_bodyCapacity)
   --                {
   --                   break;
   --                }
   --
   --                if (island.m_contactCount == island.m_contactCapacity)
   --                {
   --                   break;
   --                }
   --
   --                b2Contact* contact = ce->contact;
   --
   --                // Has this contact already been added to the island?
   --                if (contact->m_flags & b2Contact::e_islandFlag)
   --                {
   --                   continue;
   --                }
   --
   --                // Only add static, kinematic, or bullet bodies.
   --                b2Body* other = ce->other;
   --                if (other->m_type == b2_dynamicBody &&
   --                   body->IsBullet() == false && other->IsBullet() == false)
   --                {
   --                   continue;
   --                }
   --
   --                // Skip sensors.
   --                bool sensorA = contact->m_fixtureA->m_isSensor;
   --                bool sensorB = contact->m_fixtureB->m_isSensor;
   --                if (sensorA || sensorB)
   --                {
   --                   continue;
   --                }
   --
   --                // Tentatively advance the body to the TOI.
   --                b2Sweep backup = other->m_sweep;
   --                if ((other->m_flags & b2Body::e_islandFlag) == 0)
   --                {
   --                   other->Advance(minAlpha);
   --                }
   --
   --                // Update the contact points
   --                contact->Update(m_contactManager.m_contactListener);
   --
   --                // Was the contact disabled by the user?
   --                if (contact->IsEnabled() == false)
   --                {
   --                   other->m_sweep = backup;
   --                   other->SynchronizeTransform();
   --                   continue;
   --                }
   --
   --                // Are there contact points?
   --                if (contact->IsTouching() == false)
   --                {
   --                   other->m_sweep = backup;
   --                   other->SynchronizeTransform();
   --                   continue;
   --                }
   --
   --                // Add the contact to the island
   --                contact->m_flags |= b2Contact::e_islandFlag;
   --                island.Add(contact);
   --
   --                // Has the other body already been added to the island?
   --                if (other->m_flags & b2Body::e_islandFlag)
   --                {
   --                   continue;
   --                }
   --
   --                // Add the other body to the island.
   --                other->m_flags |= b2Body::e_islandFlag;
   --
   --                if (other->m_type != b2_staticBody)
   --                {
   --                   other->SetAwake(true);
   --                }
   --
   --                island.Add(other);
   --             }
   --          }
   --       }
   --
   --       b2TimeStep subStep;
   --       subStep.dt = (1.0f - minAlpha) * step.dt;
   --       subStep.inv_dt = 1.0f / subStep.dt;
   --       subStep.dtRatio = 1.0f;
   --       subStep.positionIterations = 20;
   --       subStep.velocityIterations = step.velocityIterations;
   --       subStep.warmStarting = false;
   --       island.SolveTOI(subStep, bA->m_islandIndex, bB->m_islandIndex);
   --
   --       // Reset island flags and synchronize broad-phase proxies.
   --       for (int32 i = 0; i < island.m_bodyCount; ++i)
   --       {
   --          b2Body* body = island.m_bodies[i];
   --          body->m_flags &= ~b2Body::e_islandFlag;
   --
   --          if (body->m_type != b2_dynamicBody)
   --          {
   --             continue;
   --          }
   --
   --          body->SynchronizeFixtures();
   --
   --          // Invalidate all contact TOIs on this displaced body.
   --          for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
   --          {
   --             ce->contact->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
   --          }
   --       }
   --
   --       // Commit fixture proxy movements to the broad-phase so that new contacts are created.
   --       // Also, some contacts can be destroyed.
   --       m_contactManager.FindNewContacts();
   --
   --       if (m_subStepping)
   --       {
   --          m_stepComplete = false;
   --          break;
   --       }
   --    }
   --  }
   --

   procedure SolveTOI (Self : in out b2World;   step : in b2TimeStep)
   is
      use b2_Island,
          b2_Common;

      use type b2_Body.Flag,
               b2_Contact.Flag;

      island : b2Island := to_b2Island (b2_maxTOIContacts * 2,
                                        b2_maxTOIContacts,
                                        0,
                                        Self.m_contactManager.m_contactListener);
      b :        b2Body_ptr;
      c : access b2Contact;
   begin
      if Self.m_stepComplete
      then
         b := Self.m_bodyList;

         while b /= null
         loop
            b.m_Flags_is (b.m_flags and not b2_Body.e_islandFlag);
            b.m_sweep.alpha0 := 0.0;

            b := b.m_next;
         end loop;


         c := Self.m_contactManager.m_contactList;

         while c /= null
         loop
            -- Invalidate TOI.
            --
            c.m_Flags_is (c.m_flags and not (b2_Contact.e_toiFlag or b2_Contact.e_islandFlag));

            c.m_toiCount_is (0);
            c.m_toi_is      (1.0);

            c := c.m_next;
         end loop;
      end if;


      -- Find TOI events and solve them.
      --
      loop
         -- Find the first TOI.
         --
         declare
            minContact : access b2Contact;
            minAlpha   :        Real     := 1.0;
            alpha      :        Real;
         begin
            c := Self.m_contactManager.m_contactList;

            while c /= null
            loop
               -- Is this contact disabled?
               --
               if c.isEnabled = False
               then
                  goto Continue_2;
               end if;

               -- Prevent excessive sub-stepping.
               --
               if c.m_toiCount > b2_maxSubSteps
               then
                  goto Continue_2;
               end if;

               alpha := 1.0;


               if (c.m_flags and b2_Contact.e_toiFlag) /= 0
               then
                  -- This contact has a valid cached TOI.
                  --
                  alpha := c.m_toi;

               else
                  declare
                     fA : constant access b2Fixture := c.getFixtureA;
                     fB : constant access b2Fixture := c.getFixtureB;
                  begin
                     -- Is there a sensor?
                     --
                     if fA.isSensor or fB.isSensor
                     then
                        goto Continue_2;
                     end if;

                     declare
                        bA : constant access b2Body := fA.getBody;
                        bB : constant access b2Body := fB.getBody;

                        typeA : constant b2BodyType := bA.getType;
                        typeB : constant b2BodyType := bB.getType;

                        pragma assert (   typeA = b2_dynamicBody
                                       or typeB = b2_dynamicBody);

                        activeA : constant Boolean := bA.isAwake and typeA /= b2_staticBody;
                        activeB : constant Boolean := bB.isAwake and typeB /= b2_staticBody;

                        collideA : Boolean;
                        collideB : Boolean;

                        alpha0   : Real;
                     begin
                        -- Is at least one body active (awake and dynamic or kinematic)?
                        --
                        if    activeA = False
                          and activeB = False
                        then
                           goto Continue_2;
                        end if;

                        collideA := bA.isBullet or typeA /= b2_dynamicBody;
                        collideB := bB.isBullet or typeB /= b2_dynamicBody;

                        -- Are these two non-bullet dynamic bodies?
                        --
                        if    collideA = False
                          and collideB = False
                        then
                           goto Continue_2;
                        end if;

                        -- Compute the TOI for this contact.
                        -- Put the sweeps onto the same time interval.
                        --
                        alpha0 := bA.m_sweep.alpha0;

                        if bA.m_sweep.alpha0 < bB.m_sweep.alpha0
                        then
                           alpha0 := bB.m_sweep.alpha0;
                           bA.m_sweep.advance (alpha0);

                        elsif bB.m_sweep.alpha0 < bA.m_sweep.alpha0
                        then
                           alpha0 := bA.m_sweep.alpha0;
                           bB.m_sweep.advance (alpha0);
                        end if;


                        pragma assert (alpha0 < 1.0);

                        declare
                           use b2_Time_of_impact;

                           indexA : constant Natural := c.getChildIndexA;
                           indexB : constant Natural := c.getChildIndexB;

                           input  : aliased b2TOIInput;
                           output : aliased b2TOIOutput;

                           beta   : Real;
                        begin
                           -- Compute the time of impact in interval [0, minTOI].
                           --
                           input.proxyA.set (fA.getShape, indexA);
                           input.proxyB.set (fB.getShape, indexB);

                           input.sweepA := bA.m_sweep.all;
                           input.sweepB := bB.m_sweep.all;
                           input.tMax   := 1.0;


                           b2TimeOfImpact (output,
                                           input);

                           -- Beta is the fraction of the remaining portion of the .
                           --
                           beta := output.t;

                           if output.state = e_touching
                           then
                              alpha := Real'min (alpha0 + (1.0 - alpha0) * beta,
                                                 1.0);
                           else
                              alpha := 1.0;
                           end if;

                           c.m_toi_is (alpha);
                           c.m_Flags_is (c.m_flags or b2_Contact.e_toiFlag);
                        end;
                     end;
                  end;
               end if;

               if alpha < minAlpha
               then
                  -- This is the minimum TOI found so far.
                  --
                  minContact := c;
                  minAlpha   := alpha;
               end if;

               <<Continue_2>>
               c := c.m_next;
            end loop;


            if   minContact               =  null
              or 1.0 - 10.0 * b2_epsilon  <  minAlpha
            then
               -- No more TOI events. Done!
               --
               Self.m_stepComplete := True;
               exit;
            end if;


            -- Advance the bodies to the TOI.
            --
            declare
               fA : constant access b2Fixture := minContact.getFixtureA;
               fB : constant access b2Fixture := minContact.getFixtureB;

               bA : constant b2Body_ptr      := fA.getBody;
               bB : constant b2Body_ptr      := fB.getBody;

               backup1 : constant b2Sweep     := bA.m_sweep.all;
               backup2 : constant b2Sweep     := bB.m_sweep.all;
            begin
               bA.advance (minAlpha);
               bB.advance (minAlpha);

               -- The TOI contact likely has some new contact points.
               --
               minContact.update        (Self.m_contactManager.m_contactListener);
               minContact.m_Flags_is    (minContact.m_flags and not b2_Contact.e_toiFlag);
               minContact.m_toiCount_is (minContact.m_toiCount + 1);

               -- Is the contact solid?
               --
               if   minContact.isEnabled  = False
                 or minContact.isTouching = False
               then
                  -- Restore the sweeps.
                  --
                  minContact.setEnabled (False);

                  bA.m_sweep.all := backup1;
                  bB.m_sweep.all := backup2;

                  bA.synchronizeTransform;
                  bB.synchronizeTransform;

                  goto Continue_1;
               end if;

               bA.setAwake (True);
               bB.setAwake (True);

               -- Build the island.
               --
               island.clear;
               island.add (bA);
               island.add (bB);
               island.add (minContact);

               bA.m_Flags_is (bA.m_flags or b2_Body.e_islandFlag);
               bB.m_Flags_is (bB.m_flags or b2_Body.e_islandFlag);

               minContact.m_Flags_is (minContact.m_flags or b2_Contact.e_islandFlag);


               -- Get contacts on bodyA and bodyB.
               --
               declare
                  bodies   : constant array (0 .. 1) of b2Body_ptr := [bA, bB];
                  the_body : b2Body_ptr;
               begin
                  for i in 0 .. 1
                  loop
                     the_body := bodies (i);

                     if the_body.getType = b2_dynamicBody
                     then
                        declare
                           ce      : access b2ContactEdge := the_body.m_contactList;
                           contact : access b2Contact;
                           other   :        b2Body_ptr;
                           backup  :        b2Sweep;
                        begin
                           while ce /= null
                           loop
                              if island.m_bodyCount = island.m_bodyCapacity
                              then
                                 exit;
                              end if;

                              if island.m_contactCount = island.m_contactCapacity
                              then
                                 exit;
                              end if;

                              contact := ce.contact;

                              -- Has this contact already been added to the island?
                              --
                              if (contact.m_flags and b2_Contact.e_islandFlag) /= 0
                              then
                                 goto Continue_3;
                              end if;

                              -- Only add static, kinematic, or bullet bodies.
                              --
                              other := ce.other;

                              if    other.getType     = b2_dynamicBody
                                and the_body.isBullet = False
                                and other.isBullet    = False
                              then
                                 goto Continue_3;
                              end if;

                              -- Skip sensors.
                              --
                              declare
                                 sensorA : constant Boolean := contact.getFixtureA.isSensor;
                                 sensorB : constant Boolean := contact.getFixtureB.isSensor;
                              begin
                                 if sensorA or sensorB
                                 then
                                    goto Continue_3;
                                 end if;
                              end;

                              -- Tentatively advance the body to the TOI.
                              --
                              backup := other.m_sweep.all;

                              if (other.m_flags and b2_Body.e_islandFlag) = 0
                              then
                                 other.advance (minAlpha);
                              end if;

                              -- Update the contact points
                              --
                              contact.update (Self.m_contactManager.m_contactListener);

                              -- Was the contact disabled by the user?
                              --
                              if contact.isEnabled = False
                              then
                                 other.m_sweep.all := backup;
                                 other.synchronizeTransform;

                                 goto Continue_3;
                              end if;

                              -- Are there contact points?
                              --
                              if contact.isTouching = False
                              then
                                 other.m_sweep.all := backup;
                                 other.synchronizeTransform;

                                 goto Continue_3;
                              end if;

                              -- Add the contact to the island.
                              --
                              contact.m_Flags_is (contact.m_flags or b2_Contact.e_islandFlag);
                              island.add (contact);

                              -- Has the other body already been added to the island?
                              --
                              if (other.m_flags and b2_Body.e_islandFlag) /= 0
                              then
                                 goto Continue_3;
                              end if;

                              -- Add the other body to the island.
                              --
                              other.m_flags_is (other.m_flags or b2_Body.e_islandFlag);

                              if other.getType /= b2_staticBody
                              then
                                 other.setAwake (True);
                              end if;

                              island.add (other);

                              <<Continue_3>>
                              ce := ce.next;
                           end loop;
                        end;
                     end if;

                  end loop;
               end;


               declare
                  subStep : b2TimeStep;
               begin
                  subStep.dt                 := (1.0 - minAlpha) * step.dt;
                  subStep.inv_dt             := 1.0 / subStep.dt;
                  subStep.dtRatio            := 1.0;
                  subStep.positionIterations := 20;
                  subStep.velocityIterations := step.velocityIterations;
                  subStep.warmStarting       := False;

                  island.solveTOI (subStep, bA.m_islandIndex,
                                            bB.m_islandIndex);
               end;


               -- Reset island flags and synchronize broad-phase proxies.
               --
               for i in 0 .. island.m_bodyCount - 1
               loop
                  --  put_Line (island.m_bodyCount'Image);

                  declare
                     the_Body : constant access b2Body       := island.m_bodies (i);
                     ce       : access          b2ContactEdge;
                  begin
                     the_body.m_Flags_is (the_body.m_flags and not b2_Body.e_islandFlag);

                     if the_body.getType /= b2_dynamicBody
                     then
                        goto Continue_4;
                     end if;

                     the_body.synchronizeFixtures;

                     -- Invalidate all contact TOIs on this displaced body.
                     --
                     ce := the_body.m_contactList;

                     while ce /= null
                     loop
                        ce.contact.m_Flags_is (ce.contact.m_flags and not (b2_Contact.e_toiFlag or b2_Contact.e_islandFlag));
                        ce := ce.next;
                     end loop;
                  end;

                  <<Continue_4>>
               end loop;


               -- Commit fixture proxy movements to the broad-phase so that new contacts are created.
               -- Also, some contacts can be destroyed.
               --
               Self.m_contactManager.findNewContacts;

               if Self.m_subStepping
               then
                  Self.m_stepComplete := False;
                  exit;
               end if;
            end;

         end;

         <<Continue_1>>
      end loop;


      Island.destruct;
   end SolveTOI;







   --  void b2World::Step(float dt, int32 velocityIterations, int32 positionIterations)
   --  {
   --    b2Timer stepTimer;
   --
   --    // If new fixtures were added, we need to find the new contacts.
   --    if (m_newContacts)
   --    {
   --       m_contactManager.FindNewContacts();
   --       m_newContacts = false;
   --    }
   --
   --    m_locked = true;
   --
   --    b2TimeStep step;
   --    step.dt = dt;
   --    step.velocityIterations = velocityIterations;
   --    step.positionIterations = positionIterations;
   --    if (dt > 0.0f)
   --    {
   --       step.inv_dt = 1.0f / dt;
   --    }
   --    else
   --    {
   --       step.inv_dt = 0.0f;
   --    }
   --
   --    step.dtRatio = m_inv_dt0 * dt;
   --
   --    step.warmStarting = m_warmStarting;
   --
   --    // Update contacts. This is where some contacts are destroyed.
   --    {
   --       b2Timer timer;
   --       m_contactManager.Collide();
   --       m_profile.collide = timer.GetMilliseconds();
   --    }
   --
   --    // Integrate velocities, solve velocity constraints, and integrate positions.
   --    if (m_stepComplete && step.dt > 0.0f)
   --    {
   --       b2Timer timer;
   --       Solve(step);
   --       m_profile.solve = timer.GetMilliseconds();
   --    }
   --
   --    // Handle TOI events.
   --    if (m_continuousPhysics && step.dt > 0.0f)
   --    {
   --       b2Timer timer;
   --       SolveTOI(step);
   --       m_profile.solveTOI = timer.GetMilliseconds();
   --    }
   --
   --    if (step.dt > 0.0f)
   --    {
   --       m_inv_dt0 = step.inv_dt;
   --    }
   --
   --    if (m_clearForces)
   --    {
   --       ClearForces();
   --    }
   --
   --    m_locked = false;
   --
   --    m_profile.step = stepTimer.GetMilliseconds();
   --  }
   --

   procedure step (Self : in out b2World;   timeStep           : in Real;
                                            velocityIterations : in Positive;
                                            positionIterations : in Positive)
   is
      use b2_Timer;

      dt        : Real renames timeStep;
      stepTimer : constant b2Timer := to_b2Timer;

   begin
      -- If new fixtures were added, we need to find the new contacts.
      --
      if Self.m_newContacts
      then
         Self.m_contactManager.findNewContacts;
         Self.m_newContacts := False;
      end if;

      Self.m_locked := True;

      declare
         step : b2TimeStep;
      begin
         step.dt                 := dt;
         step.velocityIterations := velocityIterations;
         step.positionIterations := positionIterations;

         if dt > 0.0
         then
            step.inv_dt := 1.0 / dt;
         else
            step.inv_dt := 0.0;
         end if;

         step.dtRatio      := Self.m_inv_dt0 * dt;
         step.warmStarting := Self.m_warmStarting;

         -- Update contacts. This is where some contacts are destroyed.
         --
         declare
            timer : constant b2Timer := to_b2Timer;
         begin
            Self.m_contactManager.collide;
            Self.m_profile.collide := timer.getMilliseconds;
         end;

         -- Integrate velocities, solve velocity constraints, and integrate positions.
         --
         if Self.m_stepComplete and step.dt > 0.0
         then
            declare
               timer : constant b2Timer := to_b2Timer;
            begin
               Self.solve (step);
               Self.m_profile.solve := timer.getMilliseconds;
            end;
         end if;

         -- Handle TOI events.
         --
         if Self.m_continuousPhysics and step.dt > 0.0
         then
            declare
               timer : constant b2Timer := to_b2Timer;
            begin
               Self.solveTOI (step);
               Self.m_profile.solveTOI := timer.getMilliseconds;
            end;
         end if;

         if step.dt > 0.0
         then
            Self.m_inv_dt0 := step.inv_dt;
         end if;
      end;


      if Self.m_clearForces
      then
         Self.clearForces;
      end if;

      Self.m_locked       := False;
      Self.m_profile.step := stepTimer.getMilliseconds;
   end step;




   --  void b2World::ClearForces()
   --  {
   --    for (b2Body* body = m_bodyList; body; body = body->GetNext())
   --    {
   --       body->m_force.SetZero();
   --       body->m_torque = 0.0f;
   --    }
   --  }
   --

   procedure clearForces (Self : in out b2World)
   is
      the_body : access b2Body := Self.m_bodyList;
   begin
      while the_body /= null
      loop
         the_Body.m_Force_is ((0.0, 0.0));
         the_Body.m_Torque_is (0.0);

         the_Body := the_Body.getNext;
      end loop;
   end clearForces;




   --  struct b2WorldQueryWrapper
   --  {
   --    bool QueryCallback(int32 proxyId)
   --    {
   --       b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
   --       return callback->ReportFixture(proxy->fixture);
   --    }
   --
   --    const b2BroadPhase* broadPhase;
   --    b2QueryCallback* callback;
   --  };
   --

   type b2WorldQueryWrapper is
      record
         broadPhase : access constant b2BroadPhase'Class;
         callback   : access          b2QueryCallback'Class;
      end record;


   function queryCallback (Self : in out b2WorldQueryWrapper;   proxyID : in Natural) return Boolean
   is
      proxy : b2fixtureProxy
        with  import,
                            Address => Self.broadPhase.getUserData  (proxyId);
   begin
      return Self.callback.reportFixture (proxy.fixture);
   end queryCallback;






   --  void b2World::QueryAABB (b2QueryCallback* callback, const b2AABB& aabb) const
   --  {
   --    b2WorldQueryWrapper wrapper;
   --    wrapper.broadPhase = &m_contactManager.m_broadPhase;
   --    wrapper.callback = callback;
   --    m_contactManager.m_broadPhase.Query(&wrapper, aabb);
   --  }
   --

   procedure queryAABB (Self : access constant b2World;   callback : access b2QueryCallback;
                                                          aabb     : in     b2AABB)
   is
      procedure my_Query is new b2_broad_Phase.query (callback_t    => b2WorldQueryWrapper,
                                                      queryCallback => queryCallback);

      wrapper : aliased b2WorldQueryWrapper;
   begin
      wrapper.broadPhase := Self.m_contactManager.m_broadPhase'Access;
      wrapper.callback   := callback;

      my_Query (Self.m_contactManager.m_broadPhase, wrapper'Access, aabb);
   end QueryAABB;






   --  struct b2WorldRayCastWrapper
   --  {
   --    float RayCastCallback(const b2RayCastInput& input, int32 proxyId)
   --    {
   --       void* userData = broadPhase->GetUserData(proxyId);
   --       b2FixtureProxy* proxy = (b2FixtureProxy*)userData;
   --       b2Fixture* fixture = proxy->fixture;
   --       int32 index = proxy->childIndex;
   --       b2RayCastOutput output;
   --       bool hit = fixture->RayCast(&output, input, index);
   --
   --       if (hit)
   --       {
   --          float fraction = output.fraction;
   --          b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
   --          return callback->ReportFixture(fixture, point, output.normal, fraction);
   --       }
   --
   --       return input.maxFraction;
   --    }
   --
   --    const b2BroadPhase* broadPhase;
   --    b2RayCastCallback* callback;
   --  };
   --

   type b2WorldRayCastWrapper is
      record
         broadPhase : access constant b2BroadPhase'Class;
         callback   : access          b2RayCastCallback'Class;
      end record;


   function RayCastCallback (Self : in out b2WorldRayCastWrapper;   input   : in b2RayCastInput;
                                                                    proxyId : in Natural) return Real
   is
      userData : constant void_ptr := Self.broadPhase.getUserData (proxyId);
      proxy    : b2FixtureProxy
        with  import, Address => userData;

      fixture : constant access b2Fixture      := proxy.fixture;
      index   : constant        Natural        := proxy.childIndex;
      output  :                 b2RayCastOutput;
      hit     : constant        Boolean        := fixture.raycast (output, input, index);

   begin
      if hit
      then
         declare
            fraction : constant Real   := output.fraction;
            point    : constant b2Vec2 :=   (1.0 - fraction) * input.p1
                                          + fraction         * input.p2;
         begin
            return Self.callback.reportFixture (fixture, point, output.normal, fraction);
         end;
      end if;

      return input.maxFraction;
   end RayCastCallback;





   --  void b2World::RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const
   --  {
   --    b2WorldRayCastWrapper wrapper;
   --    wrapper.broadPhase = &m_contactManager.m_broadPhase;
   --    wrapper.callback = callback;
   --    b2RayCastInput input;
   --    input.maxFraction = 1.0f;
   --    input.p1 = point1;
   --    input.p2 = point2;
   --    m_contactManager.m_broadPhase.RayCast(&wrapper, input);
   --  }
   --

   procedure raycast (Self : access constant b2World;   callback : access b2RayCastCallback;
                                                        point1,
                                                        point2   : in     b2Vec2)
   is
      procedure my_raycast is new b2_broad_Phase.raycast (callback_t      => b2WorldRayCastWrapper,
                                                          raycastCallback => RayCastCallback);


      wrapper : b2WorldRayCastWrapper;
      input   : b2RayCastInput;
   begin
      wrapper.broadPhase := Self.m_contactManager.m_broadPhase'Access;
      wrapper.callback   := callback;
      input.maxFraction  := 1.0;
      input.p1           := point1;
      input.p2           := point2;

      my_raycast (Self.m_contactManager.m_broadPhase, wrapper, input);
   end raycast;





   --  void b2World::DrawShape(b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
   --  {
   --    switch (fixture->GetType())
   --    {
   --    case b2Shape::e_circle:
   --       {
   --          b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();
   --
   --          b2Vec2 center = b2Mul(xf, circle->m_p);
   --          float radius = circle->m_radius;
   --          b2Vec2 axis = b2Mul(xf.q, b2Vec2(1.0f, 0.0f));
   --
   --          m_debugDraw->DrawSolidCircle(center, radius, axis, color);
   --       }
   --       break;
   --
   --    case b2Shape::e_edge:
   --       {
   --          b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();
   --          b2Vec2 v1 = b2Mul(xf, edge->m_vertex1);
   --          b2Vec2 v2 = b2Mul(xf, edge->m_vertex2);
   --          m_debugDraw->DrawSegment(v1, v2, color);
   --
   --          if (edge->m_oneSided == false)
   --          {
   --             m_debugDraw->DrawPoint(v1, 4.0f, color);
   --             m_debugDraw->DrawPoint(v2, 4.0f, color);
   --          }
   --       }
   --       break;
   --
   --    case b2Shape::e_chain:
   --       {
   --          b2ChainShape* chain = (b2ChainShape*)fixture->GetShape();
   --          int32 count = chain->m_count;
   --          const b2Vec2* vertices = chain->m_vertices;
   --
   --          b2Vec2 v1 = b2Mul(xf, vertices[0]);
   --          for (int32 i = 1; i < count; ++i)
   --          {
   --             b2Vec2 v2 = b2Mul(xf, vertices[i]);
   --             m_debugDraw->DrawSegment(v1, v2, color);
   --             v1 = v2;
   --          }
   --       }
   --       break;
   --
   --    case b2Shape::e_polygon:
   --       {
   --          b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
   --          int32 vertexCount = poly->m_count;
   --          b2Assert(vertexCount <= b2_maxPolygonVertices);
   --          b2Vec2 vertices[b2_maxPolygonVertices];
   --
   --          for (int32 i = 0; i < vertexCount; ++i)
   --          {
   --             vertices[i] = b2Mul(xf, poly->m_vertices[i]);
   --          }
   --
   --          m_debugDraw->DrawSolidPolygon(vertices, vertexCount, color);
   --       }
   --       break;
   --
   --    default:
   --    break;
   --    }
   --  }
   --

   procedure drawShape (Self : in out b2World;   shape : access b2Fixture;
                                                 xf    : in     b2Transform;
                                                 color : in     b2Color)
   is
   begin
      raise Program_Error with "TODO";
   end drawShape;





   --  void b2World::DebugDraw()
   --  {
   --    if (m_debugDraw == nullptr)
   --    {
   --       return;
   --    }
   --
   --    uint32 flags = m_debugDraw->GetFlags();
   --
   --    if (flags & b2Draw::e_shapeBit)
   --    {
   --       for (b2Body* b = m_bodyList; b; b = b->GetNext())
   --       {
   --          const b2Transform& xf = b->GetTransform();
   --          for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext())
   --          {
   --             if (b->GetType() == b2_dynamicBody && b->m_mass == 0.0f)
   --             {
   --                // Bad body
   --                DrawShape(f, xf, b2Color(1.0f, 0.0f, 0.0f));
   --             }
   --             else if (b->IsEnabled() == false)
   --             {
   --                DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
   --             }
   --             else if (b->GetType() == b2_staticBody)
   --             {
   --                DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
   --             }
   --             else if (b->GetType() == b2_kinematicBody)
   --             {
   --                DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
   --             }
   --             else if (b->IsAwake() == false)
   --             {
   --                DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
   --             }
   --             else
   --             {
   --                DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
   --             }
   --          }
   --       }
   --    }
   --
   --    if (flags & b2Draw::e_jointBit)
   --    {
   --       for (b2Joint* j = m_jointList; j; j = j->GetNext())
   --       {
   --          j->Draw(m_debugDraw);
   --       }
   --    }
   --
   --    if (flags & b2Draw::e_pairBit)
   --    {
   --       b2Color color(0.3f, 0.9f, 0.9f);
   --       for (b2Contact* c = m_contactManager.m_contactList; c; c = c->GetNext())
   --       {
   --          b2Fixture* fixtureA = c->GetFixtureA();
   --          b2Fixture* fixtureB = c->GetFixtureB();
   --          int32 indexA = c->GetChildIndexA();
   --          int32 indexB = c->GetChildIndexB();
   --          b2Vec2 cA = fixtureA->GetAABB(indexA).GetCenter();
   --          b2Vec2 cB = fixtureB->GetAABB(indexB).GetCenter();
   --
   --          m_debugDraw->DrawSegment(cA, cB, color);
   --       }
   --    }
   --
   --    if (flags & b2Draw::e_aabbBit)
   --    {
   --       b2Color color(0.9f, 0.3f, 0.9f);
   --       b2BroadPhase* bp = &m_contactManager.m_broadPhase;
   --
   --       for (b2Body* b = m_bodyList; b; b = b->GetNext())
   --       {
   --          if (b->IsEnabled() == false)
   --          {
   --             continue;
   --          }
   --
   --          for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext())
   --          {
   --             for (int32 i = 0; i < f->m_proxyCount; ++i)
   --             {
   --                b2FixtureProxy* proxy = f->m_proxies + i;
   --                b2AABB aabb = bp->GetFatAABB(proxy->proxyId);
   --                b2Vec2 vs[4];
   --                vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
   --                vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
   --                vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
   --                vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
   --
   --                m_debugDraw->DrawPolygon(vs, 4, color);
   --             }
   --          }
   --       }
   --    }
   --
   --    if (flags & b2Draw::e_centerOfMassBit)
   --    {
   --       for (b2Body* b = m_bodyList; b; b = b->GetNext())
   --       {
   --          b2Transform xf = b->GetTransform();
   --          xf.p = b->GetWorldCenter();
   --          m_debugDraw->DrawTransform(xf);
   --       }
   --    }
   --  }
   --

   procedure DebugDraw (Self : in out b2World)
   is
   begin
      raise Program_Error with "TODO";
   end DebugDraw;





   --  int32 b2World::GetProxyCount() const
   --  {
   --    return m_contactManager.m_broadPhase.GetProxyCount();
   --  }
   --

   function getProxyCount (Self : in b2World) return Natural
   is
   begin
      return Self.m_contactManager.m_broadPhase.getProxyCount;
   end getProxyCount;




   --  int32 b2World::GetTreeHeight() const
   --  {
   --    return m_contactManager.m_broadPhase.GetTreeHeight();
   --  }
   --

   function GetTreeHeight (Self : in b2World) return Positive
   is
   begin
      return Self.m_contactManager.m_broadPhase.getTreeHeight;
   end GetTreeHeight;




   --  int32 b2World::GetTreeBalance() const
   --  {
   --    return m_contactManager.m_broadPhase.GetTreeBalance();
   --  }
   --

   function GetTreeBalance (Self : in b2World) return Natural
   is
   begin
      return Self.m_contactManager.m_broadPhase.getTreeBalance;
   end GetTreeBalance;



   --  float b2World::GetTreeQuality() const
   --  {
   --    return m_contactManager.m_broadPhase.GetTreeQuality();
   --  }
   --

   function GetTreeQuality (Self : in b2World) return Real
   is
   begin
      return Self.m_contactManager.m_broadPhase.getTreeQuality;
   end GetTreeQuality;




   --  void b2World::ShiftOrigin(const b2Vec2& newOrigin)
   --  {
   --    b2Assert(m_locked == false);
   --    if (m_locked)
   --    {
   --       return;
   --    }
   --
   --    for (b2Body* b = m_bodyList; b; b = b->m_next)
   --    {
   --       b->m_xf.p -= newOrigin;
   --       b->m_sweep.c0 -= newOrigin;
   --       b->m_sweep.c -= newOrigin;
   --    }
   --
   --    for (b2Joint* j = m_jointList; j; j = j->m_next)
   --    {
   --       j->ShiftOrigin(newOrigin);
   --    }
   --
   --    m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
   --  }
   --

   procedure shiftOrigin (Self : in out b2World;   newOrigin : in b2Vec2)
   is
      pragma assert (Self.m_locked = False);

   begin
     if Self.m_locked
     then
        return;
     end if;

      declare
         b : access b2Body  := Self.m_bodyList;
         j : access b2Joint := Self.m_jointList;
      begin
         while b /= null
         loop
            b.m_xf.p     := b.m_xf.p     - newOrigin;
            b.m_sweep.c0 := b.m_sweep.c0 - newOrigin;
            b.m_sweep.c  := b.m_sweep.c  - newOrigin;

            b := b.m_next;
         end loop;


         while j /= null
         loop
            j.shiftOrigin (newOrigin);

            j := j.m_next;
         end loop;
      end;

      Self.m_contactManager.m_broadPhase.shiftOrigin (newOrigin);
   end shiftOrigin;



   --  void b2World::Dump()
   --  {
   --    if (m_locked)
   --    {
   --       return;
   --    }
   --
   --    b2OpenDump("box2d_dump.inl");
   --
   --    b2Dump("b2Vec2 g(%.9g, %.9g);\n", m_gravity.x, m_gravity.y);
   --    b2Dump("m_world->SetGravity(g);\n");
   --
   --    b2Dump("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
   --    b2Dump("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);
   --
   --    int32 i = 0;
   --    for (b2Body* b = m_bodyList; b; b = b->m_next)
   --    {
   --       b->m_islandIndex = i;
   --       b->Dump();
   --       ++i;
   --    }
   --
   --    i = 0;
   --    for (b2Joint* j = m_jointList; j; j = j->m_next)
   --    {
   --       j->m_index = i;
   --       ++i;
   --    }
   --
   --    // First pass on joints, skip gear joints.
   --    for (b2Joint* j = m_jointList; j; j = j->m_next)
   --    {
   --       if (j->m_type == e_gearJoint)
   --       {
   --          continue;
   --       }
   --
   --       b2Dump("{\n");
   --       j->Dump();
   --       b2Dump("}\n");
   --    }
   --
   --    // Second pass on joints, only gear joints.
   --    for (b2Joint* j = m_jointList; j; j = j->m_next)
   --    {
   --       if (j->m_type != e_gearJoint)
   --       {
   --          continue;
   --       }
   --
   --       b2Dump("{\n");
   --       j->Dump();
   --       b2Dump("}\n");
   --    }
   --
   --    b2Dump("b2Free(joints);\n");
   --    b2Dump("b2Free(bodies);\n");
   --    b2Dump("joints = nullptr;\n");
   --    b2Dump("bodies = nullptr;\n");
   --
   --    b2CloseDump();
   --  }

   procedure dump (Self : in out b2World)
   is
   begin
      raise Program_Error with "TODO";
   end dump;





   ------------------------------
   -- Protected b2World Functions
   --

   function m_contactManager (Self : access b2World) return access b2ContactManager
   is
   begin
      return Self.m_contactManager'Access;
   end m_contactManager;



   procedure m_newContacts_is (Self : in out b2World;   Now : in Boolean)
   is
   begin
      Self.m_newContacts := Now;
   end m_newContacts_is;


end box2d.b2_World;
