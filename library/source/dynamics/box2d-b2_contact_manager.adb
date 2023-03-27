with
     box2d.b2_Body,
     box2d.b2_contact_Manager,
     box2d.b2_Fixture,

     Interfaces;


package body box2d.b2_contact_Manager
is
   use b2_Fixture,
       b2_Body;



   type b2DefaultListener is new b2ContactListener with null record;

   --  b2ContactFilter b2_defaultFilter;
   --  b2ContactListener b2_defaultListener;
   --

   b2_defaultFilter   : aliased b2ContactFilter;
   b2_defaultListener : aliased b2DefaultListener;





   --  b2ContactManager::b2ContactManager()
   --  {
   --    m_contactList = nullptr;
   --    m_contactCount = 0;
   --    m_contactFilter = &b2_defaultFilter;
   --    m_contactListener = &b2_defaultListener;
   --    m_allocator = nullptr;
   --  }
   --

   function to_b2ContactManager return b2ContactManager
   is
      Self : b2ContactManager;
   begin
      Self.m_contactList     := null;
      Self.m_contactCount    := 0;
      Self.m_contactFilter   := b2_defaultFilter  'Access;
      Self.m_contactListener := b2_defaultListener'Access;

      Self.m_broadPhase := to_b2BroadPhase;

      return Self;
   end to_b2ContactManager;




   procedure destruct (Self : in out b2ContactManager)
   is
   begin
      Self.m_broadPhase.destruct;
   end destruct;



   --  void b2ContactManager::Destroy(b2Contact* c)
   --  {
   --    b2Fixture* fixtureA = c->GetFixtureA();
   --    b2Fixture* fixtureB = c->GetFixtureB();
   --    b2Body* bodyA = fixtureA->GetBody();
   --    b2Body* bodyB = fixtureB->GetBody();
   --
   --    if (m_contactListener && c->IsTouching())
   --    {
   --       m_contactListener->EndContact(c);
   --    }
   --
   --    // Remove from the world.
   --    if (c->m_prev)
   --    {
   --       c->m_prev->m_next = c->m_next;
   --    }
   --
   --    if (c->m_next)
   --    {
   --       c->m_next->m_prev = c->m_prev;
   --    }
   --
   --    if (c == m_contactList)
   --    {
   --       m_contactList = c->m_next;
   --    }
   --
   --    // Remove from body 1
   --    if (c->m_nodeA.prev)
   --    {
   --       c->m_nodeA.prev->next = c->m_nodeA.next;
   --    }
   --
   --    if (c->m_nodeA.next)
   --    {
   --       c->m_nodeA.next->prev = c->m_nodeA.prev;
   --    }
   --
   --    if (&c->m_nodeA == bodyA->m_contactList)
   --    {
   --       bodyA->m_contactList = c->m_nodeA.next;
   --    }
   --
   --    // Remove from body 2
   --    if (c->m_nodeB.prev)
   --    {
   --       c->m_nodeB.prev->next = c->m_nodeB.next;
   --    }
   --
   --    if (c->m_nodeB.next)
   --    {
   --       c->m_nodeB.next->prev = c->m_nodeB.prev;
   --    }
   --
   --    if (&c->m_nodeB == bodyB->m_contactList)
   --    {
   --       bodyB->m_contactList = c->m_nodeB.next;
   --    }
   --
   --    // Call the factory.
   --    b2Contact::Destroy(c, m_allocator);
   --    --m_contactCount;
   --  }

   procedure destroy (Self : in out b2ContactManager;   c : access b2Contact)
   is
      fixtureA : constant access b2Fixture := c.getFixtureA;
      fixtureB : constant access b2Fixture := c.getFixtureB;

      bodyA    : constant access b2Body := fixtureA.getBody;
      bodyB    : constant access b2Body := fixtureB.getBody;

   begin
      if    Self.m_contactListener /= null
        and c.isTouching
      then
         Self.m_contactListener.endContact (c.all);
      end if;

      -- Remove from the world.
      --
      if c.m_prev /= null
      then
         c.m_prev.m_Next_is (c.m_next);
      end if;

      if c.m_next /= null
      then
         c.m_next.m_Prev_is (c.m_prev);
      end if;


      if c = Self.m_contactList
      then
         Self.m_contactList := c.m_next;
      end if;


      -- Remove from body 1
      --
      if c.m_nodeA.prev /= null
      then
         c.m_nodeA.prev.next := c.m_nodeA.next;
      end if;

      if c.m_nodeA.next /= null
      then
         c.m_nodeA.next.prev := c.m_nodeA.prev;
      end if;

      if c.m_nodeA = bodyA.getContactList
      then
         bodyA.m_contactList_is (c.m_nodeA.next);
      end if;


      -- Remove from body 2
      --
      if c.m_nodeB.prev /= null
      then
         c.m_nodeB.prev.next := c.m_nodeB.next;
      end if;

      if c.m_nodeB.next /= null
      then
         c.m_nodeB.next.prev := c.m_nodeB.prev;
      end if;

      if c.m_nodeB = bodyB.getContactList
      then
         bodyB.m_contactList_is (c.m_nodeB.next);
      end if;


      -- Call the factory.
      --
      b2_Contact.destroy (c);

      Self.m_contactCount := Self.m_contactCount - 1;
   end destroy;





   --  // This is the top level collision call for the time step. Here
   --  // all the narrow phase collision is processed for the world
   --  // contact list.
   --
   --  void b2ContactManager::Collide()
   --  {
   --    // Update awake contacts.
   --    b2Contact* c = m_contactList;
   --    while (c)
   --    {
   --       b2Fixture* fixtureA = c->GetFixtureA();
   --       b2Fixture* fixtureB = c->GetFixtureB();
   --       int32 indexA = c->GetChildIndexA();
   --       int32 indexB = c->GetChildIndexB();
   --       b2Body* bodyA = fixtureA->GetBody();
   --       b2Body* bodyB = fixtureB->GetBody();
   --
   --       // Is this contact flagged for filtering?
   --       if (c->m_flags & b2Contact::e_filterFlag)
   --       {
   --          // Should these bodies collide?
   --          if (bodyB->ShouldCollide(bodyA) == false)
   --          {
   --             b2Contact* cNuke = c;
   --             c = cNuke->GetNext();
   --             Destroy(cNuke);
   --             continue;
   --          }
   --
   --          // Check user filtering.
   --          if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
   --          {
   --             b2Contact* cNuke = c;
   --             c = cNuke->GetNext();
   --             Destroy(cNuke);
   --             continue;
   --          }
   --
   --          // Clear the filtering flag.
   --          c->m_flags &= ~b2Contact::e_filterFlag;
   --       }
   --
   --       bool activeA = bodyA->IsAwake() && bodyA->m_type != b2_staticBody;
   --       bool activeB = bodyB->IsAwake() && bodyB->m_type != b2_staticBody;
   --
   --       // At least one body must be awake and it must be dynamic or kinematic.
   --       if (activeA == false && activeB == false)
   --       {
   --          c = c->GetNext();
   --          continue;
   --       }
   --
   --       int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
   --       int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
   --       bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);
   --
   --       // Here we destroy contacts that cease to overlap in the broad-phase.
   --       if (overlap == false)
   --       {
   --          b2Contact* cNuke = c;
   --          c = cNuke->GetNext();
   --          Destroy(cNuke);
   --          continue;
   --       }
   --
   --       // The contact persists.
   --       c->Update(m_contactListener);
   --       c = c->GetNext();
   --    }
   --  }
   --

   procedure collide (Self : in out b2ContactManager)
   is
      c : access b2Contact := Self.m_contactList;
   begin
      -- Update awake contacts.
      --
      while c /= null
      loop
         declare
            use type Interfaces.Unsigned_32;

            fixtureA : constant access b2Fixture := c.getFixtureA;
            fixtureB : constant access b2Fixture := c.getFixtureB;

            indexA   : constant Natural          := c.getChildIndexA;
            indexB   : constant Natural          := c.getChildIndexB;

            bodyA    : constant access b2Body    := fixtureA.getBody;
            bodyB    : constant access b2Body    := fixtureB.getBody;

         begin
            -- Is this contact flagged for filtering?
            --
            if (c.m_flags and b2_Contact.e_filterFlag) /= 0
            then
               -- Should these bodies collide?
               --
               if bodyB.shouldCollide (bodyA) = False
               then
                  declare
                     cNuke : constant access b2Contact := c;
                  begin
                     c := cNuke.getNext;
                     Self.destroy (cNuke);
                     goto Continue;
                  end;
               end if;

               -- Check user filtering.
               --
               if         Self.m_contactFilter /= null
                 and then Self.m_contactFilter.shouldCollide (fixtureA, fixtureB) = False
               then
                  declare
                     cNuke : constant access b2Contact := c;
                  begin
                     c := cNuke.getNext;
                     Self.destroy (cNuke);
                     goto Continue;
                  end;
               end if;

               -- Clear the filtering flag.
               --
               c.m_flags_is (c.m_flags and (not b2_Contact.e_filterFlag));
            end if;

            declare
               activeA : constant Boolean := bodyA.isAwake and bodyA.getType /= b2_staticBody;
               activeB : constant Boolean := bodyB.isAwake and bodyB.getType /= b2_staticBody;
            begin
               -- At least one body must be awake and it must be dynamic or kinematic.
               --
               if activeA = False and activeB = False
               then
                  c := c.getNext;
                  goto Continue;
               end if;
            end;

            declare
               proxyIdA : constant Natural := fixtureA.m_proxies (indexA).proxyId;
               proxyIdB : constant Natural := fixtureB.m_proxies (indexB).proxyId;

               overlap  : constant Boolean := Self.m_broadPhase.testOverlap (proxyIdA, proxyIdB);
            begin
               -- Here we destroy contacts that cease to overlap in the broad-phase.
               --
               if overlap = False
               then
                  declare
                     cNuke : constant access b2Contact := c;
                  begin
                     c := cNuke.getNext;
                     Self.destroy (cNuke);
                     goto Continue;
                  end;
               end if;
            end;

            -- The contact persists.
            --
            c.update (Self.m_contactListener);
            c := c.getNext;
         end;

         <<Continue>>
      end loop;
   end collide;






   --  void b2ContactManager::FindNewContacts()
   --  {
   --    m_broadPhase.UpdatePairs(this);
   --  }
   --



   procedure findNewContacts (Self : in out b2ContactManager)
   is
      procedure update_Pairs is new b2_broad_Phase.updatePairs (callback_t => b2ContactManager,
                                                                addPair    => addPair);
   begin
      update_Pairs (Self.m_broadPhase, Self'Access);
   end findNewContacts;





   --  void b2ContactManager::AddPair(void* proxyUserDataA, void* proxyUserDataB)
   --  {
   --    b2FixtureProxy* proxyA = (b2FixtureProxy*)proxyUserDataA;
   --    b2FixtureProxy* proxyB = (b2FixtureProxy*)proxyUserDataB;
   --
   --    b2Fixture* fixtureA = proxyA->fixture;
   --    b2Fixture* fixtureB = proxyB->fixture;
   --
   --    int32 indexA = proxyA->childIndex;
   --    int32 indexB = proxyB->childIndex;
   --
   --    b2Body* bodyA = fixtureA->GetBody();
   --    b2Body* bodyB = fixtureB->GetBody();
   --
   --    // Are the fixtures on the same body?
   --    if (bodyA == bodyB)
   --    {
   --       return;
   --    }
   --
   --    // TODO_ERIN use a hash table to remove a potential bottleneck when both
   --    // bodies have a lot of contacts.
   --    // Does a contact already exist?
   --    b2ContactEdge* edge = bodyB->GetContactList();
   --    while (edge)
   --    {
   --       if (edge->other == bodyA)
   --       {
   --          b2Fixture* fA = edge->contact->GetFixtureA();
   --          b2Fixture* fB = edge->contact->GetFixtureB();
   --          int32 iA = edge->contact->GetChildIndexA();
   --          int32 iB = edge->contact->GetChildIndexB();
   --
   --          if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
   --          {
   --             // A contact already exists.
   --             return;
   --          }
   --
   --          if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
   --          {
   --             // A contact already exists.
   --             return;
   --          }
   --       }
   --
   --       edge = edge->next;
   --    }
   --
   --    // Does a joint override collision? Is at least one body dynamic?
   --    if (bodyB->ShouldCollide(bodyA) == false)
   --    {
   --       return;
   --    }
   --
   --    // Check user filtering.
   --    if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
   --    {
   --       return;
   --    }
   --
   --    // Call the factory.
   --    b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
   --    if (c == nullptr)
   --    {
   --       return;
   --    }
   --
   --    // Contact creation may swap fixtures.
   --    fixtureA = c->GetFixtureA();
   --    fixtureB = c->GetFixtureB();
   --    indexA = c->GetChildIndexA();
   --    indexB = c->GetChildIndexB();
   --    bodyA = fixtureA->GetBody();
   --    bodyB = fixtureB->GetBody();
   --
   --    // Insert into the world.
   --    c->m_prev = nullptr;
   --    c->m_next = m_contactList;
   --    if (m_contactList != nullptr)
   --    {
   --       m_contactList->m_prev = c;
   --    }
   --    m_contactList = c;
   --
   --    // Connect to island graph.
   --
   --    // Connect to body A
   --    c->m_nodeA.contact = c;
   --    c->m_nodeA.other = bodyB;
   --
   --    c->m_nodeA.prev = nullptr;
   --    c->m_nodeA.next = bodyA->m_contactList;
   --    if (bodyA->m_contactList != nullptr)
   --    {
   --       bodyA->m_contactList->prev = &c->m_nodeA;
   --    }
   --    bodyA->m_contactList = &c->m_nodeA;
   --
   --    // Connect to body B
   --    c->m_nodeB.contact = c;
   --    c->m_nodeB.other = bodyA;
   --
   --    c->m_nodeB.prev = nullptr;
   --    c->m_nodeB.next = bodyB->m_contactList;
   --    if (bodyB->m_contactList != nullptr)
   --    {
   --       bodyB->m_contactList->prev = &c->m_nodeB;
   --    }
   --    bodyB->m_contactList = &c->m_nodeB;
   --
   --    ++m_contactCount;
   --  }
   --

   procedure addPair (Self : access b2ContactManager;   proxyUserDataA : void_ptr;
                                                        proxyUserDataB : void_ptr)
   is
      proxyA   : b2FixtureProxy with Address => proxyUserDataA ,   import;
      proxyB   : b2FixtureProxy with Address => proxyUserDataB ,   import;

      fixtureA : access b2Fixture := proxyA.fixture;
      fixtureB : access b2Fixture := proxyB.fixture;

      indexA   : Natural := proxyA.childIndex;
      indexB   : Natural := proxyB.childIndex;

      bodyA    : access b2Body := fixtureA.getBody;
      bodyB    : access b2Body := fixtureB.getBody;

      edge     : access b2ContactEdge;

   begin
      -- Are the fixtures on the same body?
      --
      if bodyA = bodyB
      then
         return;
      end if;

      -- TODO_ERIN use a hash table to remove a potential bottleneck when both
      -- bodies have a lot of contacts.
      -- Does a contact already exist?
      --
      edge := bodyB.getContactList;

      while edge /= null
      loop
         if edge.other = bodyA
         then
            declare
               fA : constant access b2Fixture := edge.contact.getFixtureA;
               fB : constant access b2Fixture := edge.contact.getFixtureB;

               iA : constant        Natural   := edge.contact.getChildIndexA;
               iB : constant        Natural   := edge.contact.getChildIndexB;
            begin
               if    fA = fixtureA
                 and fB = fixtureB
                 and iA = indexA
                 and iB = indexB
               then
                  return;     -- A contact already exists.
               end if;

               if    fA = fixtureB
                 and fB = fixtureA
                 and iA = indexB
                 and iB = indexA
               then
                  return;     -- A contact already exists.
               end if;
            end;
          end if;

          edge := edge.next;
       end loop;


      -- Does a joint override collision? Is at least one body dynamic?
      --
      if bodyB.shouldCollide (bodyA) = False
      then
         return;
      end if;


      -- Check user filtering.
      --
      if         Self.m_contactFilter /= null
        and then Self.m_contactFilter.shouldCollide (fixtureA, fixtureB) = False
      then
         return;
      end if;


      -- Call the factory.
      --
      declare
         c : constant b2Contact_ptr := b2_Contact.create (fixtureA.all, indexA,
                                                          fixtureB.all, indexB);
      begin
         if c = null
         then
            return;
         end if;


         -- Contact creation may swap fixtures.
         --
         fixtureA := c.getFixtureA;
         fixtureB := c.getFixtureB;

         indexA   := c.getChildIndexA;
         indexB   := c.getChildIndexB;

         bodyA    := fixtureA.getBody;
         bodyB    := fixtureB.getBody;

         -- Insert into the world.
         --
         c.m_prev_is (null);
         c.m_next_is (Self.m_contactList);

         if Self.m_contactList /= null
         then
            Self.m_contactList.m_prev_is (c);
         end if;

         Self.m_contactList := c;

         -- Connect to island graph.
         --

         -- Connect to body A
         --
         c.m_nodeA.contact := c;
         c.m_nodeA.other   := bodyB;

         c.m_nodeA.prev := null;
         c.m_nodeA.next := bodyA.m_contactList;

         if bodyA.m_contactList /= null
         then
            bodyA.m_contactList.prev := c.m_nodeA;
         end if;

         bodyA.m_contactList_is (c.m_nodeA);

         -- Connect to body B
         --
         c.m_nodeB.contact := c;
         c.m_nodeB.other   := bodyA;

         c.m_nodeB.prev := null;
         c.m_nodeB.next := bodyB.m_contactList;

         if bodyB.m_contactList /= null
         then
            bodyB.m_contactList.prev := c.m_nodeB;
         end if;

         bodyB.m_contactList_is (c.m_nodeB);

         Self.m_contactCount := Self.m_contactCount + 1;
      end;
   end addPair;


end box2d.b2_contact_Manager;
