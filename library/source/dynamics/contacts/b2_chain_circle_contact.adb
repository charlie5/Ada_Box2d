with
     b2_chain_Shape,
     b2_circle_Shape,
     b2_edge_Shape,
     b2_collide_Edge,
     b2_Shape,
     ada.unchecked_Deallocation;


package body b2_chain_circle_Contact
is


--  b2Contact* b2ChainAndCircleContact::Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
--  {
--    void* mem = allocator->Allocate(sizeof(b2ChainAndCircleContact));
--    return new (mem) b2ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
--  }
--

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      return new b2ChainAndCircleContact' (to_b2ChainAndCircleContact (fixtureA'Access, indexA,
                                                                       fixtureB'Access, indexB));
   end create;




   --  void b2ChainAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    ((b2ChainAndCircleContact*)contact)->~b2ChainAndCircleContact();
   --    allocator->Free(contact, sizeof(b2ChainAndCircleContact));
   --  }
   --

   procedure destroy (contact : access b2Contact'Class)
   is
      type b2ChainAndCircleContact_ptr is access all b2ChainAndCircleContact;

      procedure free is new ada.unchecked_Deallocation (b2ChainAndCircleContact, b2ChainAndCircleContact_ptr);

      Self : b2ChainAndCircleContact_ptr := b2ChainAndCircleContact_ptr (contact);
   begin
      Self.destruct;
      free (Self);
   end destroy;



   --  b2ChainAndCircleContact::b2ChainAndCircleContact(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB)
   --  : b2Contact(fixtureA, indexA, fixtureB, indexB)
   --  {
   --    b2Assert(m_fixtureA->GetType() == b2Shape::e_chain);
   --    b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
   --  }
   --

   function to_b2ChainAndCircleContact (fixtureA : access b2Fixture;   indexA : in Natural;
                                        fixtureB : access b2Fixture;   indexB : in Natural) return b2ChainAndCircleContact
   is
      use b2_Shape;

      Self : aliased b2ChainAndCircleContact;
   begin
      define (Self, fixtureA, indexA,
                    fixtureB, indexB);

      pragma assert (Self.getFixtureA.getType = b2_Shape.e_chain);
      pragma assert (Self.getFixtureB.getType = b2_Shape.e_circle);

      return Self;
   end to_b2ChainAndCircleContact;




   --    ~b2ChainAndCircleContact() {}
   --

   procedure destruct (contact : in out b2ChainAndCircleContact) is null;



   --  void b2ChainAndCircleContact::Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2ChainShape* chain = (b2ChainShape*)m_fixtureA->GetShape();
   --    b2EdgeShape edge;
   --    chain->GetChildEdge(&edge, m_indexA);
   --    b2CollideEdgeAndCircle( manifold, &edge, xfA,
   --                      (b2CircleShape*)m_fixtureB->GetShape(), xfB);
   --  }

   overriding
   procedure evaluate (Self : in out b2ChainAndCircleContact;   manifold : in out b2Manifold;
                                                                xfA, xfB : in     b2Transform)
   is
      use b2_chain_Shape,
          b2_edge_Shape,
          b2_circle_Shape,
          b2_collide_Edge;

      type b2chainShape_ptr  is access all b2chainShape;
      type b2circleShape_ptr is access all b2circleShape;

      chain  : constant access b2ChainShape  := b2ChainShape_ptr  (Self.getFixtureA.getShape);
      circle : constant access b2CircleShape := b2CircleShape_ptr (Self.getFixtureB.getShape);
      edge   :                 b2EdgeShape;
   begin
      chain.getChildEdge (edge, Self.getChildIndexA);

      b2collideEdgeAndCircle (manifold, edge,       xfA,
                                        circle.all, xfB);
   end evaluate;


end b2_chain_circle_Contact;
