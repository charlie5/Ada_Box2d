with
     b2_circle_Shape,
     b2_edge_Shape,
     b2_collide_Edge,
     b2_Shape,
     ada.unchecked_Deallocation;


package body b2_edge_circle_Contact
is

   --  b2Contact* b2EdgeAndCircleContact::Create(b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2EdgeAndCircleContact));
   --    return new (mem) b2EdgeAndCircleContact(fixtureA, fixtureB);
   --  }
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      return new b2EdgeAndCircleContact' (to_b2EdgeAndCircleContact (fixtureA'Access,
                                                                     fixtureB'Access));
   end create;




   --  void b2EdgeAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    ((b2EdgeAndCircleContact*)contact)->~b2EdgeAndCircleContact();
   --    allocator->Free(contact, sizeof(b2EdgeAndCircleContact));
   --  }
   --

   procedure destroy (contact : access b2Contact'Class)
   is
      type b2EdgeAndCircleContact_ptr is access all b2EdgeAndCircleContact;

      procedure free is new ada.unchecked_Deallocation (b2EdgeAndCircleContact, b2EdgeAndCircleContact_ptr);

      Self : b2EdgeAndCircleContact_ptr := b2EdgeAndCircleContact_ptr (contact);
   begin
      Self.destruct;
      free (Self);
   end destroy;



   --  b2EdgeAndCircleContact::b2EdgeAndCircleContact(b2Fixture* fixtureA, b2Fixture* fixtureB)
   --  : b2Contact(fixtureA, 0, fixtureB, 0)
   --  {
   --    b2Assert(m_fixtureA->GetType() == b2Shape::e_edge);
   --    b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
   --  }
   --

   function to_b2EdgeAndCircleContact (fixtureA : access b2Fixture;
                                       fixtureB : access b2Fixture) return b2EdgeAndCircleContact
   is
      use b2_Shape;

      Self : aliased b2EdgeAndCircleContact;
   begin
      define (Self, fixtureA, 0,
                    fixtureB, 0);

      pragma assert (Self.getFixtureA.getType = b2_Shape.e_edge);
      pragma assert (Self.getFixtureB.getType = b2_Shape.e_circle);

      return Self;
   end to_b2EdgeAndCircleContact;




   --  void b2EdgeAndCircleContact::Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2CollideEdgeAndCircle (manifold,
   --                           (b2EdgeShape*)m_fixtureA->GetShape(),   xfA,
   --                           (b2CircleShape*)m_fixtureB->GetShape(), xfB);
   --  }
   --

   overriding
   procedure evaluate (Self : in out b2EdgeAndCircleContact;   manifold : in out b2Manifold;
                                                               xfA, xfB : in     b2Transform)
   is
      use b2_circle_Shape,
          b2_edge_Shape,
          b2_collide_Edge;

      type b2edgeShape_ptr   is access all b2edgeShape;
      type b2circleShape_ptr is access all b2circleShape;

      edge   : constant access b2EdgeShape   := b2EdgeShape_ptr   (Self.getFixtureA.getShape);
      circle : constant access b2circleShape := b2circleShape_ptr (Self.getFixtureB.getShape);
   begin
      b2CollideEdgeAndCircle (manifold, edge  .all, xfA,
                                        circle.all, xfB);
   end evaluate;


end b2_edge_circle_Contact;
