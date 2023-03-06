with
     b2_circle_Shape,
     b2_edge_Shape,
     b2_collide_Circle,
     b2_Shape,
     ada.unchecked_Deallocation;


package body b2_circle_Contact
is

   --  b2Contact* b2CircleContact::Create (b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2CircleContact));
   --    return new (mem) b2CircleContact(fixtureA, fixtureB);
   --  }
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      return new b2CircleContact' (to_b2CircleContact (fixtureA'Access, indexA,
                                                       fixtureB'Access, indexB));
   end create;




   --  void b2CircleContact::Destroy (b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    ((b2CircleContact*)contact)->~b2CircleContact();
   --    allocator->Free(contact, sizeof(b2CircleContact));
   --  }
   --

   procedure destroy (contact : access b2Contact'Class)
   is
      type b2CircleContact_ptr is access all b2CircleContact;

      procedure free is new ada.unchecked_Deallocation (b2CircleContact, b2CircleContact_ptr);

      Self : b2CircleContact_ptr := b2CircleContact_ptr (contact);
   begin
      Self.destruct;
      free (Self);
   end destroy;



   --  b2CircleContact::b2CircleContact (b2Fixture* fixtureA, b2Fixture* fixtureB)
   --    : b2Contact(fixtureA, 0, fixtureB, 0)
   --  {
   --    b2Assert(m_fixtureA->GetType() == b2Shape::e_circle);
   --    b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
   --  }
   --

   function to_b2CircleContact (fixtureA : access b2Fixture;   indexA : in Natural;
                                fixtureB : access b2Fixture;   indexB : in Natural) return b2CircleContact
   is
      use b2_Shape;

      Self : aliased b2CircleContact;
   begin
      define (Self, fixtureA, indexA,
                    fixtureB, indexB);

      pragma assert (Self.getFixtureA.getType = b2_Shape.e_circle);
      pragma assert (Self.getFixtureB.getType = b2_Shape.e_circle);

      return Self;
   end to_b2CircleContact;




   --  void b2CircleContact::Evaluate (b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2CollideCircles(manifold,
   --                (b2CircleShape*)m_fixtureA->GetShape(), xfA,
   --                (b2CircleShape*)m_fixtureB->GetShape(), xfB);
   --  }
   --

   overriding
   procedure evaluate (Self : in out b2CircleContact;   manifold : in out b2Manifold;
                                                        xfA, xfB : in     b2Transform)
   is
      use b2_circle_Shape,
          b2_collide_Circle;

      type b2circleShape_ptr is access all b2circleShape;

      circleA : constant access b2circleShape := b2circleShape_ptr (Self.getFixtureA.getShape);
      circleB : constant access b2circleShape := b2circleShape_ptr (Self.getFixtureB.getShape);
   begin
      b2collideCircles (manifold, circleA.all, xfA,
                                  circleB.all, xfB);
   end evaluate;


end b2_circle_Contact;
