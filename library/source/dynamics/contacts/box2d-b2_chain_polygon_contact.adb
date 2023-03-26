with
     box2d.b2_chain_Shape,
     box2d.b2_polygon_Shape,
     box2d.b2_edge_Shape,
     box2d.b2_collide_Edge,
     box2d.b2_Shape,

     ada.unchecked_Deallocation;


package body box2d.b2_chain_polygon_Contact
is

   --  b2Contact* b2ChainAndPolygonContact::Create (b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2ChainAndPolygonContact));
   --    return new (mem) b2ChainAndPolygonContact(fixtureA, indexA, fixtureB, indexB);
   --  }
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      return new b2ChainAndPolygonContact' (to_b2ChainAndPolygonContact (fixtureA'Access, indexA,
                                                                         fixtureB'Access, indexB));
   end create;




   --  void b2ChainAndPolygonContact::Destroy (b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    ((b2ChainAndPolygonContact*)contact)->~b2ChainAndPolygonContact();
   --    allocator->Free(contact, sizeof(b2ChainAndPolygonContact));
   --  }
   --

   procedure destroy (contact : access b2Contact'Class)
   is
      type b2ChainAndPolygonContact_ptr is access all b2ChainAndPolygonContact;

      procedure free is new ada.unchecked_Deallocation (b2ChainAndPolygonContact, b2ChainAndPolygonContact_ptr);

      Self : b2ChainAndPolygonContact_ptr := b2ChainAndPolygonContact_ptr (contact);
   begin
      Self.destruct;
      free (Self);
   end destroy;



   --  b2ChainAndPolygonContact::b2ChainAndPolygonContact (b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB)
   --  : b2Contact(fixtureA, indexA, fixtureB, indexB)
   --  {
   --    b2Assert(m_fixtureA->GetType() == b2Shape::e_chain);
   --    b2Assert(m_fixtureB->GetType() == b2Shape::e_polygon);
   --  }
   --

   function to_b2ChainAndPolygonContact (fixtureA : access b2Fixture;   indexA : in Natural;
                                         fixtureB : access b2Fixture;   indexB : in Natural) return b2ChainAndPolygonContact
   is
      use b2_Shape;

      Self : aliased b2ChainAndPolygonContact;
   begin
      define (Self, fixtureA, indexA,
                    fixtureB, indexB);

      pragma assert (Self.getFixtureA.getType = b2_Shape.e_chain);
      pragma assert (Self.getFixtureB.getType = b2_Shape.e_polygon);

      return Self;
   end to_b2ChainAndPolygonContact;




   --  void b2ChainAndPolygonContact::Evaluate (b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2ChainShape* chain = (b2ChainShape*)m_fixtureA->GetShape();
   --    b2EdgeShape edge;
   --    chain->GetChildEdge(&edge, m_indexA);
   --    b2CollideEdgeAndPolygon(   manifold, &edge, xfA,
   --                         (b2PolygonShape*)m_fixtureB->GetShape(), xfB);
   --  }
   --

   overriding
   procedure evaluate (Self : in out b2ChainAndPolygonContact;   manifold : in out b2Manifold;
                                                                 xfA, xfB : in     b2Transform)
   is
      use b2_chain_Shape,
          b2_edge_Shape,
          b2_polygon_Shape,
          b2_collide_Edge;

      type b2chainShape_ptr   is access all b2chainShape;
      type b2polygonShape_ptr is access all b2polygonShape;

      chain   : constant access b2ChainShape   := b2ChainShape_ptr   (Self.getFixtureA.getShape);
      polygon : constant access b2polygonShape := b2polygonShape_ptr (Self.getFixtureB.getShape);
      edge    :                 b2EdgeShape;
   begin
      chain.getChildEdge (edge, Self.getChildIndexA);

      b2collideEdgeAndPolygon (manifold, edge,        xfA,
                                         polygon.all, xfB);
   end evaluate;


end box2d.b2_chain_polygon_Contact;
