with
     box2d.b2_Contact,
     box2d.b2_Fixture,
     box2d.b2_Collision,
     box2d.b2_Math;


package box2d.b2_chain_polygon_Contact
is
   use b2_Contact,
       b2_Fixture,
       b2_Collision,
       b2_Math;



   --  class b2ChainAndPolygonContact : public b2Contact
   --

   type b2ChainAndPolygonContact is new b2Contact with private;



   --    static b2Contact* Create (b2Fixture* fixtureA, int32 indexA,
   --                              b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator);

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class;


   --    static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);
   --

   procedure destroy (contact : access b2Contact'Class);



   --    b2ChainAndPolygonContact(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB);
   --

   function to_b2ChainAndPolygonContact (fixtureA : access b2Fixture;   indexA : in Natural;
                                         fixtureB : access b2Fixture;   indexB : in Natural) return b2ChainAndPolygonContact;


   --    ~b2ChainAndPolygonContact() {}
   --

   procedure destruct (contact : in out b2ChainAndPolygonContact) is null;



   --    void Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB) override;
   --

   overriding
   procedure evaluate (Self : in out b2ChainAndPolygonContact;   manifold : in out b2Manifold;
                                                                 xfA, xfB : in     b2Transform);



private

   type b2ChainAndPolygonContact is new b2Contact with null record;

end box2d.b2_chain_polygon_Contact;
