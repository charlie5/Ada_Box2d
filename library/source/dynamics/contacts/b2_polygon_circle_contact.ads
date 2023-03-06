with
     b2_Contact,
     b2_Fixture,
     b2_Collision,
     b2_Math;


package b2_polygon_circle_Contact
is

   use b2_Contact,
       b2_Fixture,
       b2_Collision,
       b2_Math;


   --  class b2PolygonAndCircleContact : public b2Contact
   --

   type b2PolygonAndCircleContact is new b2Contact with private;



   --    static b2Contact* Create (b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator);
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class;


   --    ~b2PolygonAndCircleContact() {}
   --

   procedure destroy (contact : access b2Contact'Class);



   --    b2PolygonAndCircleContact(b2Fixture* fixtureA, b2Fixture* fixtureB);
   --

   function to_b2PolygonAndCircleContact (fixtureA : access b2Fixture;
                                          fixtureB : access b2Fixture) return b2PolygonAndCircleContact;


   --    ~b2EdgeAndPolygonContact() {}
   --

   procedure destruct (contact : in out b2PolygonAndCircleContact) is null;



   --    void Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB) override;
   --

   overriding
   procedure evaluate (Self : in out b2PolygonAndCircleContact;   manifold : in out b2Manifold;
                                                                  xfA, xfB : in     b2Transform);



private

   type b2PolygonAndCircleContact is new b2Contact with null record;

end b2_polygon_circle_Contact;

