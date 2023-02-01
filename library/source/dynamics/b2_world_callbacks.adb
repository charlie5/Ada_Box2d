with
     b2_Fixture,
     b2_world_Callbacks;


package body b2_world_Callbacks
is
   procedure dummy is null;


   --
--  // Return true if contact calculations should be performed between these two shapes.
--  // If you implement your own collision filter you may want to build from this implementation.
--  bool b2ContactFilter::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB)
--  {
--    const b2Filter& filterA = fixtureA->GetFilterData();
--    const b2Filter& filterB = fixtureB->GetFilterData();
--
--    if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
--    {
--       return filterA.groupIndex > 0;
--    }
--
--    bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
--    return collide;
--  }
end b2_world_Callbacks;
