with
     box2d.b2_Math,
     --  box2d.b2_Settings,
     interfaces.C.Pointers;


package box2d.b2_Pointers
is
   use b2_Math;
       --  b2_Settings;


   type b2Vec2_array is array (Natural range <>) of aliased b2_Math.b2Vec2;

   Terminator : constant b2_Math.b2Vec2 := (Real'Last, Real'Last);

   package b2Vec2_Pointers is new interfaces.C.Pointers (Index              => Natural,
                                                         Element            => b2Vec2,
                                                         Element_array      => b2Vec2_array,
                                                         Default_Terminator => Terminator);
   subtype b2Vec2_ptr is b2Vec2_Pointers.Pointer;


end box2d.b2_Pointers;
