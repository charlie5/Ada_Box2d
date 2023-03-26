with
     box2d.b2_Math,
     box2d.b2_Distance;
     --  box2d.b2_Settings;


package box2d.b2_Time_of_impact
is
   use b2_Distance,
       b2_Math;
       --  b2_Settings;


   --  Input parameters for b2TimeOfImpact
   --
   --  struct b2TOIInput
   --  {
   --    b2DistanceProxy proxyA;
   --    b2DistanceProxy proxyB;
   --    b2Sweep sweepA;
   --    b2Sweep sweepB;
   --    float tMax;    // defines sweep interval [0, tMax]
   --  };
   --

   type b2TOIInput is
      record
         proxyA,
         proxyB : aliased b2DistanceProxy;

         sweepA,
         sweepB : b2Sweep;

         tMax   : Real;         -- Defines sweep interval [0, tMax].
      end record;



   --    enum State
   --    {
   --       e_unknown,
   --       e_failed,
   --       e_overlapped,
   --       e_touching,
   --       e_separated
   --    };
   --

   type State_t is (e_unknown,
                    e_failed,
                    e_overlapped,
                    e_touching,
                    e_separated);



   --  Output parameters for b2TimeOfImpact.
   --
   --  struct b2TOIOutput
   --  {
   --    State state;
   --    float t;
   --  };
   --

   type b2TOIOutput is
      record
         State : State_t;
         t     : Real;
      end record;



   --  Compute the upper bound on time before two shapes penetrate. Time is represented as
   --  a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
   --  non-tunneling collisions. If you change the time interval, you should call this function
   --  again.
   --
   --  Note: use b2Distance to compute the contact point and normal at the time of impact.
   --
   --  void b2TimeOfImpact(b2TOIOutput* output, const b2TOIInput* input);
   --

   procedure b2TimeOfImpact (output :    out b2TOIOutput;
                             input  : in     b2TOIInput);


end box2d.b2_Time_of_impact;
