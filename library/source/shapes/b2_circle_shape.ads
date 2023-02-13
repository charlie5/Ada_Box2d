with
     b2_Shape,
     b2_Collision,
     b2_Math,
     b2_Types;


package b2_circle_Shape
is
   use b2_Shape,
       b2_Collision,
       b2_Math,
       b2_Types;



   --  A solid circle shape.
   --
   --  class b2CircleShape : public b2Shape
   --  {
   --  public:
   --
   --    /// Position
   --    b2Vec2 m_p;
   --  };

   type b2circleShape is new b2Shape with
      record
         m_p : aliased b2Vec2;     -- Position.
      end record;




   --    b2CircleShape();
   --

   function to_b2circleShape return b2circleShape
     with Inline;



   -- Implement b2Shape.
   --
   -- b2Shape* Clone(b2BlockAllocator* allocator) const override;
   --

   overriding
   function clone (Self : in b2circleShape) return b2Shape_ptr;



   -- @see b2Shape::GetChildCount.
   --
   --   int32 GetChildCount() const override;
   --

   overriding
   function getChildCount (Self : in b2circleShape) return Natural;



   --    Implement b2Shape.
   --
   --    bool TestPoint (const b2Transform& transform, const b2Vec2& p) const override;
   --

   overriding
   function testPoint (Self : in b2circleShape;   Transform : in b2Transform;
                                                  p         : in b2Vec2) return Boolean;



   --    Implement b2Shape.
   --
   --    @note because the circle is solid, rays that start inside do not hit because the normal is not defined.
   --
   --    bool RayCast (b2RayCastOutput* output, const b2RayCastInput& input,
   --                  const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   function raycast (Self : in b2circleShape;   Output     :    out b2RayCastOutput;
                                                Input      : in     b2RayCastInput;
                                                Transform  : in     b2Transform;
                                                childIndex : in     Natural)        return Boolean;



   --    @see b2Shape::ComputeAABB.
   --
   --    void ComputeAABB (b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   procedure computeAABB (Self : in b2circleShape;   aabb       :    out b2AABB;
                                                     Transform  : in     b2Transform;
                                                     childIndex : in     Natural);



   --    @see b2Shape::ComputeMass
   --
   --    void ComputeMass(b2MassData* massData, float density) const override;
   --

   overriding
   procedure computeMass (Self : in b2circleShape;   massData :    out b2MassData;
                                                     Density  : in     Real);


end b2_circle_Shape;
