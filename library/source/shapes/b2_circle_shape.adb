with
     b2_circle_Shape,
     b2_Common;


package body b2_circle_Shape
is

   --  inline b2CircleShape::b2CircleShape()
   --  {
   --    m_type = e_circle;
   --    m_radius = 0.0f;
   --    m_p.SetZero();
   --  }
   --

   function to_b2circleShape return b2circleShape
   is
      Self : b2circleShape;
   begin
      Self.m_Type   := e_Circle;
      Self.m_Radius := 0.0;

      setZero (Self.m_p);

      return Self;
   end to_b2circleShape;





   --  b2Shape* b2CircleShape::Clone(b2BlockAllocator* allocator) const
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2CircleShape));
   --    b2CircleShape* clone = new (mem) b2CircleShape;
   --    *clone = *this;
   --    return clone;
   --  }
   --

   type b2circleShape_ptr is access all b2circleShape;


   overriding
   function clone (Self : in b2circleShape) return b2Shape_ptr
   is
      Clone : constant b2CircleShape_ptr := new b2CircleShape;
   begin
      Clone.all := Self;

     return Clone.all'Access;
   end clone;




   --  int32 b2CircleShape::GetChildCount() const
   --  {
   --    return 1;
   --  }

   overriding
   function getChildCount (Self : in b2circleShape) return Natural
   is
   begin
      return 1;
   end getChildCount;




   --  bool b2CircleShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
   --  {
   --    b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
   --    b2Vec2 d = p - center;
   --    return b2Dot(d, d) <= m_radius * m_radius;
   --  }
   --

   overriding
   function testPoint (Self : in b2circleShape;   Transform : in b2Transform;
                                                  p         : in b2Vec2) return Boolean
   is
      Center : constant b2Vec2 := Transform.p + b2Mul (transform.q, Self.m_p);
      d      : constant b2Vec2 := p - Center;
   begin
      return b2Dot (d, d) <= Self.m_Radius * Self.m_Radius;
   end testPoint;





   --  @note because the circle is solid, rays that start inside do not hit because the normal is not defined.
   --
   --  Collision Detection in Interactive 3D Environments by Gino van den Bergen.
   --  From Section 3.1.2
   --
   --  x = s + a * r
   --  norm(x) = radius
   --
   --
   --  bool b2CircleShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
   --                      const b2Transform& transform, int32 childIndex) const
   --  {
   --    B2_NOT_USED(childIndex);
   --
   --    b2Vec2 position = transform.p + b2Mul(transform.q, m_p);
   --    b2Vec2 s = input.p1 - position;
   --    float b = b2Dot(s, s) - m_radius * m_radius;
   --
   --    // Solve quadratic equation.
   --    b2Vec2 r = input.p2 - input.p1;
   --    float c =  b2Dot(s, r);
   --    float rr = b2Dot(r, r);
   --    float sigma = c * c - rr * b;
   --
   --    // Check for negative discriminant and short segment.
   --    if (sigma < 0.0f || rr < b2_epsilon)
   --    {
   --       return false;
   --    }
   --
   --    // Find the point of intersection of the line with the circle.
   --    float a = -(c + b2Sqrt(sigma));
   --
   --    // Is the intersection point on the segment?
   --    if (0.0f <= a && a <= input.maxFraction * rr)
   --    {
   --       a /= rr;
   --       output->fraction = a;
   --       output->normal = s + a * r;
   --       output->normal.Normalize();
   --       return true;
   --    }
   --
   --    return false;
   --  }
   --

   overriding
   function raycast (Self : in b2circleShape;   Output     :    out b2RayCastOutput;
                                                Input      : in     b2RayCastInput;
                                                Transform  : in     b2Transform;
                                                childIndex : in     Natural)        return Boolean
   is
      pragma Unreferenced (childIndex);
      use b2_Common;

      Position : constant b2Vec2 := Transform.p  + b2Mul (Transform.q, Self.m_p);
      s        : constant b2Vec2 := Input.p1     - Position;
      b        : constant Real   := b2Dot (s, s) - Self.m_Radius * Self.m_Radius;

      -- Solve quadratic equation.
      --
      r     : constant b2Vec2 := Input.p2 - Input.p1;
      c     : constant Real   := b2Dot (s, r);
      rr    : constant Real   := b2Dot (r, r);
      Sigma : constant Real   := c * c  -  rr * b;

   begin
      -- Check for negative discriminant and short segment.
      --
      if Sigma < 0.0 or rr < b2_Epsilon
      then
         return False;
      end if;


      -- Find the point of intersection of the line with the circle.
      --
      declare
         a : Real := -(c + b2Sqrt (Sigma));
      begin
         -- Is the intersection point on the segment?
         --
         if    0.0 <= a
           and   a <= input.maxFraction * rr
         then
            a               := a / rr;
            Output.Fraction := a;
            Output.Normal   := s  +  a * r;
            normalize (Output.Normal);

            return True;
         end if;
      end;

      return False;
   end raycast;




   --    @see b2Shape::ComputeAABB.
   --
   --  void b2CircleShape::ComputeAABB (b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
   --  {
   --    B2_NOT_USED(childIndex);
   --
   --    b2Vec2 p = transform.p + b2Mul(transform.q, m_p);
   --    aabb->lowerBound.Set(p.x - m_radius, p.y - m_radius);
   --    aabb->upperBound.Set(p.x + m_radius, p.y + m_radius);
   --  }
   --

   overriding
   procedure computeAABB (Self : in b2circleShape;   aabb       :    out b2AABB;
                                                     Transform  : in     b2Transform;
                                                     childIndex : in     Natural)
   is
      pragma Unreferenced (childIndex);

      p : constant b2Vec2 := Transform.p  +  b2Mul (Transform.q, Self.m_p);
   begin
      set (aabb.lowerBound,  p.x - Self.m_radius,  p.y - Self.m_radius);
      set (aabb.upperBound,  p.x + Self.m_radius,  p.y + Self.m_radius);
   end computeAABB;




   --    @see b2Shape::ComputeMass
   --
   --  void b2CircleShape::ComputeMass(b2MassData* massData, float density) const
   --  {
   --    massData->mass = density * b2_pi * m_radius * m_radius;
   --    massData->center = m_p;
   --
   --    // inertia about the local origin
   --    massData->I = massData->mass * (0.5f * m_radius * m_radius + b2Dot(m_p, m_p));
   --  }
   --

   overriding
   procedure computeMass (Self : in b2circleShape;   massData :    out b2MassData;
                                                     Density  : in     Real)
   is
      use b2_Common;
   begin
      massData.Mass   := Density * b2_Pi * Self.m_Radius * Self.m_Radius;
      massData.Center := Self.m_p;

      -- Inertia about the local origin.
      --
      massData.I := massData.Mass * (0.5 * Self.m_Radius * Self.m_Radius + b2Dot (Self.m_p, Self.m_p));
   end computeMass;


end b2_circle_Shape;
