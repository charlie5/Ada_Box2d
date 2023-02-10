with
     b2_edge_Shape,
     b2_Common,
     b2_Settings,
     ada.unchecked_Conversion;


package body b2_edge_Shape
is

   --  inline b2EdgeShape::b2EdgeShape()
   --  {
   --    m_type = e_edge;
   --    m_radius = b2_polygonRadius;
   --    m_vertex0.x = 0.0f;
   --    m_vertex0.y = 0.0f;
   --    m_vertex3.x = 0.0f;
   --    m_vertex3.y = 0.0f;
   --    m_oneSided = false;
   --  }
   --

   function to_b2EdgeShape return b2EdgeShape
   is
      use b2_Common;

      Self : b2EdgeShape;
   begin
      Self.m_Type      := e_edge;
      Self.m_Radius    := b2_polygonRadius;
      Self.m_Vertex0.x := 0.0;
      Self.m_Vertex0.y := 0.0;
      Self.m_Vertex3.x := 0.0;
      Self.m_Vertex3.y := 0.0;
      Self.m_oneSided  := False;

      return Self;
   end to_b2EdgeShape;




   --  void b2EdgeShape::SetOneSided(const b2Vec2& v0, const b2Vec2& v1, const b2Vec2& v2, const b2Vec2& v3)
   --  {
   --    m_vertex0 = v0;
   --    m_vertex1 = v1;
   --    m_vertex2 = v2;
   --    m_vertex3 = v3;
   --    m_oneSided = true;
   --  }
   --

   procedure setOneSided (Self : in out b2edgeShape;   v0, v1, v2, v3 : in b2Vec2)
   is
   begin
     Self.m_vertex0  := v0;
     Self.m_vertex1  := v1;
     Self.m_vertex2  := v2;
     Self.m_vertex3  := v3;
     Self.m_oneSided := True;
   end setOneSided;




   --  void b2EdgeShape::SetTwoSided(const b2Vec2& v1, const b2Vec2& v2)
   --  {
   --    m_vertex1 = v1;
   --    m_vertex2 = v2;
   --    m_oneSided = false;
   --  }
   --

   procedure setTwoSided (Self : in out b2edgeShape;   v1, v2 : in b2Vec2)
   is
   begin
      Self.m_vertex1  := v1;
      Self.m_vertex2  := v2;
      Self.m_oneSided := False;
   end setTwoSided;



   --  b2Shape* b2EdgeShape::Clone(b2BlockAllocator* allocator) const
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2EdgeShape));
   --    b2EdgeShape* clone = new (mem) b2EdgeShape;
   --    *clone = *this;
   --    return clone;
   --  }
   --

   type b2edgeShape_ptr is access all b2edgeShape;


   overriding
   function clone (Self : in b2edgeShape;   Allocator : in out b2BlockAllocator) return b2Shape_ptr
   is
      use b2_Settings;
      use type int32;

      function to_b2edgeShape_ptr is new ada.unchecked_Conversion (void_ptr, b2edgeShape_ptr);

      Mem   : constant void_ptr        := allocate (Allocator, b2EdgeShape'Size / 8);
      Clone : constant b2edgeShape_ptr := new (Mem) b2EdgeShape;
   begin
      Clone.all := Self;

      return Clone.all'Access;
   end clone;



   --  int32 b2EdgeShape::GetChildCount() const
   --  {
   --    return 1;
   --  }
   --

   overriding
   function getChildCount (Self : in b2edgeShape) return Natural
   is
   begin
      return 1;
   end getChildCount;



   --  bool b2EdgeShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
   --  {
   --    B2_NOT_USED(xf);
   --    B2_NOT_USED(p);
   --    return false;
   --  }
   --

   overriding
   function testPoint (Self : in b2edgeShape;   Transform : in b2Transform;
                                                p         : in b2Vec2) return Boolean
   is
   begin
      return False;
   end testPoint;



   -- p = p1 + t * d
   -- v = v1 + s * e
   -- p1 + t * d = v1 + s * e
   -- s * e - t * d = p1 - v1
   --
   --  bool b2EdgeShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
   --                            const b2Transform& xf, int32 childIndex) const
   --  {
   --    B2_NOT_USED(childIndex);
   --
   --    -- Put the ray into the edge's frame of reference.
   --    b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
   --    b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
   --    b2Vec2 d = p2 - p1;
   --
   --    b2Vec2 v1 = m_vertex1;
   --    b2Vec2 v2 = m_vertex2;
   --    b2Vec2 e = v2 - v1;
   --
   --    -- Normal points to the right, looking from v1 at v2
   --    b2Vec2 normal(e.y, -e.x);
   --    normal.Normalize();
   --
   --    -- q = p1 + t * d
   --    -- dot(normal, q - v1) = 0
   --    -- dot(normal, p1 - v1) + t * dot(normal, d) = 0
   --    float numerator = b2Dot(normal, v1 - p1);
   --    if (m_oneSided && numerator > 0.0f)
   --    {
   --       return false;
   --    }
   --
   --    float denominator = b2Dot(normal, d);
   --
   --    if (denominator == 0.0f)
   --    {
   --       return false;
   --    }
   --
   --    float t = numerator / denominator;
   --    if (t < 0.0f || input.maxFraction < t)
   --    {
   --       return false;
   --    }
   --
   --    b2Vec2 q = p1 + t * d;
   --
   --    -- q = v1 + s * r
   --    -- s = dot(q - v1, r) / dot(r, r)
   --
   --    b2Vec2 r = v2 - v1;
   --    float rr = b2Dot(r, r);
   --    if (rr == 0.0f)
   --    {
   --       return false;
   --    }
   --
   --    float s = b2Dot(q - v1, r) / rr;
   --    if (s < 0.0f || 1.0f < s)
   --    {
   --       return false;
   --    }
   --
   --    output->fraction = t;
   --    if (numerator > 0.0f)
   --    {
   --       output->normal = -b2Mul(xf.q, normal);
   --    }
   --    else
   --    {
   --       output->normal = b2Mul(xf.q, normal);
   --    }
   --    return true;
   --  }
   --

   overriding
   function raycast (Self : in b2edgeShape;   Output     :    out b2RayCastOutput;
                                              Input      : in     b2RayCastInput;
                                              Transform  : in     b2Transform;
                                              childIndex : in     Natural) return Boolean
   is
      xf : b2Transform renames Transform;

      -- Put the ray into the edge's frame of reference.
      --
      p1 : constant b2Vec2 := b2MulT (xf.q,  input.p1 - xf.p);
      p2 : constant b2Vec2 := b2MulT (xf.q,  input.p2 - xf.p);
      d  : constant b2Vec2 := p2 - p1;

      v1 : constant b2Vec2 := Self.m_vertex1;
      v2 : constant b2Vec2 := Self.m_vertex2;
      e  : constant b2Vec2 := v2 - v1;

      Normal : b2Vec2 := (e.y, -e.x);

      Numerator   : Real;
      Denominator : Real;

      t  : Real;
      q  : b2Vec2;
      r  : b2Vec2;
      rr : Real;
      s  : Real;

   begin
      -- Normal points to the right, looking from v1 at v2.
      --
      normalize (Normal);


      -- q = p1 + t * d
      -- dot (normal, q - v1) = 0
      -- dot (normal, p1 - v1)  +  t * dot(normal, d)  =  0

      Numerator := b2Dot (Normal,  v1 - p1);

      if    Self.m_oneSided
        and Numerator > 0.0
      then
         return False;
      end if;

      Denominator := b2Dot (Normal, d);

      if Denominator = 0.0
      then
         return False;
      end if;

      t := Numerator / Denominator;

      if   t                 < 0.0
        or Input.maxFraction < t
      then
         return False;
      end if;

      q := p1  +  t * d;

      --  q = v1 + s * r
      --  s = dot(q - v1, r) / dot(r, r)

      r  := v2 - v1;
      rr := b2Dot (r, r);

      if rr = 0.0
      then
         return False;
      end if;

      s :=   b2Dot (q - v1, r)
           / rr;

      if   s   < 0.0
        or 1.0 < s
      then
         return False;
      end if;

      Output.Fraction := t;

      if Numerator > 0.0 then   Output.Normal := -b2Mul (xf.q, Normal);
                         else   Output.Normal :=  b2Mul (xf.q, Normal);
      end if;

      return True;
   end rayCast;





   --  void b2EdgeShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
   --  {
   --    B2_NOT_USED(childIndex);
   --
   --    b2Vec2 v1 = b2Mul(xf, m_vertex1);
   --    b2Vec2 v2 = b2Mul(xf, m_vertex2);
   --
   --    b2Vec2 lower = b2Min(v1, v2);
   --    b2Vec2 upper = b2Max(v1, v2);
   --
   --    b2Vec2 r(m_radius, m_radius);
   --    aabb->lowerBound = lower - r;
   --    aabb->upperBound = upper + r;
   --  }
   --

   overriding
   procedure computeAABB (Self : in b2edgeShape;   aabb       :    out b2AABB;
                                                   Transform  : in     b2Transform;
                                                   childIndex : in     Natural)
   is
      v1    : constant b2Vec2 := b2Mul (Transform, Self.m_vertex1);
      v2    : constant b2Vec2 := b2Mul (Transform, Self.m_vertex2);

      Lower : constant b2Vec2 := b2Min (v1, v2);
      Upper : constant b2Vec2 := b2Max (v1, v2);

      r     : constant b2Vec2 := (Self.m_radius,
                                  Self.m_radius);
   begin
      aabb.lowerBound := Lower - r;
      aabb.upperBound := Upper + r;
   end computeAABB;





   --  void b2EdgeShape::ComputeMass(b2MassData* massData, float density) const
   --  {
   --    B2_NOT_USED(density);
   --
   --    massData->mass = 0.0f;
   --    massData->center = 0.5f * (m_vertex1 + m_vertex2);
   --    massData->I = 0.0f;
   --  }
   --

   overriding
   procedure computeMass (Self : in b2edgeShape;   massData :    out b2MassData;
                                                   Density  : in     Real)
   is
   begin
      massData.Mass   := 0.0;
      massData.Center := 0.5 * (  Self.m_vertex1
                                + Self.m_vertex2);
      massData.I      := 0.0;
   end computeMass;


end b2_edge_Shape;
