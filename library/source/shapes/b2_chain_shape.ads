with
     b2_Shape,
     b2_edge_Shape,
     b2_Collision,
     b2_Math,
     b2_Pointers,
     b2_Types,
     b2_Settings;


package b2_chain_Shape
  --
  --  A chain shape is a free form sequence of line segments.
  --  The chain has one-sided collision, with the surface normal pointing to the right of the edge.
  --  This provides a counter-clockwise winding like the polygon shape.
  --  Connectivity information is used to create smooth collisions.
  --
  --  Warning: The chain will not collide properly if there are self-intersections.
  --
is
   use b2_Shape,
       b2_edge_Shape,
       b2_Collision,
       b2_Math,
       b2_Pointers,
       b2_Types,
       b2_Settings;



   type b2EdgeShape_ptr is access all b2EdgeShape;




   --  class b2chainShape : public b2Shape
   --  {
   --  public:
   --    The vertices. Owned by this class.
   --    b2Vec2* m_vertices;
   --
   --    The vertex count.
   --    int32   m_count;
   --
   --    b2Vec2  m_prevVertex, m_nextVertex;
   --  };
   --

   type b2chainShape is new b2Shape with
      record
         m_Vertices : b2Vec2s (0 .. default_max_chain_Vertices - 1);     -- The vertices. Owned by this class.
         m_Count    : Natural;                                           -- The vertex count.


         m_prevVertex,
         m_nextVertex : b2Vec2;
      end record;





   --    b2chainShape();
   --

   function to_b2chainShape return b2chainShape
     with inline ;




   --    The destructor frees the vertices using b2Free.
   --
   --    ~b2chainShape();
   --

   overriding
   procedure destruct (Self : in out b2chainShape);     -- The destructor frees the vertices using b2Free.




   --    Clear all data.
   --
   --    void Clear();
   --

   procedure clear (Self : in out b2chainShape);        -- Clear all data.




   --    Create a loop. This automatically adjusts connectivity.
   --
   --    @param vertices an array of vertices, these are copied
   --    @param count the vertex count
   --
   --    void CreateLoop (const b2Vec2* vertices, int32 count);

   procedure createLoop (Self : in out b2chainShape;   Vertices : in b2Vec2s;
                                                       Count    : in int32);




   --    Create a chain with ghost vertices to connect multiple chains together.
   --
   --    @param vertices an array of vertices, these are copied
   --    @param count the vertex count
   --    @param prevVertex previous vertex from chain that connects to the start
   --    @param nextVertex next vertex from chain that connects to the end
   --
   --    void CreateChain (const b2Vec2* vertices, int32 count,
   --                      const b2Vec2& prevVertex, const b2Vec2& nextVertex);
   --

   procedure createChain (Self : in out b2chainShape;   Vertices   : in b2Vec2s;
                                                        Count      : in Natural;
                                                        prevVertex,
                                                        nextVertex : in b2Vec2);




   --    Implement b2Shape. Vertices are cloned using b2Alloc.
   --
   --    b2Shape* Clone (b2BlockAllocator* allocator) const override;
   --

   overriding
   function clone (Self : in b2chainShape) return b2Shape_ptr;     -- Implement b2Shape. Vertices are cloned using b2Alloc.





   --    @see b2Shape::GetChildCount
   --
   --    int32 GetChildCount() const override;
   --

   overriding
   function getChildCount (Self : in b2chainShape) return Natural;     -- @see b2Shape::GetChildCount




   --    Get a child edge.
   --
   --    void GetChildEdge (b2EdgeShape* edge, int32 index) const;
   --

   procedure getChildEdge (Self  : in b2chainShape;   Edge  : in b2EdgeShape_ptr;     --  Get a child edge.
                                                      Index : in Natural);




   --    This always return false.
   --
   --    @see b2Shape::TestPoint
   --
   --    bool TestPoint (const b2Transform& transform, const b2Vec2& p) const override;
   --

   overriding
   function testPoint (Self : in b2chainShape;   Transform : in b2Transform;                     -- @see b2Shape::TestPoint
                                                 p         : in b2Vec2)      return Boolean;     -- This always return false.





   --    Implement b2Shape.
   --
   --    bool RayCast (b2RayCastOutput* output, const b2RayCastInput& input,
   --                 const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   function raycast (Self : in b2chainShape;   Output     :    out b2RaycastOutput;     -- Implement b2Shape.
                                               Input      : in     b2RaycastInput;
                                               Transform  : in     b2Transform;
                                               childIndex : in     Natural)         return Boolean;



   --    @see b2Shape::ComputeAABB
   --
   --    void ComputeAABB (b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   procedure computeAABB (Self : in b2chainShape;   aabb       :    out b2AABB;     --  @see b2Shape::ComputeAABB.
                                                    Transform  : in     b2Transform;
                                                    childIndex : in     Natural);



   --    Chains have zero mass.
   --
   --    @see b2Shape::ComputeMass
   --
   --    void ComputeMass (b2MassData* massData, float density) const override;
   --

   overriding
   procedure computeMass (Self : in b2chainShape;   massData :    out b2MassData;
                                                    Density  : in     b2_Math.Real);




   --  inline b2chainShape::b2chainShape()     -- Moved to body.


end b2_chain_Shape;
