with
     box2d.b2_growable_Stack,
     box2d.b2_Types,
     box2d.b2_Common,

     ada.unchecked_Deallocation;

with Ada.Text_IO; use Ada.Text_IO;


package body box2d.b2_dynamic_Tree
is
   use b2_growable_Stack;

   package natural_Stacks is new b2GrowableStack (Natural, 256);
   use     natural_Stacks;

   subtype natural_Stack is natural_Stacks.Stack;



   --    bool b2TreeNode::IsLeaf() const
   --    {
   --       return child1 == b2_nullNode;
   --    }
   --

   function isLeaf (Self : in b2TreeNode) return Boolean
   is
   begin
      return Self.child1 = b2_nullNode;
   end isLeaf;





   --    Get proxy user data.
   --    @return the proxy user data or 0 if the id is invalid.
   --
   --
   --  inline void* b2DynamicTree::GetUserData(int32 proxyId) const
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    return m_nodes[proxyId].userData;
   --  }
   --

   function getUserData (Self : in b2DynamicTree;   proxyId : Natural) return system.Address
   is
      pragma assert (          0 <= proxyId
                     and proxyId <  Self.m_nodeCapacity);
   begin
      return Self.m_nodes (proxyId).userData;
   end getUserData;




   --    Constructing the tree initializes the node pool.
   --
   --    b2DynamicTree();
   --
   --  b2DynamicTree::b2DynamicTree()
   --  {
   --    m_root = b2_nullNode;
   --
   --    m_nodeCapacity = 16;
   --    m_nodeCount = 0;
   --    m_nodes = (b2TreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2TreeNode));
   --    memset(m_nodes, 0, m_nodeCapacity * sizeof(b2TreeNode));
   --
   --    // Build a linked list for the free list.
   --    for (int32 i = 0; i < m_nodeCapacity - 1; ++i)
   --    {
   --       m_nodes[i].next = i + 1;
   --       m_nodes[i].height = -1;
   --    }
   --    m_nodes[m_nodeCapacity-1].next = b2_nullNode;
   --    m_nodes[m_nodeCapacity-1].height = -1;
   --    m_freeList = 0;
   --
   --    m_insertionCount = 0;
   --  }
   --

   function to_b2DynamicTree return b2DynamicTree
   is
      Self : b2DynamicTree;
   begin
      Self.m_root         := b2_nullNode;

      Self.m_nodeCapacity := 16;
      Self.m_nodeCount    := 0;
      Self.m_nodes        := new b2TreeNodes (0 .. Self.m_nodeCapacity - 1);

      -- Build a linked list for the free list.
      --
      for i in 0 .. Self.m_nodeCapacity - 2
      loop
         --  memset (Self.m_nodes, 0, Self.m_nodeCapacity * sizeof (b2TreeNode));
         --
         Self.m_nodes (i).userData := system.null_Address;
         Self.m_nodes (i).child1   := 0;
         Self.m_nodes (i).child2   := 0;
         Self.m_nodes (i).Moved    := False;
         Self.m_nodes (i).aabb     := (lowerBound => b2Vec2' (x => 0.0, y => 0.0),
                                       upperBound => b2Vec2' (x => 0.0, y => 0.0));

         Self.m_nodes (i).PorN.next := i + 1;
         Self.m_nodes (i).height    := -1;
      end loop;

      Self.m_nodes (Self.m_nodeCapacity - 1).userData := system.null_Address;
      Self.m_nodes (Self.m_nodeCapacity - 1).child1   := 0;
      Self.m_nodes (Self.m_nodeCapacity - 1).child2   := 0;
      Self.m_nodes (Self.m_nodeCapacity - 1).Moved    := False;
      Self.m_nodes (Self.m_nodeCapacity - 1).aabb     := (lowerBound => b2Vec2' (x => 0.0, y => 0.0),
                                                          upperBound => b2Vec2' (x => 0.0, y => 0.0));

      Self.m_nodes (Self.m_nodeCapacity - 1).PorN.next := b2_nullNode;
      Self.m_nodes (Self.m_nodeCapacity - 1).height    := -1;

      Self.m_freeList       := 0;
      Self.m_insertionCount := 0;

      return Self;
   end to_b2DynamicTree;




   --    Destroy the tree, freeing the node pool.
   --
   --    ~b2DynamicTree();
   --
   --  b2DynamicTree::~b2DynamicTree()
   --  {
   --    // This frees the entire tree in one shot.
   --    b2Free(m_nodes);
   --  }
   --

   procedure destruct (Self : in out b2DynamicTree)
   is
      procedure free is new ada.unchecked_Deallocation (b2TreeNodes, b2TreeNodes_ptr);
   begin
      -- This frees the entire tree in one shot.
      --
      free (Self.m_nodes);
   end destruct;




   --    Create a proxy. Provide a tight fitting AABB and a userData pointer.
   --
   --    int32 CreateProxy(const b2AABB& aabb, void* userData);
   --
   --  // Create a proxy in the tree as a leaf node. We return the index
   --  // of the node instead of a pointer so that we can grow
   --  // the node pool.
   --
   --  int32 b2DynamicTree::CreateProxy(const b2AABB& aabb, void* userData)
   --  {
   --    int32 proxyId = AllocateNode();
   --
   --    // Fatten the aabb.
   --    b2Vec2 r(b2_aabbExtension, b2_aabbExtension);
   --    m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
   --    m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
   --    m_nodes[proxyId].userData = userData;
   --    m_nodes[proxyId].height = 0;
   --    m_nodes[proxyId].moved = true;
   --
   --    InsertLeaf(proxyId);
   --
   --    return proxyId;
   --  }
   --

   function createProxy (Self : in out b2DynamicTree;   aabb     : in b2AABB;
                                                        userData : in system.Address) return Natural
   is
      use b2_Common;

      proxyId : constant Natural := Self.allocateNode;
      r       : constant b2Vec2  := (b2_aabbExtension,
                                     b2_aabbExtension);
   begin
      -- Fatten the aabb.
      --
      Self.m_nodes (proxyId).aabb.lowerBound := aabb.lowerBound - r;
      Self.m_nodes (proxyId).aabb.upperBound := aabb.upperBound + r;
      Self.m_nodes (proxyId).userData        := userData;
      Self.m_nodes (proxyId).height          := 0;
      Self.m_nodes (proxyId).moved           := True;

      Self.insertLeaf (proxyId);

      return proxyId;
   end createProxy;




   --    Destroy a proxy. This asserts if the id is invalid.
   --
   --    void DestroyProxy(int32 proxyId);
   --
   --  void b2DynamicTree::DestroyProxy(int32 proxyId)
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    b2Assert(m_nodes[proxyId].IsLeaf());
   --
   --    RemoveLeaf(proxyId);
   --    FreeNode(proxyId);
   --  }
   --

   procedure destroyProxy (Self : in out b2DynamicTree;   proxyId : in Natural)
   is
      pragma assert (          0 <= proxyId
                     and proxyId <  Self.m_nodeCapacity);

      pragma assert (isLeaf (Self.m_nodes (proxyId)));
   begin
      Self.removeLeaf (proxyId);
      Self.freeNode   (proxyId);
   end destroyProxy;




   --    Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
   --    then the proxy is removed from the tree and re-inserted. Otherwise
   --    the function returns immediately.
   --    @return true if the proxy was re-inserted.
   --
   --    bool MoveProxy(int32 proxyId, const b2AABB& aabb1, const b2Vec2& displacement);
   --
   --  bool b2DynamicTree::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --
   --    b2Assert(m_nodes[proxyId].IsLeaf());
   --
   --    // Extend AABB
   --    b2AABB fatAABB;
   --    b2Vec2 r(b2_aabbExtension, b2_aabbExtension);
   --    fatAABB.lowerBound = aabb.lowerBound - r;
   --    fatAABB.upperBound = aabb.upperBound + r;
   --
   --    // Predict AABB movement
   --    b2Vec2 d = b2_aabbMultiplier * displacement;
   --
   --    if (d.x < 0.0f)
   --    {
   --       fatAABB.lowerBound.x += d.x;
   --    }
   --    else
   --    {
   --       fatAABB.upperBound.x += d.x;
   --    }
   --
   --    if (d.y < 0.0f)
   --    {
   --       fatAABB.lowerBound.y += d.y;
   --    }
   --    else
   --    {
   --       fatAABB.upperBound.y += d.y;
   --    }
   --
   --    const b2AABB& treeAABB = m_nodes[proxyId].aabb;
   --    if (treeAABB.Contains(aabb))
   --    {
   --       // The tree AABB still contains the object, but it might be too large.
   --       // Perhaps the object was moving fast but has since gone to sleep.
   --       // The huge AABB is larger than the new fat AABB.
   --       b2AABB hugeAABB;
   --       hugeAABB.lowerBound = fatAABB.lowerBound - 4.0f * r;
   --       hugeAABB.upperBound = fatAABB.upperBound + 4.0f * r;
   --
   --       if (hugeAABB.Contains(treeAABB))
   --       {
   --          // The tree AABB contains the object AABB and the tree AABB is
   --          // not too large. No tree update needed.
   --          return false;
   --       }
   --
   --       // Otherwise the tree AABB is huge and needs to be shrunk
   --    }
   --
   --    RemoveLeaf(proxyId);
   --
   --    m_nodes[proxyId].aabb = fatAABB;
   --
   --    InsertLeaf(proxyId);
   --
   --    m_nodes[proxyId].moved = true;
   --
   --    return true;
   --  }

   function moveProxy (Self : in out b2DynamicTree;   proxyId      : in Natural;
                                                      aabb1        : in b2AABB;
                                                      displacement : in b2Vec2) return Boolean
   is
      use b2_Common;

      pragma assert (          0 <= proxyId
                     and proxyId <  Self.m_nodeCapacity);

      pragma assert (isLeaf (Self.m_nodes (proxyId)));

      r : constant b2Vec2 := (b2_aabbExtension,
                              b2_aabbExtension);

      -- Extend AABB.
      --
      fatAABB : b2AABB := (lowerBound => aabb1.lowerBound - r,
                           upperBound => aabb1.upperBound + r);

      -- Predict AABB movement.
      --
      d : constant b2Vec2 := b2_aabbMultiplier * displacement;

   begin
      if d.x < 0.0 then   fatAABB.lowerBound.x := @ + d.x;
      else                fatAABB.upperBound.x := @ + d.x;
      end if;

      if d.y < 0.0 then   fatAABB.lowerBound.y := @ + d.y;
      else                fatAABB.upperBound.y := @ + d.y;
      end if;

      declare
         treeAABB : b2AABB renames Self.m_nodes (proxyId).aabb;
         hugeAABB : b2AABB;
      begin
         if contains (treeAABB, aabb1)
         then
            -- The tree AABB still contains the object, but it might be too large.
            -- Perhaps the object was moving fast but has since gone to sleep.
            -- The huge AABB is larger than the new fat AABB.
            --
            hugeAABB.lowerBound := fatAABB.lowerBound  -  4.0 * r;
            hugeAABB.upperBound := fatAABB.upperBound  +  4.0 * r;

            if contains (hugeAABB, treeAABB)
            then
               -- The tree AABB contains the object AABB and the tree AABB is
               -- not too large. No tree update needed.
               return False;
           end if;

            -- Otherwise the tree AABB is huge and needs to be shrunk.
         end if;
      end;

      Self.removeLeaf (proxyId);
      Self.m_nodes    (proxyId).aabb  := fatAABB;
      Self.insertLeaf (proxyId);
      Self.m_nodes    (proxyId).moved := True;

      return True;
   end moveProxy;




   --  inline bool b2DynamicTree::WasMoved(int32 proxyId) const
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    return m_nodes[proxyId].moved;
   --  }
   --
   --    bool WasMoved(int32 proxyId) const;
   --

   function wasMoved (Self : in b2DynamicTree;   proxyId : in Natural) return Boolean
   is
      pragma assert (          0 <= proxyId
                     and proxyId <  Self.m_nodeCapacity);
   begin
      return Self.m_nodes (proxyId).moved;
   end WasMoved;




   --    void ClearMoved(int32 proxyId);
   --
   --  inline void b2DynamicTree::ClearMoved(int32 proxyId)
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    m_nodes[proxyId].moved = false;
   --  }
   --

   procedure clearMoved (Self : in out b2DynamicTree;   proxyId : in Natural)
   is
      pragma assert (          0 <= proxyId
                     and proxyId <  Self.m_nodeCapacity);
   begin
      Self.m_nodes (proxyId).moved := False;
   end clearMoved;




   --    Get the fat AABB for a proxy.
   --
   --    const b2AABB& GetFatAABB(int32 proxyId) const;
   --
   --  inline const b2AABB& b2DynamicTree::GetFatAABB(int32 proxyId) const
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    return m_nodes[proxyId].aabb;
   --  }
   --

   function getFatAABB (Self : in b2DynamicTree;   proxyId : in Natural) return b2AABB
   is
      pragma assert (          0 <= proxyId
                     and proxyId <  Self.m_nodeCapacity);
   begin
      return Self.m_nodes (proxyId).aabb;
   end getFatAABB;




   --    Query an AABB for overlapping proxies. The callback class
   --    is called for each proxy that overlaps the supplied AABB.
   --
   --    template <typename T>
   --    void Query(T* callback, const b2AABB& aabb) const;
   --
   --  template <typename T>
   --  inline void b2DynamicTree::Query(T* callback, const b2AABB& aabb) const
   --  {
   --    b2GrowableStack<int32, 256> stack;
   --    stack.Push(m_root);
   --
   --    while (stack.GetCount() > 0)
   --    {
   --       int32 nodeId = stack.Pop();
   --       if (nodeId == b2_nullNode)
   --       {
   --          continue;
   --       }
   --
   --       const b2TreeNode* node = m_nodes + nodeId;
   --
   --       if (b2TestOverlap(node->aabb, aabb))
   --       {
   --          if (node->IsLeaf())
   --          {
   --             bool proceed = callback->QueryCallback(nodeId);
   --             if (proceed == false)
   --             {
   --                return;
   --             }
   --          }
   --          else
   --          {
   --             stack.Push(node->child1);
   --             stack.Push(node->child2);
   --          }
   --       }
   --    }
   --  }
   --

   procedure query (Self : in b2DynamicTree;   Callback : in out Callback_t;
                                               aabb     : in     b2AABB)
   is
      stack : natural_Stack := to_Stack;
   begin
      push (stack, Self.m_root);

      while getCount (stack) > 0
      loop
         declare
            nodeId  : constant Integer   := pop (stack);
            node    : access   b2TreeNode;
            proceed :          Boolean;
         begin
            if nodeId /= b2_nullNode
            then
               node := Self.m_nodes (nodeId)'Access;

               if b2testOverlap (node.aabb, aabb)
               then
                  if isLeaf (node.all)
                  then
                     proceed := queryCallback (callback, nodeId);

                     if proceed = False
                     then
                        return;
                     end if;

                  else
                     push (stack, node.child1);
                     push (stack, node.child2);
                  end if;
               end if;
            end if;
         end;
      end loop;
   end query;





   --    Ray-cast against the proxies in the tree. This relies on the callback
   --    to perform a exact ray-cast in the case were the proxy contains a shape.
   --    The callback also performs the any collision filtering. This has performance
   --    roughly equal to k * log(n), where k is the number of collisions and n is the
   --    number of proxies in the tree.
   --    @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
   --    @param callback a callback class that is called for each proxy that is hit by the ray.
   --
   --    template <typename T>
   --    void RayCast(T* callback, const b2RayCastInput& input) const;
   --
   --  template <typename T>
   --  inline void b2DynamicTree::RayCast(T* callback, const b2RayCastInput& input) const
   --  {
   --    b2Vec2 p1 = input.p1;
   --    b2Vec2 p2 = input.p2;
   --    b2Vec2 r = p2 - p1;
   --    b2Assert(r.LengthSquared() > 0.0f);
   --    r.Normalize();
   --
   --    // v is perpendicular to the segment.
   --    b2Vec2 v = b2Cross(1.0f, r);
   --    b2Vec2 abs_v = b2Abs(v);
   --
   --    // Separating axis for segment (Gino, p80).
   --    // |dot(v, p1 - c)| > dot(|v|, h)
   --
   --    float maxFraction = input.maxFraction;
   --
   --    // Build a bounding box for the segment.
   --    b2AABB segmentAABB;
   --    {
   --       b2Vec2 t = p1 + maxFraction * (p2 - p1);
   --       segmentAABB.lowerBound = b2Min(p1, t);
   --       segmentAABB.upperBound = b2Max(p1, t);
   --    }
   --
   --    b2GrowableStack<int32, 256> stack;
   --    stack.Push(m_root);
   --
   --    while (stack.GetCount() > 0)
   --    {
   --       int32 nodeId = stack.Pop();
   --       if (nodeId == b2_nullNode)
   --       {
   --          continue;
   --       }
   --
   --       const b2TreeNode* node = m_nodes + nodeId;
   --
   --       if (b2TestOverlap(node->aabb, segmentAABB) == false)
   --       {
   --          continue;
   --       }
   --
   --       // Separating axis for segment (Gino, p80).
   --       // |dot(v, p1 - c)| > dot(|v|, h)
   --       b2Vec2 c = node->aabb.GetCenter();
   --       b2Vec2 h = node->aabb.GetExtents();
   --       float separation = b2Abs(b2Dot(v, p1 - c)) - b2Dot(abs_v, h);
   --       if (separation > 0.0f)
   --       {
   --          continue;
   --       }
   --
   --       if (node->IsLeaf())
   --       {
   --          b2RayCastInput subInput;
   --          subInput.p1 = input.p1;
   --          subInput.p2 = input.p2;
   --          subInput.maxFraction = maxFraction;
   --
   --          float value = callback->RayCastCallback(subInput, nodeId);
   --
   --          if (value == 0.0f)
   --          {
   --             // The client has terminated the ray cast.
   --             return;
   --          }
   --
   --          if (value > 0.0f)
   --          {
   --             // Update segment bounding box.
   --             maxFraction = value;
   --             b2Vec2 t = p1 + maxFraction * (p2 - p1);
   --             segmentAABB.lowerBound = b2Min(p1, t);
   --             segmentAABB.upperBound = b2Max(p1, t);
   --          }
   --       }
   --       else
   --       {
   --          stack.Push(node->child1);
   --          stack.Push(node->child2);
   --       }
   --    }
   --  }
   --

   procedure raycast (Self : in b2DynamicTree;   Callback : in out Callback_t;
                                                 input    : in     b2RayCastInput)
   is
      p1 : constant b2Vec2 := input.p1;
      p2 : constant b2Vec2 := input.p2;
      r  :          b2Vec2 := p2 - p1;

      pragma assert (lengthSquared (r) > 0.0);

   begin
      normalize (r);

      declare
         -- v is perpendicular to the segment.
         --
         v     : constant b2Vec2 := b2Cross (1.0, r);
         abs_v : constant b2Vec2 := b2Abs (v);

         -- Separating axis for segment (Gino, p80).
         -- |dot(v, p1 - c)| > dot(|v|, h)
         --
         maxFraction : Real := input.maxFraction;

         -- Build a bounding box for the segment.
         --
         segmentAABB : b2AABB;
         t           : constant b2Vec2 := p1  +  maxFraction * (p2 - p1);

         stack       :  natural_Stack := to_Stack;
      begin
         segmentAABB.lowerBound := b2Min (p1, t);
         segmentAABB.upperBound := b2Max (p1, t);

         push (stack, Self.m_root);

         while getCount (stack) > 0
         loop
            declare
               nodeId : constant Natural   := pop (stack);
               node   : access   b2TreeNode;
            begin
               if nodeId /= b2_nullNode
               then
                  node := Self.m_nodes (nodeId)'Access;

                  if b2testOverlap (node.aabb, segmentAABB) /= False
                  then
                     -- Separating axis for segment (Gino, p80).
                     -- |dot(v, p1 - c)| > dot(|v|, h)
                     --
                     declare
                        c : constant b2Vec2 := getCenter  (node.aabb);
                        h : constant b2Vec2 := getExtents (node.aabb);

                        separation : constant Real := abs (   b2Dot (v,  p1 - c))
                                                            - b2Dot (abs_v, h);
                     begin
                        if not (separation > 0.0)
                        then
                           if isLeaf (node.all)
                           then
                              declare
                                 subInput : b2RayCastInput;
                                 Value    : Real;
                                 t        : b2Vec2;
                              begin
                                 subInput.p1          := input.p1;
                                 subInput.p2          := input.p2;
                                 subInput.maxFraction := maxFraction;

                                 Value := raycastCallback (callback, subInput, nodeId);

                                 if value = 0.0
                                 then
                                    return;     -- The client has terminated the ray cast.
                                 end if;

                                 if value > 0.0
                                 then
                                    -- Update segment bounding box.
                                    --
                                    maxFraction := value;
                                    t           := p1  +  maxFraction * (p2 - p1);

                                    segmentAABB.lowerBound := b2Min (p1, t);
                                    segmentAABB.upperBound := b2Max (p1, t);
                                 end if;
                              end;
                           else
                              push (stack, node.child1);
                              push (stack, node.child2);
                           end if;
                        end if;
                     end;
                  end if;
               end if;
            end;
         end loop;
      end;
   end raycast;




   --    Validate this tree. For testing.
   --
   --    void Validate() const;
   --
   --  void b2DynamicTree::Validate() const
   --  {
   --  #if defined(b2DEBUG)
   --    ValidateStructure(m_root);
   --    ValidateMetrics(m_root);
   --
   --    int32 freeCount = 0;
   --    int32 freeIndex = m_freeList;
   --    while (freeIndex != b2_nullNode)
   --    {
   --       b2Assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
   --       freeIndex = m_nodes[freeIndex].next;
   --       ++freeCount;
   --    }
   --
   --    b2Assert(GetHeight() == ComputeHeight());
   --
   --    b2Assert(m_nodeCount + freeCount == m_nodeCapacity);
   --  #endif
   --  }
   --

   procedure validate (Self : in b2DynamicTree)
   is
      use b2_Common;
   begin
      if b2DEBUG
      then
         Self.validateStructure (Self.m_root);
         Self.validateMetrics   (Self.m_root);

         declare
            freeCount : Natural := 0;
            freeIndex : Integer := Self.m_freeList;
         begin
            while freeIndex /= b2_nullNode
            loop
               pragma assert (            0 <= freeIndex
                              and freeIndex <  Self.m_nodeCapacity);

               freeIndex := Self.m_nodes (freeIndex).PorN.next;
               freeCount := freeCount + 1;
            end loop;

            pragma assert (Self.getHeight               = Self.computeHeight);
            pragma assert (Self.m_nodeCount + freeCount = Self.m_nodeCapacity);
         end;
      end if;
   end validate;




   --    Compute the height of the binary tree in O(N) time. Should not be
   --    called often.
   --
   --    int32 GetHeight() const;
   --
   --  int32 b2DynamicTree::GetHeight() const
   --  {
   --    if (m_root == b2_nullNode)
   --    {
   --       return 0;
   --    }
   --
   --    return m_nodes[m_root].height;
   --  }
   --

   function getHeight (Self : in b2DynamicTree) return Natural
   is
   begin
      if Self.m_root = b2_nullNode
      then
         return 0;
      end if;

      return Self.m_nodes (Self.m_root).height;
   end getHeight;




   --    Get the maximum balance of an node in the tree. The balance is the difference
   --    in height of the two children of a node.
   --
   --    int32 GetMaxBalance() const;
   --
   --  int32 b2DynamicTree::GetMaxBalance() const
   --  {
   --    int32 maxBalance = 0;
   --    for (int32 i = 0; i < m_nodeCapacity; ++i)
   --    {
   --       const b2TreeNode* node = m_nodes + i;
   --       if (node->height <= 1)
   --       {
   --          continue;
   --       }
   --
   --       b2Assert(node->IsLeaf() == false);
   --
   --       int32 child1 = node->child1;
   --       int32 child2 = node->child2;
   --       int32 balance = b2Abs(m_nodes[child2].height - m_nodes[child1].height);
   --       maxBalance = b2Max(maxBalance, balance);
   --    }
   --
   --    return maxBalance;
   --  }
   --

   function getMaxBalance (Self : in b2DynamicTree) return Natural
   is
      maxBalance :        Natural   := 0;
      Node       : access b2TreeNode;
   begin
      for i in 0 .. Self.m_nodeCapacity - 1
      loop
         node := Self.m_nodes (i)'Access;

         if not (node.height <= 1)
         then
            pragma assert (isLeaf (node.all) = False);

            declare
               child1  : constant Natural := node.child1;
               child2  : constant Natural := node.child2;
               balance : constant Natural := abs (  Self.m_nodes (child2).height
                                                  - Self.m_nodes (child1).height);
            begin
               maxBalance := Natural'max (maxBalance, balance);
            end;
         end if;
      end loop;

      return maxBalance;
   end getMaxBalance;




   --    Get the ratio of the sum of the node areas to the root area.
   --
   --    float GetAreaRatio() const;
   --
   --  float b2DynamicTree::GetAreaRatio() const
   --  {
   --    if (m_root == b2_nullNode)
   --    {
   --       return 0.0f;
   --    }
   --
   --    const b2TreeNode* root = m_nodes + m_root;
   --    float rootArea = root->aabb.GetPerimeter();
   --
   --    float totalArea = 0.0f;
   --    for (int32 i = 0; i < m_nodeCapacity; ++i)
   --    {
   --       const b2TreeNode* node = m_nodes + i;
   --       if (node->height < 0)
   --       {
   --          // Free node in pool
   --          continue;
   --       }
   --
   --       totalArea += node->aabb.GetPerimeter();
   --    }
   --
   --    return totalArea / rootArea;
   --  }
   --

   function getAreaRatio (Self : in b2DynamicTree) return Real
   is
   begin
      if Self.m_root = b2_nullNode
      then
         return 0.0;
      end if;

      declare
         root      : constant access constant b2TreeNode := Self.m_nodes (Self.m_root)'Access;
         rootArea  : constant Real                       := getPerimeter (root.aabb);
         totalArea :                 Real                := 0.0;
         node      : access constant b2TreeNode;
      begin
         for i in 0 .. Self.m_nodeCapacity - 1
         loop
            node := Self.m_nodes (i)'Access;

            if not (node.height < 0)
            then
               -- Free node not in pool.
               --
               totalArea := totalArea + getPerimeter (node.aabb);
            end if;
         end loop;

         return totalArea / rootArea;
      end;
   end getAreaRatio;




   --    Build an optimal tree. Very expensive. For testing.
   --
   --    void RebuildBottomUp();
   --
   --  void b2DynamicTree::RebuildBottomUp()
   --  {
   --    int32* nodes = (int32*)b2Alloc(m_nodeCount * sizeof(int32));
   --    int32 count = 0;
   --
   --    // Build array of leaves. Free the rest.
   --    for (int32 i = 0; i < m_nodeCapacity; ++i)
   --    {
   --       if (m_nodes[i].height < 0)
   --       {
   --          // free node in pool
   --          continue;
   --       }
   --
   --       if (m_nodes[i].IsLeaf())
   --       {
   --          m_nodes[i].parent = b2_nullNode;
   --          nodes[count] = i;
   --          ++count;
   --       }
   --       else
   --       {
   --          FreeNode(i);
   --       }
   --    }
   --
   --    while (count > 1)
   --    {
   --       float minCost = b2_maxFloat;
   --       int32 iMin = -1, jMin = -1;
   --       for (int32 i = 0; i < count; ++i)
   --       {
   --          b2AABB aabbi = m_nodes[nodes[i]].aabb;
   --
   --          for (int32 j = i + 1; j < count; ++j)
   --          {
   --             b2AABB aabbj = m_nodes[nodes[j]].aabb;
   --             b2AABB b;
   --             b.Combine(aabbi, aabbj);
   --             float cost = b.GetPerimeter();
   --             if (cost < minCost)
   --             {
   --                iMin = i;
   --                jMin = j;
   --                minCost = cost;
   --             }
   --          }
   --       }
   --
   --       int32 index1 = nodes[iMin];
   --       int32 index2 = nodes[jMin];
   --       b2TreeNode* child1 = m_nodes + index1;
   --       b2TreeNode* child2 = m_nodes + index2;
   --
   --       int32 parentIndex = AllocateNode();
   --       b2TreeNode* parent = m_nodes + parentIndex;
   --       parent->child1 = index1;
   --       parent->child2 = index2;
   --       parent->height = 1 + b2Max(child1->height, child2->height);
   --       parent->aabb.Combine(child1->aabb, child2->aabb);
   --       parent->parent = b2_nullNode;
   --
   --       child1->parent = parentIndex;
   --       child2->parent = parentIndex;
   --
   --       nodes[jMin] = nodes[count-1];
   --       nodes[iMin] = parentIndex;
   --       --count;
   --    }
   --
   --    m_root = nodes[0];
   --    b2Free(nodes);
   --
   --    Validate();
   --  }
   --

   procedure rebuildBottomUp (Self : in out b2DynamicTree)
   is
      use b2_Types;

      nodes : Naturals (0 .. Self.m_nodeCount - 1);
      Count : Natural := 0;

   begin
      -- Build array of leaves. Free the rest.
      --
      for i in 0 .. Self.m_nodeCapacity - 1
      loop
         if not (Self.m_nodes (i).height < 0)     -- Don't free node in pool.
         then
            if isLeaf (Self.m_nodes (i))
            then
               Self.m_nodes (i).PorN.parent := b2_nullNode;
               nodes (count)               := i;
               count                       := count + 1;
            else
               Self.freeNode (i);
            end if;
         end if;
      end loop;


      while count > 1
      loop
         declare
            minCost : Real    := Real'Last;
            iMin    : Integer := -1;
            jMin    : Integer := -1;

            aabbi   : b2AABB;
            aabbj   : b2AABB;

            b       : b2AABB;
            cost    : Real;
         begin
            for i in 0 .. count - 1
            loop
               aabbi := Self.m_nodes (nodes (i)).aabb;

               for j in i + 1 .. count - 1
               loop
                  aabbj := Self.m_nodes (nodes (j)).aabb;

                  combine (b, aabbi, aabbj);

                  cost := getPerimeter (b);

                  if cost < minCost
                  then
                     iMin    := i;
                     jMin    := j;
                     minCost := cost;
                  end if;
               end loop;
            end loop;

            declare
               index1 : constant Natural := nodes (iMin);
               index2 : constant Natural := nodes (jMin);

               child1 : constant access b2TreeNode := Self.m_nodes (index1)'Access;
               child2 : constant access b2TreeNode := Self.m_nodes (index2)'Access;

               parentIndex : constant        Natural    := Self.allocateNode;
               parent      : constant access b2TreeNode := Self.m_nodes (parentIndex)'Access;
            begin
               parent.child1 := index1;
               parent.child2 := index2;
               parent.height := 1 + Integer'max (child1.height,
                                                 child2.height);
               combine (parent.aabb, child1.aabb,
                                     child2.aabb);
               parent.PorN.parent := b2_nullNode;

               child1.PorN.parent := parentIndex;
               child2.PorN.parent := parentIndex;

               nodes (jMin)  := nodes (count - 1);
               nodes (iMin)  := parentIndex;
               count         := count - 1;
            end;
         end;
      end loop;

      Self.m_root := nodes (0);
      Self.validate;
   end rebuildBottomUp;




   --    Shift the world origin. Useful for large worlds.
   --    The shift formula is: position -= newOrigin
   --    @param newOrigin the new origin with respect to the old origin
   --
   --    void ShiftOrigin(const b2Vec2& newOrigin);
   --
   --  void b2DynamicTree::ShiftOrigin(const b2Vec2& newOrigin)
   --  {
   --    // Build array of leaves. Free the rest.
   --    for (int32 i = 0; i < m_nodeCapacity; ++i)
   --    {
   --       m_nodes[i].aabb.lowerBound -= newOrigin;
   --       m_nodes[i].aabb.upperBound -= newOrigin;
   --    }
   --  }
   --

   procedure shiftOrigin (Self : in out b2DynamicTree;   newOrigin : in b2Vec2)
   is
   begin
      -- Build array of leaves. Free the rest.
      --
      for i in 0 .. Self.m_nodeCapacity - 1
      loop
         Self.m_nodes (i).aabb.lowerBound := @ - newOrigin;
         Self.m_nodes (i).aabb.upperBound := @ - newOrigin;
      end loop;
   end shiftOrigin;





   --    int32 AllocateNode();
   --
   --  // Allocate a node from the pool. Grow the pool if necessary.
   --  int32 b2DynamicTree::AllocateNode()
   --  {
   --    // Expand the node pool as needed.
   --    if (m_freeList == b2_nullNode)
   --    {
   --       b2Assert(m_nodeCount == m_nodeCapacity);
   --
   --       // The free list is empty. Rebuild a bigger pool.
   --       b2TreeNode* oldNodes = m_nodes;
   --       m_nodeCapacity *= 2;
   --       m_nodes = (b2TreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2TreeNode));
   --       memcpy(m_nodes, oldNodes, m_nodeCount * sizeof(b2TreeNode));
   --       b2Free(oldNodes);
   --
   --       // Build a linked list for the free list. The parent
   --       // pointer becomes the "next" pointer.
   --       for (int32 i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
   --       {
   --          m_nodes[i].next = i + 1;
   --          m_nodes[i].height = -1;
   --       }
   --       m_nodes[m_nodeCapacity-1].next = b2_nullNode;
   --       m_nodes[m_nodeCapacity-1].height = -1;
   --       m_freeList = m_nodeCount;
   --    }
   --
   --    // Peel a node off the free list.
   --    int32 nodeId = m_freeList;
   --    m_freeList = m_nodes[nodeId].next;
   --    m_nodes[nodeId].parent = b2_nullNode;
   --    m_nodes[nodeId].child1 = b2_nullNode;
   --    m_nodes[nodeId].child2 = b2_nullNode;
   --    m_nodes[nodeId].height = 0;
   --    m_nodes[nodeId].userData = nullptr;
   --    m_nodes[nodeId].moved = false;
   --    ++m_nodeCount;
   --    return nodeId;
   --  }
   --

   function allocateNode (Self : in out b2DynamicTree) return Natural
   is
      procedure free is new ada.Unchecked_Deallocation (b2TreeNodes, b2TreeNodes_ptr);

      oldNodes : b2TreeNodes_ptr;
   begin
      -- Expand the node pool as needed.
      --
     if Self.m_freeList = b2_nullNode
      then
         pragma assert (Self.m_nodeCount = Self.m_nodeCapacity);

         -- The free list is empty. Rebuild a bigger pool.
         --
         oldNodes            := Self.m_nodes;
         Self.m_nodeCapacity := Self.m_nodeCapacity * 2;
         Self.m_nodes        := new b2TreeNodes (0 .. Self.m_nodeCapacity - 1);

         --  memcpy (Self.m_nodes, oldNodes, Self.m_nodeCount * sizeof (b2TreeNode));
         Self.m_nodes (0 .. Self.m_nodeCount - 1) := oldNodes.all;
         free (oldNodes);

         -- Build a linked list for the free list. The parent
         -- pointer becomes the "next" pointer.
         --
         for i in Self.m_nodeCount .. Self.m_nodeCapacity - 2
         loop
            Self.m_nodes (i).PorN.next := i + 1;
            Self.m_nodes (i).height    := -1;
         end loop;

         Self.m_nodes (Self.m_nodeCapacity - 1).PorN.next := b2_nullNode;
         Self.m_nodes (Self.m_nodeCapacity - 1).height    := -1;

         Self.m_freeList := Self.m_nodeCount;
      end if;

      -- Peel a node off the free list.
      --
      declare
         nodeId : constant Integer := Self.m_freeList;
      begin
         Self.m_freeList                   := Self.m_nodes (nodeId).PorN.next;
         Self.m_nodes (nodeId).PorN.parent := b2_nullNode;
         Self.m_nodes (nodeId).child1      := b2_nullNode;
         Self.m_nodes (nodeId).child2      := b2_nullNode;
         Self.m_nodes (nodeId).height      := 0;
         Self.m_nodes (nodeId).userData    := system.null_Address;
         Self.m_nodes (nodeId).moved       := False;
         Self.m_nodeCount                  := Self.m_nodeCount + 1;

         return nodeId;
      end;
   end allocateNode;





   --    void FreeNode(int32 node);
   --
   --  // Return a node to the pool.
   --
   --  void b2DynamicTree::FreeNode (int32 nodeId)
   --  {
   --    b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
   --    b2Assert(0 < m_nodeCount);
   --    m_nodes[nodeId].next = m_freeList;
   --    m_nodes[nodeId].height = -1;
   --    m_freeList = nodeId;
   --    --m_nodeCount;
   --  }
   --

   procedure freeNode (Self : in out b2DynamicTree;   node : in Natural)
   is
      pragma assert (       0 <= node
                     and node < Self.m_nodeCapacity);

      pragma assert (0 < Self.m_nodeCount);
   begin
      Self.m_nodes (node).PorN.next := Self.m_freeList;
      Self.m_nodes (node).height    := -1;

      Self.m_freeList  := node;
      Self.m_nodeCount := Self.m_nodeCount - 1;
   end freeNode;




   --    void InsertLeaf(int32 node);
   --
   --  void b2DynamicTree::InsertLeaf (int32 leaf)
   --  {
   --    ++m_insertionCount;
   --
   --    if (m_root == b2_nullNode)
   --    {
   --       m_root = leaf;
   --       m_nodes[m_root].parent = b2_nullNode;
   --       return;
   --    }
   --
   --    // Find the best sibling for this node
   --    b2AABB leafAABB = m_nodes[leaf].aabb;
   --    int32 index = m_root;
   --    while (m_nodes[index].IsLeaf() == false)
   --    {
   --       int32 child1 = m_nodes[index].child1;
   --       int32 child2 = m_nodes[index].child2;
   --
   --       float area = m_nodes[index].aabb.GetPerimeter();
   --
   --       b2AABB combinedAABB;
   --       combinedAABB.Combine(m_nodes[index].aabb, leafAABB);
   --       float combinedArea = combinedAABB.GetPerimeter();
   --
   --       // Cost of creating a new parent for this node and the new leaf
   --       float cost = 2.0f * combinedArea;
   --
   --       // Minimum cost of pushing the leaf further down the tree
   --       float inheritanceCost = 2.0f * (combinedArea - area);
   --
   --       // Cost of descending into child1
   --       float cost1;
   --       if (m_nodes[child1].IsLeaf())
   --       {
   --          b2AABB aabb;
   --          aabb.Combine(leafAABB, m_nodes[child1].aabb);
   --          cost1 = aabb.GetPerimeter() + inheritanceCost;
   --       }
   --       else
   --       {
   --          b2AABB aabb;
   --          aabb.Combine(leafAABB, m_nodes[child1].aabb);
   --          float oldArea = m_nodes[child1].aabb.GetPerimeter();
   --          float newArea = aabb.GetPerimeter();
   --          cost1 = (newArea - oldArea) + inheritanceCost;
   --       }
   --
   --       // Cost of descending into child2
   --       float cost2;
   --       if (m_nodes[child2].IsLeaf())
   --       {
   --          b2AABB aabb;
   --          aabb.Combine(leafAABB, m_nodes[child2].aabb);
   --          cost2 = aabb.GetPerimeter() + inheritanceCost;
   --       }
   --       else
   --       {
   --          b2AABB aabb;
   --          aabb.Combine(leafAABB, m_nodes[child2].aabb);
   --          float oldArea = m_nodes[child2].aabb.GetPerimeter();
   --          float newArea = aabb.GetPerimeter();
   --          cost2 = newArea - oldArea + inheritanceCost;
   --       }
   --
   --       // Descend according to the minimum cost.
   --       if (cost < cost1 && cost < cost2)
   --       {
   --          break;
   --       }
   --
   --       // Descend
   --       if (cost1 < cost2)
   --       {
   --          index = child1;
   --       }
   --       else
   --       {
   --          index = child2;
   --       }
   --    }
   --
   --    int32 sibling = index;
   --
   --    // Create a new parent.
   --    int32 oldParent = m_nodes[sibling].parent;
   --    int32 newParent = AllocateNode();
   --    m_nodes[newParent].parent = oldParent;
   --    m_nodes[newParent].userData = nullptr;
   --    m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
   --    m_nodes[newParent].height = m_nodes[sibling].height + 1;
   --
   --    if (oldParent != b2_nullNode)
   --    {
   --       // The sibling was not the root.
   --       if (m_nodes[oldParent].child1 == sibling)
   --       {
   --          m_nodes[oldParent].child1 = newParent;
   --       }
   --       else
   --       {
   --          m_nodes[oldParent].child2 = newParent;
   --       }
   --
   --       m_nodes[newParent].child1 = sibling;
   --       m_nodes[newParent].child2 = leaf;
   --       m_nodes[sibling].parent = newParent;
   --       m_nodes[leaf].parent = newParent;
   --    }
   --    else
   --    {
   --       // The sibling was the root.
   --       m_nodes[newParent].child1 = sibling;
   --       m_nodes[newParent].child2 = leaf;
   --       m_nodes[sibling].parent = newParent;
   --       m_nodes[leaf].parent = newParent;
   --       m_root = newParent;
   --    }
   --
   --    // Walk back up the tree fixing heights and AABBs
   --    index = m_nodes[leaf].parent;
   --    while (index != b2_nullNode)
   --    {
   --       index = Balance(index);
   --
   --       int32 child1 = m_nodes[index].child1;
   --       int32 child2 = m_nodes[index].child2;
   --
   --       b2Assert(child1 != b2_nullNode);
   --       b2Assert(child2 != b2_nullNode);
   --
   --       m_nodes[index].height = 1 + b2Max(m_nodes[child1].height, m_nodes[child2].height);
   --       m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
   --
   --       index = m_nodes[index].parent;
   --    }
   --
   --    //Validate();
   --  }
   --

   procedure insertLeaf (Self : in out b2DynamicTree;   node : in Natural)
   is
      leaf : Natural renames node;

      leafAABB : b2AABB;
      index    : Natural;
   begin
      Self.m_insertionCount := Self.m_insertionCount + 1;

      if Self.m_root = b2_nullNode
      then
         Self.m_root := leaf;
         Self.m_nodes (Self.m_root).PorN.parent := b2_nullNode;

         return;
      end if;

      -- Find the best sibling for this node
      --
      leafAABB := Self.m_nodes (leaf).aabb;
      index    := Self.m_root;

      while isLeaf (Self.m_nodes (index)) = False
      loop
         declare
            child1 : constant Natural := Self.m_nodes (index).child1;
            child2 : constant Natural := Self.m_nodes (index).child2;

            area   : constant Real := getPerimeter (Self.m_nodes (index).aabb);

            combinedAABB : b2AABB;
            combinedArea : Real;

            cost            : Real;     -- Cost of creating a new parent for this node and the new leaf.
            inheritanceCost : Real;     -- Minimum cost of pushing the leaf further down the tree.
            cost1           : Real;     -- Cost of descending into child1.
         begin
            combine (combinedAABB,
                     Self.m_nodes (index).aabb,
                     leafAABB);

            combinedArea    := getPerimeter (combinedAABB);
            cost            := 2.0 * combinedArea;
            inheritanceCost := 2.0 * (combinedArea - area);


            declare
               oldArea,
               newArea : Real;
               aabb    : b2AABB;
            begin
               if isLeaf (Self.m_nodes (child1))
               then
                  combine (aabb,
                           leafAABB,
                           Self.m_nodes (child1).aabb);

                  cost1 := getPerimeter (aabb) + inheritanceCost;
               else
                  combine (aabb,
                           leafAABB,
                           Self.m_nodes (child1).aabb);

                  oldArea := getPerimeter (Self.m_nodes (child1).aabb);
                  newArea := getPerimeter (aabb);
                  cost1   := (newArea - oldArea) + inheritanceCost;
               end if;
            end;

            -- Cost of descending into child2.
            declare
               oldArea,
               newArea : Real;
               cost2   : Real;
               aabb    : b2AABB;
            begin
               if isLeaf (Self.m_nodes (child2))
               then
                  combine (aabb,
                           leafAABB,
                           Self.m_nodes (child2).aabb);

                  cost2 := getPerimeter (aabb) + inheritanceCost;
               else
                  combine (aabb,
                           leafAABB,
                           Self.m_nodes (child2).aabb);

                  oldArea := getPerimeter (Self.m_nodes (child2).aabb);
                  newArea := getPerimeter (aabb);
                  cost2   := (newArea - oldArea) + inheritanceCost;
               end if;

               -- Descend according to the minimum cost.
               --
               if    cost < cost1
                 and cost < cost2
               then
                  exit;
               end if;

               -- Descend.
               --
               if cost1 < cost2 then index := child1;
               else                    index := child2;
               end if;
            end;
         end;
      end loop;


      declare
         sibling   : constant Natural := index;

         -- Create a new parent.
         --
         oldParent : constant Integer := Self.m_nodes (sibling).PorN.parent;
         newParent : constant Integer := Self.allocateNode;
      begin
         Self.m_nodes (newParent).PorN.parent := oldParent;
         Self.m_nodes (newParent).userData    := system.null_Address;
         Self.m_nodes (newParent).height      := Self.m_nodes (sibling).height + 1;

         combine (Self.m_nodes (newParent).aabb,
                  leafAABB,
                  Self.m_nodes (sibling).aabb);

         if oldParent /= b2_nullNode
         then
            -- The sibling was not the root.
            --
            if Self.m_nodes (oldParent).child1 = sibling
            then
               Self.m_nodes (oldParent).child1 := newParent;
            else
               Self.m_nodes (oldParent).child2 := newParent;
            end if;

            Self.m_nodes (newParent).child1    := sibling;
            Self.m_nodes (newParent).child2    := leaf;

            Self.m_nodes (sibling).PorN.parent := newParent;
            Self.m_nodes (leaf)   .PorN.parent := newParent;

         else
            -- The sibling was the root.
            --
            Self.m_nodes (newParent).child1    := sibling;
            Self.m_nodes (newParent).child2    := leaf;

            Self.m_nodes (sibling).PorN.parent := newParent;
            Self.m_nodes (leaf)   .PorN.parent := newParent;
            Self.m_root                        := newParent;
         end if;
      end;

      -- Walk back up the tree fixing heights and AABBs.
      --
      declare
         Child1,
         Child2 : Natural;
         Index  : Integer := Self.m_nodes (leaf).PorN.parent;
      begin
         while Index /= b2_nullNode
         loop
            index := Self.balance (index);

            child1 := Self.m_nodes (index).child1;
            child2 := Self.m_nodes (index).child2;

            pragma assert (child1 /= b2_nullNode);
            pragma assert (child2 /= b2_nullNode);

            Self.m_nodes (index).height := 1 + Integer'max (Self.m_nodes (child1).height,
                                                            Self.m_nodes (child2).height);
            combine (Self.m_nodes (index) .aabb,
                     Self.m_nodes (child1).aabb,
                     Self.m_nodes (child2).aabb);

            index := Self.m_nodes (index).PorN.parent;
         end loop;
      end;

      -- Self.validate
   end insertLeaf;





   --    void RemoveLeaf(int32 node);
   --
   --  void b2DynamicTree::RemoveLeaf(int32 leaf)
   --  {
   --    if (leaf == m_root)
   --    {
   --       m_root = b2_nullNode;
   --       return;
   --    }
   --
   --    int32 parent = m_nodes[leaf].parent;
   --    int32 grandParent = m_nodes[parent].parent;
   --    int32 sibling;
   --    if (m_nodes[parent].child1 == leaf)
   --    {
   --       sibling = m_nodes[parent].child2;
   --    }
   --    else
   --    {
   --       sibling = m_nodes[parent].child1;
   --    }
   --
   --    if (grandParent != b2_nullNode)
   --    {
   --       // Destroy parent and connect sibling to grandParent.
   --       if (m_nodes[grandParent].child1 == parent)
   --       {
   --          m_nodes[grandParent].child1 = sibling;
   --       }
   --       else
   --       {
   --          m_nodes[grandParent].child2 = sibling;
   --       }
   --       m_nodes[sibling].parent = grandParent;
   --       FreeNode(parent);
   --
   --       // Adjust ancestor bounds.
   --       int32 index = grandParent;
   --       while (index != b2_nullNode)
   --       {
   --          index = Balance(index);
   --
   --          int32 child1 = m_nodes[index].child1;
   --          int32 child2 = m_nodes[index].child2;
   --
   --          m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
   --          m_nodes[index].height = 1 + b2Max(m_nodes[child1].height, m_nodes[child2].height);
   --
   --          index = m_nodes[index].parent;
   --       }
   --    }
   --    else
   --    {
   --       m_root = sibling;
   --       m_nodes[sibling].parent = b2_nullNode;
   --       FreeNode(parent);
   --    }
   --
   --    //Validate();
   --  }
   --

   procedure removeLeaf (Self : in out b2DynamicTree;   node : in Natural)
   is
      leaf : Natural renames node;
   begin
      if leaf = Self.m_root
      then
         Self.m_root := b2_nullNode;
         return;
      end if;

      declare
         parent      : constant Natural := Self.m_nodes (leaf)  .PorN.parent;
         grandParent : constant Integer := Self.m_nodes (parent).PorN.parent;
         sibling     :          Natural;
      begin
         if Self.m_nodes (parent).child1 = leaf
         then
            sibling := Self.m_nodes (parent).child2;
         else
            sibling := Self.m_nodes (parent).child1;
         end if;

         if grandParent /= b2_nullNode
         then
            -- Destroy parent and connect sibling to grandParent.
            --
            if Self.m_nodes (grandParent).child1 = parent
            then
               Self.m_nodes (grandParent).child1 := sibling;
            else
               Self.m_nodes (grandParent).child2 := sibling;
            end if;

            Self.m_nodes (sibling).PorN.parent := grandParent;
            Self.freeNode (parent);

            -- Adjust ancestor bounds.
            --
            declare
               index  : Integer := grandParent;
               child1 : Natural;
               child2 : Natural;
            begin
               while index /= b2_nullNode
               loop
                  index := Self.balance (index);

                  child1 := Self.m_nodes (index).child1;
                  child2 := Self.m_nodes (index).child2;

                  combine (Self.m_nodes (index).aabb,
                           Self.m_nodes (child1).aabb,
                           Self.m_nodes (child2).aabb);

                  Self.m_nodes (index).height := 1 + Integer'max (Self.m_nodes (child1).height,
                                                                  Self.m_nodes (child2).height);
                  index := Self.m_nodes (index).PorN.parent;
               end loop;
            end;

         else
            Self.m_root                        := sibling;
            Self.m_nodes (sibling).PorN.parent := b2_nullNode;

            Self.freeNode (parent);
         end if;
      end;

      -- Self.validate;
   end removeLeaf;





   --    int32 Balance(int32 index);
   --
   --  // Perform a left or right rotation if node A is imbalanced.
   --  // Returns the new root index.
   --
   --  int32 b2DynamicTree::Balance(int32 iA)
   --  {
   --    b2Assert(iA != b2_nullNode);
   --
   --    b2TreeNode* A = m_nodes + iA;
   --    if (A->IsLeaf() || A->height < 2)
   --    {
   --       return iA;
   --    }
   --
   --    int32 iB = A->child1;
   --    int32 iC = A->child2;
   --    b2Assert(0 <= iB && iB < m_nodeCapacity);
   --    b2Assert(0 <= iC && iC < m_nodeCapacity);
   --
   --    b2TreeNode* B = m_nodes + iB;
   --    b2TreeNode* C = m_nodes + iC;
   --
   --    int32 balance = C->height - B->height;
   --
   --    // Rotate C up
   --    if (balance > 1)
   --    {
   --       int32 iF = C->child1;
   --       int32 iG = C->child2;
   --       b2TreeNode* F = m_nodes + iF;
   --       b2TreeNode* G = m_nodes + iG;
   --       b2Assert(0 <= iF && iF < m_nodeCapacity);
   --       b2Assert(0 <= iG && iG < m_nodeCapacity);
   --
   --       // Swap A and C
   --       C->child1 = iA;
   --       C->parent = A->parent;
   --       A->parent = iC;
   --
   --       // A's old parent should point to C
   --       if (C->parent != b2_nullNode)
   --       {
   --          if (m_nodes[C->parent].child1 == iA)
   --          {
   --             m_nodes[C->parent].child1 = iC;
   --          }
   --          else
   --          {
   --             b2Assert(m_nodes[C->parent].child2 == iA);
   --             m_nodes[C->parent].child2 = iC;
   --          }
   --       }
   --       else
   --       {
   --          m_root = iC;
   --       }
   --
   --       // Rotate
   --       if (F->height > G->height)
   --       {
   --          C->child2 = iF;
   --          A->child2 = iG;
   --          G->parent = iA;
   --          A->aabb.Combine(B->aabb, G->aabb);
   --          C->aabb.Combine(A->aabb, F->aabb);
   --
   --          A->height = 1 + b2Max(B->height, G->height);
   --          C->height = 1 + b2Max(A->height, F->height);
   --       }
   --       else
   --       {
   --          C->child2 = iG;
   --          A->child2 = iF;
   --          F->parent = iA;
   --          A->aabb.Combine(B->aabb, F->aabb);
   --          C->aabb.Combine(A->aabb, G->aabb);
   --
   --          A->height = 1 + b2Max(B->height, F->height);
   --          C->height = 1 + b2Max(A->height, G->height);
   --       }
   --
   --       return iC;
   --    }
   --
   --    // Rotate B up
   --    if (balance < -1)
   --    {
   --       int32 iD = B->child1;
   --       int32 iE = B->child2;
   --       b2TreeNode* D = m_nodes + iD;
   --       b2TreeNode* E = m_nodes + iE;
   --       b2Assert(0 <= iD && iD < m_nodeCapacity);
   --       b2Assert(0 <= iE && iE < m_nodeCapacity);
   --
   --       // Swap A and B
   --       B->child1 = iA;
   --       B->parent = A->parent;
   --       A->parent = iB;
   --
   --       // A's old parent should point to B
   --       if (B->parent != b2_nullNode)
   --       {
   --          if (m_nodes[B->parent].child1 == iA)
   --          {
   --             m_nodes[B->parent].child1 = iB;
   --          }
   --          else
   --          {
   --             b2Assert(m_nodes[B->parent].child2 == iA);
   --             m_nodes[B->parent].child2 = iB;
   --          }
   --       }
   --       else
   --       {
   --          m_root = iB;
   --       }
   --
   --       // Rotate
   --       if (D->height > E->height)
   --       {
   --          B->child2 = iD;
   --          A->child1 = iE;
   --          E->parent = iA;
   --          A->aabb.Combine(C->aabb, E->aabb);
   --          B->aabb.Combine(A->aabb, D->aabb);
   --
   --          A->height = 1 + b2Max(C->height, E->height);
   --          B->height = 1 + b2Max(A->height, D->height);
   --       }
   --       else
   --       {
   --          B->child2 = iE;
   --          A->child1 = iD;
   --          D->parent = iA;
   --          A->aabb.Combine(C->aabb, D->aabb);
   --          B->aabb.Combine(A->aabb, E->aabb);
   --
   --          A->height = 1 + b2Max(C->height, D->height);
   --          B->height = 1 + b2Max(A->height, E->height);
   --       }
   --
   --       return iB;
   --    }
   --
   --    return iA;
   --  }
   --

   function balance (Self : in out b2DynamicTree;   Index : in Natural) return Natural
   is
      iA      : Natural renames Index;
      iB, iC  : Natural;
      balance : Integer;

      pragma assert (iA /= b2_nullNode);

      A      : constant access b2TreeNode := Self.m_nodes (iA)'Access;
      B, C   :          access b2TreeNode;

   begin
      if   isLeaf (A.all)
        or A.height < 2
      then
         return iA;
      end if;


      iB := A.child1;
      iC := A.child2;

      pragma assert (0 <= iB  and  iB < Self.m_nodeCapacity);
      pragma assert (0 <= iC  and  iC < Self.m_nodeCapacity);

      B := Self.m_nodes (iB)'Access;
      C := Self.m_nodes (iC)'Access;

      balance := C.height - B.height;

      -- Rotate C up.
      --
      if balance > 1
      then
         declare
            indexF : constant Natural := C.child1;
            indexG : constant Natural := C.child2;

            F : constant access b2TreeNode := Self.m_nodes (indexF)'Access;
            G : constant access b2TreeNode := Self.m_nodes (indexG)'Access;

            pragma assert (0 <= indexF  and  indexF < Self.m_nodeCapacity);
            pragma assert (0 <= indexG  and  indexG < Self.m_nodeCapacity);

         begin
            -- Swap A and C.
            --
            C.child1      := iA;
            C.PorN.parent := A.PorN.parent;
            A.PorN.parent := iC;

            -- A's old parent should point to C.
            --
            if C.PorN.parent /= b2_nullNode
            then
               if Self.m_nodes (C.PorN.parent).child1 = iA
               then
                  Self.m_nodes (C.PorN.parent).child1 := iC;
               else
                  pragma assert (Self.m_nodes (C.PorN.parent).child2 = iA);
                  Self.m_nodes (C.PorN.parent).child2 := iC;
               end if;

            else
               Self.m_root := iC;
            end if;

            -- Rotate.
            --
            if F.height > G.height
            then
               C.child2 := indexF;
               A.child2 := indexG;
               G.PorN.parent := iA;

               combine (A.aabb, B.aabb, G.aabb);
               combine (C.aabb, A.aabb, F.aabb);

               A.height := 1 + Integer'max (B.height, G.height);
               C.height := 1 + Integer'max (A.height, F.height);

            else
               C.child2 := indexG;
               A.child2 := indexF;

               F.PorN.parent := iA;

               combine (A.aabb, B.aabb, F.aabb);
               combine (C.aabb, A.aabb, G.aabb);

               A.height := 1 + Integer'max (B.height, F.height);
               C.height := 1 + Integer'max (A.height, G.height);
            end if;

            return iC;
         end;
      end if;


      -- Rotate B up.
      --
      if balance < -1
      then
         declare
            iD : constant Natural := B.child1;
            iE : constant Natural := B.child2;

            D : constant access b2TreeNode := Self.m_nodes (iD)'Access;
            E : constant access b2TreeNode := Self.m_nodes (iE)'Access;

            pragma assert (0 <= iD  and  iD < Self.m_nodeCapacity);
            pragma assert (0 <= iE  and  iE < Self.m_nodeCapacity);

         begin
            -- Swap A and B.
            --
            B.child1      := iA;
            B.PorN.parent := A.PorN.parent;
            A.PorN.parent := iB;

            -- A's old parent should point to B.
            --
            if B.PorN.parent /= b2_nullNode
            then
               if Self.m_nodes (B.PorN.parent).child1 = iA
               then
                  Self.m_nodes (B.PorN.parent).child1 := iB;
               else
                  pragma assert (Self.m_nodes (B.PorN.parent).child2 = iA);
                  Self.m_nodes (B.PorN.parent).child2 := iB;
               end if;
            else
               Self.m_root := iB;
            end if;


            -- Rotate.
            --
            if D.height > E.height
            then
               B.child2 := iD;
               A.child1 := iE;

               E.PorN.parent := iA;

               combine (A.aabb, C.aabb, E.aabb);
               combine (B.aabb, A.aabb, D.aabb);

               A.height := 1 + Integer'max (C.height, E.height);
               B.height := 1 + Integer'max (A.height, D.height);

            else
               B.child2 := iE;
               A.child1 := iD;

               D.PorN.parent := iA;

               combine (A.aabb, C.aabb, D.aabb);
               combine (B.aabb, A.aabb, E.aabb);

               A.height := 1 + Integer'max (C.height, D.height);
               B.height := 1 + Integer'max (A.height, E.height);
            end if;

            return iB;
         end;
      end if;

      return iA;
   end balance;




   --    int32 ComputeHeight() const;
   --
   --  // Compute the height of a sub-tree.
   --
   --  int32 b2DynamicTree::ComputeHeight(int32 nodeId) const
   --  {
   --    b2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
   --    b2TreeNode* node = m_nodes + nodeId;
   --
   --    if (node->IsLeaf())
   --    {
   --       return 0;
   --    }
   --
   --    int32 height1 = ComputeHeight(node->child1);
   --    int32 height2 = ComputeHeight(node->child2);
   --    return 1 + b2Max(height1, height2);
   --  }
   --

   function computeHeight (Self : in b2DynamicTree;   nodeId : in Natural) return Natural
   is
      pragma assert (0 <= nodeId and nodeId < Self.m_nodeCapacity);

      node : constant access b2TreeNode := Self.m_nodes (nodeId)'Access;
   begin
      if isLeaf (node.all)
      then
         return 0;
      end if;

      declare
         height1 : constant Natural := Self.computeHeight (node.child1);
         height2 : constant Natural := Self.computeHeight (node.child2);
      begin
         return 1 + Integer'max (height1, height2);
      end;
   end computeHeight;




   --    int32 ComputeHeight(int32 nodeId) const;
   --
   --  int32 b2DynamicTree::ComputeHeight() const
   --  {
   --    int32 height = ComputeHeight(m_root);
   --    return height;
   --  }
   --

   function computeHeight (Self : in b2DynamicTree) return Natural
   is
      height : constant Natural := Self.computeHeight (Self.m_root);
   begin
      return height;
   end computeHeight;




   --    void ValidateStructure(int32 index) const;
   --
   --  void b2DynamicTree::ValidateStructure(int32 index) const
   --  {
   --    if (index == b2_nullNode)
   --    {
   --       return;
   --    }
   --
   --    if (index == m_root)
   --    {
   --       b2Assert(m_nodes[index].parent == b2_nullNode);
   --    }
   --
   --    const b2TreeNode* node = m_nodes + index;
   --
   --    int32 child1 = node->child1;
   --    int32 child2 = node->child2;
   --
   --    if (node->IsLeaf())
   --    {
   --       b2Assert(child1 == b2_nullNode);
   --       b2Assert(child2 == b2_nullNode);
   --       b2Assert(node->height == 0);
   --       return;
   --    }
   --
   --    b2Assert(0 <= child1 && child1 < m_nodeCapacity);
   --    b2Assert(0 <= child2 && child2 < m_nodeCapacity);
   --
   --    b2Assert(m_nodes[child1].parent == index);
   --    b2Assert(m_nodes[child2].parent == index);
   --
   --    ValidateStructure(child1);
   --    ValidateStructure(child2);
   --  }
   --

   procedure validateStructure (Self : in b2DynamicTree;   Index : in Integer)
   is
   begin
     if index = b2_nullNode
     then
        return;
     end if;

     if index = Self.m_root
     then
        pragma assert (Self.m_nodes (index).PorN.parent = b2_nullNode);
     end if;

      declare
         node   : constant access b2TreeNode := Self.m_nodes (index)'Access;

         child1 : constant Integer := node.child1;
         child2 : constant Integer := node.child2;
      begin
         if isLeaf (node.all)
         then
            pragma assert (child1 = b2_nullNode);
            pragma assert (child2 = b2_nullNode);
            pragma assert (node.height = 0);

            return;
         end if;

         pragma assert (0 <= child1  and  child1 < Self.m_nodeCapacity);
         pragma assert (0 <= child2  and  child2 < Self.m_nodeCapacity);

         pragma assert (Self.m_nodes (child1).PorN.parent = index);
         pragma assert (Self.m_nodes (child2).PorN.parent = index);

         Self.validateStructure (child1);
         Self.validateStructure (child2);
      end;
   end validateStructure;




   --    void ValidateMetrics(int32 index) const;
   --
   --
   --  void b2DynamicTree::ValidateMetrics(int32 index) const
   --  {
   --    if (index == b2_nullNode)
   --    {
   --       return;
   --    }
   --
   --    const b2TreeNode* node = m_nodes + index;
   --
   --    int32 child1 = node->child1;
   --    int32 child2 = node->child2;
   --
   --    if (node->IsLeaf())
   --    {
   --       b2Assert(child1 == b2_nullNode);
   --       b2Assert(child2 == b2_nullNode);
   --       b2Assert(node->height == 0);
   --       return;
   --    }
   --
   --    b2Assert(0 <= child1 && child1 < m_nodeCapacity);
   --    b2Assert(0 <= child2 && child2 < m_nodeCapacity);
   --
   --    int32 height1 = m_nodes[child1].height;
   --    int32 height2 = m_nodes[child2].height;
   --    int32 height;
   --    height = 1 + b2Max(height1, height2);
   --    b2Assert(node->height == height);
   --
   --    b2AABB aabb;
   --    aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
   --
   --    b2Assert(aabb.lowerBound == node->aabb.lowerBound);
   --    b2Assert(aabb.upperBound == node->aabb.upperBound);
   --
   --    ValidateMetrics(child1);
   --    ValidateMetrics(child2);
   --  }
   --

   procedure validateMetrics (Self : in b2DynamicTree;   Index : in Integer)
   is
   begin
      if index = b2_nullNode
      then
         return;
      end if;

      declare
         node   : constant access b2TreeNode := Self.m_nodes (index)'Access;

         child1 : constant Integer := node.child1;
         child2 : constant Integer := node.child2;
      begin
         if isLeaf (node.all)
         then
            pragma assert (child1 = b2_nullNode);
            pragma assert (child2 = b2_nullNode);

            pragma assert (node.height = 0);

           return;
         end if;

        pragma assert (0 <= child1 and child1 < Self.m_nodeCapacity);
        pragma assert (0 <= child2 and child2 < Self.m_nodeCapacity);

         declare
            height1 : constant Integer := Self.m_nodes (child1).height;
            height2 : constant Integer := Self.m_nodes (child2).height;

            height  : Integer;
            aabb    : b2AABB;
         begin
            height := 1 + Integer'max (height1, height2);
            pragma assert (node.height = height);

            combine (aabb, Self.m_nodes (child1).aabb,
                           Self.m_nodes (child2).aabb);

            pragma assert (aabb.lowerBound = node.aabb.lowerBound);
            pragma assert (aabb.upperBound = node.aabb.upperBound);

            Self.validateMetrics (child1);
            Self.validateMetrics (child2);
         end;
      end;
   end validateMetrics;


end box2d.b2_dynamic_Tree;
