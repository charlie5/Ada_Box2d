with
     box2d.b2_Collision,
     box2d.b2_Math,
     --  box2d.b2_Settings,

     System;


package box2d.b2_dynamic_Tree
is
   use b2_Collision,
       b2_Math;
       --  b2_Settings;




   --  #define b2_nullNode (-1)

   b2_nullNode : constant := -1;





   --  A node in the dynamic tree. The client does not interact with this directly.
   --
   --  struct b2TreeNode
   --  {
   --    Enlarged AABB
   --    b2AABB aabb;
   --
   --    void* userData;
   --
   --    union
   --    {
   --       int32 parent;
   --       int32 next;
   --    };
   --
   --    int32 child1;
   --    int32 child2;
   --
   --    // leaf = 0, free node = -1
   --    int32 height;
   --
   --    bool moved;
   --  };
   --

   type Parent_or_Next (is_Parent : Boolean := False) is
      record
         case is_Parent
         is
            when True  =>   parent : Integer;
            when False =>   next   : Integer;
         end case;
      end record
     with  unchecked_Union;



   type b2TreeNode is
      record
         aabb     : b2AABB;             -- Enlarged AABB.
         userData : system.Address;
         PorN     : Parent_or_Next;

         child1,
         child2   : Integer;

         -- leaf = 0, free node = -1

         height   : Integer;
         moved    : Boolean;
      end record;



   --    bool b2TreeNode::IsLeaf() const
   --    {
   --       return child1 == b2_nullNode;
   --    }
   --

   function isLeaf (Self : in b2TreeNode) return Boolean;





   --  A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
   --  A dynamic tree arranges data in a binary tree to accelerate
   --  queries such as volume queries and ray casts. Leafs are proxies
   --  with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
   --  so that the proxy AABB is bigger than the client object. This allows the client
   --  object to move by small amounts without triggering a tree update.
   --  ///
   --  Nodes are pooled and relocatable, so we use node indices rather than pointers.
   --

   type b2DynamicTree is tagged private;




   --  public:



   --    Get proxy user data.
   --    @return the proxy user data or 0 if the id is invalid.
   --

   --  inline void* b2DynamicTree::GetUserData(int32 proxyId) const
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    return m_nodes[proxyId].userData;
   --  }
   --

   function getUserData (Self : in b2DynamicTree;   proxyId : Natural) return system.Address
     with inline;




   --    Constructing the tree initializes the node pool.
   --
   --    b2DynamicTree();
   --

   function to_b2DynamicTree return b2DynamicTree;




   --    Destroy the tree, freeing the node pool.
   --
   --    ~b2DynamicTree();
   --

   procedure destruct (Self : in out b2DynamicTree);




   --    Create a proxy. Provide a tight fitting AABB and a userData pointer.
   --
   --    int32 CreateProxy(const b2AABB& aabb, void* userData);
   --

   function createProxy (Self : in out b2DynamicTree;   aabb     : in b2AABB;
                                                        userData : in system.Address) return Natural;




   --    Destroy a proxy. This asserts if the id is invalid.
   --
   --    void DestroyProxy(int32 proxyId);
   --

   procedure destroyProxy (Self : in out b2DynamicTree;   proxyId : in Natural);




   --    Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
   --    then the proxy is removed from the tree and re-inserted. Otherwise
   --    the function returns immediately.
   --    @return true if the proxy was re-inserted.
   --
   --    bool MoveProxy(int32 proxyId, const b2AABB& aabb1, const b2Vec2& displacement);
   --

   function moveProxy (Self : in out b2DynamicTree;   proxyId      : in Natural;
                                                      aabb1        : in b2AABB;
                                                      displacement : in b2Vec2) return Boolean;




   --  inline bool b2DynamicTree::WasMoved(int32 proxyId) const
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    return m_nodes[proxyId].moved;
   --  }
   --
   --    bool WasMoved(int32 proxyId) const;
   --

   function wasMoved (Self : in b2DynamicTree;   proxyId : in Natural) return Boolean
     with inline;




   --    void ClearMoved(int32 proxyId);
   --
   --  inline void b2DynamicTree::ClearMoved(int32 proxyId)
   --  {
   --    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
   --    m_nodes[proxyId].moved = false;
   --  }
   --

   procedure clearMoved (Self : in out b2DynamicTree;   proxyId : in Natural)
     with inline;




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
     with inline;




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

   --  type Callback_t is access procedure;

   generic
      type Callback_t is private;

      with function queryCallback (Callback : in out Callback_t;
                                   nodeId   : in     Natural) return Boolean;

   procedure query (Self : in b2DynamicTree;   Callback : in out Callback_t;
                                               aabb     : in     b2AABB)
     with inline;





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

   generic
      type Callback_t is private;

      with function raycastCallback (Callback : in out Callback_t;
                                     subInput : in     b2RayCastInput;
                                     nodeId   : in     Natural) return Real;

   procedure raycast (Self : in b2DynamicTree;   Callback : in out Callback_t;
                                                 input    : in     b2RayCastInput)
     with inline;




   --    Validate this tree. For testing.
   --
   --    void Validate() const;
   --

   procedure validate (Self : in b2DynamicTree);




   --    Compute the height of the binary tree in O(N) time. Should not be
   --    called often.
   --
   --    int32 GetHeight() const;
   --

   function getHeight (Self : in b2DynamicTree) return Natural;




   --    Get the maximum balance of an node in the tree. The balance is the difference
   --    in height of the two children of a node.
   --
   --    int32 GetMaxBalance() const;
   --

   function getMaxBalance (Self : in b2DynamicTree) return Natural;




   --    Get the ratio of the sum of the node areas to the root area.
   --
   --    float GetAreaRatio() const;
   --

   function getAreaRatio (Self : in b2DynamicTree) return Real;




   --    Build an optimal tree. Very expensive. For testing.
   --
   --    void RebuildBottomUp();
   --

   procedure rebuildBottomUp (Self : in out b2DynamicTree);




   --    Shift the world origin. Useful for large worlds.
   --    The shift formula is: position -= newOrigin
   --    @param newOrigin the new origin with respect to the old origin
   --
   --    void ShiftOrigin(const b2Vec2& newOrigin);

   procedure shiftOrigin (Self : in out b2DynamicTree;   newOrigin : in b2Vec2);



private

   --  class b2DynamicTree
   --  {
   --  public:
   --
   --  private:
   --
   --    int32 m_root;
   --
   --    b2TreeNode* m_nodes;
   --    int32 m_nodeCount;
   --    int32 m_nodeCapacity;
   --
   --    int32 m_freeList;
   --
   --    int32 m_insertionCount;
   --  };
   --

   type b2TreeNodes     is array (Natural range <>) of aliased b2TreeNode;
   type b2TreeNodes_ptr is access all b2TreeNodes;

   type b2DynamicTree is tagged
      record
         m_root           : Integer;

         m_nodes          : b2TreeNodes_ptr;
         m_nodeCount      : Natural;
         m_nodeCapacity   : Natural;

         m_freeList       : Integer;
         m_insertionCount : Natural;
      end record;



   --  private:
   --


   --    int32 AllocateNode();

   function allocateNode (Self : in out b2DynamicTree) return Natural;





   --    void FreeNode(int32 node);
   --

   procedure freeNode (Self : in out b2DynamicTree;   node : in Natural);




   --    void InsertLeaf(int32 node);
   --

   procedure insertLeaf (Self : in out b2DynamicTree;   node : in Natural);




   --    void RemoveLeaf(int32 node);
   --

   procedure removeLeaf (Self : in out b2DynamicTree;   node : in Natural);





   --    int32 Balance(int32 index);
   --

   function balance (Self : in out b2DynamicTree;   Index : in Natural) return Natural;




   --    int32 ComputeHeight() const;
   --

   function computeHeight (Self : in b2DynamicTree) return Natural;




   --    int32 ComputeHeight(int32 nodeId) const;
   --

   function computeHeight (Self : in b2DynamicTree;   nodeId : in Natural) return Natural;




   --    void ValidateStructure(int32 index) const;
   --

   procedure validateStructure (Self : in b2DynamicTree;   Index : in Integer);




   --    void ValidateMetrics(int32 index) const;
   --

   procedure validateMetrics (Self : in b2DynamicTree;   Index : in Integer);


end box2d.b2_dynamic_Tree;
