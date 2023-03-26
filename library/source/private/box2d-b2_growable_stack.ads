private
with
     ada.Containers.Vectors;


package box2d.b2_growable_Stack
is
   --  This is a growable LIFO stack with an initial capacity of N.
   --  If the stack size exceeds the initial capacity, the heap is used
   --  to increase the size of the stack.
   --
   --  template <typename T, int32 N>
   --  class b2GrowableStack
   --  {
   --  public:
   --    b2GrowableStack()
   --    {
   --       m_stack = m_array;
   --       m_count = 0;
   --       m_capacity = N;
   --    }
   --
   --    ~b2GrowableStack()
   --    {
   --       if (m_stack != m_array)
   --       {
   --          b2Free(m_stack);
   --          m_stack = nullptr;
   --       }
   --    }
   --
   --    void Push(const T& element)
   --    {
   --       if (m_count == m_capacity)
   --       {
   --          T* old = m_stack;
   --          m_capacity *= 2;
   --          m_stack = (T*)b2Alloc(m_capacity * sizeof(T));
   --          memcpy(m_stack, old, m_count * sizeof(T));
   --          if (old != m_array)
   --          {
   --             b2Free(old);
   --          }
   --       }
   --
   --       m_stack[m_count] = element;
   --       ++m_count;
   --    }
   --
   --    T Pop()
   --    {
   --       b2Assert(m_count > 0);
   --       --m_count;
   --       return m_stack[m_count];
   --    }
   --
   --    int32 GetCount()
   --    {
   --       return m_count;
   --    }
   --
   --  private:
   --    T* m_stack;
   --    T m_array[N];
   --    int32 m_count;
   --    int32 m_capacity;
   --  };
   --

   generic
      type Element_t is private;
      initial_Capacity : Positive;

   package b2GrowableStack
   is
      type Stack is limited private;

      function  to_Stack return Stack;

      procedure push (Self : in out Stack; E : in Element_T);
      function  pop  (Self : in out Stack) return Element_T;

      function  getCount    (Self : in     Stack)    return Natural;

      function  getCapacity (Self : in     Stack)    return Natural;
      procedure setCapacity (Self : in out Stack;   To : in Natural);


   private
      package Vectors is new ada.Containers.Vectors (Positive, Element_t);

      type Stack is limited
         record
            Elements : Vectors.Vector;
         end record;

   end b2GrowableStack;


end box2d.b2_growable_Stack;
