with
     b2_Settings,
     ada.Calendar,
     Interfaces;


package b2_Timer
--
--  Timer for profiling. This has platform specific code and may
--  not work on every platform.
--
is
   use b2_Settings,
       Interfaces;



   --  class b2Timer
   --  {
   --  public:
   --
   --  private:
   --
   --  #if defined(_WIN32)
   --    double m_start;
   --    static double s_invFrequency;
   --  #elif defined(__linux__) || defined (__APPLE__)
   --    unsigned long long m_start_sec;
   --    unsigned long long m_start_usec;
   --  #endif
   --  };
   --

   type b2Timer is tagged
      record
         Start : ada.Calendar.Time;
      end record;


   --    Constructor
   --
   --    b2Timer();
   --
   function to_b2Timer return b2Timer;



   --    Reset the timer.
   --
   --    void Reset();
   --

   procedure reset (Self : out b2Timer);



   --    Get the time since construction or the last reset.
   --
   --    float GetMilliseconds() const;
   --

   function getMilliseconds (Self : in b2Timer) return Real;


end b2_Timer;
