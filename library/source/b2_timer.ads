with
     b2_Settings;


package b2_Timer
is
   procedure dummy;


   --
--  /// Timer for profiling. This has platform specific code and may
--  /// not work on every platform.
--  class B2_API b2Timer
--  {
--  public:
--
--    /// Constructor
--    b2Timer();
--
--    /// Reset the timer.
--    void Reset();
--
--    /// Get the time since construction or the last reset.
--    float GetMilliseconds() const;
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
end b2_Timer;
