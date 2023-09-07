// Credits : Giovanni Dicanio <giovanni.dicanio@gmail.com>

#pragma once

#include <crtdbg.h>       
#include <Windows.h>       

namespace PerformanceTools
{
    class Stopwatch
    {
    public:
        Stopwatch() noexcept;

        void Reset() noexcept;

        void Start() noexcept;

        void Stop() noexcept;

        double ElapsedMilliseconds() const noexcept;


    private:
        Stopwatch(const Stopwatch&) = delete;
        Stopwatch& operator=(const Stopwatch&) = delete;


    private:
        bool m_running;                     
        long long m_start;                 
        long long m_finish;                
        const long long m_frequency;       

        static long long Counter() noexcept;

        static long long Frequency() noexcept;

        double ElapsedMilliseconds(long long start, long long finish) const noexcept;
    };

    inline Stopwatch::Stopwatch() noexcept
        : m_running{ false }
        , m_start{ 0 }
        , m_finish{ 0 }
        , m_frequency{ Frequency() }
    {}

    inline void Stopwatch::Reset() noexcept
    {
        m_finish = m_start = 0;
        m_running = false;
    }

    inline void Stopwatch::Start() noexcept
    {
        m_running = true;
        m_finish = 0;

        m_start = Counter();
    }

    inline void Stopwatch::Stop() noexcept
    {
        m_finish = Counter();
        m_running = false;
    }

    inline double Stopwatch::ElapsedMilliseconds() const noexcept
    {
        if (m_running)
        {
            const long long current{ Counter() };
            return ElapsedMilliseconds(m_start, current);
        }

        return ElapsedMilliseconds(m_start, m_finish);
    }

    inline long long Stopwatch::Counter() noexcept
    {
        LARGE_INTEGER li;
        ::QueryPerformanceCounter(&li);
        return li.QuadPart;
    }

    inline long long Stopwatch::Frequency() noexcept
    {
        LARGE_INTEGER li;
        ::QueryPerformanceFrequency(&li);
        return li.QuadPart;
    }

    inline double Stopwatch::ElapsedMilliseconds(long long start, long long finish) const noexcept
    {
        _ASSERTE(start >= 0);
        _ASSERTE(finish >= 0);
        _ASSERTE(start <= finish);

        return ((finish - start) * 1000.0) / m_frequency;
    }
}   