[Index](../index.hpp.md#index)

# Clock

Useful class for managing the passing of time.<br>
Required by TaskManager.

```c++
const uint64_t cpuFreq = 16000000;

using SystemTimer = nblib::hw::Timer1;

using Clock = nblib::Clock<SystemTimer, cpuFreq>;

uint64_t ticks = Clock::getTicks();
uint64_t millis = Clock::ticksToMillis(ticks);
```

Every cpu clock cycle is 1 / freq seconds. (62.5ns at 16MHz)<br>
Every 64 clock cycles is a tick. (4us at 16MHz)<br>
Every 2^64 ticks the clock overflows. (~2339769 years at 16Mhz)

If Clock is given an 8 bit timer (rather than 16 bit) each tick will be
4x longer.

## class Clock<class Timer, uint64_t cpuFreq, uint64_t maxCalls = 8\>

#### static constexpr uint64_t millisToTicks(uint64_t ms)
Convert milliseconds to ticks.

#### static constexpr uint64_t microsToTicks(uint64_t us)
Convert microseconds to ticks.

#### static constexpr uint64_t ticksToMillis(uint64_t ticks)
Convert ticks to milliseconds.

#### static constexpr uint64_t ticksToMicros(uint64_t ticks)
Convert ticks to microseconds.

#### static uint64_t getTicks()
Get the current value of the 64 bit tick counter.

#### static uint64_t getTicks_()
Non-atomic version of getTicks().

#### static bool delayedCall(Callback callback, void\* data, uint64_t delay)
Add a callback to call after delay ticks.<br>
Returns true if successfully added.

#### static bool delayedCall_(Callback callback, void\* data, uint64_t delay)
Non-atomic version of delayedCall().

#### static void delay<uint64_t ns\>()
Delays the cpu for the given number of nanoseconds.<br>
Should only be used for very short delays.<br>
Limited to 2 milliseconds.<br>
Rounds to the nearest possible delay. E.g. at 16MHz, delay<50\>() will
delay for 62.5 nanoseconds (1 cpu clock cycle).
