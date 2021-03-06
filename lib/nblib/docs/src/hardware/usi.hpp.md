[Index](../../index.hpp.md#index)

# Universal Serial Interfaces

```c++
// TODO
```

## class UsiN

#### type PinDo

#### type PinDi

#### type PinSda

#### type PinScl

#### type PinUsck

#### enum WireMode
* disabled
* threeWire
* twoWire
* twoWireOverflow

#### enum Clock
* none
* timer0CompareMatch
* externalPositiveEdge
* externalNegativeEdge

#### static HardwareType getHardwareType()

#### static void data(uint8_t d)

#### static uint8_t data()

#### static void startIntEnable(bool b)
Enable the start condition interrupt.

#### static void overflowIntEnable(bool b)
Enable the counter overflow interrupt.

#### static void wireMode(WireMode m)

#### static void clock(Clock c)

#### static void clockStrobe()
Increment counter and Shift the USI data register.

#### static void toggleClockPin()
Toggle the clock pin or increment the counter

#### static void startIntFlag()
Get the start condition interrupt flag's state.

#### static void startIntFlagClear()
Clear the start condition interrupt flag.

#### static void stopIntFlag()
Get the stop condition interrupt flag's state.

#### static void stopIntFlagClear()
Clear the stop condition interrupt flag.

#### static bool overflowIntFlag()
Get the counter overflow interrupt flag's state.

#### static void overflowIntFlagClear()
Clear the counter overflow interrupt flag.

#### static bool collisionFlag()
Get the data output collision flag's state.

#### static void counter(uint8_t c)

#### static uint8_t counter()

#### static void setStartCallback([Callback](../callback.hpp.md#callbackt--void-t)<T\> function, T\* data)
Set callback for start condition interrupt.

#### static void callStartCallback()

#### static void setOverflowCallback([Callback](../callback.hpp.md#callbackt--void-t)<T\> function, T\* data)

#### static void callOverflowCallback()
