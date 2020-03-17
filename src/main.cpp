#include "main.hpp"

constexpr uint64_t cpuFreq = 16000000;

using SpindleTimer = nblib::hw::Timer0;
using SpindlePwm = SpindleTimer::OutputA;
using XYTimer = nblib::hw::Timer1;
using ZTimer = nblib::hw::Timer3;
using XPwm = XYTimer::OutputA;
using YPwm = XYTimer::OutputB;
using ZPwm = ZTimer::OutputB;
using Usart = nblib::hw::Usart0;
using SpindleOnOffPin = nblib::hw::PortB::Pin0;
using XStepPin = XPwm::Pin; // B1
using XDirectionPin = nblib::hw::PortB::Pin3;
using YStepPin = YPwm::Pin; // B2
using YDirectionPin = nblib::hw::PortB::Pin4;
using ZStepPin = ZPwm::Pin; // D2
using ZDirectionPin = nblib::hw::PortD::Pin3;
using EStopPin = nblib::hw::PortD::Pin7;
using XLimitPin = nblib::hw::PortC::Pin0;
using YLimitPin = nblib::hw::PortC::Pin1;
using ZLimitPin = nblib::hw::PortC::Pin2;
using ProbePin = nblib::hw::PortC::Pin3;
using SpindlePWMPin = SpindlePwm::Pin; // D6
using LedPin = nblib::hw::PortB::Pin5;

using Pin = nblib::hw::Pin;

struct Action {
    int16_t _xCount;
    int16_t _yCount;
    int16_t _zCount;

    uint16_t _xSpeed;
    uint16_t _ySpeed;
    uint16_t _zSpeed;

    Action() {
        _xCount = 0;
        _yCount = 0;
        _zCount = 0;

        _xSpeed = 0;
        _ySpeed = 0;
        _zSpeed = 0;
    }

    Action(int16_t x, int16_t y, int16_t z, uint16_t sx, uint16_t sy, uint16_t sz) {
        _xCount = x;
        _yCount = y;
        _zCount = z;

        _xSpeed = sx;
        _ySpeed = sy;
        _zSpeed = sz;
    }
};

// constexpr float axisStepSize = float(1.25e-3); // mm

void init();
void serialHeader1();
void serialHeader2();
void serialId();
void serialAction();
void respond(uint8_t id);
void handleEStop();
void home();
void xCallback(void*);
void yCallback(void*);
void zCallback(void*);
void startNextAction();
void xyzTimerMiddle(void*);
void xyzTimerOverflow(void*);
void usartRxComplete(nblib::Queue<uint8_t>* in);
void usartDataRegisterEmpty(nblib::Queue<uint8_t>* out);
void flush();
void pauseXyzTimers();
void resumeXyzTimers();

static nblib::Queue<uint8_t, 64> _sout;

static nblib::Queue<uint8_t, 64> _sin;

static nblib::Queue<Action, 16> _actionQueue;

Action _currentAction;

void (*handleSerial)();

void main() {
    init();

    nblib::interruptsEnable(true);

    while(true) {
        handleSerial();
        handleEStop();
    }
}

void init() {
    handleSerial = serialHeader1;

    Usart::setBaudRate<cpuFreq, 115200>();
    Usart::transmitterEnable(true);
    Usart::txCompleteIntEnable(false);
    Usart::dataRegisterEmptyIntEnable(false);
    Usart::setDeCallback(usartDataRegisterEmpty, _sout.ptr());
    Usart::receiverEnable(true);
    Usart::rxCompleteIntEnable(true);
    Usart::setRxCallback(usartRxComplete, _sin.ptr());

    XYTimer::waveform(XYTimer::Waveform::normal);
    ZTimer::waveform(ZTimer::Waveform::normal);
    ZTimer::setCallback(xyzTimerOverflow);
    ZTimer::intEnable(true);
    ZTimer::OutputA::value(32768);
    ZTimer::OutputA::setCallback(xyzTimerMiddle);
    ZTimer::OutputA::intEnable(true);
    pauseXyzTimers();
    XYTimer::clock(XYTimer::Clock::div8);
    ZTimer::clock(ZTimer::Clock::div8);
    XYTimer::counter(0);
    ZTimer::counter(0);
    resumeXyzTimers();

    SpindleOnOffPin::mode(Pin::Mode::output);
    XDirectionPin::mode(Pin::Mode::output);
    XStepPin::mode(Pin::Mode::output);
    YStepPin::mode(Pin::Mode::output);
    ZStepPin::mode(Pin::Mode::output);
    YDirectionPin::mode(Pin::Mode::output);
    ZDirectionPin::mode(Pin::Mode::output);
    EStopPin::mode(Pin::Mode::input);
    XLimitPin::mode(Pin::Mode::input);
    YLimitPin::mode(Pin::Mode::input);
    ZLimitPin::mode(Pin::Mode::input);
    ProbePin::mode(Pin::Mode::input);
    SpindlePWMPin::mode(Pin::Mode::output);
    LedPin::mode(Pin::Mode::output);

    // Looks like a bug in 328pb requires this before Z pwm works...?
    // https://www.avrfreaks.net/forum/atmega328pb-timer-34-output-compare-pwm-issue
    ZStepPin::output(Pin::Value::high);

    XPwm::setCallback(xCallback);
    YPwm::setCallback(yCallback);
    ZPwm::setCallback(zCallback);
    XPwm::intEnable(true);
    YPwm::intEnable(true);
    ZPwm::intEnable(true);
    XPwm::mode(XPwm::Mode::toggle);
    YPwm::mode(YPwm::Mode::toggle);
    ZPwm::mode(ZPwm::Mode::toggle);
}

void serialHeader1() {
    const Optional<uint8_t> d = _sin.pop();

    if(d) {
        if(*d == 69) {
            handleSerial = serialHeader2;
        }
    }
}

void serialHeader2() {
    const Optional<uint8_t> d = _sin.pop();

    if(d) {
        if(*d == 69) {
            handleSerial = serialId;
        } else {
            handleSerial = serialHeader1;
        }
    }
}

void serialId() {
    const Optional<uint8_t> d = _sin.pop();

    if(d) {
        switch(*d) {
        case 0:
            respond(0);
            handleSerial = serialHeader1;
            break;
        case 1:
            // TODO Check if busy first.
            home();
            respond(1);
            handleSerial = serialHeader1;
            break;
        case 2:
            handleSerial = serialAction;
            break;
        }
    }
}

void serialAction() {
    if(_sin.size() >= (6 * 2)) {
        const int16_t x = (*_sin.pop() << 8) | *_sin.pop();
        const int16_t y = (*_sin.pop() << 8) | *_sin.pop();
        const int16_t z = (*_sin.pop() << 8) | *_sin.pop();
        const uint16_t sx = uint16_t((*_sin.pop() << 8) | *_sin.pop());
        const uint16_t sy = uint16_t((*_sin.pop() << 8) | *_sin.pop());
        const uint16_t sz = uint16_t((*_sin.pop() << 8) | *_sin.pop());

        _actionQueue.push(Action(x, y, z, sx, sy, sz));

        startNextAction();

        respond(2);

        handleSerial = serialHeader1;
    }
}

void respond(uint8_t id) {
    _sout.push(69);
    _sout.push(69);
    _sout.push(id);
    flush();
}

void handleEStop() {
    block() {
        if(EStopPin::input() == Pin::Value::low) {
            pauseXyzTimers();
        } else {
            resumeXyzTimers();
        }
    }
}

// TODO
void home() {
    // _homing = true;
}

void xCallback(void*) {
    --_currentAction._xCount;

    // assert(_currentAction._xCount >= 0, "_currentAction._xCount is negative");

    if(_currentAction._xCount) {
        XPwm::value(XPwm::value() + _currentAction._xSpeed);
    } else {
        startNextAction();
    }
}

void yCallback(void*) {
    --_currentAction._yCount;

    if(_currentAction._yCount) {
        YPwm::value(YPwm::value() + _currentAction._ySpeed);
    } else {
        startNextAction();
    }
}

void zCallback(void*) {
    --_currentAction._zCount;

    if(_currentAction._zCount) {
        ZPwm::value(ZPwm::value() + _currentAction._zSpeed);
    } else {
        startNextAction();
    }
}

void startNextAction() {
    if((_currentAction._xCount == 0) && (_currentAction._yCount == 0) && (_currentAction._zCount == 0)) {
        const Optional<Action> nextAction = _actionQueue.pop();

        if(nextAction) {
            _currentAction = *nextAction;

            const uint16_t t = XYTimer::counter();

            if(_currentAction._xCount) {
                XPwm::value(t + _currentAction._xSpeed);
            }

            if(_currentAction._yCount) {
                YPwm::value(t + _currentAction._ySpeed);
            }

            if(_currentAction._zCount) {
                ZPwm::value(t + _currentAction._zSpeed);
            }

            XDirectionPin::output((_currentAction._xCount < 0) ? Pin::Value::low : Pin::Value::high);
            YDirectionPin::output((_currentAction._yCount < 0) ? Pin::Value::high : Pin::Value::low);
            ZDirectionPin::output((_currentAction._zCount < 0) ? Pin::Value::high : Pin::Value::low);

            _currentAction._xCount = abs(_currentAction._xCount);
            _currentAction._yCount = abs(_currentAction._yCount);
            _currentAction._zCount = abs(_currentAction._zCount);
        }
    }

}

void xyzTimerMiddle(void*) {
    if(_currentAction._xCount == 0) {
        XPwm::value(uint16_t(32768 - 1));
    }

    if(_currentAction._yCount == 0) {
        YPwm::value(uint16_t(32768 - 1));
    }

    if(_currentAction._zCount == 0) {
        ZPwm::value(uint16_t(32768 - 1));
    }
}

void xyzTimerOverflow(void*) {
    if(_currentAction._xCount == 0) {
        XPwm::value(uint16_t(-1));
    }

    if(_currentAction._yCount == 0) {
        YPwm::value(uint16_t(-1));
    }

    if(_currentAction._zCount == 0) {
        ZPwm::value(uint16_t(-1));
    }
}

void usartRxComplete(nblib::Queue<uint8_t>* in) {
    in->push(Usart::pop());
}

void usartDataRegisterEmpty(nblib::Queue<uint8_t>* out) {
    const Optional<uint8_t> d = out->pop();

    if(d) {
        Usart::push(*d);
    } else {
        Usart::dataRegisterEmptyIntEnable(false);
    }
}

void flush() {
    Usart::dataRegisterEmptyIntEnable(true);
}

void pauseXyzTimers() {
    block() {
        XYTimer::synchronizeMode(true);
        XYTimer::prescalerReset();
    }
}

void resumeXyzTimers() {
    block() {
        XYTimer::synchronizeMode(false);
    }
}
