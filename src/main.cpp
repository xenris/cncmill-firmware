#include "main.hpp"

constexpr uint64_t cpuFreq = 16000000;

// using SpindleTimer = nblib::hw::Timer0;
// using SpindlePwm = SpindleTimer::OutputA;
using XYTimer = nblib::hw::Timer1;
using ZTimer = nblib::hw::Timer3;
using ETimer = nblib::hw::Timer4;
using XPwm = XYTimer::OutputA;
using YPwm = XYTimer::OutputB;
using ZPwm = ZTimer::OutputB;
using Usart = nblib::hw::Usart0;
// using SpindleOnOffPin = nblib::hw::PortB::Pin0;
using XStepPin = XPwm::Pin; // B1
using XDirectionPin = nblib::hw::PortB::Pin3;
using YStepPin = YPwm::Pin; // B2
using YDirectionPin = nblib::hw::PortB::Pin4;
using ZStepPin = ZPwm::Pin; // D2
using ZDirectionPin = nblib::hw::PortD::Pin3;
using EStopPin = nblib::hw::PortD::Pin7;
// using XLimitPin = nblib::hw::PortC::Pin0;
// using YLimitPin = nblib::hw::PortC::Pin1;
// using ZLimitPin = nblib::hw::PortC::Pin2;
// using ProbePin = nblib::hw::PortC::Pin3;
// using SpindlePWMPin = SpindlePwm::Pin; // D6
using LedPin = nblib::hw::PortB::Pin5;

using Pin = nblib::hw::Pin;

constexpr int32_t minXYZDelay = 1000;

enum class Direction : int8_t {
    forward = int8_t(Pin::Value::low),
    reverse = int8_t(Pin::Value::high),
};

struct Action {
    struct Axis {
        Direction direction;
        int32_t count;
        int32_t delay;

        Axis() : direction(Direction::forward), count(0), delay(0){
        }

        Axis(Direction direction, int32_t count, int32_t delay) : direction(direction), count(count), delay(delay) {
        }
    };

    Axis x;
    Axis y;
    Axis z;

    Action() {
    }

    Action(Axis x, Axis y, Axis z) : x(x), y(y), z(z) {
    }

    bool valid() const {
        return !((x.count && (x.delay < minXYZDelay))
            || (y.count && (y.delay < minXYZDelay))
            || (z.count && (z.delay < minXYZDelay)));
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
void xCallback(void*);
void yCallback(void*);
void zCallback(void*);
void xyzTimerOverflow(void*);
void xyzTimerMiddle(void*);
void usartRxComplete(nblib::Queue<uint8_t>* in);
void usartDataRegisterEmpty(nblib::Queue<uint8_t>* out);
void flush();
void pauseTimers();
void resumeTimers();
int32_t calculatePreviousDelay(int32_t def);

static nblib::Queue<uint8_t, 64> _sout;

static nblib::Queue<uint8_t, 64> _sin;

static nblib::Queue<Action, 16> _actionQueue;

Action _action;
int32_t _previousDelay = 0;

void (*handleSerial)();

int16_t cycle;
bool new_cycle;
int32_t xNext, yNext, zNext;

template <class Pwm>
void enablePwm();

template <class Pwm>
void disablePwm();

void main() {
    init();

    for(int i = 0; i < 1000; i++) {
        nblib::delay<cpuFreq, 1000000>();
    }

    nblib::interruptsEnable(true);

    while(true) {
        handleSerial();

        if(atomic(!_action.x.count && !_action.y.count && !_action.z.count) && !_actionQueue.empty()) {
            _previousDelay = calculatePreviousDelay(_previousDelay);

            _action = *_actionQueue.pop();

            pauseTimers();

            XYTimer::counter(0);
            ZTimer::counter(0);
            ETimer::counter(0);

            cycle = 0;
            new_cycle = true;
            xNext = yNext = zNext = 0;

            if(_action.x.count > 0) {
                XDirectionPin::output(Pin::Value(_action.x.direction));
                const int32_t first = _action.x.delay / 2 + _previousDelay / 2;
                if(first >= 65536) {
                    xNext = first;
                } else {
                    enablePwm<XPwm>();
                    XPwm::value(uint16_t(first));
                }
            }

            if(_action.y.count > 0) {
                YDirectionPin::output(Pin::Value(_action.y.direction));
                const int32_t first = _action.y.delay / 2 + _previousDelay / 2;
                if(first >= 65536) {
                    yNext = first;
                } else {
                    enablePwm<YPwm>();
                    YPwm::value(uint16_t(first));
                }
            }

            if(_action.z.count > 0) {
                ZDirectionPin::output(Pin::Value(_action.z.direction));
                const int32_t first = _action.z.delay / 2 + _previousDelay / 2;
                if(first >= 65536) {
                    zNext = first;
                } else {
                    enablePwm<ZPwm>();
                    ZPwm::value(uint16_t(first));
                }
            }

            resumeTimers();
        }

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

    XYTimer::waveform(XYTimer::Waveform::ctcIcr);
    XYTimer::Input::value(65536 - 1);

    ZTimer::waveform(ZTimer::Waveform::ctcIcr);
    ZTimer::Input::value(65536 - 1);

    ETimer::waveform(ETimer::Waveform::ctcIcr);
    ETimer::Input::value(65536 - 1);

    XPwm::setCallback(xCallback);
    YPwm::setCallback(yCallback);
    ZPwm::setCallback(zCallback);

    ETimer::OutputB::value(65536 - 1);
    ETimer::OutputB::setCallback(xyzTimerOverflow);
    ETimer::OutputB::intEnable(true);
    ETimer::OutputA::value(32768);
    ETimer::OutputA::setCallback(xyzTimerMiddle);
    ETimer::OutputA::intEnable(true);
    pauseTimers();
    XYTimer::clock(XYTimer::Clock::div1);
    ZTimer::clock(ZTimer::Clock::div1);
    ETimer::clock(ETimer::Clock::div1);
    XYTimer::counter(0);
    ZTimer::counter(0);
    ETimer::counter(0);
    resumeTimers();

    // SpindleOnOffPin::mode(Pin::Mode::output);
    XDirectionPin::mode(Pin::Mode::output);
    XStepPin::mode(Pin::Mode::output);
    YStepPin::mode(Pin::Mode::output);
    ZStepPin::mode(Pin::Mode::output);
    YDirectionPin::mode(Pin::Mode::output);
    ZDirectionPin::mode(Pin::Mode::output);
    EStopPin::mode(Pin::Mode::input);
    // XLimitPin::mode(Pin::Mode::input);
    // YLimitPin::mode(Pin::Mode::input);
    // ZLimitPin::mode(Pin::Mode::input);
    // ProbePin::mode(Pin::Mode::input);
    // SpindlePWMPin::mode(Pin::Mode::output);
    LedPin::mode(Pin::Mode::output);

    // A bug in 328pb requires this before Z pwm works.
    // https://www.avrfreaks.net/forum/atmega328pb-timer-34-output-compare-pwm-issue
    ZStepPin::output(Pin::Value::high);
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
            respond(0); // TODO Return firmware version number.
            handleSerial = serialHeader1;
            break;
        case 1:
            {
                const uint16_t n = _actionQueue.free();
                _sout.push(69);
                _sout.push(69);
                _sout.push(1);
                _sout.push(uint8_t(n));
                _sout.push(uint8_t(n >> 8));
                flush();
                handleSerial = serialHeader1;
            }
            break;
        case 2:
            handleSerial = serialAction;
            break;
        // TODO case 3 to send char strings.
        // TODO case 4 to reset estop, i.e. clear action queue and call resumeTimers().
        }
    }
}

// [x, y, z, dx, dy, dz]
void serialAction() {
    if(_sin.size() >= (6 * 4)) {
        uint8_t buffer[6 * 4];

        for(uint8_t& b : buffer) {
            b = *_sin.pop();
        }

        const int32_t a = ((int32_t*)buffer)[0];
        const int32_t b = ((int32_t*)buffer)[1];
        const int32_t c = ((int32_t*)buffer)[2];
        const int32_t d = ((int32_t*)buffer)[3];
        const int32_t e = ((int32_t*)buffer)[4];
        const int32_t f = ((int32_t*)buffer)[5];

        const Action::Axis xAxis((a > 0) ? Direction::reverse : Direction::forward, abs(a), d);
        const Action::Axis yAxis((b > 0) ? Direction::forward : Direction::reverse, abs(b), e);
        const Action::Axis zAxis((c > 0) ? Direction::forward : Direction::reverse, abs(c), f);

        const Action action(xAxis, yAxis, zAxis);

        if(action.valid()) {
            _actionQueue.push(action);
            respond(2);
        } else {
            respond(20);
        }

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
            pauseTimers();
        }
    }
}

// TODO home/calibrate
void home() {
    // _homing = true;
}

int32_t getnow(uint16_t pwm) {
    int16_t my_cycle = cycle;
    if (!new_cycle && pwm < 32768) {
        my_cycle = cycle + 1;
    }
    return int32_t(my_cycle) * 65536 + pwm;
}

void xCallback(void*) {
    _action.x.count--;

    if(_action.x.count) {
        if (_action.x.delay >= 65536) {
            const int32_t now = getnow(XPwm::value());
            xNext = now + _action.x.delay;
            disablePwm<XPwm>();
        } else {
            XPwm::value(XPwm::value() + uint16_t(_action.x.delay));
        }
    } else {
        disablePwm<XPwm>();
    }
}

void yCallback(void*) {
    _action.y.count--;

    if(_action.y.count) {
        if (_action.y.delay >= 65536) {
            const int32_t now = getnow(YPwm::value());
            yNext = now + _action.y.delay;
            disablePwm<YPwm>();
        } else {
            YPwm::value(YPwm::value() + uint16_t(_action.y.delay));
        }
    } else {
        disablePwm<YPwm>();
    }
}

void zCallback(void*) {
    _action.z.count--;

    if(_action.z.count) {
        if (_action.z.delay >= 65536) {
            const int32_t now = getnow(ZPwm::value());
            zNext = now + _action.z.delay;
            disablePwm<ZPwm>();
        } else {
            ZPwm::value(ZPwm::value() + uint16_t(_action.z.delay));
        }
    } else {
        disablePwm<ZPwm>();
    }
}

void xyzTimerOverflow(void*) {
    cycle += 1;
    new_cycle = true;
    const int32_t now = int32_t(cycle) * 65536;
    if (xNext) {
        const int32_t remaining = xNext - now;
        if (remaining < 65536) {
            XPwm::value(uint16_t(remaining));
            xNext = 0;
            enablePwm<XPwm>();
        }
    }
    if (yNext) {
        const int32_t remaining = yNext - now;
        if (remaining < 65536) {
            YPwm::value(uint16_t(remaining));
            yNext = 0;
            enablePwm<YPwm>();
        }
    }
    if (zNext) {
        const int32_t remaining = zNext - now;
        if (remaining < 65536) {
            ZPwm::value(uint16_t(remaining));
            zNext = 0;
            enablePwm<ZPwm>();
        }
    }
}

void xyzTimerMiddle(void*) {
    new_cycle = false;
    const int32_t now = int32_t(cycle) * 65536 + 32768;
    if (xNext) {
        const int32_t remaining = xNext - now;
        if (remaining < 65536) {
            XPwm::value(uint16_t(remaining) + 32768);
            xNext = 0;
            enablePwm<XPwm>();
        }
    }
    if (yNext) {
        const int32_t remaining = yNext - now;
        if (remaining < 65536 - 1) {
            YPwm::value(uint16_t(remaining) + 32768);
            yNext = 0;
            enablePwm<YPwm>();
        }
    }
    if (zNext) {
        const int32_t remaining = zNext - now;
        if (remaining < 65536 - 1) {
            ZPwm::value(uint16_t(remaining) + 32768);
            zNext = 0;
            enablePwm<ZPwm>();
        }
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

// Also resets the timers' prescaler.
void pauseTimers() {
    block() {
        XYTimer::synchronizeMode(true);
        XYTimer::prescalerReset();
    }
}

void resumeTimers() {
    block() {
        XYTimer::synchronizeMode(false);
    }
}

int32_t calculatePreviousDelay(int32_t def) {
    int32_t previous = max<int32_t>();

    if(_action.x.delay != 0) {
        previous = min(previous, _action.x.delay);
    }

    if(_action.y.delay != 0) {
        previous = min(previous, _action.y.delay);
    }

    if(_action.z.delay != 0) {
        previous = min(previous, _action.z.delay);
    }

    if(previous != max<int32_t>()) {
        return previous;
    } else {
        return def;
    }
}

template <class Pwm>
void enablePwm() {
    block() {
        Pwm::mode(Pwm::Mode::toggle);
        Pwm::Pin::output(Pin::Value::high);
        Pwm::intFlagClear();
        Pwm::intEnable(true);
    }
}

template <class Pwm>
void disablePwm() {
    block() {
        Pwm::intEnable(false);
        Pwm::Pin::output(Pwm::Pin::input());
        Pwm::mode(Pwm::Mode::disconnected);
    }
}
