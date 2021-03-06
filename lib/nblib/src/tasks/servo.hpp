#ifndef NBLIB_SERVO_HPP
#define NBLIB_SERVO_HPP

// Servos have a range of -128 to 127.
// Servo speed is a number between 0 and 255, slowest to fastest. Speed is the
//  maximum step size done each cycle.

#include <nblib.hpp>

#define SERVO_MIN_TIME Clock::microsToTick(600)
#define SERVO_MAX_TIME Clock::microsToTick(2400)
#define SERVO_UPDATE_DELAY Clock::millisToTick(20)

template <class Clock>
class Servo {
    bool _awake = false;
    int8_t _positionCurrent = 0;
    int8_t _positionGoal = 0;
    uint8_t _speed = 255;

protected:
    Servo() { };

public:
    void sleep() {
        _awake = false;
    }

    void wake() {
        _awake = true;
    }

    bool awake() const {
        return _awake;
    }

    void set(int8_t position, uint8_t speed = 255) {
        _positionGoal = position;
        _speed = speed;
        _awake = true;
    }

    int8_t position() const {
        return _positionCurrent;
    }

    uint8_t speed() const {
        return _speed;
    }

    uint16_t update() {
        if(_positionCurrent != _positionGoal) {
            int16_t step = _speed;
            int16_t diff = _positionCurrent - _positionGoal;
            diff = abs(diff);

            if(diff < step) {
                _positionCurrent = _positionGoal;
            } else {
                if(_positionGoal < _positionCurrent) {
                    step = -step;
                }

                _positionCurrent += step;
            }
        }

        uint16_t middle = (SERVO_MAX_TIME + SERVO_MIN_TIME) / 2;
        uint16_t range = SERVO_MAX_TIME - SERVO_MIN_TIME;
        uint16_t adjust = _positionCurrent * (range / 256);

        uint16_t ticks = middle + adjust;

        return ticks;
    }

    virtual void pulseStart() const = 0;
    virtual void pulseEnd() const = 0;
};

template <class Clock, class Pin>
struct ServoT : Servo<Clock> {
    ServoT() {
        static_assert(Pin::getHardwareType() == hw::HardwareType::pin, "ServoT requires a Pin");

        Pin::direction(Pin::Direction::Output);
        Pin::value(Pin::Value::Low);
    }

    void pulseStart() const override {
        Pin::value(Pin::Value::High);
    }

    void pulseEnd() const override {
        Pin::value(Pin::Value::Low);
    }
};

template <class Clock>
class ServoManager : public Task<Clock> {
    Servo<Clock>** _servos;
    const uint8_t _servoCount;
    int8_t index = 0;
    uint16_t start = 0;
    Servo<Clock>* _activeServo = nullptr;

public:
    template <uint8_t S>
    ServoManager(Servo<Clock>* (&servos)[S]) : _servos(servos), _servoCount(S) {
    }

private:
    void loop() override {
        if(index == 0) {
            start = Clock::getTicks16();
        }

        if(index < _servoCount) {
            _activeServo = _servos[index];

            if(_activeServo->awake()) {
                uint16_t pulseLength = _activeServo->update();

                atomic {
                    if(Clock::addInterrupt(end, this, pulseLength)) {
                        // _activeServo->pin() = Pin::Value::High;
                        _activeServo->pulseStart();
                        this->sleep();
                    }
                }
            }

            index += 1;
        } else {
            index = 0;

            uint16_t diff = Clock::getTicks16() - start;

            if(diff < SERVO_UPDATE_DELAY) {
                this->sleep(SERVO_UPDATE_DELAY - diff);
            }
        }
    }

    static void end(void* data) {
        ServoManager* self = reinterpret_cast<ServoManager*>(data);

        self->_activeServo->pulseEnd();

        self->wake();
    }
};

#endif
