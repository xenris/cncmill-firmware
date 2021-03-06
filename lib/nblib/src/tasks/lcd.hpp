#ifndef NBLIB_LCD_HPP
#define NBLIB_LCD_HPP

#include <nblib.hpp>

// \r = clear display and return curser to home position.
// \n = move to start of next line, wrapping to first line.
// \v = move curser to the position defined by the next two bytes.
//  e.g. \v\1\2 = move to start of second line.
// \a = clear current line and put curser at start.

#define CLEAR_DISPLAY 0x01
#define CURSER_HOME 0x02
#define CURSER_START_LINE_ONE 0x80
#define CURSER_START_LINE_TWO 0xC0
#define CURSER_LEFT 0x10
#define CURSER_RIGHT 0x14
#define ENTRY_MODE_LEFT 0x04
#define ENTRY_MODE_RIGHT 0x06
#define ENTRY_MODE_LEFT_SCROLL 0x05
#define ENTRY_MODE_RIGHT_SCROLL 0x07
#define SCROLL_LEFT 0x18
#define SCROLL_RIGHT 0x1C
#define DISPLAY_OFF 0x08
#define DISPLAY_ON 0x0C
#define DISPLAY_ON_CURSER 0x0E
#define DISPLAY_ON_CURSER_BLINK 0x0F
#define FUNCTION_FOUR_BITS_TWO_LINES 0x28
#define SET_DDRAM_ADDRESS 0x80

template <class Clock, class lcdout_t, class D4, class D5, class D6, class D7, class RW, class RS, class E>
class LCD : public Task<Clock> {
    struct Range {
        uint8_t min;
        uint8_t max;
    };

    struct Coord {
        uint8_t x;
        uint8_t y;
    };

    const Range memoryMap[2] = {
        {0x00, 0x0f},
        {0x40, 0x4f}};
    static constexpr uint8_t lineCount = 2;
    static constexpr uint8_t lineLength = 16;

    typedef void (LCD::*StateFunction)();

    bool firstRun = true;
    StateFunction state;
    bool outOfBounds = false;
    lcdout_t& lcdout;

public:
    LCD(lcdout_t& lcdout) : lcdout(lcdout) {
        static_assert(D4::getHardwareType() == HardwareType::pin, "LCD requires 7 Pins");
        static_assert(D5::getHardwareType() == HardwareType::pin, "LCD requires 7 Pins");
        static_assert(D6::getHardwareType() == HardwareType::pin, "LCD requires 7 Pins");
        static_assert(D7::getHardwareType() == HardwareType::pin, "LCD requires 7 Pins");
        static_assert(RW::getHardwareType() == HardwareType::pin, "LCD requires 7 Pins");
        static_assert(RS::getHardwareType() == HardwareType::pin, "LCD requires 7 Pins");
        static_assert(E::getHardwareType() == HardwareType::pin, "LCD requires 7 Pins");

        RW::direction(hw::Direction::Output);
        RS::direction(hw::Direction::Output);
        E::direction(hw::Direction::Output);

        RW::output(hw::Value::Low);
        RS::output(hw::Value::Low);
        E::output(hw::Value::Low);

        state = &LCD::init0;
    }

private:
    void loop() override {
        if(firstRun) {
            firstRun = false;

            this->sleep(Clock::millisToTicks(50));
        } else {
            if(busy()) {
                return;
            }

            (this->*state)();
        }
    }

    void run() {
        char byte = 0;
        char x = 0;
        char y = 0;

        if(lcdout.pop(&byte)) {
            switch(byte) {
            case '\r':
                sendByte(false, CLEAR_DISPLAY);
                outOfBounds = false;
                break;
            case '\n':
                newLine();
                outOfBounds = false;
                break;
            case '\v':
                if(lcdout.pop(&x) && lcdout.pop(&y)) {
                    setCurser(x - 1, y - 1);
                    outOfBounds = false;
                }
                break;
            case '\a':
                state = &LCD::clearCurrentLine;
                outOfBounds = false;
                break;
            default:
                if(!outOfBounds) {
                    writeCharacter(byte);
                }
            }
        } else {
            this->sleep(Clock::millisToTicks(20));
        }
    }

    void writeCharacter(uint8_t character) {
        uint8_t address = getCurserAddress();
        Coord coord = addressToCoord(address);

        if(coord.x == lineLength - 1) {
            outOfBounds = true;
            state = &LCD::stepBack;
        }

        sendByte(true, character);
    }

    void setCurser(uint8_t x, uint8_t y) {
        setCurserAddress(coordToAddress(x, y));
    }

    void setCurserAddress(uint8_t address) {
        sendByte(false, SET_DDRAM_ADDRESS | address);
    }

    void newLine(void) {
        uint8_t address = getCurserAddress();
        Coord coord = addressToCoord(address);

        uint8_t y = (coord.y + 1) % lineCount;

        setCurserAddress(memoryMap[y].min);
    }

    bool busy(void) {
        uint8_t data = getByte(false);

        return data & bv(7);
    }

    uint8_t getCurserAddress(void) {
        uint8_t data = getByte(false);

        return data & ~bv(7);
    }

    uint8_t coordToAddress(uint8_t x, uint8_t y) {
        y = clip(y, uint8_t(0), uint8_t(lineCount - 1));
        x = clip(x, uint8_t(0), uint8_t(lineLength - 1));

        return memoryMap[y].min + x;
    }

    Coord addressToCoord(uint8_t address) {
        Coord coord = {0, 0};

        for(uint8_t i = 0; i < lineCount; i++) {
            if((address >= memoryMap[i].min) && (address <= memoryMap[i].max)) {
                coord.x = (address - memoryMap[i].min);
                coord.y = i;
                break;
            }
        }

        return coord;
    }

    void clearCurrentLine() {
        uint8_t count = 255;
        uint8_t address = 0;

        if(count == 255) {
            address = getCurserAddress();
            Coord coord = addressToCoord(address);
            address = memoryMap[coord.y].min;
            setCurserAddress(address);
        } else if(count < lineLength) {
            sendByte(true, ' ');
        } else {
            state = &LCD::run;
            count = 255;
            setCurserAddress(address);
        }

        count++;
    }

    void stepBack() {
        sendByte(false, CURSER_LEFT);
        state = &LCD::run;
    }

    void sendByte(bool rs, uint8_t data) {
        uint8_t high = data >> 4;
        uint8_t low = data;
        sendNibble(rs, high);
        sendNibble(rs, low);
    }

    void sendNibble(bool rs, uint8_t data) {
        D4::direction(hw::Direction::Output);
        D5::direction(hw::Direction::Output);
        D6::direction(hw::Direction::Output);
        D7::direction(hw::Direction::Output);

        D4::output((data & bv(0)) ? hw::Value::High : hw::Value::Low);
        D5::output((data & bv(1)) ? hw::Value::High : hw::Value::Low);
        D6::output((data & bv(2)) ? hw::Value::High : hw::Value::Low);
        D7::output((data & bv(3)) ? hw::Value::High : hw::Value::Low);
        RW::output(hw::Value::Low);
        RS::output(rs ? hw::Value::High : hw::Value::Low);

        Clock::delay(1);
        E::output(hw::Value::High);
        Clock::delay(1);
        E::output(hw::Value::Low);
    }

    // XXX May need to wait 43us when reading data.
    uint8_t getByte(bool rs) {
        uint8_t data = 0;

        D4::direction(hw::Direction::Input);
        D5::direction(hw::Direction::Input);
        D6::direction(hw::Direction::Input);
        D7::direction(hw::Direction::Input);

        D4::output(hw::Value::Low);
        D5::output(hw::Value::Low);
        D6::output(hw::Value::Low);
        D7::output(hw::Value::Low);
        RW::output(hw::Value::High);
        RS::output(rs ? hw::Value::High : hw::Value::Low);

        Clock::delay(1);

        E::output(hw::Value::High);
        Clock::delay(1);

        data |= (D7::input() == hw::Value::High) ? (1 << 7) : 0;
        data |= (D6::input() == hw::Value::High) ? (1 << 6) : 0;
        data |= (D5::input() == hw::Value::High) ? (1 << 5) : 0;
        data |= (D4::input() == hw::Value::High) ? (1 << 4) : 0;

        E::output(hw::Value::Low);

        Clock::delay(1);

        E::output(hw::Value::High);
        Clock::delay(1);

        data |= (D7::input() == hw::Value::High) ? (1 << 3) : 0;
        data |= (D6::input() == hw::Value::High) ? (1 << 2) : 0;
        data |= (D5::input() == hw::Value::High) ? (1 << 1) : 0;
        data |= (D4::input() == hw::Value::High) ? (1 << 0) : 0;

        E::output(hw::Value::Low);

        return data;
    }

    void init0() {
        // Reset 1
        sendNibble(false, 0x3);
        state = &LCD::init1;
        this->sleep(Clock::millisToTicks(5));
    }

    void init1() {
        // Reset 2
        sendNibble(false, 0x3);
        state = &LCD::init2;
        this->sleep(Clock::millisToTicks(1));
    }

    void init2() {
        // Reset 3
        sendNibble(false, 0x3);
        state = &LCD::init3;
        this->sleep(Clock::millisToTicks(1));
    }

    void init3() {
        // initial 4 bit mode
        sendNibble(false, 0x2);
        state = &LCD::init4;
        this->sleep(Clock::millisToTicks(1));
    }

    void init4() {
        sendByte(false, FUNCTION_FOUR_BITS_TWO_LINES);
        state = &LCD::init5;
        this->sleep(Clock::millisToTicks(1));
    }

    void init5() {
        sendByte(false, DISPLAY_ON);
        state = &LCD::init6;
        this->sleep(Clock::millisToTicks(1));
    }

    void init6() {
        sendByte(false, CLEAR_DISPLAY);
        state = &LCD::init7;
        this->sleep(Clock::millisToTicks(4));
    }

    void init7() {
        sendByte(false, ENTRY_MODE_RIGHT);
        state = &LCD::run;
        this->sleep(Clock::millisToTicks(1));
    }
};

#endif
