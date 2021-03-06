/// [[Index]]

/// # {{Two Wire Serial Interfaces}}

#ifndef NBLIB_TWI_HPP

#include "isr.hpp"
#include "chip.hpp"
#include "hardwaretype.hpp"
#include "macros.hpp"
#include "system.hpp"
#include "callback.hpp"

#include "loopi"

#ifdef _I
    #define N _I
    #define TwiN CAT(Twi, N)
    #define TWI_N(A) CAT(CHIP_TWI_, N, _, A)
    #define _TWI_N(A) UNDERLINE(TWI, N, A)

    #if CAT(CHIP_TWI_, N)

//------------------------------------------------------------------

namespace nblib::hw {

/// ## class {{TwiN}}
struct TwiN {
    TwiN() = delete;
    TwiN& operator=(const TwiN&) = delete;
    TwiN(const TwiN&) = delete;

    /// #### enum {{TwiN::Prescaler}}
    /// * div1
    /// * div4
    /// * div16
    /// * div64
    enum class Prescaler {
        div1 = TWI_N(PRESCALER_1_ID),
        div4 = TWI_N(PRESCALER_4_ID),
        div16 = TWI_N(PRESCALER_16_ID),
        div64 = TWI_N(PRESCALER_64_ID),
    };

    /// #### enum {{TwiN::Status}}
    /// * startTransmitted
    /// * repeatedStartTransmitted
    /// * slawTransmittedAck
    /// * slawTransmittedNack
    /// * dataTransmittedAck
    /// * dataTransmittedNack
    /// * arbitrationLost
    /// * slarTransmittedAck
    /// * slarTransmittedNack
    /// * dataReceivedAck
    /// * dataReceivedNack
    /// * ownSlawReceivedAck
    /// * arbitrationLostOwnSlawAck
    /// * generalCallAddressReceivedAck
    /// * arbitrationLostGeneralCallAddressReceivedAck
    /// * prevAddressedOwnSlawDataReceivedAck
    /// * prevAddressedOwnSlawDataReceivedNack
    /// * prevAddressedGeneralCallDataReceivedAck
    /// * prevAddressedGeneralCallDataReceivedNack
    /// * stopOrRepeatedStartWhileAddressedAsSlave
    /// * ownSlarReceivedAck
    /// * arbitrationLostOwnSlarAck
    /// * dataInTwdrTransmittedAck
    /// * dataInTwdrTransmittedNack
    /// * lastDataTransmittedAck
    /// * noState
    /// * busError
    enum class Status : uint8_t {
        busError = 0x00,
        startTransmitted = 0x08,
        repeatedStartTransmitted = 0x10,
        slawTransmittedAck = 0x18,
        slawTransmittedNack = 0x20,
        dataTransmittedAck = 0x28,
        dataTransmittedNack = 0x30,
        arbitrationLost = 0x38,
        slarTransmittedAck = 0x40,
        slarTransmittedNack = 0x48,
        dataReceivedAck = 0x50,
        dataReceivedNack = 0x58,
        ownSlawReceivedAck = 0x60,
        arbitrationLostOwnSlawAck = 0x68,
        generalCallAddressReceivedAck = 0x70,
        arbitrationLostGeneralCallAddressReceivedAck = 0x78,
        prevAddressedOwnSlawDataReceivedAck = 0x80,
        prevAddressedOwnSlawDataReceivedNack = 0x88,
        prevAddressedGeneralCallDataReceivedAck = 0x90,
        prevAddressedGeneralCallDataReceivedNack = 0x98,
        stopOrRepeatedStartWhileAddressedAsSlave = 0xA0,
        ownSlarReceivedAck = 0xA8,
        arbitrationLostOwnSLARAck = 0xB0,
        dataInTwdrTransmittedAck = 0xB8,
        dataInTwdrTransmittedNack = 0xC0,
        lastDataTransmittedAck = 0xC8,
        noState = 0xF8,
    };

    /// #### static [[HardwareType]] getHardwareType()
    /// Get the type of hardware that this class represents.
    static constexpr HardwareType getHardwareType() {
        return HardwareType::twi;
    }

    /// #### static void enable(bool e)
    /// Enable/disable the Twi.
    static force_inline void enable(bool e) {
        setBit_(REG(TWI_N(ENABLE_REG)), TWI_N(ENABLE_BIT), e);
    }

    /// #### static void sendStart(bool intEnable = true)
    /// Send a start condition.
    static force_inline void sendStart(bool intEnable = true) {
        using T = REGTYPE(TWI_N(CONTROL_REG));

        const T enable = bv<T>(TWI_N(ENABLE_BIT));
        const T start = bv<T>(TWI_N(START_CONDITION_BIT));
        const T flagClear = bv<T>(TWI_N(INT_FLAG_BIT));
        const T interrupt = intEnable ? bv<T>(TWI_N(INT_ENABLE_BIT)) : 0;

        setReg_(REG(TWI_N(CONTROL_REG)), T(enable | start | flagClear | interrupt));
    }

    /// #### static void sendStop(bool intEnable = true)
    /// Send stop condition.
    static force_inline void sendStop() {
        using T = REGTYPE(TWI_N(CONTROL_REG));

        const T enable = bv<T>(TWI_N(ENABLE_BIT));
        const T stop = bv<T>(TWI_N(STOP_CONDITION_BIT));
        const T flagClear = bv<T>(TWI_N(INT_FLAG_BIT));

        setReg_(REG(TWI_N(CONTROL_REG)), T(enable | stop | flagClear));
    }

    /// #### static void sendAck(bool intEnable = true)
    /// Send acknowledge condition.
    static force_inline void sendAck(bool intEnable = true) {
        using T = REGTYPE(TWI_N(CONTROL_REG));

        const T enable = bv<T>(TWI_N(ENABLE_BIT));
        const T ack = bv<T>(TWI_N(ENABLE_ACK_BIT));
        const T flagClear = bv<T>(TWI_N(INT_FLAG_BIT));
        const T interrupt = intEnable ? bv<T>(TWI_N(INT_ENABLE_BIT)) : 0;

        setReg_(REG(TWI_N(CONTROL_REG)), T(enable | ack | flagClear | interrupt));
    }

    /// #### static void sendNack(bool intEnable = true)
    /// Send not acknowledge condition.
    static force_inline void sendNack(bool intEnable = true) {
        using T = REGTYPE(TWI_N(CONTROL_REG));

        const T enable = bv<T>(TWI_N(ENABLE_BIT));
        const T flagClear = bv<T>(TWI_N(INT_FLAG_BIT));
        const T interrupt = intEnable ? bv<T>(TWI_N(INT_ENABLE_BIT)) : 0;

        setReg_(REG(TWI_N(CONTROL_REG)), T(enable | flagClear | interrupt));
    }

    /// #### static void bitRate(uint8_t b)
    /// Set Twi bitRate.
    static force_inline void bitRate(uint8_t b) {
        setReg_(REG(TWI_N(BIT_RATE_REG)), b);
    }

    /// #### static bool intFlag()
    /// Returns true if the interrupt flag is set.
    static force_inline bool intFlag() {
        return getBit_(REG(TWI_N(INT_FLAG_REG)), TWI_N(INT_FLAG_BIT));
    }

    /// #### static bool active()
    /// Returns true if the twi hardware is busy.
    static force_inline bool active() {
        return getBit_(REG(TWI_N(CONTROL_REG)), TWI_N(ENABLE_BIT))
            && (getBit_(REG(TWI_N(CONTROL_REG)), TWI_N(START_CONDITION_BIT))
            || getBit_(REG(TWI_N(CONTROL_REG)), TWI_N(STOP_CONDITION_BIT)));
    }

    /// #### static bool writeCollisionFlag()
    /// Returns true if the write collision flag is set.
    static force_inline bool writeCollisionFlag() {
        return getBit_(REG(TWI_N(WRITE_COLLISION_FLAG_REG)), TWI_N(WRITE_COLLISION_FLAG_BIT));
    }

    /// #### static void intEnable(bool e)
    /// Enable/disable the Twi interrupt.
    static force_inline void intEnable(bool e) {
        setBit_(REG(TWI_N(INT_ENABLE_REG)), TWI_N(INT_ENABLE_BIT), e);
    }

    /// #### static void setCallback([[Callback]]<T\> function, T\* data)
    /// Set the callback for Twi interrupts.
    template <class T>
    static force_inline void setCallback(Callback<T> function, T* data = nullptr) {
        callback((Callback<void>)function, data);
    }

    /// #### static void callCallback()
    static force_inline void callCallback() {
        callback();
    }

    /// #### static [[TwiN::Status]] status()
    /// Get the Twi status.
    static force_inline Status status() {
        return Status(int(getReg_(REG(TWI_N(STATUS_REG))) & 0xF8));
    }

    /// #### static void prescaler([[TwiN::Prescaler]] p)
    /// Set the prescaler.
    static force_inline void prescaler(Prescaler p) {
        setBit_(REG(TWI_N(PRESCALER_BIT_0_REG)), TWI_N(PRESCALER_BIT_0_BIT), int(p) & 0x01);
        setBit_(REG(TWI_N(PRESCALER_BIT_1_REG)), TWI_N(PRESCALER_BIT_1_BIT), int(p) & 0x02);
    }

    /// #### static void push(uint8_t b)
    /// Put byte in data buffer.
    static force_inline void push(uint8_t b) {
        setReg_(REG(TWI_N(DATA_REG)), b);
    }

    /// #### static uint8_t pop()
    /// Get byte from data buffer.
    static force_inline uint8_t pop() {
        return getReg_(REG(TWI_N(DATA_REG)));
    }

    /// #### static void slaveAddress(uint8_t b)
    /// Set the address for transmitting and receiving as a slave.
    static force_inline void slaveAddress(uint8_t b) {
        setReg_(REG(TWI_N(SLAVE_ADDRESS_REG)), uint8_t(b & 0xfe));
    }

    /// #### static void slaveAddressMask(uint8_t b)
    /// Set the slave address mask.
    static force_inline void slaveAddressMask(uint8_t b) {
        setReg_(REG(TWI_N(SLAVE_ADDRESS_MASK_REG)), uint8_t(b & 0xfe));
    }

    /// #### static void generalCallRecognitionEnable(bool e)
    /// Enable/disable the recognition of a Twi general call.
    static force_inline void generalCallRecognitionEnable(bool e) {
        setBit_(REG(TWI_N(GEN_CALL_REC_ENABLE_REG)), TWI_N(GEN_CALL_REC_ENABLE_BIT), e);
    }

private:

    static force_inline void callback(Callback<void> function = nullptr, void* data = nullptr) {
        static Callback<void> f = nullptr;
        static void* d = nullptr;

        if(function == nullptr) {
            if(f != nullptr) {
                f(d);
            }
        } else {
            f = function;
            d = data;
        }
    }
};

ISR(TWI_N(INT_VECTOR)) {
    TwiN::callCallback();
}

#ifdef TEST

TEST(TwiN, getHardwareType) {
    ASSERT_EQ(TwiN::getHardwareType(), HardwareType::twi);
}

TEST(TwiN, enable) {
    TEST_REG_WRITE(TwiN::enable(true));
    TEST_REG_WRITE(TwiN::enable(false));
}

TEST(TwiN, sendStart) {
    TEST_REG_WRITE(TwiN::sendStart(true));
    TEST_REG_WRITE(TwiN::sendStart(false));
}

TEST(TwiN, sendStop) {
    TEST_REG_WRITE(TwiN::sendStop());
}

TEST(TwiN, sendAck) {
    TEST_REG_WRITE(TwiN::sendAck(true));
    TEST_REG_WRITE(TwiN::sendAck(false));
}

TEST(TwiN, sendNack) {
    TEST_REG_WRITE(TwiN::sendNack(true));
    TEST_REG_WRITE(TwiN::sendNack(false));
}

TEST(TwiN, bitRate) {
    TEST_REG_WRITE(TwiN::bitRate(0x12));
}

TEST(TwiN, intFlag) {
    TEST_REG_READ_WRITE(TwiN::intFlag());
}

TEST(TwiN, active) {
    TEST_REG_READ_WRITE(TwiN::active());
}

TEST(TwiN, writeCollisionFlag) {
    TEST_REG_READ_WRITE(TwiN::writeCollisionFlag());
}

TEST(TwiN, intEnable) {
    TEST_REG_WRITE(TwiN::intEnable(true));
    TEST_REG_WRITE(TwiN::intEnable(false));
}

TEST(TwiN, status) {
    TEST_REG_READ_WRITE(TwiN::status());
}

TEST(TwiN, prescaler) {
    TEST_REG_WRITE(TwiN::prescaler(TwiN::Prescaler::div1));
    TEST_REG_WRITE(TwiN::prescaler(TwiN::Prescaler::div4));
    TEST_REG_WRITE(TwiN::prescaler(TwiN::Prescaler::div16));
    TEST_REG_WRITE(TwiN::prescaler(TwiN::Prescaler::div64));
}

TEST(TwiN, push) {
    TEST_REG_WRITE(TwiN::push(0x12));
}

TEST(TwiN, pop) {
    TEST_REG_READ_WRITE(TwiN::pop());
}

TEST(TwiN, slaveAddress) {
    TEST_REG_WRITE(TwiN::slaveAddress(0x12));
}

TEST(TwiN, slaveAddressMask) {
    TEST_REG_WRITE(TwiN::slaveAddressMask(0x12));
}

TEST(TwiN, generalCallRecognitionEnable) {
    TEST_REG_WRITE(TwiN::generalCallRecognitionEnable(true));
    TEST_REG_WRITE(TwiN::generalCallRecognitionEnable(false));
}

#endif // TEST

} // nblib::hw

//--------------------------------------------------------

    #endif

    #undef N
    #undef TwiN
    #undef TWI_N
    #undef _TWI_N

    #include "twi.hpp"
#else
    #define NBLIB_TWI_HPP
#endif

#endif
