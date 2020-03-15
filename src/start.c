// Startup handler.

void main(void);

void _estack(void);
void _start(void) __attribute__((noreturn));

extern unsigned long _sidata;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;

void _start(void) {
    unsigned long* src;
    unsigned long* dst;

    src = &_sidata;
    dst = &_sdata;

    while(dst < &_edata) {
        *(dst++) = *(src++);
    }

    dst = &_sbss;

    while(dst < &_ebss) {
        *(dst++) = 0;
    }

    main();

    while(1);
}

void vector_default(void) {
}

void vector_1(void) __attribute__((weak, alias("vector_default")));
void vector_2(void) __attribute__((weak, alias("vector_default")));
void vector_3(void) __attribute__((weak, alias("vector_default")));
void vector_4(void) __attribute__((weak, alias("vector_default")));
void vector_5(void) __attribute__((weak, alias("vector_default")));
void vector_6(void) __attribute__((weak, alias("vector_default")));
void vector_7(void) __attribute__((weak, alias("vector_default")));
void vector_8(void) __attribute__((weak, alias("vector_default")));
void vector_9(void) __attribute__((weak, alias("vector_default")));
void vector_10(void) __attribute__((weak, alias("vector_default")));
void vector_11(void) __attribute__((weak, alias("vector_default")));
void vector_12(void) __attribute__((weak, alias("vector_default")));
void vector_13(void) __attribute__((weak, alias("vector_default")));
void vector_14(void) __attribute__((weak, alias("vector_default")));
void vector_15(void) __attribute__((weak, alias("vector_default")));
void vector_16(void) __attribute__((weak, alias("vector_default")));
void vector_17(void) __attribute__((weak, alias("vector_default")));
void vector_18(void) __attribute__((weak, alias("vector_default")));
void vector_19(void) __attribute__((weak, alias("vector_default")));
void vector_20(void) __attribute__((weak, alias("vector_default")));
void vector_21(void) __attribute__((weak, alias("vector_default")));
void vector_22(void) __attribute__((weak, alias("vector_default")));
void vector_23(void) __attribute__((weak, alias("vector_default")));
void vector_24(void) __attribute__((weak, alias("vector_default")));
void vector_25(void) __attribute__((weak, alias("vector_default")));
void vector_26(void) __attribute__((weak, alias("vector_default")));
void vector_27(void) __attribute__((weak, alias("vector_default")));
void vector_28(void) __attribute__((weak, alias("vector_default")));
void vector_29(void) __attribute__((weak, alias("vector_default")));
void vector_30(void) __attribute__((weak, alias("vector_default")));
void vector_31(void) __attribute__((weak, alias("vector_default")));
void vector_32(void) __attribute__((weak, alias("vector_default")));
void vector_33(void) __attribute__((weak, alias("vector_default")));
void vector_34(void) __attribute__((weak, alias("vector_default")));
void vector_35(void) __attribute__((weak, alias("vector_default")));
void vector_36(void) __attribute__((weak, alias("vector_default")));
void vector_37(void) __attribute__((weak, alias("vector_default")));
void vector_38(void) __attribute__((weak, alias("vector_default")));
void vector_39(void) __attribute__((weak, alias("vector_default")));
void vector_40(void) __attribute__((weak, alias("vector_default")));
void vector_41(void) __attribute__((weak, alias("vector_default")));
void vector_42(void) __attribute__((weak, alias("vector_default")));
void vector_43(void) __attribute__((weak, alias("vector_default")));
void vector_44(void) __attribute__((weak, alias("vector_default")));
void vector_45(void) __attribute__((weak, alias("vector_default")));
void vector_46(void) __attribute__((weak, alias("vector_default")));
void vector_47(void) __attribute__((weak, alias("vector_default")));
void vector_48(void) __attribute__((weak, alias("vector_default")));
void vector_49(void) __attribute__((weak, alias("vector_default")));

typedef void (*vector_f)(void);

static vector_f vectors[] __attribute__((section(".isr_vector"), used)) = {
    _estack,
    _start,
    vector_1,
    vector_2,
    vector_3,
    vector_4,
    vector_5,
    vector_6,
    vector_7,
    vector_8,
    vector_9,
    vector_10,
    vector_11,
    vector_12,
    vector_13,
    vector_14,
    vector_15,
    vector_16,
    vector_17,
    vector_18,
    vector_19,
    vector_20,
    vector_21,
    vector_22,
    vector_23,
    vector_24,
    vector_25,
    vector_26,
    vector_27,
    vector_28,
    vector_29,
    vector_30,
    vector_31,
    vector_32,
    vector_33,
    vector_34,
    vector_35,
    vector_36,
    vector_37,
    vector_38,
    vector_39,
    vector_40,
    vector_41,
    vector_42,
    vector_43,
    vector_44,
    vector_45,
    vector_46,
    vector_47,
    vector_48,
    vector_49,
};
