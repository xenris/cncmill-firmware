/* STM32f103c8 linker script. */

_min_stack_size = 256;

ENTRY(_start)

MEMORY {
    RAM : org = 0x20000000, len = 20K
    FLASH : org = 0x8000000, len = 64K
}

__ram_start__ = ORIGIN(RAM);
__ram_size__ = LENGTH(RAM);
__ram_end__ = __ram_start__ + __ram_size__;
_estack = __ram_end__;

PROVIDE(_Stack_Limit = _estack - _min_stack_size);

SECTIONS {
    .text : {
        KEEP(*(.isr_vector))
        *(.text)
        *(.text.*)
    } > FLASH

    . = ALIGN(4);

    _etext = .;

    _sidata = _etext;

    .data : AT(_sidata) {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    PROVIDE(end = _ebss);
    PROVIDE(_end = _ebss);
    PROVIDE(_exit = _ebss);
    PROVIDE(_stackend = ORIGIN(RAM) + LENGTH(RAM) - _min_stack_size);

    ._usrstack : {
        . = ALIGN(4);
        _susrstack = . ;
        . = . + _min_stack_size ;
        . = ALIGN(4);
        _eusrstack = . ;
    } > RAM
}
