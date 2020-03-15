#!/bin/sh

# build.sh - 6

# Copyright (c) 2017 Henry Shepperd

# MIT License (https://opensource.org/licenses/MIT)

set -e

source ./build.config

if [[ "$mcu" =~ ^at.* ]]; then
    arch="avr"
    mmcu=$mcu
elif [[ "$mcu" =~ ^stm32f1.* ]]; then
    arch="arm"
    mmcu="cortex-m3"
fi

# Initialise tup.
if [[ ! -d ".tup" ]]; then
    tup init
fi

# Initialise git and submodule nblib.
# (For new projects.)
if [[ ! -d ".git" ]]; then
    rmdir --ignore-fail-on-non-empty lib/nblib/
    if [[ -d "lib/nblib" ]]; then
        mv lib/nblib lib/nblib.bck
        echo "!!!!!!!!!!!!!!!!!! lib/nblib/ renamed to lib/nblib.bck/       !!!!!!!!!!!!!!!!!!"
        echo "!!!!!!!!!!!!!!!!!! Delete lib/nblib.bck/ if you don't need it !!!!!!!!!!!!!!!!!!"
    fi
    git init
    git submodule add https://github.com/xenris/nblib.git lib/nblib
fi

# Make sure nblib is initialised.
# (For cloned projects.)
if [[ $(git submodule status "lib/nblib") =~ ^-.* ]]; then
    git submodule init lib/nblib
fi

# Make sure nblib is populated.
# (For cloned projects.)
if [[ ! -d "lib/nblib/src" ]]; then
    git submodule update lib/nblib
fi

echo "-I./lib/nblib/src" > .clang_complete
echo "-ffreestanding" >> .clang_complete
echo "-Dexternally_visible=used" >> .clang_complete
echo "-Dsignal=used" >> .clang_complete
echo "-D__${mcu}__" >> .clang_complete

elf="gen/firmware.elf"
hex="gen/firmware.hex"
bin="gen/firmware.bin"

# Iterate through args.
args=$*
for ((i = 0; i < ${#args}; i++)); do
    arg="${args:$i:1}"

    case "$arg" in
    'b')
        echo "---------------------------------"
        echo "Building"
        echo "---------------------------------"

        PATHS=$(find src/ -name '*.c' -o -name '*.cpp' -o -name '*.S')

        rm -f Tupresources

        echo "arch = $arch" >> Tupresources
        echo "mmcu = $mmcu" >> Tupresources
        echo "CFLAGS += -D__${mcu}__" >> Tupresources

        for p in $PATHS; do
            echo "SRC_FILES += $p" >> Tupresources
        done

        tup

        ;;
    'm')
        echo "---------------------------------"

        case $arch in
        "avr")
            avr-objdump -Pmem-usage $elf | tail -n +3

            ;;
        "arm")
            arm-none-eabi-size $elf

            ;;
        esac

        ;;
    'u')
        echo "---------------------------------"
        echo "Uploading"
        echo "---------------------------------"

        case $arch in
        "avr")
            avrdude -C +avrdude.conf -b $upload_baud -B $bitclock -p $mmcu -P $upload_port -c $programmer -e -U flash:w:$hex

            ;;
        "arm")
            st-flash --reset write $bin 0x08000000

            ;;
        esac

        ;;
    's')
        echo "---------------------------------"
        echo "Serial - Exit with ctrl+a -> ctrl+x"
        echo "---------------------------------"

        picocom --imap lfcrlf --omap crlf --baud $serial_baud $serial_port

        ;;
    'n')
        echo "---------------------------------"
        echo "Updating nblib"
        echo "---------------------------------"

        git submodule update --remote lib/nblib

        ;;
    'f')
        echo "---------------------------------"
        echo "Updating fuse bytes"
        echo "---------------------------------"

        echo "!!! Warning: Setting the wrong fuse values can make your avr unusable. !!!"
        echo "!!! Double check the values in \"fuses.config\". !!!"
        echo "!!! If you're sure you want to update the fuses type \"update fuses\" to continue. !!!"

        read line

        if [[ $line == "update fuses" ]]; then
            case $arch in
            "avr")
                avrdude -C +avrdude.conf -b $upload_baud -B $bitclock -p $mmcu -P $upload_port -c $programmer -U efuse:w:$efuse:m -U hfuse:w:$hfuse:m -U lfuse:w:$lfuse:m

                ;;
            "arm")
                echo "Fuse programming not available for arm processors"

                ;;
            esac
        else
            echo "Cancelled"
        fi


        ;;
    ' ')
        ;;
    *)
        echo "$arg is not an option"
    esac
done

if [[ $# -eq 0 ]]; then
    echo "b -> Build"
    echo "m -> Show memory usage"
    echo "u -> Upload"
    echo "s -> Serial communication"
    echo "n -> Update nblib"
    echo ""
    echo "e.g. ./build.sh bmus"
fi
