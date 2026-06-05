*** Settings ***
Documentation     Ф2 boot smoke: the F446RE micro-ROS firmware boots in Renode,
...               FreeRTOS starts, rclc_support_init runs, and the CPU does not fault.
Suite Setup       Setup
Suite Teardown    Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${ELF}            ${CURDIR}/../build/firmware_stm32.elf

*** Test Cases ***
Firmware Boots Without Faulting
    Execute Command           mach create "f446"
    Execute Command           machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
    Execute Command           sysbus LoadELF @${ELF}
    Start Emulation
    Execute Command           emulation RunFor "1.0"
    ${halted}=                Execute Command    cpu IsHalted
    Should Contain            ${halted}          False
    ${instr}=                 Execute Command    cpu ExecutedInstructions
    Should Not Contain        ${instr}           0x0000000000000000
