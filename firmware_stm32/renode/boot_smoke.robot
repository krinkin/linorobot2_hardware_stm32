*** Settings ***
Documentation     F2 boot smoke (CI): the F446RE micro-ROS firmware boots in Renode, the
...               FreeRTOS scheduler actually starts and ticks, and a second task exists.
...               Asserting the scheduler TICKED (xTickCount > 0) catches startup hangs that
...               do not fault -- e.g. a configASSERT(uxPriority < configMAX_PRIORITIES) spin
...               that disables interrupts, where SysTick never fires (a plain "no HardFault"
...               check passes such a hang). The stronger byte-level ping check lives in
...               boot_smoke.sh / agent_bridge.sh (they need socat / docker, awkward in Robot).
Suite Setup       Setup
Suite Teardown    Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${ELF}            ${CURDIR}/../build/firmware_stm32.elf

*** Test Cases ***
Scheduler Starts And Ticks
    Execute Command           mach create "f446"
    Execute Command           machine LoadPlatformDescription @platforms/cpus/stm32f4.repl
    Execute Command           sysbus LoadELF @${ELF}
    # RunFor auto-starts, runs deterministically, then pauses. Do NOT also call
    # Start Emulation -- that leaves the machine running and RunFor errors out
    # ("already started"), aborting the rest of the script.
    Execute Command           emulation RunFor "2.5"

    # Instructions executed (machine is paused after RunFor, so it is "halted" -- that is
    # expected; ExecutedInstructions proves it actually ran).
    ${instr}=                 Execute Command    cpu ExecutedInstructions
    Should Not Contain        ${instr}           0x0000000000000000

    # FreeRTOS scheduler actually started: a second task exists and the tick advanced.
    ${ntasks_addr}=           Execute Command    sysbus GetSymbolAddress "uxCurrentNumberOfTasks"
    ${ntasks}=                Execute Command    sysbus ReadDoubleWord ${ntasks_addr}
    Should Not Contain        ${ntasks}          0x00000000

    ${tick_addr}=             Execute Command    sysbus GetSymbolAddress "xTickCount"
    ${ticks}=                 Execute Command    sysbus ReadDoubleWord ${tick_addr}
    Should Not Contain        ${ticks}           0x00000000
