*** Settings ***
Suite Setup       Setup
Suite Teardown    Teardown
Test Setup        Reset Emulation
Resource          ${RENODEKEYWORDS}

*** Keywords ***
Boot Firmware
    Execute Command         mach create "fan"
    Execute Command         machine LoadPlatformDescription @${CURDIR}/g0b1.repl
    Execute Command         sysbus LoadELF @${CURDIR}/../../build/fan_controller.elf
    Execute Command         sysbus.usart2 CreateFileBackend @${CURDIR}/uart_console.log
    Create Terminal Tester  sysbus.usart2  timeout=20
    Start Emulation
    Wait For Line On Uart   fan controller v1.0.0
    Wait For Line On Uart   running, heap free

*** Test Cases ***
Boots Without Optional Hardware
    Boot Firmware
    Wait For Prompt On Uart  fan>
    Write Line To Uart      status
    Wait For Line On Uart   state
    Wait For Line On Uart   oled missing

Auto Mode Without Sensor Goes To Fail Safe
    Boot Firmware
    Wait For Line On Uart   SENSOR RAISED    
    Wait For Line On Uart   -> FAULT
    Write Line To Uart      status
    Wait For Line On Uart   100.0 %

Manual Mode From Console
    Boot Firmware
    Wait For Prompt On Uart  fan>
    Write Line To Uart      mode manual
    Wait For Line On Uart   -> MANUAL        
    Write Line To Uart      duty 40
    Write Line To Uart      setpoint 27.5
    Write Line To Uart      cfg
    Wait For Line On Uart   mindut 200
    Write Line To Uart      tasks
    Wait For Line On Uart   control
    Wait For Line On Uart   heap free
    Write Line To Uart      foo
    Wait For Line On Uart   unknown command 'foo'
