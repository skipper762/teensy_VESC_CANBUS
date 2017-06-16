# teensy_VESC_CANBUS

# Currently Untested

A lib for running multiple VESCS over CAN-BUS from a teensy, this code is bassed from the [FLEX-CAN](https://github.com/teachop/FlexCAN_Library) and vedder's [BLDC code](https://github.com/vedderb/bldc_uart_comm_stm32f4_discovery).

The files: 
* datatypes.h
* buffer.h
* bldc_interface.h

including there sources are all unmodified. 

## Hardware Setup
  The code was tested using a [sn65hvd232d](http://www.ti.com/product/SN65HVD232) transeiver and a Teensy 3.6.
  The can transeiver is wired to the Teensy using pins 3 & 4, as shown in the figure below. When using one VESC and the transeiver a 120 Ohm teriminating resistor is needed, if more than one VESC is in the CAN network a terimnating resistor is not needed (But still recomended).
  
  

## Methods
`void vesc_can_begin()` Starts the CAN-BUS interface at the correct baud rate.
`void vesc_can_end()` Ends the CAN-BUS interface. 
  
### Setters
`void vesc_can_set_duty(uint8_t controller_id, float duty)` Set the duty from 0 - 1.0
`void comm_can_set_current(uint8_t controller_id, float current)` Set the current in amps
`void comm_can_set_current_brake(uint8_t controller_id, float current)` Set the brake current in amps
`void comm_can_set_rpm(uint8_t controller_id, float rpm)` Set the target RPM 
`void comm_can_set_pos(uint8_t controller_id, float pos)` Set the target possistion

### Getters
`comm_can_get_values(uint8_t controller_id)` Asks the VESC for a values data type, does not reutrn the vlaues data type. 
 
 `int vesc_can_read()` 
 call this in a polling or interupt routeen to process incoming data, will return an int bassed on what is returned
 


