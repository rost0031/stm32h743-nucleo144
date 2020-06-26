# STM32H743ZI-Nucleo144 Example of STM32 HAL UART3 driver with DMA for TX and RX

This example demonstrates UART3 running with DMA for TX and RX with variable length data. Doing this for TX is simple but RX examples from STM don't allow variable length data nor do they handle IDLE interrupt. This example shows how to set it this up. Note that this includes modifications to STM HAL drivers.

The uart connection settings are 115200,8,n,1.

In addition, pay attention to the linker file since default linker files from STM used to incorrectly put all the variables into the D1 memory which is not accessible via DMA. 

The example is meant to be as simple as possible and the HW initialization and handling code is interspersed with "application" code. This is meant to show how to deal with hardware and should not form the basis of any serious project.