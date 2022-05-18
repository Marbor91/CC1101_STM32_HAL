# CC1101_HAL_STM32 library
## Can be used for every STM32 MCU

How to use?

1 - Correct  name of SPI chip select port and pin in "CC1101_port.c to name in you code"

>void ___CC1101_USER_CS_High()<br>
>{
>HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);
>}

>void ___CC1101_USER_CS_Low()<br>
>{
>  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
>}
