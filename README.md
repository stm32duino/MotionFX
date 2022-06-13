# MotionFX Library
The MotionFX is a software library that can run only on STM32 boards. It provides real-time motion-sensor data fusion.
It also performs gyroscope bias and magnetometer hard iron calibration. This library is intended to work with ST MEMS only.
The algorithm is provided in static library format and is designed to be used on STM32 microcontrollers based on the ARM Cortex-M0+, 
ARM Cortex-M3, ARM Cortex-M4, ARM Cortex-M7 or ARM Cortex-M33 architectures.
The library acquires data from the accelerometer, gyroscope (6-axis fusion) and magnetometer (9-axis fusion)
and provides real-time motion-sensor data fusion. The MotionFX filtering and predictive software uses advanced algorithms to intelligently integrate outputs from
multiple MEMS sensors, regardless of environmental conditions, for an optimum performance.
The complexity of the library dedicated to the Cortex-M0+ core is reduced due to the performance limitation of
Cortex-M0+ architecture. This library uses different APIs and has less features in comparison with the version for
Cortex-M3, Cortex-M4, Cortex-M7 and Cortex-M33. The recommended sensor data sampling frequency is 100 Hz. The output orientation of
the algorithm can be ENU (East-North-Up) Android format or NED (North-East-Up) format.

## Examples

There are several examples with the MotionFX library.
* Fusion6X_IKS01A2: This application provides a simple example of usage of the 6-axis fusion (accelerometer and gyroscope) with the X-NUCLEO-IKS01A2 
Expansion Board. It shows how to display on a hyperterminal the values of yaw, pitch and roll.
* Fusion6X_IKS01A3: This application provides a simple example of usage of the 6-axis fusion (accelerometer and gyroscope) with the X-NUCLEO-IKS01A3 
Expansion Board. It shows how to display on a hyperterminal the values of yaw, pitch and roll.
* Fusion6X_IKS02A1: This application provides a simple example of usage of the 6-axis fusion (accelerometer and gyroscope) with the X-NUCLEO-IKS02A1 
Expansion Board. It shows how to display on a hyperterminal the values of yaw, pitch and roll.
* Fusion9X_IKS01A2: This application provides a simple example of usage of the 9-axis fusion (accelerometer, gyroscope and magnetometer) with the X-NUCLEO-IKS01A2 
Expansion Board. It shows how to do the magnetometer hard iron calibration and display on a hyperterminal the values of yaw, pitch and roll.
* Fusion9X_IKS01A3: This application provides a simple example of usage of the 9-axis fusion (accelerometer, gyroscope and magnetometer) with the X-NUCLEO-IKS01A3 
Expansion Board. It shows how to do the magnetometer hard iron calibration and display on a hyperterminal the values of yaw, pitch and roll.
* Fusion9X_IKS02A1: This application provides a simple example of usage of the 9-axis fusion (accelerometer, gyroscope and magnetometer) with the X-NUCLEO-IKS02A1 
Expansion Board. It shows how to do the magnetometer hard iron calibration and display on a hyperterminal the values of yaw, pitch and roll.

## Setting CRC
Up to [STM32 core](https://github.com/stm32duino/Arduino_Core_STM32) version 2.2.0 it is necessary to apply the following steps:

Check where the Arduino `preferences.txt` file are in `Arduino IDE -> Preferences`.

	For Example: `C:\Users\Duino\AppData\Local\Arduino15` (Windows)

Then go to `.\packages\STM32\hardware\stm32\<release version>\cores\arduino\stm32\stm32yyxx_hal_conf.h` and open the file.

In the file add these lines of code:

```C
    #if !defined(HAL_CRC_MODULE_ENABLED)
      #define HAL_CRC_MODULE_ENABLED
    #else
      #undef HAL_CRC_MODULE_ENABLED
    #endif
```

## Note

The MotionFX Library can be used on STM32 boards with ST MEMS sensors only. For more information, please give a look at the license terms.
The MotionFX Library currently does not support the Cortex-M4 core of the STM32MP1xx devices. 

## Dependencies

The MotionFX library requires the following STM32duino libraries:

* STM32duino LSM6DSO: https://github.com/stm32duino/LSM6DSO
* STM32duino LIS2MDL: https://github.com/stm32duino/LIS2MDL
* STM32duino LSM6DSL: https://github.com/stm32duino/LSM6DSL
* STM32duino LSM303AGR: https://github.com/stm32duino/LSM303AGR
* STM32duino ISM330DHCX: https://github.com/stm32duino/ISM330DHCX
* STM32duino IIS2MDC: https://github.com/stm32duino/IIS2MDC

## Documentation

You can find the library files at
https://github.com/stm32duino/MotionFX

The MotionFX Library user manual is available at
https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/31/0e/66/39/cb/f7/4e/cd/DM00394369/files/DM00394369.pdf/jcr:content/translations/en.DM00394369.pdf
