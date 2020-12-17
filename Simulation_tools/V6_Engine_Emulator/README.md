# CDI_do_Jerena - Powerful and calibratable electronic ignition module to optimize internal combustion engine efficiency.

The objective this project is build an Electronic Ignition module with satisfactory performance and accuracy, basically integrating low cost electronic circuits/dev boards that can be found easily in online stores like: eBay, AliExpress, DealeXtreme and others using a minimal number of external electronic components.

Technical Information about this module:

To simplify module adaptation in different vehicles and engines, I decided to develop a DC CDI (don´t need a rotor magnets and excitor coil to generate high voltage to charger the capacitor), reducing the ignition system to:
- Battery
- CDI Module
- Crank shaft position sensor 
- Ignition Coil (CDI compatible)

Features:

- System configuration (engine type: two stroke or four stroke, cylinder numbers, sensor numbers and angular sensor position, Ignition timing map calibratable (spark advance x engine speed))
- Auto-diagnostic (functional and electrical)
- Human Machine Interface (Electronic instrument cluster displayed: diagnostic, engine and vehicle data in real time and calibratable by smart phone app)

Hardware description:

- Step-Down Voltage Regulator
- Blue Pill (dev board) / Detailed Information in: https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html
- CDI circuit (shield circuit built on perforated board, consult the project documentation to found it!)
- Inversor (12V DC -> 220V AC)
- Mini OLED Display (SPI)
- Bluetooth module HC-05

Development tools:
- ST-LINK V2 Dongle
- Keil MDK
- STM32Cube

/* PS: To obtain more detailed information please consult all available documentation or contact me directly */


