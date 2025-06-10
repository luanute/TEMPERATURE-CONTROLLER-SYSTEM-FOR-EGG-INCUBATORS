# STM32 Egg Incubator Firmware

This is the firmware for an STM32F1-based egg incubator controller. It provides precise, automated temperature control through a user-friendly LCD menu interface.

## 🌟 Key Features

* **Smart Menu System:** A multi-level menu on a 20x4 I2C LCD, navigated by 5 push buttons.
* **Precise Temperature Control:** Uses a PID algorithm and a high-accuracy PT100 sensor (with a MAX31865 module).
* **Optimized Power Control:** Manages the heater's SSR using Phase-Angle Control with a Zero-Cross Detector for stable and efficient heating.
* **Flexible Operation Modes:**
    * **AUTO Mode:** Pre-set incubation programs for Chicken, Duck, and Quail eggs.
    * **MANUAL Mode:** Allows users to set a custom target temperature and timer.
* **Peripheral Management:**
    * Real-time temperature display on a 7-segment LED.
    * Software control for the fan, light, and LCD backlight.
    * Internal RTC for timed operations.

## 📖 How to Use

* **UP / DOWN:** Navigate through the current menu.
* **NEXT:** Enter a sub-menu or an editing screen. In edit mode, it cycles through fields (e.g., HH:MM:SS).
* **BACK:** Return to the previous menu or cancel an edit.
* **SELECT:** Confirm a selection, execute an action, or save changes from an editing screen.
