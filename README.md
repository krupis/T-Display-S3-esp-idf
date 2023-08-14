| Supported Targets | ESP32 | ESP32-S2 | ESP32-S3 |

| ----------------- | ----- | -------- | -------- |

# ST7789V (T-Display-S3) from LilyGo example using LVGL V8.3.6

### Hardware Required

- T-Display-S3 development board (https://www.lilygo.cc/products/t-display-s3)

- An USB cable for power supply and programming

### Hardware

Detailed description of T-Display-S3 development board can be found here:

https://github.com/Xinyuan-LilyGO/T-Display-S3

### How to use the example

This example project was built and tested using esp-idf v5.0.1

In order to try different LVGL functions, uncomment the following functions one by one in main.c

```
// display_meter();

// display_image();

// display_window();

// display_dropdown();
```

Everytime display functions are called, be sure to wrap them in `bsp_display_lock(0);` and `bsp_display_unlock();` to ensure no conflicts when trying to display using different threads/tasks.
