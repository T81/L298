# L298 Dual Full-Bridge Driver library for Arduino

### Yet another L298 Dual H-Bridge driver library for arduino (with advanced functionality though!)
Eventhough there are a lot of libraries/code out there dealing with this IC,
the lack of some basic or even advanced functionalities lead to the development of this one.

### Features
- [x] Non blocking code
- [x] [Unittested](https://github.com/T81/L298/new/master?readme=1#unittests)
- [x] Adjustable code
- [x] Optional Debug messages

* **Basic**
  - [x] Adjustable speed
  - [x] Coasting
  - [x] Braking
  - [ ] Motor status flags
    - [x] Running
    - [x] Direction
    - [x] Accelerating
    - [x] Braking
    - [x] Coasting
    - [x] Full stop brake on
    - [x] CW limit triggered
    - [x] CCW limit triggered
    - [x] Overcurrent
    - [ ] Configuration error

* **Advanced / Optional**
  - [x] Acceleration
  - [x] Deceleration
  - [x] Position feedback
  - [x] Limits setup
    - [x] Collision detection (digital)
    - [x] Motor position (analog)
    - [x] Max current



## Getting started
Download and unzip `L298` folder in arduino's `Library` folder

## Non blocking code
All time-dependent functions are written without using the delay() function

## Unittests


### Acceleration / Deceleration

### Position feedback


### Adjustable code
> With a lot of functionlaity, comes a lot of overhead. -- **__Unknown programmer__**

Since having a ton of functionality available on your disposal sounds great,
on the contrary unecessary code adds up to limited resources.

The user has the ability to strip out not needed code by editing the `L298.h` header file.
```
// comment out the following line to disable "ACCELERATION" functions
#define ACCELERATION_FUNCTIONS

// comment out the following line to disable "CURRENT" functions
#define CURRENT_FUNCTIONS

// comment out the following line to disable "LIMITING" functions
#define LIMITING_FUNCTIONS

// comment out the following line to disable "POSITION" functions
#define POSITION_FUNCTIONS
```
### Unittests

### Contributing
Feel free to fork this repository, mess around and make pull requests.