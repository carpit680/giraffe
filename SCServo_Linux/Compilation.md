# Compilation Instructions

1. ### **Generate Static Library libSCServo.a**
   Use `cmake .` to generate the Makefile.  
   Use `make` to compile and generate the static library `libSCServo.a`.

2. ### **Example:**
   * Navigate to `cd examples/SMS_STS/WritePos` to enter the directory.  
   * Use `cmake .` to generate the Makefile.  
   * Use `make` to compile and generate the executable file `WritePos`.  
   * Run the executable with the command: `sudo ./WritePos /dev/ttyUSB0`  

> **Note**: `/dev/ttyUSB0` is an example of the actual device address, please specify it as needed.  

> **Note**: When controlling SMS/STS servos, `WritePos` is a command that requires appropriate parameters based on the specific motor model and options.
