
### **Communication Layer:** SCS  

### **Hardware Interface Layer:** SCSerial  

**Application Layer:** SMSBL, SMSCL, and SCSCL correspond to the three series of servos.  
**Application Layer:** SMS_STS is compatible with the SMS/STS series servos.

```cpp
SMSBL sm;  // Define SMSBL series servo
SMSCL sm;  // Define SMSCL series servo
SCSCL sc;  // Define SCSCL series servo
SMS_STS sm_st;  // Define SMS/STS series servo
```

The interfaces for SMSCL, SMSBL, SCSCL, and SMS_STS are referenced in their respective header files.

- **INST.h** - Instruction definition header file  
- **SCS.h/SCS.cpp** - Communication layer program  
- **SCSerial.h/SCSerial.cpp** - Hardware interface program  
- **SMSBL.h/SMSBL.cpp** - SMSBL application layer program  
- **SMSCL.h/SMSCL.cpp** - SMSCL application layer program  
- **SCSCL.h/SCSCL.cpp** - SCSCL application layer program  
- **SMS_STS.h/SMS_STS.cpp** - SMS/STS application layer program  

(The memory table definitions are in the application layer header files SMSBL.h, SMSCL.h, SCSCL.h, and SMS_STS.h. The memory table definitions vary between different servo series.)

### Class Structure

```plaintext
                SMSBL class
SCS class <---  SCSerial class <--- SMSCL class
                SCSCL class
                SMS_STS class
```
