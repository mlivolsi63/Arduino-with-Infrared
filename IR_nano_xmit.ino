#include <Arduino.h>

#include "PinDefinitionsAndMore.h"  // Define macros for input and output pin etc.

#if !defined(ARDUINO_ESP32C3_DEV) // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif

#include <IRremote.hpp>

#define DELAY_AFTER_SEND 2000
#define DELAY_AFTER_LOOP 5000

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to become available. Is optimized away for some cores.

    #if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
        delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
    #endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    #if defined(IR_SEND_PIN)
        IrSender.begin(); // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin
    //    disableLEDFeedback(); // Disable feedback LED at default feedback LED pin
    #  if defined(IR_SEND_PIN_STRING)
       Serial.println(F("Send IR signals at pin " IR_SEND_PIN_STRING));
    #  else
       Serial.println(F("Send IR signals at pin " STR(IR_SEND_PIN)));
    #  endif
    #else
    // Here the macro IR_SEND_PIN is not defined or undefined above with #undef IR_SEND_PIN
    uint8_t tSendPin = 3;
    IrSender.begin(tSendPin, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN); // Specify send pin and enable feedback LED at default feedback LED pin
    // You can change send pin later with IrSender.setSendPin();

    Serial.print(F("Send IR signals at pin "));
    Serial.println(tSendPin);
#endif

#if !defined(SEND_PWM_BY_TIMER)
    /*
     * Print internal software PWM signal generation info
     */
    IrSender.enableIROut(38); // Call it with 38 kHz just to initialize the values printed below
    Serial.print(F("Send signal mark duration is "));
    Serial.print(IrSender.periodOnTimeMicros);
    Serial.print(F(" us, pulse narrowing correction is "));
    Serial.print(IrSender.getPulseCorrectionNanos());
    Serial.print(F(" ns, total period is "));
    Serial.print(IrSender.periodTimeMicros);
    Serial.println(F(" us"));
#endif

#if defined(LED_BUILTIN) && !defined(NO_LED_FEEDBACK_CODE)
#  if defined(FEEDBACK_LED_IS_ACTIVE_LOW)
    Serial.print(F("Active low "));
#  endif
    Serial.print(F("FeedbackLED at pin "));
    Serial.println(LED_BUILTIN); // Works also for ESP32: static const uint8_t LED_BUILTIN = 8; #define LED_BUILTIN LED_BUILTIN
#endif

}


uint16_t sAddress = 0x0;
uint8_t sCommand = 0xC;
uint8_t sRepeats = 0;

void loop() {
    /*
     * Print values
     */
    Serial.println();
    Serial.print(F("address=0x"));
    Serial.print(sAddress, HEX);
    Serial.print(F(" command=0x"));
    Serial.print(sCommand, HEX);
    Serial.print(F(" repeats="));
    Serial.println(sRepeats);
    Serial.println();
    Serial.println();
    Serial.flush();

    Serial.println(F("Send NEC with 16 bit address"));
    Serial.flush();
    IrSender.sendNEC(sAddress, sCommand, sRepeats);
    delay(DELAY_AFTER_SEND);
  
}
