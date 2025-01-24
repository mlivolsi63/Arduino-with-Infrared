#include <Arduino.h>
#include <ESP8266WiFi.h>
// ----------------------------------------
// For the PWM Functions and PCA board
#include <Wire.h>
// ----------------------------------------
#include "PinDefinitionsAndMore.h"  // Define macros for input and output pin etc.
#include <IRremote.hpp>

#if !defined(ARDUINO_ESP32C3_DEV) // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif


#define DELAY_AFTER_SEND 500

const char* ssid     = "UB1";
const char* password = "2324070289";

char inBuffer[4096];

WiFiClient client;
WiFiServer server(7890);

uint16_t RED=0xC;
uint16_t YELLOW=0x5E;
uint16_t GREEN=0x4A;
uint16_t BLUE=0x42;


unsigned int globalColor=0;


void setup() {

    while (!Serial)
        ; // Wait for Serial to become available. Is optimized away for some cores.

    Serial.begin(9600);


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
    uint8_t tSendPin = 3; // original code

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

    //----------------------------------------------
    // Connect to Wi-Fi network with SSID and password
    //----------------------------------------------  
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
       delay(500);    
    }
    //----------------------------------------------
    // Print local IP address and start web server
    //----------------------------------------------
    Serial.print("Connected - IP address: "); Serial.println(WiFi.localIP());
    server.begin();
    Wire.begin();

}

void setFloodLight(int a, uint8_t r, uint8_t g, uint8_t b) {

    // If 1 < R <= 5  turn on  flood light
    // if 1 < B <= 5  turn off flood light
    uint16_t sAddress = 0x0;
    uint8_t sCommand = 0xC;
    uint8_t sRepeats = 0;
    short int color=0;

    if (r > 0) {
        if (r <= 128) {
          if (color <=5)  color=100;
          else            color=1;
        }  
        else              color=2;
    }
    if (g > 0 ) {        
        if (g <= 128) {
            if (g <=5 ) color=101;
            else        color+=3;
        }  
        else          color+=6;
    }
    if (b > 0 ) {
        if (b <= 128) {
            if(b <=10) color=101; 
            else       color+=9;
        }  
        else          color+=18;
    } 

    if ( (color > 0) && (globalColor != color) )  {
        // Serial.print("Color:"); Serial.print(color); Serial.print(" ");

        switch(color) {
        case 0:
                break;
        case 1: // dim red
                //Serial.println("Dim red");                
                IrSender.sendNEC(0x0, 0x1C, 0);  
                delay(5);              
                IrSender.sendNEC(0x0, 0xC,  0);  // red
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                break;
        case 2: // bright red
                //Serial.println("Bright red");
                IrSender.sendNEC(0x0, 0x1C, 0);  
                delay(5);              
                IrSender.sendNEC(0x0, 0xC,  0);  // red              
                delay(5);
                IrSender.sendNEC(0x0, 0x45, 2);  // bright
                break;
        case 3: // dim green
                //Serial.println("Bright Green");             
                IrSender.sendNEC(0x0, 0x1C, 0);  
                delay(5);              
                IrSender.sendNEC(0x0, 0x4A, 0);  // Full Green
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                break;
        case 4: // dim green and red (aka orange)
                //Serial.println("Orange");                
                IrSender.sendNEC(0x0, 0x5E, 0);   // Yellow
                delay(5);
                IrSender.sendNEC(0x0, 0xC, 6);   // Add more red
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                break;        
        case 5: // High red with low green
                //Serial.println("Bright orange");                
                IrSender.sendNEC(0x0, 0x1C, 0);  
                delay(5);              
                IrSender.sendNEC(0x0, 0x5E, 0);   // Orange              
                delay(5);
                IrSender.sendNEC(0x0, 0xC, 8);   // Add more red
                delay(5);
                IrSender.sendNEC(0x0, 0x45, 2);  // dim
                break;
        case 6: // High green
                //Serial.println("Bright green");
                IrSender.sendNEC(0x0, 0x1C, 0);  
                delay(5);              
                IrSender.sendNEC(0x0, 0x4A, 0);  // full Green              
                delay(5);
                IrSender.sendNEC(0x0, 0x45, 2);  // bright
                break;
        case 7: // Full green with a hint of red
                //Serial.println("orange-ish");    
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);              
                IrSender.sendNEC(0x0, 0x7, 0);  // full                 
                break;
        case 8: // Full green with a good chunk of red
                //Serial.println("Bright yellow");                
                IrSender.sendNEC(0x0, 0x1C, 0);
                delay(5);
                IrSender.sendNEC(0x0, 0x5E, 0);   // Orange             
                break;
        case 9:  // Full dim blue
                //Serial.println("Dim Blue");                
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);              
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                break;
        case 10: // Dim blue and dim red
                //Serial.println("Dark Purple ?");                
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);              
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(5);
                IrSender.sendNEC(0x0, 0xC,  0);  // Red
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                break;
        case 11: // Dum blue with more red      
                //Serial.println("Barney Purple ?");                
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);              
                IrSender.sendNEC(0x0, 0xC,  0);  // Red              
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 4);  // Blue                            
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                break;
        case 12: // blue with hint of green
                //Serial.println("Dark Turquoise");      
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(10);                                        
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(10);
                IrSender.sendNEC(0x0, 0x4A, 2);  // Green
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                break;      
        case 13: // dim white              
                //Serial.println("Dim white");
                IrSender.sendNEC(0x0, 0x1C, 0);  // 
                IrSender.sendNEC(0x0, 0x46, 2);  // dim             
                break;      
        case 15: // Dim blue with strong green
                //Serial.println("Aqua");              
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);              
                IrSender.sendNEC(0x0, 0x4A, 0);  // Green
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 2);  // Blue                           
                break;
        case 16: // Dim blue , dim red, strong green
                //Serial.println("I have no idea.. some shit white");              
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);              
                IrSender.sendNEC(0x0, 0x4A, 0);  // Green
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue                           
                delay(5);
                IrSender.sendNEC(0x0, 0xC,  0);   // red
                break;
        case 17: // Strong red and green, dim blue
                //Serial.println("I have no idea.. some shit white version 2");
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);              
                IrSender.sendNEC(0x0, 0xC,  0);  // red
                delay(5);
                IrSender.sendNEC(0x0, 0x4A, 0);  // Green              
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue                                         
                break;
        case 18: // Blue              
                //Serial.println("Shit should be blue");
                IrSender.sendNEC(0x0, 0x1C, 0);  //white
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(5);
                IrSender.sendNEC(0x0, 0x45, 2);  // bright
                break;
        case 19:        
                IrSender.sendNEC(0x0, 0x1C, 0);              
                delay(5);                    
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(5);                    
                IrSender.sendNEC(0x0, 0xC,  2);  // Red
                break;
        case 20:    
                // Serial.println("BLue/Pink");    
                IrSender.sendNEC(0x0, 0x1C, 0);  // white    
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(5);
                IrSender.sendNEC(0x0, 0xC,  4);  // Red
                delay(5);
                IrSender.sendNEC(0x0, 0x45, 2);  // bright
                break;
        case 21:              
                IrSender.sendNEC(0x0, 0x1C, 0);  // white    
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(5);
                IrSender.sendNEC(0x0, 0x4A, 0);  // Green              
                break;
        case 22:          
                IrSender.sendNEC(0x0, 0x1C, 0);  // white    
                delay(5);    
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue              
                delay(5);
                IrSender.sendNEC(0x0, 0x4A, 4);  // Green 
                IrSender.sendNEC(0x0, 0x45, 2);  // bright             
                break;
        case 23: // Blue with bright red and dim green
                IrSender.sendNEC(0x0, 0x1C, 0);  // white    
                delay(5);    
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue        
                delay(5);    
                IrSender.sendNEC(0x0, 0xC,  2);  // Red      
                delay(5);    
                IrSender.sendNEC(0x0, 0x4A, 2);  // Green 
                delay(5);    
                IrSender.sendNEC(0x0, 0x45, 2);  // bright             
                break;
        case 24: // Blue with bright green
                // Serial.println("Turquoise");
                IrSender.sendNEC(0x0, 0x1C, 0);  //white
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue                      
                delay(5);
                IrSender.sendNEC(0x0, 0x4A, 3);  // Green 
                delay(5);
                IrSender.sendNEC(0x0, 0x45, 2);  // bright             
                break;
        case 25: // Blue with bright green  dim red
                IrSender.sendNEC(0x0, 0x1C, 0);  // white    
                delay(5);
                IrSender.sendNEC(0x0, 0x42, 0);  // Blue                      
                delay(5);
                IrSender.sendNEC(0x0, 0x4A, 2);  // Green 
                delay(5);
                IrSender.sendNEC(0x0, 0xC,  0);  // Green 
                IrSender.sendNEC(0x0, 0x45, 2);  // bright             
                break;
        case 26: //Full white                 
                // Serial.println("White");
                IrSender.sendNEC(0x0, 0x1C, 0);  // White              
                delay(5);
                IrSender.sendNEC(0x0, 0x45, 0);  // bright             
                break;  
        case 100:          
                //Serial.println("On");                
                IrSender.sendNEC(0x0, 0x1C, 0);  // White   
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                delay(5);
                IrSender.sendNEC(0x0, 0x47, 0);  //  On
                break;
        case 101:
                //Serial.println("Off");
                IrSender.sendNEC(0x0, 0x1C, 0);  // White              
                delay(5);
                IrSender.sendNEC(0x0, 0x46, 2);  // dim
                delay(5);
                IrSender.sendNEC(0x0, 0x43, 0);  // Off
                break;
        default: Serial.println("Unknown");
        }
        globalColor=color;
    }       

}


bool looper=true;

uint8_t r1 = 0;
uint8_t g1 = 0;
uint8_t b1 = 0;


void loop() {

    client = server.available();                                                   // Listen for incoming clients

       if (client) {                                                                  // If a new client connects,       
        Serial.print(millis()); Serial.println("--> client connected");            // print a message out in the serial port        

        while (looper && client.connected() ) {  // loop while the client's connected       
   
               if (client.available()) {                                              // There's bytes coming in
                for(int i=0; i<4; i++) {
                  inBuffer[i]=client.read();
                }
                uint8_t channelId = inBuffer[0];
                uint8_t command = inBuffer[1];
                uint16_t dataLength = ntohs(*(uint16_t*)&inBuffer[2]);

                if(dataLength > 1 ) {
                    //Serial.print("Channel : ");         Serial.print(channelId);                                                // print it out the serial monitor
                    //Serial.print(", Command : ");       Serial.print(command);                                                 // print it out the serial monitor
                    //Serial.print(", Data Length : ");   Serial.println(dataLength);

                    for (int i=0; i< dataLength; i++) {
                        inBuffer[i]=client.read();    
                    }
                    for (int i=0; i < dataLength; i+=3) {
                        r1 = inBuffer[i];
                        g1 = inBuffer[i + 1];
                        b1 = inBuffer[i + 2];                         
                        setFloodLight(i/3, r1, g1, b1);
                    }
      
                }    
             }
         
             delay(100);
           
         }                                     // end while        
         Serial.println("Disconnecting from client");
         client.stop();         
     }

   
  
}
