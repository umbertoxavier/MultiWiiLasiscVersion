// DEF.H - LaSisC VERSION 
// VERSÃO SEM GPS

    #define MEGA
    //define o microcontrolador em uso = (Atmega 2560);
    
    #if defined(MEGA)
      #define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);  // pino digital 13(PB7) e 30(PC7) como saídas;      //PB7 ACENDE EM ALTO; PC7 ACENDE EM BAIXO    https://www.arduino.cc/reference/en/language/functions/digital-io/pinmode/
      #define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);           // [(leitura no PORTB) OR (10000000)] ; [(leitura no PORTC) OR (10000000)];                    //Writing a logic one to PINxn toggles the value of PORTxn, independent on the value of DDRxn.
                                                                                                                                                                                    //https://www.arduino.cc/en/Reference/PortManipulation
                                                                                                                                                                                    
      #define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);           // [PORTB OR (10000000)] ; [PORTC OR (10000000)];                                                https://www.arduino.cc/reference/en/language/structure/compound-operators/compoundbitwiseor/  
      #define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);          // [PORTB AND (01111111)] ; [PORTC AND (01111111)];                                              https://www.arduino.cc/reference/en/language/structure/compound-operators/compoundbitwiseand/
                                                                                     //                                                                                               https://www.arduino.cc/reference/en/language/structure/bitwise-operators/bitwisenot/
    
      #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;               // [PORTD OR (00000001)]; [PORTD OR (00000010)];   [pino digital 21(PD0/SCL);  pino digital 20(PD1/SDA)];  
      #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);         // [PORTD AND (11111110)]; [PORTD AND (11111101)];
                                                                                     // //If PORTxn is written logic one when the pin is configured as an input pin, the pull-up resistor is activated.
      
      #define STABLEPIN_PINMODE          pinMode (31, OUTPUT);                       // pino digital 31(PC6) como saída;    //PC6 ACENDE EM BAIXO
      #define STABLEPIN_ON               PORTC |= 1<<6;                              // [PORTC OR (01000000)];
      #define STABLEPIN_OFF              PORTC &= ~(1<<6);                           // [PORTC AND (10111111)];
    
      //RX PIN assignment inside the port /
      //PORTK Serve como entrada analógica para o conversor A/D  
      #define THROTTLEPIN                0  // ADC8  (PK0)
      #define ROLLPIN                    1  // ADC9  (PK1)
      #define PITCHPIN                   2  // ADC10 (PK2)
      #define YAWPIN                     3  // ADC11 (PK3)
      #define AUX1PIN                    4  // ADC12 (PK4)
      #define AUX2PIN                    5  // ADC13 (PK5)
      #define AUX3PIN                    6  // ADC14 (PK6)
      #define AUX4PIN                    7  // ADC15 (PK7)
      
      #define PCINT_PIN_COUNT            8                                                         // Número de pinos de interrupção externa. 
      #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0),(1<<1),(1<<3)   // 00000100, 00010000, 00100000, 01000000, 10000000, 00000001, 00000010, 00001000;
      #define PCINT_RX_PORT              PORTK                                                     // PORTK para pinos de interrupção externa;
      #define PCINT_RX_MASK              PCMSK2                                                    // Registrador para configuração de Interrupção externa. 
      #define PCIR_PORT_BIT              (1<<2)                                                    // 00000100
      #define RX_PC_INTERRUPT            PCINT2_vect                                               // PCI2 Interrupt Vector (VETOR PARA INTERRUPÇÃO EXTERNA)
      #define RX_PCINT_PIN_PORT          PINK                                                      // leitura das portas de interrupção. 
      #define ISR_UART                   ISR(USART0_UDRE_vect)                                     // Interrupção da UART com seu vetor na diretiva
    #endif
   
    /**************************************************************************************/
    /***************      IMU Orientations and Sensor definitions      ********************/
    /**************************************************************************************/
    #if defined(FFIMUv2)
      #define ITG3200   //giroscópio
      #define BMA180    //acelerômetro
      #define BMP085    //barômetro
      #define HMC5883   //magnetrômeto 
      #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;} //orientação para acelerômetro
      #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;} //orientação para giroscópio
      #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  =  -X; magADC[YAW]  = -Z;} //orientação para magnetrômeto 
    #endif
    
    #define ACC 1    //acelerômetro acionado
    #define MAG 1    //magnetrômeto acionado 
    #define GYRO 1   //giroscópio acionado
    #define BARO 1   //barômetro acionado
    #define GPS 0    //GPS não acionado
    #define SONAR 0  //sonar não acionado

    /**************************************************************************************/
    /***************      Multitype decleration for the GUI's          ********************/
    /*************************************************************************************star*/ 
    #if defined(QUADX)
      #define MULTITYPE 3  
    #endif
    
    /**************************************************************************************/
    /***************          Some unsorted "chain" defines            ********************/
    /**************************************************************************************/
    #define STANDARD_RX          //receptor padrão
    
    /**************************************************************************************/
    /***************             motor and servo numbers               ********************/
    /**************************************************************************************/
    #if defined(QUADX) 
      #define NUMBER_MOTOR     4 //número de motores
    #endif
    
    /**************************************************************************************/
    /***************                       I2C GPS                     ********************/
    /**************************************************************************************/
    #if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
      #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 20
    #endif 


