// OUTPUT.H - LaSisC VERSION 
// VERSÃO SEM GPS


/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/

uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};    //Pinos Digitais na saída no Microcontrolador que fazem a geração de PWM 
                                            // motor[0] = pino digital (PWM) 3 =  PE5 -> OC3C
                                            // motor[1] = pino digital (PWM) 5 =  PE3 -> OC3A
                                            // motor[2] = pino digital (PWM) 6 =  PH3 -> OC4A
                                            // motor[3] = pinO digital (PWM) 2 =  PE4 -> OC3B
                                          
                                            
/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/

void writeMotors() {               // Aqui a função recebe no parâmetro valores entre [1000;2000] e estes serão convertidos para o registrador de comparação [8000;16000]

                                   // Aqui será utilizado o Timer 3 e Timer 4, ambos de 16bits, para a geração de um sinal PWM
                                   
                                   // motor[NUMBER_MOTOR]; declarado em Multiwii.h
                                   // NUMBER_MOTOR = 4;    declarado em def.h   
                                   
                                   // OCRnx = Registrador de comparação
                                   // The Output Compare Registers contain a 16-bit value that is continuously compared with the counter value(TCNTn). 
                                   // A match can be used to generate an Output Compare interrupt, or to generate a waveform output on the OCnx pin.  
                                   
 OCR3C = motor[0]<<3; //  pin 3      
 OCR3A = motor[1]<<3; //  pin 5     
 OCR4A = motor[2]<<3; //  pin 6     
 OCR3B = motor[3]<<3; //  pin 2    // Aqui o valor recebido em cada posição do vetor motor[] é deslocado 3 bits para esquerda.
                                   // [1000:2000] => [8000:16000]  
                                   
//           Exemplo:
//           [1000]    = (0000 0011 1110 1000)
//           [1000]<<3 = (0001 1111 0100 0000) = [8000]
//
//           [2000]    = (0000 0111 1101 0000)
//           [2000]<<3 = (0011 1110 1000 0000) = [16000]
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/

void writeAllMotors(int16_t mc) {            // Envia mesmo comando para todos os motores. 
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;                             //motor[0] = mc
  }                                          //motor[1] = mc
                                             //motor[2] = mc
                                             //motor[3] = mc
                                             
  writeMotors();  //converte e repassa os valores para o registrador de comparação gerar o PWM.  [1000;2000] => [8000;16000]                          
}                            

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/

void initOutput() {  // Configuração dos registradores do PWM
  
/****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {          // NUMBER_MOTOR = 4
 
    pinMode(PWM_PIN[i],OUTPUT);                  // Aqui declara os pinos digitias PWM como saídas. 
                                                 // PWM_PIN[8] = {3,5,6,2,7,8,9,10}
                                                 
                                                 // PWM_PIN[0] = Pino digital (PWM) 3 = PE5 -> OC3C
                                                 // PWM_PIN[1] = Pino digital (PWM) 5 = PE3 -> OC3A
                                                 // PWM_PIN[2] = Pino digital (PWM) 6 = PH3 -> OC4A
                                                 // PWM_PIN[3] = Pino digital (PWM) 2 = PE4 -> OC3B
  }
    
/****************  Specific PWM Timers & Registers for the MEGA's    ******************/

  // OBS =   TCCR3B e TCCR4B inicia em = (00000011)
  //         TCCR3A e TCCT4A inicia em = (00000001)
  
  //  Inicialização do TIMER 3
  TCCR3A |= (1<<WGM31); // Phase Correct PWM Mode, with ICR3 as TOP valor & no prescaler
  TCCR3A &= ~(1<<WGM30);
  TCCR3B |= (1<<WGM33);    
  TCCR3B &= ~(1<<CS31);    
  ICR3   |= 0x3FFF; // TOP to 16383;      
  
  TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C           //Clear OCnA/OCnB/OCnC on compare match when up-counting
  TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A           //Set OCnA/OCnB/OCnC on compare match when downcounting
                                                                         //Nesta configuração, quanto maior o valor do contador, maior o DC do sinal PWM
                                                                         //https://www.microchip.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_use_bv.html
                                                                         //http://maxembedded.com/2012/01/avr-timers-pwm-mode-part-ii/

  //  Inicialização do TIMER 4
  TCCR4A |= (1<<WGM41); // Phase Correct PWM Mode, with ICR4 as TOP valor & no prescale
  TCCR4A &= ~(1<<WGM40);
  TCCR4B |= (1<<WGM43);
  TCCR4B &= ~(1<<CS41); 
  ICR4   |= 0x3FFF; // TOP to 16383;    
  
  TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A          //Clear OCnA/OCnB/OCnC on compare match when up-counting
  TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B          //Set OCnA/OCnB/OCnC on compare match when downcounting
                                                                        //Nesta configuração, quanto maior o valor do contador, maior o DC do sinal PWM
                                                         
//------------------------------------------------------------------------------------------------------------------------------------------------------------                                   
// Phase Correct PWM Mode:                                 
//                                
//                     TOP         TOP         TOP         TOP                    TOP = ICRn
//                      /\          /\          /\          /\
// OCRnx1 =============/==\========/==\========/==\========/==\=============      OCRnx1>OCRnx2  = Contadores
//                    /    \      /    \      /    \      /    \
//                   /      \    /      \    /      \    /      \
// OCRnx2 ==========/========\==/========\==/========\==/========\==========
//                 /          \/          \/          \/          \
//                 ____    ________    ________    ________    ____
//                     |  |        |  |        |  |        |  |                                                   Limpa OCnA/OCnB/OCnC na comparação quando up-counting
// PWM for OCRnx1      |  |        |  |        |  |        |  |      Saída nos Pinos OCnA/OCnB/OCnC               Seta OCnA/OCnB/OCnC na comparação quando downcounting
//                     |__|        |__|        |__|        |__|       
//                                                                 
//                 _          __          __          __          _
//                  |        |  |        |  |        |  |        |
// PWM for OCRnx2   |        |  |        |  |        |  |        |   Saída nos Pinos OCnA/OCnB/OCnC   
//                  |________|  |________|  |________|  |________|  
//
//------------------------------------------------------------------------------------------------------------------------------------------------------------                                                               


 /********  special version of MultiWii to calibrate all attached ESCs ************/
 
  #if defined(ESC_CALIB_CANNOT_FLY)    // Se setada calibração dos ESCs 
    writeAllMotors(ESC_CALIB_HIGH);    // Escreve nos motores o PWM máximo;  ESC_CALIB_HIGH = 2000
    delay(3000);                       // Espera 3 segundos. 
    
    writeAllMotors(ESC_CALIB_LOW);     // Escreve nos motores o PWM mínimo;  ESC_CALIB_LOW = MINCOMMAND = 1000
    delay(500);                        // Espera 0,5 segundos.
   
    while (1) {                        // looping infinito                    
      delay(5000);                     // Espera de 5 segundos.                  
      blinkLED(2,20, 2);               // Leds acendem e apagam num intervalo de 0.02 segundos (2 vezes)
    }
    exit;                              // Aqui nunca será atingido. 
  #endif

//----------------------------------------------------------------------------------------------------------------

  writeAllMotors(MINCOMMAND); // Após configurar as saídas e registradores pra o PWM, os motores são todos iniciados com MINCOMMAND = 1000. 
  delay(300);                 // Espera de 0.3 segundos
  
}

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

void mixTable() { //Necessita fazer o estudo da Multiwii.h e EEPROM.h
  
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  
  /****************                   main Mix Table                ******************/
  motor[0] = PIDMIX(-1,+1,-1);   //REAR_R     //rotações
  motor[1] = PIDMIX(-1,-1,+1);   //FRONT_R
  motor[2] = PIDMIX(+1,+1,+1);   //REAR_L
  motor[3] = PIDMIX(+1,-1,-1);   //FRONT_L

  /****************                Filter the Motors values                ******************/
  
  maxMotor=motor[0];

  for(i=1;i< NUMBER_MOTOR;i++){
    if (motor[i]>maxMotor) maxMotor=motor[i];}
    
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);    
    if ((rcData[THROTTLE]) < MINCHECK)
      #ifndef MOTOR_STOP
        motor[i] = MINTHROTTLE;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (!f.ARMED)
      motor[i] = MINCOMMAND;
  }  
}

