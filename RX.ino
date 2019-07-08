// RX.H - LaSisC VERSION 
// VERSÃO SEM GPS


//OLHAR SEÇÃO 1.5 DO DATASHEET
//OLHAR https://www.youtube.com/watch?v=w37FOUrXO6s&list=PLtQdQmNK_0DRhBWYZ32BEILOykXLpJ8tP&index=5


/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
volatile uint16_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502};  // intervalo [1000;2000] 
                                                                                  //Uma variável deve ser declarada volatile quando seu valor pode ser mudado por algo além do controle da seção de código na qual ela aparece, como uma thread paralela. 
                                                                                  //No Arduino, o único lugar do código onde isso pode ocorrer é em seções associadas com interrupções, chamadas de rotina de serviço de interrupção (interrupt service routine).
                                                                                  //https://www.arduino.cc/reference/pt/language/variables/variable-scope--qualifiers/volatile/

// Standard Channel order
static uint8_t rcChannel[8]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};  //{1, 2, 3, 0, 4, 5, 6, 7}      -> diretivas definidas em def.h
                                                                                                           //A palavra-chave static é usada para criar variáveis que são visíveis para apenas uma função. 
                                                                                                           //No entanto, diferentemente de variáveis locais, que são criadas e destruidas toda vez que uma função é chamada, 
                                                                                                           //variáveis static persistem entre chamadas da função, preservando seu valor.
                                                                                                           //Variáveis declaradas como static são criadas e inicializadas apenas a primeria vez que uma função é chamada.
                                                                                                           //https://www.arduino.cc/reference/pt/language/variables/variable-scope--qualifiers/static/
  
  
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};                                           // PCINT_PIN_COUNT = 8      -> diretiva definida em def.h 
                                                                                                           // PCINT_RX_BITS= {00000100, 00010000, 00100000, 01000000, 10000000, 00000001, 00000010, 00001000};     -> diretiva definida em def.h
                                                                                                     


/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/

void configureReceiver() {  //Configuração dos registradores da interrupçao externa
  
/******************    Configure each rc pin for PCINT    ***************************/

  DDRK = 0;                //PORTK é definido como ENTRADA digital, e não analógica. 
 
  // THROTTLEPIN                0    // PCINT16 (PK0)  
  // ROLLPIN                    1    // PCINT17 (PK1)
  // PITCHPIN                   2    // PCINT18 (PK2)
  // YAWPIN                     3    // PCINT19 (PK3)
  // AUX1PIN                    4    // PCINT20 (PK4)
  // AUX2PIN                    5    // PCINT21 (PK5)
  // AUX3PIN                    6    // PCINT22 (PK6)
  // AUX4PIN                    7    // PCINT23 (PK7)  
  

// PCINT activation
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){   //PCINT_PIN_COUNT = 8
  
    PCINT_RX_PORT |= PCInt_RX_Pins[i];            // PCINT_RX_PORT = PORTK                                 //Resitor de Pull-ups no PORTK ligados
                                                                                                           //If PORTxn is written logic one when the pin is configured as an input pin, the pull-up resistor is activated.
                                                 
                                                  // PORTK OR 00000100  (PK2)    -> PITCHPIN       i = 0     
                                                  // PORTK OR 00010000  (PK4)    -> AUX1PIN        i = 1     
                                                  // PORTK OR 00100000  (PK5)    -> AUX2PIN        i = 2     
                                                  // PORTK OR 01000000  (PK6)    -> AUX3PIN        i = 3     
                                                  // PORTK OR 10000000  (PK7)    -> AUX4PIN        i = 4     
                                                  // PORTK OR 00000001  (PK0)    -> THROTTLEPIN    i = 5     
                                                  // PORTK OR 00000010  (PK1)    -> ROLLPIN        i = 6     
                                                  // PORTK OR 00001000  (PK3)    -> YAWPIN         i = 7     
    
    
    PCINT_RX_MASK |= PCInt_RX_Pins[i];            //  PCINT_RX_MASK = PCMSK2
    
                                                  //  PCMSK2 = Pin Change Mask Register 2;
                                                  //  Each PCINT23:16-bit selects whether pin change interrupt is enabled on the corresponding I/O pin. 
                                                  //  If PCINT23:16 is set and the PCIE2 bit in PCICR is set, pin change interrupt is enabled on the corresponding I/O pin. 
                                                  //  IfPCINT23:16 is cleared, pin change interrupt on the corresponding I/O pin is disabled.
                                                 
                                                  // PCMSK2 OR 00000100  (PK2)    -> PITCHPIN       i = 0     -> PCINT18 ATIVO
                                                  // PCMSK2 OR 00010000  (PK4)    -> AUX1PIN        i = 1     -> PCINT20 ATIVO
                                                  // PCMSK2 OR 00100000  (PK5)    -> AUX2PIN        i = 2     -> PCINT21 ATIVO
                                                  // PCMSK2 OR 01000000  (PK6)    -> AUX3PIN        i = 3     -> PCINT22 AITVO
                                                  // PCMSK2 OR 10000000  (PK7)    -> AUX4PIN        i = 4     -> PCINT23 ATIVO
                                                  // PCMSK2 OR 00000001  (PK0)    -> THROTTLEPIN    i = 5     -> PCINT16 AITVO
                                                  // PCMSK2 OR 00000010  (PK1)    -> ROLLPIN        i = 6     -> PCINT17 AITVO
                                                  // PCMSK2 OR 00001000  (PK3)    -> YAWPIN         i = 7     -> PCINT19 AITVO
    
                                                  //Agora todos os pinos no PORTK estão configurados para interrupção externa
      
}
  PCICR = PCIR_PORT_BIT;                          // PCICR = Pin Change Interrupt Control Register 
                                                  // PCIR_PORT_BIT = 00000100  -> PCIE2
                                                  
                                                  // PCIE2: Pin Change Interrupt Enable 
                                                  // When the PCIE2 bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt 2 is enabled. 
                                                  // Any change on any enabled PCINT23:16 pin will cause an interrupt. 
                                                  // The corresponding interrupt of Pin Change Interrupt Request is executed from the PCI2 Interrupt Vector. 
                                                  // PCINT23:16 pins are enabled individually by the PCMSK2 Register.
    
}                                                 // Como dito acima, qualquer mudança no pino de interrupção configurado causará uma interrupção. 
      
/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/

 // ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define RX_PIN_CHECK(pin_pos, rc_value_pos)                                                        \
  if (mask & PCInt_RX_Pins[pin_pos]) {                                                             \                             
    if (!(pin & PCInt_RX_Pins[pin_pos])) {                                                         \      
      dTime = cTime-edgeTime[pin_pos]; if (900<dTime && dTime<2200) rcValue[rc_value_pos] = dTime; \      
    } else edgeTime[pin_pos] = cTime;                                                              \    
  }
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 /* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  // pin_pos = 0    rc_value_pos = 2 
  
  if (mask & PCInt_RX_Pins[pin_pos]) {                                                                      //localiza pino de mundança
                                                                                                            //PCInt_RX_Pins = {00000100, 00010000, 00100000, 01000000, 10000000, 00000001, 00000010, 00001000};  
                                                                                                            //Se mask & PCInt_RX_Pins[0]}          //ex =   (00000100) AND (00000100) = retorna algo diferente de 0     
                                               
    if (!(pin & PCInt_RX_Pins[pin_pos])) {                                                                    // Se houve a segunda variação no PORTK (caso em que bit do mask é 1, mas o bit de PINK é 0 )    //pin = PINK 
      dTime = cTime-edgeTime[pin_pos]; if (900<dTime && dTime<2200) rcValue[rc_value_pos] = dTime;            // dTime = Ctime - edgeTime[0]; 
      
                                                                                                                // Se 900<dTime<2200
                                                                                                                // rcValue[2] = dTime  
                                                                                                            
    } else edgeTime[pin_pos] = cTime;                                                                         // Se houve a primeira variação no PORTK                                          
  }                                                                                                           // edgeTime[0] = Ctime
  
              
                  (1) dTime (2)
                    ---------   
                    |       |              (1) = Primeira variação
ENTRADA   =         |       |              (2) = Segunda variação 
                    |       |
           ---------|       |--------      //aqui pode ser vista um pwm contrário ao observado no osciloscópio devido aos registradores de pull up. 
           edgeTime   
           ------------------
                  Ctime         
 
*///------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
//AVR microcontroller architecture is based around Single core = one chunk of instruction at a time. 
ISR(RX_PC_INTERRUPT) { //Esta ISR é chamada toda vez que ocorer uma mudança no pino de entrada RX, em qualquer canal   
                       //RX_PC_INTERRUPT = PCINT2_vect   Vetor responsável pela interrupção externa nos pinos PCINT23:16

  uint8_t mask;                  // variável para verificar se houve mudança entre o valor atual e antigo do receptor.
  uint8_t pin;                   // variável que guarda a leitura de PINK.
  uint16_t cTime,dTime;          // tempo para atualizacão de DC
  static uint16_t edgeTime[8];   // tempo para atualização de DC 
  static uint8_t PCintLast;      // variável que memoriza o valor lido em PINK para comparação em interrupção futura.
  

  pin = RX_PCINT_PIN_PORT;       // RX_PCINT_PIN_PORT  = PINK   // leitura dos valores do receptor no PORTK                                       
  mask = pin ^ PCintLast;        // XOR para verificar em qual pino houve mudança entre o valor atual e o anterior em PORTK    
  sei();                         // Habilita as interrupçõe no registrador global. //Resto dessa interrupçao não tem tempo crítico
  PCintLast = pin;               // Memoriza o estado atual dos pinos do PORTK(receptor).
 
//----------------------------------------------------------------------    
//                 Exemplo do código acima:
  
//                 PCintLast = (00000000);
//                 PINK = (00000100);
//                 mask = (00000000) ^ (00000100) = 00000100  
//                 PCintLast = (00000100)  
//----------------------------------------------------------------------    
  cTime = micros();             // tempo em microssegundos para calculo de Duty Cycle.             //https://www.arduino.cc/reference/en/language/functions/time/micros/
  
  // PCINT_PIN_COUNT = 8        //Todas as condições abaixo serão verificadas.                     //#if PCINT_PIN_COUNT retirado
  // RX_PIN_CHECK(pin_pos, rc_value_pos)  
  // pin_pos = {00000100, 00010000, 00100000, 01000000, 10000000, 00000001, 00000010, 00001000};
     
  RX_PIN_CHECK(0,2);            //(PK2)    -> PITCHPIN
  RX_PIN_CHECK(1,4);            //(PK4)    -> AUX1PIN 
  RX_PIN_CHECK(2,5);            //(PK5)    -> AUX2PIN
  RX_PIN_CHECK(3,6);            //(PK6)    -> AUX3PIN
  RX_PIN_CHECK(4,7);            //(PK7)    -> AUX4PIN
  RX_PIN_CHECK(5,0);            //(PK0)    -> THROTTLEPIN
  RX_PIN_CHECK(6,1);            //(PK1)    -> ROLLPIN 
  RX_PIN_CHECK(7,3);            //(PK3)    -> YAWPIN
  
}//FIM DA ISR


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/


uint16_t readRawRC(uint8_t chan) {
  
  uint16_t data;
  uint8_t oldSREG;
  
  oldSREG = SREG; cli();             // Desabilita as interrupções
  
  data = rcValue[rcChannel[chan]];   // data recebe o valor de rcValue do canal requerido. 
                                     // o valor em rcValue foi calculado pelo dTime, visto acima. 
                                     // rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502};
                                     // rcChannel[8]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN} = {1, 2, 3, 0, 4, 5, 6, 7} 
  
  SREG = oldSREG; sei();             // Habilita as interrupçöes 
  
  return data;                       // retorna data
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/


void computeRC() {
  
  static int16_t rcData4Values[8][4], rcDataMean[8];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  
  rc4ValuesIndex++;   //posição coluna da matriz
  
  for (chan = 0; chan < 8; chan++) {                                                //                         [0]  [1]   [2]   [3]  
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);                        //                [chan0] | 25   1     9     17 |    
    rcDataMean[chan] = 0;                                                           //rcData4Values = [chan1] | 26   2     10    18 | = ReadRawRC(chan)               //rcDataMean = {0, 0, 0, 0, 0, 0, 0, 0,}  
                                                                                    //                [chan2] | 27   3     11    19 |
                                                                                    //                [chan3] | 28   4     12    20 |
                                                                                    //                [chan4] | 29   5     13    21 |
                                                                                    //                [chan5] | 30   6     14    22 |
                                                                                    //                [chan6] | 31   7     15    23 |
                                                                                    //                [chan7] | 32   8     16    24 |  
    
    for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];                   // rcDataMean[chan] =  rcDataMean[chan] + rcData4Values[chan][0]      chan[0->7]
                                                                                    // rcDataMean[chan] =  rcDataMean[chan] + rcData4Values[chan][1]
                                                                                    // rcDataMean[chan] =  rcDataMean[chan] + rcData4Values[chan][2]
                                                                                    // rcDataMean[chan] =  rcDataMean[chan] + rcData4Values[chan][3]
   
    rcDataMean[chan]= (rcDataMean[chan]+2)/4;                                       // rcDataMean[chan] = (rcDataMean[chan]+2)/4                          chan[0->7]
    
    if ( rcDataMean[chan] < rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;    //Atualização de RcData    //RcData declarado em Multiwii.h
    
    if ( rcDataMean[chan] > rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;    //Atualização de RcData 
  }

}


