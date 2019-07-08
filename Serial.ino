// SERIAL.H - LaSisC VERSION 
// VERSÃO SEM GPS

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define UART_NUMBER 4       // NÚMERO DE COMUNICAÇÕES UARTs
#define RX_BUFFER_SIZE 64   // TAMANHO DO VETOR PARA RECEPÇÃO
#define TX_BUFFER_SIZE 128  // TAMANHO DO VERTOR PARA TRANSMISSÃO DA Serial0 TX
#define INBUF_SIZE 64       //


//RX
static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];    //vetores das 4 posições de cabeça e das 4 posições de cauda                             //cabeça armazena os valores que são recebidos                     
                                                                                                                                                                         //cauda lê os valores armazenados 
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];                     //                               [0]   [1]   [2]   [3] 
                                                                                //                           [0]  
                                                                                //      serialBufferRX =     [1]
                                                                                //                            ~
                                                                                //                          [63]
   
                                                                      
//TX                                                              
static volatile uint8_t headTX,tailTX; // posições cabeça e cauda do vetor bufTX                                                                                        //cabeça escreve valores que serão transmitidos
static uint8_t bufTX[TX_BUFFER_SIZE];  // vetor que é escrito e depois enviado ( Serial0 TX)                        TX_BUFFER_SIZE = 128                                //cauda envia os valores escritos


//RXIN
static uint8_t inBuf[INBUF_SIZE];      // vetor utilizado na função read8(), read16() e read32();      INBUF_SIZE = 64



//----------------------------------------- Multiwii Serial Protocol 0 -----------------------------------------------------------------------------------------------------------------------------------------------------------
#define MSP_VERSION				 0

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
//#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan 
//#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
//#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
//#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_WP_SET               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4



#define MATLAB                    24  //protocolo para leitura no MATLAB

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
// Aqui são lidos os dados do vetor inBuf

static uint8_t indRX;                  // função para leitura de inBuff (variando valores de 8, 16 e 32bits)                                                    | 
                                       //  --------------------------------------------------------------------------------------                               ^
uint32_t read32() {                    // inBuf[indRX] = [(0000 0000) (0000 0001) (0000 0010) (0000 0011) (0000 0100) ....]                                     |
  uint32_t t = read16();               // t = (0000 0000 0000 0000 0000 0010 0000 0001)    indRX = 1                                                            |
  t+= (uint32_t)read16()<<16;          // t = (0000 0100 0000 0011 0000 0010 0000 0001)    indRX = 3 (valor de retorno)      read16() faz dois read8()          |
  return t;                            //  --------------------------------------------------------------------------------------                               |
}                                      //                                                                                                                       ^
uint16_t read16() {                    // inBuf[indRX] = [(0000 0000) (0000 0001) (0000 0010) (0000 0011) (0000 0100) ....]                                     |
  uint16_t t = read8();                // t = (0000 0000 0000 0001)    indRX = 0                                                                                |
  t+= (uint16_t)read8()<<8;            // t = (0000 0010 0000 0001)    indRX = 1 (valor de retono)                                                              |
  return t;                            //  --------------------------------------------------------------------------------------                               |
}                                      //                                                                                                                       ^
                                       //                                                                                                                       |                                                                                                         
uint8_t read8()  {                     // Retorna valor de 8bits da posição "indRX" do vetor "inBuf"                                                            |
  return inBuf[indRX++]&0xff;          // inBuf[indRX] = [(0000 0001) (0000 0010) (0000 0011) (0000 0100) (0000 0101) ....]                                     |
}                                      // retorno = (0000 0001)       indRX = 0                                                                                 |
                                       // operação AND com "0xff" para garantir operação com apenas 8 bits
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

static uint8_t cmdMSP;
static uint8_t checksum;


//FUNÇÃO QUE RECEBE TAMANHO DOS DADOS A SER TRANSMITIDOS E VERIFICAÇÃO DE ERROS
void headSerialResponse(uint8_t err, uint8_t s) {     
  serialize8('$');                                    // escreve '$' no vetor bufTX (buffer)
  serialize8('M');                                    // escreve 'M' no vetor bufTX (buffer)
  serialize8(err ? '!' : '>');                        // ternário =>    err => verdadeiro = '!'   err => falso = '>'
  checksum = 0;                                       // start calculating a new checksum
  serialize8(s);                                      // escreve a variável recebida "s" (size) no vetor bufTX (buffer)
  serialize8(cmdMSP);                                 // escreve a variável "cmdMSP" no vetor bufTX (buffer)
}

void headSerialReply(uint8_t s) {                     // Chama a função "headSerialResponse" com os parâmetros (0, s)     [ $ M > s cmdMSP ]         //s = size            
  headSerialResponse(0, s);                           
}

void inline headSerialError(uint8_t s) {              // Chama a função "headSerialResponse" com os parâmetros (1, s)     [ $ M ! s cmdMSP ]         //s = size
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum);UartSendData();                // Escreve a variável "checksum" no vetor bufTX (buffer)
}                                                     // Habilita a interrupção do buffer de transmissão, quando este está vazio para escrever. 

void serializeNames(PGM_P s) {                        // http://www.nongnu.org/avr-libc/user-manual/group__avr__pgmspace.html
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Aqui verifica espaço do vetor bufTx para transmissão 
//      verifica qual o protocolo para recepção e transmissão 
//      

void serialCom() {
  
  uint8_t c;                    // "c" recebe os dados armazenados na coluna 0 da matriz serialBufferRX 
  static uint8_t offset;        // incrementa posição no vetor inBuf
  static uint8_t dataSize;      // guarda tamanho do dado a ser recebido
  
  static enum _serial_state {                      //enum =>   http://linguagemc.com.br/enum-em-c/      
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;                                //variável do tipo enum      // https://www.geeksforgeeks.org/enumeration-enum-c/  
  
  while (SerialAvailable(0)) {                     // permanece nesse loop, apenas se a função "SerialAvailable" retornar verdadeiro. //retorna verdadeiro apenas se a cabeça for diferente da cauda [na coluna 0 da matriz serialBufferRX]
  
    uint8_t bytesTXBuff = ((uint8_t)(headTX-tailTX))%TX_BUFFER_SIZE;     // indica o número de bytes ocupados no bufTX (buffer)
    
                                                                         // [     |            |                  ]  =>  bufTX
                                                                         //       ^            ^                  ^
                                                                         //    tailTX        headTX        TX_BUFFER_SIZE
                                                                         //   
                                                                         // ex = tailTX         = 5              posição da cauda = 5
                                                                         //      headTX         = 10             posição da cabeça = 10
                                                                         //      TX_BUDDER_SIZE = 15             tamanho do vetor TX_BUDDER_SIZE = 15
                                                                         //     
                                                                         //  bytesTXBuff = (headTX - tailTX) % TX_BUDDER_SIZE => (10 - 3) % 15 = 7
                                                                         //  portando, no exemplo acima teríamos 7 bytes do vetor ocupados
                                                                         //  isso acontece porque, mesmo a cabeça tendo escrevido 10 bytes, já foram enviados 3 bytes pela cauda, liberando 3 bytes para serem escritos novamente
                                                                                                                                             
    if (bytesTXBuff > TX_BUFFER_SIZE - 40 ) return;                      //  garante que há espaço livre o suficiente no TX buffer para ir adiante.(margem de 40 bytes para continuar)       TX_BUFFER_SIZE = 128
                                                                         
    c = SerialRead(0);                                                   //  "c" recebe os dados armazenados na coluna 0 da matriz serialBufferRX 
    

    if (c_state == IDLE) {                                    // se c_state == IDLE :     
      c_state = (c=='$') ? HEADER_START : IDLE;               // ternário =>   (c=='$')   =>   [verdadeiro => c_state = HEADER_START]      [falso => c_state = IDLE]
      if (c_state == IDLE) evaluateOtherData(c);              // se c_state == IDLE :     =>   avaliar todos os outros dados seriais recebidos ;   caracteres diferentes de "$" 
      
    } else if (c_state == HEADER_START) {                     // se c_state == HEADER_START :       
      c_state = (c=='M') ? HEADER_M : IDLE;                   // ternário =>   (c=='M')   =>   [verdadeiro => c_state = HEADER_M]      [falso => c_state = IDLE]
      
    } else if (c_state == HEADER_M) {                         // se c_state == HEADER_M : 
      c_state = (c=='<') ? HEADER_ARROW : IDLE;               // ternário =>   (c=='<')   =>   [verdadeiro => c_state = HEADER_ARROW]      [falso => c_state = IDLE]
      
    } else if (c_state == HEADER_ARROW) {                     // se c_state == HEADER_ARROW : 
      if (c > INBUF_SIZE) {                                        // Se (c > INBUF_SIZE) : esperando o tamanho da carga útil 
        c_state = IDLE;                                            // c_state = IDLE;
        continue;                                                  //http://linguagemc.com.br/o-comando-continue/    pula para próxima iteração.
      }
      dataSize = c;                                                          // tamanho do dado
      offset = 0;                                                            // zera offset
      checksum = 0;                                                          // zera checksum
      indRX = 0;                                                             // zera vetor posição da função  "read8()"
      checksum ^= c;                                                         // XOR 
      c_state = HEADER_SIZE;  // the command is to follow                    // c_state = HEADER_SIZE
      
    } else if (c_state == HEADER_SIZE) {                                     // se c_state == HEADER_SIZE :
      cmdMSP = c;                                                            // cmdMSP recebe c
      checksum ^= c;                                                         // XOR 
      c_state = HEADER_CMD;                                                  // c_state = HEADER_CMD
      
    } else if (c_state == HEADER_CMD && offset < dataSize) {                 // se c_state == HEADER_CMD  E   offset < dataSize:
      checksum ^= c;                                                         // XOR 
      inBuf[offset++] = c;                                                   // Vetor inBuff recebe "c" na posição offset
      
    } else if (c_state == HEADER_CMD && offset >= dataSize) {                // se c_state == HEADER_CMD  E   offset >= dataSize:
      if (checksum == c) {                                                   // compare calculated and transferred checksum   
        evaluateCommand();                                                   // we got a valid packet, evaluate it
      }
      c_state = IDLE;                                                        // c_state = IDLE                       
    }
  }
}



//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//               IDLE      =>      HEADER_START     =>     HEADER_M       =>       HEADER_ARROW       =>         HEADER_SIZE       =>          HEADER_CMD              =>                                    => retorna INICIO
//                       c = '$'                  c = 'M'               c = '<'                  c = dataSize                  c = cmdMSP                     inBuf[0]          = c
//                                                                                                                                                            inBuf[1]          = c         
//                                                                                                                                                            inBuf[2]          = c
//                                                                                                                                                            inbuf[dataSize-1] = c
//                                                                                                                                                            RECEBE MAIS UM PARA VERIFICAÇAO DE ERRO COM O SOFTWARE
//
//
//               recebe o pacote   $ M > s cmdMSP      onde, s é o tamanho do dado e cmdMSP o comando do protocolo          

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void evaluateCommand() {
  switch(cmdMSP) {
    
   case MSP_SET_RAW_RC:                                   
     for(uint8_t i=0;i<8;i++) {
       rcData[i] = read16();
     }
     headSerialReply(0);
     break;
     
   case MSP_SET_PID:                                   
     for(uint8_t i=0;i<PIDITEMS;i++) {
       conf.P8[i]=read8();
       conf.I8[i]=read8();
       conf.D8[i]=read8();
     }
     headSerialReply(0);
     break;
     
   case MSP_SET_BOX:
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {            
       conf.activate[i]=read16();
     }
     headSerialReply(0);
     break;
     
   case MSP_SET_RC_TUNING:                             
     conf.rcRate8 = read8();
     conf.rcExpo8 = read8();
     conf.rollPitchRate = read8();
     conf.yawRate = read8();
     conf.dynThrPID = read8();
     conf.thrMid8 = read8();
     conf.thrExpo8 = read8();
     headSerialReply(0);
     break;
     
   /*case MSP_SET_MISC:                                  
     #if defined(POWERMETER)
       conf.powerTrigger1 = read16() / PLEVELSCALE;
     #endif
     headSerialReply(0);
     break;*/
     
   case MSP_IDENT:                                     
     headSerialReply(7);
     serialize8(VERSION);   // multiwii version
     serialize8(MULTITYPE); // type of multicopter
     serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
     serialize32(0);        // "capability"
     break;
     
   case MSP_STATUS:
     headSerialReply(10);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
     serialize32(f.ACC_MODE<<BOXACC|f.BARO_MODE<<BOXBARO|f.MAG_MODE<<BOXMAG|f.ARMED<<BOXARM|
                 rcOptions[BOXCAMSTAB]<<BOXCAMSTAB | rcOptions[BOXCAMTRIG]<<BOXCAMTRIG |
                 f.GPS_HOME_MODE<<BOXGPSHOME|f.GPS_HOLD_MODE<<BOXGPSHOLD|f.HEADFREE_MODE<<BOXHEADFREE|
                 f.PASSTHRU_MODE<<BOXPASSTHRU|rcOptions[BOXBEEPERON]<<BOXBEEPERON|rcOptions[BOXLEDMAX]<<BOXLEDMAX|rcOptions[BOXLLIGHTS]<<BOXLLIGHTS|rcOptions[BOXHEADADJ]<<BOXHEADADJ);
     break;
     
   case MSP_RAW_IMU:
     headSerialReply(18);
     for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]);
     for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]);
     for(uint8_t i=0;i<3;i++) serialize16(magADC[i]);
     break;
     
   /*case MSP_SERVO:
     headSerialReply(16);
     for(uint8_t i=0;i<8;i++)
       #if defined(SERVO)
       serialize16(servo[i]);
       #else
       serialize16(0);
       #endif
     break;*/
     
   case MSP_MOTOR:
     headSerialReply(16);
     for(uint8_t i=0;i<8;i++) {
       serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
     
   case MSP_RC:
     headSerialReply(16);
     for(uint8_t i=0;i<8;i++) serialize16(rcData[i]);
     break;
     
   case MSP_ATTITUDE:
     headSerialReply(8);
     for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
     serialize16(heading);
     serialize16(headFreeModeHold);
     break;
     
   case MSP_ALTITUDE:
     headSerialReply(4);
     serialize32(EstAlt);
     break;
     
   case MSP_RC_TUNING:
     headSerialReply(7);
     serialize8(conf.rcRate8);
     serialize8(conf.rcExpo8);
     serialize8(conf.rollPitchRate);
     serialize8(conf.yawRate);
     serialize8(conf.dynThrPID);
     serialize8(conf.thrMid8);
     serialize8(conf.thrExpo8);
     break;
     
   case MSP_PID:
     headSerialReply(3*PIDITEMS);
     for(uint8_t i=0;i<PIDITEMS;i++) {
       serialize8(conf.P8[i]);
       serialize8(conf.I8[i]);
       serialize8(conf.D8[i]);
     }
     break;
     
   case MSP_BOX:
     headSerialReply(2*CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize16(conf.activate[i]);
     }
     break;
     
   case MSP_BOXNAMES:
     headSerialReply(strlen_P(boxnames));
     serializeNames(boxnames);
     break;
     
   case MSP_PIDNAMES:
     headSerialReply(strlen_P(pidnames));
     serializeNames(pidnames);
     break;
     
   /*case MSP_MISC:
     headSerialReply(2);
     serialize16(intPowerTrigger1);
     break;*/
     
   case MSP_MOTOR_PINS:
     headSerialReply(8);
     for(uint8_t i=0;i<8;i++) {
       serialize8(PWM_PIN[i]);
     }
     break; 
     
   case MSP_RESET_CONF:
     conf.checkNewConf++;
     checkFirstTime();
     headSerialReply(0);
     break;
     
   case MSP_ACC_CALIBRATION:
     calibratingA=400;
     headSerialReply(0);
     break;
     
   case MSP_MAG_CALIBRATION:
     f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;
     
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
     
   case MATLAB:
     headSerialReply(16);
     for(uint8_t i=0;i<2;i++) serialize16(angle[i]);       // angulos roll e pitch
     for(uint8_t w=0;w<2;w++) serialize16(rcCommand[w]);   // set points
     for(uint8_t j=0;j<4;j++)serialize16(motor[j]);        // ação de controle (pwm motores)
     break;
     
  /* case MSP_DEBUG:
     headSerialReply(8);
     for(uint8_t i=0;i<4;i++) {
       serialize16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;*/
     
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Avaliar todos os outros dados seriais recebidos
void evaluateOtherData(uint8_t sr) {
  switch (sr) {
  // Note: we may receive weird characters here which could trigger unwanted features during flight.
  //       this could lead to a crash easily.
  //       Please use if (!f.ARMED) where neccessary
    
  }
}



// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Aqui serão escritos os dados no vetor bufTX (buffer)

void serialize32(uint32_t a) {           // Função recebe valor "a" de 32 bits                 ex =   a = (1000 0111 0110 0101 0100 0011 0010 0001)                                            ^
  serialize8((a    ) & 0xFF);            // serialize8 com [0 - 7] bits de "a"                 serialize8 (0010 0001)                                                                          |
  serialize8((a>> 8) & 0xFF);            // serialize8 com [8 - 15] bits de "a"                serialize8 (0100 0011)                                                                          |
  serialize8((a>>16) & 0xFF);            // serialize8 com [16 - 23] bits de "a"               serialize8 (0110 0101)                                                                          |
  serialize8((a>>24) & 0xFF);            // serialize8 com [24 - 31] bits de "a"               serialize8 (1000 0111)                                                                          |
}                                        //                                                                                                                                                    ^
                                         // -----------------------------------------------------------------------------------------------------------------------------------                |
void serialize16(int16_t a) {            // Função recebe valor "a" de 16 bits                            ex =   a = (0100 0011 0010 0001)                                                     |
  serialize8((a   ) & 0xFF);             // serialize8 com os 8 bits menos significativos de "a"                 serialize8 (0010 0001)                                                        |
  serialize8((a>>8) & 0xFF);             // serialize8 coms os 8 bits mais significativos de "a"                 serialize8 (0100 0011)                                                        |
}                                        //                                                                                                                                                    ^
                                         //  ----------------------------------------------------------------------------------------------------------------------------------                |
void serialize8(uint8_t a) {             // Função recebe valor "a" de 8 bits                                                                                                                  |
  uint8_t t = headTX;                    // t = posição da cabeça do vetor bufTX                                                                                                               |
  if (++t >= TX_BUFFER_SIZE) t = 0;      // ++t = incrementa t em um e retorna o novo valor de t;  se(t >= 128) volta-se para início do vetor => bufTX[0];       TX_BUFFER_SIZE =   = 128;     |   
  bufTX[t] = a;                          // posição t do vetor bufTX recebe "a"                                                                                                                ^
  checksum ^= a;                         // checksum = (checksum ^ a);   ( ^ =  XOR);    (verifica mudança entre checksum anterior e o valor recebido "a" )                                    |
  headTX = t;                            // headTX = t    (pois houve ++t acima)    nova posição da cabeça                                                                                     |
}                                        // https://www.arduino.cc/reference/pt/language/structure/compound-operators/increment/                                                               |
                                         //                                                                                                                                                    |
                                         
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------                                      
//Aqui serão transmitidos os dados do vetor bufTX (buffer)
//Aqui serve apenas para a Serial0 TX

ISR_UART {                                               // ISR_UART = ISR(USART0_UDRE_vect)         
                             
  uint8_t t = tailTX;                                    // t = posição da cauda do vetor bufTX 
  
  if (headTX != t) {                                     // Se a posição da cabeça for diferente da cauda
    if (++t >= TX_BUFFER_SIZE) t = 0;                    // Se a posição da cauda+1 for maior que 128, posição reseta para 0;      TX_BUFFER_SIZE = 128
    UDR0 = bufTX[t];                                     // Transmite dado da posição cauda+1  (UDR0 => transmit buffer)
    tailTX = t;                                          // tailTX = t    (pois houve ++t acima)=> nova posição da cauda
  }
  if (t == headTX) UCSR0B &= ~(1<<UDRIE0);               // Se cauda for igual cabeça => Se todos os dados foram transmitido, desliga-se a interrupção de transmissão. 
}                                                             
                                                         // Interrupção => Data Register Empty                   
                                                         // Interrupção pelo vetor => USART0_UDRE_vect 
                                                         
                                                         // UDRIEn: USART Data Register Empty Interrupt Enable n
                                                         // Writing this bit to one enables interrupt on the UDREn Flag. 
                                                         // A Data Register Empty interrupt will be generated only if the UDRIEn bit is written to one, 
                                                         // the Global Interrupt Flag in SREG is written to one and the UDREn bit in UCSRnA is set.
                                                         
                                                         // UDREn: USART Data Register Empty
                                                         // The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. 
                                                         // If UDREn is one, the buffer is empty, and therefore ready to be written.
                                                         // The UDREn Flag can generate a Data Register Empty interrupt (see description of the UDRIE bit). 
                                                         // UDREn is set after a reset to indicate that the Transmitter is ready.
                                                         
                                                         // UDRn – USART I/O Data Register n
                                                         // The transmit buffer can only be written when the UDREn Flag in the UCSRnA Register is set. 
                                                         // Data written to UDRn when the UDREn Flag is not set, will be ignored by the USART Transmitter. 
                                                         // When data is written to the transmit buffer, and the Transmitter is enabled, the Transmitter will load the data into the Transmit Shift Register when the
                                                         // Shift Register is empty. Then the data will be serially transmitted on the TxDn pin.
                                                         
// Quando a flag UDRE = 1, indica-se que o Buffer de transmissão (UDR0) está vazio, ou seja, pronto para ser escrito, gerando uma interrupção através do vetor USART0_UDRE_vect, que apenas verifica a flag dita. 
 


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Aqui os registradores para a comunicação serial serão configurados

void UartSendData() {           // Habilita a interrupção do buffer de transmissão, quando este está vazio para escrever.  
    UCSR0B |= (1<<UDRIE0);                                                                                    
}

static void inline SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;                                                              // Modo = Asyncrono Double Speed => UBRRn = (Fosc / 8 / baud -1)         = 16.36
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);                                                                   //                               => UBRRn = ((F_CPU  / 4 / baud -1) / 2) = 16.86  como é uma variável inteira, ignora-se depois da virgula   // F_CPU = 16 MHZ
  switch (port) {                                                                                                                                                                                                                                          // baud = 115200
    case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;  // U2Xn: Double the USART Transmission Speed
    case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;  // This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation. 
    case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;  // Writing this bit to one will reduce the divisor of the baud rate divider from 16 to 8 effectively doubling 
    case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;  // the transfer rate for asynchronous communication.
                                                                                                              // Esse bit é setado para ativar o modo "Asyncrono Double Speed"
  }                                                    
}                                                                                                             // UBRRnL and UBRRnH – USART Baud Rate Registers
                                                                                                              // UBRR11:0: USART Baud Rate Register
                                                                                                              // This is a 12-bit register which contains the USART baud rate. The UBRRH contains the four most significant bits,
                                                                                                              // and the UBRRL contains the eight least significant bits of the USART baud rate. Ongoing transmissions by the
                                                                                                              // Transmitter and Receiver will be corrupted if the baud rate is changed. Writing UBRRL will trigger an immediate
                                                                                                              // update of the baud rate prescaler.
                                                                                                               
                                                                                                              // RXENn: Receiver Enable n
                                                                                                              // Writing this bit to one enables the USART Receiver. 
                                                                                                              
                                                                                                              // TXENn: Transmitter Enable n
                                                                                                              // Writing this bit to one enables the USART Transmitter.
                                                                                                              
                                                                                                              // RXCIEn: RX Complete Interrupt Enable n
                                                                                                              // Writing this bit to one enables interrupt on the RXCn Flag. A USART Receive Complete interrupt will be generated
                                                                                                              // only if the RXCIEn bit is written to one, the Global Interrupt Flag in SREG is written to one and the RXCn bit in
                                                                                                              // UCSRnA is set.
                                                                                                              
                                               // ALERTA = initial value UCSRNC = 00000110
                                               // Asynchronous USART
                                               // Parity Mode = DISABLE
                                               // Stop bit = 1-bit
                                               // Character Size = 8-bit


                                                         
static void inline SerialEnd(uint8_t port) {                                        // Desliga os registradores RXENn, TXENn, RXCIEn, UDRIE0. Todos já citados acima.  
  switch (port) {
    case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
    case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)); break;
    case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)); break;
    case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)); break;
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Aqui são armazenados os dados na matriz  serialBufferRX

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {               // recebe o dado para armazenar na matriz, e a coluna da matriz, referente á porta RX0, RX1, RX2 ou RX3 
  uint8_t h = serialHeadRX[portnum];                                                // h = posição da cabeça da linha da coluna específica (portnum)                  serialHeadRX[UART_NUMBER];      UART_NUMBER = 4
  if (++h >= RX_BUFFER_SIZE) h = 0;                                                 // se (cabeça+1 >= 64), reseta posição da cabeça para 0                           RX_BUFFER_SIZE = 64
  if (h == serialTailRX[portnum]) return;                                           // se cabeça+1 for igual a posição da cauda da linha da coluna específica, não retona nada. Não escrever em cima do valor ainda não lido. 
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;                            // escreve-se na matriz o dado "data" na linha "cabeça+1" e coluna "portnum. 
  serialHeadRX[portnum] = h;                                                        // (pois houve ++h acima) => nova posição da cabeça da linha da coluna específica (portnum)      
}



ISR(USART1_RX_vect) { store_uart_in_buf(UDR1, 1); }                                // armazena os dados recebidos por UDR1 na coluna 1 da matriz serialBufferRX
ISR(USART0_RX_vect) { store_uart_in_buf(UDR0, 0); }                                // armazena os dados recebidos por UDR0 na coluna 0 da matriz serialBufferRX
ISR(USART2_RX_vect) { store_uart_in_buf(UDR2, 2); }                                // armazena os dados recebidos por UDR2 na coluna 2 da matriz serialBufferRX
ISR(USART3_RX_vect) { store_uart_in_buf(UDR3, 3); }                                // armazena os dados recebidos por UDR3 na coluna 3 da matriz serialBufferRX
   
                                                                                   
                                                                                   // Interrupção pelo vetor => USART1_RX_vect
                                                                                   // Interrupçào => USART Receive Complete 
                                                                                   
                                                                                   // RXCn: USART Receive Complete
                                                                                   // This flag bit is set when there are unread data in the receive buffer and cleared
                                                                                   // when the receive buffer is empty (that is, does not contain any unread data).
                                                                                   // The RXCn Flag can be used to generate a Receive Complete interrupt (see description of the RXCIEn bit).
                                                                                   
                                                                                   // RXCIEn: RX Complete Interrupt Enable n  (ativado acima) 
                                                                                   // Writing this bit to one enables interrupt on the RXCn Flag. A USART Receive Complete interrupt will be generated
                                                                                   // only if the RXCIEn bit is written to one, the Global Interrupt Flag in SREG is written to one and the RXCn bit in
                                                                                   // UCSRnA is set.

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Aqui são lidos os dados na matriz serialBufferRx

uint8_t SerialRead(uint8_t port) {                          // port = coluna da matriz serialBufferRX =>  RX0, RX1, RX2 ou RX3  
  uint8_t t = serialTailRX[port];                           // t    = posição da cauda da linha da coluna específica (port)                                            
  uint8_t c = serialBufferRX[t][port];                      // c    = dado armazenado na matriz serialBufferRX na linha "t" e coluna "port"                        //          [0] [1] [2] [3]   =>  (UART_NUMBER)
  if (serialHeadRX[port] != t) {                            // se a posição cabeça é diferente da cauda                                                            //       [0]     15
    if (++t >= RX_BUFFER_SIZE) t = 0;                       // se (cauda+1 >= 64), reseta posição da cauda para 0                                                  //       [1]    
    serialTailRX[port] = t;                                 // (pois houve ++t acima) => nova posição da cauda                                                     //        ~
  }                                                                                                                                                                //      [63]       
  return c;                                                 // retorna o dado para leitura                                                                         //                 (t = 0; port = 1;) => c = 15
}
  
uint8_t SerialAvailable(uint8_t port) {                     // recebe a coluna da matriz serialBufferRX =>  RX0, RX1, RX2 ou RX3  
  return (serialHeadRX[port] != serialTailRX[port]);        // retorna verdadeiro se a posição cabeça for diferente de cauda 
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Aqui seleciona a porta TX para fazer transmissão. 

void SerialWrite(uint8_t port,uint8_t c){                          // Aqui recebe a porta TX (port = 0 ,1 ,2 ou 3; e o dado para ser enviado (c)   // Serial0 TX é dirigido via buffer(bufTX) e uma interrupção de fundo.
                                                                                                                                                   // Serial1 Serial2 e Serial3 TX não são dirigidas via interrupção.
 switch (port) {                                                   // Switch de acordo com a porta TX                                                               
    case 0: serialize8(c);UartSendData(); break;                   // porta = 0; serialize8 => escreve o dado no buffer;   UartSendData => habilita a interrupção para envio.                            
    case 1: while (!(UCSR1A & (1 << UDRE1))) ; UDR1 = c; break;    // porta = 1;  envia o dado "c" via  UDR1;   Sai do looping vazio While e envia, apenas quando UDRE1 = 1            
    case 2: while (!(UCSR2A & (1 << UDRE2))) ; UDR2 = c; break;    // porta = 2;  envia o dado "c" via  UDR2;   Sai do looping vazio While e envia, apenas quando UDRE2 = 1
    case 3: while (!(UCSR3A & (1 << UDRE3))) ; UDR3 = c; break;    // porta = 3;  envia o dado "c" via  UDR3;   Sai do looping vazio While e envia, apenas quando UDRE3 = 1
  }                                                                // UDREn indica se o UDRn está pronto para receber novos dados. Se UDREn = 1, UDRn está vazio e pronto para ser escrito.   
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
