// Sensors - LaSisC VERSION 
// VERSÃO SEM GPS

//https://www.youtube.com/watch?v=Mx-srugCZgg
//https://newbiehack.com/I2C-TWI-FirstExample.aspx

/***-------------------- I2C address------------------------------- ***/
// Aqui é definido o endereço dos dispositivos para a interface i2c

    #define BMA180_ADDRESS 0x40    // Endereço do Acelerômetro = 0x40
    #define ITG3200_ADDRESS 0X68   // Endereço do Giroscópio = 0x68

//----------------------------------------------------------------------

//Configurações para os registradores do giroscópio. 
    #define ITG3200_SMPLRT_DIV 0  //8000Hz
    #define ITG3200_DLPF_CFG   0

uint8_t rawADC[6];                    // vetor para receber as saídas crus do acelerômetro e giroscópio. 

static uint32_t neutralizeTime = 0;   // tempo decorrido até erro de transmissão TWI ( atualizado em waitTransmissionI2C())


// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     Exemplo do ADXL=
//
//     Single-Byte Write => 
//     | Master| Start | | Slave Address + Write |     | Register Address |     |       Data      |     | Stop |
//     | Slave |                                 | ACK |                  | ACK |                 | ACK |      |
//
//     Multiple-Byte Write =>
//     | Master| Start | | Slave Address + Write |     | Register Address |     |       Data      |     |       Data      |     | Stop |
//     | Slave |                                 | ACK |                  | ACK |                 | ACK |                 | ACK |      |
//
//     Single-Byte Read =>
//     | Master| Start | | Slave Address + Write |     | Register Address |     | Repeat Start | | Slave Address + Read |                         | NACK | | Stop |
//     | Slave |                                 | ACK |                  | ACK |                                       | ACK | |       Data      |               |  
//
//     Multiple-Byte Read =>
//     | Master| Start | | Slave Address + Write |     | Register Address |     | Repeat Start | | Slave Address + Read |                         | ACK |                 | NACK | | Stop |
//     | Slave |                                 | ACK |                  | ACK |                                       | ACK | |       Data      |     |       Data      |               |        
//
//
//  Slave Address + Write/Read    =>  Write = 0 / Read  = 1
//        7bits        1bit       
// 
//  Slave Address         =   0x1D      
//                        =   0011101                              
//
//  Slave Address + Write =   0x3A       (shift right +0)
//                        =   0111010
//
//  Slave Address + Read  =   0x3B       (shift right +1)
//                        =   0111011
//
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Aqui se faz as configurações para iniciar a comunicação TWI --------------------------------------------------------------------------------------------------------------------------------------------------------------
void i2c_init(void) {
  
  I2C_PULLUPS_DISABLE                          // PORTD &= ~(1<<0); PORTD &= ~(1<<1)     [PORTD AND (11111110)]; [PORTD AND (11111101)];       macro em def.h
                                               // (PD0) e (PD1) com resistores de pull-up desabilitados 
                                               
  // Cálculo do período de SCL quando operando em Modo mestre--------------------------------------------------------------------  
  
  TWSR = 0;                                    // Configuração do Prescaler para o controle do período do SCL
                                               // TWPS1 = 0  TWPS0 = 0  =>  no prescaler => prescaler = 1  => TWPS = 0       
  
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // Registrador para Unidade geradora de Bit Rate                                         
                                               // TWBR = 72   para  F_CPU = 16Mhz e I2C_SPEED = 100Khz        // 4^(TWPS) = 1
                                               
  //-----------------------------------------------------------------------------------------------------------------------------                                             
  
  TWCR = 1<<TWEN;                              // TWEN: TWI Enable Bit: Este bit habilita a operação TWI e ativa a interface TWI. Quando TWEN é escrito em 1, O TWI toma controle sobre os pinos I/O conectados no SCL e SDA,        
}                                              //       Se este bit é escrito em 0, O TWI é desligado e todas transmissôes TWI são encerradas de qualquer operação no momento. (Sem interrupção)

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Condição de Start e Repeat Start ---------------------------------------------------------------------------------------------
void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  // Limpa a Flag TWINT; Gera condição de Início; Habilita TWI. 
  
                                               // TWINT = TWI Interrupt Flag: Esta flag é setada (nível lógico 0) pelo próprio hardware quando o TWI termina seu trabalho atual e espera resposta do software
                                               //         A flag TWINT deve ser limpada pelo software escrevendo-se um valor lógico alto (1).
                                               //         Limpando essa flag, inicia-se a operação do TWI, então todos os acessos para o TWI Address Register (TWAR), TWI Status Register (TWSR) e TWI Data Register (TWDR) devem estar completos antes de limpar esta flag. 
  
                                               // TWSTA = TWI START Condition Bit = Escreve-se o bit TWSTA para 1 quando este deseja se tornar o Master (mestre) na comunicação TWI. O software TWI checa se a linha está disponível, e gera uma condição de início se esta
                                               //         estiver liver. Contudo, se a via não estiver livre, O TWI espera até uma condição de parada ser detectada, e então gera uma nova condição de início para reivindicar o status de Master. 
                                               //         TWSTA deve ser limpado pelo softwware quando a condição de início ser transmitida. 
                                               
                                               // TWEN = TWI Enable Bit: Este bit habilita a operação TWI e ativa a interface TWI. Quando TWEN é escrito em 1, O TWI toma controle sobre os pinos I/O conectados no SCL e SDA,        
                                               //        Se este bit é escrito em 0, O TWI é desligado e todas transmissôes TWI são encerradas de qualquer operação no momento. (Sem interrupção)

  
  waitTransmissionI2C();                       // Espera a transmissão ser completada. (Flag TWINT ser setada)
  
  TWDR = address;                              // Envia endereço do slave.        (com bit de escrita ou leitura)
  TWCR = (1<<TWINT) | (1<<TWEN);               // Limpa a Flag TWINT; Habilita TWI 
  waitTransmissionI2C();                       // Espera a transmissão ser completada. (Flag TWINT ser setada)
}
//-------------------------------------------------------------------------------------------------------------------------------

// Condição de Stop -------------------------------------------------------------------------------------------------------------
void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Limpa a Flag TWINT; Habilita TWI; Gera condição de parada 
}                                                    
                                                     // TWSTO = Quando TWSTO = 1 em modo mestre, será gerada uma condição de parada na linha de comunicação
                                                     // Quando a condição de parada é executada na linha, o bit TWSTO é limpado automaticamente 
//-------------------------------------------------------------------------------------------------------------------------------

// Envio de Dados ---------------------------------------------------------------------------------------------------------------
void i2c_write(uint8_t data ) {	               
  TWDR = data;                                 // envia dados para o dispositivo endereçado anteriormente 
  TWCR = (1<<TWINT) | (1<<TWEN);               // Limpa a Flag TWINT; Habilita TWI 
  waitTransmissionI2C();                       // Espera a transmissão ser completada. (Flag TWINT ser setada)
}
//-------------------------------------------------------------------------------------------------------------------------------

// Leitura dos dados recebidos---------------------------------------------------------------------------------------------------
uint8_t i2c_read(uint8_t ack) {                               // RECEBE =>    1 = ACK     0 = NACK
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);       // Limpa a Flag TWINT; Habilita TWI; Checa ternário
                                                              // ack => verdadeiro (1) = (1<<TWEA)   ack => falso (0) = 0
                                                              
                                                              // TWEA = O bit TWEA controla a geração de um pulso acknowledge. Se TWEA=1, O PULSO ACK é gerado na linha TWI, se as seguintes condições forem encontradas:
                                                              //        * O endereço do dispositivo escravo foi recebido
                                                              //        * Uma chamada geral foi recebida. 
                                                              //        * Um byte de dados foi recebido nos modos Master Receive ou Slave receiver. [no caso em uso, Master Receive]
                                                              //        Se TWEA = 0, o dispositivo pode ser virtualmente desconectado da linha temporariamente. 
                                                              
  waitTransmissionI2C();                                      // Espera a transmissão ser completada. (Flag TWINT ser setada)
  uint8_t r = TWDR;                                           // Recebe dado enviado pelo slave, e armazena-o na variável "r" 
  if (!ack) i2c_stop();                                       // Se recebeu Nack, gera-se condição de parada
  return r;                                                   // retorna valor em "r"  
}
//------------------------------------------------------------------------------------------------------------------------------

// Leitura de Acknowledge ------------------------------------------------------------------------------------------------------
uint8_t i2c_readAck() {
  return i2c_read(1);         // SCL(9) => SLA = 1 => ACK     //No nono clock tem-se o retorno do slave, se esse for alto, tem-se ACK
}
//------------------------------------------------------------------------------------------------------------------------------

// Leitura de Not Acknowledge --------------------------------------------------------------------------------------------------
uint8_t i2c_readNak(void) {
  return i2c_read(0);         // SCL(9) => SLA = 0 => ACK    //No nono clock tem-se o retorno do slave, se esse for baixo, tem-se NACK
}
//------------------------------------------------------------------------------------------------------------------------------

// Espera de transmissão TWI ---------------------------------------------------------------------------------------------------
void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {  // Fica no loop enquanto a Flag TWINT estiver limpa.
    count--;
    if (count==0) {              // Se o contador zerar. Estamos em um estado de bloqueio, e não insistimos.
      TWCR = 0;                  // Reset forçado no registrador de controle (TWCR).
      neutralizeTime = micros(); // tira-se um fuso para neutrazilar o valor durante um curto delay 
      i2c_errors_count++;        // soma-se ao número de erros da comunicação.
      break;                     // Sai da rotina while
    }
  }
}
//------------------------------------------------------------------------------------------------------------------------------

// Leitura dos dados para o Buffer ---------------------------------------------------------------------------------------------                                          -------------------------------------
                                                                                                                                                                                                        //    |
// http://linguagemc.com.br/ponteiros-em-c/                                                                                                                                                                   |
// https://www.ime.usp.br/~pf/algoritmos/aulas/pont.html                                                                                                                                                //    ^
//                                                                                                                                                                                                            |
size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {  // endereço do dispositivo, endereço do vetor de dados, numero de bytes a serem recebidos                                                      |
                                                                                                                                                                                                        //    |
                                                                                                                                                                                                        //    |
  i2c_rep_start((add<<1) | 1);	// bit LSB = 1 => modo de leitura              //   |0|0|0|0|0|0|0|  |1|                                                                                                      |
                                                                               //     add(7bits)     R/W                                                                                                      |
                                                                               //                  R=1/W=0                                                                                                    ^
                                                                               // i2c_rep_start((add<<1) | 1) => Envia condição de start e 7bits de endereço + 1 bit de R/W                                   |
                                                                                                                                                                                                        //    |
  size_t bytes_read = 0;                                                       // variável para contar número de bytes lido                                                                                   |
                                                                                                                                                                                                        //    |
  uint8_t *b = (uint8_t*)buf;                                                  //  mudou para um ponteiro de inteiro (cast)                                                                                |
                                                                               //  https://www.vivaolinux.com.br/dica/Operador-cast                                                                     //    |
                                                                               //  https://www.youtube.com/watch?v=jG_Pp_Cozwc                                                                          //    ^
  while (size--) {                                                             // Enquanto ainda haver dados, mais bytes a ser recebidos                                                                      |
    /* acknowledge all but the final byte */                                   // 6=>5 ack   5=>4 ack  4=>3 ack  3=>2 ack  2=>1 ack  1=>0 nack                                                                                                                        //    |
    *b++ = i2c_read(size > 0);                                                 // leia-se cada byte, incrementando byte a byte no vetor, assim, incrementando a posicao                                       |
    /* TODO catch I2C errors here and abort */                                 // i2c_read(1) => ack    i2c_read(0) => nack                                                                             //    |
    bytes_read++;                                                              // calcula o número de bytes lidos                                                                                       //    |
  }                                                                                                                                                                                                     //    |
                                                                                                                                                                                                        //    ^
  return bytes_read;                                                           // retorna número de bytes lidos                                                                                               |
                                                                                                                                                                                                        //    |
}                                                                                                                                                                                                       //    | 
//-----------------------------------------------------------------------------------------------------------------------------                                                                               |
                                                                                                                                                                                                        //    |
// Envio do registrador de controle -------------------------------------------------------------------------------------------                                                                               |
                                                                                                                                                                                                        //    ^
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {  // endereço do dispositivo; registrador de controle; endereço do vetor de dados; numero de byte a serem recebidos             |
                                                                                                                                                                                                        //    |
  i2c_rep_start(add<<1); // bit LSB = 0 => modo de escrita                     //   |0|0|0|0|0|0|0|  |0|                                                                                                      |
                                                                               //     add(7bits)     R/W                                                                                                      |
                                                                               //                  R=1/W=0                                                                                                    |
                                                                                                                                                                                                        //    ^
                                                                                                                                                                                                        //    |
  i2c_write(reg);                                                              // Envia o registrador de controle                                                                                             |
  return i2c_read_to_buf(add, buf, size);                                      // Passa o endereço do dispositivo ; endereço do vetor de dados; tamanho dos dados a serem recebidos                           |
}                                                                                                                                                                                                       //    |
                                                                                                                                                                                                        //    |
//------------------------------------------------------------------------------------------------------------------------------                                         --------------------------------------

// Inversão dos bytes-----------------------------------------------------------------------------------------------------------
//                                        __________________________________
//                                        |                                |
//                                    ---------                         ---------
//                          0001 0010 0011 0100 => (swap_endianness) => 0011 0100 0001 0010
//                          --------                                              ---------      
//                             |_______________________________________________________|
//                             
//                                            
  
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}
//---------------------------------------------------------------------------------------------------------------------------

//Recebe os dados do dispositivo referente aos eixos-------------------------------------------------------------------------
void i2c_getSixRawADC(uint8_t add, uint8_t reg) {     // Endereço do dispositivo; Registrador de controle;
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);          // repassa para a função i2c_read_reg_to_buf
}
//---------------------------------------------------------------------------------------------------------------------------

// Escreve no Registrador de Controle----------------------------------------------------------------------------------------
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {   // Endereço do dispositivo; registrador de controle do dispositivo; configuração a ser escrita no registrador do dispositivo
  
  
  i2c_rep_start(add<<1); // bit LSB = 0 => modo de escrita                     //   |0|0|0|0|0|0|0|  |0|                                                                                                      
                                                                               //     add(7bits)     R/W                                                                                                      
                                                                               //                  R=1/W=0                                                                                                    
                                                                                                                              
  i2c_write(reg);        // escreve registrador de controle
  i2c_write(val);        // escreve o valor da configuração  no registrador do dispositivo
  i2c_stop();            // escreve condição de parada
}
//--------------------------------------------------------------------------------------------------------------------------

// Leitura do Registrador de Controle --------------------------------------------------------------------------------------
uint8_t i2c_readReg(uint8_t add, uint8_t reg) {           // Endereço do dispositivo; registrador de controle do dispositivo;
  uint8_t val;                                            // varialvel para armazenar configuração do registrador de controle 
  i2c_read_reg_to_buf(add, reg, &val, 1);                 // &val => endereço da variável que irá armazenara configuração do registrador de controle  => rawADC (GIROSCOPIO E ACELEROMETRO) 
                                                          //                                                                                          => bmp085_ctx.ac1(BAROMETRO) primeira posição da struct bmp085_ctx
  return val;                                             // retorna configuração do registrador de controle
}
//--------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ****************
// GYRO common part
// ****************

// Função de calibração do giroscópio 
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];                                 // vetor para armazenar a soma de 400 leituras da taxa de variação angular em torno de cada eixo. 
  uint8_t axis;                                        // declara variável para os 3 eixos 
  
  if (calibratingG>0) {                                // A calibração se inicia ao inicializar o drone ou ao comondao => throttle=min, yaw=left, pitch=min
    for (axis = 0; axis < 3; axis++) {                 // calibratingG = 400 no início da  calibração do giroscópio
      // Reset g[axis] at start of calibration
      if (calibratingG == 400) g[axis]=0;              // reseta g[axis] no início da calibração 
      // Soma de 400 leituras
      g[axis] +=gyroADC[axis];                         // g[axis] recebe a saída já tratada do giroscópio
      // Limpa variáveis globais para próxima leitur
      gyroADC[axis]=0;                                 // garante a reinicilização dos valores de saída do giroscópio para próxima leitura.  
      gyroZero[axis]=0;                                // configuração da calibração de cada eixo zerada
      if (calibratingG == 1) {                         // Calculo da média
        gyroZero[axis]=g[axis]/400;                    // média das 400 leituras de cada eixo
        blinkLED(10,15,1);                             // pisca os leds
      }
    }
    calibratingG--;           //reduz de 400 até 0
  }

  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];                 //diferença com base no valor de calibração de cada eixo   // (nesta etapa ja envia valor negativo ou positivo)
    
    //anti gyro glitch => limita a variação entre duas leituras consecutivas
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800); //limta a leitura atual numa faixa de +800 e -800 em relação à leitura antiga. (+/- 195.3125°/s)        800*(4000/(2^14))  
    previousGyroADC[axis] = gyroADC[axis];                                                        //atualiza leitura antiga com a atual para próxima iteração. 
  }
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ****************
// ACC common part
// ****************

// Função de calibração do acelerômetro  
void ACC_Common() {                                                          
  static int32_t a[3];                                  // vetor para armazenar a soma de 400 leituras de aceleração de cada eixo (calibração)
  
  if (calibratingA>0) {                                 // CalibratingA diminui a cada ciclo até 0, entrando no estado normal. //O comando de calibração =>  throttle=max, yaw=left, pitch=min
    for (uint8_t axis = 0; axis < 3; axis++) {        
                                                        // calibratingA=400 no início da  calibração do acelerômetro
      if (calibratingA == 400) a[axis]=0;               // reseta a[axis] no início da calibração 
      // Soma de 400 leituras
      a[axis] +=accADC[axis];                           // a[axis] recebe a saída já tratada do acelerômetro
      // Limpa variáveis globais para próxima leitura
      accADC[axis]=0;                                   // garante a reinicilização dos valores de saída do acelerômetro para próxima leitura.         
      conf.accZero[axis]=0;                             // configuração da calibração de cada eixo zerada
    }
   
    if (calibratingA == 1) {                      // Calculo da média
      conf.accZero[ROLL]  = a[ROLL]/400;          // média das 400 leituras de x
      conf.accZero[PITCH] = a[PITCH]/400;         // média das 400 leituras de y
      conf.accZero[YAW]   = a[YAW]/400-acc_1G;    // média das 400 leituras de z - força gravitacional (PARA NAO PERDER O VALOR DA FORÇA GRAVITACIONAL)
      conf.angleTrim[ROLL]   = 0;                 // trim do acelerometro (ROLL) zerado    //este é configurado pelo rádio (ver comandos de sticks)
      conf.angleTrim[PITCH]  = 0;                 // trim do acelerometro (PITCH) zerado   //este é configurado pelo rádio (ver comandos de sticks)
      writeParams(1);                             // escreve accZero na EEPROM
    }
    calibratingA--;                  //reduz de 400 até 0
  }
  accADC[ROLL]  -=  conf.accZero[ROLL] ;    //diferença com base no valor de calibração do eixo X  (nesta etapa ja envia valor negativo ou positivo)
  accADC[PITCH] -=  conf.accZero[PITCH];    //diferença com base no valor de calibração do eixo Y
  accADC[YAW]   -=  conf.accZero[YAW] ;     //diferença com base no valor de calibração do eixo Z
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

#if defined(BMP085)
#define BMP085_ADDRESS 0x77                   //endereço do barômetro = 0x77
static int32_t  pressure;

static struct {                               // struct para referenciar os registadores do sensor BMP085 
                                              // na struct o endereço das variáveis é em sequência, uma abordagem ideal para preenche-las atraves da função "i2c_read_reg_to_buf"
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {uint16_t val; uint8_t raw[2]; } ut;  //uncompensated T     //https://pt.stackoverflow.com/questions/46668/oque-s%C3%A3o-unions-porque-utiliz%C3%A1-los-dentro-de-structs
  union {uint32_t val; uint8_t raw[4]; } up;  //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx;  
#define OSS 2 //we can get more unique samples and get better precision using average

//--------------------------------------------------------(MEMÓRIA EEPROM BMP085)-------------------------------------------------------------------------------- ver pagina 12
//
//  
// | |-----16-----|------16-----|------16-----|------16-----|------16-----|------16-----|------16-----|------16-----|------16-----|------16-----|------16-----| | => (8 bits, 8bits) => (MSB, LSB)
// | 0xAA-----0xAB-0xAC-----0xAD-0xAE-----0xAF-0xB0-----0xB1-0xB2-----0xB3-0xB4-----0xB5-0xB6-----0xB7-0xB8-----0xB9-0xBA-----0xBB-0xBC-----0xBD-0xBE-----0xBF  | =>  Endereços da EEPROM do BMP085, ver página 13 datasheet
// |      AC1           AC2           AC3            AC4          AC5            AC6           B1            B2            MB            MC            MD       | =>  variáveis da struct bmp085_ctx   
// |                                                                                                                                                            | 
// |---------------------------------------------leitura dos registers da EEPROM--------------------------------------------------------------------------------|
//
//
//  |--------16--------|--------32---------|------8-----|-------32-----|      
//  add0-----------add1-add2-----------add5-add6-------add7----------add8
//      val/raw[2]           val/raw[4]        state         deadline
//         UT                    UP          (estado do 
//     0xF6(MSB)              0xF6(MSB)      processo)
//     0xF7(LSB)              0xF7(LSB)
//                            0xF8(XLSB)
//--------------------------------------------------------------------------------------------------------------------------------------------------------------


// leitura dos dados de calibração do BMP085 ---------------------------------------------- ver pagina 13
void i2c_BMP085_readCalibration(){                 
  delay(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1); // s_bytes = (&md - &ac1 + 2) = (0xBE - 0xAA + 2) = 22 bytes (DEBUGADO) // Calcula número de bytes que serão lidos para a struct        
  i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, &bmp085_ctx.ac1, s_bytes);                            // lê os dados de calibração na memória EEPROM do BMP085 (0xAA até 0xB9) => [AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD]
                                                                                                  // os dados lidos são armazenados na struct bmp085_ctx nas variávies (ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md)
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {                                           
    swap_endianness(p, sizeof(*p));                                                               // inverte os bytes de (ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md)   // (MSB, LSB) => (LSB, MSB)
  }
}
//----------------------------------------------------------------------------------------

// Inicialização do Barômetro (temperatura) ----------------------------------------------
void  Baro_init() {
  delay(10);
  i2c_BMP085_readCalibration(); // lê os dados de calibração 
  i2c_BMP085_UT_Start();        // envia comando (0x2E) para temperatura
  delay(5);
  i2c_BMP085_UT_Read();         // lê os valores de temperatura. 
}
//----------------------------------------------------------------------------------------

// read uncompensated temperature value: send command first------------------------------- ver paginas 17 e 18
void i2c_BMP085_UT_Start() {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e);        // [COMANDO] = Gera condição de start => Escreve endereço do slave em modo de escrta => Envia endereço do registrador de controle (0xF4) = > Escreve (0x2E) no registrador de controle (0xF4)              
  i2c_rep_start(BMP085_ADDRESS<<1);              // Repeat start, enviando endereço do barômetro em modo de escrita. 
  i2c_write(0xF6);                               // Envia o registador de controle na qual lerá os dados. 
  i2c_stop();                                    // Envia condição de parada 
}
//----------------------------------------------------------------------------------------

// read uncompensated pressure value: send command first---------------------------------- ver paginas 17 e 18
void i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(OSS<<6)); // [COMANDO] = Gera condição de start => Escreve endereço do slave em modo de escrta => Envia endereço do registrador de controle (0xF4) = > Escreve (0xB4) no registrador de controle (0xF4)(OSRS = 2)
  i2c_rep_start(BMP085_ADDRESS<<1);                // Repeat start, enviando endereço do barômetro em modo de escrita. 
  i2c_write(0xF6);                                 // Envia o registador de controle na qual lerá os dados. 0xF6(MSB), 0xF7(LSB)
  i2c_stop();                                      // Envia condição de parada 
}

//--------------------------------------------------------------------------------------- ver paginas 17 e 18

// read uncompensated pressure value: read result bytes----------------------------------
void i2c_BMP085_UP_Read () {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1); // Repeat start, enviando endereço do barômetro em modo de leitura.
  bmp085_ctx.up.raw[2] = i2c_readAck();   // RECEBE 0xF6 (MSB)
  bmp085_ctx.up.raw[1] = i2c_readAck();   // RECEBE 0xF7 (LSB)
  bmp085_ctx.up.raw[0] = i2c_readNak();   // RECEBE 0xF8 (XLSB)
}
//--------------------------------------------------------------------------------------- ver paginas 17 e 18

// read uncompensated temperature value: read result bytes-------------------------------

void i2c_BMP085_UT_Read() {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1); // Repeat start, enviando endereço do barômetro em modo de leitura.
  bmp085_ctx.ut.raw[1] = i2c_readAck();   // RECEBE 0xF6 (MSB)  
  bmp085_ctx.ut.raw[0] = i2c_readNak();   // RECEBE 0xF7 (LSB)
}

//---------------------------------------------------------------------------------------

//CALCULOS------------------------------------------------------------------------------- ver pagina 13

void i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  
  // Temperature calculations 
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;   //rever divisoes de  giro e baro 
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  // temperatura = (b5 + 8) >> 4
  
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

//-------------------------------------------------------------------------------------

// Atualização dos dados do barômetro. ------------------------------------------------
void Baro_update() {                               
  if (currentTime < bmp085_ctx.deadline) return;   // Se o tempo decorrido, desde que o microcontrolador se iniciou, for menor que a variável deadline, sai da função. (garantindo 23.6 milissegundos para todo processo)
  bmp085_ctx.deadline = currentTime;               // deadline recebe o tempo de operação em microssegundos, desde que o microcontrolador foi iniciado. 
  TWBR = ((F_CPU / 400000L) - 16) / 2;             // Aumenta a frequência da TWI (400KHz).
  switch (bmp085_ctx.state) {
    case 0: 
      i2c_BMP085_UT_Start();                            // envia comando para temperatura                         
      bmp085_ctx.state++; bmp085_ctx.deadline += 4600;  // incrementa estado de 0 para 1 
      break;                                            // incrementa deadline em 4600 microssegundos. 
    case 1: 
      i2c_BMP085_UT_Read();                             // lê os valores descompensados de temperatura
      bmp085_ctx.state++;                               // incremeta estado de 1 para 2
      break;
    case 2: 
      i2c_BMP085_UP_Start();                             // envia comando para pressão (OSCS 2)
      bmp085_ctx.state++; bmp085_ctx.deadline += 14000;  // incrementa estado de 2 para 3
      break;                                             // incrementa deadline em 14000 microssegundos. 
    case 3: 
      i2c_BMP085_UP_Read();                                                // lê os valores de descompensados de pressão
      i2c_BMP085_Calculate();                                              // calcula presão real
      BaroAlt = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 4433000.0f;  // formula pagina 14      (altitude em centímetros)   
      bmp085_ctx.state = 0; bmp085_ctx.deadline += 5000;                   // retorna estado para 0
      break;                                                               // incrementa deadline em 5000 microssegundos. 
  } 
}
//--------------------------------------------------------------------------------------
#endif

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//https://learn.sparkfun.com/tutorials/accelerometer-basics/all

// ************************************************************************************************************
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit 
//
// Control registers:
//
// 0x20    bw_tcs:      |                                           bw<3:0> |                        tcs<3:0> |
//                      |                                             150Hz |                        xxxxxxxx |
// 0x30    tco_z:       |                                                tco_z<5:0>    |     mode_config<1:0> |
//                      |                                                xxxxxxxxxx    |                   00 |
// 0x35    offset_lsb1: |          offset_x<3:0>              |                   range<2:0>       | smp_skip |
//                      |          xxxxxxxxxxxxx              |                    8G:   101       | xxxxxxxx |
// ************************************************************************************************************

#if defined(BMA180)
void ACC_init () {                                // inicialização do acelerômetro
  delay(10);                                      // não é possível escrever na EEPROM durante 10ms após a energização. 
  i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4);         //  ee_w = 1    // Habilita escrita nos registradores de imagem (20h...3Bh)  
  delay(5);                                       // tempo para escrita na EEPROM
  uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
  control = control & 0x0F;                       // salva o tcs  do registrador
  control = control | (0x01 << 4);                // set low pass filter to 20Hz
  i2c_writeReg(BMA180_ADDRESS, 0x20, control);    // bw = 0001  //filtro Butterworth de segunda ordem com frequência de polo de 20Hz
  delay(5);
  control = i2c_readReg(BMA180_ADDRESS, 0x30);
  control = control & 0xFC;                       // salva o tco_z do registrador 
  control = control | 0x00;                       // set mode_config to 0
  i2c_writeReg(BMA180_ADDRESS, 0x30, control);    // mode_config = 00   //Low noise mode  (página 29 do BMA180 Datasheet)
  delay(5); 
  control = i2c_readReg(BMA180_ADDRESS, 0x35);
  control = control & 0xF1;                      // salv o offset_x e smp_skip do registrador
  control = control | (0x05 << 1);               // set range to 8G
  i2c_writeReg(BMA180_ADDRESS, 0x35, control);   // range = 101      // 8G é menos sensivel e preciso do que 1G (página 52 do BMA180 Datasheet)
  delay(5); 
  acc_1G = 255;                                  //ver na tabela de resolução abaixo
}

void ACC_getADC () {                     //aquisição dos dados
  TWBR = ((F_CPU / 400000L) - 16) / 2;   // Aumenta a frequência da TWI (400KHz).
  
  i2c_getSixRawADC(BMA180_ADDRESS,0x02); //recebe os dados do sensor no vetor rawADC 
  // 14 bits  [2-15] bits  /4  =>  [0-13] bits  /4 => 12 bits de resolução:
  // ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/16 ,         //accADC[ROLL]  = [rawADC[1]; rawADC[0]]/16  => [0000 0000 0000]   retira-se os 4 bits menos significativos com a divisão. 
                   ((rawADC[3]<<8) | rawADC[2])/16 ,         //accADC[PITCH] = [rawADC[3]; rawADC[2]]/16  => [0000 0000 0000]
                   ((rawADC[5]<<8) | rawADC[4])/16 );        //accADC[YAW]   = [rawADC[5]; rawADC[4]]/16  => [0000 0000 0000]  
                              
  ACC_Common(); // Aqui se faz a calibração.
}               // ACC_Common()também faz a diferença entre o valor de aceleração atual e o valor de aceleração calibrado. 
#endif

//-------------(TABELA DE RESOLUÇÃO)------------------------------------------------------------------------------------------                         
// -8g            => 1000 0000 0000   (2048)                       Resolução de 12 bits =>   2^12 = 4096   =>  1    => 2047  |
// -7.996         => 1000 0000 0001   (2049)                                                                   2048 => 4095  |
// .                                                                                                                         |
// .                                                                                                                         |
// .                                                                                                                         |
// -0.003906248   => 1111 1111 1111   (4095)                                                                                 |
// 0.0g           => 0000 0000 0000                                                                                          | 
// 0.003906248    => 0000 0000 0001   (1)                                                                                    |
// .                                                                                                                         |
// .                                                                                                                         |
// .                                                                                                                         |
// 7.992g         => 0111 1111 1110                                                                                          |
// 7.996g         => 0111 1111 1111   (2047)                         1g =~ 255                                               |
//----------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ************************************************************************************************************
// I2C Gyroscope ITG3200 
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************

#if defined(ITG3200) 
void Gyro_init() {         //inicialização do giroscópio
  delay(100);
  i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80);                     // 0x3E = PWR_MGM register => H_RESET = 1         // reinicializa o dispositivo e registradores internos para as configurações padrão.       
//  delay(5);
//  i2c_writeReg(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV);     // 0x15 = SMPLRT_DIV register => SIMPLRT_DIV = 0  // divisor da taxa de amostragem = 1 
  i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG);  // 0x16 = DLPF_FS register => FS_SEL = 0b11       // Máximo alcance da escala do sensor de gyro = +/-2000 graus/s
  delay(5);                                                      //                         => DLPF_CFG = 0b00     // Configuração da largura de banda do filtro passa baixa digital = 256Hz. // Taxa de amostragem interna = 8KHz
  i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03);                     // 0x3E = PWR_MGM register => CLK_SEL = 0b11      // Fonte de clock do dispositivo = PLL com referência no giroscópio Z.
  delay(100);
}

void Gyro_getADC () {                      // aquisição dos dados
  TWBR = ((F_CPU / 400000L) - 16) / 2;     // Aumenta a frequência da TWI (400KHz)
  i2c_getSixRawADC(ITG3200_ADDRESS,0X1D);  // recebe os dados do sensor no vetor rawADC 
  // GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/4 ,   //gyroADC[ROLL]  = [rawADC[1] rawADC[0]]/4  => [00 0000 0000 0000]              //retira-se os 2 bits menos significativos com a divisão. 
                    ((rawADC[2]<<8) | rawADC[3])/4 ,   //gyroADC[PITCH] = [rawADC[3] rawADC[2]]/4  => [00 0000 0000 0000]
                    ((rawADC[4]<<8) | rawADC[5])/4 );  //gyroADC[YAW]   = [rawADC[5] rawADC[4]]/4  => [00 0000 0000 0000] 
  GYRO_Common(); // Aqui se faz a calibração.
}                // GYRO_Common()também faz a diferença entre o valor da taxa de variação angular atual e o valor da taxa de variação angular calibrado.
#endif
//-------------(TABELA DE RESOLUÇÃO)---------------------------------------------------------------------------------------------------                         
//                                                                       Resolução de 14 bits =>   2^14 = 16384  =>  1    => 8191    
//                                                                                                                  8192  => 16383   
//
// SENSIBILIDADE DE 14.375
// 
// VALOR EM (°/s) = VALOR(ADC)/14.375  ???? ou igual ao acelerometro
//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
static float   magCal[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

void Mag_getADC() {
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
  magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
  magADC[YAW]   = magADC[YAW]   * magCal[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      conf.magZero[axis] = 0;
      magZeroTempMin[axis] = magADC[axis];
      magZeroTempMax[axis] = magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    magADC[ROLL]  -= conf.magZero[ROLL];
    magADC[PITCH] -= conf.magZero[PITCH];
    magADC[YAW]   -= conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
        if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
      writeParams(1);
    }
  }
}
#endif

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x1E (7bit)
// ************************************************************************************************************
#if defined(HMC5883)
  #define MAG_ADDRESS 0x1E
  #define MAG_DATA_REGISTER 0x03
  
  void Mag_init() { 
    delay(100);
    // force positiveBias
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
    delay(50);
    // set gains for calibration
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode

    // read values from the compass -  self test operation
    // by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
    // The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
    delay(100);
      getADC();
    delay(10);
    
    magCal[ROLL]  =  766.0 / abs(magADC[ROLL]);             //   766/ abs(magADC[ROLL])                               1160
    magCal[PITCH] =  766.0 / abs(magADC[PITCH]);            //   766/ abs(magADC[PITCH])                              1160
    magCal[YAW]   =  750.0 / abs(magADC[YAW]);              //   750/ abs(magADC[YAW])        ????                    1080
    

    // leave test mode
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

    magInit = 1; //INICIALIZAÇÃO FEITA
  }

void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER); 
  MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );
  
}

void Device_Mag_getADC() {
  getADC();
}

#endif


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void initSensors() {
  delay(200);
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) Gyro_init();
  if (BARO) Baro_init();
  if (MAG) Mag_init();
  if (ACC) {ACC_init();acc_25deg = acc_1G * 0.423;}
  f.I2C_INIT_DONE = 1;
}

