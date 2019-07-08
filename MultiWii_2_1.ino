// MULTIWII - LaSisC VERSION 
// VERSÃO SEM GPS

//INCLUSÃO DE BIBLIOTECAS-------------------------------------------------------------------------------------------------------------------------------------------
#include <avr/io.h>                
#include "config.h"
#include "def.h"
#include <avr/pgmspace.h>


//DEFINIÇÕES DE MACROS GERAIS--------------------------------------------------------------------------------------------------------------------------------------
#define  VERSION  210  //MULTIWII VERSÃO 2.1

/*********** macros do rc *****************/

#define ROLL       0   //rolagem
#define PITCH      1   //arfagem
#define YAW        2   //guinada
#define THROTTLE   3   //potência
#define AUX1       4   //auxiliar 1
#define AUX2       5   //auxiliar 2
#define AUX3       6   //auxiliar 3
#define AUX4       7   //auxiliar 4

/*********** macros do PID ****************/

                       // referência para os ganhos de cada controle [PIDITEMS] (presente na EEPROM)
#define PIDALT     3   // ganhos do controle de ALTITUDE
#define PIDPOS     4   // ganhos do controle do GPS                           (DESABILITADO)
#define PIDPOSR    5   // ganhos do controle do GSP no modo POSITION HOLD     (DESABILITADO)
#define PIDNAVR    6   // ganhos do controle do GSP no modo NAVEGATION        (DESABILITADO)
#define PIDLEVEL   7   // ganhos do controle no modo LEVEL
#define PIDMAG     8   // ganhos do controle no modo MAG
#define PIDVEL     9   // ganhos do controle no modo VELOCIDADE               (DESABILITADO)
                       
                       // [0,1,2] sõs os ganhos de controle no modo ACRO para cada eixo
                       
/************ BOXES DA GUI ***************/

// através dos switcher auxiliares do rádio controle, pode-se ativar e desativas os modos de voo descritos abaixo. 
// https://oscarliang.com/multiwii-different-flight-modes-names-gui/
// http://www.multiwii.com/wiki/index.php?title=Flightmodes         

#define BOXACC       0   // Ativa LEVEL MODE - necessita dos dados do giroscópio e acelerômetro

#define BOXBARO      1   // Ativa ALTITUDE HOLD MODE - necessita dos dados do giroscópio, acelerômetro e barômetro
                         // A altitude do quadrirrotor é mantida, a não ser que haja outro comando do piloto. 

#define BOXMAG       2   // Ativa MAG MODE - necessita dos dados do giroscópio, acelerômetro e magnetômetro
                         // O quadrirrotor aponta sempre para a mesma direção, a não ser que haja comando de yaw

#define BOXCAMSTAB   3   // Establilização da câmera por servo             (DESABILITADO)
#define BOXCAMTRIG   4   // trigger da câmera                              (DESABILITADO)

#define BOXARM       5   // Armar quadrirrotores - necessita dos dados do giroscópio.
                         // Se apenas essa box estiver ativa = ACRO MODE

#define BOXGPSHOME   6   // Ativa BACK TO HOME   - GPS                     (DESABILITADO)
#define BOXGPSHOLD   7   // Ativa HOLD POSITIION - GPS                     (DESABILITADO)
#define BOXPASSTHRU  8   // Passagem direta do PWM - usado em aeromodelos  (DESABILITADO)

#define BOXHEADFREE  9   // Ativa HEAD FREE MODE - ativa apenas com MAG MODE ativado
                         // Fixa uma perspectiva de voo, fixa a frente do quadrirrotor, mantendo os controles em relação a esta.

#define BOXBEEPERON  10  // indicador de bateria por buzzer                (DESABILITADO)
#define BOXLEDMAX    11  // Ativa máxima iluminação                        (DESABILITADO)
#define BOXLLIGHTS   12  // Ativa luz de aterrissagem                      (DESABILITADO)

#define BOXHEADADJ   13  // Permite selecionar uma nova perspectiva de voo, para o HEAD FREE MODE


#define PIDITEMS 10       // numero de PID's diferentes (PRESENTES DA EEPROM)
#define CHECKBOXITEMS 14  // numero de BOXES da GUI


//PASSAGEM DE CHARS PARA SOFTWARE DE CONTROLE DE SOLO---------------------------------------------------------------------------------------------------------------
const char boxnames[] PROGMEM =     // Nomes das BOXES de configurações de modos na GUI
  "ACC;"
  "BARO;"
  "MAG;"
  "CAMSTAB;"
  "CAMTRIG;"
  "ARM;"
  "GPS HOME;"
  "GPS HOLD;"
  "PASSTHRU;"
  "HEADFREE;"
  "BEEPER;"
  "LEDMAX;"
  "LLIGHTS;"
  "HEADADJ;"
;

const char pidnames[] PROGMEM =   // Nomes das congfigurações de controle(PID) na GUI
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;


//DECLARAÇÃO DE VARIÁVEIS GLOBAIS GERAIS---------------------------------------------------------------------------------------------------------------------------
static uint32_t currentTime = 0;      // 
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;        // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;     // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint16_t acc_1G;               // this is the 1G measured acceleration
static int16_t  acc_25deg;            // 0.25* 1G 
static int16_t  headFreeModeHold;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t  heading,magHold;
           
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  BaroAlt;
static int32_t  EstAlt;             // in cm
static int16_t  BaroPID = 0;
static int32_t  AltHold;
static int16_t  errorAltitudeI = 0;
static int16_t  debug[4];


//FLAGS DE STATUS-------------------------------------------------------------------------------------------------------------------------------------------------
struct flags_struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to know when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ACC_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
} f;


//CONTADORES GLOBAIS---------------------------------------------------------------------------------------------------------------------------------------------
static int16_t  i2c_errors_count = 0;               // contador de erros da comunicação i2c
static int16_t  annex650_overrun_count = 0;         // tempo de 650ms na função annex excedido 


// *****************************************************************************************************************
//                                                    rc functions
// *****************************************************************************************************************
#define MINCHECK 1100
#define MAXCHECK 1900

static int16_t rcData[8];             // interval [1000;2000]
static int16_t rcCommand[4];          // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t lookupPitchRollRC[6];  // lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];  // lookup table for expo & mid THROTTLE
volatile uint8_t rcFrameComplete;     // for serial rc receiver Spektrum


// ***************************************************************************************************************
//                                                    gyro+acc IMU
// ***************************************************************************************************************
static int16_t gyroData[3] = {0,0,0};   // gyroData = {(LEITURA-ATUAL 1 + LEITURA-ATUAL 2) + [ (LEITURA-ANTERIOR 1 + LEITURA-ANTERIOR 2)/2 ] } / 3
static int16_t gyroZero[3] = {0,0,0};   // valor calibrado do magnetômetro (calculado em GYRO_Common( ))
static int16_t angle[2]    = {0,0};     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800


// **************************************************************************************************************
//                                              motor and servo functions
// **************************************************************************************************************
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];


// **************************************************************************************************************
//                                              EEPROM Layout definition
// **************************************************************************************************************
static uint8_t dynP8[3], dynD8[3];
static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t accZero[3];                    // valor calibrado do acelerômetro (calculado em ACC_Common( )
  int16_t magZero[3];                    // valor calibrado do magnetômetro (caluculado em MAG_getADC( ) )
  int16_t angleTrim[2];                  // configuração do trim do acelerometro (acesso por meio dos sticks)
  uint16_t activate[CHECKBOXITEMS];      // verifica se a BOX da GUI foi ativada (feita através da comunicação serial do placa com o software)
  uint8_t powerTrigger1;
} conf;


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX NÃO UTILIZADO (GPS)XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// **********************
// GPS common variables
// **********************
  static int32_t  GPS_coord[2];
  static int32_t  GPS_home[2];
  static int32_t  GPS_hold[2];
  static uint8_t  GPS_numSat;
  static uint16_t GPS_distanceToHome;                          // distance to home in meters
  static int16_t  GPS_directionToHome;                         // direction to home in degrees
  static uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
  static uint8_t  GPS_update = 0;                              // it's a binary toogle to distinct a GPS position update
  static int16_t  GPS_angle[2] = { 0, 0};                      // it's the angles that must be applied for GPS correction
  static uint16_t GPS_ground_course = 0;                       // degrees*10
  static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
  static uint8_t  GPS_Enable  = 0;

  #define LAT  0
  #define LON  1
  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  static int16_t  nav[2];
  static int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  // default POSHOLD control gains
  #define POSHOLD_P              .11
  #define POSHOLD_I              0.0
  #define POSHOLD_IMAX           20        // degrees

  #define POSHOLD_RATE_P         2.0
  #define POSHOLD_RATE_I         0.08      // Wind control
  #define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
  #define POSHOLD_RATE_IMAX      20        // degrees

  // default Navigation PID gains
  #define NAV_P                  1.4
  #define NAV_I                  0.20      // Wind control
  #define NAV_D                  0.08      //
  #define NAV_IMAX               20        // degrees

  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2
  static uint8_t nav_mode = NAV_MODE_NONE;            //Navigation mode
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


//FUNÇÃO DE PISCAR OS LEDS--------------------------------------------------------------------------------------------------------------------------------------
void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(wait);
    }
    delay(60);
  }
}


//FUNÇÃO ANNEX-------------------------------------------------------------------------------------------------------------------------------------------------
void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE]<BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp/100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp/100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
  
 
  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 500000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  serialCom();

}

//SETUP PRINCIPAL----------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  #if !defined(GPS_PROMINI)
  SerialOpen(0,SERIAL_COM_SPEED);
  #endif 
  LEDPIN_PINMODE;
  STABLEPIN_PINMODE;
  initOutput();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  initSensors();
  previousTime = micros();
  calibratingG = 400;
 
 
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  f.SMALL_ANGLES_25=1; // important for gyro only conf
  
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;


  #define RC_FREQ 50

  if (currentTime > rcTime ) {      // garante leitura a cada 50Hz  
    rcTime = currentTime + 20000;   // 1/50 = 20.0000 us  
    computeRC();
    
    if (rcData[THROTTLE] < MINCHECK) {                                                                                         // THROTTLE < 1100  ------------------------------------------------------------------------------------------------ |
    
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;                                                                             // zera integradores de erro do giroscópio da malha de controle  (ACRO MODE)                    |
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;                                                                                                // zera integradores de erro do ângulo da malha de controle      (LEVEL MODE)                   |
      rcDelayCommand++;                                                                                                                             // + 20ms                                                                                       |
      
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && !f.ARMED) {                                                          // THROTTLE=MIN ; YAW=ESQUERDA ; PITCH=MIN ; NÃO ARMADO (CALIBRAÇÃO DO GIROSCÓPIO) -------|                    |
        if (rcDelayCommand == 20) {                                                                                                        // deve permanecer nessa posição por 0.4 segundos                                   |                    |
          calibratingG=400;                                                                                                                // inicia as 400 medidas para calibração do giroscópio                              |                    |
                                                                                                                                           //                                                                                  |                    |
                                                                                                                                     // ---------------------------------------------------------------------------------------|                    |
        }
      } 
      else if (conf.activate[BOXARM] > 0) {                                                                                          // SE FOI SELECIONADA CONFIGURAÇÃO PARA ARMAR QUADRIRROTOR PELOS AUXILIARES --------------|                    |
        if ( rcOptions[BOXARM] && f.OK_TO_ARM) {                                                                                           // (POSIÇÃO DO AUX = ARMAR QUADRIRROTOR) & (TUDO OK PARA ARMAR)---------|           |                    |
	  f.ARMED = 1;                                                                                                                             // Flag indicando que quadrirrotor está armado                  |           |                    |          
                                                                                                                                           // ---------------------------------------------------------------------|           |                    |
        } else if (f.ARMED) f.ARMED = 0;                                                                                                   // (SE QUADRIRROTOR ESTÁ ARMADO) ---------------------------------------|           |                    |                                         
        rcDelayCommand = 0;                                                                                                                        // Flag indicando que quadrirrotor está desarmado               |           |                    |
                                                                                                                                                   // reseta contador                                              |           |                    |
                                                                                                                                           // ---------------------------------------------------------------------|           |                    |         
                                                                                              
      #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW                                                                                            //VERIFICA SE FOI DEFINIDO O ARM E DESARM PELO CANA YAW-----------------------------------------|               |
      } else if ( (rcData[YAW] < MINCHECK )  && f.ARMED) {                                                                                    // THROTTLE=MIN ; YAW=ESQUERDA ; ARMADO --------------------|                         |               |
        if (rcDelayCommand == 20) f.ARMED = 0;                                                                                                     // deve permanecer nessa posição por 0.4 segundos      |                         |               |
                                                                                                                                                   // flag indicando que quadrirrotor está desarmado      |                         |               |
                                                                                                                                              // ---------------------------------------------------------|                         |               |   
      } else if ( (rcData[YAW] > MAXCHECK ) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {              // THROTTLE=MIN ; YAW=DIREITA; PITCH<MAX ; calibratingG = 0; ACELERÔMETRO CALIBRADO--| |               |
        if (rcDelayCommand == 20) {                                                                                                                // deve permanecer nessa posição por 0.4 segundos                              | |               |         
	  f.ARMED = 1;                                                                                                                             // Flag indicando que quadrirrotor está armado                                 | |               |                        
        }                                                                                                                                          //-----------------------------------------------------------------------------| |               |
      #endif                                                                                                                                 //-------------------------------------------------------------------------------------|               |
   
      } else                                                                                                                         //QUALQUER OUTRO COMANDO--------------|                                                                        |
        rcDelayCommand = 0;                                                                                                                  // reseta contador            |                                                                        |   
                                                                                                                                     //------------------------------------|                                                                        |
                                                                                                                                //------------------------------------------------------------------------------------------------------------------|   
    } else if (rcData[THROTTLE] > MAXCHECK && !f.ARMED) {                                                                       // THROTTLE > 1900 ; NÃO ARMADO-------------------------------------------------------------------------------------|
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {                                                                             // THROTTLE=MAX ;  YAW=ESQUERDA, PITCH=MIN (CALIBRAÇÃO ACELERÔMETRO)--------------------|               |
        if (rcDelayCommand == 20) calibratingA=400;                                                                                                 // deve permanecer nessa posição por 0.4 segundos                               |               |
        rcDelayCommand++;                                                                                                                           // inicia as 400 medidas para calibração do acelerômetro                        |               |
                                                                                                                                            // -------------------------------------------------------------------------------------|               |
        
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) {                                                                      // THROTTLE=MAX ;  YAW=DIREITA, PITCH=MIN (CALIBRAÇÃO MAGNETÔMETRO)---------------------|               |
        if (rcDelayCommand == 20) f.CALIBRATE_MAG = 1;                                                                                              // deve permanecer nessa posição por 0.4 segundos                               |               |
        rcDelayCommand++;                                                                                                                           // Entra na calibração do magnetômetro                                          |               |
                                                                                                                                            //--------------------------------------------------------------------------------------|               |
                                 
      } else if (rcData[PITCH] > MAXCHECK) {                                                                                                //conf.angleTrim[PITCH] + 2                                                                             |                                                          
         conf.angleTrim[PITCH]+=2;writeParams(1);
      } else if (rcData[PITCH] < MINCHECK) {                                                                                                //conf.angleTrim[PITCH] - 2                                                                             |
         conf.angleTrim[PITCH]-=2;writeParams(1);
      } else if (rcData[ROLL] > MAXCHECK) {                                                                                                 //conf.angleTrim[ROLL] + 2                                                                              |
         conf.angleTrim[ROLL]+=2;writeParams(1);
      } else if (rcData[ROLL] < MINCHECK) {                                                                                                 //conf.angleTrim[ROLL] - 2                                                                              |
         conf.angleTrim[ROLL]-=2;writeParams(1);
      } else {                                                                                                                              //QUALQUER OUTRO COMANDO--------------|                                                                 |
        rcDelayCommand = 0;                                                                                                                        // reseta contador             |                                                                 |                                                                                    
      }                                                                                                                                      //-----------------------------------|                                                                 |
    }
                                                                                                                                // -----------------------------------------------------------------------------------------------------------------|
    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2); 
    for(i=0;i<CHECKBOXITEMS;i++)
      rcOptions[i] = (auxState & conf.activate[i])>0;        //conf.activate vem da GUI, variando seu valor de acordo com a configuração feito nela
      
      //auxState = 0000 0000 0000 0000
      //                 _                      _                     _
      //                |_|                    |_|                   |_| 
      // AUX0   = 0000 0000 0000 0001 | 0000 0000 0000 0010 | 0000 0000 0000 0100
      // AUX1   = 0000 0000 0000 1000 | 0000 0000 0001 0000 | 0000 0000 0010 0000
      // AUX2   = 0000 0000 0100 0000 | 0000 0000 1000 0000 | 0000 0001 0000 0000
      // AUX3   = 0000 0010 0000 0000 | 0000 0100 0000 0000 | 0000 1000 0000 0000
      //
      // Se na GUI o LVL mode foi configurado para ativar com o AUX2 em valor alto  =>  conf.activate[BOXACC] =  0000 0001 0000 0000
      //
      // rcOptions[BOXACC] = auxState & conf.activate[BOXACC] 
      // 
       
    if ( rcOptions[BOXACC] && ACC ) {                              // Se selecionado LEVEL MODE
      if (!f.ACC_MODE) {                                                  //se flag está abaixada
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;                             // zera interagradores da malha de controle (LEVEL MODE)
        f.ACC_MODE = 1;                                                            // seta a flag
      }  
    } else {                                                       // Se não está selecionado LEVEL MODE
      f.ACC_MODE = 0;                                                     // abaixa a flag
    }

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;                  // Se não selecionado ARMAR, seta a flag deixando prontO
    
    if (f.ACC_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}         // Se LEVEL MODE ativado = apaga LED azul       Se LEVEL MODE desligado = acende LED azul    

    #if BARO
      if (rcOptions[BOXBARO]) {                                   // Se selecionado ALTITUDE HOLD MODE
        if (!f.BARO_MODE) {                                             //se flag está abaixada
          f.BARO_MODE = 1;                                                       // seta a flag
          AltHold = EstAlt;                                                      // AltHold = altitude atual
          initialThrottleHold = rcCommand[THROTTLE];                             // initialThrottleHold = potência atual
          errorAltitudeI = 0;                                                    // zera integrador da malha de controle do altitude hold 
          BaroPID=0;                                                             // reinicia sinal de controle 
        }
      } else {                                                   // Se não está selecionado ALTITUDE HOLD MODE
        f.BARO_MODE = 0;                                                // abaixa a flag
      }
    #endif
    
    #if MAG                                                   
      if (rcOptions[BOXMAG]) {                                  // Se selecionado MAG MODE           
        if (!f.MAG_MODE) {                                             //se flag está abaixada
          f.MAG_MODE = 1;                                                     // seta a flag
          magHold = heading;                                                  // maghold = orientação atual
        }
      } else {                                                  // Se não está selecionado MAG MODE
        f.MAG_MODE = 0;                                                 // abaixa a flag
      } 
    #endif    
   
  } else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes  //vai ate 256 e retorna
    switch (taskOrder++ % 5) {  // 0 1 2 3 4 0 1 2 3 4 0 1 2 3 4....
      case 0:
        #if MAG
          Mag_getADC();
        #endif
        break;
      case 1:
        #if BARO
          Baro_update();
        #endif
        break;
      case 2:
        #if BARO
          getEstimatedAltitude();
        #endif
        break;
        
      case 3:
      
        break;
        
      case 4:
        
        break;
    }
  }
  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  #if MAG
    if (abs(rcCommand[YAW]) <70 && f.MAG_MODE) {
      int16_t dif = heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif*conf.P8[PIDMAG]/30;  // 18 deg
    } else magHold = heading;
  #endif

  #if BARO
    if (f.BARO_MODE) {
      if (abs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        f.BARO_MODE = 0; // so that a new althold reference is defined
      }
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
  #endif
  

  //**** PITCH & ROLL & YAW PID ****    
  for(axis=0;axis<3;axis++) {
    if (f.ACC_MODE && axis<2 ) { //LEVEL MODE
      // 50 degrees max inclination
      errorAngle = constrain(2*rcCommand[axis] /*+GPS_angle[axis]*/,-300,+300) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      PTerm      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      PTerm = constrain(PTerm,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

      errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
      ITerm              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    } else { //ACRO MODE or YAW axis
      if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                               else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
      error -= gyroData[axis];

      PTerm = rcCommand[axis];
      
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
      ITerm = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if (abs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
                            else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation   

    delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    if (abs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
                      else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation
                      
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  mixTable();
  writeMotors();
}

