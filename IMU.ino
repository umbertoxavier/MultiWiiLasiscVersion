// IMU - LaSisC VERSION 
// VERSÃO SEM GPS

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void computeIMU () {
  
  uint8_t axis;                                        // variável para referenciar os eixos            
  static int16_t gyroADCprevious[3] = {0,0,0};         // armazena ((LEITURA 1 + LEITURA 2)/2) da iteração anterior    //inciando-se com 0 para primeira iteração
  int16_t gyroADCp[3];                                 // armazena LEITURA 1
  int16_t gyroADCinter[3];                             // armazena (LEITURA 1 + LEITURA 2)
  static uint32_t timeInterleave = 0;                  // tempo entre aquisição de dois valores consecutivos do conversor analógico/digital do giroscópio 

  //O TEMPO PARA LER DOIS VALORES CONSECUTIVOS DO GIROSCÓPIO É DE 650 MICROSSEGUNDOS:     
 
  #if ACC
    ACC_getADC();                           // Pega os valores do conversor analogíco/digital do acelerômetro
    getEstimatedAttitude();                 // Função para calcular angulos estimados atraves do filtro complementar linear
  #endif
  #if GYRO                      
    Gyro_getADC();                          // LEITURA 1 - Pega os valores do conversor analógioco/digital do girscópio
  #endif
  
  for (axis = 0; axis < 3; axis++)
    gyroADCp[axis] =  gyroADC[axis];        // gyroADCp guarda valor de gyroADC  (essa linha guarda para os 3 eixos)
    
  timeInterleave=micros();                  // recebe tempo desde inicialização em microssegundos
  annexCode();                              // annexCode(); rcData => (atenuacoes) => rcCommand   /alem de fazer modificações nos ganhos atuadores do controle PID 
                                            // annexCode() é executado em cada loop, e nao interfere no loop do controle se demorar menos que 650 microssegundos                                       
  if ((micros()-timeInterleave)>650) {      // |Se annexCode() demorar mais que 650 microssegundos =>   |
     annex650_overrun_count++;              // |incrementa contador de superação de tempo               |
  } else {                                  //                                                          |Se annexCode() completar em menos de 650 microssegundos =>                                                              |
     while((micros()-timeInterleave)<650) ; //                                                          |(ESPERA ATE COMPLETAR 650 microssegundos) Empírico , atraso de intercalação entre 2 leituras consecutivas do giroscópio |
  }
  #if GYRO
    Gyro_getADC();                          // LEITURA 2 - Pega novos valores do conversor analógioco/digital do giroscópio
  #endif       
  
  for (axis = 0; axis < 3; axis++) {
    gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];                               // gyroADCinter = (LEITURA 1 + LEITURA 2)          
    // Empírico, tomamos um valor ponderado dos valores atual e anterior =>
    gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;                    // gyroDara = (gyroADCinter + gyroADCprevious) / 3     {(LEITURA-ATUAL 1 + LEITURA-ATUAL 2) + [ (LEITURA-ANTERIOR 1 + LEITURA-ANTERIOR 2)/2 ] } / 3 
    gyroADCprevious[axis] = gyroADCinter[axis]/2;                                     // gyroADCprevious = gyroADCinter/2                    // atualiza-se com a média das duas leituras atuais. 
    if (!ACC) accADC[axis]=0;
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//  IMU Simplificada baseada no Filtro Complementar
//  Inspirado por: http://starlino.com/imu_guide.html
//
//  The following ideas was used in this project:
//  1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//  2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
//  3) C. Hastings approximation for atan2()
//  4) Optimization tricks: http://www.hackersdelight.org/
//
//  Currently Magnetometer uses separate CF which is used only
//  for heading approximation.
//

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//****** FUNÇÕES AVANÇADAS PARA O USUÁRIO ****** 

/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
#ifndef ACC_LPF_FACTOR
   #define ACC_LPF_FACTOR 100
#endif

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
#ifndef MG_LPF_FACTOR
  //#define MG_LPF_FACTOR 4
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 400.0f
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#ifndef GYR_CMPFM_FACTOR
  #define GYR_CMPFM_FACTOR 200.0f
#endif

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f)) //divisor do filtro complementar Gyro/Acc
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f)) //divisor do filtro complementar Gyro/Magnetometer
#if GYRO
  #define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result                     2279.44 * PI                     2279.44/2^(13) = 3.59       3.59x4 = 14.375
  // +-2000/sec deg scale                                                                                                                   _________________________          
  // should be in rad/sec                                                                                                                      2^(13) * 180 * 10^(6)    
#endif//                                                                                                                                                                        
//
//                                                                                                                                                                                    
//                                                                                                                                                                                 
//                                                                                                                                                                           
//                                                                                                                                                                              
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Small angle approximation
#define ssin(val) (val)         // utilizados na matriz de roatacao
#define scos(val) 1.0f          // valores em rad

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;
//
//
//            |    X     |     y    |    Z     |    =   ESTRUTURA                                   |    X     |     y    |    Z     |    =   ESTRUTURA 
//
//            t_fp_vector EstG =>                                                                    t_fp_vector EstM =>
//
//            |                V               |    =  EstG.V                                       |                V               |    =  EstM.V  
//            | EstG.V.X | EstG.V.Y | EstG.V.Z |                                                    | EstM.V.X | EstM.V.Y | EstM.V.Z |
//            |   A[0]   |   A[1]   |   A[2]   |    =  EstG.A                                       |   A[0]   |   A[1]   |   A[2]   |    =  EstM.A                   
//
//
//
//---------------------------------------------------------------------------------------------------------------------------------------------------------
                                                                     // verificar se valor é negativo (fp_is_neg(val)) :
                                                                     // val =  EstG.V.X  
                                                                                                 //           [0]         [1]        [2]         [3]        =>  llitle endianess (cuidado)
int16_t _atan2(float y, float x){                                                                //       |0000 0000 | 0000 0000| 0000 0000 | 1000 0000 |   2's complement => (sempre terá (1) no bit mais significativo, se este representar valor negativo)                         
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)                                     //       |                  EstG.V.X                   |                      
  float z = y / x;                                                                               //                                                                                                                 
  int16_t zi = abs(int16_t(z * 100));                                                            //            [3]
  int8_t y_neg = fp_is_neg(y);                                                                   //       | 1000 0000 |   &  (1000 0000)   !=  0            => PORTANDO SERIA NEGATIVO
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);                     
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10);   //retorna em    graus*10
  return z;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {                                                   // struct fp_vector *v = declaracao do ponteiro para a struct fp_vector       
                                                                                                   // float* delta = declaração de um ponteiro para apontar ao vetor (deltaGyroAngle[axis])
                                                                                                   // não colocamos o operador de endereço em deltaGyroAngle; fazemos isso porque um vetor já representa um endereço
                                                                                                   
  fp_vector v_tmp = *v;                                                                            // v_tmp = variavel da struct fp_vector, que recebe os valores que EstG.V ou EstM.V 
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;                                         // v->Z  => gets the member Z from the struct that v points to
  v->X += delta[ROLL]  * v_tmp.Z + delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z - delta[YAW]   * v_tmp.X; 
}
//           EstG.V                                                                                      EstG.V 
//            |X|                                                                                          |X|
//   v  ->    |Y|                       delta  = deltaGyroAngle[axis]                     v_tmp  = *v =    |Y|
//            |Z|                                                                                          |Z|
//              
//  v aponta para  EstG.V           se faz isso, pois vetores são ponteiros               v_tmp guarda valores de EstG.V
//        ou EstM.V                                                                        nao alterando a fonte original
//                                                                                              em operações futuras
//                
//  
//   
//    EstG.V.X [N+1]  =   EstG.V.X [N] + [ ( delta[ROLL]  * EstG.V.Z [N] ) + ( delta[YAW]   * EstG.V.Y [N] ) ]  
//    EstG.V.Y [N+1]  =   EstG.V.Y [N] + [ ( delta[PITCH] * EstG.V.Z [N] ) - ( delta[YAW]   * EstG.V.X [N] ) ]
//    EstG.V.Z [N+1]  =   EstG.V.Z [N] - [ ( delta[ROLL]  * EstG.V.X [N] ) + ( delta[PITCH] * EstG.V.Y [N] ) ]  
//
//
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void getEstimatedAttitude(){
  uint8_t axis;                                // Variável para referenciar os 3 eixos (ROLL, PITCH, YAW) 
  int32_t accMag = 0;                          // Variável que armazena a magnitude do vetor de aceleração.
  static t_fp_vector EstG;                     // Declara EstG  
  
#if MAG
  static t_fp_vector EstM;                     // Declara EstM
#endif

#if defined(MG_LPF_FACTOR)                     // FILTRO LOW-PASS DO MAGNETOMETRO
  static int16_t mgSmooth[3];                  // SE ATIVO, Declara o vetor mgSmooth[3];
#endif

#if defined(ACC_LPF_FACTOR)                    // FILTRO LOW-PASS DO ACELEROMETRO
  static float accLPF[3];                      // SE ATIVO, Declara o vetor accLPF[3]; 
#endif

  static uint16_t previousT;                   // previousT = armazena tempo anterior (em microssegundos)
  uint16_t currentT = micros();                // currentT  = armazena tempo atual (em microssegundos)      
  float scale, deltaGyroAngle[3];              // scale = escala do sensor giroscópico
                                               // deltaGyroAngle[3] = distancia percorrida pelos 3 eixos (em rad)
                                    
  scale = (currentT - previousT) * GYRO_SCALE;                                                                // multiplicamos a escala do giroscopio ao intervalo de tempo para a integração simples
  previousT = currentT;                        // atualização de previousT

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * scale;                                                            //  Distancia percorrida por integração simples  ( v (dt) =>  s (dt) ) =>  (rad/s => rad)
                                                                                                              //  deltaGyroAngle[0] = ROLL  deltaGyroAngle[1] = PITCH  deltaGyroAngle[2] = YAW 
    
                                                                                                              // ____________________________________________________________________________________________________________________________________________________
    #if defined(ACC_LPF_FACTOR)                                                                               // |FILTRO PARA ACELERÔMETRO ATIVADO    =>   reduce ACC noise (visible in GUI), but would increase ACC lag time                                       |
      accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);    // |   accLPF[axis] = (accLPF[axis]* 0.99)  +  (accADC[axis] * 0.01)  // demora mais iterações para accLPF[axis] alcançar o valor de accADC[axis]     |  
      accSmooth[axis] = accLPF[axis];                                                                         // |   accSmooth[axis] = accLPF[axis]                                                                                                                 |
      #define ACC_VALUE accSmooth[axis]                                                                       // |   ACC_VALUE = accSmooth[axis]                                                                                                                    |
    #else                                                                                                     // |FILTRO PARA ACELERÔMETRO DESATIVADO =>                                                                                                            | 
      accSmooth[axis] = accADC[axis];                                                                         // |   accSmooth[axis] = accADC[axis]                                                                                                                 |
      #define ACC_VALUE accADC[axis]                                                                          // |   ACC_VALUE = accADC[axis]                                                                                                                       |
    #endif                                                                                                    // |__________________________________________________________________________________________________________________________________________________|                                                                                                                                                  
     
     
                                                                                                              //   accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);           (10^2) = 100, como pode-se obervar na ultima aquisição       
    accMag += (int32_t)ACC_VALUE*ACC_VALUE ;                                                                  //   (accADC[0]^2  +  accADC[1]^2  +  accADC[3]^2)                                                         de accMag
    
    
    #if MAG                                                                                                   // ___________________________________________________________________________________________________________________________________________________ 
      #if defined(MG_LPF_FACTOR)                                                                              // |FILTRO PARA MAGNETÔMETRO ATIVADO     =>                                                                                                           |
        mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR;               // |   mgSmooth[axis] = (mgSmooth[axis] * 3 + magADC[axis]) / 4     // demora mais iterações para mgSmooth[axis] alcançar o valor de magADC[axis]     |                                
        #define MAG_VALUE mgSmooth[axis]                                                                      // |   MAG_VALUE = mgSmooth[axis]                                                                                                                     |
      #else                                                                                                   // |FILTRO PARA MAGNETÔMETRO DESATIVADO  =>                                                                                                           |
        #define MAG_VALUE magADC[axis]                                                                        // |   MAG_VALUE = magADC[axis]                                                                                                                       |
      #endif                                                                                                  // |                                                                                                                                                  |
    #endif                                                                                                    // |__________________________________________________________________________________________________________________________________________________|
  }
  
  
                                                                                                              // valores estão multiplicaos por 10
  accMag = accMag*100/((int32_t)acc_1G*acc_1G);                                                               // Mag*(1g)^2 = 100*(accADC[0]^2  +  accADC[1]^2  +  accADC[3]^2)   =>  Mag*(255^2) = 100* (accADC[0]^2  +  accADC[1]^2  +  accADC[3]^2)              
                                                                                                              // Mag = 100*((accADC[0]^2  +  accADC[1]^2  +  accADC[3]^2)/ (255^2))
                                                                                                                                                                                                                       
                                                                                                           
  // Aplica-se a matriz de rotação -----------------------------------------------------------------------------------------------------------------------------                                                                                                            
  rotateV(&EstG.V,deltaGyroAngle);                                                                             // Matriz de rotação para valores estimados EstG
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);                                                                           // Matriz de rotação para valores estimados EstM
  #endif 
  //------------------------------------------------------------------------------------------------------------------------------------------------------------
                                                                                                  //                                                                                         
                                                                                                  //   acc_1G * 0.423  =>    255 * 0.423 = 107.865   = 0.42g                                  |       
  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {   //                                                                                          |     
    f.SMALL_ANGLES_25 = 1;                                                                        //   Se ROLL < 24.84° e PITCH < 24.84°  e accZ > 0                        __________________|___________________   x/y = 0.42g  
  } else {                                                                                        //            flag f.SMALL_ANGLES_25 = 1;                                                   |\   
    f.SMALL_ANGLES_25 = 0;                                                                        //                                                                                          | \  24.84° de inclinação máxima de PITCH e ROLL
  }                                                                                               //                                                                                          |  \ 
                                                                                                  //                                                                                          |- -\   
                                                                                                  //                                                                                          |    \  R = 1
                                                                                                  //                                                                                          |     
                                                                                                  //                                                                                          z = 0.907g 
 //-------------------------------------------------------------------------------------------------------------------------------------------------------------                                                                                                                                                                                          
                                                                                                                                                                                            
  // Aplica-se o Filtro Complementar
  // Se a magnitude do acelerômetro for >1.4G ou <0.6G e os vetores do acelerômetros ultrapassarem 24.84° => neutraliza-se o efeito do acelerômetro da estimação do ângulo (acelerômetro com peso 0)
  // Para fazer isto, basta pular o filtro complementar, com EstV já rotacionado pelo Gyro
  
  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 )                             //| Se a magnitude do acelerômetro estiver no intervalo de   0.6g < accMAg < 1.4g, ou se ROLL e PITCH tiveram angulos menores que 24.84° =>  | 
    for (axis = 0; axis < 3; axis++) {                                                    //|   utiliza-se acelerômetro com seu peso                                                                                                   |
      int16_t acc = ACC_VALUE;                                                            //|   acc = ACC_VALUE = accSmooth[axis]                                                                                                                      |
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;        //|   aplica-se pesos do filtro complementar                                                                                                 |
    }   
    
    //    (1.4g / ( 16/ 2^12 )) = 358
    //    358^2 / 255^2 = 1.96
    //    1.97 * 100 = 196
    
//--------------------------------------------------------------------------------------------------------------------------------------------------------------


  #if MAG
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;  //  aplica-se pesos do filtro complementar  
  #endif
  
  
  //------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Attitude of the estimated vector                    //Aplica a aproximação de arctan para calcular ângulos de ROLL  e PITCH 
  angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
  angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;
  //------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  
  #if MAG
    // Attitude do produto vetorial entre GxM, refletindo (x-z -> y) e (y-z -> x), e calculando assim, o ângulo entre os dois. 
    heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );  // Aplica a aproximação de arctan para calcular a orientação da arfagem
    heading += MAG_DECLINIATION * 10; //add declination                                                          // Aplica-se declinação magnética da região de operação para apontar ao norte verdadeiro
    heading = heading /10;
    if ( heading > 180)      heading = heading - 360;
    else if (heading < -180) heading = heading + 360;
  #endif
}


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//PID PARA ALTITUDE
#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000  // 4 sec initialization delay
#define BARO_TAB_SIZE   40

void getEstimatedAltitude(){
  uint8_t index;
  static uint32_t deadLine = INIT_DELAY;

  static int16_t BaroHistTab[BARO_TAB_SIZE];
  static int8_t BaroHistIdx;
  static int32_t BaroHigh,BaroLow;
  int32_t temp32;
  int16_t last;

  if (abs(currentTime - deadLine) < UPDATE_INTERVAL) return;                //espera 4sec na primeira inicialização;   espera 25ms a cada iteração após a inicialização 
  deadLine = currentTime; 

  //**** Alt. Set Point stabilization PID ****                                                                                    
  //calculate speed for D calculation                                      
  last = BaroHistTab[BaroHistIdx];                                          
  BaroHistTab[BaroHistIdx] = BaroAlt/10;                                   
  BaroHigh += BaroHistTab[BaroHistIdx];                                     
  index = (BaroHistIdx + (BARO_TAB_SIZE/2))%BARO_TAB_SIZE;                  
  BaroHigh -= BaroHistTab[index];                                           
  BaroLow  += BaroHistTab[index];                                           
  BaroLow  -= last;                                                         

  BaroHistIdx++;                                                            
  if (BaroHistIdx == BARO_TAB_SIZE) BaroHistIdx = 0;                        
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
  BaroPID = 0;
  //D
  temp32 = conf.D8[PIDALT]*(BaroHigh - BaroLow) / 40;
  BaroPID-=temp32;

  EstAlt = BaroHigh*10/(BARO_TAB_SIZE/2);
  
  temp32 = AltHold - EstAlt;
  if (abs(temp32) < 10 && abs(BaroPID) < 10) BaroPID = 0;  //remove small D parametr to reduce noise near zero position
  
  //P
  BaroPID += conf.P8[PIDALT]*constrain(temp32,(-2)*conf.P8[PIDALT],2*conf.P8[PIDALT])/100;   
  BaroPID = constrain(BaroPID,-150,+150); //sum of P and D should be in range 150

  //I
  errorAltitudeI += temp32*conf.I8[PIDALT]/50;
  errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
  temp32 = errorAltitudeI / 500; //I in range +/-60
  BaroPID+=temp32;
}

