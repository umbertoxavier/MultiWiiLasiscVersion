// CONFIG.H - LaSisC VERSION 
// VERSÃO SEM GPS
    
    #define QUADX                            // definição da estrutura utilizada para controle, no caso um quadrirotor orientado em X.
    
    #define MINTHROTTLE 1050                 // valor mínimo permitido de aceleração enviado para o ESC. (QUANDO ARMADO)
  
    #define MAXTHROTTLE 1850                 // valor máximo permitido de aceleração enviado para o ESC. (QUANDO ARMADO)
  
    #define MINCOMMAND  1000                 // valor enviado para o ESC quando o quadrirotor não está armado. 
  
    #define I2C_SPEED 100000L                // velocidade da comuinicação i2C  
                                             // "L" força a constante em um formato de data "long"
                                             //https://www.arduino.cc/reference/en/language/variables/constants/integerconstants/
                 
    #define FFIMUv2                          // IMU (Inertial Measurement unit) utilizada
                                             // levando em consideração o acelerômetro, giroscópio, magnetômetro e barômetro presente na placa. 
                                                 
    #define YAW_DIRECTION 1                  // YAW (GUINADA) - para inverter a direção da correção de guinada (-1)
    
    #define ALLOW_ARM_DISARM_VIA_TX_YAW      // armar e desarmar o quadricoptero apenas com o stick de yaw 
                                             //(THROTTLE DOWN + YAW RIGHT) = ARMAR
                                             //(THROTTLE DOWN + YAW LEFT) = DESARMAR        
    
    #define SERIAL_COM_SPEED 115200          // velocidade da comunicação serial.
    
    #define MAG_DECLINIATION  -19.48f        // Cálculo da declinição magnética 
                                             //http://magnetic-declination.com/
                                             //grau = 19  
                                             //minuto = 29
                                             //d = grau + minuto*(1/60)
                                             //d = -19,4833 (negativo por estar em WEST)                           
     
    #define MIDRC 1500                       //Ponto neutro dos sticks do rádio.
    
     
    //------- CALIBRAÇÃO DOS ESCS -------//
    #define ESC_CALIB_LOW  MINCOMMAND        //Valor mínimo para calibração do ESC.
    
    #define ESC_CALIB_HIGH 2000              //Valor máximo para calibração do ESC
    
    //#define ESC_CALIB_CANNOT_FLY
    //Descomentando acima, entra em modo de calibração dos ESCS. 
    
    //define DEADBAND
