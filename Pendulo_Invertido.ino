

#include <Wire.h>
#include <Kalman.h>

uint8_t i2c_data[14];
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

uint32_t timer;

Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;

double KalAngleX;
double KalAngleY;
double KalAngleZ;

double gyroXangle;
double gyroYangle;


void setup() {

  
  Serial.begin(115200);                                                   /* Inicializando a Serial para exibir mensagens de Debug */

  
  Wire.begin();                                                           /* Inicializando o Barramento i2c para comunicação com a MPU6050 */

/* Definição do Clock*/
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Freq = 400kHz                                                        
#else
  TWBR = ((F_CPU/400000UL) - 16) / 2; // Freq = 400kHz
#endif

  i2c_data[0] = 7;                                                        /* 0x19 - Taxa de amostragem  8kHz/(7 + 1) = 1000Hz */
  i2c_data[1] = 0x00;                                                     /* 0x1A - Desabilitar FSYNC, Configurar o Filtro de ACC 260Hz, Configurar Filtro de Gyro 256Hz, Amostragem de 8Khz */
  i2c_data[2] = 0x00;                                                     /* 0x1B - Configurar o fundo de escala do Gyro ±250deg/s - Faixa */
  i2c_data[3] = 0x00;                                                     /* 0x1C - Configurar o fundo de escala do Acelerômetro para ±2g - Faixa */


  while(i2cWrite(0x19, i2c_data, 4, false));                              /* Configirações do i2c*/

 
  while(i2cWrite(0x6B, 0x01, true));                                      /* PLL tenha como referência o gyro de eixo X, Desabilitando Sleep Mode */


  while(i2cRead(0x75, i2c_data, 1));
  }

  
  delay(100);                                                             /* Tempo de estabilização do Sensor MPU6050 */

  
  while(i2cRead(0x3B, i2c_data, 14));                                     

  /* Organizar os dados de Acc XYZ */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])  
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])

 
  double angX = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;    /*Calculo do angX*/     
  double angY = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;    /*Calculo do angY*/      

  /* Inicialização do Filtro de Kalman XY */
  KalmanX.setAngle(angY);
  KalmanY.setAngle(angX);

  gyroXangle = angY;
  gyroYangle = angX;
 
  timer = micros();

  init_motores();
}

void loop() {

  
  while(i2cRead(0x3B, i2c_data, 14));                                        /* Leitura dos Dados de Aceleração e Gyro do sensor MPU6050 */

  /* Leitura dos dados de Aceleração do sensor MPU6050 */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])      
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])

  /* Leitura dos dados do Giroscópio do sensor MPU6050 */
  gyroX = (int16_t)((i2c_data[8] << 8) | i2c_data[9]); // ([ MSB ] [ LSB ])
  gyroY = (int16_t)((i2c_data[10] << 8) | i2c_data[11]); // ([ MSB ] [ LSB ])
  gyroZ = (int16_t)((i2c_data[12] << 8) | i2c_data[13]); // ([ MSB ] [ LSB ])


  
  /* Calculo do Delta Time */
  double dt = (double)(micros() - timer)/1000000;
  timer = micros();

  double angX = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;    /* Convertendo o ângulo referente a X de Radiano para graus */
  double angY = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;    /* Convertendo o ângulo referente a X de Radiano para graus */

  /* Convertendo de Rad/Segundo para Graus/Segundo */
  /* Calculo da Taxa angular baseado no Giroscópio */
  gyroXangle = gyroX / 131.0; //deg/s                                       
  gyroYangle = gyroY / 131.0; //deg/s

  
  KalAngleX = KalmanX.getAngle(angY, gyroXangle, dt);                       /* Calculo estimado de ãngulo em X usando o filtro de Kalman */
  KalAngleY = KalmanY.getAngle(angX, gyroYangle, dt);                       /* Calculo estimado de ãngulo em Y usando o filtro de Kalman */


  double res = Compute(KalAngleY);                                           /* Resposta do ângulo Y após aplicação do filtro de Kalman */
  PMWControleMotores(res);  

}
