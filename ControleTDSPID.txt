#include <Adafruit_ADS1015.h>
#include <PID_v1.h>

// Pino de entrada analógica para leitura do valor de TDS
const int TDS_PIN = A0;

Adafruit_ADS1015 ads;  // Objeto para o conversor analógico-digital ADS1015

// Configuração do PID
double setpoint = 500.0; // Valor de TDS desejado em ppm
double input, output;
double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error, lastError;
double cumError, rateError;

// Definindo os limites de saída do PID
double outMin = 0;
double outMax = 255;

// Pinagem do dispositivo de controle de TDS (por exemplo, uma bomba peristáltica para ajustar a solução)
// Substitua pelo pino que controla o dispositivo em seu hardware específico
const int CONTROL_PIN = 13;

void setup() {
  Serial.begin(9600);

  ads.begin();

  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);
}

void loop() {
  double currentTDS = readTDS();

  // Chama a função do PID
  PID_Controller(currentTDS);

  // Exibe informações no Serial Monitor
  Serial.print("Valor de TDS Atual: ");
  Serial.print(currentTDS);
  Serial.print(" ppm | Saída do PID: ");
  Serial.println(output);

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

double readTDS() {
  // Lê o valor de TDS do sensor Adafruit STEMMA Analog TDS
  // Certifique-se de calibrar o sensor adequadamente
  int rawValue = ads.readADC_SingleEnded(TDS_PIN);
  double TDSValue = map(rawValue, 0, 1023, 0, 5000); // Mapeia a leitura para o intervalo de TDS (0 a 5000 ppm)
  return TDSValue;
}

void PID_Controller(double currentTDS) {
  // Calcula o erro
  error = setpoint - currentTDS;

  // Calcula o tempo decorrido
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  // Calcula as componentes PID
  double P = Kp * error;
  cumError += error * elapsedTime;
  double I = Ki * cumError;
  rateError = (error - lastError) / elapsedTime;
  double D = Kd * rateError;

  // Calcula a saída PID
  output = P + I + D;

  // Limita a saída do PID dentro dos limites especificados
  output = constrain(output, outMin, outMax);

  // Aplica a saída do PID para controlar algum dispositivo (por exemplo, uma bomba peristáltica)
  // Neste exemplo, apenas exibimos a saída no Serial Monitor
  // Você deve adaptar esta parte para controlar seu sistema específico
  // (por exemplo, controle de uma bomba peristáltica para ajustar a solução)
  digitalWrite(CONTROL_PIN, output > 50); // Liga o dispositivo se a saída for maior que 50

  // Atualiza variáveis para a próxima iteração
  lastError = error;
  previousTime = currentTime;
}
