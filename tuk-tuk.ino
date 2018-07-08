#include <QTRSensors.h>

//Mapeamento de pinos
#define STBY 6
#define AIN1 5
#define AIN2 4
#define PWMB 9
#define PWMA 3
#define BIN1 8
#define BIN2 7
#define NUM_SENSORS 6
#define NUM_SAMPLES_PER_SENSOR 4
#define EMITTER_PIN 11
#define LED 13
#define S_DIR A0
#define S_ESQ A7

#define FILTERS_SIZE 10
#define RIGHT_NOTATION_NUM 6
#define NOTATION_EXISTS_IF 500 

unsigned int right_filters[FILTERS_SIZE] = {0};
unsigned int left_filters[FILTERS_SIZE] = {0};

// Constantes para PID
float KP = 0.10;
float KD = 1.10;
float Ki = 0.0150;

// Velocidade Máxima
//int Velmax = 125;
int Velmax = 70;

// Data para integral
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

unsigned int position = 0;

//declaraos variables para utilizar PID
int proporcional = 0; // Proporcional
int integral = 0;     //Intrgral
int derivativo = 0;   // Derivativo

int diferencial = 0; // Diferencia aplicada a los motores
int last_prop;       // Última valor del proporcional (utilizado para calcular la derivada del error)
int Target = 2500;   // Setpoint (Como utilizamos 6 sensores, la línea debe estar entre 0 y 5000, por lo que el ideal es que esté en 2500)

unsigned long setup_time = 0;

QTRSensorsAnalog qtra((unsigned char[]){A1, A2, A3, A4, A5, A6}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void Motoriz(int value)
{
    if (value >= 0)
    {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    else
    {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        value *= -1;
    }
    analogWrite(PWMB, value);
}

void Motorde(int value)
{
    if (value >= 0)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        value *= -1;
    }
    analogWrite(PWMA, value);
}

void Motor(int left, int righ)
{
    digitalWrite(STBY, HIGH);
    Motoriz(left);
    Motorde(righ);
}

void freno(boolean left, boolean righ, int value)
{
    digitalWrite(STBY, HIGH);
    if (left)
    {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMB, value);
    }
    if (righ)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, value);
    }
}

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
  
    for (int i = 0; i < 70; i++)
    {
      digitalWrite(LED, HIGH);
      delay(20);
      qtra.calibrate();
      digitalWrite(LED, LOW);
      delay(20);
    }
    delay(3000);
}


void loop()
{
    position = qtra.readLine(sensorValues, true, true);
    proporcional = ((int)position) - 2500;
    
    if (proporcional <= -Target)
    {
      Motorde(0);
      freno(true, false, 255);
    }
    else if (proporcional >= Target)
    {
      Motoriz(0);
      freno(false, true, 255);
    }
    
    derivativo = proporcional - last_prop;
    integral = error1 + error2 + error3 + error4 + error5 + error6;
    last_prop = proporcional;
    
    error6 = error5;
    error5 = error4;
    error4 = error3;
    error3 = error2;
    error2 = error1;
    error1 = proporcional;

    int diferencial = (proporcional * KP) + (derivativo * KD) + (integral * Ki);
  
    if (diferencial > Velmax)
      diferencial = Velmax;
    else if (diferencial < -Velmax)
      diferencial = -Velmax;
    
    (diferencial < 0) ? Motor(Velmax + diferencial, Velmax) : Motor(Velmax, Velmax - diferencial);

    filter_left_update(analogRead(S_ESQ));
    filter_right_update(analogRead(S_DIR));
    stop(millis());
}

int finish;
unsigned long seconds = 0;

unsigned long previousMillisDir = 0;
const long intervalDir = 22000;

void stop(long currentMillisChegada)
{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    if (currentMillisChegada - previousMillisDir >= intervalDir)
    {
        if (filter_right_sum() < 7250 && filter_left_sum() > 8000)
        {
            Motor(0, 0);
            delay(2000);
        }
    }
}

int filter_right_sum()
{
    int sum = 0;
    for(int i = 0; i < FILTERS_SIZE; i++)
    {
        sum = sum + right_filters[i];
    }
    return sum;
}

int filter_left_sum()
{
    int sum = 0;
    for(int i = 0; i < FILTERS_SIZE; i++)
    {
        sum = sum + left_filters[i];
    }
    return sum;
}

void filter_left_update(int value)
{
    for (int i = FILTERS_SIZE - 1; i > 0; i--)
    {
        left_filters[i] = left_filters[i-1];
    }
    left_filters[0] = value;
}

void filter_right_update(int value)
{
    for (int i = FILTERS_SIZE - 1; i > 0; i--)
    {
        right_filters[i] = right_filters[i-1];
    }
    right_filters[0] = value;
}

