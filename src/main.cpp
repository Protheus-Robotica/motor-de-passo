#include <Arduino.h>
#define MOTOR_OUT_1 35
#define MOTOR_DIR_1 33
#define MOTOR_EN_1 31

#define MOTOR_OUT_2 37
#define MOTOR_DIR_2 39
#define MOTOR_EN_2 41

#define MOTOR_OUT_3 43
#define MOTOR_DIR_3 45
#define MOTOR_EN_3 47


class Stepper
{
private:
    volatile uint16_t *TCNT;
    volatile uint8_t *TCCRA, *TCCRB, *TIMSK;

public:
    int16_t steps;
    uint16_t timer_reset_v;
    int out_port, dir_port, en_port; 
    bool direction;

    Stepper(volatile uint16_t *TCNT, volatile uint8_t *TCCRA, volatile uint8_t *TCCRB, volatile uint8_t *TIMSK, const int out_port, const int dir_port, const int en_port);
    void setSpeed(short speed);
    void setDirection(bool direction, int16_t steps);
};

Stepper::Stepper(volatile uint16_t *c_TCNT, volatile uint8_t *c_TCCRA, volatile uint8_t *c_TCCRB, volatile uint8_t *c_TIMSK, const int c_out_port, const int c_dir_port, const int c_en_port)
{
    TCNT = c_TCNT;
    TCCRA = c_TCCRA;
    TCCRB = c_TCCRB;
    TIMSK = c_TIMSK;
    out_port = c_out_port;
    dir_port = c_dir_port;
    en_port = c_en_port;

    *TCNT = this->timer_reset_v;
    *TCCRA = 0b00000000;
    *TCCRB = 0b00000010;
    *TIMSK = 0b00000000;
    direction = false;
    steps = -1;
    
    pinMode(out_port, OUTPUT);
    pinMode(dir_port, OUTPUT);
    pinMode(en_port, OUTPUT);
    setSpeed(0);
    setDirection(direction, -1);
};

void Stepper::setSpeed(short speed)
{
    if (speed == 0)
    {
        *TIMSK = 0b00000000;
        digitalWrite(en_port, HIGH);

    } else
    {
        *TIMSK = 0b00000001;
        digitalWrite(en_port, LOW);
    }
    
    int frequency = 8*speed + 200;
    timer_reset_v = 65536 - (1000000/frequency);
};

void Stepper::setDirection(bool direction, int16_t distance)
{
    Serial.print("Porta: ");
    Serial.println(out_port);
    Serial.print("Ativo: ");
    Serial.println(*TIMSK);
    digitalWrite(dir_port, direction);
    steps = (distance == -1)?-1:(distance/1.419)*800;
}

//Stepper motor[] = {Stepper(&TCNT1, &TCCR1A, &TCCR1B, &TIMSK1, MOTOR_OUT_1, MOTOR_DIR_1), Stepper(&TCNT3, &TCCR3A, &TCCR3B, &TIMSK3, MOTOR_OUT_2, MOTOR_DIR_2), Stepper(&TCNT4, &TCCR4A, &TCCR4B, &TIMSK4, MOTOR_OUT_3, MOTOR_DIR_3)};
Stepper *motor = (Stepper*) malloc(3 * sizeof(Stepper));

ISR(TIMER1_OVF_vect)
{
    TCNT1 = motor[0].timer_reset_v;
    if (motor[0].steps == 0)
    {
        motor[0].setSpeed(0);
    } else if (motor[0].steps > -1)
    {
        motor[0].steps--;
    }
    
    digitalWrite(motor[0].out_port, !digitalRead(motor[0].out_port));
}

ISR(TIMER3_OVF_vect)
{
    TCNT3 = motor[1].timer_reset_v;
    if (motor[1].steps == 0)
    {
        motor[1].setSpeed(0);
    } else if (motor[1].steps > -1)
    {
        motor[1].steps--;
    }

    digitalWrite(motor[1].out_port, !digitalRead(motor[1].out_port));
}

ISR(TIMER4_OVF_vect)
{
    TCNT4 = motor[2].timer_reset_v;
    if (motor[2].steps == 0)
    {
        motor[2].setSpeed(0);
    } else if (motor[2].steps > -1)
    {
        motor[2].steps--;
    }

    digitalWrite(motor[2].out_port, !digitalRead(motor[2].out_port));

}

void setup() {
    Serial.begin(9600);
    motor[0] = Stepper(&TCNT1, &TCCR1A, &TCCR1B, &TIMSK1, MOTOR_OUT_1, MOTOR_DIR_1, MOTOR_EN_1);
    motor[1] = Stepper(&TCNT3, &TCCR3A, &TCCR3B, &TIMSK3, MOTOR_OUT_2, MOTOR_DIR_2, MOTOR_EN_2);
    motor[2] = Stepper(&TCNT4, &TCCR4A, &TCCR4B, &TIMSK4, MOTOR_OUT_3, MOTOR_DIR_3, MOTOR_EN_3);
}

void loop() {
    short motor_n;
    short speed;
    bool direction;
    int16_t distance;

    do {
        Serial.println("Qual motor voce gostaria de alterar? 1/2/3");
        while (!Serial.available()){}
        motor_n = Serial.readStringUntil('\n').toInt();
    } while (motor_n < 1 || motor_n > 3);

    motor_n--;

    do {
        Serial.println("Insira a velocidade: 0-100%");
        while (!Serial.available()){}
        speed = Serial.readStringUntil('\n').toInt();
    } while (speed < 0 || speed > 100);

    Serial.println("Insira a direcao: 0 - Horario/ 1 - Anti-horario");
    while (!Serial.available()){}
    direction = Serial.readStringUntil('\n').toInt();

    do
    {
        Serial.println("Insira a distancia de movimento ou -1 para movimento continuo (mm):");
        while (!Serial.available()){}
        distance = Serial.readStringUntil('\n').toInt();
    } while (distance != -1 && (distance < 0 || distance > 1000));
    
    Serial.print("Velocidade: ");
    Serial.println(speed);
    motor[motor_n].setSpeed(speed);
    motor[motor_n].setDirection(direction, distance);
    Serial.println();
}
