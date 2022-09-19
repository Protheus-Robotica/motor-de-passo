#include <Arduino.h>
#define MOTOR_OUT_1 11
#define MOTOR_DIR_1 31
#define MOTOR_EN_1 32

#define MOTOR_OUT_2 5
#define MOTOR_DIR_2 33
#define MOTOR_EN_2 34

#define MOTOR_OUT_3 6
#define MOTOR_DIR_3 35
#define MOTOR_EN_3 36

#define MOTOR_OUT_4 12
#define MOTOR_DIR_4 37
#define MOTOR_EN_4 38

#define MOTOR_OUT_5 2
#define MOTOR_DIR_5 39
#define MOTOR_EN_5 40

#define MOTOR_OUT_6 7
#define MOTOR_DIR_6 41
#define MOTOR_EN_6 42

class Stepper
{
    private:
        volatile uint8_t *TCCRA, *TCCRB, *TIMSK;
        uint8_t OCIE, COM0;
    
    public:
        int16_t steps;
        uint16_t timer_reset_v;
        volatile uint16_t *OCR;
        int out_port, dir_port, en_port; 
        bool direction;

        Stepper(volatile uint16_t *OCR, volatile uint8_t *TCCRA, volatile uint8_t *TCCRB, volatile uint8_t *TIMSK, const uint8_t OCIE, const uint8_t COM0, const int out_port, const int dir_port, const int en_port);
        void setSpeed(short speed);
        void setDirection(bool direction, int16_t steps);
};

Stepper::Stepper(volatile uint16_t *c_OCR, volatile uint8_t *c_TCCRA, volatile uint8_t *c_TCCRB, volatile uint8_t *c_TIMSK, const uint8_t c_OCIE, const uint8_t c_COM0, const int c_out_port, const int c_dir_port, const int c_en_port)
{
    OCR = c_OCR;
    COM0 = c_COM0;
    OCIE = c_OCIE;

    TCCRA = c_TCCRA;
    TCCRB = c_TCCRB;
    TIMSK = c_TIMSK;
    

    out_port = c_out_port;
    dir_port = c_dir_port;
    en_port = c_en_port;

    *OCR = timer_reset_v; // Set compare value
    *TCCRA |= (1 << COM0); // Enable Toggle OCnX on Compare Match
    *TCCRB |= (1 << CS11); // CSn1 = Set Prescaler to 8 || (1 << WGMn2) to enable CTC Mode
    *TIMSK |= (1 << OCIE); // OCIEnX = Enable Output on Compare Interrupts for A/B/C
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
        *TCCRA &= ~(1 << COM0); // Disconnect OCnX from port
        digitalWrite(en_port, HIGH); // Disable motor by setting TB6560 EN+ to high
        digitalWrite(out_port, LOW);

    } else
    {
        *TCCRA |= (1 << COM0); // Enable Toggle OCnX on Compare Match
        digitalWrite(en_port, LOW);
    }
    
    int frequency = 8*speed + 200; 
    timer_reset_v = (1000000/frequency) - 1; // Calculate timer count for frequency based on ATMEGA2560 datasheet
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

Stepper *motor = (Stepper*) malloc(6 * sizeof(Stepper)); 

ISR(TIMER1_COMPA_vect)
{
    *(motor[0].OCR) += motor[0].timer_reset_v; // Set next Compare Value
    if (motor[0].steps == 0)
    {
        motor[0].setSpeed(0);
    } else if (motor[0].steps > -1)
    {
        motor[0].steps--;
    }
}

ISR(TIMER3_COMPA_vect)
{
    *(motor[1].OCR) += motor[1].timer_reset_v; 
    if (motor[1].steps == 0)
    {
        motor[1].setSpeed(0);
    } else if (motor[1].steps > -1)
    {
        motor[1].steps--;
    }
}

ISR(TIMER4_COMPA_vect)
{
    *(motor[2].OCR) += motor[2].timer_reset_v;  
    if (motor[2].steps == 0)
    {
        motor[2].setSpeed(0);
    } else if (motor[2].steps > -1)
    {
        motor[2].steps--;
    }
}

ISR(TIMER1_COMPB_vect)
{
    *(motor[3].OCR) += motor[3].timer_reset_v;  
    if (motor[3].steps == 0)
    {
        motor[3].setSpeed(0);
    } else if (motor[3].steps > -1)
    {
        motor[3].steps--;
    }
}

ISR(TIMER3_COMPB_vect)
{
    *(motor[4].OCR) += motor[4].timer_reset_v;  
    if (motor[4].steps == 0)
    {
        motor[4].setSpeed(0);
    } else if (motor[4].steps > -1)
    {
        motor[4].steps--;
    }
}

ISR(TIMER4_COMPB_vect)
{
    *(motor[5].OCR) += motor[5].timer_reset_v;  
    if (motor[5].steps == 0)
    {
        motor[5].setSpeed(0);
    } else if (motor[5].steps > -1)
    {
        motor[5].steps--;
    }
}


void setup() {
    Serial.begin(9600);
    motor[0] = Stepper(&OCR1A, &TCCR1A, &TCCR1B, &TIMSK1, OCIE1A, COM1A0 ,MOTOR_OUT_1, MOTOR_DIR_1, MOTOR_EN_1);
    motor[1] = Stepper(&OCR3A, &TCCR3A, &TCCR3B, &TIMSK3, OCIE3A, COM3A0 ,MOTOR_OUT_2, MOTOR_DIR_2, MOTOR_EN_2);
    motor[2] = Stepper(&OCR4A, &TCCR4A, &TCCR4B, &TIMSK4, OCIE4A, COM4A0 ,MOTOR_OUT_3, MOTOR_DIR_3, MOTOR_EN_3);
    motor[3] = Stepper(&OCR1B, &TCCR1A, &TCCR1B, &TIMSK1, OCIE1B, COM1B0 ,MOTOR_OUT_4, MOTOR_DIR_4, MOTOR_EN_4);
    motor[4] = Stepper(&OCR3B, &TCCR3A, &TCCR3B, &TIMSK3, OCIE3B, COM3B0 ,MOTOR_OUT_5, MOTOR_DIR_5, MOTOR_EN_5);
    motor[5] = Stepper(&OCR4B, &TCCR4A, &TCCR4B, &TIMSK4, OCIE4B, COM4B0 ,MOTOR_OUT_6, MOTOR_DIR_6, MOTOR_EN_6);
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
    } while (motor_n < 1 || motor_n > 6);

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
