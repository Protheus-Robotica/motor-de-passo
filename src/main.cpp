#include <Arduino.h>

// DO NOT CHANGE OUTPUT PINS, DEFINED BY INTERNAL TIMER WIRING 
// NÃO MUDAR PINOS DE SAÍDA, DEFINIDOS POR LIGAÇÕES INTERNAS DO TIMER

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

// #define DEBUG // Enable visualization of register values on serial monitor
//                  Habilita visualização de valores dos registradores no monitor serial

// Class to control each motor
// Classe para controle de cada motor
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
        int distance;
        bool moving;
        int totalsteps;

        Stepper(volatile uint16_t *OCR, volatile uint8_t *TCCRA, volatile uint8_t *TCCRB, volatile uint8_t *TIMSK, const uint8_t OCIE, const uint8_t COM0, const int out_port, const int dir_port, const int en_port);
        void setSpeed(short speed);
        void setDirection(bool direction, int16_t steps);
        void setFrequency(uint16_t frequency);
        void setSteps(bool direction, int c_steps);
};

Stepper::Stepper(volatile uint16_t *c_OCR, volatile uint8_t *c_TCCRA, volatile uint8_t *c_TCCRB, volatile uint8_t *c_TIMSK, const uint8_t c_OCIE, const uint8_t c_COM0, const int c_out_port, const int c_dir_port, const int c_en_port)
{
    cli();
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
                          // Configura o comparador  

    *TCCRA |= (1 << COM0); // Enable Toggle OCnX on Compare Match
                           // Habilita Alternar OCnX em verificação de comparação

    *TCCRB |= (1 << CS11); // CSn1 = Set Prescaler to 8 | (1 << WGMn2) to enable CTC Mode
                           // CSn1 = Configura Prescaler para 8 | (1 << WGMn2) habilita modo CTC

    *TIMSK |= (1 << OCIE); // OCIEnX = Enable Compare Interrupts
                           // OCIEnX = Habilita Interrupções em Comparação
                           

    direction = false;
    steps = -1;
    moving = false;
    
    pinMode(out_port, OUTPUT);
    pinMode(dir_port, OUTPUT);
    pinMode(en_port, OUTPUT);
    
    setSteps(0, 0);
    totalsteps = 0;
    // setDirection(direction, -1);
    sei();
};

void Stepper::setSpeed(short speed)
{
    if (speed == 0)
    {
        *TCCRA &= ~(1 << COM0); // Disconnect OCnX from port
                                // Disconecta a saída de OCnX do pino

        // digitalWrite(en_port, HIGH); // Disable motor by setting TB6560 EN+ to high
                                     // Desabilita motor configurando TB6560 EN+ para estado lógico alto

        digitalWrite(out_port, LOW);
        moving = false;
        return;
    } else
    {
        *TCCRA |= (1 << COM0);
        moving = true;
        // digitalWrite(en_port, LOW);
    }
    int frequency = 8*speed + 50; 
    timer_reset_v = (1000000/frequency) - 1; // Calculate timer count for frequency based on ATMEGA2560 datasheet
                                             // Calcula contador para frequência baseado no datasheet do ATMEGA2560
};

void Stepper::setFrequency(uint16_t frequency)
{
  timer_reset_v = (1000000/frequency) - 1;
  // Serial.println(timer_reset_v);
}

void Stepper::setSteps(bool direction, int c_steps)
{
    if (c_steps == 0) 
    {
        *TCCRA &= ~(1 << COM0); // Disconnect OCnX from port
                                // Disconecta a saída de OCnX do pino

        // digitalWrite(en_port, HIGH); // Disable motor by setting TB6560 EN+ to high
                                     // Desabilita motor configurando TB6560 EN+ para estado lógico alto

        digitalWrite(out_port, LOW);
        moving = false;
        return;
    } else
    {
        *TCCRA |= (1 << COM0);
        moving = true;
    }

    digitalWrite(dir_port, direction);
    steps = c_steps;
}


// Configure direction and distance of movement based on axle of 1.419 cm
// Configura direção e distância de movimento baseado em fuso com 1.419 cm
void Stepper::setDirection(bool direction, int16_t distance)
{
    digitalWrite(dir_port, direction);
    steps = (distance == -1)?-1:(distance/1.8)*2;
}

// Allocate space for the motor array
// Aloca espaço para a array de motores
Stepper *motor = (Stepper*) malloc(6 * sizeof(Stepper)); 

// Function trigerred by compare match on A for Timer 1
// Função ativada por comparação em A para Timer 1
ISR(TIMER1_COMPA_vect)
{
    // Set next OCR value based on timer_reset_v calculated by setSpeed(), will overflow accordingly to TCNT
    // Configura próximo valor OCR baseado em timer_reset_v calculado por setSpeed(), vai dar overflow corretamente para o TCNT
    *(motor[0].OCR) += motor[0].timer_reset_v; 
    if (motor[0].steps == 0)
    {
        // motor[0].setDirection(!(motor[0].direction), motor[0].distance);
        // motor[0].direction = !(motor[0].direction);
        motor[0].setSteps(0, 0);
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
        // motor[1].setDirection(!(motor[1].direction), motor[1].distance);
        // motor[1].direction = !(motor[1].direction);
        motor[1].setSteps(0, 0);
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
        // Serial.println("Mudou");
        // motor[2].setDirection(!(motor[2].direction), motor[2].distance);
        // motor[2].direction = !(motor[2].direction);
        
        // motor[2].setSpeed(0);
        motor[2].direction = !(motor[2].direction);
        motor[2].setSteps(motor[2].direction, motor[2].totalsteps);
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
        motor[3].setDirection(!(motor[3].direction), motor[3].distance);
        motor[3].direction = !(motor[3].direction);
        // motor[3].setSpeed(0);
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
        motor[4].setDirection(!(motor[4].direction), motor[4].distance);
        motor[4].direction = !(motor[4].direction);
        // motor[4].setSpeed(0);
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
        motor[5].setDirection(!(motor[5].direction), motor[5].distance);
        motor[5].direction = !(motor[5].direction);
        // motor[5].setSpeed(0);
    } else if (motor[5].steps > -1)
    {
        motor[5].steps--;
    }
}

// Set registers to 0 to use bitshift for setting up timers
// Configura registradores para 0 para usar bitshift ao configurar timers 
void clean()
{
    cli();
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    
    TCNT3 = 0;
    TCCR3A = 0;
    TCCR3B = 0;

    TCNT4 = 0;
    TCCR4A = 0;
    TCCR4B = 0;
    sei();
}

void setup() {
    Serial.begin(9600);
    clean();

    // Instantiate all motors
    // Cria instâncias para todos motores
    motor[0] = Stepper(&OCR1A, &TCCR1A, &TCCR1B, &TIMSK1, OCIE1A, COM1A0, MOTOR_OUT_1, MOTOR_DIR_1, MOTOR_EN_1);
    motor[1] = Stepper(&OCR3A, &TCCR3A, &TCCR3B, &TIMSK3, OCIE3A, COM3A0, MOTOR_OUT_2, MOTOR_DIR_2, MOTOR_EN_2);
    motor[2] = Stepper(&OCR4A, &TCCR4A, &TCCR4B, &TIMSK4, OCIE4A, COM4A0, MOTOR_OUT_3, MOTOR_DIR_3, MOTOR_EN_3);
    // motor[3] = Stepper(&OCR1B, &TCCR1A, &TCCR1B, &TIMSK1, OCIE1B, COM1B0, MOTOR_OUT_4, MOTOR_DIR_4, MOTOR_EN_4);
    // motor[4] = Stepper(&OCR3B, &TCCR3A, &TCCR3B, &TIMSK3, OCIE3B, COM3B0, MOTOR_OUT_5, MOTOR_DIR_5, MOTOR_EN_5);
    // motor[5] = Stepper(&OCR4B, &TCCR4A, &TCCR4B, &TIMSK4, OCIE4B, COM4B0, MOTOR_OUT_6, MOTOR_DIR_6, MOTOR_EN_6);
    motor[0].setFrequency(1600);
    motor[1].setFrequency(1400);
    motor[2].setFrequency(1200);
    motor[2].totalsteps = 1300;
    motor[2].setSteps(0, motor[2].totalsteps);
}

void loop() {
    motor[0].setSteps(0, 4600);


    while(motor[0].moving) {
        delay(1);
    }
    
    motor[1].setSteps(0, 2400);
    
    while(motor[1].moving) {
        delay(1);
    }

    motor[1].setSteps(1, 2400);

    while(motor[1].moving) {
        delay(1);
    }

    motor[0].setSteps(1, 4600);

    while(motor[0].moving) {
        delay(1);
    }
}
