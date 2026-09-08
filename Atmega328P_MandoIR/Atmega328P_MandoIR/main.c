/*
* main.c
*
* Created: 5/4/2026 8:12:34 PM
*  Author: Vlady-Chuwi
*/

#define F_CPU 16000000ul

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>


/************************************************************************/
/*			Creamos la estructura para cada entrada de pines            */
/************************************************************************/

typedef struct
{
	int16_t contador;
	uint16_t ledTime;
	bool    ValidarPulso;
	bool	flagPulso;
	bool	flagPin;
	bool    flagLed;
	uint8_t pin;
	volatile uint8_t * port;
	uint8_t pinLED;
	volatile uint8_t * portLED;
	
}InputPin_t;

InputPin_t pinStart={0u,0u,true,false,false,false,(1<<PINB3),&PINB,(1<<PORTC0),&PORTC};
InputPin_t pinStop={0u,0u,true,false,false,false,(1<<PINB4),&PINB,(1<<PORTC1),&PORTC};
InputPin_t pinProg={0u,0u,true,false,false,false,(1<<PINB0),&PINB,(1<<PORTC2),&PORTC};

/************************************************************************/
/*      Creamos la estructura para el modulo RC5 y la inicializamos     */
/************************************************************************/
typedef struct  
{
	uint16_t dataRC5;
	uint8_t  Address;
	uint8_t Command;
	uint8_t Toggle;
	bool flagRC5;
	bool flag10ms;	
	
}RC5_struct;

RC5_struct rc5={0u,0u,0u,0u,true,false};
	
void LeerEntrada(InputPin_t *P,bool PulsoLargo);
void Salida(InputPin_t *P,uint16_t tiempo,bool blink);
void InicializarSistema();

void activar_Timer2() {
	TCNT2 = 0;
	TCCR2B |= (1 << CS22);
}

void detener_Timer2() {
	TCCR2B &= ~(1 << CS22);
}

void activar_PWM_Global() {
	TCNT1 = 0;
    // Volver a conectar el pin al hardware PWM (Non-inverting mode)
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
    // Arrancar el reloj si lo habías detenido
    TCCR1B |= (1 << CS10);
	
}

void detener_PWM_Global() {
	// 1. Desconectar el hardware de PWM de los pines (Vuelve a ser GPIO normal)
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));

	// 2. Detener el reloj del timer (opcional si quieres ahorrar energía)
	TCCR1B &= ~(1 << CS10);

	// 3. Ahora sí, el PORTB tiene el mando. Forzar a LOW:
	PORTB &= ~((1 << PORTB1) | (1 << PORTB2));

}

// Funciones para conectar/desconectar pines específicos durante la trama
void conectar_PB1() { TCCR1A |= (1 << COM1A1); }
void desconectar_PB1() {
	TCCR1A &= ~(1 << COM1A1);
	PORTB &= ~(1 << PORTB1);
}

void conectar_PB2() { TCCR1A |= (1 << COM1B1); }
void desconectar_PB2() {
	TCCR1A &= ~(1 << COM1B1);
	PORTB &= ~(1 << PORTB2);
}
int main(void)
{
	
	InicializarSistema();

	while (1) {
		//Leemos las entradas y al ser validadas se activa el led correspondiente X tiempo y no entra hasta enviar la trama
		if (rc5.flag10ms)
		{
			if(!pinStart.flagPin && !pinStop.flagPin && rc5.flagRC5){// esto es revisar si afecta
				LeerEntrada(&pinStart,false);
				LeerEntrada(&pinStop,false);
				}
			LeerEntrada(&pinProg,true);
			rc5.flag10ms=false;
		}

		// Se envia lo que se tiene en el DIP de start como STOP
		if ((pinStart.flagPin || pinStop.flagPin) && rc5.flagRC5)
		{
			if (pinStart.flagPin)	rc5.Command=PIND&0x0F;		
				else				rc5.Command=(PIND>>4)&0x0F;	
			
			rc5.dataRC5=(0x03<<12)|((rc5.Toggle&0x01)<<11)|((rc5.Address&0x1F)<<6)|(rc5.Command&0x3F);
			rc5.Toggle^=1;
			
			
			if (pinStop.flagPin && pinProg.flagPulso){
				pinProg.flagPulso=false; //si se programo al usar stop se sale del modo
			}
			else{
				rc5.flagRC5=false;	//Entra aca siempre que no este en prog asi evito mandar datos erroneos
				activar_Timer2();		
			}
			pinStart.flagPin=false;
			pinStop.flagPin=false;

		}
		// entrara para cambiar el addres si es necesario y programar un addres nuevo mandando un address maestro
		// este address maestro debera estar metido en el robot
		// se puede mejorar no solo mandando el address nuevo si no mandando comandos nuevos
		if (pinProg.flagPin )
		{
			if (!pinProg.flagPulso)
			{
				rc5.Address=PIND&0x1F; // Se lee los 5 bits de los DIP's y se guarda como adrress local
				pinProg.flagPulso=false;
				PORTC^=(1<<PINC3);
			}
			else{ //en tal caso de mantener en pulso se mandara un addres maestro para configurar la nueva direccion
				//address master = 0x1f     y command sera para el nuevo address que debera tener el robot de 5bits
				rc5.Address=0x1F;
				rc5.Command=PIND&0x1F; // solo se enmascara 5 bits para evitar errores al trasmitir el nuevo address
				
				rc5.dataRC5=(0x03<<12)|((rc5.Toggle&0x01)<<11)|((rc5.Address&0x1F)<<6)|(rc5.Command&0x3F);
				rc5.Toggle^=1;
				
				rc5.flagRC5=false;	
				activar_Timer2();
				PORTC^=(1<<PINC4);	
			}
			pinProg.flagPin=false;
		}

	}
}
void InicializarSistema()
{
	cli();
	
	
	//timer 0 para cada 1 ms
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 249;
	TIMSK0 |= (1 << OCIE0A);
	// timer 2 para 889us 
	TCCR2A = (1 << WGM21);  
	OCR2A = 221;           
	TIMSK2 |= (1 << OCIE2A); 
	TCCR2B = 0;             
	
	//pwm a 36khz para la modulacion 
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12);

	ICR1 = 443;
	OCR1A = 133;
	OCR1B = 133;

	DDRB |= (1 << DDB1) | (1 << DDB2);

	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));

	
	sei();

	//0 entrada y 1 salida
	// PinOut Remote IR
	/*
	PB0= INPUT PROGRAM
	PB1= PWM START
	PB2= PWM_STOP
	PB3= INPUT 1		start
	PB4= INPUT 2		stop
	PC0= LED START
	PC1= LED STOP
	PC2= LED PROG
	PC3= LED 1
	PC4= LED 2
	PD0= SWITCH START 0
	PD1= SWITCH START 1
	PD2= SWITCH START 2
	PD3= SWITCH START 3
	
	PD4= SWITCH STOP 0
	PD5= SWITCH STOP 1
	PD6= SWITCH STOP 2
	PD7= SWITCH STOP 3
	
	*/
	DDRC |= 0x1F;
	DDRB &=~((1<<DDB0)|(1<<DDB3)|(1<<DDB4));
	DDRD =0X00;
	
}
void LeerEntrada(InputPin_t *P,bool PulsoLargo)
{
	//esto solo funciona para un valor ya que si se quiere un pulso largo para cada boton
	// deberia agregar mas valores para que sea independiente
	static uint16_t valorPulso=0u;
	
	if (!(*(P->port)&(P->pin)))
	{
		if (P->contador<=10)
		{
			P->contador++;
		}
		if ((P->contador)>9)
		{
			if(P->ValidarPulso)
			{	
				//En vez de validar el pin validamos el led para generar el pulso de 100ms
				//para luego recien generar el flagpin
				//P->flagPin=true;
				P->flagLed=true;
				//En esta linea se lee el puerto y se niega dicha salida pero se deja de usar
				//para activar la salida del led el tiempo que se quiera dentro el timer 0
				//*(P->portLED)^=P->pinLED; 
				*(P->portLED)|=P->pinLED; 
				P->ValidarPulso=false;	
			}
			//Sumamos un contador extra pero solo para cuando se supere 1 seg =10ms*50
			//y se pida un pulso largo
			if (PulsoLargo)
			{
				if(valorPulso++>50)P->flagPulso=true;
			}
		}
		
	}
	else
	{
		P->contador=0;
		P->ValidarPulso=true;
		if(PulsoLargo)valorPulso=0;
	}
}

void Salida(InputPin_t *P,uint16_t tiempo,bool blink)
{
	// Genera un pulso corto al pulsar cualquier boton
	if (P->flagLed)
	{
		if(P->ledTime++>tiempo)
		{
			P->flagLed=false;
			P->flagPin=true;
			*(P->portLED)&=~(P->pinLED);
			P->ledTime=0;
		}
	}
	else{
		if (!blink) *(P->portLED)&=~(P->pinLED);
		// si se manda un blink parpadea el led 
		else{
			if (P->flagPulso)
			{
				if(P->ledTime++>(tiempo*3))
				{
					*(P->portLED)^=(P->pinLED);
					P->ledTime=0;
				}
			}
			else{
				*(P->portLED)&=~(P->pinLED);
				P->ledTime=0;
			}
		}
	}

}
ISR(TIMER0_COMPA_vect) {

	static uint16_t milisegundos=0;
	static uint16_t tiempoLED=20;
	if(milisegundos++>10)
	{
		Salida(&pinStart,tiempoLED,false);
		Salida(&pinStop,tiempoLED,false);
		Salida(&pinProg,tiempoLED,true);

		
		rc5.flag10ms=true;
		milisegundos=0;
	}
}
ISR(TIMER2_COMPA_vect) {
	// Timer a 889 microsegundos
	static int8_t Posicion=27;  //iniciamos con una posicion

	if (Posicion>=0)
	{
		int8_t DesplazamientoBit=Posicion>>1;
		bool ValorRC5=(rc5.dataRC5>>DesplazamientoBit)&0x01;
		if ((Posicion&1)^ValorRC5)
		{
			activar_PWM_Global();
		}
		else{
			detener_PWM_Global();
		}
		Posicion--;
	}
	else
	{
		detener_PWM_Global();
		detener_Timer2();
		Posicion=27;
		rc5.flagRC5=true;
	}
	
}

