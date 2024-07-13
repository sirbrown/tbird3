/*
 * T-bird3_progs.c
 *
 * Created: 2017.05.02. 15:38:54
 * Author : Berenyi Tibor
 */ 

#define F_CPU 8000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//_____________________
//funkci� v�laszt�

#define SEVSEG
#define LED
// #define GOMB
// #define MATRIX
#define LCD
// #define ADC_SW
// #define RGB_LED
#define FIFO
#define UART

//______________________

//Dekl

//Sevseg...

#ifdef SEVSEG

#define EN			0x80		// SEVSEG enable
#define SEVSEG_NUM 	0x04		// a h�tszegmensesre ki�rhat� sz�mok sz�ma

#define SEVSEG_BLINKING_TIME	(1000UL)

typedef struct _sevseg{
	uint8_t digit[SEVSEG_NUM];
	uint8_t digit_cnt: 7;
	uint8_t dot_en: 1;
}SEVSEGTYPE;

SEVSEGTYPE d;

void SEVSEG_init(void);
void SEVSEG_write(void);
void SEVSEG_PutDigit(uint8_t val, uint8_t n);
void SEVSEG_PutNumber(uint16_t val);
void SEVSEG_PutDigitShift(uint8_t val);
void SEVSEG_PutTime(uint8_t hm, uint8_t ms);

void SEVSEG_DigitEN(uint8_t n);
void SEVSEG_DigitDIS(uint8_t n);
void SEVSEG_DigitToggle(uint8_t n);

#endif
//_____Sevseg

// Led...
#ifdef LED
void led_set(uint8_t);
void led_init(void);
void led_off(uint8_t);
void led_toggle(uint8_t);
void led_kr(void);

#endif
//____Led

// Gomb
#ifdef GOMB
void gomb_init(void);
uint8_t gomb_input(void);
#endif
//____Gomb

// Matrix...
#ifdef MATRIX
const uint8_t bill_tomb[]={0x45, 0xE, 0xD, 0xB, 0x16, 0x15, 0x13, 0x26, 0x25, 0x23, 0x46, 0x43, 0x0);
void matrix_init(void);
uint8_t matrix_input(void);
#endif
// Matrix...

// LCD...
#ifdef LCD
#define LCD_DATA_DDR	(DDRE)
#define LCD_CMD_DDR		(DDRF)

#define LCD_DATA_PORT 	(PORTE)
#define LCD_DATA_PIN 	(PINE)
#define LCD_CMD_PORT	(PORTF)

#define LCD_RS			(PF1)
#define LCD_RW			(PF2)
#define LCD_EN			(PF3)

//https://mil.ufl.edu/3744/docs/lcdmanual/commands.html

#define LCD_COMMAND_4B2L	0b00101000		//4 bites �s 2 soros be�ll�t�s
#define LCD_COMMAND_CL		0b00000001		//t�rli a kijelz�t
#define LCD_COMMAND_CH		0b00000010		// cursor home
#define LCD_COMMAND_DISP_ON	0b00001111		//bekapcsol�s utols� bit: villog� kurzor




void LCD_init(void);
void LCD_data(uint8_t data);
void LCD_cmd(uint8_t cmd);
void LCD_string(unsigned char* p);
void LCD_sendInt(int32_t);
void LCD_xy(uint8_t x, uint8_t y);
void LCD_clearline(uint8_t line);
void LCD_busy(void);

#endif
//____LCD

// ADC...
#ifdef ADC_SW
void ADC_init();
void ADC_measure(uint8_t ch, uint16_t *p);
#endif
//___ADC

// RGB_led...
#ifdef RGB_LED
#define APP_PI                          (3.1415926535897932384626433832795f)
#define APP_AUTO_COLOR_STEP             (APP_PI/ 48.0f)
#define APP_INTENSITY_DEFAULT           (0.4f)

#define MODE_AUTO						(1)
#define MODE_MANUAL						(2)

typedef struct _rgb_t{
	uint8_t red: 1;
	uint8_t green: 1 ;
	uint8_t blue: 1;
	uint8_t mode: 2;
	uint16_t ui16_Timer;
	uint32_t color;
	float fColorWheelPos;
	float fIntensity;

}RGB_t;

RGB_t rgb;

void RGB_init(void);
void RGB_on(void);
void RGB_off(void);
void RGB_rainbow(void);
#endif
//____RGB_led

// FIFO...
#ifdef FIFO

#define FIFOSize				(128)

typedef struct FIFOu8{
	uint8_t *FiFo;
	uint8_t Head,Tail,Number;
	uint8_t MaxNumber;
}_FIFOu8;

// typedef struct FIFOu32{
// uint32_t *FiFo;
// uint32_t Head,Tail,Number;
// uint32_t MaxNumber;
// }_FIFOu32;

uint8_t	FIFO_Pushu8(_FIFOu8 *fifo, uint8_t data);
uint8_t	FIFO_Popu8(_FIFOu8 *fifo,uint8_t *data);
uint8_t	FIFO_Clear(_FIFOu8 *fifo);
void FIFO_Init(_FIFOu8 *fifo, uint8_t *p, uint16_t maxnum);
#endif
//___FIFO

// UART...
#ifdef UART

_FIFOu8 RXFifo;
_FIFOu8 TXFifo;

uint8_t rxfifo[FIFOSize];
uint8_t txfifo[FIFOSize];

#define UART0_DDR		DDRE
#define UART0_PORT		PORTE
#define UART0_TX		PE1
#define UART0_RX		PE0

void UART_init();
void UART_SendByte(unsigned char byte);
unsigned char UART_ReadByte();
void UART_SendString(unsigned char* pui8Buffer);
void UART_FifoStr(unsigned char* pui8Buffer);
void UART_FifoNStr(unsigned char* pui8Buffer, uint8_t n);

void UART_sendInt(int32_t val, uint8_t len);
void UART_sendFloat(float val, uint8_t ilen, uint8_t flen);

void UART_StartTX(void);
#endif
//___UART

//PWM

uint8_t osc_dir = 0;

void setup_pwm();
void oscillate_pwm();

//__PWM

//Timer0
// Define the prescaler and compare match value
#define F_MS 1
#define PRESCALER 1024
#define COMPARE_MATCH (uint8_t) ((F_CPU *  ((float) F_MS / 1000)) / PRESCALER) // interval
//__Timer0

//___Dekl

// Impl..

// SEVSEG
#ifdef SEVSEG

void SEVSEG_init(void){
	// felt�lti a sevsegtype-ot �s a portokat inicializ�lja
	uint8_t tmp = 0;
	while(tmp < SEVSEG_NUM){d.digit[tmp++] = 0;}
	d.digit_cnt = 0x0;
	d.dot_en=0x0;
	DDRA = 0xFF;
	PORTA = 0x0;
}

void SEVSEG_write(void){
	// kiteszi a sevsegtype-ot a kijelz�re
	d.digit_cnt&=0x3;
	uint8_t cnt = d.digit_cnt;
	while (cnt>=0){
	PORTA = ((cnt<<4) | d.digit[cnt]);
	cnt--;
	}

}

void SEVSEG_PutDigit(uint8_t val, uint8_t n){
	//a val �rt�k�t az n-edik digitk�nt let�rolja
	if(n>3) return;
	d.digit[n] = val|EN;
}

void SEVSEG_PutDigitShift(uint8_t val){
	// a val �rt�k�t a 4 digit elej�re teszi
	uint8_t i;
	for(i=3;i>0;i--) d.digit[i]=d.digit[i-1];
	d.digit[0]=val|EN;
}

void SEVSEG_PutNumber(uint16_t val){
	// a val �rt�k�t helyi�rt�k helyesen let�rolja
	d.digit[3] = (val / 1000 % 10)|EN;
	d.digit[2] = (val / 100 % 10)|EN;
	d.digit[1] = (val / 10 % 10)|EN;
	d.digit[0] = (val % 10)|EN;
	while (d.digit[d.digit_cnt++]);
}

void SEVSEG_PutTime (uint8_t hm, uint8_t ms)
{
	// a hm (�ra vagy perc) �s az ms (perc vagy m�sodperc) �rt�k�t a megfelel� helyi�rt�k szerint let�rolja
	d.digit[3] = (hm / 10 % 10)|EN;
	d.digit[2] = (hm % 10)|EN;
	d.digit[1] = (ms / 10 % 10)|EN;
	d.digit[0] = (ms % 10)|EN;
	d.dot_en =1;
}

void SEVSEG_DigitEN(uint8_t n){
	//az n-edik digitet enged�lyezi
	d.digit[n] |= EN;
}

void SEVSEG_DigitDIS(uint8_t n){
	//az n-edik digitet tiltja
	d.digit[n] &= ~EN;
}

void SEVSEG_DigitToggle(uint8_t n){
	//az n-edik digitet kapcsolja
	d.digit[n] ^= EN;
}

#endif

// Led...
#ifdef LED
void led_init(void)
{
	DDRB |= 0xF0;
	DDRD |= 0xF0;
}

void led_set(uint8_t ledbits)
{
	PORTB |= ((ledbits&0x0F)<<4);
	PORTD |= ledbits&0xF0;
}


void led_toggle(uint8_t led) {
	PORTB ^= ((1<<led)<<4);	//eltolja 1-et led-del, azt eltolja 4-gyel, komplemens, ut�na xor
	PORTD ^= ((1<<led)&0xF0);
}

void led_on(uint8_t lednum) {
	PORTB |= ((1<<lednum)<<4);
	PORTD |= ((1<<lednum)&0xF0);
}

void led_off(uint8_t lednum) {
	PORTB &= (~((1<<lednum)<<4));	//eltolja 1-et led-del, azt eltolja 4-gyel, komplemens, ut�na �s
	PORTD &= (~((1<<lednum)&0xF0));
}

void led_kr(void) {
	static uint8_t l1 = 0x01, l2 = 0x80, state = 1;
	if(state) {
		l1 <<= 1;
		l2 >>= 1;
		if(l1 == 0x08) state = 0;
	}
	else {
		l1 >>= 1;
		l2 <<= 1;
		if(l1 == 0x01) state = 1;
	}

	led_set(l1|l2);
}

#endif
//____Led

// Gomb
#ifdef GOMB
void gomb_init(void)
{
	DDRG=0x0;
}

uint8_t gomb_input(void)
{
	// a lenyomott gomb sz�m�t adja vissza	
	
	uint8_t btnum=1, portbuff=0;
	
	portbuff = PING&0x1F;
	if (portbuff==0x0) return 0;		// ha nincs lenyomott gomb, p�
	while (PING&0x1F);					//prellmenets�t�s
	
	while(~(portbuff&0x1)) 
	{
		portbuff=portbuff>>1;	//Ha az LSB nem 1, akkor a PING let�rolt �rt�k�t eltolja jobbra, �s eggyel emeli a gombsz�mot
		btnum++;
	}
	return  btnum;
}

#endif
//____Gomb

// Matrix...
#ifdef MATRIX
void matrix_init(void)
{
	/* A m�trix billenty�zet m�k�d�se:
	PORTC als� 3 bitje a 3 oszlop, a k�vetkez� 4 bit a 4 sor. Az oszlopok alaphelyzetben logikai 1-ben vannak, a lenyomott billenty� eset�n az �rintett oszlop 
	logikai 0-ra v�lt. Soronk�nt kell lek�rdezni, �s n�zni, hogy melyik oszlop van lenyomva (azaz a bit �rt�ke 0). �gy meghat�rozhat� minden egyesz sz�mnak egy bitminta.
	A sorokat �s oszlopokat a billenty�zet bal fels� sark�t�l kell kezdeni.
	*/
	/*deklar�ci�ba ker�ltm hogy glob�lis legyen
	uint8_t bill_tomb[]={0x45,_	// a t�mb elem�nek indexe a le�t�tt sz�mot adja, �gy 0x45 = 1000 101, azaz 4. sor, k�z�ps� oszlop, a sz�m=0
		0xE,_					//0xE= 1 110, azaz 1. sor �s 1. oszlop, �s �gy tov�bb...
		0xD, 0xB, 0x16, 0x15, 0x13, 0x26, 0x25, 0x23, 0x46, 0x43,_
		0x0};					//0x0 a t�mb v�g�t jel�li
	*/
	
	DDRC|=(1<<DDC3)|(1<<DDC4)|(1<<DDC5)|(1<<DDC6);		// C port 3, 4, 5 �s 6 bitje kimenet (sorok), ez�rt logikai 1-be kell �ll�tani a DDRC regisztert
	

}
uint8_t matrix_input(void)
{
	uint8_t num=0, bill=0;
	
	PORTC |= (1<<PC3)|(1<<PC4)|(1<<PC5)|(1<<PC6);	//tesztelem, ha nincs lenyomott billenty�, akkor ne is keresse
	_delay_ms(1);
	if ((PINC&0x7)==0x7) return 0xFF;				//azaz, ha egyik oszlop sincs lenyomva (vagy is a PORTC als� 3 bitje 1, ami =0x7), akkor l�pjen ki 0xFF-fel
		
	
	
	while (bill_tomb[num])
	{
		PORTC |= (bill_tomb[num]&0x78);		//az egyes billenty�knek megfelel� bitmez�k sorokra vonatkoz� bitj�t kiteszem a C portra, �s v�rok 1 ms-ot, majd kiolvasom az eredm�nyt
		_delay_ms(1);
		bill = PINC&0x7F;					//0x7F-fel az als� 7 bitre maszkolok, a 8. bit az RGB led�
		if (bill == bill_tomb[num]){
			while ((PINC&0x7F)==bill);		// prellmenets�t�s
			return num;	//ha az eredm�ny egyenl� 
		}
		num++;
	}
	
	return 0xFF;
}
#endif
// Matrix...

// LCD
#ifdef LCD
void LCD_init(void)
{
	LCD_DATA_DDR |= 0xF0;										//DDRE fels� 4 bitje kimenetre
	LCD_CMD_DDR |= ((1<<LCD_RS)	| (1<<LCD_RW) | (1<<LCD_EN));	//DDRF als� n�gy bitje

// a kijelz� alaphelyezetbe hoz�sa
	LCD_cmd(LCD_COMMAND_4B2L);
	LCD_cmd(LCD_COMMAND_DISP_ON);
	LCD_cmd(LCD_COMMAND_CL);
	LCD_cmd(LCD_COMMAND_CH);

}

void LCD_data(uint8_t data){
	LCD_busy();
	
	LCD_CMD_PORT &= ~(1<<LCD_EN);
		_delay_ms(3);
	LCD_CMD_PORT &= ~(1<<LCD_RW);		//adatot �runk
		_delay_ms(3);
	LCD_CMD_PORT |=  (1<<LCD_RS);		//data register kiv�laszt�sa


	LCD_DATA_PORT &= 0x0f;				//az adatport fels� 4 bitj�t null�zza
	LCD_DATA_PORT |= (data&0xf0);		//adat fels� 4 bit k�ld�se
	_delay_ms(3);
	LCD_CMD_PORT |= (1<<LCD_EN);
	_delay_ms(3);
	LCD_CMD_PORT &= ~(1<<LCD_EN);
	_delay_ms(3);
	LCD_DATA_PORT |= data<<4;
	_delay_ms(3);
	LCD_CMD_PORT |= (1<<LCD_EN);
	_delay_ms(3);
	LCD_CMD_PORT &= ~(1<<LCD_EN);

}

void LCD_cmd(uint8_t cmd)
{
	LCD_busy();
	LCD_CMD_PORT &= ~(1<<LCD_RW);		// RW pin 0-ra, mert �runk
	LCD_CMD_PORT &= ~(1<<LCD_RS);		// RS pin 0-ra, mert parancs
	
	LCD_CMD_PORT &= ~(1<<LCD_EN);		// a parancsot a lefut� �lre veszi, ez�rt null�zzuk EN bitet
	
	LCD_DATA_PORT &= 0xF;				//az als� 4 bitet meghagyjuk, azok nem az lcd-hez tartoznak
	
	LCD_DATA_PORT |= cmd&0xF0;			// a parancs fels� 4 bitj�t k�ldj�k
	LCD_CMD_PORT |= (1<<LCD_EN);
	_delay_ms(3);
	LCD_CMD_PORT &= ~(1<<LCD_EN);
	
	LCD_DATA_PORT |= cmd<<4;			// als� n�gy bit
	_delay_ms(3);
	LCD_CMD_PORT |= (1<<LCD_EN);
	_delay_ms(3);
	LCD_CMD_PORT &= ~(1<<LCD_EN);		
}

void LCD_string(unsigned char* p) {
	while(*p) {
		LCD_data(*p);
		p++;
	}
}

void LCD_sendInt(int32_t val){		
	//32 biten �br�zolhat� eg�szt �r ki (32 biten el�jelesen max 10 jegy� sz�m �br�zolhat� + egy a - jelnek
	char s[11]={0x0};
	uint32_t i = 0, negative = 0;
	if(val < 0){					//ha negat�v
		negative = 1;
		val = -val;
	}
	while(i<11 && val){
		s[i++]=val%10+'0';			//h�tulr�l el�re beteszi a sz�mjegyeket a t�mbbe, a + '0' az ASCII k�dd� alak�t�shoz kell
		val /= 10;
	}
	if(negative) s[i] = '-';			//ha negat�v, a v�g�re biggyeszt egy - jelet
		
	while(i--) LCD_data(s[i]);		//h�tulr�l el�re kik�ldi a t�mbb�l
}

void LCD_xy(uint8_t x, uint8_t y){
	// DD RAM c�m megad�sa
	// x= kurzor poz�ci� a sorban, 0-t�l kezdve
	// y = sorok sz�ma 0-t�l kezdve
	// 0x80 a set address command
	switch(y){
		case 0: y=0x00 + 0x80 + x; break;
		case 1: y=0x40 + 0x80 + x; break;
		case 2: y=0x10 + 0x80 + x; break;
		case 3: y=0x50 + 0x80 + x; break;
		default: break;
	}
	LCD_cmd(y);
}

void LCD_clearline(uint8_t line) {
	uint8_t i = 0;
	LCD_xy(0, line);
	while(i++ < 16) LCD_data(' ');
	LCD_xy(0, line);
}


void LCD_busy(void){
	uint8_t busy;
	LCD_DATA_DDR &= ~(1<<PE7);			//a data direction bemenetre m�dos�t�sa
	LCD_CMD_PORT &= ~(1<<LCD_RS);
	LCD_CMD_PORT |= (1<<LCD_RW);		//RW=1 olvas�s

	do{									// addig p�r�g, am�g a busy flag 1
		busy = 0;
		LCD_CMD_PORT |= (1<<LCD_EN);
		_delay_us(1);
		busy = (LCD_DATA_PIN & (1<<PE7));	//7-es adatbit a busy flag
		LCD_CMD_PORT &= ~(1<<LCD_EN);
		_delay_us(1);

		LCD_CMD_PORT |= (1<<LCD_EN);
		_delay_us(1);
		LCD_CMD_PORT &= ~(1<<LCD_EN);
		_delay_us(1);

	}while(busy);

	LCD_CMD_PORT &= ~(1<<LCD_RW);		//RW=0
	LCD_DATA_DDR |= (1<<PE7);			//DDR = 1, kimenet
}


#endif
//___LCD

//ADC
#ifdef ADC_SW

void ADC_init(){

	ADMUX |= (0<<REFS0);	                            ///ADCref AREF T-bird3 ~3,8-4V capacitive
	ADCSRA |= ((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));	///Prescale for speed (128) from XTAL
	ADCSRA |= (1<<ADFR);                                ///ADC --> free running
	ADCSRA |= (1<<ADEN);                                ///ADC --> enable
}

void ADC_measure(uint8_t ch, uint16_t *p){

	ch = ch&0x07;                       ///channel mask 0-2bit
	ADMUX |= ch;                        ///channel enable on MUX
	ADCSRA |= (1<<ADSC);			    ///START a conversion
	while(!(ADCSRA & (1<<ADIF)));		///WAIT for done
	*p = ADC;                           ///READ 16bit
	ADMUX &= ~ch;                       ///CHANEL --> be free
}

#endif
//___ADC

// RGB_LED
#ifdef RGB_LED

void RGB_init(void)
{
	DDRE |= (1<<PE2) | (1<<PE3);
	DDRC |= (1<<PC7);
}

void RGB_on(void)
{
	if (rgb.mode == MODE_AUTO) return;
	if (rgb.blue) PORTE |= (1<<PE2);
	if (rgb.green) PORTE |= (1<<PE3);
	if (rgb.red) PORTC |= (1<<PC7);
}

void RGB_off(void)
{
	if (rgb.mode == MODE_AUTO) return;
	PORTE &= ~((1<<PE2) | (1<< PE3));
	PORTC &= ~(1<<PC7);
}

void RGB_rainbow(void){
	//    static float fPrevPos[NUM_OF_LEDS];
	static float fPrevPos;
	float fCurPos;
	float fTemp;
	if(rgb.mode == MODE_MANUAL) return;
	//    volatile unsigned long * pulColors;
	rgb.fColorWheelPos += APP_AUTO_COLOR_STEP;
	if(rgb.fColorWheelPos >= (APP_PI * 1.5f))
	{
		rgb.fColorWheelPos = 0.0f;
		// UART_FifoStr("\tzepp");
	}
	// if(rgb.fColorWheelPos <= 0.0f)
	// {
	//     rgb.fColorWheelPos = APP_PI * 1.5f;
	// }
	// pulColors = rgbColors;
	fCurPos = rgb.fColorWheelPos;


	if((fCurPos != fPrevPos)){
		// Preserve the new color wheel position
		fPrevPos = fCurPos;
		// UART_FifoStr("\tyeah");
		// Adjust the BLUE value based on the control state
		fTemp = 255.0f * (float)sin(fCurPos);
		// UART_SendString("\r\nftemp: ");
		// UART_sendInt((int16_t)fTemp, 5);
		if(fTemp < 0){
			rgb.green = 0;
		}
		else{
			rgb.green = (uint8_t) (fTemp*rgb.fIntensity);
			// UART_SendString("\r\ngreen: ");
			// UART_sendInt((uint8_t)(fTemp*rgb.fIntensity), 5);
		}

		// Adjust the RED value based on the control state
		fTemp = 255.0f * (float)sin(fCurPos - APP_PI / 2.0f);
		if(fTemp < 0){
			rgb.blue = 0;
		}
		else{
			rgb.blue = (uint8_t) (fTemp*rgb.fIntensity);
		}

		// Adjust the GREEN value based on the control state
		if(fCurPos < APP_PI){
			fTemp = 255.0f * (float)sin(fCurPos + APP_PI * 0.5f);
		}
		else{
			fTemp = 255.0f * (float)sin(fCurPos + APP_PI);
		}
		if(fTemp < 0){
			rgb.red = 0;
		}
		else{
			rgb.red = (uint8_t) (fTemp*rgb.fIntensity);
		}
	}
	rgb.color =  ((uint32_t)rgb.red<<16)|((uint32_t)rgb.green<<8)|((uint32_t)rgb.blue);

}

#endif
//___RGB_LED

// FIFO
#ifdef FIFO

uint8_t FIFO_Pushu8(_FIFOu8 *fifo, uint8_t data){
	if(fifo->MaxNumber == fifo->Number){
		return 0;
	}
	fifo->FiFo[fifo->Head++] = data;
	if(fifo->Head == fifo->MaxNumber){
		fifo->Head = 0;
	}
	fifo->Number++;
	return 1;
}

uint8_t FIFO_Popu8(_FIFOu8 *fifo,uint8_t *data){
	if(fifo->Number == 0){
		return 0;
	}

	*data = fifo->FiFo[fifo->Tail];
	if(++fifo->Tail == fifo->MaxNumber){
		fifo->Tail = 0;
	}
	fifo->Number--;

	return 1;
}


uint8_t* FIFO_PeekLastu8(_FIFOu8 *fifo){
	if(fifo->Number == 0){
		return 0;
	}
	return &(fifo->FiFo[fifo->Tail]);
}

uint8_t FIFO_Clear(_FIFOu8 *fifo){
	if(fifo != 0){
		fifo->Head = 0;
		fifo->Tail = 0;
		fifo->Number = 0;
		return 1;
	}
	else return 0;
}

void FIFO_Init(_FIFOu8 *fifo, uint8_t *p, uint16_t maxnum){
	fifo->FiFo = p;
	fifo->Head = 0;
	fifo->Tail = 0;
	fifo->Number = 0;
	fifo->MaxNumber = (uint8_t) (maxnum - 1);
}


#endif
//___FIFO

// UART...
#ifdef UART

void UART_init(){
	FIFO_Init(&RXFifo, rxfifo, FIFOSize);
	FIFO_Init(&TXFifo, txfifo, FIFOSize);
	FIFO_Clear(&RXFifo);
	FIFO_Clear(&TXFifo);

	UART0_DDR |= (1<<UART0_TX);
	UART0_DDR &= ~(1<<UART0_RX);

	UBRR0H = 0;
	UBRR0L = 51;    	// 8MHz 9600 bps
	
	UCSR0A |= (1<<RXC0);
//	UCSR0B |= ((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(0<<TXCIE0));		// TX RX enable + Complete Interrupt Enable
	UCSR0B |= ((1<<RXEN0)|(1<<TXEN0));
	UCSR0C |= (0<<USBS0)|(3<<UCSZ00)|(1<<UCSZ01);					// 2 stop bit, 8 bit character

	sei();

}

void UART_SendByte(unsigned char byte){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = byte;
}

unsigned char UART_ReadByte() {
	while ( !(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void UART_SendString(unsigned char* pui8Buffer) {
	while(*pui8Buffer){
		UART_SendByte(*pui8Buffer);
		pui8Buffer++;
	}
}

void UART_FifoStr(unsigned char* pui8Buffer){
	uint8_t ch;
	//cli();
	ch = (UCSR0B&(1<<TXCIE0));
	while(*pui8Buffer){
		// while(!FIFO_Pushu8(&TXFifo, *pui8Buffer));
		FIFO_Pushu8(&TXFifo, *pui8Buffer);

		pui8Buffer++;
	}
	// if(!(UCSR0B&(1<<TXCIE0)))
	if(!ch) UART_StartTX();
	// while(!FIFO_Popu8(&TXFifo, &ch));
	// UCSR0B |=(1<<TXCIE0);
	// while(!(UCSR0A&(1<<UDRE0)));
	//  	UDR0 = ch;
	//sei();

}

void UART_FifoNStr(unsigned char* pui8Buffer, uint8_t n){
	//   uint8_t ch;
	//   cli();
	// UCSR0B &= ~(1<<TXCIE0);
	while(n--) {
		while(!FIFO_Pushu8(&TXFifo, *pui8Buffer));
		pui8Buffer++;
	}
	// if(!(UCSR0B&(1<<TXCIE0)))
	UART_StartTX();
	//   FIFO_Popu8(&TXFifo, &ch);
	// UCSR0B |=(1<<TXCIE0);
	//   while(!(UCSR0A&(1<<UDRE0)));
	//   UDR0 = ch;
	//   sei();
}

void UART_sendInt(int32_t val, uint8_t len){
	char s[16];
	uint8_t ch;
	uint32_t i = 0, negative = 0;
	if(val < 0){
		negative = 1;
		val = -val;
	}
	while(i<15 && i<len){
		s[i++]=val%10+'0';
		val /= 10;
	}
	if(negative){
		if(i>15) i=15;
		s[i++] = '-';
	}
	//  while(i--) MAP_UARTCharPut(UartBase, s[i]);
	//cli();
	ch = UCSR0B&(1<<TXCIE0);
	while(i--) while(!FIFO_Pushu8(&TXFifo, s[i]));
	//   FIFO_Popu8(&TXFifo, &ch);
	// UCSR0B |=(1<<TXCIE0);
	//   while(!(UCSR0A&(1<<UDRE0)));
	//   UDR0 = ch;
	//   sei();
	// if(!(UCSR0B&(1<<TXCIE0)))
	if(!ch) UART_StartTX();
}


void UART_sendFloat(float val, uint8_t ilen, uint8_t flen) {
	// char s[20];
	uint8_t i = 0;
	// int32_t ipart;
	UART_sendInt((int32_t) val, ilen);
	UART_FifoStr((unsigned char*) ".");
	if(val < 0) val = -val;
	for(i=0;i<flen;i++) val *= 10;
	UART_sendInt((int32_t) val, flen);

}

void UART_StartTX(void) {
	uint8_t ch;
	// TIMSK &= ~(1<<OCIE1A);
	if( !(UCSR0B&(1<<TXCIE0)) && TXFifo.Number){
		while(!FIFO_Popu8(&TXFifo, &ch));
		UCSR0B |=(1<<TXCIE0);
		while(!(UCSR0A&(1<<UDRE0)));
		UDR0 = ch;
	}
	// TIMSK |= (1<<OCIE1A);
}



ISR(USART0_RX_vect) {

	while(!FIFO_Pushu8(&RXFifo, UDR0));

}

ISR(USART0_TX_vect){
	uint8_t ch;
	if(TXFifo.Number){
		while(!FIFO_Popu8(&TXFifo, &ch));
		// {
		UDR0 = ch;
		// }
		// else{
		// 	// UCSR0B &= ~(1<<TXCIE0);
		// 	LCD_clearline(2);
		// 	LCD_string((unsigned char*)"txvect fifo err");
		//
		// }
	}
	else{
		//FIFO_Clear(&TXFifo);
		UCSR0B &= ~(1<<TXCIE0);
	}
}
#endif
//___UART



//PWM

void setup_pwm() {
	//PWM be�ll�t�sa Timer/Analog 8. pinre
	
	// Set OC3A pin as output
	DDRE |= (1 << DDE3);  // OC3A is on PE3

	// Set Timer/Counter3 to phase-correct PWM mode with TOP value
	TCCR3A |= (1 << WGM31);  // Phase Correct PWM
	TCCR3B |= (1 << WGM33);  // Phase Correct PWM with ICR3 as TOP

	// Set prescaler to 8
	TCCR3B |= (1 << CS31);

	// Set TOP value for 20 ms period
	ICR3 = 10000;

	// Set Compare Match value for 1 ms duty cycle
	OCR3A = 775;

	// Enable output on OC3A pin
	TCCR3A |= (1 << COM3A1);
}

void oscillate_pwm()
{
	uint16_t reg_temp = OCR3A;
	
	if (osc_dir)
	{
		if ((reg_temp - 1) == 235)// CW
		{
			osc_dir = 0;
		}
		reg_temp--;
	}
	else
	{
		if ((reg_temp + 1) == 1315) //CCW
		{
			osc_dir = 1;
		}
		reg_temp++;
	}
	OCR3A = reg_temp;
}

//__PWM

//Timer0 interrupt

void setup_timer0()
{
	cli();
	//  CTC mode | prescaler = 1024 => 7,8125 kHz => 128�s tick time
	TCCR0 = (1 << WGM01) | (1 << CS02) | (1 << CS01) | (1 << CS00);
	// compare match
	OCR0 = COMPARE_MATCH;
	
	//enable interrupt
	TIMSK = (1 << OCIE0);
	
	sei();
	
}

ISR(TIMER0_COMP_vect)
{
	oscillate_pwm();
}

//_Timer0

int main(void)
{
    //SEVSEG_init();
	//LCD_init();
	led_init();
	//UART_init();
	setup_pwm();
	// setup_timer0();
	
	//uint8_t data =0;
	//_FIFOu8 rxstr;
	
	led_on(1);
	led_on(3);
	led_on(5);
	led_on(7);
	
    while (1) 
    {
		/*
		_delay_ms(3);
 		SEVSEG_PutNumber(12);
 		SEVSEG_write();
		data = UART_ReadByte();
		if (data)
		{
			UART_SendByte(data);
			FIFO_Pushu8(&rxstr, data);
			if (data==0xd)
			{
				UART_SendByte(0x0A);
				while(FIFO_Popu8(&rxstr,&data))
				{
					LCD_data(data);
				}
			}
			data=0;
		}*/
		
    }
}



