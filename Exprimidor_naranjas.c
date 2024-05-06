// CONFIG1L
#pragma config PLLSEL = PLL4X    // PLL Selection->4x clock multiplier
#pragma config CFGPLLEN = OFF    // PLL Enable Configuration bit->PLL Disabled (firmware controlled)
#pragma config CPUDIV = NOCLKDIV    // CPU System Clock Postscaler->CPU uses system clock (no divide)
#pragma config LS48MHZ = SYS24X4    // Low Speed USB mode with 48 MHz system clock->System clock at 24 MHz, USB clock divider is set to 4

// CONFIG1H
#pragma config FOSC = HSH    // Oscillator Selection->Internal oscillator
#pragma config PCLKEN = ON    // Primary Oscillator Shutdown->Primary oscillator enabled
#pragma config FCMEN = OFF    // Fail-Safe Clock Monitor->Fail-Safe Clock Monitor disabled
#pragma config IESO = OFF    // Internal/External Oscillator Switchover->Oscillator Switchover mode disabled

// CONFIG2L
#pragma config nPWRTEN = ON    // Power-up Timer Enable->Power up timer enabled
#pragma config BOREN = SBORDIS    // Brown-out Reset Enable->BOR enabled in hardware (SBOREN is ignored)
#pragma config BORV = 190    // Brown-out Reset Voltage->BOR set to 2.2V nominal
#pragma config nLPBOR = OFF    // Low-Power Brown-out Reset->Low-Power Brown-out Reset disabled

// CONFIG2H
#pragma config WDTEN = OFF    // Watchdog Timer Enable bits->WDT disabled in hardware (SWDTEN ignored)
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler->1:32768

// CONFIG3H
#pragma config CCP2MX = RB3   // CCP2 MUX bit->CCP2 input/output is multiplexed with RC1
#pragma config PBADEN = OFF   // PORTB A/D Enable bit->PORTB<5:0> pins are configured as digital I/O on Reset
#pragma config T3CMX = RC0    // Timer3 Clock Input MUX bit->T3CKI function is on RC0
#pragma config SDOMX = RC7    // SDO Output MUX bit->SDO function is on RB3
#pragma config MCLRE = ON     // Master Clear Reset Pin Enable->MCLR pin enabled; RE3 input disabled

// CONFIG4L
#pragma config STVREN = ON    // Stack Full/Underflow Reset->Stack full/underflow will cause Reset
#pragma config LVP = OFF      // Single-Supply ICSP Enable bit->Single-Supply ICSP disabled
#pragma config ICPRT = OFF    // Dedicated In-Circuit Debug/Programming Port Enable->ICPORT disabled
#pragma config XINST = OFF    // Extended Instruction Set Enable bit->Instruction set extension and Indexed Addressing mode disabled
#pragma config DEBUG = OFF    // Background Debugger Enable bit->Background debugger disabled, RB6 and RB7 configured as general purpose I/O pins

// CONFIG5L
#pragma config CP0 = OFF    // Block 0 Code Protect->Block 0 is not code-protected
#pragma config CP1 = OFF    // Block 1 Code Protect->Block 1 is not code-protected
#pragma config CP2 = OFF    // Block 2 Code Protect->Block 2 is not code-protected
#pragma config CP3 = OFF    // Block 3 Code Protect->Block 3 is not code-protected

// CONFIG5H
#pragma config CPB = OFF    // Boot Block Code Protect->Boot block is not code-protected
#pragma config CPD = OFF    // Data EEPROM Code Protect->Data EEPROM is not code-protected

// CONFIG6L
#pragma config WRT0 = OFF    // Block 0 Write Protect->Block 0 (0800-1FFFh) is not write-protected
#pragma config WRT1 = OFF    // Block 1 Write Protect->Block 1 (2000-3FFFh) is not write-protected
#pragma config WRT2 = OFF    // Block 2 Write Protect->Block 2 (04000-5FFFh) is not write-protected
#pragma config WRT3 = OFF    // Block 3 Write Protect->Block 3 (06000-7FFFh) is not write-protected

// CONFIG6H
#pragma config WRTC = OFF    // Configuration Registers Write Protect->Configuration registers (300000-3000FFh) are not write-protected
#pragma config WRTB = OFF    // Boot Block Write Protect->Boot block (0000-7FFh) is not write-protected
#pragma config WRTD = OFF    // Data EEPROM Write Protect->Data EEPROM is not write-protected

// CONFIG7L
#pragma config EBTR0 = OFF    // Block 0 Table Read Protect->Block 0 is not protected from table reads executed in other blocks
#pragma config EBTR1 = OFF    // Block 1 Table Read Protect->Block 1 is not protected from table reads executed in other blocks
#pragma config EBTR2 = OFF    // Block 2 Table Read Protect->Block 2 is not protected from table reads executed in other blocks
#pragma config EBTR3 = OFF    // Block 3 Table Read Protect->Block 3 is not protected from table reads executed in other blocks

// CONFIG7H
#pragma config EBTRB = OFF    // Boot Block Table Read Protect->Boot block is not protected from table reads executed in other blocks


#define _XTAL_FREQ 16000000
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <pic18f45k50.h>
#include "map_function.h"
#include "i2c.h"                                // Libreria del protocolo i2c
#include "ssd1306_oled.h"                       // Libreria de la pantalla oled i2c
#include "dht11.h"                      // Libreria del sensor DHT11
#include "hc_sr04.h"                // Libreria del sensor ultrasonico HC-SR04
#include "float_str.h"              // Libreria para convertir float a string

short dht_ok;                           // Flag de verificacion del bit de paridad
float temperatura;                      // Almacena la temperatura
float humedad;                          // Almacena la humedad
char buffer[20];
char valorB;
 
void init_ADC(void);
void init_PWM(void);
uint8_t adc_value;

short selected_pin = 0;

void main(void) {

    // Inicialización de periféricos
    init_ADC();
    init_PWM();
    
    ANSELC=0;        // Apagamos los pines analogicos del puerto C
    ANSELD=0;
    TRISCbits.RC6 = 1;
    TRISDbits.RD0 = 0; 
    LATDbits.LD0 = 0;
        
    I2C_Init_Master(I2C_400KHZ);                // Inicializa el protocolo i2c
    OLED_Init();                                // Inicializa la pantalla oled
    
    OLED_SetFont(FONT_2);
    OLED_Write_Text(30, 0, "Valor ADC");		// Texto inicial
    OLED_Rectangle(6, 20, 121, 63, WHITE);		// Dibuja un rectangulo
    OLED_Update();
    
    HCSR04_Init();                  // Inicializa el sensor ultrasonico HC-SR04
    
    LATCbits.LATC4 = 0;
    TRISCbits.TRISC4 = 1;
 
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.IOCIE = 1;
    INTCONbits.IOCIF = 0;
    INTCON2bits.IOCIP = 1;
        
    while(1)
    {
        //if(selected_pin == 1) //pull down
        //{
        //    LATDbits.LD0 = 1;
        //}
        //else
        //{
        //    LATDbits.LD0 = 0;
        //}
              
        // Leer el valor del potenciómetro (Canal AN0)
        ADCON0bits.CHS = 0;        // Configurar canal AN0
        ADCON0bits.GO = 1;         // Iniciar la conversión
        while (ADCON0bits.GO);     // Esperar hasta que termine la conversión
        adc_value = ADRESH;
        // Ajustar el valor del PWM
        CCP1CONbits.DC1B = 0;
        CCPR1L = 0.388*adc_value ;                      // CCPR1L = 99/255 * adc_value
        
        //sprintf(buffer, "%u ", valor_adc);
        //OLED_SetFont(FONT_2);
        //OLED_Write_Text(25, 26, buffer); 
        
        //dht_ok = DHT11_Read_Data(&temperatura, &humedad);
        
        //if(dht_ok == 1)
        //{
            //sprintf(buffer, "Hum: %0.1f %%", humedad);
            //floattostr(humedad, buffer, 2);
            //OLED_SetFont(FONT_1);
            //OLED_Write_Text(40, 26, buffer); 
            //sprintf(buffer, "Tem: %0.1f C", temperatura);
            //floattostr(temperatura, buffer, 2);
            //OLED_Write_Text(60, 26, buffer);
            //OLED_Update();
        //}
                
        float distancia = HCSR04_Get_Distance();    // Obtiene la distancia
        OLED_SetFont(FONT_1);
        OLED_Write_Text(20, 26, "Sensor HC-SR04"); 
        floattostr(distancia, buffer, 2);           // Convierte la variable float a string
        OLED_Write_Text(10, 40, "Dist: ");
        OLED_Write_Text(35, 40, buffer);
        OLED_Write_Text(60, 40, " cm");
        OLED_Update();
    }
    return;
}
void init_ADC(void) {
    // Configuración del ADC
    ADCON0bits.CHS = 0b0000;     // Canal 0
    ADCON2bits.ADCS = 0b110;     // Tiempo de adquisición del ADC (FRC)
    ADCON2bits.ACQT = 0b111;     // Tiempo de conversión del ADC 20 TAD
    ADCON2bits.ADFM = 0;         // Justificación a la izquierda
    ADCON0bits.ADON = 1;         // Encender el ADC
}

void init_PWM(void) {
    // Configuración del PWM
    TRISCbits.RC2 = 0;           // RC2/P1A como salida
    PR2 = 0x63;        // 99 en decimal           // Valor de periodo para una frecuencia de 10 kHz
    T2CONbits.T2CKPS = 0b01;     // Preescalador 1:4
    T2CONbits.TMR2ON = 1;        // Encender el temporizador T2
    
    CCP1CONbits.CCP1M = 0b1100;  // Modo PWM en P1A
    CCP1CONbits.P1M = 0b00;      // Salida simple
    CCPR1L = 0x00;               // Valor inicial del ciclo de trabajo
}

void __interrupt(high_priority) ISR_high(){
        //Increment the selected pin
    LATDbits.LD0 = 1;

    //If the selected pin reaches 5, wrap it around
    //if(selected_pin == 2) selected_pin = 0;

    //Short delay for debouncing
    __delay_ms(100);
    
    //Clear the interrupt flag
    INTCONbits.IOCIF = 0;

}