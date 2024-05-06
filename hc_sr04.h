/* #include "hc_sr04.h"
*
* Creada por: Ing. Abiezer Hernandez O.
* Fecha de creacion: 23/02/2020
* Electronica y Circuitos
*
*/

#include <xc.h>
#define _XTAL_FREQ 16000000

#define TRIG_DIR TRISBbits.RB3
#define ECHO_DIR TRISBbits.RB4

#define TRIG_PIN LATBbits.LB3
#define ECHO_PIN PORTBbits.RB4

void HCSR04_Init(void);
float HCSR04_Get_Distance(void);