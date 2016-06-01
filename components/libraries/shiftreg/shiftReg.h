#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "boards.h"


//Define LED colors, bitwise
#define blueA	2           // 0b0000 0010
#define greenA	4           // 0b0000 0100
#define redA 	8           // 0b0000 1000
#define whiteA 	14          // 0b0000 1110
#define blueB	32          // 0b0010 0000
#define greenB	64          // 0b0100 0000
#define redB 	128         // 0b1000 0000
#define whiteB 	224         // 0b1110 0000

/**@brief Function for initializating// the shift register .
 * @details 
 */
void shiftRegInit(void);
/**@brief 
 * @details 
 */
void shiftRegPulse(void);
/**@brief 
 * @details 
 */
void shiftRegLatch(void);
/**@brief 
 * @details 
 * @param[in]
 */
void shiftRegWrite(uint8_t reg1, uint8_t reg2, uint8_t reg3);
uint8_t btnCounter(uint8_t btn, uint8_t btncounter);
