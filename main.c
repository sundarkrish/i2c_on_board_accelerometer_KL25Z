#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include <MKL25Z4.h>
#include "KL25Z_delay.h"
#include "accel.h"




// main function
int main(void)
{




    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    i2c_init();

    if (!init_mma())
    {
    	PRINTF("ACCELOROMETER INITIALIATION FAILURE\n\r");
    	while(1);
    }

    delay(100);
    flag = 1;
    while (1)
    {
    	if (flag == 1)
    		read_full_xyz();




			PRINTF("X-AXIS : %d\t", acc_X);
			PRINTF("Y-AXIS : %d\t", acc_Y);
			PRINTF("Z-AXIS : %d\n\r", acc_Z);

    }

}


