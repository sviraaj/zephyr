#include "display/Eve2_81x.h"

#define DISPLAY_SPI_FREQUENCY  500000
#define DISPLAY_CS_PIN         13

//boot_screen
void boot_screen(uint8_t DotSize)
{
	Send_CMD(CMD_DLSTART);                   //Start a new display list
	Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));      //Determine the clear screen color
	Send_CMD(CLEAR(1, 1, 1));	             //Clear the screen and the curren display list
	Send_CMD(COLOR_RGB(18, 97, 217));        // change colour to blue
	Send_CMD(POINT_SIZE(DotSize * 16));      // set point size to DotSize pixels. Points = (pixels x 16)
	Send_CMD(BEGIN(POINTS));                 // start drawing point
	//Send_CMD(TAG(1));                        // Tag the blue dot with a touch ID
	Send_CMD(VERTEX2II(DWIDTH / 2, DHEIGHT / 2, 0, 0));     // place blue point
	Send_CMD(END());                         // end drawing point
	Send_CMD(COLOR_RGB(255, 255, 255));      //Change color to white for text
	Cmd_Text(DWIDTH / 2, DHEIGHT / 2, 30, OPT_CENTER, " ZEDBLOX         SACRACA 2.0  "); //Write text in the center of the screen
	Send_CMD(DISPLAY());                     //End the display list
	Send_CMD(CMD_SWAP);                      //Swap commands into RAM
	UpdateFIFO();                            // Trigger the CoProcessor to start processing the FIFO
}

#if defined (TOUCH_RESISTIVE) || defined (TOUCH_CAPACITIVE)
// A calibration screen for the touch digitizer 
void Calibrate(void)
{
	Calibrate_Manual(DWIDTH, DHEIGHT, PIXVOFFSET, PIXHOFFSET);
}
#endif

// A Clear screen function 
void ClearScreen(void)
{
	Send_CMD(CMD_DLSTART);
	Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
	Send_CMD(CLEAR(1, 1, 1));
	Send_CMD(DISPLAY());
	Send_CMD(CMD_SWAP);
	UpdateFIFO();                            // Trigger the CoProcessor to start processing commands out of the FIFO
	Wait4CoProFIFOEmpty();                   // wait here until the coprocessor has read and executed every pending command.		
	k_sleep(10);
}

int display_init(struct device* spi_lcd_dev, struct device* gpio_lcd_dev)
{
    struct spi_config spi_lcd_config;

	if (gpio_lcd_dev == NULL) {
		printk("gpio LCD dev null!");
		return -1;
	}

	if (spi_lcd_dev == NULL) {
		error_log("LCD dev null!");
		return -1;
	}

	spi_lcd_config.frequency = DISPLAY_SPI_FREQUENCY;
	spi_lcd_config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);

	spi_lcd_config.cs = NULL;

    gpio_pin_configure(gpio_lcd_dev, DISPLAY_CS_PIN, GPIO_DIR_OUT);

	FT81x_Init();                          //Initialize the EVE graphics controller. Make sure to define which display 
										   //you are using in the MatrixEveConf.h
	ClearScreen();	                       //Clear any remnants in the RAM

										   //If you are using a touch screen, make sure to define what 
										   //variant you are using in the MatrixEveConf.h file
#if defined (TOUCH_RESISTIVE) || defined (TOUCH_CAPACITIVE)
	Calibrate();
#endif

	boot_screen(30);		  //Draw the ZedBlox boot Screen

#if 0
	uint8_t pressed = 0;

	while (1)
	{
		uint8_t Tag = rd8(REG_TOUCH_TAG + RAM_REG);                    // Check for touches
		switch (Tag)
		{
			case 1:
				if (!pressed)
				{
					boot_screen(120); //Blue dot is 120 when not touched
					pressed = 1;
				}
				break;
			default:
				if (pressed)
				{
					pressed = 0;
					boot_screen(30); //Blue dot size is 30 when not touched
				}
				break;
		}		
	}
#endif

	return 0;
}
