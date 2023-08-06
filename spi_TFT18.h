#ifndef SPI_TFT_H
#define SPI_TFT_H

#include "STC32G_GPIO.h"

#define TFT18W        162
#define TFT18H        132
#define u16RED		0xf800
#define u16GREEN	0x07e0
#define u16BLUE	    0x001f
#define u16PURPLE	0xf81f
#define u16YELLOW	0xffe0
#define u16CYAN	    0x07ff 		//蓝绿色
#define u16ORANGE	0xfc08
#define u16BLACK	0x0000
#define u16WHITE	0xffff


#define TFTSPI_GPIO_Port  GPIO_P2     //GPIO_P6
#define TFTSPI_SCL_PIN    GPIO_Pin_5  //GPIO_Pin_4
#define TFTSPI_SDI_PIN    GPIO_Pin_3  //GPIO_Pin_3
#define TFTSPI_RST_PIN    GPIO_Pin_0  //GPIO_Pin_2		//液晶复位引脚定义
#define TFTSPI_DC_PIN     GPIO_Pin_1  //GPIO_Pin_1   	//液晶命令位引脚定义
#define TFTSPI_CS_PIN     GPIO_Pin_2  //GPIO_Pin_0   	//定义SPI_CS引脚

#define TFTSPI_SCK  P25        //P25 = x    //P64 = x
#define TFTSPI_SDI  P23        //P23 = x    //P63 = x
#define TFTSPI_RST  P20        //P20 = x    //P62 = x
#define TFTSPI_DC   P21        //P21 = x    //P61 = x
#define TFTSPI_CS   P22        //P22 = x    //P60 = x

#define TFTSPI_CS_H     TFTSPI_CS = 1      /*!< CS管脚 */
#define TFTSPI_SCK_H    TFTSPI_SCK = 1     /*!< SCL管脚 */
#define TFTSPI_SDI_H    TFTSPI_SDI = 1     /*!< SDI管脚 */
#define TFTSPI_DC_H     TFTSPI_DC = 1      /*!< DC管脚 */
#define TFTSPI_RST_H    TFTSPI_RST = 1     /*!< RST管脚 */

#define TFTSPI_CS_L     TFTSPI_CS = 0     /*!< CS管脚 */
#define TFTSPI_SCK_L    TFTSPI_SCK = 0    /*!< SCL管脚 */
#define TFTSPI_SDI_L    TFTSPI_SDI = 0    /*!< SDI管脚 */
#define TFTSPI_DC_L     TFTSPI_DC = 0     /*!< DC管脚 */
#define TFTSPI_RST_L    TFTSPI_RST = 0    /*!< RST管脚 */

void TFTSPI_Init(unsigned char type);
void TFTSPI_Write_Cmd(unsigned char cmd);
void TFTSPI_Write_Byte(unsigned char dat);
void TFTSPI_Write_Word(unsigned short dat);
void TFTSPI_Set_Pos(unsigned char xs,unsigned char ys,unsigned char xe,unsigned char ye);
void TFTSPI_Fill_Area(unsigned char xs,unsigned char ys,unsigned char xe,unsigned char ye,unsigned short color);
void TFTSPI_CLS(unsigned short color);
void TFTSPI_P8X8Str(unsigned char x, unsigned char y, char *s_dat,unsigned short word_color,unsigned short back_color);


#endif
