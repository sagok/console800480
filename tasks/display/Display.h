/*
 * Display.h
 *
 *  Created on: 08.04.2013
 *      Author: sagok
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

/* Includes ------------------------------------------------------------------*/
#include "integer.h"

enum
{
  NRESET = 0,
  BL = 1,
  XL = 2,
  XR = 3,
  YU = 4,
  YD = 5
};

typedef enum
{
  IOTFT_NRESET = 0,
  IOTFT_BL = 1,
  IOTFT_XL = 2,
  IOTFT_XR = 3,
  IOTFT_YU = 4,
  IOTFT_YD = 5

}IOLCD_TypeDef;


/**
  * @brief  LCD status structure definition
  */
#define LED_OK         0x00
#define LED_ERROR      0x01
#define LED_NOPU	   0x02


#define IOTFTn                           6

#define IOTFT_NRESET_PIN                    GPIO_PIN_9
#define IOTFT_NRESET_GPIO_PORT              GPIOG
#define IOTFT_NRESET_GPIO_CLK_ENABLE()		__GPIOG_CLK_ENABLE()
#define IOTFT_NRESET_GPIO_CLK_DISABLE()     __GPIOG_CLK_DISABLE()


#define IOTFT_BL_PIN                    GPIO_PIN_12
#define IOTFT_BL_GPIO_PORT              GPIOD
#define IOTFT_BL_GPIO_CLK_ENABLE()		__GPIOD_CLK_ENABLE()
#define IOTFT_BL_TIM_CLK_ENABLE()       __TIM4_CLK_ENABLE()
#define IOTFT_BL_GPIO_CLK_DISABLE()     __TIM4_CLK_DISABLE()


#define IOTFT_GPIO_PORT              	GPIOF

#define IOTFT_XL_PIN                    GPIO_PIN_5
#define IOTFT_XL_GPIO_PORT              GPIOF
#define IOTFT_XL_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()
#define IOTFT_XL_GPIO_CLK_DISABLE()     __GPIOF_CLK_DISABLE()

#define IOTFT_XR_PIN                    GPIO_PIN_3					//2
#define IOTFT_XR_GPIO_PORT              GPIOF
#define IOTFT_XR_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()
#define IOTFT_XR_GPIO_CLK_DISABLE()     __GPIOF_CLK_DISABLE()


#define IOTFT_YU_PIN                    GPIO_PIN_4
#define IOTFT_YU_GPIO_PORT              GPIOF
#define IOTFT_YU_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()
#define IOTFT_YU_GPIO_CLK_DISABLE()     __GPIOF_CLK_DISABLE()

#define IOTFT_YD_PIN                    GPIO_PIN_2					//3
#define IOTFT_YD_GPIO_PORT              GPIOF
#define IOTFT_YD_GPIO_CLK_ENABLE()      __GPIOF_CLK_ENABLE()
#define IOTFT_YD_GPIO_CLK_DISABLE()     __GPIOF_CLK_DISABLE()

/**
  * @brief  LCD color
  */
#define LCD_COLOR_BLUE          0x001F
#define LCD_COLOR_GREEN         0x07E0
#define LCD_COLOR_RED           0xF800
#define LCD_COLOR_CYAN          0x07FF
#define LCD_COLOR_MAGENTA       0xF81F
#define LCD_COLOR_YELLOW        0xFFE0
#define LCD_COLOR_LIGHTBLUE     0x841F
#define LCD_COLOR_LIGHTGREEN    0x87F0
#define LCD_COLOR_LIGHTRED      0xFC10
#define LCD_COLOR_LIGHTCYAN     0x87FF
#define LCD_COLOR_LIGHTMAGENTA  0xFC1F
#define LCD_COLOR_LIGHTYELLOW   0xFFF0
#define LCD_COLOR_DARKBLUE      0x0010
#define LCD_COLOR_DARKGREEN     0x0400
#define LCD_COLOR_DARKRED       0x8000
#define LCD_COLOR_DARKCYAN      0x0410
#define LCD_COLOR_DARKMAGENTA   0x8010
#define LCD_COLOR_DARKYELLOW    0x8400
#define LCD_COLOR_WHITE         0xFFFF
#define LCD_COLOR_LIGHTGRAY     0xD69A
#define LCD_COLOR_GRAY          0x8410
#define LCD_COLOR_DARKGRAY      0x4208
#define LCD_COLOR_BLACK         0x0000
#define LCD_COLOR_BROWN         0xA145
#define LCD_COLOR_ORANGE        0xFD20


#define IOTFTx_GPIO_CLK_ENABLE(__INDEX__)  (((__INDEX__) == 0) ? IOTFT_NRESET_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 1) ? IOTFT_BL_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 2) ? IOTFT_XL_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 3) ? IOTFT_XR_GPIO_CLK_ENABLE() :\
											((__INDEX__) == 4) ? IOTFT_YU_GPIO_CLK_ENABLE() : IOTFT_YD_GPIO_CLK_ENABLE())

#define IOTFTx_GPIO_CLK_DISABLE(__INDEX__) (((__INDEX__) == 0) ? IOTFT_NRESET_GPIO_CLK_DISABLE() :\
											((__INDEX__) == 1) ? IOTFT_BL_GPIO_CLK_DISABLE() :\
											((__INDEX__) == 2) ? IOTFT_XL_GPIO_CLK_DISABLE() :\
											((__INDEX__) == 3) ? IOTFT_XR_GPIO_CLK_DISABLE() :\
											((__INDEX__) == 4) ? IOTFT_YU_GPIO_CLK_DISABLE() : IOTFT_YD_GPIO_CLK_DISABLE())


// ---------------------------------
// .fnt file structure
// ---------------------------------
typedef struct fnt_struct{

	unsigned char PngFileName[16];
	unsigned char FntName[16];
	unsigned char FntSize[10];

	unsigned int  UNICODENump;			//UNICODE number of your char (codepage varies and may be specified before export). For example, 32 is a 'space'

	unsigned int  Xpos;					//x position of glyph on texture
	unsigned int  Ypos;					//y position of glyph on texture

	unsigned int  Width;				//width of glyph on texture (glyphs are cropped and Width and Orig Width aren't equal)
	unsigned int  Height;				//height of glyph on texture

	unsigned int  Xoffset;				//distance on the x-axis, on which glyph must be shifted
	unsigned int  Yoffset;				//distance on the y-axis, on which glyph must be shifted

	unsigned int  ORigW;				//Orig W - original width of glyph
	unsigned int  ORigH;				//Orig H - original height of glyph

}fnt_struct;


//--------------------------------------------------------------------
// PNG - Portable Network Graphics Format
// http://netghost.narod.ru/gff2/graphics/summary/png.htm
//--------------------------------------------------------------------
/* Flag values for the unknown chunk location byte. */
#define PNG_HAVE_IHDR  0x01
#define PNG_HAVE_PLTE  0x02
#define PNG_HAVE_IDAT  0x04
#define PNG_AFTER_IDAT 0x08
#define PNG_HAVE_IEND  0x10

/* These describe the color_type field in png_info. */
/* color type masks */
#define PNG_COLOR_MASK_PALETTE    1
#define PNG_COLOR_MASK_COLOR      2
#define PNG_COLOR_MASK_ALPHA      4

/* color types.  Note that not all combinations are legal */
#define PNG_COLOR_TYPE_GRAY 0
#define PNG_COLOR_TYPE_PALETTE  (PNG_COLOR_MASK_COLOR | PNG_COLOR_MASK_PALETTE)
#define PNG_COLOR_TYPE_RGB        (PNG_COLOR_MASK_COLOR)
#define PNG_COLOR_TYPE_RGB_ALPHA  (PNG_COLOR_MASK_COLOR | PNG_COLOR_MASK_ALPHA)
#define PNG_COLOR_TYPE_GRAY_ALPHA (PNG_COLOR_MASK_ALPHA)
/* aliases */
#define PNG_COLOR_TYPE_RGBA  PNG_COLOR_TYPE_RGB_ALPHA
#define PNG_COLOR_TYPE_GA  PNG_COLOR_TYPE_GRAY_ALPHA


#define png_IDAT 0x49444154		//( 73,  68,  65,  84)
#define png_IEND 0x49454E44		//( 73,  69,  78,  68)
#define png_IHDR 0x49484452		//( 73,  72,  68,  82)
#define png_PLTE 0x504C5445		//( 80,  76,  84,  69)
#define png_bKGD uint32_t( 98,  75,  71,  68)
#define png_cHRM uint32_t( 99,  72,  82,  77)
#define png_dSIG uint32_t(100,  83,  73,  71) /* separate spec */
#define png_fRAc uint32_t(102,  82,  65,  99) /* registered, not defined */
#define png_gAMA uint32_t(103,  65,  77,  65)
#define png_gIFg uint32_t(103,  73,  70, 103)
#define png_gIFt uint32_t(103,  73,  70, 116) /* deprecated */
#define png_gIFx uint32_t(103,  73,  70, 120)
#define png_hIST uint32_t(104,  73,  83,  84)
#define png_iCCP uint32_t(105,  67,  67,  80)
#define png_iTXt uint32_t(105,  84,  88, 116)
#define png_oFFs uint32_t(111,  70,  70, 115)
#define png_pCAL uint32_t(112,  67,  65,  76)
#define png_pHYs 0x70485973		//(112,  72,  89, 115)
#define png_sBIT uint32_t(115,  66,  73,  84)
#define png_sCAL uint32_t(115,  67,  65,  76)
#define png_sPLT uint32_t(115,  80,  76,  84)
#define png_sRGB uint32_t(115,  82,  71,  66)
#define png_sTER uint32_t(115,  84,  69,  82)
#define png_tEXt uint32_t(116,  69,  88, 116)
#define png_tIME uint32_t(116,  73,  77,  69)
#define png_tRNS uint32_t(116,  82,  78,  83)
#define png_zTXt uint32_t(122,  84,  88, 116)

typedef struct _PngSignature
{
    BYTE Signature[8];  /* Идентификатор (всегда 89504E470D0A1A0Ah) */
} PNGSIGNATURE;

typedef struct _HDRChunk
{
    DWORD Length;
    DWORD Name;
} HDRCHUNK;

/*
typedef struct _PngChunk
{
    DWORD DataLength;   // Размер поля данных в байтах
    DWORD Type;         // Код, идентифицирующий тип блока
    BYTE  Data[];       // Собственно данные, хранящиеся в блоке
    DWORD Crc;          // CRC-32 значение полей Type и Data
} PNGCHUNK;
*/
typedef struct _IHDRChunk
{
    DWORD Width;        /* Ширина изображения в пикселях */
    DWORD Height;       /* Высота изображения в пикселях */
    BYTE BitDepth;      // Количество битов на пиксель и образец :может принимать значения 1, 2, 4 и 8
    BYTE ColorType;     // Индикатор интерпретации цвета: 0 (чёрно-белое), 2 (полноцветное), 3 (индексированное изображение), 4 (чёрно-белое с альфа данными) и 6 (полноцветное с альфа данными).
    BYTE Compression;   /* Индикатор типа сжатия */
    BYTE Filter;        /* Индикатор типа фильтра */
    BYTE Interlace;     /* Тип использованной схемы чересстрочной развёртки */
} IHDRCHUNK;

typedef struct _PLTEChunkEntry
{
    BYTE Red;           /* Красный компонент (0 = чёрный, 255 = максимум оттенка) */
    BYTE Green;         /* Зелёный компонент (0 = чёрный, 255 = максимум оттенка) */
    BYTE Blue;          /* Синий компонент   (0 = чёрный, 255 = максимум оттенка) */
} PLTECHUNKENTRY;

//Блок Фактического Размера в Пикселях определяет разрешение, предназначенное для отображения изображения.
typedef struct _pHYsChunkEntry
{
   DWORD PixelsPerUnitX;    /* Пикселей на единицу измерения, ось X */
   DWORD PixelsPerUnitY;    /* Пикселей на единицу измерения, ось Y */
   BYTE  UnitSpecifier;     /* 0 = неизвестная, 1 = метрическая единица измерения */
} PHYSCHUNKENTRY;
//--------------------------------------------------------------------

void vTaskDisplay(void *pvParameters ) __attribute__((naked));

void IOTFT_TS_Init(IOLCD_TypeDef IOLCDPin,uint32_t Mode);
void IOLCD_High(IOLCD_TypeDef IOLCDPin);
void IOLCD_Low(IOLCD_TypeDef IOLCDPin);

#endif /* DISPLAY_H_ */
