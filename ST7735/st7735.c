/* vim: set ai et ts=4 sw=4: */
#include "../../ST7735/st7735.h"

#define DELAY 0x80
#define SWAP_BYTES(c) (((c) << 8) | ((c) >> 8))

static uint16_t line_buffer[160];
static uint16_t char_buffer[416];
// 160 * 80 * 2 байта = 25600 байт. Для H7 это мелочь (там сотни КБ в этой секции).
uint16_t your_big_buffer[160 * 80];


static volatile bool is_writing_string = false;
static volatile bool st7735_dma_busy = false;

// based on Adafruit ST7735 library for Arduino
static const uint8_t
  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x4F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F ,            //     XEND = 159
    ST7735_INVON, 0 },        //  3: Invert colors
#endif

  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay

static void ST7735_Select() {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}

void ST7735_Unselect() {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_LED_ON() {
    HAL_GPIO_WritePin(ST7735_LED_GPIO_Port, ST7735_LED_Pin, GPIO_PIN_RESET);
}

static void ST7735_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, sizeof(cmd), 0x50);

}

static void ST7735_WriteData(uint8_t* buff, size_t buff_size) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size, 0x50);


}

void ST7735_IT_Callback(SPI_HandleTypeDef *hspi) {
    if (hspi == &ST7735_SPI_PORT) {
        st7735_dma_busy = false;
        // Поднимаем CS только если это не часть длинной строки
        if (!is_writing_string) {
            ST7735_Unselect();
        }
    }
}

// Полезная функция для проверки статуса извне (если нужно)
bool ST7735_IsBusy(void) {
    return st7735_dma_busy;
}

static void ST7735_ExecuteCommandList(const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs) {
            ST7735_WriteData((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    // column address set
    ST7735_WriteCommand(ST7735_CASET);
    uint8_t data[] = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
    ST7735_WriteData(data, sizeof(data));

    // row address set
    ST7735_WriteCommand(ST7735_RASET);
    data[1] = y0 + ST7735_YSTART;
    data[3] = y1 + ST7735_YSTART;
    ST7735_WriteData(data, sizeof(data));

    // write to RAM
    ST7735_WriteCommand(ST7735_RAMWR);
}

void ST7735_Init() {
    ST7735_Select();
    ST7735_LED_ON();
    ST7735_ExecuteCommandList(init_cmds1);
    ST7735_ExecuteCommandList(init_cmds2);
    ST7735_ExecuteCommandList(init_cmds3);
    ST7735_Unselect();
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;

    while(st7735_dma_busy);

    ST7735_Select();

    ST7735_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ST7735_WriteData(data, sizeof(data));

    ST7735_Unselect();
}

static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {

	while(st7735_dma_busy);

    // Проверка на корректность символа (таблица ASCII начинается с 32 - пробел)
    if(ch < 32 || ch > 126) ch = '?';

    uint16_t color_be = (color << 8) | (color >> 8);
    uint16_t bgcolor_be = (bgcolor << 8) | (bgcolor >> 8);

    ST7735_SetAddressWindow(x, y, x + font.width - 1, y + font.height - 1);

    uint32_t pixel_idx = 0;
    for(uint32_t i = 0; i < font.height; i++) {
        // Читаем 16-битное слово строки символа
        uint16_t b = font.data[(ch - 32) * font.height + i];

        for(uint32_t j = 0; j < font.width; j++) {
            // Проверяем биты слева направо
            if((b << j) & 0x8000)  {
                char_buffer[pixel_idx++] = color_be;
            } else {
                char_buffer[pixel_idx++] = bgcolor_be;
            }
        }
    }

    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    // Отправляем количество байт: ширина * высота * 2
    HAL_SPI_Transmit(&ST7735_SPI_PORT, (uint8_t*)char_buffer, font.width * font.height * 2, 100);
}

/*
Simpler (and probably slower) implementation:

static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color) {
    uint32_t i, b, j;

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                ST7735_DrawPixel(x + j, y + i, color);
            }
        }
    }
}
*/

void ST7735_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    ST7735_Select();

    while(*str) {
        // Проверка выхода за границы по горизонтали
        if(x + font.width > ST7735_WIDTH) {
            x = 0;
            y += font.height;
            // Проверка выхода за границы по вертикали
            if(y + font.height > ST7735_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                str++;
                continue;
            }
        }

        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    ST7735_Unselect();
}

// Функция принимает массив указателей на строки (макс 8 штук для Font_7x10)
// Функция выводит "пачку" из 8 строк (каждая до 22 символов для Font_7x10) одним махом
void ST7735_PrintTelemetry_IT(char lines[8][24], FontDef font, uint16_t color, uint16_t bgcolor) {
    if(st7735_dma_busy) while(st7735_dma_busy); // Ждем, если предыдущий кадр еще шлется

    uint16_t color_be = SWAP_BYTES(color);
    uint16_t bgcolor_be = SWAP_BYTES(bgcolor);

    // 1. Очистка буфера (160x80) в памяти RAM
    for(uint32_t i = 0; i < (160 * 80); i++) {
        your_big_buffer[i] = bgcolor_be;
    }

    // 2. Отрисовка всех строк в буфер
    for (uint8_t l = 0; l < 8; l++) {
        uint16_t current_x = 0;
        uint16_t current_y = l * font.height;
        char *ptr = lines[l];

        if (ptr[0] == '\0') continue; // Пропуск пустых строк

        while (*ptr && (current_x + font.width <= 160)) {
            char ch = *ptr++;
            if (ch < 32 || ch > 126) ch = '?';

            for (uint16_t row = 0; row < font.height; row++) {
                uint16_t bits = font.data[(ch - 32) * font.height + row];
                for (uint16_t col = 0; col < font.width; col++) {
                    uint32_t idx = (uint32_t)(current_y + row) * 160 + current_x + col;
                    if ((bits << col) & 0x8000) your_big_buffer[idx] = color_be;
                }
            }
            current_x += font.width;
        }
    }

    // 3. Один блокирующий вызов настройки окна на весь экран
    ST7735_Select();
    ST7735_SetAddressWindow(0, 0, 159, 79);
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

    st7735_dma_busy = true;

    // 4. Запуск фоновой передачи (25600 пикселей * 2 байта = 51200 байт)
    // Это займет ~13мс при SPI 30Мбит/с, и всё это время CPU будет свободен для IMU
    HAL_SPI_Transmit_IT(&ST7735_SPI_PORT, (uint8_t*)your_big_buffer, 160 * 80 * 2);
}


void ST7735_PrintTelemetry_DMA(char lines[8][24], FontDef font, uint16_t color, uint16_t bgcolor) {
       if(st7735_dma_busy) return; // Просто выходим, если занято (не блокируем!)

       uint16_t color_be = SWAP_BYTES(color);
       uint16_t bgcolor_be = SWAP_BYTES(bgcolor);

       // 1. Отрисовка в буфер (CPU делает это за ~0.5мс)
       for(uint32_t i = 0; i < (160 * 80); i++) your_big_buffer[i] = bgcolor_be;

       for (uint8_t l = 0; l < 8; l++) {
           uint16_t current_x = 0;
           uint16_t current_y = l * font.height;
           char *ptr = lines[l];
           while (*ptr && (current_x + font.width <= 160)) {
               char ch = *ptr++;
               if (ch < 32 || ch > 126) ch = '?';
               for (uint16_t row = 0; row < font.height; row++) {
                   uint16_t bits = font.data[(ch - 32) * font.height + row];
                   for (uint16_t col = 0; col < font.width; col++) {
                       uint32_t idx = (uint32_t)(current_y + row) * 160 + current_x + col;
                       if ((bits << col) & 0x8000) your_big_buffer[idx] = color_be;
                   }
               }
               current_x += font.width;
           }
       }

       // 2. Настройка окна (блокирующе, но это всего 11 байт)
       ST7735_Select();
       ST7735_SetAddressWindow(0, 0, 159, 79);
       HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

       st7735_dma_busy = true;
       is_writing_string = false;

       uint32_t size_bytes = 160 * 80 * 2;

       // 4. ЗАПУСК DMA
       // Используем HAL_SPI_Transmit_DMA. На H7 размер (size) это uint32_t в дескрипторе,
       // но функция HAL принимает uint16_t. 51200 влезает.
       if (HAL_SPI_Transmit_DMA(&ST7735_SPI_PORT, (uint8_t*)your_big_buffer, (uint16_t)size_bytes) != HAL_OK) {
           st7735_dma_busy = false;
           ST7735_Unselect();
       }
   }

void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // 1. Clipping (отсечение за границами экрана)
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w) > ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h) > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    // 2. Подготовка цвета (ST7735 ожидает Big-Endian)
    uint16_t color_be = (color << 8) | (color >> 8);

    // 3. Заполнение строкового буфера цветом
    // Используем максимум доступной ширины (160 пикселей)
    uint16_t pixels_to_fill = (w < 160) ? w : 160;
    for(uint16_t i = 0; i < pixels_to_fill; i++) {
        line_buffer[i] = color_be;
    }

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    // Устанавливаем пин DC в режим данных один раз для всей операции
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

    // 4. Построчная отправка данных
    for(uint16_t row = 0; row < h; row++) {
        // Отправляем всю строку за одну транзакцию SPI
        // Размер в байтах: w * 2
        HAL_SPI_Transmit(&ST7735_SPI_PORT, (uint8_t*)line_buffer, w * sizeof(uint16_t), 0x50);
    }

    ST7735_Unselect();
}

void ST7735_FillScreen(uint16_t color) {
    // Теперь FillScreen будет летать, так как использует построчную передачу
    ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_FillRectangle_IT(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    while(st7735_dma_busy); // Используем тот же флаг

    // Clipping...
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w) > ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h) > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    uint32_t total_pixels = w * h;
    uint16_t color_be = (color << 8) | (color >> 8);

    for (uint32_t i = 0; i < total_pixels; i++) {
        your_big_buffer[i] = color_be;
    }

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

    st7735_dma_busy = true;

    // Для IT кэш сбрасывать НЕ ОБЯЗАТЕЛЬНО (процессор сам видит свои данные),
    // но хуже не будет.

    // Запуск через прерывания (IT)
    if (HAL_SPI_Transmit_IT(&ST7735_SPI_PORT, (uint8_t*)your_big_buffer, total_pixels * 2) != HAL_OK) {
        st7735_dma_busy = false;
        ST7735_Unselect();
    }
}

void ST7735_FillScreen_IT(uint16_t color) {
    // Просто вызываем нашу DMA функцию на весь размер экрана
	ST7735_FillRectangle_IT(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}


void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) return;
    if((y + h - 1) >= ST7735_HEIGHT) return;

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
    ST7735_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    ST7735_Unselect();
}

void ST7735_DrawImage_IT(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;

    while(st7735_dma_busy); // Ждем, если экран занят

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

    st7735_dma_busy = true;

    // Запускаем передачу всей картинки через прерывания
    if (HAL_SPI_Transmit_IT(&ST7735_SPI_PORT, (uint8_t*)data, w * h * sizeof(uint16_t)) != HAL_OK) {
        st7735_dma_busy = false;
        ST7735_Unselect();
    }
}


void ST7735_InvertColors(uint8_t invert) {
    ST7735_Select();
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect();
}
