/*********************
 *      INCLUDES
 *********************/

#include "lvgl_port_display.h"
#include "main.h"

/**********************
 *     VARIABLES
 **********************/

extern SPI_HandleTypeDef hspi1;
static lv_disp_drv_t disp_drv;
static lv_disp_draw_buf_t disp_buf;
static __attribute__((aligned(32))) lv_color_t buf[240*16];

/**********************
 *  STATIC FUNCTIONS
 **********************/

/*
 * disp_spi_write()
 */
static void
disp_spi_write (uint8_t data)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1, &data, 1, 1000);

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/*
 * disp_spi_write_cmd()
 */
static void
disp_spi_write_cmd (uint8_t cmd)
{
  HAL_GPIO_WritePin(LCD_WRX_GPIO_Port, LCD_WRX_Pin, GPIO_PIN_RESET);

  disp_spi_write(cmd);
}

/*
 * disp_spi_write_data()
 */
static void
disp_spi_write_data (uint8_t data)
{
  HAL_GPIO_WritePin(LCD_WRX_GPIO_Port, LCD_WRX_Pin, GPIO_PIN_SET);

  disp_spi_write(data);
}

/*
 * disp_set_address_window()
 */
static void
disp_set_address_window (uint16_t x0,
                         uint16_t y0,
                         uint16_t x1,
                         uint16_t y1)
{
  uint16_t x_start = x0 + 0, x_end = x1 + 0;
  uint16_t y_start = y0 + 0, y_end = y1 + 0;

  /* column address set */
  disp_spi_write_cmd(0x2a);
  {
    uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
    disp_spi_write_data(data[0]);
    disp_spi_write_data(data[1]);
    disp_spi_write_data(data[2]);
    disp_spi_write_data(data[3]);
  }

  /* row address set */
  disp_spi_write_cmd(0x2b);
  {
    uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
    disp_spi_write_data(data[0]);
    disp_spi_write_data(data[1]);
    disp_spi_write_data(data[2]);
    disp_spi_write_data(data[3]);
  }

  /* write to RAM */
  disp_spi_write_cmd(0x2c);
}

/*
 * disp_flush()
 */
static void
disp_flush (lv_disp_drv_t    *drv,
            const lv_area_t  *area,
            lv_color_t       *color_p)
{
  uint32_t  size;
  uint16_t *buf16;

  size = lv_area_get_width(area) * lv_area_get_height(area);
  buf16 = (uint16_t*)color_p;

  disp_set_address_window(area->x1, area->y1, area->x2, area->y2);

  /* TODO: use DMA! */
  for (uint32_t i = 0; i <size; i++ )
    {
      disp_spi_write_data((*buf16)>>8);
      disp_spi_write_data(*buf16);
      buf16++;
    }

  lv_disp_flush_ready (&disp_drv);

}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void
lvgl_display_init (void)
{
  /* reset display */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  /* display settings */
  disp_spi_write_cmd(0x36);
  disp_spi_write_data(0x00);
  disp_spi_write_cmd(0x3a);
  disp_spi_write_data(0x05);
  disp_spi_write_cmd(0x21);
  disp_spi_write_cmd(0x2a);
  disp_spi_write_data(0x00);
  disp_spi_write_data(0x00);
  disp_spi_write_data(0x00);
  disp_spi_write_data(0xef);
  disp_spi_write_cmd(0x2b);
  disp_spi_write_data(0x00);
  disp_spi_write_data(0x00);
  disp_spi_write_data(0x00);
  disp_spi_write_data(0xef);

  /* frame rate settings */
  disp_spi_write_cmd(0xb2);
  disp_spi_write_data(0x0c);
  disp_spi_write_data(0x0c);
  disp_spi_write_data(0x00);
  disp_spi_write_data(0x33);
  disp_spi_write_data(0x33);
  disp_spi_write_cmd(0xb7);
  disp_spi_write_data(0x35);

  /* power settings */
  disp_spi_write_cmd(0xbb);
  disp_spi_write_data(0x1f);
  disp_spi_write_cmd(0xc0);
  disp_spi_write_data(0x2c);
  disp_spi_write_cmd(0xc2);
  disp_spi_write_data(0x01);
  disp_spi_write_cmd(0xc3);
  disp_spi_write_data(0x12);
  disp_spi_write_cmd(0xc4);
  disp_spi_write_data(0x20);
  disp_spi_write_cmd(0xc6);
  disp_spi_write_data(0x0f);
  disp_spi_write_cmd(0xd0);
  disp_spi_write_data(0xa4);
  disp_spi_write_data(0xa1);

  /* gamma settings */
  disp_spi_write_cmd(0xe0);
  disp_spi_write_data(0xd0);
  disp_spi_write_data(0x08);
  disp_spi_write_data(0x11);
  disp_spi_write_data(0x08);
  disp_spi_write_data(0x0c);
  disp_spi_write_data(0x15);
  disp_spi_write_data(0x39);
  disp_spi_write_data(0x33);
  disp_spi_write_data(0x50);
  disp_spi_write_data(0x36);
  disp_spi_write_data(0x13);
  disp_spi_write_data(0x14);
  disp_spi_write_data(0x29);
  disp_spi_write_data(0x2d);
  disp_spi_write_cmd(0xe1);
  disp_spi_write_data(0xd0);
  disp_spi_write_data(0x08);
  disp_spi_write_data(0x10);
  disp_spi_write_data(0x08);
  disp_spi_write_data(0x06);
  disp_spi_write_data(0x06);
  disp_spi_write_data(0x39);
  disp_spi_write_data(0x44);
  disp_spi_write_data(0x51);
  disp_spi_write_data(0x0b);
  disp_spi_write_data(0x16);
  disp_spi_write_data(0x14);
  disp_spi_write_data(0x2f);
  disp_spi_write_data(0x31);

  /* sleep out */
  disp_spi_write_cmd(0x11);
  HAL_Delay(120);

  /* display on */
  disp_spi_write_cmd(0x29);

  /* display buffer initialization */
  lv_disp_draw_buf_init(&disp_buf, (void*)buf, NULL, 240*16);

  /* register the display in LVGL */
  lv_disp_drv_init(&disp_drv);

  /* set the resolution of the display */
  disp_drv.hor_res = MY_DISP_HOR_RES;
  disp_drv.ver_res = MY_DISP_VER_RES;

  /* set callback for display driver */
  disp_drv.flush_cb = disp_flush;

  /* set a display buffer */
  disp_drv.draw_buf = &disp_buf;

  /* finally register the driver */
  lv_disp_drv_register(&disp_drv);
}
