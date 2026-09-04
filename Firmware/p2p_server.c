#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;


#include "stm32wbxx_hal_spi.h"
#include "stm32wbxx_hal_gpio.h"
#include "stm32wbxx_hal_adc.h"
#include "stm32wbxx_hal_i2c.h"

#include "MAX30102.h"
#include "LSM6DS3.h"
#include "ads1299.h"



#define CFG_TASK_BLE_CONNECTED_ID 0x01


uint8_t local[120];

uint8_t PPG_sensor_3_bytes [3];
uint8_t PPG_sensor_3_bytes_send_data [3];
extern volatile uint8_t ildaron;



//uint8_t local[5] = {0x00, 0x6F, 0x00, 0x00, 0x00};
uint8_t start_data[2] = {0x00, 0x08};
uint8_t for_test_BLE[2] = {0x00, 0x01};
uint8_t count = 13;
uint8_t test_for_count = 0x00;
uint32_t output[9]= {0};


uint8_t un_temp_1;
uint8_t un_temp_2;
uint8_t un_temp_3;


int8_t PPG_count = 0;

// extern uint8_t ildaron;

int flag = 11;

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */










uint16_t  buf_x;
uint16_t  buf_y;
uint16_t  buf_z;
int8_t OUT_X1_data;
int8_t OUT_X2_data;
uint8_t OUT_Y1_data;
uint8_t OUT_Y2_data;
uint8_t OUT_Z1_data;
uint8_t OUT_Z2_data;
int8_t OUT_X1_data_gyroscope;
int8_t OUT_X2_data_gyroscope;
uint8_t OUT_Y1_data_gyroscope;
uint8_t OUT_Y2_data_gyroscope;
uint8_t OUT_Z1_data_gyroscope;
uint8_t OUT_Z2_data_gyroscope;
uint8_t status_readed;

uint8_t set_speed_acc = 0x60;
uint8_t set_speed_gyr = 0x60;
uint8_t status = 0x1E;
uint8_t test_acceler = 0x00;










static void P2PS_Send_Notification(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */
//	P2PS_Send_Notification();
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */
 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    	UTIL_SEQ_SetTask( 1<<CFG_TASK_BLE_CONNECTED_ID, CFG_SCH_PRIO_0);
   // 	P2PS_Send_Notification();
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */

/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
    //	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;


    case P2PS_STM_WRITE_EVT:
    //	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

    case P2PS_STM_READ_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */

   // 	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

    	 //	P2PS_Send_Notification();
    	  if (pNotification->DataTransfered.pPayload[1] == 0x01)

    	    	{
    		  flag = 0;

    	    	}

    	    	else

    	    	{
    	      flag = 1;
    	    	}


/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
 /* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}



void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
//	P2PS_Send_Notification();
	UTIL_SEQ_RegTask( 1<< CFG_TASK_BLE_CONNECTED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );

/* USER CODE END P2PS_APP_Init */
  return;
}








void write_byte(uint8_t reg_addr, uint8_t val_hex)
{
    uint8_t test    = 0x00;
    uint8_t address = 0x40 | reg_addr;          /* WREG | addr */

    HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &address, 1, 0x1000);
    HAL_SPI_Transmit(&hspi1, &test,    1, 0x1000);   /* n = 0 (write 1 reg) */
    HAL_SPI_Transmit(&hspi1, &val_hex, 1, 0x1000);
    HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_SET);
}

void send_command(uint8_t cmd)
{
    HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 0x1000);
    HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_SET);
}

/* ---- one-time ADS1299 configuration ------------------------------------- */
static void ads1299_init(void)
{
	HAL_I2C_Mem_Write(&hi2c1, adress_write,  reg_set_speed_acc, 1, (uint8_t*)&set_speed_acc, 1, 1000);
	HAL_I2C_Mem_Write(&hi2c1, adress_write,  reg_set_speed_gyr, 1, (uint8_t*)&set_speed_gyr, 1, 1000);


    uint8_t RESETS = 0x06;                       /* RESET command */

    HAL_GPIO_WritePin(GPIOA, SW1_Pin, GPIO_PIN_RESET);

    /* RESET must be clocked in with CS LOW (your old code sent it CS HIGH,
       so the chip never saw it). Then wait for reset to complete. */
    HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &RESETS, 1, 0x1000);
    HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    send_command(SDATAC);                        /* stop RDATAC before WREG */

    write_byte(CONFIG1, 0x96);
    write_byte(CONFIG2, 0xD0);
    write_byte(CONFIG3, 0xEC);                   /* internal reference on   */
    HAL_Delay(10);                               /* let reference settle    */

    write_byte(0x04, 0x00);
    write_byte(0x0D, 0x00);                      /* BIAS_SENSP */
    write_byte(0x0E, 0x00);                      /* BIAS_SENSN */
    write_byte(0x0F, 0x00);                      /* LOFF_SENSP */
    write_byte(0x10, 0x00);                      /* LOFF_SENSN */
    write_byte(0x11, 0x00);                      /* LOFF_FLIP  */
    write_byte(0x14, 0x80);                      /* GPIO       */
    write_byte(0x15, 0x20);                      /* MISC1      */
    write_byte(0x17, 0x00);                      /* CONFIG4    */

    write_byte(CH1SET, 0x00);
    write_byte(CH2SET, 0x00);
    write_byte(CH3SET, 0x00);
    write_byte(CH4SET, 0x00);
    write_byte(CH5SET, 0x00);
    write_byte(CH6SET, 0x00);
    write_byte(CH7SET, 0x00);
    write_byte(CH8SET, 0x01);

    send_command(RDATAC);                        /* continuous read mode    */
    send_command(START);                         /* start conversions       */
}

/* ---- read exactly ONE 27-byte frame when DRDY presents a new sample ------
 *  27 bytes = 3 status bytes + 8 channels x 3 bytes.
 *  The 3 status bytes (i == 0) are skipped, so 24 payload bytes per sample.
 *  Every 120 bytes (5 samples) a notification is pushed.
 * ------------------------------------------------------------------------- */


	// I2C accelerometer




void LSM6DS3()
									  {

										//    HAL_I2C_Mem_Read(&hi2c1, adress_read, LISR,1, (uint8_t*)&test_acceler, 1, 1000);
										//    HAL_I2C_Mem_Read(&hi2c1, adress_read, status, 1, (uint8_t*)&status_readed, 1, 1000);

							//			if (status_readed == data_was_ready)
							//		  {

											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_X1, 1, (int8_t*)&OUT_X1_data, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_X2, 1, (int8_t*)&OUT_X2_data, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Y1, 1, (uint8_t*)&OUT_Y1_data, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Y2, 1, (uint8_t*)&OUT_Y2_data, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Z1, 1, (uint8_t*)&OUT_Z1_data, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Z2, 1, (uint8_t*)&OUT_Z2_data, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_X1_gyroscope, 1, (int8_t*)&OUT_X1_data_gyroscope, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_X2_gyroscope, 1, (int8_t*)&OUT_X2_data_gyroscope, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Y1_gyroscope, 1, (uint8_t*)&OUT_Y1_data_gyroscope, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Y2_gyroscope, 1, (uint8_t*)&OUT_Y2_data_gyroscope, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Z1_gyroscope, 1, (uint8_t*)&OUT_Z1_data_gyroscope, 1, 1);
											HAL_I2C_Mem_Read(&hi2c1, adress_read, OUT_Z2_gyroscope, 1, (uint8_t*)&OUT_Z2_data_gyroscope, 1, 1);

											local[0] = 0; //status_readed;
											local[1] = OUT_X1_data;
											local[2] = OUT_X2_data;
											local[3] = OUT_Y1_data;
											local[4] = OUT_Y2_data;
											local[5] = OUT_Z1_data;
											local[6] = OUT_Z2_data;
											local[7] = OUT_X1_data_gyroscope;
											local[8] = OUT_X2_data_gyroscope;
											local[9] = OUT_Y1_data_gyroscope;
											local[10] = OUT_Y2_data_gyroscope;
											local[11] = OUT_Z1_data_gyroscope;
											local[12] = OUT_Z2_data_gyroscope;

											//P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *) (&local));

								//	}
								  }



static void ads1299_read_frame(void)
{





    static int zad = 0;                          /* persists across calls   */
    uint8_t test = 0x00;
    uint8_t received_Byte;

    /* detect a fresh DRDY high -> low transition */
    if (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == GPIO_PIN_SET)
        zad = 5;

    if (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == GPIO_PIN_RESET && zad == 5)
    {
        zad = 0;
        HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET);

        for (int i = 0; i < 9; i++)
        {
            uint8_t ch[3];
            for (int j = 0; j < 3; j++)
            {
                HAL_SPI_TransmitReceive(&hspi1, &test, &received_Byte, 1, 0x1000);
                ch[j] = received_Byte;
            }

            if (i != 0)                          /* skip the 3 status bytes */
            {
                for (int b = 0; b < 3; b++)
                {
                    local[count++] = ch[b];
                    if (count == 109)

                    {
                        count = 13;
                        LSM6DS3();
                        P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID,
                                                 (uint8_t *)(&local));
                    }
                }
            }
        }

        HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_SET);
    }
}

/* ---- sequencer task: configure once, then read one frame and RETURN ------ */
void P2PS_Send_Notification(void)
{
    static uint8_t initialized = 0;

    if (!initialized)
    {
        ads1299_init();

        initialized = 1;
    }

    ads1299_read_frame();

    /* Re-arm so we keep polling, but the task RETURNS first so
       UTIL_SEQ_Run() can service the BLE stack between reads. */
    UTIL_SEQ_SetTask(1 << CFG_TASK_BLE_CONNECTED_ID, CFG_SCH_PRIO_0);
}
