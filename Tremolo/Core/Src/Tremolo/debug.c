/*
 * debug.c
 *
 *  Created on: Oct 13, 2023
 *      Author: jeremiah
 */

#include <main.h>
#include <Tremolo/debug.h>

/**
 * @brief Hacky method of transmitting wavetables
 *
 * Increments through number of tables, sends 1 at a time.
 * Sends start packet first, then starts DMA transfer of table.
 *
 * TODO try to send all tables in one transfer. Could maybe turn tables
 * 		into 2D array, with "end packets" attached to the end of each
 *
 * TODO improve error/bounds checking
 *
 * @param huart
 * @param tables
 * @param table_qty
 * @param table_width
 */
void transmit_wavetables(UART_HandleTypeDef* huart, uint16_t* tables[],
		uint16_t table_qty, uint16_t table_width)
{
	static int index = 0;

	if (huart->gState == HAL_UART_STATE_READY)
	{
		transmit_wavetable(huart, tables[index], table_width, index);
		index = (index + 1) % table_qty;
	}

	return;

}

/*
 * Debug & UART Stuff
 */

void transmit_wavetable(UART_HandleTypeDef* huart, uint16_t wavetable[], uint16_t table_width,
		uint8_t table_index)
{
	uint8_t uart_start[5] = {0x01, 0x0D, 0x0A, table_index, 0x02};

	HAL_StatusTypeDef hal_status;
	hal_status = HAL_UART_Transmit(huart, uart_start, sizeof(uart_start), 1);
	if (hal_status != HAL_OK)
	{
		if (hal_status == HAL_ERROR)
		{
			Error_Handler();
		}
		else
		{
			return;
		}
	}

	hal_status = HAL_UART_Transmit_DMA(huart, (uint8_t*)wavetable, table_width*2);
	if (hal_status != HAL_OK)
	{
		if (hal_status == HAL_ERROR)
		{
			Error_Handler();
		}
		else
		{
			return;
		}
	}
}
