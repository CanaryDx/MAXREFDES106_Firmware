/*******************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

#include <stdio.h>
#include <string.h>

#include "rtc.h"
#include "mxc_config.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"

#include "mrd106.h"

#include "board.h"
#include "tmr.h"
#include "trng.h"
#include "tmr_utils.h"

#include "sensorhub.h"
#include "button.h"

#include "max20356.h"
#include "max17260.h"

#include "utils.h"

#include "app_flash.h"
#include "app_led_wrapper.h"
#include "app_led.h"

#include "algohub_api.h"
#include "algohub_config_api.h"
#include "sensorhub_api.h"
#include "algohub_sensorhub_manager.h"

#include "app_fatfs.h"

#include "main.h"
#include "Drivers/GUI/app_gui.h"
#include "Drivers/GUI/gui.h"
#include "Drivers/GUI/app_interface_process.h"
#include "Drivers/GUI/app_interface_ble_process.h"
#include "regHandler_spi.h"
#include "sh_defs.h"
#include "Drivers/MAX20356/max20356_registers.h"
#include "max20356_platform.h"
#include "max20356.h"
#include "windows_gui_packet_defs.h"
#include "button_gesture_handler.h"
#include "locationfinder.h"
#include "ble_api.h"
/**************************************************************************************************
  Macros
**************************************************************************************************/
#define HWREG(x) (*((volatile unsigned long *)(x)))
/* Size of buffer for stdio functions */
#define PRINTF_BUF_SIZE 128

/**************************************************************************************************
  Local Variables
**************************************************************************************************/
static volatile unsigned int g_app_evt = 0;

// Sensor Related Variables
static sensor_t *acc;
static sensor_t *biosensor;
#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
static sensor_t *temp;
#endif

/*! Buffer for stdio functions */
char printf_buffer[PRINTF_BUF_SIZE];

extern bool_t resetFlag;

static queue_t msgQueue;
static uint8_t msgQueueBuf[PACKETSIZE * NUM_EVTS];

static queue_t adapReportQueue;
static uint8_t adapReportQueueBuf[PACKETSIZE * 1024];

static mrd_t *m_mrd = NULL;

uint8_t power_but_evt = 0;
uint8_t loc_finder_enable = 0;

uint8_t ble_connected = 0;

unsigned int time = 0;

/* RTC Update  variables */
uint64_t rtc_time_update_value = 0;

/*SpO2 App settings*/
#define SPO2_MAX_DATA_COLLECTION_TIME_MS 1.5 * 60 * 1000
uint8_t spo2_meas_selected_pd = SPO2_APP_SETTING_USE_PD1;
uint8_t spo2_meas_selected_meas_int = SPO2_APP_SETTING_MEAS_CONTINIOUS;
uint8_t spo2_meas_selected_meas_int_values_min[SPO2_APP_SETTING_MEAS_INTERVAL_LIMIT] = {30 , 60 , 120};

int printf_enable = 0;

extern queue_t queue_algo;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/



int spo2_state_machine(uint8_t restart)
{
	static uint64_t last_calculation_start_time = 0;
	static uint8_t ppg_active_meas_set = 0;
	static uint8_t ppg_active_meas_backup[8] = {0};
	static uint8_t spo2_sm_state = SPO2_SM_NOT_INITIALIZED;

	uint64_t current_time = utils_get_time_ms();
	uint64_t time_interval = spo2_meas_selected_meas_int_values_min[spo2_meas_selected_meas_int] * 60 * 1000; // minute to msc

	if (restart)
	{
		ppg_active_meas_set = 0;
	}

	if (restart || spo2_sm_state == SPO2_SM_NOT_INITIALIZED)
	{
		last_calculation_start_time = current_time;
		spo2_sm_state = SPO2_SM_ACTIVE;
		queue_reset(&queue_algo);

		if (ppg_active_meas_set)
		{
			max86178_enable_channels(ppg_active_meas_backup);
		}

		return 0;
	}

	if (spo2_sm_state == SPO2_SM_WAITING)
	{
		if (get_periodicpacket_status() && !gEcgEn && !gIqEn)
		{
			set_periodicpacket_status(0);
		}

		if (current_time - last_calculation_start_time > time_interval)
		{
			spo2_sm_state = SPO2_SM_NOT_INITIALIZED;
			gPpgEn = 1;
			max86178_enable_channels(ppg_active_meas_backup);
		}

		return 0;
	}

	if (spo2_sm_state == SPO2_SM_ACTIVE)
	{

		if (spo2_meas_selected_meas_int < SPO2_APP_SETTING_MEAS_INTERVAL_LIMIT && // If spo2 measurement is not continuous
			current_time - last_calculation_start_time > SPO2_MAX_DATA_COLLECTION_TIME_MS)
		{ // and if time spent in spo2 measurement reached to the limit

			spo2_sm_state = SPO2_SM_WAITING;
			max86178_get_list_of_enabled_channels(ppg_active_meas_backup);
			ppg_active_meas_set = 1;
			uint8_t ppg_all_disabled[8] = {0};
			gPpgEn = 0;
			max86178_enable_channels(ppg_all_disabled);

			return 0;
		}
		return 1;
	}

	return 0;
}

/*************************************************************************************************/
/*!
 *  \fn     main
 *
 *  \brief  Entry point for demo software.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/

int main(void)
{
	int retVal = 0; // Generic return-code accumulator for helper calls.

#ifndef __IAR_SYSTEMS_ICC__
	setvbuf(stdout, printf_buffer, _IOLBF, PRINTF_BUF_SIZE); // Use line-buffered stdout unless the IAR runtime manages buffering.
#endif

	int index = 0; // Loop iterator reused when draining queues.
	int ret = 0; // Temporary status holder for initialisation routines.
	uint8_t temp_report_buff[PACKETSIZE * NUM_PACKS_PER_CHUNK] = {0}; // Working buffer that batches BLE notification packets.

	m_mrd = getMRD(); // Retrieve the singleton MRD controller that owns subsystem hooks.
	m_mrd->init(); // Initialise BLE, peripherals, timers, ISRs, and sensor objects.
	m_mrd->setStatusLedEnable(1); // Turn on the status LED to indicate boot completion.

	biosensor = m_mrd->getSensor(SH_MAX86178); // Cache the MAX86178 biosensor handle for fast access.
	acc = m_mrd->getSensor(SH_ACC_SENSOR); // Cache the accelerometer handle.

#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
	temp = m_mrd->getSensor(SH_TEMP_SENSOR); // Acquire the temperature sensor handle when temp sensing is enabled.
	temp->init(); // Initialise the temperature sensor driver.
#endif

	ret = queue_init(&adapReportQueue, adapReportQueueBuf, PACKETSIZE, PACKETSIZE * 1024); // Prepare the adapter report queue used for BLE streaming.(the adapter queue is the sent data to gui from device)
	pr_info("adapter report queue init = %d ", ret); // Log the adapter queue initialisation status.

	ret = queue_init(&msgQueue, msgQueueBuf, PACKETSIZE, PACKETSIZE * NUM_EVTS); // Prepare the GUI/message queue that carries host commands.(the messagequeue is the receive data from the gui to device)
	pr_info("msg queue init = %d ", ret); // Log the GUI queue initialisation status.

	adapter_set_report_queue(&adapReportQueue); // Point the adapter layer at the queue it should use.
	gui_set_msg_queue(&msgQueue); // Register the GUI message queue with the GUI subsystem.

	int ppg_len; // Holds the number of pending PPG samples per loop.

	max86178_register_fifo_read_callback(apply_afe_requests); // Apply pending AFE updates each time the FIFO is drained.

	uint64_t time_to_led = utils_get_time_ms(); // Timestamp tracker for LED refresh cadence.
	uint64_t sensorhub_evt_time = utils_get_time_ms(); // Placeholder timestamp for future sensorhub events.

	UNUSED(ret); // Suppress warnings if ret is only used for logging in some builds.
	uint64_t last_loop_time = 0; // Timestamp snapshot of the previous loop iteration.
	uint64_t current_time = 0; // Timestamp snapshot for the current loop iteration.

	while (1) // Main firmware scheduler loop.
	{
		current_time = utils_get_time_ms(); // Capture the loop start time.
		if (current_time - last_loop_time > 15) // Check for loops taking longer than 15 ms.
		{
			//usb_debug_printf("loop_time: %llu\r\n", current_time -last_loop_time); // Optional diagnostic output (disabled to save bandwidth).
		}
		last_loop_time = current_time; // Retain this iteration’s timestamp.

		if (utils_get_time_ms() - time_to_led > 500) // Refresh LED/battery status twice per second.
		{
			time_to_led = utils_get_time_ms(); // Reset the LED-rate limiter.
			uint8_t batt_per = 0; // Storage for the latest battery percentage.
			uint8_t charging = 0; // Storage flag indicating USB power presence.
			max17260_get_battery_percent(&batt_per); // Query the MAX17260 fuel gauge.
			max20356_is_usb_connected(&charging); // Query the PMIC for USB power status.

			m_mrd->setBatteryPercentage(batt_per); // Publish battery information to MRD consumers.
			m_mrd->setChargingStatus(charging); // Publish charging status for LEDs/GUI.

			check_LED_conditions(); // Update LED patterns based on the new power state.
		}

		if (g_app_evt) // Process any pending application events.
		{
			if (g_app_evt & EVT_TEMP_SENSOR_SAMPLE) // Temperature sampling requested?
			{
				g_app_evt &= ~EVT_TEMP_SENSOR_SAMPLE; // Clear the event flag immediately.
#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
				if (m_mrd->getMeasurementEnable() && m_mrd->getTempSensorEnable()) // Ensure measurement and temp sensing are enabled.
				{
					temp->execute_once(NULL); // Execute one temperature reading.
					if (temp->num_items) // Proceed if data was produced.
					{
						uint64_t temp_report_time = utils_get_time_ms(); // Timestamp the reading.
						format_data_temp(temp->report_buf.buf, temp_report_time); // Format and queue the temperature data.
					}
				}
#endif
			}

			if (g_app_evt & EVT_BLE_CONN_OPEN) // BLE connection opened?
			{
				if (!m_mrd->getFlashLogEnable()) // Send GUI notification only if not logging to flash.
				{
					app_gui_connected(); // Inform the GUI of the BLE connection.
				}
				ble_connected = 1; // Remember BLE is connected.
				g_app_evt &= ~EVT_BLE_CONN_OPEN; // Clear the event flag.
			}

			if (g_app_evt & EVT_BLE_CONN_CLOSE) // BLE connection closed?
			{
				if (!m_mrd->getFlashLogEnable()) // Stop measurement only if not flash-logging.
				{
					app_main_evt_post(EVT_MEASUREMENT_STOP); // Request measurement stop.
					app_gui_reset(); // Reset GUI state machines.
					app_gui_disconnected(); // Update GUI to disconnected status.
				}

				gui_queue_reset(); // Clear pending GUI messages.
				app_interface_process_reset(); // Reset BLE interface state.
				ble_connected = 0; // Mark BLE as disconnected.
				if (!m_mrd->getMeasurementEnable()) // Clear event only when streaming halted.
				{
					g_app_evt &= ~EVT_BLE_CONN_CLOSE; // Clear the event flag.
				}
			}

			button_gesture_handler_iterate(); // Update the button gesture state machine.

			if (g_app_evt & EVT_BUTTON_SINGLE_PRESS) // Single press detected?
			{
				if (!loc_finder_enable && !m_mrd->getMeasurementEnable() && !m_mrd->getFlashLogEnable()) // Only when idle.
				{
					app_main_evt_post(EVT_LOC_FIND_STARTED); // Start location finder workflow.
				}
				g_app_evt &= ~EVT_BUTTON_SINGLE_PRESS; // Clear the single-press event.
			}

			if (g_app_evt & EVT_BUTTON_DOUBLE_PRESS) // Double press detected?
			{
				if (!loc_finder_enable && !m_mrd->getMeasurementEnable() && !m_mrd->getFlashLogEnable()) // Only when idle.
				{
					app_main_evt_post(EVT_FLASH_LOGGING_STARTED); // Begin flash logging.
				}

				g_app_evt &= ~EVT_BUTTON_DOUBLE_PRESS; // Clear the double-press event.
			}

			if (g_app_evt & EVT_BUTTON_TRIPLE_PRESS) // Triple press detected?
			{
				static uint8_t led_on_off_state = 1; // Persist status LED state between toggles.
				led_on_off_state = !led_on_off_state; // Toggle LED enable flag.
				m_mrd->setStatusLedEnable(led_on_off_state); // Apply the new LED state.
				check_LED_conditions(); // Update LED behaviour to match.
				g_app_evt &= ~EVT_BUTTON_TRIPLE_PRESS; // Clear the triple-press event.
			}

			if (g_app_evt & EVT_BUTTON_LONG_PRESS) // Long press detected?
			{
				max20356_send_power_off_cmd(); // Request a PMIC power-off.
				g_app_evt &= ~EVT_BUTTON_LONG_PRESS; // Clear the long-press event.
			}

			if (g_app_evt & EVT_FLASH_LOGGING_STARTED) // Flash logging start requested?
			{
				if (!ble_connected) // Only start when BLE is disconnected.
				{
					m_mrd->setFlashLogEnable(1); // Flag flash logging as active.
					m_mrd->enableSensors(1); // Ensure sensors are running to collect data.
				}
				g_app_evt &= ~EVT_FLASH_LOGGING_STARTED; // Clear the logging-start event.
			}

			if (g_app_evt & EVT_FLASH_LOGGING_FINISHED) // Flash logging stop requested?
			{
				m_mrd->setFlashLogEnable(0); // Flag flash logging as inactive.

				uint8_t footer[18]; // Footer buffer used to store stop metadata.
				memset(footer, 0, sizeof(footer)); // Clear footer contents.
				uint64_t wall = gui_get_flash_stop_wall_time(); // Retrieve wall-clock stop time.

				footer[0] = wall >> 24; // Encode stop time into footer bytes.
				footer[1] = wall >> 16;
				footer[2] = wall >> 8;
				footer[3] = wall;
				footer[4] = wall >> 40;
				footer[5] = wall >> 32;

				app_fatfs_write_file(footer, sizeof(footer)); // Append footer to flash log.
				app_fatfs_close_file(); // Close the log file to flush data.
				max20356_send_power_off_cmd(); // Power down after logging completes.

				g_app_evt &= ~EVT_FLASH_LOGGING_FINISHED; // Clear the logging-finished event.
			}

			if (g_app_evt & EVT_MEASUREMENT_START) // Start measurement request?
			{
				m_mrd->setMeasurementEnable(1); // Mark measurement as enabled.
				m_mrd->enableSensors(1); // Turn sensors on.

				g_app_evt &= ~EVT_MEASUREMENT_START; // Clear the start event.
			}

			if (g_app_evt & EVT_MEASUREMENT_STOP) // Stop measurement request?
			{
				m_mrd->setMeasurementEnable(0); // Mark measurement as disabled.
				m_mrd->enableSensors(0); // Turn sensors off.
				adapter_clear_sensor_running(); // Reset adapter tracking state.

				if (m_mrd->getFlashLogEnable()) // If flash logging was active…
				{
					app_main_evt_post(EVT_FLASH_LOGGING_FINISHED); // Schedule logging finalisation.
				}
				g_app_evt &= ~EVT_MEASUREMENT_STOP; // Clear the stop event.
			}

			if (g_app_evt & EVT_USB_WRITE_CB) // USB write callback event?
			{
				unsigned char rxbuff[256] = {0}; // Buffer to receive USB data.
				g_app_evt &= ~EVT_USB_WRITE_CB; // Clear the USB event.

				usb_cdc_read(rxbuff, sizeof(rxbuff)); // Read incoming USB data.
				//USB commands can be processed here // Placeholder for USB command handling.
			}

			if (g_app_evt & EVT_BLE_WRITE_CB) // BLE write callback event?
			{
				g_app_evt &= ~EVT_BLE_WRITE_CB; // Event handled elsewhere, so just clear it.
			}

			if (g_app_evt & EVT_BLE_READ_CB) // BLE read callback event?
			{
				g_app_evt &= ~EVT_BLE_READ_CB; // Event handled elsewhere, so just clear it.
			}

			if (g_app_evt & EVT_TEMP_SENSOR_UPDATE) // Temperature timer update needed?
			{
				g_app_evt &= ~EVT_TEMP_SENSOR_UPDATE; // Clear the event flag.
#if defined(ENABLE_MAX30208) || defined(ENABLE_MAX30210)
				int temp_period = 1000 / temp_sensor_sampling_freq[m_mrd->getTempSensorFreq()]; // Convert sampling frequency to period.
				m_mrd->updateTimerPeriod(TEMP_TIMER, temp_period / 2, TEMP_TIMER_PS); // Update the timer that paces temperature reads.
#endif
			}

			if (g_app_evt & EVT_RTC_UPDATE) // RTC needs updating?
			{
				g_app_evt &= ~EVT_RTC_UPDATE; // Clear the event flag.

				uint32_t sec_update = rtc_time_update_value / 1000; // Convert milliseconds to seconds.
				uint16_t subsec_update = (rtc_time_update_value - (sec_update * 1000)) * 0xFFFF; // Convert leftover milliseconds to subseconds.

				RTC_DisableRTCE(MXC_RTC); // Disable the RTC while updating.
				RTC_Init(MXC_RTC, sec_update, subsec_update, NULL); // Load the new time values.
				RTC_EnableRTCE(MXC_RTC); // Re-enable the RTC counter.
			}

			static app_algo_state_t oldState; // Preserve previous algorithm hub state across location finder mode.
			static uint8_t oldAccelState; // Preserve previous accelerometer usage state.
			if (g_app_evt & EVT_LOC_FIND_STARTED) // Location finder requested?
			{
				loc_finder_enable = 1; // Flag that location finder mode is active.
				oldAccelState = gUseAcc; // Save current accelerometer usage flag.
				oldState = app_gui_get_ah_state(); // Save current algorithm hub state.
				if (oldState == APP_ALGO_STATE_AH_ENABLED) // If algorithm hub is running…
				{
					app_gui_set_ah_state(APP_ALGO_STATE_AH_SH_NOT_ENABLED); // Indicate algorithm hub will be paused.
				}
				max86178_locationfinder_enable(); // Configure MAX86178 for location finder mode.
				m_mrd->enableSensors(1); // Ensure sensors are running for location finding.
				g_app_evt &= ~EVT_LOC_FIND_STARTED; // Clear the start event.
			}

			if (g_app_evt & EVT_LOC_FIND_STOPPED) // Location finder stop requested?
			{
				loc_finder_enable = 0; // Disable location finder mode flag.
				gUseAcc = 1; // Force accelerometer on briefly to flush data.
				m_mrd->enableSensors(0); // Turn sensors off.
				gUseAcc = oldAccelState; // Restore previous accelerometer usage state.
				app_gui_set_ah_state(oldState); // Restore previous algorithm hub state.

				max86178_locationfinder_disable(); // Return MAX86178 to normal operation.
				g_app_evt &= ~EVT_LOC_FIND_STOPPED; // Clear the stop event.
			}
		}

		DISPATCH_BLE(10); // Run the Cordio dispatcher ten iterations to service BLE stack tasks.

		if (APP_INTERFACE_BLE_MRD104GUI == app_interface_ble_selected()) // Only continue if the MRD104 GUI profile is active.
		{
			if (biosensor->initialized) // Ensure biosensor has been initialised.
			{
				biosensor->execute_once(NULL); // Acquire a biosensor sample.
			}

			if (m_mrd->getMeasurementEnable() && !gPpgEn && (gEcgEn || gIqEn) && gUseAcc && (current_time - last_acc_execute_time >= 40)) // Decide if accelerometer should run.
			{
				acc->execute_once(NULL); // Poll the accelerometer sensor.
				adapter_acc_execute_push(acc->queue); // Push accelerometer data into the adapter queue.
				last_acc_execute_time = current_time; // Update accelerometer execution timestamp.
			}

			if (APP_ALGO_STATE_AH_ENABLED == app_gui_get_ah_state() && spo2_state_machine(0)) // If algorithm hub wants processing time…
			{
				algohub_process(current_time); // Run one algorithm hub processing step.
			}

			if (!m_mrd->getMeasurementEnable() || (m_mrd->getMeasurementEnable() && !spo2_state_machine(0))) // When measurement is idle or SpO2 state machine idle…
			{
				ble_sanity_handler(); // Run BLE keepalive logic.
			}

			ppg_len = queue_len(biosensor->queue); // Determine size of the biosensor sample queue.

			if (loc_finder_enable) // Special handling during location finder mode.
			{
				if (ppg_len >= 4) // Wait until sufficient samples have accumulated.
				{
					locationfinder(biosensor->queue); // Run the location finder algorithm.
					DISPATCH_BLE(10); // Allow BLE to transmit location finder results.
					queue_reset(biosensor->queue); // Flush used PPG samples.
				}
			}
			else // Normal measurement flow.
			{
				if (ppg_len) // If there are PPG samples to process…
				{
					for (index = 0; index < ppg_len; index++) // Process each sample.
					{
						adapter_execute_push(biosensor->queue, acc->queue); // Merge biosensor and accelerometer data for downstream reporting.
					}
				}
			}

			adapter_execute_pop(); // Pop formatted packets from the adapter into BLE or logging flows.

			if (m_mrd->getFlashLogEnable()) // If flash logging is active…
			{
				int numPackReportQueue = queue_len(&adapReportQueue); // Count queued BLE packets.
				int ind = 0; // Loop counter for draining packets.
				uint8_t buff[20]; // Scratch buffer for each dequeued packet.

				for (ind = 0; ind < numPackReportQueue; ind++) // Drain queue into flash.
				{
					dequeue(&adapReportQueue, buff); // Remove one packet from the queue.
					app_fatfs_write_file(buff, sizeof(buff)); // Append packet to the flash log.
				}
			}
		}

		/*
		  To minimize drop packets, we are checking how much times based since the last transmission.
		  According to the time diff, we are sending different number of BLE notification packets to solve that problem.
		  This is a temporary solution and it can removed in the future.
		  For now, it seems that it is working fine.
		*/
		if (m_mrd->getBleSentEvt()) // Did the BLE stack signal room for more notifications?
		{
			if (APP_INTERFACE_BLE_MRD104GUI == app_interface_ble_selected()) // Only manage chunks for this interface.
			{
				int numPackReportQueue = 0; // Number of pending packets.

				numPackReportQueue = queue_len(&adapReportQueue); // Count packets waiting to send.

				if (m_mrd->getStopCmd() || (spo2_state_machine(0) == 0 && get_sensorstop_status())) // Should a stop packet be flushed?
				{

					int remainingPack = NUM_PACKS_PER_CHUNK - (numPackReportQueue % NUM_PACKS_PER_CHUNK); // Determine padding needed to align chunk.

					if (remainingPack == NUM_PACKS_PER_CHUNK) // Already aligned case.
					{
						format_stop_packet(); // Enqueue a stop packet.

						for (int ind = 0; ind < (remainingPack - 1); ind++) // Pad remaining slots with don't-care packets.
							format_dont_care_packet();
					}
					else if (remainingPack == 1) // Exactly one slot free.
					{
						format_stop_packet(); // Use slot for stop packet.
					}
					else // Partial chunk needs stop packet plus padding.
					{
						format_stop_packet(); // Enqueue stop packet.
						remainingPack--; // Account for stop packet occupying one slot.

						for (int ind = 0; ind < remainingPack; ind++) // Fill remainder with don't-care packets.
							format_dont_care_packet();
					}

					m_mrd->setStopCmd(0); // Clear the stop command request.
				}

				while (numPackReportQueue >= NUM_PACKS_PER_CHUNK) // While a full chunk is ready to transmit…
				{
					for (int index_ble = 0; index_ble < NUM_PACKS_PER_CHUNK; index_ble++) // Collect chunk packets contiguously.
					{
						dequeue(&adapReportQueue, &temp_report_buff[index_ble * 20]); // Copy a 20-byte packet into the chunk buffer.
					}

					ble_send_notify(temp_report_buff, sizeof(temp_report_buff)); // Transmit the chunk via BLE notifications.
					memset(temp_report_buff, 0, sizeof(temp_report_buff)); // Clear buffer for next use.
					numPackReportQueue = queue_len(&adapReportQueue); // Recalculate queued packet count after sending.
				}
			}

			m_mrd->setBleSentEvt(0); // Clear BLE sent event to await the next trigger.
		}

		if (resetFlag && wsfOsReadyToSleep() /*&& (Console_PrepForSleep() == E_NO_ERROR)*/) // Ready to hand off to bootloader for reset?
		{
			__disable_irq(); // Prevent interrupts from interfering with reset.

			BbDrvDisable(); // Power down BLE baseband hardware.

			NVIC_SystemReset(); // Trigger MCU reset so bootloader can run the next image.
		}
	}
}

/*****************************************************************/
void HardFault_Handler(void)
{
	const uint32_t gcr = 0x40000000;
	const uint32_t gcr_rst0 = 0x00000004;

	const uint32_t gcr_reset = gcr + gcr_rst0;

	HWREG(gcr_reset) = 0x80000000;

	while (HWREG(gcr_reset) == 0x80000000)
		;

	pr_info("\nFaultISR: CFSR %08X, BFAR %08x\n", (unsigned int)SCB->CFSR, (unsigned int)SCB->BFAR);

	// Loop forever
	while (1);
}

void app_main_evt_post(unsigned int evt)
{
	g_app_evt |= evt;
}
