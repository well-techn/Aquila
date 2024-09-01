
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>
#include "VL53L1x.h"
#include "freertos/FreeRTOS.h"
#include "ve_i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "ve_alldef.h"

const uint32_t TimingGuard = 4528;
const uint16_t TargetRate = 0x0A00;
uint16_t fast_osc_frequency;
uint16_t osc_calibrate_val;
static const char *TAG_VL53L1x = "VL53L1x";

esp_err_t VL53L1x_communication_check()
{
  esp_err_t err = ESP_FAIL;

  if ((i2c_read_byte_from_address_x16(I2C_EXT_PORT, VL53L1x_ADDRESS, IDENTIFICATION__MODEL_ID) == 0xEA) && (i2c_read_byte_from_address_x16(I2C_EXT_PORT, VL53L1x_ADDRESS, IDENTIFICATION__MODULE_TYPE) == 0xCC) && (i2c_read_byte_from_address_x16(I2C_EXT_PORT, VL53L1x_ADDRESS, IDENTIFICATION__REVISION_ID) == 0x10))
  {
     ESP_LOGI(TAG_VL53L1x,"VL531L is online");
     err = ESP_OK;
  }
  else ESP_LOGE(TAG_VL53L1x,"VL53L1x is offline\n");

  return err;
}

uint16_t encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

uint32_t decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

int32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint32_t calcMacroPeriod(uint8_t vcsel_period)
{
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

uint32_t getMeasurementTimingBudget()
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = calcMacroPeriod(i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"
  uint32_t temp = i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS,RANGE_CONFIG__TIMEOUT_MACROP_A_HI);
  temp = (temp << 8) | i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS,RANGE_CONFIG__TIMEOUT_MACROP_A_LO);
  uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(temp), macro_period_us);

  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
}

bool setMeasurementTimingBudget(uint32_t budget_us)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS

  if (budget_us <= TimingGuard) { return false; }

  uint32_t range_config_timeout_us = budget_us -= TimingGuard;
  if (range_config_timeout_us > 1100000) { return false; } // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  macro_period_us = calcMacroPeriod(i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing A timeout"
  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = calcMacroPeriod(i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_B));
  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing B timeout"
  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // VL53L1_calc_timeout_register_values() end

  return true;
}

bool setDistanceMode(uint8_t mode)
{
  // save existing timing budget
  uint32_t budget_us = getMeasurementTimingBudget();

  switch (mode)
  {
    case 1:
      // from VL53L1_preset_mode_standard_ranging_short_range()

      // timing config
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

      // dynamic config
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__WOI_SD0, 0x07);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__WOI_SD1, 0x05);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

      break;

    case 2:
      // from VL53L1_preset_mode_standard_ranging()

      // timing config
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

      // dynamic config
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__WOI_SD0, 0x0B);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__WOI_SD1, 0x09);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

      break;

    case 3: // long
      // from VL53L1_preset_mode_standard_ranging_long_range()

      // timing config
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

      // dynamic config
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__WOI_SD0, 0x0F);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__WOI_SD1, 0x0D);
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
      i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

      break;

    default:
      // unrecognized mode - do nothing
      return false;
  }

  setMeasurementTimingBudget(budget_us);

  // save mode so it can be returned by getDistanceMode()
  //distance_mode = mode;

  return true;

}

uint8_t configure_VL53L1x (void) {

  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SOFT_RESET, 0x00);
  vTaskDelay(1/portTICK_PERIOD_MS);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SOFT_RESET, 0x01);
  vTaskDelay(100/portTICK_PERIOD_MS);
  
  fast_osc_frequency = i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS,OSC_MEASURED__FAST_OSC__FREQUENCY_HI);
  fast_osc_frequency =  (fast_osc_frequency <<8) | i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS,OSC_MEASURED__FAST_OSC__FREQUENCY_LO); 

  osc_calibrate_val = i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS,RESULT__OSC_CALIBRATE_VAL_HI);
  osc_calibrate_val = (osc_calibrate_val << 8) | i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS,RESULT__OSC_CALIBRATE_VAL_LO);

  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, 0x0A00); 
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, GPIO__TIO_HV_STATUS, 0x02);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 0x08);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 1);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, ALGO__RANGE_MIN_CLIP, 0);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, ALGO__CONSISTENCY_CHECK__TOLERANCE, 0x02);

  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__THRESH_RATE_HIGH, 0x0000);
  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__THRESH_RATE_LOW, 0x0000);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__SIGMA_THRESH, 360);
  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);

  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SD_CONFIG__QUANTIFIER, 0x02);

  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__SEED_CONFIG, 0x01);

  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__SEQUENCE_CONFIG, 0x8B);
  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, DSS_CONFIG__ROI_MODE_CONTROL, 0x02);

  setDistanceMode(3);//long
  setMeasurementTimingBudget(50000);

  uint8_t temp = i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, MM_CONFIG__OUTER_OFFSET_MM_HI);
  temp = (temp << 8) | i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, MM_CONFIG__OUTER_OFFSET_MM_LO);

  i2c_write_2_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, ALGO__PART_TO_PART_RANGE_OFFSET_MM,temp * 4);

  ESP_LOGI(TAG_VL53L1x,"VL531L is configured\n");
  
  return(1);
}

void VL53L1x_startContinuous(uint32_t period_ms)
{
  // from VL53L1_set_inter_measurement_period_ms()
  i2c_write_4_bytes_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  i2c_write_byte_to_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, SYSTEM__MODE_START, 0x40); // mode_range__timed
}

/*

void stopContinuous()
{
  i2c_write_byte_to_address_x16(VL53L1x_ADDRESS, SYSTEM__MODE_START, 0x80); // mode_range__abort

  // VL53L1_low_power_auto_data_stop_range() begin

  calibrated = false;

  // "restore vhv configs"
  if (saved_vhv_init != 0)
  {
    i2c_write_byte_to_address_x16(VL53L1x_ADDRESS, VHV_CONFIG__INIT, saved_vhv_init);
  }
  if (saved_vhv_timeout != 0)
  {
     i2c_write_byte_to_address_x16(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
  }

  // "remove phasecal override"
  i2c_write_byte_to_address_x16(VL53L1x_ADDRESS, PHASECAL_CONFIG__OVERRIDE, 0x00);

  // VL53L1_low_power_auto_data_stop_range() end
}

*/
uint16_t VL53L1x_readResults(void)
{
 uint16_t result = (i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI) << 8) | i2c_read_byte_from_address_x16(I2C_INT_PORT, VL53L1x_ADDRESS, RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO);
 return result;
}



