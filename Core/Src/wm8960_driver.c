#include "wm8960_driver.h"
#include "wm8960_driver_internals.h"

I2C_HandleTypeDef* i2c_handler;

bool wm8960__setup_i2c(I2C_HandleTypeDef* hi2c)
{
	i2c_handler = hi2c;
	return true;
}

bool _writeRegister(int address, int data)
{
	uint8_t message[2] = {0};
	message[0] = address;
	message[1] = data;
	HAL_I2C_Master_Transmit(i2c_handler, DEVICE_ADDRESS, message, 2, DEFAULT_I2C_TIMEOUT); //Sending in Blocking mode
	return true;
}


bool _writeRegisterBit(uint8_t registerAddress, uint8_t bitNumber, bool bitValue)
{
    // Get the local copy of the register
    uint16_t regvalue = _registerLocalCopy[registerAddress];

    if(bitValue == 1)
    {
      regvalue |= (1<<bitNumber); // Set only the bit we want
    }
    else {
      regvalue &= ~(1<<bitNumber); // Clear only the bit we want
    }

    // Write modified value to device
    // If successful, update local copy
    if (_writeRegister(registerAddress, regvalue))
    {
        _registerLocalCopy[registerAddress] = regvalue;
        return true;
    }
  return false;
}


bool _writeRegisterMultiBits(uint8_t registerAddress, uint8_t settingMsbNum, uint8_t settingLsbNum, uint8_t setting)
{
  uint8_t numOfBits = (settingMsbNum - settingLsbNum) + 1;

  // Get the local copy of the register
  uint16_t regvalue = _registerLocalCopy[registerAddress];

  for(int i = 0 ; i < numOfBits ; i++)
  {
      regvalue &= ~(1 << (settingLsbNum + i)); // Clear bits we care about
  }

  // Shift and set the bits from in incoming desired setting value
  regvalue |= (setting << settingLsbNum);

  // Write modified value to device
  // If successful, update local copy
  if (_writeRegister(registerAddress, regvalue))
  {
      _registerLocalCopy[registerAddress] = regvalue;
      return true;
  }
  return false;
}


// enableVREF
// Necessary for all other functions of the CODEC
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
bool wm8960__enableVREF()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 1);
}

// disableVREF
// Use this to save power
// VREF is a single bit we can flip in Register 25 (19h), WM8960_REG_PWR_MGMT_1
// VREF is bit 6, 0 = power down, 1 = power up
// Returns 1 if successful, 0 if something failed (I2C error)
bool wm8960__disableVREF()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 6, 0);
}

// reset
// Use this to reset all registers to their default state
// Note, this can also be done by cycling power to the device
// Returns 1 if successful, 0 if something failed (I2C error)
bool wm8960__reset()
{
  // Doesn't matter which bit we flip, writing anything will cause the reset
  if (_writeRegisterBit(WM8960_REG_RESET, 7, 1))
  {
    // Update our local copy of the registers to reflect the reset
    for(int i = 0 ; i < 56 ; i++)
    {
      _registerLocalCopy[i] = _registerDefaults[i];
    }
    return true;
  }
  return false;
}

bool wm8960__enableAINL()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 1);
}

bool wm8960__disableAINL()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 5, 0);
}

bool wm8960__enableAINR()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 1);
}

bool wm8960__disableAINR()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 4, 0);
}

bool wm8960__enableLMIC()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
}

bool wm8960__disableLMIC()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
}

bool wm8960__enableRMIC()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
}

bool wm8960__disableRMIC()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0);
}

bool wm8960__enableLMICBOOST()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 1);
}

bool wm8960__disableLMICBOOST()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 5, 0);
}

bool wm8960__enableRMICBOOST()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 1);
}

bool wm8960__disableRMICBOOST()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 4, 0);
}

// PGA input signal select
// Each PGA (left and right) has a switch on its non-inverting input.
// On PGA_LEFT:
// 	*You can select between VMID, LINPUT2 or LINPUT3
// 	*Note, the inverting input of PGA_LEFT is perminantly connected to LINPUT1
// On PGA_RIGHT:
//	*You can select between VMIN, RINPUT2 or RINPUT3
// 	*Note, the inverting input of PGA_RIGHT is perminantly connected to RINPUT1

// 3 options: WM8960_PGAL_LINPUT2, WM8960_PGAL_LINPUT3, WM8960_PGAL_VMID
bool wm8960__pgaLeftNonInvSignalSelect(uint8_t signal)
{
  // Clear LMP2 and LMP3
  // Necessary because the previous setting could have either set,
  // And we don't want to confuse the codec.
  // Only 1 input can be selected.

  // LMP3
  bool result1 = _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 0);

  // LMP2
  bool result2 = _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 0);
  bool result3 = false;

  if(signal == WM8960_PGAL_LINPUT2)
  {
    // LMP2
    result3 = _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 6, 1);
  }
  else if(signal == WM8960_PGAL_LINPUT3)
  {
    // LMP3
    result3 = _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 7, 1);
  }
  else if(signal == WM8960_PGAL_VMID)
  {
    // Don't set any bits. When both LMP2 and LMP3 are cleared, then the signal
    // is set to VMID
  }
  return (result1 && result2 && result3);
}

 // 3 options: WM8960_PGAR_RINPUT2, WM8960_PGAR_RINPUT3, WM8960_PGAR_VMID
bool wm8960__pgaRightNonInvSignalSelect(uint8_t signal)
{
  // Clear RMP2 and RMP3
  // Necessary because the previous setting could have either set,
  // And we don't want to confuse the codec.
  // Only 1 input can be selected.

  // RMP3
  bool result1 = _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 0);

  // RMP2
  bool result2 = _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 0);
  bool result3 = false;

  if(signal == WM8960_PGAR_RINPUT2)
  {
    // RMP2
    result3 = _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 6, 1);
  }
  else if(signal == WM8960_PGAR_RINPUT3)
  {
    // RMP3
    result3 = _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 7, 1);
  }
  else if(signal == WM8960_PGAR_VMID)
  {
    // Don't set any bits. When both RMP2 and RMP3 are cleared, then the signal
    // is set to VMID
  }
  return (result1 && result2 && result3);
}

// Connection from each INPUT1 to the inverting input of its PGA
bool wm8960__connectLMN1()
{
  return _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 1);
}

// Disconnect LINPUT1 to inverting input of Left Input PGA
bool wm8960__disconnectLMN1()
{
  return _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 8, 0);
}

// Connect RINPUT1 from inverting input of Right Input PGA
bool wm8960__connectRMN1()
{
  return _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 1);
}

// Disconnect RINPUT1 to inverting input of Right Input PGA
bool wm8960__disconnectRMN1()
{
  return _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 8, 0);
}

// Connections from output of PGAs to downstream "boost mixers".

// Connect Left Input PGA to Left Input Boost mixer
bool wm8960__connectLMIC2B()
{
  return _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 1);
}

// Disconnect Left Input PGA to Left Input Boost mixer
bool wm8960__disconnectLMIC2B()
{
  return _writeRegisterBit(WM8960_REG_ADCL_SIGNAL_PATH, 3, 0);
}

// Connect Right Input PGA to Right Input Boost mixer
bool wm8960__connectRMIC2B()
{
  return _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 1);
}

// Disconnect Right Input PGA to Right Input Boost mixer
bool wm8960__disconnectRMIC2B()
{
  return _writeRegisterBit(WM8960_REG_ADCR_SIGNAL_PATH, 3, 0);
}

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
bool wm8960__setLINVOL(uint8_t volume)
{
  if(volume > 63) volume = 63; // Limit incoming values max
  bool result1 = _writeRegisterMultiBits(WM8960_REG_LEFT_INPUT_VOLUME,5,0,volume);
  bool result2 = wm8960__pgaLeftIPVUSet();
  return (result1 && result2);
}

// setLINVOLDB
// Sets the volume of the PGA input buffer amp to a specified dB value
// passed in as a float argument.
// Valid dB settings are -17.25 up to +30.00
// -17.25 = -17.25dB (MIN)
// ... 0.75dB steps ...
// 30.00 = +30.00dB  (MAX)
bool wm8960__setLINVOLDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setLINVOL()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

  return wm8960__setLINVOL(volume);
}

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
bool wm8960__setRINVOL(uint8_t volume)
{
  if(volume > 63) volume = 63; // Limit incoming values max
  bool result1 = _writeRegisterMultiBits(WM8960_REG_RIGHT_INPUT_VOLUME,5,0,volume);
  bool result2 = wm8960__pgaRightIPVUSet();
  return (result1 && result2);
}

// setRINVOLDB
// Sets the volume of the PGA input buffer amp to a specified dB value
// passed in as a float argument.
// Valid dB settings are -17.25 up to +30.00
// -17.25 = -17.25dB (MIN)
// ... 0.75dB steps ...
// 30.00 = +30.00dB  (MAX)
bool wm8960__setRINVOLDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setRINVOL()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE, WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

  return wm8960__setRINVOL(volume);
}

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right PGAs
bool wm8960__enablePgaZeroCross()
{
  if (_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 1) == 0) return false;
  return _writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 1);
}

bool wm8960__disablePgaZeroCross()
{
  if (_writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 6, 0) == 0) return false;
  return _writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 6, 0);
}

bool wm8960__enableLINMUTE()
{
  return _writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 1);
}

bool wm8960__disableLINMUTE()
{
  _writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 7, 0);
  return _writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
}

bool wm8960__enableRINMUTE()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 1);
}

bool wm8960__disableRINMUTE()
{
  _writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 7, 0);
  return _writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
}

// Causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
bool wm8960__pgaLeftIPVUSet()
{
  return _writeRegisterBit(WM8960_REG_LEFT_INPUT_VOLUME, 8, 1);
}

 // Causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
bool wm8960__pgaRightIPVUSet()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_INPUT_VOLUME, 8, 1);
}


// Input Boosts

// 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
bool wm8960__setLMICBOOST(uint8_t boost_gain)
{
  if(boost_gain > 3) boost_gain = 3; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ADCL_SIGNAL_PATH,5,4,boost_gain);
}

// 0-3, 0 = +0dB, 1 = +13dB, 2 = +20dB, 3 = +29dB
bool wm8960__setRMICBOOST(uint8_t boost_gain)
{
  if(boost_gain > 3) boost_gain = 3; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ADCR_SIGNAL_PATH,5,4,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
bool wm8960__setLIN3BOOST(uint8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,6,4,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
bool wm8960__setLIN2BOOST(uint8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_1,3,1,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
bool wm8960__setRIN3BOOST(uint8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2,6,4,boost_gain);
}

// 0-7, 0 = Mute, 1 = -12dB ... 3dB steps ... 7 = +6dB
bool wm8960__setRIN2BOOST(uint8_t boost_gain)
{
  if(boost_gain > 7) boost_gain = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_INPUT_BOOST_MIXER_2,3,1,boost_gain);
}

// Mic Bias control
bool wm8960__enableMicBias()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 1);
}

bool wm8960__disableMicBias()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 1, 0);
}

// WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD)
// or WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
bool wm8960__setMicBiasVoltage(bool voltage)
{
  return _writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_4, 0, voltage);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// ADC
/////////////////////////////////////////////////////////

bool wm8960__enableAdcLeft()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 1);
}

bool wm8960__disableAdcLeft()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 3, 0);
}

bool wm8960__enableAdcRight()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 1);
}

bool wm8960__disableAdcRight()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_1, 2, 0);
}

// ADC digital volume
// Note, also needs to handle control of the ADCVU bits (volume update).
// Valid inputs are 0-255
// 0 = mute
// 1 = -97dB
// ... 0.5dB steps up to
// 195 = +0dB
// 255 = +30dB

bool wm8960__setAdcLeftDigitalVolume(uint8_t volume)
{
  bool result1 = _writeRegisterMultiBits(WM8960_REG_LEFT_ADC_VOLUME,7,0,volume);
  bool result2 = wm8960__adcLeftADCVUSet();
  return (result1 && result2);
}
bool wm8960__setAdcRightDigitalVolume(uint8_t volume)
{
  bool result1 = _writeRegisterMultiBits(WM8960_REG_RIGHT_ADC_VOLUME,7,0,volume);
  bool result2 = wm8960__adcRightADCVUSet();
  return (result1 && result2);
}

// Causes left and right input adc digital volumes to be updated
bool wm8960__adcLeftADCVUSet()
{
  return _writeRegisterBit(WM8960_REG_LEFT_ADC_VOLUME, 8, 1);
}

// Causes left and right input adc digital volumes to be updated
bool wm8960__adcRightADCVUSet()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_ADC_VOLUME, 8, 1);
}

// ADC digital volume DB
// Sets the volume of the ADC to a specified dB value passed in as a float
// argument.
// Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
// -97.50 (or lower) = MUTE
// -97.00 = -97.00dB (MIN)
// ... 0.5dB steps ...
// 30.00 = +30.00dB  (MAX)

bool wm8960__setAdcLeftDigitalVolumeDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setAdcLeftDigitalVolume()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX);

  return wm8960__setAdcLeftDigitalVolume(volume);
}
bool wm8960__setAdcRightDigitalVolumeDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setAdcRightDigitalVolume()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_ADC_GAIN_OFFSET, WM8960_ADC_GAIN_STEPSIZE, WM8960_ADC_GAIN_MIN, WM8960_ADC_GAIN_MAX);

  return wm8960__setAdcRightDigitalVolume(volume);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// ALC
/////////////////////////////////////////////////////////

// Automatic Level Control
// Note that when the ALC function is enabled, the settings of registers 0 and
// 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and RINMUTE) are ignored.
bool wm8960__enableAlc(uint8_t mode)
{
  bool bit8 = (mode>>1);
  bool bit7 = (mode & 0b00000001);
  if (_writeRegisterBit(WM8960_REG_ALC1, 8, bit8) == 0) return false;
  return _writeRegisterBit(WM8960_REG_ALC1, 7, bit7);
}

 // Also sets alc sample rate to match global sample rate.
bool wm8960__disableAlc()
{
  if (_writeRegisterBit(WM8960_REG_ALC1, 8, 0) == 0) return false;
  return _writeRegisterBit(WM8960_REG_ALC1, 7, 0);
}

// Valid inputs are 0-15, 0 = -22.5dB FS, ... 1.5dB steps ... , 15 = -1.5dB FS
bool wm8960__setAlcTarget(uint8_t target)
{
  if(target > 15) target = 15; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ALC1,3,0,target);
}

// Valid inputs are 0-10, 0 = 24ms, 1 = 48ms, ... 10 = 24.58seconds
bool wm8960__setAlcDecay(uint8_t decay)
{
  if(decay > 10) decay = 10; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ALC3,7,4,decay);
}

// Valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms, ... 10 = 6.14seconds
bool wm8960__setAlcAttack(uint8_t attack)
{
  if(attack > 10) attack = 10; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ALC3,3,0,attack);
}

// Valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
bool wm8960__setAlcMaxGain(uint8_t maxGain)
{
  if(maxGain > 7) maxGain = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ALC1,6,4,maxGain);
}

// Valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB
bool wm8960__setAlcMinGain(uint8_t minGain)
{
  if(minGain > 7) minGain = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ALC2,6,4,minGain);
}

// Valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s
bool wm8960__setAlcHold(uint8_t hold)
{
  if(hold > 15) hold = 15; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_ALC2,3,0,hold);
}

// Peak Limiter
bool wm8960__enablePeakLimiter()
{
  return _writeRegisterBit(WM8960_REG_ALC3, 8, 1);
}

bool wm8960__disablePeakLimiter()
{
  return _writeRegisterBit(WM8960_REG_ALC3, 8, 0);
}

// Noise Gate
bool wm8960__enableNoiseGate()
{
  return _writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 1);
}

bool wm8960__disableNoiseGate()
{
  return _writeRegisterBit(WM8960_REG_NOISE_GATE, 0, 0);
}

// 0-31, 0 = -76.5dBfs, 31 = -30dBfs
bool wm8960__setNoiseGateThreshold(uint8_t threshold)
{
  return true;
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// DAC
/////////////////////////////////////////////////////////

// Enable/disble each channel
bool wm8960__enableDacLeft()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 1);
}

bool wm8960__disableDacLeft()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 8, 0);
}

bool wm8960__enableDacRight()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 1);
}

bool wm8960__disableDacRight()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 7, 0);
}

// DAC digital volume
// Valid inputs are 0-255
// 0 = mute
// 1 = -127dB
// ... 0.5dB steps up to
// 255 = 0dB
bool wm8960__setDacLeftDigitalVolume(uint8_t volume)
{
  bool result1 = _writeRegisterMultiBits(WM8960_REG_LEFT_DAC_VOLUME,7,0,volume);
  bool result2 = wm8960__dacLeftDACVUSet();
  return (result1 && result2);
}

bool wm8960__setDacRightDigitalVolume(uint8_t volume)
{
  bool result1 = _writeRegisterMultiBits(WM8960_REG_RIGHT_DAC_VOLUME,7,0,volume);
  bool result2 = wm8960__dacRightDACVUSet();
  return (result1 && result2);
}

// Causes left and right input dac digital volumes to be updated
bool wm8960__dacLeftDACVUSet()
{
  return _writeRegisterBit(WM8960_REG_LEFT_DAC_VOLUME, 8, 1);
}

 // Causes left and right input dac digital volumes to be updated
bool wm8960__dacRightDACVUSet()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_DAC_VOLUME, 8, 1);
}

// DAC digital volume DB
// Sets the volume of the DAC to a specified dB value passed in as a float
// argument.
// Valid dB settings are -97.00 up to +30.0 (0.5dB steps)
// -97.50 (or lower) = MUTE
// -97.00 = -97.00dB (MIN)
// ... 0.5dB steps ...
// 30.00 = +30.00dB  (MAX)

bool wm8960__setDacLeftDigitalVolumeDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setDacLeftDigitalVolume()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX);

  return wm8960__setDacLeftDigitalVolume(volume);
}

bool wm8960__setDacRightDigitalVolumeDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setDacRightDigitalVolume()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_DAC_GAIN_OFFSET, WM8960_DAC_GAIN_STEPSIZE, WM8960_DAC_GAIN_MIN, WM8960_DAC_GAIN_MAX);

  return wm8960__setDacRightDigitalVolume(volume);
}

// DAC mute
bool wm8960__enableDacMute()
{
  return _writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 1);
}

bool wm8960__disableDacMute()
{
  return _writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 3, 0);
}

// 3D Stereo Enhancement
// 3D enable/disable
bool wm8960__enable3d()
{
  return _writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 1);
}

bool wm8960__disable3d()
{
  return _writeRegisterBit(WM8960_REG_3D_CONTROL, 0, 0);
}

bool wm8960__set3dDepth(uint8_t depth) // 0 = 0%, 15 = 100%
{
  if(depth > 15) depth = 15; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_3D_CONTROL,4,1,depth);
}

// DAC output -6dB attentuation enable/disable
bool wm8960__enableDac6dbAttenuation()
{
  return _writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 1);
}

bool wm8960__disableDac6dbAttentuation()
{
  return _writeRegisterBit(WM8960_REG_ADC_DAC_CTRL_1, 7, 0);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// OUTPUT mixers
/////////////////////////////////////////////////////////

// What's connected to what? Oh so many options...
// LOMIX	Left Output Mixer
// ROMIX	Right Output Mixer
// OUT3MIX		Mono Output Mixer

// Enable/disable left and right output mixers
bool wm8960__enableLOMIX()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 1);
}

bool wm8960__disableLOMIX()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 3, 0);
}

bool wm8960__enableROMIX()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 1);
}

bool wm8960__disableROMIX()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_3, 2, 0);
}

bool wm8960__enableOUT3MIX()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 1);
}

bool wm8960__disableOUT3MIX()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 1, 0);
}

// Enable/disable audio path connections/vols to/from output mixers
// See datasheet page 35 for a nice image of all the connections.
bool wm8960__enableLI2LO()
{
  return _writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 1);
}

bool wm8960__disableLI2LO()
{
  return _writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 7, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
bool wm8960__setLI2LOVOL(uint8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_LEFT_OUT_MIX_1,6,4,volume);
}

bool wm8960__enableLB2LO()
{
  return _writeRegisterBit(WM8960_REG_BYPASS_1, 7, 1);
}

bool wm8960__disableLB2LO()
{
  return _writeRegisterBit(WM8960_REG_BYPASS_1, 7, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
bool wm8960__setLB2LOVOL(uint8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_BYPASS_1,6,4,volume);
}

bool wm8960__enableLD2LO()
{
  return _writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 1);
}

bool wm8960__disableLD2LO()
{
  return _writeRegisterBit(WM8960_REG_LEFT_OUT_MIX_1, 8, 0);
}

bool wm8960__enableRI2RO()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 1);
}

bool wm8960__disableRI2RO()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 7, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
bool wm8960__setRI2ROVOL(uint8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_RIGHT_OUT_MIX_2,6,4,volume);
}

bool wm8960__enableRB2RO()
{
  return _writeRegisterBit(WM8960_REG_BYPASS_2, 7, 1);
}

bool wm8960__disableRB2RO()
{
  return _writeRegisterBit(WM8960_REG_BYPASS_2, 7, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
bool wm8960__setRB2ROVOL(uint8_t volume)
{
  if(volume > 7) volume = 7; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_BYPASS_2,6,4,volume);
}

bool wm8960__enableRD2RO()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 1);
}

bool wm8960__disableRD2RO()
{
  return _writeRegisterBit(WM8960_REG_RIGHT_OUT_MIX_2, 8, 0);
}

// Mono Output mixer.
// Note, for capless HPs, we'll want this to output a buffered VMID.
// To do this, we need to disable both of these connections.
bool wm8960__enableLI2MO()
{
  return _writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 1);
}

bool wm8960__disableLI2MO()
{
  return _writeRegisterBit(WM8960_REG_MONO_OUT_MIX_1, 7, 0);
}

bool wm8960__enableRI2MO()
{
  return _writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 1);
}

bool wm8960__disableRI2MO()
{
  return _writeRegisterBit(WM8960_REG_MONO_OUT_MIX_2, 7, 0);
}

// Enables VMID in the WM8960_REG_PWR_MGMT_2 register, and set's it to
// playback/record settings of 2*50Kohm.
// Note, this function is only hear for backwards compatibility with the
// original releases of this library. It is recommended to use the
// setVMID() function instead.
bool wm8960__enableVMID()
{
  return wm8960__setVMID(WM8960_VMIDSEL_2X50KOHM);
}

bool wm8960__disableVMID()
{
  return wm8960__setVMID(WM8960_VMIDSEL_DISABLED);
}

// setVMID
// Sets the VMID signal to one of three possible settings.
// 4 options:
// WM8960_VMIDSEL_DISABLED
// WM8960_VMIDSEL_2X50KOHM (playback / record)
// WM8960_VMIDSEL_2X250KOHM (for low power / standby)
// WM8960_VMIDSEL_2X5KOHM (for fast start-up)
bool wm8960__setVMID(uint8_t setting)
{
  return _writeRegisterMultiBits(WM8960_REG_PWR_MGMT_1, 8, 7, setting);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Headphones
/////////////////////////////////////////////////////////

// Enable and disable headphones (mute)
bool wm8960__enableHeadphones()
{
  return (wm8960__enableRightHeadphone() & wm8960__enableLeftHeadphone());
}

bool wm8960__disableHeadphones()
{
  return (wm8960__disableRightHeadphone() & wm8960__disableLeftHeadphone());
}

bool wm8960__enableRightHeadphone()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 1);
}

bool wm8960__disableRightHeadphone()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 5, 0);
}

bool wm8960__enableLeftHeadphone()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 1);
}

bool wm8960__disableLeftHeadphone()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 6, 0);
}

bool wm8960__enableHeadphoneStandby()
{
  return _writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 1);
}

bool wm8960__disableHeadphoneStandby()
{
  return _writeRegisterBit(WM8960_REG_ANTI_POP_1, 0, 0);
}

// SetHeadphoneVolume
// Sets the volume for both left and right headphone outputs
//
// Although you can control each headphone output independently, here we are
// Going to assume you want both left and right to do the same thing.
//
// Valid inputs: 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ... 127 = +6dB
bool wm8960__setHeadphoneVolume(uint8_t volume)
{
  // Updates both left and right channels
	// Handles the OUT1VU (volume update) bit control, so that it happens at the
  // same time on both channels. Note, we must also make sure that the outputs
  // are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]
  // Grab local copy of register
  // Modify the bits we need to
  // Write register in device, including the volume update bit write
  // If successful, save locally.

  // Limit inputs
  if (volume > 127) volume = 127;

  // LEFT
    bool result1 = _writeRegisterMultiBits(WM8960_REG_LOUT1_VOLUME,6,0,volume);
  // RIGHT
    bool result2 = _writeRegisterMultiBits(WM8960_REG_ROUT1_VOLUME,6,0,volume);
  // UPDATES

  // Updated left channel
    bool result3 = _writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 8, 1);

  // Updated right channel
    bool result4 = _writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 8, 1);

    if (result1 && result2 && result3 && result4) // If all writes ACK'd
    {
        return true;
    }
  return false;
}

// Set headphone volume dB
// Sets the volume of the headphone output buffer amp to a specified dB value
// passed in as a float argument.
// Valid dB settings are -74.0 up to +6.0
// Note, we are accepting float arguments here, in order to keep it consistent
// with other volume setting functions in this library that can do partial dB
// values (such as the PGA, ADC and DAC gains).
// -74 (or lower) = MUTE
// -73 = -73dB (MIN)
// ... 1dB steps ...
// 0 = 0dB
// ... 1dB steps ...
// 6 = +6dB  (MAX)
bool wm8960__setHeadphoneVolumeDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setHeadphoneVolume()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_HP_GAIN_OFFSET, WM8960_HP_GAIN_STEPSIZE, WM8960_HP_GAIN_MIN, WM8960_HP_GAIN_MAX);

  return wm8960__setHeadphoneVolume(volume);
}

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right Headphone outputs
bool wm8960__enableHeadphoneZeroCross()
{
  // Left
  bool result1 = _writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 1);

  // Right
  bool result2 = _writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 1);
  return (result1 & result2);
}

bool wm8960__disableHeadphoneZeroCross()
{
  // Left
  bool result1 = _writeRegisterBit(WM8960_REG_LOUT1_VOLUME, 7, 0);

  // Right
  bool result2 = _writeRegisterBit(WM8960_REG_ROUT1_VOLUME, 7, 0);
  return (result1 & result2);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Speakers
/////////////////////////////////////////////////////////

// Enable and disable speakers (mute)
bool wm8960__enableSpeakers()
{
  return (wm8960__enableRightSpeaker() & wm8960__enableLeftSpeaker());
}

bool wm8960__disableSpeakers()
{
  return (wm8960__disableRightHeadphone() & wm8960__disableLeftHeadphone());
}

bool wm8960__enableRightSpeaker()
{
  // SPK_OP_EN
  bool result1 = _writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 1);

  // SPKR
  bool result2 = _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 1);
  return (result1 & result2);
}

bool wm8960__disableRightSpeaker()
{
  // SPK_OP_EN
  bool result1 = _writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 7, 0);

  // SPKR
  bool result2 = _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 3, 0);
  return (result1 & result2);
}

bool wm8960__enableLeftSpeaker()
{
  // SPK_OP_EN
  bool result1 = _writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 1);

  // SPKL
  bool result2 = _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 1);
  return (result1 & result2);
}

bool wm8960__disableLeftSpeaker()
{
  // SPK_OP_EN
  bool result1 = _writeRegisterBit(WM8960_REG_CLASS_D_CONTROL_1, 6, 0);

  // SPKL
  bool result2 = _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 4, 0);
  return (result1 & result2);
}

// SetSpeakerVolume
// Sets to volume for both left and right speaker outputs
//
// Although you can control each Speaker output independently, here we are
// Going to assume you want both left and right to do the same thing.
//
// Valid inputs are 47-127. 0-47 = mute, 48 = -73dB, ... 1dB steps ... , 127 = +6dB
bool wm8960__setSpeakerVolume(uint8_t volume)
{
  // Updates both left and right channels
	// Handles the SPKVU (volume update) bit control, so that it happens at the
  // same time on both channels. Note, we must also make sure that the outputs
  // are enabled in the WM8960_REG_PWR_MGMT_2 [4:3], and the class D control
  // reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

  // Limit inputs
  if (volume > 127) volume = 127;

  // LEFT
  bool result1 = _writeRegisterMultiBits(WM8960_REG_LOUT2_VOLUME,6,0,volume);

  // RIGHT
  bool result2 = _writeRegisterMultiBits(WM8960_REG_ROUT2_VOLUME,6,0,volume);

  // SPKVU

  // Updated left channel
  bool result3 = _writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 8, 1);

  // Updated right channel
  bool result4 = _writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 8, 1);

  if (result1 && result2 && result3 && result4) // If all writes ACK'd
    {
        return true;
    }
  return false;
}

// Set speaker volume dB
// Sets the volume of the class-d speaker output amp to a specified dB value
// passed in as a float argument.
// Valid dB settings are -74.0 up to +6.0
// Note, we are accepting float arguments here, in order to keep it consistent
// with other volume setting functions in this library that can do partial dB
// values (such as the PGA, ADC and DAC gains).
// -74 (or lower) = MUTE
// -73 = -73dB (MIN)
// ... 1dB steps ...
// 0 = 0dB
// ... 1dB steps ...
// 6 = +6dB  (MAX)
bool wm8960__setSpeakerVolumeDB(float dB)
{
  // Create an unsigned integer volume setting variable we can send to
  // setSpeakerVolume()
  uint8_t volume = _convertDBtoSetting(dB, WM8960_SPEAKER_GAIN_OFFSET, WM8960_SPEAKER_GAIN_STEPSIZE, WM8960_SPEAKER_GAIN_MIN, WM8960_SPEAKER_GAIN_MAX);

  return wm8960__setSpeakerVolume(volume);
}

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right Speaker outputs
bool wm8960__enableSpeakerZeroCross()
{
  // Left
  bool result1 = _writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 1);

  // Right
  bool result2 = _writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 1);
  return (result1 & result2);
}

bool wm8960__disableSpeakerZeroCross()
{
  // Left
  bool result1 = _writeRegisterBit(WM8960_REG_LOUT2_VOLUME, 7, 0);

  // Right
  bool result2 = _writeRegisterBit(WM8960_REG_ROUT2_VOLUME, 7, 0);
  return (result1 & result2);
}

// SetSpeakerDcGain
// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// Valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
bool wm8960__setSpeakerDcGain(uint8_t gain)
{
  if(gain > 5) gain = 5; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,5,3,gain);
}

// SetSpeakerAcGain
// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// Valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
bool wm8960__setSpeakerAcGain(uint8_t gain)
{
  if(gain > 5) gain = 5; // Limit incoming values max
  return _writeRegisterMultiBits(WM8960_REG_CLASS_D_CONTROL_3,2,0,gain);
}

//////////////////////////////////////////////
////////////////////////////////////////////// Digital audio interface control
//////////////////////////////////////////////

// Defaults to I2S, peripheral-mode, 24-bit word length

// Loopback
// When enabled, the output data from the ADC audio interface is fed directly
// into the DAC data input.
bool wm8960__enableLoopBack()
{
  return _writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 1);
}

bool wm8960__disableLoopBack()
{
  return _writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 0, 0);
}

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Clock controls
/////////////////////////////////////////////////////////

// Getting the Frequency of SampleRate as we wish
// Our MCLK (an external clock on the SFE breakout board) is 24.0MHz.
// According to table 40 (DS pg 58), we want SYSCLK to be 11.2896 for a SR of
// 44.1KHz. To get that Desired Output (SYSCLK), we need the following settings
// on the PLL stuff, as found on table 45 (ds pg 61):
// PRESCALE DIVIDE (PLLPRESCALE): 2
// POSTSCALE DVIDE (SYSCLKDIV[1:0]): 2
// FIXED POST-DIVIDE: 4
// R: 7.5264
// N: 7h
// K: 86C226h

// Example at bottom of table 46, shows that we should be in fractional mode
// for a 44.1KHz.

// In terms of registers, this is what we want for 44.1KHz
// PLLEN=1			(PLL enable)
// PLLPRESCALE=1	(divide by 2) *This get's us from MCLK (24MHz) down to 12MHZ
// for F2.
// PLLN=7h			(PLL N value) *this is "int R"
// PLLK=86C226h		(PLL K value) *this is int ( 2^24 * (R- intR))
// SDM=1			(Fractional mode)
// CLKSEL=1			(PLL select)
// MS=0				(Peripheral mode)
// WL=00			(16 bits)
// SYSCLKDIV=2		(Divide by 2)
// ADCDIV=000		(Divide by 1) = 44.1kHz
// DACDIV=000		(Divide by 1) = 44.1kHz
// BCLKDIV=0100		(Divide by 4) = 64fs
// DCLKDIV=111		(Divide by 16) = 705.6kHz

// And now for the functions that will set these registers...
bool wm8960__enablePLL()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 1);
}

bool wm8960__disablePLL()
{
  return _writeRegisterBit(WM8960_REG_PWR_MGMT_2, 0, 0);
}

bool wm8960__setPLLPRESCALE(bool div)
{
  return _writeRegisterBit(WM8960_REG_PLL_N, 4, div);
}

bool wm8960__setPLLN(uint8_t n)
{
  return _writeRegisterMultiBits(WM8960_REG_PLL_N,3,0,n);
}

// Send each nibble of 24-bit value for value K
bool wm8960__setPLLK(uint8_t one, uint8_t two, uint8_t three)
{
  bool result1 = _writeRegisterMultiBits(WM8960_REG_PLL_K_1,5,0,one);
  bool result2 = _writeRegisterMultiBits(WM8960_REG_PLL_K_2,8,0,two);
  bool result3 = _writeRegisterMultiBits(WM8960_REG_PLL_K_3,8,0,three);
  if (result1 && result2 && result3) // If all I2C sommands Ack'd, then...
  {
    return true;
  }
  return false;
}

// 0=integer, 1=fractional
bool wm8960__setSMD(bool mode)
{
  return _writeRegisterBit(WM8960_REG_PLL_N, 5, mode);
}

 // 0=MCLK, 1=PLL_output
bool wm8960__setCLKSEL(bool sel)
{
  return _writeRegisterBit(WM8960_REG_CLOCKING_1, 0, sel);
}

// (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
bool wm8960__setSYSCLKDIV(uint8_t div)
{
  return _writeRegisterMultiBits(WM8960_REG_CLOCKING_1,2,1,div);
}

// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
bool wm8960__setADCDIV(uint8_t div)
{
  return _writeRegisterMultiBits(WM8960_REG_CLOCKING_1,8,6,div);
}

// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
bool wm8960__setDACDIV(uint8_t div)
{
  return _writeRegisterMultiBits(WM8960_REG_CLOCKING_1,5,3,div);
}

bool wm8960__setBCLKDIV(uint8_t div)
{
  return _writeRegisterMultiBits(WM8960_REG_CLOCKING_2,3,0,div);
}

// Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
bool wm8960__setDCLKDIV(uint8_t div)
{
  return _writeRegisterMultiBits(WM8960_REG_CLOCKING_2,8,6,div);
}

bool wm8960__setALRCGPIO()
{
  // This setting should not be changed if ADCs are enabled.
  return _writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_2, 6, 1);
}

bool wm8960__enableMasterMode()
{
  return _writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 1);
}

bool wm8960__enablePeripheralMode()
{
  return _writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 6, 0);
}

bool wm8960__setWL(uint8_t word_length)
{
  return _writeRegisterMultiBits(WM8960_REG_AUDIO_INTERFACE_1,3,2,word_length);
}

bool wm8960__setLRP(bool polarity)
{
  return _writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 4, polarity);
}

bool wm8960__setALRSWAP(bool swap)
{
  return _writeRegisterBit(WM8960_REG_AUDIO_INTERFACE_1, 8, swap);
}

bool wm8960__setVROI(bool setting)
{
  return _writeRegisterBit(WM8960_REG_ADDITIONAL_CONTROL_3, 6, setting);
}

bool wm8960__setVSEL(uint8_t setting)
{
  return _writeRegisterMultiBits(WM8960_REG_ADDITIONAL_CONTROL_1,7,6,setting);
}

// convertDBtoSetting
// This function will take in a dB value (as a float), and return the
// corresponding volume setting necessary.
// For example, Headphone volume control goes from 47-120.
// While PGA gain control is from 0-63.
// The offset values allow for proper conversion.
//
// dB - float value of dB
//
// offset - the differnce from lowest dB value to lowest setting value
//
// stepSize - the dB step for each setting (aka the "resolution" of the setting)
// This is 0.75dB for the PGAs, 0.5 for ADC/DAC, and 1dB for most other amps.
//
// minDB - float of minimum dB setting allowed, note this is not mute on the
// amp. "True mute" is always one stepSize lower.
//
// maxDB - float of maximum dB setting allowed. If you send anything higher, it
// will be limited to this max value.
uint8_t _convertDBtoSetting(float dB, float offset, float stepSize, float minDB, float maxDB)
{
  // Limit incoming dB values to acceptable range. Note, the minimum limit we
  // want to limit this too is actually one step lower than the minDB, because
  // that is still an acceptable dB level (it is actually "true mute").
  // Note, the PGA amp does not have a "true mute" setting available, so we
  // must check for its unique minDB of -17.25.

  // Limit max. This is the same for all amps.
  if (dB > maxDB) dB = maxDB;

  // PGA amp doesn't have mute setting, so minDB should be limited to minDB
  // Let's check for the PGAs unique minDB (-17.25) to know we are currently
  // converting a PGA setting.
  if(minDB == WM8960_PGA_GAIN_MIN)
  {
    if (dB < minDB) dB = minDB;
  }
  else // Not PGA. All other amps have a mute setting below minDb
  {
    if (dB < (minDB - stepSize)) dB = (minDB - stepSize);
  }

  // Adjust for offset
  // Offset is the number that gets us from the minimum dB option of an amp
  // up to the minimum setting value in the register.
  dB = dB + offset;

  // Find out how many steps we are above the minimum (at this point, our
  // minimum is "0". Note, because dB comes in as a float, the result of this
  // division (volume) can be a partial number. We will round that next.
  float volume = dB / stepSize;

  volume = round(volume); // round to the nearest setting value.

  // Serial debug (optional)
  // Serial.print("\t");
  // Serial.print((uint8_t)volume);

  return (uint8_t)volume; // cast from float to unsigned 8-bit integer.
}
