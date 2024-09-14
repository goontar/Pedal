#ifndef SRC_WM8960_DRIVER_WM8960_DRIVER_H_
#define SRC_WM8960_DRIVER_WM8960_DRIVER_H_
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define DEVICE_ADDRESS (0x34)
#define DEFAULT_I2C_TIMEOUT (1000)

// I2C address (7-bit format for Wire library)
#define WM8960_ADDR 0x1A

// WM8960 register addresses
#define WM8960_REG_LEFT_INPUT_VOLUME 0x00
#define WM8960_REG_RIGHT_INPUT_VOLUME 0x01
#define WM8960_REG_LOUT1_VOLUME 0x02
#define WM8960_REG_ROUT1_VOLUME 0x03
#define WM8960_REG_CLOCKING_1 0x04
#define WM8960_REG_ADC_DAC_CTRL_1 0x05
#define WM8960_REG_ADC_DAC_CTRL_2 0x06
#define WM8960_REG_AUDIO_INTERFACE_1 0x07
#define WM8960_REG_CLOCKING_2 0x08
#define WM8960_REG_AUDIO_INTERFACE_2 0x09
#define WM8960_REG_LEFT_DAC_VOLUME 0x0A
#define WM8960_REG_RIGHT_DAC_VOLUME 0x0B
#define WM8960_REG_RESET 0x0F
#define WM8960_REG_3D_CONTROL 0x10
#define WM8960_REG_ALC1 0x11
#define WM8960_REG_ALC2 0x12
#define WM8960_REG_ALC3 0x13
#define WM8960_REG_NOISE_GATE 0x14
#define WM8960_REG_LEFT_ADC_VOLUME 0x15
#define WM8960_REG_RIGHT_ADC_VOLUME 0x16
#define WM8960_REG_ADDITIONAL_CONTROL_1 0x17
#define WM8960_REG_ADDITIONAL_CONTROL_2 0x18
#define WM8960_REG_PWR_MGMT_1 0x19
#define WM8960_REG_PWR_MGMT_2 0x1A
#define WM8960_REG_ADDITIONAL_CONTROL_3 0x1B
#define WM8960_REG_ANTI_POP_1 0x1C
#define WM8960_REG_ANTI_POP_2 0x1D
#define WM8960_REG_ADCL_SIGNAL_PATH 0x20
#define WM8960_REG_ADCR_SIGNAL_PATH 0x21
#define WM8960_REG_LEFT_OUT_MIX_1 0x22
#define WM8960_REG_RIGHT_OUT_MIX_2 0x25
#define WM8960_REG_MONO_OUT_MIX_1 0x26
#define WM8960_REG_MONO_OUT_MIX_2 0x27
#define WM8960_REG_LOUT2_VOLUME 0x28
#define WM8960_REG_ROUT2_VOLUME 0x29
#define WM8960_REG_MONO_OUT_VOLUME 0x2A
#define WM8960_REG_INPUT_BOOST_MIXER_1 0x2B
#define WM8960_REG_INPUT_BOOST_MIXER_2 0x2C
#define WM8960_REG_BYPASS_1 0x2D
#define WM8960_REG_BYPASS_2 0x2E
#define WM8960_REG_PWR_MGMT_3 0x2F
#define WM8960_REG_ADDITIONAL_CONTROL_4 0x30
#define WM8960_REG_CLASS_D_CONTROL_1 0x31
#define WM8960_REG_CLASS_D_CONTROL_3 0x33
#define WM8960_REG_PLL_N 0x34
#define WM8960_REG_PLL_K_1 0x35
#define WM8960_REG_PLL_K_2 0x36
#define WM8960_REG_PLL_K_3 0x37

// PGA input selections
#define WM8960_PGAL_LINPUT2 0
#define WM8960_PGAL_LINPUT3 1
#define WM8960_PGAL_VMID 2
#define WM8960_PGAR_RINPUT2 0
#define WM8960_PGAR_RINPUT3 1
#define WM8960_PGAR_VMID 2

// Mic (aka PGA) BOOST gain options
#define WM8960_MIC_BOOST_GAIN_0DB 0
#define WM8960_MIC_BOOST_GAIN_13DB 1
#define WM8960_MIC_BOOST_GAIN_20DB 2
#define WM8960_MIC_BOOST_GAIN_29DB 3

// Boost Mixer gain options
// These are used to control the gain (aka volume) at the following settings:
// LIN2BOOST
// LIN3BOOST
// RIN2BOOST
// RIN3BOOST
#define WM8960_BOOST_MIXER_GAIN_MUTE 0
#define WM8960_BOOST_MIXER_GAIN_NEG_12DB 1
#define WM8960_BOOST_MIXER_GAIN_NEG_9DB 2
#define WM8960_BOOST_MIXER_GAIN_NEG_6DB 3
#define WM8960_BOOST_MIXER_GAIN_NEG_3DB 4
#define WM8960_BOOST_MIXER_GAIN_0DB 5
#define WM8960_BOOST_MIXER_GAIN_3DB 6
#define WM8960_BOOST_MIXER_GAIN_6DB 7

// Output Mixer gain options
// These are used to control the gain (aka volume) at the following settings:
// LI2LOVOL
// LB2LOVOL
// RI2LOVOL
// RB2LOVOL
// These are useful as analog bypass signal path options.
#define WM8960_OUTPUT_MIXER_GAIN_0DB 0
#define WM8960_OUTPUT_MIXER_GAIN_NEG_3DB 1
#define WM8960_OUTPUT_MIXER_GAIN_NEG_6DB 2
#define WM8960_OUTPUT_MIXER_GAIN_NEG_9DB 3
#define WM8960_OUTPUT_MIXER_GAIN_NEG_12DB 4
#define WM8960_OUTPUT_MIXER_GAIN_NEG_15DB 5
#define WM8960_OUTPUT_MIXER_GAIN_NEG_18DB 6
#define WM8960_OUTPUT_MIXER_GAIN_NEG_21DB 7

// Mic Bias voltage options
#define WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD 0
#define WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD 1

// SYSCLK divide
#define WM8960_SYSCLK_DIV_BY_1 0
#define WM8960_SYSCLK_DIV_BY_2 2
#define WM8960_CLKSEL_MCLK 0
#define WM8960_CLKSEL_PLL 1
#define WM8960_PLL_MODE_INTEGER 0
#define WM8960_PLL_MODE_FRACTIONAL 1
#define WM8960_PLLPRESCALE_DIV_1 0
#define WM8960_PLLPRESCALE_DIV_2 1

// Class d clock divide
#define WM8960_DCLKDIV_16 7

// Word length settings (aka bits per sample)
// Audio Data Word Length
#define WM8960_WL_16BIT 0
#define WM8960_WL_20BIT 1
#define WM8960_WL_24BIT 2
#define WM8960_WL_32BIT 3

// Additional Digital Audio Interface controls
// LRP (aka left-right-polarity)
// Right, left and I2S modes – LRCLK polarity
// 0 = normal LRCLK polarity
// 1 = inverted LRCLK polarity
#define WM8960_LR_POLARITY_NORMAL 0
#define WM8960_LR_POLARITY_INVERT 1

// ALRSWAP (aka ADC left/right swap)
// Left/Right ADC channel swap
// 1 = Swap left and right ADC data in audio interface
// 0 = Output left and right data as normal
#define WM8960_ALRSWAP_NORMAL 0
#define WM8960_ALRSWAP_SWAP 1

// Gain mins, maxes, offsets and step-sizes for all the amps within the codec.
#define WM8960_PGA_GAIN_MIN -17.25
#define WM8960_PGA_GAIN_MAX 30.00
#define WM8960_PGA_GAIN_OFFSET 17.25
#define WM8960_PGA_GAIN_STEPSIZE 0.75
#define WM8960_HP_GAIN_MIN -73.00
#define WM8960_HP_GAIN_MAX 6.00
#define WM8960_HP_GAIN_OFFSET 121.00
#define WM8960_HP_GAIN_STEPSIZE 1.00
#define WM8960_SPEAKER_GAIN_MIN -73.00
#define WM8960_SPEAKER_GAIN_MAX 6.00
#define WM8960_SPEAKER_GAIN_OFFSET 121.00
#define WM8960_SPEAKER_GAIN_STEPSIZE 1.00
#define WM8960_ADC_GAIN_MIN -97.00
#define WM8960_ADC_GAIN_MAX 30.00
#define WM8960_ADC_GAIN_OFFSET 97.50
#define WM8960_ADC_GAIN_STEPSIZE 0.50
#define WM8960_DAC_GAIN_MIN -97.00
#define WM8960_DAC_GAIN_MAX 30.00
#define WM8960_DAC_GAIN_OFFSET 97.50
#define WM8960_DAC_GAIN_STEPSIZE 0.50

// Automatic Level Control Modes
#define WM8960_ALC_MODE_OFF 0
#define WM8960_ALC_MODE_RIGHT_ONLY 1
#define WM8960_ALC_MODE_LEFT_ONLY 2
#define WM8960_ALC_MODE_STEREO 3

// Automatic Level Control Target Level dB
#define WM8960_ALC_TARGET_LEVEL_NEG_22_5DB 0
#define WM8960_ALC_TARGET_LEVEL_NEG_21DB 1
#define WM8960_ALC_TARGET_LEVEL_NEG_19_5DB 2
#define WM8960_ALC_TARGET_LEVEL_NEG_18DB 3
#define WM8960_ALC_TARGET_LEVEL_NEG_16_5DB 4
#define WM8960_ALC_TARGET_LEVEL_NEG_15DB 5
#define WM8960_ALC_TARGET_LEVEL_NEG_13_5DB 6
#define WM8960_ALC_TARGET_LEVEL_NEG_12DB 7
#define WM8960_ALC_TARGET_LEVEL_NEG_10_5DB 8
#define WM8960_ALC_TARGET_LEVEL_NEG_9DB 9
#define WM8960_ALC_TARGET_LEVEL_NEG_7_5DB 10
#define WM8960_ALC_TARGET_LEVEL_NEG_6DB 11
#define WM8960_ALC_TARGET_LEVEL_NEG_4_5DB 12
#define WM8960_ALC_TARGET_LEVEL_NEG_3DB 13
#define WM8960_ALC_TARGET_LEVEL_NEG_1_5DB 14

// Automatic Level Control Max Gain Level dB
#define WM8960_ALC_MAX_GAIN_LEVEL_NEG_12DB 0
#define WM8960_ALC_MAX_GAIN_LEVEL_NEG_6DB 1
#define WM8960_ALC_MAX_GAIN_LEVEL_0DB 2
#define WM8960_ALC_MAX_GAIN_LEVEL_6DB 3
#define WM8960_ALC_MAX_GAIN_LEVEL_12DB 4
#define WM8960_ALC_MAX_GAIN_LEVEL_18DB 5
#define WM8960_ALC_MAX_GAIN_LEVEL_24DB 6
#define WM8960_ALC_MAX_GAIN_LEVEL_30DB 7

// Automatic Level Control Min Gain Level dB
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_17_25DB 0
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_11_25DB 1
#define WM8960_ALC_MIN_GAIN_LEVEL_NEG_5_25DB 2
#define WM8960_ALC_MIN_GAIN_LEVEL_0_75DB 3
#define WM8960_ALC_MIN_GAIN_LEVEL_6_75DB 4
#define WM8960_ALC_MIN_GAIN_LEVEL_12_75DB 5
#define WM8960_ALC_MIN_GAIN_LEVEL_18_75DB 6
#define WM8960_ALC_MIN_GAIN_LEVEL_24_75DB 7

// Automatic Level Control Hold Time (MS and SEC)
#define WM8960_ALC_HOLD_TIME_0MS 0
#define WM8960_ALC_HOLD_TIME_3MS 1
#define WM8960_ALC_HOLD_TIME_5MS 2
#define WM8960_ALC_HOLD_TIME_11MS 3
#define WM8960_ALC_HOLD_TIME_21MS 4
#define WM8960_ALC_HOLD_TIME_43MS 5
#define WM8960_ALC_HOLD_TIME_85MS 6
#define WM8960_ALC_HOLD_TIME_170MS 7
#define WM8960_ALC_HOLD_TIME_341MS 8
#define WM8960_ALC_HOLD_TIME_682MS 9
#define WM8960_ALC_HOLD_TIME_1365MS 10
#define WM8960_ALC_HOLD_TIME_3SEC 11
#define WM8960_ALC_HOLD_TIME_5SEC 12
#define WM8960_ALC_HOLD_TIME_10SEC 13
#define WM8960_ALC_HOLD_TIME_23SEC 14
#define WM8960_ALC_HOLD_TIME_44SEC 15

// Automatic Level Control Decay Time (MS and SEC)
#define WM8960_ALC_DECAY_TIME_24MS 0
#define WM8960_ALC_DECAY_TIME_48MS 1
#define WM8960_ALC_DECAY_TIME_96MS 2
#define WM8960_ALC_DECAY_TIME_192MS 3
#define WM8960_ALC_DECAY_TIME_384MS 4
#define WM8960_ALC_DECAY_TIME_768MS 5
#define WM8960_ALC_DECAY_TIME_1536MS 6
#define WM8960_ALC_DECAY_TIME_3SEC 7
#define WM8960_ALC_DECAY_TIME_6SEC 8
#define WM8960_ALC_DECAY_TIME_12SEC 9
#define WM8960_ALC_DECAY_TIME_24SEC 10

// Automatic Level Control Attack Time (MS and SEC)
#define WM8960_ALC_ATTACK_TIME_6MS 0
#define WM8960_ALC_ATTACK_TIME_12MS 1
#define WM8960_ALC_ATTACK_TIME_24MS 2
#define WM8960_ALC_ATTACK_TIME_482MS 3
#define WM8960_ALC_ATTACK_TIME_964MS 4
#define WM8960_ALC_ATTACK_TIME_1928MS 5
#define WM8960_ALC_ATTACK_TIME_3846MS 6
#define WM8960_ALC_ATTACK_TIME_768MS 7
#define WM8960_ALC_ATTACK_TIME_1536MS 8
#define WM8960_ALC_ATTACK_TIME_3SEC 9
#define WM8960_ALC_ATTACK_TIME_6SEC 10

// Speaker Boost Gains (DC and AC)
#define WM8960_SPEAKER_BOOST_GAIN_0DB 0
#define WM8960_SPEAKER_BOOST_GAIN_2_1DB 1
#define WM8960_SPEAKER_BOOST_GAIN_2_9DB 2
#define WM8960_SPEAKER_BOOST_GAIN_3_6DB 3
#define WM8960_SPEAKER_BOOST_GAIN_4_5DB 4
#define WM8960_SPEAKER_BOOST_GAIN_5_1DB 5

// VMIDSEL settings
#define WM8960_VMIDSEL_DISABLED 0
#define WM8960_VMIDSEL_2X50KOHM 1
#define WM8960_VMIDSEL_2X250KOHM 2
#define WM8960_VMIDSEL_2X5KOHM 3

// VREF to Analogue Output Resistance
// (Disabled Outputs)
// 0 = 500 VMID to output
// 1 = 20k VMID to output
#define WM8960_VROI_500 0
#define WM8960_VROI_20K 1

// Analogue Bias Optimisation
// 00 = Reserved
// 01 = Increased bias current optimized for
// AVDD=2.7V
// 1X = Lowest bias current, optimized for
// AVDD=3.3V

#define WM8960_VSEL_INCREASED_BIAS_CURRENT 1
#define WM8960_VSEL_LOWEST_BIAS_CURRENT 3


bool wm8960__setup_i2c(I2C_HandleTypeDef* hi2c);
bool wm8960__enableVREF(); // Necessary for all other functions
bool wm8960__disableVREF(); // Use for turning this off to save power

bool wm8960__reset(); // Resets all registers to their default state

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// PGA
/////////////////////////////////////////////////////////

bool wm8960__enableAINL();
bool wm8960__disableAINL();
bool wm8960__enableAINR();
bool wm8960__disableAINR();

bool wm8960__enableLMIC();
bool wm8960__disableLMIC();
bool wm8960__enableRMIC();
bool wm8960__disableRMIC();

bool wm8960__enableLMICBOOST();
bool wm8960__disableLMICBOOST();
bool wm8960__enableRMICBOOST();
bool wm8960__disableRMICBOOST();

// PGA input signal select
// Each PGA (left and right) has a switch on its non-inverting input.
// On PGA_LEFT:
// You can select between VMID, LINPUT2 or LINPUT3
// Note, the inverting input of PGA_LEFT is perminantly connected to
// LINPUT1
// On PGA_RIGHT:
// You can select between VMIN, RINPUT2 or RINPUT3
// Note, the inverting input of PGA_RIGHT is perminantly connected to
// RINPUT1

// 3 options: WM8960_PGAL_LINPUT2, WM8960_PGAL_LINPUT3, WM8960_PGAL_VMID
bool wm8960__pgaLeftNonInvSignalSelect(uint8_t signal);

// 3 options: WM8960_PGAR_RINPUT2, WM8960_PGAR_RINPUT3, WM8960_PGAR_VMID
bool wm8960__pgaRightNonInvSignalSelect(uint8_t signal);

// Connections from each INPUT1 to the inverting input of its PGA

// Connect LINPUT1 to inverting input of Left Input PGA
bool wm8960__connectLMN1();

// Disconnect LINPUT1 from inverting input of Left Input PGA
bool wm8960__disconnectLMN1();

// Connect RINPUT1 to inverting input of Right Input PGA
bool wm8960__connectRMN1();

// Disconnect RINPUT1 from inverting input of Right Input PGA
bool wm8960__disconnectRMN1();

// Connection from output of PGAs to downstream "boost mixers"

// Connect Left Input PGA to Left Input Boost mixer
bool wm8960__connectLMIC2B();

// Disconnect Left Input PGA to Left Input Boost mixer
bool wm8960__disconnectLMIC2B();

// Connect Right Input PGA to Right Input Boost mixer
bool wm8960__connectRMIC2B();

// Disconnect Right Input PGA to Right Input Boost mixer
bool wm8960__disconnectRMIC2B();

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
bool wm8960__setLINVOL(uint8_t volume);
bool wm8960__setLINVOLDB(float dB);

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
bool wm8960__setRINVOL(uint8_t volume);
bool wm8960__setRINVOLDB(float dB);

// Zero Cross prevents zipper sounds on volume changes
bool wm8960__enablePgaZeroCross(); // Sets both left and right PGAs
bool wm8960__disablePgaZeroCross(); // Sets both left and right PGAs

bool wm8960__enableLINMUTE();
bool wm8960__disableLINMUTE();
bool wm8960__enableRINMUTE();
bool wm8960__disableRINMUTE();

// Causes left and right input PGA volumes to be updated
// (LINVOL and RINVOL)
bool wm8960__pgaLeftIPVUSet();

// Causes left and right input PGA volumes to be updated
// (LINVOL and RINVOL)
bool wm8960__pgaRightIPVUSet();

// Boosts

// WM8960_MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
bool wm8960__setLMICBOOST(uint8_t boost_gain);

// WM8960_MIC_BOOST_GAIN_0DB or _13DB, _20DB, _29DB
bool wm8960__setRMICBOOST(uint8_t boost_gain);

// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
bool wm8960__setLIN3BOOST(uint8_t boost_gain);

// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
bool wm8960__setLIN2BOOST(uint8_t boost_gain);

// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
bool wm8960__setRIN3BOOST(uint8_t boost_gain);

// WM8960_BOOST_MIXER_GAIN_MUTE, WM8960_BOOST_MIXER_GAIN_NEG_12DB, ...
bool wm8960__setRIN2BOOST(uint8_t boost_gain);

// Mic Bias control
bool wm8960__enableMicBias();
bool wm8960__disableMicBias();

// WM8960_MIC_BIAS_VOLTAGE_0_9_AVDD (0.9*AVDD)
// or WM8960_MIC_BIAS_VOLTAGE_0_65_AVDD (0.65*AVDD)
bool wm8960__setMicBiasVoltage(bool voltage);

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// ADC
/////////////////////////////////////////////////////////

bool wm8960__enableAdcLeft();
bool wm8960__disableAdcLeft();
bool wm8960__enableAdcRight();
bool wm8960__disableAdcRight();

// ADC digital volume
// Note, also needs to handle control of the ADCVU bits (volume update).
// Valid inputs are 0-255
// 0 = mute
// 1 = -97dB
// ... 0.5dB steps up to
// 195 = 0dB
// 255 = +30dB
bool wm8960__setAdcLeftDigitalVolume(uint8_t volume);
bool wm8960__setAdcRightDigitalVolume(uint8_t volume);
bool wm8960__setAdcLeftDigitalVolumeDB(float dB);
bool wm8960__setAdcRightDigitalVolumeDB(float dB);

// Causes left and right input ADC volumes to be updated
bool wm8960__adcLeftADCVUSet();

// Causes left and right input ADC volumes to be updated
bool wm8960__adcRightADCVUSet();

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// ALC
/////////////////////////////////////////////////////////

// Automatic Level Control
// Note that when the ALC function is enabled, the settings of
// Registers 0 and 1 (LINVOL, IPVU, LIZC, LINMUTE, RINVOL, RIZC and
// RINMUTE) are ignored.

// Also sets alc sample rate to match global sample rate.
bool wm8960__enableAlc(uint8_t mode);

bool wm8960__disableAlc();

// Valid inputs are 0-15
// 0 = -22.5dB FS ... 1.5dB steps ... 15 = -1.5dB FS
bool wm8960__setAlcTarget(uint8_t target);

// Valid inputs are 0-10, 0 = 24ms, 1 = 48ms ... 10 = 24.58seconds
bool wm8960__setAlcDecay(uint8_t decay);

// Valid inputs are 0-10, 0 = 6ms, 1 = 12ms, 2 = 24ms ...
// 10 = 6.14seconds
bool wm8960__setAlcAttack(uint8_t attack);

// Valid inputs are 0-7, 0 = -12dB, ... 7 = +30dB
bool wm8960__setAlcMaxGain(uint8_t maxGain);

// Valid inputs are 0-7, 0 = -17.25dB, ... 7 = +24.75dB
bool wm8960__setAlcMinGain(uint8_t attack);

// Valid inputs are 0-15, 0 = 0ms, ... 15 = 43.691s
bool wm8960__setAlcHold(uint8_t attack);

// Peak Limiter
bool wm8960__enablePeakLimiter();
bool wm8960__disablePeakLimiter();

// Noise Gate
bool wm8960__enableNoiseGate();
bool wm8960__disableNoiseGate();

// 0-31, 0 = -76.5dBfs, 31 = -30dBfs
bool wm8960__setNoiseGateThreshold(uint8_t threshold);

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// DAC
/////////////////////////////////////////////////////////

// Enable/disble each channel
bool wm8960__enableDacLeft();
bool wm8960__disableDacLeft();
bool wm8960__enableDacRight();
bool wm8960__disableDacRight();

// DAC digital volume
// Note, also needs to handle control of the DACVU bits (volume update).
// Valid inputs are 0-255
// 0 = mute
// 1 = -127dB
// ... 0.5dB steps up to
// 255 = 0dB
bool wm8960__setDacLeftDigitalVolume(uint8_t volume);
bool wm8960__setDacRightDigitalVolume(uint8_t volume);
bool wm8960__setDacLeftDigitalVolumeDB(float dB);
bool wm8960__setDacRightDigitalVolumeDB(float dB);

// Causes left and right input DAC volumes to be updated
bool wm8960__dacLeftDACVUSet();

// Causes left and right input DAC volumes to be updated
bool wm8960__dacRightDACVUSet();

// DAC mute
bool wm8960__enableDacMute();
bool wm8960__disableDacMute();

// DE-Emphasis

// 3D Stereo Enhancement
// 3D enable/disable
bool wm8960__enable3d();
bool wm8960__disable3d();
bool wm8960__set3dDepth(uint8_t depth); // 0 = 0%, 15 = 100%

// 3D upper/lower cut-off frequencies.

// DAC output -6dB attentuation enable/disable
bool wm8960__enableDac6dbAttenuation();
bool wm8960__disableDac6dbAttentuation();

//////////////////////////////////////////////////////
////////////////////////////////////////////////////// OUTPUT mixers
//////////////////////////////////////////////////////

// What's connected to what? Oh so many options...
// LOMIX	Left Output Mixer
// ROMIX	Right Output Mixer
// OUT3MIX		Mono Output Mixer

// Enable/disable left and right output mixers
bool wm8960__enableLOMIX();
bool wm8960__disableLOMIX();
bool wm8960__enableROMIX();
bool wm8960__disableROMIX();
bool wm8960__enableOUT3MIX();
bool wm8960__disableOUT3MIX();

// Enable/disable audio path connections/vols to/from output mixers
// See datasheet page 35 for a nice image of all the connections.

bool wm8960__enableLI2LO();
bool wm8960__disableLI2LO();

// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
bool wm8960__setLI2LOVOL(uint8_t volume);

bool wm8960__enableLB2LO();
bool wm8960__disableLB2LO();

// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
bool wm8960__setLB2LOVOL(uint8_t volume);

bool wm8960__enableLD2LO();
bool wm8960__disableLD2LO();

bool wm8960__enableRI2RO();
bool wm8960__disableRI2RO();

// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
bool wm8960__setRI2ROVOL(uint8_t volume);

bool wm8960__enableRB2RO();
bool wm8960__disableRB2RO();

// 0-7, 0 = 0dB, ... 3dB steps ... 7 = -21dB
bool wm8960__setRB2ROVOL(uint8_t volume);

bool wm8960__enableRD2RO();
bool wm8960__disableRD2RO();

// Mono Output mixer.
// Note, for capless HPs, we'll want this to output a buffered VMID.
// To do this, we need to disable both of these connections.
bool wm8960__enableLI2MO();
bool wm8960__disableLI2MO();
bool wm8960__enableRI2MO();
bool wm8960__disableRI2MO();

// This will disable both connections, thus enable VMID on OUT3. Note,
// to enable VMID, you also need to enable OUT3 in the
// WM8960_REG_PWR_MGMT_2 [1]
bool wm8960__enableOUT3asVMID();

// Enables VMID in the WM8960_REG_PWR_MGMT_1 register, and set's it to
// playback/record settings of 2*50Kohm.
bool wm8960__enableVMID();
bool wm8960__disableVMID();
bool wm8960__setVMID(uint8_t setting);

/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Headphones
/////////////////////////////////////////////////////////

// Enable and disable headphones (mute)
bool wm8960__enableHeadphones();
bool wm8960__disableHeadphones();
bool wm8960__enableRightHeadphone();
bool wm8960__disableRightHeadphone();
bool wm8960__enableLeftHeadphone();
bool wm8960__disableLeftHeadphone();

bool wm8960__enableHeadphoneStandby();
bool wm8960__disableHeadphoneStandby();

// Set headphone volume
// Although you can control each headphone output independently, here
// we are going to assume you want both left and right to do the same
// thing.

// Valid inputs are 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ...
// 127 = +6dB
bool wm8960__setHeadphoneVolume(uint8_t volume);
// Updates both left and right channels
// Handles the OUT1VU (volume update) bit control, so that it happens at
// the same time on both channels. Note, we must also make sure that the
// outputs are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right Headphone outputs
bool wm8960__enableHeadphoneZeroCross();
bool wm8960__disableHeadphoneZeroCross();

// Set headphone volume dB
// Sets the volume of the headphone output buffer amp to a speicified
// dB value passed in as a float argument.
// Valid dB settings are -74.0 up to +6.0
// User input will be rounded to nearest whole integer
// -74 (or lower) = MUTE
// -73 = -73dB (MIN)
// ... 1dB steps ...
// 0 = 0dB
// ... 1dB steps ...
// 6 = +6dB  (MAX)
bool wm8960__setHeadphoneVolumeDB(float dB);


/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// Speakers
/////////////////////////////////////////////////////////

// Enable and disable speakers (mute)
bool wm8960__enableSpeakers();
bool wm8960__disableSpeakers();
bool wm8960__enableRightSpeaker();
bool wm8960__disableRightSpeaker();
bool wm8960__enableLeftSpeaker();
bool wm8960__disableLeftSpeaker();

// Set Speaker output volume
// Although you can control each Speaker output independently, here we
// are going to assume you want both left and right to do the same thing.
// Valid inputs are 47-127. 0-47 = mute, 48 = -73dB ... 1dB steps ...
// 127 = +6dB

bool wm8960__setSpeakerVolume(uint8_t volume);
// Updates both left and right channels
// Handles the SPKVU (volume update) bit control, so that it happens at
// the same time on both channels. Note, we must also make sure that the
// outputs are enabled in the WM8960_REG_PWR_MGMT_2 [4:3]
// And the class D control reg WM8960_REG_CLASS_D_CONTROL_1 [7:6]

bool wm8960__setSpeakerVolumeDB(float dB);

// Zero Cross prevents zipper sounds on volume changes
// Sets both left and right Speaker outputs
bool wm8960__enableSpeakerZeroCross();
bool wm8960__disableSpeakerZeroCross();

// DC and AC gain - allows signal to be higher than the DACs swing
// (use only if your SPKVDD is high enough to handle a larger signal)
// Valid inputs are 0-5
// 0 = +0dB (1.0x boost) ... up to ... 5 = +5.1dB (1.8x boost)
bool wm8960__setSpeakerDcGain(uint8_t gain);
bool wm8960__setSpeakerAcGain(uint8_t gain);

//////////////////////////////////////
////////////////////////////////////// Digital audio interface control
//////////////////////////////////////

// Defaults to I2S, peripheral-mode, 24-bit word length

// Loopback
// When enabled, the output data from the ADC audio interface is fed
// directly into the DAC data input.
bool wm8960__enableLoopBack();
bool wm8960__disableLoopBack();

///////////////////////////////////////////////////////
/////////////////////////////////////////////////////// Clock controls
///////////////////////////////////////////////////////

// Getting the Frequency of SampleRate as we wish
// Our MCLK (an external clock on the SFE breakout board) is 24.0MHz.
// According to table 40 (DS pg 58), we want SYSCLK to be 11.2896 for a
// SR of 44.1KHz. To get that Desired Output (SYSCLK), we need the
// following settings on the PLL stuff:
// As found on table 45 (ds pg 61).
// PRESCALE DIVIDE (PLLPRESCALE): 2
// POSTSCALE DVIDE (SYSCLKDIV[1:0]): 2
// FIXED POST-DIVIDE: 4
// R: 7.5264
// N: 7h
// K: 86C226h

// Example at bottom of table 46, shows that we should be in fractional
// mode for a 44.1KHz.

// In terms of registers, this is what we want for 44.1KHz
// PLLEN=1			(PLL enable)
// PLLPRESCALE=1	(divide by 2) *This get's us from MCLK (24MHz) down
// to 12MHZ for F2
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
bool wm8960__enablePLL();
bool wm8960__disablePLL();

// Valid options are WM8960_PLLPRESCALE_DIV_1, WM8960_PLLPRESCALE_DIV_2
bool wm8960__setPLLPRESCALE(bool div);

bool wm8960__setPLLN(uint8_t n);

// Send each nibble of 24-bit value for value K
bool wm8960__setPLLK(uint8_t one, uint8_t two, uint8_t three);

bool wm8960__setSMD(bool mode); // 0=integer, 1=fractional
bool wm8960__setCLKSEL(bool sel); // 0=MCLK, 1=PLL_output

// (0=divide by 1), (2=div by 2) *1 and 3 are "reserved"
bool wm8960__setSYSCLKDIV(uint8_t div);

// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
bool wm8960__setADCDIV(uint8_t div);

// 000 = SYSCLK / (1.0*256). See ds pg 57 for other options
bool wm8960__setDACDIV(uint8_t div);

// 0100 (4) = sufficiently high for 24bit, div by 4 allows for max word
// length of 32bit
bool wm8960__setBCLKDIV(uint8_t div);

// Class D amp, 111= SYSCLK/16, so 11.2896MHz/16 = 705.6KHz
bool wm8960__setDCLKDIV(uint8_t div);

// Set LR clock to be the same for ADC & DAC (needed for loopback mode)
bool wm8960__setALRCGPIO();

bool wm8960__enableMasterMode();
bool wm8960__enablePeripheralMode();

bool wm8960__setWL(uint8_t word_length);

bool wm8960__setLRP(bool polarity);

bool wm8960__setALRSWAP(bool swap);

bool wm8960__setVROI(bool setting);

bool wm8960__setVSEL(uint8_t setting);

#endif /* SRC_WM8960_DRIVER_WM8960_DRIVER_H_ */
