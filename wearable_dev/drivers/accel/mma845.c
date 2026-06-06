#include "mma845.h"

#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* m_twi declared via main.h (included through mma845.h) */
uint8_t m_deviceAddress = 0x1C;
accel_result_t g_accel = {0};

static volatile bool s_fall_fired = false;

static void fall_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin; (void)action;
    s_fall_fired = true;
}

/* Configure FF_MT freefall detection and route interrupt to INT1 */
static void mma8452q_setup_freefall(void)
{
    /* Freefall on X+Y+Z, latch enabled, OAE=0 (freefall) */
    MMA8452Q_writeRegister(FF_MT_CFG, 0xB8);
    /* Threshold: 3 × 0.063g ≈ 0.19g (at 2g full-scale) */
    MMA8452Q_writeRegister(FF_MT_THS, 0x03);
    /* Debounce: 6 × 10ms = 60ms at ODR_100 */
    MMA8452Q_writeRegister(FF_MT_COUNT, 0x06);
    /* Enable FF_MT interrupt (bit 2) in CTRL_REG4 */
    uint8_t reg4 = 0;
    MMA8452Q_readRegister(CTRL_REG4, &reg4);
    MMA8452Q_writeRegister(CTRL_REG4, reg4 | 0x04);
    /* Route FF_MT to INT1 (bit 2) in CTRL_REG5 */
    uint8_t reg5 = 0;
    MMA8452Q_readRegister(CTRL_REG5, &reg5);
    MMA8452Q_writeRegister(CTRL_REG5, reg5 | 0x04);
}
MMA8452Q_Scale m_scale;
static int16_t x, y, z;
static float cx, cy, cz;
static float current_accelerator = 0;
static uint32_t s_i2c_error_count = 0;

// BEGIN INITIALIZATION (New Implementation of Init)
// 	This will be used instead of init in future sketches
// 	to match Arudino guidelines. We will maintain init
// 	for backwards compatability purposes.
bool MMA8452Q_init(uint8_t deviceAddress, MMA8452Q_Scale fsr, MMA8452Q_ODR odr)
{
    //m_twi = twi;
   
    m_scale = fsr;

    uint8_t tmp;
    if ((!MMA8452Q_readRegister(WHO_AM_I, &tmp))|| ( tmp != 0x2A))
	{
                NRF_LOG_INFO("tmp = %02x", tmp);
		NRF_LOG_INFO("Init false");
		NRF_LOG_FLUSH();
		return false;
	}

    MMA8452Q_standby(); // Must be in standby to change registers

    MMA8452Q_setScale(m_scale);  // Set up accelerometer scale
    MMA8452Q_setDataRate(odr); // Set up output data rate
    MMA8452Q_setupPL();		  // Set up portrait/landscape detection

    // Multiply parameter by 0.0625g to calculate threshold.
    MMA8452Q_setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g

    mma8452q_setup_freefall(); // Configure freefall detection + INT1 routing

    MMA8452Q_setActive(); // Set to active to start reading
    return true;
}

// GET FUNCTIONS FOR RAW ACCELERATION DATA
// Returns raw X acceleration data
int16_t MMA8452Q_getX()
{
    uint8_t rawData[2];
    if(MMA8452Q_readRegisters(OUT_X_MSB, rawData, 2) == false){
        NRF_LOG_INFO("Fail to read OUT_X_MSB");
        return 0;
    } // Read the X data into a data array
    return (((int16_t)(rawData[0] << 8 | rawData[1])) >> 4);
}

// Returns raw Y acceleration data
int16_t MMA8452Q_getY()
{
    uint8_t rawData[2];
    if(MMA8452Q_readRegisters(OUT_Y_MSB, rawData, 2) == false){
        NRF_LOG_INFO("Fail to read OUT_Y_MSB");
        return 0;
    }; // Read the Y data into a data array
    return (((int16_t)(rawData[0] << 8 | rawData[1])) >> 4);
}

// Returns raw Z acceleration data
int16_t MMA8452Q_getZ()
{
    uint8_t rawData[2];
    if( MMA8452Q_readRegisters(OUT_Z_MSB, rawData, 2) == 0){
        NRF_LOG_INFO("Fail to read OUT_Z_MSB");
        return 0;
    } // Read the Z data into a data array

    return (((int16_t)(rawData[0] << 8 | rawData[1])) >> 4);
}

// GET FUNCTIONS FOR CALCULATED ACCELERATION DATA
// Returns calculated X acceleration data
float MMA8452Q_getCalculatedX()
{
    x = MMA8452Q_getX();
    return (float)x / (float)(1 << 11) * (float)(m_scale);
}

// Returns calculated Y acceleration data
float MMA8452Q_getCalculatedY()
{
    y = MMA8452Q_getY();
    return ((float)y / (float)(1 << 11) * (float)(m_scale));
}

// Returns calculated Z acceleration data
float MMA8452Q_getCalculatedZ()
{
    z = MMA8452Q_getZ();
    return ((float)z / (float)(1 << 11) * (float)(m_scale));
}

// READ ACCELERATION DATA
//  This function will read the acceleration values from the MMA8452Q. After
//	reading, it will update two triplets of variables:
//		* int's x, y, and z will store the signed 12-bit values read out
//		  of the acceleromter.
//		* floats cx, cy, and cz will store the calculated acceleration from
//		  those 12-bit values. These variables are in units of g's.
void MMA8452Q_read()
{
    uint8_t rawData[6]; // x/y/z accel register data stored here
    MMA8452Q_readRegisters(OUT_X_MSB, rawData, 6); // Read the six raw data registers into data array

    x = ((int16_t)(rawData[0] << 8 | rawData[1])) >> 4;
    y = ((int16_t)(rawData[2] << 8 | rawData[3])) >> 4;
    z = ((int16_t)(rawData[4] << 8 | rawData[5])) >> 4;
    cx = (float)x / (float)(1 << 11) * (float)(m_scale);
    cy = (float)y / (float)(1 << 11) * (float)(m_scale);
    cz = (float)z / (float)(1 << 11) * (float)(m_scale);

    g_accel.ax        = cx;
    g_accel.ay        = cy;
    g_accel.az        = cz;
    g_accel.magnitude = sqrtf(cx*cx + cy*cy + cz*cz) * 9.80665f;  /* m/s² */
    g_accel.new_data  = true;

    if (s_fall_fired)
    {
        s_fall_fired = false;
        uint8_t src = 0;
        MMA8452Q_readRegister(FF_MT_SRC, &src);  /* read clears the latch */
        g_accel.fall_detected = true;
        NRF_LOG_INFO("Fall detected! FF_MT_SRC=0x%02x", src);
        NRF_LOG_FLUSH();
    }
}

// CHECK IF NEW DATA IS AVAILABLE
//	This function checks the status of the MMA8452Q to see if new data is availble.
//	returns 0 if no new data is present, or a 1 if new data is available.
uint8_t MMA8452Q_available()
{
    uint8_t tmp;
    MMA8452Q_readRegister(STATUS_MMA8452Q, &tmp);
    return ((tmp & 0x08) >> 3);
}

// SET FULL-SCALE RANGE
//	This function sets the full-scale range of the x, y, and z axis accelerometers.
//	Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
void MMA8452Q_setScale(MMA8452Q_Scale fsr)
{
    // Must be in standby mode to make changes!!!
    // Change to standby if currently in active state
    if (MMA8452Q_isActive() == true)
        MMA8452Q_standby();

    uint8_t cfg;
    if (!MMA8452Q_readRegister(XYZ_DATA_CFG, &cfg))
        return;
    cfg &= 0xFC;	   // Mask out scale bits
    cfg |= (fsr >> 2); // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
    MMA8452Q_writeRegister(XYZ_DATA_CFG, cfg);

    // Return to active state when done
    // Must be in active state to read data
    MMA8452Q_setActive();

    m_scale = fsr;
}

// SET THE OUTPUT DATA RATE
//	This function sets the output data rate of the MMA8452Q.
//	Possible values for the odr parameter are: ODR_800, ODR_400, ODR_200,
//	ODR_100, ODR_50, ODR_12, ODR_6, or ODR_1
void MMA8452Q_setDataRate(MMA8452Q_ODR odr)
{
    // Must be in standby mode to make changes!!!
    // Change to standby if currently in active state
    if (MMA8452Q_isActive() == true)
        MMA8452Q_standby();

    uint8_t ctrl;
    MMA8452Q_readRegister(CTRL_REG1, &ctrl);
    ctrl &= 0xC7; // Mask out data rate bits
    ctrl |= (odr << 3);
    MMA8452Q_writeRegister(CTRL_REG1, ctrl);

    // Return to active state when done
    // Must be in active state to read data
    MMA8452Q_setActive();
}

// SET UP TAP DETECTION
//	This function can set up tap detection on the x, y, and/or z axes.
//	The xThs, yThs, and zThs parameters serve two functions:
//		1. Enable tap detection on an axis. If the 7th bit is SET (0x80)
//			tap detection on that axis will be DISABLED.
//		2. Set tap g's threshold. The lower 7 bits will set the tap threshold
//			on that axis.
void MMA8452Q_setupTap(uint8_t xThs, uint8_t yThs, uint8_t zThs)
{
    // Must be in standby mode to make changes!!!
    // Change to standby if currently in active state
    if (MMA8452Q_isActive() == true)
        MMA8452Q_standby();

    // Set up single and double tap - 5 steps:
    // for more info check out this app note:
    // http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
    // Set the threshold - minimum required acceleration to cause a tap.
    uint8_t temp = 0;
    // If top bit ISN'T set
    if (!(xThs & 0x80)) {
        temp |= 0x3;					 // Enable taps on x
        MMA8452Q_writeRegister(PULSE_THSX, xThs); // x thresh
    }

    if (!(yThs & 0x80)) {
        temp |= 0xC;					 // Enable taps on y
        MMA8452Q_writeRegister(PULSE_THSY, yThs); // y thresh
    }

    if (!(zThs & 0x80)) {
        temp |= 0x30;					 // Enable taps on z
        MMA8452Q_writeRegister(PULSE_THSZ, zThs); // z thresh
    }
    // Set up single and/or double tap detection on each axis individually.
    MMA8452Q_writeRegister(PULSE_CFG, temp | 0x40);
    // Set the time limit - the maximum time that a tap can be above the thresh
    MMA8452Q_writeRegister(PULSE_TMLT, 0x30); // 30ms time limit at 800Hz odr
    // Set the pulse latency - the minimum required time between pulses
    MMA8452Q_writeRegister(PULSE_LTCY, 0xA0); // 200ms (at 800Hz odr) between taps min
    // Set the second pulse window - maximum allowed time between end of
    //	latency and start of second pulse
    MMA8452Q_writeRegister(PULSE_WIND, 0xFF); // 5. 318ms (max value) between taps max

    // Return to active state when done
    // Must be in active state to read data
    MMA8452Q_setActive();
}

// READ TAP STATUS
//	This function returns any taps read by the MMA8452Q. If the function
//	returns no new taps were detected. Otherwise the function will return the
//	lower 7 bits of the PULSE_SRC register.
uint8_t MMA8452Q_readTap()
{
    uint8_t tapStat;
    if (MMA8452Q_readRegister(PULSE_SRC, &tapStat)) {
        if (tapStat & 0x80) // Read EA bit to check if a interrupt was generated
            return tapStat & 0x7F;
        else
            return 0;
    }
    return 0;
}

// SET UP PORTRAIT/LANDSCAPE DETECTION
//	This function sets up portrait and landscape detection.
void MMA8452Q_setupPL()
{
    // Must be in standby mode to make changes!!!
    // Change to standby if currently in active state
    if (MMA8452Q_isActive())
        MMA8452Q_standby();

    // For more info check out this app note:
    //	http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
    // 1. Enable P/L
    uint8_t pl;
    MMA8452Q_readRegister(PL_CFG, &pl);
    MMA8452Q_writeRegister(PL_CFG, pl | 0x40); // Set PL_EN (enable)
    // 2. Set the debounce rate
    MMA8452Q_writeRegister(PL_COUNT, 0x50); // Debounce counter at 100ms (at 800 hz)

    // Return to active state when done
    // Must be in active state to read data
    MMA8452Q_setActive();
}

// READ PORTRAIT/LANDSCAPE STATUS
//	This function reads the portrait/landscape status register of the MMA8452Q.
//	It will return either PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L,
//	or LOCKOUT. LOCKOUT indicates that the sensor is in neither p or ls.
uint8_t MMA8452Q_readPL()
{
    uint8_t plStat;
    if (MMA8452Q_readRegister(PL_STATUS, &plStat)) {
        if (plStat & 0x40) // Z-tilt lockout
            return LOCKOUT;
        else // Otherwise return LAPO status
            return (plStat & 0x6) >> 1;
    }
    return 0;
}

// CHECK FOR ORIENTATION
bool MMA8452Q_isRight()
{
    return (MMA8452Q_readPL() == LANDSCAPE_R);
}

bool MMA8452Q_isLeft()
{
    return (MMA8452Q_readPL() == LANDSCAPE_L);
}

bool MMA8452Q_isUp()
{
    return (MMA8452Q_readPL() == PORTRAIT_U);
}

bool MMA8452Q_isDown()
{
    return (MMA8452Q_readPL() == PORTRAIT_D);
}

bool MMA8452Q_isFlat()
{
    if (MMA8452Q_readPL() == LOCKOUT)
        return true;
    return false;
}

// SET STANDBY MODE
//	Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Q_standby()
{
    uint8_t c;
    if (MMA8452Q_readRegister(CTRL_REG1, &c))
        MMA8452Q_writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// SET ACTIVE MODE
//	Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Q_setActive()
{
    uint8_t c;
    if (MMA8452Q_readRegister(CTRL_REG1, &c))
        MMA8452Q_writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// CHECK STATE (ACTIVE or STANDBY)
//	Returns true if in Active State, otherwise return false
bool MMA8452Q_isActive()
{
    uint8_t currentState;
    if (MMA8452Q_readRegister(SYSMOD, &currentState)) {
        return ((currentState & 0x03) == SYSMOD_STANDBY);
    }
    return false;
}

// WRITE A SINGLE REGISTER
// 	Write a single uint8_t of data to a register in the MMA8452Q.
bool MMA8452Q_writeRegister(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg, data};
    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_tx(&m_twi, m_deviceAddress, tx_buf, 2, false) != NRF_SUCCESS)
        return false;
    return twi_wait();
}

bool MMA8452Q_readRegister(uint8_t reg, uint8_t *dest)
{
    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_tx(&m_twi, m_deviceAddress, &reg, 1, true) != NRF_SUCCESS)
        return false;
    if (!twi_wait()) return false;

    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_rx(&m_twi, m_deviceAddress, dest, 1) != NRF_SUCCESS)
        return false;
    return twi_wait();
}

bool MMA8452Q_readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_tx(&m_twi, m_deviceAddress, &reg, 1, true) != NRF_SUCCESS)
        return false;
    if (!twi_wait()) return false;

    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_rx(&m_twi, m_deviceAddress, buffer, len) != NRF_SUCCESS)
        return false;
    return twi_wait();
}


float read_accelerator(void){
 if (MMA8452Q_available()){
          MMA8452Q_read();
          float ax = MMA8452Q_getCalculatedX();
          float ay = MMA8452Q_getCalculatedY();
          float az = MMA8452Q_getCalculatedZ();
          float ax_ms2 = ax * 9.80665f;
          float ay_ms2 = ay * 9.80665f;
          float az_ms2 = az * 9.80665f;
	 
   
          float sum_squares = (ax_ms2 * ax_ms2) + (ay_ms2 * ay_ms2) + (az_ms2 * az_ms2);
          float total_accel = sqrtf(sum_squares);

          return total_accel;
  }
  return 0.0f;  // No new data available
}

void set_current_accelerator(float current_acc){
    current_accelerator = current_acc;
}

float get_current_accelerator(void){
    return current_accelerator;
}

uint32_t MMA8452Q_get_i2c_error_count(void) {
    return s_i2c_error_count;
}

void mma8452q_alert_init(void)
{
    if (MMA8452Q_INT1_PIN == NRF_GPIO_PIN_NOT_CONNECTED) { return; }

    if (!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }

    /* INT1 is active-low — detect falling edge */
    nrf_drv_gpiote_in_config_t cfg = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    cfg.pull = NRF_GPIO_PIN_PULLUP;
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(MMA8452Q_INT1_PIN, &cfg, fall_handler));
    nrf_drv_gpiote_in_event_enable(MMA8452Q_INT1_PIN, true);
}