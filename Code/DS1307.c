///////////////////////////////////////////////////////////////////////////
////                                                                   ////
////                             DS1307.c                              ////
////                                                                   ////
////                      Driver for CCS C compiler                    ////
////                                                                   ////
////     Driver for Maxim DS1307 serial I2C real-time clock (RTC).     ////
////                                                                   ////
///////////////////////////////////////////////////////////////////////////
////                                                                   ////
////                     https://simple-circuit.com/                   ////
////                                                                   ////
///////////////////////////////////////////////////////////////////////////


#if defined DS1307_I2C_NO_STREAM
  #define RTC_I2C_START()   i2c_start()
  #define RTC_I2C_STOP()    i2c_stop()
  #define RTC_I2C_WRITE(x)  i2c_write(x)
  #define RTC_I2C_READ(x)   i2c_read(x)
#elif defined DS1307_I2C_STREAM
  #define RTC_I2C_START()   i2c_start(DS1307_I2C_STREAM)
  #define RTC_I2C_STOP()    i2c_stop(DS1307_I2C_STREAM)
  #define RTC_I2C_WRITE(x)  i2c_write(DS1307_I2C_STREAM, x)
  #define RTC_I2C_READ(x)   i2c_read(DS1307_I2C_STREAM, x)
#else
  #define RTC_I2C_START()   i2c_start(DS1307_STREAM)
  #define RTC_I2C_STOP()    i2c_stop(DS1307_STREAM)
  #define RTC_I2C_WRITE(x)  i2c_write(DS1307_STREAM, x)
  #define RTC_I2C_READ(x)   i2c_read(DS1307_STREAM, x)
#endif

#include <stdint.h>

#define DS1307_ADDRESS      0xD0
#define DS1307_REG_SECONDS  0x00
#define DS1307_REG_CONTROL  0x07

enum SQWE
{
  OUT_OFF = 0x00,
  OUT_ON  = 0x80,
  OUT_1Hz = 0x10,
  OUT_4096Hz  = 0x11,
  OUT_8192Hz  = 0x12,
  OUT_32768Hz = 0x13
};

typedef enum
{
  SUNDAY = 1,
  MONDAY,
  TUESDAY,
  WEDNESDAY,
  THURSDAY,
  FRIDAY,
  SATURDAY
} RTC_DOW;

typedef enum
{
  JANUARY = 1,
  FEBRUARY,
  MARCH,
  APRIL,
  MAY,
  JUNE,
  JULY,
  AUGUST,
  SEPTEMBER,
  OCTOBER,
  NOVEMBER,
  DECEMBER
} RTC_Month;

typedef struct rtc_tm
{
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  RTC_DOW dow;
  uint8_t day;
  RTC_Month month;
  uint8_t year;
} RTC_Time;

///////////////////////// All Functions /////////////////////////
                                                               //
uint8_t bcd_to_decimal(uint8_t number);                        //
uint8_t decimal_to_bcd(uint8_t number);                        //
void RTC_Set(RTC_Time *time_t);                                //
RTC_Time *RTC_Get();                                           //
void OSC_Enable();                                             //
void OSC_Disable();                                            //
uint8_t RTC_Read_Reg(uint8_t reg_address);                     //
void RTC_Write_Reg(uint8_t reg_address, uint8_t reg_value);    //
void SQWE_Set(SQWE pin_out);                                   //
                                                               //
/////////////////////////////////////////////////////////////////

// converts BCD to decimal
uint8_t bcd_to_decimal(uint8_t number)
{
  return ( (number >> 4) * 10 + (number & 0x0F) );
}

// converts decimal to BCD
uint8_t decimal_to_bcd(uint8_t number)
{
  return ( ((number / 10) << 4) + (number % 10) );
}

// sets time and date
void RTC_Set(RTC_Time *time_t)
{
  uint8_t sec_reg = RTC_Read_Reg(DS1307_REG_SECONDS);

  // convert decimal to BCD
  time_t->day     = decimal_to_bcd(time_t->day);
  time_t->month   = decimal_to_bcd(time_t->month);
  time_t->year    = decimal_to_bcd(time_t->year);
  time_t->hours   = decimal_to_bcd(time_t->hours);
  time_t->minutes = decimal_to_bcd(time_t->minutes);
  time_t->seconds = decimal_to_bcd(time_t->seconds);
  // end conversion
  
  // test if oscillator is disabled (CH bit = 1)
  if(sec_reg & 0x80)
    time_t->seconds |= 0x80;

  // write data to the RTC chip
  RTC_I2C_START();
  RTC_I2C_WRITE(DS1307_ADDRESS);
  RTC_I2C_WRITE(DS1307_REG_SECONDS);
  RTC_I2C_WRITE(time_t->seconds);
  RTC_I2C_WRITE(time_t->minutes);
  RTC_I2C_WRITE(time_t->hours);
  RTC_I2C_WRITE(time_t->dow);
  RTC_I2C_WRITE(time_t->day);
  RTC_I2C_WRITE(time_t->month);
  RTC_I2C_WRITE(time_t->year);
  RTC_I2C_STOP();
}

RTC_Time c_time;

// reads time and date
RTC_Time *RTC_Get()
{
  RTC_I2C_START();
  RTC_I2C_WRITE(DS1307_ADDRESS);
  RTC_I2C_WRITE(DS1307_REG_SECONDS);
  RTC_I2C_START();
  RTC_I2C_WRITE(DS1307_ADDRESS | 0x01);
  c_time.seconds = RTC_I2C_READ(1) & 0x7F;
  c_time.minutes = RTC_I2C_READ(1);
  c_time.hours   = RTC_I2C_READ(1);
  c_time.dow   = RTC_I2C_READ(1);
  c_time.day   = RTC_I2C_READ(1);
  c_time.month = RTC_I2C_READ(1);
  c_time.year  = RTC_I2C_READ(0);
  RTC_I2C_STOP();

  // convert BCD to decimal
  c_time.seconds = bcd_to_decimal(c_time.seconds);
  c_time.minutes = bcd_to_decimal(c_time.minutes);
  c_time.hours   = bcd_to_decimal(c_time.hours);
  c_time.day     = bcd_to_decimal(c_time.day);
  c_time.month   = bcd_to_decimal(c_time.month);
  c_time.year    = bcd_to_decimal(c_time.year);
  // end conversion

  return &c_time;
}

// writes 'reg_value' to register of address 'reg_address'
void RTC_Write_Reg(uint8_t reg_address, uint8_t reg_value)
{
  RTC_I2C_START();
  RTC_I2C_WRITE(DS1307_ADDRESS);
  RTC_I2C_WRITE(reg_address);
  RTC_I2C_WRITE(reg_value);
  RTC_I2C_STOP();
}

// returns the value stored in register of address 'reg_address'
uint8_t RTC_Read_Reg(uint8_t reg_address)
{
  uint8_t reg_data;

  RTC_I2C_START();
  RTC_I2C_WRITE(DS1307_ADDRESS);
  RTC_I2C_WRITE(reg_address);
  RTC_I2C_START();
  RTC_I2C_WRITE(DS1307_ADDRESS | 0x01);
  reg_data = RTC_I2C_READ(0);
  RTC_I2C_STOP();

  return reg_data;
}

// sets SQWE pin output
void SQWE_Set(SQWE pin_out)
{
  RTC_I2C_START();
  RTC_I2C_WRITE(DS1307_ADDRESS);
  RTC_I2C_WRITE(DS1307_REG_CONTROL);
  RTC_I2C_WRITE(pin_out);
  RTC_I2C_STOP();
}

// enables RTC oscillator
void OSC_Enable()
{
  uint8_t sec_reg = RTC_Read_Reg(DS1307_REG_SECONDS);
  sec_reg &= 0x7F;
  RTC_Write_Reg(DS1307_REG_SECONDS, sec_reg);
}

// disables RTC oscillator
void OSC_Disable()
{
  uint8_t sec_reg = RTC_Read_Reg(DS1307_REG_SECONDS);
  sec_reg |= 0x80;
  RTC_Write_Reg(DS1307_REG_SECONDS, sec_reg);
}
