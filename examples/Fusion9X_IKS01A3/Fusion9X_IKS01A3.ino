#if (__CORTEX_M == 0U)
#include "motion_fx_cm0p.h"
#else
#include "motion_fx.h"
#endif
#include "LSM6DSOSensor.h"
#include "LIS2MDLSensor.h"

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif

#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS  500.0f

#define STATE_SIZE                      (size_t)(2432)

#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)
#define GBIAS_MAG_TH_SC                 (2.0f*0.001500f)

#define DECIMATION                      1U

/* Private variables ---------------------------------------------------------*/
#if !(__CORTEX_M == 0U)
static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;
static uint8_t mfxstate[STATE_SIZE];
#endif

static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

char LibVersion[35];
int LibVersionLen;

static volatile uint32_t TimeStamp = 0;

int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];
int32_t MagOffset[3];

LSM6DSOSensor AccGyr(&Wire);
LIS2MDLSensor Mag(&Wire);

HardwareTimer *MyTim;

volatile uint8_t fusion_flag;

bool mag_calibrated = false;

void fusion_update(void)
{
  fusion_flag = 1;
}

void setup() {
  /* Initialize Serial */
  Serial.begin(115200);
  while (!Serial) yield();

  /* Initialize LED */
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialize I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  /* Start communication with IMU */
  AccGyr.begin();
  AccGyr.Set_X_ODR((float)ALGO_FREQ);
  AccGyr.Set_X_FS(4);
  AccGyr.Set_G_ODR((float)ALGO_FREQ);
  AccGyr.Set_G_FS(2000);
  AccGyr.Enable_X();
  AccGyr.Enable_G();
  Mag.begin();
  Mag.SetOutputDataRate((float)ALGO_FREQ);
  Mag.Enable();
  delay(10);

  /* Initialize sensor fusion library */
#if (__CORTEX_M == 0U)
  MotionFX_CM0P_initialize(MFX_CM0P_MCU_STM32);
  MotionFX_CM0P_setOrientation("seu", "seu", "neu");
  MotionFX_CM0P_enable_gbias(MFX_CM0P_ENGINE_ENABLE);
  MotionFX_CM0P_enable_euler(MFX_CM0P_ENGINE_ENABLE);
  MotionFX_CM0P_enable_6X(MFX_CM0P_ENGINE_DISABLE);
  MotionFX_CM0P_enable_9X(MFX_CM0P_ENGINE_ENABLE);

  /* OPTIONAL */
  /* Get library version */
  LibVersionLen = (int)MotionFX_CM0P_GetLibVersion(LibVersion);

  /* Enable magnetometer calibration */
  MotionFX_CM0P_MagCal_init(ALGO_PERIOD, 1);
#else
  MotionFX_initialize((MFXState_t *)mfxstate);

  MotionFX_getKnobs(mfxstate, ipKnobs);

  ipKnobs->acc_orientation[0] = 's';
  ipKnobs->acc_orientation[1] = 'e';
  ipKnobs->acc_orientation[2] = 'u';
  ipKnobs->gyro_orientation[0] = 's';
  ipKnobs->gyro_orientation[1] = 'e';
  ipKnobs->gyro_orientation[2] = 'u';
  ipKnobs->mag_orientation[0] = 'n';
  ipKnobs->mag_orientation[1] = 'e';
  ipKnobs->mag_orientation[2] = 'u';

  ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
  ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
  ipKnobs->gbias_mag_th_sc = GBIAS_MAG_TH_SC;

  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->modx = DECIMATION;

  MotionFX_setKnobs(mfxstate, ipKnobs);
  MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);

  /* OPTIONAL */
  /* Get library version */
  LibVersionLen = (int)MotionFX_GetLibVersion(LibVersion);

  /* Enable magnetometer calibration */
  MotionFX_MagCal_init(ALGO_PERIOD, 1);
#endif

  Serial.println("Please, slowly rotate the device in a figure 8 pattern in space to calibrate the magnetometer...");

  MyTim = new HardwareTimer(TIM3);
  MyTim->setOverflow(ALGO_FREQ, HERTZ_FORMAT);
  MyTim->attachInterrupt(fusion_update);
  MyTim->resume();
}

void loop() {
  if(!mag_calibrated)
  {
    if(fusion_flag)
    {
      float ans_float;
#if (__CORTEX_M == 0U)
      MFX_CM0P_MagCal_input_t mag_data_in;
      MFX_CM0P_MagCal_output_t mag_data_out;
#else
      MFX_MagCal_input_t mag_data_in;
      MFX_MagCal_output_t mag_data_out;
#endif
      fusion_flag = 0;
      Mag.GetAxes(magnetometer);
#if (__CORTEX_M == 0U)
      mag_data_in.Mag[0] = (float)magnetometer[0] * FROM_MGAUSS_TO_UT50;
      mag_data_in.Mag[1] = (float)magnetometer[1] * FROM_MGAUSS_TO_UT50;
      mag_data_in.Mag[2] = (float)magnetometer[2] * FROM_MGAUSS_TO_UT50;
      MotionFX_CM0P_MagCal_run(&mag_data_in);
      MotionFX_CM0P_MagCal_getParams(&mag_data_out);

      if (mag_data_out.CalQuality == MFX_CM0P_CALQSTATUSBEST)
      {
        mag_calibrated = true;
        ans_float = (mag_data_out.HI_Bias[0] * FROM_UT50_TO_MGAUSS);
        MagOffset[0] = (int32_t)ans_float;
        ans_float = (mag_data_out.HI_Bias[1] * FROM_UT50_TO_MGAUSS);
        MagOffset[1] = (int32_t)ans_float;
        ans_float = (mag_data_out.HI_Bias[2] * FROM_UT50_TO_MGAUSS);
        MagOffset[2] = (int32_t)ans_float;
        /* Disable magnetometer calibration */
        MotionFX_CM0P_MagCal_init(ALGO_PERIOD, 0);
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Magnetomer calibration done!");
      }
#else
      mag_data_in.mag[0] = (float)magnetometer[0] * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1] = (float)magnetometer[1] * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2] = (float)magnetometer[2] * FROM_MGAUSS_TO_UT50;
      mag_data_in.time_stamp = (int)TimeStamp;

      TimeStamp += (uint32_t)ALGO_PERIOD;

      MotionFX_MagCal_run(&mag_data_in);
      MotionFX_MagCal_getParams(&mag_data_out);

      if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
      {
        mag_calibrated = true;
        ans_float = (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MagOffset[0] = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MagOffset[1] = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        MagOffset[2] = (int32_t)ans_float;
        /* Disable magnetometer calibration */
        MotionFX_MagCal_init(ALGO_PERIOD, 0);
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Magnetomer calibration done!");
      }
#endif
    }
  } else
  {
    if(fusion_flag)
    {
#if (__CORTEX_M == 0U)
      MFX_CM0P_input_t data_in;
      MFX_CM0P_output_t data_out;
#else
      MFX_input_t data_in;
      MFX_output_t data_out;
#endif
      float delta_time = MOTION_FX_ENGINE_DELTATIME;
      fusion_flag = 0;
      AccGyr.Get_X_Axes(accelerometer);
      AccGyr.Get_G_Axes(gyroscope);
      Mag.GetAxes(magnetometer);

      /* Convert angular velocity from [mdps] to [dps] */
      data_in.gyro[0] = (float)gyroscope[0] * FROM_MDPS_TO_DPS;
      data_in.gyro[1] = (float)gyroscope[1] * FROM_MDPS_TO_DPS;
      data_in.gyro[2] = (float)gyroscope[2] * FROM_MDPS_TO_DPS;

      /* Convert acceleration from [mg] to [g] */
      data_in.acc[0] = (float)accelerometer[0] * FROM_MG_TO_G;
      data_in.acc[1] = (float)accelerometer[1] * FROM_MG_TO_G;
      data_in.acc[2] = (float)accelerometer[2] * FROM_MG_TO_G;

      /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
      data_in.mag[0] = (float)(magnetometer[0] - MagOffset[0]) * FROM_MGAUSS_TO_UT50;
      data_in.mag[1] = (float)(magnetometer[1] - MagOffset[1]) * FROM_MGAUSS_TO_UT50;
      data_in.mag[2] = (float)(magnetometer[2] - MagOffset[2]) * FROM_MGAUSS_TO_UT50;

      if (discardedCount == sampleToDiscard)
      {
#if (__CORTEX_M == 0U)
        MotionFX_CM0P_update(&data_out, &data_in, delta_time);
#else
        MotionFX_propagate(mfxstate, &data_out, &data_in, &delta_time);
        MotionFX_update(mfxstate, &data_out, &data_in, &delta_time, NULL);
#endif

#if (__CORTEX_M == 0U)
        Serial.print("Yaw: ");
        Serial.print(data_out.rotation_9X[0]);
        Serial.print("  Pitch: ");
        Serial.print(data_out.rotation_9X[1]);
        Serial.print("  Roll: ");
        Serial.print(data_out.rotation_9X[2]);
        Serial.println("");
#else
        Serial.print("Yaw: ");
        Serial.print(data_out.rotation[0]);
        Serial.print("  Pitch: ");
        Serial.print(data_out.rotation[1]);
        Serial.print("  Roll: ");
        Serial.print(data_out.rotation[2]);
        Serial.println("");
#endif
      }
      else
      {
        discardedCount++;
      }
    }
  }
}
