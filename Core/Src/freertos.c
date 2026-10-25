/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "board_config.h"
#include "battery_config.h"
#include "battery_monitor.h"
#include "chassis_controller.h"
#include "chassis_config.h"
#include "chassis_ground_test.h"
#include "chassis_motion_test.h"
#include "chassis_runtime.h"
#include "chassis_runtime_test.h"
#include "chassis_usb_protocol.h"
#include "debug_log.h"
#include "control_timing.h"
#include "motion_trace.h"
#include "emergency_stop.h"
#include "encoder_service.h"
#include "fault_manager.h"
#include "fault_injection_test.h"
#include "four_wheel_speed_control_test.h"
#include "imu_config.h"
#include "imu_bias_storage.h"
#include "imu_calibration_guard.h"
#include "imu_service.h"
#include "imu_static_test.h"
#include "imu_spi_identity_test.h"
#include "motor_characterization_test.h"
#include "motor_driver.h"
#include "motor_open_loop_test.h"
#include "motor_speed_control_test.h"
#include "usb_cdc_transport.h"
#include "watchdog_service.h"
#include "wheel_speed_estimator.h"
#if CHASSIS_CONTROL_REVISION >= 203
#include "chassis_yaw_feedback.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for systemInitTask */
osThreadId_t systemInitTaskHandle;
const osThreadAttr_t systemInitTask_attributes = {
  .name = "systemInitTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for safetyTask */
osThreadId_t safetyTaskHandle;
const osThreadAttr_t safetyTask_attributes = {
  .name = "safetyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for usbRxTask */
osThreadId_t usbRxTaskHandle;
const osThreadAttr_t usbRxTask_attributes = {
  .name = "usbRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for batteryTask */
osThreadId_t batteryTaskHandle;
const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSystemInitTask(void *argument);
void StartMotorTask(void *argument);
void StartSafetyTask(void *argument);
void StartImuTask(void *argument);
void StartUsbRxTask(void *argument);
void StartTelemetryTask(void *argument);
void StartBatteryTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationAssertHook(void)
{
  /* The motor shutdown must precede the permanent assert halt. */
  Board_EmergencyMotorShutdown();
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  Board_EmergencyMotorShutdown();
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
  Board_EmergencyMotorShutdown();
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  if (!DebugLog_Init())
  {
    Error_Handler();
  }
  DebugLog_PrintBootBanner();
  UsbCdcTransport_Init();
  ChassisUsbProtocol_Init(HAL_GetTick());
  FaultManager_Init(HAL_GetTick());
  FaultInjectionTest_Init();
  WatchdogService_Init(HAL_GetTick());
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of systemInitTask */
  systemInitTaskHandle = osThreadNew(StartSystemInitTask, NULL, &systemInitTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

  /* creation of safetyTask */
  safetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &safetyTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* creation of usbRxTask */
  usbRxTaskHandle = osThreadNew(StartUsbRxTask, NULL, &usbRxTask_attributes);

  /* creation of telemetryTask */
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);

  /* creation of batteryTask */
  batteryTaskHandle = osThreadNew(StartBatteryTask, NULL, &batteryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSystemInitTask */
/**
  * @brief  Function implementing the systemInitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSystemInitTask */
void StartSystemInitTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartSystemInitTask */
  UsbCdcTransport_EnableSofMonitor();
  FaultManager_MarkSystemInitComplete(HAL_GetTick());
  DEBUG_LOG_INFO("USB initialization completed");
  DEBUG_LOG_INFO("systemInitTask completed; exiting one-shot task");
  osThreadExit();
  /* USER CODE END StartSystemInitTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  if (!DebugLog_RegisterCurrentTaskNonBlocking()) { Error_Handler(); }
  uint32_t next_wake_tick;

  DEBUG_LOG_INFO("motorTask started");
  if (!MotorDriver_Init())
  {
    DEBUG_LOG_ERROR("Motor driver initialization failed");
    Error_Handler();
  }
  DEBUG_LOG_INFO("Motor driver ready: TIM8 PWM=20kHz duty=0, D24A STBY/PD10 disabled");

  if (!EncoderService_Init())
  {
    DEBUG_LOG_ERROR("Encoder initialization failed");
    Error_Handler();
  }
  DEBUG_LOG_INFO("Encoder counters started: LF=TIM1/16-bit LR=TIM2/32-bit RF=TIM3/16-bit RR=TIM4/16-bit; d/total forward-normalized");

  if (!WheelSpeedEstimator_Init(HAL_GetTick()))
  {
    DEBUG_LOG_ERROR("Wheel speed estimator initialization failed");
    Error_Handler();
  }
  DEBUG_LOG_INFO("Wheel speed ready: %lu ms sample, %lu ms LPF, %lu ms stop timeout",
                 (unsigned long)CHASSIS_SPEED_SAMPLE_PERIOD_MS,
                 (unsigned long)CHASSIS_SPEED_FILTER_TIME_CONSTANT_MS,
                 (unsigned long)CHASSIS_SPEED_STOP_TIMEOUT_MS);
  ChassisController_Init();
  DEBUG_LOG_INFO("Chassis controller ready: +vx=forward +wz=CCW-left, command timeout=250 ms");
  (void)ChassisRuntime_Init(HAL_GetTick());
  MotorOpenLoopTest_Init();
#if CHASSIS_CONTROL_V2
  ControlTiming_EnableCycleCounter();
#endif

  next_wake_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    ControlTiming_RecordStart(CONTROL_TIMING_MOTOR, osKernelGetTickCount());
#if CHASSIS_CONTROL_V2
    const uint32_t loop_cycles=ControlTiming_ReadCycles();
#endif
    if (FaultInjectionTest_ShouldReportHeartbeat(FAULT_MANAGER_TASK_MOTOR))
    {
      FaultManager_Heartbeat(FAULT_MANAGER_TASK_MOTOR, HAL_GetTick());
    }
    const uint32_t control_now_ms=HAL_GetTick();
#if CHASSIS_CONTROL_V2
    uint32_t section_cycles=ControlTiming_ReadCycles();
#endif
    EncoderService_Update();
    WheelSpeedEstimator_Update(control_now_ms);
#if CHASSIS_CONTROL_V2
    ControlTiming_RecordSection(CONTROL_SECTION_ACQUISITION,
      ControlTiming_ReadCycles()-section_cycles,SystemCoreClock);
    section_cycles=ControlTiming_ReadCycles();
#endif
#if CHASSIS_CONTROL_REVISION >= 203
    {
      static ImuServiceSnapshot yaw_imu;
      static WheelSpeedState yaw_wheels[ENCODER_WHEEL_COUNT];
      static MotorDriverStatus yaw_motor;
      const bool got_imu=ImuService_GetSnapshot(&yaw_imu);
      const bool valid_imu=got_imu && yaw_imu.valid &&
        yaw_imu.state==IMU_SERVICE_STATE_READY && yaw_imu.consecutive_errors==0U;
      const float accel_squared=yaw_imu.accel_x_m_s2*yaw_imu.accel_x_m_s2+
        yaw_imu.accel_y_m_s2*yaw_imu.accel_y_m_s2+yaw_imu.accel_z_m_s2*yaw_imu.accel_z_m_s2;
      bool stationary=valid_imu && isfinite(accel_squared) &&
        !ImuStaticTest_IsActive() && !MotorOpenLoopTest_IsActive() &&
        accel_squared>=81.0f && accel_squared<=116.64f &&
        MotorDriver_GetStatus(&yaw_motor) && !yaw_motor.standby_enabled &&
        WheelSpeedEstimator_GetSnapshot(yaw_wheels);
      if (stationary) {
        for (uint32_t w=0U;w<ENCODER_WHEEL_COUNT;++w) {
          if (!yaw_wheels[w].valid || yaw_wheels[w].quality_flags!=0U ||
              (uint32_t)(control_now_ms-yaw_wheels[w].sample_time_ms)>40U ||
              yaw_wheels[w].delta_counts!=0 ||
              !isfinite(yaw_wheels[w].filtered_counts_per_second) ||
              fabsf(yaw_wheels[w].filtered_counts_per_second)>5.0f) stationary=false;
        }
      }
      ChassisYawFeedback_Observe(control_now_ms,stationary,valid_imu,
        yaw_imu.angular_velocity_z_rad_s,yaw_imu.sample_time_ms,
        yaw_imu.sample_count,yaw_imu.acquisition_epoch);
    }
#endif
    ChassisRuntime_Process(control_now_ms,
                           MotorOpenLoopTest_IsActive() ||
                           ImuStaticTest_IsActive());
    MotorOpenLoopTest_Process();
#if CHASSIS_CONTROL_V2
    ControlTiming_RecordSection(CONTROL_SECTION_CONTROLLER,
      ControlTiming_ReadCycles()-section_cycles,SystemCoreClock);
    section_cycles=ControlTiming_ReadCycles();
#endif
    MotionTrace_Record(HAL_GetTick());
#if CHASSIS_CONTROL_V2
    ControlTiming_RecordSection(CONTROL_SECTION_TRACE,
      ControlTiming_ReadCycles()-section_cycles,SystemCoreClock);
    ControlTiming_RecordExecution(CONTROL_TIMING_MOTOR,
      ControlTiming_ReadCycles()-loop_cycles,SystemCoreClock);
#endif
    next_wake_tick = ControlTiming_NextWake(CONTROL_TIMING_MOTOR,
      next_wake_tick, osKernelGetTickCount(), CHASSIS_SPEED_SAMPLE_PERIOD_MS);
    (void)osDelayUntil(next_wake_tick);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the safetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyTask */
  if (!DebugLog_RegisterCurrentTaskNonBlocking()) { Error_Handler(); }
  uint32_t next_wake_tick;

  DEBUG_LOG_INFO("safetyTask started");
  next_wake_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    ControlTiming_RecordStart(CONTROL_TIMING_SAFETY, osKernelGetTickCount());
#if CHASSIS_CONTROL_V2
    const uint32_t loop_cycles=ControlTiming_ReadCycles();
#endif
    const uint32_t now_ms = HAL_GetTick();
    /* Single safetyTask instance: its private scratch snapshots need not
     * occupy the task's 1 KiB stack across every nested safety/log call.
     * No pointer escapes this task; successful getters overwrite all fields. */
    static ChassisRuntimeStatus runtime;
    static ChassisControllerStatus controller;
    static float health_measured_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT];

    if (FaultInjectionTest_ShouldReportHeartbeat(FAULT_MANAGER_TASK_SAFETY))
    {
      FaultManager_Heartbeat(FAULT_MANAGER_TASK_SAFETY, now_ms);
    }
    taskENTER_CRITICAL();
    const bool observed=ChassisRuntime_GetStatus(&runtime) &&
                        ChassisController_GetStatus(&controller);
    taskEXIT_CRITICAL();
    if (observed)
    {
      FaultInjectionTest_FilterMeasuredSpeed(
        controller.measured_counts_per_second,
        health_measured_counts_per_second);
#if CHASSIS_CONTROL_V2
      FaultManager_ObserveMotionWithModes(now_ms,
                                 runtime.state == CHASSIS_RUNTIME_STATE_ACTIVE,
                                 controller.targets.counts_per_second,
                                 health_measured_counts_per_second,
                                 controller.output_permille,controller.pivot_mask);
#else
      FaultManager_ObserveMotion(now_ms,
        runtime.state==CHASSIS_RUNTIME_STATE_ACTIVE,controller.targets.counts_per_second,
        health_measured_counts_per_second,controller.output_permille);
#endif
    }
    else
    {
      FaultManager_ObserveMotion(now_ms, false, NULL, NULL, NULL);
    }
    FaultManager_Process(now_ms);
    if (FaultManager_HasCriticalFault())
    {
      ChassisRuntime_LatchHealthFault(now_ms);
    }
    ChassisRuntime_SafetyProcess(now_ms);
    ChassisController_SafetyProcess(now_ms);
    MotorOpenLoopTest_SafetyProcess();
    if (EmergencyStop_TakeTripNotice())
    {
      DEBUG_LOG_ERROR("ESTOP latched: motor outputs disabled; MCU reset required");
    }
    WatchdogService_Process(now_ms);
#if CHASSIS_CONTROL_V2
    ControlTiming_RecordExecution(CONTROL_TIMING_SAFETY,
      ControlTiming_ReadCycles()-loop_cycles,SystemCoreClock);
#endif
    next_wake_tick = ControlTiming_NextWake(CONTROL_TIMING_SAFETY,
      next_wake_tick, osKernelGetTickCount(), 10U);
    (void)osDelayUntil(next_wake_tick);
  }
  /* USER CODE END StartSafetyTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  if (!DebugLog_RegisterCurrentTaskNonBlocking()) { Error_Handler(); }
#if !IMU_SPI_ID_PROBE_ONLY && IMU_CALIBRATION_ENABLED
  ImuServiceSnapshot imu_snapshot;
  bool bias_restore_attempted = false;
#endif
#if !IMU_USE_MPU6500_SPI
  uint32_t next_wake_tick;
#endif

  DEBUG_LOG_INFO("imuTask started");
  ImuStaticTest_Init();
#if IMU_SPI_ID_PROBE_ONLY
  ImuSpiIdentityTest_Init(HAL_GetTick());
#else
#if IMU_CALIBRATION_ENABLED
  ImuBiasStorage_Init();
#endif
  ImuService_Init(HAL_GetTick());
#endif
#if !IMU_USE_MPU6500_SPI
  next_wake_tick = osKernelGetTickCount();
#endif
  /* Infinite loop */
  for(;;)
  {
    const uint32_t now_ms = HAL_GetTick();

#if IMU_SPI_ID_PROBE_ONLY
    ImuSpiIdentityTest_Update(now_ms);
#else
    ImuService_Update(now_ms);
#if IMU_CALIBRATION_ENABLED
    if (ImuService_GetSnapshot(&imu_snapshot))
    {
      if ((!bias_restore_attempted) &&
          (imu_snapshot.state == IMU_SERVICE_STATE_READY))
      {
        int32_t bias_x_milli_raw;
        int32_t bias_y_milli_raw;
        int32_t bias_z_milli_raw;
        int16_t calibration_temp_centi_c;

        bias_restore_attempted = true;
        if (ImuBiasStorage_LoadMilliRaw(&bias_x_milli_raw,
                                        &bias_y_milli_raw,
                                        &bias_z_milli_raw,
                                &calibration_temp_centi_c))
        {
          if (ImuService_SetGyroBiasMilliRaw(bias_x_milli_raw,
                                              bias_y_milli_raw,
                                              bias_z_milli_raw))
          {
            ImuService_SetCalibrationTemperatureCentiC(
              calibration_temp_centi_c);
            DEBUG_LOG_INFO(
              "IMU gyro bias restored from Flash: milli_raw=[%ld,%ld,%ld] calibration_temp=%d.%02uC",
              (long)bias_x_milli_raw,
              (long)bias_y_milli_raw,
              (long)bias_z_milli_raw,
              (int)(calibration_temp_centi_c / 100),
              (unsigned int)((calibration_temp_centi_c < 0) ?
                -(calibration_temp_centi_c % 100) :
                 (calibration_temp_centi_c % 100)));
          }
          else
          {
            DEBUG_LOG_ERROR(
              "IMU gyro bias Flash record passed storage checks but service rejected it");
          }
        }
      }
      ImuStaticTest_ProcessSample(&imu_snapshot, HAL_GetTick());
      {
        int64_t stationary_wheel_counts[ENCODER_WHEEL_COUNT];
        const bool stationary_allowed =
          ImuCalibrationGuard_Read(stationary_wheel_counts, false);
        ImuService_UpdateStationaryBias(stationary_allowed,
                                        ImuStaticTest_IsActive(),
                                        HAL_GetTick());
      }
    }
#endif
#endif
    if (FaultInjectionTest_ShouldReportHeartbeat(FAULT_MANAGER_TASK_IMU))
    {
      FaultManager_Heartbeat(FAULT_MANAGER_TASK_IMU, now_ms);
    }

#if IMU_USE_MPU6500_SPI
    /* DRDY wakes this task immediately. A bounded wait keeps initialization,
     * missing-INT detection, calibration timeout and health heartbeat alive. */
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
#else
    next_wake_tick += IMU_SAMPLE_PERIOD_MS;
    if ((int32_t)(osKernelGetTickCount() - next_wake_tick) >= 0)
    {
      next_wake_tick = osKernelGetTickCount() + IMU_SAMPLE_PERIOD_MS;
    }
    (void)osDelayUntil(next_wake_tick);
#endif
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartUsbRxTask */
/**
* @brief Function implementing the usbRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbRxTask */
void StartUsbRxTask(void *argument)
{
  /* USER CODE BEGIN StartUsbRxTask */
  uint32_t next_wake_tick;

  DEBUG_LOG_INFO("usbRxTask started");
  next_wake_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    if (FaultInjectionTest_ShouldReportHeartbeat(FAULT_MANAGER_TASK_USB_RX))
    {
      FaultManager_Heartbeat(FAULT_MANAGER_TASK_USB_RX, HAL_GetTick());
    }
    ChassisUsbProtocol_Process(HAL_GetTick());
    next_wake_tick += 2U;
    (void)osDelayUntil(next_wake_tick);
  }
  /* USER CODE END StartUsbRxTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  EncoderState encoder_states[ENCODER_WHEEL_COUNT];
  WheelSpeedState speed_states[ENCODER_WHEEL_COUNT];
#if !IMU_SPI_ID_PROBE_ONLY
  ImuServiceSnapshot imu_snapshot;
#endif
  char encoder_totals[ENCODER_WHEEL_COUNT][DEBUG_LOG_INT64_TEXT_SIZE];
#if !IMU_SPI_ID_PROBE_ONLY
  uint32_t next_imu_log_ms;
#endif
  uint32_t next_wake_tick;

  DEBUG_LOG_INFO("telemetryTask started");
  uint32_t next_control_log_ms = HAL_GetTick() + 1000U;
#if !IMU_SPI_ID_PROBE_ONLY
  next_imu_log_ms = HAL_GetTick() + IMU_VALID_LOG_PERIOD_MS;
#endif
  next_wake_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    const uint32_t now_ms = HAL_GetTick();

    if (FaultInjectionTest_ShouldReportHeartbeat(
          FAULT_MANAGER_TASK_TELEMETRY))
    {
      FaultManager_Heartbeat(FAULT_MANAGER_TASK_TELEMETRY, now_ms);
    }
    /* USER-owned logging service: USB task never waits for polling UART.
     * Drain even when routine encoder diagnostics are suppressed. */
    DebugLog_ProcessDeferred(4U);
    if (MotionTrace_ProcessDump())
    {
      next_wake_tick = osKernelGetTickCount() + 200U;
      (void)osDelayUntil(next_wake_tick);
      continue;
    }
    if ((int32_t)(now_ms - next_control_log_ms) >= 0)
    {
      ControlTimingStatus motor_timing, safety_timing;
      (void)ControlTiming_GetStatus(CONTROL_TIMING_MOTOR, &motor_timing);
      (void)ControlTiming_GetStatus(CONTROL_TIMING_SAFETY, &safety_timing);
      DEBUG_LOG_INFO(
        "CTRL TIMING: motor_dt=%lu..%lu safety_dt=%lu..%lu overrun=%lu/%lu log_drop=%lu stack_free=%lu/%lu",
        (unsigned long)motor_timing.period_min_ms,
        (unsigned long)motor_timing.period_max_ms,
        (unsigned long)safety_timing.period_min_ms,
        (unsigned long)safety_timing.period_max_ms,
        (unsigned long)motor_timing.overrun_count,
        (unsigned long)safety_timing.overrun_count,
        (unsigned long)DebugLog_GetDeferredDroppedCount(),
        (unsigned long)osThreadGetStackSpace(motorTaskHandle),
        (unsigned long)osThreadGetStackSpace(safetyTaskHandle));
      next_control_log_ms = now_ms + 1000U;
#if CHASSIS_CONTROL_V2
      ControlSectionTiming acquisition,control,trace;
      (void)ControlTiming_GetSection(CONTROL_SECTION_ACQUISITION,&acquisition);
      (void)ControlTiming_GetSection(CONTROL_SECTION_CONTROLLER,&control);
      (void)ControlTiming_GetSection(CONTROL_SECTION_TRACE,&trace);
      DEBUG_LOG_INFO("CTRL EXEC us max: motor=%lu safety=%lu acquire=%lu control=%lu trace=%lu",
        (unsigned long)motor_timing.execution_max_us,(unsigned long)safety_timing.execution_max_us,
        (unsigned long)acquisition.max_us,(unsigned long)control.max_us,(unsigned long)trace.max_us);
      static ChassisRunMetrics metrics;
      static ChassisControllerStatus control_status;
#if CHASSIS_CONTROL_REVISION >= 202
      /* UART output above can span a motor update. Copy a coherent pair and
       * sample time after it; the loop-entry time is too old for this age. */
      uint32_t diagnostic_now_ms;
      taskENTER_CRITICAL();
      const bool diagnostic_valid=ChassisController_GetRunMetrics(&metrics) &&
                                  ChassisController_GetStatus(&control_status);
      diagnostic_now_ms=HAL_GetTick();
      taskEXIT_CRITICAL();
      if (diagnostic_valid) {
#else
      if (ChassisController_GetRunMetrics(&metrics) && ChassisController_GetStatus(&control_status)) {
#endif
        uint32_t rms[4]={0};
        for (uint32_t w=0U;w<4U;++w)
          if (metrics.wheel[w].samples!=0U)
            rms[w]=(uint32_t)sqrtf(metrics.wheel[w].squared_error_sum/(float)metrics.wheel[w].samples);
        DEBUG_LOG_INFO("CTRL V2 gen=%lu n=%lu feedback=%lu age=%lu mode=%lu scale_x1000=%lu rms=%lu/%lu/%lu/%lu zero_max_ms=%lu/%lu/%lu/%lu",
          (unsigned long)metrics.generation,(unsigned long)metrics.samples,
          (unsigned long)control_status.feedback_sequence,
#if CHASSIS_CONTROL_REVISION >= 202
          (unsigned long)(diagnostic_now_ms-control_status.feedback_time_ms),
#else
          (unsigned long)(now_ms-control_status.feedback_time_ms),
#endif
          (unsigned long)control_status.pivot_mask,(unsigned long)(control_status.targets.scale*1000),
          (unsigned long)rms[0],(unsigned long)rms[1],(unsigned long)rms[2],(unsigned long)rms[3],
          (unsigned long)metrics.wheel[0].longest_zero_net_count_ms,
          (unsigned long)metrics.wheel[1].longest_zero_net_count_ms,
          (unsigned long)metrics.wheel[2].longest_zero_net_count_ms,
          (unsigned long)metrics.wheel[3].longest_zero_net_count_ms);
      }
#if CHASSIS_CONTROL_REVISION >= 203
      {
        ChassisYawFeedback yaw={0}; YawRateControlState state={0};
        (void)ChassisYawFeedback_Get(&yaw); (void)ChassisRuntime_GetYawControl(&state);
        DEBUG_LOG_INFO("YAW3 ready=%u valid=%u bias_urad=%ld rate_mrad=%ld ref_mrad=%ld correction_mrad=%ld i_mrad=%ld warm=%lu inhibit=%u",
          (unsigned)yaw.ready,(unsigned)yaw.valid,(long)(yaw.bias_rad_s*1000000.0f),
          (long)(yaw.rate_rad_s*1000.0f),(long)(state.reference*1000.0f),
          (long)(state.correction*1000.0f),(long)(state.integral*1000.0f),
          (unsigned long)state.warm_ms,(unsigned)state.inhibited);
      }
#endif
#endif
    }
    if (MotorCharacterizationTest_IsActive() ||
        MotorSpeedControlTest_IsActive() ||
        FourWheelSpeedControlTest_IsActive() ||
        ChassisMotionTest_IsActive() ||
        ChassisGroundTest_IsActive() ||
        ChassisRuntimeTest_IsActive())
    {
      next_wake_tick += 200U;
      (void)osDelayUntil(next_wake_tick);
      continue;
    }

    if (EncoderService_GetSnapshot(encoder_states))
    {
      for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
      {
        DebugLog_FormatInt64(encoder_states[wheel].accumulated_count,
                             encoder_totals[wheel]);
      }

      DEBUG_LOG_INFO(
        "ENC L: LF raw=%lu d=%ld total=%s dir=%s | LR raw=%lu d=%ld total=%s dir=%s",
        (unsigned long)encoder_states[ENCODER_WHEEL_LF].raw_count,
        (long)encoder_states[ENCODER_WHEEL_LF].delta,
        encoder_totals[ENCODER_WHEEL_LF],
        EncoderService_GetDirectionName(encoder_states[ENCODER_WHEEL_LF].direction),
        (unsigned long)encoder_states[ENCODER_WHEEL_LR].raw_count,
        (long)encoder_states[ENCODER_WHEEL_LR].delta,
        encoder_totals[ENCODER_WHEEL_LR],
        EncoderService_GetDirectionName(encoder_states[ENCODER_WHEEL_LR].direction));
      DEBUG_LOG_INFO(
        "ENC R: RF raw=%lu d=%ld total=%s dir=%s | RR raw=%lu d=%ld total=%s dir=%s",
        (unsigned long)encoder_states[ENCODER_WHEEL_RF].raw_count,
        (long)encoder_states[ENCODER_WHEEL_RF].delta,
        encoder_totals[ENCODER_WHEEL_RF],
        EncoderService_GetDirectionName(encoder_states[ENCODER_WHEEL_RF].direction),
        (unsigned long)encoder_states[ENCODER_WHEEL_RR].raw_count,
        (long)encoder_states[ENCODER_WHEEL_RR].delta,
        encoder_totals[ENCODER_WHEEL_RR],
        EncoderService_GetDirectionName(encoder_states[ENCODER_WHEEL_RR].direction));
    }

    if (WheelSpeedEstimator_GetSnapshot(speed_states))
    {
      DEBUG_LOG_INFO(
        "SPD L: LF dc=%ld cps=%ld rpm_x10=%ld mmps=%ld stop=%u | LR dc=%ld cps=%ld rpm_x10=%ld mmps=%ld stop=%u",
        (long)speed_states[ENCODER_WHEEL_LF].delta_counts,
        (long)speed_states[ENCODER_WHEEL_LF].filtered_counts_per_second,
        (long)(speed_states[ENCODER_WHEEL_LF].wheel_rpm * 10.0f),
        (long)(speed_states[ENCODER_WHEEL_LF].wheel_linear_velocity_m_s * 1000.0f),
        speed_states[ENCODER_WHEEL_LF].stopped ? 1U : 0U,
        (long)speed_states[ENCODER_WHEEL_LR].delta_counts,
        (long)speed_states[ENCODER_WHEEL_LR].filtered_counts_per_second,
        (long)(speed_states[ENCODER_WHEEL_LR].wheel_rpm * 10.0f),
        (long)(speed_states[ENCODER_WHEEL_LR].wheel_linear_velocity_m_s * 1000.0f),
        speed_states[ENCODER_WHEEL_LR].stopped ? 1U : 0U);
      DEBUG_LOG_INFO(
        "SPD R: RF dc=%ld cps=%ld rpm_x10=%ld mmps=%ld stop=%u | RR dc=%ld cps=%ld rpm_x10=%ld mmps=%ld stop=%u",
        (long)speed_states[ENCODER_WHEEL_RF].delta_counts,
        (long)speed_states[ENCODER_WHEEL_RF].filtered_counts_per_second,
        (long)(speed_states[ENCODER_WHEEL_RF].wheel_rpm * 10.0f),
        (long)(speed_states[ENCODER_WHEEL_RF].wheel_linear_velocity_m_s * 1000.0f),
        speed_states[ENCODER_WHEEL_RF].stopped ? 1U : 0U,
        (long)speed_states[ENCODER_WHEEL_RR].delta_counts,
        (long)speed_states[ENCODER_WHEEL_RR].filtered_counts_per_second,
        (long)(speed_states[ENCODER_WHEEL_RR].wheel_rpm * 10.0f),
        (long)(speed_states[ENCODER_WHEEL_RR].wheel_linear_velocity_m_s * 1000.0f),
        speed_states[ENCODER_WHEEL_RR].stopped ? 1U : 0U);
    }

#if !IMU_SPI_ID_PROBE_ONLY
    if ((!ImuStaticTest_IsActive()) &&
        ((int32_t)(now_ms - next_imu_log_ms) >= 0) &&
        ImuService_GetSnapshot(&imu_snapshot))
    {
      if (imu_snapshot.valid)
      {
        DEBUG_LOG_INFO(
          "IMU RAW: a=[%d,%d,%d] temp=%d g=[%d,%d,%d] samples=%lu errors=%lu",
          (int)imu_snapshot.accel_x_raw,
          (int)imu_snapshot.accel_y_raw,
          (int)imu_snapshot.accel_z_raw,
          (int)imu_snapshot.temperature_raw,
          (int)imu_snapshot.gyro_x_raw,
          (int)imu_snapshot.gyro_y_raw,
          (int)imu_snapshot.gyro_z_raw,
          (unsigned long)imu_snapshot.sample_count,
          (unsigned long)imu_snapshot.read_error_count);
        DEBUG_LOG_INFO(
          "IMU SI: a_mmps2=[%ld,%ld,%ld] w_mrads=[%ld,%ld,%ld] temp_mC=%ld bias=%u",
          (long)(imu_snapshot.accel_x_m_s2 * 1000.0f),
          (long)(imu_snapshot.accel_y_m_s2 * 1000.0f),
          (long)(imu_snapshot.accel_z_m_s2 * 1000.0f),
          (long)(imu_snapshot.angular_velocity_x_rad_s * 1000.0f),
          (long)(imu_snapshot.angular_velocity_y_rad_s * 1000.0f),
          (long)(imu_snapshot.angular_velocity_z_rad_s * 1000.0f),
          (long)(imu_snapshot.temperature_c * 1000.0f),
          imu_snapshot.gyro_bias_valid ? 1U : 0U);
#if IMU_USE_MPU6500_SPI
        ImuStaticTestStatus calibration;
        (void)ImuStaticTest_GetStatus(&calibration);
        DEBUG_LOG_INFO(
          "IMU QUALITY: epoch=%lu drdy=%lu missed=%lu race=%lu timeout=%lu cal=%u persist=%u",
          (unsigned long)imu_snapshot.acquisition_epoch,
          (unsigned long)imu_snapshot.data_ready_count,
          (unsigned long)imu_snapshot.missed_samples,
          (unsigned long)imu_snapshot.read_races,
          (unsigned long)imu_snapshot.data_ready_timeouts,
          (unsigned int)calibration.result,
          (unsigned int)calibration.persistence);
#endif
        next_imu_log_ms = now_ms + IMU_VALID_LOG_PERIOD_MS;
      }
      else
      {
        DEBUG_LOG_WARN(
          "IMU status=%s WHO_AM_I=0x%02X samples=%lu errors=%lu",
          ImuService_StateToString(imu_snapshot.state),
          (unsigned int)imu_snapshot.who_am_i,
          (unsigned long)imu_snapshot.sample_count,
          (unsigned long)imu_snapshot.read_error_count);
        next_imu_log_ms = now_ms + IMU_INVALID_LOG_PERIOD_MS;
      }
    }
#endif

    next_wake_tick += 200U;
    (void)osDelayUntil(next_wake_tick);
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartBatteryTask */
/**
  * @brief  Function implementing the batteryTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBatteryTask */
void StartBatteryTask(void *argument)
{
  /* USER CODE BEGIN StartBatteryTask */
  uint32_t next_wake_tick;
#if BATTERY_MONITOR_ENABLED
  uint32_t next_log_ms;
#endif

  DEBUG_LOG_INFO("batteryTask started");
#if BATTERY_MONITOR_ENABLED
  if (!BatteryMonitor_Init(HAL_GetTick()))
  {
    DEBUG_LOG_ERROR("Battery monitor unavailable; task remains alive for diagnostics");
  }
  next_log_ms = HAL_GetTick() + BATTERY_DEBUG_LOG_PERIOD_MS;
#else
  DEBUG_LOG_WARN(
    "Battery ADC monitoring deferred: batteryTask heartbeat only, no battery telemetry");
#endif
  next_wake_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    const uint32_t now_ms = HAL_GetTick();
#if BATTERY_MONITOR_ENABLED
    BatteryMonitorSnapshot battery = { 0 };

    (void)BatteryMonitor_Update(now_ms);
#endif
    if (FaultInjectionTest_ShouldReportHeartbeat(FAULT_MANAGER_TASK_BATTERY))
    {
      FaultManager_Heartbeat(FAULT_MANAGER_TASK_BATTERY, now_ms);
    }
#if BATTERY_MONITOR_ENABLED
    if (((int32_t)(now_ms - next_log_ms) >= 0) &&
        BatteryMonitor_GetSnapshot(&battery))
    {
      DEBUG_LOG_INFO(
        "BAT: raw=%u filtered=%u adc=%umV supply=%umV state=%s samples=%lu errors=%lu",
        (unsigned int)battery.raw_average,
        (unsigned int)battery.filtered_raw,
        (unsigned int)battery.adc_input_mv,
        (unsigned int)battery.supply_mv,
        BatteryMonitor_GetStateName(battery.state),
        (unsigned long)battery.sample_count,
        (unsigned long)battery.adc_error_count);
      next_log_ms = now_ms + BATTERY_DEBUG_LOG_PERIOD_MS;
    }
#endif
    next_wake_tick += BATTERY_SAMPLE_PERIOD_MS;
    (void)osDelayUntil(next_wake_tick);
  }
  /* USER CODE END StartBatteryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

