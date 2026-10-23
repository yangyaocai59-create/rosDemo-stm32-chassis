/**
 * @file imu_bias_storage.c
 * @brief Append-only A/B journals, with backend-specific bias compatibility.
 */

#include "imu_bias_storage.h"

#include <stddef.h>
#include <string.h>

#include "debug_log.h"
#include "imu_config.h"
#include "imu_bias_units.h"
#include "stm32f4xx_hal.h"
#if IMU_USE_MPU6500_SPI
#include "mpu6500.h"
#endif

#define IMU_BIAS_FLASH_SECTOR_A_ADDRESS  0x080C0000UL
#define IMU_BIAS_FLASH_SECTOR_B_ADDRESS  0x080E0000UL
#define IMU_BIAS_FLASH_SECTOR_SIZE       0x00020000UL

#define IMU_BIAS_RECORD_MAGIC            0x494D5542UL /* "IMUB" */
#define IMU_BIAS_RECORD_COMMITTED        0x434F4D54UL /* "COMT" */

#if IMU_USE_MPU6500_SPI
#define IMU_BIAS_RECORD_FORMAT_VERSION   3U
#define IMU_BIAS_RECORD_PROFILE          0x0270U /* Algorithm 2, WHO 0x70. */
#define IMU_BIAS_BACKEND_TEXT            "MPU6500 SPI"
#else
/* Same fixed-point representation, separate backend identity. */
#define IMU_BIAS_RECORD_FORMAT_VERSION   3U
#define IMU_BIAS_RECORD_PROFILE          0x0268U
#define IMU_BIAS_BACKEND_TEXT            "MPU6050 I2C"
#endif

typedef struct
{
  uint32_t magic;
  uint16_t format_version;
  uint16_t record_size;
  uint32_t sequence;
  uint32_t imu_identity;
  int32_t gyro_bias_x_milli_raw;
  int32_t gyro_bias_y_milli_raw;
  int32_t gyro_bias_z_milli_raw;
  int16_t calibration_temp_centi_c;
  uint16_t reserved;
  uint32_t crc32;
  uint32_t commit_marker;
} ImuBiasPersistRecord;

typedef struct
{
  uint32_t first_free_address;
  uint32_t valid_count;
} ImuBiasJournalScan;

_Static_assert(sizeof(ImuBiasPersistRecord) == 40U,
               "IMU bias Flash record layout changed");
_Static_assert((IMU_BIAS_FLASH_SECTOR_A_ADDRESS % 4U) == 0U,
               "IMU bias Flash sector A must be word aligned");
_Static_assert((IMU_BIAS_FLASH_SECTOR_B_ADDRESS % 4U) == 0U,
               "IMU bias Flash sector B must be word aligned");

static ImuBiasPersistRecord s_newest_record;
static ImuBiasJournalScan s_journal_a;
static ImuBiasJournalScan s_journal_b;
static bool s_initialized;
static bool s_has_valid_record;
static bool s_newest_is_a;
static uint32_t s_expected_identity;

static uint32_t ImuBiasStorage_Crc32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t index = 0U; index < length; ++index)
  {
    crc ^= data[index];
    for (uint32_t bit = 0U; bit < 8U; ++bit)
    {
      const uint32_t mask = (uint32_t)-(int32_t)(crc & 1UL);
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

static bool ImuBiasStorage_IsBiasValid(int32_t value)
{
  return (value >= -IMU_GYRO_BIAS_ABS_MAX_RAW * IMU_BIAS_MILLI_RAW_PER_RAW) &&
         (value <= IMU_GYRO_BIAS_ABS_MAX_RAW * IMU_BIAS_MILLI_RAW_PER_RAW);
}

static uint32_t ImuBiasStorage_ProfileIdentity(void)
{
#if IMU_USE_MPU6500_SPI
  /* Versioned profile, not a physical sensor serial number. CRC includes
   * sensor/WHO, algorithm/temperature convention, rate, filter, scales and
   * body-frame mapping. Altering a profile rejects old corrections. */
  uint32_t profile[] = {
    0x6500U, MPU6500_WHO_AM_I_EXPECTED, 2U, 1U, IMU_SAMPLE_PERIOD_MS,
    Mpu6500_ConfigurationSignature(),
    IMU_BODY_X_SOURCE_AXIS, IMU_BODY_Y_SOURCE_AXIS, IMU_BODY_Z_SOURCE_AXIS,
    (uint32_t)IMU_BODY_X_SOURCE_SIGN, (uint32_t)IMU_BODY_Y_SOURCE_SIGN,
    (uint32_t)IMU_BODY_Z_SOURCE_SIGN, 0U, 0U, IMU_BIAS_MILLI_RAW_PER_RAW
  };
  const float accel_scale = IMU_ACCEL_LSB_PER_G;
  const float gyro_scale = IMU_GYRO_LSB_PER_DEGREE_PER_SECOND;
  _Static_assert(sizeof(float) == sizeof(uint32_t), "Profile float size changed");
  memcpy(&profile[12], &accel_scale, sizeof(accel_scale));
  memcpy(&profile[13], &gyro_scale, sizeof(gyro_scale));
  return ImuBiasStorage_Crc32((const uint8_t *)profile, sizeof(profile));
#else
  return 0x60500503UL;
#endif
}

static bool ImuBiasStorage_IsErased(const ImuBiasPersistRecord *record)
{
  const uint32_t *words = (const uint32_t *)(const void *)record;

  for (uint32_t index = 0U;
       index < (sizeof(ImuBiasPersistRecord) / sizeof(uint32_t));
       ++index)
  {
    if (words[index] != 0xFFFFFFFFUL)
    {
      return false;
    }
  }
  return true;
}

static bool ImuBiasStorage_IsValid(const ImuBiasPersistRecord *record)
{
  if ((record->magic != IMU_BIAS_RECORD_MAGIC) ||
      (record->format_version != IMU_BIAS_RECORD_FORMAT_VERSION) ||
      (record->record_size != sizeof(ImuBiasPersistRecord)) ||
      (record->imu_identity != s_expected_identity) ||
      (record->reserved != IMU_BIAS_RECORD_PROFILE) ||
      (record->commit_marker != IMU_BIAS_RECORD_COMMITTED) ||
      (!ImuBiasStorage_IsBiasValid(record->gyro_bias_x_milli_raw)) ||
      (!ImuBiasStorage_IsBiasValid(record->gyro_bias_y_milli_raw)) ||
      (!ImuBiasStorage_IsBiasValid(record->gyro_bias_z_milli_raw)))
  {
    return false;
  }

  return record->crc32 == ImuBiasStorage_Crc32(
    (const uint8_t *)(const void *)record,
    offsetof(ImuBiasPersistRecord, crc32));
}

static bool ImuBiasStorage_IsNewer(uint32_t candidate, uint32_t reference)
{
  return ((int32_t)(candidate - reference) > 0);
}

static void ImuBiasStorage_ScanJournal(uint32_t base_address,
                                       bool journal_is_a,
                                       ImuBiasJournalScan *scan)
{
  const uint32_t record_count =
    IMU_BIAS_FLASH_SECTOR_SIZE / sizeof(ImuBiasPersistRecord);

  scan->first_free_address = 0U;
  scan->valid_count = 0U;

  for (uint32_t index = 0U; index < record_count; ++index)
  {
    const uint32_t address =
      base_address + (index * sizeof(ImuBiasPersistRecord));
    const ImuBiasPersistRecord *record =
      (const ImuBiasPersistRecord *)(uintptr_t)address;

    if (ImuBiasStorage_IsErased(record))
    {
      if (scan->first_free_address == 0U)
      {
        scan->first_free_address = address;
      }
      continue;
    }

    if (ImuBiasStorage_IsValid(record))
    {
      ++scan->valid_count;
      if ((!s_has_valid_record) ||
          ImuBiasStorage_IsNewer(record->sequence,
                                 s_newest_record.sequence))
      {
        s_newest_record = *record;
        s_has_valid_record = true;
        s_newest_is_a = journal_is_a;
      }
    }
  }
}

void ImuBiasStorage_Init(void)
{
  s_initialized = false;
  s_expected_identity = ImuBiasStorage_ProfileIdentity();
  memset(&s_newest_record, 0, sizeof(s_newest_record));
  memset(&s_journal_a, 0, sizeof(s_journal_a));
  memset(&s_journal_b, 0, sizeof(s_journal_b));
  s_has_valid_record = false;
  s_newest_is_a = false;

  ImuBiasStorage_ScanJournal(IMU_BIAS_FLASH_SECTOR_A_ADDRESS,
                             true,
                             &s_journal_a);
  ImuBiasStorage_ScanJournal(IMU_BIAS_FLASH_SECTOR_B_ADDRESS,
                             false,
                             &s_journal_b);
  s_initialized = true;

  if (s_has_valid_record)
  {
    DEBUG_LOG_INFO(
      "IMU bias Flash ready: " IMU_BIAS_BACKEND_TEXT " profile=0x%08lX newest=%c seq=%lu records=%lu/%lu",
      (unsigned long)s_expected_identity,
      s_newest_is_a ? 'A' : 'B',
      (unsigned long)s_newest_record.sequence,
      (unsigned long)s_journal_a.valid_count,
      (unsigned long)s_journal_b.valid_count);
  }
  else
  {
    DEBUG_LOG_INFO("IMU bias Flash ready: " IMU_BIAS_BACKEND_TEXT " profile=0x%08lX no compatible saved record",
                   (unsigned long)s_expected_identity);
  }
}

bool ImuBiasStorage_Load(int32_t *x_raw,
                         int32_t *y_raw,
                         int32_t *z_raw,
                         int16_t *calibration_temp_centi_c)
{
  int32_t x, y, z;
  if ((x_raw == NULL) || (y_raw == NULL) || (z_raw == NULL) ||
      !ImuBiasStorage_LoadMilliRaw(&x, &y, &z, calibration_temp_centi_c))
  {
    return false;
  }
  *x_raw = x / IMU_BIAS_MILLI_RAW_PER_RAW;
  *y_raw = y / IMU_BIAS_MILLI_RAW_PER_RAW;
  *z_raw = z / IMU_BIAS_MILLI_RAW_PER_RAW;
  return true;
}

bool ImuBiasStorage_LoadMilliRaw(int32_t *x_milli_raw,
                                  int32_t *y_milli_raw,
                                  int32_t *z_milli_raw,
                                  int16_t *calibration_temp_centi_c)
{
  if ((!s_initialized) || (!s_has_valid_record) ||
      (x_milli_raw == NULL) || (y_milli_raw == NULL) || (z_milli_raw == NULL))
  {
    return false;
  }

  *x_milli_raw = s_newest_record.gyro_bias_x_milli_raw;
  *y_milli_raw = s_newest_record.gyro_bias_y_milli_raw;
  *z_milli_raw = s_newest_record.gyro_bias_z_milli_raw;
  if (calibration_temp_centi_c != NULL)
  {
    *calibration_temp_centi_c =
      s_newest_record.calibration_temp_centi_c;
  }
  return true;
}

static bool ImuBiasStorage_ProgramRecord(
  uint32_t address,
  const ImuBiasPersistRecord *record)
{
  const uint32_t *words = (const uint32_t *)(const void *)record;
  const uint32_t commit_word_index =
    offsetof(ImuBiasPersistRecord, commit_marker) / sizeof(uint32_t);
  bool programmed = true;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                         FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                         FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  /* The commit marker is programmed last. A reset at any earlier point leaves
   * a record that startup validation will reject while the previous record
   * remains untouched. */
  for (uint32_t index = 0U; index < commit_word_index; ++index)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          address + (index * sizeof(uint32_t)),
                          words[index]) != HAL_OK)
    {
      programmed = false;
      break;
    }
  }

  if (programmed &&
      (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                         address + offsetof(ImuBiasPersistRecord,
                                            commit_marker),
                         record->commit_marker) != HAL_OK))
  {
    programmed = false;
  }

  (void)HAL_FLASH_Lock();
  return programmed;
}

bool ImuBiasStorage_Save(int32_t x_raw,
                         int32_t y_raw,
                         int32_t z_raw,
                         int16_t calibration_temp_centi_c)
{
  if ((x_raw < -IMU_GYRO_BIAS_ABS_MAX_RAW) || (x_raw > IMU_GYRO_BIAS_ABS_MAX_RAW) ||
      (y_raw < -IMU_GYRO_BIAS_ABS_MAX_RAW) || (y_raw > IMU_GYRO_BIAS_ABS_MAX_RAW) ||
      (z_raw < -IMU_GYRO_BIAS_ABS_MAX_RAW) || (z_raw > IMU_GYRO_BIAS_ABS_MAX_RAW))
  {
    return false;
  }
  return ImuBiasStorage_SaveMilliRaw(x_raw * IMU_BIAS_MILLI_RAW_PER_RAW,
                                      y_raw * IMU_BIAS_MILLI_RAW_PER_RAW,
                                      z_raw * IMU_BIAS_MILLI_RAW_PER_RAW,
                                      calibration_temp_centi_c);
}

bool ImuBiasStorage_SaveMilliRaw(int32_t x_milli_raw,
                                  int32_t y_milli_raw,
                                  int32_t z_milli_raw,
                                  int16_t calibration_temp_centi_c)
{
  ImuBiasPersistRecord record;
  ImuBiasJournalScan *target_journal;
  uint32_t target_address;
  bool target_is_a;

  if ((!s_initialized) ||
      (!ImuBiasStorage_IsBiasValid(x_milli_raw)) ||
      (!ImuBiasStorage_IsBiasValid(y_milli_raw)) ||
      (!ImuBiasStorage_IsBiasValid(z_milli_raw)))
  {
    return false;
  }

  /* Prefer the opposite journal so every successful update retains an older
   * record in the other physical erase sector. */
  target_is_a = s_has_valid_record ? (!s_newest_is_a) : true;
  target_journal = target_is_a ? &s_journal_a : &s_journal_b;
  if (target_journal->first_free_address == 0U)
  {
    target_is_a = !target_is_a;
    target_journal = target_is_a ? &s_journal_a : &s_journal_b;
  }
  if (target_journal->first_free_address == 0U)
  {
    DEBUG_LOG_ERROR(
      "IMU bias Flash journals are full; RAM bias remains valid, saved bias unchanged");
    return false;
  }
  target_address = target_journal->first_free_address;

  memset(&record, 0, sizeof(record));
  record.magic = IMU_BIAS_RECORD_MAGIC;
  record.format_version = IMU_BIAS_RECORD_FORMAT_VERSION;
  record.record_size = sizeof(record);
  record.sequence = s_has_valid_record ?
    (s_newest_record.sequence + 1U) : 1U;
  record.imu_identity = s_expected_identity;
  record.reserved = IMU_BIAS_RECORD_PROFILE;
  record.gyro_bias_x_milli_raw = x_milli_raw;
  record.gyro_bias_y_milli_raw = y_milli_raw;
  record.gyro_bias_z_milli_raw = z_milli_raw;
  record.calibration_temp_centi_c = calibration_temp_centi_c;
  record.crc32 = ImuBiasStorage_Crc32(
    (const uint8_t *)(const void *)&record,
    offsetof(ImuBiasPersistRecord, crc32));
  record.commit_marker = IMU_BIAS_RECORD_COMMITTED;

  if ((!ImuBiasStorage_ProgramRecord(target_address, &record)) ||
      (!ImuBiasStorage_IsValid(
        (const ImuBiasPersistRecord *)(uintptr_t)target_address)) ||
      (memcmp((const void *)(uintptr_t)target_address,
              &record,
              sizeof(record)) != 0))
  {
    DEBUG_LOG_ERROR(
      "IMU bias Flash save/verify failed at 0x%08lX; previous saved bias preserved",
      (unsigned long)target_address);
    /* Rescan so a partially programmed slot is never selected again. */
    ImuBiasStorage_Init();
    return false;
  }

  s_newest_record = record;
  s_has_valid_record = true;
  s_newest_is_a = target_is_a;
  ++target_journal->valid_count;
  target_journal->first_free_address =
    target_address + sizeof(ImuBiasPersistRecord);
  const uint32_t journal_end =
    (target_is_a ? IMU_BIAS_FLASH_SECTOR_A_ADDRESS :
                   IMU_BIAS_FLASH_SECTOR_B_ADDRESS) + IMU_BIAS_FLASH_SECTOR_SIZE;
  /* A free hole can precede an occupied legacy/partial slot. Never assume
   * the following slot is erased or attempt to overwrite existing records. */
  while (target_journal->first_free_address <=
           journal_end - sizeof(ImuBiasPersistRecord))
  {
    if (ImuBiasStorage_IsErased((const ImuBiasPersistRecord *)(uintptr_t)
                               target_journal->first_free_address))
    {
      break;
    }
    target_journal->first_free_address += sizeof(ImuBiasPersistRecord);
  }
  if (target_journal->first_free_address >
        journal_end - sizeof(ImuBiasPersistRecord))
  {
    target_journal->first_free_address = 0U;
  }

  DEBUG_LOG_INFO(
    "IMU bias Flash saved: slot=%c seq=%lu address=0x%08lX temp=%d.%02uC",
    target_is_a ? 'A' : 'B',
    (unsigned long)record.sequence,
    (unsigned long)target_address,
    (int)(calibration_temp_centi_c / 100),
    (unsigned int)((calibration_temp_centi_c < 0) ?
      -(calibration_temp_centi_c % 100) :
       (calibration_temp_centi_c % 100)));
  return true;
}
