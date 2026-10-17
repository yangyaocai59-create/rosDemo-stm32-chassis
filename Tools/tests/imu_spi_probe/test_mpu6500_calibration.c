/* Actual SPI/service/calibration/storage integration; register mocks and a
 * private Windows virtual-memory Flash image. No device or filesystem I/O. */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define MPU6500_CALIBRATION_INTEGRATION_TEST
#define main PollingRegression_NotInvoked
#include "test_mpu6500_polling.c"
#undef main
static uint32_t profile_override;
static uint32_t Test_ConfigurationSignature(void)
{ return Mpu6500_ConfigurationSignature() ^ profile_override; }
#define Mpu6500_ConfigurationSignature Test_ConfigurationSignature
#include "../../../Application/Storage/imu_bias_storage.c"
#undef Mpu6500_ConfigurationSignature

static bool flash_unlocked, unlock_fail, corrupt_commit;
static uint32_t program_calls, program_fail_at, last_address;
static uint32_t unlock_calls, lock_calls;
static bool test_start_during_save;

void Test_FlashClearFlags(uint32_t flags)
{ assert(flags == 63U && flash_unlocked && !critical_depth); }
HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
  assert(!critical_depth);
  ++unlock_calls;
  if (test_start_during_save)
  {
    assert(ImuStaticTest_IsActive());
    assert(!ImuStaticTest_Start(clock_ms,false,IMU_STATIC_SOURCE_UART));
  }
  if (unlock_fail) return HAL_ERROR;
  flash_unlocked = true;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Lock(void)
{ ++lock_calls; flash_unlocked = false; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value)
{
  assert(!critical_depth && flash_unlocked && type == FLASH_TYPEPROGRAM_WORD);
  assert(address >= 0x080C0000U && address <= 0x080FFFFCU && !(address & 3U));
  ++program_calls;
  last_address = address;
  if (program_fail_at == program_calls) return HAL_ERROR;
  uint32_t *word = (uint32_t *)(uintptr_t)address;
  assert(*word == UINT32_MAX); /* No overwrite even in a hole-filled journal. */
  *word = (uint32_t)value;
  if (corrupt_commit && (uint32_t)value == IMU_BIAS_RECORD_COMMITTED)
  {
    *(uint32_t *)(uintptr_t)(address-4U) ^= 1U; /* CRC readback error. */
  }
  return HAL_OK;
}

static void EraseMock(void)
{
  memset((void *)(uintptr_t)0x080C0000U, 0xFF, 0x40000U);
  flash_unlocked = unlock_fail = corrupt_commit = false;
  test_start_during_save = false;
  program_calls = program_fail_at = unlock_calls = lock_calls = last_address = 0U;
  profile_override = 0U;
  ImuBiasStorage_Init();
  assert(!program_calls && !unlock_calls && !lock_calls);
}
static void SeedRecord(uint32_t address, uint16_t version, uint32_t identity,
                       uint16_t profile, uint32_t sequence, int32_t bias)
{
  ImuBiasPersistRecord record = {0};
  record.magic = IMU_BIAS_RECORD_MAGIC;
  record.format_version = version;
  record.record_size = sizeof(record);
  record.sequence = sequence;
  record.imu_identity = identity;
  record.reserved = profile;
  record.gyro_bias_x_milli_raw = record.gyro_bias_y_milli_raw =
    record.gyro_bias_z_milli_raw = bias;
  record.calibration_temp_centi_c = 2800;
  record.crc32 = ImuBiasStorage_Crc32((const uint8_t *)&record, offsetof(ImuBiasPersistRecord,crc32));
  record.commit_marker = IMU_BIAS_RECORD_COMMITTED;
  memcpy((void *)(uintptr_t)address,&record,sizeof(record));
}
static bool LoadExpected(int32_t x, int32_t y, int32_t z, int16_t temperature)
{
  int32_t a,b,c; int16_t t;
  return ImuBiasStorage_Load(&a,&b,&c,&t) && a==x && b==y && c==z && t==temperature;
}
static bool HasSaved(void)
{ int32_t x,y,z; return ImuBiasStorage_Load(&x,&y,&z,NULL); }

static void TestStorage(void)
{
  EraseMock();
  SeedRecord(0x080C0000U,1U,0x60500501U,0U,900U,800);
  ImuBiasStorage_Init();
  assert(!HasSaved());
  assert(ImuBiasStorage_Save(-38,2,68,2812));
  assert(LoadExpected(-38,2,68,2812) && !flash_unlocked && program_calls==10);
  ImuBiasPersistRecord saved = *(ImuBiasPersistRecord *)(uintptr_t)0x080C0028U;
  assert(saved.format_version==3 && saved.reserved==0x0270 && saved.sequence==1);
  assert(saved.imu_identity==s_expected_identity && saved.imu_identity!=0x60500501U);
  assert(((ImuBiasPersistRecord *)(uintptr_t)0x080C0000U)->imu_identity==0x60500501U);
  ImuBiasStorage_Init(); /* Cold boot: rebuild RAM state from memory image. */
  assert(LoadExpected(-38,2,68,2812));
  assert(ImuBiasStorage_Save(-30,1,60,2820));
  assert(last_address==0x080E0024U && LoadExpected(-30,1,60,2820));
  ImuBiasStorage_Init(); assert(LoadExpected(-30,1,60,2820));
  profile_override=1U;ImuBiasStorage_Init();assert(!HasSaved());
  profile_override=0U;ImuBiasStorage_Init();assert(LoadExpected(-30,1,60,2820));

  for (unsigned word=1;word<=10;++word)
  {
    EraseMock();assert(ImuBiasStorage_Save(1,2,3,2100));
    program_fail_at=program_calls+word;
    assert(!ImuBiasStorage_Save(4,5,6,2200) && !flash_unlocked);
    assert(LoadExpected(1,2,3,2100));
    ImuBiasStorage_Init();assert(LoadExpected(1,2,3,2100));
    program_fail_at=0;
    assert(ImuBiasStorage_Save(7,8,9,2300));
    assert(LoadExpected(7,8,9,2300));
  }
  EraseMock();assert(ImuBiasStorage_Save(1,2,3,2100));
  corrupt_commit=true;assert(!ImuBiasStorage_Save(4,5,6,2200));
  assert(LoadExpected(1,2,3,2100) && !flash_unlocked);
  corrupt_commit=false;unlock_fail=true;
  uint32_t before=program_calls;
  assert(!ImuBiasStorage_Save(4,5,6,2200) && program_calls==before);
  assert(!ImuBiasStorage_Save(1001,0,0,2100));
  assert(!ImuBiasStorage_Save(-1001,0,0,2100));
  assert(!ImuBiasStorage_Load(NULL,NULL,NULL,NULL));

  /* Full journal, partial slots, wrong format/profile/CRC/size/commit/bias. */
  for(unsigned field=0;field<8;++field)
  {
    EraseMock();
    ImuBiasPersistRecord bad=saved;
    switch(field) {
      case 0:bad.format_version=1;break;
      case 1:bad.imu_identity^=1;break;
      case 2:bad.reserved^=1;break;
      case 3:bad.record_size=36;break;
      case 4:bad.commit_marker=UINT32_MAX;break;
      case 5:bad.gyro_bias_x_milli_raw=1000001;break;
      case 6:bad.magic^=1;break;
      default:bad.crc32^=1;break;
    }
    if(field!=7)bad.crc32=ImuBiasStorage_Crc32((const uint8_t *)&bad,offsetof(ImuBiasPersistRecord,crc32));
    memcpy((void *)(uintptr_t)0x080C0000U,&bad,sizeof(bad));
    ImuBiasStorage_Init();assert(!HasSaved());
  }
  EraseMock();memset((void *)(uintptr_t)0x080C0000U,0,0x40000U);
  ImuBiasStorage_Init();assert(!ImuBiasStorage_Save(1,2,3,2100) && !program_calls);

  EraseMock(); /* Empty A0, occupied A1; next save must skip A1. */
  SeedRecord(0x080C0028U,1U,0x60500501U,0U,900U,800);
  ImuBiasStorage_Init();
  assert(ImuBiasStorage_Save(1,2,3,2100));
  assert(s_journal_a.first_free_address==0x080C0050U);
  assert(ImuBiasStorage_Save(4,5,6,2200));
  assert(ImuBiasStorage_Save(7,8,9,2300));
  assert(((ImuBiasPersistRecord *)(uintptr_t)0x080C0028U)->gyro_bias_x_milli_raw==800);
  assert(LoadExpected(7,8,9,2300));

  EraseMock();
  SeedRecord(0x080C0000U,3U,s_expected_identity,0x0270U,UINT32_MAX,12000);
  ImuBiasStorage_Init();assert(ImuBiasStorage_Save(13,14,15,2300));
  assert(s_newest_record.sequence==0);
  ImuBiasStorage_Init();assert(LoadExpected(13,14,15,2300));
  puts("PASS: actual A/B storage cold restore, old MPU6050/profile/format/CRC rejection, sequence wrap, holes and full journals");
  puts("PASS: failures at all 10 Flash words, unlock and readback corruption preserve previous committed record; no erase API");
}

static void TestFractionalStorage(void)
{
  EraseMock();
  /* Previously valid MPU6500 integer record must not become milli-RAW. */
  SeedRecord(0x080C0000U,2U,0xDB18A362U,0x0170U,100U,146);
  ImuBiasStorage_Init();
  assert(!HasSaved());
  assert(ImuBiasStorage_SaveMilliRaw(-52514, 19500, 145516, 2716));
  int32_t x,y,z; int16_t temp;
  ImuBiasStorage_Init();
  assert(ImuBiasStorage_LoadMilliRaw(&x,&y,&z,&temp));
  assert(x==-52514 && y==19500 && z==145516 && temp==2716);
  assert(LoadExpected(-52,19,145,2716)); /* Coarse view explicitly truncates. */
  assert(!ImuBiasStorage_SaveMilliRaw(1000001,0,0,2716));
  assert(!ImuBiasStorage_SaveMilliRaw(-1000001,0,0,2716));
  assert(!ImuBiasStorage_Save(INT32_MAX,0,0,2716));
  assert(!ImuBiasStorage_Save(INT32_MIN,0,0,2716));
  for (unsigned word=1U;word<=10U;++word)
  {
    EraseMock(); assert(ImuBiasStorage_SaveMilliRaw(-52514,19500,145516,2716));
    program_fail_at=program_calls+word;
    assert(!ImuBiasStorage_SaveMilliRaw(-52515,19501,145517,2717));
    ImuBiasStorage_Init();
    assert(ImuBiasStorage_LoadMilliRaw(&x,&y,&z,&temp));
    assert(x==-52514 && y==19500 && z==145516 && temp==2716);
  }
  puts("PASS: exact signed milli-RAW cold restore, integer v2 rejection, coarse view, bounds and all 10 torn-write positions");
}

static void PutStationary(void)
{
  Put16(0x3B,-570);Put16(0x3D,30);Put16(0x3F,8360);
  Put16(0x41,2404);Put16(0x43,-38);Put16(0x45,2);Put16(0x47,68);
}
static void FeedCalibration(unsigned count, uint32_t begin)
{
  for(unsigned i=0;i<count;++i)
  {
    Update(begin+i*10U);
    ImuServiceSnapshot value=Snapshot();
    ImuStaticTest_ProcessSample(&value,begin+i*10U);
  }
}
static void TestCalibration(void)
{
  EraseMock();ResetMock();ImuStaticTest_Init();StartReady(0);PutStationary();Update(310);
  assert(!Snapshot().gyro_bias_valid && !HasSaved());
  assert(!ImuStaticTest_Start(310,true,IMU_STATIC_SOURCE_USB));
  assert(!ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_NONE));
  assert(ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_USB));
  assert(!ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_UART));
  FeedCalibration(499,320);
  assert(ImuStaticTest_IsActive() && !HasSaved());
  ImuServiceSnapshot duplicate=Snapshot();
  ImuStaticTest_ProcessSample(&duplicate,5310);
  assert(ImuStaticTest_IsActive() && !HasSaved());
  test_start_during_save=true;
  FeedCalibration(1,5310);
  test_start_during_save=false;
  ImuStaticTestStatus status;assert(ImuStaticTest_GetStatus(&status));
  assert(status.result==IMU_STATIC_RESULT_PASS && !ImuStaticTest_IsActive());
  assert(status.persistence==IMU_STATIC_PERSIST_SAVED && status.collected_samples==500U);
  int16_t expected_temp=(int16_t)lroundf(Mpu6500_TemperatureC(2404)*100.0f);
  assert(LoadExpected(-38,2,68,expected_temp));
  assert(expected_temp==2820 && Snapshot().gyro_bias_valid && program_calls==10);
  Update(5320);
  assert(fabsf(Snapshot().angular_velocity_x_rad_s)<0.000001f);
  assert(fabsf(Snapshot().angular_velocity_z_rad_s)<0.000001f);

  /* Boot orchestration: initialize storage, sensor startup, load only once
   * READY. No Start() and no Flash writes during restoration. */
  ImuBiasStorage_Init();ResetMock();ImuStaticTest_Init();StartReady(0);
  int32_t x,y,z;int16_t t;unsigned before=program_calls;
  assert(!Snapshot().gyro_bias_valid);
  assert(ImuBiasStorage_Load(&x,&y,&z,&t) && ImuService_SetGyroBiasRaw(x,y,z));
  PutStationary();Update(310);
  assert(Snapshot().valid && Snapshot().gyro_bias_valid && program_calls==before);
  assert(ImuStaticTest_GetStatus(&status) && status.result==IMU_STATIC_RESULT_NOT_RUN);
  assert(fabsf(Snapshot().angular_velocity_z_rad_s)<0.000001f);
  (void)t;

  assert(ImuStaticTest_Start(320,false,IMU_STATIC_SOURCE_USB));
  FeedCalibration(20,320);ImuStaticTest_Cancel();FeedCalibration(500,530);
  assert(program_calls==before && LoadExpected(-38,2,68,expected_temp));
  assert(ImuStaticTest_Start(5530,false,IMU_STATIC_SOURCE_USB));
  assert(!ImuStaticTest_CancelIfSource(IMU_STATIC_SOURCE_UART));
  assert(ImuStaticTest_CancelIfSource(IMU_STATIC_SOURCE_USB));
  FeedCalibration(500,5540);assert(program_calls==before);
  assert(ImuStaticTest_Start(10540,false,IMU_STATIC_SOURCE_UART));
  FeedCalibration(250,clock_ms+10U);Put16(0x43,400);FeedCalibration(250,clock_ms+10U);
  assert(ImuStaticTest_GetStatus(&status) && status.result==IMU_STATIC_RESULT_FAIL_MOVED);
  assert(program_calls==before && LoadExpected(-38,2,68,expected_temp));
  PutStationary();Put16(0x43,1001);
  assert(ImuStaticTest_Start(15550,false,IMU_STATIC_SOURCE_UART));
  FeedCalibration(500,clock_ms+10U);
  assert(ImuStaticTest_GetStatus(&status) && status.result==IMU_STATIC_RESULT_FAIL_BIAS_RANGE);
  assert(program_calls==before && LoadExpected(-38,2,68,expected_temp));
  assert(Snapshot().gyro_bias_x_raw==-38);
  PutStationary();Put16(0x43,-30);Put16(0x47,60);
  assert(ImuStaticTest_Start(20560,false,IMU_STATIC_SOURCE_USB));
  program_fail_at=program_calls+3;
  FeedCalibration(500,clock_ms+10U);
  assert(ImuStaticTest_GetStatus(&status) && status.result==IMU_STATIC_RESULT_PASS);
  assert(status.persistence==IMU_STATIC_PERSIST_FAILED);
  assert(Snapshot().gyro_bias_valid && Snapshot().gyro_bias_x_raw==-30);
  assert(LoadExpected(-38,2,68,expected_temp) && !ImuStaticTest_IsActive());
  program_fail_at=0;
  puts("PASS: actual 500-sample calibration, MPU6500 temperature, duplicate/motion/owner gates, cancel, old bias preservation and corrected units");
  puts("PASS: persistence blocks a concurrent calibration start; bias-range rejection and RAM-only bias on save failure preserve old Flash");
  puts("PASS: cold startup orchestration restores compatible bias without recalibration or additional Flash programming");
}

static void TestCalibrationQuality(void)
{
  for (unsigned fault = 0U; fault < 10U; ++fault)
  {
    EraseMock(); ResetMock(); ImuStaticTest_Init(); StartReady(0);
    PutStationary(); Update(310);
    assert(ImuService_SetGyroBiasRaw(1,2,3));
    assert(ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_USB));
    FeedCalibration(10,320);
    ImuServiceSnapshot value = Snapshot();
    switch(fault)
    {
      case 0: test_motion_allowed=false; break;
      case 1: ++test_wheel_counts[2]; break;
      case 2: value.valid=false; break;
      case 3: ++value.acquisition_epoch; break;
      case 4: ++value.read_error_count; break;
      case 5: ++value.missed_samples; break;
      case 6: ++value.read_races; break;
      case 7: value.sample_count+=2U; break;
      case 8: ++value.sample_count; value.sample_time_ms-=1U; break;
      default: clock_ms+=26U; break; /* Duplicate samples cannot wait forever. */
    }
    ImuStaticTest_ProcessSample(&value,clock_ms);
    ImuStaticTestStatus status; assert(ImuStaticTest_GetStatus(&status));
    assert(status.result==(fault < 2U ? IMU_STATIC_RESULT_FAIL_MOVED : IMU_STATIC_RESULT_FAIL_DATA));
    assert(!ImuStaticTest_IsActive() && !program_calls && !HasSaved());
    assert(Snapshot().gyro_bias_x_raw==1 && Snapshot().gyro_bias_z_raw==3);
  }
  for (unsigned axis_kind=0;axis_kind<2U;++axis_kind)
  {
    EraseMock(); ResetMock(); ImuStaticTest_Init(); StartReady(0);
    PutStationary(); Update(310);
    assert(ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_UART));
    for(unsigned i=0;i<500U;++i)
    {
      /* P2P alone passes: gyro 40<96, accel 200<500. STD must reject. */
      if(axis_kind==0U) Put16(0x43,(int16_t)(-38+(i%2U ? 20 : -20)));
      else Put16(0x3B,(int16_t)(-570+(i%2U ? 100 : -100)));
      FeedCalibration(1,320+i*10U);
    }
    ImuStaticTestStatus status; assert(ImuStaticTest_GetStatus(&status));
    assert(status.result==IMU_STATIC_RESULT_FAIL_MOVED && !program_calls);
  }
  EraseMock(); ResetMock(); ImuStaticTest_Init();
  const uint32_t begin=UINT32_MAX-1000U;
  StartReady(begin); PutStationary(); Update(begin+310U);
  assert(ImuStaticTest_Start(begin+310U,false,IMU_STATIC_SOURCE_USB));
  FeedCalibration(500,begin+320U);
  ImuStaticTestStatus status; assert(ImuStaticTest_GetStatus(&status));
  assert(status.result==IMU_STATIC_RESULT_PASS && status.persistence==IMU_STATIC_PERSIST_SAVED);
  puts("PASS: full-window wheel/interlock checks, invalid/recovery/read-error/missed/raced/gapped data abort, STD-only rejection and tick wrap");
  EraseMock(); ResetMock(); ImuStaticTest_Init(); StartReady(0);
  PutStationary(); Update(310);
  assert(ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_USB));
  FeedCalibration(10,320); Update(420);
  ImuServiceSnapshot value=Snapshot(); test_restart_on_guard=true;
  ImuStaticTest_ProcessSample(&value,420);
  assert(ImuStaticTest_GetStatus(&status) && status.result==IMU_STATIC_RESULT_RUNNING);
  assert(status.collected_samples==0U && !program_calls);
  FeedCalibration(500,430);
  assert(ImuStaticTest_GetStatus(&status) && status.result==IMU_STATIC_RESULT_PASS);
  puts("PASS: Cancel/new Start during validation cannot append or commit the old calibration generation");
}

static void TestFractionalCalibration(void)
{
  EraseMock(); ResetMock(); ImuStaticTest_Init(); StartReady(0);
  PutStationary(); Update(310);
  assert(ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_USB));
  for(unsigned i=0U;i<500U;++i)
  {
    Put16(0x43,(int16_t)(i%2U ? -53 : -52));
    Put16(0x45,(int16_t)(i%2U ? 20 : 19));
    Put16(0x47,(int16_t)(i%2U ? 146 : 145));
    FeedCalibration(1,320U+i*10U);
  }
  ImuStaticTestStatus status; assert(ImuStaticTest_GetStatus(&status));
  assert(status.result==IMU_STATIC_RESULT_PASS && status.persistence==IMU_STATIC_PERSIST_SAVED);
  ImuServiceSnapshot sample=Snapshot();
  assert(sample.gyro_bias_x_milli_raw==-52500 && sample.gyro_bias_y_milli_raw==19500 &&
         sample.gyro_bias_z_milli_raw==145500);
  int32_t x,y,z; int16_t t;
  assert(ImuBiasStorage_LoadMilliRaw(&x,&y,&z,&t));
  assert(x==-52500 && y==19500 && z==145500);
  const unsigned before=program_calls;
  ImuBiasStorage_Init(); ResetMock(); ImuStaticTest_Init(); StartReady(0);
  assert(ImuBiasStorage_LoadMilliRaw(&x,&y,&z,&t));
  assert(ImuService_SetGyroBiasMilliRaw(x,y,z));
  float sum_wz=0.0f;
  for(unsigned i=0U;i<1000U;++i)
  {
    PutStationary(); Put16(0x47,(int16_t)(i%2U ? 146 : 145));
    Update(310U+i*10U); sample=Snapshot();
    assert(fabsf(sample.angular_velocity_z_rad_s)>0.00001f); /* No dead zone. */
    sum_wz+=sample.angular_velocity_z_rad_s;
  }
  assert(fabsf(sum_wz)<0.000001f && program_calls==before);
  assert(ImuStaticTest_GetStatus(&status) && status.result==IMU_STATIC_RESULT_NOT_RUN);
  assert(!ImuService_SetGyroBiasMilliRaw(1000001,0,0));
  assert(!ImuService_SetGyroBiasMilliRaw(INT32_MIN,0,0));
  assert(!ImuService_SetGyroBiasRaw(INT32_MAX,0,0));
  assert(Snapshot().gyro_bias_z_milli_raw==145500);
  ImuService_ClearGyroBias(); sample=Snapshot();
  assert(!sample.gyro_bias_valid && sample.gyro_bias_z_milli_raw==0 && sample.gyro_bias_z_raw==0);
  int32_t mean;
  assert(ImuBias_MeanMilliRaw(-26257,500,&mean) && mean==-52514);
  assert(!ImuBias_MeanMilliRaw(INT64_MIN,500,&mean));
  assert(!ImuBias_MeanMilliRaw(1,0,&mean));
  assert(ImuBias_MeanMilliRaw(1,2000,&mean) && mean==1);
  assert(ImuBias_MeanMilliRaw(-1,2000,&mean) && mean==-1);
  puts("PASS: actual 500-sample fractional calibration, exact Flash/service reboot chain and zero modeled mean without zeroing slow rotation");
}

int main(void)
{
  void *mapped=VirtualAlloc((void *)(uintptr_t)0x080C0000U,0x40000U,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
  assert(mapped==(void *)(uintptr_t)0x080C0000U);
  TestDriver();TestDataReady();TestStorage();TestFractionalStorage();TestCalibration();TestCalibrationQuality();TestFractionalCalibration();
  printf("MPU6500 profile fingerprint=0x%08lX; format=3 record_size=40 WHO/algorithm=0x0270 scale=1000\n",
         (unsigned long)s_expected_identity);
  assert(!critical_depth && !flash_unlocked);
  assert(VirtualFree(mapped,0,MEM_RELEASE));
  return 0;
}
