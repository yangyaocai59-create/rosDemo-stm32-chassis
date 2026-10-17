"""Freeze interface-verified 0.14.3 delivery; no device operations, no overwrite.

VERIFIED refers to firmware RAW interface/calibration/restore, not ROS bias
estimator or corrected-yaw acceptance. Requires actual completed evidence.
"""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

from package_imu_raw_release import digest


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = {
    'old_baseline': 'imu_stationary_20261016_093044_334143',
    'old_recalibration': 'imu_stationary_20261016_093626_886016',
    'old_restart': 'imu_stationary_20261016_094113_934068',
    'raw_before_calibration': 'imu_raw_20261016_100648_722828',
    'new_calibration': 'imu_stationary_20261016_100712_748143',
    'raw_after_calibration': 'imu_raw_20261016_100928_889027',
    'raw_after_restart': 'imu_raw_20261016_101504_525621',
}


def read_evidence(name):
    return json.loads((ROOT/'build/hardware_tests'/EVIDENCE[name]/'summary.json').read_text())


def read_audit(name):
    audit=json.loads((ROOT/'build/hardware_tests'/EVIDENCE[name]/'full_receive_analysis.json').read_text())
    assert audit['result']=='PASS_FULL_RECEIVE_LOG_AUDIT'
    assert audit['source_sample_count_gaps']==0 and audit['crc_errors']==0
    assert audit['status_crc_rxoverflow_txdrop']==[0,0,0]
    assert audit['wheel_count_change']==[0,0,0,0]
    return audit


def validate_raw(summary, flags):
    assert summary['result']=='PASS_INTERFACE_ONLY' and not summary['errors']
    assert summary['baseline']==[-55,26,150] and summary['flags']==[flags]
    assert summary['crc_errors']==0 and summary['raw']['read_error_counts']==[0]
    assert summary['raw']['consecutive_error_counts']==[0]
    assert summary['raw']['samples']>=11900 and 99<=summary['raw']['source_rate_hz']<=101
    assert summary['uart_raw_exact_matches']>=119 and summary['uart_raw_mismatches']==0
    assert summary['dtr_reset_to_legacy']=='PASS'
    assert summary['switch_back_and_session_expiry']=='PASS'
    assert summary['motion_commands']==summary['calibration_commands']==0
    assert all(t['type'] in (4,6) for t in summary['tx'])
    assert not summary['uart']['calibration_start_lines'] and not summary['uart']['flash_save_lines']


def main():
    before=read_evidence('raw_before_calibration')
    assert before['result']=='PASS_INTERFACE_ONLY' and before['baseline']==[0,0,0] and before['flags']==[5]
    calibrated=read_evidence('new_calibration')
    assert calibrated['capture_result']=='complete' and not calibrated['errors']
    assert calibrated['calibration_commands_sent']==1 and calibrated['motion_commands_sent']==0
    assert calibrated['calibration']['flash_save_confirmed']
    assert calibrated['uart']['bias_raw']==[-55,26,150]
    assert len(calibrated['uart']['flash_save_lines'])==1
    assert 'seq=1' in calibrated['uart']['flash_save_lines'][0]
    post=read_evidence('raw_after_calibration');restored=read_evidence('raw_after_restart')
    validate_raw(post,0x37);validate_raw(restored,0x07)
    post_audit=read_audit('raw_after_calibration');restore_audit=read_audit('raw_after_restart')
    assert restore_audit['baseline_values']==[[-55,26,150]]
    assert restore_audit['raw']['flags']==[7]
    assert not restore_audit['calibration_start_lines'] and not restore_audit['flash_save_lines']
    verification=ROOT/'build/verification/imu_raw_0.14.3_20261016'
    artifact=json.loads((verification/'artifact_verification.json').read_text())
    candidate=ROOT/'build/deliveries/imu_raw_0.14.3_20261016'
    baseline_status=json.loads((candidate/'release_status.json').read_text())
    for name,expected in artifact['sha256'].items():
        assert digest(ROOT/'build/Debug'/name)==expected
        assert digest(candidate/'firmware'/name)==expected
    # Never expand the hardware change scope while packaging.
    changes=[]
    with zipfile.ZipFile(ROOT/'build/deliveries/mpu6500_ros_handoff_0.14.2_verified_20261014/source_snapshot.zip') as old:
        for name in old.namelist():
            if name.startswith(('Application/','Core/','Drivers/','Middlewares/','USB_DEVICE/','cmake/')) or name.endswith(('.ioc','.ld')) or name in ('CMakeLists.txt','CMakePresets.json','startup_stm32f407xx.s'):
                assert (ROOT/name).is_file()
                if (ROOT/name).read_bytes()!=old.read(name):changes.append(name)
    assert sorted(changes)==sorted(baseline_status['firmware_files_changed'])
    destination=ROOT/'build/deliveries/imu_raw_0.14.3_interface_verified_20261016'
    archive=destination.with_name('STM32_IMU_RAW_0.14.3_INTERFACE_VERIFIED_20261016.zip')
    assert not archive.exists() and not destination.exists(), 'refuse to overwrite frozen delivery'
    destination.mkdir(parents=True,exist_ok=False)
    for folder in ('firmware','docs','verification','evidence'):(destination/folder).mkdir()
    for name in artifact['sha256']:shutil.copy2(candidate/'firmware'/name,destination/'firmware'/name)
    docs=['STM32_IMU_RAW_V1_HANDOFF_20261016.md','STM32_IMU_RAW_HARDWARE_VERIFICATION_20261016.md',
          'IMU_DRIFT_CORRECTION_PLAN_20261016.md','ROS_IMU_DRIFT_ENGINEER_NOTICE_20261016.md',
          'IMU_STATIC_DRIFT_DIAGNOSIS_20261016.md','ROS2_UBUNTU_HANDOFF.md']
    for name in docs:shutil.copy2(ROOT/'docs'/name,destination/'docs'/name)
    for name in ('configure.log','debug_clean_build.log','native_tests.log','python_tests.log',
                 'legacy_python_selftest.log','collector_regression.log','artifact_verification.json'):
        shutil.copy2(verification/name,destination/'verification'/name)
    for name in EVIDENCE.values():
        shutil.copytree(ROOT/'build/hardware_tests'/name,destination/'evidence'/name)
    status=dict(baseline_status)
    status.update(status='INTERFACE_CALIBRATION_RESTORE_HARDWARE_VERIFIED',
        user_reports_flashed=True,assistant_flashed=False,new_firmware_hardware_tests_run=True,
        flash_readback_identity_verified=False,boot_banner_captured=False,
        saved_and_restored_baseline=[-55,26,150],saved_sequence=1,
        hardware_raw_after_calibration=post_audit['raw'],hardware_raw_after_restart=restore_audit['raw'],
        raw_statistics_source='full_receive_analysis.json: complete USB bytes/log cross-check',
        original_collector_suffix_issue_fixed=True,python_capture_regressions_passed=1,
        dtr_legacy_restore_verified=True,session_timeout_legacy_restore_verified=True,
        ros_estimator_implemented=False,final_yaw_drift_target_verified=False,
        new_version_physical_motion_disconnect_tested=False,evidence=EVIDENCE)
    (destination/'release_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf8')
    (destination/'README_FIRST.md').write_text(
        '# 0.14.3：固件 RAW 接口、显式校准和断电恢复已验证\n\n'
        '这里的 INTERFACE_VERIFIED 不表示最终 ROS 静止漂移修复完成。\n'
        '用户已经烧录本包同一 HEX，完成一次校准并普通断电恢复；无需为了交接再次刷机。\n'
        '版本 0.14.3-imu-raw-v1，Build Oct 16 2026 09:56:40；无 Git 元数据，以 SHA256 和源码快照定位。\n'
        'Error 0、Warning 0；FLASH 159508 B、RAM 54936 B、CCMRAM 0 B。\n\n'
        '已保存并恢复基准 [-55,26,150]，seq=1；重上电 RAW flags=0x07，没有再次校准/保存。\n'
        '若以后重刷，用户的全片擦除流程会清掉记录，刷后需重新显式校准一次。\n'
        '助手未擦除、烧录或主动复位开发板；镜像身份依据用户报告和新增接口实测，未全量回读 Flash。\n\n'
        'ROS 工程师先读 docs/ROS_IMU_DRIFT_ENGINEER_NOTICE_20261016.md，\n'
        '再读 docs/STM32_IMU_RAW_V1_HANDOFF_20261016.md 和 docs/IMU_DRIFT_CORRECTION_PLAN_20261016.md。\n'
        'STM32 不再自动校准、不动态修正零偏；ROS 独立模块唯一负责 RAM 中的残余 Z 偏置估计。\n'
        'RAW/基准字段、模式命令和精确布局已冻结；不要改变旧 0x84 的单位。\n'
        '实际硬件记录与未验项目见 docs/STM32_IMU_RAW_HARDWARE_VERIFICATION_20261016.md。\n\n'
        'firmware/：HEX、ELF、BIN（BIN 基地址 0x08000000）；source_snapshot.zip：完整工程/工具/文档。\n'
        'evidence/：旧漂移对照和本版校准/RAW/断电恢复的完整原始日志。\n'
        'verification/：真实构建、native/Python 回归和镜像一致性日志。\n'
        'SHA256SUMS.txt 校验整个交付；SOURCE_SHA256SUMS.txt 校验内部源码快照。\n\n'
        '尚未完成：ROS 估计器和 corrected 漂移验收、新版本实际运动中拔线/启停等回归。\n'
        '不要将本包接口验收扩大为传感器零漂或整车安全认证。\n',encoding='utf8')
    source=[]
    for name in ('Application','Core','Drivers','Middlewares','USB_DEVICE','cmake','Tools','docs'):
        source.extend(p for p in (ROOT/name).rglob('*') if p.is_file() and
                      '__pycache__' not in p.parts and p.suffix not in ('.pyc','.exe','.o','.obj'))
    for name in ('CMakeLists.txt','CMakePresets.json','rosDemo.ioc','STM32F407xx_FLASH.ld',
                 'startup_stm32f407xx.s','.gitignore'):source.append(ROOT/name)
    with zipfile.ZipFile(destination/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(source):z.write(p,p.relative_to(ROOT).as_posix())
    (destination/'SOURCE_SHA256SUMS.txt').write_text(''.join(
        digest(p)+'  '+p.relative_to(ROOT).as_posix()+'\n' for p in sorted(source)),encoding='utf8')
    (destination/'SHA256SUMS.txt').write_text(''.join(
        digest(p)+'  '+p.relative_to(destination).as_posix()+'\n'
        for p in sorted(destination.rglob('*')) if p.is_file()),encoding='utf8')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(destination.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(destination).as_posix())
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for line in z.read('SHA256SUMS.txt').decode().splitlines():
            expected,name=line.split('  ',1)
            assert hashlib.sha256(z.read(name)).hexdigest()==expected
    print('INTERFACE-VERIFIED DELIVERY: '+str(archive))
    print('SHA256: '+digest(archive))
    print('Bytes: '+str(archive.stat().st_size)+'; sources: '+str(len(source)))


if __name__=='__main__':main()
