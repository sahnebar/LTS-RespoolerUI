import shutil
import os
import subprocess

build_dir = "/root/LTS-Respooler/Firmware/build"
target_dir = "/root/LTS-Respooler/Firmware"
esptool_path = "/root/.platformio/packages/tool-esptoolpy/esptool.py"
python_path = "/root/venv/bin/python3"

envs = {
    "control_board": {
        "chip": "esp32",
        "flash_size": "4MB",
        "boot_offset": "0x1000",
        "output_name": "LTS_Respooler_ControlBoard_Web"
    },
    "driver_board": {
        "chip": "esp32",
        "flash_size": "4MB",
        "boot_offset": "0x1000",
        "output_name": "LTS_Respooler_DriverBoard_Web"
    },
    "esp32s3_generic": {
        "chip": "esp32s3",
        "flash_size": "8MB",
        "boot_offset": "0x0",
        "output_name": "LTS_Respooler_S3_Generic_Web"
    },
    "esp32c3_generic": {
        "chip": "esp32c3",
        "flash_size": "4MB",
        "boot_offset": "0x0",
        "output_name": "LTS_Respooler_C3_Generic_Web"
    }
}

for env, config in envs.items():
    env_build_path = os.path.join(build_dir, ".pio", "build", env)
    app_bin = os.path.join(env_build_path, "firmware.bin")
    boot_bin = os.path.join(env_build_path, "bootloader.bin")
    part_bin = os.path.join(env_build_path, "partitions.bin")
    
    if not os.path.exists(app_bin):
        print(f"Error: Build files for {env} not found at {app_bin}. Skipping.")
        continue
        
    # Copy app binary (OTA)
    target_app = os.path.join(target_dir, f"{config['output_name']}.bin")
    shutil.copyfile(app_bin, target_app)
    print(f"Copied OTA binary: {target_app}")
    
    # Merge binary (Flash)
    target_merged = os.path.join(target_dir, f"{config['output_name']}_Merged.bin")
    
    merge_cmd = [
        python_path, esptool_path,
        "--chip", config["chip"],
        "merge_bin",
        "-o", target_merged,
        "--flash_mode", "dio",
        "--flash_freq", "40m",
        "--flash_size", config["flash_size"],
        config["boot_offset"], boot_bin,
        "0x8000", part_bin,
        "0x10000", app_bin
    ]
    
    print(f"Merging {env}...")
    res = subprocess.run(merge_cmd, capture_output=True, text=True)
    if res.returncode == 0:
        print(f"Created Merged binary: {target_merged}")
    else:
        print(f"Failed to merge {env}: {res.stderr}")

print("Done copying and merging binaries!")
