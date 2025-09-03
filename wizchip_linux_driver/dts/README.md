# Device Tree Compilation Guide

## Overview
A single Linux driver can support **W5100, W5500, W6100, and W6300** chips.  
However, you must **define which chip will be used in the Device Tree** and compile the Device Tree overlay (DTBO) accordingly.

## Requirements
- `wizchip-driver.dts` (Device Tree source file)  
- `compile_dts.sh` (script to select chip and compile)  
- Device Tree Compiler (`dtc`) installed:
  ```bash
  sudo apt update
  sudo apt install device-tree-compiler
  ```

## Usage
Run the script with the desired chip name as an argument:

```bash

cd ./wizchip_linux_driver./dts
sudo chmod +x ./compile_dts.sh
# Example: generate overlay for W5100
./compile_dts.sh w5100

# Example: generate overlay for W5500
./compile_dts.sh w5500

# Example: generate overlay for W6100
./compile_dts.sh w6100

# Example: generate overlay for W6300
./compile_dts.sh w6300
```

## Output
The script updates the `compatible = "wiznet,<chip>"` property in the DTS file and compiles it.  
The compiled DTBO file will be generated in the following format:

```
<project>/output/<chip>-driver.dtbo   # e.g., output/w5500-driver.dtbo
```

## Applying on Raspberry Pi (example)
Copy the generated DTBO to the overlays directory and update `config.txt`:

```bash
sudo cp output/w5500-driver.dtbo /boot/firmware/overlays/
echo "dtoverlay=w5500-driver" | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

## Notes
- If no argument is given, the script will print the usage and exit.  
- Additional parameters like `interrupts` or `spi-max-frequency` can be extended in the script if needed.


