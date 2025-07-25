#development environment

```bash
lihan@raspberrypi:~ $ uname -r
6.6.51+rpt-rpi-v8
lihan@raspberrypi:~ $ cat /etc/os-release | grep PRETTY_NAME
PRETTY_NAME="Debian GNU/Linux 12 (bookworm)"
lihan@raspberrypi:~ $ gcc --version | head -n 1
gcc (Debian 12.2.0-14) 12.2.0
lihan@raspberrypi:~ $ make --version | head -n 1
GNU Make 4.3
lihan@raspberrypi:~ $ uname -m
aarch64
```
```bash
wizchip_Linux_driver
├── build                   #The build artifacts are stored here.
├── dts
│   ├── \
│   ├── compile_dts.sh
│   └── w5500-driver.dts
├── install_wizchip_driver.sh
├── Kconfig
├── Makefile
├── output                  #The build artifacts are stored here.
├── reg_define.c
├── reg_define.h
├── w5300.c
├── wizchip_driver.c
├── wizchip_driver.h
└── wizchip-spi.c

4 directories, 12 files
```



