## SPDX-License-Identifier: GPL-2.0-only

# Module name configuration (multiple modules possible)
obj-m += wizchip.o
obj-m += wizchip-spi.o

# Object files to be included in wizchip.ko
wizchip-objs := wizchip_driver.o \
                reg_define.o

# Auto-detect kernel build directory location
KDIR := /lib/modules/$(shell uname -r)/build
# Current source directory
PWD  := $(shell pwd)

# Build output directories
BUILD_DIR  := $(PWD)/build
OUTPUT_DIR := $(PWD)/output

.PHONY: all dirs clean

# Default build target
all: dirs
	# In-tree build: directly use kernel tree
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	# Move build artifacts
	@mv *.o   $(BUILD_DIR) 2>/dev/null || true
	@mv *.mod.c $(BUILD_DIR) 2>/dev/null || true
	@mv .*mod $(BUILD_DIR) 2>/dev/null || true
	@mv .*cmd $(BUILD_DIR) 2>/dev/null || true
	@mv .*d $(BUILD_DIR) 2>/dev/null || true
	@mv *.mod $(BUILD_DIR) 2>/dev/null || true
	@mv *o.cmd $(BUILD_DIR) 2>/dev/null || true
	@mv *.d $(BUILD_DIR) 2>/dev/null || true
	@mv *.symvers $(BUILD_DIR) 2>/dev/null || true
	@mv *.d    $(BUILD_DIR) 2>/dev/null || true
	# Move only .ko files to OUTPUT_DIR
	@mv *.ko  $(OUTPUT_DIR) 2>/dev/null || true
	# Display module load status
	@lsmod | grep wizchip || true

# Create output directories
dirs:
	@mkdir -p $(BUILD_DIR) $(OUTPUT_DIR)	

# Cleanup
clean:
	# In-tree clean
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	# Clear output directories
	@rm -rf $(BUILD_DIR)/* $(OUTPUT_DIR)/*