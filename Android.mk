ifneq ($(ONE_SHOT_MAKEFILE),)

ANDROID_MK_GOAL=$(filter-out all_modules,$(MAKECMDGOALS))
$(info "The build target is: ${ANDROID_MK_GOAL}")

LENOVO_ARCH_COMPILE_OPT=ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
LENOVO_KCFLAGS_OPT=KCFLAGS=-mno-android
LENOVO_KERNEL_OBJ_DIR_OPT=O=../out/target/product/$(TARGET_PRODUCT)/obj/KERNEL_OBJ
LENOVO_JOB_NUM=-j8

VALID_MK_GOAL=$(filter bootimg recoveryimg,$(ANDROID_MK_GOAL))
$(info "The valid is: ${VALID_MK_GOAL}")

ifneq ($(VALID_MK_GOAL),)
LENOVO_HOST_OS=`uname -s|tr A-Z a-z`
LENOVO_OUT_HOSTBIN_DIR="out/host/$(LENOVO_HOST_OS)-x86/bin"
LENOVO_KERNEL_OBJ_DIR="out/target/product/$(TARGET_PRODUCT)/obj/KERNEL_OBJ"

.PHONY: bootimg
bootimg:
	@echo "building kernel ......"
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) $(KERNEL_DEFCONFIG) && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) headers_install && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) modules && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) INSTALL_MOD_PATH=../../system INSTALL_MOD_STRIP=1 modules_install
	@echo "generating dt.img ......"
	$(LENOVO_OUT_HOSTBIN_DIR)/dtbTool -o $(ANDROID_PRODUCT_OUT)/dt.img -s 2048 -p $(LENOVO_KERNEL_OBJ_DIR)/scripts/dtc/ $(LENOVO_KERNEL_OBJ_DIR)/arch/arm64/boot/dts/qcom/
	cp $(LENOVO_KERNEL_OBJ_DIR)/arch/arm64/boot/Image $(ANDROID_PRODUCT_OUT)/kernel
	@echo "generating bootimage ramdisk.img"
	$(LENOVO_OUT_HOSTBIN_DIR)/mkbootfs $(ANDROID_PRODUCT_OUT)/root | $(LENOVO_OUT_HOSTBIN_DIR)/minigzip > $(ANDROID_PRODUCT_OUT)/ramdisk.img
	make bootimage-nodeps

.PHONY: recoveryimg
recoveryimg:
	@echo "building kernel ......"
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) $(KERNEL_DEFCONFIG) && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) headers_install && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) modules && \
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) INSTALL_MOD_PATH=../../system INSTALL_MOD_STRIP=1 modules_install
	@echo "generating dt.img ......"
	$(LENOVO_OUT_HOSTBIN_DIR)/dtbTool -o $(ANDROID_PRODUCT_OUT)/dt.img -s 2048 -p $(LENOVO_KERNEL_OBJ_DIR)/scripts/dtc/ $(LENOVO_KERNEL_OBJ_DIR)/arch/arm64/boot/dts/qcom/
	cp $(LENOVO_KERNEL_OBJ_DIR)/arch/arm64/boot/Image $(ANDROID_PRODUCT_OUT)/kernel
	@echo "generating recovery ramdisk.img"
	$(LENOVO_OUT_HOSTBIN_DIR)/mkbootfs $(ANDROID_PRODUCT_OUT)/recovery/root | $(LENOVO_OUT_HOSTBIN_DIR)/minigzip > $(ANDROID_PRODUCT_OUT)/ramdisk-recovery.img
	make recoveryimage-nodeps

else
KERNEL_BUILD_TARGETS=$(subst kernel-,,$(ANDROID_MK_GOAL))
$(ANDROID_MK_GOAL):
	make -C kernel $(LENOVO_KERNEL_OBJ_DIR_OPT) $(LENOVO_ARCH_COMPILE_OPT) $(LENOVO_KCFLAGS_OPT) $(LENOVO_JOB_NUM) $(KERNEL_BUILD_TARGETS)

endif
endif
