GCC_VERSION      := $(shell $(CONFIG_SHELL) $(srctree)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
ccflags-y += -Idrivers/media/video/msm
ccflags-y += -Idrivers/media/platform/msm/camera_v2_lenovo/common
ccflags-y += -Idrivers/media/platform/msm/camera_v2_lenovo
ccflags-y += -Idrivers/media/platform/msm/camera_v2_lenovo/pproc/cpp
ccflags-y += -Idrivers/media/platform/msm/camera_v2_lenovo/msm_buf_mgr/

obj-$(CONFIG_MSM_FD) += msm_fd_dev.o msm_fd_hw.o
