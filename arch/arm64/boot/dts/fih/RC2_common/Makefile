ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_SDM845) += \
		sdm845-v2-mtp-overlay_rc2_pre-evt.dtbo \
		sdm845-v2-mtp-overlay_rc2_pre-evt2.dtbo \
		sdm845-v2-mtp-overlay_rc2_evt1.dtbo \
		sdm845-v2-mtp-overlay_rc2_dvt.dtbo \
		sdm845-v2-mtp-overlay_rc2_pvt.dtbo \
		sdm845-v2-mtp-overlay_rc2_mp.dtbo

sdm845-v2-mtp-overlay_rc2_pre-evt.dtbo-base := sdm845-v2_rc2.dtb
sdm845-v2-mtp-overlay_rc2_pre-evt2.dtbo-base := sdm845-v2_rc2.dtb
sdm845-v2-mtp-overlay_rc2_evt1.dtbo-base := sdm845-v2_rc2.dtb
sdm845-v2-mtp-overlay_rc2_dvt.dtbo-base := sdm845-v2_rc2.dtb
sdm845-v2-mtp-overlay_rc2_pvt.dtbo-base := sdm845-v2_rc2.dtb
sdm845-v2-mtp-overlay_rc2_mp.dtbo-base := sdm845-v2_rc2.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
