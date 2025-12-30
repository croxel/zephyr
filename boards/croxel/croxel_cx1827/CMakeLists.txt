if(CONFIG_BOARD_CROXEL_CX1827_NRF54H20_CPUAPP)
	# Add board-specific source files
	zephyr_library()
	zephyr_library_sources(peripheral_init.c)

	if (CONFIG_USB_DEVICE_STACK_NEXT)
		zephyr_library_include_directories(${ZEPHYR_BASE}/samples/subsys/usb/common/)
		zephyr_library_sources(${ZEPHYR_BASE}/samples/subsys/usb/common/sample_usbd_init.c)
		zephyr_library_sources(usbd_next_init.c)
	endif()
endif()
