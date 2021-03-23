ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_SERIAL TRUE,$(HALCONF)),)
PLATFORMSRC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/hal_serial_lld.c ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/fsl_lpuart.c ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/fsl_clock.c
endif
else
PLATFORMSRC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/hal_serial_lld.c ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/fsl_lpuart.c ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/fsl_clock.c
endif

PLATFORMINC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1
