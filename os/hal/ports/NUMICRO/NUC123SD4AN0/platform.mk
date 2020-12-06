PLATFORMSRC := $(CHIBIOS)/os/hal/ports/common/ARMCMx/nvic.c \
               $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/NUC123SD4AN0/hal_lld.c

# Required include directories.
PLATFORMINC := $(CHIBIOS)/os/hal/ports/common/ARMCMx \
               $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/NUC123SD4AN0 \
               $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD

# Optional platform files.
ifeq ($(USE_SMART_BUILD),yes)
HALCONF := $(strip $(shell cat halconf.h | egrep -e "\#define"))

endif

# Drivers compatible with the platform.
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/GPIOv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/TIMv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/USBv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)