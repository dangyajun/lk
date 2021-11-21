LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/debug.cpp \
	$(LOCAL_DIR)/pci.cpp \
\
	$(LOCAL_DIR)/backend/ecam.cpp \
	$(LOCAL_DIR)/backend/bios32.cpp \
	$(LOCAL_DIR)/backend/type1.cpp \

MODULE_DEPS += lib/libcpp

include make/module.mk
