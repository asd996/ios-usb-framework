GO_EASY_ON_ME = 1

include $(THEOS)/makefiles/common.mk

ADDITIONAL_CFLAGS = -I../../libusb/include -I./dosfs
ADDITIONAL_LDFLAGS = -L../../libusb/lib -lusb-1.0

TOOL_NAME = usb_ls usb_cp

usb_ls_FILES = ls.c
usb_ls_SUBPROJECTS = dosfs

usb_cp_FILES = ls.c
usb_cp_SUBPROJECTS = dosfs

include $(THEOS_MAKE_PATH)/subproject.mk
include $(THEOS_MAKE_PATH)/tool.mk