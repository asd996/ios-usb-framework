include $(THEOS)/makefiles/common.mk

internal-stage::
	$(ECHO_NOTHING)mkdir -p $(THEOS_STAGING_DIR)/usr/include$(ECHO_END)
	$(ECHO_NOTHING)cp -r include/ $(THEOS_STAGING_DIR)/usr/include$(ECHO_END)
	
	$(ECHO_NOTHING)mkdir -p $(THEOS_STAGING_DIR)/usr/lib$(ECHO_END)
	$(ECHO_NOTHING)cp -r lib/ $(THEOS_STAGING_DIR)/usr/lib$(ECHO_END)
	
internal-after-install::
