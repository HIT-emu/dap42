## Copyright (c) 2016, Devan Lai
##
## Permission to use, copy, modify, and/or distribute this software
## for any purpose with or without fee is hereby granted, provided
## that the above copyright notice and this permission notice
## appear in all copies.
##
## THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
## WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
## WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
## AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
## CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
## LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
## NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
## CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q              := @
NULL           := 2>/dev/null
MAKE           := $(MAKE) --no-print-directory
endif
export V

FORMATS ?= hex dfu

BUILD_DIR      ?= ./build

all: DAP42 DAP42DC KITCHEN42 \
     DAP103 DAP103-DFU \
     DAP103-BLUEPILL DAP103-BLUEPILL-DFU \
     DAP103-NUCLEO-STBOOT \
     BRAINv3.3 \
     DAP42K6U \
     UMDK-EMB UMDK-RF
clean:
	$(Q)$(RM) $(BUILD_DIR)/*.bin
	$(Q)$(MAKE) -C src/ clean
	$(Q)$(MAKE) -C libopencm3/ clean

.PHONY = all clean

$(BUILD_DIR):
	$(Q)mkdir -p $(BUILD_DIR)
	
UMDK-RF:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=$(@) -C src/ clean
	$(Q)$(MAKE) TARGET=$(@) -C src/ $(FORMATS)
	$(Q)$(MAKE) TARGET=$(@) copy
	
UMDK-EMB:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=$(@) -C src/ clean
	$(Q)$(MAKE) TARGET=$(@) -C src/ $(FORMATS)
	$(Q)$(MAKE) TARGET=$(@) copy

DAP42:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F042 -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F042 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

DAP42DC:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=DAP42DC -C src/ clean
	$(Q)$(MAKE) TARGET=DAP42DC -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

KITCHEN42:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=KITCHEN42 -C src/ clean
	$(Q)$(MAKE) TARGET=KITCHEN42 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

DAP103:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F103 -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F103 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

DAP103-DFU:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F103-DFUBOOT -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F103-DFUBOOT -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

DAP103-BLUEPILL:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F103-BLUEPILL -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F103-BLUEPILL -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

DAP103-BLUEPILL-DFU:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F103-BLUEPILL-DFUBOOT -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F103-BLUEPILL-DFUBOOT -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

BRAINv3.3:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=BRAINV3.3 -C src/ clean
	$(Q)$(MAKE) TARGET=BRAINV3.3 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

DAP42K6U:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=DAP42K6U -C src/ clean
	$(Q)$(MAKE) TARGET=DAP42K6U -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

DAP103-NUCLEO-STBOOT:| $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STLINKV2-1-STBOOT -C src/ clean
	$(Q)$(MAKE) TARGET=STLINKV2-1-STBOOT -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@)

copy:
	$(Q)cp src/DAP42.hex $(BUILD_DIR)/$(TARGET).hex
	$(Q)cp src/DAP42.dfu $(BUILD_DIR)/$(TARGET).dfu
	$(Q)$(MAKE) TARGET=$(TARGET) -C src/ size