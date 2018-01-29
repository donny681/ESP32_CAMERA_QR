# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)/components/json/include $(IDF_PATH)/components/json/port/include
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/json -ljson
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += json
component-json-build: 
