
# generic source files - need to add system/crypto before system to properly generated build dirs.. weird.
COMPONENT_SRCDIRS := src/board src/mac src/system/crypto src/system 

# add include directories
COMPONENT_ADD_INCLUDEDIRS := include/lora/board include/lora/system include/lora/mac include/lora/radio include/lora/system/crypto

# generic header files for compilation
#COMPONENT_PRIV_INCLUDEDIRS := include/lora/board include/lora/system include/lora/mac include/lora/radio include/lora/system/crypto

# selects which radio module to add to compile and header sources
ifdef CONFIG_LORA_RADIO_MODULE_SX1276
COMPONENT_SRCDIRS += src/radio/sx1276
COMPONENT_ADD_INCLUDEDIRS += include/lora/radio/sx1276
CFLAGS += -DLORA_RADIO_SX1276
endif
ifdef CONFIG_LORA_RADIO_MODULE_SX1272
COMPONENT_SRCDIRS += src/radio/sx1272
COMPONENT_ADD_INCLUDEDIRS += include/lora/radio/sx1272
CFLAGS += -DLORA_RADIO_SX1272
endif