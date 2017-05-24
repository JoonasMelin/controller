###| CMake Kiibohd Controller Scan Module |###
#
###


###
# Sub-module flag, cannot be included stand-alone
#
set ( SubModule 1 )
#AddModule ( Scan Devices/ISSILed )
AddModule ( Scan I2CConnect )

###
# Module C files
#
set ( Module_SRCS
        mcp23018.c
)


###
# Compiler Family Compatibility
#
set ( ModuleCompatibility
	arm
)

