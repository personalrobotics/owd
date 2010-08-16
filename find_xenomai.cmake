find_program(XENO-CONFIG NAMES xeno-config PATHS $ENV{XENOMAI_PATH}/bin /usr/bin /usr/xenomai/bin)
if (XENO-CONFIG)
   execute_process(COMMAND ${XENO-CONFIG} --xeno-cflags
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE RT_DEFS
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
       message (SEND_ERROR "Could not find Xenomai include files (command xeno-config --xeno-cflags failed)\n")
   endif(XENOMAI_NOTFOUND)
   execute_process(COMMAND ${XENO-CONFIG} --xeno-ldflags
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE RT_LIBS
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
      message (SEND_ERROR "Could not find Xenomai libs (command xeno-config --xeno-ldflags failed)\n")
   endif (XENOMAI_NOTFOUND)
   
elseif ("$ENV{XENOMAI_PATH}" STRGREATER "")
message(STATUS "Using XENOMAI_PATH=$ENV{XENOMAI_PATH}")
   set (RT_DEFS "-I$ENV{XENOMAI_PATH}/include")
   set (RT_LIBS "-L$ENV{XENOMAI_PATH}/lib -lnative")
endif (XENO-CONFIG)

set (RT_DEFS "-DOWD_RT ${RT_DEFS}")
