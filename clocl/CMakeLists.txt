if (SHANNON_BUILD OR HAWKING_CROSS_COMPILE)
  add_custom_command(OUTPUT x86/clocl  COMMAND make -j4
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  add_custom_target(x86_clocl DEPENDS x86/clocl)
  set(CROSS_TARGET cross)
  install(PROGRAMS x86/clocl DESTINATION bin/x86 ${OCL_BPERMS})  
endif()

if (HAWKING_BUILD)
 add_custom_command(OUTPUT arm/clocl COMMAND make -j4 ${CROSS_TARGET} TARGET=arm
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 add_custom_target(arm_clocl  DEPENDS arm/clocl)
 install(PROGRAMS arm/clocl DESTINATION bin/arm ${OCL_BPERMS})
endif()
