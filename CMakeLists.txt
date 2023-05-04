#--------------------------------------------------------
# The CMakeLists.txt for:                       pGenRescue
# Author(s):                              Adam Pressel
#--------------------------------------------------------

SET(SRC
  GenRescue.cpp
  GenRescue_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pGenRescue ${SRC})

TARGET_LINK_LIBRARIES(pGenRescue
   ${MOOS_LIBRARIES}
   contacts
   apputil
   geometry
   mbutil
   m
   pthread)

