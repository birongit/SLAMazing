FILE(GLOB CSparseSources *.c)

add_library(csparse cs.h ${CSparseSources})
