add_executable(gy-511 gy-511.c)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(gy-511 pico_stdlib hardware_i2c)

pico_enable_stdio_usb(gy-511 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(gy-511)

# add url via pico_set_program_url
example_auto_set_url(gy-511)
