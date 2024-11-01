# Create an executable from the hello_pwm.c source file
add_executable(hello_pwm
    hello_pwm.c
)

# Pull in common dependencies, including the standard library, hardware_pwm for PWM control, and hardware_gpio for GPIO functionality needed by the encoder
target_link_libraries(hello_pwm pico_stdlib hardware_pwm hardware_gpio hardware_timer)

# Generate extra output files such as .bin and .uf2 which are useful for flashing and debugging
pico_add_extra_outputs(hello_pwm)

# Set a URL for the program's information page; useful for project documentation or version control webpages
example_auto_set_url(hello_pwm)

# Enable USB output for the stdio, allowing printf() statements to send output over the USB connection to a connected computer
pico_enable_stdio_usb(hello_pwm 1)