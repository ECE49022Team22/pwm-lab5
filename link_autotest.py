Import("env")

# Link the prebuilt autotest.o object into the firmware.
# It cannot go through build_flags/build_src_flags because PlatformIO would
# treat a ".o" token as a library and pass it to ld as "-l<path>".
# Adding it as a File node to PIOBUILDFILES makes SCons link it directly and
# handles the spaces in the project path correctly.
env.Append(PIOBUILDFILES=[env.File("$PROJECT_SRC_DIR/autotest.o")])
