Import("env")

env.AddCustomTarget("upload_only", "$BUILD_DIR/${PROGNAME}.elf", "")
