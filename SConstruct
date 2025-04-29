#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

# For reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=["src/"])
sources = Glob("src/*.cpp")

# --- CUDA Configuration (Windows Only) ---
if env["platform"] == "windows":
    # Try to get CUDA path from environment, fallback to a common default or the user-provided one
    # Note: User provided include path, derive base path
    cuda_base_path_user = "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.8"
    cuda_path = os.environ.get("CUDA_PATH", cuda_base_path_user)

    if not cuda_path or not os.path.isdir(cuda_path):
        print("Warning: CUDA_PATH not found or invalid: {}. CUDA compilation disabled.".format(cuda_path))
        cuda_enabled = False
    else:
        print("Using CUDA Path: {}".format(cuda_path))
        cuda_enabled = True
        cuda_include_path = os.path.join(cuda_path, "include")
        cuda_lib_path = os.path.join(cuda_path, "lib", "x64") # Assuming 64-bit
        nvcc_path = os.path.join(cuda_path, "bin", "nvcc.exe")

        if not os.path.isfile(nvcc_path):
            print("Warning: nvcc.exe not found at {}. CUDA compilation disabled.".format(nvcc_path))
            cuda_enabled = False

    if cuda_enabled:
        # Add CUDA include path for C++ files
        env.Append(CPPPATH=[cuda_include_path])

        # Add CUDA lib path for linker
        env.Append(LIBPATH=[cuda_lib_path])

        # Enable C++ Exceptions for MSVC when CUDA is enabled (since CUDA wrapper uses try/catch)
        env.Append(CXXFLAGS=["/EHsc"])

        # Define NVCC flags (adjust sm_XX to your target GPU architecture)
        nvcc_flags = [] # Start fresh

        # Add src path for includes within src (e.g., boid_cuda.h)
        src_include_path = os.path.join("src")
        nvcc_flags.extend([
            "-I" + src_include_path # Add src path
        ])
        # REMOVED Godot-CPP include paths from NVCC flags, as they caused compilation errors in .cu files.
        # The C++ compiler will handle Godot includes for wrapper files via CPPPATH.
        # godot_cpp_include_path = os.path.join("godot-cpp", "include")
        # godot_cpp_gen_include_path = os.path.join("godot-cpp", "gen", "include")
        # godot_cpp_gdextension_include_path = os.path.join("godot-cpp", "gdextension")
        # nvcc_flags.extend([
        #     "-I" + godot_cpp_include_path,
        #     "-I" + godot_cpp_gen_include_path,
        #     "-I" + godot_cpp_gdextension_include_path,
        # ])

        # Add target architecture (Example: Turing architecture)
        # nvcc_flags.append("-arch=sm_75") # Uncomment and adjust if needed

        if env["target"] == "debug":
            nvcc_flags.extend(["-G", "-g"])
        else:
            nvcc_flags.extend(["--use_fast_math"]) # Optional: Optimization

        env["NVCC"] = nvcc_path
        # Store flags as a list in the environment
        env["NVCCFLAGS"] = nvcc_flags
        # Define CUDA source file suffix and object file suffix
        env["NVCCSUFFIX"] = ".cu"
        env["OBJSUFFIX"] = env.subst('$OBJSUFFIX') # Get object suffix from env (e.g., .obj)

        # Define a function for the CUDA build action
        def cuda_build_action(target, source, env):
            # NVCC path - ensure it's quoted if it contains spaces
            nvcc_cmd = '"{}"'.format(env["NVCC"]) if " " in env["NVCC"] else env["NVCC"]
            # Retrieve flags from environment and join them into a string
            flags_str = " ".join(env["NVCCFLAGS"]) # Correctly get flags here
            # Construct the command string for Execute
            cmd_str = "{} {} -c -o {} {}".format(
                nvcc_cmd,
                flags_str,
                str(target[0]),
                str(source[0])
            )
            print("Executing CUDA command:", cmd_str) # Debug print
            # Use env.Execute with the constructed command string and pass the environment
            # Passing env['ENV'] ensures PATH is inherited for cl.exe
            return env.Execute(cmd_str, env=env['ENV'])

        # Define a builder for .cu files using the function action
        cuda_builder = Builder(
            action = cuda_build_action,
            suffix = env["OBJSUFFIX"],
            src_suffix = env["NVCCSUFFIX"]
        )
        env.Append(BUILDERS={"CudaObject": cuda_builder})

        # Find and compile CUDA sources
        cuda_sources = Glob("src/*.cu")
        cuda_objects = [env.CudaObject(s) for s in cuda_sources]

        # Add CUDA runtime library for linking (static linking is often preferred)
        env.Append(LIBS=["cudart_static"])

else:
    # Platform is not Windows, disable CUDA
    cuda_enabled = False
    cuda_objects = []

sources = Glob("src/*.cpp")

# Combine C++ and CUDA objects (if CUDA enabled)
all_objects = sources + cuda_objects

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "demo/bin/libgdexample.{}.{}.framework/libgdexample.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=all_objects, # Use combined list
    )
elif env["platform"] == "ios":
    if env["ios_simulator"]:
        library = env.StaticLibrary(
            "demo/bin/libgdexample.{}.{}.simulator.a".format(env["platform"], env["target"]),
            source=all_objects, # Use combined list
        )
    else:
        library = env.StaticLibrary(
            "demo/bin/libgdexample.{}.{}.a".format(env["platform"], env["target"]),
            source=all_objects, # Use combined list
        )
else:
    library = env.SharedLibrary(
        "demo/bin/libgdexample{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=all_objects, # Use combined list
    )

Default(library)
