{
  "targets": [{
    "target_name": "physics",
    "sources": [
      "src/physics_wrapper.cpp",
      "src/physics.cpp",
      "src/World.cpp",
      "src/RigidBody.cpp",
      "src/Collision.cpp",
      "src/UniformGridBroadPhase.cpp",
      "src/MathUtils.cpp",
      "src/Timer.cpp"
    ],
    "include_dirs": [
      "<!@(node -p \"require('node-addon-api').include\")",
      "src"
    ],
    "dependencies": [
      "<!(node -p \"require('node-addon-api').gyp\")"
    ],
    "cflags!": [ "-fno-exceptions" ],
    "cflags_cc!": [ "-fno-exceptions" ],
    "xcode_settings": {
      "GCC_ENABLE_CPP_EXCEPTIONS": "YES",
      "CLANG_CXX_LIBRARY": "libc++",
      "MACOSX_DEPLOYMENT_TARGET": "10.7"
    },
    "msvs_settings": {
      "VCCLCompilerTool": { 
        "ExceptionHandling": 1,
        "EnableEnhancedInstructionSet": "2",
        "PlatformToolset": "v143",
        "WindowsTargetPlatformVersion": "10.0"
      }
    },
    "defines": [ "NAPI_DISABLE_CPP_EXCEPTIONS" ],
    "conditions": [
      ['OS=="win"', {
        "defines": [
          "NOMINMAX",
          "_USE_MATH_DEFINES"
        ]
      }]
    ]
  }]
} 