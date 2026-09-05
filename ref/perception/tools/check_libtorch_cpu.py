#!/usr/bin/env python3
import os, sys
print(f"python={sys.executable}")
try:
    import torch
except Exception as e:
    print(f"torch_import=FAIL: {e}")
    raise SystemExit(2)
abi_fn = getattr(torch, "compiled_with_cxx11_abi", None)
abi = abi_fn() if callable(abi_fn) else getattr(torch._C, "_GLIBCXX_USE_CXX11_ABI", None)
print(f"torch_version={torch.__version__}")
print(f"torch_file={torch.__file__}")
print(f"torch_cmake_prefix={torch.utils.cmake_prefix_path}")
print(f"cxx11_abi={abi}")
print(f"cuda_available={torch.cuda.is_available()}")
print(f"torch_lib={os.path.join(os.path.dirname(torch.__file__), 'lib')}")
if abi is False:
    print("compatible_ros2_humble_cpp=NO")
    raise SystemExit(3)
print("compatible_ros2_humble_cpp=YES_OR_UNKNOWN")
