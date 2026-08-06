# Compiling the Code
For this guide, we assume that you meet the following requirements: 

1. You have already cloned the repository to your machine.
2. You have either set up the [native installation dependencies](install/native.md) or are working inside a [dev container](install/dev-container.md).

Open a terminal and navigate into the repository (`cd /path/to/the/repo/`).

There, create a new folder `build` and enter it:
```bash
mkdir build
cd build
```

Then, compile the code using CMake:
```bash
cmake ..
cmake --build . -j4
```

If everything succeeded, you can now run the following commands to interact with the dog:
```bash
./sit-down
./stand-up
```