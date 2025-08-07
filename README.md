# qi_sdk
qi robot sdk .

### Build examples

To build the examples inside this repository:

```bash
mkdir build
cd build
cmake ..
make
```

### Suggested installation location

To build your own application with the SDK, you can install the qi_sdk to your specified directory:

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="~/Programs/qi_sdk_install"
make install
```

### Install in the system directory by default

You can install the qi_sdk to your system directory:

```bash
mkdir build
cd build
cmake ..
sudo make install
```


