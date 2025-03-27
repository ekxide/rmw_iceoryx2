# benchmark

Scripts for gathering and plotting benchmark data.

## Test Procedure

1. Set up the environment for `rmw_iceoryx2` as per [these instructions](../README.md#Setup)
1. Clone `performance_test`
    ```console
    git clone -b 2.3.0 https://gitlab.com/ApexAI/performance_test.git ~/workspace/src/performance_test
    ```
1. Patch `performance_test` to recognize `rmw_iceoryx2_cxx` as zero-copy-capable
    1. NOTE: This shall soon be merged upstream to `performance_test` for convenience
    ```console
    cd ~/workspace/src/performance_test
    git apply ~/workspace/src/rmw_iceoryx2/benchmark/patch/recognize-rmw-iceoryx2-cxx-as-zero-copy.patch
    ```
1. Build `performance_test` and `rmw_iceoryx2_cxx`
    ```console
    cd ~/workspace/
    export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --build-base "build_perf_$RMW_IMPLEMENTATION" --install-base "install_perf_$RMW_IMPLEMENTATION" --packages-up-to "$RMW_IMPLEMENTATION" performance_test
    ```
1. Install dependencies into python env
    ```console
    cd ~/workspace/src/rmw_iceoryx2/benchmark/
    poetry install
    ```
1. Collect data
    ```console
    export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
    export ROS_DISABLE_LOANED_MESSAGES=0 # ensures loaning is enabled

    source ~/workspace/install_perf_$RMW_IMPLEMENTATION/setup.zsh
    cd ~/workspace/src/rmw_iceoryx2/benchmark
    poetry run python benchmark.py $RMW_IMPLEMENTATION ~/workspace/install_perf_$RMW_IMPLEMENTATION --zero-copy
    ```
1. Generate plots
    ```console
    cd ~/workspace/src/rmw_iceoryx2/benchmark
    poetry run python plot.py ./results
    ```

