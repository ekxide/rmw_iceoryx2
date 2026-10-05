# performance_test

Scripts for gathering and plotting latency data using the standardized [performance_test](https://gitlab.com/ApexAI/performance_test) tool.

## Test Procedure

1. Set up the environment for `rmw_iceoryx2` as per [these instructions](../README.md#Setup)
1. Clone `performance_test`
    1. NOTE: Release 2.3.0 and `master` don't build on Rolling, which removed `ament_target_dependencies`. This commit of the unmerged `christophebedard/support-lyrical` branch adds Rolling support
    ```console
    git clone https://gitlab.com/ApexAI/performance_test.git ~/workspace/src/performance_test
    git -C ~/workspace/src/performance_test checkout 00a5b4c15291b48b8aaf1679f9a6a2bdd0e748b6
    ```
1. Patch `performance_test` to recognize `rmw_iceoryx2_cxx` as zero-copy-capable
    1. NOTE: This shall soon be merged upstream to `performance_test` for convenience
    ```console
    cd ~/workspace/src/performance_test
    git apply ~/workspace/src/rmw_iceoryx2/performance_test/patch/recognize-rmw-iceoryx2-cxx-as-zero-copy.patch
    ```
1. Build `performance_test` and `rmw_iceoryx2_cxx`
    1. NOTE: `-Wno-template-body` lets gcc 15 compile the `rapidjson` bundled with `performance_test`
    ```console
    cd ~/workspace/
    export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Wno-template-body --build-base "build_perf_$RMW_IMPLEMENTATION" --install-base "install_perf_$RMW_IMPLEMENTATION" --packages-up-to "$RMW_IMPLEMENTATION" performance_test
    ```
1. Install dependencies into python env
    ```console
    cd ~/workspace/src/rmw_iceoryx2/performance_test/
    poetry install
    ```
1. Collect data
    ```console
    export RMW_IMPLEMENTATION=rmw_iceoryx2_cxx
    export ROS_DISABLE_LOANED_MESSAGES=0 # ensures loaning is enabled

    source ~/workspace/install_perf_$RMW_IMPLEMENTATION/setup.zsh
    cd ~/workspace/src/rmw_iceoryx2/performance_test
    poetry run python performance_test.py $RMW_IMPLEMENTATION ~/workspace/install_perf_$RMW_IMPLEMENTATION --zero-copy
    ```
1. Generate plots
    ```console
    cd ~/workspace/src/rmw_iceoryx2/performance_test
    poetry run python plot.py ./results
    ```

