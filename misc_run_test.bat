@echo off
cls
echo [misc_run_test:]
pushd build\core\Debug

rem ctest -V
misc_core_test


popd