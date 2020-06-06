@echo off
cls
echo [run_test:]
pushd build\core\Debug

rem ctest -V
core_test


popd