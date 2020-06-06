@echo off
cls
echo [run_CTEST_debug:]
pushd build\core

ctest -V

popd