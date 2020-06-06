#include <spdlog/spdlog.h>
#include "gtest/gtest.h"

int main() {
  spdlog::set_pattern("[%^%l%$] %v");
  spdlog::info("agh");
}

