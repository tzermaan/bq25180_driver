cmake_minimum_required(VERSION 3.14)
project(bq25180_driver C)

set(CMAKE_C_STANDARD 99)

add_library(bq25180
    bq25180.c
)
target_include_directories(bq25180 PUBLIC .)

enable_testing()
add_executable(test_bq25180
    tests/test_bq25180.c
)
target_include_directories(test_bq25180 PRIVATE .)
target_link_libraries(test_bq25180 bq25180)

add_test(NAME unit_tests COMMAND test_bq25180)

