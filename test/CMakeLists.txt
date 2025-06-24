cmake_minimum_required(VERSION 3.14)
project(ArduinoHDLC_RS485_Tests)

# C++14を要求
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ネイティブテスト用フラグを設定
add_definitions(-DNATIVE_TEST)

# Google Testを取得
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
)
FetchContent_MakeAvailable(googletest)

# インクルードディレクトリ
include_directories(${CMAKE_SOURCE_DIR}/../include)

# ソースファイル
set(SOURCES
    ../src/RS485Driver.cpp
    ../src/HDLC.cpp
)

# テストファイル
set(TEST_SOURCES
    main.cpp
)

# テスト実行ファイル
add_executable(
    rs485_hdlc_tests
    ${SOURCES}
    ${TEST_SOURCES}
)

# Google Testをリンク
target_link_libraries(
    rs485_hdlc_tests
    gtest_main
    gtest
    pthread
)

# テストの登録
include(GoogleTest)
gtest_discover_tests(rs485_hdlc_tests)
