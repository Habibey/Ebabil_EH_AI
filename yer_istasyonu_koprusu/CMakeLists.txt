cmake_minimum_required(VERSION 3.10)
project(yer_istasyonu_koprusu)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)
find_package(Threads REQUIRED)

add_executable(yer_istasyonu_koprusu main.cpp)

# mavlink_cerceve.hpp bu dizinde YEREL bir kopya -- ebabil-eh-backend
# reposundaki parametreCikarimi/ebabil_sdr'a (artık kullanılmayan C++ RPi
# portu, bkz. Ebabil_EH_AI CLAUDE.md) bağımlı olmasın diye buraya taşındı
# (kaynak: o reponun mavlink_cerceve.hpp'si, değişmeden kopyalandı).

target_link_libraries(yer_istasyonu_koprusu
    PkgConfig::ZMQ
    Threads::Threads
)
