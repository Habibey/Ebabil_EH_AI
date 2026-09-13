cmake_minimum_required(VERSION 3.10)
project(konum_servisi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)

# Bu repo (Ebabil_EH_AI) içinde SELF-CONTAINED -- yon_bulma/ ve konum_kestirimi/
# artık ayrı bir yonKonum1905 klonuna değil, doğrudan bu dizinin altına
# vendor edildi (bkz. main.cpp başındaki köken notu). Böylece RPi'da SADECE
# bu repo klonlanarak konum_servisi derlenebilir, ikinci bir repo gerekmez.
set(YON_KONUM_SOURCES
    yon_bulma/ortak/aci_yardimci.cpp
    yon_bulma/ortak/guven_hesabi.cpp
    konum_kestirimi/ortak/rf_kalibrasyon.cpp
    konum_kestirimi/ortak/rf_kalibrasyon_deposu.cpp
    konum_kestirimi/ekf/ekf_konum.cpp
    konum_kestirimi/parcacik_filtresi/particle_filter.cpp
    konum_kestirimi/orkestrasyon/konum_orkestrasyonu.cpp
)

add_executable(konum_servisi main.cpp ${YON_KONUM_SOURCES})

target_include_directories(konum_servisi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(konum_servisi PkgConfig::ZMQ)
