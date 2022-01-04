cmake_minimum_required(VERSION 3.1)

get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME 3rd_${MODULE_NAME})
project(${PROJECT_NAME})



add_library(${PROJECT_NAME} STATIC 
  chkder.c
    chkder_.c
    covar.c
    covar1.c
    covar_.c
    dogleg.c
    dogleg_.c
    dpmpar.c
    dpmpar_.c
    enorm.c
    enorm_.c
    fdjac1.c
    fdjac1_.c
    fdjac2.c
    fdjac2_.c
    hybrd.c
    hybrd1.c
    hybrd1_.c
    hybrd_.c
    hybrj.c
    hybrj1.c
    hybrj1_.c
    hybrj_.c
    lmder.c
    lmder1.c
    lmder1_.c
    lmder_.c
    lmdif.c
    lmdif1.c
    lmdif1_.c
    lmdif_.c
    lmpar.c
    lmpar_.c
    lmstr.c
    lmstr1.c
    lmstr1_.c
    lmstr_.c
    qform.c
    qform_.c
    qrfac.c
    qrfac_.c
    qrsolv.c
    qrsolv_.c
    r1mpyq.c
    r1mpyq_.c
    r1updt.c
    r1updt_.c
    rwupdt.c
    rwupdt_.c
    cminpack.h
    cminpackP.h
    minpack.h
    minpackP.h
)

set_target_properties(${PROJECT_NAME} PROPERTIES
                      FOLDER "3rd_party")


target_include_directories(${PROJECT_NAME} PRIVATE ${EASY3D_cminpack_INCLUDE_DIR})