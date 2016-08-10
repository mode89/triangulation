# Configuration

    include(CheckIncludeFiles)

    check_include_files(ieeefp.h HAVE_DECL_FINITE)
    set(HAVE_IEEE_COMPARISONS 1)

    configure_file("config.h.unix" "config.h")

# Targets

    # Build only necessary funcionality
    add_library(gsl
        blas/blas.c
        block/init.c
        cblas/caxpy.c
        cblas/ccopy.c
        cblas/cdotc_sub.c
        cblas/cdotu_sub.c
        cblas/cgemm.c
        cblas/cgemv.c
        cblas/cgerc.c
        cblas/cgeru.c
        cblas/chemm.c
        cblas/chemv.c
        cblas/cher.c
        cblas/cher2.c
        cblas/cher2k.c
        cblas/cherk.c
        cblas/cscal.c
        cblas/csscal.c
        cblas/cswap.c
        cblas/csymm.c
        cblas/csyr2k.c
        cblas/csyrk.c
        cblas/ctrmm.c
        cblas/ctrmv.c
        cblas/ctrsm.c
        cblas/ctrsv.c
        cblas/dasum.c
        cblas/daxpy.c
        cblas/dcopy.c
        cblas/ddot.c
        cblas/dgemm.c
        cblas/dgemv.c
        cblas/dger.c
        cblas/dnrm2.c
        cblas/drot.c
        cblas/drotg.c
        cblas/drotm.c
        cblas/drotmg.c
        cblas/dscal.c
        cblas/dsdot.c
        cblas/dswap.c
        cblas/dsymm.c
        cblas/dsymv.c
        cblas/dsyr.c
        cblas/dsyr2.c
        cblas/dsyr2k.c
        cblas/dsyrk.c
        cblas/dtrmm.c
        cblas/dtrmv.c
        cblas/dtrsm.c
        cblas/dtrsv.c
        cblas/dzasum.c
        cblas/dznrm2.c
        cblas/icamax.c
        cblas/idamax.c
        cblas/isamax.c
        cblas/izamax.c
        cblas/sasum.c
        cblas/saxpy.c
        cblas/scasum.c
        cblas/scnrm2.c
        cblas/scopy.c
        cblas/sdot.c
        cblas/sdsdot.c
        cblas/sgemm.c
        cblas/sgemv.c
        cblas/sger.c
        cblas/snrm2.c
        cblas/srot.c
        cblas/srotg.c
        cblas/srotm.c
        cblas/srotmg.c
        cblas/sscal.c
        cblas/sswap.c
        cblas/ssymm.c
        cblas/ssymv.c
        cblas/ssyr.c
        cblas/ssyr2.c
        cblas/ssyr2k.c
        cblas/ssyrk.c
        cblas/strmm.c
        cblas/strmv.c
        cblas/strsm.c
        cblas/strsv.c
        cblas/zaxpy.c
        cblas/zcopy.c
        cblas/zdotc_sub.c
        cblas/zdotu_sub.c
        cblas/zdscal.c
        cblas/zgemm.c
        cblas/zgemv.c
        cblas/zgerc.c
        cblas/zgeru.c
        cblas/zhemm.c
        cblas/zhemv.c
        cblas/zher.c
        cblas/zher2.c
        cblas/zher2k.c
        cblas/zherk.c
        cblas/zscal.c
        cblas/zswap.c
        cblas/zsymm.c
        cblas/zsyr2k.c
        cblas/zsyrk.c
        cblas/ztrmm.c
        cblas/ztrmv.c
        cblas/ztrsm.c
        cblas/ztrsv.c
        cblas/xerbla.c
        err/error.c
        err/stream.c
        matrix/getset.c
        matrix/init.c
        matrix/matrix.c
        matrix/rowcol.c
        matrix/submatrix.c
        multimin/convergence.c
        multimin/fminimizer.c
        multimin/simplex2.c
        vector/copy.c
        vector/init.c
        vector/minmax.c
        vector/vector.c
        sys/fdiv.c
        sys/infnan.c
    )

# Includes

    target_include_directories(gsl
        PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
    )
