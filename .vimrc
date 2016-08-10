command! -nargs=* Build !clear && cmake --build .build <args>
command! -nargs=* Debug !clear && nemiver <args> .build/triangulation-test &
command! -nargs=* Run !clear && .build/triangulation-test <args>

noremap <leader>b :Build 
noremap <leader>d :Debug 
noremap <leader>r :Run 
