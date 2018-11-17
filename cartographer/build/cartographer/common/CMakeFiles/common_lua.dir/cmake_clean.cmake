file(REMOVE_RECURSE
  "libcommon_lua.pdb"
  "libcommon_lua.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/common_lua.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
