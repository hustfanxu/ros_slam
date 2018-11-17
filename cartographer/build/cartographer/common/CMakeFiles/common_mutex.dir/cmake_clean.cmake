file(REMOVE_RECURSE
  "libcommon_mutex.pdb"
  "libcommon_mutex.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/common_mutex.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
