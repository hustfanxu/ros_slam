file(REMOVE_RECURSE
  "libcommon_make_unique.pdb"
  "libcommon_make_unique.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/common_make_unique.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
