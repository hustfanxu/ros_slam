file(REMOVE_RECURSE
  "libcommon_math.pdb"
  "libcommon_math.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/common_math.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
